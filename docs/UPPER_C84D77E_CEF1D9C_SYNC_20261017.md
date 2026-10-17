# 2026-10-17 上位机最新增量修改建议

请在现有上位机代码上增量修改，保留新增加的ROI诊断、侧别签名与上下文处理。不要整套应用旧20261016补丁，不要回退或整文件覆盖，不新增测试程序、协议字段、通用ESCAPE或故障锁存。

本次核对基线：
- 上位机远端codex/gamepad-teleop：c84d77e97439c5023c45f80d0cf8e95f5a2f9b05。
- 上位机main仍为6848456，不能用它代替上述分支。
- 下位机当前功能提交：cef1d9ccc5bac3d4106fcce418899aad6ec61c63，NormalRun。
- 上位机本次主要对齐的是旧下位机4385b0d，尚未完整吸收0bd7d4f及cef1d9c的后续增量。本说明为代码核对结果，不能证明实际设备已经部署对应提交。

## 一、已完成并应保留的部分

1. make_vision_snapshot已经先按60%逐物体纳入ROI，爪外/局部入框目标不再否决其它清楚的爪内物体。不要再把这一项列成未实现。
2. audit_from_cargo已经保留左右类别组成，区分已知混合物和真实未知/未分配目标；_side_is_disperse_candidate允许部分已知混合侧参与分离。
3. _preferred_keep_side已改为绿色最多侧；相同时保持锁定侧、再参考无关物块数量。
4. _start_disperse会更新当前选侧数量；_arm_disperse新增keep_count参数并正确生成SIDE_VALID/TARGET_RIGHT。可靠选侧优先于FIRST_GREEN_BUMP。
5. 选侧投票增加线上审核签名，pending分离证据也检查左右组成/绿色数量变化。这些后续增量要保留，不能被旧补丁覆盖。
6. 两帧、两次分离、橙色49/23/50/22、mode48、mode51及已有上下文/缺pending处理继续保留。
7. 主程序已将args.capture_roi.is_file()传入图像质量检查，缺少标定文件会判无观察。无需仅因CaptureRois没有calibrated字段就重复增加一套同义机制。

## 二、优先修复：新增日志可能中断规划线程

位置：competition_rescue/run_competition_rescue.py:1899的_log_capture_decision、:1956的调用，以及state_machine.py:1377的_audit_payload。

当前日志在output没有线上audit时，尝试对output.audit、vision.capture_audit或pending_audit调用mission._audit_payload。该函数要求selected_batch非空。搜索、空爪恢复或任务清理后selected_batch可能已经清空，而缓存画面或本周期output仍带CargoAudit，此时日志触发AssertionError。

_log_capture_decision在_publish之前运行，异常会进入_run外层异常处理并结束规划线程。主循环随后检测planner.error并报规划线程退出。这不是单纯日志缺一行，而是可能阻断后续控制命令。

修改要求：
- 日志有本周期实际线上payload时直接记录其semantic_signature；没有实际payload时将wire_signature记为null，并另记原始观察，不为写日志调用依赖任务批次的业务编码函数。
- selected_batch=None、缓存审核仍存在、刚刚清空pending、SEARCH/WAIT_START均应能正常记录。
- 日志必须是无副作用的诊断。局部处理日志序列化/写入异常并记录限频错误计数，保持本周期合法命令发布；不要吞掉真正的规划异常。
- 合法命令发布不应等待可选诊断写盘。至少调整调用先后和局部异常边界；异步日志队列可作为进一步优化，不用为此改串口协议。

已通过调用当前实际函数复现：selected_batch=None、VisionSnapshot携带CargoAudit()、SEARCH/HOLD输出时触发AssertionError。

## 三、补齐初次空爪继续30cm的上位机分支

位置：state_machine.py:_audit_output，当前约3207行的stable且total_count==0分支。

当前普通mode21和聚集mode38两帧空爪都会进入AUDIT_CONFIRM，pending上下文为recheck_empty_to_search，等待F407回SEARCH。新F407在初次空爪时会继续限距前进，因此两端对本阶段的结束预期不同。当前代码仍会处理后续新画面，并不是每次都会立即卡死或发释放命令，但不应将初次空爪描述为抓取后丢载。

新增分支应位于通用空爪处理之前：
- 仅mode21/38、非post_grab_audit_active、非cargo_recheck_pending、尚未执行分离且确认为纯空爪时生效。
- 纯空爪要求左右数量/总数为0，左右类别为NONE，没有危险、未知、伤员混装标志；缺图/占位不算真实空爪证据。
- 保持CAPTURE_AUDIT，继续发送真实帧及原有两帧STABLE，reason=capture_empty_approach；不创建recheck_empty_to_search的等待，不GRAB、不释放、不提前RETURN。
- 后续物块入爪时使用新帧正常审核，形成合法结果后抓取，需要分离则转分离。继续遵守两帧、至少100ms及下位机READY/VALID。
- 两端实际行程以F407为准，不按200mm/s乘上位机计时估算30cm，也不重复发APP重置同堆预算。到300mm仍空时由下位机停车并走既有决策/恢复出口，保持原15/17秒期限。
- mode23抓后、橙色后退复核、分离后和mode48空爪仍按原丢载逻辑处理，不能统一改成继续前进。

已复现当前普通21及聚集38均进入AUDIT_CONFIRM/recheck_empty_to_search。下位机cef1d9c的普通/聚集目标慢爬已统一200mm/s，核心物块额外50mm前移单独保持180mm/s。

## 四、补齐普通抓取遇到多物块的曲线入口

位置：state_machine.py:_audit_output的disperse_vote_context（约3221行）及其后非法审核路径（约3303行）。

当前分离候选函数已支持已知混合侧，但侧别投票仍只覆盖cluster_audit_active和既有分离复核上下文。普通mode21意外同时进入绿色和橙色时，_side_is_disperse_candidate可以返回True，但流程依然落到_choose_release_side；侧类别为unknown时它返回both，pending为separate_then_search。

修改要求：
- 初次普通抓取中出现多个物块，且至少一侧经当前候选规则确认可以保留时，也纳入现有两帧侧别投票。
- 审核稳定且选侧证据一致后，通过现有disperse_reaudit或等价的非聚集分离握手，发送带SIDE_VALID的DISPERSE_PILE。不能伪装成mode38并等待下位机进入mode37。
- 保留c84d77e新增的侧别签名、pending组成变化撤销和keep_count实参。
- 不直接把所有unknown都判可分离，不修改实际线上类别、总数及非法标志；正式GRAB/运输规则不放宽。
- 不改变单侧释放的方向含义：RELEASE_LEFT是打开左侧，与保留左侧相反。

已复现“普通mode21，左侧绿色1+橙色1”：候选True，但pending=separate_then_search、release=both。该场景应进入受控选侧曲线。

## 五、已知重叠堆仍会被整体判无观察

位置：run_competition_rescue.py:669附近的框重叠质量判断。

当前两个夹内框重叠比例达到0.85时，仍直接将capture_observation_valid设为False。只有INITIAL_STASH有非空豁免，普通分离观察尚未区分“可辨认的多物块堆”和“无法观察”。即使选侧函数已修好，没有capture_audit也无法走到它。

建议按任务阶段处理：
- 在mode21/35/37/38的分离观察中，对两帧能确认的多个独立、已知类别且未被图像边缘截断的目标，允许生成真实非空多件审核；它依然是非法运输载荷，只用于受控分离。
- 保留mode23/45/48正式审核、单件可靠性、真正未知、缺图/全黑/过曝和实际边缘截断的限制；不能单纯删除所有遮挡判断。
- 当前target_quality在重叠循环中会覆盖先前的claw_target_truncated。若放开重叠，必须保留截断优先级或保存缺陷集合，避免“截断+重叠”被误认为仅重叠而放行。
- 保留mode43只针对当前扫障目标的质量判断和INITIAL_STASH真实非空语义。

已用当前make_vision_snapshot和真实OpenCV复现：mode38中两个未截断、已知绿色/橙色框互相重叠，结果为claw_targets_occluded、capture_audit=None。

## 六、补齐当前参数、部署基线和诊断开销

rescue_map/field_model.py:321的expected_lower_firmware仍指向4385b0d；更新为cef1d9ccc5bac3d4106fcce418899aad6ec61c63或后续包含相同功能的实际功能提交。expected不等于actual，不得将推荐版本直接写成已烧录版本。

competition_rescue/README.md和docs/f407_mg310_competition_sync.md顶部部署说明需更新：普通/聚集限距慢爬均200mm/s；抓前空爪继续至30cm或发现物块；核心50mm前移仍180mm/s；新增固定航向段已共用电机层10ms控制。历史段落中的三帧/旧固件只作为历史参考，不应作为部署目标。

保持以下参数和流程：
- 125/130度原地对准，140度250ms准备；不加新的125/130度固定等待。
- 视觉车头P=0.18、D=0.0025；俯仰P=0.55；轮速5/0.12/0，航向PI=2.2/0.005；MG310计数1061.268、70mm轮径、慢衰减、60%PWM。
- 同堆最多2次物理分离、每次额外夹紧10度；重发不计次数；第二次后真实复核失败按既有退让返中，不新增第三次。
- 橙色23→49→23→50→22，mode49后退2秒，第二轮独立新审核，mode50保留已提交结果等待闭爪。
- mode48、扫障角色、缺pending出口、mode51、投送计数、TASK_COMPLETE、串口配对和T265外参保持。

诊断性能的进一步收尾：
- 当前每个框先单独算overlap，再由bbox_in_capture_roi重复计算，进入capture_side_for_bbox后还会重复overall纳入判断。可复用单帧计算结果/ROI掩码，保留相同60%几何语义。
- output.event的事件日志已经限频，但_diagnostics仍被原始output.event每周期触发。复用实际事件变化/周期判据，避免事件文字没变仍反复写诊断JSON。
- 保留本次新增的ROI明细、阶段耗时与真实采集时间字段；这些优化不应改变采集帧号或审核有效期。

## 七、交付范围和核对边界

本次仅补全修改建议并保存核对记录，不修改下位机固件或上位机业务代码，不新增测试程序。使用隔离本地源码调用复现了日志断言、初次空爪等待状态、普通混合侧释放路径及重叠无观察；相机/BPU硬件未执行，不代表实车故障已经复现。

请上位机Codex优先处理日志断言、初次空爪分支和普通多物块入口，再处理重叠质量与诊断性能。现有20261016补丁只可作为摘取遗漏逻辑的参考，不能整套覆盖c84d77e。最终列出实际修改文件、功能提交和两端部署说明，不另建测试程序。
