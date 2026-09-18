# d616eba配套：单件运输、统一分离计次与明确拒绝回执

基线F407 9d4f07f；已fetch并核对上位机最新 `codex/gamepad-teleop@d616eba`。本次只改F407仓库，不修改LCD、调试输出或新增测试，不烧录。

## 已实现

### 单件正式审核

`task_audit_semantic`首先保留INITIAL_STASH任意非空例外；正式任务必须总数恰好1且左右数量一致，空侧必须是NONE。
首次正式运输仍只允许1 GREEN；之后允许1 GREEN或1 CORE，伤员单件且目的地标志正确。
任意MIXED_MATERIAL均非法，即使total=1；两绿、两核心、绿+核心及伤员混装均非法。没有裁剪/改写原始ROI总数。
合爪前、mode23、mode45及mode48继续共用此判据；mode43扫障取障独立非空语义不变。
已持有并合法审核的单核心不会因为普通物资优先而被下位机中途换目标；选择优先级仍由上位机负责。

### 分离总次数与同侧次数

- separation_total_attempts：本目标所有带侧DISPERSE/单侧RELEASE的已接受次数。
- separation_side_attempts：当前保留侧的递进次数，仅用于第一次0°、第二次松10°、第三次松20°。
- 换保留侧只清同侧次数并更新夹持基准，总次数不清；同action重发不计数，无侧20°观察不计数。
- 合法审核、GRAB、NAV、mode48、新action及动作编号进位不清总次数；新目标/新会话初始化才重新开始。
- task_begin_capture_escape、task_capture_observe、task_prepare_separation统一依据总次数判第三次非法、禁止第四次。
- 第三次后READY=0保持停车等待，不再由普通4秒观察窗口提前判失败；READY=1且非法才走24→47，普通放弃仍走24→3。

### 0x1E明确拒绝

| 载荷字段 | 定义 |
| --- | --- |
| P0/P1 | 真实已接受task_id |
| P2/P3 | 真实已接受action_id |
| P4 | 真实已接受opcode |
| P5 bit0/bit1 | 接受/完成，含义不变 |
| P5 bit2 | 当前有同任务请求的明确拒绝回执 |
| P6/P7 | 被拒action_id，大端；无拒绝标志时发0 |

拒绝不会覆盖accepted上下文、ACK或当前控制指令，也不刷新命令/视觉年龄。
只有完整、正确CRC和紧邻同SEQ的请求对进入处理并被拒绝后才报告。有效CRC但非法动作参数的完整帧对也转交Task生成拒绝；缺前半帧、坏CRC、噪声不是“已审查拒绝”。
不同task的过时/陌生请求不能用当前task冒充其拒绝归属，不设置此同任务回执；已有回执不因此被清除。
新拒绝更新拒绝ID。接受该action或后续新action后清除；HOLD/PAUSE不清，旧已接受动作心跳也不清较新的拒绝记录。
0x1E→0x17仍是同快照、同SEQ、连续30字节队列项。tools/vision_protocol.py同步解码bit2和rejected_action_id。

## 不变的流程

进攻藏点由上位机改为红(+0.15,-0.74)、蓝(-0.15,+0.74)m，F407没有新增坐标，继续INITIAL_STASH和实时H/D。防守、颜色、出发区、定位原点不变。
保留聚集360°恢复、mode24/47分流、S1退区、扫障42～46及所有已有预算/推进参数。
自动地图边界恢复仍关闭，不重新启用50 mm入口。
mode48不因时间放行，也不伪造READY/VALID；mode15的1秒视觉超时TASK_COMPLETE、delivery_visual_timeout及既有计次/首件策略保持。

## 上下位机配套核对

新版上位机 `_reconcile_transport_rejection` 检查新鲜状态、task匹配、bit2及拒绝ID等于当前请求，再进行mode10恢复STAGE或mode48重审；不再仅凭旧accepted状态猜测被拒。
下位机未放宽mode10/48的任何接受条件。占位0帧仍不算空爪，改道仍要完整合法审核与STAGE NAV。
上位机已将 `_latch_carried_manifest` 的分离计数清理改为保留，新增同侧计数；本次下位机统一总次数后，旧“换侧清零造成47分流不一致”的口径冲突已补齐。

## 尚需注意的问题与建议

1. **拒绝恢复仅覆盖特定阶段。** 上位机目前主要在transport_audit_handoff_active/TRANSPORT_AUDIT且mode10/48时消费拒绝。其它阶段的错误action、错误状态命令仍可能进入持续重发等待；不同task拒绝无法用此同任务格式反馈，也不能直接假定允许接管。应按具体业务状态补恢复，不用超时伪造接受。
2. **mode10“未闭爪”不等于“还没到预备点”。** `_reconcile_transport_rejection`将未DISTANCE_DONE或未GRIPPER_CLOSED都送入恢复STAGE导航。NAV本身不会把爪子重新夹好，若确实出现未闭爪状态，可能重复D=0而无法进入48。建议把未闭爪单列为物资/机构恢复，不能只重发导航。
3. **恢复导航还需核对定位年龄。** `_resume_stage_navigation`直接调用_navigation_command；后者会对pose.valid=false发PAUSE，但没有独立检查pose.age_ms。拒绝恢复在普通mission.step前执行，建议此入口也使用_pose_fresh，避免valid仍为真但已过期的定位生成一条H/D。
4. **审核字段需一致编码。** 下位机拒绝空侧有类别、非空侧为NONE、任意MIXED_MATERIAL等矛盾载荷。上位机Counter在零计数类别上的处理可能更宽；编码时请把真正空侧规范为NONE/0，否则可能上位机判合法、下位机READY但INVALID，反复等待或重审。
5. **回执不是可靠消息队列。** 当前仍是最新命令/最新拒绝快照，快速变化的中间请求或状态可能没被对端看到。应持续重发当前请求，按task和拒绝ID消费；没看到明确拒绝仍按延迟处理，不能擅自取消或递增目标。HOLD/PAUSE不清回执是为此保留观察窗口。
6. **持续停车不必然是卡死。** mode48缺少稳定三帧、第三次复审READY=0、或mode47没有RETURN时，固件按规则停车等待。不能以“电机没动”就重启APP/GRAB；上位机应检查真实视觉帧、READY、上下文和当前mode。
7. **关闭边界与限时投送的现实边界仍在。** 定位漂移/错误但新鲜的H/D不会被自动mode41兜底；1秒未视觉确认仍按既定策略结束投送。单件规则与拒绝协议不能保证物块实际已经入区。

## 验证与部署

NormalRun正式固件编译通过、无新增编译警告；git diff检查通过。未新增/运行测试，未做失联/乱序注入，未烧录实机。
上位机Python任务程序、localization的0x1E解析/JSON输出及固件需配套部署；不能继续把1E的P6/P7强制要求为0。
本报告为源码核对结论，不能据此声称所有现场异常都已验证不会卡死。
