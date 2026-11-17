# 上下位动作归属与流程交接检查

检查日期：2026-11-18。按用户要求先等上位本轮完成、提交推送，再修改本端；只对真实疑点做必要检查，不运行整套动作回归。

上位最终源为`danmo-teng/shijue_fangan` / `codex/gamepad-teleop` / `cfc697b2ece4f3d906bd3793d182926045433eb1`，运行代码修复为`96936aa4addfbeef62943e8ee9366e554eca928c`及`fd1fea8b1bfdc44bde1a52e479cf4d46dd03870c`。已核对96936aa到cfc697b只有定位分析文档变化；最新固定交接没有新增下位接口。下位审查基线为`57ddffedad0834c6065fb0b463997e78340f39c8`及当前工作区。本轮未操作设备、烧录或读取板端actual。

## 本轮新发现并修复：同opcode新action冒领旧动作

位置为`Main/Src/Task.c`的`task_accept_mission_legacy`中REMOTE_ACTION同opcode重发分支。原分支只比较command与remote_action.command，没有比较task/action。外层将不同action当作新请求后仍会进入这里，直接ACK并改写accepted_context，却没有开始新物理动作。

生产Task硬件替身的直接dispatch复现如下，未执行整套分离轨迹：

| 输入和前置状态 | 修改前 | 修改后 |
| --- | --- | --- |
| task7/action11的DISPERSE或YIELD仍在运行，收到task7/action12同opcode | 接受12，机械仍为11的phase | 不接受12，保持11及原phase |
| 上述旧动作已经DONE，再收到12同opcode | 接受12，旧remote_action.done可被报告为12的DONE | 不接受12，旧DONE仍归11 |
| 重发原action11 | 原动作保持 | 仍确认原动作，不重启或归零 |
| 合法连续两段step13，旧段DONE后发新action | 原专用入口开始新段 | 仍由原bulldozer入口开始新段，不受修补影响 |

本次只在原重发分支加入7行归属核对/注释：必须是已接受的同task/action才能复用执行或完成状态。其他action沿原动作入口、实际审核及允许状态接续。沿原拒绝回执报告当前请求未被接受，不新增FAULT、等待、新鲜度、审核帧数或握手，也不改变速度和机构动作。没有发现本缺陷在已发布实跑日志中触发的直接证据，不能拿此复现代替现场根因。

## 已提交的上位修复与下位适配

1. **分段完成后卡约5秒：上位96936aa已修。** 正式planner的52/53快捷分支现在按实际protocol_task/protocol_action填回context_matches，再交给原_range_approach_output。下位现有aux1已经按原要求到原105/120度、mode52及本action DONE后等待新图、新action续段；最终aux0才转140度。本轮不改下位距离、5秒出口或强制每段低头。已独立核对生产调用路径与本端接收/完成代码；上位提交报告的6项检查不算作本端重新运行的测试。
2. **真实绿黑重叠导致闭爪复审等待：上位fd1fea8已修。** 新例外只覆盖23/48中真实、不同编号/框且原质量条件通过的绿黑对象。下位已有1～3件材料、code6/7及真实两样本审核，协议和合法条件未变化，无需新增固件配套；没有放宽首绿、橙蓝混装或真实四件的资格。
3. 先前57ddffe的U1放下/300mm/+90度、视觉横移270～400，f1dd046的橙附加0，以及c6480b3的aux4仍在当前代码中，不因对端旧“下位待应用”文字重复实施。

## 仍未解决：上位禁轮策略恢复遗漏

位置为当前`competition_rescue/run_competition_rescue.py`的`LocalizationWheelPolicy.update`，亦是上位TODO53已记录的问题。当前恢复只覆盖mode17、部分RETURN/NAV以及默认关闭的抽障候选；以下原业务阶段没有匹配到恢复分支：

`ALIGN_SAFE_ZONE -> OBSTACLE_ROUTE -> OBSTACLE_PICKUP -> SEARCH(mode3/accepted20) -> APPROACH(mode20/accepted9)`。

将当前真实类提取到离线替身，按上述阶段及匹配任务回包执行后，suppressed仍为true，只生成过禁用包，没有恢复normal_motion。替身不打开真实socket、相机、串口或SDK。该结果证明阶段策略有遗漏，不证明全部定位偏差都由此造成。上位本轮明确定位仅分析，本次下位任务只记录此缺陷，不改上位定位融合、轮式策略或雷达阈值。

另一项功能缺口仍存在：aux3安全区疑似误排除抬头观察在下位已有入口，上位尚未接续/启用。归档opcode0x15抽障候选默认关闭、下位未实现，不恢复为当前实施任务。这两项不能列为本轮已修复。

## 同类路径逐项检查

以下以源码和实际调用路径检查为主，未把所有行都变成动态测试：

| 范围 | 核对内容与结论 |
| --- | --- |
| runner早返回及上下文 | 52/53匹配传递已由96936aa补齐；aux4与PRE_GRAB使用实际匹配上下文；复审、拒绝接管和原恢复有各自的归属处理。相机交接的HOLD本身不确认新动作完成。 |
| aux0/1/2/3/4 | 字段含义、机械完成及新图接续各自分开；aux4 DONE不是车头对准；aux3 HOLD回搜与aux4短HOLD保目标没有混用。 |
| 搜索点与扫障 | step13实时H/D和连续两段独立action；step14 A1/A2和B方向；A2完成表示准备好搜索，非到中心；同action不重置扫描。 |
| U1/U0与U8200 | 原因/高度字段有明确执行含义；U1全过程结束才mode3/DONE；U8200不作为航向；普通释放和普通分离沿原入口。 |
| 审核与载荷 | 占位帧不冒充空爪，UNKNOWN/四件溢出不伪造成合法载荷；首绿、绿黑材料和单橙分开；实际确认类型与当前漏框缓存分开。 |
| PAUSE/HOLD/ABORT与重发 | 查看外层dispatch到实际执行器的路径；相同有限action由外层幂等承接，原暂停恢复/停车收纳保持；不能仅凭不可达的旧legacy分支推断现场bug。 |
| 退出、返程和快速重发 | 投送本地退出与TASK_COMPLETE竞争、返程/D0、0x52配置及停车收纳；当前没有发现其他明确的同类冲突。 |

## 必要验证与部署边界

- 下位仅对上述一个缺陷做修前复现、修后直接回归，覆盖运行中/旧DONE、原action重发和合法连续step13；没有重跑未改的整段运动、分离或全部历史测试。
- 上位只对已发现的禁轮恢复遗漏做一次离线生产类复现；没有修改上位代码或把上位已有测试冒称本端重跑。
- 本轮只重建受影响的正式NormalRun：编译链接通过，APP_AUTO_BACKUP=OFF。MotionDebug未重建；未改调试流程或公共配置。
- NormalRun路径为`build/F405/NormalRun/NormalRun.elf`，build_git=57ddffe/dirty1，firmware ID=`fc109a70ad0f02900dfbfe51ee13f2bef13c7642cdc71dff1d9cbe033331b887`。构建包含原独立WheelCheck与Task空白工作区内容；这些原改动不夹带提交。
- 新NormalRun未烧录，actual本轮未读。源码检查、替身和DONE不能证明整车全部场景无缺陷，也不能当作退距/角度/定位精度的实车验收。
