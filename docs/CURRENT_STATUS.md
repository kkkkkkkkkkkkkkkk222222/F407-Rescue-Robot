# 当前状态：F405 烧录入口

2026-11-07 安全区预备点末段纠偏，基于下位机 eadf376 / RDK f8eaf67。
- 实跑报告为 eadf376、source_dirty=1、NormalRun TASK。本机对应构建的源码差异是 Task.c 两行空白。
  日志确认连续21秒三轮增量为零，实际PWM未知。源码中的同动作H/D更新保留斜坡和轮速PI状态。
- STAGE 的 FINE 目标速度为 min(120, D×1/s) mm/s。保持入段gyro航向，按最新H全向平移。
  轮目标零速带为0.5mm/s，微小轮目标的静摩擦前馈逐步减小；每轮同向实测速差提供最多150‰的附加前馈。
  换向减速和等零期间使用原控制。60%总PWM限幅、粗段限程/减速、藏点及RETURN的控制沿用原值。
- STAGE 的 motors_active 表示有非零驱动PWM指令，轮是否转动看编码器。D=0走原到点确认和载荷复审。
  上位机独立使用预备点±60mm/±50mm容差。补偿参数和实际起转效果待实车验证，可用原TSTAT读取TD_DRIVE。

2026-11-06 本地粗导航与像素对准，基于下位机 b2dd1d2 / RDK 5051376：
- STAGE、藏点 NAV（含空爪返藏点）及 RETURN 复用本地 Location 实测编码器＋主gyro位移。
  同一已接受 task/action 仅建立一次初始 D/H 与行驶基准；H/D/SEQ 心跳不重开快速段。
  最高600mm/s，本地剩余约600mm开始减速，预留250mm刹停后转120～200mm/s全向修正；
  初始D≤600mm直接低速修正，末端不再因最新方位变化重新掉头。仅等原Motor_Stop刹车周期结束。
- 粗段结束不是全局到点，不置distance_done/nav_done。STAGE/藏点D=0及RETURN原80mm到点合同不变。
  不清零/重锚里程，不加数据年龄、资格等待或握手；保留此前去年龄停车及近点全向修正成果。
- 仅TASK_APPROACH原地像素对准的最低轮切向请求试设为60mm/s；符号、P/D、8/16px死区、
  24/40px到中阈值、mode52测距及其后流程不变。扫障和运动中跟踪仍用原10mm/s最低请求。
- 600/250/120～200和60均为工程初值，未实测标定。接口、任务审核、人工STOP、mode51、
  已接受动作与重复REJECT处理不变。两轮实跑1703394/dirty1仅据RDK报告，未亲自读取或烧录本轮镜像。
- 实现、边界和构建记录见 [本地粗导航与像素对准](COARSE_NAV_AND_PIXEL_ALIGN_20261106.md)。

2026-11-06 导航配套，基于下位机 f1f346bb / RDK fcc4f9eb：
- STAGE_NAV 与持物藏点 NAV 共用现有 300mm 末端全向平移。只在进入末端时停止原转向/
  平移并启动 Motor_MoveAngle，后续按场地几何方位换算车体系方向，沿用原航向保持、矢量斜坡和限速。
  D=0 仍立即停止并报告 mode10/distance_done；0x18、原配对、mode48/ALIGN/ENTER继续使用原入口。
- 删除 nav_follow 的 500ms 无进展计时与停车作用。导航元数据只保留现行距离、路径与有效状态，
  供原藏点退让和 HOLD 交接使用。保留线上 nav_stale 字段，其值不再由该已删除的巡检置位。
- 连续导航/靠近、定距接近、伤员后退均不再按收包/视觉年龄中断；仍检查实际已接受上下文与目标存在。
  接近阶段原执行预算、伤员两秒后退预算、合爪审核、有限动作超时及人工STOP/ABORT继续执行。
  TaskDiag原年龄数值仍可查看，移除已失去实际控制含义的年龄等待位和500ms分母。
- 核对到：稳定空爪回SEARCH取消旧目标但保留分离次数；原RETURN仅在扫描结束的SEARCH受理。
  RDK明确REJECT后立即返中的兼容窄入口已向用户提出，等待选择；未擅自放开任务ID、预算或审核。
- 本轮无新增协议/中断/诊断统计/测试程序，未烧录或操作实车。问题日志的设备版本仅据RDK
  报告 d6ae45c4、dirty=1，不能据此确认当前板端实际固件。

运动调试用 `MotionDebug`（原 NormalDebugTimed），正式比赛用 `NormalRun`（原 NormalDebugTask）。
二者沿用协议 1.3、mode=5、UART4=230400；service 分别为 1、2。
产物和 CLion 配置见 [README](../README.md)。本次只迁移构建入口。
正式 Task 含 mode51 对整体有效动态定位的恢复支持；RDK 当前采用无框事件及视频回放，
不启用早先交接中的 H±8° 观察扫描或人工重试建议。

## 历史：正式任务接入轮反馈

三轮停滞后的导航修改及 RDK 待办见[预备点、临时藏点与安全区观察交接](STAGE_STASH_ACQUIRE_20261105.md)。

2026-11-05。下位机基线 `4ac1d3d`，配套上位机 `codex/gamepad-teleop@2fe0381`。
正式入口用 `NormalDebugTask`，启用完整救援 Task 和原 UART4 上的协议 1.3 反馈。
RDK 接 JP506，UART4 为 230400/8N1。配置回执优先，比赛状态上下文＋状态同一槽发送，
优先于排队的测量。在途 DMA 帧正常发完，未发的同类 07/08 仍可合并为最新记录。

- 身份回执 02 的 payload 偏移 38 是 board (u16 BE=405)，40 是 mode (u8=5)，
  41 是 service (u8，1=MOTIONDEBUG，2=RESCUE_TASK)。波特率、Git、构建指纹和 dirty
  沿用现有字段。构建指纹现在同时包含模式和服务构建开关。
- 每个 SDK 会话使用既有 ID_REQ 和一次 CONFIG_REQ(enable=1, profile=1)。
  上电遥测关闭；查询、订阅及其回执只服务遥测。Task 仍按原比赛配置与命令执行。
- 07 为车体中心前/左正的实测速度 (µm/s)、三轮原始累计计数及采样区间，标称 50Hz；
  08 为 CCW 正的累计 yaw (mdeg) 和主 gyro 角速度 (mdps)，有新源时最高 100Hz。
  帧格式、源 ID、epoch 和计算使用原实现。两帧流合计 14250 B/s，UART4 理论 TX 容量 23040 B/s。
- 原 0x16、0x18/0x1D、0x17/0x1E、STOP、合爪审核及 mode51 均走原处理路径。
  已知历史行为单列：F405 对部分连续动作使用指令/视觉 500ms 超时，上位机有 STM 状态
  250ms PAUSE 及任务续发 250ms 限制。本次仅登记，后续若调整须由用户决定。
- 交付检查限正式构建和调用链。待部署项目为烧录后的 TASK 身份读回、正式链路 07/08
  接收和 SDK 实际接受计数。11月4日最后一次 Flash 比对对应 `4ac1d3d` 的
  NormalDebugTimed/MOTIONDEBUG。11月5日本轮未连接实车或读取 TASK 实烧身份。

## 前次修复记录（2026-11-04）

2026-11-04。仅按用户本轮要求及上位机 `582a1f6` 的 CHANGE_WORKFLOW、
F405_REPAIR_HANDOFF_20261104 核对，历史阶段要求不自动继承。

## 核对与范围

- 起始下位机 HEAD/远端 main 均为 `11eeacfda71f419834ff78401faf1a0d35a3831e`。
- 既有 IDE 修改（含已暂存的两项 IDE 文件）、tmp/、Task.c 两行空白均保留，不夹入代码提交。
- 既有 NormalDebugTimed 构建嵌入 Git=11eeacf，source_dirty=1，firmware_id=
  `8b863b182c04e8ca1e43977f99883440febdb1e3855c107650c5b919c9317213`。
- 交接报告板端 actual=11eeacf/dirty1/F405/MOTIONDEBUG/230400；这是上位机报告，
  不当作本轮亲自读回。expected 没有被用来替代 actual。
- 代码已证：IMU_Update 在临界区外取 now，ISR 更新 angle_ms/rate_ms 后可产生
  无符号年龄下溢；ready 清除通过 Location、Debug、motor/Task 引起动作结束。
- 真实 UART 错误是否同时发生仍不确定。

## 本轮方向

修共享 UART IMU 读取竞态及短暂恢复动作连续性；精简额外时间资格、逐沿/逐字节
附加计时及高频诊断。保留真实计数、速度计算区间、主 yaw、动作身份、F405 运动学、
人工 STOP、mode51、单一 DMA 发送者和稳定缓冲区。不改变 PID/速度和比赛业务流程。
用户本轮已明确选择：恢复或持续无反馈期间沿用最近有效反馈和控制目标，
继续原动作，保留原动作完成/超时和人工STOP，不新增失联暂停/取消分支。
显式ORE/FE/NE/PE及启动波特率切换不再清除已建立的ready；从未收到合法反馈仍不可用。
已核对Location、Debug MOVE/TURN、底层motor、Task和共享Gamepad的判断链，
RECOVERING不再经ready清除引发连锁误停。代码范围已完成，待人工实车复核。

## 本轮实际结果

- 规则提交：`ca87bfa956a6a6bc1126d65738fbeff4eb1545b5`。
- 前一步实现为c2deed2；本次补齐用户选择的恢复持续执行策略，协议语义更新为1.3。
- 最终实现提交：`0a2dbf7f523cba577c6ccacc3db5fae31b8dae48`。
- 最终本地构建：NormalDebugTimed，MODE5/MOTIONDEBUG，UART4=230400，协议1.3。
  嵌入Git=0a2dbf7，source_dirty=1；firmware_id=
  `acbe5483edf9e02ba767be0d5cebba9457552ea0f3b67c4c3aebc2bd34e6a80d`。
  ELF SHA256=`c6c09d74f3a98dfac40129ed3e3b6310011aac0a0a4225373d0c4745912bb500`。
  NormalDebugTask本轮ELF SHA256=`6425941bc39b38a839bf3a025e40952b2d8cd55bf4f17e7ec835564a1b08175d`。
  当前源差异只有既有Task.c的1547行后一个空行和一个两空格行；IDE/tmp修改未混入提交。
  后续文档HEAD不等于构建身份；重新编译会更新Git/firmware_id。上述均非板端实烧证明。
- 变更说明及接口语义增量：[F405_REPAIR_20261104](F405_REPAIR_20261104.md)。
- NormalDebugTimed（230400）和NormalDebugTask（115200）均编译通过；未新增比赛07/08。
- 直接相关的现有UART驱动、遥测模块、32帧DMA队列主机检查通过；无新增测试框架。
  本机MSYS gcc未成功生成新程序，改用CLion随附MinGW编译后执行，旧exe输出不作本轮证据。
- 已审核diff：没有新增时间门控、ISR、巡检、永久FAULT或ESCAPE；PID/速度/Task业务没有改动。
- 直接回归覆盖：RX错误/长间隔仍ready、源值/ID不伪造、新帧继续更新、冷启动错误不假装ready。
  现有轮速闭环继续运行，保持控制目标不等于锁死一次PWM。
- 本次选择确认后，只重跑上述UART驱动直接回归及两个受影响F405构建，全部通过。
- 未烧录、未复位、未发送运动；COM18只读尝试无回执且调用阻塞，已终止关闭，不能确认当前实烧身份。

## TODO（另定范围）

- 此处原列的正式比赛 07/08 软件接入已在 11月5日处理，部署状态见页首。
- 上位机收到修复后审查简化后的有效位语义，不再把未知时间界当本轮前置资格。
- 本轮不自动烧录或运动；短实测需当前授权。DONE仍不是地面距离/角度真值。
