# 两端当前要求与回执

更新日期为 2026-11-09，时区 Asia/Shanghai。

## 固定入口与协作方式

- F405 仓库为 `kkkkkkkkkkkkkkkk222222/F407-Rescue-Robot`，分支为 `main`。[本端固定入口](https://github.com/kkkkkkkkkkkkkkkk222222/F407-Rescue-Robot/blob/main/docs/CROSS_REPO_REQUESTS.md)
- RDK 仓库为 `danmo-teng/shijue_fangan`，分支为 `codex/gamepad-teleop`。[对端固定入口](https://github.com/danmo-teng/shijue_fangan/blob/codex/gamepad-teleop/docs/CROSS_REPO_REQUESTS.md)

涉及相关跨端接口或用户通知更新时，双方 AI 直接读取对端入口，核对源提交和请求 ID，
在自己仓库记录当前相关要求及审核/实现回执。旧请求按 Git 历史取证，不自动恢复为当前任务。
发布、读取或镜像要求不等于审核通过或实施授权，也不建立常驻自动修改服务。

接收方先审查自己的当前代码，向人说明问题、最小方案、成本和两端影响，可接受、缩小、替代或拒绝。
人保留知情、拒绝和选择权。已经明确批准的范围直接推进，超范围的重要取舍先由人决定，AI 不能互相授权。
不得擅自新增新鲜度、反复握手、资格等待、停车/FAULT、高频巡检或无效测试。
持久化规则见 [AGENTS.md](../AGENTS.md)，对端规则见 [CHANGE_WORKFLOW.md](https://github.com/danmo-teng/shijue_fangan/blob/codex/gamepad-teleop/docs/CHANGE_WORKFLOW.md)。

## TRAJECTORY-20261109 审核回执

核对的 RDK 交接提交为 `0d7256800f0f21c334543499406370064a2d96af`，F405 行为基线为
`08559cd57bfd2b83145bbdb6240de14c71bf64b5`。交接内两场实跑为 F405 `9434608`、`source_dirty=1`、
NormalRun/TASK 和 RDK `4cfaba9`。新版行为待实车验证。
[RDK 交接](https://github.com/danmo-teng/shijue_fangan/blob/0d7256800f0f21c334543499406370064a2d96af/docs/CROSS_REPO_REQUESTS.md#L23) ·
[联合分析](https://github.com/danmo-teng/shijue_fangan/blob/0d7256800f0f21c334543499406370064a2d96af/docs/analysis/motion_competition_20261109/ANALYSIS.md)

- 已实现 `b980c6a` 的后轮小量纠偏、`37b4c00` 的开局限加速度和现有 `0x16` 补中心，以及仅限 STAGE 的
  FINE 110～180 mm/s。`08559cd` 按本地陀螺仪锁红方 90°、蓝方 270°，旧视觉 ALIGN 继续回执。
  RDK 仍使用原 STAGE `D=0`、载荷复审、扫障和 ENTER。
- 建议先以新版复核有载末段。旧版五次观察到 `D=0` 后 120 ms 的轮里程约 0.4～1.3 mm，
  到点前 100 ms 的速度中位约 14～72 mm/s。这组数据无法标定新版较高速的刹停。
  若仍有非零目标但轮速上不来，复用 `TSTAT` 的 `TD_DRIVE` 区分驱动输出与机械阻力后，
  再决定是否只调 FINE 的现有助推。若是跑过到点框，按有载实测刹停余量限制 FINE 速度。
  200～250 mm/s 上限仍为待验证提案。
- 旧实跑三次直推起点的估计 X 离半区中心约 5.5～6.9 cm。固定航向后的判障走廊仍取自安全区框，
  需用已有画面对照实际直推是否覆盖。先保持现有 RDK 和到点容差。若发现走廊偏移，再单独讨论现有
  走廊几何。若走廊覆盖且起点横偏，再单独讨论 STAGE 的 X 容差。
- RDK 的轮式已可疑后遇 T265 跳变仍回到轮式，是独立的定位来源问题。适合先按旧日志回放核对，
  来源策略由人另行决定。115 mm 有效轮臂用于对照诊断，当前固件沿用原控制参数。

本条为源码核对和提案回执。运行代码沿用已有提交，实烧身份沿用最近有证据的记录。

## NormalRun 分离数量档

用户批准先用现有审核报文的单侧 0–3 块。RDK 参考提交为
`347d787c6b42471d1265380141e1b77a540bd81c`。RDK 现有审核包已提供分档所需字段，可直接联调。
F405 实现提交为 `62be90bfb5c295bbc759740143e8c3928b587b4b`，NormalRun 已编译并链接成功。
本地镜像 `source_dirty=1` 来自原有的 Task.c 两行空白。本次未烧录，分离角度和成功率待实车验证。

- F405 从已接受的审核锁定释放侧数量和保留侧，第一次有明确合格单件可保留时，
  释放侧 1–2 块用额外夹紧 20°、向释放侧转 10°，释放侧 3 块用额外夹紧 10°、转 15°。
  保留侧多件或单件证据不足时，额外夹紧为 5°。第二次按复审重选转角，额外夹紧为 5°。
- 双侧非空、左右计数之和等于真实总数且没有 UNKNOWN 标记，才使用精确数量档。
  已接受的带侧动作若证据不足，采用 5° 夹紧与 10° 转向。每批最多两次。
- 带侧 DISPERSE 与 RELEASE 加 YIELD 共用该选择。20 mm 直退后起弯，
  达到该档航向就收转向，沿用 70 mm 尾段和现有侧偏预算。角度是目标值，实车待验证。

RDK 可沿用当前代码直接联调。现有 `CARGO_AUDIT` 携带左右计数、真实总数和 UNKNOWN 标记，
单侧超过 3 块时现有编码已将侧计数饱和为 3 并置 UNKNOWN。
F405 在接受分离动作时，从已审核的左右计数和现有选边命令推算释放侧，内部计算夹角及转角。

联调时用现有日志核对稳定审核的左右计数、总数和动作方向是否对应。
带侧 `DISPERSE_PILE` 的 TARGET_RIGHT 表示保留右侧，`RELEASE_RIGHT` 表示放开右侧，
后者沿用原流程，开爪完成后发 `YIELD_BACKOFF`。分离后沿用现有 140° 复审和两次预算。
优先保留堆边独立单件、整团未松动时提前退让可作为后续优化，等实际画面证实需要再讨论。

代码参考 [RDK 审核编码](https://github.com/danmo-teng/shijue_fangan/blob/347d787c6b42471d1265380141e1b77a540bd81c/competition_rescue/state_machine.py#L295)、
[RDK 选边](https://github.com/danmo-teng/shijue_fangan/blob/347d787c6b42471d1265380141e1b77a540bd81c/competition_rescue/state_machine.py#L3059)、
[F405 分离准备](../Main/Src/Task.c#L2309)。

## 2026-11-09 正式流程离线复核

复核的 F405 固件源码为 `3b1da3836fdf20533d49f8b94cc5977d7b331cd0`，
RDK 为 `a8ff1a19e308efd84be0ab5c32a271e413e0184e`。本轮测试使用合成状态、报文和硬件替身。

| 流程 | 本轮结果 |
| --- | --- |
| CONFIG、启动和搜寻 | RDK 启动握手用例通过。F405 正常等待 CONFIG，模拟 45° 倒退、转身、225° 前进后进入 SEARCH |
| 普通/成堆靠近、抓取和藏点 | 105°/120° 测距交接、空爪限距、两帧和 STABLE、普通物资及藏点的抓后审核通过 |
| 分离、伤员和运输复审 | 10°/15° 镜像曲线、两次预算、末步等待 260 ms、伤员半闭/后退/复审/再闭合、mode48 分离再审核通过 |
| 导航、扫障和进安全区 | D=0 预备点、扫障审核角色、走廊有障碍发 CLEAR、无障碍发 ENTER 通过 |
| 投送、退出和返回 | 合成区外再区内证据发 COMPLETE，超时分支发 COMPLETE，F405 验收等待后进入退出，D=0 返回 SEARCH |
| 人工停止与故障恢复 | RDK ABORT 回执、F405 ABORT 停车与 mode51 重新定位后回审核通过 |

NormalRun 编译成功。F405 的 5 组主机测试和 24 项 Python 协议测试通过。RDK 的协议脚本及 C
动作去重测试通过。逐字节对照 20 个 opcode、15 组命令配对和 3 组左右数量审核通过。
RDK 的 21 个状态机用例逐项调用后有 8 个通过、13 个失败，已确认其中包括旧的无测距抓取交接、
缺少已确认投送目的地及调用已删除边界接口的夹具。记录为整套未通过，尚未逐个更新旧夹具。
用相机/NPU 替身跑的 11 个 RDK runner 用例有 8 个通过，余下 3 个在 Windows 缺 Linux AF_UNIX
或原生接收器环境。另用当前状态入口模拟 6 个安全区走廊、投送和退出分支，全部通过。

已修复下位机 Python 协议工具对 0x1D/0x1E 的类型校验缺项，并更新 F405 主机替身和相关用例。
板端 C 运行逻辑和 RDK 源码此次沿用原提交。当前本地构建不是实烧身份。

仍需留意的实际策略边界如下。

- 本方安全区连续无框时，RDK 在 5 秒后报告 `SAFE_CORRIDOR_UNAVAILABLE` 并继续等有效画面。
  走廊两次扫障后仍堵塞则进入 `WAIT_SAFE_ZONE_CLEAR`。两种情况均已离线复现。
- 投送视觉未确认且观察达到 1 秒时，上位机按现有策略发 COMPLETE 并计入投送。
  这代表任务进入退出流程，物资是否进区要结合画面验证。

正式部署仍需验证实际 NormalRun、Linux IPC、相机/T265 及真实轮子与舵机动作。
RDK 使用正式比赛启动入口的 921600/8N1。

## UART-100HZ-01 的来源与审核

| 项目 | 本次核对结果 |
| --- | --- |
| 请求 ID | `UART-100HZ-01` |
| 用户批准范围 | UART4 改为 921600/8N1，07 使用每个真实相邻编码器区间，建立固定协作入口 |
| RDK 要求源提交 | `347791666cd3d32ecfe157979fe8e772ff30f4a8` |
| RDK 实现源提交 | `6bfed9ebc794d897f957aa7a253648948077c00a` |
| RDK 分支读取结果 | 核对时 HEAD 为上述 `3477916`，固定入口与该提交内容一致 |
| 本端起始 HEAD | `6302bf2e643c034f5cca694c519be8245dcf5dd2`，与远端 main 一致 |
| 审核结论 | 接受本轮最小方案，源码证实原 10ms 采样被两段汇总为 20ms |
| 代码实现提交 | `14b2a3a9077e6980a540631ce94345e61eb0487d` |
| 构建状态 | MotionDebug 和 NormalRun 均编译、链接成功 |
| 后续授权烧录 | 用户最终指定 MotionDebug，已烧录并通过 OpenOCD Flash verify，NormalRun 未烧录 |
| 板端 actual | Git 与 firmware_id 已从板端 Flash 经 SWD 读回，见下表 |
| 板端 boot_epoch | 已从板端 RAM 经 SWD 读回，见下表 |
| 实收频率、SDK 与地面精度 | 本轮未实测 |

源要求见 [固定源提交](https://github.com/danmo-teng/shijue_fangan/blob/347791666cd3d32ecfe157979fe8e772ff30f4a8/docs/CROSS_REPO_REQUESTS.md)。
审核仅接受本次明确批准范围，来源文件本身不授权新增功能。

## 实现及两端影响

`CMakeLists.txt` 为 MotionDebug 与 NormalRun 共用 `APP_NORMALDEBUG_UPPER_BAUD=921600`。
`board_f405.c` 已按此宏配置句柄 `huart3` 对应的物理 UART4，PC10/PC11、JP506，8N1。
主 gyro UART5 继续为 230400，程序器文本 USART1 继续为 115200。
RDK 部署新固件时须同时选 921600，尚未升级的板仍按其实际固件选择波特率。

`DebugTelemetry_WheelSample` 取消两段累计。首次快照建立基准，此后每次采样生成一个新的记录和 measurement_id。
原始累计计数与 delta 来自 `encoder_f405.c`，保留原始计数符号和回绕处理。
每个记录的 t0/t1 为相邻计数快照各自读数括区的中点，速度使用实测 `t1-t0`。
capture_window_us 为两个端点读数括区宽度的较大值，物理源时间误差仍未知。
原有效位含义、INT32_MIN 无效速度、未知误差字段均沿用现有协议 1.3。

07 仍为车体旋转中心 B 点、车体系 B 轴的前/左速度，单位 µm/s。
轮序为右前 M1、左前 M2、后轮 M3，counts/rev 为 1061.268，轮径为 0.070m。
令 K 为 π×0.070/1061.268 m/count，dt 为区间秒数，则前向为 K×(dn1-dn2)/(√3×dt)，
左向为 K×(dn1+dn2-2×dn3)/(3×dt)。这里使用未翻转的原始计数，不重复应用电机反馈负号。

08 有新源时最高 100Hz。yaw 为 mdeg，rate 为 mdps，均为逆时针正。
独立源 ID、源接收时刻与 NEW/REPEAT 语义沿用原实现。没有新源时不伪造更新。
未知 drift_bound_ppm、source_uncertainty_us 和 gyro uncertainty 继续为 FFFFFFFF。
原源时间事件及未标定状态不因为波特率或发布频率改变而成为标定结论。

协议及 CONFIG_REPLY 的本轮配置值如下。

| 字段或属性 | 当前值 |
| --- | --- |
| 身份协议 major/minor | 1.3 |
| D4 4D envelope version | 1 |
| uart_baud | 921600，读取 UART4 的 Init.BaudRate |
| required_baud | 921600 |
| wheel_period_us | 10000，标称每个原 10ms 调度区间 |
| gyro_period_us | 10000，发布上限 |
| board / mode | 405 / 5 |
| MotionDebug service | 1，MOTIONDEBUG |
| NormalRun service | 2，RESCUE_TASK |
| 07 / 08 长度 | 各 95 字节，布局、单位和 CRC 延用现有定义 |

原控制协议、任务配对、合爪业务、STOP、mode51、电机参数和舵机均由原实现执行。
发送继续使用 `Uart.c` 唯一 DMA 启动者、短快照和未发同类遥测合并。
主循环只消费最新未消费的测量 ID，不补发积压的旧采样。软件或链路忙时允许缺号，不能据配置值宣称实收无丢样。
07/08 共 19000B/s，占 921600/8N1 理论容量 92160B/s 的 20.62%，另需计入原状态和回执。
95 字节的理论线传输约 1.03ms，尚无本轮板端总延迟或抖动实测。
RDK 现有控制/显示 20Hz 快照链的改进由对端后续处理，本次发布频率不等于控制频率。

## 必要检查与构建身份

调用链已核对为 `Robot` 原 10ms 调度 → `Encoder_Sample10ms` →
`DebugTelemetry_WheelSample` → 原主循环 `DebugTelemetry_Process` → `Uart_Send` → 原 DMA 发送。
两个构建均启用 DEBUG_TIMED_LINK，身份 service 仍按实际构建选择。
本轮未新增采样中断、计时检查、诊断框架、协议字段或测试程序。

构建命令使用已有固件目标，提交推送单独执行以保留工作区的无关改动。

```powershell
cmake --preset MotionDebug
cmake --build --preset MotionDebug --target MotionDebug
cmake --preset NormalRun
cmake --build --preset NormalRun --target NormalRun
```

| 构建 | ELF | 编译结果 | 内嵌 Git | source_dirty | firmware_id |
| --- | --- | --- | --- | --- | --- |
| MotionDebug | build/F405/MotionDebug/MotionDebug.elf | 编译、链接成功 | 14b2a3a9077e6980a540631ce94345e61eb0487d | 1 | 6839200f0d0354e8decaa242632e7b637e3d709e349ce3164844a9bf39543c21 |
| NormalRun | build/F405/NormalRun/NormalRun.elf | 编译、链接成功 | 14b2a3a9077e6980a540631ce94345e61eb0487d | 1 | 98dc7fb784f057b543dc05eb84b0044d2fbd3b7333629e952886c05bcf5a2268 |

source_dirty=1 的实际源差异为开始本轮前已存在的 `Main/Src/Task.c` 两行空白，
位于 task_initialize 中 tracking_tick_ms 清零之后，其中一行含两个空格。本轮保留它们。
IDE 修改及 tmp 也保留在工作区，不纳入本次提交。
表内身份来自本地构建生成头，不是板端自报。后续文档提交与这些镜像的内嵌 Git 分开记录。
本地构建日志位于各模式构建目录下的 `UART-100HZ-01-build.log`。
两次构建退出码均为 0，本轮改动文件的 diff 空白检查通过。
编译命令已核对两种构建都将 921600 传入 board_f405.c 和 DebugTelemetry.c，service 分别为 1 和 2。

| 本地 ELF | SHA256 |
| --- | --- |
| MotionDebug.elf | fdbb1249b11ddcd5ea92ab76e84f53a58740b2116849e0c0f1f68ebd893535bd |
| NormalRun.elf | 1e95d0b8b8909fe3774c977235c7333c6341c4e894e041acd0dc4c3d710efeb6 |

## 尚未完成

- RDK 经 UART4 原身份查询读回并记录 actual、source_dirty、boot_epoch、mode/service 与波特率。
- 通过原日志核对不同 measurement_id、相邻约 10ms 的真实 t0/t1 区间及 07/08 实收频率、缺号和延迟。
- RDK 对本次实现提交复核，部署后核对实际 SDK API 接受情况及人工地面验证。

以上实机项目尚未执行，配置回执、编译或 SDK 接口成功均不作为地面精度证明。

## 后续授权烧录与只读读回

2026-11-08 用户将烧录目标改为 MotionDebug，并要求先不启动车轮。
本次使用表内原 MotionDebug.elf，SHA256 与先前构建一致。OpenOCD 已返回 `Verified OK` 并复位运行。
COM18/115200 启动输出确认 F405_ROBO、MODE=5、SERVICE=MOTIONDEBUG。
仅发送 `PING`、`STAT`。状态为 IDLE，三轮 TARGET、ACTUAL、PWM 均为 0。

| 板端读取项 | 实际结果 | 依据 |
| --- | --- | --- |
| actual Git | 14b2a3a9077e6980a540631ce94345e61eb0487d | SWD 读 Flash 0x0801ba70 的 20 字节 |
| actual firmware_id | 6839200f0d0354e8decaa242632e7b637e3d709e349ce3164844a9bf39543c21 | SWD 读 Flash 0x0801ba50 的 32 字节 |
| source_dirty | 1 | 已校验镜像的内嵌值，尚未经 UART4 ID_REPLY 自报读取 |
| 板型 / mode / service | F405_ROBO / 5 / MOTIONDEBUG | COM18 实际启动与 STAT 回执 |
| UART4 波特率配置 | 921600/8N1 | SWD 读运行中 huart3 的实例地址及 Init 字段 |
| UART4 BRR | 0x0000002D | SWD 读寄存器 0x40004c08，未测物理位宽 |
| boot_epoch | 2b40340370e209faed7d2123a1015296 | SWD 读 RAM 0x2000048c 的 16 字节 |

本次 UART4 没有连接第二个主机发送 ID_REQ，不能把 SWD 读回说成 UART4 身份查询已通过。
波特率表项为实际运行配置，主机应设 921600，物理线速未用仪器测量。
本地原始日志位于 `build/F405/MotionDebug/UART-100HZ-01-flash-20261108.log`、
`UART-100HZ-01-console-20261108.log` 和 `UART-100HZ-01-swd-readback-20261108.log`。
07/08 订阅、100Hz 实收、SDK 接受和地面精度仍待验证。
