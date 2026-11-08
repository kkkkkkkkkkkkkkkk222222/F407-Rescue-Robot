# 两端当前要求与回执

更新日期为 2026-11-08，时区 Asia/Shanghai。

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
| 本轮烧录 | 未烧录 |
| 板端 actual / boot_epoch | 本轮未连接串口或下载器读取，未知 |
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

- 部署及通过原身份查询亲自读取板端 actual、source_dirty、boot_epoch、mode/service 与波特率。
- 通过原日志核对不同 measurement_id、相邻约 10ms 的真实 t0/t1 区间及 07/08 实收频率、缺号和延迟。
- RDK 对本次实现提交复核，部署后核对实际 SDK API 接受情况及人工地面验证。

以上实机项目尚未执行，配置回执、编译或 SDK 接口成功均不作为地面精度证明。
