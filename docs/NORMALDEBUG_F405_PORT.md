# NormalDebug：F405 工训 ROBO 控制板移植

日期：2026-10-21。依据用户提供的《STM32F405竞赛控制板用户说明书(1)》V1.0，
目标为 STM32F405RGT6、LQFP64、8MHz HSE、168MHz、1MB Flash。
用户确认保留外置单轴串口陀螺仪，不接 LCD；不改用板载 LSM6DS3TR。

## 配置与保留范围

```powershell
cmake --preset NormalDebug
cmake --build --preset NormalDebug
```

烧录 `build/NormalDebug/WWW.elf`；同时生成同目录的 HEX、BIN。
下载器选择 **STM32F405RG / STM32F405RGT6**，SWD；Horco CMSIS-DAP + 当前
OpenOCD `dap.cfg` 的通用 `stm32f4x.cfg` 也支持该系列。不要使用旧 Ozone 工程中
固定的 STM32F407VE 和 `build/Debug/WWW.elf`，也不要把 NormalDebug 烧到旧 F407 板。

NormalDebug 是应用模式 5 + 独立 `F405_ROBO` 板级构建。
**现默认运行与旧MotionDebug共享的 `Debug.c` / `WheelCheck.c`，不执行比赛Task，
不会因收到比赛配置启动搜寻/抓取，也不自动启动舵机。** 两个串口可以分别用于程序器
文本测试和上位机运动测试；上位机0x19命令、0x1A状态、0x15原始编码器累计计数均不变。

先前移植的F405比赛入口保留为编译开关：

```powershell
cmake --preset NormalDebug -DAPP_NORMALDEBUG_MOTION_TEST=OFF
cmake --build --preset NormalDebug
```

此时才直接执行共享 `Task.c` 的抓取、分离、审核、投送、返中流程。
再次配置 `cmake --preset NormalDebug` 会恢复预设的运动测试ON，重新编译并烧录后才生效。
不做运行中热切换，避免测试指令误启动比赛程序。

保留 MG310 每转 1061.268 计数、70mm 轮径、当前 PID/运动极性、60%驱动占空比上限、
关闭堵转联停的配置；本次不加运行故障锁存。三轮仍为 M1 右前、M2 左前、M3 后轮。
NormalRun、MotionDebug、Gamepad 仍使用旧 F407 板级配置，不改变默认配置。

F405 专用启动文件、CMSIS 器件头和 1MB 链接脚本单独加入；CMSIS 头/启动文件来自
ST 官方 `cmsis-device-f4` v2.6.8，与原工程 CMSIS 版本一致，保留原许可证。

## 电机与编码器

| 车轮 | 插座 | PWM | 方向 IN1/IN2 | 编码器 A/B |
| --- | --- | --- | --- | --- |
| 右前 M1 | JP300 | PA0 / TIM5_CH1 | PA1 / PA2 | PB14 / PB15，TIM12 TI12 |
| 左前 M2 | JP301 | PA5 / TIM2_CH1 | PA3 / PA4 | PC6 / PC7，TIM8 TI12 |
| 后 M3 | JP302 | PA6 / TIM3_CH1 | PA7 / PC4 | PC8 / PC9，EXTI双边沿四倍频 |

三路都是 **20kHz 独立硬件 PWM**。PA0 不采用手册列出的 TIM2_CH1，而采用官方
复用表支持的 TIM5_CH1，因此不与 PA5 的 TIM2_CH1 共占一个比较寄存器。

TB6612 使用独立 PWM 引脚 + IN1/IN2，不能沿用 AT8236 的反相双输入 PWM：
正转 IN1/IN2=10，反转=01，PWM 关断期间芯片短路制动；主动制动用 IN1/IN2=11。
普通停止用 IN1/IN2=00 且 PWM 高电平，实现高阻滑行。
所以普通停车时示波器可能看到 PWM 引脚保持高电平，这不是100%驱动车轮，实际驱动为0。
变化方向/占空比时先关闭通道输出，再更新方向及比较值，恢复输出。

JP300~302 六针顺序：1=OUT1，2=GND，3=ENC_B，4=ENC_A，5=3V3，6=OUT2。
**不能把原来的5V编码器供电直接接进3.3V信号，也不能带电换线序。**
先确认 MG310 霍尔编码器在3.3V供电下能稳定输出；不兼容时需按电机资料另供电并转换A/B电平。

ENC3 并非 TIM8 的“另一组硬件编码器”，不能用 CH3/CH4开启独立 TI12。
本移植按 PC8/PC9 双边沿软件四倍频采集，计数方向与 STM32 硬件 TI12 一致：
A 上升且 B 低时计数加一，仍使用原每转计数。非法双位跳变计入 `M3_INVALID`，
只做诊断，不猜测漏计数，也不触发新的停车锁存。

## 舵机重新接线

| 原逻辑ID/功能 | 新板插座 | 信号 |
| --- | --- | --- |
| S1 升降 | JP304 | PC13 |
| S2 右爪 | JP307 | PC0 |
| S3 摄像头 | JP308 | PC1 |
| S4 左爪 | JP309 | PC2 |

所有插座 Pin1=信号、Pin2=+5V_MOTOR、Pin3=GND。
**原逻辑ID不等于新板插座SERVO编号**，必须按此表插接。
JP305/JP306 的 PC14/PC15 当前不使用；高频调整的两爪及摄像头优先放到普通 PC0~PC2。

手册称 PC0/PC1/PC2 可复用 TIM5_CH1/2/3，与 ST 官方复用表不符。
采用 TIM4 内部比较中断控制 GPIO，1us计时、20ms周期、500~2500us脉宽；
比较值预装载，下一帧统一生效，没有主循环软件延时，不抢占电机或编码器定时器。
PC13 使用最低 GPIO 速度；该脚有电流/负载电容限制，升降舵机信号线应短，
如输入负载超出官方限制需增加缓冲，软件无法消除电气限制。

仅USB供电没有+5V_MOTOR，舵机不转不等于控制指令无效。

## 三路串口接线

均为115200、8N1、3.3V逻辑、共地。

| 用途 | 新板接口 | 本板RX | 本板TX |
| --- | --- | --- | --- |
| RDK上位机 | JP506 / UART4 | Pin4 PC11 | Pin5 PC10 |
| 单轴陀螺仪 | JP507 / UART5 | Pin4 PD2 | Pin5 PC12 |
| 程序器文本调试 | JP500~505任一I2C插座改用USART1 | Pin5 PB7 | Pin4 PB6 |

JP506/JP507 Pin1=5V_LOGIC，Pin2=3V3，Pin3=GND。
陀螺仪VCC接JP507 Pin1，模块TX接Pin4，模块RX接Pin5；不要沿用旧板PD5/PD6。
陀螺仪协议、角度展开、输出速率200Hz和方向配置沿用上次移植，不重复积分。

最新接线统一到同一个JP500~505插座：程序器TX接Pin5/PB7，RX接Pin4/PB6，GND接Pin3。
使用3.3V TTL；不要把程序器电源线接到Pin1/Pin2。本次不再使用JP303的PA9，
但第四路电机仍未加入底盘控制，不能把它当成已支持的第四轮。

**此版为直连条件下的UART复用验证，板载IMU仍与PB6/PB7物理共线。**
用户确认程序器TX直连PB7、没有串联电阻，因此固件不在启动时临时将PB7设为输出写I2C，
避免与程序器TX发生主动驱动争用；也没有声称关闭或隔离板载IMU。
启动提示 `ONBOARD_IMU_I2C=SHARED_NOT_DISABLED`，先做发请求后完整接收回执的PING/STAT检查，
不要定时发送，不把该检查结果当成连续日志、并发收发或运动过程下的可靠性证明。
**PB6/PB7整条总线不能再接I2C模块或另一主机。** 若后续出现异常或要正式可靠使用，
需进一步隔离板载IMU，或在确认限流/时序条件后关闭其I2C接口；不是删除IMU初始化即可解决。
板载USB未移植CDC，插USB不会因此自动出现程序器串口。

上位机协议及帧序不变；仅把原来的RDK串口线接到JP506。
内部 `huart3` 为历史逻辑上位机句柄，新板实际绑定UART4及DMA1 Stream2 Channel4，
不表示仍然连接USART3。陀螺仪UART5走独立接收中断。

## 无LCD检查方法

默认运动测试配置上电显示 `NORMALDEBUG BOARD=F405_ROBO SERVICE=MOTIONDEBUG`、
`MODE=5` 及 `NO AUTOMATIC MOTION`。串口115200 8N1，每条命令需要回车/换行。
命令解析、接收恢复、异步发送、DIAG和FINAL回执复用旧MotionDebug：

| 命令 | 含义 |
| --- | --- |
| `PING` / `STAT` | 串口健康、当前运动状态及三轮目标/实际速度 |
| `IMU` / `GYRO` / `ENC` / `MOTOR` | UART5陀螺仪、累计角度及三轮反馈 |
| `MOVE 200 150` | 前进200mm，目标速度150mm/s |
| `MOVE -200 150` | 后退200mm，目标速度150mm/s |
| `TURN 60 30` / `TURN -60 30` | 正/负向60度，目标角速度30度/秒 |
| `TURN 180 60` / `TURN 360 60` | 单次相对旋转180/360度 |
| `CIRCLES 3 60` | 固定参考连续三圈，圈间静止等待 |
| `BODY 200 100 150` | 车体坐标前200mm、左100mm的定距测试 |
| `FIELD 200 0 150` | 场地坐标方向的位移测试，非比赛导航 |
| `WHEELS` | 架空后依次M1右前、M2左前、M3后轮，25%各400ms |
| `CAL 1 100 500` | 架空后M1用正10%测试500ms；沿用原CAL租约上限 |
| `DIAG` / `TRACE 0` / `TRACE 1` / `CRASH` | 控制/循环/串口陀螺仪诊断、跟踪及异常记录 |
| `STOP` / `HOLD` / `RESET` | 停车取消、保持、静止重设里程计参考 |

先架空验证WHEELS与反馈极性，再落地短距离测试。每条运动命令等 `FINAL` 再发送下一条。
STOP可以随时取消；参数范围和超时仍与旧MotionDebug一致。测试模式下不支持
`DEBUG` / `S` / `RUN`，不会为了运动调试将全部舵机突然驱动到90度。

`IMU_UART`显示UART5 PC12/PD2，PING中的上位机链路显示UART4，而不是旧板UART3。
CPU异常记录也使用F405实际电机引脚，不访问旧GPIOE驱动引脚。

如果编译开关设为OFF恢复比赛入口，仍使用以下原维护命令：

```text
PING
STAT
IMU
ENC
MOTOR
DEBUG
S 3 105
RUN
```

此OFF配置下，PING不运动；STAT/IMU/ENC/MOTOR返回任务和传感器快照。
DEBUG停车并暂停自动任务，随后S 3 105设置摄像头105度；RUN沿用原行为，复位主控，
需要上位机重新配置。这些舵机命令仅适用于OFF比赛配置，不适用于默认运动测试。

## 尚需硬件核对

1. 未在公开搜索中找到可确认与此手册同版、同位号的完整厂家源码/原理图。
   不套用其它F405飞控或其它工训板的引脚。实际板丝印若与本手册不一致需重新核对。
2. 手册未列两颗TB6612的STBY具体连接，固件没有虚构STBY GPIO。
   若PWM/IN正确仍不转，需要确认两颗芯片STBY在使能电平。
3. TB6612额定持续电流余量较小，官方工作条件约1A、绝对最大平均1.2A，
   峰值电流带有短时条件，不能把3.2A视为连续能力。MG310堵转电流可能超过它，
   仍需实测启动/带载温升。本次不增加用户已拒绝的堵转锁存。
4. 保留原PID只是移植起点；驱动、电源压降、编码器输入和接线改变后需要实车核对，
   不能保证首次上电闭环响应与AT8236完全一致。
5. 首次不接RDK自动程序，先检查串口、陀螺仪和手转三轮的编码器方向，再架空低功率联调。
   仅USB无电机/舵机电源；所有针脚接线应断电完成。

参考：
- [ST STM32F405/407 DS8626，复用表及PC13限制](https://www.st.com/resource/en/datasheet/stm32f405rg.pdf)
- [Toshiba TB6612FNG，真值表与电流条件](https://toshiba.semicon-storage.com/info/docget.jsp?did=10660)
- [ST CMSIS F4 v2.6.8](https://github.com/STMicroelectronics/cmsis-device-f4/tree/v2.6.8)

## 验证范围

对F405实际底层代码进行模拟寄存器检查：独立PWM、正反转/制动/滑行/限幅、
编码器四倍频符号/回绕/非法跳变、舵机映射/105度脉宽/延迟中断。
构建NormalDebug并回归构建旧板NormalRun、MotionDebug、Gamepad，检查F405向量和链接容量。
这些不是示波器、电气检查或实车测试，本次没有烧录或驱动车辆。
