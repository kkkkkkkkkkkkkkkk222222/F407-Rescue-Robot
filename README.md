# 当前固件：IMU实时角度显示测试

当前配置设置为`APP_ENABLE_MOTION_TEST=1`，`APP_ENABLE_TASK`会自动变为0。该模式不需要上位机赛前配置帧，但会完整初始化IMU660RC：上电时车辆必须保持静止，128个样本零偏校准通过后LCD显示`IMU660RC: OK`一秒，再进入`IMU MONITOR`界面；校准或通信失败则显示`IMU660RC: ERROR`。当前监视测试不会向任何电机发送运动命令。

测试步骤：

1. 上电后保持小车和IMU完全静止，等待LCD显示`IMU660RC: OK`一秒；若显示`ERROR`，检查3.3 V、共地和SPI接线。
2. 进入`IMU MONITOR`后用手水平旋转整车，LCD每100 ms刷新相对偏航角`YAW`和Z轴角速度`GYRO Z`，IMU仍按10 ms采样。
3. `MOTOR`固定显示`STOP`，按S1不会启动电机；测试时只验证零偏、角度方向、累计角度和静止漂移。
4. 若运行中IMU掉线或连续250 ms没有有效样本，`STATE`显示`IMU ERR`，必须检查接线并复位重新校准。

封装好的定角度旋转参数集中在`Main/Inc/app_config.h`的`APP_MOTOR_TURN_*`配置中，但当前监视测试不调用该函数。测试结束、准备恢复完整任务时，只需将`APP_ENABLE_MOTION_TEST`改为0，`APP_ENABLE_TASK`会自动恢复为1。

# 代码思路（救援流程与上位机协议）

当前固件实现赛前配置、一键驶出、搜索、分类靠近、抓取合规检查、上位机引导返航、进入安全区、放下目标、低头复核和后退重搜。上位机负责从图像判断路线和安全区位置，F407负责命令超时停车、目的地校验、连续帧确认、编码器定距进入/退出和机构动作；上位机不能直接控制PWM。

IMU660RC已经作为独立底层传感器移植：F407按10 ms释放节拍通过四线软件SPI读取LSM6DSV16X三轴角速度和加速度，并在完成静止零偏校准后积分Z轴偏航角。`Motor_TurnAngle()`已经封装IMU定角度旋转，但当前独立测试只实时显示`IMU_GetData().yaw_mdeg`，不会驱动电机；正式救援状态机也暂未调用该函数。

协议设计采用固定长度、消息类型、递增序号和整帧CRC16。固定帧适合当前STM32的64字节循环DMA；CRC16用于拒绝误码帧；坐标、距离、类别数量和状态放在同一视觉报告中，避免旧协议把不同采样周期的三帧拼在一起。导航采用“连续命令+失联停车”，与[ROS 2控制器的过期速度命令自动停车](https://control.ros.org/master/doc/ros2_controllers/diff_drive_controller/doc/userdoc.html)和[Nav2传感器超时即停车](https://docs.nav2.org/configuration/packages/collision_monitor/configuring-collision-monitor-node.html)的失效安全思路一致。串口设计参考：[ST UART Receive-to-IDLE DMA](https://dev.st.com/stm32cube-docs/hal1-to-hal2-migration/1.0.0/en/docs/markup/drivers_documentation/hal_drivers/uart/hal_uart_exported_functions_io_operation.html)、[Modbus串口CRC规范](https://modbus.org/docs/Modbus_over_serial_line_V1_02.pdf)、[IETF RFC 1662 FCS](https://www.rfc-editor.org/info/rfc1662/)。比赛规则以[2027智能+工程创新赛道官方解析](https://gcxl.edu.cn/new/res/intelligence20260617.pdf)为准。

## 固定15字节协议

USART3使用115200 8N1。上位机发给F407的每帧固定15字节：

```text
索引:  0  1   2    3   4  5  6  7  8  9 10 11   12     13    14
数据: A3 B3 TYPE  SEQ P0 P1 P2 P3 P4 P5 P6 P7 CRC_LO CRC_HI C3
```

| 字段 | 说明 |
| --- | --- |
| `A3 B3` | 固定帧头 |
| `TYPE` | 高4位为协议版本1，低4位为消息类型 |
| `SEQ` | 0～255循环递增；上位机每发一帧都加1 |
| `P0..P7` | 8字节消息载荷 |
| `CRC_LO CRC_HI` | 对`TYPE、SEQ、P0..P7`共10字节计算Modbus CRC-16，初值`FFFF`、多项式`A001`，低字节先发 |
| `C3` | 固定帧尾 |

F407只有在帧头、帧尾、消息类型和CRC全部正确时才更新数据快照。CRC错误、未知类型和填充字节不为0的控制帧全部丢弃。上位机可直接复用[tools/vision_protocol.py](tools/vision_protocol.py)生成帧，避免两端CRC和字节序不一致。

## `TYPE=0x11`：赛前配置

```text
P0=颜色，P1=半区，P2..P7=00
```

| 类别 | 代码 | 含义 |
| --- | --- | --- |
| 颜色 | `0x11` | 红方 |
| 颜色 | `0x12` | 蓝方 |
| 半区 | `0x01..0x04` | 1～4号半区 |

同一配置必须连续发送3帧，载荷相同且`SEQ`每帧加1（`FF`后回到`00`）。例如红方1号半区：

```text
A3 B3 11 00 11 01 00 00 00 00 00 00 F0 57 C3
A3 B3 11 01 11 01 00 00 00 00 00 00 FD C7 C3
A3 B3 11 02 11 01 00 00 00 00 00 00 E9 37 C3
```

配置接受后，F407仍按之前约定连续回复3次4字节ACK：

```text
A3 B3 01 C3
```

重复、倒序或跳变的`SEQ`不会累计配置次数；三帧之间出现其他消息、CRC错误或配置变化会重新计数。配置确认后会锁定颜色和半区，只有复位MCU才能重新配置。

## `TYPE=0x12`：原子视觉报告

上位机每30～50 ms只发送一帧视觉报告：

| 载荷 | 内容 | 字节序 |
| --- | --- | --- |
| `P0 P1` | 目标中心X，0～639 | 大端 |
| `P2 P3` | 目标中心Y，0～479 | 大端 |
| `P4 P5` | 目标距离mm；`FOUND=1`时必须为1～65535 | 大端 |
| `P6` | 抓取ROI内四类目标的数量 | 位打包 |
| `P7` | 识别和抓取状态 | 位标志 |

`P6`每类使用2位：

| 位 | 数量 |
| --- | --- |
| bit0～1 | 普通物资，0～3 |
| bit2～3 | 核心物资，0～3 |
| bit4～5 | 伤员，0～3 |
| bit6～7 | 危险目标，0～3 |

`P7`定义：

| 位 | 名称 | 含义 |
| --- | --- | --- |
| bit0 | `FOUND` | 当前存在选中的目标/抓取ROI内目标 |
| bit1 | `NEAR` | 已到抓取距离 |
| bit2 | `GRABBED` | 目标已稳定进入地面导向/围挡机构，仍与地面接触 |
| bit3 | `CLASS_VALID` | 四类数量识别可信 |
| bit4 | `UNKNOWN` | ROI内还存在无法分类的物体 |
| bit5 | `CLAW_VIEW` | 当前报告来自低头后的夹内检查ROI |
| bit6～7 | 保留 | 必须为0 |

`GRABBED=0`时，P6只统计当前选中目标的抓取ROI，不得统计整张画面的所有目标；`GRABBED=1`时，P6必须统计机构内全部目标。没有目标时清除`FOUND/GRABBED/NEAR/UNKNOWN`并把P6置0；低头检查确认夹内为空时同时置`CLAW_VIEW=1、CLASS_VALID=1`。F407会拒绝标志与数量互相矛盾、目标坐标越界以及`FOUND=1`但距离为0的报告；250 ms内没收到新的有效视觉报告就停车，不会沿用旧坐标。

目标位于`(320,240)`、距离350 mm、识别为1个普通物资：

```text
A3 B3 12 10 01 40 00 F0 01 5E 01 09 7C 3D C3
```

抓住1个普通物资并稳定分类：

```text
A3 B3 12 11 01 40 00 F0 00 64 01 0D 51 9F C3
```

## `TYPE=0x13`：安全事件

### 永久停车

```text
P0=01，P1..P7=00
A3 B3 13 30 01 00 00 00 00 00 00 00 14 50 C3
```

收到后进入`STOPPED`，只能复位MCU恢复。

`TYPE=13`目前只保留永久停车。是否成功放下不再由一个“已送达”事件直接宣告，而是由F407在实际进入安全区、打开机构并连续看到3帧夹内为空之后确认，避免上位机误报成功。

## `TYPE=0x14`：安全区导航

```text
P0=运动方向
P1=安全区接近状态
P2=目的地区类型
P3..P7=00
```

`P0`定义：

| 代码 | 动作 | F407行为 |
| --- | --- | --- |
| `00` | HOLD | 停车等待 |
| `01` | FORWARD | 向前行驶 |
| `02` | TURN_LEFT | 原地左转 |
| `03` | TURN_RIGHT | 原地右转 |
| `04` | BACKWARD | 向后行驶 |

`P1`定义：`01`表示仍在去安全区的路上，`02`表示已到正确安全区附近。`P2=01`表示物资区，`P2=02`表示伤员区，必须与F407当前货物目的地一致。上位机说“到附近”时必须发`P0=00、P1=02`，不能一边声明到达一边继续给运动命令。

上位机在`RETURN_SAFE`中每30～50 ms发送一次导航帧，并持续递增`SEQ`。F407只执行最近200 ms内的帧；相同`SEQ`的重复帧不会刷新时间戳，因此上位机卡死重发旧缓存时仍会自动停车。超时、CRC错误、非法方向、目的地不匹配时立即停车但仍夹紧货物。到附近必须连续3帧`P0=00、P1=02`且序号逐帧加1，随后才进入放置流程。

导航帧只给方向，不给PWM。F407当前把前进、后退、旋转分别映射为388、224和179 mm/s，再经过`Motor_Move()`三轮运动学和10 ms轮速PID；`Motor_Move(forward_mm_s, lateral_mm_s, rotate_mm_s)`三个参数全部统一为mm/s，超过轮速上限时三轮同比缩放。左右方向必须先悬空轮胎低速核对，方向相反时修改映射，不能让上位机把“左”和“右”互换来掩盖底层配置错误。

例如，序号`20`、去物资区、直行：

```text
A3 B3 14 20 01 01 01 00 00 00 00 00 79 5B C3
```

随后确认已到物资区附近（此帧必须停车）：

```text
A3 B3 14 21 00 02 01 00 00 00 00 00 86 07 C3
```

实际程序不要手填CRC，直接调用`tools/vision_protocol.py`中的`nav_frame()`。

## 返航、放置和复核闭环

1. 合法货物确认后进入`RETURN_SAFE`，夹爪保持闭合、摄像头回到90度广角。
2. 上位机依据本车颜色、当前位置、货物类别和安全区图像持续输出`HOLD/FORWARD/TURN_LEFT/TURN_RIGHT/BACKWARD`。F407只映射为限速底盘动作，上位机不发送占空比。
3. 导航中任何通信中断都会停车；恢复收到目的地正确的新帧后可继续，不会丢弃当前货物。
4. 连续3帧确认到安全区附近后进入`DROP_OBJECT`，F407不再接受导航动作，使用编码器向前0.20 m，使目标越过区域边界。
5. 停车、打开夹爪并等待600 ms；摄像头转到0度低头角，等待400 ms让舵机和图像稳定。
6. 上位机只检测夹爪/导向机构ROI，并发送连续3帧带`CLAW_VIEW`的视觉报告：夹内空为`FOUND=0、P6=0、CLASS_VALID=1`；仍有目标则`FOUND=1、P6=实际剩余数量、CLASS_VALID=1`。模糊或遮挡但能确定ROI内有物体时置`FOUND=1、UNKNOWN=1`。进入复核后3秒内未形成连续3帧一致结论，F407会把摄像头转回90°并重新执行打开、低头和复核；最多重试2次，仍失败则进入`STOPPED`，不会猜测放置成功或永久等待。
7. 连续3帧确认夹内为空后才计为送达；摄像头回90度，夹爪保持打开，编码器后退0.50 m，回到`FIND_OBJECT`继续旋转搜索。
8. 若连续3帧确认夹内仍有货物，则重新闭合夹爪、后退0.50 m回到`RETURN_SAFE`，由上位机重新对准并再放一次。剩余货物不合规或电机故障时进入`STOPPED`，防止带着不确定货物继续高速运动。

当前实车参数为`进入0.20 m、退出0.50 m、夹内检查0°`：0.20 m要保证目标越过边界但车体不压线，0.50 m要保证旋转时机构不扫到已放下目标，0°要保证夹内ROI完整进入画面。摄像头软件最小限位已同步设为0°，因此检查命令不会再被夹到10°。

当前F407没有激光雷达、碰撞开关或本地地图，无法独立判断返航路径上突然出现的障碍；上位机识别不确定、目标被遮挡或路线被占用时必须发送`HOLD`，不能沿用上一次方向。200 ms通信看门狗、电机堵转/反向故障和180秒总超时是底层最后保护，但不能替代上位机的避障判断。

## 抓取顺序与合规判断

初赛目标为普通物资、核心物资、伤员和危险目标。所有救援目标是双方公共目标，红蓝只表示安全区，不存在“对方颜色目标”；因此协议不再设置`A2/B2`颜色判断，也不允许`E3=任意目标都可抓`。

F407执行以下硬规则：

1. 第一次成功送达普通物资前，只选择普通物资；抓到1～3个普通物资可进入`RETURN_SAFE`。
2. 普通物资送达后，普通与核心物资合计1～3个可送物资区。规则没有禁止二者混运，当前代码允许混运。
3. 伤员必须单独转运，数量必须正好为1，目的地为伤员区。
4. 危险数量非0、存在未知目标、总数为0、总数超过3或伤员与其他目标混装，都判为非法货物。
5. 抓取分类必须连续3个`SEQ`逐帧加1的报告保持相同，才做最终决定。重复、乱序或丢帧会从当前帧重新计数；非法货物会开机构、后退250 ms并重新搜索。
6. 第一趟建议抓最近的1个普通物资立即送回以尽快解锁，不必等待装满3个；之后再按距离和剩余时间选择核心、伤员或普通物资。

## 小车状态

TIM6每20 ms发布一次`Task_FindObject(now_ms)`运行请求，由最低优先级PendSV非阻塞执行：

1. `WAIT_CONFIG`：停车等待3帧配置并回复3次ACK。
2. `START`：等待PA0/S1；启动180秒倒计时，以300 mm/s执行`Go_distance(0.7f)`。
3. `FIND_OBJECT`：等待700 ms后原地搜索；只接近当前阶段允许且分类明确的单个目标，危险/未知/混杂候选不会进入靠近状态。
4. `CRAB_OBJECT`：按X修正底盘、按Y调整摄像头、按距离分段减速；闭合机构后进行3帧货物确认。合法则进入`RETURN_SAFE`，非法或抓取失败则开机构、后退并重搜。
5. `RETURN_SAFE`：夹紧目标，按最新且目的地匹配的`TYPE=14`帧前进、转向、后退或停车；导航超过200 ms未更新立即停车。连续3帧确认到区后进入放置。
6. `DROP_OBJECT`：编码器前进0.20 m、打开机构、摄像头转到0°检查夹内；空夹确认后退0.50 m重新搜索，仍有货物则夹回并后退后重新返航。
7. `STOPPED`：180秒结束、永久停车事件、电机故障或复核出非法剩余货物后保持停车。

S4默认开角90度、闭角30度。安装机构后必须先断开机构负载标定角度，确认不会顶死舵机。

LCD所有阶段始终显示当前状态，并把最近一帧通过CRC检查的完整15字节数据拆成`RX0..RX3`四行；其余5行随状态显示配置、时间、累计行驶距离、分类数量、返航方向、导航超时、到区状态、放置阶段或夹内复核结果。`FIND_OBJECT`的`DIST`由三轮编码器增量反解底盘前向/横向位移后累计路径长度，不是摄像头目标距离，也不会因掉头返回而相互抵消；UART状态在最后一个接收字节超过250 ms后显示`TIMEOUT`。动态值每100 ms刷新，只有状态改变时清屏重画布局。所有布局、文案和动态数据显示都封装在`Lcd.c`，`Robot.c`只负责调度刷新。

底层仍为：编码器和速度PID严格每10 ms在TIM6中断执行；IMU每10 ms释放一次主循环采样请求，主循环延迟时合并为最新一次而不补跑旧采样；任务由TIM6严格每20 ms发布、最低优先级PendSV消费，延迟时只执行最新一次；LCD每100 ms刷新；USART3使用64字节循环DMA，不申请动态内存。PendSV可被TIM6、DMA和USART3抢占，高层状态机不会再占用电机实时中断。

---

# CLion + DAPLink (STM32F407)

This project is configured for the YLJ2000 `DAP_HS_ESP_Open` / Horco CMSIS-DAP v2 probe. OpenOCD connects to the STM32F407 through SWD.

## Wiring

Connect `SWDIO`, `SWCLK`, `GND`, `VTref/3V3`, and preferably `NRST` between the DAPLink receiver (or the single probe in wired mode) and the STM32F407 board. Do not use the probe to power a target whose supply requirements exceed the probe/board rating.

## CLion

Select **DAPLink · OpenOCD** in the Run/Debug configuration list.

- Run flashes `build/Debug/WWW.elf` and resets the MCU.
- Debug rebuilds, flashes, resets, and attaches `arm-none-eabi-gdb` on port 3333.
- The board configuration is the project-root `dap.cfg`.

The existing CLion debug profile named `DAP` is unrelated: it means Debug Adapter Protocol, not DAPLink/CMSIS-DAP. Pointing that profile at port 3333 sends `Content-Length`/JSON messages to a GDB server and cannot work. Use the `DAPLink · OpenOCD` run configuration with GDB instead.

OpenOCD 0.12 warns about CLion's legacy `tcl_port`, `gdb_port`, and `telnet_port` command spelling. The project-root `dap.cfg` defines compatible aliases, so these deprecation warnings are suppressed. OpenOCD still writes normal informational output to stderr, which some CLion color schemes render in red; red text alone does not mean failure.

## Copy to another Windows PC

Copy the project, including these files:

- `dap.cfg`
- `.idea/runConfigurations/DAPLink_OpenOCD.xml`
- `setup-clion-daplink.ps1`

Install or copy OpenOCD and Arm GNU Toolchain on the new PC, start and close CLion once, then run from PowerShell in the project root:

```powershell
powershell -ExecutionPolicy Bypass -File .\setup-clion-daplink.ps1
```

The script detects `openocd.exe`, the newest CLion settings directory, and the CMake project/target name. Debugging uses CLion's bundled GDB because it includes the Python support required by CLion. If OpenOCD auto-detection fails, pass its path explicitly:

```powershell
powershell -ExecutionPolicy Bypass -File .\setup-clion-daplink.ps1 `
  -OpenOcd 'D:\Toolchain\OpenOCD\bin\openocd.exe'
```

## First connection check

Power the target, connect the USB/transmitter side, and verify that Windows shows `Horco CMSIS-DAP v2`. If it finds the probe but cannot read the target, check ground, VTref, SWDIO/SWCLK, and NRST; then temporarily lower `adapter speed` in the board configuration from 2000 to 1000.

---

# F407 固件说明

上面的 DAPLink/CLion 配置继续保留。本节是当前 STM32F407 固件、三轮底盘和测试行为的绝对说明；LED_3 旧工程只作为算法来源，不能覆盖这里的引脚、定时器和安全逻辑。

## 当前上电行为

1. M1-M3六路PWM以0占空比启动，四路舵机输出90度，三路硬件编码器开始计数。
2. IMU660RC先检查`WHO_AM_I=0x70`，随后在车辆静止时采集128个陀螺仪样本校准零偏，正常约需1.1秒。
3. LCD初始化完成后显示`IMU660RC: OK`一秒；连接失败时显示`IMU660RC: ERROR`一秒。提示结束后清屏并切换到当前任务状态界面，不执行RGB色块测试。
4. TIM6提供1 ms基础节拍：每10 ms采样编码器并执行一次电机速度环，同时只释放一次IMU主循环采样请求；自主任务开启时每20 ms向最低优先级PendSV发布一次任务请求；每100 ms只发布一次LCD刷新请求。
5. USART1已经停用，PA9恢复为空闲引脚；USART3使用PD9 RX和64字节循环DMA接收视觉帧，PD8 TX只发送3帧赛前配置确认。
6. 当前配置为`APP_ENABLE_TASK=1`、`APP_ENABLE_AUTOMATIC_MOTOR_TEST=0`。上电后停车等待颜色和半区配置；配置确认后等待PA0/S1一键启动。

接通电机12 V前应确认三轮与编码器方向正确，并给车辆留出安全空间；只有配置成功后按下S1才开始180秒倒计时并定距前进0.7 m。

## 自主救援流程

完整流程只公开一个非阻塞入口`Task_FindObject(now_ms)`。TIM6每20 ms发布节拍，最低优先级PendSV执行高层状态机，电机10 ms闭环仍留在TIM6中断。当前状态为`WAIT_CONFIG`、`START`、`FIND_OBJECT`、`CRAB_OBJECT`、`RETURN_SAFE`、`DROP_OBJECT`和`STOPPED`，详细数据帧、抓取规则、LCD内容及返航放置流程以README最前面的“代码思路”为准。

## 引脚与外设

| 功能 | 引脚/外设 | 参数 |
| --- | --- | --- |
| M1 | PA2/PA3 TIM5 CH3/CH4；PE9/PE11 TIM1编码器 | 20 kHz，照片左下、舵机方向前轮 |
| M2 | PE5/PE6 TIM9 CH1/CH2；PA6/PA7 TIM3编码器 | 20 kHz，照片左上轮 |
| M3 | PB10/PB11 TIM2 CH3/CH4；PD12/PD13 TIM4编码器 | 20 kHz，照片右侧轮 |
| 舵机S1-S4 | PC6-PC9，TIM8 CH1-CH4 | 50 Hz，500-2500 us |
| LCD | PB13 SCK、PB15 MOSI、PB12 CS、PB14 RESET、PC5 DC、PB1 BL | ST7735，128x160 |
| IMU660RC | PE2 SCK、PE3 MISO、PE4 MOSI、PC13 CS | 4线软件SPI Mode 3，120 Hz采样 |
| USART3 | PD8 TX、PD9 RX | 115200 8N1；RX为64字节循环DMA，TX只回3帧配置确认 |
| 启动按键 | PA0/S1，内部下拉 | 配置成功后30 ms消抖，一键启动任务 |

M4、TIM10/TIM11、PB8/PB9、PD3/PD4和EXTI3已经整体删除，不再存在第四电机软编码器。

## IMU660RC接线与驱动

资料包中的IMU660RC使用`LSM6DSV16X`六轴芯片。随包“各单片机例程”目录本身是空的，资料说明明确指出网页下载ZIP不会带Git子模块；本工程因此按同芯片官方轮询示例的“检查ID、复位、配置量程与ODR、查询数据就绪、连续读取输出寄存器”流程移植，并保持当前F407 HAL和工程结构。

底板原理图和PCB工程确认以下引脚已引到H3/H4排针，并且不与三路电机、编码器、LCD、四路舵机、USART或SWD复用。没有使用看似空闲的SPI3：购买的F407开发板原理图显示`PC10/PC11/PC12/PD2`已经硬连板载MicroSD；SPI1的`PB3/PB4/PB5`也已硬连板载W25Q Flash。为避免隐蔽总线冲突，当前采用四个真正空闲GPIO模拟四线SPI。

| IMU660RC丝印 | F407网络 | 底板排针 | 说明 |
| --- | --- | --- | --- |
| `VCC` | `3V3` | H3-1（也可用H4-1） | 使用3.3 V供电，不接电机电源 |
| `GND` | `GND` | H3-2（也可用H4-2） | 必须与F407共地 |
| `SCL/SPC` | `PE2` | H4-22 | 软件SPI SCK |
| `SDA/SDI` | `PE4` | H4-21 | 软件SPI MOSI，F407发往IMU |
| `SA0/SDO` | `PE3` | H3-22 | 软件SPI MISO，IMU发往F407 |
| `CS` | `PC13` | H3-20 | 低电平选中，空闲保持高电平 |
| `INT1/INT2` | 不接 | 不接 | 当前采用10 ms轮询，未占用额外引脚 |

模块安装时应让Z轴垂直车体平面，最好让模块X轴与车头方向一致；否则加速度轴含义和Z轴角速度正负号需要做安装映射。上电后的约1.1秒校准期间必须让小车和模块完全静止，搬动或震动会把真实角速度误当成零偏。

驱动当前配置为：陀螺仪±500 dps、加速度计±4 g、两者120 Hz、BDU和寄存器自动递增开启、陀螺仪LPF1开启。公开接口保持精简：

```c
bool IMU_Init(void);
void IMU_Update(uint32_t now_ms);  /* 主循环收到10 ms释放请求后调用 */
IMUData IMU_GetData(void);         /* mg、mdps、mdeg和诊断计数 */
void IMU_ZeroYaw(void);            /* 开始一次相对转角动作前清零 */
```

`IMU_Update()`不在TIM6中断里执行SPI，只由中断递增释放序号，主循环消费最新请求；这样SPI通信或偶发错误不会拖慢10 ms电机速度环。初始化会读取`WHO_AM_I`、复位芯片并逐项回读关键配置寄存器；128个校准样本的各轴均值和极差还必须落在静止门限内，否则初始化失败，避免车辆上电时被搬动却写入错误零偏。全部检查通过后LCD才显示`IMU660RC: OK`一秒。运行中每250 ms复查一次芯片ID，或连续250 ms没有有效样本，都会锁存故障、令`IMUData.ready=false`并只增加一次`error_count`，只能复位重新初始化；`IMU_GetData()`和`IMU_ZeroYaw()`使用原子快照/临界区，避免64位偏航积分被并发读取撕裂。`yaw_mdeg`是校准后的Z轴角速度积分值，例如`90000`表示相对转过约90°，它仍会随时间缓慢漂移，不能替代磁力计或视觉绝对航向。

## 电机与PID

公开接口保持为：

```c
typedef struct {
    int16_t command;
    int16_t target;
    int16_t measured_speed_mm_s;
    int16_t target_speed_mm_s;
    bool direction_fault;
    bool stall_fault;
} MotorStatus;

void Motor_Init(void);
void Motor_SetSpeed(float target_speed, uint8_t id); /* mm/s，id=1..3 */
MotorDistanceStatus Go_distance(float distance_m);  /* m */
MotorTurnStatus Motor_TurnAngle(float angle_deg);   /* deg */
void Motor_Move(float forward_mm_s, float lateral_mm_s, float rotate_mm_s);
void Motor_Stop(void);
void Motor_Update(void);                            /* TIM6每10 ms调用 */
MotorStatus Motor_GetStatus(uint8_t id);
```

轮径70 mm、减速比34、13线编码器按四倍频计算为：

```text
34 * 13 * 4 = 1768 count/wheel-rev
110 mm/s = 8.84 count/10 ms ≈ 0.5 wheel-rev/s
```

1768是理论值，必须用轮胎标记手动准确转10圈，用总计数除以10重新校准。所有速度和距离结果都会按该参数成比例变化。

恒速控制仍按当前测试要求使用固定45%基础PWM：

```text
output = sign(target) * 450 + PID(target_count - measured_count)
```

PID修正限幅为±450，所以同方向输出范围为0%-90%。PID已经加入条件积分抗饱和：输出饱和且误差仍把输出推向饱和方向时停止累积积分；停车、换向和定距进入减速区时复位PID。`APP_MOTOR_MAX_COUNT_10MS=60`对应约0.746 m/s，未来要求1 m/s时必须提高上限并重新整定。

## 故障与停车

- 启动/换向后先等待200 ms；之后若编码器连续60 ms与目标反向，记录`DIR`。
- 启动后先等待500 ms；若上一周期PWM绝对值至少50%，但编码器连续500 ms仍为0，记录`STALL`，用于检测堵转和编码器断线。
- 任意一个轮子出现`DIR`或`STALL`，三轮立即一起停车；普通恒速测试、`Motor_Move()`、`Go_distance()`和`Motor_TurnAngle()`使用同一联停规则。
- 故障会锁存，`Motor_Stop()`不会清除，状态机下一周期也不能重新启动电机。检查接线和机械问题后复位MCU才能恢复。
- AT8236停车时内部先使用`IN1=IN2=1`低侧制动约60 ms，然后切换到`IN1=IN2=0`高阻滑行/休眠；重复调用`Motor_Stop()`不会无限延长制动，新运动命令会保存目标但必须等剩余制动周期结束后才真正输出。

电机、编码器和舵机的HAL启动失败会进入`Error_Handler()`，不会再静默继续运行。

## `Go_distance()`

返回状态：

```c
typedef enum {
    MOTOR_DISTANCE_IDLE = 0,
    MOTOR_DISTANCE_RUNNING,
    MOTOR_DISTANCE_DONE,
    MOTOR_DISTANCE_FAULT,
    MOTOR_DISTANCE_INVALID
} MotorDistanceStatus;
```

正数朝带舵机的一侧前进、负数后退，单位为米。根据实际方向测试，前进时M1反转、M2停止、M3正转。函数记录M1/M3的64位起点，用：

```text
forward_distance = (M3_distance - M1_distance) / sqrt(3)
```

估算底盘前进距离。正式前进轮速比例为`M1=-0.866、M2=0、M3=+0.866`，后退三者符号相反；横移比例为`M1=-0.5、M2=+1.0、M3=-0.5`，原地旋转仍为三轮同号。当前驶出出发区使用300 mm/s巡航；减速区取100 mm和全程一半中的较小值，进入减速区后清除巡航积分，并把速度目标从300 mm/s线性降到120 mm/s。PWM仍完全由固定45%基础值和实际编码器速度误差的PI修正得到，不再人为同步压低PWM上限，因此不会因为轮胎低于约50%无法克服静摩擦而在终点前提前停住。进入3 mm容差后主动制动。

完成、故障状态会锁存，循环调用不会再次启动：

```c
MotorDistanceStatus result = Go_distance(0.5f);
if (result == MOTOR_DISTANCE_DONE) {
    Motor_Stop(); /* 确认完成、回到IDLE，之后才能启动下一段 */
}
```

定距运行时若连续1秒没有获得至少0.25 mm的新进度，会作为`FAULT`三轮联停。`NaN`或无穷大参数返回`INVALID`，不会写入控制状态。

## `Motor_TurnAngle()`

```c
typedef enum {
    MOTOR_TURN_IDLE = 0,
    MOTOR_TURN_RUNNING,
    MOTOR_TURN_DONE,
    MOTOR_TURN_FAULT,
    MOTOR_TURN_INVALID
} MotorTurnStatus;

MotorTurnStatus result = Motor_TurnAngle(180.0f);
```

参数单位为度，正数和负数分别选择两个相反的原地旋转方向，允许范围为-360°到+360°。函数第一次调用时检查IMU、清零相对偏航角并启动三轮，此后由10 ms的`Motor_Update()`自动读取IMU并控制停止，调用方只需周期性重复调用以取得状态。剩余30°时由250 mm/s降至120 mm/s，进入1°提前量后制动；IMU故障、电机故障或运行超过10秒均返回`MOTOR_TURN_FAULT`并停车。

完成或故障状态会锁存，处理结果后调用`Motor_Stop()`回到空闲状态，才能开始下一次定角度动作。当前`IMU MONITOR`测试没有调用`Motor_TurnAngle()`，因此小车不会自动旋转。

## 编码器、并发与视觉

`EncoderStatus.position`已经改为`int64_t`，避免32位累计位置连续运行数天后溢出。16位定时器仍用`(int16_t)(current-previous)`正确处理回绕，前提是单个10 ms增量不超过32767。

电机公开设置函数和状态读取使用短临界区；编码器只保留初始化、10 ms采样和`Encoder_GetAll(status[3])`三个接口，一次临界区原子取得三轮位置及本周期增量。电机状态、编码器状态和视觉坐标都按快照读取，避免主循环界面与TIM6/串口中断互相读到半更新数据。舵机角度缓存使用`volatile`。

USART3只解析固定15字节、版本1的新协议：

```text
A3 B3 TYPE SEQ P0 P1 P2 P3 P4 P5 P6 P7 CRC_LO CRC_HI C3
```

- `TYPE=0x11`：赛前颜色和半区配置。
- `TYPE=0x12`：同一采样周期的坐标、距离、四类数量和状态。
- `TYPE=0x13`：只支持永久停车事件；实际送达由低头连续3帧空夹复核确认。
- CRC按`TYPE`到`P7`共10字节计算，低字节先发；错误帧不更新任何任务数据。
- 视觉报告超过250 ms未更新即视为过期，不会继续驱动底盘靠近。
- 配置确认仍是F407单独发送的4字节`A3 B3 01 C3`，不属于15字节上位机输入帧。

循环DMA在`Size=64`时消费本圈剩余数据并把软件位置归零，可持续解析无IDLE的连续数据。DMA首次启动或错误恢复失败时，主循环每100 ms重试，LCD显示`DMA ERR`，不会静默失效。

## 编译与安全测试顺序

每次源码修改后成功链接`WWW.elf`，CMake都会调用`tools/AutoBackup.ps1`：先创建标题带`yyyy-MM-dd HH:mm:ss`的本地提交，再根据改动路径附加`IMU`、`motor/PID`、`task/vision`、`scheduler/LCD`、`board/build config`或`documentation`等摘要，最后推送到`origin`。当前DAPLink OpenOCD配置开启了烧录前构建，因此点击烧录也会先完成同样的自动备份。没有文件变化时不会产生空提交；网络或GitHub认证失败时保留本地提交并在下次成功编译后重试，不阻止生成固件和烧录。疑似凭据文件（例如`.env`、`.pem`、`.key`）出现时自动跳过并警告。

```powershell
cmake --preset Debug
cmake --build --preset Debug
```

输出位于`build/Debug/WWW.elf/.hex/.bin`。当前固件上电后停车等待配置和S1。建议先断开电机12 V，用`tools/vision_protocol.py`验证三帧带CRC配置、三次4字节ACK、15字节LCD原始帧、危险/未知/超量拒绝、导航超时停车、目的地不匹配拒绝和永久停车；再悬空轮胎验证0.7 m驶出、分段靠近、抓取失败后退以及故障联停。最后在低速落地条件下分别验证返航方向、0.20 m入区、0度夹内画面和0.50 m退出距离。S4开闭角、方向、堵转门限、1768计数/圈、45%基础PWM和PID参数都必须结合实车数据继续整定。
