# 当前固件：普通物资抓取与安全区投送Task

当前只启用完整`Task`，所有测试模式和独立`CenteringTask`均关闭。固件已对齐`danmo-teng/shijue_fangan@c09aef2`任务协议：视觉坐标为原生1280×1024、中心`(640,512)`；接收`TYPE=0x11/0x12/0x16/0x18`，发送`TYPE=0x15/0x17`。当前临时关闭开局打散；先保持机构收纳直线倒车500 mm，离开障碍区后再并行转向和打开双爪，累计1.70 m后直接搜索，抓取观察角为140°。抓取命令持续重发直到F407回报夹爪闭合，返航使用上位机实时航向并保留分层失联停车。详细状态机和协议见[MISSION_PROTOCOL.md](MISSION_PROTOCOL.md)，本轮临时修改、调试命令和恢复方法见[临时底盘修改交接记录](docs/TEMP_CHASSIS_HANDOFF.md)。

> 从“历史设计”到CLion章节之间保留的是旧状态机设计记录，不再作为当前烧录行为或通信协议依据。

## 定位坐标系与算法

- 以3000×3000 mm场地中心为原点，向图纸右侧为`+X`，向图纸上方为`+Y`，航向角从`+X`逆时针增加。4号出发区中心约为`(1350,-1350) mm`，照片中的车头初始航向约为315°；倒车方向为135°，以300 mm/s行驶3秒的理想终点约为`(714,-714) mm`，实际LCD位置以编码器测量为准。
- `Location.c`每10 ms取得同一周期的三轮编码器增量。根据当前接线`M1=右轮=v3、M2=左轮=v2、M3=后轮=v1`，先求车体前向位移`F=(M1-M2)/sqrt(3)`和物理左向位移`L=(M1+M2-2*M3)/3`，再用本周期IMU中间航向旋转到场地坐标系并积分。三轮共同的原地旋转分量会在这两个式子中抵消，航向完全采用静止校准后的陀螺仪积分值。
- 实车验证表明，当前IMU安装方向下车体逆时针旋转时Z轴累计角度为负，因此定位对IMU角度增量乘`-1`后再更新场地航向。该符号只修正坐标系方向，不修改`Motor.c`已经确认的M1/M2/M3前后运动定义。
- `LocationPose`只公开`x_mm、y_mm、heading_mdeg、path_mm、start_zone、valid、inside_field`，LCD和后续任务读取原子快照，不直接接触内部浮点累加器。`Motor_TurnAngle()`改为记录动作起始航向并计算相对角度，不再调用`IMU_ZeroYaw()`破坏全局航向。
- LCD地图按赛事图绘制四个300×300 mm出发区、上下两个约660×360 mm安全区、中心轴线，以及四个出发区两条场内方向上的全部减速带；带白色边框的亮黄色实心圆表示车体中心，白色短线连接前方独立的红色圆点表示车头方向，绿色轨迹点保留最近48个实测位置。车头点根据IMU航向实时旋转，不是预先绘制的固定图标；IMU失效时车体变红、车头点变白。

地图只预设开始时的出发区坐标与车头方向，不保存或播放预先设计的运动路线。每个后续位置都来自该10 ms周期三路编码器的真实变化和IMU的真实角度变化：编码器没有新增计数时坐标不会因为“已经运行了多久”而自动改变；轮胎空转或打滑时，编码器航迹推算仍会产生虚假位移，这是轮式里程计的物理局限，后续必须由视觉绝对坐标进行校正。

该方案属于轮式航迹推算，不是绝对定位。全向轮滚子打滑、越减速带悬空、70 mm轮径误差、1768计数/圈误差和陀螺仪零偏都会随行驶距离累积；赛事文件还明确说明外围围栏不是定位基准。正式比赛应在视觉模块装好后，用已知安全区/出发区边界或场地图像周期性调用`Location_Reset()`或增加坐标校正，不能只凭该粗定位高速盲走。ROS 2官方全向轮控制器同样用轮位置/速度反馈计算里程计，并把轮半径列为决定速度与位移尺度的关键参数；三轮全向平台的系统误差需要通过多方向标定轨迹校正。[ROS 2 omni wheel controller](https://control.ros.org/rolling/doc/ros2_controllers/omni_wheel_drive_controller/doc/userdoc.html)、[三轮全向里程计系统误差研究](https://www.mdpi.com/2076-3417/12/5/2606)、[三轮全向运动学研究](https://doi.org/10.57417/jrnal.11.2_134)

Task模式上电时Location保持未配置状态，不再假定4号位；收到1帧合法赛前配置后，程序用`Location_Reset((LocationStart)start_zone)`按1～4号出发区建立对应初始坐标和航向。本地`Location`仍只由编码器和IMU推算；安全区导航和SEARCH边界预测优先使用上位机下发的T265融合位姿。融合位姿不可用时只有SEARCH边界判断和回正允许退回本地位姿；安全区NAV立即停车并等待融合位姿恢复，不使用本地位姿继续返航。

# 历史设计（已停用，仅供追溯）

当前固件实现赛前配置、一键驶出、搜索、分类靠近、抓取合规检查、上位机引导返航、进入安全区、放下目标、低头复核和后退重搜。上位机负责从图像判断路线和安全区位置，F407负责命令超时停车、目的地校验、单帧合法性确认、编码器定距进入/退出和机构动作；上位机不能直接控制PWM。

IMU660RC已经作为独立底层传感器移植：F407通过独立的5.25 MHz硬件SPI3读取LSM6DSV16X，传感器的陀螺仪和加速度计都工作在高精度1000 Hz模式，TIM6每1 ms发布一次主循环采样请求，并在完成静止零偏校准后积分Z轴偏航角。`Motor_TurnAngle()`封装IMU定角度旋转，`Motor_MoveAngle()`在任意方向平移期间保持启动航向，`Motor_MoveDistance()`则把同一航向闭环叠加到编码器定距动作；当前Task已使用这些接口。

协议设计采用固定长度、消息类型、递增序号和整帧CRC16。固定帧适合当前STM32的64字节循环DMA；CRC16用于拒绝误码帧；坐标、距离、类别数量和状态放在同一视觉报告中，避免旧协议把不同采样周期的三帧拼在一起。导航采用“连续命令+失联停车”，与[ROS 2控制器的过期速度命令自动停车](https://control.ros.org/master/doc/ros2_controllers/diff_drive_controller/doc/userdoc.html)和[Nav2传感器超时即停车](https://docs.nav2.org/configuration/packages/collision_monitor/configuring-collision-monitor-node.html)的失效安全思路一致。串口设计参考：[ST UART Receive-to-IDLE DMA](https://dev.st.com/stm32cube-docs/hal1-to-hal2-migration/1.0.0/en/docs/markup/drivers_documentation/hal_drivers/uart/hal_uart_exported_functions_io_operation.html)、[Modbus串口CRC规范](https://modbus.org/docs/Modbus_over_serial_line_V1_02.pdf)、[IETF RFC 1662 FCS](https://www.rfc-editor.org/info/rfc1662/)。比赛规则以[2027智能+工程创新赛道官方解析](https://gcxl.edu.cn/new/res/intelligence20260617.pdf)为准。

## 固定15字节协议

新PCB“串口1”使用115200 8N1，其MCU外设是USART3（PD8 TX、PD9 RX）。除原有4字节配置ACK外，双向业务消息使用固定15字节帧：

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

配置只需发送1帧，且必须通过载荷范围、填充字节和CRC校验。例如红方1号半区：

```text
A3 B3 11 00 11 01 00 00 00 00 00 00 F0 57 C3
```

配置接受后，F407回复1次原有4字节ACK：

```text
A3 B3 01 C3
```

首帧合法配置确认后会锁定颜色和半区并回复1次ACK，只有复位MCU才能重新配置；CRC错误或载荷非法的配置帧仍会被拒绝。

## 现有Task状态帧（`TYPE=0x17`，当前模式不发送）

完整救援Task使用`TYPE=0x17`发送任务状态：状态改变时立即发送，并在任务运行期间每200 ms发送一次。当前该Task关闭、`APP_ENABLE_CENTERING_TASK=1`，因此视觉居中模式不发送任务状态帧。`TYPE=0x16`已按上位机定位代码固定用于RDK向F407发送融合位姿，避免双向消息类型冲突。

| 载荷 | 内容 |
| --- | --- |
| `P0` | `TaskState` |
| `P1` | 目的地：0无、1物资区、2伤员区 |
| `P2 P3` | 剩余秒数，大端 |
| `P4` | bit0比赛已开始、bit1发现、bit2已抓、bit3货物有效、bit4已送普通物资、bit5导航新鲜、bit6已到安全区、bit7夹爪为空 |
| `P5` | 停车/故障原因，0表示无故障 |
| `P6` | 本场抓取超时恢复次数 |
| `P7` | 四类目标数量位域，与视觉报告`P6`相同 |

`P5`依次定义为：0无故障、1上位机急停、2比赛时间结束、3电机故障、4出发超时、5融合/本地位姿超时、6任务命令超时、7撞送定距故障、8非法任务状态、9接近阶段目标重捕获失败。上位机应校验CRC和`SEQ`后再更新界面，不应把状态帧当成运动命令。

## `TYPE=0x15`：F407编码器里程计

该帧独立于Task，每次10 ms编码器采样后发布，即100 Hz。三个计数取`EncoderStatus.position`累计原始计数的低16位；RDK按16位模减法处理回绕，不能把它当作单帧增量。F407只上传轮式数据，T265继续使用自身IMU。

| 载荷 | 内容 | 字节序 |
| --- | --- | --- |
| `P0 P1` | M1累计原始计数低16位 | 大端 |
| `P2 P3` | M2累计原始计数低16位 | 大端 |
| `P4 P5` | M3累计原始计数低16位 | 大端 |
| `P6` | 采样周期`DT=10 ms` | 单字节 |
| `P7` | `STATUS`：bit0～2三路有效，bit3为计数刚复位，bit4为编码器故障 | 位标志 |

上位机用`轮周长 / 1768`把计数转换成每只轮子的线位移，再按当前三轮运动学反解车体平移速度并转成m/s，随后通过librealsense的`send_wheel_odometry()`送入T265。`tools/vision_protocol.py`中的`parse_odometry()`可直接解析该帧。T265的轮式里程计接口还必须加载与实际安装位置和坐标轴一致的标定JSON，不能只发送速度而省略外参。

## `TYPE=0x16`：RDK融合位姿

上位机定位程序以20 Hz向F407发送融合位姿。F407会完成帧、CRC、航向范围和保留位校验，重复`SEQ`不刷新时间戳；建议使用150 ms新鲜度超时。当前视觉居中Task只使用`TYPE=0x12`，因此该位姿仅保存到`VisionData.fused_pose`，不会改变底盘控制或本地`Location`。

| 载荷 | 内容 | 字节序 |
| --- | --- | --- |
| `P0 P1` | 场地X坐标，int16，mm | 大端 |
| `P2 P3` | 场地Y坐标，int16，mm | 大端 |
| `P4 P5` | 航向角，uint16，0～35999，单位0.01° | 大端 |
| `P6` | 位姿状态位 | 位标志 |
| `P7` | 置信度和位置标准差 | 位打包 |

`P6`的bit0～6依次为`VALID、T265_GOOD、WHEEL_ACTIVE、OBSTACLE_GATE、ODOM_FRESH、INSIDE_FIELD、T265_UPDATE_REJECTED`，bit7必须为0。`P7`中bit0～1为tracker confidence，bit2～3为mapper confidence，bit4～7为位置标准差cm（15表示不小于15 cm）。

## `TYPE=0x12`：原子视觉报告

上位机每30～50 ms只发送一帧视觉报告：

| 载荷 | 内容 | 字节序 |
| --- | --- | --- |
| `P0 P1` | 串口坐标X，0～639；中心为320 | 大端 |
| `P2 P3` | 串口坐标Y，0～479；中心为240 | 大端 |
| `P4 P5` | 目标距离mm；无有效距离时必须为0 | 大端 |
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
| bit6 | `DISTANCE_VALID` | P4/P5为有效距离；置位时距离必须为1～65535 mm |
| bit7 | 保留 | 必须为0 |

`GRABBED=0`时，P6只统计当前选中目标的抓取ROI，不得统计整张画面的所有目标；`GRABBED=1`时，P6必须统计机构内全部目标。没有目标时整个P0～P7必须为0。F407会拒绝坐标越界、bit7非0、距离有效位与距离数值矛盾以及无目标但载荷非0的报告；250 ms内没收到新的有效目标报告就停车，不会沿用旧坐标。当前上位机普通物资程序没有测距时发送`FOUND=1、CLASS_VALID=1、DISTANCE_VALID=0、distance=0`，视觉居中Task可以直接使用X/Y；完整救援Task只有在`DISTANCE_VALID=1`时才允许进入依赖距离的靠近流程。

上位机当前发送的典型帧：目标位于串口坐标中心`(320,240)`、暂无有效距离、识别为1个普通物资：

```text
A3 B3 12 10 01 40 00 F0 00 00 01 09 1C 13 C3
```

如果后续增加测距，350 mm时需同时置`DISTANCE_VALID`：

```text
A3 B3 12 10 01 40 00 F0 01 5E 01 49 7D CD C3
```

## `TYPE=0x13`：安全事件

### 永久停车

```text
P0=01，P1..P7=00
A3 B3 13 30 01 00 00 00 00 00 00 00 14 50 C3
```

收到后进入`STOPPED`，只能复位MCU恢复。

### 请求自救

```text
P0=02，P1..P7=00
```

该事件只在`RETURN_SAFE`有效。F407把新`SEQ`作为一次独立请求，500 ms内收到后执行850 mm/s车体后退1秒；相同`SEQ`不会重复触发，每次返航最多接受2次。850 mm/s低于当前编码器60 count/10 ms对应的纯前进物理上限，避免三轮同比缩放后自救距离与配置不一致。何时卡住以及是否需要再次自救完全由上位机判断，F407不再根据本地编码器和Location自动触发。是否成功放下由F407收到1帧合法的夹内为空报告后确认。

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

上位机在`RETURN_SAFE`中每30～50 ms发送一次导航帧，并持续递增`SEQ`。最近200 ms内的帧会把状态标记为`NAV_FRESH`并允许执行对应动作；超过200 ms未更新就强制停车，收到新的合法匹配帧后才恢复。进入返航后尚未收到匹配命令、目的地不匹配或自救刚结束尚未收到新命令时同样停车。收到1帧合法的`P0=00、P1=02`到区报告后进入放置流程。

导航帧只给方向，不给PWM。F407当前把前进、后退、旋转分别映射为776、448和358 mm/s，再经过`Motor_Move()`三轮运动学和10 ms轮速PID；`Motor_Move(forward_mm_s, lateral_mm_s, rotate_mm_s)`三个参数全部统一为mm/s，超过轮速上限时三轮同比缩放。左右方向必须先悬空轮胎低速核对，方向相反时修改映射，不能让上位机把“左”和“右”互换来掩盖底层配置错误。

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

1. 合法货物确认后进入`RETURN_SAFE`，夹爪保持抓取姿态、摄像头回到90度广角。
2. 上位机依据本车颜色、当前位置、货物类别和安全区图像持续输出`HOLD/FORWARD/TURN_LEFT/TURN_RIGHT/BACKWARD`。F407只映射为限速底盘动作，上位机不发送占空比。
3. 导航短时中断但最后一帧仍未超过200 ms时保持最后一条合法动作；超过200 ms后F407强制停车。上位机在丢失定位、图像不确定或准备停止发送前仍应主动发送`HOLD`。
4. 收到1帧合法的安全区附近报告后进入`DROP_OBJECT`，F407不再接受导航动作，使用编码器向前0.20 m，使目标越过区域边界。
5. 停车后舵机1转到65°，夹爪按“左爪舵机4先到128°、右爪舵机2再到52°”的Open顺序释放目标并等待600 ms；摄像头转到0度低头角，等待400 ms让舵机和图像稳定。
6. 上位机只检测夹爪/导向机构ROI，并发送带`CLAW_VIEW`的视觉报告：夹内空为`FOUND=0、P6=0、CLASS_VALID=1`；仍有目标则`FOUND=1、P6=实际剩余数量、CLASS_VALID=1`。模糊或遮挡但能确定ROI内有物体时置`FOUND=1、UNKNOWN=1`。进入复核后3秒内未收到1帧合法结论，F407会把摄像头转回90°并重新执行打开、低头和复核；最多重试2次，仍失败则进入`STOPPED`，不会猜测放置成功或永久等待。
7. 收到1帧合法的夹内为空报告后计为送达；摄像头回90度，夹爪回到右100°/左80°的Touch姿态后编码器后退0.50 m，回到`FIND_OBJECT`继续旋转搜索。
8. 若收到1帧合法的夹内仍有货物报告，则重新Touch夹紧目标，后退0.50 m回到`RETURN_SAFE`。剩余货物不合规或电机故障时进入`STOPPED`。

当前实车参数为`进入0.20 m、退出0.50 m、夹内检查0°`：0.20 m要保证目标越过边界但车体不压线，0.50 m要保证旋转时机构不扫到已放下目标，0°要保证夹内ROI完整进入画面。摄像头软件最小限位已同步设为0°，因此检查命令不会再被夹到10°。

当前F407没有激光雷达、碰撞开关或本地地图，无法独立判断返航路径上突然出现的障碍；200 ms以内的短时断帧会沿用最后一条合法方向，超过200 ms强制停车。上位机识别不确定、目标被遮挡或路线被占用时必须先发送`HOLD`。电机堵转/反向故障、30秒返航超时和180秒总超时仍是底层最后保护，但不能替代上位机避障。

## 抓取顺序与合规判断

初赛目标为普通物资、核心物资、伤员和危险目标。所有救援目标是双方公共目标，红蓝只表示安全区，不存在“对方颜色目标”；因此协议不再设置`A2/B2`颜色判断，也不允许`E3=任意目标都可抓`。

F407执行以下硬规则：

1. 第一次成功送达普通物资前，只选择普通物资；抓到1～3个普通物资可进入`RETURN_SAFE`。
2. 普通物资送达后，普通与核心物资合计1～3个可送物资区。规则没有禁止二者混运，当前代码允许混运。
3. 伤员必须单独转运，数量必须正好为1，目的地为伤员区。
4. 危险数量非0、存在未知目标、总数为0、总数超过3或伤员与其他目标混装，都判为非法货物。
5. 搜索阶段必须先连续3个`SEQ`确认同一个合法候选才进入靠近；靠近中遇到异常帧立即停车，250 ms内恢复合法目标则继续，否则Touch后退重搜。抓取分类也必须连续3个`SEQ`逐帧加1的报告保持相同，才做最终决定。重复、乱序或丢帧会从当前帧重新计数；非法货物会保持Touch、后退250 ms并重新搜索。
6. 第一趟建议抓最近的1个普通物资立即送回以尽快解锁，不必等待装满3个；之后再按距离和剩余时间选择核心、伤员或普通物资。

## 小车状态

TIM6每20 ms发布一次`Task_Process(now_ms)`运行请求，由最低优先级PendSV非阻塞执行：

1. `WAIT_CONFIG`：停车等待1帧合法配置并回复1次ACK。
2. `START`：上电后仍依次将左爪舵机4转到23°、右爪舵机2转到147°形成安全Retract姿态；收到1帧合法配置后自动启动180秒倒计时。前500 mm保持舵机1为55°、夹爪收纳，以850 mm/s进行IMU航向保持的直线倒退；离开障碍区后舵机1到85°、左右爪同时打开，并在剩余1.20 m内保持地面直线、闭环转约180°。仅最后100 mm降到160 mm/s，进入10 mm容差后制动并直接开始搜索。
3. `DISPERSE`：原有“前进0.20 m、正反各转一圈、后退0.30 m”代码仍保留，但当前`APP_ENABLE_START_SCATTER=0`，正常流程不会进入这些状态。
4. `FIND_OBJECT`：摄像头保持120°。进入每一轮搜索时记录最后一个合法视觉报告的本地接收代次，只有收到代次更新的新报告后才允许X/Y触发目标锁定；开局倒车、入堆和撞散期间收到的坐标不会参与运动控制。等待700 ms后以160 mm/s原地搜索；累计实际航向转满360°仍未找到目标时，先预测前进0.80 m后的落点。落点距离场地边界不足200 mm就先转向场地中心，防止继续向场外推进。只接近当前阶段允许且分类明确的单个目标。
5. `GRAB_OBJECT`：Task起始搜索角为120°；收到1帧合法目标报告后按X修正底盘并由舵机3根据目标Y坐标执行视觉PID。水平和摄像头PID都只在新视觉`SEQ`到达时更新，并按实际帧间隔计算I/D。舵机3低于140°时丢失目标，先停车500 ms，再沿目标最后出现的方向进入`REACQ`；每转一圈反向并抬高摄像头10°，恢复目标后继续APPROACH，不返回SEARCH，最长25秒后故障停车。达到140°后进入抓取观察；收到`GRAB_CONFIRMED`时左右爪同时合到Touch姿态，保留2秒完成窗口再上报闭合。
6. `RETURN_SAFE`：夹紧目标后摄像头保持抓取观察角，等待上位机`TYPE=0x18`导航命令和`TYPE=0x16`融合位姿。F407先对准安全区前置点，再以800 mm/s直行，距离500 mm内降到250 mm/s；行驶期间持续更新目标方位，偏差达到7°时停车重新对正。NAV命令短暂中断时分级降速，超过1秒停车等待恢复；融合位姿失效也停车等待恢复。
7. `DELIVER`：完成安全区入口对正后，持续收到`ENTER_SAFE_ZONE`才允许左右爪同时打开、以250 mm/s后退0.30 m；制动等待200 ms后再软启动前冲0.55 m，最高850 mm/s。打开、后退、换向等待和前冲期间每20 ms检查命令新鲜度；失联立即停车。未送入就重复；收到`TASK_COMPLETE`确认送入后摄像头才抬到120°，随后后退0.45 m、转向场地中心并重新搜索。
8. `STOPPED`：180秒结束、上位机`STOP/ABORT`、电机/位姿/任务命令故障或目标重捕获失败后保持停车。

当前夹爪角度为：收缩左23°/右147°，Touch左80°/右100°，Open左128°/右52°。安装机构后必须先断开机构负载标定角度，确认不会顶死舵机。

Task模式LCD不绘制场地图或目标距离，只保留当前任务状态、上位机视觉报告中的X/Y坐标、颜色/出发区/串口状态和摄像头角度。超过250 ms没有新的有效目标时X/Y显示`----`；串口超过250 ms未收到任何新帧时显示`TMO`。动态值每100 ms刷新，固定宽度文字直接覆盖旧内容，不先清空整行，以减少闪烁。

底层仍为：编码器和速度PID严格每10 ms在TIM6中断执行；IMU每1 ms释放一次主循环采样请求，主循环延迟时合并为最新一次而不重复读取已经过去的样本；任务由TIM6严格每20 ms发布、最低优先级PendSV消费，延迟时只执行最新一次；LCD每100 ms刷新；USART3使用64字节循环DMA，不申请动态内存。PendSV可被TIM6、DMA和USART3抢占，高层状态机不会再占用电机实时中断。

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

1. M1-M3六路PWM以0占空比启动，三路硬件编码器开始计数，TIM8四路舵机PWM启动；Task先将左右爪收缩，收到配置前保持停车。
2. IMU660RC先检查`WHO_AM_I=0x70`，随后在车辆静止时采集128个陀螺仪样本校准零偏；1000 Hz配置下采样本身约需0.13秒，连同复位和配置仍应保持静止直到LCD给出结果。
3. LCD初始化完成后显示`IMU660RC: OK`一秒；连接失败时显示`IMU660RC: ERROR`一秒。提示结束后清屏并切换到当前任务状态界面，不执行RGB色块测试。
4. TIM6提供1 ms基础节拍：每1 ms发布一次IMU主循环采样请求，每10 ms采样编码器并执行一次电机速度环；自主任务开启时每20 ms向最低优先级PendSV发布一次任务请求；每100 ms只发布一次LCD刷新请求。
5. 新PCB“串口1”USART3使用PD8/PD9和64字节循环DMA接收视觉帧；通过统一的中断发送队列发送4字节配置ACK及100 Hz三路编码器累计计数帧。
6. 当前`APP_ENABLE_TASK=1`，独立视觉居中Task及所有运动测试、定位演示和舵机扫描均为0；收到配置后自动执行退出安全区、开爪直接搜索、视觉靠近、140°抓取观察、融合位姿返航和安全区投送。原打散状态保留但由`APP_ENABLE_START_SCATTER=0`暂时绕过。

接通电机12 V前应确认三轮与编码器方向正确，并在车后方预留至少1.8 m空间；上位机一旦发出1帧有效配置，F407就会自动开始180秒倒计时，保持IMU航向并按编码器倒车1.70 m，没有额外按键确认。

## 自主救援流程

完整流程只公开一个非阻塞入口`Task_Process(now_ms)`。TIM6每20 ms发布节拍，最低优先级PendSV执行高层状态机，电机10 ms闭环仍留在TIM6中断。当前状态覆盖配置、出发、开爪、搜索、靠近、抓取观察/抬高/旋转恢复、合爪、等待导航、前置点导航、安全区对正、慢速入区、投送和完成；准确流程及协议以[MISSION_PROTOCOL.md](MISSION_PROTOCOL.md)为准。

## 引脚与外设

| 功能 | 引脚/外设 | 参数 |
| --- | --- | --- |
| M1 | PA2/PA3 TIM5 CH3/CH4；PE9/PE11 TIM1编码器 | 20 kHz，车头右轮 |
| M2 | PE5/PE6 TIM9 CH1/CH2；PA6/PA7 TIM3编码器 | 20 kHz，车头左轮 |
| M3 | PB10/PB11 TIM2 CH3/CH4；PD12/PD13 TIM4编码器 | 20 kHz，车尾后轮 |
| 舵机S1-S4 | PC6-PC9，TIM8 CH1-CH4 | 50 Hz，500-2500 us |
| LCD | PB13 SCK、PB15 MOSI、PB12 CS、PB14 RESET、PC5 DC、PB1 BL | ST7735，128x160 |
| IMU660RC | PC10 SCK、PC11 MISO、PC12 MOSI、PC13 CS | SPI3硬件全双工，Mode 3，1000 Hz采样 |
| PCB串口1 / USART3 | PD8 TX、PD9 RX | 115200 8N1；RX为DMA1 Stream1的64字节循环DMA，TX发送4字节配置ACK和100 Hz累计编码器里程计 |
| PCB串口2 / USART1 | PA9 TX、PA10 RX | 115200 8N1文本舵机调试；`DEBUG`进入、`S id angle`调角、`RUN`复位回正常模式，不用于RDK通信 |
| 预留按键 | PA0/S1，内部下拉 | 当前Task不读取该按键 |

M4、TIM10/TIM11、PB8/PB9、PD3/PD4和EXTI3已经整体删除，不再存在第四电机软编码器。

### 运行时舵机调试

USART1发送`DEBUG`并回车后立即停车并暂停Task；随后可发送如`S 3 140`的命令手动调整1～4号舵机。发送`RUN`会停车并复位MCU，回到正常模式后需要重新启动上位机任务或重新发送`TYPE=0x11`配置。调试模式不占用USART3，不恢复按键启动。完整操作和安全注意事项见[临时底盘修改交接记录](docs/TEMP_CHASSIS_HANDOFF.md#6-舵机调试模式)。

## IMU660RC接线与驱动

资料包中的IMU660RC使用`LSM6DSV16X`六轴芯片。随包“各单片机例程”目录本身是空的，资料说明明确指出网页下载ZIP不会带Git子模块；本工程因此按同芯片官方轮询示例的“检查ID、复位、配置量程与ODR、查询数据就绪、连续读取输出寄存器”流程移植，并保持当前F407 HAL和工程结构。

当前明确放弃板载MicroSD功能，将卡座原有的`PC10/PC11/PC12`网络改作IMU专用SPI3。这块板的卡座原本采用SDIO连接，`PD2`是SDIO_CMD而不是普通SPI片选；代码将`PD2`配置为上拉推挽输出并始终保持高电平，同时给SPI3引脚配置弱上拉，避免未使用卡座产生悬空干扰。由于卡座仍然电气连接在这些网络上，测试和比赛时必须保持卡槽为空，不能一边插卡一边运行IMU。IMU使用独立的`PC13`片选。SPI3固定为Mode 3、5.25 MHz，满足LSM6DSV16X最高10 MHz的限制。LCD恢复为原来的SPI2 Mode 0、10.5 MHz，不再与IMU共享总线，也不再进行运行时SPI模式切换。SPI1的`PB3/PB4/PB5`继续留给板载W25Q Flash，没有受到本次修改影响。

| IMU660RC丝印 | F407网络 | 底板排针 | 说明 |
| --- | --- | --- | --- |
| `VCC` | `3V3` | H3-1（也可用H4-1） | 使用3.3 V供电，不接电机电源 |
| `GND` | `GND` | H3-2（也可用H4-2） | 必须与F407共地 |
| `SCL/SPC` | `PC10` | H1-13 | SPI3 SCK，板载卡座也连接此网络 |
| `SDA/SDI` | `PC12` | H1-14 | SPI3 MOSI，F407发往IMU |
| `SA0/SDO` | `PC11` | H2-14 | SPI3 MISO，IMU发往F407 |
| `CS` | `PC13` | H3-20 | 低电平选中，空闲保持高电平 |
| `INT1/INT2` | 不接 | 不接 | 当前采用1 ms轮询，未占用额外引脚 |

模块安装时应让Z轴垂直车体平面，最好让模块X轴与车头方向一致；否则加速度轴含义和Z轴角速度正负号需要做安装映射。上电后的约1.1秒校准期间必须让小车和模块完全静止，搬动或震动会把真实角速度误当成零偏。

原来的`PE2/PE3/PE4`软件SPI接线以及随后使用的`PB13/PC2/PC3` SPI2接线都已停用，必须按上表重新接线。`PD2`是板载MicroSD的SDIO_CMD（H2-16），不要把它接到IMU。驱动当前配置为：陀螺仪±500 dps、加速度计±4 g、两者高精度1000 Hz、BDU和寄存器自动递增开启、陀螺仪LPF1开启。公开接口保持精简：

```c
bool IMU_Init(void);
void IMU_Update(uint32_t now_ms);  /* 主循环收到1 ms释放请求后调用 */
IMUData IMU_GetData(void);         /* mg、mdps、mdeg和诊断计数 */
void IMU_ZeroYaw(void);            /* 仅供人工重新定义航向；定位运行时不要调用 */
```

`IMU_Update()`不在TIM6中断里执行SPI，只由中断递增1 ms释放序号，主循环消费最新请求；这样硬件SPI通信或偶发错误不会拖慢10 ms电机速度环。传感器内部ODR是1000 Hz，但该无中断、无FIFO的主循环轮询方案不是硬实时采集器：LCD绘制等操作占用主循环时会合并过期请求，恢复后读取最新样本，不会伪造补采样；可通过`sample_count`实测有效采样率。初始化会读取`WHO_AM_I`、复位芯片，在加速度计和陀螺仪均处于掉电状态时选择1000 Hz高精度ODR，再逐项回读关键配置寄存器；128个校准样本的各轴均值和极差还必须落在静止门限内，否则初始化失败，避免车辆上电时被搬动却写入错误零偏。全部检查通过后LCD才显示`IMU660RC: OK`一秒。运行中每250 ms复查一次芯片ID，或连续250 ms没有有效样本，都会锁存故障、令`IMUData.ready=false`并只增加一次`error_count`，只能复位重新初始化；`IMU_GetData()`和`IMU_ZeroYaw()`使用原子快照/临界区，避免64位偏航积分被并发读取撕裂。`yaw_mdeg`是校准后的Z轴角速度积分值，例如`90000`表示相对转过约90°，它仍会随时间缓慢漂移，不能替代磁力计或视觉绝对航向。

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
MotorDistanceStatus Motor_MoveDistance(float distance_m, float max_speed_mm_s);
MotorTurnStatus Motor_TurnAngle(float angle_deg);   /* deg */
void Motor_Move(float forward_mm_s, float lateral_mm_s,
                float yaw_tangent_mm_s);            /* 三项均为mm/s */
bool Motor_MoveAngle(float speed_mm_s, float angle_deg); /* 达到目标矢量返回true */
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

## 三轮全向运动学

本节按[轮趣科技R680/ROS资料包](https://pan.baidu.com/s/186VvHGOcfHoDA3TKxAP9tw)中“ROS机器人通用资料/运动底盘的控制与运动学解析/全向轮底盘”整理。R680车型本身采用阿克曼转向，不能直接套到当前三轮车；本工程只采用资料包里的120°三轮全向轮模型和逐轮速度闭环思路。

以车体坐标`Vx`为朝舵机方向的前进速度、`Vy`为运动学横向速度、`Rω`为旋转在轮周方向产生的切向速度，标准逆运动学为：

```text
v1 =                         Vy + Rω
v2 = -sqrt(3)/2 * Vx - 1/2 * Vy + Rω
v3 =  sqrt(3)/2 * Vx - 1/2 * Vy + Rω
```

结合当前实车接线，物理对应关系是`M1=右轮=v3、M2=左轮=v2、M3=后轮=v1`。因此车体前进时为`M1=+0.866、M2=-0.866、M3=0`。`Motor_Move()`第三项不是度/秒，而是公式中的`Rω`，统一使用mm/s；这样不依赖尚未精确标定的轮心到车体中心距离。三轮目标有任意一个超过编码器配置的轮速上限时，三个目标同时乘同一个比例，保留运动方向和旋转/平移比例，不逐轮单独截断。

### 平滑转向方案来源与取舍

- 网盘中的轮趣STM32讲义先在固定周期内平滑`VX/VY/VZ`，再调用底盘逆运动学和四个独立速度PI；松开指令时也不是直接把运动量清零。当前工程沿用“先平滑车体速度、再进行轮速分解”的层次，但使用二维同比矢量斜坡，避免分别修改X/Y导致斜向角度暂时失真。[轮趣R680/ROS资料包](https://pan.baidu.com/s/186VvHGOcfHoDA3TKxAP9tw)
- ROS官方Nav2速度平滑器同样使用固定周期插值、独立加减速度限制、速度死区和同比缩放，并说明高频低延迟里程计才适合闭环平滑。本工程10 ms速度环使用上一平滑指令推进，只有反向零速确认读取同周期编码器，避免编码器低速量化噪声参与每一步斜坡。[Nav2 Velocity Smoother](https://github.com/ros-navigation/navigation2/tree/main/nav2_velocity_smoother)
- ROS 2全向轮控制器以车体`x/y/yaw`速度为统一输入、用轮速/位置反馈计算底盘状态；移动控制器还提供速度、加速度、减速度和jerk限制。当前F407保留三轮逐轮PI、航向PI和同比轮速限幅；暂不增加jerk状态，因为固定45%起步PWM和当前低速死区会让第三阶轨迹参数难以独立标定，先把可测的加减速度与停稳阈值调准更可靠。[ROS 2 omni wheel controller](https://control.ros.org/kilted/doc/ros2_controllers/omni_wheel_drive_controller/doc/userdoc.html)、[ROS 2移动底盘运动限制](https://control.ros.org/rolling/doc/ros2_controllers/mecanum_drive_controller/doc/userdoc.html)
- 轮趣R550全向轮版本标称最高速度0.84 m/s，当前任务的最高直线请求为850 mm/s，已经处于同类教育底盘的极限速度区；高速动作必须保留减速和制动过程，并在实车上检查供电压降、轮胎打滑与电机温升。[轮趣R550产品手册](https://wheeltec.net/R550.pdf)

`Motor_MoveAngle(speed, angle)`规定`0°=前、90°=左、180°=后、270°=右`，将速度分解成`Vx/Vy`后进入同一逆运动学。第一次启动记录IMU累计航向，之后10 ms一次用航向PI产生`Rω`修正。启动和小角度换向采用二维矢量限速，当前加速度为2500 mm/s²；新旧方向夹角小于120°时直接在速度空间连续过渡，达到120°或更大时以3000 mm/s²先减到0。命令归零后要求三轮实测速度连续3个周期不超过40 mm/s，最多等待400 ms，随后清除三轮速度PI及航向PI积分、保留原目标航向，再向新方向加速。函数在平滑矢量达到目标后返回`true`；当前测试从此时才开始计算3秒倒车时间，单次加速超过2秒则安全停车。只有`Motor_Stop()`、IMU故障或切换到其他运动模式才结束本次航向保持。

## 故障与停车

- 启动/换向后先等待200 ms；之后若编码器连续60 ms与目标反向，记录`DIR`。
- 启动后先等待500 ms；若上一周期PWM绝对值至少50%，但编码器连续500 ms仍为0，记录`STALL`，用于检测堵转和编码器断线。
- 任意一个轮子出现`DIR`或`STALL`，三轮立即一起停车；普通恒速测试、`Motor_Move()`、`Motor_MoveDistance()`和`Motor_TurnAngle()`使用同一联停规则。
- 故障会锁存，`Motor_Stop()`不会清除，状态机下一周期也不能重新启动电机。检查接线和机械问题后复位MCU才能恢复。
- AT8236停车时内部先使用`IN1=IN2=1`低侧制动约60 ms，然后切换到`IN1=IN2=0`高阻滑行/休眠；重复调用`Motor_Stop()`不会无限延长制动，新运动命令会保存目标但必须等剩余制动周期结束后才真正输出。

电机、编码器和舵机的HAL启动失败会进入`Error_Handler()`，不会再静默继续运行。

## `Motor_MoveDistance()`

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

正数表示车体前进、负数表示后退，单位为米。按照当前接口接线，前进时M1正转、M2反转、M3停止。函数记录M1/M2的64位起点，用：

```text
forward_distance = (M1_distance - M2_distance) / sqrt(3)
```

估算底盘前进距离。正式前进轮速比例为`M1=+0.866、M2=-0.866、M3=0`，后退时M1/M2符号相反；横移比例为`M1=-0.5、M2=-0.5、M3=+1.0`，原地旋转仍为三轮同号。Task开局退出安全区使用本地`Location.path_mm`累计三轮编码器解算出的平移路程，并由`Motor_MoveAngle()`锁住IMU航向，在1.70 m终点前分级减速。每次`Motor_MoveDistance()`启动时先确认三轮速度连续3个10 ms周期不超过40 mm/s，最多等待400 ms，再锁存编码器起点和IMU航向；运行中10 ms一次叠加航向PI修正，并以3,000 mm/s²目标速度软启动、3,500 mm/s²减速。编码器前向解算仍独立决定剩余距离和3 mm容差制动，因此上一动作的惯性和旋转修正不会被累计成前进距离。

完成、故障状态会锁存，循环调用不会再次启动：

```c
MotorDistanceStatus result = Motor_MoveDistance(0.5f, 300.0f);
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

参数单位为度，正数和负数分别选择两个相反的原地旋转方向，允许范围为-360°到+360°。函数第一次调用时检查IMU、清零相对偏航角并启动三轮，此后由10 ms的`Motor_Update()`自动读取IMU并控制停止，调用方只需周期性重复调用以取得状态。剩余30°时由500 mm/s降至120 mm/s，进入1°提前量后制动；IMU故障、电机故障或运行超过10秒均返回`MOTOR_TURN_FAULT`并停车。

完成或故障状态会锁存，处理结果后调用`Motor_Stop()`回到空闲状态，才能开始下一次定角度动作。函数保存动作开始时的累计IMU航向并计算相对转角，不再清零全局航向，因此能和`Location.c`连续工作。

## 编码器、并发与视觉

`EncoderStatus.position`已经改为`int64_t`，避免32位累计位置连续运行数天后溢出。16位定时器仍用`(int16_t)(current-previous)`正确处理回绕，前提是单个10 ms增量不超过32767。

电机公开设置函数和状态读取使用短临界区；编码器只保留初始化、10 ms采样和`Encoder_GetAll(status[3])`三个接口，一次临界区原子取得三轮位置及本周期增量。电机状态、编码器状态和视觉坐标都按快照读取，避免主循环界面与TIM6/串口中断互相读到半更新数据。舵机角度缓存使用`volatile`。

新PCB“串口1”（MCU USART3）解析固定15字节、版本1的新协议：

```text
A3 B3 TYPE SEQ P0 P1 P2 P3 P4 P5 P6 P7 CRC_LO CRC_HI C3
```

- `TYPE=0x11`：赛前颜色和半区配置。
- `TYPE=0x12`：同一采样周期的坐标、距离、四类数量和状态。
- `TYPE=0x13`：`P0=01`永久停车，`P0=02`请求一次上位机判定的高速倒车自救。
- CRC按`TYPE`到`P7`共10字节计算，低字节先发；错误帧不更新任何任务数据。
- 视觉报告超过250 ms未更新即视为过期，不会继续驱动底盘靠近。
- 配置ACK保留原有4字节`A3 B3 01 C3`；`TYPE=0x15`是F407以100 Hz发出的三轮编码器累计计数低16位、10 ms采样间隔和状态位。
- `TYPE=0x16`是RDK以20 Hz发回的融合位姿，F407已接收并保存，但当前视觉居中Task不使用。
- `TYPE=0x17`是关闭中的完整救援Task状态上报，当前视觉居中模式不发送。

循环DMA在`Size=64`时消费本圈剩余数据并把软件位置归零，可持续解析无IDLE的连续数据。DMA首次启动或错误恢复失败时，主循环每100 ms重试，LCD显示`DMA ERR`，不会静默失效。

串口底层集中在`Uart.c`，对外只保留`Uart_Init()`、`Uart_Send()`和`Uart_Receive()`三个函数。DMA缓冲、回绕解析、统一静态发送队列和HAL错误恢复均为文件内部实现；ACK优先，队列满时淘汰最旧的待发里程计帧，当前固定服务新PCB“串口1”（USART3 PD8/PD9）。

## 编译与安全测试顺序

每次默认CMake构建都会在固件目标完成后运行独立的`github_backup`目标并调用`tools/AutoBackup.ps1`，即使`WWW.elf`已经是最新、无需重新链接也不会跳过：有改动时先创建标题带`yyyy-MM-dd HH:mm:ss`的本地提交，根据路径附加模块摘要，随后推送到`origin`并核对上游提交与本地`HEAD`完全一致。当前DAPLink OpenOCD配置开启了烧录前默认构建，因此点击烧录也会先完成这项强制GitHub同步；网络不可用、认证失败、远端拒绝、存在未处理分歧或疑似凭据文件（例如`.env`、`.pem`、`.key`）时构建会以错误结束并阻止烧录。没有文件变化时不会产生空提交，但仍会连接GitHub执行同步检查。Git无法绕过本地commit直接写远端，因此“只提交GitHub”的实际保证是：本地commit未成功同步到GitHub时，本次构建和烧录不会被视为成功。

```powershell
cmake --preset Debug
cmake --build --preset Debug
```

输出位于`build/Debug/WWW.elf/.hex/.bin`。当前固件只运行完整救援Task，独立视觉居中Task和所有独立测试模式关闭；新PCB“串口1”（USART3）继续以100 Hz发送`TYPE=0x15`三路累计编码器计数，并以20 Hz发送`TYPE=0x17`任务状态。烧录后先架空车轮验证启动定距、重复GRAB/闭合回报握手和返航断流保护，再落地测试；正式Task必须先收到合法`TYPE=0x11`配置，不需要按键确认。可用`tools/vision_protocol.py`验证协议帧。
