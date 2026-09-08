# F407 运动调试交接：RDK X5 ↔ USART3

本文是 F407 运动调试模式给上位机/视觉负责人的交接说明。该模式只服务于
T265 安装参数、三轮编码器和底盘方向的联调，不启用完整救援流程。

## 1. 当前固件和安全约定

仓库默认为原仓库 68a0802 的完整跑图流程（NormalRun）。进行本交接中的测试前，
在 Main/Inc/app_config.h 只改一行：

```c
#define APP_ACTIVE_MODE APP_MODE_MOTION_DEBUG_TASK
```

测试结束后把同一行恢复成`APP_MODE_RESCUE_TASK`。也可以在CLion中选择
`MotionDebug`或`NormalRun` CMake配置来切换。两种模式是编译期互斥的，切换后
必须重新编译并烧录，不能在MCU运行中热切换。

F407 上电后初始化 LSM6DSV16X/IMU660RC 并静止校准陀螺仪零偏，把本地里程计
参考点设为车体三轮运动学中心 (0, 0, 0°)，每 10 ms 发送 TYPE=0x15 编码器
累计计数，接收 TYPE=0x1B 后在 F407 内部执行陀螺仪定角度或里程计定距运动，
并以约 20 Hz 发送 TYPE=0x1C 运动状态。

调试必须先架空车轮验证命令、符号和停止帧，再落地测试。新的 TURN/MOVE
命令会中止当前动作并从当前状态建立参考；STOP 会停车。动作是一次性自主
动作，受 F407 内部超时保护；上位机进程退出时仍应主动发送 STOP。

## 2. 接线和串口

```text
RDK X5 UART1_TX（物理 8 脚） ---> F407 PD9 / USART3_RX
RDK X5 UART1_RX（物理 10 脚） <--- F407 PD8 / USART3_TX
RDK X5 GND                    --- F407 GND
```

两端为 3.3 V TTL，使用 115200 8N1，TX/RX 交叉且共地，不能接 RS-232 电平。
USART1/串口2保留给后续用途，本阶段不使用。

## 3. 公共帧格式

除旧的 4 字节配置 ACK 外，业务帧固定 15 字节：

```text
索引:  0  1   2    3   4  5  6  7  8  9 10 11   12     13    14
数据: A3 B3 TYPE  SEQ P0 P1 P2 P3 P4 P5 P6 P7 CRC_LO CRC_HI C3
```

- TYPE 高 4 位为协议版本 1；SEQ 在同一发送方向独立递增，0xFF 后回到 0x00。
- CRC 覆盖 TYPE、SEQ、P0..P7 共 10 字节，使用 CRC-16/Modbus：初值 0xFFFF、多项式 0xA001、低字节先发。
- CRC 错误、帧尾错误、未知命令或保留位不为 0 时，F407 不更新控制快照。
- 上位机必须按字节流解析，支持拆包、粘包和噪声重同步。
- 同一个运动命令 SEQ 只执行一次；上位机每次新动作必须递增 SEQ，不能每次都固定发 0。

## 4. 上位机 → F407：运动命令 TYPE=0x1B

统一载荷如下，三个参数均为大端无符号 16 位数：

| 字段 | 含义 |
|---|---|
| P0 | 命令码 |
| P1 | 命令选项，只允许当前命令定义的 bit0 |
| P2..P3 | 参数 A |
| P4..P5 | 参数 B |
| P6..P7 | 参数 C |

### P0=0x00：STOP

P1..P7 必须全部为 0。例：

```text
A3 B3 1B 05 00 00 00 00 00 00 00 00 3F 27 C3
```

### P0=0x01：TURN_REL，陀螺仪闭环相对转角

- P1 bit0=0 为正方向，定义为场地航向逆时针；bit0=1 为负方向，顺时针。
- P2..P3 是转角绝对值，单位 0.01°，范围 1..36000。
- P4..P5 是旋转速度，单位车轮切向 mm/s；填 0 使用 F407 默认值，非 0 范围 50..700。
- P6..P7 必须为 0。

正转 90°、速度 300 mm/s：

```text
A3 B3 1B 00 01 00 23 28 01 2C 00 00 66 DB C3
```

F407 保存动作起始陀螺仪航向，闭环计算：

```text
field_yaw_delta = (imu_yaw - start_imu_yaw) * APP_LOCATION_IMU_YAW_SIGN
```

只有目标方向上的有符号误差进入范围才结束，不会因为向反方向转了同样
角度而误判完成。当前板上 APP_LOCATION_IMU_YAW_SIGN=-1；若实车正逆方向
相反，只改该坐标系符号并重新做 90°测试。

### P0=0x02：MOVE_DISTANCE，F407 本地里程计定向定距

- P1 bit0=0：方向相对当前车体，0°=当前车体 forward，90°=当前车体 physical left。
- P1 bit0=1：方向使用本地场地坐标，0°=+X，90°=+Y。
- P2..P3：方向，单位 0.01°，范围 0..359.99°。
- P4..P5：距离，单位 mm，范围 1..10000。
- P6..P7：速度，单位 mm/s；填 0 使用默认值，非 0 范围 50..700。

F407 每 10 ms 用三轮编码器和 IMU 更新本地位姿，将位移投影到目标方向，
接近目标时减速并停车，不用上位机返回的位置结束动作。三轮正运动学必须保持：

```text
forward = (M1 - M2) / sqrt(3)
left    = (M1 + M2 - 2*M3) / 3
```

当前端口约定为 M1=右轮、M2=左轮、M3=后轮；编码器符号由
APP_OMNI_M1/M2/M3_ENCODER_SIGN 统一修正。不要交换上位机的 forward/left
公式来掩盖底层轮序或符号错误。

车体左移 1 m、速度 300 mm/s：

```text
A3 B3 1B 03 02 00 23 28 03 E8 01 2C 72 36 C3
```

## 5. F407 → 上位机：运动状态 TYPE=0x1C

```text
P0 STATE
P1 COMMAND
P2..P3 PROGRESS
P4..P5 REMAINING
P6 HEALTH
P7 COMMAND_SEQ
```

状态值：0 IDLE、1 RUNNING、2 DONE、3 FAULT、4 STOPPED。

- TURN_REL 时 PROGRESS/REMAINING 单位为 0.01°；
- MOVE_DISTANCE 时 PROGRESS/REMAINING 单位为 mm；
- HEALTH bit0=IMU ready，bit1=F407 本地里程计有效，bit2=电机故障；
- P7 是被执行命令的 SEQ，外层 SEQ 是 F407 状态帧自己的发送序号。

上位机以 P7 关联动作，以 STATE=DONE/FAULT 结束测试；动作期间仍需读取
TYPE=0x15，不要只看 TYPE=0x1C 的进度值。

仓库内的参考编码器、状态解析器和单次测试工具是：

```text
tools/vision_protocol.py
tools/test_motion_uart.py
```

```bash
python -m tools.test_motion_uart turn --angle 90 --speed 300
python -m tools.test_motion_uart move --direction 0 --distance 1000 --speed 300
python -m tools.test_motion_uart move --direction 90 --distance 1000 --speed 300
python -m tools.test_motion_uart stop
```

## 6. 和 T265 机械安装参数的联调顺序

上位机继续负责 T265 原始 tracking origin、robot center 修正和融合日志；
F407 负责发送原始轮计数并按本地 IMU/里程计执行命令。诊断时保持：

```ini
camera_offset_forward_m = -0.0296
camera_offset_left_m = -0.0301
navigation_distance_compensation_enabled = false
```

偏置长度约 42.2 mm。每组动作结束后先停车，再保存完整日志。顺序：

1. 原地旋转 90°、180°、270°、360°；
2. 车体 forward 方向直行约 1 m；
3. 车体 physical left 方向横移约 1 m；
4. 带一次转向的直线返航。

原地旋转时，当前 r0=(-0.0296,-0.0301)m 的 raw tracking origin 在初始车体
坐标中应近似为：

| 相对旋转角 | tracking-origin 位移 |
|---|---|
| 90° | forward +59.7 mm，left +0.5 mm |
| 180° | forward +59.2 mm，left +60.2 mm |
| 270° | forward -0.5 mm，left +59.7 mm |
| 360° | 接近 0 |

重点同时查看：

```text
tracking_origin_delta_forward_m/left_m
robot_center_delta_forward_m/left_m
t265_vs_wheel_odom_distance_m
odom_increment_forward_m/left_m
fusion_increment_forward_m/left_m
wheel_kinematic_yaw_deg
gyro_yaw_delta_deg
```

正确表现是 raw tracking origin 走约 60 mm 圆周，robot_center_delta_* 接近 0，
T265 修正后的 t265.pose 不绕圈，轮式里程计位置也基本不变。若修正后仍有
同方向圆周，优先查 forward/left 偏置符号、T265 yaw 正负、重复补偿和实际
旋转中心；360°后积累几十厘米不可能只由 42.2 mm 杠杆臂造成。

直行 1 m 应接近 forward=1.0 m、left=0；横移 1 m 应接近 forward=0、
left=1.0 m；原地旋转应是 odom_increment_forward_m≈0、
odom_increment_left_m≈0，只有 yaw 变化。比例误差多查轮径、1768 counts/rev
和尺度；单方向错误多查轮序、单轮符号和机械安装。

## 7. 后续手柄链路预留

当前唯一执行入口是 F407 的 TYPE=0x1B，建议后续保持：

```text
手柄 -> 上位机 -> TYPE=0x1B canonical command -> F407
F407 -> TYPE=0x15/0x1C -> 上位机 -> 手柄界面
```

TYPE=0x19 预留给速度/方向保持类手柄命令，TYPE=0x1A 预留给参数标定/查询，
当前固件不会接受这两个类型。后续手柄控制必须有独立 HOLD/STOP 和周期性
失联停车看门狗，不要下发 PWM，也不要把一次性 MOVE_DISTANCE 与连续速度
保持命令混用。

## 8. 交付验收清单

- [ ] 逻辑分析仪确认 USART3 为 115200 8N1、3.3 V TTL；
- [ ] 上位机支持拆包/粘包，CRC 错帧不触发运动；
- [ ] 架空轮发送 STOP、90° TURN、1 m forward、1 m left；
- [ ] 每 10 ms 收到 TYPE=0x15，累计计数回绕按 16 位模差处理；
- [ ] 收到 TYPE=0x1C，并用 P7 COMMAND_SEQ 关联 DONE/FAULT；
- [ ] 地面完成 90/180/270/360°、forward 1 m、left 1 m 四组日志；
- [ ] 确认 navigation_distance_compensation_enabled=false 后再比较杠杆臂修正；
- [ ] 完成调试后切回完整跑图：`APP_ACTIVE_MODE APP_MODE_RESCUE_TASK`。

## 合并最新正常协议后的兼容性说明

正常模式基准为原仓库 `68a0802`，原有 `0x17` 状态、`0x18` 任务命令保持不变。
运动调试改用 `0x1B` 命令、`0x1C` 状态；旧调试程序必须更新，不能照旧编号发送。
本文十六进制示例已更新，也可用 `tools/vision_protocol.py` 生成帧（CRC 随 TYPE 改变）。
调试停车使用 `motion_stop_frame()`；旧 `0x13` 紧急停车不再适用。调试模式不启动舵机。
NormalRun / MotionDebug 配置会覆盖源码中的模式选择；切换配置后烧录对应目录的 WWW.elf。
