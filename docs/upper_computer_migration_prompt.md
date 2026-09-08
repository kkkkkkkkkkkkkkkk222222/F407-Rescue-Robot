# 给 shijue_fangan 负责人的修改提示词

以下正文可直接复制给上位机负责人或代码助手。核对基准：上位机 origin/main 39ae746，下位机功能分支 8d61de8（正常流程基于原仓库 68a0802）。

---

请在 danmo-teng/shijue_fangan 中完成 F407 运动调试协议迁移，保留正常比赛功能。先核对当前代码，不要全仓库替换 0x17/0x18。

## 参考来源

- 下位机：https://github.com/gandizm/F407-Rescue-Robot/tree/feat/uart-motion-debug
- 已核对的实现提交：8d61de8
- 交接：https://github.com/gandizm/F407-Rescue-Robot/blob/feat/uart-motion-debug/docs/f407_motion_debug_handoff.md
- 编码器/解析参考：同仓库 tools/vision_protocol.py
- 正常跑图基准：kkkkkkkkkkkkkkkk222222/F407-Rescue-Robot@68a0802

## 已发现的问题和修改范围

localization/tools/f407_motion_protocol.py 仍定义 MSG_MOTION_COMMAND=0x17、MSG_MOTION_STATUS=0x18；这对应旧下位机，现已不兼容。正常比赛的 localization/include/f407_protocol.hpp 使用 0x17 状态、0x18 任务命令，这是正确的，必须保留。

只将独立调试协议改成：
- 上位机→F407：0x1B。
- F407→上位机：0x1C。
- 原始编码器累计计数：0x15，不变。

检查 localization/tools/t265_f407_motion_map.py、t265_f407_debug.py、t265_f407_debug_map.py、test_f407_motion_protocol.py 的所有调用与硬编码。更新 t265_f407_debug.py 中旧的 9774dac 下位机版本说明，以及运行文档和样例。

## 协议内容

固定15字节：A3 B3 TYPE SEQ P0..P7 CRC_LO CRC_HI C3。
CRC16-Modbus，初值FFFF，覆盖TYPE、SEQ和8字节载荷；CRC低字节先发。载荷16位字段大端。TYPE变化后必须重新计算样例CRC。

0x1B：
- STOP：P0=0，其余全部0。
- TURN_REL：P0=1；P1 bit0=1表示负转角/顺时针，0表示正转角/逆时针；P2..3为绝对转角，0.01°，1..36000；P4..5为车轮切向速度mm/s，0用固件默认，否则50..700；P6..7=0。
- MOVE_DISTANCE：P0=2；P1 bit0=0是启动时车体方向，1是F407本地场地坐标；P2..3方向0.01°，0..35999；P4..5距离mm，1..10000；P6..7速度mm/s，0用默认，否则50..700。车体0°前、90°左。F407本地场地坐标不是T265地图坐标，不可直接混用。

0x1C：
- P0：0 IDLE、1 RUNNING、2 DONE、3 FAULT、4 STOPPED。
- P1：对应动作编号。
- P2..3：进度；P4..5：剩余量。转动单位0.01°，平移单位mm。
- P6：bit0 IMU就绪、bit1里程计有效、bit2电机故障。
- P7：关联的命令SEQ，不是故障码。当前帧没有详细fault-code字段，不能凭空解析。

正确样例：
STOP SEQ5：A3 B3 1B 05 00 00 00 00 00 00 00 00 3F 27 C3
+90° 300mm/s SEQ0：A3 B3 1B 00 01 00 23 28 01 2C 00 00 66 DB C3
左移1m 300mm/s SEQ3：A3 B3 1B 03 02 00 23 28 03 E8 01 2C 72 36 C3

## 模式隔离和安全

NormalRun是完整跑图；MotionDebug是独立测试。F407通过重新编译烧录切换，不支持上位机发一个指令热切换。不要把USART1原有舵机维护控制台当成这个模式开关。

调试界面先等到有效0x1C健康状态，确认实际固件支持调试，再允许动作。不允许收到正常0x17就当调试就绪。F407启动会发0x1C并约20Hz更新。可见健康位不是硬件运动安全证明，必须先架空验证。

同一USART3只能有一个串口所有者，正常流程和调试不能同时发运动命令。若共用串口服务，显式区分模式并路由状态，保留T265及0x15日志。模式不匹配应报错，不自动降级或兼容发送旧调试帧。

按钮动作一次提交；P7关联当前命令，忽略不匹配的完成帧。考虑SEQ回绕、重复帧、取消、超时和串口异常。禁止自动换SEQ重发动作导致重复运动。取消、超时及可控退出时尽力发送新的SEQ的0x1B STOP，停止发送其他动作。旧0x13急停不是当前固件调试接口；不要依赖进程退出让F407自动停下。

## T265安装测试及日志

保留±90/180/270/360°、forward 1m、left 1m、转向后返航实验。F407自主完成陀螺仪转角和里程计定距，上位机记录对照数据，不用T265位置提前终止测试以掩盖误差。

保留tracking_origin_delta_forward_m/left_m、robot_center_delta_forward_m/left_m、t265_vs_wheel_odom_distance_m、odom_increment_forward_m/left_m、fusion_increment_forward_m/left_m、wheel_kinematic_yaw_deg、gyro_yaw_delta_deg等已有诊断字段，附加命令SEQ、动作参数、0x1C状态/进度/剩余量及时间戳。0x1C不提供原始陀螺仪yaw，不可把命令角度伪装成实测gyro字段；没有来源时明确标为不可用。

保持navigation_distance_compensation_enabled=false。不要为让轨迹重合而改T265偏置、轮序、符号、编码器尺度或重复补偿。调试约定F=(M1-M2)/sqrt(3)、L=(M1+M2-2*M3)/3；若当前上位机映射不同，先记录映射与依据并报告差异，不能顺带改坏正常里程计。

## 验收交付

增加正负转角、360°、四方向距离、非法参数、CRC错误、拆包粘包、SEQ回绕/状态关联、STOP、模式不匹配的测试。
测试正常0x17/0x18不受影响，旧调试帧不能当新协议使用。
运行相关Python测试，给出可执行启动命令、串口所有权说明、修改清单和测试结果。没有实车测试就明确标注，不宣称机械标定已通过。
