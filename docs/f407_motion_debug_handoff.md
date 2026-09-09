# F407 T265建图调试交接

对齐上位机：`danmo-teng/shijue_fangan@4007ae0`的`t265_map`。

## 模式切换

当前普通CLion `Debug - Debug`默认编译建图固件：

```c
#define APP_ACTIVE_MODE APP_MODE_MOTION_DEBUG_TASK
```

它只执行`Main/Src/Debug.c`，不调用`Task.c`且不启动舵机。需要恢复正式任务时把这一行改为：

```c
#define APP_ACTIVE_MODE APP_MODE_RESCUE_TASK
```

两种模式都重新编译并使用现有`DAPLink OpenOCD`烧录`build/Debug/WWW.elf`。

## 接线和公共帧

```text
RDK UART1 TX（物理8）  -> F407 PD9/USART3 RX
RDK UART1 RX（物理10） <- F407 PD8/USART3 TX
GND                     -- GND
```

115200 8N1、3.3 V TTL、固定15字节：

```text
A3 B3 TYPE SEQ P0 P1 P2 P3 P4 P5 P6 P7 CRC_LO CRC_HI C3
```

CRC覆盖TYPE至P7共10字节，CRC-16/Modbus，低字节先发；参数为大端。

## 数据方向

- `0x15`：F407→RDK，约100 Hz三路编码器累计低16位。
- `0x19`：RDK→F407，T265扫描运动命令。
- `0x1A`：F407→RDK，约20 Hz扫描运动状态。

LCD显示`RX:True`表示已经收到至少一帧CRC正确、命令及参数合法的0x19帧；`RX:False`表示尚未收到合法扫描命令。

## 0x19命令

```text
P0 command
P1 flags
P2..P3 signed arg1
P4..P5 signed arg2
P6..P7 unsigned speed
```

命令：

- `0 STOP`：立即停车。
- `1 HOLD`：保持停车并回复DONE。
- `2 TURN_REL`：arg1为有符号0.1°；speed为0.1°/s。
- `3 MOVE_BODY`：arg1为forward mm，arg2为left mm，speed为mm/s。
- `4 MOVE_FIELD`：arg1为场地+X mm，arg2为场地+Y mm，speed为mm/s。
- `5 RESET_ODOM`：重置F407本地建图参考坐标。

flags：bit0 VALID、bit1 KEEP_HEADING、bit2 FIELD_FRAME、bit3 ACK_REQUIRED、bit4 CLEAR_FAULT。Debug.c会拒绝保留位、错误命令组合、零位移、超范围距离和速度。

TURN使用IMU闭环；MOVE_BODY/MOVE_FIELD使用三轮里程计投影定距、横向误差修正和IMU航向保持。新SEQ只执行一次，STOP优先，动作还受电机/IMU/里程计故障及超时保护。

## 0x1A状态

```text
P0 acknowledged command SEQ
P1 state: 0 IDLE, 1 RUNNING, 2 DONE, 3 ERROR, 4 STOPPED
P2 fault
P3 command
P4..P5 signed progress（移动mm，旋转0.1°）
P6..P7 IMU场地航向0..35999（0.01°）
```

上位机只有收到匹配命令SEQ的DONE才进入多步测试下一步。

## 运行

1. 选择CLion普通`Debug - Debug`，Rebuild并用`DAPLink OpenOCD`烧录。
2. 首次必须架空车轮。
3. RDK更新并编译最新版：

```bash
cd /home/sunrise/RDK_X5/shijue_fangan
git pull
cd t265_map
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j2
./install_desktop_launcher.sh
```

4. 打开桌面“T265环境扫描与建图”。最新版桌面入口自动带`--enable-motion`。
5. LCD应从`RX:False`变为`RX:True`，上位机不再停在`waiting 0x1A`。
6. 依次测试STOP、+90°、+180°、+360°、前进1 m、左移1 m和出发-转180°-返回。

没有实车验证前不得直接落地执行长距离动作。
