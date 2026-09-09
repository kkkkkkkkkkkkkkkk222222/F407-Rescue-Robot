# 飞智冰原狼4遥控与建图交接

## 设计边界

手柄不直接接STM32。飞智冰原狼4以PC/XInput模式通过2.4 GHz USB接收器连接RDK X5，RDK读取Linux `evdev`事件，再通过现有PCB串口1向F407发送15字节遥控帧。这样不需要把F407改成USB Host，也不会改动正式救援状态机。

固件分为三个互斥配置：

- `NormalRun`：原来的完整自动抓取和投送，执行`Task.c`。
- `MotionDebug`：上位机分段下发定角度、定距离动作，执行`Debug.c`。
- `Gamepad`：手柄连续遥控、机构安装调试和人工扫图，执行`Gamepad.c`，不执行`Task.c`。

在CLion的CMake配置中选择`Gamepad`，编译并烧录`build/Gamepad/WWW.elf`。恢复正式运行时选择`NormalRun`并烧录`build/NormalRun/WWW.elf`。这不是运行中热切换，两种固件不能同时控制电机。

## 手柄映射

必须持续按住RB才允许电机和舵机动作；松开RB立即发送未使能帧并停车。LB是35%精细速度。

| 输入 | 功能 |
|---|---|
| 右摇杆上/下 | 车体前进/后退 |
| 右摇杆左/右 | 全向左移/右移 |
| 左摇杆左/右 | 原地逆时针/顺时针旋转 |
| 左摇杆上/下 | 摄像头舵机3连续抬头/低头 |
| 十字键左 | 两侧夹爪对称连续闭合 |
| 十字键右 | 两侧夹爪对称连续张开 |
| 十字键上/下 | 大舵机1连续抬起/放下 |
| A/B/X/Y | 已进入协议但暂不执行动作，供后续扩展 |
| RB | 保持使能（deadman） |
| LB | 精细速度，最大速度降至35% |

摇杆在RDK端使用12%径向死区和缓升曲线，F407端再做5%小死区。右摇杆斜向量经过归一化，不会因对角输入超过单轴最大速度。默认最大平移速度700 mm/s，旋转等效速度300 mm/s；舵机3为60°/s，舵机1为45°/s，夹爪为35°/s。

限位保持为：舵机3 `0..165°`，舵机1 `45..120°`，左爪舵机4 `80..108°`，右爪舵机2 `100..72°`。夹爪方向相反，因此张开时左侧角度增大、右侧角度减小。

## RDK运行

安装运行依赖：

```bash
sudo apt install python3-evdev python3-serial
```

把用户加入串口和输入设备组后重新登录（组名以当前系统为准）：

```bash
sudo usermod -aG dialout,input "$USER"
```

插入2.4 GHz接收器，将手柄切到说明书所述PC/XInput连接模式，然后运行：

```bash
python3 tools/flydigi_vader4_remote.py --uart /dev/ttyS1
```

脚本会自动选择同时具有左右摇杆的输入设备，并优先选择名称包含Flydigi、Vader或Xbox的设备。自动识别不正确时，用`--device /dev/input/eventN`明确指定。

需要让现有T265扫图进程同时读取F407里程计时，由遥控脚本独占真实串口，并给扫图程序提供伪终端；`{pty}`会被替换为实际伪串口路径：

```bash
python3 tools/flydigi_vader4_remote.py \
  --uart /dev/ttyS1 \
  --map-command "/path/to/run_t265_map.sh --uart {pty}"
```

遥控脚本只把F407发来的里程计/状态转给扫图程序，并丢弃扫图程序向伪串口写回的数据，避免两个进程同时控制真实串口。扫图程序不能再单独打开`/dev/ttyS1`，也不要启用它原有的自动运动命令。

## 串口协议与保护

沿用115200 8N1和固定15字节外壳。`TYPE=0x19`、`P0=0x06`表示连续遥控：

```text
P0 TELEOP=0x06
P1 bit0 VALID, bit3 ACK_REQUIRED, bit5 RB使能
P2 forward  -100..100
P3 left     -100..100
P4 yaw      -100..100
P5 camera   -100..100
P6 十字键和ABXY位图
P7 速度百分比0..100
```

F407以`TYPE=0x1A`约20 Hz回复状态。LCD显示接收、RB使能、三轴速度以及四个舵机角度。任一载荷越界、CRC错误或保留标志异常均不会更新有效命令；超过150 ms没有合法新帧、电机故障或IMU失效都会立即停车。脚本正常退出时会发送数帧零输入和STOP；物理断线仍由F407的150 ms看门狗兜底。

## 首次测试顺序

1. 架空三个车轮，烧录`Gamepad`，确认LCD显示`RX:NO`且电机静止。
2. 启动脚本，确认`RX:YES`；不按RB时推动所有摇杆，车轮和舵机都不应动作。
3. 按住RB逐轴测试，并核对LCD正负方向；方向不符先停止落地测试。
4. 测试十字键长按在机械限位自动停止，再测试松RB和拔接收器均在150 ms左右停车。
5. 最后以LB精细速度落地测试，再启用正常速度和T265扫图。

资料依据：[飞智官方冰原狼4说明书](https://shops.flydigi.com/pages/flydigi-vader-4-pro-gaming-controller-user-manual)说明了PC端有线/无线连接；[Linux内核Gamepad规范](https://cdn.kernel.org/doc/html/latest/input/gamepad.html)规定左右摇杆通常映射为`ABS_X/Y`与`ABS_RX/RY`、十字键为`ABS_HAT0X/Y`或`BTN_DPAD_*`；[SDL上游Flydigi驱动](https://github.com/libsdl-org/SDL/blob/main/src/joystick/hidapi/SDL_hidapi_flydigi.c)也明确识别Vader 4 Pro及其有线/无线设备。
