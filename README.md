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
2. TIM6提供1 ms基础节拍：每10 ms采样编码器并执行一次电机速度环；自主任务开启时每20 ms更新一次目标；每100 ms只发布LCD/USART1刷新请求。
3. LCD直接进入状态界面，不执行RGB色块测试。
4. USART1只有PA9 TX，用于启动信息和遥测；USART3只有PD9 RX，使用DMA1 Stream1 Channel4循环DMA接收视觉帧，PD8已经释放。
5. `APP_ENABLE_AUTOMATIC_MOTOR_TEST=1`只表示编译按键测试，上电仍保持停车。必须先释放再按下PA0/S1才会启动三轮110 mm/s测试，再按一次停车。LCD分别显示`KEY START`、`PID 110MM`或`FAULT RESET`。

三只全向轮都设置为同方向110 mm/s时，底盘趋向原地旋转；这只验证单轮约0.5 r/s的闭环，不等于底盘每2秒自转一圈。接电机12 V前必须悬空车轮。

## 引脚与外设

| 功能 | 引脚/外设 | 参数 |
| --- | --- | --- |
| M1 | PA2/PA3 TIM5 CH3/CH4；PE9/PE11 TIM1编码器 | 20 kHz，左前轮 |
| M2 | PE5/PE6 TIM9 CH1/CH2；PA6/PA7 TIM3编码器 | 20 kHz，右前轮 |
| M3 | PB10/PB11 TIM2 CH3/CH4；PD12/PD13 TIM4编码器 | 20 kHz，后轮 |
| 舵机S1-S4 | PC6-PC9，TIM8 CH1-CH4 | 50 Hz，500-2500 us |
| LCD | PB13 SCK、PB15 MOSI、PB12 CS、PB14 RESET、PC5 DC、PB1 BL | ST7735，128x160 |
| USART1 | PA9 TX | 115200 8N1，只发送 |
| USART3 | PD9 RX | 115200 8N1，只接收，64字节循环DMA |
| 测试按键 | PA0/S1，内部下拉 | 30 ms消抖，切换测试启动/停车 |

M4、TIM10/TIM11、PB8/PB9、PD3/PD4和EXTI3已经整体删除，不再存在第四电机软编码器。

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
void Motor_Move(int16_t forward, int16_t lateral, int16_t rotate);
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
- 任意一个轮子出现`DIR`或`STALL`，三轮立即一起停车；普通恒速测试、`Motor_Move()`和`Go_distance()`使用同一联停规则。
- 故障会锁存，`Motor_Stop()`不会清除，状态机下一周期也不能重新启动电机。检查接线和机械问题后复位MCU才能恢复。
- AT8236停车时内部先使用`IN1=IN2=1`低侧制动约60 ms，然后切换到`IN1=IN2=0`高阻滑行/休眠；重复调用`Motor_Stop()`不会无限延长制动。

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

正数前进、负数后退，单位为米。它记录M1/M2的64位起点，用：

```text
forward_distance = (M2_distance - M1_distance) / sqrt(3)
```

估算底盘前进距离，M1/M2轮速为`-0.866/+0.866`倍，M3停止。默认110 mm/s巡航；减速区取100 mm和全程一半中的较小值，进入减速区后同时线性降低速度目标和PWM上限，并清除巡航积分。进入3 mm容差后主动制动。

完成、故障状态会锁存，循环调用不会再次启动：

```c
MotorDistanceStatus result = Go_distance(0.5f);
if (result == MOTOR_DISTANCE_DONE) {
    Motor_Stop(); /* 确认完成、回到IDLE，之后才能启动下一段 */
}
```

定距运行时若连续1秒没有获得至少0.25 mm的新进度，会作为`FAULT`三轮联停。`NaN`或无穷大参数返回`INVALID`，不会写入控制状态。

## 编码器、并发与视觉

`EncoderStatus.position`已经改为`int64_t`，避免32位累计位置连续运行数天后溢出。16位定时器仍用`(int16_t)(current-previous)`正确处理回绕，前提是单个10 ms增量不超过32767。

电机公开设置函数和状态读取使用短临界区；电机状态、编码器状态和视觉坐标都按快照读取，避免主循环遥测与TIM6/串口中断互相读到半更新数据。舵机角度缓存使用`volatile`。

USART3继续解析：

```text
A3 B3 XH XL YH YL C3
```

- `X=Y=0xFFFF`：停止；自主任务需复位后重新运行。
- `X=Y=0xFFFE`：无目标。
- 只有`X<=639`且`Y<=479`的其他坐标才有效，异常坐标不会驱动底盘。
- 超过250 ms未更新视为过期。

循环DMA在`Size=64`时消费本圈剩余数据并把软件位置归零，可持续解析无IDLE的连续数据。DMA首次启动或错误恢复失败时，主循环每100 ms重试；LCD显示`DMA ERR`，USART1显示`USART3 RX: ERR`，不再静默失效。

## 编译与安全测试顺序

```powershell
cmake --preset Debug
cmake --build --preset Debug
```

输出位于`build/Debug/WWW.elf/.hex/.bin`。建议顺序：先断开电机12 V验证LCD和串口；手动转每个轮子核对原始编码器符号；悬空轮胎后按S1启动110 mm/s测试；分别验证拔掉某一路编码器和反向编码器能否三轮联停；最后才让车轮着地并测试`Go_distance()`。方向、堵转门限、1768计数/圈、45%基础PWM和PID参数都必须结合实车数据继续整定。
