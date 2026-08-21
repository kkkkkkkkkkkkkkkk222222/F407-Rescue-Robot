# WWW - STM32F407VET6 硬件测试与可选控制层工程

本工程首先用于验证购买的 STM32F407VET6 开发板与自制扩展板，并在已经验证的底层之上移植了 LED_3 的通用 PID、三电机速度闭环、三轮全向运动学、视觉帧解析、摄像头/夹爪动作和目标跟随状态机。开发环境是 CLion + CMake + GNU Arm Embedded，调试器是 Ozone + J-Link。

底层引脚、定时器、DMA、中断和驱动接口仍以本 README 与当前 F407 工程为唯一基准。移植控制层默认关闭，`APP_ENABLE_TASK=0`，所以上电行为仍是原来的板级测试，不会因为新增状态机而改变。USART3 会解析 LED_3 的 7 字节视觉帧用于联调，但这不等同于完整 RDK 正式通信协议。

## 1. 现在能不能直接烧

可以烧当前重新编译后的 `build/Debug/WWW.elf`，但这一版已经默认开启低速电机往返测试。第一次验证屏幕时建议不接电机 12 V；连接电机电源前必须把车轮悬空，并确认周围没有机械干涉。

当前默认上电行为：

1. 四路电机 PWM 外设先以 0 占空比启动；进入主循环后四台同时运行，默认持续输出 50% PWM。正转 0.5 秒后停车 0.1 秒，再反转 0.5 秒并停车 0.1 秒，如此循环。
2. 三路硬件编码器和第四路 GPIO 编码器开始计数。
3. 四路舵机输出 90 度，即约 1500 us 脉宽；舵机可能在上电时移动到中位。
4. TIM6 每 1 ms 进入一次中断，除增加软件毫秒计数外，还用硬件节拍分频：每 10 ms 采样编码器，开启自主任务时每 20 ms 更新状态机和三电机速度环；中断内仍不做串口、格式化或 LCD 阻塞操作。
5. TFT 初始化后依次显示红、绿、蓝 250 ms，然后进入状态界面。
6. TIM6 每 100 ms 发布一次低优先级刷新序号，主循环合并处理最新一次 USART1/USART3 遥测和 LCD 刷新；接收均使用独立的循环 DMA 缓冲和空闲线事件。
7. USART3 收到的数据还会按 DMA 新增区间送入视觉帧解析器；自主任务默认关闭，因此有效视觉帧只出现在遥测中，不会驱动电机或舵机。

如果舵机已经安装在机构上，第一次烧录前建议先拔掉舵机 5V_BUS，确认 PWM 波形和中位方向后再供电。

## 2. 常见现实问题原因

开发板 P2 TFT 排针网络顺序不同于常规spi接口，是：

```text
1  3V3
2  GND
3  PB15  MOSI/SDA
4  PB13  SCK/SCL
5  PB12  CS
6  PB14  RESET/RES
7  PC5   DC/RS/A0
8  PB1   BL/BLK
```

上一版按旧方案错误地使用了 `PB14=DC、PC5=RESET`。实际 PB14 是 RESET，因而被程序当作 DC 长期拉低，LCD 始终处于硬件复位状态，只能看到背光亮和白屏。现在已经恢复为：

```text
PB12 = CS
PB14 = RESET
PC5  = DC/RS
PB1  = BACKLIGHT
```

新固件开机带红、绿、蓝全屏自检：

- 能看到色块和文字：LCD 的 SPI、CS、RESET、DC 基本正常。
- 能看到完整色块和文字：LCD 的 SPI、CS、RESET、DC 和显存起点基本正常。
- 仍然全白：先用万用表或示波器检查 PB14 是否先低后高、PB12 是否有片选脉冲、PB13 是否有时钟、PB15 是否有数据。
- 背光不亮：检查 PB1 和 TFT 模组 BL 极性；当前代码按高电平点亮。
- 色块颜色交换但能显示：面板的 RGB/BGR 顺序不同，需要调整 ST7735 的 MADCTL 参数，不是 SPI 通信失败。
- 本屏幕顶部、左侧残留彩边是地址窗口从 `(2,1)` 开始造成的，当前已把显存偏移改为 `(0,0)`；方向和 RGB/BGR 正常，因此没有改动 MADCTL。

## 3. 最终引脚和外设表

### 3.1 电机

| 电机 | IN1 | IN2 | PWM 外设 | PSC/ARR | 频率 | 自主控制角色 |
| --- | --- | --- | --- | --- | --- | --- |
| M1 | PA2 | PA3 | TIM5 CH3/CH4 | 0/4199 | 20 kHz | LED_3 左前轮 |
| M2 | PE5 | PE6 | TIM9 CH1/CH2 | 1/4199 | 20 kHz | LED_3 右前轮 |
| M3 | PB10 | PB11 | TIM2 CH3/CH4 | 0/4199 | 20 kHz | LED_3 后轮 |
| M4 | PB8 | PB9 | TIM10 CH1/TIM11 CH1 | 0/8399 | 20 kHz | 保留，不参与三轮运动 |

### 3.2 编码器

| 编码器 | A/B | 实现 |
| --- | --- | --- |
| M1 | PE9/PE11 | TIM1 Encoder Mode，滤波 4 |
| M2 | PA6/PA7 | TIM3 Encoder Mode，滤波 4 |
| M3 | PD12/PD13 | TIM4 Encoder Mode，滤波 4 |
| M4 | PD3/PD4 | PD3 上升沿 EXTI，PD4 判断方向，x1 软件解码 |

自主任务只使用 M1-M3 及其三路硬件编码器；M4 的 PWM 和编码器仍保留用于底板完整性测试，但 `Motor_Update()` 会强制 M4 停止。

### 3.3 舵机、LCD 和串口

- S1-S4：PC6-PC9，TIM8 CH1-CH4，50 Hz，500-2500 us。
- LCD：PB13 SCK、PB15 MOSI、PB12 CS、PB14 RESET、PC5 DC、PB1 BL。
- USART1：PA9 TX、PA10 RX，115200 8N1；RX 使用 DMA2 Stream2 Channel4。
- USART3/RDK：PD8 TX、PD9 RX，115200 8N1；RX 使用 DMA1 Stream1 Channel4。
- 用户按键 S1：PA0，按下接通 3.3 V，GPIO Input + Pull-down；每次按下把电机占空比按 10% 循环调节。
- 系统节拍：TIM6，1 ms。
- 调试：PA13 SWDIO、PA14 SWCLK。

## 4. 工程目录

```text
WWW/
├─ Main/                 手写主应用层，与 CubeMX 的 Core/main.c 区分
│  ├─ Inc/
│  │  ├─ app_config.h    自动测试和 LCD 参数
│  │  ├─ Robot.h         机器人系统初始化与总调度接口
│  │  ├─ encoder.h       编码器接口
│  │  ├─ mechanism.h     摄像头与夹爪动作封装
│  │  ├─ motor.h         电机接口
│  │  ├─ pid.h           通用限幅 PID
│  │  ├─ Task.h          可选目标跟随状态机接口
│  │  ├─ servo.h         舵机接口
│  │  ├─ Lcd.h           LCD 接口与颜色定义
│  │  └─ vision.h        LED_3 视觉帧解析接口
│  └─ Src/
│     ├─ Robot.c         上电流程、周期调度、UART 和 HAL 回调
│     ├─ encoder.c       三路硬件编码器和一路软解码
│     ├─ mechanism.c     基于 TIM8 舵机驱动的摄像头/夹爪动作
│     ├─ motor.c         四路底层 PWM、三电机速度环与全向运动学
│     ├─ pid.c           PID 计算、积分和输出限幅
│     ├─ Task.c          非阻塞视觉目标跟随状态机
│     ├─ servo.c         四路 TIM8 舵机 PWM
│     ├─ Lcd.c           ST7735 初始化、填充、几何图形和 5x7 字符显示
│     └─ vision.c        可跨 DMA 分段/回绕的视觉协议解析
├─ Core/                 CubeMX 生成代码
├─ Drivers/              STM32 HAL/CMSIS
├─ WWW.ioc               CubeMX 配置源文件
├─ WWW.jdebug            Ozone 工程
├─ CMakeLists.txt        CLion/CMake 构建入口
└─ HARDWARE_REVIEW.md    PCB 原理图风险复核
```

`Main` 是手写应用代码目录，`Core/Src/main.c` 仍是程序入口，两者不是重复的 `main`。`Task.h` 与 `Task.c` 也不是两个任务：`.h` 声明状态和公开函数，`.c` 实现同一个视觉自主任务。`Robot.c` 负责整个机器人程序的初始化、周期调度、遥测和 HAL 回调；`Task.c` 只负责视觉自主状态机，两者职责不同。

## 5. 程序启动流程

`Core/Src/main.c` 完成 HAL、时钟和 CubeMX 外设初始化后调用：

```c
Robot_Init();
```

主循环持续调用：

```c
Robot_Process();
```

### 5.1 `Robot_Init()`

依次完成：

1. CubeMX 先打开 DMA1/DMA2 时钟和两路 RX DMA 中断；中断优先级为 TIM6=4、DMA=5、EXTI3=5、USART=7，保证控制节拍最高，DMA 满缓冲又先于 UART IDLE 处理回绕边界。
2. `Motor_Init()`：启动八个电机 PWM 通道、初始化 M1-M3 速度 PID，并将四路命令清零。
3. `Encoder_Init()`：启动 TIM1/TIM3/TIM4 Encoder Mode，计数从 0 开始。
4. `Servo_Init()`：启动 TIM8 四个 PWM 通道并将四路角度设置为 90 度。
5. 初始化视觉帧解析器，为 USART1、USART3 分别启动 64 字节循环 DMA，并用 `HAL_UARTEx_ReceiveToIdle_DMA()` 在线路空闲或缓冲区满时报告新数据；关闭无必要的半传输中断。
6. 若 `APP_ENABLE_TASK=1`，初始化摄像头动作和目标跟随状态机；默认不执行自主任务。
7. 初始化 ST7735，执行 RGB 色块测试，绘制固定标签和实时值。
8. USART1、USART3 分别发送启动信息。
9. 所有阻塞式上电初始化结束后才启动 TIM6 1 ms 更新中断，避免 LCD 延时在调度器中形成历史周期积压。

### 5.2 `Robot_Process()`

控制周期与低优先级工作的调度已经分离：

- TIM6 中断每 10 ms 直接调用 `Encoder_Sample10ms()`，不受 LCD、串口格式化或主循环耗时影响。
- `APP_ENABLE_TASK=1` 时，TIM6 中断每 20 ms 在当次编码器采样之后调用 `Task_Update()`；状态机和 M1-M3 速度闭环因此共用同一硬件节拍，M4 保持停止。
- TIM6 每 100 ms递增一次刷新发布序号；主循环只消费最新序号并更新一次遥测和 LCD。若低优先级工作曾被延迟，不会连续补跑过期的100 ms任务，也不会反过来拖慢10/20 ms控制周期。
- 若编译时开启舵机测试，每 1500 ms 切换一次 0/90/180/90 度。
- 默认开启四电机同步测试：四台同时正转 0.5 秒、停车保护、同时反转 0.5 秒，再循环。
- 若开启自主任务，它优先于自动往返测试，后者不参与编译。
- PA0 按键采用 30 ms 软件消抖；默认 50%，每按一次按 `50% -> 60% -> ... -> 100% -> 50%` 循环。

10 ms和20 ms是由TIM6分频得到的硬实时发布周期。100 ms是显示/遥测的发布周期：实际 SPI 刷新仍有传输耗时，属于低优先级软实时工作，但采用序号合并后既不会产生启动补任务风暴，也不会破坏电机控制周期。

## 6. 各驱动详细说明

### 6.1 电机 `motor.c`

公开接口：

```c
void Motor_Init(void);
void Motor_Control(uint8_t id, int16_t speed);
void Motor_StopAll(void);
int16_t Motor_GetCommand(uint8_t id);
void Motor_Update(void);
void Motor_Stop(void);
void Motor_Forward(int16_t speed);
void Motor_Back(int16_t speed);
void Motor_RotateLeft(int16_t speed);
void Motor_RotateRight(int16_t speed);
void Motor_Move(int16_t forward, int16_t lateral, int16_t rotate);
void Motor_Follow(int16_t speed, int16_t turn);
int16_t Motor_GetTarget(uint8_t id);
```

`Motor_Control()` 是板级原始 PWM 接口，将 `-1000..1000` 映射为对应定时器的 CCR。因为 M1-M3 的 ARR 是 4199、M4 的 ARR 是 8399，代码根据每个定时器自己的 Period 计算占空比，不假设所有电机使用相同 ARR。

示例：

```c
Motor_Control(1, 300);   /* M1 正转约 30% */
Motor_Control(2, -500);  /* M2 反转约 50% */
Motor_Control(3, 0);     /* M3 停止 */
```

自主任务使用的是上层目标接口。`Motor_Move(forward, lateral, rotate)` 保留 LED_3 的三轮全向逆运动学：

```text
M1 = -0.866 * forward - 0.5 * lateral + rotate
M2 = +0.866 * forward - 0.5 * lateral + rotate
M3 =                  lateral + rotate
```

如果任一路绝对值超过 1000，三个目标会按同一比例缩放，保持运动方向而不单独裁剪某一路。`Motor_Forward()`、`Motor_Back()`、`Motor_RotateLeft()`、`Motor_RotateRight()` 和 `Motor_Follow()` 都是对该公式的封装。

`Motor_Update()` 由 TIM6 每 20 ms调用一次，并在同一中断中消费刚完成的第二次10 ms编码器采样。它将 `-1000..1000` 目标换算为计数目标，并使用“65% 前馈 + PID 修正”生成实际 PWM。目标为零或方向翻转时会复位对应 PID，M4 每次更新都写 0。当前的 `120 count/20 ms`、前馈和 PID 参数来自 LED_3，代码已移植完成，但仍需要在当前轮径、减速比和供电条件下实测整定。

当前采用 AT8236 的快衰减 PWM 控制：

- 正转：`IN1=PWM，IN2=0`。
- 反转：`IN1=0，IN2=PWM`。
- 停止：`IN1=0，IN2=0`，输出高阻、滑行，并在保持低电平后进入休眠。

因此硬件使用两路定时器 PWM 引脚是为了让两个方向都能调速，但正常运行时不是两路同时输出互补 PWM，而是每次仅一路输出 PWM。AT8236 也支持在 PWM 关断阶段切换到 `IN1=IN2=1` 的制动/慢衰减方式，但当前测试程序没有使用这种模式。参考：[AT8236 数据手册](https://datasheet.lcsc.com/lcsc/2109242230_ZHONGKEWEI-AT8236_C2827823.pdf)。

实测约 50% 才能从静止起转不是芯片规定的固定门槛。12 V 下 50% 快衰减 PWM 的等效驱动能力大约相当于数伏量级；更低占空比时，绕组电流和启动转矩可能不足以克服电刷、轴承、减速箱和车轮的静摩擦。快衰减模式在关断阶段让电流下降较快，也会使低占空比的启动能力比慢衰减弱。已经转起来后，维持转动所需占空比通常会低于静止起转占空比。如果始终必须高于 50%，还应测量 12 V 母线是否下跌，并检查模块的 VREF/ISEN 电流限制。

### 6.2 编码器 `encoder.c`

公开接口：

```c
void Encoder_Init(void);
void Encoder_Sample10ms(void);
int32_t Encoder_Get(uint8_t id);
int32_t Encoder_GetDelta10ms(uint8_t id);
int32_t Encoder_TakeControlDelta(uint8_t id);
void Encoder_Reset(uint8_t id);
```

M1-M3 的硬件计数器是 16 位。采样时使用：

```c
int16_t delta = (int16_t)(current - previous);
```

这样能在正常 10 ms 增量没有超过 32767 的前提下自动处理 65535 到 0 的回绕，并累加到 32 位位置。

每次 10 ms 采样还会把增量加入独立的控制累计值。`Encoder_TakeControlDelta()` 读取并清零该累计值，所以 20 ms 速度环能消费两次采样之和，同时不影响 LCD/遥测继续读取最近一次 `Encoder_GetDelta10ms()`。`Encoder_Reset()` 可单独清零任一路计数和控制累计值。

M4 当前只在 PD3 上升沿计数一次，并读取 PD4 决定方向，所以是 x1 解码。若方向相反，可交换编码器 A/B 线，或反转 `Encoder_OnExti()` 中的正负号。

### 6.3 舵机 `servo.c`

公开接口：

```c
void Servo_Init(void);
void Servo_Set(uint8_t id, uint8_t angle);
uint8_t Servo_GetAngle(uint8_t id);
```

TIM8 的计数频率为 1 MHz，所以 CCR 数值就是高电平微秒数：

```text
0°   -> 500 us
90°  -> 1500 us 左右
180° -> 2500 us
```

不同舵机的安全机械范围可能小于 500-2500 us。安装到机构后应缩小范围，不能直接假设所有舵机都允许 0-180 度全行程。

### 6.4 LCD `Lcd.c`

SPI2 使用：

```text
Master
1-line transmit
8 bit
MSB first
Mode 0: CPOL Low, CPHA 1 Edge
Software NSS
Prescaler 4 -> 10.5 MHz
```

每次命令事务由 PB12 软件控制 CS，PC5 控制命令/数据，PB14 执行硬件复位。初始化包含退出睡眠、帧率、电源、VCOM、16-bit RGB565、Gamma、Normal Display 和 Display On。

LCD 没有 MISO，因此 `HAL_SPI_Transmit()` 返回成功只能说明 STM32 SPI 外设完成发送，不能证明屏幕真实收到命令。RGB 色块自检和示波器波形才是实际反馈。

公开接口：

```c
bool LCD_Init(void);
void LCD_SetBacklight(bool enabled);
void LCD_FillScreen(uint16_t color);
void LCD_FillRect(...);
void LCD_DrawPixel(...);
void LCD_DrawLine(...);
void LCD_DrawRectangle(...);
void LCD_DrawCircle(...);
void LCD_DrawText(...);
```

LED_3 的逐点、Bresenham 直线、矩形和中点圆算法已移植到当前驱动，并增加屏幕边界裁剪。没有复制 LED_3 的 240x320 LCD 初始化、旧引脚和字体表；这些内容与当前已验证的 128x160 ST7735 不兼容。

这块 1.8 英寸 128x160 ST7735 屏幕的可见区域与显存 `(0,0)` 对齐。旧配置的 `(2,1)` 会让全屏填充漏掉顶部和左侧像素，保留上一次显存中的彩边；当前配置为：

```c
#define APP_LCD_WIDTH       128U
#define APP_LCD_HEIGHT      160U
#define APP_LCD_X_OFFSET    0U
#define APP_LCD_Y_OFFSET    0U
```

偏移只会造成画面错位、边缘残留或裁切，不会造成完全白屏。如果以后更换不同标签版本的 ST7735 模组，才可能需要重新测定偏移。

### 6.5 UART、DMA 和中断

TIM6 每 100 ms发布一次刷新序号；主循环收到新序号后，让USART1、USART3同时输出同一份遥测：

```text
Encoder: M1=累计值(10ms增量) ... M4=累计值(10ms增量)
Servo: S1=90 S2=90 S3=90 S4=90
PWM: motor=20kHz servo=50Hz
Motor: M1=实际PWM/闭环目标 M2=实际PWM/闭环目标 M3=实际PWM/闭环目标 M4=实际PWM/RES
Vision3: TARGET/NONE/STOP X=坐标 Y=坐标 age=帧龄 task=ON/OFF
USART1: WAIT 0x00  USART3: WAIT 0x00
```

两路串口各有一个 64 字节循环 DMA 接收缓冲。`HAL_UARTEx_RxEventCallback()` 在收到一段数据后记录该路最近的一个字节；LCD 的 `UART` 行分别显示 `1:xx`、`3:xx`，遥测也分别显示两路最近字节。当前不会自动在 USART1 与 USART3 之间转发数据。

USART3/RDK 还会按“软件读位置 -> 本次 DMA 写位置”取出所有新增字节，再送给 `Vision_ParseBytes()`。当 HAL 在循环 DMA 满缓冲时报告 `Size=64`，代码会消费本圈剩余区间并把软件位置归零，因此连续多个满缓冲回调也会逐圈解析，不依赖线路出现 IDLE；DMA 中断优先级高于 USART，又避免回绕点的 IDLE 先于 TC 导致重复消费。这比 LED_3 原来的 USART1 单字节中断更适合当前 F407 的 DMA 架构。当前只解析以下 LED_3 旧帧，不代表已经实现完整 RDK 协议：

```text
A3 B3 XH XL YH YL C3
```

- `X=Y=0xFFFF`：立即进入停止状态；自主任务一旦停止，需复位后才重新运行。
- `X=Y=0xFFFE`：当前没有目标。
- 其他坐标：保存为有效目标；超过 250 ms 未更新后视为超时。

DMA 采用循环模式，正常收到数据后不需要在每次回调里重新启动；只有 UART 发生溢出、噪声或帧错误时，错误回调才终止并重新挂接对应 DMA。代码关闭了 DMA 半传输中断，降低短消息接收时的无效中断频率。

周期遥测使用 `HAL_UART_Transmit_IT()`，两路 UART 共用一份只读发送缓冲；只有两路都完成发送后，主循环才会生成下一帧，避免缓冲在发送过程中被改写。启动横幅仍使用一次阻塞发送，之后的周期输出均为中断发送。

中断入口：

- `EXTI3_IRQHandler()` -> M4 编码器。
- `TIM6_DAC_IRQHandler()` -> 1 ms 系统节拍。
- `DMA2_Stream2_IRQHandler()` -> USART1 RX DMA 缓冲区完成事件。
- `DMA1_Stream1_IRQHandler()` -> USART3 RX DMA 缓冲区完成事件。
- `USART1_IRQHandler()` -> USART1 空闲线、发送完成和错误回调。
- `USART3_IRQHandler()` -> USART3 空闲线、发送完成和错误回调。

### 6.6 从 LED_3 移植的控制层

#### PID `pid.c`

保留 LED_3 的固定周期位置式 PID，并补充空指针保护。积分项和最终输出均限幅，第一次更新不计算微分，状态切换、停车或方向变化时可调用 `Pid_Reset()` 防止旧积分继续影响输出。该算法不包含时间参数，所以调用周期必须固定；当前由最高优先级的 TIM6 中断按 20 ms硬件节拍调用，不再依赖主循环及时返回。

#### 视觉解析 `vision.c`

协议含义与 LED_3 一致，但解析器不再占用 HAL 回调，也不依赖某个串口。`Robot.c` 负责从 USART3 循环 DMA 取出新增字节，解析器只负责帧头重同步、拼帧和生成并发安全快照，因此以后更换串口或加入环形队列时不需要改协议算法。

#### 摄像头与夹爪 `mechanism.c`

保留 `Camera_SetAngle()`、`Camera_ChangeAngle()`、`Camera_Wide()`、`Claw_Open()` 和 `Claw_Close()` 等 LED_3 接口语义，但底层改为调用 F407 已验证的 `Servo_Set()`。没有复制 LED_3 的 PE12-PE15 GPIO 软件 PWM，也没有占用 TIM6；当前默认把摄像头映射到 S3、夹爪映射到 S4，此角色分配来自 LED_3，机械接线后仍需确认。

#### 目标跟随 `Task.c`

状态流程保留为：摄像头广角 -> 等待目标 -> 前移脱困 -> 旋转搜索 -> 接近目标 -> 停止。转向和摄像头分别使用独立 PID，所有等待均基于毫秒截止时间。TIM6 每20 ms调用一次状态机，代码只进行快照读取、PID计算、CCR更新和状态切换，不在中断里执行延时、SPI、串口发送或格式化；正常运动状态随后调用 `Motor_Update()`，因此运动学目标和编码器速度闭环使用同一个固定20 ms周期。STOP 分支由 `Motor_Stop()`直接清零目标和PWM后提前返回。

#### 三轮运动与速度闭环 `motor.c`

LED_3 使用的三路正好对应当前 F407 的 M1/TIM5+TIM1、M2/TIM9+TIM3、M3/TIM2+TIM4，因此三轮全向运动学、速度目标归一化、前馈、PID 修正和方向切换复位逻辑均已移植。M4/TIM10+TIM11 不参与运动学，也不进入速度环，避免其 x1 软件编码倍率混入三路硬件 x4 闭环。

电机物理正方向和编码器正方向分别由 `APP_OMNI_Mx_MOTOR_SIGN`、`APP_OMNI_Mx_ENCODER_SIGN` 控制。若闭环一启动就朝错误方向加速，应立即断开电机电源，先单独验证编码器符号与 PWM 符号，不能靠继续增大 PID 抑制。

## 7. 自动测试开关

文件：`Main/Inc/app_config.h`

```c
#define APP_ENABLE_AUTOMATIC_MOTOR_TEST  1
#define APP_MOTOR_TEST_MIN_COMMAND       500U
#define APP_MOTOR_TEST_DEFAULT_COMMAND   APP_MOTOR_TEST_MIN_COMMAND
#define APP_MOTOR_TEST_COMMAND_STEP      100U
#define APP_MOTOR_TEST_DIRECTION_MS      500U
#define APP_MOTOR_TEST_REVERSAL_PAUSE_MS 100U
#define APP_MOTOR_KEY_DEBOUNCE_MS        30U
#define APP_ENABLE_SERVO_SWEEP_TEST      0
#define APP_LCD_STARTUP_COLOR_TEST       1
#define APP_ENABLE_TASK                  0
#define APP_MOTOR_MAX_COUNT_20MS         120
#define APP_MOTOR_FEED_FORWARD_PERCENT   65
```

- 电机命令范围为 `-1000..1000`，当前测试限定在 50%-100%，负号代表反向。四路始终使用相同命令。
- PA0/S1 按下为高电平，内部下拉保证松开时为低；按键事件只在稳定按下沿触发一次，长按不会连续跳档。
- LCD 的 `MOTOR` 行显示 `ALL +50%`、`ALL -80%` 或 `STOP`；串口遥测中 M1-M4 应显示相同命令。
- 四台电机同时从静止状态以 100% 启动时，瞬时电流可能远高于四台额定电流之和。只允许在车轮悬空、电源和线束能够承受启动电流时测试；需要停止自动测试时把 `APP_ENABLE_AUTOMATIC_MOTOR_TEST` 改回 `0`。
- 舵机循环默认关闭，但舵机初始化仍会输出 90 度。
- LCD 色块测试建议在屏幕确认正常之前保持 1，之后可改为 0 缩短启动时间。
- `APP_ENABLE_TASK=1` 时任务优先于自动电机测试并屏蔽舵机扫动测试；但仍建议显式把另外两个测试开关改为 0，避免配置意图含糊。
- 开启自主任务前必须核对 M1-M3 的 `APP_OMNI_Mx_MOTOR_SIGN`、`APP_OMNI_Mx_ENCODER_SIGN`、每 20 ms 最大计数、摄像头/夹爪舵机编号和机械安全角度。新增代码“已编译通过”不代表这些机械参数已经实车验证。

## 8. `.ioc` 配置复核

原配置中不合适、现已修正的项目：

| 项目 | 原问题 | 当前配置 |
| --- | --- | --- |
| TFT RESET/DC | 曾被方案写反，导致白屏 | PB14 RESET，PC5 DC |
| SPI2 NSS | PB12 使用硬件 NSS，不能按命令控制 CS | Software NSS，PB12 GPIO |
| SPI2 时钟/模式 | 原先约 21 MHz，方案又写 Mode 3 | 10.5 MHz、Mode 0 |
| PLLQ | 4 时 PLL48CLK 为 84 MHz | PLLQ=7，PLL48CLK=48 MHz |
| TIM8 舵机 | PSC 曾为 81，频率不等于 50 Hz | PSC=167、ARR=19999 |
| TIM9 电机 | PSC 曾为 0，实际 40 kHz | PSC=1、ARR=4199 |
| TIM10/TIM11 电机 | 曾按舵机参数配置成 50 Hz | PSC=0、ARR=8399 |
| TIM6 中断 | 只配置定时器，没有完整 IRQ | TIM6_DAC NVIC 已开启 |
| M4 编码器 | EXTI 配置后未开启 NVIC | EXTI3 NVIC 已开启 |
| USART1/USART3 RX | 逐字节中断会增加 CPU 开销，且原 `.ioc` 没有 DMA | 两路均改为 64 字节循环 DMA + 空闲线；TIM6 优先级4、DMA优先级5、USART优先级7，保证控制节拍和回绕事件顺序 |
| 编码器空闲输入 | 未接编码器时可能漂浮 | M1-M4 输入启用 Pull-up |
| 代码生成删除 | 重新生成可能删除旧生成文件 | Delete previously generated files 已关闭 |

当前仍需知道的限制：

- USART1 和 USART3 现在同时工作；两路均为 115200、8N1、无流控，并各自拥有独立的 64 字节循环 DMA 接收缓冲；LED_3 视觉旧帧从 USART3 解析。
- M4 使用 x1 软件解码，分辨率与三路硬件 x4 Encoder Mode 不同；因此它只参与板级测试，不参与三轮运动学和速度闭环。
- `.ioc` 只能验证复用、时钟和参数，不能验证实际 TFT 模组引脚顺序、电机电源或 PCB 载流能力。
- 如果再次用 CubeMX Generate Code，检查 `main.c` 用户代码块、`stm32f4xx_it.c` 中断入口和根 `CMakeLists.txt` 的 Main 源文件列表仍然存在，然后重新完整构建。

## 9. CLion 构建

CLion 打开 `WWW` 目录，选择 `Debug` CMake Preset，构建目标 `WWW`。

命令行等价操作：

```text
cmake --preset Debug
cmake --build --preset Debug
```

输出：

```text
build/Debug/WWW.elf
build/Debug/WWW.hex
build/Debug/WWW.bin
```

## 10. Ozone + J-Link 烧录

连接：

```text
J-Link VTref -> 开发板 3V3
J-Link GND   -> GND
J-Link SWDIO -> PA13/SWDIO
J-Link SWCLK -> PA14/SWCLK
```

用 Ozone 打开 `WWW.jdebug`。工程已选择 `STM32F407VE`、SWD、4 MHz，并加载 `build/Debug/WWW.elf`。

开发板 SWD 五针口没有 NRST，普通下载通常可用，但无法可靠 connect-under-reset。如果连接失败：

1. 将 SWD 速度降至 1 MHz。
2. 按住开发板复位键，点击连接后松开。
3. 确认程序没有重新配置 PA13/PA14；当前 `.ioc` 已选择 Serial Wire。
4. 后续改板建议将 NRST 引到调试接口。

## 11. 推荐逐项测试顺序

1. 只接 J-Link 和 TFT，烧录后确认 RGB 色块与文字。
2. 分别接 USART1、USART3 的 USB-TTL，确认两端都能看到相同的 115200 遥测；PA9/PD8 接各自对方 RX，PA10/PD9 接各自对方 TX，并共地。
3. 接编码器但不接电机 12 V，手动转轴，观察 M1-M4 数值和增量。
4. 接舵机 5 V，先只接一只并确认 90 度方向，再逐只增加。
5. 不接电机 12 V，通过 USART3 依次发送被拆分、连续和跨 64 字节边界的视觉帧，确认遥测的 `Vision3` 能稳定显示 `TARGET/NONE/STOP` 与坐标。
6. 抬起全部车轮并清除机械干涉，确认电源可承受四台同时启动，再接电机 12 V；程序会默认以 50% PWM 同步正反转，每个方向 0.5 秒。
7. 按开发板 PA0/S1，确认 LCD 占空比从 50% 逐次增加到 100%，随后回到 50%；核对四路编码器方向。车轮首次着地前应把自动测试开关改回 0。
8. 关闭自动往返测试，悬空车轮，分别给 M1-M3 小目标验证电机与编码器符号；确认 `实际PWM/闭环目标` 没有持续反向发散后，再整定 `APP_MOTOR_MAX_COUNT_20MS`、前馈和 PID。
9. 最后才开启 `APP_ENABLE_TASK`；首次验证三轮运动学与自主状态机仍应悬空车轮并准备断开 12 V，确认 M4 始终为 0。

更完整的引脚和 CubeMX 操作见上一级目录的 `最终PCB方案.md`；电源与打板风险见 `HARDWARE_REVIEW.md`。

## 12. 本次移植思路

1. **固定正确底层**：不替换 F407 的 `main.c`、`.ioc`、GPIO、TIM、DMA、IRQ、LCD、电机、编码器和舵机实现；LED_3 中与这些资源冲突的代码只作为算法参考。
2. **按真实三电机结构适配**：M1-M3 延续 LED_3 的左前、右前、后轮角色并使用三路硬件编码器；M4 只保留电气测试能力，在自主控制中强制停止。
3. **适配当前通信**：不复制 LED_3 的逐字节串口中断回调，而是在 F407 现有 USART3 循环 DMA 回调中按新增区间喂给解析器，保留双串口遥测和错误恢复。
4. **移植算法而非旧底层**：复用三轮逆运动学、目标归一化、前馈和速度 PID，但继续使用 F407 的四路 PWM 驱动；TIM6 保持1 ms基础节拍，并分频得到严格的10 ms编码器采样和20 ms自主控制周期。不移植 PE12-PE15 软件舵机。
5. **安全渐进启用**：新增控制层全部进入正常构建，但自主运动默认关闭；先核对 M1-M3 电机/编码器符号并整定速度环，再验证运动学和视觉协议，最后才允许状态机接管执行器。
