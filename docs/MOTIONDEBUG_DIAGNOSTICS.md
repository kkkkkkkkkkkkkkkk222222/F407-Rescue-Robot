# MotionDebug 串口诊断采集

这版用于定位转角偏差、转圈不结束和串口失响应。仅MotionDebug启用诊断。
当前为MotionDebug专用TURN_PROFILE=2转角控制，NormalRun/Gamepad仍保留原转角实现。
三轮速度PI仍为5/0.12/0、PWM上限60%、慢衰减、编码器/航向极性和MOVE均不变。
参考和修正后角速度都受剩余角度速度包络约束，只有升速受斜坡限制，降速立即夹紧。
实测角速度计算带80ms余量的制动距离；必要时主动制动，退出有速度滞回，避免反复清PI。
普通TURN静摩擦前馈仍70千分比，原速度PI不变。
角度误差进入0.3度后锁定停稳阶段，超过0.5度才退出；角速度不超过3度/秒且三轮
10ms计数均为零，在新鲜样本下确认150ms才DONE。最终内部角度允许误差为0.5度，
不能称为已保证地面0.3度精度。独立REST计时不因角度边界轻微波动反复清零。
超过目标后先停稳，再最多两次不超过12度/秒的回调。
末段8度内持续静止200ms，可使用130千分比起转脉冲，每次最多60ms、每条动作最多3次。
任一轮有反馈、角速度达到2度/秒或角移达到0.15度即撤销，冷却200ms后重新观察。
脉冲期间冻结速度PI积分；停车、失鲜、暂停、超时优先于脉冲。补偿耗尽后仍静止200ms则报错。
这些动力参数是依据当前日志选择的初值，需要地面实测；不是供应商保证值。
累计活动时间仍限制10秒；IMU超过100ms无新样本、反向偏离超过5度、超目标超过8度、
回调次数耗尽均停止并报错，不会通过不断重置目标/计时器继续绕圈。
这不能替代对CPU停滞的诊断，之前的串口卡死问题仍需故障现场记录。
当前固件包含此前已经提交的IMU算法及PI更新，不能把它与旧10月12日15:40固件混为同一版本。

## 使用

烧录 `build/MotionDebug/WWW.elf`，串口115200、8N1、文本发送，勾选发送新行。
启动头应包含 `MOTIONDEBUG SERIAL_V3 RX_GUARD=1 DIAG=1 TURN_PROFILE=2 UART_FIX=1 MODE=2`，
并带编译时间、DIAG_CFG、TURN_CFG和BOOST_CFG。没有TURN_PROFILE=2则不是本次多圈修正版。
保持完整启动输出，单次发送命令，等动作结束后的SETTLE数据输出完，再发下一条。

```text
HELP
TRACE 1
DIAG
TURN 60 60
```

动作期间每250ms自动输出原MOTION/DRIVE和新增DIAG。结束后额外输出两秒SETTLE，
因此能看到宣告DONE之后的实际角度变化。TURN的速度参数仍为度/秒，不是mm/s。
先采集一次 `TURN 60 60` 和一次 `TURN -60 60`，每次结束等至少两秒。
再把正向60度重复六次并量地面最终偏角，每次仍需等待两秒，不能连发。
小角度记录正常后，可分别采集 `TURN 180 60`、`TURN 360 60`，不要使用周期连发。
第二参数是期望巡航速度上限，不是要求全程恒速。暂不使用默认30度/秒做360度测试，
因为仅匀速一圈就需要12秒，超过保留的10秒总超时。
MOVE往返可继续用 `MOVE 1000 600` 和 `MOVE -1000 600`，每次留足结束后的两秒记录。
测试空间不足时缩短MOVE距离。STOP、STAT、GYRO、IMU、MOTOR、ENC、PING仍保留。

## 固定参考多圈测试

普通 `TURN 360 60` 仍是相对当前姿态转360度，不修改原协议语义。
新增串口专用 `CIRCLES signed_count [deg/s=60]`，允许正负1至10圈，速度60至120度/秒。
例如 `CIRCLES 3 60` 自动正向三圈，`CIRCLES -3 60` 自动反向三圈，仍使用MotionDebug配置。
每组首次接受时记录固定原始IMU锚点，第n圈目标是锚点加n个整圈，不再把上一圈残差
当作新基准。本圈可能需要略多于360度纠正上一圈欠转，内部绝对目标入口最多允许368度；
普通TURN的正负360度范围不变。

一组只发一条命令，不使用定时发送。圈间连续静止1秒，最多等待3秒；每圈10秒超时，
整组也有总超时。STOP/HOLD、上位机STOP、串口异常或任一圈失败都会取消整组，
不会自动补跑失败圈。必须明确重新发命令才会建立新锚点。
`CIRCLE EVENT=LAP_DONE/DWELL/RUNNING/DONE/CANCELLED`报告每圈/整组状态；
`CURRENT/COMPLETED/TOTAL`为当前/已完成/总圈数，`TOTAL_MDEG/GOAL_MDEG/ERR_MDEG`
为相对固定锚点的累计角度、计划目标和误差，正负采用电机转向坐标。
原MOTION的PROGRESS仍只报告本圈角度，整组结束才有FINAL，避免多圈超出0x1A字段范围。

先完成正反60度和单圈测试，再做三圈60度/秒地面测试；确认后再增加圈数或速度。
地面标出最初车头方向，每组结束记录实际偏角和CIRCLE最终误差。
固定IMU目标能消除控制目标继承残差，但不能消除IMU自身比例/时基误差。
若日志误差已在0.5度内而实物仍偏很多，再在STOP后用GYRO前后读数配合地面固定角度
做独立正反向标定；不要把转动过程中的20度偏差直接写成传感器补偿系数。

- `DIAG`：立即输出最近一个10ms电机采样周期的诊断快照。
- `TRACE 1`：开启自动扩展诊断，默认为开启。
- `TRACE 0`：关闭自动扩展诊断；原MOTION/DRIVE状态输出仍保留，DIAG/CRASH仍可手动读取。
- `CRASH`：重复打印复位前保存的记录，不制造异常，不驱动车辆。

## 字段

| 输出 | 内容 |
|---|---|
| LOOP | APP_MS为TIM6计时，HAL_MS为SysTick计时；MAIN/TASK/MOTOR为执行次数；观察各自是否继续增加 |
| STAGE | 主循环最近所在阶段：0启动、1主循环入口、2IMU、3控制串口、4UART3、5视觉协议、6LCD、7循环末尾 |
| MAIN_AGE / MAX_MAIN_GAP | 最近主循环入口距电机采样时间及历史最大间隔，单位ms |
| TURN | 目标角、未经360°取模的原始角差、执行状态、减速标志、剩余刹车周期、暂停标志、已用时间及Z角速度 |
| TURN_CTL | 新转角控制阶段、带符号误差、参考/修正后角速度、停稳确认时间、回调次数、退出原因和本次控制器记录的样本年龄 |
| CIRCLE | 固定锚点多圈测试的进度与累计残差，仅CIRCLES产生 |
| PI | 各轮误差、比例项、积分项、前馈、PID修正及最终PWM，用来判断减速时输出为什么改变 |
| DRIVE | 同一采样周期的目标/实际速度及原始、极性修正后的10ms编码器计数 |
| IMU | 就绪、采样次数、样本年龄、累计角度、采样间隔、间隔异常数、读错误及Z零偏 |

`MDEG`除以1000为度，`MDPS`除以1000为度/秒。速度为mm/s。
`ERR_C100`是编码器计数误差乘100，`P100/I100/CORR100`是PWM千分比单位再乘100。
`FF/PWM`直接为带方向的千分比，例如150表示15%。ACTIVE位0/1/2分别对应三轮，
该位为0时本周期没有运行该轮PI，不能把零输出当成一次正常PI计算。
TURN的MOTOR_STATE：0空闲、1运行、2完成、3故障、4参数无效。
TURN字段在MOVE时可能仍保留上一次转动目标，需结合CMD和MOTOR_STATE判断。
TURN_CTL阶段PHASE：0空闲/取消、1正常驱动、2制动、3到点停稳确认、4低速回调、5完成、6故障。
FAIL：0无、1IMU失鲜、2总超时、3超调超过8度、4朝错误方向偏离5度、5回调次数/范围限制、6IMU未就绪、7起转补偿耗尽仍不动。
REST_MS是独立的低角速/零轮速确认时间；SETTLE_MS是进入角度停稳区后的确认时间。
BRAKE_PRED_MDEG是根据实测角速度估计的停止距离，不是已转角。
BOOST/BOOST_MS/BOOSTS记录起转补偿活动、持续时间和次数。BOOST_EXIT：0未退出，
1检测到运动、2角移限额、3时间限额、4进入制动/停稳、5取消或故障。
补偿期间ACTIVE=0表示速度PI暂停更新，PWM仍可能为正负130，不应误判为控制失效。
TURN_CTL的IMU_AGE_MS只在转角运行时有意义，结束后归零；实时采样龄仍看DIAG IMU AGE_MS。
ERR_MDEG为目标减已转角，REF_MDPS是加速度受限的参考角速度，CMD_MDPS是角速度反馈修正后的指令。
正负均采用当前已标定的电机转向坐标；RAW_DELTA仍是原始IMU坐标，不改原有上位机串口单位。
停稳期间会持续短接制动，可能看到BRAKE=0、PWM=0，这不表示电机处于滑行。
PROGRESS现在保留实际角度，不再在DONE时强写目标值；DONE时LEFT仍按原协议为0，
具体残差看ERR_MDEG和DYAW_MDEG。新指令QUEUED后的首个10ms快照可能还是上一指令，需核对SEQ。
APP_MS与HAL_MS起点不同，应比较增量，不能要求两者绝对数值相等。

## 再次卡住时

先使电机停止。若能保持MCU逻辑供电并单独停止电机，按板上复位后读取自动PREV输出，
再发送 `CRASH` 和 `PING`。不要重新烧录来代替复位，烧录器可能覆盖RAM里的记录。
断电或欠压后RAM保留不可靠，此时PREV可能无效；以停车为先。

捕获到NMI/HardFault/MemManage/BusFault/UsageFault时，诊断处理先直接把六路电机输入
拉低，再保存异常号、PC、LR、xPSR、原始SP、EXC_RETURN及SCB故障寄存器，并等待复位。
异常处理使用独立栈；遇到压栈错误或无效地址时不读取异常栈，FRAME_VALID=0。
Error_Handler记录EXC=256，PC是调用者返回地址，不是硬件异常帧。

记录位于启动代码不清零的RAM区域，第一次上电PREV VALID=0是正常的。
PREV FAULT=0不等于没有卡死：关中断死循环、ISR阻塞和调试器停核不一定触发异常，
这时只能利用复位前最后的任务阶段、计时和电机快照继续定位。
普通复位也会产生PREV记录，必须结合你实际观察到的故障来判断。
这不是看门狗，不会自动复位；也不能保证所有形式的CPU停滞都能自动关闭PWM。

## 发回的数据

请保留完整启动头、RXCMD/ACCEPT、MOTION/DIAG、FINAL及最后两秒SETTLE。
如果卡住，再补上复位后的全部PREV/CRASH输出，并说明是小车物理过转、没有停止，
还是串口失去响应。还请注明实际转向和地面观察到的最终偏角。
诊断本身无法替代地面标记或外部角度基准，单凭同一陀螺仪不能证明绝对角度准确。

没有自动烧录或自动运动。用于解析异常PC的配套ELF会在提交后保留一份带提交号的副本。

## 本次设计依据

本次六次60度日志中，报DONE时仍有77至92度/秒角速度，最终IMU累计389.395度。
修复优先针对控制结束过早和末段超速，不用改传感器比例来掩盖已测出的过转。
采用已有C控制器接入运动约束和角度/速度双容差，没有移植WPILib框架或其示例增益。
参考官方资料：
- [WPILib运动轨迹约束](https://docs.wpilib.org/en/stable/docs/software/advanced-controls/controllers/trapezoidal-profiles.html)
- [WPILib位置/速度双容差](https://docs.wpilib.org/en/stable/docs/software/advanced-controls/controllers/pidcontroller.html)
- [WPILib前馈与实测静摩擦参数](https://docs.wpilib.org/en/stable/docs/software/advanced-controls/controllers/feedforward.html)

## 2026-10-14 串口失响应修复（36ceb53历史实机记录）

在未复位前通过Horco CMSIS-DAP读取到真实HardFault现场，旧固件为a4f6645：
- 异常PC `0x08004B88` 对应HAL `UART_Transmit_IT()`读取发送缓冲区。
- 异常栈r0 `0x20000474` 是huart3，xPSR `0x01000037` 表明异常发生在USART3中断。
- `CFSR=0x00008200`、`HFSR=0x40000000`、`BFAR=0x20020000`。
- huart3发送指针已到 `0x20020001`，原包长15字节，剩余计数却为 `0xFEAA`；错误码4为FE。

本地旧HAL的 `UART_DMAAbortOnError()`在RX DMA出错后同时清零TX计数。
TXE仍继续触发，计数从0下溢并越界读内存，最终触发HardFault。
MotionDebug异常入口会关闭电机，因此表现为MOVE中途停车、程序器串口后续指令无响应。
这不是速度PID、距离比例或正常完成后的指令锁问题；USART3即使没有上位机指令也会发送遥测。

修复只移植ST官方源码中RX错误不清TX的改动，没有整体替换HAL库。
另加TXE零计数保护、串口启动短临界区、RX错误锁存和主循环恢复，
并将USART3与其DMA设为相同抢占优先级，避免HAL状态和协议解析重入。
`PING`增加UART3错误、重启、丢包和剩余发送计数。电机/编码器参数、PID和0x19/0x1A协议不改。
底层UART修复也编译进NormalRun和Gamepad，本次实际烧录的是MotionDebug。

官方依据：[ST当前F4 HAL UART源码](https://github.com/STMicroelectronics/stm32f4xx-hal-driver/blob/master/Src/stm32f4xx_hal_uart.c)
中的 `UART_DMAAbortOnError()`只清Rx计数；
[ST UART中断使用说明](https://wiki.st.com/stm32mcu/wiki/Getting_started_with_UART)说明接收完成后需要重新挂接接收。

实际测试记录：
- 三种配置编译通过；MotionDebug通过SWD烧录并校验成功，启动标识UART_FIX=1。
- COM18连续20次PING通过；10组DIAG紧跟PING通过；CR/LF/CRLF三种结束符通过。
- 故意输入非ASCII字节、残缺指令会安全停车，随后正确PING仍可继续，无需复位。
- 架空依次执行 `MOVE 200 200`、`MOVE -200 200`、`MOVE 1000 500`、`MOVE -1000 500`，
  四次均收到FINAL DONE，且每次结束后PING正常。最后显式STOP并关闭端口。
- UART3实际出现多次FE并自动恢复；最终运动测试统计为23次错误、24次接收启动，无死机。
- USB重新连接后一度无回传；恢复后进行了上述成功测试。一轮启动时IMU校准失败，
  未绕过IMU检查，保持静止复位后READY=1才执行运动测试。

日志位于 `build/MotionDebug/serial-uart-fix-smoke.log` 和 `serial-uart-fix-motion.log`。
这验证串口连续执行、状态释放和编码器完成流程；架空测试不代表落地距离或航向精度已验证。
UART3错误计数仍可能增加，接收线路/共地/未连接时悬空等硬件因素仍需按实际接线排查。
