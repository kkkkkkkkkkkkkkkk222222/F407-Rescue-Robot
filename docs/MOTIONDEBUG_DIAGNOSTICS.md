# MotionDebug 串口诊断采集

这版用于定位转角偏差、转圈不结束和串口失响应。仅MotionDebug启用诊断。
当前增加MotionDebug专用TURN_PROFILE=1转角控制，NormalRun/Gamepad仍保留原转角实现。
三轮速度PI仍为5/0.12/0、PWM上限60%、慢衰减、编码器/航向极性和MOVE均不变。
转角不再在剩余30度时清PI或切固定120mm/s，而是按剩余角度连续降速、限制加速度，
并用实测Z角速度调整转速指令。仅TURN静摩擦前馈从100降到70千分比，属于待实测的初值。
到点持续制动，误差不超过1度、角速度不超过3度/秒且三轮10ms计数均为零，
在新鲜样本下连续确认150ms才DONE。超过目标后先停稳，再最多两次不超过12度/秒的回调。
累计活动时间仍限制10秒；IMU超过100ms无新样本、反向偏离超过5度、超目标超过8度、
回调次数耗尽均停止并报错，不会通过不断重置目标/计时器继续绕圈。
这不能替代对CPU停滞的诊断，之前的串口卡死问题仍需故障现场记录。
当前固件包含此前已经提交的IMU算法及PI更新，不能把它与旧10月12日15:40固件混为同一版本。

## 使用

烧录 `build/MotionDebug/WWW.elf`，串口115200、8N1、文本发送，勾选发送新行。
启动头应包含 `MOTIONDEBUG SERIAL_V3 RX_GUARD=1 DIAG=1 TURN_PROFILE=1 MODE=2`，
并带编译时间、DIAG_CFG和TURN_CFG。没有TURN_PROFILE=1则不是本次转角修正版。
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
FAIL：0无、1IMU失鲜、2总超时、3超调超过8度、4朝错误方向偏离5度、5回调次数/范围限制、6IMU未就绪。
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
