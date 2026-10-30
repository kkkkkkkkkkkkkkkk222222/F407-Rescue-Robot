# 当前已完成的 F405 NormalDebug 测试接口合同

导出日期：2026-10-30。本文是**现有实现清单**，不是新增协议方案。
文档修订号为1；此修订号**没有在串口上传输**，不能当成固件协议版本。
本轮只导出文档、机器可读清单与编码示例，不修改固件、正式比赛路径、上位机代码，
不连接串口、不烧录、不执行实车动作。

配套文件：[机器可读合同](F405_NORMALDEBUG_CONTRACT_20261030.json)、
[原始编码示例](F405_NORMALDEBUG_EXAMPLES_20261030.json)。

## 1. 对应版本、构建与运行身份

| 项目 | 当前源码事实 |
| --- | --- |
| 下位机仓库 | https://github.com/kkkkkkkkkkkkkkkk222222/F407-Rescue-Robot |
| 分支 | main |
| 本合同冻结的固件实现提交 | `14157e6bf2600ff0480b133ab763298e12ce400b` |
| 导出时远端main | 已用git ls-remote核对，与上述提交一致 |
| 板型 | F405_ROBO工训ROBO板，STM32F405RGT6，168MHz，无LCD |
| 运动测试预设 | NormalDebug；输出build/NormalDebug/WWW.elf |
| 必要CMake开关 | APP_BOARD=F405_ROBO；APP_ACTIVE_MODE_OVERRIDE=APP_MODE_NORMAL_DEBUG；APP_NORMALDEBUG_MOTION_TEST=ON |
| 展开后的关键开关 | APP_ACTIVE_MODE=5；APP_ENABLE_MOTION_DEBUG_TASK=1；APP_ENABLE_TASK=0 |
| 编译类型 | Debug |
| 比赛任务预设 | NormalDebugTask，同为MODE=5，但APP_NORMALDEBUG_MOTION_TEST=OFF，APP_ENABLE_TASK=1 |
| 旧板预设 | MotionDebug对应旧F407，不能拿它的ELF烧新F405板 |

构建命令仅供查阅，本轮没有执行：

```text
cmake --preset NormalDebug
cmake --build --preset NormalDebug
```

运动测试服务的程序器串口身份是`BOARD=F405_ROBO SERVICE=MOTIONDEBUG MODE=5`。
比赛入口是`SERVICE=RESCUE_TASK SHARED=NormalRun`；**MODE=5本身不能区分两者**。
普通二进制遥测目前不能返回服务名、板型或完整固件提交号。

导出开始时已有未提交内容：Main/Src/Task.c仅增加两行空白，.idea/vcs.xml为IDE仓库映射，
tmp/为未跟踪的参考仓库和本地资料。它们未纳入本轮提交；没有尚未推送的功能实现需要补丁。
本轮发布合同会新增一个纯文档提交，不代表板上固件身份发生变化。

### 历史烧录证据

2026-10-29最后一次成功烧录对应上述14157e6源码，OpenOCD返回`Verified OK`，
随后STAT返回运动测试身份及`BUILT=Oct 29 2026 20:48:06`。
历史日志：`build/imu_100hz_flash_20261029_214719.log`，原文件位于本地生成目录。
原始文本副本随本合同导出为[F405_NORMALDEBUG_FLASH_EVIDENCE_20261029.log](F405_NORMALDEBUG_FLASH_EVIDENCE_20261029.log)，
回执节选见本文第8节。

当时5.639秒内ODOM_TX、IMU_TX各增加564，证明主控UART发送完成速率约100Hz；
不证明RDK收到、解析或融合成功。**本轮没有重新读取板子，不能保证此后未被外部重新烧录。**
固件未嵌入Git提交号或镜像哈希，历史构建与烧录记录不是板端密码学身份认证。
当前本地ELF的SHA256为`2719D001745A3DA60AE6A3C4C0343B05671FCD730ECE5EF98C6E41DAB0589DE8`；
这不是本轮从板上读回的哈希。

RDK侧工作目录按用户提供为`/home/sunrise/RDK_X5/shijue_fangan`，
基线为`codex/gamepad-teleop@7e25a8104794e36af36167a638a5cc7d0a4e68f7`。
本轮只读取其Git副本核对源码，没有登录RDK，也未修改该工作目录或正式比赛路径。

## 2. 能力查询：UART4未实现

| 要求 | 现有情况 |
| --- | --- |
| 二进制能力查询request TYPE/opcode、response TYPE | **未实现**，不分配编号 |
| 线上协议版本、功能位 | **未实现** |
| 线上板型、服务模式、固件提交/构建ID | **未实现** |
| 查询nonce、boot/session epoch | **未实现** |
| 二进制查询的超时或UNSUPPORTED响应 | **未实现**；没有关联查询的否定回执 |
| 不驱动车辆的现有替代入口 | 程序器USART1文本VER、STAT、PING、HELP；不是RDK UART4接口 |

USART1位于PB6 TX/PB7 RX，程序器端历史为COM18，115200 8N1。
文本以CR、LF或CRLF结束，没有TYPE、opcode或CRC。VER/STAT/HELP可读版本文本，
PING返回运行毫秒和串口计数。SERIAL_V3是文本服务标识，不是二进制协议协商版本；
DIAG的FORMAT=10是诊断记录格式，也不是线上遥测版本。
这些查询不启动、停止或重置运动；**HOLD、STOP、RESET都不能冒充无副作用的能力查询**。
上述结论只针对完整合法文本；文本接收溢出、错误行或1秒残缺行超时可能触发现有
程序器接收保护并停车。RDK不能向UART4试发这些文本来“探测能力”。

UART4未知TYPE、未知opcode、非法字段、CRC错误通常被丢弃，不为该请求返回专用错误帧。
原动作的周期状态可能仍继续到达。上位机不能把任意状态帧当成查询成功或新动作ACK。
文本超时由客户端处理；当前没有nonce供客户端严密关联一次文本查询。

## 3. 现有控制接口

### 3.1 统一传输格式

新板RDK接口：JP506，主控TX=PC10、RX=PC11，UART4，115200 8N1，交叉连接并共地。
内部历史句柄huart3在这块板上实际绑定UART4，不是旧USART3。

| 帧字节偏移 | 内容 |
| --- | --- |
| 0、1 | A3、B3 |
| 2 | TYPE |
| 3 | 此TYPE的uint8 SEQ |
| 4..11 | 8字节payload，记作P0..P7 |
| 12、13 | CRC低字节、高字节 |
| 14 | C3 |

CRC为CRC-16/MODBUS，初值0xFFFF，多项式0xA001，覆盖偏移2..11共10字节。
payload多字节数值均大端；**只有CRC是低字节先传**。

运动请求TYPE=0x19，payload固定为：

| 字段 | 类型 |
| --- | --- |
| P0 | opcode，uint8 |
| P1 | flags，uint8 |
| P2..P3 | arg1，int16 BE |
| P4..P5 | arg2，int16 BE |
| P6..P7 | speed，uint16 BE |

flags定义：VALID=0x01、KEEP_HEADING=0x02、FIELD_FRAME=0x04、ACK_REQUIRED=0x08、
CLEAR_FAULT=0x10。不是所有命令都允许所有位；推荐下面明确列出的组合。

### 3.2 应用层前进、左移、逆时针为正

| 应用接口 | 线上请求及参数 |
| --- | --- |
| move_body(forward_mm,left_mm,speed_mm_s) | TYPE=0x19，opcode=3，flags必须0x0B；arg1=前向mm，arg2=左向mm；向量长度必须>0且<=10000mm；speed=50..700mm/s或0 |
| turn_ccw(angle_deg,rate_deg_s) | TYPE=0x19，opcode=2，flags推荐0x09；arg1=-round(angle_deg*10)，arg2=0；speed=round(rate_deg_s*10)，单位0.1度/秒；speed线上允许1..3600或0 |
| move_spin_angle | **UART4未实现**，没有opcode或配对帧；不能用其他动作拼接后宣称同步组合动作 |
| stop | TYPE=0x19，opcode=0，flags推荐0x09，arg1=arg2=speed=0；必须使用新SEQ |
| reset_reference | TYPE=0x19，opcode=5，flags推荐0x09，arg1=arg2=speed=0 |
| HOLD | opcode=1，flags推荐0x09，其余为0；取消当前动作、停止并返回DONE，不是暂停 |
| MOVE_FIELD | opcode=4，flags必须0x0D；arg1/arg2是本地Location场地坐标轴上的相对位移mm，不是绝对目标坐标 |

move_body的speed=0：纯前后使用500mm/s，有横向分量时使用300mm/s。
正speed是平移向量速率，不是每只轮子的速度，也不保证地面实速等于指令值。
BODY向量在动作开始时转换到地面固定方向，随后按当前航向反算车体系目标；
不是随着车头偏转而持续旋转的路径，也不是主动边走边转指定角度。

turn_ccw的应用层正向是逆时针，但**旧线上TURN_REL正arg1实际为顺时针**，
因此只在上位机运动协议适配层对转角反号，不能对gyro遥测反号。
有效执行角度为-360..360度且非0；解析器虽接受arg1到+/-32767个0.1度，
底层Motor_TurnAngleSpeed只接受+/-360度，超出后可能先RUNNING再以INVALID结束。
客户端应在编码前限制到+/-360度。低于或等于约1度可能不发生可见转动。
turn speed=0采用250mm/s轮缘默认速度，按R=130mm约110.18度/秒，不是默认60度/秒。
配置速度只是上限，实际有加减速、末端处理和静摩擦影响。

### 3.3 完成、停止、重置和重复请求

纯前后BODY复用Motor_MoveDistance：编码器前两轮差值判距、IMU保航向，
距离阈值3mm；斜向/侧向BODY及FIELD按Location沿目标方向的投影判距，阈值10mm。
后者DONE时进度会设置成请求总距离，不能据此推断实际地面移动恰好等于请求距离。
MOVE完成会触发制动，但DONE不等于已等待车身彻底静止。
相对TURN对未回绕的原始IMU角度差判角；约1度容差、3度/秒低角速度及轮速为零，
再稳定约150ms后DONE，现有末端修正最多3次；不保证独立地面量角精度。

请求通过后0x1A通常会先报RUNNING，再报DONE/FAULT；非常短的动作可能很快完成。
STOP在下一次调试任务处理时抢占，周期20ms，另外还有接收/调度延迟和机械制动时间，
不存在“收最后一字节就物理立即静止”的保证。STOP不改变舵机保持角度。

reset_reference先取消运动，重建Location的x=0、y=0、heading=0、path=0，
设置start_zone=UNKNOWN并开启跟踪，以当前原始IMU角度作为后续增量基准；
IMU已恢复且就绪时还会确认/清除heading_uncertain。它**不**执行IMU_ZeroYaw，
不清原始编码器累计计数，不归零0x1F累计yaw，不重置传感器sample_count或遥测SEQ。
CLEAR_FAULT位允许随RESET出现，但此分支没有独立的清故障动作，不能承诺硬件故障被清除。
RESET不会令0x15重新发启动COUNTER_RESET位，也没有参考epoch回执。
二进制RESET会停止当前动作后立即建参考，不等待机械完全静止；客户端不能把其DONE
解释成“车身已静止并完成高精度重标定”。

相同SEQ仅与最近一次收到的motion SEQ比较：相同就忽略，内容变化也忽略；
不同SEQ的新合法命令可停止并替代旧动作，不保证旧动作额外收到CANCELLED回执。
旧的、不同SEQ的迟到请求也可能被当作新命令执行；没有序号窗口或单调action ID。
USART1测试动作占用控制权时，UART4非STOP请求可能被消费但未启动，没有专用BUSY回执。
因此客户端应独占控制、一次一个动作，避免与程序器同时发动作；回执必须匹配命令身份。
接收端保存的是最新motion快照，不是动作队列；20ms处理前连续送入多个新命令可能只执行
最后一个。STOP优先针对调试任务实际看到的STOP，并非独立永久锁存；发STOP后应先等
匹配STOPPED回执，再提交后续动作。周期0x1A不受ACK_REQUIRED是否置位而关闭。

暂停/续跑的NormalDebug线上接口：**未实现**。正式比赛0x18的PAUSE不属于本合同。
断开UART4不会自动触发NormalDebug的150ms手柄看门狗；已接收的有限动作继续至结束或超时。
TURN保护超时为10秒；MOVE超时为clamp(4*distance/speed*1000+5000,5000,120000)毫秒，
按动作实际选定的请求速度计算。超时报告0x1A STATE=3；内部TURN失败也可能映射为MOTOR，
不能把所有FAULT=5都解释为物理电机损坏。

### 3.4 组合动作只有内部C API，未暴露线上合同

源码存在Motor_MoveSpinAngle(speed_mm_s,direction_deg,turn_deg,max_rate_dps)，
但Debug.c和0x19解析器均没有入口调用它，程序器文本命令也没有组合动作入口。
所以本节仅说明内部代码，**不能当作RDK可调用能力**：

- direction_deg以动作起始车头为0度、初始左侧为90度；锁存到地面固定直线，
  后续随航向变化反算车体系向量，并用Location横向误差修正该直线。
- turn_deg是逆时针正；用原始累计IMU yaw差乘APP_LOCATION_IMU_YAW_SIGN计算，
  不采用[-180,180]最短角差，支持一圈。
- 完成取决于转角误差<=1度并静止稳定150ms；**没有平移距离终点条件**。
  剩余角度不足30度开始减小平移，不足10度或越过转角时停止平移。
- C参数平移速度绝对值<=500mm/s；turn_deg绝对值<=360度；角速度20..90度/秒。
  平移方向和转角可独立给定，但没有将这四个参数传输上去的线上编码。
- 一旦开始，重复C调用返回已有状态，参数锁存到Motor_Stop后才重新接受；
  最长10秒，IMU无新样本100ms等条件可结束为FAULT。
- 线上配对请求、nonce、动作ACK、组合暂停/续跑均未实现。
  通用Motor_PauseFiniteMotion不提供保存并恢复spin_move的合同，不能承诺组合动作可续跑。

## 4. 已有遥测与实测车体系速度的缺口

### 4.1 0x15，约100Hz，三轮原始累计计数

| payload | 类型/单位 | 含义 |
| --- | --- | --- |
| P0..P1 | uint16 BE，count | W1右前原始累计计数低16位 |
| P2..P3 | uint16 BE，count | W2左前原始累计计数低16位 |
| P4..P5 | uint16 BE，count | W3后轮原始累计计数低16位 |
| P6 | uint8，ms | 固定10，标称编码器采样周期，不是采样tick或本包与上包的实际间隔 |
| P7 | uint8 | 当前bits0..2固定置1，启动后首次排队帧额外置bit3 |

这些数值未应用encoder_sign，回绕不是无效值；上位机用int16模差求相邻增量。
三个轮计数来自同一次Encoder_GetAll的原子拷贝，底层10ms采样。
bit3不是可靠boot epoch，首次帧可能丢失，且重设Location参考不会再置此位。
当前发送逻辑不生成bit4编码器故障标志，bits0..2也不能作为实时独立健康检测证据。
没有filter/version/tick/饱和标志；累计值只截低16位，不是饱和截限。

### 4.2 0x1F，约100Hz，保持当前公开合同

| payload | 类型/单位 | 含义 |
| --- | --- | --- |
| P0..P3 | int32 BE，0.001度 | 本地参考下的累计yaw，可负、可超过360度 |
| P4..P5 | int16 BE，0.1度/秒 | Z轴角速度，向零取整编码 |
| P6 | uint8，ms | 打包时最新角度、角速度两者age的较大值，超过255取255 |
| P7 | uint8 | VALID=01、READY=02、RECOVERING=04、UNCERTAIN=08、STALE=10、YAW_CLAMPED=20、RATE_CLAMPED=40；bit7=0 |

上表标志值是十六进制。正常status=03；只有READY且无其他异常标志才置VALID。
角度或角速度age>=150ms、或尚无角度样本会置STALE；传感器异常时仍继续发送帧。
yaw超int32范围时饱和到上下界并置YAW_CLAMPED；rate超int16范围时同理。
age=255不是正常新样本，0也不代表绝对零延迟；无效由flags判断，不用yaw=0作为哨兵。

yaw和rate来自一次IMU_GetData的原子结构快照，但模块分开传0xBB角度和0xAA角速度，
因此**不是同一传感器采样时刻**。age没有分别给出两者时间。
yaw为模块角度解回绕后的本地累计量，rate为最近模块反馈，不是目标轮速或指令角速度。
F405不在遥测路径新增滤波；模块内部滤波及其精确延迟未由当前合同标定。
传感器为1kHz，遥测为100Hz的最新值抽取，**不是10ms角速度均值**。

### 4.3 0x1A，约20Hz及状态变化时立即排队

| payload | 类型/单位 | 含义 |
| --- | --- | --- |
| P0 | uint8 | 被应答运动请求的SEQ，不是状态帧自身SEQ |
| P1 | uint8 | 0 IDLE、1 RUNNING、2 DONE、3 FAULT、4 STOPPED |
| P2 | uint8 | 0 NONE、1 INVALID、2 IMU、3 ODOM、4 TIMEOUT、5 MOTOR |
| P3 | uint8 | 当前运动opcode |
| P4..P5 | int16 BE | MOVE为有符号毫米标量进度；TURN为线上方向的0.1度进度 |
| P6..P7 | uint16 BE，0.01度 | Location航向0..35999；Location无效也返回0，无法区分合法0度 |

MOVE进度不是XY坐标；F<0或F=0且L<0时进度为负，其他方向为正。
TURN回执进度要按turn_ccw适配再次反号，gyro则不能反号。
状态帧没有remaining、运行服务名、控制来源、健康位、tick或epoch。
头文件中的MOTION_STATUS_IMU_READY等常量和旧注释不等于它们已写入此帧，
实际打包P6..P7明确为heading，不得再按健康位解读。

### 4.4 版本化实测forward/left/yaw-rate帧：未实现

没有这种TYPE或线上version字段，不能提供未实现帧的十六进制示例。
当前UART4没有直接上报下位机复算的车体系forward/left速度，
所以RDK界面的“下位机实测车速”该两项应显示未提供，而不是目标值、0或上位机复算值。
yaw-rate可从0x1F取得真实模块反馈，但不与某个编码器快照强制对齐。

内部Motor_GetStatus/程序器MLOG有各轮ACTUAL_MM_S，为10ms编码器增量均速，
反馈符号修正后按整数mm/s四舍五入，1count/10ms约20.72158mm/s。
它不是指令TARGET，也不是地面无滑移的独立测速。控制内部另有侧移EMA(alpha=0.25)
用于跟随判定，但MLOG ACTUAL不是该滤波值；不能给未上报车速指定滤波延迟。
MLOG的motor、RAW32、IMU、Location分别读取，MS为打印时HAL tick，整行不保证同一采样快照。
RDK若要严格显示“下位机实测、原始计数、上位机复算”三者，第一项的forward/left
目前缺少线上来源；本轮不为补齐界面而临时新增遥测。

## 5. 新F405运动学与符号

应用统一采用前进F、左移L、逆时针omega为正。

| 逻辑轮 | 插座/解码 | 原始count变为反馈的符号 |
| --- | --- | --- |
| W1右前 | JP303，PA8/PA9，TIM1 TI12 | -1 |
| W2左前 | JP301，PC6/PC7，TIM8 TI12 | -1 |
| W3后轮 | JP302，PC8/PC9，EXTI双边沿 | -1 |

原始计数正方向按A/TI1上升且B/TI2为低时计数加一；后轮软件状态表采用同一约定。
它不是车体前向。按现有电机极性，单轮正驱动通常产生负raw和正反馈，不能用
“每个轮子逆时针外观一样”替代A/B方向定义。

实际宏：M1/M2/M3_MOTOR_SIGN均+1；M1/M2/M3_ENCODER_SIGN均-1；
APP_OMNI_FORWARD_SIGN=-1；APP_OMNI_LATERAL_API_SIGN=+1；
APP_LOCATION_IMU_YAW_SIGN=+1；APP_MOTOR_HEADING_OUTPUT_SIGN=-1；
APP_OMNI_FIELD_YAW_TO_DRIVE_SIGN=-1；APP_IMU_UART_YAW_SIGN=+1。
counts/rev=20.409*13*4=1061.268，轮径70mm，K=pi*70/1061.268=0.2072157888mm/count。

设原始增量为dn_i，实际覆盖区间为dt秒；先且仅先应用一次反馈符号：

```text
dn_i = int16(new_raw_low16_i - old_raw_low16_i)  // 按65536取模后转有符号
u_i  = -K * dn_i / dt                         // 正向反馈轮缘速度mm/s
F    = (u2-u1) / sqrt(3)
L    = (2*u3-u1-u2) / 3
```

等价的直接raw公式：F=K*(dn1-dn2)/(sqrt(3)*dt)，
L=K*(dn1+dn2-2*dn3)/(3*dt)。既然用了这两个直接公式，就不要再把dn反号一次。

混轮目标形式为u1=-sqrt(3)/2*F-L/2+q，u2=+sqrt(3)/2*F-L/2+q，u3=L+q。
q为轮系公共切向速度，F405逆时针正的模型关系为q=-R*omega，R配置为130mm。
如果仅用轮计数做角速度对照，omega_enc=-(u1+u2+u3)/(3*130) rad/s，再乘180/pi得到度/秒。
R=130mm是软件转角换算参数，不是本轮标定得到的有效地面旋转半径；
正式的gyro实测yaw-rate来自0x1F，不能把该轮系模型估计冒称陀螺仪反馈。

| 理想方向 | 反馈轮速(u1,u2,u3) | 原始计数增量符号(dn1,dn2,dn3) |
| --- | --- | --- |
| 前进 | (-,+,0) | (+,-,0) |
| 左移 | (-,-,+)，理想比例-1:-1:2 | (+,+,-) |
| 逆时针 | (-,-,-) | (+,+,+) |

以上是理想纯方向示例，航向纠偏、死区、加减速和打滑时不要求每一采样都满足理想符号。
Location用上式恢复平移，再用IMU相邻航向的中值旋转到场地坐标；它没有用三轮之和
代替陀螺仪航向。上位机复算应使用新轮序，不能套旧板轮序公式。

## 6. 时间、epoch、动作身份及限制

| 项目 | 当前实现 |
| --- | --- |
| 0x15真实采样tick | 无；P6=10只是标称采样周期 |
| 0x1F真实采样tick/sample_count | 无；只有最大age，uint8毫秒，255饱和 |
| 0x1A采样或发送tick | 无 |
| boot/session epoch | 无线上字段；Vision内部session_generation不等于可查询boot身份 |
| reference-reset epoch | 无；RESET不能从0x15/0x1F的新epoch辨识 |
| action_id/nonce | 无；只有运动请求uint8 SEQ及opcode |
| SEQ规则 | 0x15、0x1F、0x1A发送计数分别独立，256回绕；Vision_Init归零，RESET_ODOM不归零 |
| 状态关联 | 0x1A.P0=请求SEQ，P3=opcode；外层状态SEQ是另外的计数 |
| 允许配对的任务帧 | 正式比赛0x1D/0x18请求、0x1E/0x17状态，同SEQ按各自既有上下文规则；不扩展到本测试接口 |

0x15和0x1F虽然在同一轮主循环附近排队，仍然独立读取、独立发帧，**不能按相等SEQ配对**。
IMU和编码器采样不保证同一tick；丢帧或主循环合并多次release后，0x15的两个累计计数
可能覆盖多个10ms区间，但P6仍为10。没有真实tick，不能保证乘SEQ差就还原真实时间。
超过半个16位计数范围的增量存在方向歧义，长时间断流后应重新建立差分基准。

RDK应记录本机单调接收时间，用age和接收超时判断新鲜度；接收时间只是近似依据，
不等于下位机采样时间，队列和串行传输延迟不包含在P6中。
同一0x1F的yaw/rate结构快照一致，但来自不同模块消息；整份0x1A各字段也不是与
0x15或0x1F严格同步的原子测量记录。

客户端可用“请求后到达、P0匹配SEQ、P3匹配opcode、合理状态转移”降低误认概率，
并串行提交动作、保持短时间不复用SEQ。但没有boot epoch或更大action ID，
**无法严格排除序号回绕、主控重启前的迟到回执，或另一控制来源恰好使用相同SEQ**。
外层0x1A SEQ的前后关系也不能替代动作epoch。不要在自动重连时盲目重放未决运动。

## 7. 0x1F兼容性声明

与用户给定的公开合同完全一致：yaw=int32大端毫度，rate=int16大端0.1度/秒，
后接age/status；二者均为逆时针正；0x15与0x1F的SEQ独立。
本轮不改变字段、标志或150ms过期规则，不反转gyro。
曾有50Hz草案和yaw-only草案，但当前冻结实现是14157e6，二者同帧100Hz；
不能用历史草案覆盖当前合同。

## 8. 原始示例、证据及检查范围

以下为**离线构造的编码示例，不是抓到的实车原始包，本轮没有发送这些字节**。
所有二进制示例均已离线核对长度与CRC；完整字段见配套EXAMPLES JSON。

| 示例 | 原始十六进制及解释 |
| --- | --- |
| 前进500mm，400mm/s | `A3 B3 19 31 03 0B 01 F4 00 00 01 90 B3 01 C3`；SEQ=31，opcode=3，F=01F4=500，L=0，speed=0190=400 |
| 左移300mm，400mm/s | `A3 B3 19 32 03 0B 00 00 01 2C 01 90 D7 C0 C3`；SEQ=32，F=0，L=012C=300 |
| 应用逆时针90度，60度/秒 | `A3 B3 19 33 02 09 FC 7C 00 00 02 58 BC F9 C3`；arg1=FC7C=-900，speed=0258=600个0.1度/秒 |
| STOP | `A3 B3 19 34 00 09 00 00 00 00 00 00 5E 7C C3`；新SEQ=34，opcode=0，其余参数0 |
| reset_reference | `A3 B3 19 35 05 09 00 00 00 00 00 00 93 D3 C3`；新SEQ=35，opcode=5 |
| 三轮原始计数 | `A3 B3 15 80 00 0A FF F6 00 00 0A 07 31 3C C3`；原始低16位10、65526、0；dt=10、status=07；若前帧全0，模差为+10,-10,0，属于前进符号 |
| 负多圈yaw和负角速度 | `A3 B3 1F 07 FF F5 03 05 FB 50 01 03 31 0E C3`；yaw=-720123毫度，rate=-1200即-120度/秒，age=1ms、status=03 |
| 正yaw/逆时针角速度 | `A3 B3 1F 08 00 01 5F 90 01 2C 01 03 7B B4 C3`；yaw=90度、rate=30度/秒；SEQ与编码器无配对含义 |
| TURN完成回执 | `A3 B3 1A 91 33 02 00 02 FC 7C 30 39 CC BF C3`；帧SEQ=91，应答请求33，DONE、fault0、opcode2、线上进度-900、Location航向123.45度 |
| STOP回执 | `A3 B3 1A 92 34 04 00 00 00 00 30 39 77 E1 C3`；应答请求34、STOPPED、opcode0 |
| 能力查询 | 未实现，无二进制示例；文本替代VER\r\n=`56 45 52 0D 0A`，只适用于USART1 |
| 版本化实测车速 | 未实现，无TYPE、版本字节或原始包 |
| 组合动作 | 线上未实现，无opcode、配对帧或原始包 |

### 历史真实回执节选，不是本轮新测

```text
NORMALDEBUG BOARD=F405_ROBO SERVICE=MOTIONDEBUG SERIAL_V3 ... MODE=5 ... BUILT=Oct 29 2026 20:48:06
STATUS STATE=IDLE SRC=NONE CMD=0 SEQ=0 PROGRESS=0 LEFT=0 FAULT=NONE(0) IMU=1 ODOM=1 DYAW_MDEG=0
PONG MS=39579 ...
UART4 ... TX_DROPS=0 ... ODOM_TX=3955 IMU_TX=3954
PONG MS=45218 ...
UART4 ... TX_DROPS=0 ... ODOM_TX=4519 IMU_TX=4518
```

省略号是节选标记，不是原始协议字段。两次计数差564、时间差5639ms，约100.02Hz。

| 能力 | 已有软件依据 | 已有历史实车证据及边界 |
| --- | --- | --- |
| STOP | 解析、抢占、Motor_Stop、STOPPED回执源码存在 | 程序器STOP OK和停止状态日志多次出现；不是UART4独立电气闭环急停时延验收 |
| 定距直行 | 编码器差值、IMU保向、距离阈值和超时分支存在 | 旧full_suite日志有500mm DONE；未附同版本卷尺测量，不宣称实际500mm精度合格 |
| 定距侧移 | BODY/FIELD投影判距及F405侧移闭环存在 | 20261029_192825日志完成三次300mm、请求400mm/s，操作者当时认可；不是独立测距或路径精度认证 |
| 定角 | 未回绕角差、稳定确认、修正与超时分支存在 | 既有90/180度及其他转角日志；192825中1度请求也出现DONE但0进度，不能宣称任意小角都准确执行 |
| yaw/rate遥测 | 0x1F打包与发送计数存在，三种配置曾编译通过 | 最后烧录后发送计数约100Hz、TX_DROPS=0；未抓取RDK端原始包，不等于上位机收到 |
| move_spin_angle线上组合 | 未实现 | 无该线上接口的软件或实车验收，不拿内部C函数替代 |
| 能力查询、版本化速度、epoch | 未实现 | 无对应验收 |

历史full_suite来自更早运动实现，侧移认可记录属于b7ee9b2所保存的控制改进，
不能直接等同于14157e6完整回归；14157e6烧录后仅有静态遥测核对。
本轮只离线校验合同JSON与示例CRC，没有执行固件回归测试、模拟运动、硬件测试或重新编译。

### 源码依据：以下链接固定到固件实现14157e6

- [构建预设](https://github.com/kkkkkkkkkkkkkkkk222222/F407-Rescue-Robot/blob/14157e6bf2600ff0480b133ab763298e12ce400b/CMakePresets.json)
- [模式、限制及运动学宏](https://github.com/kkkkkkkkkkkkkkkk222222/F407-Rescue-Robot/blob/14157e6bf2600ff0480b133ab763298e12ce400b/Main/Inc/app_config.h)
- [TYPE/opcode/flags/结构定义](https://github.com/kkkkkkkkkkkkkkkk222222/F407-Rescue-Robot/blob/14157e6bf2600ff0480b133ab763298e12ce400b/Main/Inc/vision.h)
- [CRC和帧构造](https://github.com/kkkkkkkkkkkkkkkk222222/F407-Rescue-Robot/blob/14157e6bf2600ff0480b133ab763298e12ce400b/Main/Src/vision.c#L48)
- [0x19参数验证与重复SEQ处理](https://github.com/kkkkkkkkkkkkkkkk222222/F407-Rescue-Robot/blob/14157e6bf2600ff0480b133ab763298e12ce400b/Main/Src/vision.c#L385)
- [0x15/0x1F/0x1A实际打包](https://github.com/kkkkkkkkkkkkkkkk222222/F407-Rescue-Robot/blob/14157e6bf2600ff0480b133ab763298e12ce400b/Main/Src/vision.c#L752)
- [100Hz采样/遥测调度](https://github.com/kkkkkkkkkkkkkkkk222222/F407-Rescue-Robot/blob/14157e6bf2600ff0480b133ab763298e12ce400b/Main/Src/Robot.c#L527)
- [动作启动、STOP/HOLD/RESET](https://github.com/kkkkkkkkkkkkkkkk222222/F407-Rescue-Robot/blob/14157e6bf2600ff0480b133ab763298e12ce400b/Main/Src/Debug.c#L210)
- [动作完成、投影、控制权](https://github.com/kkkkkkkkkkkkkkkk222222/F407-Rescue-Robot/blob/14157e6bf2600ff0480b133ab763298e12ce400b/Main/Src/Debug.c#L385)
- [参考重置与前左方向里程计](https://github.com/kkkkkkkkkkkkkkkk222222/F407-Rescue-Robot/blob/14157e6bf2600ff0480b133ab763298e12ce400b/Main/Src/Location.c#L133)
- [混轮公式](https://github.com/kkkkkkkkkkkkkkkk222222/F407-Rescue-Robot/blob/14157e6bf2600ff0480b133ab763298e12ce400b/Main/Src/motor.c#L416)
- [内部组合动作，非线上接口](https://github.com/kkkkkkkkkkkkkkkk222222/F407-Rescue-Robot/blob/14157e6bf2600ff0480b133ab763298e12ce400b/Main/Src/motor.c#L2489)
- [10ms实测轮速](https://github.com/kkkkkkkkkkkkkkkk222222/F407-Rescue-Robot/blob/14157e6bf2600ff0480b133ab763298e12ce400b/Main/Src/motor.c#L1463)
- [原始编码器方向、采样和原子读](https://github.com/kkkkkkkkkkkkkkkk222222/F407-Rescue-Robot/blob/14157e6bf2600ff0480b133ab763298e12ce400b/Boards/F405/Src/encoder_f405.c)
- [IMU解回绕、分消息更新与age](https://github.com/kkkkkkkkkkkkkkkk222222/F407-Rescue-Robot/blob/14157e6bf2600ff0480b133ab763298e12ce400b/Main/Src/imu_uart.c)
- [文本身份、PID诊断与原始日志](https://github.com/kkkkkkkkkkkkkkkk222222/F407-Rescue-Robot/blob/14157e6bf2600ff0480b133ab763298e12ce400b/Main/Src/WheelCheck.c#L489)
- [测试/比赛服务身份文本](https://github.com/kkkkkkkkkkkkkkkk222222/F407-Rescue-Robot/blob/14157e6bf2600ff0480b133ab763298e12ce400b/Boards/F405/Src/diagnostics_f405.c#L21)

旧资料使用注意：tools/vision_protocol.py中的IMU编解码可参考，但它的旧运动helper
仍定义0x1B/0x1C、旧opcode和旧轮序odometry_body_velocity，不是本F405测试合同；
tools/test_motion_integration.py也覆盖旧假设，不能拿它的旧“通过”证明当前0x19合同。
上位机7e25a81的t265_map/include/scan_protocol.hpp把TURN_REL正方向注释为逆时针，
与本F405实际旧TURN方向冲突，应用适配必须以本合同的显式反号为准。
