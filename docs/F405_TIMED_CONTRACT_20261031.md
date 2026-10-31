# F405 NormalDebug：带时标反馈、身份与对时接口 v1.0

日期：2026-10-31。实现提交：`43fa6ab456ff26f2cc5661d2b3934efbf482fb45`。
保留前序`a54ee46`及其全部运动改进，没有回退到14157e6。
已核对上位机`danmo-teng/shijue_fangan/codex/gamepad-teleop@278370a`，
包含轮速SDK输入阶段。本轮没有修改上位机或正式比赛流程。

**本版已实现字段、查询、对时事务和统计接口，但尚未烧录、尚未进行板上或实车验证。**
MCU接收事件时间已提供；IMU真实采集延迟、后轮EXTI边沿延迟及HSE漂移上界仍未知。
因此当前不能声称满足上位机5ms/50ms预算，严格SDK门控应继续拒绝不合格数据。
本文件中的“有效反馈”不等于“时间已标定”或“已证明无打滑”。

交付：[机器合同](F405_TIMED_CONTRACT_20261031.json)、
[31组独立示例](F405_TIMED_EXAMPLES_20261031.json)、
[验证清单与镜像哈希](F405_TIMED_VALIDATION_20261031.json)、
[实际C模块的桌面仿真日志](F405_TIMED_HOST_SIMULATION_20261031.log)。
日志的时钟、UID、epoch及UART均为测试替身，不是板上数据。

## 1. 运行范围、构建和身份

- 新接口仅编译进F405的MOTIONDEBUG服务：APP_BOARD_F405_ROBO=1且APP_ENABLE_MOTION_DEBUG_TASK=1。
- `NormalDebug`仍默认UART4=115200；支持只读查询，但拒绝高频新流配置。
- `NormalDebugTimed`是新增构建预设，不是新运行模式。它仍为MODE=5、SERVICE=MOTIONDEBUG，
  UART4改为230400，程序器USART1保持115200，传感器UART5保持230400/1kHz。
- `NormalDebugTask`、旧F407 MotionDebug和正式比赛入口不启用此接口、不占用新TIM5或TX DMA。
- 新遥测上电默认关闭。客户端必须先查身份、建立时钟映射，再显式订阅；订阅不驱动车辆。
- 不新增组合控制，不修改0x19/0x1A的动作逻辑、8位SEQ限制、mode51、ESCAPE或舵机驱动。
- 新接口没有雷达驱动、T265外参、杆臂补偿，也不把原生gyro硬加135度。

仅供部署人员查阅，本轮未执行烧录：
```text
cmake --preset NormalDebugTimed
cmake --build --preset NormalDebugTimed
# build/NormalDebugTimed/WWW.elf
```

JP506：UART4，MCU TX=PC10、RX=PC11；必须与RDK客户端同波特率。
改变波特率通过构建预设完成，无自动运行时切换。旧115200客户端不能直接连接230400镜像。
本轮编译镜像的构建身份为：
`7106765d648eee9e2b4040b782b0e1ac0aa8e0b37602080fb879bf950fe8c64e`。
嵌入Git提交为43fa6ab，SOURCE_DIRTY=1，来自既有Task.c两行空白差异；
IDE映射和tmp资料未纳入源码提交。构建身份指纹包含编译源文件及配置内容，
从实际运行镜像常量返回，不是RDK传入的“预期版本”，也不是整个ELF的自校验或签名认证。
ELF SHA256见验证JSON，不能冒充板上读回哈希。

身份还返回运行时UID、DBGMCU_IDCODE、CPUID、Flash大小。
BOARD=405说明编译的板级映射；F405/F407共享部分芯片族识别码，不能仅凭IDCODE
自动认证板子的具体商品型号或接线。实际运行身份仍需烧录后读取新身份回执。
最后一次历史烧录确认仍是14157e6，本轮不能证明板上现在运行新镜像。

## 2. 新扩展封装，旧帧保持原义

本版正式采用独立`D4 4D`扩展命名空间，不占用、重解释或分片旧A3 B3 TYPE。
新TYPE数值只在此命名空间有效，不是旧草案的TYPE/opcode。

| 帧偏移 | 字节 | 含义 |
| --- | --- | --- |
| 0 | 2 | D4 4D |
| 2 | 1 | version=1 |
| 3 | 1 | 扩展TYPE |
| 4 | 2 | payload长度u16 BE |
| 6 | 2 | sequence u16 BE |
| 8 | N | payload |
| 8+N | 2 | CRC16低字节、高字节 |
| 10+N | 1 | E4 |

CRC16/MODBUS初值FFFF、多项式A001，覆盖version至payload末尾，即帧[2:8+N]。
除CRC低字节先传外，全部数值大端，signed使用二进制补码。
最大MCU发送payload=160，完整帧最大171字节；MCU接收请求整帧最多64字节。
合法请求目前payload仅8或12字节。接收残缺扩展帧20ms超时后丢弃，不触发停车。
无分片：一个轮反馈或gyro快照在一个带CRC的完整帧中，不能从不同包拼字段。
对混合流，必须先识别并完整跳过有效旧15字节帧，不能在旧payload中搜索新magic而误同步。

| TYPE | 请求/回执 | payload字节 | 行为 |
| --- | --- | --- | --- |
| 01/02 | 身份能力查询/响应 | 8/137 | 只读，无运动、清零或停止副作用 |
| 03/04 | 对时请求/响应 | 8/46 | 只读，返回nonce及MCU时间事件 |
| 05/06 | 新遥测订阅配置/回执 | 12/48 | 只改变新流开关；不改变车辆控制、原始计数和gyro参考 |
| 07 | 实测轮反馈 | 84 | 标称50Hz，两次10ms反馈区间之和 |
| 08 | 主yaw及角速度 | 84 | 标称100Hz最新源记录快照 |
| 09/0A | 统计查询/响应 | 8/132 | 只读，不清计数 |
| 7F | 扩展请求错误 | 12 | 支持的封装内返回错误；CRC损坏直接丢弃 |

请求/回执头部sequence相等，真正的关联键是64位nonce及响应boot_epoch。
周期07/08使用一个独立u16发送计数，按65536回绕；它不是源样本ID、动作ID或时间。
所有新旧SEQ均不能替代tick；旧0x15/0x1F/0x1A的SEQ仍各自独立。
旧8位动作ACK不能靠新增身份查询自动变成带epoch的ACK。

下面表格全部为**payload内偏移**，不是帧绝对偏移。opaque字段为原样字节。

## 3. 只读身份/能力查询

01、03、09请求均为同一个8字节布局：
| 偏移 | 字节 | 字段 | 类型 | 含义 |
| --- | --- | --- | --- | --- |
| 0 | 8 | nonce | u64 | client correlation nonce, unique for outstanding requests |

02身份响应：
| 偏移 | 字节 | 字段 | 类型 | 含义 |
| --- | --- | --- | --- | --- |
| 0 | 8 | nonce | u64 | echo |
| 8 | 2 | epoch_fault | u16 | 0 RNG epoch available; 1 unavailable |
| 10 | 16 | boot_epoch | opaque | MCU hardware RNG, stable during boot |
| 26 | 4 | reference_epoch | u32 | Location reference generation |
| 30 | 8 | capabilities | u64 | presence bits, not time qualification |
| 38 | 2 | board | u16 | compiled board mapping identifier, 405 |
| 40 | 1 | mode | u8 | APP_ACTIVE_MODE=5 |
| 41 | 1 | service | u8 | 1=MOTIONDEBUG |
| 42 | 1 | protocol_major | u8 | 1 |
| 43 | 1 | protocol_minor | u8 | 0 |
| 44 | 4 | tick_hz | u32 | nominal 1000000 |
| 48 | 4 | uart_baud | u32 | actual running UART4 configuration |
| 52 | 4 | clock_flags | u32 | bit0 configured TIM5; bit1 epoch valid |
| 56 | 4 | drift_bound_ppm | u32 | 0xffffffff unknown |
| 60 | 12 | mcu_uid | opaque | three hardware UID words, each BE |
| 72 | 4 | device_idcode | u32 | runtime DBGMCU IDCODE; family does not uniquely identify board SKU |
| 76 | 4 | cpu_id | u32 | runtime SCB CPUID |
| 80 | 2 | flash_kib | u16 | runtime flash size register |
| 82 | 32 | firmware_id | opaque | image-embedded SHA256 build-input/config fingerprint, not an ELF self hash |
| 114 | 20 | git_commit | opaque | Git revision embedded when building |
| 134 | 1 | source_dirty | u8 | tracked source diff at build |
| 135 | 2 | max_payload | u16 | 160 MCU TX payload limit; MCU RX total frame limit is 64 |

capabilities位0..9依次表示：身份、对时、订阅、车中心反馈、gyro接收时标、统计、
旧协议兼容、MCU RNG128 boot epoch、reference epoch、镜像内构建身份；当前03FF。
能力存在不代表已满足SDK的时间或精度条件。
clock_flags.bit0表示TIM5已配置，bit1表示RNG epoch有效；这不是漂移或采样精度认证。
drift_bound_ppm=FFFFFFFF表示未知，不是0ppm。

137字节身份响应有完整性CRC，固件标识来自链接进镜像的字节数组。
Git字段仅作来源追踪；有未提交源文件时必须结合source_dirty和firmware_id。
无效RNG会返回epoch_fault=1、全0 epoch，所有数据均不得据此建立合格SDK会话。
不从RDK取得UUID，也不把固定UID当成每次启动的boot epoch。

## 4. 时间基准、boot和reference epoch

TIM5为1MHz的32位计数器，高32位由溢出IRQ扩展，线上始终传u64微秒。
低32位每4294967296us（约71.58分钟）回绕，完整u64线上时间不在该处回绕。
读取在极短PRIMASK临界区中合并high/CNT，并补偿尚未处理的UIF，避免跨回绕读撕裂。
TIM5 IRQ优先级0；不会修改原TIM6控制周期或软件里程计参考。

boot_epoch是启动时由MCU硬件RNG生成的128位值，运行期间固定。
在合格随机源假设下是概率唯一标识，不是持久Flash启动计数；
无法给出数学上零碰撞的无限次启动保证。超时或RNG错误时明确无效，不回退到固定epoch。
RNG等待有时间及迭代次数上限，失败不会新增运动故障锁存。

reference_epoch是boot内u32计数，每次Location_ResetReference增加；按2^32回绕。
本地参考重建只改变Location参考和此代号，不清原始轮计数、IMU累计yaw、
源样本ID或TIM5。跨参考重置的轮反馈区间标为无效。
raw/body数据不需要也不施加上位机场地135度offset。
本版未给所有其他历史内部重锚API扩展epoch合同；正式比赛重锚路径不在此接口范围。

## 5. 对时：无对称链路假设

04响应：
| 偏移 | 字节 | 字段 | 类型 | 含义 |
| --- | --- | --- | --- | --- |
| 0 | 8 | nonce | u64 | echo |
| 8 | 16 | boot_epoch | opaque | boot identity |
| 24 | 8 | receive_event_us | u64 | MCU handles last request byte from DMA parse, before main CRC validation |
| 32 | 8 | response_prepare_us | u64 | main prepares response timestamp; before CRC build/enqueue, not first UART bit |
| 40 | 4 | tick_resolution_us | u32 | 1, resolution only, not source uncertainty |
| 44 | 2 | epoch_valid | u16 | 1 hardware RNG epoch valid |

事件定义：
- t2=receive_event_us：UART4 DMA数据被IRQ解析到此请求末字节时读取的TIM5，
  早于主循环的CRC验证。不是UART引脚的第一/最后bit硬件捕获。
- t3=response_prepare_us：主循环准备响应时读取的TIM5，早于CRC计算、排队和发送。
  **它不是实际UART首bit发送时间**；字段名称不作这种承诺。
- t3之后的计算、排队、整帧串行传输和主机接收延迟均保留在往返观测中，
  不暗设为0；发送统计另外记录DMA API成功返回后的时间事件。

RDK用CLOCK_MONOTONIC记录H1（调用write之前）和H4（整帧接收校验之后），nonce每次独立。
在本次事务的时钟比例模型下，MCU_ns - host_ns的因果候选区间：
```text
lower = t3_us*1000 - H4 - tick_quantization_margin_ns
upper = t2_us*1000 - H1 + tick_quantization_margin_ns
```
不要求上、下行对称，也不把RTT/2当成已知单向延迟。
`tools/f405_timed_protocol.py:offset_interval_ns`提供这个计算，但**不自动认定映射合格**。
实际两时钟比例、事务内漂移、外推时间范围和漂移不确定度仍须纳入上位机模型。
本版MCU漂移上界未知，不能把上述初步区间当作任意时刻可外推的认证映射。
跨boot_epoch必须废弃旧映射；reference_epoch变化不重置硬件时钟，但上层场地参考另行处理。

时间源误差与对时误差是不同量。对时良好不能消除IMU内部采集/滤波/输出延迟，
也不能消除后轮EXTI边沿到软件计数的未知延迟。
当前源不确定度FFFFFFFF必须让严格SDK门控拒绝，而不是由上位机强制替换成0。

## 6. 实测轮反馈，TYPE=07

| 偏移 | 字节 | 字段 | 类型 | 含义 |
| --- | --- | --- | --- | --- |
| 0 | 16 | boot_epoch | opaque | boot identity |
| 16 | 4 | reference_epoch | u32 | reference generation at interval end |
| 20 | 4 | measurement_id | u32 | real two-interval encoder feedback record ID; wraps modulo 2^32 |
| 24 | 8 | t0_us | u64 | previous encoder capture-bracket midpoint |
| 32 | 8 | t1_us | u64 | current capture-bracket midpoint |
| 40 | 4 | forward_um_s | i32 | rotation-center body X velocity, 1e-6 m/s; INT32_MIN invalid |
| 44 | 4 | left_um_s | i32 | rotation-center body Y velocity, 1e-6 m/s; INT32_MIN invalid |
| 48 | 4 | raw1_u32 | u32 | right front unmodified cumulative count low32 |
| 52 | 4 | raw2_u32 | u32 | left front low32 |
| 56 | 4 | raw3_u32 | u32 | rear low32 |
| 60 | 4 | delta1 | i32 | raw unmodified count increment for this exact interval |
| 64 | 4 | delta2 | i32 | same |
| 68 | 4 | delta3 | i32 | same |
| 72 | 4 | flags | u32 | feedback health; VALID does not imply source-time qualification or no slip |
| 76 | 4 | source_uncertainty_us | u32 | 0xffffffff unknown, rear software edge latency unbounded |
| 80 | 4 | capture_window_us | u32 | maximum observed CPU read-bracket width in interval, not total sensor timing bound |

原始来源是Encoder_Sample10ms实际读取的两路硬件计数器和后轮软件累计反馈。
在读前、读后取TIM5，以读窗口中值作为采样端点；相邻两个反馈区间合成一条约20ms平均速度。
read bracket不是物理编码器边沿时间，顺序读取三轮的时间跨度见capture_window_us。
不使用指令目标、PID目标或DONE进度；不新增滤波，不引入滤波窗口/群延迟。

```text
K = pi*0.070/1061.268                     # m/count
dt = (t1_us-t0_us)*1e-6
v_forward = K*(delta1-delta2)/(sqrt(3)*dt)
v_left    = K*(delta1+delta2-2*delta3)/(3*dt)
forward_um_s = round(v_forward*1e6)
left_um_s    = round(v_left*1e6)
```

delta为已经按原始计数回绕规则得到的、**未取反馈负号**的增量。
M1右前、M2左前、M3后轮；该直接公式已包含当前反馈极性，不再重复取负。
counts/rev=1061.268、轮径70mm，复用当前F405参数。
raw低32位按2^32回绕，不是截限；delta属于本包区间。
丢失某个包后，相邻收到的raw差值可能覆盖多个区间，不能与后一包自己的delta强行相等。

| flags十六进制位 | 含义 |
| --- | --- |
| 0001 | VALID_FEEDBACK：本区间反馈可用于数值展示/复算，不代表时间合格 |
| 0002 | TIME_BOUND_KNOWN：本版不置位 |
| 0004 | 采样附近观察到后轮EXTI pending |
| 0008 | 后轮软件解码非法AB跳变计数变化 |
| 0010 | 区间为0或大于50ms |
| 0020 | 累加或速度数值超出表示范围 |
| 0040 | 某个反馈间隔大于15ms，可能漏掉标称10ms采样 |
| 0080 | SLIP_UNKNOWN：始终明确未知打滑 |
| 0100 | epoch无效 |
| 0200 | 没有新反馈，旧记录失效重报 |
| 0400 | 区间跨过本地参考重置 |

缺反馈/无效时速度取INT32_MIN并清VALID，不用0伪装缺数据。
有效静止反馈可以为0，但时间资格仍须单独满足。
能检测：上述pending/非法转换、间隔异常、数值问题及新数据缺失。
不能可靠检测：前轮编码器断线与静止的区别、所有后轮漏边沿、空转/地面打滑、轮胎形变、
真实地面速度或有效轮径变化。没有“已确认无打滑”标志。
后轮软件边沿服务延迟未证明上界，因此source_uncertainty_us=FFFFFFFF，
不是把两次CPU读取之间的2us左右窗口冒称整个反馈的时间误差。

## 7. 主yaw和主gyro，TYPE=08

| 偏移 | 字节 | 字段 | 类型 | 含义 |
| --- | --- | --- | --- | --- |
| 0 | 16 | boot_epoch | opaque | boot identity |
| 16 | 4 | reference_epoch | u32 | Location reference generation; raw yaw not reset |
| 20 | 4 | publication_id | u32 | publish attempt ID, not source sample ID |
| 24 | 8 | yaw_mdeg | i64 | native CCW-positive cumulative yaw, 0.001 deg |
| 32 | 8 | yaw_receive_us | u64 | MCU ISR last-byte processing time, NOT acquisition time |
| 40 | 4 | yaw_source_id | u32 | MCU valid angle-message counter, NOT vendor sensor ID |
| 44 | 4 | yaw_flags | u32 | source health/new/repeat and receive-time semantics |
| 48 | 4 | yaw_uncertainty_us | u32 | 0xffffffff unknown source-time bound |
| 52 | 4 | rate_mdps | i32 | native CCW-positive Z angular rate, 0.001 deg/s |
| 56 | 8 | rate_receive_us | u64 | own MCU ISR last-byte processing time |
| 64 | 4 | rate_source_id | u32 | MCU valid rate-message counter |
| 68 | 4 | rate_flags | u32 | independent flags; heading uncertainty applies only to yaw |
| 72 | 4 | rate_uncertainty_us | u32 | 0xffffffff unknown source-time bound |
| 76 | 8 | publish_us | u64 | main snapshot publication event, before CRC/enqueue, not physical TX |

原生逆时针正，顺时针负。旧TURN的线上正值顺时针转换仅在上位机动作协议层处理。
本版yaw为i64毫度，rate为i32毫度/秒；**不要沿用旧0x1F的i32毫度+i16的0.1度/秒比例**。
旧0x1F本身仍原样存在，没有改字段。

yaw/rate各自的接收事件：
UART5 ISR读取消息末字节DR后、CRC/语义处理前取TIM5，再在校验成功后发布对应记录。
yaw_source_id、rate_source_id是MCU收到合法消息的独立u32计数，按2^32回绕，
不是模块提供的硬件样本ID。模块没有可传递的源采样tick或样本编号。
yaw/rate可能来自不同消息和不同时刻，本版明确分开，不假设同时采样。
没有F405遥测滤波；模块内部滤波及采集到输出延迟未知。
标称模块1kHz；实际合法到达频率可由各源ID增量/接收tick差估计，
该估计是到达频率，不是对真实采集频率、漏包数或采样相位的证明。

| 每源flags位 | 含义 |
| --- | --- |
| 0001 | VALID_VALUE，模块反馈值可用；不代表采样时标已合格 |
| 0002 | READY |
| 0004 | RECOVERING |
| 0008 | UNCERTAIN，仅yaw受原航向不确定标志影响 |
| 0010 | 未收到该源或距接收事件大于20ms |
| 0020 | rate达到约正负2000度/秒模块量程边缘，保守标为饱和；yaw解回绕的正常边界不标饱和 |
| 0040/0080 | 与上次入队记录比较，该源新数据/旧值重发，二者择一 |
| 0100 | 时间字段为MCU接收处理事件，不是真实采集tick |
| 0200 | 采集到接收的不确定度上界未知 |
| 0400 | epoch无效 |

断流时允许发显式无效/旧值诊断帧，但不更新源ID、源时间，也不算成新的有效采样。
publication_id只是发布尝试编号，不能拿它替代yaw/rate源ID。
传感器5字节帧在230400 8N1下连续发送约217us；这只是链路组成部分，
不能由此推出采集到接收的上界。未知内部缓存、滤波和IRQ延迟仍未标定。
当前yaw_uncertainty_us和rate_uncertainty_us均FFFFFFFF。
上位机应先识别这个“未知”哨兵再转换为纳秒，不能以32位整数乘1000溢出，或把未知替换成0。

## 8. 订阅及发送队列

05配置请求：
| 偏移 | 字节 | 字段 | 类型 | 含义 |
| --- | --- | --- | --- | --- |
| 0 | 8 | nonce | u64 | echo |
| 8 | 1 | enabled | u8 | 0 stop new stream, 1 enable |
| 9 | 1 | profile | u8 | 1 only |
| 10 | 2 | reserved | opaque | must be zero |

06配置响应：
| 偏移 | 字节 | 字段 | 类型 | 含义 |
| --- | --- | --- | --- | --- |
| 0 | 8 | nonce | u64 | echo |
| 8 | 16 | boot_epoch | opaque | boot identity |
| 24 | 4 | reference_epoch | u32 | Location reference generation |
| 28 | 2 | status | u16 | 0 OK,4 bandwidth,5 invalid profile |
| 30 | 1 | profile | u8 | 1 |
| 31 | 1 | enabled | u8 | resulting state |
| 32 | 4 | uart_baud | u32 | actual |
| 36 | 4 | wheel_period_us | u32 | 20000 nominal |
| 40 | 4 | gyro_period_us | u32 | 10000 nominal |
| 44 | 4 | required_baud | u32 | 230400 minimum |

profile=1，reserved=0，enabled=1开启新07/08，enabled=0关闭新07/08。
实际波特率低于230400时返回status=4，保持原开关状态；不自动提速。
非法profile/reserved返回status=5，不改变开关。
旧0x15/0x1F及动作状态不随此开关停止。
状态查询、对时、订阅和统计代码不调用Motor、Servo、IMU_ZeroYaw或Location重置。

轮流标称50Hz，gyro标称100Hz；主循环延误时不补发一串伪“新采样”。
跳过的发布时隙、被覆盖的测量ID、重复源记录都有独立统计。
UART4 TX改为DMA1 Stream4 Channel4，仅在F405运动测试服务启用，RX仍为Stream2 Channel4。
这组映射依据ST RM0090表43；不占用舵机或编码器引脚/定时器。

未发送的同类新遥测只保留最新一条；被替换会计数，主机须用测量ID/tick检测缺口。
在下次TX调度时，等待超过8ms的新07/08被丢弃；不会中断正在发出的帧。
8ms是“调度时淘汰规则”，不是RAM驻留或整个串口队列的物理绝对延迟证明。
已开始帧的串行传输、查询突发、系统暂停和主机批量接收另计。
DMA报错或发送超过100ms时在主循环中尝试中止失败发送并清理该队列项，失败则隔100ms重试；
这只恢复串口，不停车、不永久锁存。该错误恢复分支尚未做板上故障注入验证。
查询洪泛可能造成遥测缺口，主机必须拒绝覆盖不完整的轮速区间，不能隐藏或补零。

## 9. 状态、统计和不支持响应

09请求同8字节nonce布局。0A响应：
| 偏移 | 字节 | 字段 | 类型 | 含义 |
| --- | --- | --- | --- | --- |
| 0 | 8 | nonce | u64 | echo |
| 8 | 16 | boot_epoch | opaque | identity |
| 24 | 4 | reference_epoch | u32 | reference generation |
| 28 | 8 | snapshot_us | u64 | snapshot creation |
| 36 | 96 | counters | u32[24] | ordered counters in validation JSON / Python STATS_NAMES |

24个u32计数按`tools/f405_timed_protocol.py:STATS_NAMES`顺序，单位见名称：
requests、crc_errors、bad_headers、request_queue_drops、partial_timeouts、
wheel_measurements、wheel_invalid、wheel_skipped、gyro_enqueued、gyro_repeated、
gyro_publish_slots_skipped、enqueue_rejected、extended_started、extended_completed、
timed_coalesced_or_expired、max_wheel_capture_us、max_wheel_end_to_dma_return_us、
max_queue_us、max_yaw_receive_gap_us、max_rate_receive_gap_us、max_wheel_interval_us、
sync_replies、identity_replies、max_oldest_gyro_receive_to_dma_return_us。

累计计数按2^32回绕；最大值为本次启动的已观察最大值，不是形式化最坏界。
max_queue_us仅统计实际启动的新扩展帧；丢弃但未启动的项不包含在其中。
DMA时间是HAL启动成功返回后记录，不能改名为UART首bit捕获。
主机SDK调用时刻只在RDK，必须由RDK另外记录，MCU不能替它证明50ms端到端预算。

7F错误响应：
| 偏移 | 字节 | 字段 | 类型 | 含义 |
| --- | --- | --- | --- | --- |
| 0 | 8 | nonce | u64 | echo if request has >=8 payload bytes, otherwise 0 |
| 8 | 2 | status | u16 | 1 bad version,2 unsupported type,3 bad length |
| 10 | 1 | request_type | u8 | request kind |
| 11 | 1 | supported_version | u8 | 1 |

CRC错误无响应；请求环有效容量3条，满时丢弃并计数。
完整但不支持的version/type、错误payload长度会返回7F；
不支持版本的回应自身仍用version=1。客户端超时应重新用新nonce查询，不重放运动。
partial新包超时/丢弃不走程序器文本串口的“错误行停车”路径。

## 10. 带宽与门控预算

旧15字节帧仍保留：
| 下行项目 | 频率 | 字节/秒 |
| --- | --- | --- |
| 旧0x15 | 100Hz | 1500 |
| 旧0x1F | 100Hz | 1500 |
| 旧0x1A状态 | 20Hz | 300 |
| 新07轮反馈（95字节） | 50Hz | 4750 |
| 新08 gyro（95字节） | 100Hz | 9500 |
| 对时响应（57字节） | 5Hz示例 | 285 |
| 身份响应（148字节） | 1Hz示例 | 148 |
| 统计响应（143字节） | 1Hz示例 | 143 |
| 合计 | 上述请求频率假设 | 18126 |

8N1每字节10bit：115200需要157.34%带宽，故拒绝高频配置；
230400约78.67%，约21.33%平均余量。配置回执59字节及异常回执是突发，另外占用余量。
旧配置ACK4字节优先；实际FIFO、串行间隙、IRQ/主循环延迟、查询突发仍需实测。
RDK发来的运动/查询是另一全双工方向，不能忽略其CPU负载，但不能把双向字节直接当同一TX容量。
一条95字节帧在230400理想连续串行时约4.123ms；这不是采样到SDK的实测时延。
本轮没有做实际串口吞吐、排队或板端时钟漂移验证，不能把容量计算当成性能通过。

| 上位机接入预算 | 本版提供/尚缺 |
| --- | --- |
| 轮区间<=50ms | 按真实TIM5端点计算，正常20ms；超限报告无效；尚无板上最大值 |
| 每源时间不确定度<=5ms | 未证明，明确FFFFFFFF；严格门控应拒绝 |
| gyro有效样本间隔<=20ms、覆盖完整轮区间 | 独立接收时间/ID可用于检查；发布100Hz不是无缺口保证 |
| 最老采样到SDK调用年龄+不确定度<=50ms | 可用映射后检查；未知采集延迟、漂移和SDK处理时间尚未验证 |
| boot可区分 | MCU RNG128及无效状态已实现；尚未板上验证RNG/复位行为 |

需要后续实机核对：TIM5回绕和debug暂停影响；RNG冷/热复位身份；实际DMA吞吐与队列计数；
编码器读区间、后轮EXTI服务延迟；模块采集/滤波/输出延迟资料或独立测量；MCU相对主机漂移。
示波器/外部时间参照或可追溯模块时间信息缺失时，不应编造5ms上界。
指令目标、DONE、同一gyro回读不构成独立距离或角度真值。

## 11. 上下位机分工和部署约束

下位机只给旋转中心B点、B轴的反馈，原生CCW yaw/rate、源事件、epoch和健康状态。
不换算到T265定位中心或内部IMU，不加相机安装旋转、杆臂或雷达外参。
上位机继续负责
`v_C^C = R_CB * (v_B^B + omega_B^B cross r_BC^B)`、
SDK轮速输入、T265输出回车中心，以及`field_yaw=local_yaw+offset`。
1号场地方向135度只由上位机建立固定参考，不写入原生gyro。

RDK唯一底盘客户端需要新增混合帧解码、nonce对时、boot/样本身份检查和资格判定，
不能让另一个测试脚本同时打开底盘串口。新固件增加字段不代表SDK自动开始注入。
未具备完整gyro覆盖、有限时间不确定度或可信映射时继续不注入；
雷达入口仍做静止几何与旋转链路记录，本版没有交付移动扫描补偿。

旧0x19定距、定角、STOP和reset_reference维持原合同；
旧TURN正值顺时针由上位机适配层反号，gyro不得反号。
reset_reference不清原始轮计数和原始累计yaw。
旧8位动作SEQ的跨重启迟到回执风险仍存在，新遥测boot_epoch不能自动绑定旧动作ACK。
没有新增永久故障锁存、通用ESCAPE、比赛路径或安全区分段清零。

## 12. 证据分类和源码

四个预设已编译：NormalDebugTimed、NormalDebug、NormalDebugTask、旧MotionDebug。
新的真实C模块在x86主机以假时钟/假平台运行，检查查询、CRC、版本错误、baud门控、
正反/横向反馈、合法静止、缺反馈、源消息ID、旧值重发、参考变化及重启。
C产生的帧再由独立Python编解码核对；日志与31组示例已导出。
另有原23项协议检查通过，但不拿旧helper测试结果替代新板物理验证。
本轮没有打开硬件串口，没有烧录或执行实车动作；板上延迟日志为空，不借用14157e6的历史发送计数冒充。

实现源码固定于43fa6ab：
- [协议与测量模块](https://github.com/kkkkkkkkkkkkkkkk222222/F407-Rescue-Robot/blob/43fa6ab456ff26f2cc5661d2b3934efbf482fb45/Main/Src/DebugTelemetry.c)
- [共同TIM5时钟及MCU RNG](https://github.com/kkkkkkkkkkkkkkkk222222/F407-Rescue-Robot/blob/43fa6ab456ff26f2cc5661d2b3934efbf482fb45/Main/Src/DebugClock.c)
- [编码器实际采样钩子](https://github.com/kkkkkkkkkkkkkkkk222222/F407-Rescue-Robot/blob/43fa6ab456ff26f2cc5661d2b3934efbf482fb45/Boards/F405/Src/encoder_f405.c)
- [IMU消息接收事件](https://github.com/kkkkkkkkkkkkkkkk222222/F407-Rescue-Robot/blob/43fa6ab456ff26f2cc5661d2b3934efbf482fb45/Main/Src/imu_uart.c)
- [UART DMA与队列](https://github.com/kkkkkkkkkkkkkkkk222222/F407-Rescue-Robot/blob/43fa6ab456ff26f2cc5661d2b3934efbf482fb45/Main/Src/Uart.c)
- [构建身份生成](https://github.com/kkkkkkkkkkkkkkkk222222/F407-Rescue-Robot/blob/43fa6ab456ff26f2cc5661d2b3934efbf482fb45/cmake/DebugBuildIdentity.cmake)
- [参考编解码与因果对时区间](https://github.com/kkkkkkkkkkkkkkkk222222/F407-Rescue-Robot/blob/43fa6ab456ff26f2cc5661d2b3934efbf482fb45/tools/f405_timed_protocol.py)
- 硬件依据：[ST RM0090，TIM5/RNG及表43 DMA映射](https://www.st.com/resource/en/reference_manual/rm0090-stm32f407-advanced-armbased-32bit-mcus-stmicroelectronics.pdf)。
