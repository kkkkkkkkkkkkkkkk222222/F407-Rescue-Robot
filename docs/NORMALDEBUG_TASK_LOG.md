# F405 比赛任务诊断日志

适用于 `NormalDebugTask` 构建，即 NormalDebug 模式、运动测试开关OFF。
共用NormalRun任务；不修改抓取/审核/导航条件，不添加任务故障锁存。
默认每秒摘要，状态/条件变化最快100ms摘要，关键事件由任务原判断点记录。
仅输出到程序器USART1 PB6/PB7，115200 8N1；不改变RDK UART4协议。

## 构建与保存

```powershell
cmake --preset NormalDebugTask
cmake --build --preset NormalDebugTask
```

固件 `build/NormalDebugTask/WWW.elf`。本次没有自动烧录，因为比赛版会初始化机构，
接收有效上位机配置后可进入比赛任务。原 `NormalDebug` 运动固件不输出TD_TASK。
烧录时先断开自动启动来源、确认机械机构安全；任务版启动机构角度按原NormalRun流程。

用串口助手保存接收文本，或关闭串口助手后运行只接收的脚本：

```powershell
powershell -ExecutionPolicy Bypass -File tools/CaptureTaskLog.ps1 -Port COM18
```

默认保存在Windows“文档/F405TaskLogs”，文件名含日期/时间/端口。
脚本不发送任何命令、不控制电机，Ctrl+C仅结束记录，不停车。两种软件不能同时占用COM18。
应从上电前开始记录，保留TD_BOOT、现象前后至少10秒及上位机对应时段日志。
说明实际现象和大致时间，尤其是爪子碰地、手扶车、人工PAUSE、上位机重启等情况。

## 控制命令

| 命令 | 效果 |
| --- | --- |
| `TLOG 1` | 开启自动事件/摘要输出，默认已开启 |
| `TLOG 0` | 关闭自动输出，不停止比赛；显式维护命令仍可应答 |
| `TSTAT` | 请求一组最新摘要，不改变任务 |
| `TDUMP` | 导出内存最近32条已排队文本，保留原时间戳 |

日志命令不驱动车辆、不清除任务、也不自动重复机械动作。原DEBUG/S/RUN维护命令保持原行为。
TDUMP是32条文本，不是32秒；事件多时覆盖较快。导出时暂停更新历史缓存，但仍正常输出新日志。
缓存不保存到Flash，断电/复位后会丢失。电脑完整文件是主要依据。

## 摘要行

同一组摘要共享ID；字段以空格分隔，数值逗号分隔。HAL和APP是不同起点的毫秒时钟，
事件用HAL，任务内部年龄以APP计算，不可直接混减。

| 前缀 | 内容 |
| --- | --- |
| TD_BOOT | 格式版本、板型、构建日期时间、RCC复位标志 |
| TD_TASK | 任务mode/故障、状态停留时间、阶段时间、task/action、暂停、主循环/任务/电机节拍计数 |
| TD_PHASE | 搜寻、接近、抓后审核、运输审核、橙色后退、远程动作、分离、扫障子阶段及限距预算、比赛剩余时间 |
| TD_AUDIT | 当前frame/audit_id、累计采样/要求、STABLE、READY/VALID、语义结果、证据年龄、采样间隔阈值、类别/数量/标志、爪内可见和机构目标 |
| TD_LINK | 配置就绪、任务/视觉数据年龄及超时阈值、目标坐标、任务命令处理计数、拒绝阶段及动作号、UART4链路和融合定位状态 |
| TD_DRIVE | 三轮目标/实际轮速mm/s、PWM千分比、原始累计编码器低32位、四路舵机目标、IMU健康及日志丢失计数 |
| TD_WIRE | 实际交给协议解析器的字节数、有效/无效帧数、最后类型/序号/接收时间、状态报文排队次数 |

三轮顺序：右前、左前、后。四舵机顺序：升降、右爪、摄像头、左爪。
SERVO_TARGET、CAM_TARGET只是软件目标，不是实际角度传感器反馈。
审核运输相机等待通过标志CAM_WAIT_OK只针对原运输审核逻辑，不可套用其他mode。
年龄4294967295表示尚无有效来源。无效帧计数合并尾字节/类型/CRC检查失败，不单指CRC。
STATUS_QUEUED表示已交给状态发送队列，不表示上位机已收到；还可能被更新的状态覆盖。
LAST_REJECT_STAGE与REJECT_ACTION为历史/锁存信息，不表示当前一定仍被拒绝。
子阶段只在对应任务状态有意义，其他状态可能保留上次值。

### WAIT位图

WAIT是观察到的未满足条件集合，不是额外的任务判断器；部分条件在当前有限机械动作中
并不要求满足。必须结合MODE、PHASE及原任务逻辑解读，不能看到命令过期就断言卡死。

| 位值 | 观察条件 |
| --- | --- |
| 0x001 | 尚无有效比赛配置 |
| 0x002 | 上位机任务暂停 |
| 0x004 | 本地DEBUG维护暂停 |
| 0x008 | 当前任务上下文无有效命令或命令年龄超过阈值 |
| 0x010 | 当前上下文无有效视觉更新或已过期 |
| 0x020 | 审核状态中摄像头目标不等于审核角度 |
| 0x040 | 审核状态中爪内可见标志未满足 |
| 0x080 | 审核有效样本数不足 |
| 0x100 | 当前审核未收到符合规则的STABLE |
| 0x200 | 审核尚未VALID，可能因证据不足或语义非法，须看SEM/N/STABLE |
| 0x400 | 运输审核的相机准备等待未通过 |

SEM对应Task.c的AuditSemantic：0非法，1开局藏堆非空，2首件绿色，3合法物资，4单个伤员。

## 事件行

格式 `TD_EVENT HAL=... KIND=... A=... B=... C=... D=... E=... F=...`。
同种事件A/B/C未变时最多每秒记录一次，其余高频重复由累计计数反映，不能把事件条数当收包数。

| KIND | A/B/C | D/E/F |
| --- | --- | --- |
| STATE | 原mode/新mode/当时故障 | task_id/action_id/原状态停留ms |
| SESSION | 旧会话代数/新代数/mode | 原task/action/0 |
| RX_COMMAND | task/action/opcode | frame/打包标志/打包载荷类别数量 |
| ACCEPT | task/action/opcode | UART序号/frame/命令flags |
| REJECT | task/action/拒绝阶段 | opcode/frame/原任务拒绝枚举 |
| REJECT_META | task/action/拒绝阶段 | 命令flags/audit_flags/mode |
| ABORT | task/action/是否属于当前上下文 | opcode/序号/frame |
| AUDIT_GATE | 1相机目标、2爪不可见、3运输相机等待/task/action | 当前值/要求值/frame |
| AUDIT_SAME_FRAME | 当前是否READY/task/action | frame/样本数/STABLE |
| AUDIT_SAMPLE | 0新ID且间隔足够、1重复ID、2间隔不足/task/action | frame/距上次计样ms/audit_id |
| AUDIT_EXPIRED | task/action/原样本数 | 证据年龄/计样年龄/frame |
| AUDIT_RESULT | 样本数/STABLE位0与VALID位1/SEM | frame/audit_id/总数低8位与audit_flags高8位 |
| SWEEP_GATE | task/action/爪内可见 | 相机目标/要求/frame |
| SWEEP_AUDIT | 样本数/STABLE/VALID | frame/audit_id/总数 |
| SERVO_TARGET | 舵机ID/目标角度/0 | 0/0/0；仅软件下发目标 |
| STATUS_QUEUED | task/action/状态打包 | acknowledged_sequence/rejected_action_id/fault |
| CONSOLE_RX_ERROR | HAL错误码/0/0 | 0/0/0；不修改比赛任务状态 |

CONSOLE_RX_ERROR的0x80000000单独表示维护串口软件接收环满，不是HAL硬件ORE。

RX_COMMAND的E：bits0..7=命令flags、8..15=audit_flags、16..23=sequence、24..31=audit_id；
F：bits0..7=左类别、8..15=右类别、16..23=两侧计数字段、24..31=真实总数。
STATUS_QUEUED的C：mode位16..23、flags位8..15、action_status位0..7。
AUDIT_SAMPLE=0只代表当时ID和时间允许，不保证样本数再加一（可能已达到上限）。

拒绝阶段来源于task_accept_mission原分支，不重新决定是否接受：
1负载格式、2已废弃ESCAPE、3上下文/配置/基础有效性、4状态/新目标/覆盖命令限制、
5任务动作编号及取消规则、6视觉帧倒退、7标定距离流程、8审核角色/无证据非法包、
9原任务状态与动作条件。阶段9再结合原command_reject_reason（0无、1状态、2审核、3空爪、4侧别）。
同组拒绝可能包含数个条件，需结合日志中的具体值及对应版本代码分析，不声称每个分支都有单独编号。

## 使用日志定位

- BYTES不增长：优先查上位机是否发送、串口配对和电气连接，不代表一定是上位机程序故障。
- BYTES增长但VALID_FRAMES不增长：查协议帧/波特率/错误计数；格式不合法不能进入任务。
- 有新命令但REJECT增加：查看拒绝阶段、任务/动作ID、角色、阶段和前后状态。
- 审核样本停在1：看重复audit_id、帧新鲜度、100ms间隔、证据窗口和STABLE；不能只看视觉框内有物体。
- 电机目标全0：结合暂停、状态及阶段判断正常等待，不能直接称电机卡死。
- 目标和PWM非0、累计编码器无进展：驱动/供电/阻力/反馈都有可能；相机是否真正到位等需现场观察。
- 编码器有进展但车身不走：检查打滑/碰地/支撑，软件里程不等于真实地面位移。
- 主循环停止会同时停止日志输出；心跳只能辅助判断恢复后的节拍，无法凭最后一行绝对定位硬卡死。
- 复位后TD_BOOT给出标志，但本版不保存断电前完整故障现场，也没有新增看门狗或自动复位。

## 实现和验证范围

事件64槽、TX4096字节、最近32条文本缓存。格式化和发送在主循环，任务/机构只写小事件；
HAL中断发送，单次发送超时清理日志队列并计数，不停车、不重试任务、不新增故障锁存。
日志拥塞只丢日志，TD_DRIVE中LOG_DROP/EVT_DROP/TX_TIMEOUT可见。
多个模块快照非同一硬件采样时刻；各行ID关联用于上下文，不用于微秒级时序推断。
PB6/PB7仍与板载IMU共线，比赛维护串口仍为原逐字节IT接收（非运动版DMA）；
新增错误恢复丢弃受损行、要求换行重发，不因维护串口错误打断比赛任务。

构建验证：NormalDebugTask、NormalDebug、NormalRun通过。
本地模拟验证日志摘要/完整行、开关、显式TSTAT、缓存TDUMP、事件溢出、TX超时通过；
仅验证日志模块，不冒充比赛状态机或真实硬件联调。未烧录比赛版，未运行真实比赛动作。

## 2026-10-25 实机收发验证补充

用户确认RDK自动程序暂停/断开、串口助手关闭后，已烧录NormalDebugTask进行静止验证。
初测自动摘要、TSTAT、TDUMP及开关正常，但发现日志入队整行复制期间关中断，
使1kHz单轴UART接收错误增长。修正为主循环单生产者、不关中断的分段memcpy；
事件生产仍用短临界区，发送完成IRQ只修改完成标志，不访问文本队列头尾。

重新编译/烧录后，约60秒运行及查询中单轴UERR=0，LOG_DROP/EVT_DROP/TX_TIMEOUT均0。
8次TSTAT均收到完整6行摘要；TLOG 0后静默，TDUMP BEGIN/END完整，TLOG 1恢复输出。
全程MODE=0 WAIT_CONFIG、CONFIG=0、三轮目标/反馈/PWM均0，没有发送比赛配置或运动指令。
机构按比赛入口初始化为升降55、右爪147、摄像头90、左爪23度，并非运动测试的85度升降。
当前实际驻留固件为比赛任务版，恢复上位机自动程序后可能启动任务，需先确认场地安全。
本次只验证日志链路和静止等待状态，尚未验证全部抓取/运输流程。
原始记录在本地build/tasklog_hardware_fixed_20261025.log及tasklog_commands_fixed_20261025.log。
