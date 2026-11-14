# 进攻带载转正与回抓单伤员审核复用

## 核对范围与结论

已读取RDK固定入口、两份任务/机器合同、protocol.py、state_machine.py、run_competition_rescue.py及路线实现，
固定源码为`b1b3531782b95d65cc7b4a24927935344ee98e7d`，本端从`63d75a88c4e13cce11fb8731aca14383269e8ce5`增量修改。
接受MATCH-ATTACK-FACING-20261114及MATCH-RECOVERED-INJURY-AUDIT-20261114的当前最小配套。
上位原两帧缺框缓存、原物资/伤员找回失败出口不需要新增下位动作，未重复实现。

原固件已支持step13及SEARCH入口，但不支持新step14和NAV bit5复用语义。
RDK引用的16:40板端63d75a8/dirty1仅是历史回执，本窗口未连接设备，不能证明板内字节与Git完全相同。
本轮未烧录、未打开UART或发实车命令；用户自行烧录。

## 进攻点先带载转正

原15字节A3 B3封包、CRC16/MODBUS、物理UART4 921600/8N1及0x1D→0x18同SEQ配对不变。
TYPE0x18、opcode0x14追加step14 ATTACK_FACE_OPPONENT；不是新TYPE0x14，也不改旧step0/4。

| 字段 | 红方 | 蓝方 |
|---|---|---|
| P1（step14＋原VALID/RED） | 0xE9 | 0xE1 |
| A/B | 0/0 | 0/0 |
| U（绝对场地yaw，厘度） | 27000 | 9000 |

仅本任务已完成initial-stash NAV、真实合法initial-stash审核及闭爪状态可以开始。
沿原动作编号检查及FFFF→0/task+1回绕；旧NAV DONE不当作新转向DONE。
复用原连续定位回正：轮周请求200～400mm/s、到角3°；不调用有10s超时的有限TURN，不改其参数。
保持原审核、READY/VALID、闭爪、route_to_stash；此步不松爪、不清载荷、不增加投送计数。
完成仍为本次task/action/opcode、mode25/COMPLETED。同action重发不重开已接受/已完成动作。

上位随后发原RELEASE_BOTH；仍由原开爪流程到mode34。再接已有step13四段返程和step12回SEARCH。
U由上位固定红270°/蓝90°；下位不硬编码路线坐标，不将step13 D=0改为转向。
原step0..13的参数与执行不变；step14仅共享原0/4的连续转向代码。

红方task10/action99、SEQ39示例（离线合成，未发送）：

```text
a3 b3 1d 27 00 0a 00 63 00 00 00 00 31 91 c3
a3 b3 18 27 14 e9 00 00 00 00 69 78 19 ca c3
```

两侧完整示例及原step13合同见[机器合同](contracts/f405_empty_body_move_20261114.json)。

## 回抓单伤员短送复用真实审核

原NAV opcode0x03的P1 bit5=0x20为REUSE_POST_GRAB_AUDIT，只有STAGE NAV可带此位；
须同时保留原VALID/DISTANCE_VALID/STAGE_ONLY。compact红0x79、蓝0x71；原完整方向双位形式也兼容，部分方向位仍非法。
A为剩余毫米、B=0、U为场地平移方向厘度。APPROACH的CLUSTER、DISPERSE的FIRST_GREEN_BUMP及0x14高四位step含义不变。

上位负责确认原载荷找回来源。本端没有来源字段，不能假装能独立核实“回抓”；
只在mode22→STAGE NAV时，根据真实已有audit_received/host_stable/valid、单伤员语义、伤员目的地、非initial-stash及闭爪记录授权。
普通未带标志、物资、缺少真实审核或到NAV之后才加标志，均不能获得直通ALIGN资格。

不改H/D、转向、速度、粗段/末段、容差或制动。STAGE D=0仍先报告mode10/distance_done/nav_done，
保留真实审核；满足授权的当前完成NAV可接受下一条原pose ALIGN，设置原伤员运输角色并调用task_begin_safe_align。
ALIGN消费本次授权，原真实审核仍在。随后原回正、视觉横移和ENTER保持。
普通/不合资格NAV仍由上位发原0x12占位请求进入mode48；不改成到D0自动切mode48，也不伪造VALID或强制ENTER。

撤销NAV标志立即撤销本次授权，重新加位不能恢复。实际runner把flags加入action键，所以只变此位时即使换新action，
也沿原NAV更新保留nav_coarse原点/阶段，不重开高速段；同action只变此位也兼容。
其余编号排序、flags不可变规则及新目标规则不放开。原0x12复审、开爪/分离、原失败恢复、空爪SEARCH、换目标、ABORT和初始化清授权。
人工PAUSE仍保留当前动作；不新增时间窗口、新鲜度门、反复握手、等待或永久FAULT。

实际取舍：仅该类回抓连续闭爪短送减少一次独立末端掉货/混入检查；原真实合爪审核和已观察冲突后的复审仍保留。
普通运输、数量/首绿/目的地规则、原15s审核恢复不改；原上位24/47/实际空爪SEARCH失败出口直接兼容。

蓝方task10/action100、SEQ40，D245mm/H339.93°示例（离线合成，未发送）：

```text
a3 b3 1d 28 00 0a 00 64 00 00 00 00 c5 a1 c3
a3 b3 18 28 03 71 00 f5 00 00 84 c9 00 28 c3
```

完整字段、撤销/降级及红蓝D0示例见[机器合同](contracts/f405_recovered_injury_audit_20261114.json)。

## 必要结果与部署边界

- NormalRun、MotionDebug构建链接通过。复用原FLOW_20261019_CHECK及其现有Task夹具直接调试调用，未新增测试代码/独立程序。
- 红蓝真实两帧审核→闭爪mode22→标记NAV→D0→原pose ALIGN→ENTER符合预期；普通伤员/标记物资/晚加标志不能跳mode48，原0x12入口保持。
- 同/新action撤销只变bit5不重置导航；重新加位不获资格；原清审核、释放、恢复、SEARCH和ABORT清授权符合预期。
- 红蓝真实initial-stash审核→NAV DONE→step14 RUNNING/DONE→原RELEASE_BOTH/mode34→step13通过；重发不重开/重复放货。
  未到点、无合法审核、非藏点、开爪、错误归属不能进入step14。原报文CRC/字段及旧step0..13校验差异核对通过。
- PID、速度/PWM、舵机、0x19/0x1A、07/08/0x1F、ISR及时间政策均未修改；原独立工具/IDE改动及Task初始化空白保留。
  本机构建如实source_dirty=1；源码提交/构建标识不能冒充设备actual。
- 用户需要烧新`build/F405/NormalRun/NormalRun.elf`并重新开启最新RDK正式入口。本端没有烧录或读取板端actual；
  转向/轮滑、机械分离/载荷保持及实际比赛时序仍待人安排实测，不把模拟/DONE/编码通过当作地面精度验收。

固定上位依据：
[入口](https://github.com/danmo-teng/shijue_fangan/blob/b1b3531782b95d65cc7b4a24927935344ee98e7d/docs/CROSS_REPO_REQUESTS.md)、
[进攻转正](https://github.com/danmo-teng/shijue_fangan/blob/b1b3531782b95d65cc7b4a24927935344ee98e7d/docs/COMPETITION_ATTACK_RETURN_20261114.md)、
[回抓审核](https://github.com/danmo-teng/shijue_fangan/blob/b1b3531782b95d65cc7b4a24927935344ee98e7d/docs/COMPETITION_RECOVERED_INJURY_20261114.md)。
