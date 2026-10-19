# 连续物资抓取与分区投送流程

2026-10-19最新功能以[定距接近、累计审核和运输分离交接](docs/UPPER_RANGE_SORT_HANDOFF_20261019.md)为准：120/105度搜索，对准后mode52新图测距，APPROACH_RANGE有限距离请求驱动mode53，之后140度补进最多200mm；累计两帧允许短缺图，mode48允许带侧曲线分离，分离目标30/45度。以下旧125/130度主接近、300mm补进和小角曲线说明仅供历史对照，不能覆盖本次协议。必须同步更新上位机后部署。

2026-10-17最新上位机c84d77e的补充交接见[当前差异与修改建议](docs/UPPER_C84D77E_CEF1D9C_SYNC_20261017.md)，下位机功能基线为cef1d9c。本次文档更新不改变固件行为。

最新第1/2/3项增量见[初次空爪、ROI及固定航向交接](docs/UPPER_CAPTURE_ROI_HEADING_FIX_20261016.md)。初次mode21/38的纯空爪可以在原300mm预算和15秒总期限内继续慢爬，抓后/分离后空爪处理不变；上位机必须同步补丁，不能把初次空爪提前变成放弃。

当前上位机已核对至`danmo-teng/shijue_fangan codex/gamepad-teleop@81879e0`，但仍有ROI和选侧差异待修改，不能视作已经完全匹配。最新总交接以[UPPER_TOTAL_SYNC_20261016.md](docs/UPPER_TOTAL_SYNC_20261016.md)为准；旧补丁和日期较早的交接文档仅供历史参考，不得覆盖当前两帧、两次分离和10°夹紧规则。

正式运输首件恰好1件GREEN，之后单件普通或单件核心、伤员单独1件；INITIAL_STASH仍按任意真实非空处理。正式运行选择NormalRun配置。RDK X5与F407使用USART3（PD8 TX、PD9 RX）、115200 8N1、3.3 V TTL和公共15字节帧；必须使用1D/18与1E/17同SEQ帧对，旧裸18除ABORT外不再接受，详见`docs/UPPER_B0F954F_CONTEXT_READY.md`。

## 整体流程

`AUDIT_READY`要求本地两个不同审核ID满足至少100ms有效采样间隔，并且当前审核由上位机置`STABLE`确认；`AUDIT_VALID`再叠加现有类别/数量/目的地合法性。上位机按图像采集时间独立完成两帧稳定判断并先发送STABLE，再等待READY/VALID，不能互等。`vision_frame=0`且证据全零、非STABLE的占位包只作心跳；活动审核阶段清除旧证据，mode22/40/49/50保留已确认快照。mode22/40等待后续命令有15s恢复路径；上位机保留已有17s握手超时出口，不用selected_batch猜载荷。

两帧有效采样总跨度至少100ms；重复帧和旧STABLE不增加样本或续期，500ms无新证据清空活动审核。正式物资普通/核心类别变化也重置计数。非法签名比较真实总数、去STABLE标志、线上类别计数直方图和零数量非NONE类别掩码，不比较track ID或左右交换。持续审核设置15s总期限，mode48可有限摆动两次后回140°重审；超时放弃目标并恢复搜索/返中，绝不按超时放行。

伤员正式抓取执行半闭爪后退复核：首次mode23合法后进入mode49，左右爪由76°/104°松至81°/99°，稳定600 ms后保持航向以150 mm/s后退2 s；停车、相机140°稳定250 ms后清空旧审核再进入mode23，重新取得两帧复核，合法进入mode50闭爪，闭爪完成才进入mode22导航。普通/核心、临时藏堆、扫障原物资找回、mode48、单件数量规则及投送超时策略保持原流程。

1. 上电后先保持TIM8舵机PWM关闭，完成IMU配置、静止稳定和陀螺仪零偏校准。只有`IMU_Init()`成功后才启动4路舵机PWM，随后摄像头到90°、舵机1到55°，左右爪依次收缩到左23°、右147°；IMU失败则机构和正式Task均不启动。
2. 收到合法`TYPE=0x11`红蓝方和出发区配置后回复一次`A3 B3 01 C3`并启动连续任务，不设置180秒自动终止。
3. 开局采用分段动作：以600 mm/s后退600 mm，稳定250 ms，原地转180°（轮端切向速度250 mm/s），稳定并完成开爪后，以800 mm/s前进1100 mm到中心附近；每段独立定距/定角。不再使用开局边平移边转180°。
4. 开爪完成后进入`SEARCH`。聚集目标持续发送带`CLUSTER_TARGET`的APPROACH，F407保持双爪Open靠近，相机130°水平对正后到140°，稳定250 ms并进入`mode=38`，以200 mm/s锁航向慢爬夹内观察；足够非空样本时停车，收到STABLE后进入`mode=37`。能分侧时发送带`SIDE_VALID`的曲线分离；不能分侧时发送无侧DISPERSE，F407只转20°并以mode35请求新审核。125°/130°不新增固定空等。
5. 每次进入`SEARCH`先把舵机3移到120°并旋转一整圈，再切到90°稳定300 ms后旋转一整圈。两层共约720°均无目标后F407保持`mode=3`并以200 mm/s继续原地扫描；上位机此时发送现有`RETURN_CENTER H/D`，F407进入mode17返回场地中心，D=0后重新从120°开始。任一扫描阶段仍可由普通/聚集APPROACH接管。
6. 收到1帧原生1280×1024合法单目标坐标后锁定类别并进入`APPROACH`。舵机3达到125°时停车并水平对正到误差≤24 px，随后移动到140°并稳定250 ms；不再要求原track重新出现，直接进入`mode=21`并置`CLAW_VISIBLE=1`。车头视觉PID为0.18/0/0.0025；摄像头俯仰PID为0.55/0.00875/0.00168，接近跟踪变化率上限45°/s，两者不可混用。
   常规跟踪或125°水平对正期间收到HOLD会立即停车；连续500 ms仍没有新的APPROACH_TARGET时，F407自动进入mode24，静止恢复500 ms后回mode3 SEARCH。已经进入“相机到140°、140°稳定或聚集夹内审核”的本地阶段后不再需要目标坐标，HOLD不会中断这些已触发动作。
7. `mode=21`保持锁存航向并以200 mm/s慢爬，同目标前进预算300 mm。普通、聚集、分离复审和抓后复审统一要求2个不同`audit_id`语义一致；同audit_id重发不累计，STABLE不跳过计数，任何不一致审核立即以当前帧重新从1计数。合法物资按任务语义归一化，不依赖左右位置；非法审核比较总数、类别和非法标志。
8. 普通mode21及聚集mode38从第一次140°夹内慢爬开始共享同目标300 mm前进额度，APP/审核更新和短暂后退不重置或返还。到限停车审核；合法等待GRAB，无合格物资或4秒内无有效分离方案时在mode24完成张爪→后退300 mm→左转90°→mode3，不再直接重靠近旧目标。
9. 抓前2帧及STABLE合法后F407置`AUDIT_VALID`，上位机才发送GRAB。审核含核心或mixed时，F407先以180 mm/s编码器前进50 mm，再Touch合爪；重复GRAB只ACK。合爪完成不直接进入mode22，而是清除旧审核并进入mode23。抓后重新取得2帧及STABLE：普通/核心合法进入mode22；正式伤员先走49→23→50→22；空爪双开回SEARCH，非法保持mode23并允许释放/分离。mode22才接受NAV。
   mode23不会无限等待：140°主窗口3.5 s→138°短暂偏视后回140°重审3.5 s→142°短暂偏视后回140°最终重审3.5 s；仍无法形成2帧则双开回SEARCH。稳定非法审核等待上位机动作4 s，超时也双开回SEARCH，不会伪造合法。同一合法序列中任意一帧含core或mixed，都锁存核心证据用于50 mm前移。
   开局临时藏堆是独立语义：CARGO_AUDIT置`INITIAL_STASH`时，只要`total_count>0`就按`STASH_NONEMPTY`累计2个不同audit_id并要求STABLE，忽略类别、数量变化、危险/未知/伤员混装、左右饱和及分配不明；该规则同时适用于合爪前和mode23抓后复审。mode22接收NAV后锁存`route_to_stash`，到藏点允许`RELEASE_BOTH`，且藏堆不会置`first_delivery_done`。
10. 抓取闭合后，上位机先用带`STAGE_ONLY(bit6)`的`NAVIGATE_WAYPOINT`引导到安全区预备点（上位机`5b6164f`当前配置为入口前600 mm）。F407继续使用H/D导航，但到`D=0`只停车、把摄像头抬到120°、置`DISTANCE_DONE=1`并保持`mode=10`，严禁启动旧安全区补推。藏堆和返中命令不使用该标志，原流程不变。
11. 上位机先用CARGO_AUDIT占位请求mode48，停车闭爪、140°稳定后完成两帧及STABLE正式复审；合法且目的地不变才发送第一次`ALIGN_SAFE_ZONE`：P6/P7为红方90°或蓝方270°，F407用IMU原地对正，并确保摄像头120°命令后至少稳定300 ms。首次进入3°，稳定期使用6°迟滞窗口，避免IMU小幅波动反复重启ALIGN；完成后上报`mode=11`。上位机安全区框冻结仍按该独立流程判断，不与夹内两帧规则混用；第二次带`VISUAL_CORRECTION_VALID(bit6)`的ALIGN中P2/P3为有符号水平像素误差。F407按可标定焦距转换为一次相对转角，最大±15°，完成后再次上报新鲜`mode=11`并锁存最终航向。视觉超时降级策略保留。
12. 第二次视觉ALIGN完成后，上位机检查最终推进走廊。无障碍直接ENTER；有障碍发送`CLEAR_SAFE_ZONE=0x13`，P2/P3=0、P4/P5普通/核心`+200`或伤员`-200`、P6/P7=0。200表示向对应侧转90°后的前进距离，不是横移。mode39暂放原物资，张爪完成后倒退200 mm回S并恢复H；mode42/43视觉抓障，43的真实审核带bit6=`AUDIT_SWEEP_PICKUP`，要求两帧及STABLE受控非空。抓障后mode39逆序倒退沿采样轨迹回S，向相反侧转90°前进200 mm放障、张爪完成后倒退回S，再转180°朝向原物资。mode44找回、mode45正式全爪审核：合计前进预算200 mm不因APP/HOLD/44↔45重置，伤员保留DESTINATION_INJURY。合法后等待GRAB，mode39合爪并沿轨迹倒退回S恢复H，mode23复审→mode40→两次ALIGN→走廊检查。找回失败先张爪、倒退回S、恢复H才上报mode46，接受RETURN_CENTER进mode17再mode3；不计投送。旧固定距离兼容参数不变。
13. `ENTER_SAFE_ZONE`必须置`DRIVE_STRAIGHT`、清除`DISTANCE_VALID`且P2～P5为0；视觉ALIGN成功时bit6=1、清除`USE_FINAL_HEADING`且P6/P7=0，定位降级时bit6=0、置`USE_FINAL_HEADING`并发送红9000/蓝27000。F407本地编码器总目标为650 mm：前400 mm使用既有接近减速，随后以330 mm/s最终补推250 mm；从ENTER统一起点累计达到650 mm，或末段有效推送时间达到1500 ms，任一满足即结束推进；PAUSE时间不计入末段计时。仅该固定距离ENTER阶段豁免地图边界恢复，急停、PAUSE及位姿有效性检查不变。停车、双爪全开且相机120°稳定300 ms后上报mode15；收到TASK_COMPLETE后至少完成1000 ms本地投送观察等待，再进入mode16退出。
14. 自动地图边界恢复入口已关闭，不再因内部x/y达到边界阈值自动触发mode41；保留mode41编号、HOLD只ACK及张爪→向中心转向→驶入后SEARCH兼容处理，50/400 mm参数不改。局部行程预算、通信/视觉帧龄、ENTER接触保护仍保留。
15. 收到`TASK_COMPLETE`后进入mode16：S1抬到开局55°并等1秒，再以400 mm/s后退0.30 m；后退完成停车，S1回85°并等1秒，再进入mode17使用RETURN H/D返中。S3保持120°；RETURN提前到达只缓存/ACK，不打断升降或重置退出行程。D=0后mode3重新搜索。

上位机在收到STM32的新鲜配对退出、返中或搜索状态后进入下一轮；第一次投送完成前只选择单个绿色物资，之后仍只允许单个普通、单个核心或单个伤员，危险目标不进入正式投送。

NormalRun不再用单轮编码器方向、单轮零速或定距短时无进展自动进入`TASK_FAULT_MOTOR`。MOTOR、POSE_TIMEOUT、START_TIMEOUT、COMMAND_TIMEOUT、RAM现通过mode51保持载荷并等待既有参考恢复/审核交接，不作为永久故障锁存。人工ABORT进入`mode18/fault1 REMOTE_STOP`后重新开放赛前配置；操作员发送新的连续合法TYPE=0x11配置可以重新执行安全收爪和自主出发。人工急停及INVALID_STATE不属于自动恢复，不得按普通等待绕过。

## 公共帧

```text
A3 B3 TYPE SEQ P0 P1 P2 P3 P4 P5 P6 P7 CRC_LO CRC_HI C3
```

CRC为CRC-16/Modbus，覆盖`TYPE、SEQ、P0..P7`共10字节，初值`0xFFFF`、多项式`0xA001`，低字节先发送。各消息类型独立使用SEQ，重复SEQ不刷新对应看门狗。

## 消息类型

| TYPE | 方向 | 作用 |
| --- | --- | --- |
| `0x11` | RDK→F407 | 颜色与出发区配置 |
| `0x12` | RDK→F407 | 原生1280×1024普通物资坐标报告 |
| `0x15` | F407→RDK | 100 Hz三路编码器累计低16位 |
| `0x16` | RDK→F407 | 旧版可选融合位姿；当前默认不发送 |
| `0x17` | F407→RDK | 20 Hz任务状态 |
| `0x18` | RDK→F407 | 抓取、导航、对正、入区和完成命令 |
| `0x19/0x1A` | 双向 | MotionDebug运动命令/状态，不能替代NormalRun任务命令 |
| `0x1D` | RDK→F407 | 与0x18同SEQ的任务/动作上下文 |
| `0x1E` | F407→RDK | 与0x17同SEQ的上下文、接受opcode及拒绝action回执 |

### `TYPE=0x12`视觉报告

`P0/P1=X`、`P2/P3=Y`、`P4/P5=距离mm`，均为大端；`P6`为四类目标2位数量字段；`P7 bit0=FOUND、bit1=NEAR、bit3=CLASS_VALID、bit6=DISTANCE_VALID`。当前上位机直接发送0～1279和0～1023的原始坐标，中心为`(640,512)`。没有目标时P0～P7全0；没有可靠距离时距离为0且bit6清零。

已知中心帧：

```text
A3 B3 12 10 02 80 02 00 00 00 01 09 DD FD C3
```

### `TYPE=0x17`STM32状态

| 字段 | 内容 |
| --- | --- |
| `P0` | bit0爪子入镜、bit1夹爪闭合、bit2电机运动、bit3自动靠近、bit4审核合法、bit5导航定距完成、bit6审核就绪AUDIT_READY、bit7故障 |
| `P1` | 当前`TaskState`编号 |
| `P2/P3` | 摄像头角度，0.01°，大端 |
| `P4` | 最近真正接受的`TYPE=0x18`命令SEQ |
| `P5` | 故障码，0为正常 |
| `P6/P7` | 固定为0 |

当前故障码：`0 NONE、1 REMOTE_STOP、2 MATCH_TIMEOUT、3 MOTOR、4 START_TIMEOUT、5 POSE_TIMEOUT、6 COMMAND_TIMEOUT、7 RAM、8 INVALID_STATE、9 TARGET_LOST`。

### `TYPE=0x18`任务命令

`P0=COMMAND、P1=FLAGS、P2..P7=命令相关载荷`。FLAGS bit0必须为1，bit1要求直行，bit2使用绝对航向，bit3表示红方，bit4表示距离有效。bit5按opcode复用：APPROACH中为`CLUSTER_TARGET`，DISPERSE中为首件绿色专用`FIRST_GREEN_BUMP`。bit6按opcode复用：NAV为`STAGE_ONLY`，ALIGN/ENTER为`VISUAL_CORRECTION_VALID`，DISPERSE为`SIDE_VALID`；bit7仅在DISPERSE中表示`TARGET_RIGHT`。

命令值：`0 STOP、1 PAUSE、2 GRAB_CONFIRMED、3 NAVIGATE_WAYPOINT、4 ALIGN_SAFE_ZONE、5 ENTER_SAFE_ZONE、6 TASK_COMPLETE、7 ABORT、8 RETURN_CENTER、9 APPROACH_TARGET、10 HOLD、11 YIELD_BACKOFF、12 退役ESCAPE_MANEUVER（拒绝）、13 RELEASE_LEFT、14 RELEASE_RIGHT、15 RELEASE_BOTH、16 DISPERSE_PILE、17 CHANGE_LANE、18 CARGO_AUDIT、19 CLEAR_SAFE_ZONE`。


- `GRAB_CONFIRMED`会重复发送直到新鲜`TYPE=0x17`置`GRIPPER_CLOSED=1`；F407对舵机动作幂等，但每个新SEQ都必须更新P4 ACK。
- `HOLD`是高层心跳：SEARCH的120°/90°两圈及其后的低速补扫继续运行，直到收到合法RETURN_CENTER。需要冻结其他阶段时使用PAUSE。
- `STOP`只在NAV中保留原有可恢复兼容行为，其他状态会进入远程停止故障；`ABORT`同样进入REMOTE_STOP。普通等待不得使用STOP/ABORT；人工REMOTE_STOP需操作员新一轮合法赛前配置重启。mode51是可恢复等待，不得当作REMOTE_STOP或永久FAULT。
- 正式投送的`STAGE_ONLY NAV`兼容两种flags：精简形式`VALID|DISTANCE_VALID|STAGE_ONLY|阵营位`，以及在此基础上同时增加`DRIVE_STRAIGHT|USE_FINAL_HEADING`的完整形式；两方向位只出现一个属于非法帧。P2/P3仍为剩余距离/0，P6/P7仍为0～35999的场地平移方向。预备点D=0后置`DISTANCE_DONE`并保持mode10，不执行旧D=0补推；旧非STAGE NAV、藏堆NAV和RETURN仍强制要求完整方向flags，原语义不变。
- 合法流程为`WAIT_NAVIGATION→STAGE NAV(mode10)→TRANSPORT_AUDIT(mode48)→定位ALIGN(mode11)→可选视觉ALIGN(新鲜mode11)→ENTER→CHECK(mode15)→TASK_COMPLETE→EXIT_SAFE_ZONE→RETURN_CENTER`。
- 第一次ALIGN的P6/P7是90°/270°绝对航向；第二次ALIGN的P2/P3是冻结框相对640 px中心的有符号像素误差。视觉修正只能应用一次，重复帧只更新ACK，不能重复累加角度。
- ENTER必须置`DRIVE_STRAIGHT`并清除`DISTANCE_VALID`，P2～P5为0；视觉对正成功时bit6置位、bit2清零且P6/P7=0，定位降级时bit6清零、bit2置位并发送红9000/蓝27000。推进距离完全由F407编码器执行。
- 红方前置点为`(0,+950 mm)`，蓝方前置点为`(0,-950 mm)`。F407不接收PWM值，只接收任务目标并在本地完成转向、速度限制和失联停车。

### 完整比赛流程扩展

- `APPROACH_TARGET`的P2/P3和P4/P5分别为原生图像X/Y；完整流程激活后它是靠近控制的唯一坐标源。F407不再按250 ms帧龄自动减速；上位机明确发送HOLD后立即停车，常规APPROACH持续HOLD 500 ms会转mode24并回SEARCH，避免永久停在mode20。PAUSE仍是真正冻结，不参与该恢复。
- 普通目标在140°稳定250 ms后进入mode21并以200 mm/s按同目标预算最多慢爬300 mm。所有审核统一要求2个不同audit_id、至少100 ms间隔及本轮STABLE；合法才置AUDIT_VALID。mode23表示合爪后复审，mode22表示抓后审核及必要的伤员49/50流程完成，才允许NAV。
- `CARGO_AUDIT`按字节编码左右类别、数量、审核标志、audit_id和总数量；相同audit_id的重复传输不增加计数。`UNKNOWN_PRESENT=1`时允许`total_count > left_count + right_count`，表示ROI内物体无法分侧；该非法审核不能直接GRAB。带SIDE_VALID的DISPERSE优先要求保留侧非空，本地还兼容UNKNOWN_PRESENT且真实总数超出两侧和的中心未分配回退；上位机仍须有可靠选侧证据，不能盲目利用回退。`AUDIT_DESTINATION_INJURY`作为本地伤员合法性复核，不自行改变目的地。
- `AUDIT_SWEEP_PICKUP=P5 bit6(0x40)`仅允许mode43扫障夹内审核使用；其他状态收到该特殊审核拒绝。mode43只按非空和不同audit_id累计，不套用正式物资合法性，空审核立即清零连续计数；退出mode43后清除此特殊审核上下文。
- 非法物资分离使用140°全夹爪ROI。Touch为S4=76°/S2=104°；保留侧额外夹紧统一10°（左66°或右114°），单件/多件、第一次/第二次一致，不逐次放松。同堆最多2次实际分离，第二次真实复审仍非法则张爪→退300 mm→闭爪→mode47等待RETURN→mode17返中，到中心重新开爪搜索。其他放弃原因保留既有恢复路径。无侧20°观察不是曲线分离，也不计作物理分离次数。
- 首件绿色若在140°ROI中与其他物资挤在中心且曲线无法取出，上位机可发送`DISPERSE_PILE + FIRST_GREEN_BUMP(bit5)`，不得同时置SIDE_VALID/TARGET_RIGHT。F407仅在首件尚未完成、审核至少2件且含绿色时接受，并且每轮任务最多一次：后退0.10 m→Touch闭爪→前进0.20 m→后退0.10 m→双爪全开→直接回SEARCH。该动作不改变无侧DISPERSE的20°观察语义。
- 临时藏堆NAV不使用`STAGE_ONLY`，也不执行任何安全区补推；到点后`RELEASE_BOTH`。释放完成后至返中完成期间，F407拒绝`APPROACH_TARGET`和`DISPERSE_PILE`，避免残留找物命令抢占藏堆回程。第一条`RETURN_CENTER`先触发F407保持释放航向直退0.35 m，编码器累计距离且IMU修正航向；退到安全距离后才使用上位机持续更新的H原地调头并按D返中。该退让只对临时藏堆生效，正式安全区投送仍使用既有mode16后退0.30 m，不会重复后退。
- `YIELD_BACKOFF`只在F407已确认完成普通单侧释放且`cargo_recheck_pending=1`时接受。单侧释放与带侧DISPERSE均使用统一10°保留侧夹紧，曲线和直退由下位机执行。不带SIDE_VALID只执行20°观察并以mode35请求新审核。通用CMD12已删除；局部capture_escape仍表示必要的抓取失败退让，不是通用脱困。
- `RELEASE_BOTH`在普通非法抓取审核中表示放弃：张爪后mode24退300 mm、转90°再SEARCH；INITIAL_STASH/藏堆及扫障原释放流程仍以mode34完成。无侧DISPERSE仍为20°观察/mode35，不能混用。
- 对外mode映射为`3 SEARCH、15 DELIVERY_VERIFY、16 EXIT_SAFE_ZONE、17 FACE_FIELD_CENTER、18 STOPPED、20 APPROACH_TARGET、21 CAPTURE_AUDIT、22 CAPTURE_DONE、23 POST_GRAB_AUDIT、30 YIELD_DONE、31 退役保留、32/33/34 RELEASE_DONE、35 DISPERSE_DONE、36 LANE_DONE、37 DISPERSE_READY、38 CLUSTER_CAPTURE_AUDIT、39 SAFE_SWEEP、40 SAFE_SWEEP_DONE、41 BOUNDARY_RECOVER、42 SAFE_SWEEP_APPROACH、43 SAFE_SWEEP_AUDIT、44 SAFE_SWEEP_RETRIEVE、45 SAFE_SWEEP_RETRIEVE_AUDIT、46 SAFE_SWEEP_RETRIEVE_FAILED、47 CAPTURE_RETURN_WAIT、48 TRANSPORT_AUDIT、49 INJURY_BACKOFF、50 INJURY_RECLOSE、51 RECOVERY_WAIT`。
- TASK_COMPLETE后的顺序为mode16本地后退、mode17执行RETURN H/D、D=0后mode3。普通SEARCH完成120°和90°两圈仍无目标时，同样允许上位机在mode3发送RETURN_CENTER，使空爪底盘进入mode17回中心。mode3后的1500 ms交接窗口仍会确认重复RETURN帧SEQ。 正式mode16内先S1抬55°，后退完成才放回85°，两个升降阶段各等1秒；S3保持120°。
