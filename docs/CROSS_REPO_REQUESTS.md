# 两端当前要求与回执

更新日期为 2026-11-13，时区 Asia/Shanghai。

## 固定入口与协作方式

- F405 仓库为 `kkkkkkkkkkkkkkkk222222/F407-Rescue-Robot`，分支为 `main`。[本端固定入口](https://github.com/kkkkkkkkkkkkkkkk222222/F407-Rescue-Robot/blob/main/docs/CROSS_REPO_REQUESTS.md)
- RDK 仓库为 `danmo-teng/shijue_fangan`，分支为 `codex/gamepad-teleop`。[对端固定入口](https://github.com/danmo-teng/shijue_fangan/blob/codex/gamepad-teleop/docs/CROSS_REPO_REQUESTS.md)

涉及相关跨端接口或用户通知更新时，双方 AI 直接读取对端入口，核对源提交和请求 ID，
在自己仓库记录当前相关要求及审核/实现回执。旧请求按 Git 历史取证，不自动恢复为当前任务。
发布、读取或镜像要求不等于审核通过或实施授权，也不建立常驻自动修改服务。

接收方先审查自己的当前代码，向人说明问题、最小方案、成本和两端影响，可接受、缩小、替代或拒绝。
人保留知情、拒绝和选择权。已经明确批准的范围直接推进，超范围的重要取舍先由人决定，AI 不能互相授权。
不得擅自新增新鲜度、反复握手、资格等待、停车/FAULT、高频巡检或无效测试。
持久化规则见 [AGENTS.md](../AGENTS.md)，对端规则见 [CHANGE_WORKFLOW.md](https://github.com/danmo-teng/shijue_fangan/blob/codex/gamepad-teleop/docs/CHANGE_WORKFLOW.md)。

## 2026-11-13 step8清障1m标准直行（未烧录）

按用户本轮要求，仅0x14/step8切到Motor_MoveHeadingLowRearDeadband，与投送直推共用固定航向入口。
原定距分支已有角度PID，本次实际差异是采用标准向量斜坡/固定U保向及0.5mm/s后轮纠偏门槛，不新增或调节PID。
仍用原前两轮有符号反馈差计满1m，原刹车结束后锁一次轮计数原点；重复/暂停继续不重设，巡航300、原末段减速/3mm容差保持。
20cm寄存前进/后退、其他分段、定位速度、舵机、协议/示例/动作归属和扫障入口条件不变；本轮不处理上一次仅分析的卡住候选。
NormalRun编译及原夹具相关调用核对通过，未新增测试程序、未烧录/动作、板端actual未知；代码构建dirty1保留原空白。
地面偏移/打滑效果仍需人工实车；合同/JSON的执行说明已更新，无须上位更改报文或路线。

## 2026-11-13 定位速度后续调整（未烧录）

用户新指定定位平移最低250mm/s、原地回正200～400mm/s轮周速度。
当前预备点/入口视觉横移250～360、返中心/藏点末段250～400、新0x14动态定位250～350；
旧沿迹扫障返位因原上限240低于新下限，最小调整为250～250。其他平移上限不变。
回正使用独立Task常量，普通有限TURN及MotionDebug原速度不受影响；PID/PWM/舵机/流程与原协议字段保持。
回正复用原角度比例/死区，独立应用400上限，不受普通NAV内部300封顶影响。
仅做NormalRun构建及数值/差异检查，不新增测试程序或新门控，不实施先前扫障卡住分析候选。
本轮未烧录、未动作、actual未读；初版13步的离线验证记录与后续调参区分，当前合同/JSON已同步数值。

## MATCH-BULLDOZER-20261113：已实现分段接口，未烧录

本端基于`1b74001bb05e6ce0487fe9a96c0214b61ac72776`增量实现，保留该提交的找物近中心减旋及新机械标定。
已读取RDK固定入口、详细流程/机器合同，并核对固定源码快照`4e9ff10f50f1e9e6eeb77ecee9b6ff3d286ac296`。
合同blob为`fc72bed22425e3e628108a1cffff5c11137d9b8d`；本端实际板端身份本轮未读取，不能把该源码或构建当actual。

**审查结论：接受原0x18/opcode0x14的13步及原0x1D/0x1E配对，不更换编号、布局或双方分工。**

- 接收器独立解释P1高四位step，不套旧聚堆/STAGE/左右标志；26组合同示例保持原字节。
- TASK_REMOTE_ACTION/mode25执行；本地200mm/-200mm/1000mm轮程和连续yaw各只启动本次动作，重复不重开。
  四个动态段同action更新A=D、B=场地H；U为固定车头航向，不混用。D>0不报完成，D=0结束，不伪造到点。
- 现有Fine没有显式U输入，补共用Fine+绝对航向薄封装，继续原向量斜坡/换向/航向PI/PWM，旧调用仍fine=false。
  平移沿原200～350mm/s范围，固定段前进300、倒退350；连续yaw沿原120～250mm/s轮周速度/3°条件。
- step2只普通开爪120°放货，完成才清原审核/STABLE/旧扫障角色及已放弃批次预算，不自动后退/转弯/合爪。
  step7全开180°直接物理端点500/2500µs；step9原S1=61°抬起；step12恢复S1=85°、普通开爪120°、相机120°。
  复用原舵机到位估算，已完成姿态不重新启动等待；角度/READY是命令状态，不冒充轴角反馈。
- step12回mode3并保留本次task/action/opcode/COMPLETED，重复只ACK；原0x09→整爪审核→GRAB继续。
  空爪SEARCH可接受新的step0，包括旧RETURN上下文已取消的交接；不解除其他取消条件或从mode51重放旧动作。
- 不自动调用旧0x13，不新增时间资格、新鲜度门、握手、ISR、遥测或永久故障；不改PID、占空比、其他导航/审核/计数。
  人工STOP/ABORT/PAUSE及原mode51保留；新连续转向不套旧有限TURN/远程宏整体超时，有限轮程沿原失败恢复。

必要结果：NormalRun/MotionDebug构建通过；复用原Task夹具直接调试调用红蓝13步及合同26组CRC/字段，
重复/实时H-D/D=0、放货清审核、全开/抬爪/READY/APP、取消RETURN后的SEARCH、PAUSE继续及ABORT符合预期。
原测距/复审流程及motor绝对航向检查通过；新Fine保留绝对目标，旧接口fine=false。未新增测试程序，原夹具仅补共用API替身。
原完整NORMALRUN_FLOW_REVIEW_CHECK在固定320ms收爪断言失败，修改前1b74001同样重现；本轮未改该旧断言，不称整套全过。

**未烧录、未启设备或执行动作，板端actual未知。** 最终源码见本文件对应Git提交；本机保留原Task初始化空白，构建如实dirty1。
部署须使用NormalRun/TASK并读回真实身份，RDK再显式启用`RDK_ENABLE_BULLDOZER=1`或`--enable-bulldozer`；本端未修改上位开关。
20cm/1m、±3cm及贴栏扫除/正常找回、联合mode51恢复待人安排实车，不把软件DONE当精度证明。

- [完整下位合同及检查边界](F405_BULLDOZER_STEP_20261113.md)
- [机器合同、字段、动作参数及原26组报文](contracts/f405_bulldozer_step_20261113.json)
- [固定上位任务](https://github.com/danmo-teng/shijue_fangan/blob/4e9ff10f50f1e9e6eeb77ecee9b6ff3d286ac296/docs/COMPETITION_BULLDOZER_20261113.md)

## 2026-11-13 RDK661bdf7固定矩形过道配套审查回执

已直接读取RDK固定入口，并通过实时Git核对`codex/gamepad-teleop`最新为
`661bdf70f0fc1da5792c8cfd9a19da2825de36c4`；本端审查基线为`9b9508660e297c0bc0739c37a75f3df1dcab335f`。
[上位当前交接](https://github.com/danmo-teng/shijue_fangan/blob/661bdf70f0fc1da5792c8cfd9a19da2825de36c4/docs/CROSS_REPO_REQUESTS.md)
[固定矩形说明](https://github.com/danmo-teng/shijue_fangan/blob/661bdf70f0fc1da5792c8cfd9a19da2825de36c4/docs/COMPETITION_CLAW_RECTANGLE_20261113.md)

**审查结论：本次上位修改无需下位运行代码配套，现有接口可直接使用；没有为形成代码改动而修改固件。**

- 661bdf7仅改变RDK画面过道：固定爪模板左右边界及near727，取消旧像素障碍自持缓存，修正完成后才判过道。
  已核对实际diff，protocol.py和state_machine.py相对7240f0c均未变。下位不接收矩形/像素多边形，无需移植此几何。
- 原0x04视觉ALIGN仍输入像素偏差，原0x05仍为锁航向、本地轮程670mm直推；3°/6°及64/32px完成规则可兼容。
  不将矩形宽度/near像素变为下位距离，不加新握手、等待、停车或永久FAULT，也不改既有速度/PID/舵机参数。
- 7240f0c完整爪内审核仍用原0x12类别/数量/危险位及STABLE；现下位已有两帧、至少100ms采样间隔和原非法分离/释放入口。
  首绿单绿、后续最多两件材料、单伤员及initial-stash非空语义保持，不能以锁定单目标替代审核总数。
- 正式RDK不再生成opcode0x13旧清障，下位只有收到该命令才进入；保留历史兼容分支，不新增拒绝门或恢复旧清障发起。
  真实过道仍阻挡时，上位继续保持载荷等待；新绕行/清障未实现，不宣称有障碍也能完成全流程。

仍属上位的问题，不能靠下位放宽条件来修：

1. RETURN恢复观察只检查mode3/RETURN/accepted，未核对本次pending task/action；随后adopt旧context并强置matches，
   可把旧RETURN完成误作新返程完成。应在上位约束新旧动作身份并处理本次明确拒绝，而非改下位到点条件或伪造DONE。
   [runner1833](https://github.com/danmo-teng/shijue_fangan/blob/661bdf70f0fc1da5792c8cfd9a19da2825de36c4/competition_rescue/run_competition_rescue.py#L1833)
   [恢复消费2063](https://github.com/danmo-teng/shijue_fangan/blob/661bdf70f0fc1da5792c8cfd9a19da2825de36c4/competition_rescue/run_competition_rescue.py#L2063)
2. 视觉ALIGN保存源frame后，组0x1D时仍仅对APPROACH/AUDIT写frame，ALIGN被置0。
   当前下位兼容0所以不据此断言必然卡住；但真实帧关联丢失，应在上位保留已有ALIGN源frame，不新增字段或时间门。
   [组包1729](https://github.com/danmo-teng/shijue_fangan/blob/661bdf70f0fc1da5792c8cfd9a19da2825de36c4/competition_rescue/run_competition_rescue.py#L1729)

本次检查为实际源码/diff及现有调用链对照，没有新增测试程序，没有构建、烧录或连接设备。
仅更新本端固定回执和当前摘要；当前本地NormalRun镜像仍为9b95086/dirty1，文档HEAD不作为新镜像或actual。
RDK提供的最近实跑13b5cc7/dirty1属于既有会话记录，不冒充本轮板端实读。两端实车效果尚未联合验证。

## 2026-11-13 定位平移最低请求调整为200mm/s

用户最新要求将已确认的定位平移下限150提高到200，以上一源码`13b5cc7f174dc8746a8395af7a78224eb256f37f`增量修改。
当前为STAGE/入口视觉200～360、返中心/藏点末段200～400、扫障返位末段200～240mm/s。
只改三个最小请求参数，上限、D=0到位、任务配对及各控制函数保持；不抬高单轮、PID纠偏输出或PWM下限。
原地回正120～250、粗段120/600、直推及最新舵机参数不动。协议布局与既有能力/遥测不变，上位无须改报文。
NormalRun编译通过；未烧录、未实车、未读板端actual，无新增测试程序或诊断/资格门。以下150版本记录仅供追溯。

## 2026-11-13 用户指定NormalRun末段平移速度

从本端`5ac69330f1fdc7a56848ec30541b5b64ff09574d`增量修改。用户在150mm/s的MotionDebug侧移观察后，
要求NormalRun定位平移微调最小请求改150，各自原最大请求翻倍，并明确原地回正保持原值。
本次是新的明确要求，不恢复此前“最小翻倍、最大加200”的旧调参。

| 场景 | 原请求范围mm/s | 新请求范围mm/s |
| --- | --- | --- |
| STAGE预备点全向FINE | 110～180 | 150～360 |
| 安全区入口视觉横移 | 110～180 | 150～360 |
| 返中心/藏点及兼容NAV末段 | 120～200 | 150～400 |
| 扫障沿迹返位末段XY修正 | 110～120 | 150～240 |

- 预备点和视觉横移仍共用Fine速度范围；返中心/藏点使用独立末段下限，扫障返位有独立范围。
  原粗段120基数及600上限保持，避免修改共用参数抬高整个导航过程。兼容NAV近点范围同样落实150下限。
- H/D、像素速度增益、D=0到位、task/action、红蓝航向、进入安全区/直推以及合爪审核保持。
  原地回正仍120～250mm/s轮周速度，保航向PI和限幅不改。未加定时门、握手、故障或控制协议。
- PWM基础前馈10%、总上限60%、Fine按速度缺口补偿和PI不改；视觉横移仍用原MoveAngleHeading，不自动获得STAGE的Fine补偿。
  MotionDebug的侧移专用PI/起步助推没有搬入NormalRun，不能把刚才调试的DONE当作本次正式路径的实车效果通过。

必要检查仅NormalRun构建及实际diff/调用链；既有STAGE夹具期望同步150，无新增测试程序。
本轮未烧录、未连接硬件或执行动作；最近亲自校验烧录并串口测试的是MotionDebug5ac6933/dirty1。
本次源码、NormalRun镜像和板端实际状态分别记录。上位不用改报文字段，新请求范围供后续联合实车观察。

## 2026-11-12 用户撤销修正速度提速

用户实测反馈“最小值翻倍、最大值加200”的提速影响此前已调好的表现，并确认原先转不动是物块卡轮、现已解决。
本轮从`3fb95d5baf3afa3734c168d4c46f3abd2436c1f9`增量反向撤销该次提速，运行代码恢复到其父提交
`ea99f310f9aa8237c20dd8c8b70ace1dd06795a1`，没有回退此前连续回正等成果，也未把旧提速建议恢复为当前要求。

| 修正阶段 | 恢复后的请求范围mm/s |
| --- | --- |
| STAGE预备点细调/安全区视觉横移 | 110～180 |
| RETURN末段 | 120～200 |
| 安全区ALIGN连续回正（轮周切向速度） | 120～250 |

同时撤掉为提速新增的RETURN、ALIGN、扫障独立速度宏及绕过nav_yaw通用上限的计算，恢复原调用。
保留原3°/6°航向条件、64/32px滞回、纯yaw连续回正、同动作更新/完成撤回和ENTER交接。
PID、PWM公式/60%总上限、粗段、扫障原110～120mm/s倒车、670mm直推和独立舵机标定保持。
原H/D、像素、task/action及线上报文不变，上位无需为本次回退改协议；机器合同同步恢复速度说明。

必要检查仅NormalRun构建及差异核对：运行代码/既有STAGE速度期望精确恢复到ea99f31，
工作区原有舵机/工具/IDE改动和暂存项保留，未新增测试程序、时间检查或停车条件。
本轮未烧录、未打开串口/设备、未执行实车动作；完整本机镜像仍包含独立舵机改动，source_dirty=1。
上一轮开爪往返已校验烧录的MotionDebug镜像为3fb95d5/dirty1，不能将本轮源码或构建写成板端actual。
用户反馈是本次撤销提速的依据，本轮编译不代表新的地面效果验收。

## MATCH-SAFE-VISUAL-20261112 实施回执

本端按用户“等待上位机本轮完成后再适配”的要求暂停候选；上位提交完成后，重新核对
`a88b031bd743493b2fd6db0d8ddd1f8d157ac834` 固定入口和正式改动，从本端 `c7a9e7a4e03af5516e130839d06c7349375c6020` 增量实施。
用户在本端明确选择安全区回正/横移范围，并选择ALIGN使用连续回正，避开该场景旧有限定角10秒超时。
[上位源交接](https://github.com/danmo-teng/shijue_fangan/blob/a88b031bd743493b2fd6db0d8ddd1f8d157ac834/docs/CROSS_REPO_REQUESTS.md)
[更新后的机器合同](F405_DELIVERY_OPENING_20261112.json)

实际运行修改仅在Task.c安全区ALIGN及原完成状态消费处。

- 航向判断前移到横移及done分支之前。初始/回正后主yaw误差≤3°才进入后续动作；横移中>6°先停止平移、恢复法向。
  像素已达标时也要求yaw≤3°才完成，不能以6°保持带代替最终到位条件。红90°、蓝270°及原64/32px滞回保持。
- ALIGN回正改用原 `Motor_Move(0,0,yaw)` 连续输出，而非有限 `Motor_TurnAngle`。
  幅度取现有nav_yaw并夹在原120～250mm/s慢/快转速范围；复用原场地yaw→轮空间符号换算，没有反转主gyro或修改PID/PWM上限。
  回正时只发纯yaw，横移时仍只用原MoveAngleHeading内部航向保持，两者按阶段执行，不并行叠加第二个yaw输出。
- 持续回正没有新定角超时/FAULT，物理硬卡时仍可能不动；保留原人工STOP/ABORT及其他任务规则，未加退让/ESCAPE或假到位。
  其他有限转弯和原超时保持，不能把这次改动解释为全局删除异常行为。
- 视觉包只更新像素及滞回请求，不直接置done。已有初始定位/相机准备完成后，偏航恢复期间也能接受原视觉动作的实时更新。
  同task/action回正后继续最新像素，不重开相机/载荷审核，不重新执行初始300ms相机准备，不加帧等待或新鲜度门。
- mode10/11、0x1E COMPLETED和ENTER统一使用实际ALIGN完成结果。若完成后主yaw又漂出3°，先清旧完成，不能让排队的ENTER消费它。
  重新回正完成后原ENTER重发可沿原路径接受，不重开已接受的直推。

上位a88b031已将聚堆对准/测距统一为同锚点；本端原APPROACH/CLUSTER及mode52/53字段可兼容，无需新增接口。
这次没有实施聚堆近中减旋/阈值修改，未改STAGE、670mm直推、Fine助推、扫障、ROI或上位矩形1/3、2/3算法。
原0x18/0x1D/0x1E布局、CRC、mode/service与波特率不变，仅更新上述执行/完成语义。

必要检查实际结果

- NormalRun编译、链接通过，F405 / mode5 / TASK / UART4 921600/8N1。工作区独立舵机倒装/标定保留，完整本机镜像dirty1。
- 原Task/mechanism夹具验证红蓝及正负初始偏航4组，回正发非零纯yaw，未调用有限TURN。
  3°/6°/6.001°边界、偏航时无平移、恢复后接最新像素、像素32/yaw4不能完成、yaw3可完成及完成后漂移撤回通过。
- 完整Task_Process模拟11s/22s航向未恢复仍保持ALIGN/yaw请求，无有限TURN超时FAULT，人工ABORT仍停车；这不是实车硬卡或持续发热测试。
  原PAUSE/同动作恢复、相机/载荷保持及6项既有导航/ENTER/扫障/ABORT/mode51相关回归通过。
- 固定源上位6项既有直接检查在本机通过，覆盖新同锚点/重关联/定距锁存以及原视觉横移、64/32px、运输复审。
  现有审核夹具仅补同一红蓝ALIGN用例的偏航/完成断言，未新增独立测试程序、遥测、ISR或高频统计。

本轮未烧录、未打开串口/设备或执行实车动作。此前串口排查曾SWD逐字节核对c7a9e7a/dirty1镜像，但不是本次新实现的实烧证明，
当前未重新读取actual。新候选身份以构建产物为准，不能用源码提交或本地镜像冒充板端已更新。
实际控向、机械卡料、像素几何和地面投送效果待原入口实测；本次不宣称全部摆头或卡料已解决。

## MATCH-TERMINAL-DRIVE-20261112 区域到位配套回执

已读取RDK最新 `800ea4447353e3a979334bdb0de7f1dc8bd75563` 的固定入口及实际代码，
也确认其已读取本端bbb719c答复。上位本次用户批准的是独立STAGE区域到位与末端轴分量选择，
未选择普通RETURN切Fine；不将旧候选恢复为实施任务。
[源交接](https://github.com/danmo-teng/shijue_fangan/blob/800ea4447353e3a979334bdb0de7f1dc8bd75563/docs/CROSS_REPO_REQUESTS.md)
[本端详细核对及例子](F405_TERMINAL_DRIVE_REPLY_20261112.md#六上位区域到位方案的配套核对)

- 当前上位STAGE为X±100mm/Y±50mm；原通用/藏点横向范围及其他返程保持。几何距离≤180mm时只修超限轴，
  同task/action的D/H是剩余修正向量，车头仍由下位原gyro保持，两轴均到位才锁D=0。
- 已核对当前F405代码兼容，无需修改运行代码。D=70的STAGE仍会修正，不能借RETURN的80mm条件提前到位；
  原D=0停机/抬相机/完成及mode48复审保留。已在FINE时H/D更新不重开粗段，未增加协议、到位判断或时间门。
- X=90/Y=40mm可到位，90/70仅纵向修正，110/40仅横向修正；110/70保斜向，90/200在180mm外保原几何H/D。
  上位的D不再始终是点到点欧氏距离，本端消费方式不受影响。原110～180mm/s Fine、法向/视觉ALIGN及670mm直推保持。
- 上位固定提交的8项现有相关检查通过。80组实际编码/CRC校验报文字段送原C Task夹具通过，
  覆盖红蓝、两种目的地、正负方向和单轴/双轴/几何/D=0交接；运动mock不当作真实轮运动。

本轮只更新对接说明，无需新固件构建或烧录；未打开串口、设备或启动车轮。实际板端身份未读，
本地现有镜像仍0d52ef8/dirty1，文档提交与实际镜像分开。独立舵机标定和其他工作区改动保留。
区域参数是工程初值，实际卡顿/起转/视觉投送仍待原入口实测；未采用RETURN切Fine、未改PID或新加保护。

## MATCH-RETURN / MATCH-TERMINAL-DRIVE-20261112 核对答复

已读取 RDK 最新 `6edf4a0dd86264c201554d952a3128e66f6a57ab` 固定入口及实际返程修改，
并对照本端 `0d52ef81c30541ea69c5f13b53b3509cfe643e2f`。
本轮是对接及方案答复，不是末端驱动实施授权，未修改固件。
[完整答复与源码依据](F405_TERMINAL_DRIVE_REPLY_20261112.md)

- **新返程点可兼容。** F405消费原H/D，不在RETURN路径写死(0,0)。mode16仅缓存/ACK，不跳过原本地退出；
  mode17收到D=0沿原完成进SEARCH。上位投送后15cm内锁定D=0，已覆盖下位原D≤80mm条件，不需全局改下位到位范围。
  开局藏点/抓取放弃/搜索失败的返中保持原语义，未把投送后偏置点扩到这些路径。
- **末端请求已是120～200和110～180mm/s。** 普通RETURN用MoveAngle，没有Fine附加项；STAGE本来就用MoveAngleFine。
  Fine将单轮零速带改为0.5mm/s，并按实测速差补偿，最多150‰，总PWM上限仍600‰。不能说预备点缺Fine。
- **没有唯一根因证据。** 非零120mm/s平移的三轮分解不可能全部落在6mm/s零速带内；不能仅用死区解释三轮不走。
  静摩擦/负载与反复换向仍需同段实际TARGET/PWM/ACTUAL/RAW32及H/D、动作身份区分。
  当前API接受、motors_active或PWM命令均不是轮子已运动的证明；既有输出不含reversing内部标志，不能伪称已观察到该分支。
- **最小建议待人选择。** 先用上位已交付的投送后15cm到位，不扩大STAGE窗口、不整体加速。
  若返程仍在容差外停，固件候选只把RETURN/FINE调用改成现有MoveAngleFine，保留原120～200mm/s、粗段和PID。
  不推广到藏点/所有动作；它不能解决已用Fine的STAGE，也不消除反复换向，具体代价见完整答复。本轮未实施候选。

固定源上位3项现有相关检查及原下位夹具的红蓝mode16/17、D=0/FINE交接通过。
本轮只改文档，未重新构建/烧录、未开串口或启动车轮；现有本地NormalRun仍为0d52ef8/dirty1镜像，此刻实板actual未读取。
独立舵机标定、工具和IDE改动保留。回复发布后供上位读取，不代表上位已接收/认可或现场问题已验收。

## MATCH-DELIVERY-OPENING-20261112 实施回执

本轮已核对 RDK `c907be05ecd8231c3b372b7593565948597bd376` 的固定入口、正式实现、接口JSON及检查说明，
从本端 `19ec9f718d0cdb9431748c89d18b229d9251063b` 增量实施三项。
[源交接](https://github.com/danmo-teng/shijue_fangan/blob/c907be05ecd8231c3b372b7593565948597bd376/docs/CROSS_REPO_REQUESTS.md)
[本端机器合同](F405_DELIVERY_OPENING_20261112.json)

- 开局接受取消地图中心收敛前提。原后退、180°、开爪、1.10 m本地轮程保留；快速前行的本地DONE后直接走原开爪/SEARCH，
  原140°静止预检接手。地图位置不能提前截断轮程，也不再产生START_CENTER_APPROACH或要求中心80 mm内。
  原速度/PID/超时保持；粗段后的位置偏差交给视觉，不把轮程DONE当成地图中心/地面距离真值。
- 视觉ALIGN改为实际横移。先沿原定位ALIGN由主gyro对红90°/蓝270°法向；随后正像素误差请求MoveAngle -90°右移，
  负误差+90°左移，原 `Motor_MoveAngleHeading` 负责斜坡和锁法向。速度夹在已有110～180 mm/s，1 mm/s/px是工程初值。
  初始绝对偏差≤64 px不启动，>64 px启动后≤32 px结束，完成后64 px内不重开。
  对准中报mode10，完成报mode11；新大偏更新清旧completed。ALIGN复用动态命令及原源帧序处理，
  同task/action更新像素和源frame，定位ALIGN重发不重开，相机0/11500选择保持；PAUSE后原动作接续。
  未加入第二套航向PID或ENTER融合XY纠偏，原0.67 m直推与扫障恢复保持。
- 推进前闭爪的真实TASK_ALIGN_SAFE_ZONE可用原CARGO_AUDIT占位请求回mode48，包含横移中的mode10。
  仅接受零frame/零证据/非STABLE及原目的地角色，进入即停横移并保持载荷，沿原相机准备和真实两帧复审。
  同动作重发不重启相机；非法载荷沿原分离，合法后重新ALIGN。未扩展任意未完成NAV、ENTER或已开爪/放货状态。

旧0x18/0x1D布局、CRC、task/action和串口波特率均保持。视觉ALIGN执行语义变更及示例见机器合同，RDK须配套部署。
本轮新增3个独立像素/速度映射参数，原电机PID、全局速度、位置/轮速输入和两件/伤员审核保持；无新鲜度/年龄门、握手、
定时等待、ISR、遥测、保护停车或永久FAULT。工作区独立舵机倒装/标定和工具/IDE改动保留，没有夹带提交。

必要检查实际结果

- NormalRun完整编译/链接通过，F405 / mode5 / TASK / UART4 921600/8N1，本机完整镜像含独立舵机设置，dirty1。
- 原Task/mechanism夹具通过开局轮程完成才交接、地图114 mm/中心/缺失均不改变轮程、红蓝两侧横移/原ENTER交接。
  正负像素、64/65/32 px滞回、同动作实时帧、原帧序去重、旧completed撤回、PAUSE/恢复和未完成不可ENTER均通过。
- 真实ALIGN的mode10/11回140°/mode48、目的地/重复请求、真实审核再ALIGN和不相关NAV/ENTER/开爪负例通过。
  伤员＋绿色在复审中仍非法，并进入原运输分离。原空爪、扫障、载荷保持、导航/ABORT和mode51相关回归通过。
- 固定提交上位机9项相关状态机检查在本机通过，覆盖实时横移/局部直推、平台混装、开局、运输、mode51及两件找回。
  上位机原始图像/历史异类证据的21项整组检查只据其报告，本机未把这些图像回放写成实车验证。

本轮未烧录、未开串口/设备或启动车轮，未亲自读取板端actual。本场RDK报告的19ec9f7/dirty1为该场日志身份，
不能替代此刻实板读取。必须同时部署RDK本轮实现与新NormalRun，旧固件的ACK不代表执行横移。
实际横移方向/速度/阈值及粗段后视觉拾取仍待原入口实车确认；定位跳变、弱雷达约束及mode22尾部等待未在本轮解决。

## MATCH-OPENING-CAPTURE-20261111 实施回执

本轮按用户要求读取 RDK 最新固定入口与实际实现，源提交为
`bf92176fc28f5954a17d757e374a6f010ab37abe`，本端从
`7653af9b5f44ff18a1aa268eaf70d125ff0ee389` 增量修改。
[源交接](https://github.com/danmo-teng/shijue_fangan/blob/bf92176fc28f5954a17d757e374a6f010ab37abe/docs/CROSS_REPO_REQUESTS.md)
[上位机实现与证据](https://github.com/danmo-teng/shijue_fangan/blob/bf92176fc28f5954a17d757e374a6f010ab37abe/docs/OPENING_CAPTURE_FIX_20261111.md)

本端审查后接受两项，沿用已有相机、审核和恢复流程。

- 开局张爪的 SEARCH 接收原 CARGO_AUDIT 视角请求后，静止走原 mode20→21、140°观察。
  不启动200 mm补进、不猜载荷、不自主合爪。重复请求只确认，不重启相机或增加样本。
  真实两帧/STABLE非空后等原GRAB、合爪后复审及藏点NAV；真实稳定空爪则回SEARCH，原action为completed、非cancelled。
  同action的完成重发不重开观察，旧action继续按原身份规则拒绝。
- 初次普通/cluster空爪补进原来每周期清掉 `capture_wait_active`，使原4秒出口无法在持续空爪时运行。
  现在首次空爪时启动原 `capture_wait_started_ms`，后续空帧不重置，达到原 `APP_CAPTURE_DECISION_WAIT_MS=4000` 后走原恢复。
  原200 mm行程额度保留。代价是持续无货/无位移时更早放弃重找，不再等待原更长审核期限。
- 预检状态在初始化和离开预检流程时清理；进入普通接近不继承预检静止标志。
  本轮只改Task运行逻辑及既有夹具的4秒预期，不改电机PID、全局速度、两件审核、返程、ROI或模型，未新增计时器/握手/FAULT。

原线上入口的扩展语义

- 仍使用15字节原0x18及0x1D同SEQ配对。opcode=0x12，command flags仅VALID=0x01，audit flags仅INITIAL_STASH=0x01。
  左右类别/计数/总数全0，STABLE为0、source frame=0。audit_id不作为观察样本。
  仅本轮规定的开局SEARCH/张爪入口启动观察，不把运输或抓后审核的占位解释成预检。
- SEQ=0、task=7、action=11、frame=0、audit_id=0的已编码示例，先发context再发audit，CRC沿原MODBUS低字节在前。

```text
context  a3 b3 1d 00 00 07 00 0b 00 00 00 00 33 69 c3
audit    a3 b3 18 00 12 01 00 00 00 01 00 00 31 ad c3
```

必要检查实际结果

- `cmake --build --preset NormalRun --target NormalRun` 构建通过，F405 / mode5 / TASK，UART4 921600/8N1。
- 复用原Task/mechanism夹具及Task_Process，预检相机推进/重发保持静止、真实空爪完成及旧action拒绝、新目标交接通过。
  真实非空→GRAB→mode23复审→mode22→藏点NAV，零frame/STABLE拒绝、ABORT/初始化复位及非开局/运输入口不误启动均通过。
  原普通/cluster补进4秒与200 mm边界、航向路径及5项已有扫障/合法载荷/导航/ABORT/恢复回归通过。
- 固定源上位机7项状态机相关检查在本机通过，覆盖开局空/非空、藏点、原启动、定距、mode51及运输回执。
  没有新增独立测试程序或把对端自行报告的真实发布器检查冒充本机执行。

本轮未烧录、未打开串口或启动车轮，未亲自读回当前实板身份。RDK晚间日志提供的7653af9/dirty1只是该两场历史实跑身份。
本机完整构建保留工作区独立舵机倒装/标定，`source_dirty=1`；这些侧边改动未夹带进本轮提交。
旧固件没有开局SEARCH观察入口，需与RDK本轮实现同步部署后联调。现场相机图像和开局收益尚未验收。
690条零轮增量/gyro静止期间的具体电机输出或机械原因仍缺PWM证据，不能将本轮流程出口修复写成已查明该唯一根因。

## MATCH-ABE-20261111 实施回执

用户本轮要求读取上位机最新交接并适配。本端核对 RDK
`9b810d05158995bb19deba5386eb3804e61f305c` 的固定入口、A/B/E说明、实际代码和参考补丁，
从本端 `ad1cc2db3d656151dcd70a52d77f7c7b8a7d46a7` 增量实施，保留此前扫障相机时序和合法载荷保持。
[源交接](https://github.com/danmo-teng/shijue_fangan/blob/9b810d05158995bb19deba5386eb3804e61f305c/docs/CROSS_REPO_REQUESTS.md)

- A 接受几何修正。原 DRIVE 在纵向已到端点、横差仍超限时继续倒退，确有不能直接消横差的问题。
  现按锁定线段计算场地纠偏向量，进入原纵向容差/越过端点后纵向分量为0，仅消超限横差。
  转当前车体系后调用原 `Motor_MoveAngleHeading` 保持锁定航向，取消调用方额外偏航输出。
  非零纠偏使用原 NAV 最低110 mm/s及本分支原120 mm/s上限，完成容差、轨迹消费、转向和500 ms相机交接保持。
  API接受仅代表运动请求已交给电机层；未接受也不伪造到点或新增FAULT。贴物横移比旧20 mm/s请求更快，实际效果仍待现场确认。
- B 已核对上位机脚点/实时色框及39→42目标重绑修复，继续使用现有APPROACH/GRAB，不需要本端增加接口。
- E 修改 `task_audit_semantic`。首件前仍恰好单绿；首件后允许1～2件绿/黑材料，同侧或分侧均可。
  同侧绿黑仍用现有类别6，且该侧计数必须恰好2；两侧计数之和必须等于真实总数。
  单伤员须伤员目的地；危险蓝、未知、伤员混装、三件及计数/类别不一致仍非法。INITIAL_STASH仍按原真实非空语义。
  保留原两帧、STABLE、角色、task/action配对、mode51及实际找回一件即可继续的原审核路径。

线上仍为原 `0x18/CARGO_AUDIT` 和1D配对，8字节payload依次为命令、命令标志、左类别、右类别、两侧2-bit计数、
审核标志、audit_id、真实总数。字段布局、CRC及回执无变化，只更新上述正式材料合法性。
未改电机PID、全局速度、ENTER/EXIT、舵机角度或等待；本轮无补推、侧绕、新计时/握手/遥测或永久故障。

必要检查实际结果

- `NormalRun` 构建通过，F405 / mode5 / TASK / UART4 921600/8N1。
- 原主机夹具编译实际Task和mechanism。16组返程几何检查通过，覆盖横差正负、端点前后、四个场地方向和段游标只消费一次；
  电机API拒绝不伪造运动或新增故障的分支通过。原容差判断沿用。
- 固定源上位机的实际报文构造生成52组载荷，首件前后与本端C合法性全部一致。
  合法双件的两帧/重复帧、GRAB重发、23→22、48→ALIGN，以及38→37→GRAB、mode51审核检查通过。
  藏点非空、错误目的地负例和9项已有相关下位检查通过；旧非法“两绿”夹具改为“三绿”，未放开非法路径。
- 固定提交的6项上位状态机相关检查在本机通过，涵盖B、两件找回只剩一件、mode51、运输分离及延迟回执交接。
  不把上位机自行报告的图像检查或原始日志复算当成本机实测。

工作区独立舵机倒装/标定、工具及IDE改动保留，不夹带进本轮提交；本机完整固件仍包含这些设置，`source_dirty=1`。
本轮未烧录、未开串口或启动车轮，未亲自读取当前板端身份。RDK旧场次报告的 `e85c738/dirty1` 仅为历史实跑证据。
两件联调需要同时部署本端新固件和RDK双件实现；旧单件固件不能与其默认双件规则视为兼容。
现场mode39长停是否另有机械原因、两件保持与投送效果仍未验证，不能以离线通过声称整场验收。

## 2026-11-11 等待 RDK 完成后的流程配套

按用户要求，先等待 RDK 从仅分析的 `60073cb` 完成实施并提交，再开始本端修改。
本轮核对源为 `ed07e323e721649af19193dff1570d1147c2c89c`，RDK 已标记为软件交付、待实车。
固定入口仍保留早先请求，因此同时核对该提交的 PROJECT_STATUS 和
[本轮实现说明](https://github.com/danmo-teng/shijue_fangan/blob/ed07e323e721649af19193dff1570d1147c2c89c/docs/COMPETITION_FLOW_RESPONSE_FIX_20261111.md)。
F405 修改基线为 `e85c738075451574e4c29ea7da05daf5a8258b6b`，没有重套历史补丁。

审核结论与最小修改

- RDK 的 mode52 新图像帧更新、mode53 已接受定距去重、mode23/48 首次真实空图换角及后续 frame0 非 STABLE 占位、
  115° ALIGN、闭爪 ENTER 扫障与合法载荷保持，均可复用本端现有接口。没有新增协议或动作许可。
- 独立核对发现，两条扫障取回路径均在原 mode39 的 500 ms 等待结束时才命令相机转到 140°，
  随即进入 mode23 并报 claw_visible。RDK 移除重复 250 ms 等待后，这会使移动中的相机过早参加审核。
  本端将 140° 命令移到这段原 500 ms 等待的开始，等待期间 claw_visible 为 false，结束再进入原复审。
  覆盖视觉扫障和旧扫障路径，总等待不增加；没有新增定时器、年龄检查、遥测或停车条件。
- 上位机的载荷 ID/bbox 关联及 115°平台区域只属上位机图像处理，本端不重复实现、不修改模型或用户 ROI。

本机必要检查实际结果

- `cmake --build --preset NormalRun --target NormalRun` 编译、链接通过，F405 / mode5 / TASK，UART4 为 921600/8N1。
- 复用现有 Task/mechanism 主机夹具，9 项相关入口通过，包含两种扫障相机等待、角色/占位、合法载荷保持、
  原定距、导航/ALIGN、恢复及分离等待。另以同一夹具离线核对 3 组两端交接，均通过。
  这 3 组为 mode23/48 首空换角后 frame0 缺框及真实两帧恢复，mode52 新像素帧到 mode53 定距且重发不重启，
  115° 定位/视觉 ALIGN 及重复请求。
- 固定读取 `ed07e32` 的 6 项相关上位机状态机用例，本机运行通过。没有把该端自行报告的载荷图像/真实发布器检查冒充本机运行。
  本端已有 24 项串口协议检查通过。只在既有测试入口补扫障相机时序断言，未增加独立测试程序。
- 交付前已核对本轮差异。工作区中侧边对话的舵机倒装/标定、工具及 IDE 改动保留，未并入本轮提交。
  本机完整构建包含这些未提交舵机设置，`source_dirty=1`，与本轮干净源码提交区分。

未烧录、未接管串口或执行实车动作，未亲自读回当前板端 actual。RDK 这场报告的 actual 为 `e85c738/dirty1`，
只能作为其现场日志依据。离线通过不能证明实际摄像头到位、识别召回、载荷不误扫或整场流程已验收，仍需原入口现场复核。

## 2026-11-11 合法载荷等待保持

本轮读取 RDK 最新 `97235a3cb1dfdcabef8c4ac52399ea4841ea5a46`，其载荷换角关联和重复拒绝回执修复沿用现有接口，
无需新增 F405 协议。RDK 记录的 competition_10 实跑为 F405 `41f23a/source_dirty=1`，这是现场日志提供的身份。
[RDK 修复与录像依据](https://github.com/danmo-teng/shijue_fangan/blob/97235a3cb1dfdcabef8c4ac52399ea4841ea5a46/docs/COMPETITION_CARGO_SWEEP_FIX_20261111.md)

用户在 F405 侧确认继续修复上轮分析的合法载荷自动开爪问题。基于 `41f23a6`，`task_audit_watchdog` 复用原
`gripper_closed`、`audit_received`、`audit_valid`：已经确认合法且闭爪时结束当前审核失败倒计时，继续保持载荷和处理后续命令。
已完成的抓后/运输审核也保持结果，原定时换角流程只处理尚未形成合法审核的情况，避免先清空合法结果再触发放弃。
无合法审核或后来合法性被撤回时，仍走原有审核业务及恢复路径。

`NormalRun` 构建通过。现有夹具的合法等待检查已按新行为更新；针对 mode22、48、40 的离线核对确认，
合法载荷等待超过 60 秒保持闭爪，随后 NAV/ALIGN 可正常接受；无合法审核时原 15 秒恢复、合法性撤回后原倒计时及 ABORT 通过。
本机镜像同时包含工作区中独立的倒装标定，`source_dirty=1`。本轮未烧录、未重新读取板端身份或执行实车动作，
实际误扫减少和连续运输效果仍需现场复核。RDK 本轮修复在重新启动正式程序后加载。

## COMP-FLOW-20261110 实施回执

RDK 源提交为 `2046c0104311a5c5b958c72c0f8a8bd3b5df7a19`，F405 修改基线为
`2d12dca9480927287ea5e398a4cfdb693a2598cb`。[上位机交接](https://github.com/danmo-teng/shijue_fangan/blob/2046c0104311a5c5b958c72c0f8a8bd3b5df7a19/docs/COMPETITION_DELIVERY_FIX_20261110.md)

- mode23/48 已合爪且相机审核就绪时，首个 `vision_frame>0` 的真实非 STABLE 空画面提前使用一次尚可用的
  原换视角步骤。保持夹爪闭合，回 140° 后沿原审核。占位帧、重发和随后用于复审的空帧不会重复触发换角。
- 合法单伤员仍完成正常合爪审核，随后直接进入 `WAIT_NAVIGATION`。扫障原载荷的合法审核仍进入 `SAFE_SWEEP_DONE`。
- 普通 105°/120° 对准使用原始图像 X 的 576～704 px 含边界。刚出中心带时减小原地旋转请求，远处沿用原滤波。
  mode52 可凭新像素帧回原地对准；已接受的 mode53 定距沿用原去重及抬相机交接。
- 原 `0x18` 的 `ALIGN_SAFE_ZONE=0x04` 仍用原标志、像素和航向字段。`arg_b` 是大端有符号 16 位，
  `0` 选 120°，`11500`（`2c ec`）选 115°。定位和视觉两种 ALIGN 均接受，同动作的换视角更新复用当前对准结果。
- 用户追加确认，原 mode11 的 `CLEAR_SAFE_ZONE=0x13` 入口及闭爪 ENTER 的整个 mode10 均可扫障，
  包括前 40 cm 接近和后 27 cm 直推。接受扫障即停原推进，由原扫障流程暂放、取回载荷。
  恢复 ENTER 时扣除已走的推进里程，并扣除已用的末段直推时间，扫障行程和停留时间单独计算。
  普通 STAGE、开爪和放货确认状态沿用原拒绝回执。

**跨过开爪边界时的拒绝回执。** 如果上位机按上一条 mode10 状态发出 CLEAR，而板端收到时已进入开爪或放货确认，
F405 仍拒绝切换。RDK `2046c01` 的 `_reconcile_sweep_rejection` 未处理这一跨状态情形。若收到匹配的
`rejected_action_id`，可沿原动作身份接续 ENTER；板端已到 mode15 时接续投送确认。

`NormalRun` 编译、链接通过。下位机针对性离线检查通过，覆盖空框/占位/重发、伤员、±64 px 边界、mode52/53、
两种 ALIGN、接近和末段扫障、按剩余里程/时间恢复，以及开爪后拒绝。RDK 四项直接相关状态机检查和本端 24 项协议检查通过，
四组 ALIGN 报文逐字节相同。已有 `NORMALRUN_FLOW_REVIEW_CHECK` 在侧边倒装标定下的固定收纳时刻断言失败，
在修改前 `2d12dca` 的 Task 和相同标定上复现。本轮未烧录或进行实车试跑，交接场次实报 `d93367e/source_dirty=1`。
本机新构建包含工作区独立的倒装标定，`source_dirty=1`。

## SORT-01-20261110 渐进分离实施回执

RDK 实现为 `afe28fbe5138f2d8019f248e0d4f43142bc31e0f`。F405 从 `b56bc80` 起实施本回执所在提交。
已核对两端的保留侧语义和放弃交接，并经用户确认第三次起沿用第二次的弧线长度上限。
[RDK 请求](https://github.com/danmo-teng/shijue_fangan/blob/afe28fbe5138f2d8019f248e0d4f43142bc31e0f/docs/CROSS_REPO_REQUESTS.md#L23) ·
[RDK 实现说明](https://github.com/danmo-teng/shijue_fangan/blob/afe28fbe5138f2d8019f248e0d4f43142bc31e0f/docs/COMPETITION_SORTING_20261110.md)

- 抓前、合爪后及运输复审的已有带侧分离入口允许第三次及后续动作。F405 已移除接受时的总次数拒绝和可分离复审的总次数放弃。
  每次仍需本轮稳定审核和原任务/动作配对，重发沿用原去重。F405 总次数用于首次/后续夹紧档、诊断和原恢复分流，
  连续无进展由 RDK 根据动作前后的稳定审核计算。
- 弧线段里程上限为首轮 250 mm、第二轮及以后 300 mm。仍按审核选择 10° 或 15°，后续额外夹紧为 5°，
  复用原单侧开爪、20 mm 直退、退转、70 mm 尾段及 140° 复审。实际行程仍受原角度、侧偏和动作预算约束。
- RDK 连续两次无进展时仍走 `RELEASE_BOTH`。F405 沿原放弃路径双开、退让 300 mm、收爪并在 mode47 接原
  `RETURN_CENTER`。合爪业务审核、原无审核超时、正式单件运输、扫障/伤员/藏点规则及 mode51 继续走现有入口。
  `TARGET_RIGHT` 表示保留右侧，`RELEASE_RIGHT` 表示释放右侧。线上字段沿用现有协议。
- `NormalRun` 固件目标编译和链接通过。现有主机检查的累计两帧/第三次分离、mode48 分离后复审及退转退出通过。
  本机编译同时包含工作区里独立的 S2/S4 倒装修改，`source_dirty=1`。本回执的提交仅包含分离及对应断言，
  本轮未烧录，板端身份未重新读取，地面分离效果待验证。

## TRAJECTORY-20261109 审核回执

核对的 RDK 交接提交为 `0d7256800f0f21c334543499406370064a2d96af`，F405 行为基线为
`08559cd57bfd2b83145bbdb6240de14c71bf64b5`。交接内两场实跑为 F405 `9434608`、`source_dirty=1`、
NormalRun/TASK 和 RDK `4cfaba9`。新版行为待实车验证。
[RDK 交接](https://github.com/danmo-teng/shijue_fangan/blob/0d7256800f0f21c334543499406370064a2d96af/docs/CROSS_REPO_REQUESTS.md#L23) ·
[联合分析](https://github.com/danmo-teng/shijue_fangan/blob/0d7256800f0f21c334543499406370064a2d96af/docs/analysis/motion_competition_20261109/ANALYSIS.md)

- 已实现 `b980c6a` 的后轮小量纠偏、`37b4c00` 的开局限加速度和现有 `0x16` 补中心，以及仅限 STAGE 的
  FINE 110～180 mm/s。`08559cd` 按本地陀螺仪锁红方 90°、蓝方 270°，旧视觉 ALIGN 继续回执。
  RDK 仍使用原 STAGE `D=0`、载荷复审、扫障和 ENTER。
- 建议先以新版复核有载末段。旧版五次观察到 `D=0` 后 120 ms 的轮里程约 0.4～1.3 mm，
  到点前 100 ms 的速度中位约 14～72 mm/s。这组数据无法标定新版较高速的刹停。
  若仍有非零目标但轮速上不来，复用 `TSTAT` 的 `TD_DRIVE` 区分驱动输出与机械阻力后，
  再决定是否只调 FINE 的现有助推。若是跑过到点框，按有载实测刹停余量限制 FINE 速度。
  200～250 mm/s 上限仍为待验证提案。
- 旧实跑三次直推起点的估计 X 离半区中心约 5.5～6.9 cm。固定航向后的判障走廊仍取自安全区框，
  需用已有画面对照实际直推是否覆盖。先保持现有 RDK 和到点容差。若发现走廊偏移，再单独讨论现有
  走廊几何。若走廊覆盖且起点横偏，再单独讨论 STAGE 的 X 容差。
- RDK 的轮式已可疑后遇 T265 跳变仍回到轮式，是独立的定位来源问题。适合先按旧日志回放核对，
  来源策略由人另行决定。115 mm 有效轮臂用于对照诊断，当前固件沿用原控制参数。

本条为源码核对和提案回执。运行代码沿用已有提交，实烧身份沿用最近有证据的记录。

## NormalRun 分离数量档（沿用的机械参数）

用户批准先用现有审核报文的单侧 0–3 块。RDK 参考提交为
`347d787c6b42471d1265380141e1b77a540bd81c`。RDK 现有审核包已提供分档所需字段，可直接联调。
F405 实现提交为 `62be90bfb5c295bbc759740143e8c3928b587b4b`，NormalRun 已编译并链接成功。
本地镜像 `source_dirty=1` 来自原有的 Task.c 两行空白。本次未烧录，分离角度和成功率待实车验证。

- F405 从已接受的审核锁定释放侧数量和保留侧，第一次有明确合格单件可保留时，
  释放侧 1–2 块用额外夹紧 20°、向释放侧转 10°，释放侧 3 块用额外夹紧 10°、转 15°。
  保留侧多件或单件证据不足时，额外夹紧为 5°。第二次按复审重选转角，额外夹紧为 5°。
- 双侧非空、左右计数之和等于真实总数且没有 UNKNOWN 标记，才使用精确数量档。
  已接受的带侧动作若证据不足，采用 5° 夹紧与 10° 转向。继续或放弃见上方 SORT-01-20261110。
- 带侧 DISPERSE 与 RELEASE 加 YIELD 共用该选择。20 mm 直退后起弯，
  达到该档航向就收转向，沿用 70 mm 尾段和现有侧偏预算。角度是目标值，实车待验证。

RDK 可沿用当前代码直接联调。现有 `CARGO_AUDIT` 携带左右计数、真实总数和 UNKNOWN 标记，
单侧超过 3 块时现有编码已将侧计数饱和为 3 并置 UNKNOWN。
F405 在接受分离动作时，从已审核的左右计数和现有选边命令推算释放侧，内部计算夹角及转角。

联调时用现有日志核对稳定审核的左右计数、总数和动作方向是否对应。
带侧 `DISPERSE_PILE` 的 TARGET_RIGHT 表示保留右侧，`RELEASE_RIGHT` 表示放开右侧，
后者沿用原流程，开爪完成后发 `YIELD_BACKOFF`。分离后沿用现有 140° 复审。
上位机保留侧的独立单件例外已见上方 SORT-01-20261110。物堆边缘/缝隙靠近仍是独立提案。

代码参考 [RDK 审核编码](https://github.com/danmo-teng/shijue_fangan/blob/347d787c6b42471d1265380141e1b77a540bd81c/competition_rescue/state_machine.py#L295)、
[RDK 选边](https://github.com/danmo-teng/shijue_fangan/blob/347d787c6b42471d1265380141e1b77a540bd81c/competition_rescue/state_machine.py#L3059)、
[F405 分离准备](../Main/Src/Task.c#L2309)。

## 2026-11-09 正式流程离线复核

复核的 F405 固件源码为 `3b1da3836fdf20533d49f8b94cc5977d7b331cd0`，
RDK 为 `a8ff1a19e308efd84be0ab5c32a271e413e0184e`。本轮测试使用合成状态、报文和硬件替身。

| 流程 | 本轮结果 |
| --- | --- |
| CONFIG、启动和搜寻 | RDK 启动握手用例通过。F405 正常等待 CONFIG，模拟 45° 倒退、转身、225° 前进后进入 SEARCH |
| 普通/成堆靠近、抓取和藏点 | 105°/120° 测距交接、空爪限距、两帧和 STABLE、普通物资及藏点的抓后审核通过 |
| 分离、伤员和运输复审 | 10°/15° 镜像曲线、两次预算、末步等待 260 ms、伤员半闭/后退/复审/再闭合、mode48 分离再审核通过 |
| 导航、扫障和进安全区 | D=0 预备点、扫障审核角色、走廊有障碍发 CLEAR、无障碍发 ENTER 通过 |
| 投送、退出和返回 | 合成区外再区内证据发 COMPLETE，超时分支发 COMPLETE，F405 验收等待后进入退出，D=0 返回 SEARCH |
| 人工停止与故障恢复 | RDK ABORT 回执、F405 ABORT 停车与 mode51 重新定位后回审核通过 |

NormalRun 编译成功。F405 的 5 组主机测试和 24 项 Python 协议测试通过。RDK 的协议脚本及 C
动作去重测试通过。逐字节对照 20 个 opcode、15 组命令配对和 3 组左右数量审核通过。
RDK 的 21 个状态机用例逐项调用后有 8 个通过、13 个失败，已确认其中包括旧的无测距抓取交接、
缺少已确认投送目的地及调用已删除边界接口的夹具。记录为整套未通过，尚未逐个更新旧夹具。
用相机/NPU 替身跑的 11 个 RDK runner 用例有 8 个通过，余下 3 个在 Windows 缺 Linux AF_UNIX
或原生接收器环境。另用当前状态入口模拟 6 个安全区走廊、投送和退出分支，全部通过。

已修复下位机 Python 协议工具对 0x1D/0x1E 的类型校验缺项，并更新 F405 主机替身和相关用例。
板端 C 运行逻辑和 RDK 源码此次沿用原提交。当前本地构建不是实烧身份。

仍需留意的实际策略边界如下。

- 本方安全区连续无框时，RDK 在 5 秒后报告 `SAFE_CORRIDOR_UNAVAILABLE` 并继续等有效画面。
  走廊两次扫障后仍堵塞则进入 `WAIT_SAFE_ZONE_CLEAR`。两种情况均已离线复现。
- 投送视觉未确认且观察达到 1 秒时，上位机按现有策略发 COMPLETE 并计入投送。
  这代表任务进入退出流程，物资是否进区要结合画面验证。

正式部署仍需验证实际 NormalRun、Linux IPC、相机/T265 及真实轮子与舵机动作。
RDK 使用正式比赛启动入口的 921600/8N1。

## UART-100HZ-01 的来源与审核

| 项目 | 本次核对结果 |
| --- | --- |
| 请求 ID | `UART-100HZ-01` |
| 用户批准范围 | UART4 改为 921600/8N1，07 使用每个真实相邻编码器区间，建立固定协作入口 |
| RDK 要求源提交 | `347791666cd3d32ecfe157979fe8e772ff30f4a8` |
| RDK 实现源提交 | `6bfed9ebc794d897f957aa7a253648948077c00a` |
| RDK 分支读取结果 | 核对时 HEAD 为上述 `3477916`，固定入口与该提交内容一致 |
| 本端起始 HEAD | `6302bf2e643c034f5cca694c519be8245dcf5dd2`，与远端 main 一致 |
| 审核结论 | 接受本轮最小方案，源码证实原 10ms 采样被两段汇总为 20ms |
| 代码实现提交 | `14b2a3a9077e6980a540631ce94345e61eb0487d` |
| 构建状态 | MotionDebug 和 NormalRun 均编译、链接成功 |
| 后续授权烧录 | 用户最终指定 MotionDebug，已烧录并通过 OpenOCD Flash verify，NormalRun 未烧录 |
| 板端 actual | Git 与 firmware_id 已从板端 Flash 经 SWD 读回，见下表 |
| 板端 boot_epoch | 已从板端 RAM 经 SWD 读回，见下表 |
| 实收频率、SDK 与地面精度 | 本轮未实测 |

源要求见 [固定源提交](https://github.com/danmo-teng/shijue_fangan/blob/347791666cd3d32ecfe157979fe8e772ff30f4a8/docs/CROSS_REPO_REQUESTS.md)。
审核仅接受本次明确批准范围，来源文件本身不授权新增功能。

## 实现及两端影响

`CMakeLists.txt` 为 MotionDebug 与 NormalRun 共用 `APP_NORMALDEBUG_UPPER_BAUD=921600`。
`board_f405.c` 已按此宏配置句柄 `huart3` 对应的物理 UART4，PC10/PC11、JP506，8N1。
主 gyro UART5 继续为 230400，程序器文本 USART1 继续为 115200。
RDK 部署新固件时须同时选 921600，尚未升级的板仍按其实际固件选择波特率。

`DebugTelemetry_WheelSample` 取消两段累计。首次快照建立基准，此后每次采样生成一个新的记录和 measurement_id。
原始累计计数与 delta 来自 `encoder_f405.c`，保留原始计数符号和回绕处理。
每个记录的 t0/t1 为相邻计数快照各自读数括区的中点，速度使用实测 `t1-t0`。
capture_window_us 为两个端点读数括区宽度的较大值，物理源时间误差仍未知。
原有效位含义、INT32_MIN 无效速度、未知误差字段均沿用现有协议 1.3。

07 仍为车体旋转中心 B 点、车体系 B 轴的前/左速度，单位 µm/s。
轮序为右前 M1、左前 M2、后轮 M3，counts/rev 为 1061.268，轮径为 0.070m。
令 K 为 π×0.070/1061.268 m/count，dt 为区间秒数，则前向为 K×(dn1-dn2)/(√3×dt)，
左向为 K×(dn1+dn2-2×dn3)/(3×dt)。这里使用未翻转的原始计数，不重复应用电机反馈负号。

08 有新源时最高 100Hz。yaw 为 mdeg，rate 为 mdps，均为逆时针正。
独立源 ID、源接收时刻与 NEW/REPEAT 语义沿用原实现。没有新源时不伪造更新。
未知 drift_bound_ppm、source_uncertainty_us 和 gyro uncertainty 继续为 FFFFFFFF。
原源时间事件及未标定状态不因为波特率或发布频率改变而成为标定结论。

协议及 CONFIG_REPLY 的本轮配置值如下。

| 字段或属性 | 当前值 |
| --- | --- |
| 身份协议 major/minor | 1.3 |
| D4 4D envelope version | 1 |
| uart_baud | 921600，读取 UART4 的 Init.BaudRate |
| required_baud | 921600 |
| wheel_period_us | 10000，标称每个原 10ms 调度区间 |
| gyro_period_us | 10000，发布上限 |
| board / mode | 405 / 5 |
| MotionDebug service | 1，MOTIONDEBUG |
| NormalRun service | 2，RESCUE_TASK |
| 07 / 08 长度 | 各 95 字节，布局、单位和 CRC 延用现有定义 |

原控制协议、任务配对、合爪业务、STOP、mode51、电机参数和舵机均由原实现执行。
发送继续使用 `Uart.c` 唯一 DMA 启动者、短快照和未发同类遥测合并。
主循环只消费最新未消费的测量 ID，不补发积压的旧采样。软件或链路忙时允许缺号，不能据配置值宣称实收无丢样。
07/08 共 19000B/s，占 921600/8N1 理论容量 92160B/s 的 20.62%，另需计入原状态和回执。
95 字节的理论线传输约 1.03ms，尚无本轮板端总延迟或抖动实测。
RDK 现有控制/显示 20Hz 快照链的改进由对端后续处理，本次发布频率不等于控制频率。

## 必要检查与构建身份

调用链已核对为 `Robot` 原 10ms 调度 → `Encoder_Sample10ms` →
`DebugTelemetry_WheelSample` → 原主循环 `DebugTelemetry_Process` → `Uart_Send` → 原 DMA 发送。
两个构建均启用 DEBUG_TIMED_LINK，身份 service 仍按实际构建选择。
本轮未新增采样中断、计时检查、诊断框架、协议字段或测试程序。

构建命令使用已有固件目标，提交推送单独执行以保留工作区的无关改动。

```powershell
cmake --preset MotionDebug
cmake --build --preset MotionDebug --target MotionDebug
cmake --preset NormalRun
cmake --build --preset NormalRun --target NormalRun
```

| 构建 | ELF | 编译结果 | 内嵌 Git | source_dirty | firmware_id |
| --- | --- | --- | --- | --- | --- |
| MotionDebug | build/F405/MotionDebug/MotionDebug.elf | 编译、链接成功 | 14b2a3a9077e6980a540631ce94345e61eb0487d | 1 | 6839200f0d0354e8decaa242632e7b637e3d709e349ce3164844a9bf39543c21 |
| NormalRun | build/F405/NormalRun/NormalRun.elf | 编译、链接成功 | 14b2a3a9077e6980a540631ce94345e61eb0487d | 1 | 98dc7fb784f057b543dc05eb84b0044d2fbd3b7333629e952886c05bcf5a2268 |

source_dirty=1 的实际源差异为开始本轮前已存在的 `Main/Src/Task.c` 两行空白，
位于 task_initialize 中 tracking_tick_ms 清零之后，其中一行含两个空格。本轮保留它们。
IDE 修改及 tmp 也保留在工作区，不纳入本次提交。
表内身份来自本地构建生成头，不是板端自报。后续文档提交与这些镜像的内嵌 Git 分开记录。
本地构建日志位于各模式构建目录下的 `UART-100HZ-01-build.log`。
两次构建退出码均为 0，本轮改动文件的 diff 空白检查通过。
编译命令已核对两种构建都将 921600 传入 board_f405.c 和 DebugTelemetry.c，service 分别为 1 和 2。

| 本地 ELF | SHA256 |
| --- | --- |
| MotionDebug.elf | fdbb1249b11ddcd5ea92ab76e84f53a58740b2116849e0c0f1f68ebd893535bd |
| NormalRun.elf | 1e95d0b8b8909fe3774c977235c7333c6341c4e894e041acd0dc4c3d710efeb6 |

## 尚未完成

- RDK 经 UART4 原身份查询读回并记录 actual、source_dirty、boot_epoch、mode/service 与波特率。
- 通过原日志核对不同 measurement_id、相邻约 10ms 的真实 t0/t1 区间及 07/08 实收频率、缺号和延迟。
- RDK 对本次实现提交复核，部署后核对实际 SDK API 接受情况及人工地面验证。

以上实机项目尚未执行，配置回执、编译或 SDK 接口成功均不作为地面精度证明。

## 后续授权烧录与只读读回

2026-11-08 用户将烧录目标改为 MotionDebug，并要求先不启动车轮。
本次使用表内原 MotionDebug.elf，SHA256 与先前构建一致。OpenOCD 已返回 `Verified OK` 并复位运行。
COM18/115200 启动输出确认 F405_ROBO、MODE=5、SERVICE=MOTIONDEBUG。
仅发送 `PING`、`STAT`。状态为 IDLE，三轮 TARGET、ACTUAL、PWM 均为 0。

| 板端读取项 | 实际结果 | 依据 |
| --- | --- | --- |
| actual Git | 14b2a3a9077e6980a540631ce94345e61eb0487d | SWD 读 Flash 0x0801ba70 的 20 字节 |
| actual firmware_id | 6839200f0d0354e8decaa242632e7b637e3d709e349ce3164844a9bf39543c21 | SWD 读 Flash 0x0801ba50 的 32 字节 |
| source_dirty | 1 | 已校验镜像的内嵌值，尚未经 UART4 ID_REPLY 自报读取 |
| 板型 / mode / service | F405_ROBO / 5 / MOTIONDEBUG | COM18 实际启动与 STAT 回执 |
| UART4 波特率配置 | 921600/8N1 | SWD 读运行中 huart3 的实例地址及 Init 字段 |
| UART4 BRR | 0x0000002D | SWD 读寄存器 0x40004c08，未测物理位宽 |
| boot_epoch | 2b40340370e209faed7d2123a1015296 | SWD 读 RAM 0x2000048c 的 16 字节 |

本次 UART4 没有连接第二个主机发送 ID_REQ，不能把 SWD 读回说成 UART4 身份查询已通过。
波特率表项为实际运行配置，主机应设 921600，物理线速未用仪器测量。
本地原始日志位于 `build/F405/MotionDebug/UART-100HZ-01-flash-20261108.log`、
`UART-100HZ-01-console-20261108.log` 和 `UART-100HZ-01-swd-readback-20261108.log`。
07/08 订阅、100Hz 实收、SDK 接受和地面精度仍待验证。
