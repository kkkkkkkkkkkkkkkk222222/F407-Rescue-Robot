# 两端当前要求与回执

更新日期为 2026-11-16，时区 Asia/Shanghai。

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

## 2026-11-16 S1运输高度修复：已构建，未烧录

按用户黑CORE运输高度偏低反馈，板端只读核对actual=6b05d94，S1 offset仍0，左右微秒标定仍+10/+50。
生产夹具复现过滤82°被中断后，原恢复/重新抓黑CORE可沿用82°进入NAV；普通合爪入口现明确恢复原85°，未调参数或加等待。
桌面工具映射/旧测试补偿及手册同步；RDK不需改报文。两目标构建及相关回归通过，需新NormalRun部署后实测。
这是一条已证实的缺陷路径，不宣称已确认实车掉物的唯一原因；actual/expected和检查见[核对记录](F405_SERVO_TRANSPORT_AUDIT_20261116.md)。

## MATCH-CLOSED-GAP-ZONE-CANCEL-20261116：下位已配套并构建，未烧录

按用户本轮“对照上位机的代码，修改代码”核对RDK功能提交及收尾分支，均为eb49d39ab695a44487cf5deebbdd26116fcf69f9。
本端基于49a4c75增量实施。已读固定入口blob 800b8693224c030070679b21fbd1f5ea4eadc0c6、
state_machine.py a8065de875a67d4e3c20861c6fd3074e4df5f2dd、
protocol.py 2d4b569b30dadc06d5df500ca32ca0418b661f25；
新合同5e4600198be5070e736ca1e4ebe2b675e75cb463，40cm路线合同cf0725c21508fe9b77f22134ce66fafe8d62455c。
上位真实runner新增取消优先于有限段重发、缺框2秒决定及复用期间矛盾复审；本端核对对应提交实际差异。
橙优先、模型、ROI不在本轮修改范围。

### 已确认载荷复用

- 原bit5扩到定位ALIGN/opcode4，严格VALID|USE_FINAL_HEADING|bit5|阵营、A0、B0或11500、
  U红9000/蓝27000，context视觉帧0；禁止将此位用于视觉ALIGN、错方向或其他参数。真实解析器和本地Python构帧工具同步。
- Task单独记录本批真正通过的闭爪载荷类别/数量及目的地权限。原当前审核清理/占位和历史权限分开：
  占位仍撤本轮READY/VALID，空框不被填入虚构类别、数量、audit_id或源帧，不补样本。
- 仅同归属mode48、保持闭爪且历史许可仍有效，接收明确复用定位ALIGN；需纠正半区时先接原STAGE NAV+bit5，
  本NAV实际D0完成后才接复用ALIGN。之后原视觉ALIGN及ENTER继续原运输流程；同action重复不重新启动。
  原单橙找回NAV bit5→普通ALIGN路径兼容，普通无复用mode48仍须本轮真实审核。
- 真实类别/数量、危险/未知/伤员混装、首绿黑冲突撤历史许可；左右位置抖动及合法GREEN+CORE与MIXED表示互换不误判新载荷。
  释放/分离/开爪、进入新批或新会话、STOP/恢复撤销；矛盾后须按原真实审核重新建立。
  导航中撤bit5会撤本段复用资格，随后重新加位不能凭空恢复。
- 为匹配上位真实runner的矛盾出口，已获复用许可的STAGE导航可在未到点时接收原运输复审占位请求，
  立即停止并转原mode48；原普通STAGE到点复审条件保持。回mode48后结束本段复用，真实矛盾按新审核撤历史许可。
- 若复用运输中触发原扫障寄存，历史类别/数量只交给原sweep_original记录，再撤复用许可；
  不把缺框时当前审核的零值误写成原载荷，也不把历史记录变成当前审核通过，找回后仍沿原复审。

### 明确取消及机械恢复

- 原RELEASE_BOTH/opcode15，严格VALID/阵营、A=B=U=0、视觉帧0、同归属新action，
  可在开爪普通/聚堆APPROACH（含52/53定距）、21/38/37抓前观察、闭爪23/48审核直接放弃，
  不再要求先提供非法或空爪STABLE。单侧释放条件没有放宽。
- 明确取消可穿过pre_grab.moving原提前拒绝分支。接受时复用task_begin_capture_escape：
  Motor_Stop清旧有限运动、清补进及旧完成/目标上下文，马上进入mode24。
  随后按原开爪完成→后退300mm→原脱离转向→SEARCH机械流程；不是先执行完旧定距，也不是只ACK不取消。
- 同task/action重复取消在恢复中及SEARCH只确认，不重启开爪/后退；旧APP及错task/action/阵营/参数不能接回。
  新目标按原搜索生命周期处理。取消不记投送次数；原放货/进攻释放及单侧分离入口保持。

### 复看及时间边界

- 真实闭爪非稳定空检测触发一次原138/142°微调并回140°后，保持原复审等上位决定，不再自动开始第二轮3.5秒复看。
  本地不另设2秒准入门，不在2秒自动放下；上位2秒可发明确复用或取消。原15秒无上位决策兜底保持。
- 原其他真实审核采样、非空处理、PAUSE/STOP、相机机械等待、抓前200mm/4秒预算、PID/速度/舵机参数保持。
  无新增ISR、遥测频率、传感器年龄门、反复握手或普通异常FAULT。
- 两观察点X±0.40m/Y0由上位控制，下位沿原step15/14/13/12，没有写死站点或调运动参数。
  本轮更新路线合同中的8组几何/40组示例，替代旧±0.30m示例；都是合成示例，不是实车数据。

### 必要检查及部署

- NormalRun、MotionDebug现有目标编译链接通过。
- 临时复用既有Task/mechanism夹具，16条红蓝×载荷×直接/半区纠正路径通过：真实审核建立许可、
  缺框复看、定位ALIGN/动态NAV到点、视觉ALIGN、ENTER；复用全程未制造本轮审核样本/READY/VALID。
- 18条红蓝×靠近/观察/测距/140°补进/闭爪审核取消路径通过：立即停止、重复幂等、旧APP拒绝、
  实际开爪/脱离完成后SEARCH、无投送计数；错归属/阵营/参数及单侧请求不获得新权限。
  类别/数量/危险/首绿黑撤销、重新真审核、MIXED/侧别归一、会话/开爪/STOP、
  复用NAV中复审和撤位、扫障历史交接、单次微调/原15秒兜底及旧单橙入口均通过。
- 实际vision.c解析器消费6组新合同及更新的40组路线成对帧通过，本地Python逐字节/CRC一致；
  32组解析边界/旧格式检查及原Python协议26项通过。原抓前补进夹具和进攻转尾8条路径回归通过。
- 临时C夹具已恢复；只扩原Python协议测试，不向仓库或对端增加独立测试程序任务。
  原Task空白、WheelCheck、IDE及标定工具独立改动保留且不夹入提交。
- 本轮未开串口/SWD、烧录、启动动作或读板端actual。构建build_git=49a4c75/dirty1、含原独立工作区改动，
  指纹见CURRENT_STATUS；不等同干净提交重建或实车通过。上位报告此前实跑49a4c75/dirty1仅作对端证据，
  不代表本次新出口已部署。须配套部署本次NormalRun及RDK eb49d39后联合实测，不能继续以旧49a4c75称新功能完整可用。

[新出口合同及6组报文](contracts/f405_closed_gap_zone_cancel_20261116.json)；
[更新40cm路线示例](contracts/f405_two_station_tail_return_20261116.json)。
本端回执已落固定入口，未代对端声称已读取、部署或验收。

## 2026-11-16 两搜索点及进攻转尾返回：已配套并构建，未烧录

请求ID：MATCH-TWO-STATION-TAIL-RETURN-20261116。按用户“根据上位机的代码，修改我的代码”核对并实施。
RDK分支codex/gamepad-teleop功能及收尾复查均为750aafae5b4ff33bb4e0ff52f394cdaf0b0eb1d1。
固定入口blob b4d155f3c73d26dc2fc1669e278033d8a1d2feaf；
合同b0813584a268e9262d38c0426e6629a76632877d；
attack_return.py a82d084986a26111c5a6237b19b982ef4de490bb；
state_machine.py e6698bf89817688939db7256419ded460b12c097；
protocol.py 7d97b3a7d8a4981b6bf57546e803729454a24b7b；
runner ed356e589317d49d181767a365241253ef7d4c99。
本端基于c14b0a7增量修改；本条取代下方旧回执中“①/③/转尾仅提案、保持100mm”的阶段判断。

### 分端行为及最小配套

- 上位负责①/③坐标、最近实际到达的来源点记忆、投送原退出后直返来源点，以及进攻后退完成后按实际位置计算并锁定车尾朝①的角度。
  起点1/3的①=(-0.30,0)，起点2/4的①=(+0.30,0)，③相反；红蓝不改变坐标。下位不硬编码这些地图点。
- step15保留枚举旧名ATTACK_BACK_100，线上A新增-150并兼容-100，B=0，红U27000/蓝U9000。
  仍只从同归属、真实完成且空爪的RELEASE_BOTH进入，原轮程控制直接执行A；不是先100再补50，也不按地图跳变提前结束。
- step14解析接受A=B=0、U=0..35999。原闭爪藏点NAV完成入口仍只接受配置阵营的对方朝向；
  新空爪转尾入口只从同归属step15实际DONE进入，不允许SEARCH任意转或放下后跳过后退。
  原转向控制/完成公差保持，不另加动作或到位等待。
- step13新增承接同归属、空爪step14实际DONE；原SEARCH、放下、step13/15完成入口兼容。
  仍使用动态D/H及本action固定U，D=0完成。同action改U拒绝，重复命令不重新起转/走段；
  新action沿原流程清旧DONE，不能把上一阶段DONE当下一阶段完成。
- step12原抬臂/搜索相机/恢复搜索保持。上位已在①原50mm范围内时可从step15完成直接恢复；
  普通投送到源点及①↔③换点复用原step13，不增加下位导航点或计数状态。
- 原PAUSE/STOP、轮程/航向/导航控制器、速度、PID、舵机及到位公差保持；没有新增时间门、握手、ISR、高频诊断或FAULT。
  合爪84°、S4+10/S2+50µs，以及既有抓前补进、相机和审核配套保留。
- 本地Python构帧工具补齐原opcode20合法格式，与实际C解析约束一致；协议类型、长度、CRC和step编号不变。
  [本端合同及上位8组几何/40组配对帧](contracts/f405_two_station_tail_return_20261116.json)。

### 实际检查及部署边界

- NormalRun、MotionDebug现有固件目标编译链接通过。
- 临时复用既有Task夹具，红蓝×4起点共8条合同几何路径通过：
  放下→实际轮程150→同归属任意角转尾→动态倒车→D0→搜索；
  未完成拒接、每阶段清旧DONE、重复/改参数、PAUSE恢复、STOP、地图跳变不提前完成、
  旧100mm、已到①跳转、闭爪/错任务/SEARCH/放下后直接转拒绝、原带载只朝对方均通过。
  后退/转向/倒车未额外改爪姿或计投送次数；地面方向、距离及稳定性尚未实测。
- 实际vision.c解析器消费全部40组上位context+command原字节通过；Python构帧逐字节一致。
  另34组红蓝边界/旧100mm/原step0和4固定角/非法参数与flags检查通过；本地原协议25项检查通过。
- 临时C夹具已恢复；没有提交额外独立测试程序。既有Task空白、WheelCheck、IDE和标定工具独立改动保留，不夹入提交。
- 构建git=c14b0a7/dirty1，具体两目标指纹见CURRENT_STATUS；包含当前工作区原独立改动，不等同干净提交重建。
  本轮未连接串口/SWD、烧录、启动动作或读取板端actual。两端代码配套不等于联合部署和实车通过。
  新上位路线须配套部署本次NormalRun，再做实车验证；旧下位不接受-150或转尾衔接。
  本端回执已落在固定入口，尚未替对端声明已读取或部署。

## MATCH-PRE-GRAB-BANDS-20261116：下位已配套并构建，未烧录

已按用户本轮“对照上位机代码仓库修改”读取RDK功能提交`04467bc2e3ba9f0368a9090d77e44ff270ce7d0d`。
收尾分支更新为`b995f5a4a1da3833acc7bcb3c15ccf19acc3b004`，再次读取state_machine/protocol/runner的blob均与功能提交一致；
新路线文档仅为①/③及转尾倒车提案，不是实施要求，本端未改原100mm/step15/搜索路线。
固定入口blob `c661e985803a3e17aefdc5ea86834809aab4e159`；state_machine `8ee7eba558a903e6e8b1e213576871b19dbfd0d7`；
protocol `71a7e974f5095de02c2d4ceabdad58446f2fdb73`；runner `57923648341dc08fe7bea6499d685e9cf9c58e92`；
合同 `949f98322dfbab3ec10dc67e679b925a573e54df`；绿橙分界配置 `ea5cd631316b552df71be5478e28cb2a29e0d92e`。

接受绿橙抓前分区补进的最小配套。上位负责锁定目标、类别和分区线判定，下位不猜像素线值、不改ROI或视觉模型。
本端在81a3557上增量修改原Task/真实解析器/vision.h和构帧工具；新的合爪84°、左右+10/+50µs及上轮三项成果保持。

### 已实现的动作与接口

- 仍TYPE0x18/opcode9/RANGE bit6，新增严格`aux=2、A=+50mm、B=14000`组合；红方/聚堆位沿原任务。
  aux0/1仍只在105/120原测距流程使用，原范围/flags/参数约束不放宽；[本端合同和相同4组成对帧](contracts/f405_pre_grab_bands_20261116.json)。
- 仅本批已进入抓前观察、开爪且140°的21/38/37接受，必须有原capture预算和真实源帧；闭爪/运输、分离后复审、无目标开局探爪不适用。
  同task/新action/聚堆归属/源帧顺序沿原检查；下位没有绿橙目标框坐标，绿橙与“上区”由上位确认，不冒充下位独立视觉判断。
- 接受时清旧抓前审核样本及许可，保持现有爪姿/140°。复用Motor_MoveDistanceConstant前进50mm，使用原普通/聚堆近场200mm/s请求，保留原加速、航向和轮程刹车。
- 内部保留入口状态，执行中只将线上mode映射53且不报DONE；仅MOTOR_DISTANCE_DONE后Stop，回原21/38/37并报本task/action/opcode9的ACCEPT|DONE。
  原轮程3mm完成公差保持，不使用aux0剩10mm提前交接；真实地面50mm仍待实测，不把控制器DONE当实尺结果。
- 同action与原源帧重发只确认，不重置轮程或预算；新action、新图才开始下一段。执行中的AUDIT/GRAB、改参数、换源帧或抢开新段会拒绝。
  完成阶段保持停止，普通21/38不会恢复自主爬行；真实CARGO_AUDIT仍需原两帧审核后再GRAB，聚堆真实稳定审核可沿原38→37。
- 复用审核源帧下界：运动中收到的图即使在完成后重发、甚至更换审核action，也不能变成完成后复看样本。
  同一规则用于下一50mm段；上位已有匹配DONE后取下一实际图的逻辑，没有新增年龄门或额外握手。
- HOLD沿原有限段语义保留执行/完成；PAUSE暂停有限轮程并沿原暂停记账恢复；STOP/ABORT及原任务/释放/恢复生命周期结束本段。
  更换会话时结束旧有限段并走原mode24恢复，避免清状态后落回原自主爬行；未增加普通FAULT。

### 原预算与性能边界

- 仍使用本批capture_forward_used_mm/200mm额度。aux2运行时按本段实际轮程增量记账，替代同段Location累计以免算两遍；之前本批已经用掉的距离不清。
- 成功一段至少计入名义50mm（若实际测得更多则保留更多），不把原到位公差反复返还为新额度；150mm已用可接末段，151mm已用则不足下一整段。
- 额度不够整段时接住该请求并走原capture escape/mode24，不动部分50mm、不报DONE，也不让上位在拒绝回包里无限重试。
- 继续原capture_wait_started_ms/4000ms决策预算；无已启动预算时首次补进沿原空审核决策起算，已有预算保留。
  接新50mm、mode53↔原审核、真实合法结果后再请求补进都不重置；人工PAUSE沿原排除暂停时间。到期走原恢复，无新每段超时。
- 不新增ISR、协议类型、高频日志/统计、传感器年龄门、等待握手或全局速度/PID；状态/预算更新在原Task调用中完成。

### 实际检查与部署

- NormalRun及MotionDebug现有目标编译链接通过。
- 临时复用原Task/mechanism夹具，3个入口×红蓝完整调用链通过：清旧审核、mode53/完成回原入口、49mm仍RUNNING不提前DONE、重复/HOLD不重走、PAUSE续行、STOP/会话替换、完成保持、新段/真审核再GRAB。
  另覆盖错误入口/参数/源帧/聚堆位、抢发AUDIT/GRAB、运动中图重发不能充当完成后图、151/150mm剩余额度、连续4段共200mm及第5段恢复、3990/4000ms预算、有限运动失败原恢复。
- 真实vision解析器消费上位红蓝×普通/聚堆4组配对帧通过；8组非法aux/距离/角度拒绝，aux0/1×105/120兼容通过。
  本地Python原合同测试新增aux2覆盖通过；上轮aux1幂等、mode48证据保留及STAGE/ALIGN相机相关原夹具回归通过。
- 临时夹具已经恢复，没有向仓库或对端新增独立测试程序；只扩原协议测试。原Task空白和WheelCheck/IDE/工具独立改动保留、不夹入提交。

本轮未连接串口/SWD、烧录、启动设备或读actual。构建Git基线为81a3557/dirty1，最终指纹见CURRENT_STATUS；不能据提交/编译称已部署。
上位`--enable-pre-grab-advance`仍默认关闭，所读绿/橙分界均null。下一步先部署新版NormalRun、人工标定并保存两类横线，再重开上位启用开关。
下位请求/完成出口/预算已在本回执说明，待对端核对及实车观察；未在此窗口修改上位文件或替对端宣称已收到。

## 2026-11-16 三项统一配套：已实现并构建，未烧录

对应MATCH-STAGE-CAMERA-FLICK-20261116、MATCH-TRANSPORT-AUDIT-RECOVERY-20261116、MATCH-TOP-EDGE-RANGE-20261116。
已核对RDK `db44f144ee3005ba16124044d08c40ebe0a1c330`（收尾再次读取分支仍为该SHA）：
固定入口blob `d132e38b1b501f5efd24d169be9b4aa6ef9bbc9d`；state_machine.py `349c3bb9aa220425d3f667687b8b06ba0a25eccc`；
protocol.py `d4337734435a27460c7813149374e92926b22dfe`；runner `16afa34d7aa09ce606d8a0cb52fa6d2a820e2708`；
顶端测距合同 `3e938d55ce76819dce542573c14e29951339bcc9`。本端基于0277770增量修改。

**接受当前三项范围。** 本端只修改Task.c、原解析器/协议宏及本地构帧工具；保留已确认的夹爪84°/+10/+50µs。
上位暂停的橙色中线/ROI/T265/闭爪抽障/异侧挤障不纳入，不复活历史候选。

### 安全区到点相机

删除STAGE且D=0到达分支提前设120°的操作，只沿原Stop/到达回报，保持140°夹内视角。
原运输复审完成后的实际ALIGN仍通过task_begin_safe_align和原接收分支选择120°或115°。
抓后单橙合法审核复用直接ALIGN的入口也有自己的设角，不依赖NAV提前设角。
没有修改普通RETURN/藏点/接近导航、相机全局速度或原138/142复看；上位不需新报文。

### 同一次闭爪mode48审核恢复

新增局部判定仅保留本轮mode48、实际闭爪、已存在非空合法样本且无首绿黑冲突的证据。
入口/新审核action/新任务或会话、STOP、分离释放等原生命周期清理仍执行；不借抓取时的旧载荷完成新复审。

- 覆盖原局部500ms证据间隔、500ms累计窗口、Task_Process全局过期，以及相机nudge/回140/settle清理。
- 保留原两个不同真实帧、不同audit_id及最小100ms间隔；重复包不增样本、不更新时间。同原帧STABLE确认仅对上述例外免年龄失效。
- 占位仍撤销audit_received/host_stable/valid/ready及mode48旧completed；只保留真实样本，不当空爪，不授予权限。
  撤销后原帧STABLE不能重新放行，需原规则的新真实图；普通漏框时上位应按新代码重发原稳定payload，而非占位。
- 新真实类别/数量/语义变化第一帧就撤旧完成、重新累计；合法材料按类别数量比较，不把两黑和一绿一黑混为同一批。
  同一绿黑组合从单侧MIXED变成两侧分开不算矛盾；左右侧抖动也不重新起算。危险/未知/空/首绿黑冲突不能借保留例外恢复资格。
- 相机非140°或原settle未完成不收新样本；原总审核超时、抓后mode23及其他审核范围保持。

代价/边界：本轮闭爪合法样本可存活超过500ms，但不是无图制造载荷；上位真实矛盾时必须撤旧STABLE。
已核对db44f14对capture_cargo新增类别/数量及遮挡混装的处理，其占位会立即撤销本端旧许可。

### 顶端远目标分段接近

0x18/opcode9原RANGE bit6的aux扩为0/1；vision.c真实解析器、Task校验、vision.h和tools/vision_protocol.py同步。
aux0保留原有限段及原末端余量交接、转140°补进/审核；aux1沿同Motor_MoveDistanceLinear，必须返回MOTOR_DISTANCE_DONE才Stop，
保持本段105/120°、回mode52，锁定本task/action/opcode9的ACCEPT|DONE；不会假报21或进入抓取。

mode52收到已完成aux1的同action和相同D/pitch/aux，仅确认完成，不再启动轮程，也不重置等待期限；HOLD保留完成。
执行中或完成后同动作改变参数会拒绝；新action仍须原新图/测距入口，才锁下一段距离。aux>1拒绝，原flags/距离/角度/任务顺序保持。
aux1不使用aux0原近端余量的提前交接，避免上位以为本段已经实际走完。原RANGE_WAIT等待/恢复照常，无新超时、循环次数、握手或FAULT。
[本端合同及与上位相同的4组红蓝配对帧](contracts/f405_top_edge_range_20261116.json)。上位只在框贴顶且原标定最上档选aux1；完整框aux0由上位决定。

### 实际检查与部署边界

- NormalRun/MotionDebug两个现有固件目标编译链接通过。
- 复用原Task/mechanism主机夹具直接调用：红蓝×普通/聚堆×105/120共8组合，真实完成/余量边界、执行/完成重发、HOLD、下一段、aux0、错参数/旧action拒绝均通过。
- 同一夹具检查mode48漏框/全局过期/138复看/回140保留，重复帧/100ms、原帧STABLE、占位撤销、新action/入口/STOP/首绿黑冲突，合法换类和MIXED侧别变动；通过。
- STAGE重复D0保持140、红蓝ALIGN120/115及单橙直接复用入口通过；原FLOW范围/审核/分离回归通过。
- 原vision解析器实际消费上位合同4组上下文+命令帧通过，aux2/3拒绝；本地Python原测距合同检查通过，CRC/字段与上位一致。
- 原STAGE/RETURN/ABORT导航检查另有既存断言仍期望200mm/s，当前源码末端下限已为250mm/s；改前Task亦复现同一失败。
  仅临时对齐该旧断言后，本次相机/RETURN/ABORT检查通过；已恢复200断言，不将整个旧导航套件宣称为原样全通过，不改本轮无关速度参数。
- 没有新增独立测试程序到仓库。临时夹具已恢复，仅更新原到达相机断言及原协议测试aux0/1与非法值覆盖。

本轮没有打开串口/SWD、烧录、启动车轮或重启上位；真实场景恢复、相机运动和分段距离仍待实车。
**必须先部署本次新NormalRun并核对身份，再在RDK启用`--enable-top-edge-range-reobserve`；目前保持默认关闭。**
提交/构建不等于actual，本端也不冒充上位已读回执。原Task空白、WheelCheck试验、IDE及工具独立改动保留，未夹入本次提交。
构建的Git基线0277770/dirty1，最终指纹见CURRENT_STATUS；本轮actual未读取。

## MATCH-PREWARM-FEEDBACK-20261116：下位警报v2已实现，上位配套与部署待完成

已读取RDK `4609ca1d81c368bd482ba02515a7d7c7a8c4f49d`固定入口blob`6300786d9ee962f16597ee017b5db0a002c08e24`，
开发说明blob`cba0ddab056db5c8669b228d32576cee644d267d`，警报合同blob`bd5eb0a5da36c2e7d8399f539c855aed90edea99`。
本端基于efe3f1b核对PA10既有蜂鸣器、BoardF405_ProcessBeep和vision.c独立0x20分派，接受本次持续警报目标，未改变上位按键程序。

- `Boards/F405/Src/board_f405.c`复用原前台服务，增加最新请求邮箱与持续警报状态。120ms响/60ms停，无长间隔组停顿；
  不增加变调硬件、PWM定时器、中断或阻塞等待。短临界区仅交换邮箱字段，GPIO仍只在前台更新，控制流程不等待声音。
- `BoardF405_SetOperatorAlarm(true)`只在从非警报切入时启动；重复01不重置节奏。普通v1查询在警报有效时忽略声音，不清警报、不排队。
  false/00取消当前及待播操作声音，下一次原服务调用即拉低PA10，不等待音段/ACK；上电静音。连续覆盖的旧警报/查询不回放。
- `Main/Src/vision.c`同TYPE0x20接v2版本02、op00/01、后6字节零，原v1保留。未知操作/版本/保留非零忽略消息，CRC沿原帧校验。
  所有0x20均在任务context_pending处理前返回，不消耗0x1D/0x18配对、不改任务动作/ACK/CONFIG连续计数；不新增声音回包。
- `vision_save_config()`仅在本次CONFIG真正接受、session递增处请求清音；未接受/非法CONFIG及已config_ready后的重复CONFIG不当成新局。
  消音使用原前台声音服务，正常100Hz调度下由下一次服务处理；这不是实测端到端响应时间，也不要求启动等待消音。

[本端v2合同与两组合成帧](contracts/f405_operator_alarm_v2_20261116.json)，字段与上位提案一致，序号不作续命/资格。
已用原CRC/构帧函数核对00与01示例，未发设备。原夹具内编译实际声音函数块及实际vision.c，GPIO替身直接核对19项：
持续周期、重复ON/v1不打断、消音/待播覆盖、v1单双长短音兼容、非法字段/CRC、CONFIG接受边界和声音穿插后的任务配对均通过。
NormalRun/MotionDebug构建通过；没有新增测试源码/程序、声音ACK、新鲜度门、自动FAULT或高频串口续发。

**未烧录、未打开设备；真实音量/节奏紧迫感、发车时延未实测。** 上位Python/C++ v2接收转发、唯一UART调度、长按3预热/短按3发车、
取消旧预约及就绪v1心跳仍需RDK完成并按原方式构建部署；下位不读取上位设备状态，未收到ON不能自行判断整个上位进程失联。
本端不改下载目录人用手册，不把下位实现冒充上位发车逻辑已修复。上位应按合同在重试/退出请求00，发车停止旧提示且不等待声音ACK。

## MATCH-SEARCH-DIRECTION-20261116：投送后奇偶方向已实现，未烧录

同一上位源版本，已核对state_machine.py blob`1f3cdcd029e1c0be767f5c81ebb4ee52bdaf73ee`的`_update_search_scan_progress()`取绝对角增量，
与下位原`task_turn_magnitude_reached()`一致，无需新转向位或改变720°额度。只改Task.c：

- 新比赛初始化搜索方向为+1；公共`task_finish_delivery_exit()`实际完成开爪后，首次保持+1，其后每趟翻转一次，复用原first_delivery_done。
  公共出口增加OPEN_FOR_RAM/RAM_VERIFY状态防重入；重复COMPLETE/RETURN不再计次。两件一趟算一趟，正常/提前/前段超时实际放下都计动作结束。
  开局藏点、扫障寄存、分离释放、抓取失败不经过该出口，不计数；超时动作结束不代表视觉送达或得分。
- 只在SEARCH_SWEEP_120、SEARCH_SWEEP_90（当前低位105°）、SEARCH_WAIT_RETURN三个旋转请求乘方向；速度仍150mm/s。
  搜索恢复、换相机、换点、返程重发保留本局方向，直到下次投送；不反转全局yaw/轮速，不改变返程、物块对准或相机角度。
- 现有F405场地正yaw为CCW，task_move_body再乘`APP_OMNI_FIELD_YAW_TO_DRIVE_SIGN=-1`映射到轮驱动；
  因此奇数趟请求场地+150/轮驱动-150为逆时针，偶数趟场地-150/轮驱动+150为顺时针。符号沿原已记录接线，不另改电机极性。

原Task夹具27项直接核对开局方向、前四趟奇偶/重复出口防重、三个搜索阶段、恢复保留、顺时针整圈绝对值累计及新局复位通过；
公共投送出口相关原39项回归及FLOW通过。物块对准、安全区回正及转角累计函数与efe3f1b一致。
RDK对面搜索去向和7/24视觉对准点沿现有接口消费，无新增下位任务，未夹带修改。

两原固件目标已构建；本轮未烧录/动车、未亲读actual，地面奇偶方向待实测。原独立工作区及Task初始化空白保留，构建dirty1。
上位报告01:10实际efe3f1b/dirty1、22个mode52动作均单次进入，作为上一版部署/运行反馈保留，不能当本次警报/交替搜索已部署。

## MATCH-DELIVERY-EARLY-EXIT-20261116：接受并已配套，未烧录

核对RDK源提交`8ae4b2c9b99d68aee2fbd79ff5269899b3cad289`，固定入口blob`710157e6fa9ea4b25db9bc842b1b7d087cc9bcde`，
方案blob`6b4cc98f18ec96fc3f9aebccc991dc5821fb8d59`，合同blob`a20f90424516c94f1a9035ca631d1c2b3e8a14de`，
state_machine.py blob`c741b2fac320caead449907782a6973b53ac6845`及实际`run_competition_rescue.py`。
用户已要求对照最新上位代码修改，本端从`7053f88c5a9b5e7bbe3c20d69149ad9e30037122`增量配套；物块对准成果保留。

接受两条正式投送收尾路径，前段预算明确4000ms，后270mm原1500ms保持，其他抓取/扫障/进攻不扩用。

- 首次从ALIGN接受正式ENTER时初始化一次有效执行预算，原任务周期累计至4000ms封顶；前400mm未达到才本地结束。
  当周期轮程已达400mm则先进入原后段，不将4秒当整段上限。重复ENTER及HOLD不重置，PAUSE和原NAV STOP暂停预算，恢复仅余量。
  HOLD本身不提供新的预算；已锁存本地收尾不依赖上位再次允许。未新增数据年龄门、诊断巡检、ISR或普通FAULT。
- 同一任务的新COMPLETE动作可在本次ENTER推进或原开爪阶段接管；原上下文顺序、任务编号回绕与侧别/零参数检查保持。
  新TASK/旧action/同action换opcode不能结束当前投送。进入开爪后重复原ENTER/COMPLETE不重置机械动作，新的ENTER动作不重推。
- 提前COMPLETE先被接受：锁存opcode6及新action，停推后原Claw_Open；开爪中mode12/ACCEPT/未DONE。
  完成后直接原抬臂→mode16→17，开爪且ACCEPT+DONE；跳过原相机观察等待。原mode15才收到COMPLETE也不另等1秒。
- 本地超时先触发：锁存超时原因，保留原ENTER task/action/opcode5；开爪中同样12/ACCEPT/未DONE，开爪退出后16/17/ACCEPT+DONE。
  晚到COMPLETE按原拒绝位报告，不能抢走ENTER的结束归属。同一任务周期沿原命令处理顺序先处理输入再检查预算；一旦锁存不可覆盖。
- `TASK_OPEN_FOR_RAM`已按实际接受的结束原因/上下文运行，不再只等待ENTER命令。已在开爪过程中接受COMPLETE不重开舵机行程。
  first_delivery_done只在真实机械开爪完成后设置，并复用现有载荷/审核/分离预算清理，随后原退出；这仍是任务流程完成，不是下位视觉得分判断。
- 原RETURN可在16接受并等待原抬臂/0.30m退出；接受真实后继后清本次收尾归属，不提前清成SEARCH。PAUSE/STOP仍按原操作优先级。

已核对上位 `_delivery_local_exit_output()` 与runner `_reconcile_delivery_local_exit()`：对本次已发布ENTER的16/17回包按
delivery_push_timeout接管；在途COMPLETE被拒不阻断原ENTER退出，高水位动作号保留。上位提前COMPLETE分支要求本次op6开爪/DONE后再返程，
与本端回包一致。0x18/0x1D/0x1E字段布局不变，无新握手。

**合同示例修正：** 上位JSON第三组RETURN合成例flags=0x09，少了原DRIVE_STRAIGHT/USE_FINAL_HEADING/DISTANCE_VALID；
实际`_return_center_output()`使用红0x1F、蓝0x17，无运行代码冲突。本端保留原校验，提供与实际代码一致的修正例及CRC：

`a3b31d2a006400ca00000000badfc3a3b3182a081f02bc00002328b169c3`

[本端接口合同及三组示例](contracts/f405_delivery_early_exit_20261116.json)。请RDK同步纠正其合成RETURN示例，不需改现有真实RETURN发送逻辑。

验证：两原固件目标编译链接通过，原FLOW检查通过；原Task夹具39项直接调用/完整Task周期检查通过，含正常推进、3999/4000、
400mm边界和后段1500ms、两条先后结束、开爪中接管/幂等、错误任务/动作/侧别/参数、暂停/STOP、HOLD和RETURN接管及红蓝。
仅同步原mode15完成不再等待额外1秒的旧断言，无新增测试程序。三组帧CRC/字段已核对，物块对准及安全区回正函数与7053f88一致。

**未烧录、未打开设备/动车，actual未读；用户自行烧录。** 原Task空白及独立WheelCheck/IDE/工具改动保留而未夹带提交，构建dirty1。
先部署新NormalRun；上位`--enable-delivery-early-exit`/`RDK_ENABLE_DELIVERY_EARLY_EXIT=1`仍默认关闭，需配套启用后才会主动发提前COMPLETE。
本地4秒出口不受上位开关限制，新上位观察器已能接住；未确认入区而超时放下可能留货区外，这是所选策略代价，不计作视觉确认。
联合部署、实际入区与机械效果仍待实车验证，不以软件检查冒充比赛验收。

## MATCH-APPROACH-OVERSHOOT-20261115-2338：已按用户批准修正，未烧录

已读[23:38交接报告](https://github.com/danmo-teng/shijue_fangan/blob/codex/gamepad-teleop/docs/COMPETITION_OVERSHOOT_DELIVERY_20261115_2338.md)
（读取内容blob `adac94e2a816360e1a4126653576299eb8fb8659`）及实跑上位
`ca5ef8eb65b7721fd5abccb823a34b55183000d2`的state_machine.py；读取时当前分支该代码与实跑版本SHA256一致。
本端从`63e352be654e620aeac0952d37f53ff8d06dce70`增量实现，不用旧快照覆盖后续成果。

接受同一绿158、task14969/action24244的14次mode52/13次回20及附近高角速度证据，认可延迟参与但不能全归上位。
补充两个代码结论：普通零速/PWM0在F405为滑行，显式Motor_Stop才接既有短刹；下位普通容差64px、聚堆40px、125°24px，
上位mode52后的原测距复核为64px，不能把各阶段都写成64。上轮63e352b只增强gyro减速，未修停转交接。

用户已明确批准“130只用于起转，提前接入真实刹车，停稳后再进入测距，安全区保持不变”，本次落实为：

- `task_track_target()`保持原始/滤波切换与P/D，运行请求整个曲线由140向0收缓，不再持续加130；保留原3倍容差减速区。
  同锁定目标连续报告的原始X差沿原PID dt估算接近趋势，不引入图像年龄/额外巡检；dt仍是接收间隔，非曝光时间。
- 新`task_approach_align_output()`只供物块对准。130沿原斜坡起转，实测同向≥3°/s撤掉起转请求；
  近区预测趋势×0.20s将到容差边界，或需消除反向余转/低到原电机零速区时，明确调用Motor_Stop复用原短刹。
  预测系数是工程初值，不把上位约110ms当作完整链路延迟；减速阶段可低于130，未新增固定等待。
- `task_process_approach()`普通/聚堆TRACK在真实新目标仍对中、|gyro|≤2°/s且Motor_IsBraking=false后才进入原52；
  此前保持20。130°/125°对准也按相同停转条件交接原相机阶段。旧图不重新起转，原新图偏出可继续纠偏。
  不是追加多帧/多毫秒资格，未改64/40/24容差、原任务/action、ACK及已接受定距动作；原6秒/全圈恢复边界保留。
- 不修改motor.c或板级全局零输出语义，不影响安全区130～250及其控制函数、普通TURN、搜索、舵机、审核或橙色推进。

上位不需改协议：0x18/APP、mode20→52→53、原新图测距和>64px回正语义保持。区别是52现在等本机旋转收敛后才报，
因此上位无需忽略偏差、重复握手或新增等待。正式有限前进、暂停恢复和重复报文不重启的原流程检查通过。

验证：NormalRun/MotionDebug固件目标构建通过；原FLOW检查和原Task夹具28项直接调用检查通过，含运行撤130、
提前真实Stop调用、未结束刹车不重启、余转继续刹、旧图不重启、四种对准只在停转后交接、双向和安全区输出。
安全区相关函数/常量与基线一致，无新增测试源码/独立程序；夹具IMU/刹车替身仅用于检查，已还原文件。

**未烧录、未打开设备/动车，用户自行烧录。** 报告actual `59d49f7/dirty1`不是本次版本；仅凭该身份无法还原实烧dirty差异。
当前独立工作区包括Task初始化空白及WheelCheck/IDE/工具修改，已保留而未夹带提交；本机构建仍如实dirty1。
软件检查不证明实车停靠精度，3°/s起转、2°/s停转和0.20s预测仍需原入口实车验证；未新增高频统计、时效门或普通FAULT。

## MATCH-SINGLE-CARGO-FILTER-20261115：接受并配套机械收尾，未烧录

已核对上位`e9c1cf382138d83723d8ccf9cacf545b10c32f88`固定入口blob`f9362ec5aac10df64724d645f4e6bb430d526c2f`及实际state_machine/runner，
本端基于`7ad0965c8d2f5a3d13662844dd4fbc22eee2fe66`增量实施。该上位提交仅说明任务，不能当作固件已烧录。

- 真正GRAB后的最终真实合法审核，首件单绿或单橙进入原mode50；绿色保持已合爪，橙色先完成原Claw_Touch再滤出。
  双爪保持合爪，S1到82°，原角度行程等待后Motor_MoveDistanceHeading(-0.100m,250mm/s,本段起始航向)，
  完成后S1回85°并等原机械行程，才交接mode22/完成。每次升降都在发指令前计算Lift_PositionWaitMs。
  用户随后将本段请求150改250，使用独立速度参数；搜索和橙色原半闭后退150保持。
- 执行期间保留原真实审核/帧号/任务归属，mode50闭爪后保持gripper_closed；CARGO_AUDIT完成位不提前发布，NAV仍不能进入。
  重发GRAB/审核/HOLD不重启滤出或补造新帧；实际新抓取重新允许一次，途中复审不重复。PAUSE走原有限运动暂停/续行，
  机械等待沿原pause计时平移；STOP/恢复退出清本段。仅阶段切换Stop，避免每周期重置轮程。
- mode50仍消费新FIRST_GREEN_CORE_CONFLICT：撤原VALID、停止当前滤出、回85°后原23复审；不清平台黑事实、不靠滤出解除冲突。
  暂存、首绿后普通材料、非法/空审核不触发；原橙色半闭复审代码未删改，61°起步限位保持。
- **兼容范围调整**：正式新0x14扫障找回单橙经普通抓取，覆盖本动作。已停用旧0x13的safe_sweep_recheck_pending出口仍40且不插滤出，
  因当前RDK_has_committed_audit把旧扫障角色与mode40绑定，直接扩到50会缺pending。保持旧出口，不重新启用或单端改变该协议等待。
  若将来启用旧扫障，需两端另审mode50角色；本轮正式新流程无需上位新增字段/握手。
- 原接口布局不变，mode50扩充含义为“重新合爪或闭爪高度滤出收尾”，上位应继续保持已提交审核等待22，不能只凭闭爪/ACK发NAV。
  没有新TYPE、等待图像、计时资格、巡检、ISR或额外FAULT；有限直行错误沿原恢复处理，PID/全局速度/公差不改。

必要结果：两原构建通过；原Task夹具直接调用核对单绿/单橙mode50、闭爪82、固定航向100mm、占位保留审核、PAUSE续行、
回85后22、重复不重走/新抓重新允许、冲突不吞并回23，以及暂存/普通材料/非法/旧40排除、STOP清理。夹具电机为替身，
不等于实车100mm或三棱锥滤出成功；没有新增测试源码/独立程序。

本轮未烧录、未开UART/SDK或动作，actual未亲自读回；上位18:49的7ad0965/dirty1仅历史记录。保留原Task初始化空白，构建如实dirty1。
用户自行烧新NormalRun并使用最新正式入口；代价是每次符合条件抓取多升降和100mm后退，实际返航起点由原动态定位更新。

## MATCH-BATCH-EXIT / MATCH-FIRST-GREEN-CORE-20261115：接受并已实现，用户自行烧录

先审查RDK `2c839793db6a76ab078c5ebcec097bf01902bed0`的弃批预算同步，再核对工作期间新增的
`0d45d94149a0a83cb28f97a703b311b7ca0f7dd1`首绿平台黑配套，接受当前两项最小方案；
最新固定入口blob`846122ebfaba91b13b2021ec26ee01bbe0c5ec80`，首绿机器合同blob`ee1cbd02d119af68352e543bdf8d745d1cb6e6d7`。
本端从`a0a1c5d635eca77431129dd4bd38998f2385fd28`增量实施，不重做step13..15、审核复用或搜索150。

- 真实稳定空审核先走原开爪，到原Claw_Open完成才清该批捕获/分离/审核重试预算并SEARCH；
  明确弃批仍完成原释放和后退脱离，原强制mode47返中分支就地结束，不再重新合空爪或为清预算返中。
  原非返中分支90°脱离保持。占位、单样本、未结束机械、mode24/51中间态及有载不能提前清。
- 旧context仍cancelled，task/action/序号判重高水位保留；实际结束后新task可即接APP，旧任务接近包仍拒绝。
  新RETURN在空爪SEARCH即接，清其继承的旧取消位；完成的同RETURN只ACK、不重返。
  原1500ms ACK宽限由动作身份替代，不新增年龄/时间窗口；普通投送返程、原排序/回绕/STOP/PAUSE保持。
- 原0x18/opcode0x12审核P5 bit7=0x80 FIRST_GREEN_CORE_CONFLICT（整帧索引9），P1和真实类型/左右计数/总数/帧号不变。
  frame0全零非STABLE占位仅撤首绿运输资格，不采样/造READY/VALID。独立持物否决不因换角、漏黑、清审或mode51清除。
- mode22持载接原23复审，入区前NAV/ALIGN接原140°mode48，重复请求不重开相机；旧单绿VALID撤销。
  当前真1绿仍报1绿，原两帧/100ms/STABLE可形成READY但不可VALID；旧同图STABLE不能恢复资格。
  原SIDE_VALID可保留真实绿侧1/另一侧0，原单件握持/曲线保持，不补黑、不置DANGER/UNKNOWN、不冒用近爪蓝标志。
- 只有本次选择性曲线实际完成，或整批实际释放/空爪结束才清否决并重新真实审核；观察、拒绝、未完成和旧/重复DONE不清。
  初始暂存、首绿完成后的普通材料及已接受ENTER保持；无新全局FAULT/永久停机、额外握手、计时资格、ISR或诊断框架。

必要结果：NormalRun/MotionDebug构建通过；复用原Task夹具直接核对物资/伤员真实空退出、新task即接、旧包拒绝、
未完成/占位/有载不清、新RETURN各扫描阶段即接及已完成RETURN只ACK；首绿metadata/真计数/旧STABLE/清审/左右单件分离/
观察与旧DONE/重新真实审核/NAV和ALIGN复审/已接受ENTER/普通材料/暂存/mode51保持证据均符合预期。
新位两组成对例CRC/字段通过；夹具为硬件替身，仅验证控制调用和状态衔接，不是地面/机械验收。
无新增检查源码/独立程序；PID/速度/PWM/舵机、step15本地轮程和其他导航动作保持。

**未烧录、未打开UART/SDK或发实车动作，actual未亲自新读；用户自行烧录。** RDK引用15:52实际a0a1c5d/dirty1仅历史回执，
不能当本轮部署证明；本机构建保留原Task初始化空白，如实dirty1，源码以本文件Git记录为准。
用户需烧新NormalRun/TASK并重开RDK最新入口；旧a0a1c5d不支持本轮预算同步及审核0x80语义。
此前120°搜索入口与失败目标关联候选、其他上位REJECT出口及安全区无框策略，本轮不夹带扩改。

- [本轮完整说明与边界](F405_BATCH_AND_FIRST_GREEN_20261115.md)
- [首绿审核新位机器合同及两组成对报文](contracts/f405_first_green_core_20261115.json)
- [固定上位交接](https://github.com/danmo-teng/shijue_fangan/blob/0d45d94149a0a83cb28f97a703b311b7ca0f7dd1/docs/CROSS_REPO_REQUESTS.md)

## MATCH-ATTACK-BACK100-WHEEL-20261115：接受并已实现，用户自行烧录

已读取RDK最新固定入口、返程说明/合同及attack_return/obstacle_route/protocol/state_machine/runner，
固定源提交`8709e43c62148197d94923db8af6af76873138f3`；入口blob`ecbe915a21cac6b4bdeda5f158d28b8de3864586`，
机器合同blob`b91c65e20afb63fe0b0ad1c84435743dd4bf8352`。本端基于`cd0d3297673613468ecacab42ef17512ed6fabbe`增量实现。

**接受仅首段100mm有限轮程后退，后三段保持原定位导航；不扩改转向/制动或上轮分析候选。**

- 原TYPE0x18/opcode0x14追加15/ATTACK_BACK_100：P1红F9/蓝F1，A固定有符号-100mm、B=0、U红27000/蓝9000。
  原0..14接收校验保持，step3仍仅-200mm原扫障航向；串口921600、原配对/完成回执不变。
- 只从本任务已完成RELEASE_BOTH/mode34且逻辑空爪进入15，复用原首次空载交接清审核/预算一次；
  不重放松爪、不记首绿/送达，无SEARCH随意发起15的入口，任务/action排序和编号回绕保持。
- 直接复用原BACK_200的Motor_MoveDistanceHeading：-0.100m、原350mm/s请求、固定U；进度为本段M1/M2有符号编码器差。
  motor.c未改，原3mm公差、起点/制动/加减速/PAUSE恢复/错误处理保持；地图XY/path_mm或纯转动不计满本段。
  同action重发不重置起点，DONE后不走第二遍，仍本次mode25/COMPLETED。
- 允许本任务完成15后的下一新action进入13 LEFT_500；后三段原目标、5cm、H/D和D0/配对DONE保持，最后仍12/SEARCH。
  原13→13及SEARCH/释放入口保持；新15不受地图D0提前结束。旧step14和回抓单伤员审核复用不重复修改。
- 搜索150、其他速度/PID/PWM、舵机、普通RETURN、STOP/mode51、ISR/遥测/时间政策不变；
  无新增测试源码/独立程序、资格等待、握手、统计或FAULT。本轮未扩改上位REJECT出口/安全区无框策略。

必要结果：NormalRun/MotionDebug构建、原FLOW_20261019_CHECK通过；原Task夹具直接调用核对红蓝mode34→15、
  参数/地图变化/重发/PAUSE/本次DONE不重走、15→13动态→D0→12以及错误入口拒绝。14组原机器示例CRC/字段通过。
夹具电机反馈为替身，只证明调用与状态交接；motor.c/配置及旧接收case差异核对为空，地面100mm/轮滑/转向余摆仍待人实测。

**未烧录、未打开UART/SDK或执行实车动作，actual未亲自新读；用户自行烧录。** 上位18:43记录actual f14d2a4/dirty1仅历史身份，
dirty差异未知，不能当新15部署证明。本机构建保留原Task初始化空白，如实dirty1；最终源码以本文件Git记录为准。
需用户烧新NormalRun/TASK并重开RDK最新正式入口，旧f14d2a4/cd0d329无法执行新15。

- [完整实现与检查边界](F405_ATTACK_BACK100_20261115.md)
- [机器合同v3及原报文](contracts/f405_empty_body_move_20261114.json)
- [固定上位请求](https://github.com/danmo-teng/shijue_fangan/blob/8709e43c62148197d94923db8af6af76873138f3/docs/CROSS_REPO_REQUESTS.md)

## MATCH-ATTACK-FACING / MATCH-RECOVERED-INJURY-AUDIT-20261114：接受并已实现，用户自行烧录

已读取RDK固定入口及详细任务/机器合同，核对protocol/state_machine/runner/路线源码，
固定快照`b1b3531782b95d65cc7b4a24927935344ee98e7d`，入口blob`813a4503a2afd20eb917c866cf66a9494c329ef2`；
本端基于`63d75a88c4e13cce11fb8731aca14383269e8ce5`增量配套，不重做此前step13/SEARCH/聚堆近爪实现。
进攻机器合同blob`c48a89934dc2aab9fd5988b0d9b0e7131e6f9e1e`，回抓机器合同blob`91e7581d72930a1641bef9a59a5bfc108cab0267`。

**接受当前两项最小方案；少一次末端复核的取舍仅限上位确认回抓来源的单伤员，普通运输不变。**

- 原0x18/opcode0x14追加step14：红270°/蓝90°、A=B=0、P1红0xE9/蓝0xE1。
  只从本任务已完成initial-stash NAV、真实合法审核、闭爪进入，复用原连续回正200～400mm/s及3°窗口，
  完成仍mode25/本次COMPLETED；旧NAV DONE不能完成新转向，同action重发不重开。
  带载转正保持真实审核/闭爪/route_to_stash，随后原RELEASE_BOTH→mode34→step13→step12；不重复放货或记送达。
- STAGE NAV bit5=REUSE_POST_GRAB_AUDIT，仅原mode22真实单伤员、闭爪、非藏点、伤员目的地审核可授权。
  找回来源由RDK负责，本端无独立来源字段；不能把任意普通伤员都自动复用。
  D0仍mode10/DONE并保留真实审核，只有获授权的完成NAV能接下一条原pose ALIGN，沿原task_begin_safe_align。
  普通/不合资格/晚加标志仍原0x12→mode48，不改变H/D/粗末段/速度、容差、制动或伪造READY/VALID。
- 只变bit5可同action撤销；当前runner按flags换新action也保留原nav_coarse基准/阶段，不重跑高速段。
  撤销后再加位不能恢复授权；原0x12、清审核、释放/分离、恢复、SEARCH、换目标、ABORT清授权。
  新旧task/action排序及其他flags不可变规则保留，不加时间窗口/资格等待。
- 上位两帧缺框ALIGN缓存及确认原伤员/物资丢失后24/47/实际空爪SEARCH出口已有兼容，本端不重复加动作/协议。
  原15s审核恢复、STOP/PAUSE/mode51、step0..13参数、PID/速度/PWM/舵机/串口/ISR/遥测均保持。

必要结果：两原构建通过，原FLOW_20261019_CHECK通过；原Task夹具直接核对红蓝真实审核到ALIGN/ENTER、
普通/标记物资/晚加位不能跳mode48及原占位入口，同/新action撤销不重启、各原退出清授权；
藏点NAV→新转向→原放货mode34→13、重复幂等及错误入口拒绝；原示例CRC/字段核对通过。
没有新增测试代码/独立程序、时间门、握手、巡检、FAULT或ISR。

**未烧录、未连接设备或发动作，actual未亲自新读；用户自行烧录。** 上位16:40的63d75a8/dirty1仅历史身份。
本机构建保留原Task初始化空白，如实dirty1；最终源码以本文件Git记录为准。转向/载荷保持和真实比赛仍待人实测。
需新NormalRun/TASK并重开最新RDK正式入口，旧63d75a8不支持本次新位/step14，不能称已部署。

- [完整实现/调用链/检查边界](F405_ATTACK_AUDIT_ADAPTATION_20261114.md)
- [step14及原step13机器合同](contracts/f405_empty_body_move_20261114.json)
- [回抓单伤员机器合同](contracts/f405_recovered_injury_audit_20261114.json)
- [固定上位入口](https://github.com/danmo-teng/shijue_fangan/blob/b1b3531782b95d65cc7b4a24927935344ee98e7d/docs/CROSS_REPO_REQUESTS.md)

## MATCH-CAPTURE-HANDOFF-20261114：接受并已实现，用户自行烧录

已读取RDK固定入口、详细说明/机器合同及协议/状态机，并核对固定快照`a13a66d87d55437a542bcc2c5a126a40e8c2c8e9`；
合同blob`d3d331c9f9c58efa0e96433170497543bd80bf72`，本端基于`6ee665b64dceddc36ef9936adfd78f115cdd1ff1`增量配套。
上位引用14:59 UART实际6ee665b/dirty1属上场记录，本窗口未新读actual，不能以dirty1认定板内字节与Git完全一致。

**接受本轮五项最小修复，45%绿色ROI与近爪几何仍由RDK负责；下位不接面积、不改真实审核数量/运输规则。**

- mode52去掉!cluster例外，真实新像素>原64px也使聚堆回TRACK；保留CLUSTER/task/action/测距角/预算和已准备爪姿，不重启整个Task。
- 仅mode53，Linear结果RUNNING且剩余=本次目标-Motor_GetDistanceTravelledMm()≤10mm时走原DONE的Stop/140°/审核交接。
  全局3mm及其他运动/导航/扫障保持，12s只是原兜底，不用时间/目标速度/T265冒充轮程，不加停滞检测。
- 原0x18/opcode0x10的P1 bit4=0x10命名DISPERSE_NEAR_KEEP；仅补SIDE_VALID、禁FIRST_GREEN_BUMP及A/B/U零组合校验。
  通用mask原来已允许bit4，不新增TYPE/opcode或握手；该位不是0x14的step，也不是0x12的audit STABLE。
- 稳定非法、非空含蓝审核继续输出真实READY，普通mode21、聚堆沿原流程进入37，供上位选择分离或原纯蓝释放。
  只排除这时“无合格载荷”的立即退让，原4s决策和15s审核兜底不延长，不加最低等待，不置非法VALID。
- near标志仅在原稳定非法危险审核、所选侧真实计数0且另一侧>0时允许选侧；数量/直方图/单件掩码/首绿不改。
  保留侧0自然使counts_known=false，沿原5°软握/10°曲线；正常已确认单件20°握持不改，分离后仍须真实合格审核及GRAB才能运输。
- 既有step13/SEARCH入口、普通DISPERSE/释放、速度/PID/PWM、舵机、串口/ISR/时间政策和STOP/PAUSE/mode51保持。
  本轮不含mode52取消或其他历史候选，无新时间资格/遥测/永久FAULT/独立测试程序；原夹具只补新增getter的必要链接替身，无新用例。

必要结果：NormalRun/MotionDebug编译通过，原FLOW_20261019_CHECK通过。原夹具独立条件核对普通/聚堆10.1/10/5.1mm、原故障恢复，
mode52新帧74px/64px/重复帧及预算/CLUSTER保持；两帧非法蓝READY21/37、左右近爪5°/真实计数/掩码/VALID不变、重发不加尝试与直接GRAB拒绝。
独立核对原4s兜底、纯蓝上位释放、无标志/错侧拒绝和普通20°握持；原成对例CRC/字段核对通过，不宣称整套旧套件或实车精度通过。

**本端未烧录、未打开UART/设备或执行动作，按用户约定自行烧录；actual未新读。** 构建dirty1保留原Task初始化空白，
最终源码以本文件Git记录为准。需烧新NormalRun/TASK并重开最新正式入口；旧固件无法完整执行本次聚堆复核与近爪标志。
近爪旁证不保证已夹持，轮滑/卡料、10mm末端及机械分离成功率仍需人安排实跑，不能以软件DONE/编码通过替代。

- [完整下位合同与检查边界](F405_CAPTURE_HANDOFF_20261114.md)
- [机器合同、原左右成对示例](contracts/f405_near_claw_separation_20261114.json)
- [固定上位任务](https://github.com/danmo-teng/shijue_fangan/blob/a13a66d87d55437a542bcc2c5a126a40e8c2c8e9/docs/COMPETITION_CAPTURE_HANDOFF_20261114.md)

## MATCH-SEARCH-ROTATION-20261114：接受并已补空爪SEARCH入口，用户自行烧录

已读取RDK最新固定入口/说明/机器合同并核对源码快照`8db11ec29b297d8c00173f42ecc6b371c01958dd`，
搜索站合同blob为`1ff8bbe3bd026a78f5165f620b83c08fde6c527c`；本端基于`f66edbc0bbaf06d4cfb52c6595fcfda564554107`增量修改。

**审查接受原最小改法，只补Task两处入口；没有重做step13接收/报文/执行器。**

- task_accept_bulldozer允许空爪SEARCH按原step0的编号规则进入13，不要求虚构mode34放货回执；仅首次清放弃批次预算/复审/退让，
  不开合爪、不记首绿/送达，后续心跳/同actionH-D及换段沿原逻辑，不重复清理。
- task_accept_mission的空爪SEARCH/new-action/cancelled-context例外从step0扩到0或13，避免前置条件仍拒绝。
  持货、相同/旧action、STOP/mode51及非SEARCH抓取中不借新入口放行；原任务/编号回绕和新目标预算规则不改。
- 原step13执行器、U不可变/D-H动态、D0/mode25完成及step12/mode3/COMPLETED保持。途中目标让位的D0只结束移动，
  不设置NAV到站；原新帧搜索→接近→整爪审核/GRAB保持。PID/速度、标准清障1m、舵机、原0..12/普通RETURN不改。
- 已核对RDK search_transfer_epoch/index进入action键，且当前点已在目标内时仍先发step13 D0再原step12；无额外下位入口需求。
  站点±0.30m/侧通道±0.50m、累计搜索及失败历史均由上位实现，下位不硬编码路线、不复制目标黑名单。
- 没有新时间/新鲜度门、握手、ISR、遥测、FAULT或测试程序；本轮不含mode52取消或其他历史待办。

必要结果：NormalRun/MotionDebug构建通过；原Task夹具核对红蓝空爪SEARCH（含cancelled及遗留预算）、D/H重发、
途中D0→原step12→正常APP、持货/相同/旧action拒绝、STOP/mode51/APP边界、原step0/mode34及编号回绕符合预期。
原FLOW_20261019_CHECK及10组成对CRC/字段核对通过，接收/枚举及整个执行函数差异为空，未新增检查源码或独立测试程序。

**用户明确“烧录我自己来”；本端未烧录、未打开UART/设备或执行动作，actual未读。** 本机构建dirty1保留原Task初始化空白，
源码提交以本文件Git记录为准。仅上一轮step13的mode34入口固件不支持本轮SEARCH入口；需用户烧新NormalRun/TASK，
再重开完整正式待机入口。地面轮换/碰撞与目标识别效果未验收，不把软件配对/DONE当作实车通过。

- [搜索站机器合同及10组成对报文](contracts/f405_search_station_20261114.json)
- [step13完整合同与新增入口](F405_EMPTY_BODY_MOVE_20261114.md)
- [固定上位任务](https://github.com/danmo-teng/shijue_fangan/blob/8db11ec29b297d8c00173f42ecc6b371c01958dd/docs/COMPETITION_SEARCH_ROTATION_20261114.md)

## MATCH-ATTACK-RETURN-20261114：接受并已实现step13，未烧录

已直接读取RDK固定入口/详细说明/机器合同并核对固定源码快照`a8d1242ac19b0e69d9c709be80adcfdafc76e1c5`；
机器合同blob为`4610fe8cb88e93313087a38e210224c9f57a5972`。本端从`f1f8f183c5f14fb24462a76834b0ab0613e75967`增量实现。

**审查结论：接受复用原0x18/opcode0x14、追加step13，不调整编号/字段或路线分工。**

- 原枚举末尾追加EMPTY_BODY_MOVE=13，单独允许U0..35999；A剩余D0..5000mm、B场地H±18000厘度，旧0..12验证保持。
- 复用原task_bulldozer_live_step/Motor_MoveAngleFineHeading，body_angle=wrap(H-当前主yaw)，U独立固定，速度250～350mm/s。
  不先朝H转头、不套旧粗导航、不按首个D提前完成；同action更新H/D不重启且U不可改，D=0原刹车后mode25/COMPLETED。
- 首个13仅从本任务已完成RELEASE_BOTH/mode34且逻辑空载进入，只首次清放货审核/STABLE、stash_backoff/旧批次预算，
  不重新开合爪、不记首绿/送达。后续13由本任务上一13完成后换action，保留原FFFF→0/task+1编号回绕。
  最后原step12恢复SEARCH并保留成对完成回执。原0..12执行函数、step8标准直行、PID/PWM/速度/舵机/普通RETURN/mode51保持。
- 已核对RDK runner把返程段index加入action键：同段不换ID、换段新action、task保持；无需新增握手或报文。
  扫障S上位59cm/94cm及红(-.56,+.91)/蓝(+.56,-.91)仅同步文档/机器样例，固件不硬编码坐标。
- 无新时间门、年龄/资格检查、反复握手、ISR、遥测、永久FAULT或测试程序；本请求不含mode52取消或旧候选。

必要结果：NormalRun/MotionDebug编译通过；原Task夹具直接调试红蓝四段、固定U/机体系方向、D>0/D0、重复/暂停、
U不可改、首次清理/首绿不误记与step12 SEARCH符合预期；任意U359.99、入口归属/完成/空载和编号回绕、原测距流程通过。
12组上位成对原报文CRC/字段核对通过，差异证明旧0..12校验及整个执行函数未变；未新增检查源码或独立测试程序。

**本轮未烧录、未启UART/设备或动作，板端actual未读。** 本机构建dirty1保留原Task初始化空白，最终源码提交以本文件Git记录为准。
上位旧记录f40b061/dirty1仅属其历史场次，不当作本轮新固件actual。部署需烧新NormalRun/TASK并读真实身份，
再重开RDK整个正式待机入口；旧0..12固件不能跑13。四段轨迹/打滑/地面5cm精度待人安排实车，不把软件DONE当验收。

- [下位完整step13合同](F405_EMPTY_BODY_MOVE_20261114.md)
- [机器JSON及12组成对报文](contracts/f405_empty_body_move_20261114.json)
- [固定上位任务](https://github.com/danmo-teng/shijue_fangan/blob/a8d1242ac19b0e69d9c709be80adcfdafc76e1c5/docs/COMPETITION_ATTACK_RETURN_20261114.md)

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
