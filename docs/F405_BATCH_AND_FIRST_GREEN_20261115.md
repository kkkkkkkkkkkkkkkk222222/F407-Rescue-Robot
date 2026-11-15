# 弃批后就地重选与首绿平台黑冲突

本端基线`a0a1c5d635eca77431129dd4bd38998f2385fd28`。先审查RDK `2c839793db6a76ab078c5ebcec097bf01902bed0`的
MATCH-BATCH-EXIT-20261115；工作期间对端更新至`0d45d94149a0a83cb28f97a703b311b7ca0f7dd1`，
已另读其固定入口、FIRST-GREEN说明/机器合同及实际协议/状态机增量，接受当前两项最小配套。
前轮step15、150mm/s搜索及舵机成果保留。本轮未连接设备、烧录或发机构/车轮动作，actual未亲自新读。

## 1. 弃批后原地结束旧预算

上位交接引用15:52日志：分离后真实空审核回SEARCH，532.517已发接近24984而被旧上下文拒绝；
RETURN24985又等约两圈才接受。此处不是“上位始终未选中”，也不是候选确认帧数导致。
先前只按源码提出的120°搜索入口等待是另一个条件性候选；本轮按这场已有证据修旧批预算和RETURN，不顺带改其相机入口。

- 真实两帧/STABLE空审核转原OPEN_CLAW时，只记录待清理；原Claw_Open完成才复用现有载荷/批次清理并进入SEARCH。
  非STABLE/frame0占位、只有一个样本、未完成分离或有载不会借此清预算。开局静止观察/暂存非空不改。
- 明确弃批沿原开爪/后退脱离，原计划强制mode47返中的分支在完成后直接SEARCH，不再重新合空爪/为清预算返中。
  原非返中分支的90°脱离动作保持，完成后也清本批预算；不增加新的开爪、后退、等待或运动。
- 清捕获/分离计数、旧审核/复审/重试，保留accepted_context的task/action和原序号高水位，旧任务仍cancelled。
  新task可立即选择下一目标；旧task的接近包仍拒绝，mode24中间态不提前开放新目标。
- 空爪SEARCH中合法新RETURN立即进入原返程，不要求SEARCH_WAIT_RETURN或两圈；该新RETURN不继承旧取消位。
  已完成同task/action RETURN只ACK、不再返中；按动作身份替代原1500ms ACK宽限分支，不增加时限或握手。
  普通投送返程/原0x08字段、排序、回绕、STOP/PAUSE和mode51继续沿原实现。

## 2. 首绿已见平台黑：撤运输资格，保留真实数量

RDK报告15:52原120°平台图2077/2080已见core195，140°仅检到绿色154一件；这项是已见持物冲突，不能用随后漏检抹掉。
本端采用原0x18/opcode0x12的审核P5 bit7=0x80 FIRST_GREEN_CORE_CONFLICT（整帧索引9），不是P1命令位。
类型/数量/左右计数/总数、audit_id、源图帧及0x1D/0x1E配对均不改；不补造第二件、DANGER、UNKNOWN或单件确认。

- bit7独立保存于当前首绿持物状态，普通clear_audit、换角/漏黑和mode51恢复不清。frame0全零非STABLE可撤资格，
  只是元数据，不是实测空、不累计样本、不制造READY/VALID。
- mode22持载收到有效撤资格请求，接原mode23合爪复审；入区前NAV/ALIGN接原140°mode48。
  仅首次进入原审核动作；重复占位不重开相机。旧单绿VALID立即失效，不再沿旧NAV/ENTER放行，无全局FAULT。
- 后续原真实帧仍须本地两帧、100ms及上位STABLE。当前真1绿仍报1绿；负资格存在时可READY但不可VALID。
  旧同图STABLE确认不能恢复旧VALID。SIDE_VALID可选择真实绿色侧1、另一侧0，沿原真实单件握持/曲线，不冒用近爪蓝标志。
- 只在本次SIDE_VALID选择性曲线完成，或原整批实际开爪/空爪结束时清负资格，然后走真实新图复审。
  观察转向、动作未完成、被拒、旧/重复DONE和清审核不会清。首绿完成后的普通材料及INITIAL_STASH保持原政策。
- 边界为ENTER接受前；已接受本地直推不被迟到负资格包改成反复ALIGN/停机。对端ENTER提交优先级保持。

原15字节封包、CRC16/MODBUS、UART4 921600/8N1、协议1.3遥测及现有动作接口不变。
新位的两组成对示例是上位离线合成，未实发，见[机器合同](contracts/f405_first_green_core_20261115.json)。

## 必要结果与部署

- NormalRun/MotionDebug构建链接通过。只复用原Task夹具进行直接调用，无新增测试源码/独立程序。
  物资/伤员稳定空→开爪等待→空爪SEARCH→新task立即APP通过；占位/单样本/未完成分离/mode24/mode51/有载不提前清；
  弃批原释放/300mm脱离完成后清预算、无重新合爪/mode47；扫描各阶段新RETURN即接、同已完成RETURN只ACK及旧包拒绝通过。
- 首绿mode22/NAV/ALIGN撤资格、frame0不采样、清审/漏黑/旧STABLE保持否决、真1绿READY非法，
  左右实际单件20°握持、未完成/观察/旧DONE不清、选择性完成后真实新图复审通过；
  已接受ENTER、普通材料、INITIAL_STASH及mode51保留证据符合预期。夹具使用电机/舵机替身，不是地面/机械效果验收。
- 两组成对新报文CRC/字段核对通过。速度/PID/PWM/舵机、step13..15运动、普通NAV/ALIGN、原协议接收、ISR及时间政策不改；
  无新计时资格、新鲜度门、反复握手、巡检或永久FAULT。原独立工具/IDE及Task初始化空白保留，构建如实dirty1。
- 用户需烧新`build/F405/NormalRun/NormalRun.elf`并重开最新RDK正式入口；旧a0a1c5d不支持本轮批预算清理及0x80业务语义。
  对端引用15:52实际a0a1c5d/dirty1仅历史回执，dirty字节未知；本窗口未烧录/读取actual，源码/构建不能冒充实烧。
  原120°搜索入口、失败目标关联、安全区无框策略及上位其他REJECT出口不是本轮扩改项。

源依据：[弃批说明](https://github.com/danmo-teng/shijue_fangan/blob/0d45d94149a0a83cb28f97a703b311b7ca0f7dd1/docs/COMPETITION_BATCH_EXIT_20261115.md)、
[首绿说明](https://github.com/danmo-teng/shijue_fangan/blob/0d45d94149a0a83cb28f97a703b311b7ca0f7dd1/docs/COMPETITION_FIRST_GREEN_CORE_20261115.md)。
