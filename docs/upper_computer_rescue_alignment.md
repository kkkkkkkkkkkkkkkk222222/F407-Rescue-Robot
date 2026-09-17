# 上位机救援流程对接要求

对照基线：`danmo-teng/shijue_fangan`分支`codex/gamepad-teleop`提交`666758f`。
下位机已实现上位机当前600 mm预备点、两次ALIGN、锁存航向ENTER和200 mm本地编码器补推；安全区退出与返中流程保持原样。

> 最新抓取握手覆盖本文后面的历史两帧描述：抓前mode21/mode38/mode37及分离复审统一等待3个不同audit_id语义一致，合法后F407置`AUDIT_VALID(bit4)`才接受GRAB。含核心/mixed时先前进50 mm再Touch；合爪后进入mode23并清空旧审核，再取得3帧。抓后合法进入mode22并置AUDIT_VALID，mode22才允许NAV；空爪回SEARCH，非法留在mode23执行释放/分离。STABLE不能跳过三帧。
>
> mode23有限恢复期间F407会短暂报告138°或142°，上位机不得发送HOLD/PAUSE冻结本地恢复；继续发送同audit_id的非STABLE空审核即可。检测到角度从非140°回到140°时，必须重新设置frame floor并清空audit_hits/signature，确保只累计恢复后的3张新帧。若F407最终双开回mode3，上位机清空抓取批次并重新SEARCH。
>
> 审核签名必须与F407一致：initial stash非空、FIRST_GREEN、`MATERIAL_LEGAL+total_count`、INJURY_SINGLE分别归一化；非法审核只比较`total_count`与DANGER/UNKNOWN/INJURY_MIXED/目的地等语义flags，不比较mixed与左右拆分的具体表达。合法三帧中只要任意一帧含core或mixed，上位机诊断应记录`core_seen_in_streak=true`，并预期F407执行50 mm前移。

## 1. SEARCH职责

- F407自主完成摄像头120°一圈、90°一圈。两圈均无目标后F407保持mode3停车；上位机看到本轮累计约720°且无候选时发送RETURN_CENTER H/D，使空爪底盘回到中心，不能继续发送HOLD让其永久等待。
- `6c802ec`一旦锁定候选即可持续发送`APPROACH_TARGET`；F407收到第一帧合法命令便进入APPROACH，不再增加第二套确认门限。
- 上位机在视觉过期、暂未选中目标或等待策略决策时可以发送HOLD，但SEARCH中的HOLD只是任务心跳，F407仍继续本地扫描。若确实要求底盘冻结当前SEARCH阶段，发送新增的`PAUSE=1`，不能再复用HOLD表达两种相反行为。
- 上位机可在两圈未完成时发送普通/聚集APPROACH接管；累计720°仍无目标时必须切入RETURN_CENTER。F407在SEARCH_WAIT_RETURN中只接受RETURN，不再开始第三圈。
- 首件正式绿色仍未完成时，如果140°overall ROI内至少2件且包含绿色、绿色位于堆中无法通过普通带侧曲线取得，上位机可对每个目标堆最多一次发送`DISPERSE_PILE | FIRST_GREEN_BUMP(bit5)`。bit5不得和SIDE_VALID/TARGET_RIGHT并存。F407执行后退0.10 m→Touch闭爪→前进0.20 m→后退0.10 m→双开并回SEARCH；上位机动作期间持续同一命令，看到mode3后清除旧cluster/audit/track并重新寻找绿色。第一件绿色完成后永久禁止该标志。
- 普通APPROACH进入`mode=21 + CLAW_VISIBLE=1`后，F407以180 mm/s最多慢爬500 mm。上位机立即切入CAPTURE_AUDIT；合法GRAB必须3张不同audit_id语义一致，超出500 mm仍无确认时识别mode24并恢复SEARCH。

### HOLD、PAUSE、STOP、ABORT语义

- `HOLD=10`：正常流程心跳。SEARCH继续扫描；APPROACH常规跟踪/125°对正、NAV、RETURN和远程动作安全停车。F407进入125→140°稳定阶段后，HOLD不打断已经触发的相机动作；需要真正冻结必须发送PAUSE。mode16若曾被PAUSE冻结，恢复后发送合法RETURN即可解除锁存并继续剩余本地后退。
- `PAUSE=1`：操作员暂停、定位短时不可用但希望保留当前阶段、或上位机内部重建状态时使用。F407 ACK后锁存停车并保持Task状态；帧过期也不会自行恢复，且不会触发DISPERSE的HOLD取消。
- 解除PAUSE必须发送一条当前状态可接受、SEQ递增的非PAUSE命令：SEARCH发HOLD，APPROACH发APPROACH_TARGET，NAV发NAVIGATE_WAYPOINT，RETURN发RETURN_CENTER，远程动作重发原动作命令。无效或阶段不匹配的命令不会解除暂停。
- `STOP=0`不是普通暂停；除NAV兼容入口外会形成远程停止故障。`ABORT=7`进入`REMOTE_STOP/fault1`；操作员下一次明确发送新的连续合法赛前配置可以重启一轮任务，其他故障不能自动清除。

## 2. 初始藏堆

- 项目最终规则覆盖`6c802ec`原有限制：`initial_stash=1`没有物资种类和数量限制，只负责把开局物资堆搬离中心。
- 稳定审核只需满足`total_count>0`。左右2-bit计数字段可能饱和，不要求`left_count + right_count == total_count`，危险、未知或混合类别也不作为本次临时搬堆的拒绝条件。
- 上位机实现上应删除`_pile_batch()`对`max_batch_count`的截断，并在`_audit_valid()`中把`selected_batch.initial_stash`的`total_count>0`判断放到正式投送的数量、类别和计数一致性判断之前。编码时左右计数仍饱和到0～3，P7保留实际总数；F407在initial_stash中不会用饱和计数拒绝动作。
- 这一放宽只适用于临时藏堆；从藏点取回并准备正式投送时必须重新执行完整审核，不能沿用暂存审核结果。
- 藏点`RELEASE_BOTH`完成后，上位机应立即进入`RETURN_CENTER`并持续发送最新H/D。F407会屏蔽残留的`APPROACH_TARGET`和`DISPERSE_PILE`，先保持释放时的车头方向直退0.35 m，再使用最新H调头返中；这段期间上位机不得因mode17尚未开始转向而改发HOLD、重发RELEASE或判定卡住。正常投送的mode16退出流程不受影响。

## 3. 普通与核心混装按上位机批次策略直接运输

为与最新版上位机保持一致，首件绿色物资正式投送完成后，F407允许总数1～3件的普通、核心或`mixed_material`组合直接进入GRAB/NAV。左右分别为`green_supply`和`core_black`同样允许直接抓取，不再进入`INVALID_RELEASE`。危险、未知、伤员混装及超过3件仍必须进入释放或恢复流程。

- 普通`RELEASE_LEFT/RIGHT → YIELD_BACKOFF`单侧释放使用25°强保持：保留左侧为左53°/右72°，保留右侧为左108°/右127°。`DISPERSE_PILE + SIDE_VALID`属于另一条聚集曲线，不论第一次还是mode35复审后的后续动作都使用15°保持：保留左63°或保留右117°。两条语义不得混用。
- 当左右两侧都是当前任务允许的物资但仍需拆成单侧时，先选保留侧：恰好一侧含绿色则保留绿色侧；两侧都含绿色时保留数量较少侧；均不含绿色时也保留数量较少的非空侧；仍相同再按锁定目标数、轨迹稳定度和距离择优。`RELEASE_LEFT/RIGHT`编码的是“打开哪侧”，而`DISPERSE_PILE + SIDE_VALID`中的`TARGET_RIGHT`编码的是“保留哪侧”，两者不能写反。
- 上位机判定无法可靠指定保留侧时，不再发送会导致双开的`RELEASE_BOTH`，而是发送不带`SIDE_VALID/TARGET_RIGHT`的`DISPERSE_PILE`。F407原地转20°、保持相机140°稳定后报告新鲜`mode=35`；上位机必须清除旧审核帧并重新生成夹爪ROI审核。分侧成功后再发送带`SIDE_VALID`的DISPERSE执行0.30 m曲线剥离。观察转向最多允许两次，仍无法分侧时回到聚集目标重新对正，不能无限旋转或恢复正面撞堆。
- 单侧释放完成后发送`YIELD_BACKOFF(-300 mm)`；该距离现在是曲线路径长度，不是直线倒车距离。等待新鲜`mode=30`后重新进入`CAPTURE_AUDIT`，不得直接SEARCH。
- 复审合法时把`selected_batch`替换为实际保留的单件，再发送`GRAB_CONFIRMED`直到F407上报`GRIPPER_CLOSED=1`。被释放物资不计入本次投送，留待后续重新搜索。
- `mode=35`后的复审同时适用于观察转向和选择性曲线分离：观察转向后必须重新选侧，曲线分离后则判断保留物资是否合法；合法时持续发送GRAB，空爪回SEARCH，仍混装则按最新左右审核再决策。

F407会校验释放侧计数：`RELEASE_LEFT/RIGHT`对应侧必须非空；复审失败阶段只接受`RELEASE_BOTH`。命令与最近稳定审核不一致时不会ACK或动作。

## 4. 去放置区与投送确认

- 正式投送NAV必须置`STAGE_ONLY(bit6)`，目标是相应半区安全区入口前600 mm预备点；持续发送几何H/D直到新鲜`mode=10 + DISTANCE_DONE + ACK变化`。F407现已兼容上位机`952853c`当前使用的精简flags `VALID|DISTANCE_VALID|STAGE_ONLY|阵营位`，也兼容旧版同时携带`DRIVE_STRAIGHT|USE_FINAL_HEADING`的完整形式；不要只设置两个方向位中的一个。F407在首次D=0时立即把摄像头抬到120°；不要在该点发送旧式ENTER，F407明确禁止预备点补推。
- 第一次ALIGN置`USE_FINAL_HEADING`，P6/P7始终发送红方9000或蓝方27000；所有投送使用相同航向，不再区分首趟偏置。F407会保证摄像头120°命令后至少稳定300 ms再上报完成，持续发送到新鲜`mode=11 + ACK变化`。
- 连续3个不同新视觉帧冻结安全区框；第二次ALIGN置`VISUAL_CORRECTION_VALID(bit6)`，P2/P3发送`target_x_px-640`的有符号像素误差，P4/P5/P6/P7为0。持续发送到第二个新鲜`mode=11 + 本阶段ACK变化`。5秒仍无法冻结时跳过第二次ALIGN，沿第一次航向进入回退ENTER。
- `STAGE_ONLY NAV`不能只在上位机`_at_target()`成立后才处理下位机完成状态。F407现在会在本段编码器达到`首帧D+100 mm`且最新`D<=30 mm`时置`DISTANCE_DONE`；上位机看到新鲜`mode=10 + DISTANCE_DONE + GRIPPER_CLOSED`且本阶段NAV已被ACK后，应锁存预备点完成、补发并确认一次STAGE `D=0`，然后进入ALIGN。否则下位机已停车完成而上位机仍持续发送动态D，两端仍可能卡在NAV。
- ENTER对齐`codex/gamepad-teleop@b827303`：始终置`DRIVE_STRAIGHT`且清除`DISTANCE_VALID`，P2/P3和P4/P5全部为0。视觉修正成功时置bit6、清除`USE_FINAL_HEADING`且P6/P7=0；定位降级时清除bit6、置`USE_FINAL_HEADING`且发送红9000/蓝27000。蓝方常用flags为0x43/0x07，红方为0x4B/0x0F。
- F407首次收到合法ENTER并取得有效LocationPose时只锁存一次编码器起点。前400 mm复用现有400→200 mm/s接近曲线，随后以300 mm/s补推200 mm，编码器总目标为600 mm，最终补推最长1200 ms。重复ENTER只ACK，不重置距离。电机停车、双爪完全打开且相机120°稳定300 ms后才上报mode15。
- `6c802ec`第二次视觉确认超时后会永久停在ENTER。上位机应增加有限兜底：F407已新鲜处于`mode=15`、观察窗口结束且没有明确“目标仍在安全区外”的证据时，发送并持续保持`TASK_COMPLETE`；若明确仍在区外则保持停车并报告人工处理，不得伪造完成。
- 第二次视觉ALIGN只叠加一次冻结框修正；ENTER无视觉回退时仍发送9000/27000，F407继续使用第一次ALIGN锁存的90°/270°目标。

## 5. 投送后返中（F407现有安全区退出不修改）

1. 上位机在`RAM_VERIFY(mode=15)`确认投送后持续发送`TASK_COMPLETE`。
2. F407接受后进入`EXIT_SAFE_ZONE(mode=16)`，以400 mm/s后退0.30 m；该动作由本地编码器定距和IMU保持航向。
3. 上位机看到新鲜mode16或mode17后进入`RETURN_CENTER`并持续发送H/D。mode16会ACK和缓存RETURN以解除PAUSE，但不会提前结束0.30 m本地后退；mode17直接使用最新帧正式返中。
4. F407退出完成后进入`FACE_FIELD_CENTER(mode=17)`，先原地转到H误差3°内，再最高800 mm/s向前行驶；途中偏差达到8°才停车重对。
5. 上位机进入中心容差后必须继续发送`RETURN_CENTER D=0`，直到看到F407上报`mode=3 SEARCH`。不能用HOLD代替返中完成。

## 6. 退让与脱困

- `YIELD_BACKOFF`只能在上位机已经看到对应释放命令ACK变化和新鲜`mode=32/33`后发送，并持续发送到新鲜`mode=30`；F407现在拒绝APPROACH、NAV、RETURN以及其他动作后直接到来的YIELD。`RELEASE_BOTH`始终是真实双开并结束于mode34；左右不明的20°观察必须发送无侧DISPERSE并等待mode35。普通脱困改用`ESCAPE_MANEUVER`，不能复用YIELD。
- `ESCAPE_MANEUVER`持续发送到新鲜`mode=31`。F407会先把大舵机恢复85°行驶位置，再执行旋转和横移。
- 远程动作暂时丢帧、但仍允许F407执行既有恢复策略时可发HOLD；若要求动作原地冻结且恢复后从当前阶段继续，应发PAUSE。两者都不能把未完成动作直接标记完成，恢复时继续重复原命令并递增SEQ。
- 聚集目标不能在SEARCH中直接发送`DISPERSE_PILE`。先用`CLUSTER_TARGET`靠近到mode38并完成140°审核，再在mode37发送带侧DISPERSE。第一次以及mode35复审后的所有带侧曲线统一使用15°保持（保留左63°或保留右117°），不升级为25°。25°仅属于普通RELEASE/YIELD单侧释放。稳定空爪回SEARCH，无法分侧才使用无侧20°观察。
- 对齐上位机`db76b00`：普通mode21及聚集/分离复审后最终允许GRAB，都必须取得2个不同`frame_sequence`、内容一致的合法审核；每个新视觉帧使用新的`audit_id`，同帧重复保持不变。无效审核的分侧判断继续只需frame floor后的1张新帧，不应因两帧GRAB门槛拖慢曲线动作。F407本地也对`GRAB_CONFIRMED`强制检查`audit_consistent_count>=2`。

## ACTION/WATCH卡死的必须修复项

- STM32状态`mode=25/ACTION`而LCD显示`CMD:APP REJ`，表示APP帧格式有效但当前阶段拒绝，不是串口断线。上位机必须比较状态帧`acknowledged_sequence`，不能仅凭发送成功或LCD曾显示APP判定已接管。
- 左右不明的观察转向完成并收到新鲜mode35后，进入新的夹爪审核周期：保留批次和目标类别，但清除动作前的审核缓存及帧下限，只接受转向完成后的新帧。新的左右判断成立后发送带侧DISPERSE；空爪才清空批次回SEARCH。
- 删除旧的mode34/mode3撞分完成分支以及无命令的`DISPERSE_RESELECT`等待。观察转向期间必须持续发送本次无侧DISPERSE，mode35之后持续发送新的CARGO_AUDIT，不能用HOLD让F407退出动作。

## 7. 必测回归

1. SEARCH连续HOLD时F407仍扫描；第一帧合法APPROACH只触发一次靠近。
2. 90°/120°扫描任一阶段，上位机都能用普通/聚集APPROACH或藏点NAV接管；聚集目标必须先到mode38完成稳定非空夹内审核，DISPERSE只能在随后mode37发送。
3. initial_stash总数大于0时，即使数量超过3、左右计数饱和或类别混合也能完成暂存；总数为0时不得GRAB。
4. 左绿右核心、左核心右绿分别释放正确一侧，YIELD后重新审核。
5. 左右归属不明时发送无bit6/bit7的DISPERSE触发20°观察转向；mode35后丢弃旧审核并重新发送CARGO_AUDIT，分侧成功后再发送带SIDE_VALID的DISPERSE。
6. 释放命令与审核侧不一致时F407不动作，上位机能够重新审核恢复。
7. 正式投送NAV携带STAGE_ONLY，在600 mm预备点D=0只得到mode10+DISTANCE_DONE，不能出现张爪或本地长距离补推；分别用蓝方`0x51`、红方`0x59`精简flags验证WAITNAV能够立即ACK并进入mode10。
8. 每次投送第一次ALIGN均完成红90°/蓝270°定位对正，第二次只应用一次冻结框像素修正，不再添加首趟±10°。
9. ENTER期间上位机不再发送动态H/D；重复合法ENTER只更新ACK，不改变ALIGN锁存航向、不重置编码器起点或补推进度，完成机构和相机稳定后才出现mode15。
10. mode15视觉确认两次超时后不会无限ENTER：无区外反证时能TASK_COMPLETE，有明确区外反证时安全停车告警。
11. TASK_COMPLETE后依次看到mode16、mode17、mode3；mode17阶段H变化时重新计算朝向，D=0才结束。
12. RETURN中途HOLD只停车，恢复RETURN后继续；HOLD不能伪造回中完成。
13. 临时藏堆释放后发送RETURN，确认F407先直退0.35 m且车头不旋转，随后才按最新H调头；正常投送返中不得多退一次。
14. 聚集APPROACH到相机130°且X误差≤40 px后先转140°并出现mode38；非STABLE空审核不能结束200 mm/s慢爬，稳定非空审核才进入mode37。合法单件可GRAB，多件/混装才DISPERSE；300 mm或2000 ms无确认必须回SEARCH。

## 8. 上位机下一步优化建议

- NAV和RETURN必须以固定频率持续发送新鲜H/D，SEQ逐帧递增；不要因为整数D连续几帧相同就改发HOLD。F407现在会联合D、IMU航向和编码器里程判断短时进展，但上位机仍应以T265/融合位姿作为几何到达的主依据。
- 卡住检测应区分“真实位姿进展”和“车轮空转”：T265平移达到阈值可直接判定有进展；只有编码器增加而T265长期不动应判为疑似打滑，不能一直重置卡住计时。原地对正阶段则单独使用航向变化判断。
- 不要让SEARCH计时、目标确认计时和F407本地90°/120°扫描圈数共同决定同一次状态跳转。上位机负责目标选择和策略，F407负责扫描动作；上位机确认候选后持续发送APPROACH。SEARCH无目标时发送HOLD让扫描继续，只有明确要求冻结时才发送PAUSE。
- `DISTANCE_DONE`只在STM状态帧不超过250 ms、`mode=10 NAV`且`GRIPPER_CLOSED=1`时可用于正式投送到达；RETURN和空爪藏点仍以D=0完成。过期状态、错误mode或未闭爪状态不得触发ENTER。
- 投送复核必须有有限结论：新鲜`mode=15`且无明确区外残留证据时，观察超时后发送TASK_COMPLETE；若明确物资仍在区外则停车告警。不能永久重复ENTER或永久等待视觉重新出现。
- RETURN_CENTER持续发送到F407上报`mode=3 SEARCH`，中心附近也要发送D=0，不能用HOLD或地图圆周容差提前结束。
- RETURN完成握手不能只在最后比较`acknowledged_sequence != return_initial_ack`：任务SEQ只有8位，100 Hz发送约2.56 s就会回绕。进入RETURN时清零`return_command_accepted`；本阶段确认已发送RETURN且观察到一次对应ACK后永久锁存为真。之后收到新鲜`mode=3`且该锁存为真即可进入上位机SEARCH。F407目前会在返中完成后的1500 ms内继续ACK重复RETURN作为旧版本兼容，但正确性不能依赖这个窗口。
- 正式投送必须采用STAGE NAV→定位ALIGN→可选视觉ALIGN→ENTER，不能再沿用D≤50 mm本地法向锁存或在预备点直接发送ENTER。第二次ALIGN的P2/P3保持发送有符号像素误差；不要擅自改成0.01°角度，除非同步修改F407协议和标定。
- initial_stash只要求稳定且`total_count>0`；正式投送继续执行危险、未知、伤员混装和总数检查，首件绿色完成后接受普通、核心及`mixed_material`批次。单侧分离后必须YIELD、重新审核，合法后再GRAB，不能直接回SEARCH。
- 摄像头3到140°后，只有140°专用overall/left/right夹爪ROI内的物体组成当前待分离物资堆；全局画面只负责SEARCH和APPROACH，不能参与夹内数量或分侧。保留侧采用确定性暴力策略：仅一侧有绿色则保留该侧；两侧都有绿色则保留绿色数量较少侧，平局保留左侧；两侧都无绿色则保留左侧，左侧为空才保留右侧。随后发送`DISPERSE_PILE + SIDE_VALID`。mode35后F407置`CLAW_VISIBLE`，上位机设置新frame floor并只用动作后的140°ROI帧重新构建物资堆；合法则GRAB，仍不合法继续带侧曲线，空爪才回SEARCH。无侧20°观察只作为完全没有可强制归侧候选的异常兜底。

## 9. 2026-09-17 最终走廊扫障与边界交接

- 删除SEARCH、APPROACH以及抓取完成到600 mm预备点之间因`danger_ahead`触发`CHANGE_LANE`的分支。F407已拒绝旧CHANGE_LANE；爪外物资不能中断搜索、抓取和STAGE NAV。
- STAGE完成条件不要在最终时刻只比较当前8位ACK与阶段初始ACK。看到新鲜`mode=10 + DISTANCE_DONE + GRIPPER_CLOSED`即可锁存F407已完成预备点动作；匹配STAGE D=0经relay发送与ACK证据用于诊断，不能成为永久阻塞ALIGN的单点条件。随后持续发送第一次定位ALIGN。F407允许从STAGE NAV直接接受该ALIGN，并使用1.5°进入/4°保持迟滞完成mode11。
- 第二次视觉ALIGN完成后建立“最终推进走廊ROI”，只统计安全区多边形外、位于当前车头到对应安全半区入口之间的物体。首件正式绿色投送时，任意类别物体进入该走廊都触发扫障；首件完成后的普通、核心或伤员投送，仅`danger_cyan`或`injured_orange`触发。安全区内部物体必须排除。
- 走廊候选还必须排除当前正在运送的物资：把本轮`delivery_items`/carried track ID传入走廊函数并直接跳过；再用夹爪近场ROI过滤track重建后仍位于夹内的当前货物。否则首件绿色和伤员会把自己识别成障碍，连续触发两次扫障后进入PAUSE。
- 新增命令`CLEAR_SAFE_ZONE=0x13`，仍使用TYPE 0x18和原CRC：P1仅`CMD_VALID|RED_SIDE`；P2/P3为障碍进入夹爪所需的前进距离80～600 mm；P4/P5为有符号货物暂放横移，普通/核心发`+150`表示右侧，伤员发`-150`表示左侧；P6/P7=0。只有第二次视觉ALIGN已完成时发送，持续到ACK和mode39，不能在SEARCH/APPROACH/STAGE途中发送。
- P2/P3不能直接使用`relative_xy_m[1]*1000`；应减去相机/定位参考点到夹爪入口的实测机械偏移和希望物体进入爪内的余量，再裁剪到80～600 mm。该标定值应放在上位机配置中，不能散落成魔法数字。
- mode39期间停止发送NAV、ALIGN、ENTER、CHANGE_LANE和目标命令，持续发送同一语义的CLEAR新SEQ。F407会侧放原货物、回中抓障碍、把障碍放到相反侧、退回预备点、重新夹回原货物并回中。随后F407进入mode23；上位机清除旧审核和frame floor，按既有抓后规则发送3个不同audit_id。空爪回SEARCH，非法按既有释放/分离处理，合法后F407进入mode40。
- F407完成物理扫障后会保持mode39至少500 ms再进入mode23。上位机应把“CLEAR已ACK且看到mode23+GRIPPER_CLOSED+CLAW_VISIBLE”作为充分条件直接开始复审；`safe_sweep_execution_seen`只用于诊断，不能成为硬门槛，以免状态链路恢复时已经错过mode39而永久重发CLEAR。
- mode40表示“原货物已重新夹回并通过审核，底盘回到预备点中心”，不是ENTER许可。上位机必须重新执行第一次定位ALIGN和第二次视觉ALIGN，确认走廊后才发送ENTER。建议同一投送最多扫障2次；两次后走廊仍被危险物或伤员占据时保持停车并提示人工处理，不能无限搬运，也不能直接ENTER。
- F407 mode41表示本地30 cm场地边界保护已触发：当前动作已取消，夹爪打开、车头转向场地中心，并继续向场内行驶到距边至少40 cm后才进入mode3。上位机看到mode41应立即清除selected_batch、track、审核、NAV/ALIGN/ENTER和扫障上下文，只发送HOLD；看到新鲜mode3及动作后的新视觉帧后重新SEARCH。不得把mode41当作电机故障或继续重发旧命令。
- ENTER本地600 mm推进期间F407把边界阈值放宽到50 mm，其余主动动作使用300 mm。上位机自身边界判断应采用同样的阶段差异，不能在合法最终推进中提前ABORT，也不能在普通阶段覆盖mode41继续运动。
- mode23非法且无法分侧时，不要把`RELEASE_BOTH`当观察转向。无侧观察必须发送无SIDE_VALID的`DISPERSE_PILE`并等待mode35；真正最终双开才发送RELEASE_BOTH并等待mode34。
- 审核达到上位机3帧后若F407尚未置AUDIT_VALID，不要永久重复同一个audit_id；继续等待下一张新视觉帧并生成新audit_id，直到下位机也实际累计到3个不同ID。
