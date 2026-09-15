# 上位机救援流程对接要求

对照基线：`danmo-teng/shijue_fangan`分支`codex/gamepad-teleop`提交`775bef3`。
下位机已实现400 mm预备点、两次ALIGN、锁存航向ENTER和200 mm本地编码器补推；安全区退出与返中流程保持原样。

## 1. SEARCH职责

- F407自主完成摄像头90°一圈、120°一圈并循环扫描；上位机不需要发送底盘旋转命令。
- `6c802ec`一旦锁定候选即可持续发送`APPROACH_TARGET`；F407收到第一帧合法命令便进入APPROACH，不再增加第二套确认门限。
- 上位机在视觉过期、暂未选中目标或等待策略决策时可以发送HOLD，但SEARCH中的HOLD只是任务心跳，F407仍继续本地扫描。若确实要求底盘冻结当前SEARCH阶段，发送新增的`PAUSE=1`，不能再复用HOLD表达两种相反行为。
- 上位机可在SEARCH的任意扫描阶段发送普通/聚集APPROACH或空爪藏点NAV接管，不必等待F407额外报告“720°完成”；聚集目标必须先靠近到mode38完成夹内审核，再由F407进入mode37，不能从远处直接发送DISPERSE。
- 普通APPROACH进入`mode=21 + CLAW_VISIBLE=1`后，F407会以150 mm/s继续慢爬，而不是原地等待。上位机应立即切入CAPTURE_AUDIT，即使普通目标已经因过近而离开全局检测结果；暂时空爪持续发送非STABLE全零审核，稳定非空审核到达后F407立即停车，审核ACK后再发送GRAB。若F407在累计300 mm内仍未确认物体，会进入mode24/REACQ并最终mode3；CAPTURE_AUDIT/AUDIT_CONFIRM必须识别这两个mode，清除待审核数据并回到搜索恢复，不能继续发送审核帧。

### HOLD、PAUSE、STOP、ABORT语义

- `HOLD=10`：正常流程心跳。SEARCH继续扫描；APPROACH常规跟踪/125°对正、NAV、RETURN和远程动作安全停车。F407进入125→140°、140°最多300 mm限距重获或140→90°慢抬重获后，HOLD表示“本帧未见目标”但不会打断已触发的本地序列；需要真正冻结必须发送PAUSE。
- `PAUSE=1`：操作员暂停、定位短时不可用但希望保留当前阶段、或上位机内部重建状态时使用。F407 ACK后锁存停车并保持Task状态；帧过期也不会自行恢复，且不会触发DISPERSE的HOLD取消。
- 解除PAUSE必须发送一条当前状态可接受、SEQ递增的非PAUSE命令：SEARCH发HOLD，APPROACH发APPROACH_TARGET，NAV发NAVIGATE_WAYPOINT，RETURN发RETURN_CENTER，远程动作重发原动作命令。无效或阶段不匹配的命令不会解除暂停。
- `STOP=0`不是普通暂停；除NAV兼容入口外会形成远程停止故障。`ABORT=7`始终用于不可自动恢复的终止。

## 2. 初始藏堆

- 项目最终规则覆盖`6c802ec`原有限制：`initial_stash=1`没有物资种类和数量限制，只负责把开局物资堆搬离中心。
- 稳定审核只需满足`total_count>0`。左右2-bit计数字段可能饱和，不要求`left_count + right_count == total_count`，危险、未知或混合类别也不作为本次临时搬堆的拒绝条件。
- 上位机实现上应删除`_pile_batch()`对`max_batch_count`的截断，并在`_audit_valid()`中把`selected_batch.initial_stash`的`total_count>0`判断放到正式投送的数量、类别和计数一致性判断之前。编码时左右计数仍饱和到0～3，P7保留实际总数；F407在initial_stash中不会用饱和计数拒绝动作。
- 这一放宽只适用于临时藏堆；从藏点取回并准备正式投送时必须重新执行完整审核，不能沿用暂存审核结果。
- 藏点`RELEASE_BOTH`完成后，上位机应立即进入`RETURN_CENTER`并持续发送最新H/D。F407会屏蔽残留的`APPROACH_TARGET`和`DISPERSE_PILE`，先保持释放时的车头方向直退0.35 m，再使用最新H调头返中；这段期间上位机不得因mode17尚未开始转向而改发HOLD、重发RELEASE或判定卡住。正常投送的mode16退出流程不受影响。

## 3. 普通与核心混装按上位机批次策略直接运输

为与最新版上位机保持一致，首件绿色物资正式投送完成后，F407允许总数1～3件的普通、核心或`mixed_material`组合直接进入GRAB/NAV。左右分别为`green_supply`和`core_black`同样允许直接抓取，不再进入`INVALID_RELEASE`。危险、未知、伤员混装及超过3件仍必须进入释放或恢复流程。

- 上位机能够可靠确认非法物资所在侧时，发送`RELEASE_LEFT/RIGHT`，随后持续发送`YIELD_BACKOFF(-300 mm)`直到新鲜mode30。释放侧完全打开，保留侧只维持普通Touch；F407沿镜像曲线退出后到140°重新审核。稳定空爪仍发送带`STABLE`的全零`CARGO_AUDIT`，暂未识别到时的全零占位必须保持非STABLE。
- 上位机判定审核非法但无法可靠指定释放侧时，发送`RELEASE_BOTH`。F407会自行双开留物、以500 mm/s后退0.40 m、Touch闭爪并等待机构到位、以1000 mm/s恒速前撞0.60 m、再以500 mm/s后退0.60 m回到撞击起点，最后重新双开并报告mode34。约0.40 m处接触物块后还会继续推散0.20 m。该分支已将整批物资留在场上，不能进入夹内复审；确认新鲜mode34及ACK后进入`DISPERSE_RESELECT`，先在原区域重选原目标，重选失败时才清空批次并HOLD回SEARCH。
- 单侧释放完成后发送`YIELD_BACKOFF(-300 mm)`；该距离现在是曲线路径长度，不是直线倒车距离。等待新鲜`mode=30`后重新进入`CAPTURE_AUDIT`，不得直接SEARCH。
- 复审合法时把`selected_batch`替换为实际保留的单件，再发送`GRAB_CONFIRMED`直到F407上报`GRIPPER_CLOSED=1`。被释放物资不计入本次投送，留待后续重新搜索。
- 上述复审只适用于`RELEASE_LEFT/RIGHT`单侧保留分支：合法时持续发送GRAB，仍非法时按策略最终释放。左右不明的撞分分支不再复审。

F407会校验释放侧计数：`RELEASE_LEFT/RIGHT`对应侧必须非空；复审失败阶段只接受`RELEASE_BOTH`。命令与最近稳定审核不一致时不会ACK或动作。

## 4. 去放置区与投送确认

- 正式投送NAV必须置`STAGE_ONLY(bit6)`，目标是相应半区安全区入口前400 mm预备点；持续发送几何H/D直到新鲜`mode=10 + DISTANCE_DONE + ACK变化`。不要在该点发送旧式ENTER，F407明确禁止预备点补推。
- 第一次ALIGN置`USE_FINAL_HEADING`，P6/P7始终发送红方9000或蓝方27000；所有投送使用相同航向，不再区分首趟偏置。持续发送到新鲜`mode=11 + ACK变化`。
- 连续3个不同新视觉帧冻结安全区框；第二次ALIGN置`VISUAL_CORRECTION_VALID(bit6)`，P2/P3发送`target_x_px-640`的有符号像素误差，P4/P5/P6/P7为0。持续发送到第二个新鲜`mode=11 + 本阶段ACK变化`。5秒仍无法冻结时跳过第二次ALIGN，沿第一次航向进入回退ENTER。
- ENTER始终置`DRIVE_STRAIGHT | DISTANCE_VALID`，P2/P3持续发送当前位置到围栏直线的法向剩余距离，P4/P5为0。视觉修正成功时置bit6且P6/P7=0；回退路径置`USE_FINAL_HEADING`且P6/P7=9000/27000。F407不会再按ENTER的动态H转向。
- F407在D约113 mm时进入本地最终补推并忽略后续D，以300 mm/s编码器推进200 mm，最长1200 ms；张爪和相机120°稳定完成后才上报mode15。上位机等待新鲜`mode=15 + ACK变化`后进入原投送视觉确认。
- `6c802ec`第二次视觉确认超时后会永久停在ENTER。上位机应增加有限兜底：F407已新鲜处于`mode=15`、观察窗口结束且没有明确“目标仍在安全区外”的证据时，发送并持续保持`TASK_COMPLETE`；若明确仍在区外则保持停车并报告人工处理，不得伪造完成。
- 第二次视觉ALIGN只叠加一次冻结框修正；ENTER无视觉回退时仍发送9000/27000，F407继续使用第一次ALIGN锁存的90°/270°目标。

## 5. 投送后返中（F407现有安全区退出不修改）

1. 上位机在`RAM_VERIFY(mode=15)`确认投送后持续发送`TASK_COMPLETE`。
2. F407接受后进入`EXIT_SAFE_ZONE(mode=16)`，以400 mm/s后退0.30 m；该动作由本地编码器定距和IMU保持航向。
3. 上位机看到新鲜mode16或mode17后进入`RETURN_CENTER`，根据最新融合位姿持续发送：`H=当前点到中心的场地航向`、`D=到中心的剩余距离`。
4. F407退出完成后进入`FACE_FIELD_CENTER(mode=17)`，先原地转到H误差3°内，再最高800 mm/s向前行驶；途中偏差达到8°才停车重对。
5. 上位机进入中心容差后必须继续发送`RETURN_CENTER D=0`，直到看到F407上报`mode=3 SEARCH`。不能用HOLD代替返中完成。

## 6. 退让与脱困

- `YIELD_BACKOFF`只能在上位机已经看到对应释放命令ACK变化和新鲜`mode=32/33`后发送，并持续发送到新鲜`mode=30`；F407现在拒绝APPROACH、NAV、RETURN以及其他动作后直接到来的YIELD。第一次不明左右的特殊`RELEASE_BOTH`已在F407内部完成退让和撞分，是明确例外，不得追加YIELD。普通脱困改用`ESCAPE_MANEUVER`，不能复用YIELD。
- `ESCAPE_MANEUVER`持续发送到新鲜`mode=31`。F407会先把大舵机恢复85°行驶位置，再执行旋转和横移。
- 远程动作暂时丢帧、但仍允许F407执行既有恢复策略时可发HOLD；若要求动作原地冻结且恢复后从当前阶段继续，应发PAUSE。两者都不能把未完成动作直接标记完成，恢复时继续重复原命令并递增SEQ。
- 聚集目标不能在SEARCH中直接发送`DISPERSE_PILE`。先用`APPROACH_TARGET.flags bit5=CLUSTER_TARGET`持续发送聚集中心X/Y；F407闭合到Touch并以最高300 mm/s靠近，相机到130°且X误差≤40 px后先转到140°并稳定500 ms，再报告`mode=38`、以200 mm/s保持航向慢爬。上位机看到新鲜mode38后立即切换夹爪ROI：暂未确认时持续发送非STABLE全零`CARGO_AUDIT`，确认夹内非空时发送带STABLE的非空审核。F407收到后停车并进入mode37。审核合法且无需分离时持续发送`GRAB_CONFIRMED`；需要分离且左右可靠时发送带`SIDE_VALID(bit6)`的`DISPERSE_PILE`，保留右侧时再置`TARGET_RIGHT(bit7)`；左右不明则发送仅带`CMD_VALID`的`DISPERSE_PILE`触发整堆撞分。F407在300 mm或2000 ms内未收到稳定非空审核会张爪回SEARCH，上位机必须清除该轮审核，不能继续发送旧GRAB/DISPERSE。

## ACTION/WATCH卡死的必须修复项

- STM32状态`mode=25/ACTION`而LCD显示`CMD:APP REJ`，表示APP帧格式有效但当前阶段拒绝，不是串口断线。上位机必须比较状态帧`acknowledged_sequence`，不能仅凭发送成功或LCD曾显示APP判定已接管。
- 左右不明的整堆撞分完成并收到新鲜mode34后，必须进入`DISPERSE_RESELECT`而不是`CAPTURE_AUDIT`：清空夹内审核缓存，但保留原目标类别/track线索；此时可以直接发送新的APPROACH，不能发送YIELD或CARGO_AUDIT。
- 删除该分支的`cargo_recheck_pending=True`、`audit_recheck_frame_floor`和无限`CAPTURE_AUDIT`跳转。只有重选窗口结束且确实没有候选时才发送HOLD，让F407从mode34进入mode3。此前WATCH永久等待的直接原因是撞分后车已退回、爪内为空，上位机却继续等待夹内审核。

## 7. 必测回归

1. SEARCH连续HOLD时F407仍扫描；第一帧合法APPROACH只触发一次靠近。
2. 90°/120°扫描任一阶段，上位机都能用普通/聚集APPROACH或藏点NAV接管；聚集目标必须先到mode38完成稳定非空夹内审核，DISPERSE只能在随后mode37发送。
3. initial_stash总数大于0时，即使数量超过3、左右计数饱和或类别混合也能完成暂存；总数为0时不得GRAB。
4. 左绿右核心、左核心右绿分别释放正确一侧，YIELD后重新审核。
5. 左右归属不明时发送无bit6/bit7的DISPERSE触发本地撞分；mode34后先重选原目标并可直接APPROACH，重选失败才HOLD回SEARCH，不发送YIELD或CARGO_AUDIT。
6. 释放命令与审核侧不一致时F407不动作，上位机能够重新审核恢复。
7. 正式投送NAV携带STAGE_ONLY，在400 mm预备点D=0只得到mode10+DISTANCE_DONE，不能出现张爪或本地长距离补推。
8. 每次投送第一次ALIGN均完成红90°/蓝270°定位对正，第二次只应用一次冻结框像素修正，不再添加首趟±10°。
9. ENTER期间改变上位机动态H不影响锁存航向；D到约113 mm后改变或回跳也不打断300 mm/s、200 mm本地补推，最终才出现mode15。
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
- 聚集目标进入mode38后先完成稳定非空夹内审核；mode37中审核合法且不需分离时直接GRAB，需要分离且左右可靠时使用`DISPERSE_PILE + SIDE_VALID`并在mode35后复审，左右不可靠时使用不带bit6/bit7的DISPERSE，mode34后进入`DISPERSE_RESELECT`。重选期间不要发HOLD，否则F407会立即离开原地进入普通SEARCH；目标重现可从mode34直接发普通或聚集APPROACH，整堆撞分最多重试2次。
