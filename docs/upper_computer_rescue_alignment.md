# 上位机救援流程对接要求

对照基线：`danmo-teng/shijue_fangan`提交`6c802ec`。
下位机约束：安全区退出流程，以及放置区NAV的D≤50 mm本地航向锁存、D=0末端慢推由F407执行。

## 1. SEARCH职责

- F407自主完成摄像头90°一圈、120°一圈并循环扫描；上位机不需要发送底盘旋转命令。
- `6c802ec`一旦锁定候选即可持续发送`APPROACH_TARGET`；F407收到第一帧合法命令便进入APPROACH，不再增加第二套确认门限。
- 上位机在视觉过期、暂未选中目标或等待策略决策时可以发送HOLD，但SEARCH中的HOLD只是任务心跳，F407仍继续本地扫描。若确实要求底盘冻结当前SEARCH阶段，发送新增的`PAUSE=1`，不能再复用HOLD表达两种相反行为。
- 上位机可在SEARCH的任意扫描阶段发送普通/聚集APPROACH或空爪藏点NAV接管，不必等待F407额外报告“720°完成”；聚集目标必须先靠近到mode37，不能从远处直接发送DISPERSE。

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

## 3. 普通与核心混装必须先拆分

当前机械结构不能可靠地把普通与核心混装批次一次卸出。请在`_audit_valid()`正式物资分支之前增加“普通+核心同时存在”的判断，不要直接进入GRAB/NAV。

- 左右侧类别分别为`green_supply`和`core_black`：进入`INVALID_RELEASE`。首件绿色尚未完成时保留绿色侧并释放核心侧；之后优先保留数量为1且类别明确的一侧。
- 无法确定普通/核心分别位于哪侧：发送`RELEASE_BOTH`。F407会自行双开留物、后退0.40 m、Touch闭爪、以700 mm/s前撞0.40 m、再以450 mm/s后退0.40 m回到撞击起点，最后重新双开并报告mode34。该分支已将整批物资留在场上，不能再进入夹内复审；确认新鲜mode34及ACK后清空`selected_batch`和相关审核状态，转SEARCH并持续发送HOLD直到F407上报mode3。
- 单侧释放完成后发送`YIELD_BACKOFF(-250 mm)`；等待新鲜`mode=30`后重新进入`CAPTURE_AUDIT`，不得直接SEARCH。
- 复审合法时把`selected_batch`替换为实际保留的单件，再发送`GRAB_CONFIRMED`直到F407上报`GRIPPER_CLOSED=1`。被释放物资不计入本次投送，留待后续重新搜索。
- 上述复审只适用于`RELEASE_LEFT/RIGHT`单侧保留分支：合法时持续发送GRAB，仍非法时按策略最终释放。左右不明的撞分分支不再复审。

F407会校验释放侧计数：`RELEASE_LEFT/RIGHT`对应侧必须非空；复审失败阶段只接受`RELEASE_BOTH`。命令与最近稳定审核不一致时不会ACK或动作。

## 4. 去放置区与投送确认

- D>50 mm时，上位机持续发送当前位置到目标点的真实H和D，并在进入50 mm区域前完成主要横向对位。
- 第一次进入D≤50 mm后，F407立即停车并根据赛前红蓝方锁存最终航向：首件红方80°、蓝方280°，后续红方90°、蓝方270°。D重新大于100 mm才解除锁存。
- 锁存后F407忽略上位机后续H，仅使用IMU保持固定车头方向；上位机必须继续发送新鲜D和正确红蓝方标志，不要尝试用末段H覆盖本地锁存。
- 上位机可按`6c802ec`在视觉已明确进入、到达地图容差或收到新鲜`DISTANCE_DONE`时发送ENTER；F407即使提前收到ENTER，也会先完成固定航向对正和200 mm/s、1.5～2.0 s的末端补推，再张爪进入`mode=15`。
- `6c802ec`第二次视觉确认超时后会永久停在ENTER。上位机应增加有限兜底：F407已新鲜处于`mode=15`、观察窗口结束且没有明确“目标仍在安全区外”的证据时，发送并持续保持`TASK_COMPLETE`；若明确仍在区外则保持停车并报告人工处理，不得伪造完成。

## 5. 投送后返中（F407现有安全区退出不修改）

1. 上位机在`RAM_VERIFY(mode=15)`确认投送后持续发送`TASK_COMPLETE`。
2. F407接受后进入`EXIT_SAFE_ZONE(mode=16)`，以400 mm/s后退0.30 m；该动作由本地编码器定距和IMU保持航向。
3. 上位机看到新鲜mode16或mode17后进入`RETURN_CENTER`，根据最新融合位姿持续发送：`H=当前点到中心的场地航向`、`D=到中心的剩余距离`。
4. F407退出完成后进入`FACE_FIELD_CENTER(mode=17)`，先原地转到H误差3°内，再最高800 mm/s向前行驶；途中偏差达到8°才停车重对。
5. 上位机进入中心容差后必须继续发送`RETURN_CENTER D=0`，直到看到F407上报`mode=3 SEARCH`。不能用HOLD代替返中完成。

## 6. 退让与脱困

- 凡是实际进入`YIELD_BACKOFF`的分支都必须持续发送到新鲜`mode=30`。第一次不明左右的特殊`RELEASE_BOTH`已在F407内部完成退让和撞分，是明确例外，不得追加YIELD。
- `ESCAPE_MANEUVER`持续发送到新鲜`mode=31`。F407会先把大舵机恢复85°行驶位置，再执行旋转和横移。
- 远程动作暂时丢帧、但仍允许F407执行既有恢复策略时可发HOLD；若要求动作原地冻结且恢复后从当前阶段继续，应发PAUSE。两者都不能把未完成动作直接标记完成，恢复时继续重复原命令并递增SEQ。
- 聚集目标不能在SEARCH中直接发送`DISPERSE_PILE`。先用`APPROACH_TARGET.flags bit5=CLUSTER_TARGET`持续发送聚集中心X/Y；F407以最高300 mm/s靠近，到相机130°并水平对正后报告mode37。此后持续发送DISPERSE并等待mode35：F407先把双爪闭到Touch，以接受第一条DISPERSE命令时的IMU航向为0°，用400 mm/s分别闭环到`+45/-45/+90/-90°`，不回正，最后以550 mm/s后退0.30 m。打散Touch不会置`GRIPPER_CLOSED`，上位机不得因此等待抓取闭合标志；建议超过18秒仍未收到mode35时停车告警。

## ACTION/WATCH卡死的必须修复项

- STM32状态`mode=25/ACTION`而LCD显示`CMD:APP REJ`，表示APP帧格式有效但当前阶段拒绝，不是串口断线。上位机必须比较状态帧`acknowledged_sequence`，不能仅凭发送成功或LCD曾显示APP判定已接管。
- 左右不明的`RELEASE_BOTH`完成并收到新鲜mode34后，必须把它当作`separate_then_search`：清空批次并发送HOLD，不能发YIELD、CARGO_AUDIT或APP。F407收到HOLD后从mode34进入mode3，之后才允许重新选择目标。
- 删除该分支的`cargo_recheck_pending=True`、`audit_recheck_frame_floor`和`CAPTURE_AUDIT`跳转。此前WATCH永久等待的直接原因是撞分后车已退回0.40 m、爪内为空，上位机`capture_audit is None`时又会无限清零`audit_hits`并保持审核。

## 7. 必测回归

1. SEARCH连续HOLD时F407仍扫描；第一帧合法APPROACH只触发一次靠近。
2. 90°/120°扫描任一阶段，上位机都能用普通/聚集APPROACH或藏点NAV接管；DISPERSE必须等聚集APPROACH到mode37。
3. initial_stash总数大于0时，即使数量超过3、左右计数饱和或类别混合也能完成暂存；总数为0时不得GRAB。
4. 左绿右核心、左核心右绿分别释放正确一侧，YIELD后重新审核。
5. 左右归属不明时双开触发本地撞分，mode34后清空批次并HOLD回SEARCH，不发送YIELD或CARGO_AUDIT。
6. 释放命令与审核侧不一致时F407不动作，上位机能够重新审核恢复。
7. 首件红/蓝方向分别锁存80°/280°，后续锁存90°/270°；D≤50 mm后改变上位机H不能改变锁存值，D>100 mm才解锁。
8. mode15视觉确认两次超时后不会无限ENTER：无区外反证时能TASK_COMPLETE，有明确区外反证时安全停车告警。
9. TASK_COMPLETE后依次看到mode16、mode17、mode3；mode17阶段H变化时重新计算朝向，D=0才结束。
10. RETURN中途HOLD只停车，恢复RETURN后继续；HOLD不能伪造回中完成。
11. 临时藏堆释放后发送RETURN，确认F407先直退0.35 m且车头不旋转，随后才按最新H调头；正常投送返中不得多退一次。
12. 聚集APPROACH到相机130°且X误差≤40 px才出现mode37；随后DISPERSE完成四段原地摆头和0.30 m后退才出现mode35，上位机不得定时伪完成。

## 8. 上位机下一步优化建议

- NAV和RETURN必须以固定频率持续发送新鲜H/D，SEQ逐帧递增；不要因为整数D连续几帧相同就改发HOLD。F407现在会联合D、IMU航向和编码器里程判断短时进展，但上位机仍应以T265/融合位姿作为几何到达的主依据。
- 卡住检测应区分“真实位姿进展”和“车轮空转”：T265平移达到阈值可直接判定有进展；只有编码器增加而T265长期不动应判为疑似打滑，不能一直重置卡住计时。原地对正阶段则单独使用航向变化判断。
- 不要让SEARCH计时、目标确认计时和F407本地90°/120°扫描圈数共同决定同一次状态跳转。上位机负责目标选择和策略，F407负责扫描动作；上位机确认候选后持续发送APPROACH。SEARCH无目标时发送HOLD让扫描继续，只有明确要求冻结时才发送PAUSE。
- `DISTANCE_DONE`只在STM状态帧不超过250 ms、`mode=10 NAV`且`GRIPPER_CLOSED=1`时可用于正式投送到达；RETURN和空爪藏点仍以D=0完成。过期状态、错误mode或未闭爪状态不得触发ENTER。
- 投送复核必须有有限结论：新鲜`mode=15`且无明确区外残留证据时，观察超时后发送TASK_COMPLETE；若明确物资仍在区外则停车告警。不能永久重复ENTER或永久等待视觉重新出现。
- RETURN_CENTER持续发送到F407上报`mode=3 SEARCH`，中心附近也要发送D=0，不能用HOLD或地图圆周容差提前结束。
- initial_stash只要求稳定且`total_count>0`；正式投送继续执行危险/未知/伤员混装、总数和普通+核心混装检查。单侧分离后必须YIELD、重新审核，合法后再GRAB，不能直接回SEARCH。
