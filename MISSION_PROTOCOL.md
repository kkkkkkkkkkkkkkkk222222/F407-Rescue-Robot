# 连续物资抓取与分区投送流程

本固件已接入`danmo-teng/shijue_fangan`的`codex/gamepad-teleop@b827303`完整比赛流程命令，运行模式为`APP_ENABLE_TASK=1`，其他测试和独立视觉居中Task均关闭。RDK X5与F407使用USART3（PD8 TX、PD9 RX）、115200 8N1、3.3 V TTL和公共15字节帧。临时底盘改动及恢复方法见[`docs/TEMP_CHASSIS_HANDOFF.md`](docs/TEMP_CHASSIS_HANDOFF.md)。

## 整体流程

1. 上电后先保持TIM8舵机PWM关闭，完成IMU配置、静止稳定和陀螺仪零偏校准。只有`IMU_Init()`成功后才启动4路舵机PWM，随后摄像头到90°、舵机1到55°，左右爪依次收缩到左23°、右147°；IMU失败则机构和正式Task均不启动。
2. 收到合法`TYPE=0x11`红蓝方和出发区配置后回复一次`A3 B3 01 C3`并启动连续任务，不设置180秒自动终止。
3. 用编码器累计路程倒车1.70 m。前600 mm由`Motor_MoveDistance(-0.60 m, 800 mm/s)`直接锁存两路前向编码器起点，以3 mm容差做定距闭环并用IMU保持启动航向；舵机1保持55°、左右爪保持Retract，确保机构不碰障碍区。完成600 mm后舵机1才到85°、左右爪同时打开到左108°/右72°；剩余约1.10 m以最高900 mm/s使用`Motor_MoveSpin()`保持地面直线路径，同时根据IMU航向误差闭环转向约180°。总里程最后100 mm降到160 mm/s，进入10 mm容差后制动。
4. 到达中心附近后不执行Touch、单独转180°、停车等待或无条件开局打乱；确认并行开爪动作已经完成后立即进入`SEARCH`。聚集目标持续发送带`CLUSTER_TARGET`的APPROACH，F407保持双爪Open靠近，相机130°水平对正后到140°并进入`mode=38`夹内审核；稳定非空审核后进入`mode=37`。能分侧时发送带`SIDE_VALID`的曲线分离；不能分侧时发送无侧DISPERSE，F407只转12°并以mode35请求新审核。
5. 每次进入`SEARCH`都把舵机3从当前命令角移到90°，稳定300 ms后以200 mm/s原地旋转一整圈；随后切到120°、稳定300 ms再转一圈。两层共约720°完成后F407仍保持`SEARCH`并重新开始90°扫描，不自行启动本地返中心动作。上位机可在任意阶段通过普通或聚集标记的`APPROACH_TARGET`、空爪藏点NAV接管；SEARCH中的`HOLD`不停止本地扫描。F407收到第一条合法APPROACH即进入靠近，不在下位机重复增加确认帧数。
6. 收到1帧原生1280×1024合法单目标坐标后锁定类别并进入`APPROACH`。舵机3达到125°时停车并水平对正到误差≤24 px，随后移动到140°并稳定500 ms；不再要求原track重新出现，直接进入`mode=21`并置`CLAW_VISIBLE=1`。
7. `mode=21`保持锁存航向并以180 mm/s慢爬，最多500 mm。普通抓取要求两个不同`audit_id`且内容一致的非空审核：第一张新视觉帧可非STABLE，第二张置STABLE；同一视觉帧重复转发保持audit_id，不增加F407计数。第二帧到达后停车并允许`GRAB_CONFIRMED`。聚集mode38/mode37和分离复审仍允许1张显式STABLE新帧。
8. mode21累计慢爬500 mm仍未得到两张一致非空审核时，F407停车进入`APPROACH_RECOVER`，随后回到SEARCH重新选择目标；不会再执行140→90°抬头后重新低头继续推物块的循环。
9. 上位机看到新鲜`mode21 + CLAW_VISIBLE`后停止APPROACH、记录frame floor并发送上述两张审核；等待第二张审核ACK后才重复发送`GRAB_CONFIRMED`。F407第一次接受时让双爪同时合到Touch，后续帧只更新ACK而不重启舵机动作，2秒后持续置`GRIPPER_CLOSED=1`。
10. 抓取闭合后，上位机先用带`STAGE_ONLY(bit6)`的`NAVIGATE_WAYPOINT`引导到安全区预备点（上位机`5b6164f`当前配置为入口前600 mm）。F407继续使用H/D导航，但到`D=0`只停车、把摄像头抬到120°、置`DISTANCE_DONE=1`并保持`mode=10`，严禁启动旧安全区补推。藏堆和返中命令不使用该标志，原流程不变。
11. 上位机随后发送第一次`ALIGN_SAFE_ZONE`：P6/P7为红方90°或蓝方270°，F407用IMU原地对正，并确保摄像头120°命令后至少稳定300 ms，再在1.5°范围内上报`mode=11`。上位机连续3帧冻结安全区框后发送第二次带`VISUAL_CORRECTION_VALID(bit6)`的ALIGN；P2/P3为有符号水平像素误差而不是角度。F407按可标定焦距把像素误差转换为一次相对转角，最大限制±15°，完成后再次上报新鲜`mode=11`并锁存最终航向。视觉5秒失败时，上位机跳过第二次ALIGN，直接沿第一次90°/270°结果推进。
12. `ENTER_SAFE_ZONE`必须置`DRIVE_STRAIGHT`、清除`DISTANCE_VALID`且P2～P5为0；视觉ALIGN成功时bit6=1、清除`USE_FINAL_HEADING`且P6/P7=0，定位降级时bit6=0、置`USE_FINAL_HEADING`并发送红9000/蓝27000。F407按本地编码器执行既有接近和补推，停车、双爪全开且相机120°稳定300 ms后才上报mode15。
13. 收到`TASK_COMPLETE`进入mode16，以400 mm/s后退0.30 m。上位机看到mode16或17后持续发送RETURN H/D；mode16可校验、缓存并ACK RETURN以解除PAUSE，但必须继续完成剩余本地后退，不提前切换或重置距离。随后mode17直接使用最新RETURN返中，D=0后进入mode3 SEARCH。

上位机在收到STM32的`TYPE=0x17`退出、返中或搜索状态后进入下一轮；第一次投送完成前只选择单个绿色物资，之后允许同类普通/核心批次或单个伤员，危险目标不进入正式投送。

NormalRun不再用单轮编码器方向、单轮零速或定距短时无进展自动进入`TASK_FAULT_MOTOR`。收到ABORT进入`mode18/fault1 REMOTE_STOP`后会重新开放赛前配置；操作员发送一组新的连续合法TYPE=0x11配置即可只清除REMOTE_STOP，并重新执行安全收爪和自主出发。MOTOR、POSE_TIMEOUT、INVALID_STATE、IMU及其他硬件故障不能由配置清除。

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

### `TYPE=0x12`视觉报告

`P0/P1=X`、`P2/P3=Y`、`P4/P5=距离mm`，均为大端；`P6`为四类目标2位数量字段；`P7 bit0=FOUND、bit1=NEAR、bit3=CLASS_VALID、bit6=DISTANCE_VALID`。当前上位机直接发送0～1279和0～1023的原始坐标，中心为`(640,512)`。没有目标时P0～P7全0；没有可靠距离时距离为0且bit6清零。

已知中心帧：

```text
A3 B3 12 10 02 80 02 00 00 00 01 09 DD FD C3
```

### `TYPE=0x17`STM32状态

| 字段 | 内容 |
| --- | --- |
| `P0` | bit0爪子入镜、bit1夹爪闭合、bit2电机运动、bit3自动靠近、bit5导航定距完成、bit7故障 |
| `P1` | 当前`TaskState`编号 |
| `P2/P3` | 摄像头角度，0.01°，大端 |
| `P4` | 最近真正接受的`TYPE=0x18`命令SEQ |
| `P5` | 故障码，0为正常 |
| `P6/P7` | 固定为0 |

当前故障码：`0 NONE、1 REMOTE_STOP、2 MATCH_TIMEOUT、3 MOTOR、4 START_TIMEOUT、5 POSE_TIMEOUT、6 COMMAND_TIMEOUT、7 RAM、8 INVALID_STATE、9 TARGET_LOST`。

### `TYPE=0x18`任务命令

`P0=COMMAND、P1=FLAGS、P2..P7=命令相关载荷`。FLAGS bit0必须为1，bit1要求直行，bit2使用绝对航向，bit3表示红方，bit4表示距离有效。bit5仅在`APPROACH_TARGET`中表示`CLUSTER_TARGET`。bit6按opcode复用：NAV为`STAGE_ONLY`，ALIGN/ENTER为`VISUAL_CORRECTION_VALID`，DISPERSE为`SIDE_VALID`；bit7仅在DISPERSE中表示`TARGET_RIGHT`。其他命令携带这些高位会被整帧拒绝。

命令值：`0 STOP、1 PAUSE、2 GRAB_CONFIRMED、3 NAVIGATE_WAYPOINT、4 ALIGN_SAFE_ZONE、5 ENTER_SAFE_ZONE、6 TASK_COMPLETE、7 ABORT、8 RETURN_CENTER、9 APPROACH_TARGET、10 HOLD、11 YIELD_BACKOFF、12 ESCAPE_MANEUVER、13 RELEASE_LEFT、14 RELEASE_RIGHT、15 RELEASE_BOTH、16 DISPERSE_PILE、17 CHANGE_LANE、18 CARGO_AUDIT`。


- `GRAB_CONFIRMED`会重复发送直到新鲜`TYPE=0x17`置`GRIPPER_CLOSED=1`；F407对舵机动作幂等，但每个新SEQ都必须更新P4 ACK。
- `HOLD`是高层“本周期没有新动作”的心跳：在`SEARCH`中F407仍执行本地90°/120°扫描；在APPROACH正常跟踪、125°水平对正、NAV、RETURN和远程动作中安全停车。相机由125°移到140°并稳定属于已经触发的本地近距序列。需要冻结时使用`PAUSE=1`；mode16中的合法RETURN可以解除PAUSE，但只恢复剩余0.30 m本地后退。
- `STOP`只在NAV中保留原有可恢复兼容行为，其他状态会进入远程停止故障；`ABORT`同样进入REMOTE_STOP。普通等待不得使用STOP/ABORT；只有操作员新一轮合法赛前配置可以清除REMOTE_STOP，其他故障保持锁存。
- 正式投送的`STAGE_ONLY NAV`兼容两种flags：精简形式`VALID|DISTANCE_VALID|STAGE_ONLY|阵营位`，以及在此基础上同时增加`DRIVE_STRAIGHT|USE_FINAL_HEADING`的完整形式；两方向位只出现一个属于非法帧。P2/P3仍为剩余距离/0，P6/P7仍为0～35999的场地平移方向。预备点D=0后置`DISTANCE_DONE`并保持mode10，不执行旧D=0补推；旧非STAGE NAV、藏堆NAV和RETURN仍强制要求完整方向flags，原语义不变。
- 合法新流程为`WAIT_NAVIGATION→STAGE NAV(mode10)→定位ALIGN(mode11)→可选视觉ALIGN(新鲜mode11)→ENTER→CHECK(mode15)→TASK_COMPLETE→EXIT_SAFE_ZONE→RETURN_CENTER`。
- 第一次ALIGN的P6/P7是90°/270°绝对航向；第二次ALIGN的P2/P3是冻结框相对640 px中心的有符号像素误差。视觉修正只能应用一次，重复帧只更新ACK，不能重复累加角度。
- ENTER必须置`DRIVE_STRAIGHT`并清除`DISTANCE_VALID`，P2～P5为0；视觉对正成功时bit6置位、bit2清零且P6/P7=0，定位降级时bit6清零、bit2置位并发送红9000/蓝27000。推进距离完全由F407编码器执行。
- 红方前置点为`(0,+950 mm)`，蓝方前置点为`(0,-950 mm)`。F407不接收PWM值，只接收任务目标并在本地完成转向、速度限制和失联停车。

### 完整比赛流程扩展

- `APPROACH_TARGET`的P2/P3和P4/P5分别为原生图像X/Y；完整流程激活后它是靠近控制的唯一坐标源。F407不再按250 ms帧龄自动减速或退出，HOLD/PAUSE/STOP以及阶段切换必须由上位机明确发送。
- 普通目标在140°稳定500 ms后直接进入mode21、置`CLAW_VISIBLE=1`并以180 mm/s最多慢爬500 mm，不再等待原track重新出现。两个不同audit_id且内容一致的非空普通审核后停车；第一帧可非STABLE，第二帧显式STABLE。聚集和分离复审仍允许单个显式STABLE新帧。
- `CARGO_AUDIT`按字节编码左右类别、数量、审核标志、audit_id和总数量；相同audit_id的重复传输不增加普通抓取计数。`UNKNOWN_PRESENT=1`时允许`total_count > left_count + right_count`，表示物体在大ROI内但无法分侧；F407接收并锁存该审核，但禁止直接GRAB，只允许无SIDE_VALID的DISPERSE执行12°观察。带SIDE_VALID的DISPERSE仍要求保留侧count>0。`AUDIT_DESTINATION_INJURY`继续作为本地伤员合法性复核，不自行改变上位机确认的目的地。
- 非法组合优先单侧分离。普通Touch为左78°/右102°；聚集物第一次带侧曲线使用15°柔性保持（保留左63°或保留右117°），mode35复审仍不合法时改用25°强保持（保留左53°或保留右127°），另一侧完全打开。`RELEASE_LEFT/RIGHT`和带`SIDE_VALID`的DISPERSE都要求相关侧count非零。曲线完成后相机140°重新审核；稳定空爪则双开回SEARCH，无法分侧则用无侧DISPERSE触发12°观察。
- 临时藏堆NAV不使用`STAGE_ONLY`，也不执行任何安全区补推；到点后`RELEASE_BOTH`。释放完成后至返中完成期间，F407拒绝`APPROACH_TARGET`和`DISPERSE_PILE`，避免残留找物命令抢占藏堆回程。第一条`RETURN_CENTER`先触发F407保持释放航向直退0.35 m，编码器累计距离且IMU修正航向；退到安全距离后才使用上位机持续更新的H原地调头并按D返中。该退让只对临时藏堆生效，正式安全区投送仍使用既有mode16后退0.30 m，不会重复后退。
- `YIELD_BACKOFF`只在F407已确认完成单侧释放且`cargo_recheck_pending=1`时接受。聚集APPROACH保持双爪Open，mode38审核后进入mode37；第一次带侧曲线使用15°保持，mode35复审仍不合法时后续曲线使用25°保持。不带SIDE_VALID只执行12°观察并以mode35请求新审核。
- 对外mode映射为`3 SEARCH、15 DELIVERY_VERIFY、16 EXIT_SAFE_ZONE、17 FACE_FIELD_CENTER、18 STOPPED、20 APPROACH_TARGET、21 CAPTURE_AUDIT、22 CAPTURE_DONE、30 YIELD_DONE、31 ESCAPE_DONE、32/33/34 RELEASE_DONE、35 DISPERSE_DONE、36 LANE_DONE、37 DISPERSE_READY、38 CLUSTER_CAPTURE_AUDIT`。
- TASK_COMPLETE后的顺序为mode16本地后退、mode17执行RETURN H/D、D=0后mode3。mode16可缓存并ACK合法RETURN以解除PAUSE，但不会提前返中或重置后退进度。mode3后的1500 ms交接窗口仍会确认重复RETURN帧SEQ。
