# 连续物资抓取与分区投送流程

本固件已接入`danmo-teng/shijue_fangan`的`codex/gamepad-teleop@775bef3`完整比赛流程命令，运行模式为`APP_ENABLE_TASK=1`，其他测试和独立视觉居中Task均关闭。RDK X5与F407使用USART3（PD8 TX、PD9 RX）、115200 8N1、3.3 V TTL和公共15字节帧。临时底盘改动及恢复方法见[`docs/TEMP_CHASSIS_HANDOFF.md`](docs/TEMP_CHASSIS_HANDOFF.md)。

## 整体流程

1. 上电后先保持TIM8舵机PWM关闭，完成IMU配置、静止稳定和陀螺仪零偏校准。只有`IMU_Init()`成功后才启动4路舵机PWM，随后摄像头到90°、舵机1到55°，左右爪依次收缩到左23°、右147°；IMU失败则机构和正式Task均不启动。
2. 收到合法`TYPE=0x11`红蓝方和出发区配置后回复一次`A3 B3 01 C3`并启动连续任务，不设置180秒自动终止。
3. 用编码器累计路程倒车1.70 m。前600 mm由`Motor_MoveDistance(-0.60 m, 800 mm/s)`直接锁存两路前向编码器起点，以3 mm容差做定距闭环并用IMU保持启动航向；舵机1保持55°、左右爪保持Retract，确保机构不碰障碍区。完成600 mm后舵机1才到85°、左右爪同时打开到左108°/右72°；剩余约1.10 m以最高900 mm/s使用`Motor_MoveSpin()`保持地面直线路径，同时根据IMU航向误差闭环转向约180°。总里程最后100 mm降到160 mm/s，进入10 mm容差后制动。
4. 到达中心附近后不执行Touch、单独转180°、停车等待或无条件开局打乱；确认并行开爪动作已经完成后立即进入`SEARCH`。上位机确认聚集目标后不能在远处直接发`DISPERSE_PILE`，而应持续发送带`CLUSTER_TARGET`的APPROACH，使F407以最高300 mm/s靠近到相机130°并水平对正；看到新鲜`mode=37`后，能可靠判断目标左右侧便发送带`SIDE_VALID`的选择性曲线分离。无法判断时发送不带bit6/bit7的DISPERSE，F407只原地转12°改善左右视角并以mode35请求新审核，不再正撞整堆。
5. 每次进入`SEARCH`都把舵机3从当前命令角移到90°，稳定300 ms后以200 mm/s原地旋转一整圈；随后切到120°、稳定300 ms再转一圈。两层共约720°完成后F407仍保持`SEARCH`并重新开始90°扫描，不自行启动本地返中心动作。上位机可在任意阶段通过普通或聚集标记的`APPROACH_TARGET`、空爪藏点NAV接管；SEARCH中的`HOLD`不停止本地扫描。F407收到第一条合法APPROACH即进入靠近，不在下位机重复增加确认帧数。
6. 收到1帧原生1280×1024合法单目标坐标后锁定该类别并进入`APPROACH`。居中时以350 mm/s前进；后续只有同一类别的合法单目标新SEQ才能刷新X/Y。水平坐标使用0.55低通、Kp=0.72、Kd=0.020，旋转目标限制±175 mm/s并以1000 mm/s²平滑变化；相机PID以30°/s限速。舵机3首次达到125°时底盘立即停止平移并只做水平对正，横向误差≤24 px后把相机非阻塞移到140°；随后锁存当前IMU航向，以350 mm/s直行并等待目标产生新的有效报告。目标在该段重新出现且横向误差≤96 px便进入抓取观察；偏差更大则回125°重新对正。
7. 上述140°限距重获成功后，F407持续通过`TYPE=0x17`置`CLAW_VISIBLE=1`，同时保持锁存航向并以150 mm/s慢速前进，不把横向误差≤96 px当成停车条件。收到上位机带`STABLE`的非空`CARGO_AUDIT`后立即停车等待`GRAB_CONFIRMED`。350 mm/s重获段和150 mm/s观察段从140°稳定点共用300 mm编码器上限；达到上限仍未确认物体则停车进入`REACQ`，舵机3以1°/40 ms慢慢抬到90°寻找，仍无目标才回`SEARCH`。单侧分离后的夹内复审始终原地进行，不使用慢速前进。
8. 如果上位机报告画面无物体，摄像头每次抬高10°（舵机角度140→130→…→90），每次抬高后停车观察1秒；仍没有物体就以100 mm/s慢速旋转观察。转满一圈仍没有物体则再次抬高并重复。在视觉报告超时期间保持停车。
9. 上位机在F407持续上报`CLAW_VISIBLE=1`且状态帧不超过250 ms时，连续3个视觉周期确认画面仍有目标，然后以20～50 Hz重复发送`TYPE=0x18 / GRAB_CONFIRMED`。F407第一次接受时让左右爪同时合到Touch姿态，后续帧只更新`acknowledged_sequence`而不重启舵机动作；仍保留2秒机构完成窗口，之后才通过20 Hz的`TYPE=0x17`持续置`GRIPPER_CLOSED=1`。合爪后摄像头保持当前抓取观察角，不提前抬头。
10. 抓取闭合后，上位机先用带`STAGE_ONLY(bit6)`的`NAVIGATE_WAYPOINT`引导到安全区预备点（上位机`5b6164f`当前配置为入口前600 mm）。F407继续使用H/D导航，但到`D=0`只停车、把摄像头抬到120°、置`DISTANCE_DONE=1`并保持`mode=10`，严禁启动旧安全区补推。藏堆和返中命令不使用该标志，原流程不变。
11. 上位机随后发送第一次`ALIGN_SAFE_ZONE`：P6/P7为红方90°或蓝方270°，F407用IMU原地对正，并确保摄像头120°命令后至少稳定300 ms，再在1.5°范围内上报`mode=11`。上位机连续3帧冻结安全区框后发送第二次带`VISUAL_CORRECTION_VALID(bit6)`的ALIGN；P2/P3为有符号水平像素误差而不是角度。F407按可标定焦距把像素误差转换为一次相对转角，最大限制±15°，完成后再次上报新鲜`mode=11`并锁存最终航向。视觉5秒失败时，上位机跳过第二次ALIGN，直接沿第一次90°/270°结果推进。
12. `ENTER_SAFE_ZONE`持续携带当前位置到围栏直线的法向距离。F407不再使用动态H或横移，只以IMU保持已锁存的最终航向；D从300 mm向机构接触位置靠近时，前进速度由400平滑降至200 mm/s。D到约113 mm机构理论接触距离后锁存最终补推，忽略后续定位D，以300 mm/s和编码器再前进200 mm，最迟1200 ms按预期接触完成。随后停车、双爪完全张开，才上报`mode=15`供上位机执行原投送确认。
13. 1200 ms最短静止窗口结束后，收到`TASK_COMPLETE`先进入`EXIT_SAFE_ZONE(mode16)`，以400 mm/s、编码器定距和IMU航向保持后退0.30 m，避免贴围栏原地掉头。完成后进入`FACE_FIELD_CENTER(mode17)`；最新版上位机看到mode16或mode17后持续发送`RETURN_CENTER`行进方向H和剩余距离D。F407在mode17读取最新H，先原地转到3°误差内，再按D以最高800 mm/s向前返中；行驶中继续做限幅100 mm/s的小幅航向修正，偏差达到8°才停车重新对向。收到`D=0 mm`后停车并进入90°/120°两层`SEARCH`。是否真正到达场地原点完全取决于上位机发送的D。

上位机在收到STM32的`TYPE=0x17`退出、返中或搜索状态后进入下一轮；第一次投送完成前只选择单个绿色物资，之后允许同类普通/核心批次或单个伤员，危险目标不进入正式投送。

NormalRun不再用单轮编码器方向、单轮零速或定距短时无进展自动进入`TASK_FAULT_MOTOR`，行驶停滞改由上位机融合位姿负责判断和下发恢复/终止命令；独立的MotionDebug与Gamepad固件仍保留这些本地电机保护。收到`ABORT`、IMU/位姿等运动前提持续失效、动作自身超时或非法状态时仍立即停车。安全区对正、张爪确认及航向定距命令仍要求持续保持新鲜；自主出发和搜索不依赖任务命令，连续模式没有总时长终止。

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
- `HOLD`是高层“本周期没有新动作”的心跳：在`SEARCH`中F407仍执行本地90°/120°扫描；在APPROACH正常跟踪、125°水平对正、NAV、RETURN和远程动作中安全停车。相机由125°移到140°、140°限距前进以及140→90°慢抬重获属于已经触发的本地近距序列，HOLD只表示当前没有新目标帧，不会中止这三个子阶段。需要冻结任何阶段时使用`PAUSE=1`；F407会ACK并锁存停车，直到收到一条当前状态接受的新SEQ非PAUSE命令。
- `STOP`只在NAV中保留原有可恢复兼容行为，其他状态会进入远程停止故障；`ABORT`始终是锁存故障停止。普通等待、视觉暂时不确定不得用STOP或ABORT代替HOLD/PAUSE。
- 正式投送的`STAGE_ONLY NAV`兼容两种flags：精简形式`VALID|DISTANCE_VALID|STAGE_ONLY|阵营位`，以及在此基础上同时增加`DRIVE_STRAIGHT|USE_FINAL_HEADING`的完整形式；两方向位只出现一个属于非法帧。P2/P3仍为剩余距离/0，P6/P7仍为0～35999的场地平移方向。预备点D=0后置`DISTANCE_DONE`并保持mode10，不执行旧D=0补推；旧非STAGE NAV、藏堆NAV和RETURN仍强制要求完整方向flags，原语义不变。
- 合法新流程为`WAIT_NAVIGATION→STAGE NAV(mode10)→定位ALIGN(mode11)→可选视觉ALIGN(新鲜mode11)→ENTER→CHECK(mode15)→TASK_COMPLETE→EXIT_SAFE_ZONE→RETURN_CENTER`。
- 第一次ALIGN的P6/P7是90°/270°绝对航向；第二次ALIGN的P2/P3是冻结框相对640 px中心的有符号像素误差。视觉修正只能应用一次，重复帧只更新ACK，不能重复累加角度。
- ENTER的P2/P3是围栏法向剩余距离、P4/P5为0；视觉对正成功时bit6置位且P6/P7为0，视觉失败回退时bit2置位并在P6/P7重复90°/270°。F407锁存航向后不再接受动态H。
- 红方前置点为`(0,+950 mm)`，蓝方前置点为`(0,-950 mm)`。F407不接收PWM值，只接收任务目标并在本地完成转向、速度限制和失联停车。

### 完整比赛流程扩展

- `APPROACH_TARGET`的P2/P3和P4/P5分别为原生图像X/Y；完整流程激活后它是靠近控制的唯一坐标源。F407不再按250 ms帧龄自动减速或退出，HOLD/PAUSE/STOP以及阶段切换必须由上位机明确发送。
- 普通目标到达抓取前沿后，F407在舵机3命令到140°后先停车等待500 ms，并丢弃等待前的旧目标帧；随后以350 mm/s重新观察。第一帧重新出现的锁定目标若`|X-640|<=96`，只表示具备夹内观察基础条件：F407进入WATCH、置`CLAW_VISIBLE=1`，但保持IMU锁向并以150 mm/s慢速爬行，不再立即停车。上位机发送带`STABLE`的非空`CARGO_AUDIT`后F407立即停车并等待GRAB；从140°稳定点累计最多前进300 mm，仍未确认夹内物体则转入mode24/REACQ。该入口不增加Y坐标或目标连续帧门槛；单侧释放后的复审仍原地进行。
- `CARGO_AUDIT`按字节编码左右类别、数量、审核标志、audit_id和总数量。F407接受显式STABLE帧，也兼容上位机在第3帧直接切换GRAB的行为：前两帧审核内容一致即可建立本地稳定门。`initial_stash=1`只用于把开局物资堆搬离中心，稳定审核只要求`total_count>0`，不限制类别、总数量，也不把左右2-bit饱和计数之和作为容量联锁；正式投送仍要求首件恰好1件绿色，首件完成后允许1～3件普通、核心或`MIXED_MATERIAL`组合，伤员仍须单独1件。危险、未知、超过3件、计数矛盾和伤员混装均不允许直接正式投送。
- 非法组合优先单侧分离。`RELEASE_LEFT/RIGHT`要求最近稳定审核显示被打开侧确实非空；带`SIDE_VALID`的`DISPERSE_PILE`还要求指定保留侧非空。执行角度为：`RELEASE_LEFT`左108°/右115°，`RELEASE_RIGHT`左65°/右72°；释放侧完全打开，保留侧相对普通Touch额外夹紧15°。随后`YIELD_BACKOFF`不做分段直退，而按上位机给出的退让距离执行镜像曲线退出：500 mm/s后向、180 mm/s向保留侧横移、100 mm/s向释放侧转头。曲线完成后相机到140°并重新审核；合法非空批次继续GRAB，带`STABLE`的空爪审核则双开并回SEARCH。两侧选择顺序为：含绿色侧优先；两侧都有绿色时数量较少侧优先；仍相同时再比较锁定目标数、轨迹稳定度和距离。仍无法可靠分侧时不发送真正双开，而用无侧DISPERSE触发12°观察转向并重新审核。
- 临时藏堆NAV不使用`STAGE_ONLY`，也不执行任何安全区补推；到点后`RELEASE_BOTH`。释放完成后至返中完成期间，F407拒绝`APPROACH_TARGET`和`DISPERSE_PILE`，避免残留找物命令抢占藏堆回程。第一条`RETURN_CENTER`先触发F407保持释放航向直退0.35 m，编码器累计距离且IMU修正航向；退到安全距离后才使用上位机持续更新的H原地调头并按D返中。该退让只对临时藏堆生效，正式安全区投送仍使用既有mode16后退0.30 m，不会重复后退。
- `YIELD_BACKOFF`只在F407已确认完成`RELEASE_LEFT/RIGHT`且`cargo_recheck_pending=1`时接受；参数A仍是退让距离（建议-300 mm，符号仅为兼容，F407按绝对值执行曲线长度）。聚集APPROACH仍先Touch靠近至相机130°并在`mode=37`等待。`DISPERSE_PILE`有两种合法语义：带`SIDE_VALID(bit6)`时，`TARGET_RIGHT(bit7)=0/1`分别表示保留左/右目标；F407要求所选保留侧计数非零，打开非目标侧、把目标侧由Touch额外夹紧15°，以固定0.30 m镜像曲线退出，相机到140°稳定300 ms后报告`mode=35`并等待新的`CARGO_AUDIT`。不带bit6/bit7时表示上位机无法可靠判断左右，F407原地转12°、保持140°并稳定300 ms，同样报告`mode=35`等待新审核；`TARGET_RIGHT=1`但`SIDE_VALID=0`仍是非法组合。上位机重新分侧后发送带SIDE_VALID的DISPERSE，不能把观察转向误判成分离已经结束。
- 对外mode映射为`20 APPROACH_TARGET、21 CAPTURE_AUDIT、22 CAPTURE_DONE、30 YIELD_DONE、31 ESCAPE_DONE、32/33/34 RELEASE_DONE、35 DISPERSE_DONE、36 LANE_DONE、37 DISPERSE_READY`，不改变旧内部TaskState数值。
- 返中只接受新鲜`RETURN_CENTER D=0`作为到达；HOLD无论距离远近都只停车，不再用旧的650 mm兼容门提前伪造回中完成。F407进入`mode=3 SEARCH`后的1500 ms交接窗口内仍会确认重复RETURN帧的SEQ，但不会停止或改变SEARCH动作，用于兼容仅比较当前8位ACK的旧上位机。上位机仍必须把“本次RETURN曾被确认”锁存为布尔状态，不能把该兼容窗口作为长期握手机制。
