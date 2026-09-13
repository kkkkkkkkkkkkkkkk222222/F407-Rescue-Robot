# 连续物资抓取与分区投送流程

本固件已接入`danmo-teng/shijue_fangan@bdcf0f2`的完整比赛流程命令，运行模式为`APP_ENABLE_TASK=1`，其他测试和独立视觉居中Task均关闭。RDK X5与F407使用USART3（PD8 TX、PD9 RX）、115200 8N1、3.3 V TTL和公共15字节帧。临时底盘改动及恢复方法见[`docs/TEMP_CHASSIS_HANDOFF.md`](docs/TEMP_CHASSIS_HANDOFF.md)。

## 整体流程

1. 上电后先保持TIM8舵机PWM关闭，完成IMU配置、静止稳定和陀螺仪零偏校准。只有`IMU_Init()`成功后才启动4路舵机PWM，随后摄像头到90°、舵机1到55°，左右爪依次收缩到左23°、右147°；IMU失败则机构和正式Task均不启动。
2. 收到合法`TYPE=0x11`红蓝方和出发区配置后回复一次`A3 B3 01 C3`并启动连续任务，不设置180秒自动终止。
3. 用编码器累计路程倒车1.70 m。前600 mm由`Motor_MoveDistance(-0.60 m, 800 mm/s)`直接锁存两路前向编码器起点，以3 mm容差做定距闭环并用IMU保持启动航向；舵机1保持55°、左右爪保持Retract，确保机构不碰障碍区。完成600 mm后舵机1才到85°、左右爪同时打开到左108°/右72°；剩余约1.10 m以最高900 mm/s使用`Motor_MoveSpin()`保持地面直线路径，同时根据IMU航向误差闭环转向约180°。总里程最后100 mm降到160 mm/s，进入10 mm容差后制动。
4. 到达中心附近后不执行Touch、单独转180°、停车等待或无条件开局打乱；确认并行开爪动作已经完成后立即进入`SEARCH`。上位机确认聚集目标后不能在远处直接发`DISPERSE_PILE`，而应持续发送带`CLUSTER_TARGET`的APPROACH，使F407以最高300 mm/s靠近到相机130°并水平对正；看到新鲜`mode=37`后才持续发送DISPERSE。
5. 每次进入`SEARCH`都把舵机3从当前命令角移到90°，稳定300 ms后以200 mm/s原地旋转一整圈；随后切到120°、稳定300 ms再转一圈。两层共约720°完成后F407仍保持`SEARCH`并重新开始90°扫描，不自行启动本地返中心动作。上位机可在任意阶段通过普通或聚集标记的`APPROACH_TARGET`、空爪藏点NAV接管；SEARCH中的`HOLD`不停止本地扫描。F407收到第一条合法APPROACH即进入靠近，不在下位机重复增加确认帧数。
6. 收到1帧原生1280×1024合法单目标坐标后锁定该类别并进入`APPROACH`。居中时以350 mm/s前进；后续只有同一类别的合法单目标新SEQ才能刷新X/Y。水平坐标使用0.55低通、Kp=0.72、Kd=0.020，旋转目标限制±175 mm/s并以1000 mm/s²平滑变化；相机PID以30°/s限速。舵机3首次达到125°时底盘立即停止平移并只做水平对正，横向误差≤24 px后把相机非阻塞移到140°；随后锁存当前IMU航向，以350 mm/s直行最多300 mm并等待目标产生新的有效报告。目标在该段重新出现且横向误差≤96 px便停车进入抓取观察；偏差更大则回125°重新对正。300 mm内仍未重获时停车，舵机3以1°/40 ms从140°慢慢抬到90°，全过程继续接收视觉帧；重获立即恢复APPROACH，到90°仍无目标则返回SEARCH。
7. 上述140°限距重获成功后停车，并持续通过`TYPE=0x17`置`CLAW_VISIBLE=1`，让上位机检查抓取画面。
8. 如果上位机报告画面无物体，摄像头每次抬高10°（舵机角度140→130→…→90），每次抬高后停车观察1秒；仍没有物体就以100 mm/s慢速旋转观察。转满一圈仍没有物体则再次抬高并重复。在视觉报告超时期间保持停车。
9. 上位机在F407持续上报`CLAW_VISIBLE=1`且状态帧不超过250 ms时，连续3个视觉周期确认画面仍有目标，然后以20～50 Hz重复发送`TYPE=0x18 / GRAB_CONFIRMED`。F407第一次接受时让左右爪同时合到Touch姿态，后续帧只更新`acknowledged_sequence`而不重启舵机动作；仍保留2秒机构完成窗口，之后才通过20 Hz的`TYPE=0x17`持续置`GRIPPER_CLOSED=1`。合爪后摄像头保持当前抓取观察角，不提前抬头。
10. 抓取闭合后，上位机根据最新T265+编码器融合位置持续计算绝对航向和剩余距离。剩余超过300 mm时NAV最高800 mm/s；进入最后300 mm后限速约400 mm/s。首次进入`D≤50 mm`时，F407不采用当前车头或上位机后续H，而是按赛前锁存颜色选择放置区法向；第一次投送时红方由90°右偏到80°、蓝方由270°左偏到280°，第一次收到`TASK_COMPLETE`后，后续恢复红方90°、蓝方270°；立即停车并原地对正到误差≤1.5°后推进末段。行驶修正死区为1°，动态航向偏差达到4°会停车重新对正。锁存后只用IMU保持固定方向，D重新大于100 mm才解除锁存。首次收到NAV的`D=0 mm`后，不立即置`DISTANCE_DONE`，而是在IMU保持锁存方向下以200 mm/s慢推，保证至少1.5 s、最长2.0 s；编码器累计440 mm仅在满1.5 s后才允许作为安全停止上限。上位机若提前发送ENTER，F407仍会先建立同一最终航向锁存、完成对正和末端慢推，再张爪。
11. 安全区车头对正状态已经删除。上位机确认NAV到达放置点后直接发送`ENTER_SAFE_ZONE`，F407从`TASK_NAVIGATE`直接进入张爪，不再进入旧mode11，也不后退0.30 m或前冲0.55 m。双爪完全张开后摄像头转到120°，稳定300 ms才进入`RAM_VERIFY/CHECK`并原地停车至少1200 ms，给上位机连续确认物资“区外→区内”的视野和时间。命令4保留为协议兼容值；若旧上位机仍发送ALIGN，F407只停车等待新版ENTER，不会旋转。
12. 1200 ms最短静止窗口结束后，收到`TASK_COMPLETE`先进入`EXIT_SAFE_ZONE(mode16)`，以400 mm/s、编码器定距和IMU航向保持后退0.30 m，避免贴围栏原地掉头。完成后进入`FACE_FIELD_CENTER(mode17)`；最新版上位机看到mode16或mode17后持续发送`RETURN_CENTER`行进方向H和剩余距离D。F407在mode17读取最新H，先原地转到3°误差内，再按D以最高800 mm/s向前返中；行驶中继续做限幅100 mm/s的小幅航向修正，偏差达到8°才停车重新对向。收到`D=0 mm`后停车并进入90°/120°两层`SEARCH`。是否真正到达场地原点完全取决于上位机发送的D。

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

`P0=COMMAND、P1=FLAGS、P2/P3=REMAINING_DISTANCE_mm、P4/P5=0、P6/P7=HEADING_cdeg`。距离为0～5000 mm，表示上位机依据最新融合位置计算的实时剩余距离；航向范围0～35999，单位0.01°。FLAGS bit0必须为1，bit1要求直行，bit2使用航向，bit3表示红方，bit4表示距离有效。bit5仅在`APPROACH_TARGET`中表示`CLUSTER_TARGET`，其他命令携带bit5会被整帧拒绝。上位机应持续递增SEQ并更新航向/余量；F407只执行最新合法值。

命令值：`0 STOP、1 PAUSE、2 GRAB_CONFIRMED、3 NAVIGATE_WAYPOINT、4 ALIGN_SAFE_ZONE、5 ENTER_SAFE_ZONE、6 TASK_COMPLETE、7 ABORT、8 RETURN_CENTER、9 APPROACH_TARGET、10 HOLD、11 YIELD_BACKOFF、12 ESCAPE_MANEUVER、13 RELEASE_LEFT、14 RELEASE_RIGHT、15 RELEASE_BOTH、16 DISPERSE_PILE、17 CHANGE_LANE、18 CARGO_AUDIT`。


- `GRAB_CONFIRMED`会重复发送直到新鲜`TYPE=0x17`置`GRIPPER_CLOSED=1`；F407对舵机动作幂等，但每个新SEQ都必须更新P4 ACK。
- `HOLD`是高层“本周期没有新动作”的心跳：在`SEARCH`中F407仍执行本地90°/120°扫描；在APPROACH正常跟踪、125°水平对正、NAV、RETURN和远程动作中安全停车。相机由125°移到140°、140°限距前进以及140→90°慢抬重获属于已经触发的本地近距序列，HOLD只表示当前没有新目标帧，不会中止这三个子阶段。需要冻结任何阶段时使用`PAUSE=1`；F407会ACK并锁存停车，直到收到一条当前状态接受的新SEQ非PAUSE命令。
- `STOP`只在NAV中保留原有可恢复兼容行为，其他状态会进入远程停止故障；`ABORT`始终是锁存故障停止。普通等待、视觉暂时不确定不得用STOP或ABORT代替HOLD/PAUSE。
- 当上位机NAV剩余距离首次为0时，F407保持`MODE=NAVIGATE`执行200 mm/s末端慢推，至少持续1.5 s、最长2.0 s，满1.5 s后才允许440 mm编码器上限提前结束；补推完成后才置`P0 bit5 DISTANCE_DONE=1`作为诊断。正常状态切换仍由上位机发送`ENTER_SAFE_ZONE`，提前到达的ENTER会等待补推完成。
- 合法命令前驱固定为`WAIT_NAVIGATION→NAVIGATE→ENTER_SAFE_ZONE→CHECK→TASK_COMPLETE→EXIT_SAFE_ZONE→RETURN_CENTER`。命令`ALIGN_SAFE_ZONE=4`仅保留帧解析兼容，旧mode11、13、14已经从Task状态枚举和LCD中删除，但mode12、15、16、17等现用编号保持不变。需要Location的阶段允许短暂失效并停车等待，连续1500 ms无效才报告`POSE_TIMEOUT`。
- `NAVIGATE_WAYPOINT`和`RETURN_CENTER`使用持续更新的航向+剩余距离；`ENTER_SAFE_ZONE`只触发张爪放置，不再触发航向对正。
- 红方前置点为`(0,+950 mm)`，蓝方前置点为`(0,-950 mm)`。F407不接收PWM值，只接收任务目标并在本地完成转向、速度限制和失联停车。

### 完整比赛流程扩展

- `APPROACH_TARGET`的P2/P3和P4/P5分别为原生图像X/Y；完整流程激活后它是靠近控制的唯一坐标源。F407不再按250 ms帧龄自动减速或退出，HOLD/PAUSE/STOP以及阶段切换必须由上位机明确发送。
- `CARGO_AUDIT`按字节编码左右类别、数量、审核标志、audit_id和总数量。F407接受显式STABLE帧，也兼容上位机在第3帧直接切换GRAB的行为：前两帧审核内容一致即可建立本地稳定门。`initial_stash=1`只用于把开局物资堆搬离中心，稳定审核只要求`total_count>0`，不限制类别、总数量，也不把左右2-bit饱和计数之和作为容量联锁；正式投送仍要求首件恰好1件绿色、后续同类普通或同类核心为1～3件、伤员单独1件。危险、未知、超过3件、计数矛盾、伤员混装以及普通+核心混装均不允许直接正式投送。
- 非法组合优先单侧分离。F407只接受与最近稳定审核一致的释放侧：`RELEASE_LEFT`为左108°/右110°，`RELEASE_RIGHT`为左70°/右72°，这两条单侧分离仍按既有YIELD和复审流程。若上位机无法判断左右，发送`RELEASE_BOTH`触发特殊撞分：`双开留物→后退0.40 m→Touch闭爪→700 mm/s前撞0.40 m→450 mm/s退回0.40 m→再次双开`，完成后上报`mode=34`。该动作没有保留夹内物资，上位机不得发送YIELD或CARGO_AUDIT；确认新鲜mode34和ACK后清除当前批次并发送HOLD，F407进入SEARCH。临时藏堆的`RELEASE_BOTH`仍是普通双开。
- 临时藏堆NAV不执行安全区D=0末端慢推；到点后`RELEASE_BOTH`。释放完成后至返中完成期间，F407拒绝`APPROACH_TARGET`和`DISPERSE_PILE`，避免残留找物命令抢占藏堆回程。第一条`RETURN_CENTER`先触发F407保持释放航向直退0.35 m，编码器累计距离且IMU修正航向；退到安全距离后才使用上位机持续更新的H原地调头并按D返中。该退让只对临时藏堆生效，正式安全区投送仍使用既有mode16后退0.30 m，不会重复后退。正式安全区NAV仍保留200 mm/s、至少1.5 s、最长2.0 s和440 mm安全上限。
- 普通`YIELD_BACKOFF`使用750 mm/s编码器定距和IMU航向保持；单侧分离后，最初100 mm为450 mm/s，后段恢复750 mm/s，退让后舵机3到140°并稳定300 ms才报告`YIELD_DONE`。`DISPERSE_PILE`只在空爪、未等待复审且已到`mode=37`时执行：F407接受命令时锁存IMU航向为0°，双爪先同时闭到Touch（左80°、右100°），再以400 mm/s专用转速分别闭环到`+45°、-45°、+90°、-90°`，取消最后回正，并从当前航向以550 mm/s后退0.30 m。动作完成报告mode35，专用保护上限15秒；Touch仅作为打散接触面，不会置`GRIPPER_CLOSED`状态位。
- 对外mode映射为`20 APPROACH_TARGET、21 CAPTURE_AUDIT、22 CAPTURE_DONE、30 YIELD_DONE、31 ESCAPE_DONE、32/33/34 RELEASE_DONE、35 DISPERSE_DONE、36 LANE_DONE、37 DISPERSE_READY`，不改变旧内部TaskState数值。
- 返中只接受新鲜`RETURN_CENTER D=0`作为到达；HOLD无论距离远近都只停车，不再用旧的650 mm兼容门提前伪造回中完成。上位机必须持续发送RETURN直到F407上报`mode=3 SEARCH`。
