# 临时底盘修改交接记录

## 1. 修改范围与基线

- 日期：2026-09-04
- 底盘基线：`d431e37 feat: optimize autonomous motion and recovery`
- 对照上位机：`danmo-teng/shijue_fangan@4bbbb83`（抓取超时预计由队友后续提高到3秒）
- 正式RDK通信保持为PCB串口1 / USART3（PD8 TX、PD9 RX）、115200 8N1、固定15字节帧。
- 本次目标是临时取消开局打散、降低抓取俯仰阈值、提高抓取命令和返航链路容错，并增加不干扰正式协议的舵机调试入口。
- 修改前尚未完成的结构重构已单独保存在本地分支`wip/refactor-before-temp-chassis-20260904`，提交`c0dc7c8`，没有混入本次改动。

## 2. 启动与搜索

开局定距和航向参数没有修改：

1. 上电依次把左爪收至23°、右爪收至147°。
2. 收到合法赛前配置后，编码器累计路程倒车1.70 m。
3. 倒车方向为车体180°，`Motor_MoveAngle()`持续保持动作开始时的IMU航向。
4. 速度为850 mm/s，剩余300 mm降到350 mm/s，剩余100 mm降到160 mm/s，10 mm容差内停车。
5. 舵机1到85°，夹爪进入Touch姿态，IMU闭环原地转180°，保留原有5秒等待。
6. 打开夹爪（左128°、右52°）后直接进入`SEARCH`。

原打散状态、状态编号和参数没有删除。`Main/Inc/app_config.h`中的：

```c
#define APP_ENABLE_START_SCATTER 0
```

为0时直接搜索；改为1即可恢复“前进0.20 m、正反旋转、后退0.30 m”的原流程。这种处理不会改变上位机可见的Task状态编号。

进入每一轮`SEARCH`时仍记录视觉报告本地代次，只有进入搜索后收到的新报告才能触发`APPROACH`，因此开局定距、转向和开爪期间缓存的X/Y不会干扰搜索。

## 3. 抓取角度与命令容错

准备抓取的舵机3阈值由150°改为140°：

```c
#define APP_GRAB_VIEW_ANGLE           140U
#define APP_GRAB_LOSS_FORCE_ANGLE_MIN 140U
```

本机构中舵机3角度越大，相机看得越低。因此数值150→140表示机械上更早停止下压并进入抓取观察。

队友在`4bbbb83`中新增`GRABBING`状态。连续视觉确认目标后，上位机以20～50 Hz反复发送`GRAB_CONFIRMED`，只有新鲜STM32状态帧报告`GRIPPER_CLOSED=1`才转入`NAVIGATE`。

底盘兼容规则：

- 抓取观察/抬高/旋转观察状态第一次收到`GRAB_CONFIRMED`时进入`TASK_CLOSE_CLAW`。
- `TASK_CLOSE_CLAW`和`TASK_WAIT_NAVIGATION`中每个新GRAB SEQ都更新`TYPE=0x17 P4 acknowledged_sequence`，但不重新启动舵机。
- `Claw_Touch()`两步动作真正完成后才置`GRIPPER_CLOSED=1`，此后20 Hz持续上报，不发一次性脉冲。
- 旧版“先收到NAV则补做合爪”兼容逻辑已删除；新协议在确认闭合前不得发NAV。
- 合爪完成后底盘留在`TASK_WAIT_NAVIGATION`，收到后续NAV才进入正常返航。

时序注意：左右爪现已同时开始合拢，但底盘仍等待2秒后才置`GRIPPER_CLOSED`，因此上位机抓取超时必须大于2秒并预留UART/状态转发余量；已与队友约定使用3秒。

## 4. 返航断流保护

当前上位机的`NAVIGATE`状态不依赖画面内仍能看到物块，而是持续发送固定安全区前置点。舵机3回90°本身不会让状态机停止NAV。实际薄弱点是上位机只在出现新相机帧时更新命令文件，而原底盘对250 ms任务命令超时和150 ms融合位姿异常都直接进入永久故障。

本次只调整`TASK_NAVIGATE`：

| 条件 | 底盘行为 |
| --- | --- |
| NAV命令年龄≤250 ms，融合位姿有效 | 正常导航，远距离最高800 mm/s |
| NAV命令年龄250～1000 ms，融合位姿有效 | 沿已确认目标点继续，限速250 mm/s |
| NAV命令年龄>1000 ms | 立即停车并保留NAV状态，新命令恢复后重新对正并继续 |
| 融合位姿超过150 ms、T265非GOOD或更新被拒绝 | 立即停车并保留NAV状态，位姿恢复后重新对正并继续 |
| NAV中收到STOP | 立即停车但保留NAV状态，新NAV到达后恢复 |
| ABORT、其他阶段STOP、电机故障、1.5秒无导航进展 | 故障停车 |

命令刚断流后的理论附加运动上限约为：

```text
800 mm/s × 0.25 s + 250 mm/s × 0.75 s ≈ 388 mm
```

融合位姿无效时不会执行这段保持，不存在使用旧位姿盲跑。`ALIGN_SAFE_ZONE`、`ENTER_SAFE_ZONE`及撞送动作继续使用原有严格看门狗。

## 5. LCD任务界面

正常靠近界面保持为：

```text
STATE:APPROACH T:...
X:0640 Y:0512
CMD:GRAB OK
Angle:140
```

抓取和返航阶段改为显示更有用的握手数据：

```text
STATE:CLOSE T:...
GRIP:WAIT A:123
CMD:GRAB OK
Angle:140

STATE:NAV T:...
H:090 AGE:0045
CMD:NAV OK
Angle:090
```

`A`是最近确认的GRAB SEQ；`H`是上位机实时返航航向，`AGE`是该命令的本地年龄（ms）。`CMD`始终显示最近收到的原始动作，新协议持续重发GRAB，因此删除了1秒显示锁定变量。

- `OK`：该任务命令不超过250 ms。
- `HOLD`：NAV命令处于250～1000 ms降速保持期。
- `TMO`：任务命令已经超时。
- 尚未收到任务命令时显示`NONE`，后缀显示USART3整体接收状态。

## 6. 舵机调试模式

调试入口使用PCB串口2 / USART1（PA9 TX、PA10 RX），配置为115200 8N1，便于山外等通用串口助手直接选择。它不占用RDK的USART3，也没有修改正式15字节协议。

连接3.3 V USB-TTL并发送以回车或换行结尾的ASCII命令：

```text
DEBUG       进入调试模式，立即停车并暂停Task
S 1 55      舵机1转到55°
S 2 100     舵机2转到100°
S 3 140     摄像头舵机3转到140°
S 4 80      舵机4转到80°
RUN         停车并复位MCU，重新进入正常模式
```

规则：

- `S`命令只在`DEBUG`之后执行。
- 舵机编号只允许1～4，角度只允许0～180；舵机3继续受`Camera_SetAngle()`的0～170°软件限位。
- 合法命令返回`OK`，非法命令返回`ERR`。
- 调试模式暂停高层Task，但USART3视觉解析、编码器上传、IMU和LCD仍运行，可同时观察X/Y、上位机动作和相机角度。
- LCD首行显示`MODE:DEBUG`，末行显示最近手动设置的舵机和角度。
- `RUN`采用MCU复位而不是恢复被暂停的中间状态，避免继续执行过期的电机或机构步骤。复位后必须让上位机重新发送`TYPE=0x11`配置；最简单的方法是重新启动`mission_test`。
- PA0按键仍不参与任务启动或模式切换。

如不需要该调试入口，可将`APP_ENABLE_RUNTIME_SERVO_DEBUG`改为0；USART1接收中断不会启用。

## 7. 修改文件

- `Main/Inc/app_config.h`：临时打散开关、140°抓取阈值、NAV 1000 ms保持窗、运行时调试开关。
- `Main/Src/Task.c`：绕过打散、重复GRAB幂等ACK、实时返航航向、NAV断流/STOP/位姿恢复策略。
- `Main/Inc/DebugConsole.h`、`Main/Src/DebugConsole.c`：USART1文本调试模式。
- `Main/Src/Robot.c`：初始化和轮询调试控制台，调试时暂停Task。
- `Main/Inc/Lcd.h`、`Main/Src/Lcd.c`：显示上位机动作与调试状态。
- `CMakeLists.txt`：加入`DebugConsole.c`。
- `MISSION_PROTOCOL.md`、`README.md`：同步流程和操作说明。

## 8. 验证与现场测试顺序

已完成的静态验证应记录在提交说明中；烧录前后按以下顺序做台架测试：

1. 架空车轮，确认前600 mm以700 mm/s执行编码器定距，舵机1保持55°、双爪保持Retract且只做航向保持；定距完成后舵机1到85°、双爪同时打开并开始闭环转向。累计倒车1.70 m，只在最后100 mm减速，到达后不再单独转向或等待5秒。
2. 确认LCD从`START`经极短的`OPEN`直接进入`SEARCH`，不出现`PILEIN/SPIN+/SPIN-/PILEOUT`。
3. USART1发送`DEBUG`，确认车轮停止；分别发送4路舵机角度，核对机械限位；发送`RUN`后重新启动上位机。
4. 以20～50 Hz注入不同SEQ的`GRAB_CONFIRMED`，确认只启动一次合爪、LCD ACK持续跟随，动作完成后`GRIPPER_CLOSED`持续为1。
5. 合爪未完成时注入NAV，确认不再补抓或进入返航。
6. 返航注入带`DRIVE_STRAIGHT | USE_FINAL_HEADING | DISTANCE_VALID`的航向＋距离，确认先对向再定距；剩余300 mm时为巡航上限、150 mm时约为75%、接近0 mm时约为50%。暂停任务命令约0.5秒，确认LCD显示`HOLD`且速度上限降到250 mm/s；定距开始后暂停超过1秒应故障停车，不能恢复后重跑完整距离。
7. NAV尚未启动定距时注入`STOP`，确认停车等待并可由新NAV恢复；定距已经启动后注入`STOP`，必须以`COMMAND_TIMEOUT`故障停车，防止恢复时重跑完整距离。注入`ABORT`始终永久故障停车。
8. 默认不再发送`TYPE=0x16`。完成首件普通物资后注入`TASK_COMPLETE`，确认F407退出安全区并等待`RETURN_CENTER`；返中定距完成进入SEARCH后，上位机应清除旧目标并允许四类单目标开始下一轮。
9. 确认安全区对正和撞送的250 ms严格失联保护未被放宽。

## 9. 已知限制

- 开局到中心使用编码器路程和IMU航向保持，不是T265绝对坐标闭环；轮胎持续同向打滑会造成实际距离偏差。
- 假设中心附近只有一个物块；若第一圈搜索漏检，现有算法仍可能向前搜索0.8 m。
- NAV断流保持只能覆盖短暂调度或视觉卡顿，不能替代上位机命令ACK/重发。
- 连续搬运已依赖`RETURN_CENTER`握手；若上位机未运行到`c576447`或未转发命令8，F407会停在`CENTER`等待，不会自行猜测返中距离。

## 10. 2026-09-05旧版协议复核项

对照`danmo-teng/shijue_fangan@c09aef2`时确认以下小差异，当前先记录、未修改：

1. 未闭爪时提前收到`NAVIGATE_WAYPOINT / ALIGN_SAFE_ZONE / ENTER_SAFE_ZONE`仍只会被忽略；最新RDK状态机保证确认`GRIPPER_CLOSED`后才发NAV。若现场发现提前命令，需要双方共同增加显式`WAIT_GRIPPER`状态，不能用NAV触发补抓。
2. 已解决：`ALIGN_SAFE_ZONE`和定距不再依赖周期性融合位姿，使用本地Location/IMU；本地定位无效时停车等待。
3. 已解决：NAV和RETURN现在同时强制要求`DRIVE_STRAIGHT | USE_FINAL_HEADING | DISTANCE_VALID`，不再退回本地猜测航向。

## 11. 2026-09-05启动、夹爪与相机时序简化

本次只修改三个局部行为，没有改变视觉协议、返航导航、对正或撞送距离：

- `TASK_START`删除原来的“到达后制动等待→Touch→单独转180°→等待5秒”子步骤。考虑出发障碍区，前600 mm由`Motor_MoveDistance()`以700 mm/s执行编码器定距＋IMU航向保持，舵机1保持55°且双爪保持Retract；完成后才把舵机1转到85°、同时打开双爪，并由`Motor_MoveSpin()`在剩余约1.10 m内保持地面直线、闭环转约180°。总里程最后100 mm以160 mm/s收尾。
- `Claw_Open()`改为左128°/右52°同时发出；因此开局和安全区撞送前都是同时开爪。`Claw_Touch()`改为左80°/右100°同时发出，但仍等待2秒才对上位机上报闭合。上电收纳`Claw_Retract()`继续先左23°、后右147°，避免改变最敏感的机械收纳顺序。
- 进入`TASK_WAIT_NAVIGATION`时不再把摄像头抬到90°，返航期间保持抓取观察角。只有收到`TASK_COMPLETE`确认物体已经送入、进入`TASK_EXIT_SAFE_ZONE`时，摄像头才转到120°，为下一轮搜索准备。

现场重点确认：旋转与平移叠加时轮速会由底盘混控器按统一上限分配，850 mm/s是请求上限；先验证1.70 m终点和旋转方向，再根据实车惯性微调`APP_START_TURN_KP_MM_S_PER_DEG`和`APP_START_TURN_MAX_MM_S`，不要先改编码器距离。

## 12. 2026-09-05连续搬运与航向定距协议

- 对照上位机基线更新为`danmo-teng/shijue_fangan@c576447`，已应用其基于底盘`47d06f1`生成的`0001-Run-continuous-multi-cargo-rescue-cycles.patch`，保留补丁原作者信息。
- 新增命令`RETURN_CENTER=8`和`DISTANCE_VALID bit4`。`NAVIGATE_WAYPOINT`与`RETURN_CENTER`都要求bit1直行、bit2使用航向、bit4距离有效，载荷为`P2/P3=本段剩余距离、P4/P5=0、P6/P7=绝对航向`；非法组合直接拒绝执行。
- F407第一次启动定距时锁存编码器起点和命令距离，重复命令只刷新SEQ/看门狗，不会把已经行驶的路程重新执行。定距开始后命令超过1秒失联或收到STOP会故障停车，因为此时无法安全恢复完整原距离。
- `NAVIGATE_WAYPOINT`巡航上限800 mm/s，`RETURN_CENTER`上限500 mm/s；两者内部剩余距离小于300 mm后线性减速，理论终点速度分别为400和250 mm/s。NAV命令在250～1000 ms宽限期内把巡航上限降到250 mm/s，并由电机层实际更新速度目标。
- 首次投送前，目标类别门控由RDK限制为普通物资；首件完成后RDK解锁普通、核心、危险和伤员。F407完成撞送后接受`TASK_COMPLETE`、抬相机到120°并后退0.45 m，随后在`CENTER`等待RDK根据当前T265位置发送一次返中航向和`距中心距离-600 mm`。本地定距完成后进入SEARCH，RDK看到该状态后清除旧类别和旧路线，开始下一轮。
- 目标重捕获累计25秒失败或相机搜索到上限后不再永久故障，而是回到SEARCH选择新目标；连续模式不再使用180秒总时长停止，但通信、电机、IMU和机构看门狗仍保留。
