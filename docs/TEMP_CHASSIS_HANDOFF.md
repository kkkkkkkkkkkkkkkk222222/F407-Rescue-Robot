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

1. 架空车轮，确认前500 mm舵机1保持55°、双爪保持Retract且车身不旋转；超过500 mm后舵机1到85°、双爪同时打开并开始闭环转向。累计倒车1.70 m，只在最后100 mm减速，到达后不再单独转向或等待5秒。
2. 确认LCD从`START`经极短的`OPEN`直接进入`SEARCH`，不出现`PILEIN/SPIN+/SPIN-/PILEOUT`。
3. USART1发送`DEBUG`，确认车轮停止；分别发送4路舵机角度，核对机械限位；发送`RUN`后重新启动上位机。
4. 以20～50 Hz注入不同SEQ的`GRAB_CONFIRMED`，确认只启动一次合爪、LCD ACK持续跟随，动作完成后`GRIPPER_CLOSED`持续为1。
5. 合爪未完成时注入NAV，确认不再补抓或进入返航。
6. 返航时改变`HEADING_cdeg`，确认底盘按新航向重新对正。暂停任务命令约0.5秒，确认LCD显示`HOLD`且车速降到250 mm/s；暂停超过1秒，确认停车；恢复命令后应重新对正并继续。
7. NAV中注入`STOP`，确认停车但不进入`TASK_STOPPED`，恢复NAV后继续；注入`ABORT`则必须永久故障停车。
8. 返航时停止`TYPE=0x16`融合位姿，确认立即停车且不沿旧位姿继续；恢复GOOD位姿后应继续。
9. 确认安全区对正和撞送的250 ms严格失联保护未被放宽。

## 9. 已知限制

- 开局到中心使用编码器路程和IMU航向保持，不是T265绝对坐标闭环；轮胎持续同向打滑会造成实际距离偏差。
- 假设中心附近只有一个物块；若第一圈搜索漏检，现有算法仍可能向前搜索0.8 m。
- NAV断流保持只能覆盖短暂调度或视觉卡顿，不能替代上位机命令ACK/重发。
- 上位机发出`TASK_COMPLETE`后仍停留在`COMPLETE`，下一轮自动搜索需要上位机增加任务复位逻辑。

## 10. 2026-09-05协议复核待处理项

对照`danmo-teng/shijue_fangan@c09aef2`时确认以下小差异，当前先记录、未修改：

1. 未闭爪时提前收到`NAVIGATE_WAYPOINT / ALIGN_SAFE_ZONE / ENTER_SAFE_ZONE`，现代码只忽略命令，最新参考语义要求ACK并强制停车等待，不得继续扫描或靠近。
2. `TASK_ALIGN_SAFE_ZONE`中融合位姿失效会进入永久`POSE_TIMEOUT`故障；最新语义建议与NAV一样暂停并在位姿恢复后重新对正。
3. NAV命令缺少`USE_FINAL_HEADING`时，现代码会退回本地`bearing_to()`计算；最新参考实现要求停车等待合法航向。

## 11. 2026-09-05启动、夹爪与相机时序简化

本次只修改三个局部行为，没有改变视觉协议、返航导航、对正或撞送距离：

- `TASK_START`删除原来的“到达后制动等待→Touch→单独转180°→等待5秒”子步骤。考虑出发障碍区，前500 mm仅由`Motor_MoveAngle()`保持启动航向直线倒退，舵机1保持55°且双爪保持Retract；超过500 mm后才把舵机1转到85°、同时打开双爪，并由`Motor_MoveSpin()`在剩余1.20 m内保持地面直线、闭环转约180°。主体请求850 mm/s，最后100 mm以160 mm/s收尾。
- `Claw_Open()`改为左128°/右52°同时发出；因此开局和安全区撞送前都是同时开爪。`Claw_Touch()`改为左80°/右100°同时发出，但仍等待2秒才对上位机上报闭合。上电收纳`Claw_Retract()`继续先左23°、后右147°，避免改变最敏感的机械收纳顺序。
- 进入`TASK_WAIT_NAVIGATION`时不再把摄像头抬到90°，返航期间保持抓取观察角。只有收到`TASK_COMPLETE`确认物体已经送入、进入`TASK_EXIT_SAFE_ZONE`时，摄像头才转到120°，为下一轮搜索准备。

现场重点确认：旋转与平移叠加时轮速会由底盘混控器按统一上限分配，850 mm/s是请求上限；先验证1.70 m终点和旋转方向，再根据实车惯性微调`APP_START_TURN_KP_MM_S_PER_DEG`和`APP_START_TURN_MAX_MM_S`，不要先改编码器距离。
