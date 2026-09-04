# 临时底盘修改交接记录

## 1. 修改范围与基线

- 日期：2026-09-04
- 底盘基线：`d431e37 feat: optimize autonomous motion and recovery`
- 对照上位机：`danmo-teng/shijue_fangan@337aed0`
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

队友上位机连续3个视觉周期确认目标后，只在状态切换的那个周期写入一次`GRAB_CONFIRMED`，下一周期立即用`NAVIGATE_WAYPOINT`覆盖同一个`uart_command.bin`。定位进程只转发轮询时看到的最新文件，不是逐命令队列，所以存在单发GRAB被NAV覆盖的窗口。

底盘兼容规则：

- 正常收到`GRAB_CONFIRMED`时立即进入`TASK_CLOSE_CLAW`。
- 如果底盘仍在`TASK_GRAB_OBSERVE`、`TASK_GRAB_RAISE_WAIT`或`TASK_GRAB_ROTATE`，爪子尚未闭合，却先收到有效`NAVIGATE_WAYPOINT`，则认为上位机已完成抓取确认，先补做合爪。
- 该规则不适用于`SEARCH`、`APPROACH`或其他阶段，避免普通NAV帧误触发抓取。
- 合爪完成后等待下一帧NAV，再进入正常返航。

长期建议：上位机应重复发送`GRAB_CONFIRMED`，直到`TYPE=0x17`返回`GRIPPER_CLOSED=1`或对应ACK；当前底盘兼容逻辑不能替代可靠的命令确认/重发协议。

## 4. 返航断流保护

当前上位机的`NAVIGATE`状态不依赖画面内仍能看到物块，而是持续发送固定安全区前置点。舵机3回90°本身不会让状态机停止NAV。实际薄弱点是上位机只在出现新相机帧时更新命令文件，而原底盘对250 ms任务命令超时和150 ms融合位姿异常都直接进入永久故障。

本次只调整`TASK_NAVIGATE`：

| 条件 | 底盘行为 |
| --- | --- |
| NAV命令年龄≤250 ms，融合位姿有效 | 正常导航，远距离最高800 mm/s |
| NAV命令年龄250～1000 ms，融合位姿有效 | 沿已确认目标点继续，限速250 mm/s |
| NAV命令年龄>1000 ms | 立即停车并保留NAV状态，新命令恢复后重新对正并继续 |
| 融合位姿超过150 ms、T265非GOOD或更新被拒绝 | 立即停车并保留NAV状态，位姿恢复后重新对正并继续 |
| STOP/ABORT、电机故障、1.5秒无导航进展 | 保持原有故障停车 |

命令刚断流后的理论附加运动上限约为：

```text
800 mm/s × 0.25 s + 250 mm/s × 0.75 s ≈ 388 mm
```

融合位姿无效时不会执行这段保持，不存在使用旧位姿盲跑。`ALIGN_SAFE_ZONE`、`ENTER_SAFE_ZONE`及撞送动作继续使用原有严格看门狗。

## 5. LCD任务界面

正常任务界面调整为：

```text
STATE:APPROACH T:...
X:0640 Y:0512
CMD:GRAB OK
Angle:140
```

`CMD`显示最近收到的原始`TYPE=0x18`动作：`NONE / STOP / GRAB / NAV / ALIGN / ENTER / DONE / ABORT`。由于上位机只短暂发送一次GRAB，而LCD每100 ms刷新，底盘会把确实收到的`GRAB`名称保持1秒再显示后续NAV，避免现场完全看不到抓取动作；这只影响显示，不延迟抓取或导航状态机。

- `OK`：该任务命令不超过250 ms。
- `HOLD`：NAV命令处于250～1000 ms降速保持期。
- `TMO`：任务命令已经超时。
- 尚未收到任务命令时显示`NONE`，后缀显示USART3整体接收状态。

## 6. 舵机调试模式

调试入口使用PCB串口2 / USART1（PA9 TX、PA10 RX），保持现有500000 8N1。它不占用RDK的USART3，也没有修改正式15字节协议。

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
- `Main/Src/Task.c`：绕过打散、GRAB丢帧兜底、NAV断流/位姿恢复策略。
- `Main/Inc/DebugConsole.h`、`Main/Src/DebugConsole.c`：USART1文本调试模式。
- `Main/Src/Robot.c`：初始化和轮询调试控制台，调试时暂停Task。
- `Main/Inc/Lcd.h`、`Main/Src/Lcd.c`：显示上位机动作与调试状态。
- `CMakeLists.txt`：加入`DebugConsole.c`。
- `MISSION_PROTOCOL.md`、`README.md`：同步流程和操作说明。

## 8. 验证与现场测试顺序

已完成的静态验证应记录在提交说明中；烧录前后按以下顺序做台架测试：

1. 架空车轮，确认收到配置后仍倒车1.70 m、保持航向、减速、制动和转180°。
2. 确认开爪完成后LCD从`OPEN`直接进入`SEARCH`，不出现`PILEIN/SPIN+/SPIN-/PILEOUT`。
3. USART1发送`DEBUG`，确认车轮停止；分别发送4路舵机角度，核对机械限位；发送`RUN`后重新启动上位机。
4. 注入正常`GRAB_CONFIRMED`，确认合爪。
5. 跳过GRAB，直接在`CLAW_VISIBLE`阶段注入NAV，确认只在该阶段补做合爪。
6. 返航时暂停任务命令约0.5秒，确认LCD显示`HOLD`且车速降到250 mm/s；暂停超过1秒，确认停车；恢复命令后应重新对正并继续。
7. 返航时停止`TYPE=0x16`融合位姿，确认立即停车且不沿旧位姿继续；恢复GOOD位姿后应继续。
8. 确认`STOP/ABORT`仍立即进入故障停车，安全区撞送失联保护未被放宽。

## 9. 已知限制

- 开局到中心使用编码器路程和IMU航向保持，不是T265绝对坐标闭环；轮胎持续同向打滑会造成实际距离偏差。
- 假设中心附近只有一个物块；若第一圈搜索漏检，现有算法仍可能向前搜索0.8 m。
- NAV断流保持只能覆盖短暂调度或视觉卡顿，不能替代上位机命令ACK/重发。
- 上位机发出`TASK_COMPLETE`后仍停留在`COMPLETE`，下一轮自动搜索需要上位机增加任务复位逻辑。
