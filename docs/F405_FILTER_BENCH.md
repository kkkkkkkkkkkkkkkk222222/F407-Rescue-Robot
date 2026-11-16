# MotionDebug 高度筛选：左右平移测试

2026-11-16：按用户要求用左右平移各100mm替换90°转向，默认抬升80°，改善本测试50mm后退的低速停走。已烧录并经用户指令实测两次。

## 当前命令

COM18，115200/8N1，CRLF结尾。`FILTER`默认80°；`FILTER 80`显式选角，可选61°～85°。

1. S1放回85°，沿原机构等待。
2. 轮程后退50mm，请求速度由150提高到300mm/s，复用`Motor_MoveDistanceConstant`，保留原加速、航向保持、3mm到距阈值和刹车，取消本短段后半程降速。
3. 刹车结束后S1抬到80°（或命令指定角），沿原机构等待。
4. 左平移100mm，再右平移100mm；请求250mm/s，沿原横移控制和速度收尾，两段共用起始场地方向、各自记录段起点。
5. 完成后保持S1角度及左右夹爪，底盘停止。

左右夹爪/相机不由FILTER自动改变。当前实测闭合姿态S4=86°、S2=94°。上电不自动运动。
`FILTERSTAT`只查阶段，`STOP`终止剩余动作，`SOFF 1`/`SOFF`取消测试并关闭相应舵机输出。
左移结束只进入LEFT_STOP，继续RUNNING，原刹车结束才右移；右移结束才报告整段DONE。
本测试横移使用原轮程3mm到距公差；普通BODY/FIELD仍沿原10mm阈值。地面精度未实尺测量。
夹爪桌面工具的`P FILTER`是另一个静态预设入口。

## 改动依据与检查

旧日志轮速多次归零而PWM逐步积累，50mm后退出现数秒停走；原150mm/s请求在后25mm还降向120mm/s并进入原减速分支。
这次仅改FILTER后退请求/调用和后续动作，未改公共motor.c、PID、NormalRun任务、新鲜度或故障策略；没有新增ISR或周期诊断。
两个现有固件目标编译通过。临时复用原motor/Debug夹具验证恒速参数、左右段方向/共同轴、3mm阈值、中间不报DONE、STOP与不重跑；替身检查通过，夹具已恢复。

## 烧录及两次实测

- OpenOCD对`build/F405/MotionDebug/MotionDebug.elf`编程、Verified OK、复位成功。
- 实烧build_git=`c9339f996c4b3eb091b782b28dc4966e8684c4de`、dirty=1（本轮提交前构建）。expected指纹`dd0c97fad12fcac0df85b5a68b2fa4e31272bf7a9cbf70f697aad2e4d2ecf073`。
- ELF SHA256=`b6d2fa5fcbea801d1cb91309238e5cb0a49d38158d82b6e77163b64985dc0e90`。板端actual文本为F405_ROBO/MOTIONDEBUG/MODE5、BUILT Nov 16 2026 08:55:35；完整指纹未通过板端遥测读取，以OpenOCD对ELF校验为准。
- 第一次FILTER80：约1.1秒前已进入LIFT，约3.2秒查询到DONE，最终航向相对起点-3.114°。全部轮速目标/实际速度/PWM为0。
- 用户随后明确“不用修改，再测试一次”：未改代码或重新烧录，第二次FILTER80约3.2秒查询到DONE，最终航向相对起点-3.983°，三轮目标/实际速度/PWM为0。
- 两次均FAULT=NONE，当前S1=80°、左右爪闭合。上述时间为主机查询边界，不是新增板端计时，也不代表实车距离精度或过滤成功。
- 实测比旧低速后退明显缩短，但仍有约3～4°全程航向变化；直线稳定性/实际横移距离/物块效果待用户观察，未擅自继续调参。
- 日志与实烧快照保留于`tmp/filter_shift_20261116/`，重复测试日志`run80_repeat_20261116_085828.log`。
- 原YAW入口、Task空白随工作区构建保留，不纳入本次提交；其他IDE/工具改动与原暂存状态保留。

---

# 初版转向测试记录（已被左右平移版本替代）

2026-11-16，用户要求实现并自动烧录，等用户命令才执行实车动作。

## 命令与动作

程序器串口 COM18，115200/8N1，发送一行并以 CRLF 结尾：

- `FILTER` 或 `FILTER 75`：S1设85°并沿原机构等待；原轮程直线后退50mm（请求150mm/s）；原刹车结束后S1抬到75°；沿原机构等待后逆时针转90°（请求30°/s）。
- `FILTER 78` 等：本次更换抬升角，无需重新烧录。当前支持61°～85°。
- `FILTERSTAT`：只查询测试阶段、序号和目标角。
- `STOP`：沿原入口终止剩余步骤，S1保留当前目标，不自动降回85°。
- `SOFF 1` 或 `SOFF`：终止测试并关闭所选舵机输出；其他单舵机SOFF不取消测试。

上电不自动启动舵机或测试。运行时重复FILTER返回BUSY；完成后保持抬升姿态，只有新FILTER才重跑。
测试只控制S1，左右夹爪及相机保持操作人预先设定；不自动合爪。90°方向本轮未指定，采用场地正向逆时针。
这里是直接串口FILTER；夹爪桌面工具原来的`P FILTER`静态预设是另一个入口，不会运行这段后退/转向序列。

## 实现与验证

- `Main/Src/Debug.c`在原MotionDebug前台状态机串联原轮程、舵机设置及相对转角接口；没有新增ISR、协议字段或周期诊断。
- 停止、控制台占用及上位STOP沿用原入口；轮程/转向仍使用原完成及超时行为，机构等待复用`Lift_PositionWaitMs`。
- MotionDebug、NormalRun两个现有固件目标编译链接通过。测试入口仅在F405 MotionDebug启用，正式比赛动作无新增入口。
- 临时复用原motor/Debug主机夹具，验证顺序、角度范围/参数、BUSY、上电静止、完成不重跑、运行/排队STOP及上位STOP；均通过。
  编码器/舵机为替身，完成反馈人工注入，不能证明实车距离、转角或过滤效果；原夹具文件已恢复。

## 本次烧录与待命证据

- 烧录文件：`build/F405/MotionDebug/MotionDebug.elf`；OpenOCD实际编程、`Verified OK`并复位。
- 构建基础Git：`666d8d0c053fa6db869fcf634fda7203e98b766f`，dirty=1；烧录发生在本轮提交之前。
- expected构建指纹：`6509d4d4d05b001c0761e6b26d62d6c31da04c623a822dd3745fb608f17c3cae`。
- ELF SHA256：`101408212baaf22a9a953141e5bc1d175971acff7efe20f33c9976c68f1f481b`。
- 板端文本actual：`BOARD=F405_ROBO SERVICE=MOTIONDEBUG MODE=5 BUILT=Nov 16 2026 08:39:02`。
- `STATUS STATE=IDLE SRC=NONE FAULT=NONE IMU=1 ODOM=1`；`FILTER VALID=0 ACTIVE=0 PHASE=IDLE`。
- 三轮`TARGET_MM_S=0,0,0 ACTUAL_MM_S=0,0,0 PWM_PERMILLE=0,0,0`。没有发送FILTER/S/运动命令。
- 本次没有通过遥测读出完整指纹；实际Flash由OpenOCD对该ELF校验。不能把后续提交SHA当作当前板端构建SHA。
- 原工作区的YAW调试入口及Task空白保留并随本机构建存在，但不夹入本轮提交；独立IDE/夹爪工具改动同样保留。
- 原始烧录/待命日志和实烧ELF快照：`tmp/filter_bench_20261116/`。COM18查询后已关闭，等待用户发出启动指令。

尚未完成：实车50mm、75°实际高度、90°方向/角度及物块滤出效果观察。
