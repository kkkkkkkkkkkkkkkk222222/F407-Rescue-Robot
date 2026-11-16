# MotionDebug 高度筛选观察测试

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
