# 投送后返程兼容性与末端驱动答复

日期为2026-11-12，Asia/Shanghai。答复MATCH-RETURN-20261112及MATCH-TERMINAL-DRIVE-20261112。
上位源提交为 `6edf4a0dd86264c201554d952a3128e66f6a57ab`，下位审查源为
`0d52ef81c30541ea69c5f13b53b3509cfe643e2f`。
[本轮上位交接](https://github.com/danmo-teng/shijue_fangan/blob/6edf4a0dd86264c201554d952a3128e66f6a57ab/docs/CROSS_REPO_REQUESTS.md)

本轮只核对和答复，没有修改固件、PID、速度或到位阈值，没有实施下面的驱动候选。
工作区独立舵机倒装/标定及工具改动保留，不能把这些未提交设置或仓库HEAD当成已烧录字节。

## 一、新返程点兼容，F405不需要新增功能

上位只在正常投送完成后选择红方(0,+0.30)m、蓝方(0,-0.30)m，半径15cm内锁定D=0。
F405的RETURN_CENTER名称保留，但它实际消费上位给出的剩余D和场地方位H，不在这个路径里重算地图中心。

- mode16收到RETURN时仅缓存/确认，原升降、后退和抬爪流程继续，结束后才由mode17消费最新H/D。
- mode17的原到点条件是收到的D≤80mm，D=0自然满足；它不是另一个要求定位必须回(0,0)的条件。
  因为上位已在≤150mm时主动锁D=0，正常情况下先由上位的150mm范围决定结束，无需把下位80mm改成150mm。
- 完成后沿原SEARCH交接。新返程点和容差只由上位区分，开局藏点、放弃抓取和搜索失败不应自动套用该点。
- 本轮不改变原返回搜索的相机准备与新视觉帧交接，也不将D=0解释为独立地面精度证明。

源码依据为[nav_follow](https://github.com/kkkkkkkkkkkkkkkk222222/F407-Rescue-Robot/blob/0d52ef81c30541ea69c5f13b53b3509cfe643e2f/Main/Src/Task.c#L3386)、
[mode17消费](https://github.com/kkkkkkkkkkkkkkkk222222/F407-Rescue-Robot/blob/0d52ef81c30541ea69c5f13b53b3509cfe643e2f/Main/Src/Task.c#L3884)、
[mode16提前回执](https://github.com/kkkkkkkkkkkkkkkk222222/F407-Rescue-Robot/blob/0d52ef81c30541ea69c5f13b53b3509cfe643e2f/Main/Src/Task.c#L5918)。

## 二、当前末端到底输出什么

| 项目 | 普通RETURN末段 | STAGE预备点末段 |
| --- | --- | --- |
| 车体速度请求 | min(200,max(120,D)) mm/s | min(180,max(110,D)) mm/s |
| 调用 | Motor_MoveAngle | Motor_MoveAngleFine |
| 单轮目标零速带 | 起动/换向退出6mm/s，运行进入3mm/s | 非整车换向期间0.5mm/s |
| 额外实测速差补偿 | 无Fine附加项 | 每缺1mm/s加2‰，最多150‰ |
| 全局PWM上限 | 原600‰ | 原600‰ |
| 完成含义 | API请求和motors_active均不证明轮子已转 | 非零PWM命令仍不证明轮子已转 |

两条路径都复用原矢量斜坡和gyro航向保持。Fine是现有分支，不是一套新PID。
其补偿在原PWM最终限幅内工作，随着轮子跟上目标而减小；也不保证任何机械负载下必能起转。
同task/action的正常H/D更新不会重建粗段基准；若线上动作身份变了，则是新的动作，需按实际记录区分。

依据为[nav_follow_coarse](https://github.com/kkkkkkkkkkkkkkkk222222/F407-Rescue-Robot/blob/0d52ef81c30541ea69c5f13b53b3509cfe643e2f/Main/Src/Task.c#L3268)、
[单轮死区](https://github.com/kkkkkkkkkkkkkkkk222222/F407-Rescue-Robot/blob/0d52ef81c30541ea69c5f13b53b3509cfe643e2f/Main/Src/motor.c#L325)、
[实测速差补偿](https://github.com/kkkkkkkkkkkkkkkk222222/F407-Rescue-Robot/blob/0d52ef81c30541ea69c5f13b53b3509cfe643e2f/Main/Src/motor.c#L1667)。

## 三、卡住原因的判断

能确认的是分支差异，不能凭此确认现场唯一根因。

- **普通RETURN缺Fine补偿是合理候选。** 同样车体速度下，单轮小分量、地面阻力和静摩擦可能使起转慢。
  但STAGE已使用Fine，所以不能把预备点也归因于“没有走Fine”。
- **仅凭6mm/s死区不能解释持续非零车体请求时三轮全部停。** 按当前三轮混控，理想无偏航的120mm/s平移，
  至少一轮目标绝对值≥103.9mm/s，明显超过6mm/s。零速带能吃掉某些小轮分量，不能吞掉这个完整平移向量。
  这只证明混控几何，不是实际PWM、电压、轮速或摩擦的测量。
- **反复换向/斜坡受打断仍未知。** 当前车体向量大转向门槛为120°；进入reversing后先减到零，再按原轮速条件退出。
  原100Hz调度中要求连续3次接近零，最多等40周期。方向不断翻转可能反复走该分支，但尚无本次内部状态/PWM证据。
  不能把一次换向的几十到几百毫秒，直接当作持续数秒不走的已证原因。
- 代码层面同动作的FINE更新没有每帧Motor_Stop；没有证据说明本次一定是Task反复清零，也不能将全部责任归为电机体质。

换向依据为[方向变化判定](https://github.com/kkkkkkkkkkkkkkkk222222/F407-Rescue-Robot/blob/0d52ef81c30541ea69c5f13b53b3509cfe643e2f/Main/Src/motor.c#L1189)和
[原换向处理](https://github.com/kkkkkkkkkkkkkkkk222222/F407-Rescue-Robot/blob/0d52ef81c30541ea69c5f13b53b3509cfe643e2f/Main/Src/motor.c#L1220)。

区分上述候选还缺同一停滞区间已有H/D、task/action和TD_TASK/TD_DRIVE的TARGET、PWM、ACTUAL、RAW32。
三轮TARGET与PWM都为零时，加大静摩擦补偿无助于一个尚未形成的非零目标；应先区分状态/输出被清除及换向归零。
目标与PWM持续非零但反馈不走时，才进一步看起动力、负载、驱动或编码器反馈。PWM字段是软件命令，不是端电压测量。
既有TD_DRIVE不直接回报direction_move.reversing，因此它单独也不能证明该内部子阶段命中。
本轮不新增日志字段、周期巡检或测试程序，只列清现有证据缺口。
[现有输出](https://github.com/kkkkkkkkkkkkkkkk222222/F407-Rescue-Robot/blob/0d52ef81c30541ea69c5f13b53b3509cfe643e2f/Main/Src/TaskDiag.c#L151)

## 四、建议的最小方案，等人决定后实施

1. **投送后返程先沿用上位已实现的30cm偏置点＋15cm锁定到位。** 不再扩大下位全局80mm，也不改预备点X±60/Y±50mm。
   它直接减少已接近搜索点时的无意义追点，不承诺解决容差外的电机不动。
2. **若返程仍在容差外停滞，固件最小候选只补RETURN末段的Fine调用。** 只在nav_follow_coarse的RETURN/FINE中，
   将Motor_MoveAngle替换成现有Motor_MoveAngleFine；原120～200mm/s映射、粗段、航向、距离和PID保持。
   不顺带改变藏点NAV或所有MoveAngle调用。代价是较小轮分量更容易起转、瞬态PWM可能更大，定位延迟下更容易短距过冲。
   该候选不会取消原换向流程，不能承诺解决H反复翻转或STAGE本身的停滞。
3. **STAGE已是Fine，暂不给新速度/新到位范围。** 它的问题应先按现有目标/PWM证据分类，不能再以“换成Fine”当修复。
   本轮不建议全局提速，故没有新的速度初值或PID参数，也没有把待定方案写入运行代码。

## 五、本轮实际检查与交付边界

- 固定上位提交的3项现有状态机检查在本机通过，覆盖新红蓝返程点/15cm锁D=0、原返中和原STAGE交接。
  没有重跑或修改其已说明的无关旧藏点NO_CANDIDATE失败。
- 复用原下位Task/mechanism夹具，红蓝D=0在mode16只缓存、不抢本地退出；随后mode17在远离(0,0)的位置仍按D=0进入SEARCH。
  D=151时保持原FINE，随后D=0结束，通过。理想三轮混控的120mm/s分量检查通过，不当作地面或电机实测。
- 只更新答复/固定入口/当前状态，没有修改运行代码，未重新构建固件、未开串口或设备、未烧录及启动车轮。
  现有本地NormalRun镜像仍是此前0d52ef8/dirty1构建，文档提交不会自动更新镜像身份；此刻实板actual未知。

本端会将回复提交推送至main的固定入口，供RDK读取。发布回复不等于RDK已经读取、接受方案或完成实车验证。
