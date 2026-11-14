# MATCH-CAPTURE-HANDOFF-20261114：聚堆接近与近爪分离

审查基线为本端`6ee665b64dceddc36ef9936adfd78f115cdd1ff1`；上位固定源码快照
`a13a66d87d55437a542bcc2c5a126a40e8c2c8e9`，机器合同blob`d3d331c9f9c58efa0e96433170497543bd80bf72`。
接受本轮最小配套，绿色45%门槛由RDK处理。实际上次UART身份6ee665b/dirty1来自对端14:59日志引用，
不是本窗口新读；dirty1不证明源码与板内字节相同。本轮未烧录、未开UART/设备或执行动作，用户自行烧录。

## 三处交接修复

- mode52：去掉仅普通目标可回TRACK的聚堆例外。原新像素帧误差>64px时普通/聚堆均重新对准，
  保留CLUSTER、task/action、相机测距角、已用接近预算和准备好的爪子，不重启整个Task；重复帧不触发。
- mode53：仅本阶段的Motor_MoveDistanceLinear返回RUNNING且本次实测轮程剩余≤10mm时，沿原DONE分支
  Stop→相机140°→审核。剩余来自Motor_GetDistanceTravelledMm，不来自目标速度、时间或T265。
  原12s仅兜底，全局3mm及其他有限运动/导航/扫障不变；不是把超时当完成，也不添加停滞检测。
- 稳定非法、非空且含蓝/危险的审核，不再仅因“没有合格载荷”立即自行后退。普通保持mode21/READY，
  聚堆按原分支进入mode37，交上位选择分离或原释放。READY真实保留、VALID仍false。
  原4s决策及15s审核兜底不延长；不加最低等待或新握手，真正空爪/纯蓝由原业务出口处理。

## 新标志：只表示近爪保留侧

原TYPE0x18/opcode0x10（DISPERSE_PILE）的P1 bit4=0x10命名为`DISPERSE_NEAR_KEEP`。
它与该opcode原SIDE_VALID一起使用，TARGET_RIGHT沿原含义；禁FIRST_GREEN_BUMP，A/B/U必须全0。
通用allowed_flags原来已允许bit4，本次不新增TYPE/opcode或改变帧布局/CRC/0x1D配对。
不要与opcode0x14的step高四位或opcode0x12的audit_flags/STABLE混用。

下位仅在原稳定非法审核含蓝/危险、所选保留侧真实计数0、另一侧计数>0时接受这个例外。
不把未达ROI门槛的绿色伪造成1，不改原单件确认掩码和audit_valid，不记首绿或投送。
这种零计数侧使既有counts_known=false，直接沿原5°软握和10°曲线；普通已确认单件的20°握持保持。
分离后必须按实际画面重新达到正式类别/数量规则，才能走原GRAB/运输；图像近爪旁证不是物理夹持证明。

上位左侧离线示例：审核仍是左0、右蓝1、total1；分离选左侧，P1=0x59：

```text
audit ctx: a3 b3 1d 0a 00 0a 00 64 00 00 37 21 93 28 c3
audit cmd: a3 b3 18 0a 12 01 00 04 04 12 07 01 8d c8 c3
split ctx: a3 b3 1d 0b 00 0a 00 65 00 00 37 21 a3 78 c3
split cmd: a3 b3 18 0b 10 59 00 00 00 00 00 00 4b 41 c3
```

SEQ分别10/11，task10、audit action100/分离action101，原上下文frame14113仅作证据追溯，未增加帧年龄门。
该左例来自记录几何的离线编码，右侧例是镜像合成；都不是本端实际发送记录。完整字段/两例见
[机器合同](contracts/f405_near_claw_separation_20261114.json)。

## 保持与检查边界

PID/速度、舵机、原普通DISPERSE/释放/首绿规则、全局轮程3mm、step13/SEARCH入口、STOP/PAUSE/mode51保持。
没有新增ISR、定时/新鲜度资格、遥测、永久FAULT或独立测试程序；本轮不含mode52取消或其他历史任务。
原夹具只补生产新增getter的必要链接替身，不增加测试用例/程序。相关条件由原夹具独立会话直接调试核对，
不将这些合成输入或回执当作真实设备执行证据。

NormalRun/MotionDebug构建通过；原FLOW_20261019_CHECK通过。相关调用核对覆盖普通/聚堆剩余10.1/10/5.1mm、
原故障恢复、新帧74px/64px/重复帧、预算/CLUSTER保留；原两帧审核链的左右近爪软握与真实计数/VALID/掩码不变，
重复请求不增加尝试、禁止直接GRAB；独立条件核对原4s兜底、纯蓝上位释放、无标志/错侧拒绝和普通20°握持。
原成对示例CRC/字段核对通过。这些是源码/宿主夹具结果，不是板上或分离精度验收。

部署需要用户烧新的NormalRun/TASK并重开上位正式入口；本端回执/源码/expected不冒充实际固件。
本机保留原Task初始化空白，构建如实dirty1。实体卡料、10mm末端效果与机械分离成功率仍需人安排实跑。
