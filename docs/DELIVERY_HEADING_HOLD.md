# 固定速度投送直行纠偏（2026-10-16）

修改nav_push及safe_enter_follow的固定方向前进，保留原投放距离、接触超时、速度目标、审核和计数。
原先任务层nav_yaw为20mm/s/度的比例纠偏；现在固定投送段使用Motor_MoveHeading，将已锁定的场地方向转换为连续原始IMU参考，交由10ms航向PI执行。
与MotionDebug定距直行共用heading_pid及2/0.005/0参数、三轮速度闭环和极性，不再在该段叠加nav_yaw。

- 同一目标反复发送不重置航向参考或积分；不会把每次最新偏角作为新的正确航向。
- 保持绝对投送目标，包含起步残余航向误差；不是只锁定调用瞬间方向。
- 使用既有向量加速，避免固定速度阶段切换时骤变；速度目标不变，但起步不再瞬间阶跃。
- 暂停或停止后重建当前原始IMU与原目标场地方向的对应，不恢复失效的积分。
- 状态机仍拥有动作完成与投送超时，驱动接口没有另加锁存保护。
- 普通变航向NAV、视觉接近、分离弧线、Gamepad原控制路径不变。

这是航向保持，不是完整横向位置控制。横向打滑、轮径差异、初始对齐误差和IMU漂移仍可能造成横移，不能保证物理零偏移。实车应同时观察车头角度和横向位置，分别判断控制误差与打滑。

参考WPILib官方关于陀螺仪闭环保持直行航向、以及平移/航向分开控制的说明；复用本工程已有PID而不是移植其底盘库：
- https://docs.wpilib.org/en/stable/docs/software/hardware-apis/sensors/gyros-software.html
- https://docs.wpilib.org/en/stable/docs/software/advanced-controls/trajectories/holonomic.html

不需要上位机新增命令。上位机继续原NAV/ENTER握手和原锁定航向，不根据旧固定速度推算DONE，等实际完成状态。
