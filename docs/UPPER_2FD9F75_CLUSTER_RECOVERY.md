# 配套2fd9f75：聚集360°恢复与通信核对

基于F407 2d1957a；已获取并核对上位机最新 `codex/gamepad-teleop@2fd9f75`。
只修改Task内部恢复逻辑，不新增UART字段、命令、LCD、调试输出或测试。

## 修改内容

- 仅进入 `APPROACH_CLUSTER_ALIGN` 时初始化专用IMU累计器，使用IMU.ready和sample_count识别新的有效读取，重复样本不累计。
- 相邻yaw差用毫度归一化到[-180000,180000)，累计绝对值，到360000饱和。359°→1°为2°；多次往返转动也累计，不比较起止角。
- 同阶段APP新SEQ/像素变化不重置累计；离开阶段或开始新靠近任务才清理。普通SEARCH和扫障转向不使用这个累计器。
- 正常横向对正判据优先，不修改阈值；成功仍进入140°→mode38。未对正且累计≥360°则停止、清跟踪/审核并进入mode24，不GRAB、不直接跳mode3。
- 聚集对正持续HOLD触发原500 ms恢复时，同样锁存聚集刷新；普通非聚集HOLD恢复不变。
- 锁存期间同时封住 `task_process_approach_recover` 的found自动重靠近和 `task_accept_mission` 的APP立即重靠近；旧全屏报告也被屏蔽。
- mode24至少停留原500 ms，并收到**进入恢复后**的一次有效HOLD才能进入mode3。触发恢复前收到的HOLD不算握手；没收到就保持停车mode24，不新增超时故障。
- 进入mode3解除恢复锁存，但隔离被拒绝的缓存APP，直到mode3之后的新APP被接受，避免缓存报告从旁路重新启动靠近。新SEQ在mode24绝不是新目标证据。

`first_delivery_done`不由此次恢复写入；上位机的藏堆完成标志、投送次数、进攻/防守策略也不受此改动影响。首件仍恰好1 GREEN，后续普通/核心合计1～2、伤员单件；聚集靠近不是整批运输许可。
保持mode38审核、20°观察、42～46、650 mm/330 mm/s/1500 ms ENTER、闭爪S2=104/S4=76及退出相机90°→1秒→120°。

## 本轮已解决的上下位机冲突

上位机旧坐标不断换SEQ重发时，旧F407会无限原地对正；即使上位机发HOLD请求刷新，mode24也可被APP或缓存视觉拉回20。
现在本地IMU360°和上位机T265360°兜底均收敛到24→HOLD→3握手。上位机 `_cluster_approach_output` 看见24会进入WAIT_SEARCH_RECOVERY，`_search_recovery_output` 持续HOLD，见3后建立新视觉frame floor再选目标，双方对应。

## 总通信流程仍需注意的隐患

1. **新SEQ不证明新画面。** 上位机relay每10 ms重新生成SEQ，新的串口包可承载旧像素。2fd9f75已在聚集APP发布端要求真实新frame_sequence才刷新有效期，F407补足本地转角保护。UART没有视觉帧号/目标任务代号，F407仍无法独立证明mode3之后的APP来自新画面，依赖上位机frame floor；本次不扩协议。
2. **HOLD握手必须持续发送。** mode24若没有收到恢复期间HOLD就保持停车，这是本次刻意保证交接可见的行为，不是超时FAULT。旧上位机只重发APP或链路断流时会等待；2fd9f75有对应持续HOLD分支。
3. **掉线保护假设未统一。** 上位机relay在命令文件停更250 ms后停发，但F407 `task_mission_valid/task_report_valid`不检查帧龄。此次360°限制只覆盖聚集对正，不等价于所有APP/NAV/ENTER都已具备断流停车。按请求不增加超时FAULT/ABORT，建议独立设计失联策略。
4. **ACK只是接受，不是机械完成。** 状态帧只有8位ACK，无已接受opcode/事务号；100 Hz下SEQ约2.56秒回绕。上位机已有relay发送证据与accepted锁存，但仍需看新鲜mode、夹爪及审核结果。不能靠一次ACK变化判定恢复/运输完成。
5. **最新值邮箱可能丢中间命令。** F407每20 ms读取latest_data.mission，relay约10 ms一帧。HOLD/PAUSE/ABORT不能假定单帧必被任务处理；上位机持续发送安全命令的行为应保留。
6. **限时投送退出≠成功送达。** 上位机1秒视觉超时仍发送TASK_COMPLETE，下位机据此置first_delivery_done并退出；可能未实际送进却改变后续首件规则。本次按要求不改1秒策略，但不能将该标志视为绝对视觉成功证据。
7. **坐标/角度源不同。** 聚集上位机用T265，F407用IMU；导航/边界上位机用融合坐标，F407内部Location用于边界与回退。两侧阈值触发时间可以不同，应看握手与原始坐标，不以此直接认定UART丢包。

已核对仍一致：15字节帧和CRC、opcode/flags、INITIAL_STASH非空优先、实时H/D藏点、前方ROI由上位机选点、mode45全夹爪数量审核、RETURN交接、mode41 HOLD-only。
上位机step入口已有统一stm.fresh检查；CLEAR/聚集分支无需重复检查才能生效，不将其误列为缺失。

## 验证边界

Debug固件编译通过，无新增编译警告；git diff检查通过。按要求未增加/运行测试、未烧录、未实车验证。
