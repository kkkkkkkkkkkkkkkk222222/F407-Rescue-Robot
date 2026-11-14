# MATCH-ATTACK-RETURN-20261114：step13空载保向平移

2026-11-14后续MATCH-ATTACK-FACING已配套step14：进攻藏点NAV完成后先保持闭爪带载转向，红270°/蓝90°，
再原RELEASE_BOTH/mode34及本文step13返程。原0..13字段不变，详见[本次增量说明](F405_ATTACK_AUDIT_ADAPTATION_20261114.md)。
step14和原step13红蓝报文一并记录在本页机器合同，用户自行烧录；未称新转正已上板。

审查接受原0x14/step13布局，基于本端`f1f8f183c5f14fb24462a76834b0ab0613e75967`增量实现。
已核对RDK固定快照`a8d1242ac19b0e69d9c709be80adcfdafc76e1c5`及机器合同blob`4610fe8cb88e93313087a38e210224c9f57a5972`。
NormalRun和MotionDebug已编译通过；本轮未烧录、未打开UART或执行任何机构/车轮动作，板端actual未读。

后续MATCH-SEARCH-ROTATION-20261114已补空爪SEARCH入口及同场景cancelled-context例外，基于RDK`8db11ec29b297d8c00173f42ecc6b371c01958dd`。
只扩入口，不改接收/字段/执行器；两站轮换、侧通道、途中目标让位和失败记录由上位负责。当前配套及10组原报文见
[搜索轮换机器合同](contracts/f405_search_station_20261114.json)。用户明确自行烧录，本端不烧录或启设备。

## 分工和兼容

上位机在本次双爪释放完成后冻结实际放货车中心与场地yaw，再编排退10cm、左50cm、退到返程纵向线、右移到己方30cm点。
每段目标和5cm到点容差属于上位机；下位不存固定路线坐标，不按首个D独自走完后提前宣布到点。
RDK已将段index加入现有action键：同段H/D更新保持action，换段使用新action，task保持，原编号回绕规则保留。
防守开局/普通RETURN、mode51及原step0..12不变；上轮清障step8标准固定航向、定位速度及PID/PWM/舵机参数不变。

## 线上字段

物理UART4仍921600/8N1；原15字节帧和CRC16/MODBUS、0x1D先于0x18的同SEQ配对保持。
这次新增的是TYPE0x18内部opcode0x14的step13，不是新增UART TYPE或mode13。

| 字段 | 含义/范围 |
|---|---|
| P0 | opcode0x14 |
| P1 | 高四位13；低位VALID0x01、红方0x08；0x02/0x04必须为0。红0xD9、蓝0xD1 |
| A=P2/P3 | 剩余D，0..5000mm；有符号int16大端承载非负值 |
| B=P4/P5 | 场地平移H，-18000..18000厘度；有符号int16大端 |
| U=P6/P7 | 固定车头yaw，0..35999厘度；无符号uint16大端 |

只有step13接受任意合法U，原0..12的U/参数校验逐字保留。高四位不再被误用为旧聚堆/左右/STAGE标志。
原0x1E仍回本次task/action、accepted_opcode20及ACCEPTED/COMPLETED位；执行和D=0完成均mode25。

红方示例：task10/action100、SEQ40、D100mm、H90°、车头保持270°：

```text
a3 b3 1d 28 00 0a 00 64 00 00 00 00 c5 a1 c3
a3 b3 18 28 14 d9 00 64 23 28 69 78 92 7d c3
```

完整12组原始成对编码、字段及边界在[机器合同](contracts/f405_empty_body_move_20261114.json)。示例是离线编码，不是实收或地面执行记录。

## 入口、执行和交接

1. 首个13接受本任务已完成的RELEASE_BOTH/mode34且逻辑空爪，或按原step0方式从空爪SEARCH进入。
   仅首次release/SEARCH交接清旧审核/STABLE、stash_backoff_pending、route_to_stash及放弃批次预算；
   不重放松爪、不增加舵机动作，不改first_delivery_done或投送计数。原编号归属/回绕及新目标预算规则保持。
2. 后续13从本任务上一段13的完成状态接受新action；沿现有动作ID检查及FFFF→0/task+1编号回绕。
   SEARCH只能在空爪时以新action开启，前置cancelled-context例外同步支持step13；未完成释放、其他释放、持货和原mode51不放开。
   相同/过期action不会借新入口重启。不是新增时间/新鲜度资格。
3. 直接复用task_bulldozer_live_step和Motor_MoveAngleFineHeading；`body_angle=wrap(H-当前主yaw)`，U独立保向。
   当前请求速度仍250～350mm/s，原斜坡/换向/Fine补偿/航向PI保持，不先把车头转向H。
4. 同action的H/D实时更新，不重启、不改变锁定U；U变化沿原明确拒绝处理。D>0不报完成；D=0执行原刹车并COMPLETED。
   已完成动作的迟到正D/重复请求不会重新起动；只在新action建立下一段，后续段/心跳不重复清放货上下文。
5. 四段后RDK发原step12，按原普通开爪/正常高度/相机120°恢复SEARCH。mode3保留本次task/action/COMPLETED给上位消费。
   该标准SEARCH交接仍保留原清理；首次13的业务清理不是每段重新审核或重新记首绿。
6. 人工STOP/PAUSE/ABORT和mode51保持；无新ISR、定时器、时间门、反复握手、遥测、永久FAULT或测试程序。

## 必要检查和未验证项

- NormalRun/MotionDebug构建链接通过。复用原Task夹具直接调试调用红蓝四段、D/H刷新、D0完成、U固定/反向变换、
  重复/暂停继续、首绿不误记和step12回SEARCH；任意U359.99、首入口归属/完成/空载、原编号回绕符合预期。
- 12组成对报文CRC/字段检查通过。原FLOW_20261019_CHECK通过，差异核对确认原0..12校验及整个执行函数未改。
  未新增测试程序、未修改现有检查源码；宿主夹具结果不等于板上或地面精度证明。
- 本轮未烧录、未动作、actual未读；本机构建如实dirty1，保留原Task初始化空白。源码提交、构建及实烧身份分别记录。
- 扫障S上位已改为左59cm/后94cm：红(-0.56,+0.91)、蓝(+0.56,-0.91)m、各轴±3cm。
  仅同步文档/机器合同，固件不硬编码这个点，仍只消费H/D/U；原扫障基础报文示例同步上位当前几何。

旧固件即便已经支持0x14的0..12，也不能执行13。部署须烧新的NormalRun/TASK并读回真实身份，再重开RDK完整正式待机入口。
仅实现上一版mode34入口的step13固件也不支持这次SEARCH入口，需使用本次新NormalRun镜像；烧录由用户进行。
四段实际轨迹、轮滑及地面5cm精度仍待人安排验证。本轮不包含mode52取消或其他历史候选。
