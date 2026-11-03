# 当前状态：F405 短暂恢复误停修复

2026-11-04。仅按用户本轮要求及上位机 `582a1f6` 的 CHANGE_WORKFLOW、
F405_REPAIR_HANDOFF_20261104 核对，历史阶段要求不自动继承。

## 核对与范围

- 起始下位机 HEAD/远端 main 均为 `11eeacfda71f419834ff78401faf1a0d35a3831e`。
- 既有 IDE 修改（含已暂存的两项 IDE 文件）、tmp/、Task.c 两行空白均保留，不夹入代码提交。
- 既有 NormalDebugTimed 构建嵌入 Git=11eeacf，source_dirty=1，firmware_id=
  `8b863b182c04e8ca1e43977f99883440febdb1e3855c107650c5b919c9317213`。
- 交接报告板端 actual=11eeacf/dirty1/F405/MOTIONDEBUG/230400；这是上位机报告，
  不当作本轮亲自读回。expected 没有被用来替代 actual。
- 代码已证：IMU_Update 在临界区外取 now，ISR 更新 angle_ms/rate_ms 后可产生
  无符号年龄下溢；ready 清除通过 Location、Debug、motor/Task 引起动作结束。
- 真实 UART 错误是否同时发生仍不确定。

## 本轮方向

修共享 UART IMU 读取竞态及短暂恢复动作连续性；精简额外时间资格、逐沿/逐字节
附加计时及高频诊断。保留真实计数、速度计算区间、主 yaw、动作身份、F405 运动学、
人工 STOP、mode51、单一 DMA 发送者和稳定缓冲区。不改变 PID/速度和比赛业务流程。
持续无反馈/显式UART错误控制策略等待本轮用户选择，不擅自添加阈值。
此次先缩小到已证实的年龄下溢和时间资格误停：普通时间间隔不再清ready，
但显式ORE/FE/NE/PE仍按旧路径清ready，控制仍可能结束动作。未宣称此项已闭环。

## 本轮实际结果

- 规则提交：`ca87bfa956a6a6bc1126d65738fbeff4eb1545b5`。
- 实现提交：`c2deed235123d40c1be761b58bf2d7cb9c3f820a`。
- 本地交付构建：NormalDebugTimed，MODE5/MOTIONDEBUG，UART4=230400，协议1.2，
  嵌入Git=c2deed2，source_dirty=1，firmware_id=
  `97e107eecccbc14f5b7f18527bfe94788e40cc900a00918be3dbebd893d0f95b`。
  ELF SHA256=`d064eabc1b21e8706d4a4f782086f05912bda08464fd5a2ac54de474b13fb7ea`。
  dirty来自用户既有Task.c的1547行后一个空行与一个两空格行；没有功能差异。
  后续文档提交不等于该构建版本，重新编译会更新嵌入Git和firmware_id。
- 变更说明及接口语义增量：[F405_REPAIR_20261104](F405_REPAIR_20261104.md)。
- NormalDebugTimed（230400）和NormalDebugTask（115200）均编译通过；未新增比赛07/08。
- 直接相关的现有UART驱动、遥测模块、32帧DMA队列主机检查通过；无新增测试框架。
  本机MSYS gcc未成功生成新程序，改用CLion随附MinGW编译后执行，旧exe输出不作本轮证据。
- 已审核diff：没有新增时间门控、ISR、巡检、永久FAULT或ESCAPE；PID/速度/Task业务没有改动。
- 未烧录、未复位、未发送运动；COM18只读尝试无回执且调用阻塞，已终止关闭，不能确认当前实烧身份。

## TODO（另定范围）

- 正式比赛最小反馈接入：NormalDebugTask 尚未启用 07/08；默认115200，
  07/08 的50/100Hz需要14250B/s，已超过11520B/s。建议后续复用原UART4队列，
  共同选230400或降低频率，保留0x16、0x18+0x1D、0x17+0x1E配对，不启用运动测试任务。
- 上位机收到修复后审查简化后的有效位语义，不再把未知时间界当本轮前置资格。
- 本轮不自动烧录或运动；短实测需当前授权。DONE仍不是地面距离/角度真值。
