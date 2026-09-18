# b0f954f配套：READY、上下文配对、帧龄与卡死风险

基线F407 3319ab5；上位机 `codex/gamepad-teleop@b0f954f`。本次只修改F407仓库。
未增加LCD/调试输出或测试；完成Debug固件编译与静态路径检查，未烧录、未做故障注入或实车验证。

## 已实现的协议

| 方向 | 连续帧 | 内容 |
| --- | --- | --- |
| RDK→F407 | 0x1D→0x18，同SEQ | task_id(u16 BE)、action_id(u16 BE)、vision_frame(u32 BE)，随后原8字节动作字段 |
| F407→RDK | 0x1E→0x17，同SEQ | 已接受task/action、opcode、action_status(bit0接受/bit1完成)、两字节0，随后原状态 |

每帧仍15字节、原CRC16。0x19～0x1C运动调试类型不变。0x17 flags bit6=0x40为AUDIT_READY。
READY=0不允许根据单次ACK执行非法释放；READY=1/VALID=0表示收齐但非法，READY=1/VALID=1表示合法。
普通映射audit_received；43使用独立的扫障READY，三张空审核也可READY=1但VALID=0，非空三帧才VALID。

命令解析只有紧邻、同SEQ、两帧CRC/动作字段均合法才提交一个新命令快照。噪声、错SEQ、其他帧插入、CRC错误和串口重同步都清待配对状态。
赛前配置保持原单帧握手；裸0x18仅ABORT允许。ABORT单独锁存，避免被同一DMA批次后续普通命令覆盖。
状态对由一个快照构造，并作为一个30字节UART队列项发送；不能在1E/17间插入里程帧。

## 归属及编号

- 赛前配置确认后建立新会话。首次编号可随机；之后task/action按16位半区顺序比较，旧编号及相差恰好32768的歧义编号拒绝。
- 兼容上位机action=FFFF→0000时同时task+1的进位；这是编号变化，不重新初始化物资任务、审核或机械动作。
- 新目标只在允许切换目标的状态接管，不抢占正在合爪/放置/回退的有限动作。APP新像素、NAV新H/D、审核新ID保留本动作进度。
- 同一action改变普通opcode/flags拒绝；HOLD/PAUSE作为当前action覆盖，状态仍回显原动作opcode。恢复原opcode不会重新执行有限动作。
- 被拒绝的请求不替换已接受上下文或当前控制指令，不刷新last_command_rx_ms。配置会话、帧对有效、状态接受与动作完成是不同层次。
- mode41、聚集强制刷新、找回失败、进入SEARCH或STOPPED等取消路径禁止同任务旧APP/GRAB/NAV复活；允许同归属HOLD/PAUSE及合状态的RETURN恢复。新目标须新task_id。
- mode24的500 ms+恢复期间HOLD握手保留；普通非聚集的短暂目标恢复仍可继续原动作，真正回SEARCH后旧任务失效。

## 审核及年龄

- 仅新的非零vision_frame刷新视觉年龄；重复画面即使换SEQ或audit_id也不重复累计，倒退的视觉编号不能覆盖较新的目标。
- 保留三种独立语义：传输SEQ、物理action_id、真实vision_frame。APP的report_generation改用vision_frame，不能以心跳次数冒充新画面。
- 新任务、新审核action及视角切换清READY/VALID；138°/142°偏视不计140°审核，回140°需新的视觉帧。重复占位vision_frame=0可接受并ACK，但不能提供视觉证据。
- `APP_COMMAND_RX_TIMEOUT_MS=500`：连续APP/NAV/RETURN失联停车等待，保留阶段/预算。`APP_VISUAL_UPDATE_TIMEOUT_MS=500`：APP画面陈旧停车观察，通信心跳本身不能延长视觉寿命。
- HOLD覆盖前的APP像素单独保留，恢复旧opcode时不能把HOLD的零参数当作新的目标坐标。
- 已接受合爪、释放、有限距离ENTER、退出和扫障倒退不因短暂断流重启；PAUSE仍显式冻结。现有有限动作自己的完成/恢复规则不变。
- 未增加整场超时FAULT/ABORT。1秒投送视觉超时仍接受TASK_COMPLETE，计次/首件推进规则不变，不能把它称作视觉确认成功。

## 剩余上位机隐患：需要配套修正

### 高优先级：context_pending门可能挡住恢复分支

`competition_rescue/run_competition_rescue.py::_output_for_cycle`（b0f954f约1228行）在fresh但task/action不匹配时直接返回protocol_last_output，只豁免41和fault。
可发生：APP或审核期间下位机已自主进入24/3/46，上位机恰好发出新action的GRAB/APP等，该命令在新状态下被拒绝；下位机正确保留旧已接受上下文，上位机却始终重发被拒请求，永远进不了mission.step中的恢复分支。
**建议上位机**在上下文等待门之前处理新鲜的24、3、46及其它明确动作取消/恢复证据；取消pending请求，先用状态回执里的已接受task/action发HOLD或合状态RETURN，看到SEARCH后再分配新task。不能靠下位机伪造接受被拒请求解决。

### 高优先级：未接受请求的编号不能拿来PAUSE/HOLD当前动作

上位机发布新请求后立即更新protocol_task/action；若该请求还未接受就进入摄像头恢复或状态过期路径，它会用待接受编号发PAUSE。
下位机当前已接受action可能不同，严格归属检查必须拒绝这种覆盖。连续控制随后可因500 ms失联停车，但已接受有限机械动作仍按本地规则完成，不能保证该PAUSE立即冻结它。
**建议上位机**分开保存requested与accepted上下文；覆盖命令发送到已接受上下文。显式全局取消仍可用裸ABORT，而不是用未知编号抢占当前动作。

### 中优先级：状态解析器对字节噪声的紧邻检查不完整

`localization/src/f407_protocol.cpp::consume`在index=0丢弃非帧头字节、index=1帧头不匹配时，没有清have_status_context_；虽然坏CRC/完整错误帧/resynchronize会清除，但1E与17之间仅插入少量噪声时仍可能配上旧上下文。
F407发送端已原子连续发送30字节；建议C++解析器在任何跳过/异常帧头字节时清pending，并验证1E保留字节与action_status保留位。

### 行为边界：不是所有“停住/转圈”都是通信故障

- 通信或APP图像过期后，下位机故意停车保留阶段；只有新有效指令/真实新APP画面才恢复。一直重发同画面不会恢复，这是预期保护。
- SEARCH扫描和mode42无目标的局部搜索保留原逻辑；只要上位机仍发合法HOLD，仍可能持续搜索，不能承诺场景无目标时绝不转圈。
- 取消任务旧编号被拒是预期；上位机若不更换task_id而继续发旧APP，会等待，不应让下位机放宽编号检查。
- F407内部Location与T265估计仍可能不同；边界恢复、回退偏差与机械卡滞不由UART上下文自动修正。
- 1秒视觉超时结束投送是既定策略，仍可能物块未入区就推进首件状态，本次明确保持。

## 部署要求

必须同时更新/重新编译上位机Python任务程序、localization串口转发器及本固件；旧固件无1E，旧上位机发裸18，两种混用均会等待。
重新运行原赛前配置建立会话。运行中仅重启上位机并换随机编号，不是合法会话重建；应先显式取消并走原配置握手。
`tools/vision_protocol.py`增加command_context_frame、pair_mission_frame、parse_stm_status_pair和audit_ready解析。旧mission_frame仍只是单帧构造器，调试真实救援固件时需使用pair_mission_frame包装，裸ABORT例外。

## 检查结论

下位机已针对旧画面转圈、裸命令串用上下文、旧任务重放、未READY就释放、有限动作重复初始化补上保护；静态检查和编译不能证明异常通信下绝不卡死。
上述上位机等待门与覆盖命令归属冲突仍需上位机侧修复；本次未修改上位机仓库，不把这些风险标为已解决。
