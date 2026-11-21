# FinalRun2 新配套交付记录

2026-11-22，上海时间。**已构建、已离线联测；未部署上位、未烧录、未发车。** 原FinalRun源码与封存ELF未改，旧[配套锁](pairing.lock.json)不变。WWW本次只增加构建入口和说明，WWW旧固件路线无变化。

| 项目 | 实际记录 |
| --- | --- |
| 下位 | [Final_Run / FinalRun2-pose-delivery @860de9e](https://github.com/kkkkkkkkkkkkkkkk222222/Final_Run/tree/860de9e1f6dd59da19e80d723868e109d7ae61e9)，已推送；远端FinalRun2仍保留原385da2e预留快照 |
| 上位 | 本地finals-pose-delivery @46d9b0323105178071b2afc3a9bb93d94b14d856；正式仓库拒绝当前Git账号写入（403），未声称已发布到正式上位仓库 |
| 上位完整交付 | [单提交补丁](https://github.com/kkkkkkkkkkkkkkkk222222/Final_Run/blob/860de9e1f6dd59da19e80d723868e109d7ae61e9/docs/contracts/rdk_pose_delivery_46d9b03.patch)，已随下位推送；逆向应用检查及SHA-256已记录 |
| 构建 | FinalRun2，source dirty=0；Flash 186092 B，RAM 37592 B |
| firmware ID | `6cb2f0b71a51977b31a06325b329c73eadc8a72b4f69045563f3721df06c8080` |
| ELF SHA-256 | `92882885d921a2f7588e0e1d5064f49201c5003f7fc9d3a376b35c45feadbe67` |
| BIN SHA-256 | `a10ff9e55f6dad976d0ba60db2494af599ba1af40407e8b42c1ebdfd04e9906a` |
| actual | 上位实际部署、板端身份及实车结果均未知 |

完整文件哈希见[构建记录](finalrun2-build.json)。本机产物为 `WWW/build/F405/FinalRun2/FinalRun2.elf`，同目录HEX/BIN已生成；另存独立产物副本 `field_context_20261121/finalrun2_860de9e/`，与冻结回退包分开。重新加载WWW的CMake后选择 `FinalRun2 - FinalRun2` 和 `DAPLink FinalRun2`；独立打开FinalRun2也有同名预设与DAPLink配置。本次未执行下载配置。

修改点：取消门前视觉横移、定位横差10mm/航向1°放行；C/A/M/H独立分段与返程；ENTER按实际位置锁存前段/末段、83°等待；侧挤不重复抵扣；横扫只用W1；防守默认、主动搜索材料。下位修复新NAV残留DONE及同action迟到正距离重启动作。具体接口和测试边界见[下位验证记录](https://github.com/kkkkkkkkkkkkkkkk222222/Final_Run/blob/860de9e1f6dd59da19e80d723868e109d7ae61e9/docs/FINALRUN2_PAIR_VALIDATION.md)。

验证通过：14个真实距离边界报文、8条红蓝/两分区/名义与20mm补位投送、4条完整W1横扫；另有生产Task的8组推进、1°边界、83°等待、暂停与重复/旧动作、侧挤三段和旧接口兼容。上位实际发布、下位生产解析/任务处理及CRC回包参与联测，轮程、机构与画面是模拟输入，不能当作实车已走通。没有新时间资格、高频巡检或FAULT策略。

已确认的A点窗口角接触风险仍保留；真实定位、惯性、带货外廓、相机视场和实际投送效果待实车。两端由本聊天先后核对，不冒充独立互审。未经本次明确指令不烧录或发车。
