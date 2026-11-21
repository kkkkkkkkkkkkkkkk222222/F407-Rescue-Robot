# 200mm前段／只投材料：当前公开交付

2026-11-22（上海时间），用户已确认前段170→200mm，单伤员不投送并沿用原弃批动作。本轮未部署、未烧录、未发车。原FinalRun与冻结锁未改。

- **公开分支：[FinalRun2-material-push200](https://github.com/kkkkkkkkkkkkkkkk222222/Final_Run/tree/FinalRun2-material-push200)**，下位提交 `b2738ec39c06daaaf483e317ea60f00c9c490463`。
- **直接看上位源码：[upper_changes](https://github.com/kkkkkkkkkkkkkkkk222222/Final_Run/tree/b2738ec39c06daaaf483e317ea60f00c9c490463/upper_changes)**。其中完整保存相对上位28deaa6基线修改的11个文件，对应上位提交 `2fd77f918278b2e3814c1873b7fedb199a791cbd`；包含完整补丁与源码哈希，不包含模型、会话和个人地图。已逐文件验证交付镜像与上位Git内容一致。
- **距离**：从名义|y|=770mm出发，前段200，到970时等83°；末段270，名义终点1240，总470。实际起点/侧挤后按剩余距离锁存，重发不重算或重复抵扣。
- **伤员**：普通运输审核对真实单伤员报READY/INVALID；不删类别、不伪装空爪。材料混伤员保材料分离；只剩伤员按原张爪、退300mm及原转向/返搜。伤员识别、门口判障/W1扫障及开局临时移堆保持。

检查通过：14个距离边界报文；8条红蓝材料投送（名义/20mm补位、确认/未确认）；4条危险物/伤员挡路W1；4条抓取点/门前单伤员弃批；另有混装左右保材料、临时移堆、8组推进/83°等待/暂停重发及旧接口检查。真实上位构帧和下位生产解析、Task、CRC回包参与；位姿、轮程和画面是模拟输入，不等于实车走通。

| 构建项 | 值 |
| --- | --- |
| firmware ID | `1401da3dd06a75f529726e59cfef0bdf635b6fa45c7e644c2b18c279a9dd1e52` |
| ELF SHA-256 | `b3263b8deb3eaba7696cd9abca142b987ecec0602d4feabad91b52afdeb1cc1d` |
| Flash / RAM | 186076 / 37592 B |
| 源码状态 | `b2738ec` / clean |
| actual | 未读取；未烧录 |

[机器可读记录](material-push200-build.json)。本机固件在 `WWW/build/F405/FinalRun2/`，独立副本在 `field_context_20261121/finalrun2_b2738ec/`。DAPLink入口仍为FinalRun2。本机改前/改后图在 `local_route_review_20261121/push200_material_only.png`，是代码推算示意，未展开实车爪/载荷外廓。现场碰撞、制动及实际推距仍待实车验证。

[上一版170mm配套记录](FINALRUN2.md)及既有分支保留作回退，不以新开发记录覆盖第一场封存包。WWW本轮只更新交接资料，WWW旧固件路线无变化。
