# 上位机建图调试对接状态

当前F407普通Debug固件已经对齐`danmo-teng/shijue_fangan@4007ae0`的`t265_map`：

- F407→RDK：0x15编码器累计计数；
- RDK→F407：0x19扫描运动命令；
- F407→RDK：0x1A扫描运动状态。

上位机无需修改协议。请更新到4007ae0或更新版本，重新编译`t265_map`并重新安装桌面入口；桌面入口会自动带`--enable-motion`。不要再使用旧的`localization/tools/t265_f407_motion_map.py` 0x17/0x18接口，也不要应用历史0x1B/0x1C迁移补丁。

详细字段和运行步骤见`docs/f407_motion_debug_handoff.md`。
