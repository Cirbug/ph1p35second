# 备份：为"中继 HTS 被 A 覆盖"问题做修复前快照
时间: 2026-10-30 17:42:39
原因: A 板转发的 111 条初始化表把 B 的 HTS(0x320c/0x320d) 覆盖成 1600，
      帧率从 22.5fps 变成 33.7fps，载荷 567->848 Mbps/lane，
      B 的 900Mbps TX 利用率 94%，随机溢出 -> 图像错乱。
改动: cam_i2c_bridge.v 增加 FILTER_HTS，和已有的 FILTER_VTS 同样处理。
      csi2_tx_top.v 未改动（仅备份）。
A 板: jop6 分支，未做任何修改。
