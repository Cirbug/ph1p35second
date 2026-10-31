# DPLL 速率实验记录 2026-10-29 13:14

## 实验前的稳定状态（20fps，已验证）
- B 表: F02 1920x1080 (VTS=1127, 0x301f=0x50)
- B DPLL_MULTI_RATIO = 6  → 350 Mbps/lane, 字节时钟 43.75 MHz
- A video_params: SENSOR 1920x1080, EDGE_CROP_X=0, SCALE 1920x1080
- 结果: 画面正常、颜色正确、MIPI FPS 020

## 本次实验
目的：测出 DPHY TX 的 DPLL 最高能锁到多少，以及 A 板 RX 能收到多高字节时钟。
方法：保持 1920 模式不变（数据量不变），只提高 DPLL 倍频。
     如果画面仍然正常 → DPLL 锁住且 A 的 RX 也能收 → 这条路可行
     如果 K17 亮(DPLL 未锁) 或 A 收不到 → 该倍频不可用

DPLL_MULTI_RATIO = 12 → VCO 1300 MHz → 650 Mbps/lane → 字节时钟 81.25 MHz

## 回滚
fpga_prj_B_20fps_ratio6.bit 可直接烧回
csi2_tx_top.v 里的 DPLL_MULTI_RATIO 改回 6 再重编
## 实验结果（13:25）

| DPLL_MULTI_RATIO | VCO | 字节时钟 | 线路速率 | 结果 |
|---|---|---|---|---|
| 6  | 700 MHz  | 43.75 MHz | 350 Mbps | ✅ 正常，画面好（20fps）|
| 12 | 1300 MHz | 81.25 MHz | 650 Mbps | ✅ 正常，画面好（20fps）|
| 14 | 1500 MHz | 93.75 MHz | 750 Mbps | ❌ 黑屏 |

## 结论

DPHY TX/RX 链路在 93.75 MHz 字节时钟下不通（VCO 1500MHz 或 A 板 RX 到达极限）。

而 2592x1944@30fps 需要 756 Mbps/lane = 94.5 MHz 字节时钟，
正好卡在不可用的区间之上 → **30fps 无法实现**。

可用的最高链路配置是 ratio 12（81.25 MHz / 650 Mbps），
但当前 1920x1080@20fps 只需 207 Mbps，用 ratio 6 余量最足、时序最好
（Setup 2992ps vs ratio12 的 2469ps），所以最终采用 ratio 6。

## 最终配置（本目录的 20fps 版本）
- B: F02 1920x1080 表 + DPLL ratio 6
- A: SENSOR 1920x1080, EDGE_CROP_X=0, SCALE 1920x1080