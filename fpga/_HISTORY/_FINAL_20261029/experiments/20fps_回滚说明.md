# 20fps 可用配置存档（已验证画面正常、颜色正确）

时间：2026-10-29 12:15:19

## B 板
- 寄存器表: uics500reg.v / uics500regAE.v （F02 原始 1920x1080 表, VTS=1127）
- DPLL_MULTI_RATIO = 6  （350 Mbps/lane, 字节时钟 43.75 MHz）
- 位流: fpga_prj_B_20fps.bit

## A 板
- video_params.vh: SENSOR 1920x1080, SENSOR_EDGE_CROP_X=0,
                   SCALE_WIDTH/HEIGHT 1920x1080, PIPE/DISPLAY 1920x1080
- 位流: camera_to_dsi_display_A_20fps.bit

## 实测结果
- 画面正常，颜色正确
- MIPI FPS 020, HDMI FPS 030
- B 板串口: W=0960 (1920px), A=0438 (1080 行), X=0

## 回滚方法
1. 把本目录的 uics500reg.v / uics500regAE.v 复制回 uisrc/01_rtl/uics500_cfg/
2. 把 video_params_20fps.vh 复制回 A 工程 user_source/hdl_source/video_params.vh
3. B 板 DPLL_MULTI_RATIO 改回 6
4. 两边重新编译
---

# 附录：30fps 尝试结论（2026-10-29 12:50）

## 结论：这颗传感器在这两个模式下，帧率上限约 20fps

| 模式 | VTS | 有效行 | 实测帧率 | 30fps 需要 VTS | 可行性 |
|---|---|---|---|---|---|
| 1920x1080 (F02表) | 1127 | 1080 | **20.0 fps** | 751 | ✗ 751 < 1080 |
| 2592x1944 (A工程表) | 2000 | 1944 | **14.5 fps** | 967 | ✗ 967 < 1944 |

帧率 = 像素时钟 / (HTS x VTS)。VTS 不能小于有效行数，而这两张表里
根本没有 HTS(0x380c/0x380d) 和 PLL(0x0300~0x0305) 的配置项，
无法通过改 VTS 提高帧率。

## 提高发送速率（750 Mbps）的实验也失败了

试过 B 板 DPLL_MULTI_RATIO = 14（VCO 1500MHz -> 750 Mbps/lane -> 字节时钟 93.75MHz），
配 2592x1944@14.5fps：

- B 板串口一切正常：W=0CA8 (2592px), A=0797 (1943行), X=0（无 lane 错误）
- **A 板 4 颗灯全灭 = 收到 0 个 ISP 拍**
- 结论：A 板的 DPHY RX 在 93.75 MHz 字节时钟下锁不住

注：早期也试过 800 Mbps，同样 A 板收不到。这与协议无关（ECC/CRC/lane 都已修好），
是 DPHY RX 的锁定范围问题。A 板 RX 在 43.75 MHz（350 Mbps）下稳定工作。

## 后续要提高帧率的唯一途径

向供板方索取 **1080p30 或更高帧率的寄存器表**（内含正确的 HTS/PLL 配置），
而不是我们盲改 PLL。拿到表后换进 uics500reg.v 即可。