# 项目最终状态存档 —— 2026-10-29

**目标**：B 板（MLKPAI-F02 / PH1P35MDG324）采集 SC500CS 摄像头 RAW10，
经 CSI-2 解析 → 行缓冲 → 重新打包 → 由 DPHY1 发出，通过 40P FPC 送 A 板（HX1P35A）J9，
A 板出 HDMI 图像。**架构必须是"解析+行缓冲+重打包"**（不能透明转发），
因为后续要在 B 板的行缓冲处插入 CNN 识别。

---

# 一、当前状态：**已达成，画面正常**

```
摄像头 1920×1080 @ 20fps (RAW10, 2 lane)
   ↓ MIPI CSI-2
B 板 DPHY0 接收 (43.75 MHz 字节时钟)
   ↓ csi2_rx_parse（抓包头/搬载荷/产出行边界）
   ↓ 令牌 FIFO（跨时钟域）
   ↓ csi2_tx_packet（行缓冲乒乓 + 重算 ECC/CRC + 打包 FS/长包/FE）
B 板 DPHY1 发送 (350 Mbps/lane, 2 lane)
   ↓ 40P FPC（1、2 脚剪断）
A 板 DPHY0 接收
   ↓ csi_unpacket_2lane → raw10_unpacket → ISP → 缩放(1:1) 
   ↓ DDR3 三缓冲（每 bank 3.05 MB，一帧 2.97 MB）
   ↓ HDMI 1920×1080 @ 30 Hz
```

**实测结果**：画面正常、颜色正确、MIPI FPS 020、HDMI FPS 030、B 板串口 `X=0`（无 lane 错误）。

---

# 二、两个板子烧哪个（最重要）

| 板 | 文件 | 说明 |
|---|---|---|
| **B 板** | `bitstreams\B_fpga_prj_20fps_final.bit` | 20fps 可用版，DPLL ratio 6 |
| **A 板** | `bitstreams\A_camera_to_dsi_display_20fps_可用版.bit` | 20fps 可用版 |

> ⚠️ `A_工程目录_当前_2592失败版.bit` 是失败的，**不要烧**。

**A 板工程目录里的位流可能还是失败的那版**，因为 A 的源码虽然已改回 1920 参数，
但工程目录里的 .bit 没有重新编译过。**直接烧存档里的可用版最保险。**

---

# 三、关键配置（改之前务必确认）

## B 板

| 项目 | 值 | 位置 |
|---|---|---|
| 寄存器表 | **F02 原始 1920×1080 表**（`0x301f=0x50`, HTS=2852, VTS=1127）| `uisrc/01_rtl/uics500_cfg/uics500reg.v` |
| DPLL_MULTI_RATIO | **6**（VCO 700MHz → 350 Mbps/lane → 字节时钟 43.75 MHz）| `uisrc/01_rtl/csi2_tx_top.v` |
| LANE_SWAP | **1**（两块 14→40 转接板 lane 相反）| 同上，`csi2_tx_packet` 例化 |
| LANES | 2 | 同上 |
| 行缓冲 | `BUF_AW=11`（每 bank 2048 字）| 同上 |

## A 板

| 项目 | 值 | 位置 |
|---|---|---|
| SENSOR_WIDTH/HEIGHT | **1920 / 1080** | `user_source/hdl_source/video_params.vh` |
| SENSOR_EDGE_CROP_X | **0** | 同上 |
| SCALE_WIDTH/HEIGHT | **1920 / 1080**（1:1 直通）| 同上 |
| PIPE/DISPLAY | 1920×1080 | 同上 |
| `S_ISP_O_tready` | `assign ... = 1'b1;`（原本悬空，是真 bug）| `design_top_wrapper.v` |
| 调试灯 | D10/E10/C6/D6 四颗 | `constraints_source/pin.adc` |

---

# 四、协议实测结论（这些数值是唯一标准，勿凭文档推断）

| 项目 | 实测值 | 证据 |
|---|---|---|
| 包头字节序 | **DI, WC_LSB, WC_MSB, ECC** | 摄像头 `H0=2BA8`（DI=2B, WC_LSB=A8）|
| ECC 输入字节序 | **{WC_MSB, WC_LSB, DI}** | 摄像头 `H1=0C3B`，ECC(0x0CA82B)=0x3B |
| CRC16 | CCITT-FALSE，只算载荷 | 与 A 板 `crc16_csi` 一致 |
| 帧标记 | 摄像头每帧发 **DT=0x00(FS) 和 DT=0x01(FE) 各一个** | B 串口 `D` 字段同时出现 `00` 和 `01` |
| 每行拍数 | 宽×10/32（RAW10 打包成 32 位字）| 2592 → 810，1920 → 600 |
| 拍内字节序 | 先到的字节在 `[15:8]` | A 板 `csi_unpacket_2lane` 注释与代码 |

**致命陷阱**：FS 包全是 0（`DI=0/WC=0/ECC=0`），
**任何字节序/位序错误在它身上都不显现** →
所以"帧起始能过、数据包全丢"这种现象，第一个要怀疑的永远是位序。

---

# 五、帧率上限的完整结论（30fps 为什么做不到）

## 实测链路速率上限

| DPLL_MULTI_RATIO | VCO | 字节时钟 | 线路速率 | 结果 |
|---|---|---|---|---|
| 6 | 700 MHz | 43.75 MHz | 350 Mbps | ✅ 正常 |
| **12** | 1300 MHz | **81.25 MHz** | 650 Mbps | ✅ **正常** |
| **14** | 1500 MHz | **93.75 MHz** | 750 Mbps | ❌ 黑屏 |

**上限在 81.25 ~ 93.75 MHz 之间。**

## 30fps 需要多少

```
2592×1944 @ 30fps, RAW10, 2 lane
  = 1.51 Gbit/s 总带宽
  = 756 Mbps/lane
  = 94.5 MHz 字节时钟        ← 正好在上限之上
```

## 其他已排除的可能

| 怀疑 | 结论 |
|---|---|
| VTS 卡住帧率 | 确实卡（1920 模式 VTS=1127 只剩 47 行消隐；2592 模式 VTS=2000）但即使放开 VTS 也需要更高时钟 |
| 摄像头 PLL 配置错 | ❌ 原厂 Motorola 序列就是这样配的（`0x301f=0x01` + `HTS=1600` 配套）|
| 协议/ECC/CRC 问题 | ❌ 早已修好，与本问题无关 |

## 唯一的突破口

**增加 lane 数**：4 lane 时每 lane 只需 378 Mbps（我们轻松支持）。
但 **40P 连接器上只有 D0/D1，没有 D2/D3**，需要换硬件。

---

# 六、整个过程解决的 8 个问题（复盘）

| # | 问题 | 怎么发现的 |
|---|---|---|
| 1 | 摄像头排线接反 | 无 I2C 应答、无数据 |
| 2 | CSI-2 WC 字节序（应 LSB 先发）| 对照厂商 `csi_unpacket` 源码 |
| 3 | 两板寄存器表不同（1920 vs 2592）| 比对哈希 + 解表 |
| 4 | **ECC 字节序**（应 {WC_MSB,WC_LSB,DI}）| Python 复算摄像头实测值；错误值 0x9D vs 正确 0x3B |
| 5 | **两块转接板 lane 相反** | 开 `LANE_SWAP=1` 后 A 板开始接受 |
| 6 | **A 板 `S_ISP_O_tready` 悬空** | 通读 ISP 链路，下游无背压，固定为 1 |
| 7 | **A 板没有缩放器？** 实际有 `rgb96_downscale_crop` | 我之前搜错了关键词，误判为无缩放 |
| 8 | 重建代码引入的 3 个 bug（多驱动/漏最后一拍/字节对反）| 靠 testbench 抓出来 |

## 两条最重要的教训

1. **先让回归测试全绿，再出流上板。** 第 8 条就是跳过测试直接上板造成的。
2. **以实测为准，不以文档/常识为准。** 所有协议类错误都源于"我以为标准是这样"。

---

# 七、仿真回归状态

11 个 testbench，**11/11 全绿**（`run_sim.bat` 一键跑）：

```
tb_csi2_ecc_crc      PASS    ECC/CRC 算术
tb_crc_par2          PASS    并行 CRC 等价性
tb_csi2_tx_packet    PASS    打包器逐字节
tb_csi2_rx_parse     PASS    解析器
tb_csi2_pipeline     PASS    端到端 rx_parse→tx_packet
tb_csi2_ptr_fifo     PASS    跨时钟域 FIFO
tb_csi2_tx_top       PASS    整顶层
tb_csi2_full_line    PASS    真实行宽 2410 字节
tb_csi2_bank_switch  PASS    发送中切 bank
tb_csi2_lane_swap    PASS    LANE_SWAP 语义
tb_cam_cfg_i2c       PASS    I2C 配置全序列
```

---

# 八、遗留问题 / 后续可做

| 项 | 说明 |
|---|---|
| **30fps** | 受物理层速率限制，需 4 lane（硬件不支持）或更高 DPHY 速率 |
| **A 板 4 颗调试灯** | 当前是"每帧拍数"计数器（正常显示 3），收尾时可以撤掉或改成状态灯 |
| **B 板曝光按键** | 上限写死 2246（按 VTS=1127 算），初值 3000 超过上限；增益键没接（顶层只有 2 个键）|
| **B 板串口调试** | 保留（不占资源，排查有用）|
| **交付包** | 桌面 `CSI2中继_20261029\` 是 08:59 的旧版，需重新打包 |

---

# 九、目录说明

```
_FINAL_20261029\
├─ README_最终状态.md          ← 本文件
├─ bitstreams\
│   ├─ B_fpga_prj_20fps_final.bit            ★ B 板要烧的
│   ├─ A_camera_to_dsi_display_20fps_可用版.bit ★ A 板要烧的
│   └─ A_工程目录_当前_2592失败版.bit          ✗ 不要烧
├─ B_src\                      B 板全部源码（uisrc + 工程文件 + run_sim.bat）
├─ A_src\                      A 板全部源码（user_source + 工程文件 + al_ip）
└─ experiments\
    ├─ 20fps_回滚说明.md
    ├─ DPLL速率实验记录.md
    └─ fpga_prj_B_20fps_ratio6/12_OK.bit
```

## 重建方法

**B 板**（命令行，约 1 分钟）：
```powershell
cd <本目录>\B_src
D:\TD\bin\td_commands_prompt.exe D:/TD/doc/scripts/DefaultFlow.tcl
```

**A 板**：用 TD 图形界面打开 `A_src\td_project\camera_to_dsi_display.al`

⚠️ **A 工程所有源文件是 UTF-8**，改动时必须用 UTF-8 读写，
用 GBK 会破坏 `/* */` 注释导致语法错乱（踩过这个坑，浪费了 2 小时）。

---

# 十、git 状态

A 板工程：
- 分支 `jop6` = 你原本的工程（jop6.1~jop6.6），已推送 `origin/jop6` ✓
- 分支 `jop7.0` = jop6 + 我的改动（调试灯 + tready 修复 + 1920 参数）
