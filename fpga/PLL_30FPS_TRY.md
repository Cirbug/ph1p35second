# PLL 倍频试验（2026-10-30 04:10）

## 目标

不裁剪画面，靠提高传感器像素时钟（PCLK）把帧率从 20 提到 30。

## 本次改动（两处）

```
fpga/uisrc/01_rtl/uics500_cfg/uics500reg.v
  4:  0x36ea   0x3c -> 0x5a      (疑似 PLL 倍频系数 60 -> 90，×1.5)
  29: 0x320c   0x05 -> 0x0b      (HTS 1507 -> 2852，恢复满视野)
  30: 0x320d   0xE3 -> 0x24
```

其余不动：`0x36E9/36F9=0x04`、`0x36EC=0x0b`、`0x36ED=0x26`、`0x36FD=0x14`、
`0x320e/0f=0x0467`(VTS 1127)、输出 1920×1080。

## 依据

- 现测得 20fps（HTS 2852）⇒ PCLK ≈ 64.3 MHz
- 若 `PCLK = 24MHz × 0x36EA / 0x36EC / 2`，则 24×60/11/2 = 65.5 MHz，与实测吻合
- 30fps 需要 ≈96.4 MHz，即 0x36EA ≈ 88~90 → 取 0x5A(90)

## 预期与判读

| 观察 | 含义 | 下一步 |
|---|---|---|
| 满视野、画面正常、`MIPI FPS` ≈ 030 | 成功，0x36EA 就是倍频 | 收工，按需微调 |
| 仍 020，画面正常 | 0x36EA 不是倍频 | 改试 `0x36EC` 0x0b→0x07（÷11→÷7，×1.57） |
| 变成 025 / 035 等中间值 | 是倍频但比例不是线性 | `新值 = 旧值 × 30 ÷ 实测fps` 重算 |
| 花屏 / 黑屏 / 竖条纹 | 该寄存器不是倍频，是别的模拟参数 | 立即回退 |

## 回退

```powershell
cd C:\Users\21397\Desktop\ph1p35second\fpga\uisrc\01_rtl\uics500_cfg
# 回到"30fps + 裁剪 1.28×"那一版
Copy-Item uics500reg.v.bak_before_pll90 uics500reg.v -Force
# 或回到"满视野 + 20fps"最初那版
Copy-Item uics500reg.v.bak_hts2852 uics500reg.v -Force
```

## 注意

- PCLK 提高后传感器 MIPI 速率可能同步提高，B 的 RX 是 CDR 恢复，自适应，不用改；
  B 的 TX 仍是 68.75 MHz(550Mbps)，30fps 只占 57%，够用。
- 烧录产物看 `fpga\fpga_prj_Runs\best_result\fpga_prj.bit` 的时间戳，必须是本次编译之后。
