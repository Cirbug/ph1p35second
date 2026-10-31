B 板以太网（PH1P35 + MLKPAI-F02 双千兆）工作说明
================================================================
分支：feature/eth-rgmii-bufg-fix
日期：2026-10-31

----------------------------------------------------------------
一、这个分支做了什么
----------------------------------------------------------------
在 B 板（MLKPAI-F02 底板 + FC02 核心板，PH1P35MDG324）上搭起
双千兆以太网的收发通路，用来做「A 板 → B 板 → 从板」的中继。

新增/修改的文件：
  fpga/uisrc/01_rtl/eth/eth_rx_probe.v   顶层（各阶段诊断固件都在这里迭代）
  fpga/uisrc/01_rtl/eth/rgmii_interface.v  RGMII 接口（关键修复见下）
  fpga/uisrc/01_rtl/eth/net_pll.v         25M -> 125M/125M_90/25M
  fpga/uisrc/01_rtl/eth/uiphyrst.v        PHY 复位时序
  fpga/uisrc/01_rtl/eth/uart_tx_simple.v  调试串口
  fpga/uisrc/01_rtl/eth/mdio_cfg.v        MDIO 广播写（PHY 寄存器配置）
  fpga/uisrc/01_rtl/eth/mdio_probe.v      MDIO 读回（诊断）
  fpga/uisrc/04_pin/fpga_pin_eth.adc      双网口引脚约束
  fpga/uisrc/04_pin/timing_eth.sdc        时序约束
  fpga/fpga_prj_eth.al / settings_eth.cfg / run_full_eth.tcl   以太网工程配置

----------------------------------------------------------------
二、【重要】接收位错误的根本原因与修复
----------------------------------------------------------------
症状：A 板发来的帧能收到，但字节有随机位错误
      （D5->F5、00->20、4E->6E），"CNN1" 识别率仅 0.7%。

原因：rgmii_interface.v 里 RGMII 接收时钟缓冲被写成了
        PH1P_PHY_SCLK_V2  (dlytrim=0)
      厂家源码注释原文：
        "使用以下代码可以实现通信，但是存在丢包"

修复：改回 PH1P_LOGIC_BUFG
        PH1P_LOGIC_BUFG u_rgmii_rxc_bufg (.o(rgmii_rxc_int), .i(rgmii_rxc));

效果：A 板 -> B 板 接收识别率 0.7% -> 100%（串口 A=P 恒等）

参考：厂家 demo D:\HX1P35A_Contest_202606\2_Example\33_ethernet_udp_led

----------------------------------------------------------------
三、已完成的验证
----------------------------------------------------------------
1. 工程可编译，时序收敛（Setup +1.7~2.2ns，Hold +0.25~0.57ns）
2. 双口环回（P1 与 P2 网线直连）：两向各 10 帧/秒互收
3. 电脑 -> B 板：电脑发 500 广播包，B 板 P2 收到 +664 帧
4. A 板 -> B 板：接收 100% 正确（179 块/帧，480x270 RGB565）

----------------------------------------------------------------
四、尚未解决（下次继续）
----------------------------------------------------------------
B 板 -> 外部（电脑 / 从板）：对端收不到任何字节。
本分支里已经排除的假设：
  · 帧内容错误（用 PC 预先算好的 64 字节 ROM 帧 + 正确 FCS，仍收不到）
  · 缺帧间隔 IFG（已加 10ms 间隔）
  · PHY 发送延时（EXT_0xA003[3:0] 0~15 档全扫）
  · TXC 相位（gmii_tx_clk_d 接 0deg 和 90deg 都试过）
  · 电脑网卡设置（2.5G 强制 / 节能 都已改）
结论倾向：FPGA -> PHY 的 RGMII 发送在物理层不正确，需要示波器
          量 ETHB 的 TXCK(U2) 与 TXD(P4/P5/T4/T3)、TX_CTL(P1)。

----------------------------------------------------------------
五、如何编译
----------------------------------------------------------------
  cd fpga
  copy settings_eth.cfg settings.cfg
  copy fpga_prj_eth.al fpga_prj.al
  td_commands_prompt.exe "<本目录>/run_full_eth.tcl"

（注意：settings.cfg / fpga_prj.al 是工程实际读取的文件名，
 本分支以 *_eth.* 形式保存，编译前改名即可，避免覆盖 CSI-2 工程配置）
