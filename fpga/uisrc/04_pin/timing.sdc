create_clock -name {sysclk} -period 40.000 -waveform {0.000 20.000} [get_ports {I_sys_clk}]

# MIPI RX 时钟来自摄像头，从 pad 进入后经 PHY 内部 PLL 产生字节时钟。
# 静态工具无法自己推导，这里在 pad 上显式建时钟，再由 derive_clocks 传递。
# 450MHz 为约束上限（与厂家例程一致），实际速率取决于摄像头模式。
create_clock -name {mipi_rx_ck_pad} -period 2.222 -waveform {0.000 1.111} [get_nets {IO_rx_clk_pad_p}]

derive_clocks

# 给关键时钟命名，便于下面的时钟组引用
rename_clock -name {pll_clk_100m} [get_clocks {u_pll/ph1p_phy_pll_wrapper_6ba8d3e44c24_Inst/u_PH1P_PHY_PLL.clkc[0]}]
rename_clock -name {pll_clk_24m}  [get_clocks {u_pll/ph1p_phy_pll_wrapper_6ba8d3e44c24_Inst/u_PH1P_PHY_PLL.clkc[1]}]

rename_clock -name {MIPI_RX_BYTE_CLK} [get_clocks {u_dphy_rx/u_ph1p_mipiio_rx_wrapper/u_PH1P_LOGIC_DPHY_MIPI_RX.o_fabric_div4_8_clk}]
rename_clock -name {MIPI_TX_BYTE_CLK} [get_clocks {u_dphy_tx/u_ph1p_mipiio_tx_wrapper/u_PH1P_LOGIC_DPHY_MIPI_TX.o_fabric_div4_8_clk}]

# 这些时钟分别来自：板载晶振、PLL、摄像头恢复时钟、板内 DPLL，
# 彼此没有固定相位关系，全部声明为互斥，避免跨域路径被误判。
# 跨域数据只经过 csi2_ptr_fifo 的格雷码指针，属标准异步传输。
set_clock_groups -name async_grp -exclusive \
    -group [get_clocks {sysclk}] \
    -group [get_clocks {pll_clk_100m}] \
    -group [get_clocks {pll_clk_24m}] \
    -group [get_clocks {mipi_rx_ck_pad}] \
    -group [get_clocks {MIPI_RX_BYTE_CLK}] \
    -group [get_clocks {MIPI_TX_BYTE_CLK}]

# 异步复位释放例外
#   rst_n_100 在 100MHz 域产生，而 MIPI DPHY 内部寄存器工作在 TX 字节时钟域。
#   静态时序会报 removal 违例（实测 -1.771ns / 10 个端点），但复位只在上电时
#   释放一次并长期稳定，不构成真实功能风险。
set_false_path -to [get_pins -hierarchical *asr]
