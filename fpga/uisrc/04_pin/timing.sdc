create_clock -name {sysclk} -period 40.000 -waveform {0.000 20.000} [get_ports {I_sysclk}]
derive_clocks
rename_clock -name {clk0} [get_clocks {pll_inst/ph1p_phy_pll_wrapper_6ba8d3e44c24_Inst/u_PH1P_PHY_PLL.clkc[0]}]
