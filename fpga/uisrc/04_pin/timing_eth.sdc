# =============================================================================
#  B 板以太网顶层 时序约束（eth_top）
#    25 MHz 板载晶振 -> PLL -> 125 MHz（RGMII TX）
#    两颗 PHY 的 RXC 由 PHY 自己产生（1Gbps 时 125 MHz，100Mbps 时 25 MHz）
# =============================================================================

create_clock -name {sysclk} -period 40.000 -waveform {0.000 20.000} [get_ports {I_sys_clk}]

# PHY 送来的接收时钟：先按 1Gbps(125MHz, 8ns) 建约束，
# 实际若跑 100Mbps 时序只会更宽松。
create_clock -name {etha_rxc} -period 8.000 -waveform {0.000 4.000} [get_ports {I_etha_rxc}]
create_clock -name {ethb_rxc} -period 8.000 -waveform {0.000 4.000} [get_ports {I_ethb_rxc}]

derive_clocks

# 三个时钟域互不相关（晶振各自独立），声明为互斥组
set_clock_groups -name async_grp -exclusive \
    -group [get_clocks {sysclk}] \
    -group [get_clocks {etha_rxc}] \
    -group [get_clocks {ethb_rxc}]

# RGMII 发送数据相对 TXC 的 DDR 关系由 ODDR 保证，静态时序工具不建模，
# 与 A 板处理方式一致：加跨边沿伪路径，避免假违例。
set_false_path -fall_from [get_clocks {sysclk}] -rise_to [get_clocks {sysclk}]
set_false_path -rise_from [get_clocks {sysclk}] -fall_to [get_clocks {sysclk}]

# 异步复位释放不做时序要求
set_false_path -to [get_pins -hierarchical *u_phy_reset*]
