# TD 6.2.168116 documented async-FIFO constraints.
# Keep the RX/TX byte clocks asynchronous to the DDR user clock, but bound
# every crossing path and tighten Gray-pointer paths to one destination cycle.
set rx_clock [get_clocks {MIPI_RX_BYTE_CLK}]
set tx_clock [get_clocks {MIPI_TX_BYTE_CLK}]
set ddr_usr_clock [get_clocks {*u_ph1p35_324_ddr_wrapper/u_ddr2/usr_clk}]

puts "R31 clock targets RX=[llength $rx_clock] TX=[llength $tx_clock] DDR_USR=[llength $ddr_usr_clock]"
if {[llength $rx_clock] != 1 || [llength $tx_clock] != 1 || [llength $ddr_usr_clock] != 1} {
    puts "R31 timing constraint error: expected RX/TX/DDR user clocks were not found"
    exit 1
}

# The loose domain-level limits cover non-pointer status crossings.
set_max_delay -from $rx_clock -to $ddr_usr_clock -datapath_only 100.000
set_max_delay -from $ddr_usr_clock -to $rx_clock -datapath_only 100.000
set_max_delay -from $tx_clock -to $ddr_usr_clock -datapath_only 100.000
set_max_delay -from $ddr_usr_clock -to $tx_clock -datapath_only 100.000

# Gray-pointer buses need to arrive within one destination clock period.
set fp_wgray [get_regs -nowarn {u_ddr_triple_frame_cache/u_frame_pack_fifo/wgray[*]}]
set fp_rq1_wgray [get_regs -nowarn {u_ddr_triple_frame_cache/u_frame_pack_fifo/rq1_wgray[*]}]
set fp_rgray [get_regs -nowarn {u_ddr_triple_frame_cache/u_frame_pack_fifo/rgray[*]}]
set fp_wq1_rgray [get_regs -nowarn {u_ddr_triple_frame_cache/u_frame_pack_fifo/wq1_rgray[*]}]
set tx_wgray [get_regs -nowarn {u_relay_tx_token_fifo/wgray[*]}]
set tx_rq1_wgray [get_regs -nowarn {u_relay_tx_token_fifo/rq1_wgray[*]}]
set tx_rgray [get_regs -nowarn {u_relay_tx_token_fifo/rgray[*]}]
set tx_wq1_rgray [get_regs -nowarn {u_relay_tx_token_fifo/wq1_rgray[*]}]
set bank_free_meta [get_regs -nowarn {u_ddr_triple_frame_cache/bank_free_rx_meta[*]}]
set init_meta [get_regs -nowarn {u_ddr_triple_frame_cache/init_rx_meta}]
set rx_error_meta [get_regs -nowarn {u_ddr_triple_frame_cache/rx_error_ddr_meta}]

puts "R31 CDC targets frame=[llength $fp_wgray]/[llength $fp_rq1_wgray]/[llength $fp_rgray]/[llength $fp_wq1_rgray] tx=[llength $tx_wgray]/[llength $tx_rq1_wgray]/[llength $tx_rgray]/[llength $tx_wq1_rgray] status=[llength $bank_free_meta]/[llength $init_meta]/[llength $rx_error_meta]"
if {[llength $fp_wgray] != 7 || [llength $fp_rq1_wgray] != 7 || [llength $fp_rgray] != 7 || [llength $fp_wq1_rgray] != 7 || [llength $tx_wgray] != 13 || [llength $tx_rq1_wgray] != 13 || [llength $tx_rgray] != 13 || [llength $tx_wq1_rgray] != 13 || [llength $bank_free_meta] != 3 || [llength $init_meta] != 1 || [llength $rx_error_meta] != 1} {
    puts "R31 CDC target verification failed; no place/route or bitgen"
    exit 1
}

# RX writes the frame FIFO and TX reads the relay FIFO in the opposite direction.
set_max_delay -from $fp_wgray -to $fp_rq1_wgray -datapath_only 7.500
set_max_delay -from $fp_rgray -to $fp_wq1_rgray -datapath_only 8.888
set_max_delay -from $tx_wgray -to $tx_rq1_wgray -datapath_only 7.500
set_max_delay -from $tx_rgray -to $tx_wq1_rgray -datapath_only 13.332

# Status levels are synchronized in the destination domain; keep their first
# stage bounded while the second stage provides metastability settling time.
set_max_delay -from $ddr_usr_clock -to $bank_free_meta -datapath_only 8.888
set_max_delay -from $ddr_usr_clock -to $init_meta -datapath_only 8.888
set_max_delay -from $rx_clock -to $rx_error_meta -datapath_only 7.500
