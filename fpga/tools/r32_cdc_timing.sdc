# Load after update_timing so the named clocks and post-RTL nets exist.
set rx_clock [get_clocks {MIPI_RX_BYTE_CLK}]
set tx_clock [get_clocks {MIPI_TX_BYTE_CLK}]
set ddr_usr_clock [get_clocks {u_ph1p35_324_ddr_wrapper/u_ddr2/usr_clk}]

set fp_wgray_nets [get_nets -nowarn {u_ddr_triple_frame_cache/u_frame_pack_fifo/wgray[*]}]
set fp_rgray_nets [get_nets -nowarn {u_ddr_triple_frame_cache/u_frame_pack_fifo/rgray[*]}]
set tx_wgray_nets [get_nets -nowarn {u_relay_tx_token_fifo/wgray[*]}]
set tx_rgray_nets [get_nets -nowarn {u_relay_tx_token_fifo/rgray[*]}]
set bank_free_nets [get_nets -nowarn {u_ddr_triple_frame_cache/bank_free[*]}]
set init_done_nets [get_nets -nowarn {ddr_init_done}]
set rx_error_nets [get_nets -nowarn {u_ddr_triple_frame_cache/rx_error_latched}]

puts "R32 clocks RX=[llength $rx_clock] TX=[llength $tx_clock] DDR_USR=[llength $ddr_usr_clock]"
puts "R32 CDC nets frame=[llength $fp_wgray_nets]/[llength $fp_rgray_nets] tx=[llength $tx_wgray_nets]/[llength $tx_rgray_nets] status=[llength $bank_free_nets]/[llength $init_done_nets]/[llength $rx_error_nets]"
if {[llength $rx_clock] != 1 || [llength $tx_clock] != 1 || [llength $ddr_usr_clock] != 1} {
    puts "R32 timing constraint error: expected RX/TX/DDR user clocks were not found"
    exit 1
}
if {[llength $fp_wgray_nets] != 7 || [llength $fp_rgray_nets] != 7 || [llength $tx_wgray_nets] != 13 || [llength $tx_rgray_nets] != 13 || [llength $bank_free_nets] != 3 || [llength $init_done_nets] == 0 || [llength $rx_error_nets] != 1} {
    puts "R32 CDC net verification failed; no place/route or bitgen"
    exit 1
}

# Bound Gray-pointer buses to one destination cycle; exclude only their
# asynchronous hold checks. The second synchronizer stage remains timed.
set_max_delay -datapath_only 7.500 -from $rx_clock -through $fp_wgray_nets -to $ddr_usr_clock
set_false_path -hold -from $rx_clock -through $fp_wgray_nets -to $ddr_usr_clock
set_max_delay -datapath_only 8.888 -from $ddr_usr_clock -through $fp_rgray_nets -to $rx_clock
set_false_path -hold -from $ddr_usr_clock -through $fp_rgray_nets -to $rx_clock
set_max_delay -datapath_only 13.332 -from $ddr_usr_clock -through $tx_wgray_nets -to $tx_clock
set_false_path -hold -from $ddr_usr_clock -through $tx_wgray_nets -to $tx_clock
set_max_delay -datapath_only 7.500 -from $tx_clock -through $tx_rgray_nets -to $ddr_usr_clock
set_false_path -hold -from $tx_clock -through $tx_rgray_nets -to $ddr_usr_clock

# Stable status levels also terminate at first-stage synchronizer flops.
set_max_delay -datapath_only 8.888 -from $ddr_usr_clock -through $bank_free_nets -to $rx_clock
set_false_path -hold -from $ddr_usr_clock -through $bank_free_nets -to $rx_clock
set_max_delay -datapath_only 8.888 -from $ddr_usr_clock -through $init_done_nets -to $rx_clock
set_false_path -hold -from $ddr_usr_clock -through $init_done_nets -to $rx_clock
set_max_delay -datapath_only 7.500 -from $rx_clock -through $rx_error_nets -to $ddr_usr_clock
set_false_path -hold -from $rx_clock -through $rx_error_nets -to $ddr_usr_clock
