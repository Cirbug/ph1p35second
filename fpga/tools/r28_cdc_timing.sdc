# This SDC is loaded after optimize_rtl, when get_regs can resolve RTL
# register names. Fail early if the hierarchy or FIFO widths change.
set fp_wgray [get_regs -nowarn {u_ddr_triple_frame_cache/u_frame_pack_fifo/wgray[*]}]
set fp_rq1_wgray [get_regs -nowarn {u_ddr_triple_frame_cache/u_frame_pack_fifo/rq1_wgray[*]}]
set fp_rgray [get_regs -nowarn {u_ddr_triple_frame_cache/u_frame_pack_fifo/rgray[*]}]
set fp_wq1_rgray [get_regs -nowarn {u_ddr_triple_frame_cache/u_frame_pack_fifo/wq1_rgray[*]}]
if {[llength $fp_wgray] != 7 || [llength $fp_rq1_wgray] != 7 || [llength $fp_rgray] != 7 || [llength $fp_wq1_rgray] != 7} {
    puts "R28 CDC constraint error: frame FIFO Gray-pointer collection mismatch"
    exit 1
}

set tx_wgray [get_regs -nowarn {u_relay_tx_token_fifo/wgray[*]}]
set tx_rq1_wgray [get_regs -nowarn {u_relay_tx_token_fifo/rq1_wgray[*]}]
set tx_rgray [get_regs -nowarn {u_relay_tx_token_fifo/rgray[*]}]
set tx_wq1_rgray [get_regs -nowarn {u_relay_tx_token_fifo/wq1_rgray[*]}]
if {[llength $tx_wgray] != 13 || [llength $tx_rq1_wgray] != 13 || [llength $tx_rgray] != 13 || [llength $tx_wq1_rgray] != 13} {
    puts "R28 CDC constraint error: TX token FIFO Gray-pointer collection mismatch"
    exit 1
}

set bank_free_meta [get_regs -nowarn {u_ddr_triple_frame_cache/bank_free_rx_meta[*]}]
set init_meta [get_regs -nowarn {u_ddr_triple_frame_cache/init_rx_meta}]
set rx_error_meta [get_regs -nowarn {u_ddr_triple_frame_cache/rx_error_ddr_meta}]
if {[llength $bank_free_meta] != 3 || [llength $init_meta] != 1 || [llength $rx_error_meta] != 1} {
    puts "R28 CDC constraint error: status synchronizer collection mismatch"
    exit 1
}

# DDR frame-pack FIFO: MIPI RX byte clock <-> DDR user clock.
set_max_delay -from $fp_wgray -to $fp_rq1_wgray 7.500 -nowarn -datapath_only
set_false_path -hold -from $fp_wgray -to $fp_rq1_wgray
set_max_delay -from $fp_rgray -to $fp_wq1_rgray 7.500 -nowarn -datapath_only
set_false_path -hold -from $fp_rgray -to $fp_wq1_rgray

# DDR relay-token FIFO: DDR user clock <-> MIPI TX byte clock.
set_max_delay -from $tx_wgray -to $tx_rq1_wgray 7.500 -nowarn -datapath_only
set_false_path -hold -from $tx_wgray -to $tx_rq1_wgray
set_max_delay -from $tx_rgray -to $tx_wq1_rgray 7.500 -nowarn -datapath_only
set_false_path -hold -from $tx_rgray -to $tx_wq1_rgray

# Stable DDR status levels are sampled through two RX-domain flops.
set_max_delay -datapath_only 7.500 -to $bank_free_meta
set_false_path -hold -to $bank_free_meta
set_max_delay -datapath_only 7.500 -to $init_meta
set_false_path -hold -to $init_meta

# RX error level crosses back to the DDR user clock through a two-flop sync.
set_max_delay -datapath_only 7.500 -to $rx_error_meta
set_false_path -hold -to $rx_error_meta
