# Gray-pointer CDC paths: preserve a datapath-only bound and exclude hold.
set fp_wgray [get_regs -nowarn {u_ddr_triple_frame_cache/u_frame_pack_fifo/wgray[*]}]
set fp_rq1_wgray [get_regs -nowarn {u_ddr_triple_frame_cache/u_frame_pack_fifo/rq1_wgray[*]}]
set fp_rgray [get_regs -nowarn {u_ddr_triple_frame_cache/u_frame_pack_fifo/rgray[*]}]
set fp_wq1_rgray [get_regs -nowarn {u_ddr_triple_frame_cache/u_frame_pack_fifo/wq1_rgray[*]}]
set_max_delay -from $fp_wgray -to $fp_rq1_wgray 7.500 -nowarn -datapath_only
set_false_path -hold -from $fp_wgray -to $fp_rq1_wgray
set_max_delay -from $fp_rgray -to $fp_wq1_rgray 7.500 -nowarn -datapath_only
set_false_path -hold -from $fp_rgray -to $fp_wq1_rgray

set tx_wgray [get_regs -nowarn {u_relay_tx_token_fifo/wgray[*]}]
set tx_rq1_wgray [get_regs -nowarn {u_relay_tx_token_fifo/rq1_wgray[*]}]
set tx_rgray [get_regs -nowarn {u_relay_tx_token_fifo/rgray[*]}]
set tx_wq1_rgray [get_regs -nowarn {u_relay_tx_token_fifo/wq1_rgray[*]}]
set_max_delay -from $tx_wgray -to $tx_rq1_wgray 7.500 -nowarn -datapath_only
set_false_path -hold -from $tx_wgray -to $tx_rq1_wgray
set_max_delay -from $tx_rgray -to $tx_wq1_rgray 7.500 -nowarn -datapath_only
set_false_path -hold -from $tx_rgray -to $tx_wq1_rgray

# Status level CDC paths terminate only at the first synchronizer stage.
set bank_free_meta [get_regs -nowarn {u_ddr_triple_frame_cache/bank_free_rx_meta[*]}]
set init_meta [get_regs -nowarn {u_ddr_triple_frame_cache/init_rx_meta}]
set rx_error_meta [get_regs -nowarn {u_ddr_triple_frame_cache/rx_error_ddr_meta}]
set_max_delay -datapath_only 7.500 -to $bank_free_meta
set_false_path -hold -to $bank_free_meta
set_max_delay -datapath_only 7.500 -to $init_meta
set_false_path -hold -to $init_meta
set_max_delay -datapath_only 7.500 -to $rx_error_meta
set_false_path -hold -to $rx_error_meta
