# R27 CDC constraints for the two asynchronous FIFOs and DDR status crossings.
# Keep the exceptions at the first synchronizer stage; do not cut whole clocks.

# Gray pointer buses must arrive within the faster participating clock period.
set_max_delay -from [get_regs -nowarn {u_ddr_triple_frame_cache/u_frame_pack_fifo/wgray_reg[*]}] -to [get_regs -nowarn {u_ddr_triple_frame_cache/u_frame_pack_fifo/rq1_wgray_reg[*]}] 7.500 -nowarn -datapath_only
set_false_path -hold -from [get_regs -nowarn {u_ddr_triple_frame_cache/u_frame_pack_fifo/wgray_reg[*]}] -to [get_regs -nowarn {u_ddr_triple_frame_cache/u_frame_pack_fifo/rq1_wgray_reg[*]}]

set_max_delay -from [get_regs -nowarn {u_ddr_triple_frame_cache/u_frame_pack_fifo/rgray_reg[*]}] -to [get_regs -nowarn {u_ddr_triple_frame_cache/u_frame_pack_fifo/wq1_rgray_reg[*]}] 7.500 -nowarn -datapath_only
set_false_path -hold -from [get_regs -nowarn {u_ddr_triple_frame_cache/u_frame_pack_fifo/rgray_reg[*]}] -to [get_regs -nowarn {u_ddr_triple_frame_cache/u_frame_pack_fifo/wq1_rgray_reg[*]}]

set_max_delay -from [get_regs -nowarn {u_relay_tx_token_fifo/wgray_reg[*]}] -to [get_regs -nowarn {u_relay_tx_token_fifo/rq1_wgray_reg[*]}] 7.500 -nowarn -datapath_only
set_false_path -hold -from [get_regs -nowarn {u_relay_tx_token_fifo/wgray_reg[*]}] -to [get_regs -nowarn {u_relay_tx_token_fifo/rq1_wgray_reg[*]}]

set_max_delay -from [get_regs -nowarn {u_relay_tx_token_fifo/rgray_reg[*]}] -to [get_regs -nowarn {u_relay_tx_token_fifo/wq1_rgray_reg[*]}] 7.500 -nowarn -datapath_only
set_false_path -hold -from [get_regs -nowarn {u_relay_tx_token_fifo/rgray_reg[*]}] -to [get_regs -nowarn {u_relay_tx_token_fifo/wq1_rgray_reg[*]}]

# Stable level/status crossings are sampled through two flops and consumed later.
set_max_delay -datapath_only 7.500 -to [get_regs -nowarn {u_ddr_triple_frame_cache/bank_free_rx_meta_reg[*] u_ddr_triple_frame_cache/init_rx_meta_reg}]
set_false_path -hold -to [get_regs -nowarn {u_ddr_triple_frame_cache/bank_free_rx_meta_reg[*] u_ddr_triple_frame_cache/init_rx_meta_reg}]

set_max_delay -datapath_only 7.500 -to [get_regs -nowarn {u_ddr_triple_frame_cache/rx_error_ddr_meta_reg}]
set_false_path -hold -to [get_regs -nowarn {u_ddr_triple_frame_cache/rx_error_ddr_meta_reg}]
