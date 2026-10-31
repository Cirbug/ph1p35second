set root D:/2026fpga/ph1p35second_r2_jop13/fpga
set out D:/2026fpga/bank/ph1p35_csi_cnn_20261031_r33_cdc_timing_retry
file mkdir $out

cd $root
open_project $root/fpga_prj.al -single_run
import_device ph1_35p.db -package PH1P35MDG324
elaborate -top csi2_tx_top
read_adc $root/uisrc/04_pin/fpga_pin.adc
read_ip_adc -ip ddr2 -file $root/uisrc/03_ip/ddr2/src/adc/ddr2.adc
read_sdc $root/uisrc/04_pin/timing.sdc
read_sdc -ip ddr2 $root/uisrc/03_ip/ddr2/src/sdc/ddr2.sdc

cd $out
optimize_rtl
update_timing

# TD defers some commands in a separately read SDC. Keep these commands in
# the main flow after optimize_rtl so the post-RTL clock and register objects
# are resolved in the same command context as the implementation steps.
set rx_clock [get_clocks -nowarn {MIPI_RX_BYTE_CLK}]
set tx_clock [get_clocks -nowarn {MIPI_TX_BYTE_CLK}]
set ddr_usr_clock [get_clocks -nowarn {*u_ph1p35_324_ddr_wrapper/u_ddr2/usr_clk}]

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

puts "R33 clocks RX=[llength $rx_clock] TX=[llength $tx_clock] DDR_USR=[llength $ddr_usr_clock]"
puts "R33 CDC targets frame=[llength $fp_wgray]/[llength $fp_rq1_wgray]/[llength $fp_rgray]/[llength $fp_wq1_rgray] tx=[llength $tx_wgray]/[llength $tx_rq1_wgray]/[llength $tx_rgray]/[llength $tx_wq1_rgray] status=[llength $bank_free_meta]/[llength $init_meta]/[llength $rx_error_meta]"
if {[llength $rx_clock] != 1 || [llength $tx_clock] != 1 || [llength $ddr_usr_clock] != 1} {
    puts "R33 timing constraint error: expected RX/TX/DDR user clocks were not found"
    exit 1
}
if {[llength $fp_wgray] != 7 || [llength $fp_rq1_wgray] != 7 || [llength $fp_rgray] != 7 || [llength $fp_wq1_rgray] != 7 || [llength $tx_wgray] != 13 || [llength $tx_rq1_wgray] != 13 || [llength $tx_rgray] != 13 || [llength $tx_wq1_rgray] != 13 || [llength $bank_free_meta] != 3 || [llength $init_meta] != 1 || [llength $rx_error_meta] != 1} {
    puts "R33 CDC target verification failed; no place/route or bitgen"
    exit 1
}

# Async FIFO pointer buses: constrain the physical datapath to one receiver
# cycle, and exclude only the hold check. The second synchronizer stage stays
# visible to STA through the normal destination-domain timing.
set_max_delay -from $fp_wgray -to $fp_rq1_wgray -datapath_only 7.500
set_false_path -hold -from $fp_wgray -to $fp_rq1_wgray
set_max_delay -from $fp_rgray -to $fp_wq1_rgray -datapath_only 8.888
set_false_path -hold -from $fp_rgray -to $fp_wq1_rgray
set_max_delay -from $tx_wgray -to $tx_rq1_wgray -datapath_only 13.332
set_false_path -hold -from $tx_wgray -to $tx_rq1_wgray
set_max_delay -from $tx_rgray -to $tx_wq1_rgray -datapath_only 7.500
set_false_path -hold -from $tx_rgray -to $tx_wq1_rgray

# Level synchronizers: the first stage is asynchronous by construction. Keep
# a bounded data path and ignore only its meaningless hold relationship.
set_max_delay -datapath_only 8.888 -to $bank_free_meta
set_false_path -hold -to $bank_free_meta
set_max_delay -datapath_only 8.888 -to $init_meta
set_false_path -hold -to $init_meta
set_max_delay -datapath_only 7.500 -to $rx_error_meta
set_false_path -hold -to $rx_error_meta

report_area -file rtl.area
optimize_gate -packarea gate.area
legalize_phy_inst
update_timing
export_db optimized_gate.db
place
route
report_area -io_info -file final.area
update_timing -mode final
report_timing_summary -file final_timing.rpt
report_timing_status -file timing_status.txt
report_timing_exception -file timing_exceptions.rpt
report_timing_ignored -file timing_ignored.rpt
export_db routed.db

set report [open final_timing.rpt r]
set timing [read $report]
close $report
set setup_ok [regexp {SWNS:\s*(-?[0-9.]+)ns, STNS:\s*(-?[0-9.]+)ns} $timing _ sw st]
set hold_ok [regexp {HWNS:\s*(-?[0-9.]+)ns, HTNS:\s*(-?[0-9.]+)ns} $timing _ hw ht]
if {!$setup_ok || !$hold_ok} {
    puts "Missing timing summary; no R33 bit generated"
    exit 1
}
if {$sw < 0 || $st < 0 || $hw < 0 || $ht < 0} {
    puts "Timing failed: SWNS=$sw STNS=$st HWNS=$hw HTNS=$ht; no R33 bit generated"
    exit 1
}

bitgen -bit ph1p35_csi_cnn_r2_maincam_r33_cdc_timing.bit
puts PH1P35_CSI_CNN_R2_MAINCAM_R33_CDC_TIMING_BUILD_COMPLETE
exit
