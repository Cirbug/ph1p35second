set root D:/2026fpga/ph1p35second_r2_jop13/fpga
set out D:/2026fpga/bank/ph1p35_csi_cnn_20261031_r37_rx_frame_sync
file mkdir $out

cd $root
open_project $root/fpga_prj.al -single_run
import_device ph1_35p.db -package PH1P35MDG324
elaborate -top csi2_tx_top
read_adc $root/uisrc/04_pin/fpga_pin.adc
read_ip_adc -ip ddr2 -file $root/uisrc/03_ip/ddr2/src/adc/ddr2.adc

# DDR generated clocks must exist before the board timing constraints.
read_sdc -ip ddr2 $root/uisrc/03_ip/ddr2/src/sdc/ddr2.sdc
read_sdc $root/uisrc/04_pin/timing.sdc

cd $out
optimize_rtl
update_timing

set rx_clock [get_clocks -nowarn {MIPI_RX_BYTE_CLK}]
set tx_clock [get_clocks -nowarn {MIPI_TX_BYTE_CLK}]
set ddr_usr_clock [get_clocks -nowarn {*u_ph1p35_324_ddr_wrapper/u_ddr2/usr_clk}]
puts "R37 clocks RX=[llength $rx_clock] TX=[llength $tx_clock] DDR_USR=[llength $ddr_usr_clock]"
if {[llength $rx_clock] != 1 || [llength $tx_clock] != 1 || [llength $ddr_usr_clock] != 1} {
    puts "R37 timing constraint error: expected RX/TX/DDR user clocks were not found"
    exit 1
}

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
    puts "Missing timing summary; no R37 bit generated"
    exit 1
}
if {$sw < 0 || $st < 0 || $hw < 0 || $ht < 0} {
    puts "Timing failed: SWNS=$sw STNS=$st HWNS=$hw HTNS=$ht; no R37 bit generated"
    exit 1
}

bitgen -bit ph1p35_csi_cnn_r2_maincam_r37_rx_frame_sync.bit
puts PH1P35_CSI_CNN_R2_MAINCAM_R37_RX_FRAME_SYNC_BUILD_COMPLETE
exit

