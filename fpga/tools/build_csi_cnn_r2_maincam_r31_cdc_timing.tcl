set root D:/2026fpga/ph1p35second_r2_jop13/fpga
set out D:/2026fpga/bank/ph1p35_csi_cnn_20261031_r31_cdc_timing
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
read_sdc $root/tools/r31_cdc_timing.sdc
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
export_db routed.db

set report [open final_timing.rpt r]
set timing [read $report]
close $report
set setup_ok [regexp {SWNS:\s*(-?[0-9.]+)ns, STNS:\s*(-?[0-9.]+)ns} $timing _ sw st]
set hold_ok [regexp {HWNS:\s*(-?[0-9.]+)ns, HTNS:\s*(-?[0-9.]+)ns} $timing _ hw ht]
if {!$setup_ok || !$hold_ok} {puts "Missing timing summary; no R31 bit generated"; exit 1}
if {$sw < 0 || $hw < 0 || $st != 0 || $ht != 0} {puts "Timing failed: setup=$sw hold=$hw; no R31 bit generated"; exit 1}

bitgen -bit ph1p35_csi_cnn_r2_maincam_r31_cdc_timing.bit
puts PH1P35_CSI_CNN_R2_MAINCAM_R31_CDC_TIMING_BUILD_COMPLETE
exit
