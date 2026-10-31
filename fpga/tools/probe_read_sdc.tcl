set root D:/2026fpga/ph1p35second_r2_jop13/fpga
set out D:/2026fpga/bank/ph1p35_csi_cnn_20261031_read_sdc_probe
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
puts "PROBE_SDC before_extra rx=[llength [get_clocks -nowarn {MIPI_RX_BYTE_CLK}]] tx=[llength [get_clocks -nowarn {MIPI_TX_BYTE_CLK}]] ddr=[llength [get_clocks -nowarn {*u_ph1p35_324_ddr_wrapper/u_ddr2/usr_clk}]]"
read_sdc $root/tools/probe_extra.sdc
puts "PROBE_SDC after_extra"
update_timing
puts "PROBE_SDC after_extra_update"
exit
