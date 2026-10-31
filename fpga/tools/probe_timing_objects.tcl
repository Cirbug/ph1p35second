set root D:/2026fpga/ph1p35second_r2_jop13/fpga
set out D:/2026fpga/bank/ph1p35_csi_cnn_20261031_timing_probe
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
puts "PROBE before_opt clocks=[llength [get_clocks -nowarn *]]"
optimize_rtl
puts "PROBE after_opt clocks=[llength [get_clocks -nowarn *]]"
puts "PROBE pre_update named_rx=[llength [get_clocks -nowarn {MIPI_RX_BYTE_CLK}]] named_tx=[llength [get_clocks -nowarn {MIPI_TX_BYTE_CLK}]] ddr_usr=[llength [get_clocks -nowarn {*u_ph1p35_324_ddr_wrapper/u_ddr2/usr_clk}]]"
update_timing
set all_clocks [get_clocks -nowarn *]
puts "PROBE after_update clocks=[llength $all_clocks]"
puts "PROBE named_rx=[llength [get_clocks -nowarn {MIPI_RX_BYTE_CLK}]] named_tx=[llength [get_clocks -nowarn {MIPI_TX_BYTE_CLK}]]"
puts "PROBE ddr_usr=[llength [get_clocks -nowarn {*u_ph1p35_324_ddr_wrapper/u_ddr2/usr_clk}]]"
puts "PROBE rx_regs=[llength [get_regs -nowarn {u_ddr_triple_frame_cache/u_frame_pack_fifo/wgray[*]}]]"
puts "PROBE rx_nets=[llength [get_nets -nowarn {u_ddr_triple_frame_cache/u_frame_pack_fifo/wgray[*]}]]"
foreach c $all_clocks { puts "PROBE clock=$c" }
exit
