source ./settings.cfg
source D:/TD/doc/scripts/DefaultFlow.tcl
puts "==================== POST-PROCESS: FIX HOLD ===================="
fix_hold
update_timing -mode final
report_timing_status -file fpga_prj_phy_fixed.ts
report_qor -step route -file route_fixed.qor
report_area -io_info -file fpga_prj_phy_fixed.area
bitgen -bit fpga_prj.bit
puts "==================== DONE ===================="
