source ./settings.cfg
source "D:/New Folder/doc/scripts/DefaultFlow.tcl"
puts "==================== POST-PROCESS: FIX HOLD ===================="
fix_hold
update_timing -mode final
bitgen -bit fpga_prj.bit
puts "==================== DONE ===================="
