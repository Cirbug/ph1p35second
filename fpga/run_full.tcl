#==========================================================================
# run_full.tcl  —— B 板全流程出流（syn + place + route + bitgen）
#
#   在工程根目录执行：
#     "D:\New Folder\bin\td_commands_prompt.exe" run_full.tcl
#
#   为什么不用自带的 run_flow_fixhold.tcl：
#     1) 它 source 的是 D:/TD/doc/scripts/DefaultFlow.tcl，这台机器上 TD 已
#        搬到 D:\New Folder\，旧路径不存在；
#     2) 新路径 "New Folder" 里有空格，td_commands_prompt.exe 会把命令行
#        按空格切开，等于给 source 传了两个参数，直接报
#        wrong # args: should be "source ?-encoding name? fileName"。
#   所以在 Tcl 里 source（加花括号引起来），命令行只传本文件这个无空格路径。
#==========================================================================

source ./settings.cfg

# alRun 已存在 -> DefaultFlow 外层会跳过它自己的 settings.cfg 读取
source {D:/New Folder/doc/scripts/DefaultFlow.tcl}
