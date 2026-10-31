catch {connect_cable -help} result
puts "CONNECT_HELP=$result"
catch {download -help} result
puts "DOWNLOAD_HELP=$result"
catch {program -help} result
puts "PROGRAM_HELP=$result"
exit
