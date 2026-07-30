set_device -name GW5A-25A GW5A-LV25MG121NC1/I0
add_file -type verilog "io_loopback4.v"
add_file -type cst "io_loopback4.cst"
add_file -type sdc "io_loopback4.sdc"

set_option -top_module io_loopback4
set_option -verilog_std v2001
set_option -output_base_name io_loopback4
set_option -use_cpu_as_gpio 1
set_option -cst_warn_to_error 1
run all
