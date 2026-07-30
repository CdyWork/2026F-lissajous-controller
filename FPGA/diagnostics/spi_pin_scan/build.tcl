set_device -name GW5A-25A GW5A-LV25MG121NC1/I0
add_file -type verilog "spi_pin_scan.v"
add_file -type cst "spi_pin_scan.cst"
add_file -type sdc "spi_pin_scan.sdc"

set_option -top_module spi_pin_scan
set_option -verilog_std v2001
set_option -output_base_name spi_pin_scan
set_option -use_cpu_as_gpio 1
set_option -cst_warn_to_error 1
run all
