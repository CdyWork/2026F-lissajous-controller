set script_dir [file dirname [file normalize [info script]]]
cd $script_dir

set_device -name GW5A-25A GW5A-LV25MG121NC1/I0
add_file -type verilog "rtl/reset_conditioner.v"
add_file -type verilog "rtl/spi_mode0_slave.v"
add_file -type verilog "rtl/f2026_input_tracker.v"
add_file -type verilog "rtl/f2026_phase_increment.v"
add_file -type verilog "rtl/f2026_waveform_core.v"
add_file -type verilog "rtl/f2026_spi_control.v"
add_file -type verilog "rtl/f2026_top.v"
add_file -type cst "constraints/tang_primer_25k_f2026.cst"
add_file -type sdc "constraints/tang_primer_25k_f2026.sdc"

set_option -top_module f2026_top
set_option -verilog_std v2001
set_option -output_base_name f2026
set_option -place_option 0
set_option -route_option 1
set_option -replicate_resources 0
set_option -timing_driven 1
set_option -cst_warn_to_error 1
set_option -use_cpu_as_gpio 1
set_option -use_i2c_as_gpio 1
set_option -gen_sdf 1
set_option -gen_verilog_sim_netlist 1

run all
