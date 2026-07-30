set_device -name GW5A-25A GW5A-LV25MG121NC1/I0
add_file -type verilog "../../../q5_sawtooth_vision_sim/rtl/q5_probe_waveform.v"
add_file -type verilog "q5_probe_top.v"
add_file -type cst "q5_probe.cst"
add_file -type sdc "q5_probe.sdc"

set_option -top_module q5_probe_top
set_option -verilog_std v2001
set_option -output_base_name q5_probe_sram
set_option -use_cpu_as_gpio 1
set_option -cst_warn_to_error 1
run all
