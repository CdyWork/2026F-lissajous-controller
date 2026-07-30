create_clock -name clk_50m -period 20.000 [get_ports {clk_50m}]

set_output_delay -clock clk_50m -max 5.000 [get_ports {da_data[*]}]
set_output_delay -clock clk_50m -min 0.000 [get_ports {da_data[*]}]
set_false_path -to [get_ports {da_clk}]
