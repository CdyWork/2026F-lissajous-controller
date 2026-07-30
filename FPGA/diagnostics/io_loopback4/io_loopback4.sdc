create_clock -name clk_50m -period 20.000 [get_ports {clk_50m}]
set_false_path -from [get_ports {ad_data[*]}]
set_false_path -to [get_ports {da_data[*] uart_tx}]
