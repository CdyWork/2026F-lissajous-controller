create_clock -name clk_50m -period 20.000 [get_ports {clk_50m}]
set_false_path -from [get_ports {pin5 pin6 pin7 pin8 pin9 pin10}]
set_false_path -to [get_ports {uart_tx}]
