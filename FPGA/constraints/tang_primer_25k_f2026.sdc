create_clock -name clk_50m -period 20.000 [get_ports {clk_50m}]

# The SPI inputs are explicitly synchronized and oversampled by clk_50m.
set_false_path -from [get_ports {hmi_spi_cs_n hmi_spi_sck hmi_spi_mosi fpga_reset_n}]

# The FPGA launches the 25 MHz ADC clock and captures the previous settled
# conversion. The 20 ns maximum reserves half of the 40 ns ADC period.
set_max_delay 20.000 -from [get_ports {ad_data[*] ad_otr}]
set_min_delay 0.000 -from [get_ports {ad_data[*] ad_otr}]
set_false_path -hold -from [get_ports {ad_data[*] ad_otr}]

set_output_delay -clock clk_50m -max 5.000 [get_ports {hmi_spi_miso hmi_irq da_data[*] uart_tx}]
set_output_delay -clock clk_50m -min 0.000 [get_ports {hmi_spi_miso hmi_irq da_data[*] uart_tx}]
set_false_path -to [get_ports {ad_clk da_clk}]
