onerror {quit -code 1}
if {[file exists work]} {vdel -lib work -all}
vlib work
vlog ../rtl/reset_conditioner.v \
     ../rtl/spi_mode0_slave.v \
     ../rtl/f2026_input_tracker.v \
     ../rtl/f2026_phase_increment.v \
     ../rtl/f2026_waveform_core.v \
     ../rtl/f2026_spi_control.v \
     ../rtl/f2026_top.v \
     tb_f2026.v
vsim -c work.tb_f2026
onfinish stop
run -all
quit -code 0
