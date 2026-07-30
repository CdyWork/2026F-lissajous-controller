transcript file transcript
if {[file exists work]} {vdel -lib work -all}
vlib work
vlog -work work ../rtl/q5_probe_waveform.v tb_q5_probe_waveform.v
vsim -c -voptargs=+acc work.tb_q5_probe_waveform
run -all
quit -f
