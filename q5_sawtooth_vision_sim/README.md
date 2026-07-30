# Q5 Sawtooth Vision Simulation

This isolated prototype evaluates the proposed optical frequency measurement
for requirement 5. It does not modify or build the production FPGA or STM32
projects.

The probe waveform repeats every 10 ms. The DAC normally produces a visible
rising ramp for 1 ms and then parks above the oscilloscope screen for the
remainder of the frame. Above a 70 kHz coarse estimate, a second 0.5 ms ramp is
used to improve the pixels per cycle. Since every allowed source frequency is
an integer multiple of 100 Hz, successive ramps begin at the same nominal input
phase.

For a 1 ms visible ramp, the number of horizontal sine cycles on the vertical
trace is numerically equal to the input frequency in kHz:

```text
cycles = input_frequency * ramp_time
input_frequency_hz = cycles / ramp_time_seconds
```

## Contents

- `rtl/q5_probe_waveform.v`: standalone synthesizable probe waveform prototype.
- `sim/tb_q5_probe_waveform.v`: timing and monotonicity testbench.
- `vision_sim.py`: synthetic oscilloscope/camera renderer and lightweight
  frequency recognizer.
- `run_all.ps1`: runs ModelSim and the full 1-100 kHz image regression.
- `outputs/`: generated images and numeric results.

## Run

```powershell
cd F:\code\stm32\2026F\q5_sawtooth_vision_sim
.\run_all.ps1
```

The image regression covers all 991 allowed frequencies from 1 kHz through
100 kHz in 100 Hz steps. It adds grid lines, blur, sensor noise, perspective,
and several accumulated 10 ms frames with a small source clock error.

The generated `outputs/montage.png` shows representative scope images,
`outputs/frequency_error.png` plots estimation error, and
`outputs/summary.json` contains the aggregate measurements.

## Scope of the result

This simulation verifies the waveform geometry and recognition method. It
cannot model the exact oscilloscope phosphor/rendering algorithm, camera focus,
screen moire, glare, or DAC analog settling. Those require a camera-on-scope
bench test before integrating this mode into the production design.
