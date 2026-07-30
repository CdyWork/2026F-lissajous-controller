# Q5 probe SRAM diagnostic

This isolated image drives only the remapped 8-bit DAC interface:

- 10 ms frame (100 Hz repetition)
- 1 ms rising ramp from code 77 to 179
- 9 ms parked at code 255
- 25 MHz DAC clock derived from the 50 MHz board clock

Build from this directory with `gw_sh build.tcl`. Program the generated
`impl/pnr/q5_probe_sram.fs` with SRAM operation index 2. Do not write Flash.
