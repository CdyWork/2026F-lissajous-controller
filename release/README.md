# Verified hardware images

These files archive the hardware images used with the remapped Tang Primer
25K ADDA interface.

## FPGA

- `f2026_fpga_20260730.fs`: GW5A-25A bitstream with the remapped ADDA pins,
  5 ms lock holdover, fast large-step reacquisition, 1 kHz detection margin,
  and measured phase calibration.
- Build source: `FPGA/`.
- Programming target: SRAM for validation; external Flash only after final
  oscilloscope acceptance.

## STM32

- `f2026_controller_f407_20260729.bin`: raw STM32F407 firmware image.
- `f2026_controller_f407_20260729.hex`: Intel HEX version of the same build.
- Build source: `mcu/`.

SHA-256 hashes are recorded in `SHA256SUMS.txt`.
