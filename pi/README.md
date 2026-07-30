# Raspberry Pi Q5 vision

This directory contains the Raspberry Pi Zero 2 W side of the requirement-5
optical frequency measurement prototype. It uses deterministic image and
signal processing; no trained neural-network model is required.

## Implemented

- automatic oscilloscope graticule location without external markers;
- four-corner perspective rectification and reusable JSON calibration;
- trace-free background subtraction with green-trace fallback;
- vectorized row-centroid extraction and invalid-frame quality rejection;
- FFT coarse cycle estimate followed by sine least-squares refinement;
- 100 Hz source-bin decision;
- continuous fitted-phase regression for sub-bin frequency offset;
- OV5647/Picamera2 adapter with fixed exposure and achieved-metadata capture;
- adapter for the STM32 ASCII UART protocol already present in `mcu/`;
- offline tests against the independent Q5 image simulator.

The current STM32/FPGA production firmware does not yet provide a `PROBE`
command or a 40/48-bit DDS word. `serial_link.McuLink.request_probe()` is an
explicit integration point and deliberately raises `NotImplementedError`.
The existing integer `FREQ`, `PHASE`, and `AUTO` commands are supported.

## Raspberry Pi installation

Use Raspberry Pi OS Lite. Picamera2 should come from the OS packages so that
it matches the installed libcamera stack.

```bash
sudo apt update
sudo apt install python3-picamera2 python3-opencv python3-numpy python3-serial
```

The pip requirements are mainly for desktop/offline testing. Do not install a
second pip copy of Picamera2.

On the tested DietPi Debian 13 image, Picamera2 also needs the firmware ISP and
the ISP kernel module. Back up `/boot/firmware/config.txt`, set:

```text
start_x=1
gpu_mem_512=128
```

Then place `bcm2835_isp` in `/etc/modules-load.d/f2026-camera.conf` and reboot.
Without this, direct V4L2 raw capture works but Picamera2 reports no cameras.

## Offline image measurement

```bash
cd /path/to/2026F/pi
python3 main.py image camera_frame.png --ramp-us 1000 \
  --save-calibration scope_calibration.json --debug-dir debug
```

For 70-100 kHz, use a 0.5 ms probe after the FPGA has switched its probe:

```bash
python3 main.py image camera_frame.png --ramp-us 500
```

The JSON result includes continuous frequency, the selected 100 Hz bin, fitted
phase, amplitude, residual error, valid-row fraction, and mean row spread.

## Live OV5647 measurement

Before starting, the DA probe must be parked outside the visible graticule so
the first 20 frames contain only the scope background. The application then
uses those frames for localization and background subtraction.

```bash
python3 main.py camera \
  --exposure-us 800 --gain 16 --capture-fps 60 --process-fps 30 \
  --ramp-us 1000 --coarse-seconds 0.8 --phase-seconds 1.2
```

Use `--serial /dev/serial0 --apply --mode DIAG` only after the probe waveform
is controlled externally/current firmware has been extended. `--apply` sends
the measured integer frequency and selected automatic mode using the existing
STM32 protocol.

The program prints both the requested exposure and the OV5647 supported range.
Every captured frame also carries the achieved exposure from camera metadata.

## Camera and oscilloscope setup

- capture 640x480 at 60 fps and process at 30 fps;
- start at 500-1000 us exposure and fixed analogue gain; the tested scope and
  OV5647 needed about 16x gain at 793 us, while 32x was visibly noisier;
- disable scope persistence and trigger from the DA probe rising edge;
- rigidly mount the camera with the full 10x8 graticule visible;
- make the graticule at least about 450x340 camera pixels;
- turn off automatic exposure after setup;
- reject broken traces and poor sine fits instead of returning a false
  frequency. Scope persistence must still be disabled because a camera cannot
  reconstruct phase information already mixed by the scope display.

## Tests

From the project root:

```powershell
python -m unittest discover -s .\pi\tests -v
```

The tests generate camera images through `q5_sawtooth_vision_sim`, exercise
automatic graticule location at normal and high range, check 2 Hz wrapped-phase
tracking, and verify the current STM32 command strings.

The slower regression covers every allowed 100 Hz point:

```powershell
python .\pi\tests\run_full_regression.py
```
