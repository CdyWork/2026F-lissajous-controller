# J4 SPI pin scanner

This SRAM-only diagnostic counts rising edges on Dock J4 pins 5 through 10.
It reports one-second deltas over Dock FTDI channel B at 115200-8-N-1:

```text
J 5=1900/0 6=0000/0 7=0032/0 8=0000/1 9=0000/1 A=0000/0
```

`A` denotes J4-10. The value after `/` is the sampled logic level. On the
installed wiring, J4-5 is SCK (6400 rising edges/s), J4-7 is MOSI (50 rising
edges/s), J4-8 is MISO, J4-9 is RESET_N and J4-10 is IRQ. J4-6 showed no CS
activity, which led to the CS-less frame fallback in the production RTL.

The scanner defines all six J4 signals as inputs, so it does not drive MISO or
IRQ. Build with `gw_sh build.tcl` and download `impl/pnr/spi_pin_scan.fs` to
SRAM only.
