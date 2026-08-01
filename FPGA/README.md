# FPGA 工程入口

第五问当前使用 `MODE_PROBE_TABLE`：`STEP 0..31` 为 2 ms 循环的普通锯齿波表，`STEP 32/33` 为 10 ms 循环的低频兜底。香橙派经 STM32 UART/SPI 逐次选择挡位，不使用旧的固定八档 `SWEEP` 方案。

RTL 实现位于 `rtl/f2026_waveform_core.v`，构建命令、下载顺序和完整第五问流程见仓库根目录 [README](../README.md)。
