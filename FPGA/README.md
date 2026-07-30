# FPGA 实时处理

Tang Primer 25K 完成 ADC 采样、测频、锁定判定、相位同步、DDS 和 DAC 输出。
控制闭环完全位于 FPGA 内部，不依赖 STM32 的 SPI 轮询时延。

## 数据通路

- `f2026_input_tracker.v`：25 MSPS ADC 采样，至少8码迟滞的上升过零检测，周期
  Q8/16 IIR 滤波，连续有效周期后置位 `locked`。
- `f2026_phase_increment.v`：33 周期顺序除法器，计算
  `(2^32 + period/2) / period`，不占 BSRAM 和 DSP。
- `f2026_waveform_core.v`：32 位 DDS；可靠输入过零点硬同步；生成同相、
  +90 度或二倍频正弦；输入失锁时输出 DAC 中点。
- `f2026_spi_control.v`：与 STM32 交换模式、幅度和状态，协议版本为 2。
- `f2026_top.v`：选择跟踪或第五问自由运行路径，并补偿 53 个 50 MHz tick
  及实测得到的固定-2度相位偏差；12.4 kHz以下另应用低频线性相位校准。
  二倍频模式自动使用双倍延迟补偿。短时失锁时
  保持最后有效DDS状态5 ms，避免DAC瞬间返回中点在XY图形中留下横线；
  输入持续消失时仍会自动静音。

## 模式

| mode | 名称 | 相位表达式 |
| ---: | --- | --- |
| 0 | IDLE | DAC 中点 |
| 1 | DIAGONAL | `phase + trim` |
| 2 | CIRCLE | `phase + 90 deg + trim` |
| 3 | DOUBLE | `2*phase + trim` |

`free_run=0` 时，FPGA 忽略 SPI 中的调谐字并使用 ADC 实测周期。`free_run=1`
时使用 SPI 调谐字和相位偏置，这是第五问视觉闭环的预留入口。

## SPI v2

SPI Mode 0、MSB first、最高 5 MHz，每帧固定 16 字节。

- `0x01`：读取签名、协议版本、实际锁定/输出状态、周期、边沿计数、ADC
  最大最小值、当前模式和幅度码。
- `0x10`：原子设置模式、幅度、输出使能、自由运行、调谐字、相位偏置、
  DAC 中点和过零迟滞。

正常使用低有效 CS；现有 SPI PHY 也保留了基于空闲间隔的无 CS 帧同步兼容。

## 构建与仿真

```powershell
cd F:\code\stm32\2026F
FPGA\sim\run_modelsim.ps1
gw_sh FPGA\build.tcl
```

回归覆盖 1/10/100 kHz、10 到 20 kHz 动态重锁、+90 度、二倍频、四档幅度、
失锁静音、输入恢复、OTR 和第五问自由运行接口。

约束文件为 `constraints/tang_primer_25k_f2026.cst`，位流为
`impl/pnr/f2026.fs`。
