# 2026F 前四问工程说明

## 实现结论

前四问采用统一的 ADC-FPGA-DAC 数字闭环。FPGA 每次检测到稳定的 ADC 上升
过零周期后，内部计算 DDS 调谐字，并在后续过零点同步相位。STM32F407 只做
人机界面和低速配置，不参与锁相环。

不使用软核的原因是当前控制规律固定，纯 RTL 延迟更低、资源更少，并且不占
紧缺的 BSRAM。第五问接口仍保留：STM32 可通过 SPI 把 FPGA 切到自由运行，
再设置频率和相位。

## 四问对应关系

1. `DIAG`：ADC 采样锁频，DAC 输出与输入同频同相。
2. `CIRCLE`：在锁定相位上增加 90 度。
3. `DOUBLE`：相位累加器输出二倍相位，即二倍频。
4. 任一跟踪模式下通过 KEY3 切换 2/4/6/8 div 幅度，保持锁频和锁相。

失锁时立即把 DAC 返回中点，输入恢复并重新取得连续有效周期后自动恢复输出。
输入频率改变时不需要 STM32 再发送调谐字。

## 接线

```text
信号源 ──┬──────────────> 示波器 X
          └─> ADDA ADC -> FPGA -> ADDA DAC ──> 示波器 Y
```

STM32F407 到 Tang J4：

| 信号 | F407 | J4 | FPGA ball |
| --- | --- | ---: | --- |
| SCK | PA5 | 5 | C11 |
| CS | PA4 | 6 | C10 |
| MOSI | PA7 | 7 | B11 |
| MISO | PA6 | 8 | B10 |
| RESET_N | PB1 | 9 | D11 |
| IRQ | PB0 | 10 | D10 |
| GND | GND | 3/4 | - |

已确认 ADDA 数据低四位的点对点接线：

| 位 | DAC -> Tang | FPGA ball | ADC -> Tang | FPGA ball |
| ---: | --- | --- | --- | --- |
| 0 | DA_D0 -> J3-24 | G4 | AD_D0 -> J3-3 | L1 |
| 1 | DA_D1 -> J3-23 | H4 | AD_D1 -> J3-4 | L2 |
| 2 | DA_D2 -> J3-22 | J1 | AD_D2 -> J3-5 | K4 |
| 3 | DA_D3 -> J3-21 | J2 | AD_D3 -> J3-6 | J4 |

其余 AD/DA 数据位、时钟和 OTR 必须严格按
`FPGA/constraints/tang_primer_25k_f2026.cst` 对应连接。ADDA、FPGA、F407、
信号源和示波器需要共地；不要把 ADDA 排针整体直插到 Tang Dock。

## 操作

| 按键 | LCD 模式 | 预期示波器 XY 图形 |
| --- | --- | --- |
| KEY0 | DIAG | 正斜率直线 |
| KEY1 | CIRCLE | 圆或椭圆，取决于 X/Y 幅度 |
| KEY2 | DOUBLE | 横向双瓣的 8 字形 |
| KEY3 | AMP | 图形 Y 方向依次对应 2/4/6/8 div |

`LOCK=NO` 时检查 ADC 时钟、输入偏置、幅度和数据线。ADC 码应稳定跨过
`128-3` 与 `128+3` 两个阈值，最大最小值之差建议至少 100，且 `OTR=NO`。

## 构建和验证

```powershell
cd F:\code\stm32\2026F
FPGA\sim\run_modelsim.ps1
cmake --build mcu\build\Release
gw_sh FPGA\build.tcl
```

仿真必须显示 `MODELSIM_RESULT: PASS`。PnR 必须无 setup/hold violation，且
BSRAM 保持为 0。先下载 `FPGA/impl/pnr/f2026.fs` 到 SRAM，确认 LCD 显示
`FPGA ONLINE`、串口 `STATUS` 返回 `COMM=1 VER=2` 后再写外部 Flash。

## 台架验收顺序

1. 空闲模式验证 AD_CLK=25 MHz、DA_CLK=50 MHz、DAC 数据为 `0x80`。
2. 输入 10 kHz，确认 `LOCK=YES`、`OTR=NO`、KEY0 有同频同相输出。
3. 把输入依次改为 1/10/50/100 kHz，确认无需 SPI 命令即可重新锁定。
4. KEY1 验证增加 90 度；实际 ADDA 固定延迟残差可通过 `PHASE` 校准。
5. KEY2 验证输出频率为输入的两倍，最高测试 100 kHz 输入/200 kHz 输出。
6. KEY3 验证 2/4/6/8 div，并用 `CAL` 校准实际板卡的四个 DAC 幅度码。
7. 拔掉或关闭输入，确认 DAC 静音；恢复输入后确认自动重新输出。
8. SRAM 全部通过后再写 FPGA 外部 Flash，并复位验证上电启动。

数字仿真无法代替 ADDA 模块的模拟幅度、直流偏置和固定相位延迟校准。

## 2026-07-29 构建与烧录记录

- ModelSim：`MODELSIM_RESULT: PASS`，错误 0，警告 0。
- Gowin：Logic 1907/23040，Register 675/23280，CLS 1158/11520，
  DSP 0.5/28，BSRAM 0；Fmax 66.495 MHz。
- 时序：setup TNS 0、hold TNS 0；最差 setup slack +4.961 ns，
  hold slack +0.275 ns。
- STM32F407：Flash 60620 B，RAM 36700 B；OpenOCD 写入并校验成功。
- FPGA：外部 Flash `0x0B4017` 擦写和校验成功，从 Flash 重新配置后串口
  返回 `COMM=1 VER=2`。
- FPGA 位流 SHA-256：
  `ECE388139F7CA871893032CAD93A57DC32C2C6971B439C9BEA195A8E7EC42C9E`。
- MCU ELF SHA-256：
  `B7FED5CABEA865E29D5956EAD072ED331F87C04FE3A7C3131D307BC801CBC535`。

烧录后的数字自检已通过：自由运行 10 kHz 时 `FOUT=1`；恢复跟踪模式后，
在当前异常 ADC 输入 `OTR=1 / LOCK=0` 条件下 `FOUT=0`。前四问的模拟波形、
90 度残差和四档实际电压仍需在消除 ADC 过量程后用示波器验收。
