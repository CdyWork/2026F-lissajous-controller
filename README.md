# 2026 F题：李萨如图形显示控制装置

本工程面向 2026 年电赛 F 题，当前实现前四问。硬件为 Tang Primer
25K、ATK-HS-ADDA 和 STM32F407VET6。第一问采用 ADC 采样后的数字重构，
不是模拟直通。

## 版本状态

**V1（第1～4问稳定）**：使用重映射后的 ADDA IO，支持 1 kHz～100 kHz
快速测频重锁、短时失锁保持、同频/90度/二倍频输出、四档幅度以及实测相位
校准。第五问接口和树莓派视觉代码保留，但不属于 V1 稳定范围。

## 前四问

| 按键 | 模式 | FPGA 输出 |
| --- | --- | --- |
| KEY0 | DIAG | 与输入同频、同相 |
| KEY1 | CIRCLE | 与输入同频、相位增加 90 度 |
| KEY2 | DOUBLE | 输入频率的 2 倍 |
| KEY3 | AMP | 循环切换 2/4/6/8 div 幅度 |

四种功能共用同一实时链路：

```text
信号源 -> ADDA ADC -> FPGA测频/锁相 -> DDS -> ADDA DAC -> 示波器Y
   +----------------------------------------------------> 示波器X
```

FPGA 根据 ADC 的过零周期自主计算 32 位 DDS 调谐字，并在每个可靠上升过零点
同步相位。STM32 只发送模式、幅度、使能和校准参数，不参与逐周期锁相，因此
SPI 延时不会进入控制环路。当前不使用 FPGA 软核，可节省 BSRAM，并降低实时
路径延迟；第五问需要时仍可在现有余量内增加控制逻辑。

输入失锁、输入消失或模式为 IDLE 时，FPGA 自动把 DAC 静音到中点 `0x80`。

## 目录

- `FPGA/`：Verilog RTL、管脚约束、ModelSim 回归和 Gowin 构建脚本。
- `mcu/`：STM32F407 + FreeRTOS 控制器、LCD、按键、SPI 和预留 UART 接口。
- `PROJECT_2026F.md`：系统结构、接线、操作和台架校准说明。
- `F题_李萨如图形显示控制装置.pdf`：题目原文。

## 构建

```powershell
cd F:\code\stm32\2026F
FPGA\sim\run_modelsim.ps1
cmake --build mcu\build\Release
gw_sh FPGA\build.tcl
```

主要产物：

```text
FPGA\impl\pnr\f2026.fs
mcu\build\Release\f2026_controller.elf
```

先把 FPGA 位流下载到 SRAM 完成台架验证，再写入外部 Flash。详细命令和
验收顺序见 `PROJECT_2026F.md`。
