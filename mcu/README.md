# STM32F407 控制器

该目录是 CS07-F407 板的精简 FreeRTOS 工程。STM32 不参与实时测频和锁相，
只负责按键/LCD、模式和幅度配置、FPGA 状态监测，以及第五问预留 UART 命令。

## 前面板

| 按键 | 功能 |
| --- | --- |
| KEY0 / PE5 | 同频同相输出 |
| KEY1 / PE4 | 同频并增加 90 度 |
| KEY2 / PE3 | 二倍频输出 |
| KEY3 / PE2 | 循环 2/4/6/8 div |

LCD 显示模式、幅度档、测得频率、`LOCK`、ADC 最大最小值、`OTR`、FPGA
通信状态和实际 DAC 输出状态。协议版本不等于 2 时显示离线。

## STM32 到 FPGA

| 信号 | STM32F407 | Tang J4 |
| --- | --- | ---: |
| SPI1 SCK | PA5 | 5 |
| SPI1 CS | PA4 | 6 |
| SPI1 MOSI | PA7 | 7 |
| SPI1 MISO | PA6 | 8 |
| RESET_N | PB1 | 9 |
| IRQ | PB0 | 10 |
| GND | GND | 3 或 4 |

SPI 为 Mode 0、5 MHz。PB1 上电先拉低复位 FPGA，初始化后释放。

## 第五问预留接口

USART1 使用 PA9/PA10，115200-8-N-1。现有命令包括：

```text
STATUS
TRACK DIAG|CIRCLE|DOUBLE
AUTO DIAG|CIRCLE|DOUBLE
FREQ 10000
PHASE 0
AMP 2|4|6|8
CAL 2|4|6|8 code
```

`TRACK` 用于前四问，频率由 FPGA 自主测量。`AUTO` 切换到自由运行，随后
`FREQ`、`PHASE` 和 `AMP` 可由第五问视觉闭环更新。当前前四问不需要连接
树莓派或串口。

## 构建

```powershell
cd F:\code\stm32\2026F\mcu
cmake --preset Release
cmake --build build\Release
```

输出：

```text
build\Release\f2026_controller.elf
build\Release\f2026_controller.hex
build\Release\f2026_controller.bin
```

F407 在当前板上需要低速、复位下连接：

```powershell
openocd -f interface/cmsis-dap.cfg -f target/stm32f4x.cfg `
  -c "transport select swd; adapter speed 100; reset_config srst_only srst_nogate connect_assert_srst; program F:/code/stm32/2026F/mcu/build/Release/f2026_controller.elf verify reset exit"
```
