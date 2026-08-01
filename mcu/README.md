# STM32 工程入口

第五问中第 4 个按键对应 `KEY3/PE2`。STM32 只向 USART3 (`PB10/PB11`) 发送 `MEASURE`，并执行香橙派返回的 `RESULT <Hz>`；FPGA 的 `IDLE/STEP` 配置由香橙派经 STM32 转发。按键本身不直接改写 FPGA 挡位，避免与测量流程竞争。

LCD 的 `FREQ` 行以 `PI` 标记视觉测频结果。构建、烧录和完整流程见仓库根目录 [README](../README.md)。
