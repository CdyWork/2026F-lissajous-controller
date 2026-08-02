# 2026F 李萨如图形显示控制装置

当前归档版本：**V6.0（全完成，未整合）**。

本仓库当前定版包含前四问实时信号处理，以及第五问的独立视觉测频方案。第五问不从被测信号取得电气测量值：IMX219 只拍摄示波器 XY 画面，香橙派从图形估计频率并回传 STM32 LCD。

## 当前第五问方案

```text
4x4 矩阵键盘 (PD0~PD7) 数字 1/2/3
    -> USART3 PB10/PB11: MEASURE <task>
    -> Orange Pi Zero 3W 常驻服务
    -> STM32 SPI1
    -> FPGA TABLE 锯齿波
    -> 示波器 XY 画面
    -> IMX219 /dev/video0
    -> 香橙派图像测频
    -> USART3: RESULT <Hz>
    -> STM32 LCD: FREQ ... PI HZ
```

被测信号不接入香橙派、STM32 或 FPGA 的测量输入。FPGA 只产生用于示波器 XY 显示的锯齿波；频率结果完全来自相机拍摄的示波器图像。

### FPGA 锯齿波表

普通表为 `STEP 0..31`，斜坡宽度覆盖 `10 us` 到 `1000 us`，每帧固定 `10 ms`。低频兜底为：

| STEP | 斜坡 | 循环 | 用途 |
| ---: | ---: | ---: | --- |
| 32 | 2 ms | 10 ms | 低频备用 |
| 33 | 6 ms | 10 ms | 首选低频兜底 |

锯齿波默认停在 DAC 负端电平；斜坡从负端上升至正端，再回到负端。每次 `STEP` 都由 STM32 写入 FPGA 并重新触发示波器显示。

### 香橙派测频流程

1. 常驻服务启动时，在 FPGA `IDLE` 状态完成 XY 轴校准和空白背景缓存。
2. 收到 `MEASURE` 后，复用缓存，向 STM32 发送 `STEP n`。
3. 在 `STEP 17` (`125 us`) 图像中计数中线穿越点；1.1/49.9/90.9 kHz 理论值约为 `0.13/6.33/11.55`。中心线拟合失败时，再用二维青色亮带数量和占用率识别密集高频图。
4. `1.1 kHz` 候选固定使用 `STEP 33` 的 `6 ms` 斜坡复核，理论约 6 个过零点；49.9/90.9 kHz 主挡异常时分别使用 `125/69 us` 复核。失败时完整重试一次。
5. 成功后回传三个标准频点之一；失败时回传 `RESULT 0`。STM32 收到有效 `RESULT` 后立即让 FPGA 输出同频自由运行正弦波。

## 操作

香橙派服务应保持运行：

```bash
systemctl --user status q5-fpga-sweep.service
```

矩阵键盘数字 `1` 执行测频并把相位归零；数字 `2` 在相位归零后切换为同频 `+90°` 输出；数字 `3` 在相位归零后切换为二倍频输出。任务 3 将静态相位补偿和持续补偿速率同时乘 2，再通过二倍频“∞”图形的自交点复拍校准残余相位。板载第 4 个按键继续作为数字 `1` 的兼容入口。LCD 会在测量完成后显示 `PI` 标记及频率。一次成功测量只采集判别帧和复核帧；失败时再完整重试一次。

手动实时触发用于调试：

```bash
python3 /home/orangepi/2026F/pi/q5_fpga_sweep.py \
  --trigger --output-dir /home/orangepi/2026F/q5_manual_check
```

服务在启动前会执行：

```bash
v4l2-ctl -d /dev/video0 --set-ctrl=auto_exposure=1
```

这里的 `auto_exposure=1` 是该驱动的手动曝光模式。自动曝光会使短锯齿波图像随机残缺，不能恢复为自动模式。

## 工程结构

| 路径 | 当前用途 |
| --- | --- |
| `pi/q5_frequency_measure.py` | 第五问三频点视觉查表：1.1/49.9/90.9 kHz 最近理论过零数判别与低频复核 |
| `pi/q5_fpga_sweep.py` | 香橙派常驻服务、IMX219/UART、按键事件和结果回传 |
| `pi/q5_phase_lock.py` | 基波相位归零、校准时基漂移补偿与任务 3 二倍频自交点校准 |
| `pi/q5-fpga-sweep.service` | 香橙派 systemd user service 定义 |
| `FPGA/rtl/f2026_waveform_core.v` | FPGA 锯齿波表与 DAC 输出 |
| `mcu/Src/f2026_app.c` | 按键、USART3 `MEASURE/RESULT`、SPI 配置与 LCD 显示 |
| `FPGA/` | Gowin 工程、RTL、约束和构建脚本 |
| `mcu/` | STM32F407 FreeRTOS 工程 |

`q5_adaptive_table.py`、旧的连续频率拟合和 `SWEEP` 八档采集脚本仅作实验参考，不是当前按键测频路径。

## 构建与烧录

### FPGA

```powershell
cd F:\code\stm32\2026F
gw_sh FPGA\build.tcl
```

生成位流：`FPGA\impl\pnr\f2026.fs`。先下载 SRAM 验证；确认后再固化到 Flash。

### STM32

```powershell
cd F:\code\stm32\2026F\mcu
cmake --preset Release
cmake --build build\Release
openocd -f interface/cmsis-dap.cfg -f target/stm32f4x.cfg `
  -c "transport select swd; adapter speed 100; reset_config srst_only srst_nogate connect_assert_srst; program F:/code/stm32/2026F/mcu/build/Release/f2026_controller.elf verify reset exit"
```

### 香橙派部署

```powershell
$q5Key = Join-Path $env:USERPROFILE '.ssh\orangepi_vision_ed25519'
scp -i $q5Key pi/q5_frequency_measure.py pi/q5_fpga_sweep.py pi/q5_phase_lock.py pi/q5-fpga-sweep.service `
  orangepi@192.168.242.224:/home/orangepi/2026F/pi/
ssh -i $q5Key orangepi@192.168.242.224 "`
  cp /home/orangepi/2026F/pi/q5-fpga-sweep.service /home/orangepi/.config/systemd/user/; `
  systemctl --user daemon-reload; `
  systemctl --user restart q5-fpga-sweep.service"
```

## 前四问

前四问的实时控制闭环仍在 FPGA 内：ADC 采样、锁相、DDS 与 DAC 输出不依赖香橙派。第五问由矩阵键盘数字 `1/2/3` 选择归零、归零后 `+90°`、归零后二倍频任务。第五问执行期间 STM32 只做任务请求、命令转发和结果显示，不参与频率计算。

## 文档入口

本 README 是当前方案的唯一完整说明。`pi/README.md`、`FPGA/README.md`、`mcu/README.md`、`release/README.md` 与 `Q5_VISION_AI_NOTES.md` 仅保留入口或历史说明，避免与定版流程冲突。第三方 `ai-sdk/` 文档和 FPGA 诊断文档不属于本工程操作说明。
