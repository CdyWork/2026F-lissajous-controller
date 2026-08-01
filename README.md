# 2026F 李萨如图形显示控制装置

本仓库当前定版包含前四问实时信号处理，以及第五问的独立视觉测频方案。第五问不从被测信号取得电气测量值：IMX219 只拍摄示波器 XY 画面，香橙派从图形估计频率并回传 STM32 LCD。

## 当前第五问方案

```text
第 4 个按键 (STM32 KEY3/PE2)
    -> USART3 PB10/PB11: MEASURE
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

普通表为 `STEP 0..31`，斜坡宽度覆盖 `10 us` 到 `1000 us`，每帧固定 `2 ms`。低频兜底为：

| STEP | 斜坡 | 循环 | 用途 |
| ---: | ---: | ---: | --- |
| 32 | 2 ms | 10 ms | 低频备用 |
| 33 | 6 ms | 10 ms | 首选低频兜底 |

锯齿波默认停在 DAC 负端电平；斜坡从负端上升至正端，再回到负端。每次 `STEP` 都由 STM32 写入 FPGA 并重新触发示波器显示。

### 香橙派测频流程

1. 常驻服务启动时，在 FPGA `IDLE` 状态完成 XY 轴校准和空白背景缓存。
2. 收到 `MEASURE` 后，复用缓存，向 STM32 发送 `STEP n`。
3. 每次切档等待 `0.50 s`，并丢弃 8 帧相机缓存；该时序不能缩短，否则会把完整波形拍成局部轨迹。
4. 从 `STEP 16` (`108 us`) 开始按中线交点数选择挡位：交点少则增加斜坡，交点多则缩短斜坡，最多 5 次二分。
5. 对首个稀疏 `108 us` 图优先跳挡：`<=3` 个交点先试 `862 us`，`4-5` 个交点先试 `640 us`，随后仍以二分纠偏。
6. 三个不同挡位后同时尝试 IDLE 空白背景和图像 q20 背景，选取轨迹更完整的结果，以抑制示波器余晖。
7. 选中挡位最多额外复核 3 次；任意两帧未标定频率在 5% 内一致，才输出加权中位数。
8. 普通表失败后依次测试 `STEP 33`、`STEP 32`。失败时回传 `RESULT 0`。

普通频率校准为：

```text
f_hz = 1.965686502 * f_visual_hz + 111.135141
```

低频兜底使用：

```text
f_hz = 1.95 * f_visual_hz
```

## 操作

香橙派服务应保持运行：

```bash
systemctl --user status q5-fpga-sweep.service
```

按下开发板第 4 个按键后，LCD 会在测量完成后显示 `PI` 标记及频率。典型耗时取决于二分次数：高频且 `108 us` 直接可用约 3 到 4 秒；低频或需要多次换档时更长。

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
| `pi/q5_frequency_measure.py` | 第五问自适应视觉测频、背景处理、标定和一致性判断 |
| `pi/q5_fpga_sweep.py` | 香橙派常驻服务、IMX219/UART、按键事件和结果回传 |
| `pi/q5-fpga-sweep.service` | 香橙派 systemd user service 定义 |
| `FPGA/rtl/f2026_waveform_core.v` | FPGA 锯齿波表与 DAC 输出 |
| `mcu/Src/f2026_app.c` | 按键、USART3 `MEASURE/RESULT`、SPI 配置与 LCD 显示 |
| `FPGA/` | Gowin 工程、RTL、约束和构建脚本 |
| `mcu/` | STM32F407 FreeRTOS 工程 |

`q5_adaptive_table.py`、旧的 `SWEEP` 八档采集脚本和历史图片分析仅作实验参考，不是当前按键测频路径。

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
scp -i $q5Key pi/q5_frequency_measure.py pi/q5_fpga_sweep.py pi/q5-fpga-sweep.service `
  orangepi@192.168.242.224:/home/orangepi/2026F/pi/
ssh -i $q5Key orangepi@192.168.242.224 "`
  cp /home/orangepi/2026F/pi/q5-fpga-sweep.service /home/orangepi/.config/systemd/user/; `
  systemctl --user daemon-reload; `
  systemctl --user restart q5-fpga-sweep.service"
```

## 前四问

前四问的实时控制闭环仍在 FPGA 内：ADC 采样、锁相、DDS 与 DAC 输出不依赖香橙派。按键功能为同相、90 度、二倍频，以及第五问视觉测频请求。第五问执行期间 STM32 只做命令转发和结果显示，不参与频率计算。

## 文档入口

本 README 是当前方案的唯一完整说明。`pi/README.md`、`FPGA/README.md`、`mcu/README.md`、`release/README.md` 与 `Q5_VISION_AI_NOTES.md` 仅保留入口或历史说明，避免与定版流程冲突。第三方 `ai-sdk/` 文档和 FPGA 诊断文档不属于本工程操作说明。
