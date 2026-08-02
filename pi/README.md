# Orange Pi 第五问入口

当前香橙派只使用以下定版路径：

```text
STM32 KEY3/第 4 个按键 -> MEASURE -> q5-fpga-sweep.service
-> q5_frequency_measure.py -> 1.1/49.9/90.9 kHz lookup -> RESULT <Hz>
-> STM32 LCD and same-frequency FPGA output
-> STATUS/CTICKS reference import -> visual phase confirmation -> phase lock
```

完整的硬件连接、算法、部署与调试命令见仓库根目录 [README](../README.md)。

生产文件：

- `q5_fpga_sweep.py`：常驻相机/UART 服务。
- `q5_frequency_measure.py`：125 us 主挡、二维亮带后备判据、异常复核与三频点查表。
- `q5_phase_lock.py`：导入最后一次 100 kHz 校准计数，复算三个频点的 DDS
  量化残差，并用短视觉窗口确认 90.9 kHz 的实际补偿率；任务 3 将静态
  相位和补偿速率乘 2，再复拍二倍频“∞”自交点完成残余相位校准。
- `q5-fpga-sweep.service`：systemd user service。

相位报告中的 `drift_rate_source=reference_calibration_ticks` 表示采用本轮
校准计数；90.9 kHz 的理论值还会与视觉值交叉确认，差异超过保护阈值时
标记为 `current_visual_fit`。`90k9_bench_fallback` 只在两种数据都无效时使用。

旧的 `SWEEP` 八档采集、离线图片处理和均匀扫频脚本不属于当前按键测频流程。
