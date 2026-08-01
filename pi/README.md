# Orange Pi 第五问入口

当前香橙派只使用以下定版路径：

```text
STM32 KEY3/第 4 个按键 -> MEASURE -> q5-fpga-sweep.service
-> q5_frequency_measure.py -> RESULT <Hz> -> STM32 LCD
```

完整的硬件连接、算法、部署与调试命令见仓库根目录 [README](../README.md)。

生产文件：

- `q5_fpga_sweep.py`：常驻相机/UART 服务。
- `q5_frequency_measure.py`：自适应 TABLE 挡位、XY 校准、背景处理、测频与标定。
- `q5-fpga-sweep.service`：systemd user service。

旧的 `SWEEP` 八档采集、离线图片处理和均匀扫频脚本不属于当前按键测频流程。
