# Orange Pi 视觉服务入口

当前香橙派只使用以下定版路径：

```text
STM32 KEY3/第 4 个按键 -> MEASURE -> q5-fpga-sweep.service
-> q5_frequency_measure.py -> 1.1/49.9/90.9 kHz lookup -> RESULT <Hz>
-> STM32 LCD and same-frequency FPGA output
-> STATUS/CTICKS reference import -> visual phase confirmation -> phase lock
```

前三问复用同一常驻服务：STM32 发送 `TRACKCAL 1/2/3` 后，FPGA 保持输入硬锁相，香橙派从 XY 图形测量静态相位，并用已知相位探针确定补偿方向。校准成功后返回 `TRACKDONE`；该流程不执行第五问测频，也不启用自由运行 DDS 的漂移前馈。

完整的硬件连接、算法、部署与调试命令见仓库根目录 [README](../README.md)。

生产文件：

- `q5_fpga_sweep.py`：常驻相机/UART 服务。
- `key_tasks/key1_task.py` ... `key6_task.py`：数字 1~6 的独立入口。调度器每次重新加载对应脚本，并在运行前清空上一任务的 servo、FPGA 模式和相机余帧；脚本之间不保留任务状态，只共享服务启动时生成的只读 XY 标定结果。
- `q5_frequency_measure.py`：125 us 主挡、二维亮带后备判据、异常复核与三频点查表。
- `q5_phase_lock.py`：数字 1/2/3 的输入硬锁相视觉归正。
- `q5_q456_phase_lock.py`：数字 4/5/6 的冻结第五问相位链；导入最后一次
  100 kHz 校准计数，复算三个频点的 40 位 DDS 量化残差。任务 3 将静态
  相位和残余补偿速率乘 2，再复拍二倍频“∞”自交点完成残余相位校准。
- `q5-fpga-sweep.service`：systemd user service。

相位报告中的 `drift_rate_source=40bit_dds_below_deadband` 表示本轮校准后的
理论残差低于 `0.05°/s`，无需持续改写 `PHASEQ`；
`reference_calibration_ticks` 表示残差超过该死区并采用校准计数补偿。
只有没有有效参考校准的旧路径才允许使用 `current_visual_fit` 或
`90k9_bench_fallback`。

旧的 `SWEEP` 八档采集、离线图片处理和均匀扫频脚本不属于当前按键测频流程。
