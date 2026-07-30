# 新项目 AI 资料入口

本目录是 Tang Primer 25K 与 ATK-HS-ADDA 的新项目参考资料包。AI 开始新项目时按
以下顺序读取，不要直接在本目录编译或修改参考文件。

## 必读顺序

1. `README.md`：目录索引和资料来源。
2. `FPGA/NEW_PROJECT_GUIDE.md`：从参考工程创建新 FPGA 项目的步骤。
3. `FPGA/AI_HANDOFF.md`：连接、仿真、构建、SRAM 调试、调参和 Flash 工作流。
4. `FPGA/MODELSIM_WORKFLOW.md`：ModelSim 行为级、GW5A 原语级和波形调试流程。
5. `FPGA/THREE_TOOL_COSIM_WORKFLOW.md`：模拟前端、生产 RTL、SDF 和输出重建流程。
6. `ADDA/AI_HANDOFF.md`：ADDA 接口、电气安全和联调流程。
7. `FPGA/schematic/` 与 `ADDA/1，ATK-HS-ADDA模块原理图/`：按实物版本核对原理图。

## 使用原则

- 本目录是只读参考快照。新项目应复制 `FPGA/reference_application/` 到新的工程
  目录，再修改项目名、顶层、器件、引脚和协议。
- 不要把旧工程的 CST/SDC 直接用于另一块板；必须按新板原理图逐脚确认。
- 先用 `FPGA/validation_projects/self_test/` 验证下载器、时钟和基础 IO，再用
  `adda_loopback_test/` 验证 AD/DA 数字及模拟环路。
- 新 RTL 先跑 ModelSim 数学、定向分类、完整矩阵和系统行为级回归，再用 Gowin
  `prim_sim.v` 跑原语级测试；PnR 时序仍是单独的强制验收步骤。
- ModelSim 通过必须同时满足编译无错误、testbench `errors=0` 和
  `MODELSIM_RESULT: PASS`，不能只看 `$finish` 或旧日志。
- 三软件联合仿真是文件向量分阶段交换，不得表述为晶体管级完整混合信号仿真；
  正式结果必须包含新 PnR、SDF 0 error、生产 RTL 和 Multisim 输出验收。
- 新 bitstream 先下载 SRAM；Flash、OTP 和 Golden 区操作需要明确授权。
- ADDA 与 Tang Dock 的 40P 接口定义不兼容，禁止直接对插。
- 厂家资料可能包含多个硬件修订版。芯片型号、供电、引脚和模拟范围以实物丝印及
  对应版本原理图为准，不能从文件名猜测。

## 新项目应建立的文件

新工程根目录至少应创建：

```text
README.md          项目目标、硬件版本、构建和测试方法
AI_HANDOFF.md      当前架构、接线、已验证状态、风险和工作流
rtl/               可综合 RTL
sim/               testbench、输入向量和回归脚本
constraints/       经原理图核对的 CST/SDC
scripts/           构建、仿真、SRAM 下载和只读检查脚本
artifacts/         按日期保存的 fs、哈希、PnR、时序和测试记录
```

任何从本资料包复制的代码，都应在新项目 README 中记录来源和修改点。
