# MCU / Raspberry Pi 开发使用指南与注意事项

## 1. 目录分工

当前建议按下面规则放文件，后续不要混放。

```text
F:\code
```

放项目代码、模板工程、测试代码、自己的业务代码。

```text
F:\code\DietPi
```

放树莓派 Zero / Pico / Web 控制台相关项目。

```text
E:\RTOS
```

只放 RTOS 本体，例如：

```text
E:\RTOS\zephyr
E:\RTOS\FreeRTOS
```

```text
E:\MCU-env
```

放编译环境、工具链、Python 虚拟环境、CMake/Ninja 环境脚本。

```text
E:\MCU-help
```

放非 RTOS 的芯片厂商库、SDK、HAL、数据手册、安装包。

目前结构：

```text
E:\MCU-help\RaspberryPICO
E:\MCU-help\STM32HAL
E:\MCU-help\TI
```

示例：

```text
Pico SDK       -> E:\MCU-help\RaspberryPICO\SDKs
STM32 HAL/Cube -> E:\MCU-help\STM32HAL\CubePackages
TI MSPM0 SDK   -> E:\MCU-help\TI\MSPM0-SDK
```

## 2. Raspberry Pi Zero 连接

Zero 当前主要通过 PC 热点直连：

```text
Zero IP: 192.168.137.2
PC 网关: 192.168.137.1
```

SSH：

```powershell
ssh -i "$env:USERPROFILE\.ssh\windows.pem" root@192.168.137.2
```

Tailscale 作为备用远程通道：

```text
100.122.249.51
```

注意：

- PC 热点优先，Tailscale 是备用。
- Zero 没屏幕、没键盘，修改网络配置前要格外小心。
- Tailscale DNS 已关闭接管，Zero 使用普通公网 DNS，避免 PC 热点下解析失败。

## 3. Zero 烧录 / 调试 Pico 2

当前 Zero 通过 GPIO 模拟 SWD 连接 Pico 2。

接线：

```text
Zero 物理 18 脚 / GPIO24 -> Pico SWDIO
Zero 物理 22 脚 / GPIO25 -> Pico SWCLK
Zero 物理 6 脚 / GND    -> Pico GND
```

OpenOCD 配置在 Zero：

```text
/root/pico-swd.cfg
/root/pico-swd-slow.cfg
```

烧录示例：

```bash
openocd -f /root/pico-swd-slow.cfg -f target/rp2350.cfg -c 'program /root/xxx.elf verify reset exit'
```

注意：

- Pico 2 是 RP2350，不是 RP2040。
- OpenOCD 目标配置应使用 `target/rp2350.cfg`。
- 如果 SWD 不稳定，优先用 `pico-swd-slow.cfg`。

## 4. Zero 与 Pico UART 通信

Pico 2 默认 UART0：

```text
Pico GP0 = UART0 TX
Pico GP1 = UART0 RX
```

Zero 串口：

```text
Zero GPIO14 = TXD
Zero GPIO15 = RXD
```

必须交叉连接：

```text
Zero GPIO14 TXD -> Pico GP1 RX
Zero GPIO15 RXD <- Pico GP0 TX
Zero GND        -> Pico GND
```

注意：

- UART 必须共地。
- TX/RX 不要直连同名，要交叉。
- Zero 的 `/dev/serial0` 当前指向 `/dev/ttyS0`。

## 5. Zero Web 控制台

Zero 上当前有一个本地 Web 控制台服务：

```text
http://192.168.137.2/
```

服务运行在 Zero：

```text
/opt/dietpi-web/app.py
systemd service: dietpi-web
listen: 0.0.0.0:80
```

当前页面：

```text
系统状态
文件列表
交互按键
Pico引脚
远程命令行
```

`Pico引脚` 页面用于查看 Pico 各 GP 引脚当前角色和工作状态，不再显示示波器波形。点击某个引脚后，会显示：

```text
角色
方向
连接位置
当前工作
说明
实时状态
```

其中 ADC 引脚会通过 Zero 的 `/dev/serial0` 向 Pico 固件发送读取命令，并显示实时电压。Pico 2 上外部模拟输入优先使用：

```text
Pico GP26 / ADC0
Pico GP27 / ADC1
Pico GP28 / ADC2
```

注意：`GPIO29 / ADC3` 是板载电源检测通道，用于测量 `VSYS/3`，不是普通外部空闲模拟输入。USB 供电时读到约 `1.5V` 属于正常现象，反推 `VSYS` 约为 `4.5V`。

相关串口链路仍然是：

```text
Zero /dev/serial0 <-> Pico UART0
Zero GPIO14 TXD -> Pico GP1 RX
Zero GPIO15 RXD <- Pico GP0 TX
Zero GND        -> Pico GND
```

控制台维护命令：

```powershell
ssh -i "$env:USERPROFILE\.ssh\windows.pem" root@192.168.137.2 "systemctl status dietpi-web --no-pager"
ssh -i "$env:USERPROFILE\.ssh\windows.pem" root@192.168.137.2 "systemctl restart dietpi-web"
ssh -i "$env:USERPROFILE\.ssh\windows.pem" root@192.168.137.2 "journalctl -u dietpi-web -n 80 --no-pager"
```

注意：

- Web 控制台是单文件 Python HTTP 服务，源码在 `/opt/dietpi-web/app.py`。
- 修改前建议备份 `/opt/dietpi-web/app.py`。
- Pico 引脚页依赖 Pico 端 Zephyr ADC scope 固件的串口命令，例如 `read 3`。
- 如果页面读取 ADC 失败，先检查 Pico 固件是否在运行、UART 是否共地、`/dev/serial0` 是否被其他进程占用。

## 6. Zephyr

Zephyr 根目录：

```text
E:\RTOS\zephyr
```

环境脚本：

```powershell
. E:\MCU-env\zephyr\zephyr-dev-shell.ps1
```

Pico 2 模板项目：

```text
E:\MCU-help\RaspberryPICO\Projects\zephyr-pico2-template
F:\code\RaspberryPICO\Template_Project\Zephyr_Template
```

构建：

```powershell
west build -b rpi_pico2/rp2350a/m33 E:\MCU-help\RaspberryPICO\Projects\zephyr-pico2-template -d E:\MCU-env\zephyr\build\zephyr-pico2-template -p always
```

产物：

```text
E:\MCU-env\zephyr\build\zephyr-pico2-template\zephyr\zephyr.elf
E:\MCU-env\zephyr\build\zephyr-pico2-template\zephyr\zephyr.uf2
```

注意：

- Zephyr 里已支持 Pico 2、STM32、MSPM0G3507。
- `lp_mspm0g3507` 是 Zephyr 里的 TI MSPM0G3507 板名。
- STM32 支持较成熟，适合长期学习 RTOS。

## 7. FreeRTOS

FreeRTOS 根目录：

```text
E:\RTOS\FreeRTOS
```

环境脚本：

```powershell
. E:\MCU-env\FreeRTOS\freertos-dev-shell.ps1
```

模板项目：

```text
F:\code\DietPi\freertos-3boards-template
```

支持目标：

```text
Pico 2 / RP2350      -> ARM_CM33
STM32F407            -> ARM_CM4F
TI MSPM0G3507        -> ARM_CM0
```

注意：

- FreeRTOS 只是内核，不像 Zephyr 那样自带完整板级数据库。
- STM32 需要 STM32Cube/CMSIS/HAL。
- MSPM0G3507 需要 TI MSPM0 SDK。
- Pico 2 当前本机编译还受 `picolibc.specs` 工具链问题影响。

## 8. MCU-help 放库规则

`E:\MCU-help` 放的是厂商库实体文件，不是 RTOS。

当前已复制：

```text
E:\MCU-help\RaspberryPICO\SDKs\pico-sdk-v1.5.1
E:\MCU-help\RaspberryPICO\SDKs\pico-sdk-rp2350-from-zephyr
E:\MCU-help\RaspberryPICO\SDKs\pico-extras
E:\MCU-help\RaspberryPICO\Examples\pico-examples-v1.5.1
E:\MCU-help\RaspberryPICO\Examples\pico-playground
E:\MCU-help\RaspberryPICO\Projects\pico-uart-led
E:\MCU-help\RaspberryPICO\Projects\zephyr-pico2-template
E:\MCU-help\RaspberryPICO\Docs\PICO
```

后续下载后放：

```text
STM32CubeF4 -> E:\MCU-help\STM32HAL\CubePackages\STM32CubeF4
TI MSPM0 SDK -> E:\MCU-help\TI\MSPM0-SDK
Pico SDK 2.x -> E:\MCU-help\RaspberryPICO\SDKs\pico-sdk-v2.x
```

不要把这些库放进：

```text
E:\RTOS
F:\code
```

除非是项目自己的源码或模板工程。

## 9. 常用命令

打开项目：

```powershell
& "E:\IDE\Microsoft VS Code\bin\code.cmd" F:\code\DietPi
```

连接 Zero：

```powershell
ssh -i "$env:USERPROFILE\.ssh\windows.pem" root@192.168.137.2
```

打开 Zero Web 控制台：

```text
http://192.168.137.2/
```

查看 / 重启 Web 控制台：

```powershell
ssh -i "$env:USERPROFILE\.ssh\windows.pem" root@192.168.137.2 "systemctl status dietpi-web --no-pager"
ssh -i "$env:USERPROFILE\.ssh\windows.pem" root@192.168.137.2 "systemctl restart dietpi-web"
```

检测 Pico 2 SWD：

```bash
openocd -f /root/pico-swd-slow.cfg -f target/rp2350.cfg -c 'init; targets; shutdown'
```

Zephyr 构建 Pico 2：

```powershell
. E:\MCU-env\zephyr\zephyr-dev-shell.ps1
west build -b rpi_pico2/rp2350a/m33 E:\MCU-help\RaspberryPICO\Projects\zephyr-pico2-template -d E:\MCU-env\zephyr\build\zephyr-pico2-template -p always
west build -b rpi_pico2/rp2350a/m33 F:\code\RaspberryPICO\Template_Project\Zephyr_Template -d E:\MCU-env\zephyr\build\pico-zephyr-io-template -p always
```

通过 Zero 烧录 Zephyr ELF：

```powershell
E:\MCU-help\RaspberryPICO\Projects\zephyr-pico2-template\tools\flash_via_zero.ps1 -Slow
```

## 10. 操作注意事项

- 不要随便删除 `E:\MCU-env`，这里放工具链和环境。
- 不要把 RTOS、SDK、项目代码混在一个目录里。
- Zero 没屏幕，改网络前先确认有备用 SSH 路径。
- 烧录 Pico 前确认芯片型号，Pico 2 用 RP2350。
- SWD 线尽量短，GND 一定要接。
- UART 通信必须 TX/RX 交叉。
- 编译失败时先分清是源码问题、SDK 问题、工具链问题，别急着重装全部环境。
- 下载 STM32/TI 包时优先保留原始压缩包到 `Packages` 目录，再解压到对应 SDK/HAL 目录。
- 大型 SDK 不要反复复制到项目目录，项目只引用 `E:\MCU-help`。

---

## Remote Access Notes - SSH / Zero WiFi

### Windows SSH

This Windows machine can be reached over Tailscale by SSH.

```text
Host alias from local Linux/server: windows-dev
Tailscale IP: 100.114.50.114
Windows user: administrator
whoami: desktop-enujsko\administrator
```

From local Linux:

```bash
ssh windows-dev
ssh -i /home/cdy/key/windows_dev_ed25519 administrator@100.114.50.114
```

From cloud server:

```bash
ssh windows-dev
ssh -i /root/.ssh/windows_dev_ed25519 administrator@100.114.50.114
```

Key and OpenSSH notes are also stored on Windows:

```text
C:\ProgramData\ssh\windows-dev-key\README.md
C:\ProgramData\ssh\administrators_authorized_keys
```

File transfer examples:

```bash
scp local.txt windows-dev:F:/ssh-transfer-test/local.txt
scp windows-dev:F:/ssh-transfer-test/local.txt /tmp/from-windows.txt
```

### Raspberry Pi Zero WiFi Fallback

Zero still prefers the local hotspots first. Current WiFi order:

```text
cdytest priority 30 -> cdy priority 10 -> sdu_net priority 1
```

`sdu_net` is only a fallback when `cdytest` and `cdy` are unavailable. The campus SRun account is encrypted on the Zero and decrypted only by the root-only login helper when `sdu_net` is selected.

Zero-side documents:

```text
/root/README.md
/root/network/sdu_net/README.md
```

Useful Zero checks:

```bash
ssh -i ~/.ssh/dietpi_zero2w_ed25519 root@100.122.249.51
wpa_cli -i wlan0 status
/root/network/sdu_net/sdu_net_login.py --check
cat /var/log/sdu_net_login.log
```

