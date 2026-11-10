<h1 align="center">
  Zephyr SDK Glue
</h1>

<p align="center">
  <strong>先楫半导体 RISC-V MCU 的 Zephyr RTOS 适配</strong>
</p>

<p align="center">
  <a href="#-特性">特性</a> •
  <a href="#-支持的开发板">开发板</a> •
  <a href="#-快速开始">快速开始</a> •
  <a href="#-文档">文档</a> •
  <a href="#-许可证">许可证</a>
</p>

<p align="center">
  <img src="https://img.shields.io/badge/版本-0.8.0-blue?style=flat-square" alt="版本">
  <img src="https://img.shields.io/badge/Zephyr-v4.4.0-green?style=flat-square" alt="Zephyr">
  <img src="https://img.shields.io/badge/许可证-Apache%202.0-orange?style=flat-square" alt="许可证">
  <img src="https://img.shields.io/badge/架构-RISC--V-red?style=flat-square" alt="RISC-V">
</p>

---

## 📖 概述

**Zephyr SDK Glue** 是 HPMicro 面向自有 RISC-V MCU 的 Zephyr 清单和硬件适配仓库，将 HPMicro 开发板、SoC、驱动、设备树、示例和测试支持与 Zephyr 主仓及 HPM_SDK 集成在一起。

### ✨ 特性

| 特性 | 描述 |
|------|------|
| **独立 West 清单** | 从 sdk_glue 初始化完整工作空间，并固定 Zephyr、HPM_SDK、中间件和引导程序依赖 |
| **HPMicro 原生硬件支持** | 为 HPMicro MCU 提供符合 Zephyr 模型的开发板、SoC、驱动、设备树和 pinctrl 适配 |
| **可复用硬件 Snippet** | 为 CAN、USB、以太网、显示、视频、存储等外设提供可复用的 overlay 和配置 |
| **Twister 验证矩阵** | 维护面向各开发板硬件能力的示例和子系统测试配置 |
| **灵活的工具链与调试器** | 支持 Zephyr SDK、ZCC、通用交叉工具链以及可选择的 J-Link/OpenOCD runner |
| **Docker 支持** | 提供开箱即用的 Docker 开发环境 |

---

## 🎯 支持的开发板

<table>
  <tr>
    <th>开发板</th>
    <th>SoC</th>
    <th>Twister 已覆盖特性</th>
  </tr>
  <tr>
    <td><b>hpm5100evk</b></td>
    <td>HPM5151</td>
    <td><img src="https://img.shields.io/badge/CAN-✓-success?style=flat-square" alt="CAN"> <img src="https://img.shields.io/badge/USB-✓-success?style=flat-square" alt="USB"> <img src="https://img.shields.io/badge/PWM-✓-success?style=flat-square" alt="PWM"> <img src="https://img.shields.io/badge/Counter-✓-success?style=flat-square" alt="Counter"></td>
  </tr>
  <tr>
    <td><b>hpm5300evk</b></td>
    <td>HPM5361</td>
    <td><img src="https://img.shields.io/badge/CAN-✓-success?style=flat-square" alt="CAN"> <img src="https://img.shields.io/badge/USB-✓-success?style=flat-square" alt="USB"> <img src="https://img.shields.io/badge/PWM-✓-success?style=flat-square" alt="PWM"> <img src="https://img.shields.io/badge/RNG-✓-success?style=flat-square" alt="RNG"> <img src="https://img.shields.io/badge/I2C-✓-success?style=flat-square" alt="I2C"></td>
  </tr>
  <tr>
    <td><b>hpm5301evklite</b></td>
    <td>HPM5301</td>
    <td><img src="https://img.shields.io/badge/GPIO-✓-success?style=flat-square" alt="GPIO"> <img src="https://img.shields.io/badge/USB-✓-success?style=flat-square" alt="USB"> <img src="https://img.shields.io/badge/Counter-✓-success?style=flat-square" alt="Counter"> <img src="https://img.shields.io/badge/Watchdog-✓-success?style=flat-square" alt="Watchdog"></td>
  </tr>
  <tr>
    <td><b>hpm5e00evk</b></td>
    <td>HPM5E31</td>
    <td><img src="https://img.shields.io/badge/CAN-✓-success?style=flat-square" alt="CAN"> <img src="https://img.shields.io/badge/USB-✓-success?style=flat-square" alt="USB"> <img src="https://img.shields.io/badge/PWM-✓-success?style=flat-square" alt="PWM"> <img src="https://img.shields.io/badge/Ethernet-✓-success?style=flat-square" alt="以太网"> <img src="https://img.shields.io/badge/I2C-✓-success?style=flat-square" alt="I2C"></td>
  </tr>
  <tr>
    <td><b>hpm6200evk</b></td>
    <td>HPM6280</td>
    <td><img src="https://img.shields.io/badge/CAN-✓-success?style=flat-square" alt="CAN"> <img src="https://img.shields.io/badge/USB-✓-success?style=flat-square" alt="USB"> <img src="https://img.shields.io/badge/PWM-✓-success?style=flat-square" alt="PWM"> <img src="https://img.shields.io/badge/RNG-✓-success?style=flat-square" alt="RNG"></td>
  </tr>
  <tr>
    <td><b>hpm6300evk</b></td>
    <td>HPM6360</td>
    <td><img src="https://img.shields.io/badge/CAN-✓-success?style=flat-square" alt="CAN"> <img src="https://img.shields.io/badge/USB-✓-success?style=flat-square" alt="USB"> <img src="https://img.shields.io/badge/PWM-✓-success?style=flat-square" alt="PWM"> <img src="https://img.shields.io/badge/Ethernet-✓-success?style=flat-square" alt="以太网"> <img src="https://img.shields.io/badge/SDHC-✓-success?style=flat-square" alt="SDHC"> <img src="https://img.shields.io/badge/RNG-✓-success?style=flat-square" alt="RNG"></td>
  </tr>
  <tr>
    <td><b>hpm6750evk2</b></td>
    <td>HPM6750</td>
    <td><img src="https://img.shields.io/badge/CAN-✓-success?style=flat-square" alt="CAN"> <img src="https://img.shields.io/badge/USB-✓-success?style=flat-square" alt="USB"> <img src="https://img.shields.io/badge/PWM-✓-success?style=flat-square" alt="PWM"> <img src="https://img.shields.io/badge/RNG-✓-success?style=flat-square" alt="RNG"> <img src="https://img.shields.io/badge/I2C-✓-success?style=flat-square" alt="I2C"> <img src="https://img.shields.io/badge/Ethernet-✓-success?style=flat-square" alt="以太网"> <img src="https://img.shields.io/badge/Display-RGB-success?style=flat-square" alt="RGB 显示"> <img src="https://img.shields.io/badge/Video-DVP-success?style=flat-square" alt="DVP 视频"> <img src="https://img.shields.io/badge/SDHC-✓-success?style=flat-square" alt="SDHC"></td>
  </tr>
  <tr>
    <td><b>hpm6800evk</b></td>
    <td>HPM6880</td>
    <td><img src="https://img.shields.io/badge/CAN-✓-success?style=flat-square" alt="CAN"> <img src="https://img.shields.io/badge/USB-✓-success?style=flat-square" alt="USB"> <img src="https://img.shields.io/badge/RNG-✓-success?style=flat-square" alt="RNG"> <img src="https://img.shields.io/badge/I2C-✓-success?style=flat-square" alt="I2C"> <img src="https://img.shields.io/badge/Ethernet-✓-success?style=flat-square" alt="以太网"> <img src="https://img.shields.io/badge/Display-RGB%2FMIPI%2FLVDS-success?style=flat-square" alt="显示"> <img src="https://img.shields.io/badge/Video-DVP%2FMIPI-success?style=flat-square" alt="视频"> <img src="https://img.shields.io/badge/SDHC-✓-success?style=flat-square" alt="SDHC"></td>
  </tr>
  <tr>
    <td><b>hpm6e00evk</b></td>
    <td>HPM6E80</td>
    <td><img src="https://img.shields.io/badge/CAN-✓-success?style=flat-square" alt="CAN"> <img src="https://img.shields.io/badge/USB-✓-success?style=flat-square" alt="USB"> <img src="https://img.shields.io/badge/PWM-✓-success?style=flat-square" alt="PWM"> <img src="https://img.shields.io/badge/RNG-✓-success?style=flat-square" alt="RNG"> <img src="https://img.shields.io/badge/I2C-✓-success?style=flat-square" alt="I2C"> <img src="https://img.shields.io/badge/Ethernet-✓-success?style=flat-square" alt="以太网"></td>
  </tr>
  <tr>
    <td><b>hpm6p00evk</b></td>
    <td>HPM6P81</td>
    <td><img src="https://img.shields.io/badge/CAN-✓-success?style=flat-square" alt="CAN"> <img src="https://img.shields.io/badge/USB-✓-success?style=flat-square" alt="USB"> <img src="https://img.shields.io/badge/PWM-✓-success?style=flat-square" alt="PWM"> <img src="https://img.shields.io/badge/Ethernet-✓-success?style=flat-square" alt="以太网"> <img src="https://img.shields.io/badge/RNG-✓-success?style=flat-square" alt="RNG"></td>
  </tr>
</table>

---

## 🔌 支持的外设驱动

| 外设 | 驱动 | 描述 |
|:----:|:----:|------|
| ADC | `adc` | 支持 ADC12 和 ADC16 |
| CAN | `can` | 支持 CAN 和 CANFD |
| Counter | `counter` | GPTMR 计数、告警和 Top Value 支持 |
| 显示 | `display` | RGB、MIPI DSI、LVDS 和 PixelMux 显示链路 |
| 熵源 | `entropy` | 硬件随机数生成器 |
| 以太网 | `ethernet` | 以太网 MAC 支持 |
| Flash | `flash` | XPI NOR Flash 读、写和擦除 |
| I2C | `i2c` | I2C 控制器支持 |
| Pinctrl | `pinctrl` | IOC 引脚复用和电气属性配置 |
| PWM | `pwm` | 支持 PWM 和 PWMv2 |
| SDHC | `sdhc` | SD/MMC 主机控制器支持 |
| SPI | `spi` | SPI 控制器及可选 DMA 传输 |
| UART | `serial` | UART 轮询、中断和异步操作 |
| USB | `usb` | Zephyr UDC 及 CherryUSB Device/Host 集成 |
| 视频 | `video` | Camera/DVP 和 MIPI CSI 采集链路 |
| Watchdog | `watchdog` | WDG 和 EWDG 看门狗控制器 |


## 🚀 快速开始

### 前置条件

- Zephyr SDK 1.0.1
- Python 3.12+
- CMake 3.20.5+
- west 0.14.0+
- Ninja 构建系统

### 使用 West 构建

```bash
# 初始化工作空间
west init -m https://github.com/hpmicro/zephyr_sdk_glue.git zephyr_hpmicro
cd zephyr_hpmicro
west update
west supply

# 构建 hello_world 示例
west build -p always -b hpm6750evk2 zephyr/samples/hello_world

# 烧录到开发板
west flash
```

### 使用 CMake 构建

```bash
cmake -GNinja -B build -DBOARD=hpm6750evk2 zephyr/samples/hello_world
ninja -C build
```

> 使用 `-S ${snippet}` 可以为特定示例指定硬件配置代码片段。

---

## 📁 目录结构

```
sdk_glue/
├── 📂 boards/              # HPMicro 开发板定义和通用 runner 支持
├── 📂 cmake/               # 编译器、链接器和工具链扩展
├── 📂 docs/                # 中英文文档
├── 📂 drivers/             # HPMicro Zephyr 外设驱动
├── 📂 dts/                 # 设备树 binding 和 HPMicro SoC 描述
├── 📂 include/             # 公共驱动和设备树头文件
├── 📂 modules/hpm_sdk/     # HPM_SDK 模块集成
├── 📂 samples/             # sdk_glue 专用示例
├── 📂 scripts/             # West 命令、SDK 补丁和检查工具
├── 📂 snippets/            # 可复用的开发板和外设配置
├── 📂 soc/                 # HPMicro SoC 初始化和链接支持
├── 📂 tests/               # sdk_glue 专用测试
├── 📂 twister/             # HPMicro 示例和测试适配元数据
├── 📂 zephyr/              # Zephyr 模块注册
├── west.yml                # 主 West 清单
├── west_gitee.yml          # 面向 Gitee 的 West 清单
└── docker-compose.yaml     # Docker 开发环境
```

---

## 📚 文档

| 文档 | 描述 |
|------|------|
| 📖 [Linux 入门指南](docs/source/zh/starting/linux.rst) | Linux 上的开发环境搭建 |
| 📖 [Windows 入门指南](docs/source/zh/starting/windows.rst) | Windows 上的开发环境搭建 |
| 🐳 [Docker 入门指南](docs/source/zh/starting/docker.rst) | 使用 Docker 开发环境 |
| 🔧 [工具链指南](docs/source/zh/starting/toolchain.rst) | 工具链配置 (Zephyr SDK / ZCC) |

---

## 📄 许可证

本项目采用 **Apache License 2.0** 许可证 - 详见 [LICENSE](LICENSE) 文件。

---
