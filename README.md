<h1 align="center">
  Zephyr SDK Glue
</h1>

<p align="center">
  <strong>HPMicro's Zephyr RTOS Adaptation for RISC-V MCUs</strong>
</p>

<p align="center">
  <a href="#-features">Features</a> •
  <a href="#-supported-boards">Boards</a> •
  <a href="#-quick-start">Quick Start</a> •
  <a href="#-documentation">Docs</a> •
  <a href="#-license">License</a>
</p>

<p align="center">
  <img src="https://img.shields.io/badge/Version-0.8.0-blue?style=flat-square" alt="Version">
  <img src="https://img.shields.io/badge/Zephyr-v4.4.0-green?style=flat-square" alt="Zephyr">
  <img src="https://img.shields.io/badge/License-Apache%202.0-orange?style=flat-square" alt="License">
  <img src="https://img.shields.io/badge/Architecture-RISC--V-red?style=flat-square" alt="RISC-V">
</p>

---

## 📖 Overview

**Zephyr SDK Glue** is HPMicro's Zephyr manifest and hardware adaptation repository for its RISC-V MCUs. It combines HPMicro board, SoC, driver, devicetree, sample, and test support with the upstream Zephyr project and HPM_SDK.

### ✨ Features

| Feature | Description |
|---------|-------------|
| **Independent West Manifest** | Initializes the complete workspace from sdk_glue and pins Zephyr, HPM_SDK, middleware, and bootloader dependencies |
| **Native HPMicro Hardware Support** | Provides Zephyr-native board, SoC, driver, devicetree, and pinctrl integration for HPMicro MCUs |
| **Reusable Hardware Snippets** | Supplies reusable overlays and configurations for CAN, USB, Ethernet, display, video, storage, and other peripherals |
| **Twister Validation Matrix** | Maintains board-specific sample and subsystem test coverage for supported hardware features |
| **Flexible Toolchains and Debuggers** | Supports Zephyr SDK, ZCC, generic cross-compilers, and selectable J-Link/OpenOCD runners |
| **Docker Support** | Ready-to-use Docker development environment |

---

## 🎯 Supported Boards

<table>
  <tr>
    <th>Board</th>
    <th>SoC</th>
    <th>Twister-covered Features</th>
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
    <td><img src="https://img.shields.io/badge/CAN-✓-success?style=flat-square" alt="CAN"> <img src="https://img.shields.io/badge/USB-✓-success?style=flat-square" alt="USB"> <img src="https://img.shields.io/badge/PWM-✓-success?style=flat-square" alt="PWM"> <img src="https://img.shields.io/badge/Ethernet-✓-success?style=flat-square" alt="Ethernet"> <img src="https://img.shields.io/badge/I2C-✓-success?style=flat-square" alt="I2C"></td>
  </tr>
  <tr>
    <td><b>hpm6200evk</b></td>
    <td>HPM6280</td>
    <td><img src="https://img.shields.io/badge/CAN-✓-success?style=flat-square" alt="CAN"> <img src="https://img.shields.io/badge/USB-✓-success?style=flat-square" alt="USB"> <img src="https://img.shields.io/badge/PWM-✓-success?style=flat-square" alt="PWM"> <img src="https://img.shields.io/badge/RNG-✓-success?style=flat-square" alt="RNG"></td>
  </tr>
  <tr>
    <td><b>hpm6300evk</b></td>
    <td>HPM6360</td>
    <td><img src="https://img.shields.io/badge/CAN-✓-success?style=flat-square" alt="CAN"> <img src="https://img.shields.io/badge/USB-✓-success?style=flat-square" alt="USB"> <img src="https://img.shields.io/badge/PWM-✓-success?style=flat-square" alt="PWM"> <img src="https://img.shields.io/badge/Ethernet-✓-success?style=flat-square" alt="Ethernet"> <img src="https://img.shields.io/badge/SDHC-✓-success?style=flat-square" alt="SDHC"> <img src="https://img.shields.io/badge/RNG-✓-success?style=flat-square" alt="RNG"></td>
  </tr>
  <tr>
    <td><b>hpm6750evk2</b></td>
    <td>HPM6750</td>
    <td><img src="https://img.shields.io/badge/CAN-✓-success?style=flat-square" alt="CAN"> <img src="https://img.shields.io/badge/USB-✓-success?style=flat-square" alt="USB"> <img src="https://img.shields.io/badge/PWM-✓-success?style=flat-square" alt="PWM"> <img src="https://img.shields.io/badge/RNG-✓-success?style=flat-square" alt="RNG"> <img src="https://img.shields.io/badge/I2C-✓-success?style=flat-square" alt="I2C"> <img src="https://img.shields.io/badge/Ethernet-✓-success?style=flat-square" alt="Ethernet"> <img src="https://img.shields.io/badge/Display-RGB-success?style=flat-square" alt="RGB Display"> <img src="https://img.shields.io/badge/Video-DVP-success?style=flat-square" alt="DVP Video"> <img src="https://img.shields.io/badge/SDHC-✓-success?style=flat-square" alt="SDHC"></td>
  </tr>
  <tr>
    <td><b>hpm6800evk</b></td>
    <td>HPM6880</td>
    <td><img src="https://img.shields.io/badge/CAN-✓-success?style=flat-square" alt="CAN"> <img src="https://img.shields.io/badge/USB-✓-success?style=flat-square" alt="USB"> <img src="https://img.shields.io/badge/RNG-✓-success?style=flat-square" alt="RNG"> <img src="https://img.shields.io/badge/I2C-✓-success?style=flat-square" alt="I2C"> <img src="https://img.shields.io/badge/Ethernet-✓-success?style=flat-square" alt="Ethernet"> <img src="https://img.shields.io/badge/Display-RGB%2FMIPI%2FLVDS-success?style=flat-square" alt="Display"> <img src="https://img.shields.io/badge/Video-DVP%2FMIPI-success?style=flat-square" alt="Video"> <img src="https://img.shields.io/badge/SDHC-✓-success?style=flat-square" alt="SDHC"></td>
  </tr>
  <tr>
    <td><b>hpm6e00evk</b></td>
    <td>HPM6E80</td>
    <td><img src="https://img.shields.io/badge/CAN-✓-success?style=flat-square" alt="CAN"> <img src="https://img.shields.io/badge/USB-✓-success?style=flat-square" alt="USB"> <img src="https://img.shields.io/badge/PWM-✓-success?style=flat-square" alt="PWM"> <img src="https://img.shields.io/badge/RNG-✓-success?style=flat-square" alt="RNG"> <img src="https://img.shields.io/badge/I2C-✓-success?style=flat-square" alt="I2C"> <img src="https://img.shields.io/badge/Ethernet-✓-success?style=flat-square" alt="Ethernet"></td>
  </tr>
  <tr>
    <td><b>hpm6p00evk</b></td>
    <td>HPM6P81</td>
    <td><img src="https://img.shields.io/badge/CAN-✓-success?style=flat-square" alt="CAN"> <img src="https://img.shields.io/badge/USB-✓-success?style=flat-square" alt="USB"> <img src="https://img.shields.io/badge/PWM-✓-success?style=flat-square" alt="PWM"> <img src="https://img.shields.io/badge/Ethernet-✓-success?style=flat-square" alt="Ethernet"> <img src="https://img.shields.io/badge/RNG-✓-success?style=flat-square" alt="RNG"></td>
  </tr>
</table>

---

## 🔌 Supported Peripherals

| Peripheral | Driver | Description |
|:----------:|:------:|-------------|
| ADC | `adc` | ADC12 and ADC16 support |
| CAN | `can` | CAN and CAN FD support |
| Counter | `counter` | GPTMR counter, alarm, and top-value support |
| Display | `display` | RGB, MIPI DSI, LVDS, and PixelMux display pipelines |
| Entropy | `entropy` | Hardware random number generator |
| Ethernet | `ethernet` | Ethernet MAC support |
| Flash | `flash` | XPI NOR flash read, write, and erase operations |
| I2C | `i2c` | I2C controller support |
| Pinctrl | `pinctrl` | IOC pin multiplexing and electrical configuration |
| PWM | `pwm` | PWM and PWMv2 support |
| SDHC | `sdhc` | SD/MMC host controller support |
| SPI | `spi` | SPI controller with optional DMA transfers |
| UART | `serial` | UART polling, interrupt, and asynchronous operation |
| USB | `usb` | Zephyr UDC plus CherryUSB device/host integration |
| Video | `video` | Camera/DVP and MIPI CSI capture pipelines |
| Watchdog | `watchdog` | WDG and EWDG watchdog controllers |


## 🚀 Quick Start

### Prerequisites

- Zephyr SDK 1.0.1
- Python 3.12+
- CMake 3.20.5+
- west 0.14.0+
- Ninja build system

### Build with West

```bash
# Initialize workspace
west init -m https://github.com/hpmicro/zephyr_sdk_glue.git zephyr_hpmicro
cd zephyr_hpmicro
west update
west supply

# Build hello_world example
west build -p always -b hpm6750evk2 zephyr/samples/hello_world

# Flash to board
west flash
```

### Build with CMake

```bash
cmake -GNinja -B build -DBOARD=hpm6750evk2 zephyr/samples/hello_world
ninja -C build
```

> Use `-S ${snippet}` to specify hardware configuration snippets for specific samples.

---

## 📁 Directory Structure

```
sdk_glue/
├── 📂 boards/              # HPMicro board definitions and shared runner support
├── 📂 cmake/               # Compiler, linker, and toolchain extensions
├── 📂 docs/                # English and Chinese documentation
├── 📂 drivers/             # HPMicro Zephyr peripheral drivers
├── 📂 dts/                 # Devicetree bindings and HPMicro SoC descriptions
├── 📂 include/             # Public driver and devicetree headers
├── 📂 modules/hpm_sdk/     # HPM_SDK module integration
├── 📂 samples/             # sdk_glue-specific sample applications
├── 📂 scripts/             # West commands, SDK patches, and validation utilities
├── 📂 snippets/            # Reusable board and peripheral configurations
├── 📂 soc/                 # HPMicro SoC initialization and linker support
├── 📂 tests/               # sdk_glue-specific tests
├── 📂 twister/             # HPMicro sample and test adaptation metadata
├── 📂 zephyr/              # Zephyr module registration
├── west.yml                # Primary West manifest
├── west_gitee.yml          # Gitee-oriented West manifest
└── docker-compose.yaml     # Docker development environment
```

---

## 📚 Documentation

| Document | Description |
|----------|-------------|
| 📖 [Linux Getting Started](docs/source/en/starting/linux.rst) | Build environment setup on Linux |
| 📖 [Windows Getting Started](docs/source/en/starting/windows.rst) | Build environment setup on Windows |
| 🐳 [Docker Getting Started](docs/source/en/starting/docker.rst) | Using Docker development environment |
| 🔧 [Toolchain Guide](docs/source/en/starting/toolchain.rst) | Toolchain configuration (Zephyr SDK / ZCC) |

---

## 📄 License

This project is licensed under the **Apache License 2.0** - see the [LICENSE](LICENSE) file for details.

---
