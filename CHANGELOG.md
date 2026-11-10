# Change Log

## [0.8.0]: 2026.11.9

All changes since 0.7.0

### Added:
- [soc]: add support for the HPM51xx, HPM53xx, HPM5Exx, HPM63xx, and HPM6Pxx series, with initial board targets based on HPM5151, HPM5301, HPM5361, HPM5E31, HPM6360, and HPM6P81
- [boards]: add support for hpm5100evk, hpm5300evk, hpm5301evklite, hpm5e00evk, hpm6300evk, and hpm6p00evk
- [runners]: add J-Link flash and debug runner support for HPMicro EVKs, including configurable device, interface speed, Commander, and GDB Server paths alongside the existing OpenOCD runner
- [drivers]: add cache management, GPTMR counter, hardware information, entropy RNG, WDG, and EWDG support
- [twister]: add HPMicro adaptations for 15 upstream Zephyr samples and 13 upstream tests, together with snippet-based board configurations and CherryUSB test metadata
- [tests]: add the clock_tree runtime diagnostic to report PLL outputs, CPU frequencies, and enabled peripheral clocks

### Changed:
- [manifest]: update the Zephyr baseline from v3.7.0 to v4.4.0 and HPM SDK to v1.13.0; update LVGL to v9.5.0, CherryUSB to v1.6.1 or later, and refresh FatFS, MCUboot, Mbed TLS, and the remaining west dependencies
- [toolchain]: update the default Zephyr SDK toolchain to v1.0.1, port ZCC compiler and linker integration to Zephyr v4.4.0, and enable ZCC in the HPMicro board metadata
- [soc]: align RISC-V CPU and timer metadata, ROM startup, pinned linker sections, retained watchdog memory, shared SRAM, DTCM, and non-cacheable memory layouts with Zephyr v4.4.0
- [devicetree]: reorganize SoC descriptions into per-series and per-model layers and normalize device resources, memory regions, pinctrl, clock topology, provider cells, and MIPI/display phandle relationships across new and existing HPMicro families
- [drivers]: port ADC, CAN/MCAN, display, Ethernet, flash, GPIO, I2C, pinctrl, PWM, SDHC, UART, SPI, USB UDC/CherryUSB, and video/camera drivers to the Zephyr v4.4 APIs and the expanded SoC set
- [clock]: add explicit clock ownership, move peripheral clock enablement into individual driver initialization, retain only global enables at SoC startup, consolidate PLL/PLLv2 handling, and refresh clock bindings and initialization for every supported SoC
- [boards]: align board metadata with the Zephyr v4.4 schema and Twister, centralize J-Link and OpenOCD runner configuration, update OpenOCD target and probe settings, and enable optimized builds
- [usb]: adapt the UDC driver to the Zephyr v4.4 control-transfer model, add remote wakeup and board-control support, and make the UDC snippets self-contained
- [samples]: extend verified snippets to the new boards and replace the separate CherryUSB host samples with a unified demo supporting multiple CDC, HID, and mass-storage devices
- [scripts]: update the west supply workflow and clock-header generation for the revised manifests and new SoC families
- [docs]: refresh the getting-started guides, expand the bilingual snippet and clock_tree documentation, and update the Sphinx staging, navigation, and build workflows

### Fixed:
- [drivers]: fix CAN/MCAN filtering and instance selection, SDHC/SDIO operation, GPIO both-edge interrupts, watchdog reset handling, per-instance Ethernet clocks, MIPI DSI lane configuration, and USB mass-storage and control-transfer behavior
- [boards]: correct runner paths, partition overlays, GPIO key pinctrl, Ethernet PHY and clock routing, display pixelmux and DDR initialization, USB overlays, and EEPROM I2C routing

## [0.7.0]: 2026.1.30

All changes since 0.6.0

### Added:
- [boards]: add hpm6200evk board support
- [boards]: add hpm6e00evk board support
- [toolchain]: add zcc toolchain support
- [docker]: add Docker image for development environment
- [bootloader]: add MCUboot support

### Changed:
- update pwm drivers include pwm and pwmv2
- update sdk_env to v1.11.0

## [0.6.0]: 2025.8.4

All changes since 0.5.0

### Changed:
- update all soc clock name and value
- update patch for sdk_env v1.10.0
- change SOC file structure
- update openocd runner scripts
- update sdk_env to v1.10.0

## [0.5.0]: 2025.5.25

All changes since 0.2.0

### Fixed:
  - [fix] defconfig: fix nocache memory size not aligned by power of 2

### Added:
  - [add] drivers: usb device: add udc driver
  - [add] drivers: add cherryusb device driver
  - [add] drivers: add cherryusb host driver
  - [add] driver: add ethernet driver adaptaion layer
  - [add] samples: add cherryusb device cdc acm vcom sample
  - [add] samples: add cherryusb device sample: msc: ram disk
  - [add] samples: add cherryusb host cdc acm sample
  - [add] samples: cherryusb: add msc disk sample 

## [0.2.0]: 2024.11.12

All changes since 0.1.0

### Changed:
  - soc: use common nocache.ld
  - soc: boot_header offset defined in soc instead of boards
  - soc: modify common common linker file
  - boards: clean hpm6750evk2 and hpm6800evk config tree
  - kconfig: update to kconfigv2 model
  - drivers: add display drivers
  - drivers: add sdio drivers
  - drivers: add camera drivers
  - drivers: add mcan drivers

### Fixed:
  - drivers: put sw_isr_table in RAM region
  - drivers: put ISR function in ITCM region

### Added:
  - boards: add shields board support
  - snippets: add sample snippets support
  - west: adapt for zephyr v3.7.0

## [0.1.0]: 2024.7.31

### Added:
  - scripts: add generate clock name script 
  - docs: add doc system
  - drivers: add adc12 and adc16 drivers
  - drivers: add spi driver
  - drivers: add can driver
  - drivers: add dma driver 
  - drivers: add serial driver
  - drivers: add pinctrl and gpio drivers
  - boards: add boards flash and debug supoort
  - boards: add hpm6750evk2 and hpm6800evk support
  - sdk_glue: add version file
  - soc: add hpm67xx series and hpm68xx series kconfig
  - dts: add hpm67xx series and hpm68xx series dts map
  - sdk_glue: add hpm_sdk in zephyr build
  - west: add west import system, zephyr v3.4.0
