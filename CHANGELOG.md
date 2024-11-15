# Change Log

## [0.2.0] - 2024.11.12:

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

### Verified sample:
- hpm6750evk2:
  - zephyr/samples/hello_world
  - zephyr/samples/basic/blinky: blinky
  - zephyr/samples/basic/blinky_pwm: blinky_pwm
  - zephyr/samples/basic/button
  - zephyr/samples/drivers/eeprom: i2c_eeprom
  - zephyr/samples/modules/canopennode: canopennode
  - zephyr/tests/drivers/can/api: can
  - zephyr/tests/drivers/can/timing: can
  - zephyr/tests/drivers/can/shell: can
  - zephyr/tests/drivers/uart/uart_basic_api
  - zephyr/samples/drivers/display: display_rgb
  - zephyr/samples/subsys/video/capture: video_dvp
  - zephyr/tests/drivers/sdhc: sdhc
  - zephyr/tests/drivers/disk/disk_access: sdhc
  - zephyr/tests/drivers/disk/disk_performance: sdhc
  - zephyr/tests/subsys/sd/sdmmc: sdhc

- hpm6800evk:
  - zephyr/samples/hello_world
  - zephyr/samples/basic/blinky: blinky
  - zephyr/samples/basic/button
  - zephyr/samples/drivers/eeprom: i2c_eeprom
  - zephyr/samples/subsys/video/capture: video_dvp, video_mipi
  - zephyr/samples/modules/canopennode: canopennode
  - zephyr/tests/drivers/can/api: can
  - zephyr/tests/drivers/can/timing: can
  - zephyr/tests/drivers/can/shell: can
  - zephyr/tests/drivers/uart/uart_basic_api
  - zephyr/samples/drivers/display: display_rgb, display_mipi, display_lvds, display_dual_lvds
  - zephyr/tests/drivers/sdhc: sdhc
  - zephyr/tests/drivers/disk/disk_access: sdhc
  - zephyr/tests/drivers/disk/disk_performance: sdhc
  - zephyr/tests/subsys/sd/sdmmc: sdhc

## [0.1.0]:

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

