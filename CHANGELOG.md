# Change Log

## [0.2.0] - 2024-:

All changes since 0.1.0

### Changed:
  - soc: use common nocache.ld
  - soc: boot_header offset defined in soc instead of boards
  - soc: modify common common linker file
  - boards: clean hpm6750evk2 and hpm6800evk config tree
  - kconfig: update to kconfigv2 model

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
  - zephyr/samples/basic/blinky
  - zephyr/samples/basic/blinky_pwm
  - zephyr/samples/basic/button
  - zephyr/samples/drivers/eeprom
  - zephyr/samples/modules/canopennode
  - zephyr/tests/drivers/can/api
  - zephyr/tests/drivers/can/timing
  - zephyr/tests/drivers/can/shell
  - zephyr/tests/drivers/uart/uart_basic_api

- hpm6800evk:
  - zephyr/samples/hello_world
  - zephyr/samples/basic/blinky
  - zephyr/samples/basic/button
  - zephyr/samples/drivers/eeprom
  - zephyr/subsys/video/capture
  - zephyr/samples/modules/canopennode
  - zephyr/tests/drivers/can/api
  - zephyr/tests/drivers/can/timing
  - zephyr/tests/drivers/can/shell
  - zephyr/tests/drivers/uart/uart_basic_api  

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

