.. Copyright (c) 2026 HPMicro
.. SPDX-License-Identifier: Apache-2.0

.. _supported_samples_en:

Board Supported Matrix
=======================

The following samples have been verified on supported development boards.

Table legend:

- **Sample**: Sample path
- **Snippet**: Snippet required to run the sample ("-" means not required)
- **Board columns**: "yes" means the sample is supported and has been build-verified on the board; "-" means unsupported.

.. list-table::
   :header-rows: 1
   :widths: 30 18 10 10 10 10 10 10 10 10 10 10
   :class: verified-samples-table

   * - Sample
     - Snippet
     - 6750evk2
     - 6800evk
     - 6200evk
     - 6e00evk
     - 5300evk
     - 5100evk
     - 5301evklite
     - 5e00evk
     - 6300evk
     - 6p00evk
   * - samples/hello_world
     - \-
     - yes
     - yes
     - yes
     - yes
     - yes
     - yes
     - yes
     - yes
     - yes
     - yes
   * - samples/basic/blinky
     - blinky
     - yes
     - yes
     - yes
     - yes
     - yes
     - yes
     - yes
     - yes
     - yes
     - yes
   * - samples/basic/blinky_pwm
     - blinky_pwm
     - yes
     - \-
     - yes
     - yes
     - yes
     - yes
     - \-
     - yes
     - yes
     - yes
   * - samples/basic/button
     - \-
     - yes
     - yes
     - yes
     - yes
     - yes
     - yes
     - yes
     - yes
     - yes
     - yes
   * - samples/drivers/eeprom
     - i2c_eeprom
     - yes
     - yes
     - \-
     - yes
     - \-
     - \-
     - \-
     - \-
     - \-
     - \-
   * - samples/drivers/display
     - display_rgb
     - yes
     - yes
     - \-
     - \-
     - \-
     - \-
     - \-
     - \-
     - \-
     - \-
   * - samples/drivers/display
     - display_mipi
     - \-
     - yes
     - \-
     - \-
     - \-
     - \-
     - \-
     - \-
     - \-
     - \-
   * - samples/drivers/display
     - display_lvds
     - \-
     - yes
     - \-
     - \-
     - \-
     - \-
     - \-
     - \-
     - \-
     - \-
   * - samples/drivers/display
     - display_dual_lvds
     - \-
     - yes
     - \-
     - \-
     - \-
     - \-
     - \-
     - \-
     - \-
     - \-
   * - samples/drivers/video/capture
     - video_dvp
     - yes
     - yes
     - \-
     - \-
     - \-
     - \-
     - \-
     - \-
     - \-
     - \-
   * - samples/drivers/video/capture
     - video_mipi
     - \-
     - yes
     - \-
     - \-
     - \-
     - \-
     - \-
     - \-
     - \-
     - \-
   * - samples/subsys/usb/cdc_acm
     - cdc_acm
     - yes
     - yes
     - yes
     - yes
     - yes
     - yes
     - yes
     - yes
     - yes
     - yes
   * - samples/subsys/usb/hid-keyboard
     - hid-keyboard
     - yes
     - yes
     - yes
     - yes
     - yes
     - yes
     - yes
     - yes
     - yes
     - yes
   * - samples/subsys/usb/hid-mouse
     - hid-mouse
     - yes
     - yes
     - yes
     - yes
     - yes
     - yes
     - yes
     - yes
     - yes
     - yes
   * - samples/subsys/usb/mass
     - mass
     - yes
     - yes
     - yes
     - yes
     - yes
     - yes
     - yes
     - yes
     - yes
     - yes
   * - samples/net/sockets/echo_server
     - ethernet
     - yes
     - yes
     - \-
     - yes
     - \-
     - \-
     - \-
     - yes
     - yes
     - yes
   * - samples/subsys/shell/shell_module
     - shell_module
     - yes
     - yes
     - yes
     - yes
     - yes
     - yes
     - yes
     - yes
     - yes
     - yes
   * - samples/modules/canopennode
     - canopennode
     - yes
     - yes
     - yes
     - yes
     - yes
     - yes
     - \-
     - yes
     - yes
     - yes
   * - samples/drivers/watchdog
     - watchdog
     - yes
     - yes
     - yes
     - yes
     - yes
     - yes
     - yes
     - yes
     - yes
     - yes
   * - tests/drivers/counter/counter_basic_api
     - counter
     - yes
     - yes
     - yes
     - yes
     - yes
     - yes
     - yes
     - yes
     - yes
     - yes
   * - tests/drivers/can/api
     - can
     - yes
     - yes
     - yes
     - yes
     - yes
     - yes
     - \-
     - yes
     - yes
     - yes
   * - tests/drivers/can/timing
     - can
     - yes
     - yes
     - yes
     - yes
     - yes
     - yes
     - \-
     - yes
     - yes
     - yes
   * - tests/drivers/can/shell
     - can
     - yes
     - yes
     - yes
     - yes
     - yes
     - yes
     - \-
     - yes
     - yes
     - yes
   * - tests/drivers/uart/uart_basic_api
     - \-
     - yes
     - yes
     - yes
     - yes
     - yes
     - yes
     - yes
     - yes
     - yes
     - yes
   * - tests/drivers/entropy/api
     - entropy
     - yes
     - yes
     - yes
     - yes
     - yes
     - \-
     - \-
     - \-
     - yes
     - yes
   * - tests/drivers/hwinfo/api
     - hwinfo
     - yes
     - yes
     - yes
     - yes
     - yes
     - yes
     - yes
     - yes
     - yes
     - yes
   * - tests/kernel/cache
     - cache
     - yes
     - yes
     - yes
     - yes
     - yes
     - yes
     - yes
     - yes
     - yes
     - yes
   * - tests/drivers/sdhc
     - sdhc
     - yes
     - yes
     - \-
     - \-
     - \-
     - \-
     - \-
     - \-
     - yes
     - \-
   * - tests/drivers/disk/disk_access
     - sdhc
     - yes
     - yes
     - \-
     - \-
     - \-
     - \-
     - \-
     - \-
     - yes
     - \-
   * - tests/drivers/disk/disk_performance
     - sdhc
     - yes
     - yes
     - \-
     - \-
     - \-
     - \-
     - \-
     - \-
     - yes
     - \-
   * - tests/subsys/sd/sdmmc
     - sdhc
     - yes
     - yes
     - \-
     - \-
     - \-
     - \-
     - \-
     - \-
     - yes
     - \-
   * - tests/subsys/sd/mmc
     - sdhc
     - yes
     - yes
     - \-
     - \-
     - \-
     - \-
     - \-
     - \-
     - yes
     - \-
   * - tests/clock_tree
     - \-
     - yes
     - yes
     - yes
     - yes
     - yes
     - yes
     - yes
     - yes
     - yes
     - yes
   * - cherryusb/device/msc/ram_disk
     - \-
     - yes
     - yes
     - yes
     - yes
     - yes
     - yes
     - yes
     - yes
     - yes
     - yes
   * - cherryusb/device/cdc_acm/cdc_acm_vcom
     - \-
     - yes
     - yes
     - yes
     - yes
     - yes
     - yes
     - yes
     - yes
     - yes
     - yes
   * - cherryusb/host/usb_host
     - \-
     - yes
     - yes
     - yes
     - yes
     - yes
     - yes
     - yes
     - yes
     - yes
     - yes
