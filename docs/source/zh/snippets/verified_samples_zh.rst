.. Copyright (c) 2026 HPMicro
.. SPDX-License-Identifier: Apache-2.0

.. _supported_samples_zh:

开发板支持矩阵
===============

以下示例已经在支持的开发板上完成 build 验证。

表格说明：

- **示例**：示例路径
- **Snippet**：运行示例需要的 snippet，"-" 表示不需要
- **开发板列**："yes" 表示该示例受支持且已在开发板上完成 build 验证，"-" 表示不支持

.. list-table::
   :header-rows: 1
   :widths: 30 18 10 10 10 10 10 10 10 10 10 10
   :class: verified-samples-table

   * - 示例
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
