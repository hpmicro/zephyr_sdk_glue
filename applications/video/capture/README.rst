.. _video_capture-sample:

Video Capture
#############

Description
***********

This sample application uses the Video API to retrieve video frames from the
video capture device, writes a frame count message to the console, and then
discards the video frame data, and the captured image will be shown on the display

Requirements
************

This sample requires a video capture device (e.g. a camera).

- :ref:`hpm6800evk`
- `ov5640 camera(mipi csi) module`_
- `ov7725 camera(dvp) module`_

Wiring
******

On :ref:`hpm6800evk`, if use DVP, the ov7725 camera module should be plugged in the
J12 camera connector; if use MIPI CSI, the ov5640 camera module should be plugged in the Expansion Board J6  camera connector

Building and Running
********************

For :ref:`hpm6800evk`, build this sample application with the following commands:

.. US DVP camera:
.. zephyr-app-commands:: west build -p always -b hpm6800evk -S video_dvp applications/video/capture/

.. US MIPI CSI camera:
.. zephyr-app-commands:: west build -p always -b hpm6800evk -S video_mipi applications/video/capture/

Sample Output
=============

on MIPI CSI:

.. code-block:: console

      mipi csi clk freq: 13813953 Hz
      [00:00:00.001,000] <inf> hpm_camera: hpmicro cam driver interface: mipi

      [00:00:00.135,000] <dbg> display_hpm_mipi: hpm_display_init: interface: mipi-dsi
      [00:00:00.135,000] <dbg> display_hpm_mipi: hpm_display_init: mipi_base     = 0xf1020000
      [00:00:00.135,000] <dbg> display_hpm_mipi: hpm_display_init: mipi_phy_base = 0xf4140000
      [00:00:00.135,000] <dbg> display_hpm_mipi: hpm_display_init: pixelmux_base = 0xf1034000
      [00:00:00.135,000] <dbg> display_hpm_mipi: hpm_display_init: id            = 0
      [00:00:00.135,000] <dbg> display_hpm_mipi: hpm_display_init: lcdc info:
      [00:00:00.135,000] <dbg> display_hpm_mipi: hpm_display_init: id            = 0
      [00:00:00.135,000] <dbg> display_hpm_mipi: hpm_display_init: lcdc_base     = 0xf1000000
      [00:00:00.135,000] <dbg> display_hpm_mipi: hpm_display_init: pixelmux_base = 0xf1034000
      [00:00:00.135,000] <dbg> display_hpm_mipi: hpm_display_init: panel info:
      [00:00:00.135,000] <dbg> display_hpm_mipi: hpm_display_init: panel_name      = mc10128007
      [00:00:00.135,000] <dbg> display_hpm_mipi: hpm_display_init: pixel_clock_khz = 60000
      --- 1 messages dropped ---
      [00:00:00.135,000] <dbg> display_hpm_mipi: hpm_display_init: hactive         = 800
      [00:00:00.135,000] <dbg> display_hpm_mipi: hpm_display_init: hfront_porch    = 52
      [00:00:00.135,000] <dbg> display_hpm_mipi: hpm_display_init: hback_porch     = 48
      [00:00:00.135,000] <dbg> display_hpm_mipi: hpm_display_init: hsync_len       = 8
      [00:00:00.135,000] <dbg> display_hpm_mipi: hpm_display_init: vactive         = 1280
      [00:00:00.135,000] <dbg> display_hpm_mipi: hpm_display_init: vfront_porch    = 15
      [00:00:00.135,000] <dbg> display_hpm_mipi: hpm_display_init: vback_porch     = 16
      [00:00:00.135,000] <dbg> display_hpm_mipi: hpm_display_init: vsync_len       = 6
      [00:00:00.135,000] <dbg> display_hpm_mipi: hpm_display_init: hsync_pol       = 0
      [00:00:00.135,000] <dbg> display_hpm_mipi: hpm_display_init: vsync_pol       = 0
      [00:00:00.135,000] <dbg> display_hpm_mipi: hpm_display_init: de_pol          = 0
      [00:00:00.135,000] <dbg> display_hpm_mipi: hpm_display_init: pixel_clk_pol   = 0
      [00:00:00.135,000] <dbg> display_hpm_mipi: hpm_display_init: pixel_data_pol  = 0
      [00:00:00.135,000] <dbg> display_hpm_mipi: hpm_display_init: mipi_lane_num  = 4
      [00:00:00.135,000] <dbg> display_hpm_mipi: hpm_display_init: mipi_lane_speed  = 500
      [00:00:00.135,000] <dbg> display_hpm_mipi: hpm_display_init: mipi_cmd_len     = 774
      [00:00:00.135,000] <inf> display_hpm_mipi: pll clk        : 594000 KHz
      [00:00:00.135,000] <inf> display_hpm_mipi: lcdc pixel clk : 59400 KHz
      [00:00:00.135,000] <inf> display_hpm_mipi: panel pixel clk: 60000 KHz
      [00:00:00.135,000] <dbg> display_hpm: hpm_display_gpio_init: mc10128007: gpio
      [00:00:00.135,000] <dbg> display_hpm: hpm_display_gpio_init: mc10128007: no cfg_gpio
      [00:00:00.135,000] <dbg> display_hpm: hpm_display_gpio_init: mc10128007: no backlight_gpio
      [00:00:00.276,000] <inf> display_hpm: mc10128007: alloc fb(addr, size): 0x4bb897c0, 4096000
      *** Booting Zephyr OS build 36940db938a8 ***
      - Device name: video0
      - Capabilities:
      RGBP width [1280; 1280; 0] height [720; 720; 0]
      YUYV width [1280; 1280; 0] height [720; 720; 0]
      RGBP width [640; 640; 0] height [480; 480; 0]
      YUYV width [640; 640; 0] height [480; 480; 0]
      - Default format: RGBP 640x480
      Capture started
      Got frame 0! size: 1228800; timestamp 461 ms
      Got frame 1! size: 1228800; timestamp 493 ms
      Got frame 2! size: 1228800; timestamp 527 ms
      Got frame 3! size: 1228800; timestamp 560 ms
      Got frame 4! size: 1228800; timestamp 593 ms
      Got frame 5! size: 1228800; timestamp 660 ms
      Got frame 6! size: 1228800; timestamp 727 ms
      Got frame 7! size: 1228800; timestamp 760 ms
      Got frame 8! size: 1228800; timestamp 793 ms
      Got frame 9! size: 1228800; timestamp 860 ms
      Got frame 10! size: 1228800; timestamp 893 ms
      Got frame 11! size: 1228800; timestamp 927 ms
      Got frame 12! size: 1228800; timestamp 960 ms
      Got frame 13! size: 1228800; timestamp 993 ms
      Got frame 14! size: 1228800; timestamp 1027 ms
      Got frame 15! size: 1228800; timestamp 1060 ms
      Got frame 16! size: 1228800; timestamp 1093 ms
      Got frame 17! size: 1228800; timestamp 1126 ms
      Got frame 18! size: 1228800; timestamp 1160 ms
      Got frame 19! size: 1228800; timestamp 1193 ms

   <repeats endlessly>


on DVP:

.. code-block:: console

      [00:00:00.001,000] <inf> hpm_camera: hpmicro cam driver interface: dvp

      [00:00:00.039,000] <dbg> display_hpm_mipi: hpm_display_init: interface: mipi-dsi
      [00:00:00.039,000] <dbg> display_hpm_mipi: hpm_display_init: mipi_base     = 0xf1020000
      [00:00:00.039,000] <dbg> display_hpm_mipi: hpm_display_init: mipi_phy_base = 0xf4140000
      [00:00:00.039,000] <dbg> display_hpm_mipi: hpm_display_init: pixelmux_base = 0xf1034000
      [00:00:00.039,000] <dbg> display_hpm_mipi: hpm_display_init: id            = 0
      [00:00:00.039,000] <dbg> display_hpm_mipi: hpm_display_init: lcdc info:
      [00:00:00.039,000] <dbg> display_hpm_mipi: hpm_display_init: id            = 0
      [00:00:00.039,000] <dbg> display_hpm_mipi: hpm_display_init: lcdc_base     = 0xf1000000
      [00:00:00.039,000] <dbg> display_hpm_mipi: hpm_display_init: pixelmux_base = 0xf1034000
      [00:00:00.039,000] <dbg> display_hpm_mipi: hpm_display_init: panel info:
      [00:00:00.039,000] <dbg> display_hpm_mipi: hpm_display_init: panel_name      = mc10128007
      [00:00:00.039,000] <dbg> display_hpm_mipi: hpm_display_init: pixel_clock_khz = 60000
      --- 1 messages dropped ---
      [00:00:00.039,000] <dbg> display_hpm_mipi: hpm_display_init: hactive         = 800
      [00:00:00.039,000] <dbg> display_hpm_mipi: hpm_display_init: hfront_porch    = 52
      [00:00:00.039,000] <dbg> display_hpm_mipi: hpm_display_init: hback_porch     = 48
      [00:00:00.039,000] <dbg> display_hpm_mipi: hpm_display_init: hsync_len       = 8
      [00:00:00.039,000] <dbg> display_hpm_mipi: hpm_display_init: vactive         = 1280
      [00:00:00.039,000] <dbg> display_hpm_mipi: hpm_display_init: vfront_porch    = 15
      [00:00:00.039,000] <dbg> display_hpm_mipi: hpm_display_init: vback_porch     = 16
      [00:00:00.039,000] <dbg> display_hpm_mipi: hpm_display_init: vsync_len       = 6
      [00:00:00.039,000] <dbg> display_hpm_mipi: hpm_display_init: hsync_pol       = 0
      [00:00:00.039,000] <dbg> display_hpm_mipi: hpm_display_init: vsync_pol       = 0
      [00:00:00.039,000] <dbg> display_hpm_mipi: hpm_display_init: de_pol          = 0
      [00:00:00.039,000] <dbg> display_hpm_mipi: hpm_display_init: pixel_clk_pol   = 0
      [00:00:00.039,000] <dbg> display_hpm_mipi: hpm_display_init: pixel_data_pol  = 0
      [00:00:00.039,000] <dbg> display_hpm_mipi: hpm_display_init: mipi_lane_num  = 4
      [00:00:00.039,000] <dbg> display_hpm_mipi: hpm_display_init: mipi_lane_speed  = 500
      [00:00:00.039,000] <dbg> display_hpm_mipi: hpm_display_init: mipi_cmd_len     = 774
      [00:00:00.039,000] <inf> display_hpm_mipi: pll clk        : 594000 KHz
      [00:00:00.039,000] <inf> display_hpm_mipi: lcdc pixel clk : 59400 KHz
      [00:00:00.039,000] <inf> display_hpm_mipi: panel pixel clk: 60000 KHz
      [00:00:00.040,000] <dbg> display_hpm: hpm_display_gpio_init: mc10128007: gpio
      [00:00:00.040,000] <dbg> display_hpm: hpm_display_gpio_init: mc10128007: no cfg_gpio
      [00:00:00.040,000] <dbg> display_hpm: hpm_display_gpio_init: mc10128007: no backlight_gpio
      [00:00:00.180,000] <inf> display_hpm: mc10128007: alloc fb(addr, size): 0x4bb897c0, 2048000
      *** Booting Zephyr OS build 36940db938a8 ***
      - Device name: video1
      - Capabilities:
      RGBP width [640; 640; 0] height [480; 480; 0]
      - Default format: RGBP 640x480
      Capture started
      Got frame 0! size: 614400; timestamp 255 ms
      Got frame 1! size: 614400; timestamp 300 ms
      Got frame 2! size: 614400; timestamp 322 ms
      Got frame 3! size: 614400; timestamp 344 ms
      Got frame 4! size: 614400; timestamp 367 ms
      Got frame 5! size: 614400; timestamp 389 ms
      Got frame 6! size: 614400; timestamp 411 ms
      Got frame 7! size: 614400; timestamp 433 ms
      Got frame 8! size: 614400; timestamp 455 ms
      Got frame 9! size: 614400; timestamp 477 ms
      Got frame 10! size: 614400; timestamp 500 ms
      Got frame 11! size: 614400; timestamp 522 ms
      Got frame 12! size: 614400; timestamp 545 ms
      Got frame 13! size: 614400; timestamp 566 ms
      Got frame 14! size: 614400; timestamp 589 ms
      Got frame 15! size: 614400; timestamp 611 ms
      Got frame 16! size: 614400; timestamp 633 ms
      Got frame 17! size: 614400; timestamp 655 ms
      Got frame 18! size: 614400; timestamp 678 ms
      Got frame 19! size: 614400; timestamp 699 ms
      Got frame 20! size: 614400; timestamp 722 ms
      Got frame 21! size: 614400; timestamp 744 ms
      Got frame 22! size: 614400; timestamp 766 ms
      Got frame 23! size: 614400; timestamp 788 ms
      Got frame 24! size: 614400; timestamp 811 ms
      Got frame 25! size: 614400; timestamp 833 ms
      Got frame 26! size: 614400; timestamp 855 ms
      Got frame 27! size: 614400; timestamp 877 ms
      Got frame 28! size: 614400; timestamp 900 ms
      Got frame 29! size: 614400; timestamp 922 ms
      Got frame 30! size: 614400; timestamp 944 ms
      Got frame 31! size: 614400; timestamp 966 ms
      Got frame 32! size: 614400; timestamp 989 ms


   <repeats endlessly>

References
**********

.. _MT9M114 camera module: https://www.onsemi.com/PowerSolutions/product.do?id=MT9M114
