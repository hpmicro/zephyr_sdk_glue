.. _usb_host_demo:

CherryUSB Host 统一示例
=======================

概述
----

本示例包含 CherryUSB 统一 Host demo 的 Zephyr 适配版本，在一个 Host
示例中支持 USB Serial（CDC ACM、CH340、CP210x、FTDI、PL2303）、HID
和 MSC 设备。

编译
----

在 ``sdk_glue`` 目录下，以 HPM6750 EVK2 为例执行：

.. code-block:: console

   west build -p always -b hpm6750evk2 samples/cherryusb/host/usb_host

硬件连接
--------

将 USB 设备连接到开发板 USB0 Host 接口，支持的 USB 设备类型如下：

* Serial: 串口设备（可用 `hpm_sdk/samples/cherryusb/device/cdc_acm/cdc_acm_vcom` 示例来作为串口回环设备）
* HID: 鼠标、键盘设备
* MSC: FAT/FAT32 U盘设备

运行现象
--------

下载并运行程序后，Host 会自动枚举连接的 USB 设备，并根据设备类型执行相应测试：

* Serial 串口设备，Host执行串口回环测试，串口收发数据一致时打印测试成功信息。
* HID 鼠标、键盘设备，Host收到输入报告时打印报告数据。
* MSC FAT/FAT32 U盘设备，Host会挂载U盘、打印容量并列出根目录内容。
