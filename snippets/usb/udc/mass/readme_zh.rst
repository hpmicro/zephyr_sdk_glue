.. _usb_udc_mass_zh:

USB 大容量存储
===============

概述
-----

本示例使用 HPMicro USB 设备控制器，将 FAT 格式的 RAM 盘枚举为 USB 大容量存储设备。

* `Zephyr 4.4.0 USB Mass Storage sample <https://docs.zephyrproject.org/4.4.0/samples/subsys/usb/mass/README.html>`_

示例路径
---------

.. code-block:: text

   zephyr/samples/subsys/usb/mass

硬件准备
---------

将开发板配置的 USB Device 接口连接到主机，同时保留调试串口连接以查看日志。

构建
-----

在 west 工作区根目录执行。以下命令以 ``hpm6750evk2`` 为例：

.. code-block:: console

   west build -p always -b hpm6750evk2 -S mass zephyr/samples/subsys/usb/mass

预期现象
---------

烧录并重新连接 USB 后，控制台输出 ``The device is put in USB mass storage mode``。主机枚举出一个由 snippet 配置的 FAT RAM 盘，可像临时 U 盘一样挂载和读写；开发板复位或断电后，盘中内容会丢失。

注意事项
---------

.. note::

   必须使用开发板 overlay 选择的 USB 接口以及支持数据传输的 USB 线，只有供电功能的线缆无法完成枚举。
