.. _usb_udc_cdc_acm_zh:

USB CDC ACM
============

概述
-----

本示例使用 HPMicro USB 设备控制器提供 CDC ACM 虚拟串口，并回显主机发送的数据。

* `Zephyr 4.4.0 USB CDC ACM sample <https://docs.zephyrproject.org/4.4.0/samples/subsys/usb/cdc_acm/README.html>`_

示例路径
---------

.. code-block:: text

   zephyr/samples/subsys/usb/cdc_acm

硬件准备
---------

将开发板配置的 USB Device 接口连接到主机，同时保留调试串口连接以查看日志。

构建
-----

在 west 工作区根目录执行。以下命令以 ``hpm6750evk2`` 为例：

.. code-block:: console

   west build -p always -b hpm6750evk2 -S cdc_acm zephyr/samples/subsys/usb/cdc_acm

预期现象
---------

烧录并重新连接 USB 后，设备枚举为虚拟串口。使用串口终端打开该端口后，主机发送的每个字符都会原样回显到同一端口。

注意事项
---------

.. note::

   必须使用开发板 overlay 选择的 USB 接口以及支持数据传输的 USB 线，只有供电功能的线缆无法完成枚举。
