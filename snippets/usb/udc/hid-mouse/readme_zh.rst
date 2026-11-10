.. _usb_udc_hid-mouse_zh:

USB HID 鼠标
=============

概述
-----

本示例使用 HPMicro USB 设备控制器，将开发板输入按键模拟为 HID 鼠标按键和指针移动。

* `Zephyr 4.4.0 USB HID Mouse sample <https://docs.zephyrproject.org/4.4.0/samples/subsys/usb/hid-mouse/README.html>`_

示例路径
---------

.. code-block:: text

   zephyr/samples/subsys/usb/hid-mouse

硬件准备
---------

将开发板配置的 USB Device 接口连接到主机，同时保留调试串口连接以查看日志。

构建
-----

在 west 工作区根目录执行。以下命令以 ``hpm6750evk2`` 为例：

.. code-block:: console

   west build -p always -b hpm6750evk2 -S hid-mouse zephyr/samples/subsys/usb/hid-mouse

预期现象
---------

烧录并重新连接 USB 后，设备枚举为鼠标。按下第一个用户按键会发送鼠标左键报告，每次报告发送成功后 ``led0`` 翻转。若开发板提供更多输入按键，后续按键依次作为鼠标右键、使指针沿 X 轴移动 10 个单位、沿 Y 轴移动 10 个单位。

注意事项
---------

.. note::

   必须使用开发板 overlay 选择的 USB 接口以及支持数据传输的 USB 线，只有供电功能的线缆无法完成枚举。
