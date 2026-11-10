.. _usb_udc_hid-keyboard_zh:

USB HID 键盘
=============

概述
-----

本示例使用 HPMicro USB 设备控制器，将开发板输入按键模拟为 HID 键盘。

* `Zephyr 4.4.0 USB HID Keyboard sample <https://docs.zephyrproject.org/4.4.0/samples/subsys/usb/hid-keyboard/README.html>`_

示例路径
---------

.. code-block:: text

   zephyr/samples/subsys/usb/hid-keyboard

硬件准备
---------

将开发板配置的 USB Device 接口连接到主机，同时保留调试串口连接以查看日志。

构建
-----

在 west 工作区根目录执行。以下命令以 ``hpm6750evk2`` 为例：

.. code-block:: console

   west build -p always -b hpm6750evk2 -S hid-keyboard zephyr/samples/subsys/usb/hid-keyboard

预期现象
---------

烧录并重新连接 USB 后，设备枚举为键盘。按下开发板的第一个用户按键会发送 Num Lock 键值；主机切换 Num Lock 状态，``led0`` 随该状态变化，因此连续按键时 LED 会翻转。若开发板提供更多输入按键，后续按键依次发送 Caps Lock、Scroll Lock 和 ``Right Alt+1+2+3``。

注意事项
---------

.. note::

   必须使用开发板 overlay 选择的 USB 接口以及支持数据传输的 USB 线，只有供电功能的线缆无法完成枚举。
