.. _display_dual_lvds_zh:

双通道 LVDS 显示
=================

概述
-----

本示例使用 Zephyr display API，通过 HPMicro 双通道 LVDS 接口和开发板配置的面板显示测试图案。

* `Zephyr 4.4.0 display sample <https://docs.zephyrproject.org/4.4.0/samples/drivers/display/README.html>`_

示例路径
---------

.. code-block:: text

   zephyr/samples/drivers/display

硬件准备
---------

连接开发板 overlay 对应的 LCD 面板，并确认电源、背光和信号排线连接正确。

构建
-----

在 west 工作区根目录执行。以下命令以 ``hpm6800evk`` 为例：

.. code-block:: console

   west build -p always -b hpm6800evk -S display_dual_lvds zephyr/samples/drivers/display

预期现象
---------

屏幕四角按顺时针显示红、绿、蓝、灰色矩形，灰色区域由黑到白变化，可用于检查屏幕方向和 RGB 位序。

注意事项
---------

.. note::

   未连接匹配面板时，编译成功只代表构建验证；观察显示现象必须连接配置对应的显示硬件。
