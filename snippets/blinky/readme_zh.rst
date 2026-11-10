.. _blinky_zh:

LED 闪烁
=========

概述
-----

本示例通过 Zephyr GPIO API 按固定周期翻转 HPMicro 开发板上配置的用户 LED。

* `Zephyr 4.4.0 Blinky sample <https://docs.zephyrproject.org/4.4.0/samples/basic/blinky/README.html>`_

示例路径
---------

.. code-block:: text

   zephyr/samples/basic/blinky

硬件准备
---------

所选开发板带有受支持的用户 LED 时，不需要外接硬件。

构建
-----

在 west 工作区根目录执行。以下命令以 ``hpm6750evk2`` 为例：

.. code-block:: console

   west build -p always -b hpm6750evk2 -S blinky zephyr/samples/basic/blinky

预期现象
---------

烧录后，配置的 LED 每秒切换一次状态；启用日志时，串口会输出 LED 设备及其状态。
