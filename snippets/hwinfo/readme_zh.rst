.. _hwinfo_zh:

硬件信息测试
=============

概述
-----

本示例在 HPMicro 硬件上运行 Zephyr HWINFO API 测试，包括设备标识和复位原因查询。

示例路径
---------

.. code-block:: text

   zephyr/tests/drivers/hwinfo/api

硬件准备
---------

不需要外接硬件。

构建
-----

在 west 工作区根目录执行。以下命令以 ``hpm6750evk2`` 为例：

.. code-block:: console

   west build -p always -b hpm6750evk2 -S hwinfo zephyr/tests/drivers/hwinfo/api

预期现象
---------

串口输出 HWINFO ztest 测试结果；SoC 信息可用时，设备 ID 相关测试通过。

注意事项
---------

.. note::

   复位引脚不会被报告为独立 reset cause，因此按开发板复位键后报告值不会变化，对应的上游测试项可能失败。
