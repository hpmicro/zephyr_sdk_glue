.. _entropy_zh:

Entropy 驱动测试
=================

概述
-----

本示例使用 HPMicro 熵源运行 Zephyr entropy 驱动 API 测试，验证普通接口和 ISR 安全接口获取随机数据。

示例路径
---------

.. code-block:: text

   zephyr/tests/drivers/entropy/api

硬件准备
---------

不需要外接硬件。

构建
-----

在 west 工作区根目录执行。以下命令以 ``hpm6750evk2`` 为例：

.. code-block:: console

   west build -p always -b hpm6750evk2 -S entropy zephyr/tests/drivers/entropy/api

预期现象
---------

能够正常获取随机数据时，串口输出 entropy API 测试通过及最终 ztest 通过汇总。

注意事项
---------

.. note::

   该测试没有独立的 Zephyr HTML 示例页，以上链接指向官方 v4.4.0 源码。
