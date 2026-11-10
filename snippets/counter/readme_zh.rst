.. _counter_zh:

Counter 驱动测试
=================

概述
-----

本示例使用 HPMicro 定时器/计数器运行 Zephyr counter 基础 API 测试，验证 alarm、top value 和计时行为。

示例路径
---------

.. code-block:: text

   zephyr/tests/drivers/counter/counter_basic_api

硬件准备
---------

不需要外接硬件。

构建
-----

在 west 工作区根目录执行。以下命令以 ``hpm6750evk2`` 为例：

.. code-block:: console

   west build -p always -b hpm6750evk2 -S counter zephyr/tests/drivers/counter/counter_basic_api

预期现象
---------

应用运行 counter ztest 测试项；计时和回调行为符合预期时，最终输出通过汇总。

注意事项
---------

.. note::

   该测试没有独立的 Zephyr HTML 示例页，以上链接指向官方 v4.4.0 源码。
