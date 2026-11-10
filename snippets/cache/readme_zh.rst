.. _cache_zh:

Cache API 测试
===============

概述
-----

本示例在 HPMicro 硬件上运行 Zephyr 内核 cache 测试，验证 cache flush、invalidate 和数据一致性行为。

示例路径
---------

.. code-block:: text

   zephyr/tests/kernel/cache

硬件准备
---------

不需要外接硬件。

构建
-----

在 west 工作区根目录执行。以下命令以 ``hpm6750evk2`` 为例：

.. code-block:: console

   west build -p always -b hpm6750evk2 -S cache zephyr/tests/kernel/cache

预期现象
---------

串口会输出各个 ztest cache 测试项；缓存维护功能正确时，最终测试汇总为通过。

注意事项
---------

.. note::

   Zephyr 未为该测试提供独立 HTML 示例页，以上链接指向官方 v4.4.0 测试源码。
