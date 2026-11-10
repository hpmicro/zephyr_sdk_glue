.. _can_zh:

CAN 驱动测试
=============

概述
-----

本示例使用 HPMicro CAN 控制器运行 Zephyr CAN API、位时序和 shell 测试。

示例路径
---------

.. code-block:: text

   zephyr/tests/drivers/can/api
   zephyr/tests/drivers/can/timing
   zephyr/tests/drivers/can/shell

硬件准备
---------

需要外部报文的测试应将开发板连接到 CAN 收发器及正确终端匹配的 CAN 总线；控制器回环模式测试不需要其他节点。

构建
-----

在 west 工作区根目录执行。以下命令以 ``hpm6750evk2`` 为例：

.. code-block:: console

   west build -p always -b hpm6750evk2 -S can zephyr/tests/drivers/can/api
   west build -p always -b hpm6750evk2 -S can zephyr/tests/drivers/can/timing
   west build -p always -b hpm6750evk2 -S can zephyr/tests/drivers/can/shell

预期现象
---------

API 和 timing 应用输出 ztest 结果；shell 测试验证 CAN shell 命令能够配置并操作控制器。

注意事项
---------

.. note::

   这些测试没有独立 HTML 示例页，以上链接均指向官方 v4.4.0 源码。
