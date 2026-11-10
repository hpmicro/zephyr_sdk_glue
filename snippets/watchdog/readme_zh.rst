.. _watchdog_zh:

Watchdog 驱动测试
==================

概述
-----

本示例使用 HPMicro 看门狗资源运行 Zephyr watchdog 基础 API 和复位模式测试。

示例路径
---------

.. code-block:: text

   zephyr/tests/drivers/watchdog/wdt_basic_api
   zephyr/tests/drivers/watchdog/wdt_basic_reset_none

硬件准备
---------

不需要外接硬件；复位模式测试会主动复位开发板。

构建
-----

在 west 工作区根目录执行。以下命令以 ``hpm6750evk2`` 为例：

.. code-block:: console

   west build -p always -b hpm6750evk2 -S watchdog zephyr/tests/drivers/watchdog/wdt_basic_api
   west build -p always -b hpm6750evk2 -S watchdog zephyr/tests/drivers/watchdog/wdt_basic_reset_none

预期现象
---------

API 测试输出 watchdog 回调和超时检查结果；复位测试会故意等待看门狗超时并验证重启行为。

注意事项
---------

.. note::

   hpm6800evk 和 hpm6e00evk 的 EWDG 中断计数器与复位计数器相互独立；hpm6800evk 只支持复位模式，hpm6e00evk 同时支持中断和复位模式。hpm6750evk2 与 hpm6200evk 也支持两种模式，但复位计数在中断阈值到达后开始，因此总复位时间等于中断配置时间与复位配置时间之和。
