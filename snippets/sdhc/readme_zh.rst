.. _sdhc_zh:

SDHC 与 SD/MMC 测试
====================

概述
-----

本示例使用 HPMicro SDHC 控制器运行 Zephyr 的控制器、磁盘访问、性能、SD 和 MMC 测试。

示例路径
---------

.. code-block:: text

   zephyr/tests/drivers/sdhc
   zephyr/tests/drivers/disk/disk_access
   zephyr/tests/drivers/disk/disk_performance
   zephyr/tests/subsys/sd/sdmmc
   zephyr/tests/subsys/sd/mmc

硬件准备
---------

根据所选测试插入兼容的 SD 卡或 eMMC 器件。执行写入操作的测试可能破坏介质中的已有数据。

构建
-----

在 west 工作区根目录执行。以下命令以 ``hpm6750evk2`` 为例：

.. code-block:: console

   west build -p always -b hpm6750evk2 -S sdhc zephyr/tests/drivers/sdhc
   west build -p always -b hpm6750evk2 -S sdhc zephyr/tests/drivers/disk/disk_access
   west build -p always -b hpm6750evk2 -S sdhc zephyr/tests/drivers/disk/disk_performance
   west build -p always -b hpm6750evk2 -S sdhc zephyr/tests/subsys/sd/sdmmc
   west build -p always -b hpm6750evk2 -S sdhc zephyr/tests/subsys/sd/mmc

预期现象
---------

所选测试输出存储卡初始化和操作结果；成功时以 ztest 通过汇总或磁盘性能统计结束。

注意事项
---------

.. note::

   这些测试没有独立的 Zephyr HTML 示例页。写入、擦除和性能测试应使用允许数据被覆盖的介质。
