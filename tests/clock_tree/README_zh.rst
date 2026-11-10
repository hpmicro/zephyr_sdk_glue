.. _clock_tree_zh:

运行时时钟树报告
=========================

概述
-----

这是一个 HPMicro 专用诊断示例，用于在 SoC 和驱动完成正常初始化后输出运行时时钟树。
示例不读取 devicetree 中的频率属性，而是通过 HPM SDK PLLCTL/PLLCTLV2 API
读取 PLL 频率，并通过 HPM SDK clock API 查询 PLL 输出和功能时钟节点。

当功能时钟资源位于 ``sdk_glue`` 使用的 clock group 0 中时，该节点会被报告为已启用。
无法按 group 查询的共享资源或无效资源会被跳过。

该诊断程序由 HPMicro 提供，没有对应的 Zephyr 上游示例。

支持的开发板
-------------

* ``hpm5100evk``
* ``hpm5300evk``
* ``hpm5301evklite``
* ``hpm5e00evk``
* ``hpm6200evk``
* ``hpm6300evk``
* ``hpm6750evk2``
* ``hpm6800evk``
* ``hpm6e00evk``
* ``hpm6p00evk``

构建
-----

在 west 工作区根目录执行。以下命令以 ``hpm5301evklite`` 为例：

.. code-block:: console

   west build -p always -b hpm5301evklite sdk_glue/tests/clock_tree

预期现象
---------

烧录后，串口首先输出开发板和 SoC 名称，然后依次输出运行时 PLL 频率、PLL 输出频率、
CPU 频率、clock group 0 中所有已启用的时钟节点以及节点总数。具体频率取决于所选
开发板和启用的外设。典型输出格式如下：

.. code-block:: console

   HPMicro runtime clock tree report
   Board: hpm5301evklite
   SoC:   hpm5301
   All frequencies below are read from runtime driver APIs.

   PLL runtime frequencies:
     PLL0: 480000000 Hz
       CLK0 (clock_pll0_clk0): 480000000 Hz
     ...

   CPU runtime frequencies:
     clock_cpu0           480000000 Hz

   Enabled clock nodes (group 0):
     clock_cpu0           480000000 Hz
     ...
   Enabled clock nodes: 12

.. note::

   以上数值仅用于展示输出格式，目标板实际打印的数值才是当前运行时配置。
