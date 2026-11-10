.. _shell_shell_module_zh:

Shell 模块
===========

概述
-----

本示例演示如何注册 Zephyr shell 命令、子命令、动态命令、字典参数以及 bypass 回调。

* `Zephyr 4.4.0 shell module sample <https://docs.zephyrproject.org/4.4.0/samples/subsys/shell/shell_module/README.html>`_

示例路径
---------

.. code-block:: text

   zephyr/samples/subsys/shell/shell_module

硬件准备
---------

按开发板配置的波特率连接串口控制台。

构建
-----

在 west 工作区根目录执行。以下命令以 ``hpm6750evk2`` 为例：

.. code-block:: console

   west build -p always -b hpm6750evk2 -S shell_module zephyr/samples/subsys/shell/shell_module

预期现象
---------

串口出现 shell 提示符。执行 ``help`` 可看到示例命令，并可验证参数、动态条目、字典值和 bypass 输入处理。

注意事项
---------

.. note::

   可在 shell 中输入 ``help <command>`` 查看每个演示命令支持的参数。
