=========
配置环境
=========

Zephyr在不同系统上的环境配置。

HPMicro Zephyr 支持多种编译工具链，您可以根据实际情况选择：

.. list-table::
   :header-rows: 1
   :widths: 20 50

   * - 工具链
     - 说明
   * - **Zephyr SDK 1.0.1**
     - 默认推荐工具链，功能完整，兼容性最佳
   * - **ZCC 工具链**
     - 基于 LLVM/Clang，下载快速，性能优秀
   * - **GNU 交叉编译工具链**
     - 使用已有的 RISC-V GCC 工具链

.. note::

   | 默认推荐使用 Zephyr SDK 1.0.1，也可以根据需要选择其他受支持的工具链。
   | 详细的工具链配置和切换方法请参阅 :doc:`toolchain` 章节。

.. toctree::
   :maxdepth: 2

   linux
   windows
   docker
   toolchain
