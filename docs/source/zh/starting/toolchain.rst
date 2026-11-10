====================
工具链切换指南
====================

概述
====

Zephyr RTOS 支持多种工具链，包括 GCC、Clang、IAR、以及自定义工具链（如 zcc）。工具链的切换通过环境变量和 CMake 配置系统实现。本文以 zcc 工具链为例，详细说明 Zephyr 如何实现工具链的切换和适配。

支持的工具链
------------

.. list-table::
   :header-rows: 1
   :widths: 20 30 50

   * - 工具链
     - 变量值
     - 说明
   * - Zephyr SDK 1.0.1
     - ``zephyr``
     - sdk_glue 默认推荐工具链，提供 RISC-V GNU 支持
   * - ZCC
     - ``zcc``
     - 基于 LLVM/Clang 的工具链
   * - 交叉编译
     - ``cross-compile``
     - 通用交叉编译工具链

ZCC 工具链使用
==============

环境变量设置
------------

在使用 ZCC 工具链之前，需要设置以下环境变量：

.. code-block:: bash

    export ZEPHYR_TOOLCHAIN_VARIANT=zcc
    export ZCC_TOOLCHAIN_PATH=/path/to/zcc-4.1.x/
    export TOOLCHAIN_ROOT=/path/to/sdk_glue/

.. note::

    可以将以上环境变量添加到 ``~/.bashrc`` 或创建一个脚本文件（如 ``zcc.sh``）方便切换。

环境变量说明
^^^^^^^^^^^^

.. list-table::
   :header-rows: 1
   :widths: 30 70

   * - 环境变量
     - 说明
   * - ``ZEPHYR_TOOLCHAIN_VARIANT``
     - 指定要使用的工具链类型，Zephyr 会根据这个变量加载对应的工具链配置
   * - ``ZCC_TOOLCHAIN_PATH``
     - ZCC 工具链的安装路径，应指向工具链的根目录（bin 目录的上级），例如 ``~/sdk_env/toolchains/zcc-4.1.5/``
   * - ``TOOLCHAIN_ROOT``
     - 工具链适配文件的根目录，通常指向包含 ``cmake/`` 目录的路径

快速开始
--------

#. 创建环境配置脚本

    .. code-block:: bash

        cat > ~/zcc.sh << 'EOF'
        export ZEPHYR_TOOLCHAIN_VARIANT=zcc
        export ZCC_TOOLCHAIN_PATH=~/sdk_env/toolchains/zcc-4.1.5/
        export TOOLCHAIN_ROOT=~/workspace/sdk_glue/
        EOF

#. 激活 zcc 工具链环境

    .. code-block:: bash

        source ~/zcc.sh

#. 编译项目

    .. code-block:: bash

        west build -b hpm6750evk2 samples/hello_world

切换回其他工具链
----------------

切换到 Zephyr SDK
^^^^^^^^^^^^^^^^^

.. code-block:: bash

    export ZEPHYR_TOOLCHAIN_VARIANT=zephyr
    unset ZCC_TOOLCHAIN_PATH
    unset TOOLCHAIN_ROOT

切换到 GNU 交叉编译工具链
^^^^^^^^^^^^^^^^^^^^^^^^^

.. code-block:: bash

    export ZEPHYR_TOOLCHAIN_VARIANT=cross-compile
    export CROSS_COMPILE=/path/to/riscv32-unknown-elf-gcc/bin/riscv32-unknown-elf-

CMake 配置结构
==============

Zephyr 的工具链配置采用模块化设计，主要包含以下几个部分：

.. code-block:: text

    cmake/
    ├── toolchain/          # 工具链通用配置
    │   └── zcc/
    │       ├── generic.cmake    # 工具链初始化
    │       └── target.cmake     # 目标平台特定配置
    ├── compiler/           # 编译器配置
    │   └── zcc/
    │       ├── generic.cmake    # 编译器通用配置
    │       ├── target.cmake     # 编译器目标配置
    │       └── target_riscv.cmake  # RISC-V 架构特定配置
    ├── bintools/          # 二进制工具配置
    │   └── zcc/
    │       ├── target.cmake         # 二进制工具查找
    │       └── target_bintools.cmake # 二进制工具属性配置
    └── linker/            # 链接器配置
        └── ld.lld/
            └── target.cmake  # LLD 链接器配置

工具链加载流程
==============

工具链初始化
------------

工具链初始化在 ``cmake/toolchain/zcc/generic.cmake`` 中完成，主要工作包括：

#. 验证环境变量 ``ZCC_TOOLCHAIN_PATH`` 是否设置
#. 确认工具链路径是否存在
#. 设置工具链相关变量

.. code-block:: cmake

    zephyr_get(ZCC_TOOLCHAIN_PATH)
    assert(ZCC_TOOLCHAIN_PATH "ZCC_TOOLCHAIN_PATH is not set")

    if(NOT EXISTS ${ZCC_TOOLCHAIN_PATH}) 
      message(FATAL_ERROR "Nothing found at ZCC_TOOLCHAIN_PATH: '${ZCC_TOOLCHAIN_PATH}'")
    endif()

    set(TOOLCHAIN_HOME ${ZCC_TOOLCHAIN_PATH})
    set(COMPILER zcc)
    set(LINKER ld.lld)
    set(BINTOOLS zcc)
    set(CROSS_COMPILE ${TOOLCHAIN_HOME}/bin/)

编译器配置
----------

编译器配置分为两个阶段：

通用配置
^^^^^^^^

在 ``cmake/compiler/zcc/generic.cmake`` 中：

- 设置编译器名称（CC = zcc）
- 查找 C 编译器可执行文件
- 验证编译器是否可用

.. code-block:: cmake

    set_ifndef(CC zcc)
    find_program(CMAKE_C_COMPILER ${CROSS_COMPILE}${CC} PATHS ${TOOLCHAIN_HOME} NO_DEFAULT_PATH)

    if(CMAKE_C_COMPILER STREQUAL CMAKE_C_COMPILER-NOTFOUND)
      message(FATAL_ERROR "Zephyr was unable to find the toolchain...")
    endif()

目标配置
^^^^^^^^

在 ``cmake/compiler/zcc/target.cmake`` 中：

- 配置 C++ 编译器（如果启用）
- 设置标准库包含路径
- 配置编译器标志（如 ``-Wall``, ``-ffunction-sections`` 等）
- 处理系统根目录（sysroot）配置

架构特定配置
^^^^^^^^^^^^

针对 RISC-V 架构，在 ``cmake/compiler/zcc/target_riscv.cmake`` 中配置特定的 ABI 和 ISA 扩展：

.. code-block:: cmake

    if(CONFIG_64BIT)
        string(CONCAT riscv_mabi ${riscv_mabi} "64")
        string(CONCAT riscv_march ${riscv_march} "64")
    endif()

    if(CONFIG_FPU)
        if(CONFIG_FLOAT_HARD)
            string(CONCAT riscv_mabi ${riscv_mabi} "d")
        endif()
        string(CONCAT riscv_march ${riscv_march} "fd")
    endif()

    list(APPEND TOOLCHAIN_C_FLAGS -mabi=${riscv_mabi} -march=${riscv_march})

二进制工具配置
--------------

二进制工具配置在 ``cmake/bintools/zcc/`` 目录下，负责查找和配置以下工具：

.. list-table::
   :header-rows: 1
   :widths: 30 70

   * - 工具
     - 说明
   * - ``llvm-objcopy``
     - ELF 文件转换工具
   * - ``llvm-objdump``
     - 反汇编工具
   * - ``llvm-ar``
     - 归档工具
   * - ``llvm-ranlib``
     - 归档索引工具
   * - ``llvm-readelf``
     - ELF 文件读取工具
   * - ``llvm-nm``
     - 符号列表工具
   * - ``llvm-strip``
     - 符号剥离工具
   * - ``zdb``
     - 调试器

工具查找示例：

.. code-block:: cmake

    find_program(CMAKE_OBJCOPY ${CROSS_COMPILE}llvm-objcopy PATHS ${TOOLCHAIN_HOME} NO_DEFAULT_PATH)
    find_program(CMAKE_OBJDUMP ${CROSS_COMPILE}llvm-objdump PATHS ${TOOLCHAIN_HOME} NO_DEFAULT_PATH)

链接器配置
----------

链接器配置在 ``cmake/linker/ld.lld/target.cmake`` 中完成：

#. 查找链接器：通过编译器获取链接器路径，或直接查找 ``ld.lld``
#. 设置链接器变量 ``CMAKE_LINKER``
#. 配置链接标志前缀（``-Wl``）
#. 处理 C++ 异常配置
#. 链接脚本预处理和生成

.. code-block:: cmake

    execute_process(COMMAND ${CMAKE_C_COMPILER} --print-prog-name=ld.lld
                    OUTPUT_VARIABLE ZCCLD_LINKER
                    OUTPUT_STRIP_TRAILING_WHITESPACE)

    set(CMAKE_LINKER ${ZCCLD_LINKER})
    set_ifundefined(LINKERFLAGPREFIX -Wl)

工具链切换机制
==============

基于环境变量的条件加载
----------------------

Zephyr 的构建系统根据 ``ZEPHYR_TOOLCHAIN_VARIANT`` 环境变量的值，动态加载对应的工具链配置：

- ``ZEPHYR_TOOLCHAIN_VARIANT=zcc`` → 加载 ``cmake/toolchain/zcc/`` 下的配置
- ``ZEPHYR_TOOLCHAIN_VARIANT=zephyr`` → 加载 Zephyr SDK 工具链配置

添加新工具链
============

如果要为 Zephyr 添加新的工具链支持，需要：

#. 创建工具链目录结构

    .. code-block:: text

        cmake/
        ├── toolchain/<toolchain_name>/
        ├── compiler/<toolchain_name>/
        ├── bintools/<toolchain_name>/
        └── linker/<linker_name>/

#. 实现配置文件

    - ``toolchain/generic.cmake``: 初始化工具链
    - ``compiler/generic.cmake``: 配置编译器
    - ``compiler/target.cmake``: 编译器目标配置
    - ``bintools/target.cmake``: 配置二进制工具
    - ``bintools/target_bintools.cmake``: 设置工具属性

#. 设置环境变量

    - 定义 ``ZEPHYR_TOOLCHAIN_VARIANT`` 的值
    - 定义工具链路径相关的环境变量

#. 测试验证

    - 验证工具链路径设置正确
    - 验证所有必需的工具都能找到
    - 验证编译和链接过程正常

常见问题
========

Q: 提示 "ZCC_TOOLCHAIN_PATH is not set"
---------------------------------------

确保已正确设置环境变量：

.. code-block:: bash

    export ZCC_TOOLCHAIN_PATH=/path/to/zcc-4.1.x/
    echo $ZCC_TOOLCHAIN_PATH

Q: 提示 "Nothing found at ZCC_TOOLCHAIN_PATH"
---------------------------------------------

检查路径是否正确，确保路径下存在 ``bin/zcc`` 可执行文件：

.. code-block:: bash

    ls $ZCC_TOOLCHAIN_PATH/bin/zcc

Q: 编译时找不到工具链
---------------------

确保 ``TOOLCHAIN_ROOT`` 指向包含 ``cmake/toolchain/zcc/`` 目录的路径：

.. code-block:: bash

    export TOOLCHAIN_ROOT=/path/to/sdk_glue/
    ls $TOOLCHAIN_ROOT/cmake/toolchain/zcc/

Q: 如何查看当前使用的工具链
---------------------------

在编译输出中查找类似以下的信息：

.. code-block:: text

    -- Found toolchain: zcc (/path/to/zcc-4.1.x/)
