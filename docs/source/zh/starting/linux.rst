======================
linux上的环境配置
======================

**linux** (这里以ubuntu, bash进行环境配置)环境下需要自行安装hpmicro riscv-openocd, 将openocd添加到系统路径。

安装工具
--------

#. 更新apt的列表

    .. code-block:: console

        sudo apt update
        sudo apt upgrade

#. 安装所依赖的软件

    .. code-block:: console

        sudo apt install --no-install-recommends git cmake ninja-build gperf \
            ccache dfu-util device-tree-compiler wget \
            python3-dev python3-pip python3-setuptools python3-tk python3-wheel xz-utils file \
            make gcc gcc-multilib g++-multilib libsdl2-dev libmagic1

#. 确认工具版本,主要为cmake,dtc的版本,版本不够需要升级。

.. list-table::
   :header-rows: 1

   * - Tool
     - Min. Version

   * - `CMake <https://cmake.org/>`_
     - 3.20.5

   * - `Python <https://www.python.org/>`_
     - 3.12

   * - `Devicetree compiler <https://www.devicetree.org/>`_
     - 1.4.6


搭建workspace
--------------

#. 安装west,将 ``~/.local/bin`` 加入bashrc,确保terminal在启动时 ``PATH`` 含有该路径。

    .. code-block:: console
        
        pip3 install --user -U west
        echo 'export PATH=~/.local/bin:"$PATH"' >> ~/.bashrc
        source ~/.bashrc

#. 创建${workspace}目录,获取源代码

    .. code-block:: console

        mkdir ${workspace}
        cd ${workspace}
        west init -m https://github.com/hpmicro/zephyr_sdk_glue.git --mr main

#. 获取所需仓库的源代码,默认从github获取,需要切换到gitee源,请输入第一条指令:

    .. code-block:: console

        west config manifest.file west_gitee.yml
        west update

#. 将zephyr安装路径注册到cmake中

    .. code-block:: console

        west zephyr-export

#. 安装zephyr所需的python依赖

    .. code-block:: console

        pip3 install --user -r zephyr/scripts/requirements.txt

#. 增加hpm_sdk相关补丁

    .. code-block:: console

        west supply

配置工具链
--------------------

支持多种工具链，请根据您的需求选择其中一种进行安装和配置。

.. note::

   如果您已经有可用的 RISC-V 工具链，可以跳过安装步骤，直接配置环境变量即可。详细的工具链配置和切换方法请参阅 :doc:`toolchain` 章节。

Zephyr SDK
^^^^^^^^^^^^^^^^^^^^^^^^^^

Zephyr SDK 1.0.1 是默认推荐工具链。以下步骤安装 minimal SDK 包，并由 ``setup.sh`` 自动下载 RISC-V GNU 工具链。

下载地址：`ZEPHYR-SDK <https://github.com/zephyrproject-rtos/sdk-ng/releases/>`_

#. 命令行安装

    .. code-block:: console

        cd ${workspace}
        wget https://github.com/zephyrproject-rtos/sdk-ng/releases/download/v1.0.1/zephyr-sdk-1.0.1_linux-x86_64_minimal.tar.xz
        wget -O - https://github.com/zephyrproject-rtos/sdk-ng/releases/download/v1.0.1/sha256.sum | sha256sum --check --ignore-missing
        tar xvf zephyr-sdk-1.0.1_linux-x86_64_minimal.tar.xz

    该校验命令仅检查当前已下载的 minimal SDK 压缩包。RISC-V 工具链将在下一步由 ``setup.sh`` 下载。

#. 安装 RISC-V 工具链和主机工具，并注册 Zephyr SDK

    .. code-block:: console

        cd zephyr-sdk-1.0.1
        ./setup.sh -t riscv64-zephyr-elf -h -c

ZCC 工具链
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

ZCC 是基于 LLVM/Clang 的工具链，下载体积小，编译速度快。

#. 下载并解压 ZCC 工具链

    请从相应渠道获取 ZCC 工具链包并解压到指定目录，例如 ``~/sdk_env/toolchains/zcc-4.1.5/``

#. 配置环境变量

    .. code-block:: console

        export ZEPHYR_TOOLCHAIN_VARIANT=zcc
        export ZCC_TOOLCHAIN_PATH=~/sdk_env/toolchains/zcc-4.1.5/
        export TOOLCHAIN_ROOT=${workspace}/sdk_glue/

    建议将以上环境变量添加到 ``~/.bashrc`` 中以便持久化。

GNU 交叉编译工具链
^^^^^^^^^^^^^^^^^^^^^^^^^^

如果您已有 RISC-V GCC 工具链，可以直接使用。非官方支持，使用时请自行解决相关问题。

#. 配置环境变量

    .. code-block:: console

        export ZEPHYR_TOOLCHAIN_VARIANT=cross-compile
        export CROSS_COMPILE=/path/to/riscv32-unknown-elf-gcc/bin/riscv32-unknown-elf-

    请将 ``/path/to/riscv32-unknown-elf-gcc`` 替换为您的工具链实际安装路径

编译zephyr的button sample
--------------------------
    编译hpm6750evk2的button sample, **build** 目录可以放置在workspace的任意地方,推荐放在zephyr的目录下。

#. 构建与编译

    .. code-block:: console

        cd ${workspace}/zephyr
        west build -p always -b hpm6750evk2 samples/basic/button

``-p`` 选项, ``always`` 重新编译, ``auto`` 增量编译。
``-S`` 选项, 特定的硬件或者配置选项支持,如:

    .. code-block:: console

        west build -p always -b hpm6750evk2 -S blinky_pwm samples/basic/blinky_pwm    

#. 烧录或调试

    .. code-block:: console
        
        west flash / west debug

其他
-----
    一些会用到的命令：

#. Kconfig配置系统

    .. code-block:: console

        west build -t menuconfig

#. 查看可使用的board

    .. code-block:: console

        west boards | grep hpm

#. 连接板子,调用gdbserver

    .. code-block:: console

        west debugserver

#. 生成文档html格式

    .. code-block:: console

        cd sdk_glue/docs
        make html
