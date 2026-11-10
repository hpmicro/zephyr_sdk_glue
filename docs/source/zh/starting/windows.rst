======================
windows上的环境配置
======================

**windows** 环境下可以使用 **chocolatey** 去获取必要的工具。

使用chocolatey安装工具
----------------------

#. 打开 **windows** 的 **PowerShell** 安装 **chocolatey**

    .. code-block:: console

        Set-ExecutionPolicy AllSigned
        Set-ExecutionPolicy Bypass -Scope Process -Force; [System.Net.ServicePointManager]::SecurityProtocol = [System.Net.ServicePointManager]::SecurityProtocol -bor 3072; iex ((New-Object System.Net.WebClient).DownloadString('https://community.chocolatey.org/install.ps1'))

#. 打开 **cmd.exe**, 关闭软件安装确认

    .. code-block:: console

        choco feature enable -n allowGlobalConfirmation

#. 安装所依赖的软件

    .. code-block:: console

        choco install cmake --installargs 'ADD_CMAKE_TO_PATH=System'
        choco install ninja gperf python git dtc-msys2 wget 7zip

#. 重启 **cmd.exe**

搭建workspace
--------------

#. 安装west

    .. code-block:: console
        
        pip3 install -U west

#. 创建workspace目录,获取所有源代码

    .. code-block:: console

        mkdir %workspace%
        cd %workspace%
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

        pip3 install -r zephyr\scripts\requirements.txt

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

Zephyr SDK 1.0.1 是默认推荐工具链。安装时使用 minimal SDK 包，并配合 RISC-V GNU 工具链包。

下载地址：`ZEPHYR-SDK <https://github.com/zephyrproject-rtos/sdk-ng/releases/>`_

#. 命令行安装

    .. code-block:: console

        cd %workspace%
        wget https://github.com/zephyrproject-rtos/sdk-ng/releases/download/v1.0.1/zephyr-sdk-1.0.1_windows-x86_64_minimal.7z
        7z x zephyr-sdk-1.0.1_windows-x86_64_minimal.7z

#. 配置工具链必要变量

    .. code-block:: console

        cd zephyr-sdk-1.0.1
        setup.cmd /t riscv64-zephyr-elf /c

ZCC 工具链
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

ZCC 是基于 LLVM/Clang 的工具链，下载体积小，编译速度快。

#. 下载并解压 ZCC 工具链

    请从相应渠道获取 ZCC 工具链包并解压到指定目录，例如 ``C:\sdk_env\toolchains\zcc-4.1.5\``

#. 配置环境变量（在 cmd 中）

    .. code-block:: console

        set ZEPHYR_TOOLCHAIN_VARIANT=zcc
        set ZCC_TOOLCHAIN_PATH=C:\sdk_env\toolchains\zcc-4.1.5\
        set TOOLCHAIN_ROOT=%workspace%\sdk_glue\

    建议将以上环境变量添加到系统环境变量中以便持久化。

GNU 交叉编译工具链
^^^^^^^^^^^^^^^^^^^^^^^^^^

如果您已有 RISC-V GCC 工具链，可以直接使用。

#. 配置环境变量（在 cmd 中）

    .. code-block:: console

        set ZEPHYR_TOOLCHAIN_VARIANT=cross-compile
        set CROSS_COMPILE=C:\path\to\riscv32-unknown-elf-gcc\bin\riscv32-unknown-elf-

    请将 ``C:\path\to\riscv32-unknown-elf-`` 替换为您的工具链实际安装路径

编译zephyr的button sample
--------------------------
    编译hpm6750evk2的button sample, **build** 目录可以放置在workspace的任意地方,推荐放在zephyr的目录下。

#. 构建与编译

    .. code-block:: console

        cd %workspace%\zephyr
        west build -p always -b hpm6750evk2 samples\basic\button

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

        west boards | findstr hpm

#. 连接板子,调用gdbserver

    .. code-block:: console

        west debugserver

#. 生成文档html格式

    .. code-block:: console

        cd sdk_glue\docs
        make html
