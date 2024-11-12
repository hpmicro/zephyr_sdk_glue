======================
windows上的环境配置
======================

**windows** 环境下可以使用 **chocolatey** 去获取必要的工具,也可以使用打包好的工具(存放在gitlab)。

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

#. 创建workspace目录,获取所有源代码（目前源代码托管在内部服务器,以下地址为内部地址,发布后请修改对应地址）

    .. code-block:: console

        mkdir %workspace%
        cd %workspace%
        west init -m %MANIFEST_URL% --mr master

#. 获取所需仓库的源代码,默认从github获取,需要切换到国内源,请输入第一条指令:

    .. code-block:: console

        west config manifest.file west_gitee.yml
        west update

#. 配置CMake变量

    .. code-block:: console

        west zephyr-export

#. 安装zephyr所需的python依赖
    .. code-block:: console

        pip3 install -r %workspace%\zephyr\scripts\requirements.txt

#. 增加hpm_sdk相关补丁

    .. code-block:: console

        west supply

安装zephyr的工具链包
--------------------
    下载zephyr的编译工具 `ZEPHYR-SDK <https://github.com/zephyrproject-rtos/sdk-ng/tags/>`_
    
#. 命令行安装

    .. code-block:: console

        cd %workspace%
        wget https://github.com/zephyrproject-rtos/sdk-ng/releases/download/v0.16.5/zephyr-sdk-0.16.5_windows-x86_64.7z
        7z x zephyr-sdk-0.16.5_windows-x86_64.7z

#. 配置工具链必要变量

    .. code-block:: console

        cd zephyr-sdk-0.16.5
        setup.cmd

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
