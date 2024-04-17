======================
linux上的环境配置
======================

**linux** （这里以ubuntu, bash进行环境配置）环境下需要自行安装hpmicro riscv-openocd,使用默认安装路径 ``/usr/local/bin/openocd`` 

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

    #. 确认工具版本，主要为cmake，dtc的版本，版本不够需要升级。

.. list-table::
   :header-rows: 1

   * - Tool
     - Min. Version

   * - `CMake <https://cmake.org/>`_
     - 3.20.5

   * - `Python <https://www.python.org/>`_
     - 3.8

   * - `Devicetree compiler <https://www.devicetree.org/>`_
     - 1.4.6


搭建workspace
--------------

    #. 安装west，将 ``~/.local/bin`` 加入bashrc，确保terminal在启动时 ``PATH`` 含有该路径。

        .. code-block:: console
            
            pip3 install --user -U west
            echo 'export PATH=~/.local/bin:"$PATH"' >> ~/.bashrc
            source ~/.bashrc

    #. 创建${workspace}目录，获取所有源代码（目前源代码托管在内部服务器，以下地址为内部地址，发布后请修改对应地址）

        .. code-block:: console

            mkdir ${workspace}
            cd ${workspace}
            west init -m git@192.168.11.211:oss/zephyr_sdk_glue.git --mr master
            west update

    #. 配置CMake变量

        .. code-block:: console

            west zephyr-export

    #. 安装zephyr所需的python依赖
        .. code-block:: console

            pip3 install --user -r ~/${workspace}/zephyr/scripts/requirements.txt

安装zephyr的工具链包
--------------------
    下载zephyr的编译工具 `ZEPHYR-SDK <https://github.com/zephyrproject-rtos/sdk-ng/tags/>`_
    
    #. 命令行安装

        .. code-block:: console

            cd ${workspace}
            wget https://github.com/zephyrproject-rtos/sdk-ng/releases/download/v0.16.5/zephyr-sdk-0.16.5_linux-x86_64.tar.xz
            wget -O - https://github.com/zephyrproject-rtos/sdk-ng/releases/download/v0.16.5/sha256.sum | shasum --check --ignore-missing
            tar xvf zephyr-sdk-0.16.5_linux-x86_64.tar.xz

    #. 配置工具链必要变量

        .. code-block:: console

            cd zephyr-sdk-0.16.5
            source setup.sh

编译zephyr的button sample
--------------------------
    编译hpm6750evk2的button sample, **build**目录可以放置在workspace的任意地方，推荐放在zephyr的目录下。

    #. 构建与编译
    
        .. code-block:: console

            cd ${workspace}/zephyr
            west build -p always -b hpm6750evk2 samples/basic/button

    ``-p`` 选项， ``always`` 重新编译， ``auto`` 增量编译。

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

    #. 连接板子，调用gdbserver

        .. code-block:: console

            west debugserver

    #. 生成文档html格式

        .. code-block:: console

            cd sdk_glue/docs
            make html


