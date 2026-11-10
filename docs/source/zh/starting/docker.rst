===================
Docker 环境配置
===================

Docker 镜像提供 HPMicro Zephyr 开发所需的依赖和工具，当前版本内置：

- ZCC 4.1.5
- Zephyr SDK 1.0.1（默认工具链）
- HPMicro OpenOCD
- ZCC 与 Zephyr SDK 工具链切换脚本

源码不再打包到镜像中。``zephyr``、``sdk_glue``、``sdk_env``、``modules`` 和
``bootloader`` 目录由主机分别挂载到容器中，因此在主机上修改代码后，容器内可以立即
看到修改，无需重新构建docker镜像。

前置条件
=========

- Linux 主机已安装 Docker 和 Docker Compose
- 已获得完整的 ZSG 工作区，且各模块目录位于同一级目录
- 如需烧录或调试，主机已连接 CMSIS-DAP 等 USB 调试器

工作区结构
============

``docker-compose.yaml`` 位于 ``sdk_glue`` 目录。启动容器前，工作区应至少包含：

.. code-block:: text

    ZSG/
    |-- zephyr/
    |-- sdk_glue/
    |   `-- docker-compose.yaml
    |-- sdk_env/hpm_sdk


.. note::

   Compose 文件中的相对路径以 ``sdk_glue`` 目录为基准。请在该目录中执行
   ``docker compose`` 命令，否则应使用 ``-f`` 显式指定 Compose 文件。

获取 Docker 镜像
==================

当前 Compose 配置使用镜像 ``zephyr_hpmicro:v0.8.0``。可以加载发布的镜像包：

.. code-block:: console

    docker load -i zephyr_hpmicro-v0.8.0.tar
    docker image ls zephyr_hpmicro

镜像包可从以下地址获取：

- `zephyr_hpmicro-v0.8.0.tar <https://resource.hpmicro.com/zephyr_hpmicro-v0.8.0.tar>`_

如果从镜像仓库拉取后得到的是带仓库前缀的镜像名称，请将其标记为 Compose 使用的名称：

.. code-block:: console

    docker pull swhpmicro/zephyr_hpmicro:v0.8.0
    docker tag swhpmicro/zephyr_hpmicro:v0.8.0 zephyr_hpmicro:v0.8.0

启动和管理容器
================

#. 进入 ``sdk_glue`` 目录并启动容器：

   .. code-block:: console

       cd /path/to/ZSG/sdk_glue
       docker compose up -d

#. 进入容器：

   .. code-block:: console

       docker compose exec zephyr bash

#. 使用完毕后停止并删除容器：

   .. code-block:: console

       docker compose down

``docker compose down`` 不会删除主机源码。后续再次执行 ``docker compose up -d`` 即可
重新创建容器。

源码挂载方式
=============

Compose 将主机工作区的各模块映射到容器的 ``/home/User/zephyr_space``：

.. list-table:: 源码目录映射
   :header-rows: 1

   * - 主机目录
     - 容器目录
   * - ``../zephyr``
     - ``/home/User/zephyr_space/zephyr``
   * - ``.``（``sdk_glue``）
     - ``/home/User/zephyr_space/sdk_glue``
   * - ``../sdk_env``
     - ``/home/User/zephyr_space/sdk_env``
   * - ``../modules``
     - ``/home/User/zephyr_space/modules``
   * - ``../bootloader``
     - ``/home/User/zephyr_space/bootloader``

容器的默认工作目录为 ``/home/User/zephyr_space``。由于这些目录是绑定挂载，容器内
对源码的修改也会直接写入主机文件。

工具链
======

镜像同时包含 Zephyr SDK 1.0.1 和 ZCC 4.1.5，不需要从主机挂载或另外安装工具链。

Zephyr SDK 1.0.1
----------------

Zephyr SDK 是默认工具链，安装路径为
``/opt/toolchains/zephyr-sdk-1.0.1``。进入容器后可检查当前配置：

.. code-block:: console

    echo $ZEPHYR_TOOLCHAIN_VARIANT
    echo $ZEPHYR_SDK_INSTALL_DIR

默认输出应分别为 ``zephyr`` 和 ``/opt/toolchains/zephyr-sdk-1.0.1``。

ZCC 4.1.5
---------

使用镜像内置脚本切换到 ZCC：

.. code-block:: console

    source ~/zephyr_space/use-zcc.sh

切回 Zephyr SDK：

.. code-block:: console

    source ~/zephyr_space/use-zephyr-sdk.sh

.. note::

   必须使用 ``source`` 执行脚本，使环境变量在当前 shell 中生效。每次新进入一个
   shell 时，工具链都会使用 Compose 配置的默认值；如需使用 ZCC，请再次执行切换脚本。

OpenOCD、编译和烧录
====================

HPMicro OpenOCD 已打包到镜像中，不需要在主机或挂载目录中另行安装。进入容器后可直接
编译、烧录和调试：

.. code-block:: console

    cd /home/User/zephyr_space/zephyr
    west build -p always -b hpm6750evk2 -S blinky samples/basic/blinky
    west flash
    west debug

Compose 已启用 ``privileged: true``，并将 ``/dev/bus/usb`` 挂载到容器，以便 OpenOCD
访问 USB 调试器。

使用 docker run
================

推荐使用 Compose，以避免遗漏源码或 USB 挂载。如需直接使用 ``docker run``，等效命令为：

.. code-block:: console

    docker run -it --rm --privileged --hostname docker \
        -v /path/to/ZSG/zephyr:/home/User/zephyr_space/zephyr \
        -v /path/to/ZSG/sdk_glue:/home/User/zephyr_space/sdk_glue \
        -v /path/to/ZSG/sdk_env:/home/User/zephyr_space/sdk_env \
        -v /path/to/ZSG/modules:/home/User/zephyr_space/modules \
        -v /path/to/ZSG/bootloader:/home/User/zephyr_space/bootloader \
        -v /dev/bus/usb:/dev/bus/usb \
        -w /home/User/zephyr_space \
        zephyr_hpmicro:v0.8.0

请将 ``/path/to/ZSG`` 替换为主机上的实际工作区绝对路径。

常见问题
=========

容器内看不到源码
------------------

确认五个模块目录与 ``sdk_glue`` 位于同一级，并从 ``sdk_glue`` 目录启动 Compose。可用
以下命令检查实际挂载：

.. code-block:: console

    docker compose config
    docker compose exec zephyr ls /home/User/zephyr_space

找不到工具链
------------

Zephyr SDK 和 ZCC 均位于镜像内，不应再挂载主机 SDK。先切回默认工具链并检查环境变量：

.. code-block:: console

    source ~/zephyr_space/use-zephyr-sdk.sh
    echo $ZEPHYR_TOOLCHAIN_VARIANT
    echo $ZEPHYR_SDK_INSTALL_DIR

OpenOCD 找不到 USB 设备
-----------------------

#. 在主机上执行 ``lsusb``，确认调试器已连接。
#. 确认 Compose 中保留 ``privileged: true`` 和 ``/dev/bus/usb:/dev/bus/usb``。
#. 重新插拔调试器并重新创建容器。
