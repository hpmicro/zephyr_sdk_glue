============
添加 drivers
============

参考 `commit 25583e2a <http://192.168.11.211/oss/zephyr_sdk_glue/-/commit/25583e2a28b0f08a43ebc39de32e8f2fbae73a09>`_

更改的地方
===============

- 添加对应ip的驱动文件, 命名 ``${IP}_hpmicro.c``
- 添加 ``drivers/${ip}/Kconfig.hpmicro``
- 添加 ``drivers/${ip}/CMakeLists.txt`` 
- 在 ``drivers/Kconfig`` ``drivers/CMakeLists.txt`` 中添加对应ip层级
- 在对应soc设备树中添加ip节点,在对应board的设备树文件中使能相应的ip
- 在对应的 ${board}-pinctrl.dtsi 文件中添加引脚定义
- 如有需要,添加ip描述文件, ``dts/bindings/${ip}/hpmicro,hpm-${ip}.yaml``
- 在 ``${board}_defconfig`` 文件中选中ip

注意
=======

.. code-block:: Kconfig

        DT_COMPAT_HPM_CAN := hpmicro,hpm-can
        config CAN_HPMICRO
            bool "HPMICRO CAN driver"
            default y
            depends on DT_HAS_HPMICRO_HPM_CAN_ENABLED
            select HAS_HPMSDK_CAN
            help
                Enable the HPMICRO CAN driver

- ``CAN_HPMICRO`` 用于生成 ``CONFIG_CAN_HPMICRO`` 使用于cmake的构建
- ``DT_HAS_HPMICRO_HPM_CAN_ENABLED`` 当设备树中有兼容 ``hpmicro,hpm-can`` 的节点,且 ``okay`` 的时候为 ``true``
- ``HAS_HPMSDK_CAN`` 在cmake构建的时候会传递给hpm_sdk使能对应的ip driver
- 不想使用某个driver的情况下, 将节点 ``disable``, 在 ``${board}_defconfig`` 隐掉对应ip

驱动文件
=========

- 中断部分目前使用的是zephyr维护的软中断表,优先级以及中断号在devicetree中定义,中断号使用 ``hpm_sdk`` 的 ``vectors.h``, 避免大家定义重复。
- 注册到硬中断向量表使用 ``IRQ_DIRECT_CONNECT`` , 注册到软件中断向量表使用 ``IRQ_CONNECT`` ,硬中断向量表暂不使用
- ip的初始化, zephyr有五个阶段可以选择 ``EARLY`` ``PRE_KERNEL_1`` ``PRE_KERNEL_2`` ``POST_KERNEL`` ``APPLICATION``, 后两个在zephyr内核启动之后, 可以使用日志系统, 前三个在内核启动之前。
- zephyr对于ip提供了一些标准化的接口 `device APIs <https://docs.zephyrproject.org/3.4.0/doxygen/html/group__io__interfaces.html>`_ 
- 具体的驱动实例化请参考目前已有的驱动 `device model <https://docs.zephyrproject.org/3.4.0/doxygen/html/group__device__model.html>`_