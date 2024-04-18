==========
添加 soc
==========

参考 `commit 2589b391 <http://192.168.11.211/oss/zephyr_sdk_glue/-/commit/2589b391ff2e8c3bd60a11365ce500c8720c2d50>`_

更改的地方
==========

#. 设备树, 如果是双核请参考 ``hpm67xx`` 的写法

#. ``start.S`` 作为启动的开始, Kconfig中规定了入口标记

        .. code-block:: Kconfig

                config KERNEL_ENTRY
                    default "_start"

#. ``soc.c`` ``soc.h`` 参考 hpm_sdk中的 ``board_init``

#. linker file, zephyr的link分为两个阶段, 可以通过构建后的 ``linker_zephyr_pre0.cmd`` 和 ``linker_zephyr_pre1.cmd`` 排查问题,或查看最终linker file

#. Kconfig文件, ``.defconfig`` 作为预定义的最先加入Kconfig构建 ``.kconfig.soc`` 作为menuconfig的 ``soc selection``, ``.kconfig`` 作为menuconfig的 ``hardware configuration``