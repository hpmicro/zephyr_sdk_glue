============
 添加 board
============

参考 `commit f9ee3ff5 <http://192.168.11.211/oss/zephyr_sdk_glue/-/commit/f9ee3ff5e3e06a0a6144a754d5a254061411da35>`_

更改的地方
==========

- ``boards/riscv/${board}/CMakeLists.txt`` 以及 ``boards/riscv/${board}/Kconfig`` 预留做一些board的构建和配置
- ``boards/riscv/${board}/Kconfig.board`` 在Kconfig系统中参与board的选择
- ``boards/riscv/${board}/Kconfig.defconfig`` 板子的一些相关参数定义, 可在C中调用
- board所使用的设备树节点以及pinctrl的定义
- ``boards/riscv/${board}/${board}_defconfig`` 板子需要初始化的Kconfig选项
- ``${board}.yaml`` 会解析到Kconfig的系统中生成对应的宏