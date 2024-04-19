=====================
添加 flash/debug脚本
=====================

参考 `commit f9ee3ff5 <http://192.168.11.211/oss/zephyr_sdk_glue/-/commit/f9ee3ff5e3e06a0a6144a754d5a254061411da35>`_ 中的 ``board.cmake``

注意的地方
===========

- 需在 ``${board}`` 目录下创建一个support文件夹, 这里本身是存放openocd相关脚本的, 但目前我都是使用的hpm_sdk的, 这样在sdk更新的时候,sdk_glue的脚本不需要变更