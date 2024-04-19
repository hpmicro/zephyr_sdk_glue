===============
SDK GLUE 介绍
===============

- **SDK_GLUE** 作为一个west manifest repository而存在, 通过查看 ``sdk_glue/west.yaml`` 获取到所管理的仓库和对应的版本。
- **SDK_GLUE** 使用 **zephyr** 的外部构建, 通过 ``sdk_glue/zephyr/module.yaml`` 将对应路径加入到zephyr的构建中。
