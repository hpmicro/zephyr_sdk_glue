.. _video_dvp_zh:

DVP 视频采集
=============

概述
-----

本示例通过 HPMicro 并行 DVP 接口采集摄像头图像，并将其输出到配置的显示设备。

* `Zephyr 4.4.0 video capture sample <https://docs.zephyrproject.org/4.4.0/samples/drivers/video/capture/README.html>`_

示例路径
---------

.. code-block:: text

   zephyr/samples/drivers/video/capture

硬件准备
---------

连接开发板 overlay 选择的 DVP 摄像头模块和 LCD 面板，并确认电源及排线方向正确。

构建
-----

在 west 工作区根目录执行。以下命令以 ``hpm6750evk2`` 为例：

.. code-block:: console

   west build -p always -b hpm6750evk2 -S video_dvp zephyr/samples/drivers/video/capture

预期现象
---------

串口列出 video 和 display 能力，随后持续输出采集帧序号、大小和时间戳；配置显示设备时，LCD 显示摄像头实时画面。

注意事项
---------

.. note::

   显示现象依赖于使用所选开发板配置文件匹配的摄像头和面板。
