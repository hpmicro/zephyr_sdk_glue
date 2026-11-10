.. _video_mipi_zh:

MIPI-CSI 视频采集
==================

概述
-----

本示例通过 HPMicro MIPI-CSI 接口采集摄像头图像，并将其输出到配置的显示设备。

* `Zephyr 4.4.0 video capture sample <https://docs.zephyrproject.org/4.4.0/samples/drivers/video/capture/README.html>`_

示例路径
---------

.. code-block:: text

   zephyr/samples/drivers/video/capture

硬件准备
---------

将受支持的 MIPI-CSI 摄像头和 LCD 面板连接到 hpm6800evk snippet 文件选择的接口。

构建
-----

在 west 工作区根目录执行。以下命令以 ``hpm6800evk`` 为例：

.. code-block:: console

   west build -p always -b hpm6800evk -S video_mipi zephyr/samples/drivers/video/capture

预期现象
---------

串口列出摄像头格式并持续输出采集帧信息；连接配置对应的显示屏后，LCD 显示实时摄像头画面。

注意事项
---------

.. note::

   请使用 snippet 选择的摄像头和面板配置；MIPI lane 或像素格式不匹配会导致无法采集。
