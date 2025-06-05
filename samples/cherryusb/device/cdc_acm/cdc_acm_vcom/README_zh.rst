.. _cdc_acm_vcom:

CDC ACM VCOM
========================

概述
------

本示例工程展示USB CDC设备虚拟串口

- PC 通过设备管理器查看得到一个串口

硬件设置
------------

- 使用USB Type-C线缆连接PC USB端口和PWR DEBUG端口

- 使用USB Type-C线缆连接PC USB端口和开发板USB0端口

编译指令
-----------

- 以hpm6750evk2为例，在sdk_glue目录下执行如下指令

    .. code-block:: console

        west build -p always -b hpm6750evk2 samples/cherryusb/device/cdc_acm/cdc_acm_vcom/

运行现象
------------

- 将程序下载至开发板运行，电脑可自动识别并安装串口驱动，枚举出具有一个com口的设备

- 打开串口调试助手软件，选择枚举出来的com口，点击打开串口

- 然后在串口调试助手中输入字符，点击发送，串口调试助手中会回显所发送的字符。
