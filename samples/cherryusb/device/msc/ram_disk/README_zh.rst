.. _msc_ram_disk:

MSC RAM Disk
========================

概述
------

本示例工程展示USB MSC设备虚拟出一个U盘

- PC 通过设备管理器查看得到一个U盘

硬件设置
------------

- 使用USB Type-C线缆连接PC USB端口和PWR DEBUG端口

- 使用USB Type-C线缆连接PC USB端口和开发板USB0端口

编译指令
-----------

- 以hpm6750evk2为例，在sdk_glue目录下执行如下指令

    .. code-block:: console

        west build -p always -b hpm6750evk2 samples/cherryusb/device/msc/ram_disk/

运行现象
------------

- 将程序下载至开发板运行，电脑可自动识别并枚举出一个U盘

- 可以将文件copy至U盘，然后从U盘copy出来，可当做U盘使用
