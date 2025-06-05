.. _msc_disk_host:

MSC Disk Host
========================

概述
------

本示例工程展示USB Msc HOST示例，它使用到了Fatfs文件系统。

硬件设置
------------

- 使用USB Type-C线缆线连接PC USB端口和PWR DEBUG端口

- 使用USB Type-C转Type-A线缆线连接开发板USB0端口和U盘

编译指令
-----------

- 以hpm6750evk2为例，在sdk_glue目录下执行如下指令

    .. code-block:: console

        west build -p always -b hpm6750evk2 samples/cherryusb/host/msc_disk/

运行现象
------------

- 将程序下载至开发板运行，使用串口调试助手查看输出log。

- Host接上U盘后，会列出U盘的文件目录，log示例如下。


    .. code-block:: console

        *** Booting Zephyr OS build v0.5.0-zsg-rc1 ***
        [I/usbh_msc] Capacity info:
        [I/usbh_msc] Block num:15360000,block size:512
        [I/usbh_msc] Capacity info:
        [I/usbh_msc] Block num:15360000,block size:512
        [I/usbh_msc] Capacity info:
        [I/usbh_msc] Block num:15360000,block size:512
        [00:00:01.174,000] <inf> msc_main: Block count 15360000
        Sector size 512
        Memory Size(MB) 7500
        Disk mounted.

        Listing dir /USB: ...
        [FILE] SOME.DAT (size = 0)
        [DIR ] SOME
        [DIR ] SYSTEM~1
