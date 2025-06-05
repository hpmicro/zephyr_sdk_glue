.. _msc_disk_host:

MSC Disk Host
========================

Overview
--------

This example project shows USB MSC Disk HOST, which use FatFs.

Board Setting
-------------

- Connect a USB port on PC to the PWR DEBUG port on the development board with a USB Type-C cable

- Connect a USB port on the development board USB0 to a USB flash disk

Build Cmd
-----------

- Taking hpm6750evk2 as an example, execute the following command in the sdk_glue directory

    .. code-block:: console

        west build -p always -b hpm6750evk2 samples/cherryusb/host/msc_disk/

Running the example
-------------------

- Download the program to the development board and run it. Use the serial debugging assistant to view the output log

- After connecting the host to the USB drive, the file directory of the USB drive will be listed. The log example is as follows.


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
