.. _msc_ram_disk:

MSC RAM Disk
========================

Overview
--------

This example project shows USB MSC devcie

- PC sees a USB disk via Device Manager

Board Setting
-------------

- Connect a USB port on PC to the PWR DEBUG port on the development board with a USB Type-C cable

- Connect a USB port on PC to one of USB port on the development board with a USB Type-C cable

Build Cmd
-----------

- Taking hpm6750evk2 as an example, execute the following command in the sdk_glue directory

    .. code-block:: console

        west build -p always -b hpm6750evk2 samples/cherryusb/device/msc/ram_disk/

Running the example
-------------------

- Download the program and run. The computer can automatically recognize and enumerate a USB disk device.

- You can copy a file to the USB disk, and then copy it from the USB disk.
