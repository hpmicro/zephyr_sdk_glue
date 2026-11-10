.. _usb_host_demo:

CherryUSB Host Demo
===================

Overview
--------

This sample contains a Zephyr adaptation of CherryUSB's unified Host demo. It
supports serial devices (CDC ACM, CH340, CP210x, FTDI, PL2303), HID devices,
and USB mass-storage devices in one Host sample.

Build
-----

From the ``sdk_glue`` directory, build for the HPM6750 EVK2:

.. code-block:: console

   west build -p always -b hpm6750evk2 samples/cherryusb/host/usb_host

Hardware
--------

Connect a USB device to the development board's USB0 Host port. The following
USB device types are supported:

* Serial: Serial devices (the ``hpm_sdk/samples/cherryusb/device/cdc_acm/cdc_acm_vcom``
  sample can be used as a serial loopback device).
* HID: Mice and keyboards.
* MSC: FAT/FAT32 USB drives.

Running the example
-------------------

After the board is programmed and running, the Host automatically enumerates
the connected USB device and runs the corresponding test:

* Serial devices: The Host performs a serial loopback test and reports success
  when the received data matches the transmitted data.
* HID mice and keyboards: The Host prints input report data as it is received.
* MSC FAT/FAT32 USB drives: The Host mounts the USB drive, prints its capacity,
  and lists the contents of its root directory.
