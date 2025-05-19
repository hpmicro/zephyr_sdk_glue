.. _cdc_acm_vcom:

CDC ACM VCOM
========================

Overview
--------

This example project shows USB CDC Virtual Serial

- PC sees a serial port via Device Manager

Board Setting
-------------

- Connect a USB port on PC to the PWR DEBUG port on the development board with a USB Type-C cable

- Connect a USB port on PC to one of USB port on the development board with a USB Type-C cable

Build Cmd
-----------

- Taking hpm6750evk2 as an example, execute the following command in the sdk_glue directory

    .. code-block:: console

        west build -p always -b hpm6750evk2 samples/cherryusb/device/cdc_acm/cdc_acm_vcom/

Running the example
-------------------

- Download the program and run. The computer can automatically recognize and install the USB to serial port driver and enumerate a device with a com port.

- Open the "Serial Port Debugging Assistant" software, check DTR, select the enumerated com port, and click to open the serial port

- Then enter characters in the "Serial Port Debugging Assistant", click Send, and the characters sent will be echoed in the "Serial Port Debugging Assistant".
