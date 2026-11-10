.. _usb_udc_cdc_acm_en:

USB CDC ACM
============

Overview
---------

This sample uses the HPMicro USB device controller to provide a CDC ACM virtual serial port that echoes host data.

* `Zephyr 4.4.0 USB CDC ACM sample <https://docs.zephyrproject.org/4.4.0/samples/subsys/usb/cdc_acm/README.html>`_

Application paths
------------------

.. code-block:: text

   zephyr/samples/subsys/usb/cdc_acm

Hardware setup
---------------

Connect the board's configured USB device port to a host computer. Keep the debug console connection available for logs.

Building
---------

Run from the west workspace root. The following commands use ``hpm6750evk2`` as an example:

.. code-block:: console

   west build -p always -b hpm6750evk2 -S cdc_acm zephyr/samples/subsys/usb/cdc_acm

Expected behavior
------------------

After flashing and reconnecting USB, the device enumerates a virtual COM port. Open that port with a serial terminal; every character sent by the host is echoed back to the same port.

Notes
------

.. note::

   Use the USB connector selected by the board overlay; a power-only cable cannot enumerate the device.
