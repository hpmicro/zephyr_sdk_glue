.. _usb_udc_mass_en:

USB Mass Storage
=================

Overview
---------

This sample uses the HPMicro USB device controller to expose a FAT-formatted RAM disk as a USB mass-storage device.

* `Zephyr 4.4.0 USB Mass Storage sample <https://docs.zephyrproject.org/4.4.0/samples/subsys/usb/mass/README.html>`_

Application paths
------------------

.. code-block:: text

   zephyr/samples/subsys/usb/mass

Hardware setup
---------------

Connect the board's configured USB device port to a host computer. Keep the debug console connection available for logs.

Building
---------

Run from the west workspace root. The following commands use ``hpm6750evk2`` as an example:

.. code-block:: console

   west build -p always -b hpm6750evk2 -S mass zephyr/samples/subsys/usb/mass

Expected behavior
------------------

After flashing and reconnecting USB, the console reports ``The device is put in USB mass storage mode``. The host enumerates a removable disk backed by the snippet's FAT-formatted RAM disk; it can be mounted and used like a temporary USB drive. Its contents are lost when the board resets or loses power.

Notes
------

.. note::

   Use the USB connector selected by the board overlay; a power-only cable cannot enumerate the device.
