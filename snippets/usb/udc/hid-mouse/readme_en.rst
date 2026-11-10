.. _usb_udc_hid-mouse_en:

USB HID Mouse
==============

Overview
---------

This sample uses the HPMicro USB device controller to emulate HID mouse buttons and pointer movement from the board's input buttons.

* `Zephyr 4.4.0 USB HID Mouse sample <https://docs.zephyrproject.org/4.4.0/samples/subsys/usb/hid-mouse/README.html>`_

Application paths
------------------

.. code-block:: text

   zephyr/samples/subsys/usb/hid-mouse

Hardware setup
---------------

Connect the board's configured USB device port to a host computer. Keep the debug console connection available for logs.

Building
---------

Run from the west workspace root. The following commands use ``hpm6750evk2`` as an example:

.. code-block:: console

   west build -p always -b hpm6750evk2 -S hid-mouse zephyr/samples/subsys/usb/hid-mouse

Expected behavior
------------------

After flashing and reconnecting USB, the device enumerates a mouse. Pressing the first user button sends a left-button report, and ``led0`` toggles after each report is sent successfully. On boards with additional input keys, the next keys act as the right mouse button and move the pointer by 10 units along the X and Y axes.

Notes
------

.. note::

   Use the USB connector selected by the board overlay; a power-only cable cannot enumerate the device.
