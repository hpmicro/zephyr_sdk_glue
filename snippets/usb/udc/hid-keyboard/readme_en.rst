.. _usb_udc_hid-keyboard_en:

USB HID Keyboard
=================

Overview
---------

This sample uses the HPMicro USB device controller to emulate a HID keyboard from the board's input buttons.

* `Zephyr 4.4.0 USB HID Keyboard sample <https://docs.zephyrproject.org/4.4.0/samples/subsys/usb/hid-keyboard/README.html>`_

Application paths
------------------

.. code-block:: text

   zephyr/samples/subsys/usb/hid-keyboard

Hardware setup
---------------

Connect the board's configured USB device port to a host computer. Keep the debug console connection available for logs.

Building
---------

Run from the west workspace root. The following commands use ``hpm6750evk2`` as an example:

.. code-block:: console

   west build -p always -b hpm6750evk2 -S hid-keyboard zephyr/samples/subsys/usb/hid-keyboard

Expected behavior
------------------

After flashing and reconnecting USB, the device enumerates a keyboard. Pressing the board's first user button sends the Num Lock key; the host changes its Num Lock state and ``led0`` follows that state, so the LED toggles on successive presses. If the board exposes more input keys, the next keys send Caps Lock, Scroll Lock, and ``Right Alt+1+2+3`` respectively.

Notes
------

.. note::

   Use the USB connector selected by the board overlay; a power-only cable cannot enumerate the device.
