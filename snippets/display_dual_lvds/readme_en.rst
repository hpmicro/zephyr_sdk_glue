.. _display_dual_lvds_en:

Dual-channel LVDS Display
==========================

Overview
---------

This sample uses Zephyr's display API to present the test pattern through the HPMicro dual-channel LVDS interface and the board's configured panel.

* `Zephyr 4.4.0 display sample <https://docs.zephyrproject.org/4.4.0/samples/drivers/display/README.html>`_

Application paths
------------------

.. code-block:: text

   zephyr/samples/drivers/display

Hardware setup
---------------

Connect the LCD panel required by the selected board overlay and verify that its power, backlight, and signal cables are seated correctly.

Building
---------

Run from the west workspace root. The following commands use ``hpm6800evk`` as an example:

.. code-block:: console

   west build -p always -b hpm6800evk -S display_dual_lvds zephyr/samples/drivers/display

Expected behavior
------------------

The panel shows four corner rectangles in clockwise red, green, blue, and grey order. The grey area changes from black to white, which helps identify orientation and RGB ordering problems.

Notes
------

.. note::

   A successful build without the matching panel only verifies compilation; the visual result requires the configured display hardware.
