.. _blinky_en:

Blinky
=======

Overview
---------

This sample uses Zephyr's GPIO API to toggle the configured user LED on an HPMicro development board at a fixed interval.

* `Zephyr 4.4.0 Blinky sample <https://docs.zephyrproject.org/4.4.0/samples/basic/blinky/README.html>`_

Application paths
------------------

.. code-block:: text

   zephyr/samples/basic/blinky

Hardware setup
---------------

No external hardware is required when the selected board has a supported user LED.

Building
---------

Run from the west workspace root. The following commands use ``hpm6750evk2`` as an example:

.. code-block:: console

   west build -p always -b hpm6750evk2 -S blinky zephyr/samples/basic/blinky

Expected behavior
------------------

After flashing, the configured LED changes state once per second. The console prints the LED device and state when logging is enabled.
