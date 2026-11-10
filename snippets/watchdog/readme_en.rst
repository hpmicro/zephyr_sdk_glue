.. _watchdog_en:

Watchdog Driver Tests
======================

Overview
---------

This sample uses HPMicro watchdog resources to run Zephyr's basic watchdog API and reset-mode tests.

Application paths
------------------

.. code-block:: text

   zephyr/tests/drivers/watchdog/wdt_basic_api
   zephyr/tests/drivers/watchdog/wdt_basic_reset_none

Hardware setup
---------------

No external hardware is required. Be prepared for the reset-mode test to reset the board.

Building
---------

Run from the west workspace root. The following commands use ``hpm6750evk2`` as an example:

.. code-block:: console

   west build -p always -b hpm6750evk2 -S watchdog zephyr/tests/drivers/watchdog/wdt_basic_api
   west build -p always -b hpm6750evk2 -S watchdog zephyr/tests/drivers/watchdog/wdt_basic_reset_none

Expected behavior
------------------

The API test reports watchdog callback and timeout checks. The reset test intentionally lets the watchdog expire and verifies the reboot behavior.

Notes
------

.. note::

   On hpm6800evk and hpm6e00evk, EWDG interrupt and reset counters are independent; hpm6800evk supports reset mode only, while hpm6e00evk supports both modes. On hpm6750evk2 and hpm6200evk both modes are supported, but reset counting starts after the interrupt threshold, so total reset time equals the configured interrupt time plus reset time.
