.. _can_en:

CAN Driver Tests
=================

Overview
---------

This sample runs Zephyr's CAN API, bit-timing, and shell tests with the HPMicro CAN controller.

Application paths
------------------

.. code-block:: text

   zephyr/tests/drivers/can/api
   zephyr/tests/drivers/can/timing
   zephyr/tests/drivers/can/shell

Hardware setup
---------------

Connect the board to a CAN transceiver and a correctly terminated CAN bus when a test requires external traffic. Controller-loopback tests do not require another node.

Building
---------

Run from the west workspace root. The following commands use ``hpm6750evk2`` as an example:

.. code-block:: console

   west build -p always -b hpm6750evk2 -S can zephyr/tests/drivers/can/api
   west build -p always -b hpm6750evk2 -S can zephyr/tests/drivers/can/timing
   west build -p always -b hpm6750evk2 -S can zephyr/tests/drivers/can/shell

Expected behavior
------------------

The API and timing applications report ztest results. The shell test validates that CAN shell commands can configure and operate the controller.

Notes
------

.. note::

   These tests have no separate HTML sample pages; the links point to the official v4.4.0 sources.
