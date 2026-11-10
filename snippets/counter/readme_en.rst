.. _counter_en:

Counter Driver Test
====================

Overview
---------

This sample runs Zephyr's basic counter API test with an HPMicro timer/counter device, covering alarms, top values, and timing behavior.

Application paths
------------------

.. code-block:: text

   zephyr/tests/drivers/counter/counter_basic_api

Hardware setup
---------------

No external hardware is required.

Building
---------

Run from the west workspace root. The following commands use ``hpm6750evk2`` as an example:

.. code-block:: console

   west build -p always -b hpm6750evk2 -S counter zephyr/tests/drivers/counter/counter_basic_api

Expected behavior
------------------

The application runs the counter ztest cases and prints a passing summary when counter timing and callbacks behave as expected.

Notes
------

.. note::

   This test has no separate Zephyr HTML sample page; the link points to the official v4.4.0 source.
