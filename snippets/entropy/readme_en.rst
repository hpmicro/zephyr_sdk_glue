.. _entropy_en:

Entropy Driver Test
====================

Overview
---------

This sample runs Zephyr's entropy driver API test with the HPMicro entropy source and requests random data through normal and ISR-safe interfaces.

Application paths
------------------

.. code-block:: text

   zephyr/tests/drivers/entropy/api

Hardware setup
---------------

No external hardware is required.

Building
---------

Run from the west workspace root. The following commands use ``hpm6750evk2`` as an example:

.. code-block:: console

   west build -p always -b hpm6750evk2 -S entropy zephyr/tests/drivers/entropy/api

Expected behavior
------------------

The ztest output reports successful entropy API checks and a passing summary when random data can be obtained.

Notes
------

.. note::

   This test has no separate Zephyr HTML sample page; the link points to the official v4.4.0 source.
