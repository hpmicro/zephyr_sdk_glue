.. _hwinfo_en:

Hardware Information Test
==========================

Overview
---------

This sample runs Zephyr's HWINFO API test on HPMicro hardware, including device identification and reset-cause queries.

Application paths
------------------

.. code-block:: text

   zephyr/tests/drivers/hwinfo/api

Hardware setup
---------------

No external hardware is required.

Building
---------

Run from the west workspace root. The following commands use ``hpm6750evk2`` as an example:

.. code-block:: console

   west build -p always -b hpm6750evk2 -S hwinfo zephyr/tests/drivers/hwinfo/api

Expected behavior
------------------

The ztest output reports the HWINFO checks. Device ID operations pass when the SoC information is available.

Notes
------

.. note::

   The reset pin is not reported as a distinct reset cause. Resetting with the board button therefore does not change the reported cause and the corresponding upstream test case can fail.
