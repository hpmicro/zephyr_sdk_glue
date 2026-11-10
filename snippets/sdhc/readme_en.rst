.. _sdhc_en:

SDHC and SD/MMC Tests
======================

Overview
---------

This sample uses the HPMicro SDHC controller to run Zephyr's controller, disk-access, performance, SD, and MMC tests.

Application paths
------------------

.. code-block:: text

   zephyr/tests/drivers/sdhc
   zephyr/tests/drivers/disk/disk_access
   zephyr/tests/drivers/disk/disk_performance
   zephyr/tests/subsys/sd/sdmmc
   zephyr/tests/subsys/sd/mmc

Hardware setup
---------------

Insert a compatible SD or eMMC device as required by the selected test. Tests that write media can destroy existing data.

Building
---------

Run from the west workspace root. The following commands use ``hpm6750evk2`` as an example:

.. code-block:: console

   west build -p always -b hpm6750evk2 -S sdhc zephyr/tests/drivers/sdhc
   west build -p always -b hpm6750evk2 -S sdhc zephyr/tests/drivers/disk/disk_access
   west build -p always -b hpm6750evk2 -S sdhc zephyr/tests/drivers/disk/disk_performance
   west build -p always -b hpm6750evk2 -S sdhc zephyr/tests/subsys/sd/sdmmc
   west build -p always -b hpm6750evk2 -S sdhc zephyr/tests/subsys/sd/mmc

Expected behavior
------------------

The selected test reports card initialization and operation results. A successful run ends with passing ztest output or disk performance statistics.

Notes
------

.. note::

   These tests have no separate Zephyr HTML sample pages. Use expendable media for write, erase, and performance tests.
