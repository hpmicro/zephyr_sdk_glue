.. _sdhc:

Sdhc
=======

Path
---------------

.. code-block::

    zephyr/tests/drivers/sdhc
    zephyr/tests/drivers/disk/disk_access
    zephyr/tests/drivers/disk/disk_performance
    zephyr/tests/subsys/sd/sdmmc

Build Cmd
-----------

As hpm6750evk2 for example:

.. code-block:: console

    west build -p always -b hpm6750evk2 -S sdhc zephyr/tests/drivers/sdhc