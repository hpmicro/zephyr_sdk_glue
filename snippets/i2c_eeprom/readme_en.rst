.. _i2c_eeprom_en:

I2C EEPROM
===========

Overview
---------

This sample uses Zephyr's EEPROM API to read, write, and verify data in the configured EEPROM on an HPMicro development board.

* `Zephyr 4.4.0 EEPROM sample <https://docs.zephyrproject.org/4.4.0/samples/drivers/eeprom/README.html>`_

Application paths
------------------

.. code-block:: text

   zephyr/samples/drivers/eeprom

Hardware setup
---------------

Use a supported board with the EEPROM device and I2C routing described by its snippet overlay.

Building
---------

Run from the west workspace root. The following commands use ``hpm6750evk2`` as an example:

.. code-block:: console

   west build -p always -b hpm6750evk2 -S i2c_eeprom zephyr/samples/drivers/eeprom

Expected behavior
------------------

The console identifies the EEPROM, writes test data, reads it back, and reports successful verification.

Notes
------

.. note::

   The sample modifies a small EEPROM region. Do not run it where that region contains data that must be preserved.
