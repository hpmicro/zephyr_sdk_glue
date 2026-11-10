.. Copyright (c) 2026 HPMicro
.. SPDX-License-Identifier: Apache-2.0

.. _clock_tree_en:

Runtime Clock Tree Report
==================================

Overview
---------

This HPMicro diagnostic sample reports the clock tree after normal SoC and
driver initialization. It reads live driver state instead of devicetree
frequency properties. PLL frequencies are obtained through the HPM SDK
PLLCTL/PLLCTLV2 API, while PLL outputs and functional clock nodes are queried
through the HPM SDK clock API.

Functional nodes are reported as enabled when their resources are present in
clock group 0, which is the group used by ``sdk_glue``. Shared or invalid
resources that cannot be queried as group resources are skipped.

This is an HPMicro-specific diagnostic and has no matching upstream Zephyr
sample.

Supported boards
-----------------

* ``hpm5100evk``
* ``hpm5300evk``
* ``hpm5301evklite``
* ``hpm5e00evk``
* ``hpm6200evk``
* ``hpm6300evk``
* ``hpm6750evk2``
* ``hpm6800evk``
* ``hpm6e00evk``
* ``hpm6p00evk``

Building
---------

Run from the west workspace root. The following command uses
``hpm5301evklite`` as an example:

.. code-block:: console

   west build -p always -b hpm5301evklite sdk_glue/tests/clock_tree

Expected behavior
------------------

After flashing, the console identifies the board and SoC, then prints the live
PLL frequencies, PLL output frequencies, CPU frequency, every enabled clock
node in group 0, and the total number of enabled nodes. Frequencies depend on
the selected board and enabled peripherals. A typical output has this form:

.. code-block:: console

   HPMicro runtime clock tree report
   Board: hpm5301evklite
   SoC:   hpm5301
   All frequencies below are read from runtime driver APIs.

   PLL runtime frequencies:
     PLL0: 480000000 Hz
       CLK0 (clock_pll0_clk0): 480000000 Hz
     ...

   CPU runtime frequencies:
     clock_cpu0           480000000 Hz

   Enabled clock nodes (group 0):
     clock_cpu0           480000000 Hz
     ...
   Enabled clock nodes: 12

.. note::

   The numeric values above are illustrative. Use the values printed by the
   target as the authoritative runtime configuration.
