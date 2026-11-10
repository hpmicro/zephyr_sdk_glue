.. _adc_dt_en:

ADC Driver Test
=================

Overview
--------

This snippet enables one ADC channel on each EVK and wires ``zephyr,user``
``io-channels``, supporting the ``adc_dt`` sample and the ``adc_api`` test.

Each board uses the same pin that the HPM_SDK ADC demos
(``samples/drivers/adc/adc16``, ``adc12``) initialize. Silkscreen locations come
from the SDK board documentation (``boards/<board>/README_zh.rst``):

.. list-table::
   :header-rows: 1

   * - Board
     - ADC node
     - Channel
     - Pin
     - Silkscreen
   * - hpm5100evk
     - adc0
     - 2
     - PC05
     - P1[18]
   * - hpm5300evk
     - adc0
     - 13
     - PB05
     - P1[32]
   * - hpm5301evklite
     - adc0
     - 2
     - PB10
     - J3[26]
   * - hpm5e00evk
     - adc0
     - 1
     - PF26
     - J3[6]
   * - hpm6200evk
     - adc0
     - 8
     - PC12
     - P2[11]
   * - hpm6300evk
     - adc0
     - 6
     - PC10
     - J28[38]
   * - hpm6750evk2
     - adc0 (ADC12 IP)
     - 11
     - PE25
     - J10[6]
   * - hpm6800evk
     - adc0
     - 8
     - PE16
     - J19[15]
   * - hpm6e00evk
     - adc0
     - 15
     - PF07
     - J4[2] (SMA)
   * - hpm6p00evk
     - adc2
     - 10
     - PD26
     - J4[2] (SMA; J5 jumper selects DAC or onboard trimmer as source)

Hardware Setup
--------------

No external wiring is needed. Floating pins yield arbitrary sample values; the
tests validate the read flow, not the sampled value. For quantitative checks,
feed a 0~VREF(3.3V) DC level into the silkscreen location listed above.

Build
-----

Run from the west workspace root. Example with ``hpm6750evk2``:

.. code-block:: console

   west build -p always -b hpm6750evk2 -S adc_dt zephyr/samples/drivers/adc/adc_dt
   west build -p always -b hpm6750evk2 -S adc_dt zephyr/tests/drivers/adc/adc_api -- -DCONFIG_TEST_USERSPACE=n

Expected Behavior
-----------------

The adc_dt sample prints raw sample values in a loop; the adc_api test prints
the ztest pass summary.

Notes
-----

.. note::

   hpm6800evk: the SDK ``pinmux.c`` configures PE24 as analog, but ``board.h``
   and the board README designate PE16 (ADC0.IN8, silkscreen J19[15]). The DS
   pin table confirms PE16=ADC0_IN8 and PE24=ADC0_IN12. This snippet follows the
   README/DS and uses PE16 channel 8.
