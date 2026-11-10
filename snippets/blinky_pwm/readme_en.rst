.. _blinky_pwm_en:

PWM Blinky
===========

Overview
---------

This sample uses Zephyr's PWM API to drive the configured HPMicro PWM output and change its frequency every four seconds at a fixed 50% duty cycle.

* `Zephyr 4.4.0 PWM Blinky sample <https://docs.zephyrproject.org/4.4.0/samples/basic/blinky_pwm/README.html>`_

Application paths
------------------

.. code-block:: text

   zephyr/samples/basic/blinky_pwm

Default PWM pins
----------------

The snippet supports the following boards. The HPM5300EVK overlay selects PWM channel 2; the other overlays select channel 0. The pin listed here is the default output pad configured by that board's pinctrl devicetree.

* ``hpm5100evk``: PWM3, ``PA16`` (``J7[7]``)
* ``hpm5300evk``: PWM0 channel 2, ``PA26`` (``J7[11]``)
* ``hpm5e00evk``: PWM0, ``PD0`` (``J3[7]``)
* ``hpm6200evk``: PWM0, ``PB12`` (``J4[14]``)
* ``hpm6300evk``: PWM0, ``PC0`` (``J26[14]``)
* ``hpm6750evk2``: PWM2, ``PD31`` (``J10[14]``)
* ``hpm6e00evk``: PWM1, ``PE8`` (``J4[7]``)
* ``hpm6p00evk``: PWM1, ``PD8`` (``J4[7]``)

Hardware setup
---------------

Connect an external LED with current limiting, or an oscilloscope, to the default PWM pin listed above.

Building
---------

Run from the west workspace root. The following commands use ``hpm6750evk2`` as an example:

.. code-block:: console

   west build -p always -b hpm6750evk2 -S blinky_pwm zephyr/samples/basic/blinky_pwm

Expected behavior
------------------

The PWM frequency doubles every four seconds up to 128 Hz, then halves back to the calibrated minimum frequency. The duty cycle stays at 50%. The sample first tries a one-second period (1 Hz) and reduces it if the driver cannot support it. An external LED changes its blink rate; an oscilloscope shows the changing period.

Notes
------

.. note::

   The on-board LED is not connected to the selected PWM pin, so an external LED or oscilloscope is required.
