.. _blinky_pwm:

pwm驱动灯闪烁
==============
`zephyr sample link <https://docs.zephyrproject.org/3.7.0/samples/basic/blinky_pwm/README.html>`_

路径
---------------

.. code-block::

    zephyr/samples/basic/blinky_pwm

命令行
-----------

以hpm6750evk为例:

.. code-block:: console

    west build -p always -b hpm6750evk2 -S blinky_pwm zephyr/samples/basic/blinky_pwm


Note
------

- 在板子上灯并未接到pwm引脚上,需要外接器件(灯或示波器)