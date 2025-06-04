.. _mass:

mass
=============
`zephyr sample link <https://docs.zephyrproject.org/3.7.0/samples/subsys/usb/mass/README.html>`_

路径
------

.. code-block::

    zephyr/samples/subsys/usb/mass

命令行
------------

以hpm6750evk2为例:

.. code-block:: console

    west build -p always -b hpm6750evk2 -S mass samples/subsys/usb/mass -T sample.usb_device_next.mass_ram_none