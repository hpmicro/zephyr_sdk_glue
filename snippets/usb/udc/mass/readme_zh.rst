.. _mass:

mass
=============

路径
------

.. code-block::

    {workspace}/zephyr/samples/subsys/usb/mass


- 有关详细信息，请参阅 `README.rst` 文档

命令行示例
------------

以hpm6750evk为例:

.. code-block:: console

    west build -p always -b hpm6750evk2 -S mass samples/subsys/usb/mass -T sample.usb_device_next.mass_ram_none