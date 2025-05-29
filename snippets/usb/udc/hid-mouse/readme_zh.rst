.. _hid-mouse:

hid-mouse
=============

路径
------

.. code-block::

    {workspace}/zephyr/samples/subsys/usb/hid-mouse


- 有关详细信息，请参阅 `README.rst` 文档

命令行示例
------------

以hpm6750evk为例:

.. code-block:: console

    west build -p always -b hpm6750evk2 -S hid-mouse samples/subsys/usb/hid-mouse -T sample.usb_device_next.hid-mouse


已知问题
----------

- 由于hpm6750/hpm6800 gpio不支持 `GPIO_INT_EDGE_BOTH` 中断，因此不支持gpio按键功能