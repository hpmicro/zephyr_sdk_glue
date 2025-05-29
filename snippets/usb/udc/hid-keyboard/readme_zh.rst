.. _hid_keyboard:

hid_keyboard
=============

路径
------

.. code-block::

    {workspace}/zephyr/samples/subsys/usb/hid-keyboard

- 有关详细信息，请参阅 `README.rst` 文档

命令行示例
------------

以hpm6750evk为例:

.. code-block:: console

    west build -p always -b hpm6750evk2 -S hid-keyboard samples/subsys/usb/hid-keyboard -T sample.usbd.hid-keyboard 

已知问题
----------

- 由于hpm6750/hpm6800 gpio不支持 `GPIO_INT_EDGE_BOTH` 中断，因此不支持gpio按键功能