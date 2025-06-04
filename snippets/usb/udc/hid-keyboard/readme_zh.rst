.. _hid_keyboard:

hid_keyboard
=============
`zephyr sample link <https://docs.zephyrproject.org/3.7.0/samples/subsys/usb/hid-keyboard/README.html>`_

路径
------

.. code-block::

    zephyr/samples/subsys/usb/hid-keyboard

命令行
------------

以hpm6750evk2为例:

.. code-block:: console

    west build -p always -b hpm6750evk2 -S hid-keyboard samples/subsys/usb/hid-keyboard -T sample.usbd.hid-keyboard 

已知问题
----------

- 由于hpm6750/hpm6800 gpio不支持 `GPIO_INT_EDGE_BOTH` 中断,因此不支持gpio按键功能