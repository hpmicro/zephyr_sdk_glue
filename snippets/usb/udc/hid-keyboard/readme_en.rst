.. _hid_keyboard:

hid_keyboard
=============
`zephyr sample link <https://docs.zephyrproject.org/3.7.0/samples/subsys/usb/hid-keyboard/README.html>`_

Path
---------------

.. code-block::

    zephyr/samples/subsys/usb/hid-keyboard

Build Cmd
------------

As hpm6750evk2 for example:

.. code-block:: console

    west build -p always -b hpm6750evk2 -S hid-keyboard samples/subsys/usb/hid-keyboard -T sample.usbd.hid-keyboard 

Known Issues
-------------

- As hpm6750/hpm6800 gpio don't support `GPIO_INT_EDGE_BOTH` interrupt, so gpio-keys function is not supported.