.. _hid-mouse:

hid-mouse
==========
`zephyr sample link <https://docs.zephyrproject.org/3.7.0/samples/subsys/usb/hid-mouse/README.html>`_

Sample Path
---------------

.. code-block::

    zephyr/samples/subsys/usb/hid-mouse

Build Cmd
------------

As hpm6750evk2 for example:

.. code-block:: console

    west build -p always -b hpm6750evk2 -S hid-mouse samples/subsys/usb/hid-mouse -T sample.usb_device_next.hid-mouse

Known Issues
-------------

- As hpm6750/hpm6800 gpio don't support `GPIO_INT_EDGE_BOTH` interrupt, so gpio-keys function is not supported.