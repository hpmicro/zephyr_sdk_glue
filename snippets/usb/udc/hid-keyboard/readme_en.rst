.. _hid_keyboard:

hid_keyboard
=============

Sample Path
---------------

.. code-block::

    {workspace}/zephyr/samples/subsys/usb/hid-keyboard

- please refer to the `README.rst` document for details.

Build Cmd
------------

As hpm6750evk2 for example:

.. code-block:: console

    west build -p always -b hpm6750evk2 -S hid-keyboard samples/subsys/usb/hid-keyboard -T sample.usbd.hid-keyboard 

Known Issues
-------------

- As hpm6750/hpm6800 gpio don't support `GPIO_INT_EDGE_BOTH` interrupt, so gpio-keys function is not supported.