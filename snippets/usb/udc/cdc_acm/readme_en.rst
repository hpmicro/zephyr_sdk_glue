.. _cdc_acm:

cdc_acm
==========
`zephyr sample link <https://docs.zephyrproject.org/3.7.0/samples/subsys/usb/cdc_acm/README.html>`_

Path
---------------

.. code-block::

    zephyr/samples/subsys/usb/cdc_acm

Build Cmd
-----------

As hpm6750evk2 for example:

.. code-block:: console

    west build -p always -b hpm6750evk2 -S cdc_acm samples/subsys/usb/cdc_acm -T sample.usb_device_next.cdc-acm
