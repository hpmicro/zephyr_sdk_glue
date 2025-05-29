.. _mass:

mass
==========

Sample Path
---------------

.. code-block::

    {workspace}/zephyr/samples/subsys/usb/mass


- please refer to the `README.rst` document for details.

Build Cmd
------------

As hpm6750evk2 for example:

.. code-block:: console

    west build -p always -b hpm6750evk2 -S mass samples/subsys/usb/mass -T sample.usb_device_next.mass_ram_none
