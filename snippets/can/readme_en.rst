.. _can:

Can
=======

Path
---------------

.. code-block::

    zephyr/tests/drivers/can/api
    zephyr/tests/drivers/can/timing
    zephyr/tests/drivers/can/shell (need enable can-fd)

Build Cmd
-----------

As hpm6750evk2 for example:

.. code-block:: console

    west build -p always -b hpm6750evk2 -S can tests/drivers/can/api
    west build -p always -b hpm6750evk2 -S can tests/drivers/can/timing
    west build -p always -b hpm6750evk2 -S can tests/drivers/can/shell (need enable can-fd)