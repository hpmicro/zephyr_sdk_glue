.. _can:

Can (enable can-fd as default)
=======

路径
---------------

.. code-block::

    zephyr/tests/drivers/can/api
    zephyr/tests/drivers/can/timing
    zephyr/tests/drivers/can/shell

命令行
--------

以hpm6750evk为例:

.. code-block:: console

    west build -p always -b hpm6750evk2 -S can tests/drivers/can/api
    west build -p always -b hpm6750evk2 -S can tests/drivers/can/timing
    west build -p always -b hpm6750evk2 -S can tests/drivers/can/shell