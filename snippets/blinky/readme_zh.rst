.. _blinky:

IO驱动灯闪烁
=============
`zephyr sample link <https://docs.zephyrproject.org/3.7.0/samples/basic/blinky/README.html>`_

路径
---------------

.. code-block::

    zephyr/samples/basic/blinky

命令行
-----------

以hpm6750evk为例:

.. code-block:: console

    west build -p always -b hpm6750evk2 -S blinky zephyr/samples/basic/blinky
