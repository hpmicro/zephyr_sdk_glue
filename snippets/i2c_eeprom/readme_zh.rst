.. _i2c_eeprom_zh:

I2C EEPROM
===========

概述
-----

本示例使用 Zephyr EEPROM API 读取、写入并校验 HPMicro 开发板上配置的 EEPROM 数据。

* `Zephyr 4.4.0 EEPROM sample <https://docs.zephyrproject.org/4.4.0/samples/drivers/eeprom/README.html>`_

示例路径
---------

.. code-block:: text

   zephyr/samples/drivers/eeprom

硬件准备
---------

使用 snippet overlay 中已配置 EEPROM 器件和 I2C 路由的受支持开发板。

构建
-----

在 west 工作区根目录执行。以下命令以 ``hpm6750evk2`` 为例：

.. code-block:: console

   west build -p always -b hpm6750evk2 -S i2c_eeprom zephyr/samples/drivers/eeprom

预期现象
---------

串口会识别 EEPROM、写入测试数据、回读数据并报告校验成功。

注意事项
---------

.. note::

   该示例会修改一小段 EEPROM 区域，请勿在该区域保存有必须保留的数据时运行。
