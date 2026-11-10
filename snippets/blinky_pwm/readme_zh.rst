.. _blinky_pwm_zh:

PWM 闪烁灯
===========

概述
-----

本示例使用 Zephyr PWM API 驱动 HPMicro 开发板上配置的 PWM 输出，每四秒改变一次频率，占空比固定为 50%。

* `Zephyr 4.4.0 PWM Blinky 示例 <https://docs.zephyrproject.org/4.4.0/samples/basic/blinky_pwm/README.html>`_

示例路径
---------

.. code-block:: text

   zephyr/samples/basic/blinky_pwm

默认示例引脚
-------------

本 snippet 支持以下开发板。HPM5300EVK 的 overlay 选择 PWM 通道 2，其他 overlay 选择通道 0；下表列出对应板级 pinctrl 设备树配置的默认输出引脚。

* ``hpm5100evk``：PWM3，``PA16``(``J7[7]``)
* ``hpm5300evk``：PWM0 通道 2，``PA26``(``J7[11]``)
* ``hpm5e00evk``：PWM0，``PD0``(``J3[7]``)
* ``hpm6200evk``：PWM0，``PB12``(``J4[14]``)
* ``hpm6300evk``：PWM0，``PC0``(``J26[14]``)
* ``hpm6750evk2``：PWM2，``PD31``(``J10[14]``)
* ``hpm6e00evk``：PWM1，``PE8``(``J4[7]``)
* ``hpm6p00evk``：PWM1，``PD8``(``J4[7]``)

硬件准备
---------

请将带限流电阻的外部 LED 或示波器连接到上表所列的默认 PWM 引脚。

构建
-----

在 west 工作区根目录执行。以下命令以 ``hpm6750evk2`` 为例：

.. code-block:: console

   west build -p always -b hpm6750evk2 -S blinky_pwm zephyr/samples/basic/blinky_pwm

预期现象
---------

PWM 频率每四秒加倍，达到 128 Hz 后再逐步减半，回到校准后的最低频率，占空比始终为 50%。示例首先尝试一秒周期（1 Hz），若驱动不支持则缩短周期。外接 LED 的闪烁速度随之变化，示波器可观察到周期变化。

注意事项
---------

.. note::

   开发板板载 LED 未连接到示例所选的 PWM 引脚，因此需要使用外部 LED 或示波器观察现象。
