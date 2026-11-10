.. _adc_dt_zh:

ADC 驱动测试
=================

概述
-----

本 snippet 在各 EVK 上使能一路 ADC 通道，
配合 ``zephyr,user`` 的 ``io-channels``，支撑 ``adc_dt`` 示例和 ``adc_api`` 测试。

各板的通道、引脚与 HPM_SDK 的 ADC demo（``samples/drivers/adc/adc16``、``adc12``）
初始化的是同一个引脚，丝印位置取自 SDK 板级文档 ``boards/<板卡>/README_zh.rst``：

.. list-table::
   :header-rows: 1

   * - 板卡
     - ADC 节点
     - 通道
     - 引脚
     - 丝印/位置
     - 备注
   * - hpm5100evk
     - adc0
     - 2
     - PC05
     - P1[18]
     -
   * - hpm5300evk
     - adc0
     - 13
     - PB05
     - P1[32]
     -
   * - hpm5301evklite
     - adc0
     - 2
     - PB10
     - J3[26]
     - 与 demo 一样把 PB08~PB15 整组设为模拟功能
   * - hpm5e00evk
     - adc0
     - 1
     - PF26
     - J3[6]
     -
   * - hpm6200evk
     - adc0
     - 8
     - PC12
     - P2[11]
     -
   * - hpm6300evk
     - adc0
     - 6
     - PC10
     - J28[38]
     -
   * - hpm6750evk2
     - adc0 (ADC12 IP)
     - 11
     - PE25
     - J10[6]
     -
   * - hpm6800evk
     - adc0
     - 8
     - PE16
     - J19[15]
     - 见下方注意事项
   * - hpm6e00evk
     - adc0
     - 15
     - PF07
     - J4[2]
     - SMA 座子
   * - hpm6p00evk
     - adc2
     - 10
     - PD26
     - J4[2]
     - SMA 座子；输入电压可用 J5 跨针选 DAC 或板载可调电阻分压

硬件准备
---------

不需要外接硬件；引脚浮空时采样值为随机值，测试只校验读取流程不校验数值。
需要定量验证时，从表中丝印位置接入 0~VREF(3.3V) 的直流电平即可。

构建
-----

在 west 工作区根目录执行。以下命令以 ``hpm6750evk2`` 为例：

.. code-block:: console

   west build -p always -b hpm6750evk2 -S adc_dt zephyr/samples/drivers/adc/adc_dt
   west build -p always -b hpm6750evk2 -S adc_dt zephyr/tests/drivers/adc/adc_api -- -DCONFIG_TEST_USERSPACE=n

预期现象
---------

adc_dt 示例循环打印各通道原始采样值；adc_api 测试输出 ztest 通过汇总。

注意事项
---------

.. note::

   hpm6800evk：SDK 的 ``pinmux.c`` 把 PE24 设为模拟功能，但 ``board.h`` 和板级
   README 指定的测试点是 PE16（ADC0.IN8，丝印 J19[15]），两者不一致；DS 引脚表
   确认 PE16=ADC0_IN8、PE24=ADC0_IN12。本 snippet 按 README/DS 使用 PE16 通道 8。
   运行 SDK 自带的 adc16 demo 在这块板上时请注意这个问题。
