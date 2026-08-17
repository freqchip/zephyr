.. _driver_test:

Driver Test
###########

Overview
********

Freqchip FR8029D 板级驱动测试工程，覆盖以下外设：

- ADC（单端 / 差分通道读取）
- I2C（寄存器写读）
- CAN（收发过滤器注册、报文发送）
- UART（中断接收回调注册）
- GPIO / LED（定时闪烁）
- Watchdog（超时配置示例）
- Bluetooth（可连接广播）

Building and Running
********************

构建该工程：

.. zephyr-app-commands::
   :zephyr-app: samples/boards/freqchip/driver_test
   :host-os: unix
   :board: fr8029D_BIB
   :goals: build
   :compact:

Sample Output
=============

.. code-block:: console

    Driver Test! fr8029D_BIB
    ADC ready!
    I2C0 ready!
    rx_filter ret:0
    ...
