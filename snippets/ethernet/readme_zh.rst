.. _ethernet_zh:

以太网 Echo Server
===================

概述
-----

本示例使用 HPMicro 以太网控制器运行 Zephyr TCP echo server，并将收到的数据原样返回给客户端。

* `Zephyr 4.4.0 echo server sample <https://docs.zephyrproject.org/4.4.0/samples/net/sockets/echo_server/README.html>`_

示例路径
---------

.. code-block:: text

   zephyr/samples/net/sockets/echo_server

硬件准备
---------

将开发板网口连接到具备示例所需地址分配服务的网络，或在开发板和主机上配置匹配的静态地址。

构建
-----

在 west 工作区根目录执行。以下命令以 ``hpm6750evk2`` 为例：

.. code-block:: console

   west build -p always -b hpm6750evk2 -S ethernet zephyr/samples/net/sockets/echo_server

预期现象
---------

串口输出网络地址和监听端口；TCP 客户端发送的数据会被服务器原样回传。

注意事项
---------

.. note::

   主机防火墙和子网配置必须允许访问示例使用的 TCP 端口。
