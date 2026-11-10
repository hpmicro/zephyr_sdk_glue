.. _ethernet_en:

Ethernet Echo Server
=====================

Overview
---------

This sample runs Zephyr's TCP echo server with the HPMicro Ethernet controller and returns received payloads to the connected client.

* `Zephyr 4.4.0 echo server sample <https://docs.zephyrproject.org/4.4.0/samples/net/sockets/echo_server/README.html>`_

Application paths
------------------

.. code-block:: text

   zephyr/samples/net/sockets/echo_server

Hardware setup
---------------

Connect the board's Ethernet port to a network with the addressing service expected by the sample, or configure matching static addresses on the board and host.

Building
---------

Run from the west workspace root. The following commands use ``hpm6750evk2`` as an example:

.. code-block:: console

   west build -p always -b hpm6750evk2 -S ethernet zephyr/samples/net/sockets/echo_server

Expected behavior
------------------

The console prints the network address and listening port. Data sent by a TCP client is echoed back unchanged.

Notes
------

.. note::

   Host firewall and subnet settings must allow traffic to the sample's TCP port.
