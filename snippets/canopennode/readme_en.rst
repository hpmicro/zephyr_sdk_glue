.. _canopennode:

Canopen
========================
`zephyr sample link <https://docs.zephyrproject.org/3.7.0/samples/modules/canopennode/README.html>`_

Overview
----------

The canopen demo provide following features:
- Heartbeat
- NMT
- SDO
- Emergency

Path
---------------

.. code-block::

    zephyr/samples/modules/canopennode

Build Cmd
-----------

As hpm6750evk2 for example:

.. code-block:: console

    west build -p always -b hpm6750evk2 -S canopennode samples/modules/canopennode

Borad settings:
----------------
need a board connect to toomoss canbox

Running the example
--------------------
When the example runs successfully, the board shows the following prints:

.. code-block:: console

    *** Booting Zephyr OS build 36940db938a8 ***
    [00:00:00.000,000] <dbg> canopen_driver: CO_CANmodule_init: rxSize = 13, txSize = 9
    [00:00:00.001,000] <dbg> canopen_driver: CO_CANmodule_init: excessive number of concurrent CAN RX filters enabled (needs 13, 16 available)
    [00:00:00.001,000] <inf> app: CANopen stack initialized

    canbox settings:
    if driver is can, baudrate choose 1M and data baudrate choose 5M;
    if driver is mcan, baudrate choose 500k and data baudrate choose 2M;
    canbox will show receive messages:
    1	0x70A	1	7F	0.000000(time)	接收	标准帧	数据帧	CAN	CAN1	0x53800101(device_num)

    NMT test:
    (1) stop remote node
    send cmd:
    277	0x0	2	02 00	275.007680	发送	标准帧	数据帧	CAN	CAN1	0x53800101
    receive :
    278	0x70A	1	04	275.017340	接收	标准帧	数据帧	CAN	CAN1	0x53800101
    (2) enter pre-optional state
    send cmd:
    5	0x0	2	80 00	3.116790	发送	标准帧	数据帧	CAN	CAN1	0x53800101
    receive :
    6	0x70A	1	7F	4.001140	接收	标准帧	数据帧	CAN	CAN1	0x53800101
    (3) start remote node
    send cmd:
    4	0x0	2	01 00	2.391500	发送	标准帧	数据帧	CAN	CAN1	0x53800101
    receive :
    5	0x70A	1	05	3.001110	接收	标准帧	数据帧	CAN	CAN1	0x53800101
    (4) reset node
    send cmd:
    4	0x0	2	82 00	2.383120	发送	标准帧	数据帧	CAN	CAN1	0x53800101
    receive :
    5	0x70A	1	00	2.384540	接收	标准帧	数据帧	CAN	CAN1	0x53800101
    6	0x70A	1	7F	2.884110	接收	标准帧	数据帧	CAN	CAN1	0x53800101
    (5) reset communication
    send cmd:
    3	0x0	2	81 00	1.793990	发送	标准帧	数据帧	CAN	CAN1	0x53800101
    receive :
    4	0x70A	1	00	1.810980	接收	标准帧	数据帧	CAN	CAN1	0x53800101
    6	0x70A	1	7F	2.310900	接收	标准帧	数据帧	CAN	CAN1	0x53800101

    SDO test:
    (1) read SDO messages show as:
    this cmd will read index-1008 and will return 'Z', 'e', 'p', 'h', 'y', 'r', ' 
    ', 'R', 'T', 'O', 'S', '/', 'C', 'A', 'N', 'o', 'p', 'e', 'n', 'N', 'o', 'd', 'e'.
    4	0x60A	8	40 08 10 00 00 00 00 00	2.155810	发送	标准帧	数据帧	CAN	CAN1	0x53800101
    5	0x58A	8	41 08 10 00 17 00 00 00	2.156240	接收	标准帧	数据帧	CAN	CAN1	0x53800101
    6	0x60A	8	60 00 00 00 00 00 00 00	2.356310	发送	标准帧	数据帧	CAN	CAN1	0x53800101
    7	0x58A	8	00 5A 65 70 68 79 72 20	2.358200	接收	标准帧	数据帧	CAN	CAN1	0x53800101
    8	0x60A	8	70 00 00 00 00 00 00 00	2.556710	发送	标准帧	数据帧	CAN	CAN1	0x53800101
    9	0x58A	8	10 52 54 4F 53 2F 43 41	2.558200	接收	标准帧	数据帧	CAN	CAN1	0x53800101
    10	0x60A	8	60 00 00 00 00 00 00 00	2.757310	发送	标准帧	数据帧	CAN	CAN1	0x53800101
    11	0x58A	8	00 4E 6F 70 65 6E 4E 6F	2.758210	接收	标准帧	数据帧	CAN	CAN1	0x53800101
    12	0x60A	8	70 00 00 00 00 00 00 00	2.957410	发送	标准帧	数据帧	CAN	CAN1	0x53800101
    13	0x58A	8	1B 64 65 00 00 00 00 00	2.958230	接收	标准帧	数据帧	CAN	CAN1	0x53800101
    (2) write SDO messages show as:
    this cmd will write index-1017 and this 2bits is heartbeattime, default is 0x3e8
    28	0x60A	8	2B 17 10 00 E9 03 00 00	25.784190	发送	标准帧	数据帧	CAN	CAN1	0x53800101
    29	0x58A	8	60 17 10 00 00 00 00 00	25.786020	接收	标准帧	数据帧	CAN	CAN1	0x53800101
    read this index to verify if has write successfully
    158	0x60A	8	40 17 10 00 00 00 00 00	154.079570	发送	标准帧	数据帧	CAN	CAN1	0x53800101
    159	0x58A	8	4B 17 10 00 E9 03 00 00	154.080370	接收	标准帧	数据帧	CAN	CAN1	0x53800101
    (3) read times of button pressed
    this cmd read index-2120 and the content will record the button pressed times, and this also can be tested by PDO
    12	0x60A	8	40 02 21 00 00 00 00 00	10.160030	发送	标准帧	数据帧	CAN	CAN1	0x53800101
    13	0x58A	8	43 02 21 00 02(button) 00 00 00	10.161480	接收	标准帧	数据帧	CAN	CAN1	0x53800101


