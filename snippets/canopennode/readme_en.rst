.. _canopennode:

CANopen
========
`zephyr sample link <https://docs.zephyrproject.org/4.4.0/samples/modules/canopennode/README.html>`_

Overview
---------

The canopen demo provides the following features:

- Heartbeat
- NMT
- SDO
- PDO
- Emergency
- Storage
- OTA

Path
-----

.. code-block::

    zephyr/samples/modules/canopennode

Build Cmd
----------

As hpm6750evk2 for example:

.. code-block:: console

    west build -p always -b hpm6750evk2 -S canopennode samples/modules/canopennode

Board settings
---------------
The development board needs to be connected to the Toomoss Canbox, and the ECU-Bus Pro software needs to be opened to import the corresponding master test script.

Running the example
--------------------
When the example runs successfully, the console shows the following prints:

.. code-block:: console

    *** Booting Zephyr OS build zsg_v0.8.0 ***
    [00:00:00.004,000] <inf> fs_nvs: 8 Sectors of 4096 bytes
    [00:00:00.004,000] <inf> fs_nvs: alloc wra: 1, f80
    [00:00:00.004,000] <inf> fs_nvs: data wra: 1, 9c
    [00:00:00.005,000] <dbg> canopen_storage: canopen_settings_set: restored object dictionary ROM entries
    [00:00:00.006,000] <dbg> canopen_storage: canopen_settings_set: restored object dictionary EEPROM entries
    [00:00:00.006,000] <dbg> canopen_driver: CO_CANmodule_init: rxSize = 13, txSize = 9
    [00:00:00.006,000] <dbg> canopen_driver: CO_CANmodule_init: excessive number of concurrent CAN RX filters enabled (needs 13, 16 available)
    [00:00:00.007,000] <inf> app: CANopen stack initialized

In the ECU-Bus Pro software, after running the master script, you can confirm the test results through the Trace and Message columns. When you see the following message in the Message column:

.. code-block:: console

    [SUCCESS] 测试成功！

Meanwhile, when the Trace column shows the heartbeat messages (0x70a) and PDO messages (0x18a) appearing regularly, it indicates that the test was successful.

If you want to test the OTA functionality of canopen, you need to enable mcuboot. For details, please refer to: http://192.168.20.97/2025/08/20/%e5%9c%a8zephyr%e4%b8%ad%e4%bd%bf%e7%94%a8mcuboot/

After flashing the recompiled canopen sample firmware with mcuboot enabled (see note 2) to the development board, you will see the following printout:

.. code-block:: console

    *** Booting MCUboot 5af38164715e ***
    *** Using Zephyr OS build zsg_v0.8.0 ***
    I: Starting bootloader
    I: Primary image: magic=good, swap_type=0x1, copy_done=0x3, image_ok=0x1
    I: Scratch: magic=unset, swap_type=0x1, copy_done=0x3, image_ok=0x3
    I: Boot source: primary slot
    I: Image index: 0, Swap type: none
    I: Bootloader chainload address offset: 0x43000
    I: Image version: v0.0.0
    I: Jumping to the first image slot
    *** Booting Zephyr OS build zsg_v0.8.0 ***
    [00:00:00.004,000] <inf> fs_nvs: 8 Sectors of 4096 bytes
    [00:00:00.004,000] <inf> fs_nvs: alloc wra: 1, f80
    [00:00:00.004,000] <inf> fs_nvs: data wra: 1, 9c
    [00:00:00.005,000] <dbg> canopen_storage: canopen_settings_set: restored object dictionary ROM entries
    [00:00:00.006,000] <dbg> canopen_storage: canopen_settings_set: restored object dictionary EEPROM entries
    [00:00:00.006,000] <dbg> canopen_driver: CO_CANmodule_init: rxSize = 13, txSize = 9
    [00:00:00.006,000] <dbg> canopen_driver: CO_CANmodule_init: excessive number of concurrent CAN RX filters enabled (needs 13, 16 available)
    [00:00:00.007,000] <inf> app: CANopen stack initialized

In the ECU-Bus Pro software, run the master script with the MCUboot test and transfer the OTA upgrade firmware (see note 3). When the OTA transfer starts, you will see the following printout:

.. code-block:: console

    [00:00:00.007,000] <inf> app: CANopen stack initialized
    [00:00:00.007,000] <inf> app: Green LED not available
    [00:00:00.007,000] <inf> app: Red LED not available
    [00:03:19.664,000] <dbg> canopen_program: canopen_odf_1f51: program status = 1, cmd = 0
    [00:03:19.664,000] <dbg> canopen_program: canopen_program_cmd_stop: program stopped
    [00:03:19.694,000] <dbg> canopen_program: canopen_odf_1f51: program status = 0, cmd = 3
    [00:03:19.694,000] <dbg> canopen_program: canopen_program_cmd_clear: erasing flash area
    [00:03:28.315,000] <dbg> canopen_program: canopen_program_cmd_clear: program cleared
    [00:03:28.498,000] <dbg> canopen_program: canopen_odf_1f50: total = 84268

After successful transmission and verification, the development board will automatically restart and switch to the newly transferred firmware. At this point, you will see the following printout:

.. code-block:: console

    *** Booting MCUboot 5af38164715e ***
    *** Using Zephyr OS build zsg_v0.8.0 ***
    I: Starting bootloader
    I: Primary image: magic=good, swap_type=0x1, copy_done=0x3, image_ok=0x1
    I: Scratch: magic=unset, swap_type=0x1, copy_done=0x3, image_ok=0x3
    I: Boot source: primary slot
    I: Image index: 0, Swap type: test
    I: Starting swap using scratch algorithm.
    I: Bootloader chainload address offset: 0x43000
    I: Image version: v0.1.1
    I: Jumping to the first image slot
    *** Booting Zephyr OS build zsg_v0.8.0 ***
    [00:00:00.005,000] <inf> fs_nvs: 8 Sectors of 4096 bytes
    [00:00:00.005,000] <inf> fs_nvs: alloc wra: 1, f78
    [00:00:00.005,000] <inf> fs_nvs: data wra: 1, a8
    [00:00:00.006,000] <dbg> canopen_storage: canopen_settings_set: restored object dictionary ROM entries
    [00:00:00.007,000] <dbg> canopen_storage: canopen_settings_set: restored object dictionary EEPROM entries
    [00:00:00.007,000] <dbg> canopen_driver: CO_CANmodule_init: rxSize = 13, txSize = 9
    [00:00:00.007,000] <dbg> canopen_driver: CO_CANmodule_init: excessive number of concurrent CAN RX filters enabled (needs 13, 16 available)
    [00:00:00.007,000] <inf> app: CANopen stack initialized
    [00:00:00.007,000] <inf> app: ################################################
    [00:00:00.007,000] <inf> app: ##                                            ##
    [00:00:00.007,000] <inf> app: ##   ****   OTA UPDATE SUCCESS   ****         ##
    [00:00:00.007,000] <inf> app: ##   ****      VERSION 0.8.1     ****         ##
    [00:00:00.007,000] <inf> app: ##                                            ##
    [00:00:00.007,000] <inf> app: ################################################
    [00:00:00.007,000] <inf> app: Green LED not available
    [00:00:00.007,000] <inf> app: Red LED not available

Note:
    1. The ECU-Bus Pro clock frequency needs to be set to 40MHz, baud rate 500KHz, sample point 87.5%.
    2. The macros CONFIG_BOOTLOADER_MCUBOOT and CONFIG_MCUBOOT_BOOTLOADER_MODE_SWAP_SCRATCH need to be enabled in canopennode.conf when testing mcuboot. They are currently disabled by default.
    3. When testing the canopen OTA functionality, the initial version number of mcuboot_canopen is v0.0.0 by default, flashed to slot0; the OTA upgrade firmware is flashed to slot1 with version number v0.1.1. After flashing, mcuboot will automatically switch to run the firmware in slot1, and the printed version number will be v0.1.1, indicating a successful OTA upgrade; For OTA download address details, please refer to: https://docs.zephyrproject.org/latest/zephyr.pdf Chapter 4.9 Device Management.
    4. The sample firmware for OTA upgrade has been pre-built and placed in the /test_image/ directory under the current folder. Different boards require different firmware. Before flashing, confirm that the firmware path and name in the canopennode_master_mcuboot.ts script in the same directory are correct; if not, modify them manually. The currently default firmware is for hpm6750evk2.
    5. The OTA upgrade firmware is identical to the current firmware except for the printout and version number, and both can run normally.
