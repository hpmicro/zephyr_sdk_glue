.. _canopennode:

CANopen
========
`zephyr sample link <https://docs.zephyrproject.org/4.4.0/samples/modules/canopennode/README.html>`_

概述
-----

canopen demo提供以下功能:

- Heartbeat
- NMT
- SDO
- PDO
- Emergency
- Storage
- OTA

路径
-----

.. code-block::

    zephyr/samples/modules/canopennode

命令行
-------

以hpm6750evk2 为例:

.. code-block:: console

    west build -p always -b hpm6750evk2 -S canopennode samples/modules/canopennode

硬件设置
---------
需要将开发板连接到 toomoss canbox，同时打开软件 ecu-bus pro，导入相应的主站测试脚本

运行现象
---------
当示例成功运行时，控制台打印以下内容:

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

在 ecu-bus pro 软件中，运行主站脚本后，可以通过 Trace 和 Message 栏确认测试结果，当看到 Mesage 栏出现以下信息时：

.. code-block:: console

    [SUCCESS] 测试成功！

同时， Trace 栏可以看到心跳信息（0x70a）和 PDO 信息（0x18a）规律出现时，表示测试成功。

如果想要测试 canopen 的 OTA 功能，需要打开 mcuboot，具体参考：http://192.168.20.97/2025/08/20/%e5%9c%a8zephyr%e4%b8%ad%e4%bd%bf%e7%94%a8mcuboot/

将打开 mcuboot 重新编译的 canopen 示例固件（可见注释2）烧录到开发板后，可以看到以下打印内容：

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

在 ecu-bus pro 软件中，运行带 mcuboot 测试的主站脚本，传输 OTA 升级的固件（可见注释3），当开始 OTA 传输时，可以看到以下打印内容：

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

传输成功，并成功通过校验后，开发板会自动重启并切换到传输的新固件，此时可以看到以下打印内容：

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

注意：
    1. ecu bus pro clock freq 需要设置为 40MHz， 波特率 500KHz, 采样频率 87.5%
    2. 宏 CONFIG_BOOTLOADER_MCUBOOT，CONFIG_MCUBOOT_BOOTLOADER_MODE_SWAP_SCRATCH，测试 mcuboot 时需要再 canopennode.conf 开启，当前默认关闭
    3. 测试 canopen OTA 功能时，mcuboot_canopen 初始版本版本号默认为 v0.0.0，烧写地址为 slot0； OTA 升级的固件烧写地址为 slot1，版本号为 v0.1.1；烧写完毕后，mcuboot 会自动切换到 slot1 的固件运行，运行后打印的版本号为 v0.1.1， OTA 升级成功；关于 OTA 升级烧写地址的描述，可以参考 zephyr 官方文档：https://docs.zephyrproject.org/latest/zephyr.pdf 4.9 章 Device Management
    4. OTA 升级的示例固件已经全部编好放在当前目录 /test_image/ 下面，不同的板子需要烧录不同的固件，烧录前，确认在同级目录下 canopennode_master_mcuboot.ts 脚本中，烧写的固件路径和名称是否正确，如果不是需要手动修改，当前默认烧录的固件是 hpm6750evk2 的固件
    5. OTA 升级的固件除了打印和版本号不同，其他内容和当前固件完全一样，都是可以正常运行的

