=========================================
Docker Environment Configuration Guide
=========================================

The Docker image provides the dependencies and tools required for HPMicro Zephyr
development. The current image includes:

- ZCC 4.1.5
- Zephyr SDK 1.0.1 (the default toolchain)
- HPMicro OpenOCD
- Scripts for switching between ZCC and the Zephyr SDK

Source code is no longer packaged in the image. The host ``zephyr``,
``sdk_glue``, ``sdk_env``, ``modules``, and ``bootloader`` directories are
mounted separately into the container. Host source changes are therefore
immediately visible inside the container without rebuilding the image.

Prerequisites
=============

- Docker and Docker Compose installed on a Linux host
- A complete ZSG workspace with all module directories at the same level
- A connected USB debugger, such as CMSIS-DAP, when flashing or debugging

Workspace Layout
================

The ``docker-compose.yaml`` file is located in ``sdk_glue``. Before starting the
container, the workspace must contain at least:

.. code-block:: text

    ZSG/
    |-- zephyr/
    |-- sdk_glue/
    |   `-- docker-compose.yaml
    `-- sdk_env/hpm_sdk

.. note::

   Paths in the Compose file are relative to the ``sdk_glue`` directory. Run
   ``docker compose`` from that directory, or use ``-f`` to specify the Compose
   file explicitly.

Getting the Docker Image
========================

The current Compose configuration uses ``zephyr_hpmicro:v0.8.0``. Load the
released image package with:

.. code-block:: console

    docker load -i zephyr_hpmicro-v0.8.0.tar
    docker image ls zephyr_hpmicro

The image package is available from:

- `zephyr_hpmicro-v0.8.0.tar <https://resource.hpmicro.com/zephyr_hpmicro-v0.8.0.tar>`_

If pulling from a registry produces a repository-qualified image name, tag it
with the name expected by Compose:

.. code-block:: console

    docker pull swhpmicro/zephyr_hpmicro:v0.8.0
    docker tag swhpmicro/zephyr_hpmicro:v0.8.0 zephyr_hpmicro:v0.8.0

Starting and Managing the Container
===================================

#. Enter the ``sdk_glue`` directory and start the container:

   .. code-block:: console

       cd /path/to/ZSG/sdk_glue
       docker compose up -d

#. Enter the container:

   .. code-block:: console

       docker compose exec zephyr bash

#. Stop and remove the container when finished:

   .. code-block:: console

       docker compose down

``docker compose down`` does not remove source code on the host. Run
``docker compose up -d`` again to recreate the container.

Source Mounts
=============

Compose maps each host workspace module into ``/home/User/zephyr_space``:

.. list-table:: Source directory mappings
   :header-rows: 1

   * - Host path
     - Container path
   * - ``../zephyr``
     - ``/home/User/zephyr_space/zephyr``
   * - ``.`` (``sdk_glue``)
     - ``/home/User/zephyr_space/sdk_glue``
   * - ``../sdk_env``
     - ``/home/User/zephyr_space/sdk_env``
   * - ``../modules``
     - ``/home/User/zephyr_space/modules``
   * - ``../bootloader``
     - ``/home/User/zephyr_space/bootloader``

The default container working directory is ``/home/User/zephyr_space``. Because
these are bind mounts, changes made to source files inside the container are
also written directly to the host.

Toolchains
==========

The image contains both Zephyr SDK 1.0.1 and ZCC 4.1.5. No host toolchain mount
or additional toolchain installation is required.

Zephyr SDK 1.0.1
----------------

The Zephyr SDK is the default toolchain and is installed at
``/opt/toolchains/zephyr-sdk-1.0.1``. Check the active configuration with:

.. code-block:: console

    echo $ZEPHYR_TOOLCHAIN_VARIANT
    echo $ZEPHYR_SDK_INSTALL_DIR

The default values are ``zephyr`` and ``/opt/toolchains/zephyr-sdk-1.0.1``.

ZCC 4.1.5
---------

Switch to ZCC using the script included in the image:

.. code-block:: console

    source ~/zephyr_space/use-zcc.sh

Switch back to the Zephyr SDK with:

.. code-block:: console

    source ~/zephyr_space/use-zephyr-sdk.sh

.. note::

   Run these scripts with ``source`` so that they update the current shell.
   Each new shell starts with the default values from Compose; run the ZCC
   switch script again when ZCC is required.

OpenOCD, Building, and Flashing
===============================

HPMicro OpenOCD is packaged in the image, so it does not need to be installed
on the host or mounted into the container. Build, flash, and debug directly
after entering the container:

.. code-block:: console

    cd /home/User/zephyr_space/zephyr
    west build -p always -b hpm6750evk2 -S blinky samples/basic/blinky
    west flash
    west debug

Compose enables ``privileged: true`` and mounts ``/dev/bus/usb`` so OpenOCD can
access USB debuggers.

Using docker run
================

Compose is recommended because it avoids missing source or USB mounts. The
equivalent ``docker run`` command is:

.. code-block:: console

    docker run -it --rm --privileged --hostname docker \
        -v /path/to/ZSG/zephyr:/home/User/zephyr_space/zephyr \
        -v /path/to/ZSG/sdk_glue:/home/User/zephyr_space/sdk_glue \
        -v /path/to/ZSG/sdk_env:/home/User/zephyr_space/sdk_env \
        -v /path/to/ZSG/modules:/home/User/zephyr_space/modules \
        -v /path/to/ZSG/bootloader:/home/User/zephyr_space/bootloader \
        -v /dev/bus/usb:/dev/bus/usb \
        -w /home/User/zephyr_space \
        zephyr_hpmicro:v0.8.0

Replace ``/path/to/ZSG`` with the absolute path to the workspace on the host.

Troubleshooting
===============

Source Code Is Missing in the Container
---------------------------------------

Confirm that all five module directories are next to ``sdk_glue`` and that
Compose was started from ``sdk_glue``. Inspect the resolved mounts with:

.. code-block:: console

    docker compose config
    docker compose exec zephyr ls /home/User/zephyr_space

Toolchain Not Found
-------------------

Both the Zephyr SDK and ZCC are stored in the image; do not mount a host SDK.
Switch to the default toolchain and inspect its environment variables:

.. code-block:: console

    source ~/zephyr_space/use-zephyr-sdk.sh
    echo $ZEPHYR_TOOLCHAIN_VARIANT
    echo $ZEPHYR_SDK_INSTALL_DIR

OpenOCD Cannot Find the USB Device
----------------------------------

#. Run ``lsusb`` on the host and verify that the debugger is connected.
#. Keep ``privileged: true`` and ``/dev/bus/usb:/dev/bus/usb`` in Compose.
#. Reconnect the debugger and recreate the container.
