======================================
Linux Environment Configuration Guide
======================================

**Linux** (using Ubuntu with bash) requires manual installation of the hpmicro riscv-openocd tool. Use the default installation path: `/usr/local/bin/openocd`.


Installing Prerequisites
--------------------------

#. Update APT package lists

    .. code-block:: console

        sudo apt update
        sudo apt upgrade

#. Install dependencies

    .. code-block:: console

        sudo apt install --no-install-recommends git cmake ninja-build gperf \
            ccache dfu-util device-tree-compiler wget \
            python3-dev python3-pip python3-setuptools python3-tk python3-wheel xz-utils file \
            make gcc gcc-multilib g++-multilib libsdl2-dev libmagic1

#. Verify tool versions (minimum requirements below). Upgrade if versions are insufficient

.. list-table::
   :header-rows: 1

   * - Tool
     - Min. Version

   * - `CMake <https://cmake.org/>`_
     - 3.20.5

   * - `Python <https://www.python.org/>`_
     - 3.8

   * - `Devicetree compiler <https://www.devicetree.org/>`_
     - 1.4.6


Setting Up Workspace
----------------------

#. Install west and add `~/.local/bin`` to ~/.bashrc

    .. code-block:: console
        
        pip3 install --user -U west
        echo 'export PATH=~/.local/bin:"$PATH"' >> ~/.bashrc
        source ~/.bashrc

#. Create workspace directory and fetch source code

    .. code-block:: console

        mkdir ${workspace}
        cd ${workspace}
        west init -m https://github.com/hpmicro/zephyr_sdk_glue.git --mr main

#. Fetch repositories (use the first command to switch to a domestic mirror in China, default github):

    .. code-block:: console

        west config manifest.file west_gitee.yml # For domestic mirror
        west update

#. Configure CMake variables

    .. code-block:: console

        west zephyr-export

#. Install Zephyr Python dependencies

    .. code-block:: console

        pip3 install --user -r ~/${workspace}/zephyr/scripts/requirements.txt

#. Apply HPM_SDK patches

    .. code-block:: console

        west supply

Installing Zephyr Toolchain
-----------------------------
    Download the Zephyr SDK from `ZEPHYR-SDK <https://github.com/zephyrproject-rtos/sdk-ng/tags/>`_
    
#. Command-line installation

    .. code-block:: console

        cd ${workspace}
        wget https://github.com/zephyrproject-rtos/sdk-ng/releases/download/v0.16.5/zephyr-sdk-0.16.5_linux-x86_64.tar.xz
        wget -O - https://github.com/zephyrproject-rtos/sdk-ng/releases/download/v0.16.5/sha256.sum | shasum --check --ignore-missing
        tar xvf zephyr-sdk-0.16.5_linux-x86_64.tar.xz

#. Configure toolchain environment variables

    .. code-block:: console

        cd zephyr-sdk-0.16.5
        source setup.sh

Building Zephyr Button Sample
------------------------------
    Build the button sample for hpm6750evk2. The build directory can be placed anywhere in the workspace (recommended under workspace/zephyr/)

#. Build and compile

    .. code-block:: console

        cd ${workspace}/zephyr
        west build -p always -b hpm6750evk2 samples/basic/button

`-p` option: `always` for clean build, `auto` for incremental build.
`-S` option: Apply hardware-specific configurations.
    
    .. code-block:: console

        west build -p always -b hpm6750evk2 -S blinky_pwm samples/basic/blinky_pwm    

#. Flashing or Debugging

    .. code-block:: console
        
        west flash / west debug

Additional Commands
---------------------
    Useful commands:

#. Kconfig configuration

    .. code-block:: console

        west build -t menuconfig

#. List available boards

    .. code-block:: console

        west boards | grep hpm

#. Start GDB server (connect board first)

    .. code-block:: console

        west debugserver

#. Generate HTML documentation

    .. code-block:: console

        cd sdk_glue/docs
        make html

