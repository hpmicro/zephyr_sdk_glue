========================================
Windows Environment Configuration Guide
========================================

**Windows** users can install necessary tools via `Chocolatey`.

Installing Tools with Chocolatey
---------------------------------

#. Open Windows PowerShell as administrator to install Chocolatey

    .. code-block:: console

        Set-ExecutionPolicy AllSigned
        Set-ExecutionPolicy Bypass -Scope Process -Force; [System.Net.ServicePointManager]::SecurityProtocol = [System.Net.ServicePointManager]::SecurityProtocol -bor 3072; iex ((New-Object System.Net.WebClient).DownloadString('https://community.chocolatey.org/install.ps1'))

#. Open **cmd.exe** and disable installation confirmation prompts

    .. code-block:: console

        choco feature enable -n allowGlobalConfirmation

#. Install dependencies

    .. code-block:: console

        choco install cmake --installargs 'ADD_CMAKE_TO_PATH=System'
        choco install ninja gperf python git dtc-msys2 wget 7zip

#. Restart **cmd.exe**

Setting Up Workspace
---------------------

#. Install west

    .. code-block:: console
        
        pip3 install -U west

#. Create workspace directory and fetch source code

    .. code-block:: console

        mkdir %workspace%
        cd %workspace%
        west init -m https://github.com/hpmicro/zephyr_sdk_glue.git --mr main

#. Fetch the required repositories. GitHub is used by default; run the first command to switch to the Gitee source:

    .. code-block:: console

        west config manifest.file west_gitee.yml
        west update

#. Register the Zephyr installation path with CMake

    .. code-block:: console

        west zephyr-export

#. Install Zephyr Python dependencies

    .. code-block:: console

        pip3 install -r zephyr\scripts\requirements.txt

#. Apply HPM_SDK patches

    .. code-block:: console

        west supply

Configuring Toolchain
----------------------

Multiple toolchains are supported. Choose one based on your needs.

.. note::

   If you already have a RISC-V toolchain, you can skip the installation and configure environment variables directly. For detailed toolchain configuration and switching methods, refer to the :doc:`toolchain` section.

Zephyr SDK
^^^^^^^^^^^^^^^^^^^^^^^^^^

Zephyr SDK 1.0.1 is the default recommended toolchain. The minimal SDK bundle
is used together with the RISC-V GNU toolchain package.

Download from: `ZEPHYR-SDK <https://github.com/zephyrproject-rtos/sdk-ng/releases/>`_

#. Command-line installation

    .. code-block:: console

        cd %workspace%
        wget https://github.com/zephyrproject-rtos/sdk-ng/releases/download/v1.0.1/zephyr-sdk-1.0.1_windows-x86_64_minimal.7z
        7z x zephyr-sdk-1.0.1_windows-x86_64_minimal.7z

#. Configure the required toolchain variables

    .. code-block:: console

        cd zephyr-sdk-1.0.1
        setup.cmd /t riscv64-zephyr-elf /c

ZCC Toolchain
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

ZCC is an LLVM/Clang-based toolchain with smaller download size and fast compilation.

#. Download and extract ZCC toolchain

    Obtain the ZCC toolchain package from the appropriate channel and extract to a directory, e.g., ``C:\sdk_env\toolchains\zcc-4.1.5\``

#. Configure environment variables (in cmd)

    .. code-block:: console

        set ZEPHYR_TOOLCHAIN_VARIANT=zcc
        set ZCC_TOOLCHAIN_PATH=C:\sdk_env\toolchains\zcc-4.1.5\
        set TOOLCHAIN_ROOT=%workspace%\sdk_glue\

    It is recommended to add these to system environment variables for persistence.

GNU Cross-Compile Toolchain
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

If you already have a RISC-V GCC toolchain, you can use it directly.

#. Configure environment variables (in cmd)

    .. code-block:: console

        set ZEPHYR_TOOLCHAIN_VARIANT=cross-compile
        set CROSS_COMPILE=C:\path\to\riscv32-unknown-elf-gcc\bin\riscv32-unknown-elf-

    Replace ``C:\path\to\riscv32-unknown-elf-`` with your actual toolchain installation path

Building Zephyr Button Sample
------------------------------
    Build the button sample for hpm6750evk2. The build directory can be placed anywhere in the workspace (recommended under workspace/zephyr/)

#. Build and compile

    .. code-block:: console

        cd %workspace%\zephyr
        west build -p always -b hpm6750evk2 samples\basic\button

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

        west boards | findstr hpm

#. Start GDB server (connect board first)

    .. code-block:: console

        west debugserver

#. Generate HTML documentation

    .. code-block:: console

        cd sdk_glue/docs
        make html
