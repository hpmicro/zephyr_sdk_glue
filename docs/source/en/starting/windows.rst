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

#. Fetch repositories (use first command for domestic mirror in China)

    .. code-block:: console

        west config manifest.file west_gitee.yml
        west update

#. Configure CMake variables

    .. code-block:: console

        west zephyr-export

#. Install Zephyr Python dependencies
    .. code-block:: console

        pip3 install -r %workspace%\zephyr\scripts\requirements.txt

#. Apply HPM_SDK patches

    .. code-block:: console

        west supply

Installing Zephyr Toolchain
----------------------------
    Download Zephyr SDK `ZEPHYR-SDK <https://github.com/zephyrproject-rtos/sdk-ng/tags/>`_
    
#. Command-line installation

    .. code-block:: console

        cd %workspace%
        wget https://github.com/zephyrproject-rtos/sdk-ng/releases/download/v0.16.5/zephyr-sdk-0.16.5_windows-x86_64.7z
        7z x zephyr-sdk-0.16.5_windows-x86_64.7z

#. Configure toolchain environment variables

    .. code-block:: console

        cd zephyr-sdk-0.16.5
        setup.cmd

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

        west boards | grep hpm

#. Start GDB server (connect board first)

    .. code-block:: console

        west debugserver

#. Generate HTML documentation

    .. code-block:: console

        cd sdk_glue/docs
        make html
