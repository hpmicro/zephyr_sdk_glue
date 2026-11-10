=========================
Toolchain Switching Guide
=========================

Overview
========

Zephyr RTOS supports multiple toolchains, including GCC, Clang, IAR, and custom toolchains (such as zcc). Toolchain switching is implemented through environment variables and the CMake configuration system. This document uses the zcc toolchain as an example to explain how Zephyr implements toolchain switching and adaptation.

Supported Toolchains
--------------------

.. list-table::
   :header-rows: 1
   :widths: 20 30 50

   * - Toolchain
     - Variable Value
     - Description
   * - Zephyr SDK 1.0.1
     - ``zephyr``
     - Default recommended toolchain for sdk_glue, with RISC-V GNU support
   * - ZCC
     - ``zcc``
     - LLVM/Clang-based toolchain
   * - Cross-compile
     - ``cross-compile``
     - Generic cross-compilation toolchain

Using the ZCC Toolchain
=======================

Environment Variable Setup
--------------------------

Before using the ZCC toolchain, you need to set the following environment variables:

.. code-block:: bash

    export ZEPHYR_TOOLCHAIN_VARIANT=zcc
    export ZCC_TOOLCHAIN_PATH=/path/to/zcc-4.1.x/
    export TOOLCHAIN_ROOT=/path/to/sdk_glue/

.. note::

    You can add these environment variables to ``~/.bashrc`` or create a script file (e.g., ``zcc.sh``) for easy switching.

Environment Variable Description
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. list-table::
   :header-rows: 1
   :widths: 30 70

   * - Environment Variable
     - Description
   * - ``ZEPHYR_TOOLCHAIN_VARIANT``
     - Specifies the toolchain type to use. Zephyr loads the corresponding toolchain configuration based on this variable.
   * - ``ZCC_TOOLCHAIN_PATH``
     - Installation path of the ZCC toolchain. Should point to the toolchain's root directory (parent of the bin directory), for example, ``~/sdk_env/toolchains/zcc-4.1.5/``.
   * - ``TOOLCHAIN_ROOT``
     - Root directory of toolchain adaptation files, typically pointing to the path containing the ``cmake/`` directory.

Quick Start
-----------

#. Create an environment configuration script

    .. code-block:: bash

        cat > ~/zcc.sh << 'EOF'
        export ZEPHYR_TOOLCHAIN_VARIANT=zcc
        export ZCC_TOOLCHAIN_PATH=~/sdk_env/toolchains/zcc-4.1.5/
        export TOOLCHAIN_ROOT=~/workspace/sdk_glue/
        EOF

#. Activate the ZCC toolchain environment

    .. code-block:: bash

        source ~/zcc.sh

#. Build your project

    .. code-block:: bash

        west build -b hpm6750evk2 samples/hello_world

Switching to Other Toolchains
-----------------------------

Switch to Zephyr SDK
^^^^^^^^^^^^^^^^^^^^

.. code-block:: bash

    export ZEPHYR_TOOLCHAIN_VARIANT=zephyr
    unset ZCC_TOOLCHAIN_PATH
    unset TOOLCHAIN_ROOT

Switch to GNU Cross-Compile Toolchain
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. code-block:: bash

    export ZEPHYR_TOOLCHAIN_VARIANT=cross-compile
    export CROSS_COMPILE=/path/to/riscv32-unknown-elf-gcc/bin/riscv32-unknown-elf-

CMake Configuration Structure
=============================

Zephyr's toolchain configuration uses a modular design with the following components:

.. code-block:: text

    cmake/
    ├── toolchain/          # Toolchain general configuration
    │   └── zcc/
    │       ├── generic.cmake    # Toolchain initialization
    │       └── target.cmake     # Target platform specific configuration
    ├── compiler/           # Compiler configuration
    │   └── zcc/
    │       ├── generic.cmake    # Compiler general configuration
    │       ├── target.cmake     # Compiler target configuration
    │       └── target_riscv.cmake  # RISC-V architecture specific configuration
    ├── bintools/          # Binary tools configuration
    │   └── zcc/
    │       ├── target.cmake         # Binary tool discovery
    │       └── target_bintools.cmake # Binary tool properties configuration
    └── linker/            # Linker configuration
        └── ld.lld/
            └── target.cmake  # LLD linker configuration

Toolchain Loading Process
=========================

Toolchain Initialization
------------------------

Toolchain initialization is performed in ``cmake/toolchain/zcc/generic.cmake``, with the following main tasks:

#. Verify that the ``ZCC_TOOLCHAIN_PATH`` environment variable is set
#. Confirm that the toolchain path exists
#. Set toolchain-related variables

.. code-block:: cmake

    zephyr_get(ZCC_TOOLCHAIN_PATH)
    assert(ZCC_TOOLCHAIN_PATH "ZCC_TOOLCHAIN_PATH is not set")

    if(NOT EXISTS ${ZCC_TOOLCHAIN_PATH}) 
      message(FATAL_ERROR "Nothing found at ZCC_TOOLCHAIN_PATH: '${ZCC_TOOLCHAIN_PATH}'")
    endif()

    set(TOOLCHAIN_HOME ${ZCC_TOOLCHAIN_PATH})
    set(COMPILER zcc)
    set(LINKER ld.lld)
    set(BINTOOLS zcc)
    set(CROSS_COMPILE ${TOOLCHAIN_HOME}/bin/)

Compiler Configuration
----------------------

Compiler configuration is divided into two phases:

General Configuration
^^^^^^^^^^^^^^^^^^^^^

In ``cmake/compiler/zcc/generic.cmake``:

- Set compiler name (CC = zcc)
- Find C compiler executable
- Verify compiler availability

.. code-block:: cmake

    set_ifndef(CC zcc)
    find_program(CMAKE_C_COMPILER ${CROSS_COMPILE}${CC} PATHS ${TOOLCHAIN_HOME} NO_DEFAULT_PATH)

    if(CMAKE_C_COMPILER STREQUAL CMAKE_C_COMPILER-NOTFOUND)
      message(FATAL_ERROR "Zephyr was unable to find the toolchain...")
    endif()

Target Configuration
^^^^^^^^^^^^^^^^^^^^

In ``cmake/compiler/zcc/target.cmake``:

- Configure C++ compiler (if enabled)
- Set standard library include paths
- Configure compiler flags (e.g., ``-Wall``, ``-ffunction-sections``, etc.)
- Handle sysroot configuration

Architecture-Specific Configuration
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

For RISC-V architecture, ``cmake/compiler/zcc/target_riscv.cmake`` configures specific ABI and ISA extensions:

.. code-block:: cmake

    if(CONFIG_64BIT)
        string(CONCAT riscv_mabi ${riscv_mabi} "64")
        string(CONCAT riscv_march ${riscv_march} "64")
    endif()

    if(CONFIG_FPU)
        if(CONFIG_FLOAT_HARD)
            string(CONCAT riscv_mabi ${riscv_mabi} "d")
        endif()
        string(CONCAT riscv_march ${riscv_march} "fd")
    endif()

    list(APPEND TOOLCHAIN_C_FLAGS -mabi=${riscv_mabi} -march=${riscv_march})

Binary Tools Configuration
--------------------------

Binary tools configuration is located in ``cmake/bintools/zcc/`` and is responsible for finding and configuring the following tools:

.. list-table::
   :header-rows: 1
   :widths: 30 70

   * - Tool
     - Description
   * - ``llvm-objcopy``
     - ELF file conversion tool
   * - ``llvm-objdump``
     - Disassembly tool
   * - ``llvm-ar``
     - Archive tool
   * - ``llvm-ranlib``
     - Archive index tool
   * - ``llvm-readelf``
     - ELF file reading tool
   * - ``llvm-nm``
     - Symbol listing tool
   * - ``llvm-strip``
     - Symbol stripping tool
   * - ``zdb``
     - Debugger

Tool discovery example:

.. code-block:: cmake

    find_program(CMAKE_OBJCOPY ${CROSS_COMPILE}llvm-objcopy PATHS ${TOOLCHAIN_HOME} NO_DEFAULT_PATH)
    find_program(CMAKE_OBJDUMP ${CROSS_COMPILE}llvm-objdump PATHS ${TOOLCHAIN_HOME} NO_DEFAULT_PATH)

Linker Configuration
--------------------

Linker configuration is performed in ``cmake/linker/ld.lld/target.cmake``:

#. Find linker: Get linker path through compiler, or directly find ``ld.lld``
#. Set linker variable ``CMAKE_LINKER``
#. Configure linker flag prefix (``-Wl``)
#. Handle C++ exception configuration
#. Linker script preprocessing and generation

.. code-block:: cmake

    execute_process(COMMAND ${CMAKE_C_COMPILER} --print-prog-name=ld.lld
                    OUTPUT_VARIABLE ZCCLD_LINKER
                    OUTPUT_STRIP_TRAILING_WHITESPACE)

    set(CMAKE_LINKER ${ZCCLD_LINKER})
    set_ifundefined(LINKERFLAGPREFIX -Wl)

Toolchain Switching Mechanism
=============================

Conditional Loading Based on Environment Variables
---------------------------------------------------

Zephyr's build system dynamically loads corresponding toolchain configurations based on the value of the ``ZEPHYR_TOOLCHAIN_VARIANT`` environment variable:

- ``ZEPHYR_TOOLCHAIN_VARIANT=zcc`` → Load configuration from ``cmake/toolchain/zcc/``
- ``ZEPHYR_TOOLCHAIN_VARIANT=zephyr`` → Load Zephyr SDK toolchain configuration

Adding a New Toolchain
======================

To add new toolchain support for Zephyr, you need to:

#. Create toolchain directory structure

    .. code-block:: text

        cmake/
        ├── toolchain/<toolchain_name>/
        ├── compiler/<toolchain_name>/
        ├── bintools/<toolchain_name>/
        └── linker/<linker_name>/

#. Implement configuration files

    - ``toolchain/generic.cmake``: Initialize toolchain
    - ``compiler/generic.cmake``: Configure compiler
    - ``compiler/target.cmake``: Compiler target configuration
    - ``bintools/target.cmake``: Configure binary tools
    - ``bintools/target_bintools.cmake``: Set tool properties

#. Set environment variables

    - Define the value for ``ZEPHYR_TOOLCHAIN_VARIANT``
    - Define toolchain path-related environment variables

#. Test and verify

    - Verify toolchain path is set correctly
    - Verify all required tools can be found
    - Verify compilation and linking process works correctly

Troubleshooting
===============

Q: Error "ZCC_TOOLCHAIN_PATH is not set"
----------------------------------------

Ensure the environment variable is set correctly:

.. code-block:: bash

    export ZCC_TOOLCHAIN_PATH=/path/to/zcc-4.1.x/
    echo $ZCC_TOOLCHAIN_PATH

Q: Error "Nothing found at ZCC_TOOLCHAIN_PATH"
----------------------------------------------

Check if the path is correct and ensure the ``bin/zcc`` executable exists under the path:

.. code-block:: bash

    ls $ZCC_TOOLCHAIN_PATH/bin/zcc

Q: Toolchain not found during compilation
-----------------------------------------

Ensure ``TOOLCHAIN_ROOT`` points to the path containing the ``cmake/toolchain/zcc/`` directory:

.. code-block:: bash

    export TOOLCHAIN_ROOT=/path/to/sdk_glue/
    ls $TOOLCHAIN_ROOT/cmake/toolchain/zcc/

Q: How to check which toolchain is being used
---------------------------------------------

Look for information similar to the following in the build output:

.. code-block:: text

    -- Found toolchain: zcc (/path/to/zcc-4.1.x/)
