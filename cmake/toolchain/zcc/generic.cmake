# Copyright (c) 2025-2026 HPMicro
# SPDX-License-Identifier: Apache-2.0

zephyr_get(ZCC_TOOLCHAIN_PATH)
assert(    ZCC_TOOLCHAIN_PATH "ZCC_TOOLCHAIN_PATH is not set")

# Environment variables use native path separators. Convert them before the
# paths are passed through CMake command lists, where Windows backslashes can
# otherwise be interpreted as escape sequences.
file(TO_CMAKE_PATH "${ZCC_TOOLCHAIN_PATH}" ZCC_TOOLCHAIN_PATH)
file(TO_CMAKE_PATH "${TOOLCHAIN_ROOT}" TOOLCHAIN_ROOT)

if(NOT EXISTS "${ZCC_TOOLCHAIN_PATH}")
  message(FATAL_ERROR "Nothing found at ZCC_TOOLCHAIN_PATH: '${ZCC_TOOLCHAIN_PATH}'")
endif()

set(TOOLCHAIN_HOME ${ZCC_TOOLCHAIN_PATH})
set(COMPILER zcc)
set(LINKER ld.lld)
set(BINTOOLS zcc)
set(CROSS_COMPILE ${TOOLCHAIN_HOME}/bin/)
set(C++ z++)

message(STATUS "Found toolchain: zcc (${ZCC_TOOLCHAIN_PATH})")
