# Copyright (c) 2026 HPMicro
# SPDX-License-Identifier: Apache-2.0

# Use Zephyr's current LLD library ordering. The compiler template resolves
# ZCC's runtime library and its platform-specific installation directory.
include(${ZEPHYR_BASE}/cmake/linker/lld/linker_libraries.cmake)
