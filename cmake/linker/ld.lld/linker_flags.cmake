# Copyright (c) 2025-2026 HPMicro
# SPDX-License-Identifier: Apache-2.0

# Inherit Zephyr's current LLD property set before applying ZCC-specific
# options. This keeps properties added by newer Zephyr releases available.
include(${ZEPHYR_BASE}/cmake/linker/lld/linker_flags.cmake)

check_set_linker_property(TARGET linker PROPERTY memusage "${LINKERFLAGPREFIX},--print-memory-usage")

set_property(TARGET linker PROPERTY no_position_independent "${LINKERFLAGPREFIX},--no-pie")
check_set_linker_property(TARGET linker PROPERTY no_position_independent "-static")
check_set_linker_property(TARGET linker PROPERTY no_position_independent "-ffunction -sections")
check_set_linker_property(TARGET linker PROPERTY no_position_independent "-fdata-sections")
check_set_linker_property(TARGET linker PROPERTY no_position_independent "--target=riscv32-unknown-elf")
check_set_linker_property(TARGET linker PROPERTY no_position_independent "-mtune=andes-kavalan")
# check_set_linker_property(TARGET linker PROPERTY no_position_independent "-lclang_rt.builtins")


set_property(TARGET linker PROPERTY partial_linking "-r")
