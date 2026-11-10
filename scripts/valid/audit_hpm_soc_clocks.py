# Copyright (c) 2026 HPMicro
# SPDX-License-Identifier: Apache-2.0

"""Audit HPMicro SoC boot clock ownership."""

from pathlib import Path
import re
import sys


ROOT = Path(__file__).resolve().parents[2]
SOC_ROOT = ROOT / "soc" / "hpmicro"

ALLOWED = {
    "HPM5100": {
        ("clock_cpu0", 0),
        ("clock_ahb", 0),
        ("clock_lmm0", 0),
        ("clock_mchtmr0", 0),
        ("clock_rom", 0),
        ("clock_gpio", 0),
        ("clock_hdma0", 0),
        ("clock_hdma1", 0),
        ("clock_xpi0", 0),
    },
    "HPM5300": {
        ("clock_cpu0", 0),
        ("clock_ahb", 0),
        ("clock_lmm0", 0),
        ("clock_mchtmr0", 0),
        ("clock_rom", 0),
        ("clock_gpio", 0),
        ("clock_hdma", 0),
        ("clock_xpi0", 0),
    },
    "HPM5E00": {
        ("clock_cpu0", 0),
        ("clock_mchtmr0", 0),
        ("clock_ahb0", 0),
        ("clock_axif", 0),
        ("clock_axis", 0),
        ("clock_axic", 0),
        ("clock_rom0", 0),
        ("clock_xpi0", 0),
        ("clock_lmm0", 0),
        ("clock_ram0", 0),
        ("clock_hdma", 0),
        ("clock_xdma", 0),
        ("clock_gpio", 0),
    },
    "HPM6200": {
        ("clock_cpu0", 0),
        ("clock_mchtmr0", 0),
        ("clock_ahbp", 0),
        ("clock_axic", 0),
        ("clock_axis", 0),
        ("clock_xpi0", 0),
        ("clock_xdma", 0),
        ("clock_hdma", 0),
        ("clock_ram0", 0),
        ("clock_lmm0", 0),
        ("clock_lmm1", 0),
        ("clock_gpio", 0),
        ("clock_cpu1", 1),
        ("clock_mchtmr1", 1),
    },
    "HPM6300": {
        ("clock_cpu0", 0),
        ("clock_mchtmr0", 0),
        ("clock_ahbp", 0),
        ("clock_axic", 0),
        ("clock_axis", 0),
        ("clock_xpi0", 0),
        ("clock_xdma", 0),
        ("clock_hdma", 0),
        ("clock_ram0", 0),
        ("clock_lmm0", 0),
        ("clock_gpio", 0),
    },
    "HPM6700": {
        ("clock_cpu0", 0),
        ("clock_mchtmr0", 0),
        ("clock_axi0", 0),
        ("clock_axi1", 0),
        ("clock_axi2", 0),
        ("clock_ahb", 0),
        ("clock_femc", 0),
        ("clock_xpi0", 0),
        ("clock_xpi1", 0),
        ("clock_xdma", 0),
        ("clock_ram0", 0),
        ("clock_ram1", 0),
        ("clock_pdma", 0),
        ("clock_gpio", 0),
        ("clock_hdma", 0),
        ("clock_lmm0", 0),
        ("clock_lmm1", 0),
        ("clock_mchtmr1", 1),
    },
    "HPM6800": {
        ("clock_cpu0", 0),
        ("clock_ahb", 0),
        ("clock_axic", 0),
        ("clock_axis", 0),
        ("clock_axiv", 0),
        ("clock_axid", 0),
        ("clock_axig", 0),
        ("clock_mchtmr0", 0),
        ("clock_xpi0", 0),
        ("clock_xram", 0),
        ("clock_gpio", 0),
        ("clock_hdma", 0),
        ("clock_xdma", 0),
    },
    "HPM6E00": {
        ("clock_cpu0", 0),
        ("clock_mchtmr0", 0),
        ("clock_ahb0", 0),
        ("clock_axif", 0),
        ("clock_axis", 0),
        ("clock_axic", 0),
        ("clock_axin", 0),
        ("clock_rom0", 0),
        ("clock_xpi0", 0),
        ("clock_lmm0", 0),
        ("clock_lmm1", 0),
        ("clock_ram0", 0),
        ("clock_ram1", 0),
        ("clock_hdma", 0),
        ("clock_xdma", 0),
        ("clock_gpio", 0),
        ("clock_cpu1", 1),
        ("clock_mchtmr1", 1),
    },
    "HPM6P00": {
        ("clock_cpu0", 0),
        ("clock_mchtmr0", 0),
        ("clock_ahb0", 0),
        ("clock_axif", 0),
        ("clock_axis", 0),
        ("clock_axic", 0),
        ("clock_rom0", 0),
        ("clock_xpi0", 0),
        ("clock_lmm0", 0),
        ("clock_lmm1", 0),
        ("clock_ram0", 0),
        ("clock_hdma", 0),
        ("clock_xdma", 0),
        ("clock_gpio", 0),
        ("clock_cpu1", 1),
        ("clock_mchtmr1", 1),
    },
}

CALL_RE = re.compile(r"clock_add_to_group\(\s*(clock_[A-Za-z0-9_]+)\s*,\s*([01])\s*\)")


def main() -> int:
    failures = []

    for series, allowed in sorted(ALLOWED.items()):
        soc = SOC_ROOT / series / "soc.c"
        for line_no, line in enumerate(soc.read_text(encoding="utf-8").splitlines(), 1):
            match = CALL_RE.search(line)
            if not match:
                continue
            item = (match.group(1), int(match.group(2)))
            if item not in allowed:
                failures.append(f"{soc.relative_to(ROOT)}:{line_no}: unexpected {item[0]} group {item[1]}")

    if failures:
        print("Unexpected SoC-level HPM clock enables:")
        print("\n".join(failures))
        return 1

    print("HPM SoC clock ownership audit passed.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
