# Copyright (c) 2026 HPMicro
# SPDX-License-Identifier: Apache-2.0

"""Strictly audit HPMicro devicetree clocks against HPM Tool JSON data."""

from __future__ import annotations

import argparse
from dataclasses import dataclass
import json
from pathlib import Path
import re
import sys
from typing import Iterable


ROOT = Path(__file__).resolve().parents[2]
DTS_ROOT = ROOT / "dts" / "riscv" / "hpmicro"
HEADER_ROOT = ROOT / "include" / "dt-bindings" / "clock"

FAMILIES = {
    "51": ("hpm51xx.dtsi", "hpm5151-clocks.h", ["HPM5151"]),
    "53": ("hpm53xx.dtsi", None, ["HPM5301", "HPM5321", "HPM5331", "HPM5361"]),
    "5e": ("hpm5exx.dtsi", "hpm5e31-clocks.h", ["HPM5E11", "HPM5E31", "HPM5E3Y"]),
    "62": ("hpm62xx.dtsi", "hpm6280-clocks.h", [
        "HPM6220", "HPM6240", "HPM6260", "HPM6264", "HPM6280", "HPM6284",
    ]),
    "63": ("hpm63xx.dtsi", "hpm6360-clocks.h", [
        "HPM6320", "HPM6330", "HPM6340", "HPM6350", "HPM6360", "HPM6364",
    ]),
    "67": ("hpm67xx.dtsi", "hpm6750-clocks.h", [
        "HPM6430", "HPM6450", "HPM6454", "HPM64A0", "HPM64G0", "HPM6730",
        "HPM6750", "HPM6754",
    ]),
    "68": ("hpm68xx.dtsi", "hpm6880-clocks.h", [
        "HPM6830", "HPM6850", "HPM6880", "HPM68D2", "HPM68D3", "HPM68D4",
    ]),
    "6e": ("hpm6exx.dtsi", "hpm6e80-clocks.h", [
        "HPM6E50", "HPM6E60", "HPM6E6Y", "HPM6E70", "HPM6E80", "HPM6E84",
        "HPM6E8Y",
    ]),
    "6p": ("hpm6pxx.dtsi", "hpm6p81-clocks.h", ["HPM6P31", "HPM6P41", "HPM6P81"]),
}

PART_HEADERS = {
    "HPM5301": "hpm5301-clocks.h",
    "HPM5321": "hpm5361-clocks.h",
    "HPM5331": "hpm5361-clocks.h",
    "HPM5361": "hpm5361-clocks.h",
}

SPECIAL_PREFIXES = {
    "adc": "CLK_ADC_SRC_",
    "dac": "CLK_DAC_SRC_",
    "i2s": "CLK_I2S_SRC_",
    "pdm": "CLK_I2S_SRC_",
    "dao": "CLK_I2S_SRC_",
    "smix": "CLK_I2S_SRC_",
    "wdg": "CLK_WDG_SRC_",
    "ewdg": "CLK_WDG_SRC_",
    "watchdog": "CLK_WDG_SRC_",
    "pwdg": "CLK_PWDG_SRC_",
}

TOKEN_ALIASES = {
    "urt": "uart",
    "enet": "eth",
    "ethernet": "eth",
    "cam": "camera",
    "lcdc": "lcd",
    "display": "lcd",
    "watchdog": "wdg",
    "mcan": "can",
    "mchtmr": "mct",
    "mtimer": "mct",
    "gpt": "gptmr",
    "sd": "sdxc",
    "synt": "synth",
}

CLOCK_SPEC_RE = re.compile(
    r"<&clk\s+([A-Z][A-Z0-9_]*)\s+([A-Z0-9_]+|0)\s+(\d+)\s*>"
)
SOURCE_CLOCK_SPEC_RE = re.compile(r"<&(osc24|osc32)\s*>")
SECOND_CLOCK_SPEC_RE = re.compile(r"<&(pll\d+)\s+(\d+)\s+(\d+)\s*>")
NODE_NAME_RE = re.compile(r"\bnode-name\s*=\s*<\s*([A-Z][A-Z0-9_]*)\s*>")
DEFINE_RE = re.compile(r"^\s*#define\s+([A-Z][A-Z0-9_]*)\s+(.+?)\s*$", re.MULTILINE)
NODE_RE = re.compile(r"^[ \t]*([A-Za-z_][A-Za-z0-9_]*):[ \t]+[^\n{]+\{", re.MULTILINE)


@dataclass(frozen=True)
class ClockSpec:
    name: str
    source: str
    divider: int


@dataclass(frozen=True)
class SourceClockSpec:
    provider: str


@dataclass(frozen=True)
class SecondClockSpec:
    provider: str
    output: int
    divider: int


@dataclass
class SocData:
    name: str
    raw: dict
    functions: dict[str, dict]
    functions_by_macro: dict[str, dict]
    sources: dict[str, dict]
    direct: dict[str, dict]


def clock_macro(sdk_name: str) -> str:
    return sdk_name.upper()


def source_macro(source_id: str) -> str:
    compact = re.sub(r"[^a-z0-9]", "", source_id.lower())
    match = re.fullmatch(r"clk(\d+)pll(\d+)", compact)
    if not match:
        match = re.fullmatch(r"pll(\d+)clk(\d+)", compact)
        if match:
            return f"CLK_SRC_PLL{match.group(1)}_CLK{match.group(2)}"
    if match:
        return f"CLK_SRC_PLL{match.group(2)}_CLK{match.group(1)}"
    if compact in {"osc24m", "clk24m", "refclk24m"}:
        return "CLK_SRC_OSC24M"
    if compact in {"osc32k", "clk32k", "refclk32k"}:
        return "CLK_SRC_OSC32K"
    raise ValueError(f"unsupported source clock name: {source_id}")


def normalize_name(value: str) -> str:
    value = value.lower()
    value = re.sub(r"^(clock_|clk_top_)", "", value)
    match = re.fullmatch(r"([a-z_]+?)(\d+)?", value)
    if not match:
        return re.sub(r"[^a-z0-9]", "", value)
    stem = re.sub(r"[^a-z0-9]", "", match.group(1))
    stem = TOKEN_ALIASES.get(stem, stem)
    index = match.group(2) or ""
    return stem + index


def special_prefix(func: dict) -> str | None:
    normalized = normalize_name(func["sdkName"])
    stem = re.sub(r"\d+$", "", normalized)
    return SPECIAL_PREFIXES.get(stem)


def special_source_macro(prefix: str, source_func: dict) -> str:
    source = normalize_name(source_func["sdkName"]).upper()
    if source == "AHB":
        source = "AHB0"
    elif source == "AXI":
        source = "AXI0"
    elif source == "ANA":
        source = "ANA0"
    elif source == "AUD":
        source = "AUD0"
    suffix = source
    return prefix + suffix


def load_soc(json_root: Path, name: str) -> SocData:
    raw = json.loads((json_root / f"{name}.json").read_text(encoding="utf-8"))
    functions = {item["id"]: item for item in raw["funcClock"]}
    return SocData(
        name=name,
        raw=raw,
        functions=functions,
        functions_by_macro={clock_macro(item["sdkName"]): item for item in raw["funcClock"]},
        sources={item["id"]: item for item in raw["srcClock"]},
        direct={item["id"]: item for item in raw["directClock"]},
    )


def expected_spec(soc: SocData, func: dict) -> ClockSpec:
    source_id = func["defaultSource"]
    prefix = special_prefix(func)
    if source_id in soc.sources:
        source = source_macro(source_id)
        if prefix:
            source = prefix + source.removeprefix("CLK_SRC_")
    elif source_id in soc.functions:
        source_func = soc.functions[source_id]
        source = special_source_macro(prefix, source_func) if prefix else clock_macro(source_func["sdkName"])
    else:
        raise ValueError(f"{soc.name}: {func['sdkName']} has unknown source {source_id}")
    divider = int(func.get("divider", {}).get("defaultValue", 1))
    return ClockSpec(clock_macro(func["sdkName"]), source, divider)


def expected_direct_spec(item: dict) -> ClockSpec:
    return ClockSpec(f"CLOCK_{item['name'].upper()}", source_macro(item["from"]), 1)


def expected_direct_source_spec(item: dict) -> SourceClockSpec:
    providers = {"ref_clk24m": "osc24", "ref_clk32k": "osc32"}
    return SourceClockSpec(providers[item["from"]])


def expected_second_spec(soc: SocData, label: str, block: str) -> SecondClockSpec | None:
    node_name = NODE_NAME_RE.search(block)
    func = soc.functions_by_macro.get(node_name.group(1)) if node_name else None
    if not func:
        func = find_node_function(soc, label.removeprefix("clk_"))
    if not func:
        return None

    divider = 1
    visited = set()
    while func:
        if func["id"] in visited:
            raise ValueError(f"{soc.name}: cyclic clock source at {func['id']}")
        visited.add(func["id"])
        divider *= int(func.get("divider", {}).get("defaultValue", 1))
        source_id = func["defaultSource"]
        if source_id in soc.sources:
            source = soc.sources[source_id]
            if source["from"].startswith("pll"):
                return SecondClockSpec(source["from"], int(source["divIndex"]), divider)
            return None
        func = soc.functions.get(source_id)
    return None


def parse_macros(path: Path) -> set[str]:
    return {match.group(1) for match in DEFINE_RE.finditer(path.read_text(encoding="utf-8"))}


def models_for_file(family: str, path: Path) -> list[str]:
    common, _, models = FAMILIES[family]
    if path.name == common or path.name.endswith("_dualcore.dtsi"):
        return models
    stem = path.stem.upper()
    return [model for model in models if model == stem]


def header_for(family: str, model: str) -> Path:
    default = FAMILIES[family][1]
    return HEADER_ROOT / PART_HEADERS.get(model, default)


def iter_node_blocks(text: str) -> Iterable[tuple[str, str, int, int, int]]:
    for match in NODE_RE.finditer(text):
        depth = 0
        end = None
        for index in range(match.end() - 1, len(text)):
            if text[index] == "{":
                depth += 1
            elif text[index] == "}":
                depth -= 1
                if depth == 0:
                    end = index + 1
                    break
        if end is not None:
            yield (
                match.group(1),
                text[match.start():end],
                text.count("\n", 0, match.start()) + 1,
                match.start(),
                end,
            )


def find_node_function(soc: SocData, label: str) -> dict | None:
    label_key = normalize_name(label)
    label_keys = {label_key, re.sub(r"0$", "", label_key)}
    matches = []
    for func in soc.functions.values():
        keys = {
            normalize_name(func["id"]),
            normalize_name(func["name"]),
            normalize_name(func["sdkName"]),
        }
        expanded_keys = keys | {re.sub(r"0$", "", key) for key in keys}
        if label_keys & expanded_keys:
            matches.append(func)
    if len(matches) == 1:
        return matches[0]
    return None


def pll_frequency(raw_pll: dict, refs: dict[str, int]) -> int:
    ref = refs[raw_pll["from"]]
    params = raw_pll["params"]
    if raw_pll["version"] == "v2":
        mfi = params["mfi"]["defaultValue"]
        mfn = params["mfn"]["defaultValue"]
        mfd = params["mfd"]["defaultValue"]
        return round(ref * (mfi + mfn / mfd))
    refdiv = params["refdiv"]["defaultValue"]
    postdiv = params["postdiv"]["defaultValue"]
    if params["divtype"]["defaultValue"] == 1:
        factor = params["fbdiv_int"]["defaultValue"]
    else:
        factor = params["fbdiv_frac"]["defaultValue"] + params["frac"]["defaultValue"] / (2**24)
    return round(ref / refdiv * factor / postdiv)


def dts_frequency(block: str, property_name: str) -> int | None:
    match = re.search(rf"\b{re.escape(property_name)}\s*=\s*<\s*([^>]+)\s*>", block)
    if not match:
        return None
    value = match.group(1).strip()
    macro = re.fullmatch(r"DT_FREQ_([MK])\((\d+)\)", value)
    if macro:
        return int(macro.group(2)) * (1_000_000 if macro.group(1) == "M" else 1_000)
    return int(value, 0)


def dts_int(block: str, property_name: str) -> int | None:
    match = re.search(rf"\b{re.escape(property_name)}\s*=\s*<\s*(\d+)\s*>", block)
    return int(match.group(1)) if match else None


def clock_cells(block: str) -> int | None:
    match = re.search(r"#clock-cells\s*=\s*<\s*(\d+)\s*>", block)
    return int(match.group(1)) if match else None


def expected_root(soc: SocData) -> tuple[dict[str, int], dict[str, tuple[str, int, list[tuple[str, int, int]]]]]:
    refs = {item["id"]: round(item["defaultFrequency"] * 1_000_000) for item in soc.raw["refClock"]}
    outputs: dict[str, list[tuple[str, int, int]]] = {}
    for source in soc.raw["srcClock"]:
        if source["from"].startswith("pll"):
            outputs.setdefault(source["from"], []).append((
                source_macro(source["id"]),
                int(source["divIndex"]),
                int(source.get("divider", {}).get("defaultValue", 1)),
            ))
    plls = {
        pll["id"]: (pll["version"], pll_frequency(pll, refs), outputs.get(pll["id"], []))
        for pll in soc.raw["pllClock"]
    }
    return refs, plls


def audit_file(json_root: Path, family: str, path: Path, failures: list[str]) -> None:
    models = models_for_file(family, path)
    if not models:
        return
    text = path.read_text(encoding="utf-8")
    socs = [load_soc(json_root, model) for model in models]
    macro_sets = {soc.name: parse_macros(header_for(family, soc.name)) for soc in socs}

    for match in CLOCK_SPEC_RE.finditer(text):
        actual = ClockSpec(match.group(1), match.group(2), int(match.group(3)))
        if not actual.name.startswith("CLOCK_"):
            continue
        expected = []
        for soc in socs:
            func = soc.functions_by_macro.get(actual.name)
            direct = next((item for item in soc.direct.values() if f"CLOCK_{item['name'].upper()}" == actual.name), None)
            if func:
                expected.append((soc.name, expected_spec(soc, func)))
            elif direct:
                expected.append((soc.name, expected_direct_spec(direct)))
            else:
                continue
        if expected and any(spec != expected[0][1] for _, spec in expected):
            failures.append(f"{path.relative_to(ROOT)}: {actual.name} has model-specific defaults")
            continue
        if expected and actual != expected[0][1]:
            line = text.count("\n", 0, match.start()) + 1
            failures.append(
                f"{path.relative_to(ROOT)}:{line}: {actual} != {expected[0][1]} ({expected[0][0]})"
            )
        for soc_name, spec in expected:
            macros = macro_sets[soc_name]
            for macro in (spec.name, spec.source):
                if macro not in macros:
                    failures.append(f"{header_for(family, soc_name).name}: missing {macro} for {soc_name}")

    for label, block, line, _, _ in iter_node_blocks(text):
        if 'compatible = "hpmicro,hpm-second-clock"' in block:
            if clock_cells(block) != 0:
                failures.append(
                    f"{path.relative_to(ROOT)}:{line}: {label} must use #clock-cells = <0>"
                )
            specs = [expected_second_spec(soc, label, block) for soc in socs]
            if any(spec is None for spec in specs):
                missing = [soc.name for soc, spec in zip(socs, specs) if spec is None]
                failures.append(
                    f"{path.relative_to(ROOT)}:{line}: {label} cannot be mapped for {missing}"
                )
                continue
            if specs and all(spec == specs[0] for spec in specs):
                matches = list(SECOND_CLOCK_SPEC_RE.finditer(block))
                actual = SecondClockSpec(
                    matches[0].group(1), int(matches[0].group(2)), int(matches[0].group(3))
                ) if len(matches) == 1 else None
                if actual != specs[0]:
                    failures.append(
                        f"{path.relative_to(ROOT)}:{line}: {label} clocks {actual} != {specs[0]}"
                    )
            node_name = NODE_NAME_RE.search(block)
            if not node_name:
                failures.append(f"{path.relative_to(ROOT)}:{line}: {label} missing node-name")
            else:
                for soc in socs:
                    if node_name.group(1) not in macro_sets[soc.name]:
                        failures.append(
                            f"{header_for(family, soc.name).name}: missing {node_name.group(1)} "
                            f"for {soc.name}"
                        )
            continue

        funcs = [find_node_function(soc, label) for soc in socs]
        funcs = [func for func in funcs if func is not None]
        directs = [soc.direct.get(label.lower()) for soc in socs]
        directs = [item for item in directs if item is not None]
        if funcs:
            specs = [expected_spec(soc, find_node_function(soc, label)) for soc in socs if find_node_function(soc, label)]
            if specs and all(spec == specs[0] for spec in specs):
                matches = list(CLOCK_SPEC_RE.finditer(block))
                if not matches:
                    if label not in {"cpu0", "cpu1"}:
                        failures.append(f"{path.relative_to(ROOT)}:{line}: {label} missing clocks = <&clk {specs[0].name} {specs[0].source} {specs[0].divider}>")
                elif not any(
                    ClockSpec(match.group(1), match.group(2), int(match.group(3))) == specs[0]
                    for match in matches
                ):
                    failures.append(f"{path.relative_to(ROOT)}:{line}: {label} clocks do not contain {specs[0]}")
        elif directs:
            spec = expected_direct_spec(directs[0])
            if all(spec.name in macro_sets[soc.name] for soc in socs):
                if not CLOCK_SPEC_RE.search(block):
                    failures.append(f"{path.relative_to(ROOT)}:{line}: {label} missing clocks = <&clk {spec.name} {spec.source} {spec.divider}>")
            else:
                provider = expected_direct_source_spec(directs[0]).provider
                if not any(match.group(1) == provider for match in SOURCE_CLOCK_SPEC_RE.finditer(block)):
                    failures.append(
                        f"{path.relative_to(ROOT)}:{line}: {label} missing clocks = <&{provider}>"
                    )


def audit_roots(json_root: Path, failures: list[str]) -> None:
    for family, (common, _, models) in FAMILIES.items():
        root_signatures = []
        for model in models:
            soc = load_soc(json_root, model)
            refs, plls = expected_root(soc)
            signature = (refs, plls)
            root_signatures.append(signature)
        if any(signature != root_signatures[0] for signature in root_signatures[1:]):
            failures.append(f"{family}/{common}: models do not share one root clock configuration")
            continue

        refs, plls = root_signatures[0]
        path = DTS_ROOT / family / common
        text = path.read_text(encoding="utf-8")
        root_blocks = {label: (block, line) for label, block, line, _, _ in iter_node_blocks(text)}

        for label, ref_id in (("osc24", "ref_clk24m"), ("osc32", "ref_clk32k")):
            if label not in root_blocks:
                failures.append(f"{path.relative_to(ROOT)}: missing {label}")
                continue
            block, line = root_blocks[label]
            if 'compatible = "fixed-clock"' not in block:
                failures.append(
                    f"{path.relative_to(ROOT)}:{line}: {label} must use fixed-clock"
                )
            if clock_cells(block) != 0:
                failures.append(
                    f"{path.relative_to(ROOT)}:{line}: {label} must use #clock-cells = <0>"
                )
            actual = dts_frequency(block, "clock-frequency")
            if actual != refs[ref_id]:
                failures.append(
                    f"{path.relative_to(ROOT)}:{line}: {label} frequency {actual} != {refs[ref_id]}"
                )

        for pll_id, (version, frequency, outputs) in plls.items():
            if pll_id not in root_blocks:
                failures.append(f"{path.relative_to(ROOT)}: missing {pll_id}")
                continue
            block, line = root_blocks[pll_id]
            compatible = "hpmicro,hpm-pllv2-clock" if version == "v2" else "hpmicro,hpm-pll-clock"
            if f'compatible = "{compatible}"' not in block:
                failures.append(f"{path.relative_to(ROOT)}:{line}: {pll_id} must use {compatible}")
            if dts_frequency(block, "pll-frequency") != frequency:
                failures.append(
                    f"{path.relative_to(ROOT)}:{line}: {pll_id} frequency "
                    f"{dts_frequency(block, 'pll-frequency')} != {frequency}"
                )
            if dts_int(block, "pll-index") != int(pll_id.removeprefix("pll")):
                failures.append(f"{path.relative_to(ROOT)}:{line}: invalid {pll_id} pll-index")

            actual_outputs = []
            for child in re.finditer(r"\bclk\d+\s*\{(.*?)\};", block, re.DOTALL):
                body = child.group(1)
                name = re.search(r"\bsrc-name\s*=\s*<\s*([A-Z0-9_]+)\s*>", body)
                actual_outputs.append((
                    name.group(1) if name else "",
                    dts_int(body, "div-i"),
                    dts_int(body, "div-p"),
                ))
            if actual_outputs != outputs:
                failures.append(
                    f"{path.relative_to(ROOT)}:{line}: {pll_id} outputs {actual_outputs} != {outputs}"
                )

            for model in models:
                macros = parse_macros(header_for(family, model))
                for source_name, _, _ in outputs:
                    if source_name not in macros:
                        failures.append(f"{header_for(family, model).name}: missing {source_name} for {model}")

        for model in models:
            model_path = DTS_ROOT / family / f"{model.lower()}.dtsi"
            if not model_path.exists():
                continue
            model_text = model_path.read_text(encoding="utf-8")
            for override in re.finditer(r"&(?P<pll>pll\d+)\s*\{(?P<body>.*?)\};", model_text, re.DOTALL):
                actual = dts_frequency(override.group("body"), "pll-frequency")
                if actual is not None and actual != plls[override.group("pll")][1]:
                    failures.append(
                        f"{model_path.relative_to(ROOT)}: {override.group('pll')} override {actual} "
                        f"!= {plls[override.group('pll')][1]}"
                    )


def dependency_specs(soc: SocData, func: dict) -> list[ClockSpec]:
    specs = [expected_spec(soc, func)]
    if special_prefix(func) and func["defaultSource"] in soc.functions:
        specs.append(expected_spec(soc, soc.functions[func["defaultSource"]]))
    return specs


def format_clocks(
    indent: str, specs: list[ClockSpec | SourceClockSpec | SecondClockSpec], newline: str
) -> str:
    rendered = [
        f"<&clk {spec.name} {spec.source} {spec.divider}>"
        if isinstance(spec, ClockSpec) else
        f"<&{spec.provider} {spec.output} {spec.divider}>"
        if isinstance(spec, SecondClockSpec) else f"<&{spec.provider} 0 0>"
        for spec in specs
    ]
    if len(rendered) == 1:
        return f"{indent}clocks = {rendered[0]};"
    continuation = indent + ("\t" if indent.endswith("\t") else "    ")
    return f"{indent}clocks = {rendered[0]},{newline}" + \
        f"{continuation}{(',' + newline + continuation).join(rendered[1:])};"


def replace_or_insert_clocks(
    block: str, specs: list[ClockSpec | SourceClockSpec | SecondClockSpec], newline: str
) -> str:
    prop_re = re.compile(r"(?m)^(?P<indent>[ \t]*)clocks\s*=.*?;", re.DOTALL)
    peer = re.search(
        r"(?m)^(?P<indent>[ \t]+)(?:interrupt-parent|interrupt-names|interrupts|ranges|reg|compatible)\s*=",
        block,
    )
    if peer:
        indent = peer.group("indent")
    else:
        first_line = block.splitlines()[0]
        node_indent = re.match(r"[ \t]*", first_line).group(0)
        indent = node_indent + ("\t" if "\t" in node_indent or not node_indent else "    ")
    prop = prop_re.search(block)
    if prop:
        replacement = format_clocks(indent, specs, newline)
        return block[:prop.start()] + replacement + block[prop.end():]

    insertion_re = re.compile(
        r"^(?:[ \t]*)(?:interrupt-parent|interrupt-names|interrupts|ranges|reg|compatible)\s*=.*?;",
        re.MULTILINE | re.DOTALL,
    )
    candidates = list(insertion_re.finditer(block))
    if not candidates:
        return block
    insertion = candidates[-1].end()
    rendered = format_clocks(indent, specs, newline)
    return block[:insertion] + newline + rendered + block[insertion:]


def fix_file(json_root: Path, family: str, path: Path) -> bool:
    models = models_for_file(family, path)
    if not models:
        return False
    raw_bytes = path.read_bytes()
    newline = "\r\n" if b"\r\n" in raw_bytes else "\n"
    text = raw_bytes.decode("utf-8")
    socs = [load_soc(json_root, model) for model in models]
    macro_sets = {soc.name: parse_macros(header_for(family, soc.name)) for soc in socs}
    replacements: list[tuple[int, int, str]] = []

    for label, block, _, start, end in iter_node_blocks(text):
        funcs = [find_node_function(soc, label) for soc in socs]
        funcs = [func for func in funcs if func is not None]
        directs = [soc.direct.get(label.lower()) for soc in socs]
        directs = [item for item in directs if item is not None]
        desired: list[ClockSpec | SourceClockSpec | SecondClockSpec] | None = None

        if 'compatible = "hpmicro,hpm-second-clock"' in block:
            second_specs = [expected_second_spec(soc, label, block) for soc in socs]
            if all(spec is not None for spec in second_specs) and all(
                spec == second_specs[0] for spec in second_specs
            ):
                updated = replace_or_insert_clocks(block, [second_specs[0]], newline)
                if updated != block:
                    replacements.append((start, end, updated))
            continue

        if funcs:
            candidates = [dependency_specs(soc, find_node_function(soc, label)) for soc in socs if find_node_function(soc, label)]
            if candidates and all(specs == candidates[0] for specs in candidates):
                desired = candidates[0]
        elif directs:
            spec = expected_direct_spec(directs[0])
            if all(spec.name in macro_sets[soc.name] for soc in socs):
                desired = [spec]
            else:
                source_specs = [expected_direct_source_spec(item) for item in directs]
                if all(spec == source_specs[0] for spec in source_specs):
                    desired = [source_specs[0]]

        if desired is None or label in {"cpu0", "cpu1"}:
            continue

        if isinstance(desired[0], SourceClockSpec):
            updated = replace_or_insert_clocks(block, desired, newline)
            if updated != block:
                replacements.append((start, end, updated))
            continue

        existing = list(CLOCK_SPEC_RE.finditer(block))
        if existing and (not funcs or not special_prefix(funcs[0])):
            desired_name = desired[0].name
            matching = [match for match in existing if match.group(1) == desired_name]
            if matching:
                if len(existing) == 1:
                    updated = replace_or_insert_clocks(block, desired, newline)
                    if updated != block:
                        replacements.append((start, end, updated))
                    continue
                updated = block
                for match in reversed(matching):
                    rendered = f"<&clk {desired[0].name} {desired[0].source} {desired[0].divider}>"
                    updated = updated[:match.start()] + rendered + updated[match.end():]
                if updated != block:
                    replacements.append((start, end, updated))
                continue
            if len(existing) > 1:
                # Display nodes carry a separate PLL setup entry before the functional clock.
                continue

        updated = replace_or_insert_clocks(block, desired, newline)
        if updated != block:
            replacements.append((start, end, updated))

    if not replacements:
        return False
    for start, end, replacement in reversed(replacements):
        text = text[:start] + replacement + text[end:]
    path.write_bytes(text.encode("utf-8"))
    return True


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--clock-json-root", type=Path, required=True)
    parser.add_argument("--fix", action="store_true", help="rewrite mapped clocks before auditing")
    args = parser.parse_args()
    if not args.clock_json_root.is_dir():
        parser.error(f"clock JSON root does not exist: {args.clock_json_root}")

    if args.fix:
        changed = []
        for family in sorted(FAMILIES):
            for path in sorted((DTS_ROOT / family).glob("*.dtsi")):
                if fix_file(args.clock_json_root, family, path):
                    changed.append(str(path.relative_to(ROOT)))
        print(f"Updated {len(changed)} devicetree files.")

    failures: list[str] = []
    audit_roots(args.clock_json_root, failures)
    for family in sorted(FAMILIES):
        for path in sorted((DTS_ROOT / family).glob("*.dtsi")):
            audit_file(args.clock_json_root, family, path, failures)

    if failures:
        print("HPM devicetree clock audit failed:")
        print("\n".join(failures))
        return 1
    print("HPM devicetree clocks match all 44 HPM Tool JSON models.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
