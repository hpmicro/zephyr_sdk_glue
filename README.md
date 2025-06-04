# Zephyr SDK Glue Overview
Zephyr SDK Glue is a manifest repository developed by HPMicro based on Zephyr project. This repository contains all source program files adapted by HPMicro for its own MCUs in Zephyr project. Together with HPMicro's official software development kit (SDK), it forms the HPMicro chip development suite for Zephyr project.

The integration approach of Zephyr SDK Glue differs from conventional Zephyr project integration in the following aspects:
- Independent Manifest File: Unlike traditional Zephyr project that retrieve required source code from the Zephyr repository, **Glue** uses its own manifest file. When building the workspace, all required source code must be fetched starting from the Glue repository.
- Direct Dependence on HPM_SDK: Glue does not require the Zephyr HAL repository and is developed directly based on the HPM_SDK repository.
- Version Binding: This repository is bound to Zephyr v3.7.0 (LTS) and undergoes related iterations on this version basis.

License under Apache-2.0.

## Directory Structure

| Name | Description |
|--------|--------|
| boards | Boards and shields support files |
| cmake | Cmake extensions |
| docs | Documentation |
| drivers | Zephyr standard driver files |
| dts | Devicetree files |
| include | Header files |
| modules | Additional modules |
| samples | samples |
| snippets | HPMicro resources selection |
| soc | SoC specific source |
| zephyr | Zephyr build definition |

## Documentation

- Please refer to for [linux starting](docs/source/zh/starting/linux.rst) more details about start building.
- Please refer to for [windows starting](docs/source/zh/starting/windows.rst) more details about start building.

## Build Command

- Use west tool:

```bash
    west build -p always/auto -b ${board} -d ${build_path} -S ${snippet} -T ${sample.case} ${project_path}
```

- Use cmake command:

```bash
    cmake -GNinja -B ${build_path} -DBOARD=${board} -DSNIPPET=${snippet} ${project_path}
    ninja -C ${build_path}
```

