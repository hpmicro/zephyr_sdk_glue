# Zephyr SDK Glue Overview
- This repository is the adhesive for HPM_SDK and Zephyr, which supports building a zephyr project
on HPMicro soc, providing drivers, devicetree and kconfig.
- It is also a west manifest repository, containing the manifest (west.yml).
- License under Apache-2.0.

## Directory Structure

| Name | Description |
|--------|--------|
| application | Complete project |
| boards | Boards and shields support files |
| cmake | Cmake extensions |
| docs | Documentation |
| drivers | Zephyr standard driver files |
| dts | Devicetree files |
| include | Header files |
| modules | Additional modules |
| snippets | HPMicro resources selection |
| soc | SoC specific source |
| zephyr | Zephyr build definition |

## Documentation

- Please refer to for [linux starting](docs/swdev/starting/linux.rst) more details about start building.
- Please refer to for [windows starting](docs/swdev/starting/windows.rst) more details about start building.

## Build Command

- Use west tool:

```bash
    west build -p always/auto -b ${board} -d ${build_path} -S ${snippet} ${project_path}
```

- Use cmake command:

```bash
    cmake -GNinja -B ${build_path} -DBOARD=${board} -DSNIPPET=${snippet} ${project_path}
    ninja -C ${build_path}
```



