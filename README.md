# Zephyr SDK Glue Overview
- This repository is the adhesive for HPM_SDK and Zephyr, which supports building a zephyr project
on HPMicro soc, providing drivers, devicetree and kconfig.
- It is also a west manifest repository, containing the manifest (west.yml).
- License under Apache-2.0.

## Directory Structure

| Name | Description |
|--------|--------|
| application | complete project |
| boards | board support files |
| cmake | cmake extensions |
| docs | documentation |
| drivers | Zephyr standard driver files |
| dts | devicetree files |
| include | header files |
| modules | additional modules |
| soc | SoC specific source |
| zephyr | zephyr build definition |

## SDK documentation

- Please refer to for [linux starting](docs/source/swdev/starting/linux.rst) more details about start building.
- Please refer to for [windows starting](docs/source/swdev/starting/windows.rst) more details about start building.


##TODO: