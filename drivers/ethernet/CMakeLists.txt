# Copyright (c) 2025 HPMicro
# SPDX-License-Identifier: Apache-2.0

zephyr_library_amend()
zephyr_library_sources_ifdef(CONFIG_ETH_HPMICRO ethernet_hpmicro.c)

zephyr_include_directories_ifdef(CONFIG_ETH_PHY ${HPM_SDK_DIR}/components/enet_phy)

zephyr_include_directories_ifdef(CONFIG_ETH_PHY_RTL8211 ${HPM_SDK_DIR}/components/enet_phy/rtl8211)
zephyr_library_sources_ifdef(CONFIG_ETH_PHY_RTL8211 ${HPM_SDK_DIR}/components/enet_phy/rtl8211/hpm_rtl8211.c)

zephyr_include_directories_ifdef(CONFIG_ETH_PHY_RTL8201 ${HPM_SDK_DIR}/components/enet_phy/rtl8201)
zephyr_library_sources_ifdef(CONFIG_ETH_PHY_RTL8201 ${HPM_SDK_DIR}/components/enet_phy/rtl8201/hpm_rtl8201.c)
