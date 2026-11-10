/*
 * Copyright (c) 2026 HPMicro
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef HPM_CLOCK_TREE_CLOCK_CATALOG_H_
#define HPM_CLOCK_TREE_CLOCK_CATALOG_H_

#if defined(CONFIG_SOC_HPM5151)
#define HPM_PLL_IS_V1 0
#define HPM_PLL_COUNT 3U
#define HPM_CLOCK_LIST(X) \
	X(clock_cpu0) \
	X(clock_mchtmr0) \
	X(clock_gptmr0) \
	X(clock_gptmr1) \
	X(clock_i2c0) \
	X(clock_i2c1) \
	X(clock_i2c2) \
	X(clock_spi0) \
	X(clock_spi1) \
	X(clock_spi2) \
	X(clock_spi3) \
	X(clock_uart0) \
	X(clock_uart1) \
	X(clock_uart2) \
	X(clock_uart3) \
	X(clock_can0) \
	X(clock_xpi0) \
	X(clock_ref0) \
	X(clock_ref1) \
	X(clock_ahb) \
	X(clock_watchdog0) \
	X(clock_watchdog1) \
	X(clock_pwdg) \
	X(clock_lmm0) \
	X(clock_mbx0) \
	X(clock_acmp0) \
	X(clock_acmp1) \
	X(clock_opa0) \
	X(clock_opa1) \
	X(clock_opa2) \
	X(clock_opa3) \
	X(clock_mot0) \
	X(clock_gpio) \
	X(clock_hdma0) \
	X(clock_hdma1) \
	X(clock_rom) \
	X(clock_tsns) \
	X(clock_usb0) \
	X(clock_ptpc) \
	X(clock_pwm0) \
	X(clock_pwm1) \
	X(clock_pwm2) \
	X(clock_pwm3) \
	X(clock_emds) \
	X(clock_ptmr) \
	X(clock_puart) \
	X(clock_pgpio) \
	X(clock_ana0) \
	X(clock_ana1) \
	X(clock_ana2) \
	X(clock_ana3) \
	X(clock_adc0) \
	X(clock_adc1) \
	X(clock_adc2) \
	X(clock_adc3) \
	X(clock_aud0) \
	X(clock_aud1) \
	X(clock_i2s0) \
	X(clock_i2s1)

#define HPM_PLL_OUTPUT_LIST(X) \
	X(clk_pll0clk0, 0, 0) \
	X(clk_pll0clk1, 0, 1) \
	X(clk_pll1clk0, 1, 0) \
	X(clk_pll1clk1, 1, 1) \
	X(clk_pll1clk2, 1, 2) \
	X(clk_pll2clk0, 2, 0) \
	X(clk_pll2clk1, 2, 1)

#elif defined(CONFIG_SOC_HPM5301)
#define HPM_PLL_IS_V1 0
#define HPM_PLL_COUNT 2U
#define HPM_CLOCK_LIST(X) \
	X(clock_cpu0) \
	X(clock_mchtmr0) \
	X(clock_gptmr0) \
	X(clock_gptmr1) \
	X(clock_i2c0) \
	X(clock_i2c1) \
	X(clock_i2c2) \
	X(clock_i2c3) \
	X(clock_spi0) \
	X(clock_spi1) \
	X(clock_spi2) \
	X(clock_spi3) \
	X(clock_uart0) \
	X(clock_uart1) \
	X(clock_uart2) \
	X(clock_uart3) \
	X(clock_uart4) \
	X(clock_xpi0) \
	X(clock_ref0) \
	X(clock_ref1) \
	X(clock_ahb) \
	X(clock_watchdog0) \
	X(clock_watchdog1) \
	X(clock_pwdg) \
	X(clock_lmm0) \
	X(clock_mbx0) \
	X(clock_crc0) \
	X(clock_acmp0) \
	X(clock_mot0) \
	X(clock_kman) \
	X(clock_gpio) \
	X(clock_hdma) \
	X(clock_rom) \
	X(clock_tsns) \
	X(clock_usb0) \
	X(clock_ptmr) \
	X(clock_puart) \
	X(clock_pgpio) \
	X(clock_ana0) \
	X(clock_ana1) \
	X(clock_adc0) \
	X(clock_ana2) \
	X(clock_ana3)

#define HPM_PLL_OUTPUT_LIST(X) \
	X(clk_pll0clk0, 0, 0) \
	X(clk_pll0clk1, 0, 1) \
	X(clk_pll0clk2, 0, 2) \
	X(clk_pll1clk0, 1, 0) \
	X(clk_pll1clk1, 1, 1) \
	X(clk_pll1clk2, 1, 2) \
	X(clk_pll1clk3, 1, 3)

#elif defined(CONFIG_SOC_HPM5361)
#define HPM_PLL_IS_V1 0
#define HPM_PLL_COUNT 2U
#define HPM_CLOCK_LIST(X) \
	X(clock_cpu0) \
	X(clock_mchtmr0) \
	X(clock_can0) \
	X(clock_can1) \
	X(clock_can2) \
	X(clock_can3) \
	X(clock_gptmr0) \
	X(clock_gptmr1) \
	X(clock_gptmr2) \
	X(clock_gptmr3) \
	X(clock_i2c0) \
	X(clock_i2c1) \
	X(clock_i2c2) \
	X(clock_i2c3) \
	X(clock_spi0) \
	X(clock_spi1) \
	X(clock_spi2) \
	X(clock_spi3) \
	X(clock_uart0) \
	X(clock_uart1) \
	X(clock_uart2) \
	X(clock_uart3) \
	X(clock_uart4) \
	X(clock_uart5) \
	X(clock_uart6) \
	X(clock_uart7) \
	X(clock_xpi0) \
	X(clock_ref0) \
	X(clock_ref1) \
	X(clock_ahb) \
	X(clock_watchdog0) \
	X(clock_watchdog1) \
	X(clock_pwdg) \
	X(clock_lmm0) \
	X(clock_mbx0) \
	X(clock_crc0) \
	X(clock_acmp0) \
	X(clock_opa0) \
	X(clock_opa1) \
	X(clock_mot0) \
	X(clock_rng) \
	X(clock_sdp) \
	X(clock_kman) \
	X(clock_gpio) \
	X(clock_hdma) \
	X(clock_rom) \
	X(clock_tsns) \
	X(clock_usb0) \
	X(clock_ptpc) \
	X(clock_ptmr) \
	X(clock_puart) \
	X(clock_pgpio) \
	X(clock_ana0) \
	X(clock_ana1) \
	X(clock_adc0) \
	X(clock_adc1) \
	X(clock_ana2) \
	X(clock_ana3) \
	X(clock_dac0) \
	X(clock_dac1)

#define HPM_PLL_OUTPUT_LIST(X) \
	X(clk_pll0clk0, 0, 0) \
	X(clk_pll0clk1, 0, 1) \
	X(clk_pll0clk2, 0, 2) \
	X(clk_pll1clk0, 1, 0) \
	X(clk_pll1clk1, 1, 1) \
	X(clk_pll1clk2, 1, 2) \
	X(clk_pll1clk3, 1, 3)

#elif defined(CONFIG_SOC_HPM5E31)
#define HPM_PLL_IS_V1 0
#define HPM_PLL_COUNT 3U
#define HPM_CLOCK_LIST(X) \
	X(clock_cpu0) \
	X(clock_mchtmr0) \
	X(clock_ahb0) \
	X(clock_axif) \
	X(clock_axis) \
	X(clock_axic) \
	X(clock_gptmr0) \
	X(clock_gptmr1) \
	X(clock_gptmr2) \
	X(clock_gptmr3) \
	X(clock_owire0) \
	X(clock_owire1) \
	X(clock_i2c0) \
	X(clock_i2c1) \
	X(clock_i2c2) \
	X(clock_i2c3) \
	X(clock_spi0) \
	X(clock_spi1) \
	X(clock_spi2) \
	X(clock_spi3) \
	X(clock_uart0) \
	X(clock_uart1) \
	X(clock_uart2) \
	X(clock_uart3) \
	X(clock_uart4) \
	X(clock_uart5) \
	X(clock_uart6) \
	X(clock_uart7) \
	X(clock_can0) \
	X(clock_can1) \
	X(clock_can2) \
	X(clock_can3) \
	X(clock_xpi0) \
	X(clock_esc0) \
	X(clock_eth0) \
	X(clock_ptp0) \
	X(clock_ntmr0) \
	X(clock_ref0) \
	X(clock_ref1) \
	X(clock_eui0) \
	X(clock_eui1) \
	X(clock_crc0) \
	X(clock_tsns) \
	X(clock_mbx0) \
	X(clock_gpio) \
	X(clock_ppi0) \
	X(clock_hdma) \
	X(clock_lobs) \
	X(clock_acmp0) \
	X(clock_ptpc) \
	X(clock_mot0) \
	X(clock_qei0) \
	X(clock_qei1) \
	X(clock_qeo0) \
	X(clock_qeo1) \
	X(clock_sdm0) \
	X(clock_plb0) \
	X(clock_pwm0) \
	X(clock_pwm1) \
	X(clock_emds) \
	X(clock_watchdog0) \
	X(clock_watchdog1) \
	X(clock_xdma) \
	X(clock_rom0) \
	X(clock_ram0) \
	X(clock_usb0) \
	X(clock_lmm0) \
	X(clock_puart) \
	X(clock_ptmr) \
	X(clock_pwdg) \
	X(clock_ana0) \
	X(clock_ana1) \
	X(clock_adc0) \
	X(clock_adc1)

#define HPM_PLL_OUTPUT_LIST(X) \
	X(clk_pll0clk0, 0, 0) \
	X(clk_pll0clk1, 0, 1) \
	X(clk_pll1clk0, 1, 0) \
	X(clk_pll1clk1, 1, 1) \
	X(clk_pll1clk2, 1, 2) \
	X(clk_pll2clk0, 2, 0) \
	X(clk_pll2clk1, 2, 1)

#elif defined(CONFIG_SOC_HPM6280)
#define HPM_PLL_IS_V1 0
#define HPM_PLL_COUNT 3U
#define HPM_CLOCK_LIST(X) \
	X(clock_cpu0) \
	X(clock_mchtmr0) \
	X(clock_cpu1) \
	X(clock_mchtmr1) \
	X(clock_xpi0) \
	X(clock_gptmr0) \
	X(clock_gptmr1) \
	X(clock_gptmr2) \
	X(clock_gptmr3) \
	X(clock_uart0) \
	X(clock_uart1) \
	X(clock_uart2) \
	X(clock_uart3) \
	X(clock_uart4) \
	X(clock_uart5) \
	X(clock_uart6) \
	X(clock_uart7) \
	X(clock_i2c0) \
	X(clock_i2c1) \
	X(clock_i2c2) \
	X(clock_i2c3) \
	X(clock_spi0) \
	X(clock_spi1) \
	X(clock_spi2) \
	X(clock_spi3) \
	X(clock_can0) \
	X(clock_can1) \
	X(clock_can2) \
	X(clock_can3) \
	X(clock_lin0) \
	X(clock_lin1) \
	X(clock_lin2) \
	X(clock_lin3) \
	X(clock_ahb) \
	X(clock_axi) \
	X(clock_axic) \
	X(clock_axis) \
	X(clock_ahbp) \
	X(clock_ptpc) \
	X(clock_ref0) \
	X(clock_ref1) \
	X(clock_watchdog0) \
	X(clock_watchdog1) \
	X(clock_puart) \
	X(clock_ptmr) \
	X(clock_pwdg) \
	X(clock_sdp) \
	X(clock_xdma) \
	X(clock_rom) \
	X(clock_ram0) \
	X(clock_usb0) \
	X(clock_kman) \
	X(clock_gpio) \
	X(clock_mbx0) \
	X(clock_hdma) \
	X(clock_rng) \
	X(clock_mot0) \
	X(clock_mot1) \
	X(clock_mot2) \
	X(clock_mot3) \
	X(clock_crc0) \
	X(clock_acmp0) \
	X(clock_synt) \
	X(clock_sdm0) \
	X(clock_mbx1) \
	X(clock_lmm0) \
	X(clock_lmm1) \
	X(clock_tsns) \
	X(clock_ana0) \
	X(clock_ana1) \
	X(clock_ana2) \
	X(clock_adc0) \
	X(clock_adc1) \
	X(clock_adc2) \
	X(clock_ana3) \
	X(clock_ana4) \
	X(clock_dac0) \
	X(clock_dac1)

#define HPM_PLL_OUTPUT_LIST(X) \
	X(clk_pll0clk0, 0, 0) \
	X(clk_pll0clk1, 0, 1) \
	X(clk_pll0clk2, 0, 2) \
	X(clk_pll1clk0, 1, 0) \
	X(clk_pll1clk1, 1, 1) \
	X(clk_pll2clk0, 2, 0) \
	X(clk_pll2clk1, 2, 1)

#elif defined(CONFIG_SOC_HPM6360)
#define HPM_PLL_IS_V1 0
#define HPM_PLL_COUNT 3U
#define HPM_CLOCK_LIST(X) \
	X(clock_cpu0) \
	X(clock_mchtmr0) \
	X(clock_femc) \
	X(clock_xpi0) \
	X(clock_xpi1) \
	X(clock_gptmr0) \
	X(clock_gptmr1) \
	X(clock_gptmr2) \
	X(clock_gptmr3) \
	X(clock_uart0) \
	X(clock_uart1) \
	X(clock_uart2) \
	X(clock_uart3) \
	X(clock_uart4) \
	X(clock_uart5) \
	X(clock_uart6) \
	X(clock_uart7) \
	X(clock_i2c0) \
	X(clock_i2c1) \
	X(clock_i2c2) \
	X(clock_i2c3) \
	X(clock_spi0) \
	X(clock_spi1) \
	X(clock_spi2) \
	X(clock_spi3) \
	X(clock_can0) \
	X(clock_can1) \
	X(clock_sdxc0) \
	X(clock_ntmr0) \
	X(clock_ahb) \
	X(clock_axi) \
	X(clock_axic) \
	X(clock_axis) \
	X(clock_ahbp) \
	X(clock_ptpc) \
	X(clock_ref0) \
	X(clock_ref1) \
	X(clock_watchdog0) \
	X(clock_watchdog1) \
	X(clock_puart) \
	X(clock_ptmr) \
	X(clock_pwdg) \
	X(clock_eth0) \
	X(clock_ptp0) \
	X(clock_sdp) \
	X(clock_xdma) \
	X(clock_rom) \
	X(clock_ram0) \
	X(clock_usb0) \
	X(clock_kman) \
	X(clock_gpio) \
	X(clock_mbx0) \
	X(clock_hdma) \
	X(clock_rng) \
	X(clock_mot0) \
	X(clock_mot1) \
	X(clock_acmp0) \
	X(clock_pdm) \
	X(clock_dao) \
	X(clock_synt) \
	X(clock_ffa0) \
	X(clock_lmm0) \
	X(clock_tsns) \
	X(clock_ana0) \
	X(clock_ana1) \
	X(clock_ana2) \
	X(clock_adc0) \
	X(clock_adc1) \
	X(clock_adc2) \
	X(clock_ana3) \
	X(clock_dac0) \
	X(clock_aud0) \
	X(clock_aud1) \
	X(clock_i2s0) \
	X(clock_i2s1)

#define HPM_PLL_OUTPUT_LIST(X) \
	X(clk_pll0clk0, 0, 0) \
	X(clk_pll0clk1, 0, 1) \
	X(clk_pll0clk2, 0, 2) \
	X(clk_pll1clk0, 1, 0) \
	X(clk_pll1clk1, 1, 1) \
	X(clk_pll2clk0, 2, 0) \
	X(clk_pll2clk1, 2, 1)

#elif defined(CONFIG_SOC_HPM6750)
#define HPM_PLL_IS_V1 1
#define HPM_PLL_COUNT 5U
#define HPM_CLOCK_LIST(X) \
	X(clock_cpu0) \
	X(clock_cpu1) \
	X(clock_mchtmr0) \
	X(clock_mchtmr1) \
	X(clock_axi0) \
	X(clock_axi1) \
	X(clock_axi2) \
	X(clock_ahb) \
	X(clock_femc) \
	X(clock_xpi0) \
	X(clock_xpi1) \
	X(clock_gptmr0) \
	X(clock_gptmr1) \
	X(clock_gptmr2) \
	X(clock_gptmr3) \
	X(clock_gptmr4) \
	X(clock_gptmr5) \
	X(clock_gptmr6) \
	X(clock_gptmr7) \
	X(clock_uart0) \
	X(clock_uart1) \
	X(clock_uart2) \
	X(clock_uart3) \
	X(clock_uart4) \
	X(clock_uart5) \
	X(clock_uart6) \
	X(clock_uart7) \
	X(clock_uart8) \
	X(clock_uart9) \
	X(clock_uart10) \
	X(clock_uart11) \
	X(clock_uart12) \
	X(clock_uart13) \
	X(clock_uart14) \
	X(clock_uart15) \
	X(clock_i2c0) \
	X(clock_i2c1) \
	X(clock_i2c2) \
	X(clock_i2c3) \
	X(clock_spi0) \
	X(clock_spi1) \
	X(clock_spi2) \
	X(clock_spi3) \
	X(clock_can0) \
	X(clock_can1) \
	X(clock_can2) \
	X(clock_can3) \
	X(clock_display) \
	X(clock_sdxc0) \
	X(clock_sdxc1) \
	X(clock_camera0) \
	X(clock_camera1) \
	X(clock_ntmr0) \
	X(clock_ntmr1) \
	X(clock_ptpc) \
	X(clock_ref0) \
	X(clock_ref1) \
	X(clock_watchdog0) \
	X(clock_watchdog1) \
	X(clock_watchdog2) \
	X(clock_watchdog3) \
	X(clock_puart) \
	X(clock_ptmr) \
	X(clock_pwdg) \
	X(clock_eth0) \
	X(clock_eth1) \
	X(clock_ptp0) \
	X(clock_ptp1) \
	X(clock_sdp) \
	X(clock_xdma) \
	X(clock_rom) \
	X(clock_ram0) \
	X(clock_ram1) \
	X(clock_usb0) \
	X(clock_usb1) \
	X(clock_jpeg) \
	X(clock_pdma) \
	X(clock_kman) \
	X(clock_gpio) \
	X(clock_mbx0) \
	X(clock_mbx1) \
	X(clock_hdma) \
	X(clock_rng) \
	X(clock_mot0) \
	X(clock_mot1) \
	X(clock_mot2) \
	X(clock_mot3) \
	X(clock_acmp0) \
	X(clock_pdm) \
	X(clock_dao) \
	X(clock_synt) \
	X(clock_lmm0) \
	X(clock_lmm1) \
	X(clock_ana0) \
	X(clock_ana1) \
	X(clock_ana2) \
	X(clock_adc0) \
	X(clock_adc1) \
	X(clock_adc2) \
	X(clock_adc3) \
	X(clock_aud0) \
	X(clock_aud1) \
	X(clock_aud2) \
	X(clock_i2s0) \
	X(clock_i2s1) \
	X(clock_i2s2) \
	X(clock_i2s3)

#define HPM_PLL_OUTPUT_LIST(X) \
	X(clk_pll0clk0, 0, 0) \
	X(clk_pll1clk0, 1, 0) \
	X(clk_pll1clk1, 1, 1) \
	X(clk_pll2clk0, 2, 0) \
	X(clk_pll2clk1, 2, 1) \
	X(clk_pll3clk0, 3, 0) \
	X(clk_pll4clk0, 4, 0)

#elif defined(CONFIG_SOC_HPM6880)
#define HPM_PLL_IS_V1 0
#define HPM_PLL_COUNT 5U
#define HPM_CLOCK_LIST(X) \
	X(clock_axif) \
	X(clock_axis) \
	X(clock_axic) \
	X(clock_axiv) \
	X(clock_axig) \
	X(clock_axid) \
	X(clock_ahb) \
	X(clock_cpu0) \
	X(clock_mchtmr0) \
	X(clock_gpu0) \
	X(clock_can0) \
	X(clock_can1) \
	X(clock_can2) \
	X(clock_can3) \
	X(clock_can4) \
	X(clock_can5) \
	X(clock_can6) \
	X(clock_can7) \
	X(clock_lin0) \
	X(clock_lin1) \
	X(clock_lin2) \
	X(clock_lin3) \
	X(clock_lin4) \
	X(clock_lin5) \
	X(clock_lin6) \
	X(clock_lin7) \
	X(clock_i2c0) \
	X(clock_i2c1) \
	X(clock_i2c2) \
	X(clock_i2c3) \
	X(clock_spi0) \
	X(clock_spi1) \
	X(clock_spi2) \
	X(clock_spi3) \
	X(clock_uart0) \
	X(clock_uart1) \
	X(clock_uart2) \
	X(clock_uart3) \
	X(clock_uart4) \
	X(clock_uart5) \
	X(clock_uart6) \
	X(clock_uart7) \
	X(clock_gptmr0) \
	X(clock_gptmr1) \
	X(clock_gptmr2) \
	X(clock_gptmr3) \
	X(clock_gptmr4) \
	X(clock_gptmr5) \
	X(clock_gptmr6) \
	X(clock_gptmr7) \
	X(clock_xpi0) \
	X(clock_lmm0) \
	X(clock_xram) \
	X(clock_ddr0) \
	X(clock_eth0) \
	X(clock_ptp0) \
	X(clock_sdxc0) \
	X(clock_sdxc1) \
	X(clock_ntm0) \
	X(clock_ref0) \
	X(clock_ref1) \
	X(clock_camera0) \
	X(clock_camera1) \
	X(clock_lcd0) \
	X(clock_lcd1) \
	X(clock_csi0) \
	X(clock_csi1) \
	X(clock_dsi0) \
	X(clock_dsi1) \
	X(clock_gwc0) \
	X(clock_gwc1) \
	X(clock_lvb) \
	X(clock_lcb) \
	X(clock_ffa0) \
	X(clock_tsns) \
	X(clock_ptpc) \
	X(clock_watchdog0) \
	X(clock_watchdog1) \
	X(clock_puart) \
	X(clock_ptmr) \
	X(clock_pwdg) \
	X(clock_sdp) \
	X(clock_xdma) \
	X(clock_rom) \
	X(clock_usb0) \
	X(clock_jpeg) \
	X(clock_pdma) \
	X(clock_kman) \
	X(clock_gpio) \
	X(clock_mbx0) \
	X(clock_mbx1) \
	X(clock_hdma) \
	X(clock_rng) \
	X(clock_pdm) \
	X(clock_dao) \
	X(clock_smix) \
	X(clock_ana0) \
	X(clock_ana1) \
	X(clock_adc0) \
	X(clock_adc1) \
	X(clock_crc0) \
	X(clock_oamp) \
	X(clock_sdm0) \
	X(clock_aud0) \
	X(clock_aud1) \
	X(clock_aud2) \
	X(clock_aud3) \
	X(clock_i2s0) \
	X(clock_i2s1) \
	X(clock_i2s2) \
	X(clock_i2s3)

#define HPM_PLL_OUTPUT_LIST(X) \
	X(clk_pll0clk0, 0, 0) \
	X(clk_pll1clk0, 1, 0) \
	X(clk_pll1clk1, 1, 1) \
	X(clk_pll2clk0, 2, 0) \
	X(clk_pll2clk1, 2, 1) \
	X(clk_pll3clk0, 3, 0) \
	X(clk_pll4clk0, 4, 0)

#elif defined(CONFIG_SOC_HPM6E80)
#define HPM_PLL_IS_V1 0
#define HPM_PLL_COUNT 3U
#define HPM_CLOCK_LIST(X) \
	X(clock_cpu0) \
	X(clock_mchtmr0) \
	X(clock_cpu1) \
	X(clock_mchtmr1) \
	X(clock_ahb0) \
	X(clock_axif) \
	X(clock_axis) \
	X(clock_axic) \
	X(clock_axin) \
	X(clock_gptmr0) \
	X(clock_gptmr1) \
	X(clock_gptmr2) \
	X(clock_gptmr3) \
	X(clock_gptmr4) \
	X(clock_gptmr5) \
	X(clock_gptmr6) \
	X(clock_gptmr7) \
	X(clock_i2c0) \
	X(clock_i2c1) \
	X(clock_i2c2) \
	X(clock_i2c3) \
	X(clock_i2c4) \
	X(clock_i2c5) \
	X(clock_i2c6) \
	X(clock_i2c7) \
	X(clock_spi0) \
	X(clock_spi1) \
	X(clock_spi2) \
	X(clock_spi3) \
	X(clock_spi4) \
	X(clock_spi5) \
	X(clock_spi6) \
	X(clock_spi7) \
	X(clock_uart0) \
	X(clock_uart1) \
	X(clock_uart2) \
	X(clock_uart3) \
	X(clock_uart4) \
	X(clock_uart5) \
	X(clock_uart6) \
	X(clock_uart7) \
	X(clock_uart8) \
	X(clock_uart9) \
	X(clock_uart10) \
	X(clock_uart11) \
	X(clock_uart12) \
	X(clock_uart13) \
	X(clock_uart14) \
	X(clock_uart15) \
	X(clock_can0) \
	X(clock_can1) \
	X(clock_can2) \
	X(clock_can3) \
	X(clock_can4) \
	X(clock_can5) \
	X(clock_can6) \
	X(clock_can7) \
	X(clock_xpi0) \
	X(clock_femc) \
	X(clock_eth0) \
	X(clock_ptp0) \
	X(clock_ntmr0) \
	X(clock_ref0) \
	X(clock_ref1) \
	X(clock_tsn1) \
	X(clock_tsn2) \
	X(clock_tsn3) \
	X(clock_crc0) \
	X(clock_tsns) \
	X(clock_mbx0) \
	X(clock_mbx1) \
	X(clock_gpio) \
	X(clock_ppi0) \
	X(clock_hdma) \
	X(clock_lobs) \
	X(clock_acmp0) \
	X(clock_acmp1) \
	X(clock_acmp2) \
	X(clock_acmp3) \
	X(clock_ptpc) \
	X(clock_mot0) \
	X(clock_qei0) \
	X(clock_qei1) \
	X(clock_qei2) \
	X(clock_qei3) \
	X(clock_qeo0) \
	X(clock_qeo1) \
	X(clock_qeo2) \
	X(clock_qeo3) \
	X(clock_pwm0) \
	X(clock_pwm1) \
	X(clock_pwm2) \
	X(clock_pwm3) \
	X(clock_rdc0) \
	X(clock_rdc1) \
	X(clock_sdm0) \
	X(clock_sdm1) \
	X(clock_plb0) \
	X(clock_sei0) \
	X(clock_mtg0) \
	X(clock_mtg1) \
	X(clock_vsc0) \
	X(clock_vsc1) \
	X(clock_clc0) \
	X(clock_clc1) \
	X(clock_emds) \
	X(clock_watchdog0) \
	X(clock_watchdog1) \
	X(clock_watchdog2) \
	X(clock_watchdog3) \
	X(clock_rng) \
	X(clock_sdp) \
	X(clock_pka) \
	X(clock_kman) \
	X(clock_xdma) \
	X(clock_ffa0) \
	X(clock_rom0) \
	X(clock_ram0) \
	X(clock_ram1) \
	X(clock_usb0) \
	X(clock_esc0) \
	X(clock_lmm0) \
	X(clock_lmm1) \
	X(clock_puart) \
	X(clock_ptmr) \
	X(clock_pwdg) \
	X(clock_ana0) \
	X(clock_ana1) \
	X(clock_ana2) \
	X(clock_ana3) \
	X(clock_adc0) \
	X(clock_adc1) \
	X(clock_adc2) \
	X(clock_adc3) \
	X(clock_aud0) \
	X(clock_aud1) \
	X(clock_i2s0) \
	X(clock_i2s1) \
	X(clock_pdm) \
	X(clock_dao)

#define HPM_PLL_OUTPUT_LIST(X) \
	X(clk_pll0clk0, 0, 0) \
	X(clk_pll0clk1, 0, 1) \
	X(clk_pll1clk0, 1, 0) \
	X(clk_pll1clk1, 1, 1) \
	X(clk_pll1clk2, 1, 2) \
	X(clk_pll2clk0, 2, 0) \
	X(clk_pll2clk1, 2, 1)

#elif defined(CONFIG_SOC_HPM6P81)
#define HPM_PLL_IS_V1 0
#define HPM_PLL_COUNT 3U
#define HPM_CLOCK_LIST(X) \
	X(clock_cpu0) \
	X(clock_mchtmr0) \
	X(clock_cpu1) \
	X(clock_mchtmr1) \
	X(clock_ahb0) \
	X(clock_axif) \
	X(clock_axis) \
	X(clock_axic) \
	X(clock_gptmr0) \
	X(clock_gptmr1) \
	X(clock_gptmr2) \
	X(clock_gptmr3) \
	X(clock_i2c0) \
	X(clock_i2c1) \
	X(clock_i2c2) \
	X(clock_i2c3) \
	X(clock_spi0) \
	X(clock_spi1) \
	X(clock_spi2) \
	X(clock_spi3) \
	X(clock_uart0) \
	X(clock_uart1) \
	X(clock_uart2) \
	X(clock_uart3) \
	X(clock_uart4) \
	X(clock_uart5) \
	X(clock_uart6) \
	X(clock_uart7) \
	X(clock_can0) \
	X(clock_can1) \
	X(clock_can2) \
	X(clock_can3) \
	X(clock_xpi0) \
	X(clock_femc) \
	X(clock_eth0) \
	X(clock_ptp0) \
	X(clock_ntmr0) \
	X(clock_ref0) \
	X(clock_ref1) \
	X(clock_crc0) \
	X(clock_tsns) \
	X(clock_mbx0) \
	X(clock_mbx1) \
	X(clock_gpio) \
	X(clock_ppi0) \
	X(clock_hdma) \
	X(clock_lobs) \
	X(clock_acmp0) \
	X(clock_acmp1) \
	X(clock_acmp2) \
	X(clock_acmp3) \
	X(clock_ptpc) \
	X(clock_mot0) \
	X(clock_qei0) \
	X(clock_qei1) \
	X(clock_qeo0) \
	X(clock_qeo1) \
	X(clock_pwm0) \
	X(clock_pwm1) \
	X(clock_pwm2) \
	X(clock_pwm3) \
	X(clock_rdc0) \
	X(clock_sdm0) \
	X(clock_plb0) \
	X(clock_sei0) \
	X(clock_mtg0) \
	X(clock_vsc0) \
	X(clock_clc0) \
	X(clock_emds) \
	X(clock_dac0) \
	X(clock_dac1) \
	X(clock_watchdog0) \
	X(clock_watchdog1) \
	X(clock_watchdog2) \
	X(clock_watchdog3) \
	X(clock_rng) \
	X(clock_sdp) \
	X(clock_kman) \
	X(clock_xdma) \
	X(clock_ffa0) \
	X(clock_rom0) \
	X(clock_ram0) \
	X(clock_usb0) \
	X(clock_lmm0) \
	X(clock_lmm1) \
	X(clock_puart) \
	X(clock_ptmr) \
	X(clock_pwdg) \
	X(clock_ana0) \
	X(clock_ana1) \
	X(clock_ana2) \
	X(clock_ana3) \
	X(clock_adc0) \
	X(clock_adc1) \
	X(clock_adc2) \
	X(clock_adc3) \
	X(clock_aud0) \
	X(clock_aud1) \
	X(clock_i2s0) \
	X(clock_i2s1) \
	X(clock_pdm) \
	X(clock_dao)

#define HPM_PLL_OUTPUT_LIST(X) \
	X(clk_pll0clk0, 0, 0) \
	X(clk_pll0clk1, 0, 1) \
	X(clk_pll1clk0, 1, 0) \
	X(clk_pll1clk1, 1, 1) \
	X(clk_pll1clk2, 1, 2) \
	X(clk_pll2clk0, 2, 0) \
	X(clk_pll2clk1, 2, 1)

#else
#error "Unsupported HPMicro SoC for this sample"
#endif

#endif /* HPM_CLOCK_TREE_CLOCK_CATALOG_H_ */
