/*
 * Copyright (c) 2026 HPMicro
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdint.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>

#include <hpm_clock_drv.h>
#include <hpm_soc.h>

#include "clock_catalog.h"

#if HPM_PLL_IS_V1
#include <hpm_pllctl_drv.h>
#else
#include <hpm_pllctlv2_drv.h>
#endif

struct clock_desc {
	const char *name;
	clock_name_t clock;
};

struct pll_output_desc {
	const char *name;
	clock_name_t clock;
	uint8_t pll;
	uint8_t output;
};

#define CLOCK_DESC(name) { #name, name },
static const struct clock_desc clocks[] = {
	HPM_CLOCK_LIST(CLOCK_DESC)
};

#define PLL_OUTPUT_DESC(name, pll_index, output_index) \
	{ #name, name, pll_index, output_index },
static const struct pll_output_desc pll_outputs[] = {
	HPM_PLL_OUTPUT_LIST(PLL_OUTPUT_DESC)
};

static uint32_t get_pll_frequency(uint8_t pll)
{
#if HPM_PLL_IS_V1
	return pllctl_get_pll_freq_in_hz(HPM_PLLCTL, pll);
#else
	return pllctlv2_get_pll_freq_in_hz(HPM_PLLCTLV2, pll);
#endif
}

static void print_pll_frequencies(void)
{
	printk("\nPLL runtime frequencies:\n");
	for (uint8_t pll = 0U; pll < HPM_PLL_COUNT; pll++) {
		printk("  PLL%u: %u Hz\n", pll, get_pll_frequency(pll));
		for (size_t i = 0U; i < ARRAY_SIZE(pll_outputs); i++) {
			if (pll_outputs[i].pll == pll) {
				printk("    CLK%u (%s): %u Hz\n",
				       pll_outputs[i].output, pll_outputs[i].name,
				       clock_get_frequency(pll_outputs[i].clock));
			}
		}
	}
}

static void print_cpu_frequency(void)
{
	printk("\nCPU runtime frequencies:\n");
	printk("  clock_cpu0           %u Hz\n",
	       clock_get_frequency(clock_cpu0));
}

static void print_enabled_clocks(void)
{
	size_t enabled_count = 0U;

	printk("\nEnabled clock nodes (group 0):\n");
	for (size_t i = 0U; i < ARRAY_SIZE(clocks); i++) {
		uint32_t resource = GET_CLK_RESOURCE_FROM_NAME(clocks[i].clock);

		/* Shared and invalid resources cannot be queried as group resources. */
		if ((resource < sysctl_resource_linkable_start) ||
		    (resource >= sysctl_resource_linkable_end)) {
			continue;
		}

		if (clock_check_in_group(clocks[i].clock, 0U)) {
			printk("  %-20s %u Hz\n", clocks[i].name,
			       clock_get_frequency(clocks[i].clock));
			enabled_count++;
		}
	}

	printk("Enabled clock nodes: %u\n", (uint32_t)enabled_count);
}

int main(void)
{
	printk("HPMicro runtime clock tree report\n");
	printk("Board: %s\n", CONFIG_BOARD_TARGET);
	printk("SoC:   %s\n", CONFIG_SOC);
	printk("All frequencies below are read from runtime driver APIs.\n");

	print_pll_frequencies();
	print_cpu_frequency();
	print_enabled_clocks();

	return 0;
}
