/*
 * Copyright (c) 2022 hpmicro
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 */

#include <errno.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/drivers/clock_control.h>
#include <soc.h>
#include <zephyr/drivers/pinctrl.h>
#include "hpm_clock_drv.h"
#include <drivers/clock_control/hpmicro_clock_control.h>
#include "dt-bindings/pwm/hpmicro-pwm-common.h"
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(pwm_hpmicro, CONFIG_PWM_LOG_LEVEL);

/* Support both PWM and PWMv2 */
#if CONFIG_DT_HAS_HPMICRO_HPM_PWM_ENABLED && CONFIG_DT_HAS_HPMICRO_HPM_PWMV2_ENABLED
#error "Cannot enable both PWM and PWMv2 in the same build"
#elif CONFIG_DT_HAS_HPMICRO_HPM_PWM_ENABLED
#include <hpm_pwm_drv.h>
#define DT_DRV_COMPAT hpmicro_hpm_pwm
#define HPM_PWM_BASE_TYPE PWM_Type
#elif CONFIG_DT_HAS_HPMICRO_HPM_PWMV2_ENABLED  
#include <hpm_pwmv2_drv.h>
#define DT_DRV_COMPAT hpmicro_hpm_pwmv2
#define HPM_PWM_BASE_TYPE PWMV2_Type  /* PWMv2 may use different type, adjust as needed */
#else
#error "No PWM peripheral enabled"
#endif

struct pwm_hpmicro_config {
	HPM_PWM_BASE_TYPE *base;
	uint32_t clock_name;
	uint32_t dead_zone_in_half_cycle;
	const struct pinctrl_dev_config *pincfg;
};

struct pwm_hpmicro_data {
	uint32_t period_cycles[1];
};

/* PWM version specific implementations */
#if CONFIG_DT_HAS_HPMICRO_HPM_PWM_ENABLED
static int hpmicro_pwm_v1_set_cycles(const struct device *dev, uint32_t channel,
		       uint32_t period_cycles, uint32_t pulse_cycles,
		       pwm_flags_t flags)
{
	const struct pwm_hpmicro_config *config = dev->config;
	pwm_config_t pwm_config;
	pwm_cmp_config_t cmp_config[2] = {0};
	HPM_PWM_BASE_TYPE *pwm_base = config->base;
	uint32_t rld = 0, xrld = 0;
	uint32_t rld_cmp = 0, xrld_cmp = 0;
	uint16_t i;

	pwm_get_default_pwm_config(pwm_base, &pwm_config);

	/*
	 * Trigger mode: configure CMP[channel] as a compare/trigger source.
	 * Reload is expected from init (or a prior waveform set); only the
	 * compare point is updated here. pulse_cycles is the CMP value.
	 * CMP[MAX-1] is reserved as the hardware event that loads CMP shadows.
	 */
	if ((flags & PWM_TRIG_MASK) == PWM_TRIG_ENABLE) {
		if ((channel < 8U) || (channel >= (PWM_SOC_CMP_MAX_COUNT - 1U))) {
			return -EINVAL;
		}

		cmp_config[0].enable_ex_cmp = false;
		cmp_config[0].mode = pwm_cmp_mode_output_compare;
		cmp_config[0].cmp = pulse_cycles;
		cmp_config[0].update_trigger = pwm_shadow_register_update_on_hw_event;

		cmp_config[1].mode = pwm_cmp_mode_output_compare;
		cmp_config[1].cmp = period_cycles;
		cmp_config[1].update_trigger = pwm_shadow_register_update_on_modify;
		pwm_config_cmp(pwm_base, channel, &cmp_config[0]);
		pwm_load_cmp_shadow_on_capture(pwm_base, PWM_SOC_CMP_MAX_COUNT - 1, 0);
		pwm_config_cmp(pwm_base, PWM_SOC_CMP_MAX_COUNT - 1, &cmp_config[1]);

		pwm_start_counter(pwm_base);
		pwm_issue_shadow_register_lock_event(pwm_base);
		return 0;
	}

	/* Waveform outputs use PWMCFG[0..PWM_MAX-1]; ch8+ are REF/TRIG only */
	if (channel >= PWM_SOC_PWM_MAX_COUNT) {
		return -EINVAL;
	}

	pwm_config.invert_output = ((flags & PWM_POLARITY_MASK) == PWM_POLARITY_INVERTED);

	if (period_cycles > 0xffffff) {
		for (i = 1; i <= 15; i++) {
			if ((period_cycles / (i + 1)) <= 0xffffff) {
				rld = period_cycles / (i + 1);
				xrld = i;
				/*
				 * Map rising edge at (period - pulse) onto extended
				 * counter space: t = XCNT*(RLD+1) + CNT.
				 * XRLD is 4-bit (0..15).
				 */
				{
					uint32_t rise = period_cycles - pulse_cycles;
					uint32_t inner = rld + 1U;

					xrld_cmp = rise / inner;
					rld_cmp = rise % inner;
				}
				break;
			} else if (i >= 15) {
				return -ENOTSUP;
			}
		}
	} else {
		rld = period_cycles;
		xrld = 0;
		rld_cmp = period_cycles - pulse_cycles;
		xrld_cmp = 0;
	}

	pwm_config.enable_output = true;
	pwm_config.dead_zone_in_half_cycle = config->dead_zone_in_half_cycle;
	pwm_set_reload(pwm_base, xrld, rld);
	pwm_set_start_count(pwm_base, 0, 0);

	cmp_config[0].enable_ex_cmp  = true;
	cmp_config[0].mode = pwm_cmp_mode_output_compare;
	cmp_config[0].cmp = rld + 1;
	cmp_config[0].ex_cmp = xrld;
	cmp_config[0].update_trigger = pwm_shadow_register_update_on_hw_event;

	cmp_config[1].enable_ex_cmp  = true;
	cmp_config[1].mode = pwm_cmp_mode_output_compare;
	cmp_config[1].cmp = rld;
	cmp_config[1].ex_cmp = xrld;
	cmp_config[1].update_trigger = pwm_shadow_register_update_on_modify;

	if (status_success != pwm_setup_waveform(pwm_base, channel, &pwm_config, channel, &cmp_config[0], 1)) {
		LOG_ERR("failed to setup waveform\n");
		return -ENOTSUP;
	}
	/* CMP[MAX-1]=RLD selects period-boundary shadow load (SDK pwm_output) */
	if (status_success != pwm_load_cmp_shadow_on_match(pwm_base,
				PWM_SOC_CMP_MAX_COUNT - 1, &cmp_config[1])) {
		LOG_ERR("failed to config shadow load cmp\n");
		return -ENOTSUP;
	}

	pwm_start_counter(pwm_base);
	pwm_issue_shadow_register_lock_event(pwm_base);

	pwm_shadow_register_unlock(pwm_base);
	pwm_cmp_update_cmp_value(pwm_base, channel, rld_cmp, xrld_cmp);
	pwm_issue_shadow_register_lock_event(pwm_base);
	return 0;
}
#endif /* CONFIG_DT_HAS_HPMICRO_HPM_PWM_ENABLED */

#if CONFIG_DT_HAS_HPMICRO_HPM_PWMV2_ENABLED
static int hpmicro_pwm_v2_set_cycles(const struct device *dev, uint32_t channel,
		       uint32_t period_cycles, uint32_t pulse_cycles,
		       pwm_flags_t flags)
{
	const struct pwm_hpmicro_config *config = dev->config;
	HPM_PWM_BASE_TYPE *pwm_base = config->base;
	uint32_t rld;

	if (channel > 7) {
		return -ENOTSUP;
	}
	/* Calculate counter and cmp index based on channel
	 * Channel 0,1 -> Counter 0, CMP 0-3
	 * Channel 2,3 -> Counter 1, CMP 4-7
	 * Channel 4,5 -> Counter 2, CMP 8-11
	 * Channel 6,7 -> Counter 3, CMP 12-15
	 */
	pwm_counter_t counter = (pwm_counter_t)(channel >> 1);
	uint8_t cmp_start_index = (channel >> 1) << 2;  /* counter * 4 */
	uint8_t cmp_index1 = cmp_start_index + ((channel & 0x01) << 1);     /* +0/+2 for even/odd */
	uint8_t cmp_index2 = cmp_index1 + 1;                                /* +1/+3 for even/odd */
	
	/* Shadow register allocation for PWMv2:
	 * Each counter uses 5 shadow registers:
	 * - Counter 0: shadows 0(reload), 1,2(ch0), 3,4(ch1)
	 * - Counter 1: shadows 5(reload), 6,7(ch2), 8,9(ch3)  
	 * - Counter 2: shadows 10(reload), 11,12(ch4), 13,14(ch5)
	 * - Counter 3: shadows 15(reload), 16,17(ch6), 18,19(ch7)
	 */
	uint8_t shadow_reload = counter * 5;                                 /* 0, 5, 10, 15 */
	uint8_t shadow_cmp1 = shadow_reload + 1 + ((channel & 0x01) << 1);  /* ch0:1, ch1:3 */
	uint8_t shadow_cmp2 = shadow_cmp1 + 1;                              /* ch0:2, ch1:4 */

	/* Handle polarity inversion */
	bool invert_output = ((flags & PWM_POLARITY_MASK) == PWM_POLARITY_INVERTED);

	if ((period_cycles == 0U) || (pulse_cycles > period_cycles)) {
		return -EINVAL;
	}
	/* Hardware period T = reload; SDK uses freq*ms/1000 - 1 */
	if (period_cycles > 0xffffffU) {
		return -ENOTSUP;
	}
	rld = period_cycles - 1U;

	/* Unlock shadow registers */
	pwmv2_shadow_register_unlock(pwm_base);

	/* Set reload (4th arg is high-resolution tick 0..255, not XRLD) */
	pwmv2_set_shadow_val(pwm_base, shadow_reload, rld, 0, false);

	/*
	 * Edge-aligned (UM): high while a <= CNT < b.
	 * a=0, b=pulse → high for pulse cycles; pulse=0 → always low;
	 * pulse=period → b=T+1 → always high.
	 */
	pwmv2_set_shadow_val(pwm_base, shadow_cmp1, 0, 0, false);
	pwmv2_set_shadow_val(pwm_base, shadow_cmp2, pulse_cycles, 0, false);

	/* Configure counter (first time only or when period changes) */
	pwmv2_counter_select_data_offset_from_shadow_value(pwm_base, counter, shadow_reload);
	pwmv2_counter_burst_disable(pwm_base, counter);
	pwmv2_set_reload_update_time(pwm_base, counter, pwm_reload_update_on_reload);

	/* Configure compare sources for two CMP values */
	pwmv2_select_cmp_source(pwm_base, cmp_index1, cmp_value_from_shadow_val, shadow_cmp1);
	pwmv2_select_cmp_source(pwm_base, cmp_index2, cmp_value_from_shadow_val, shadow_cmp2);
	pwmv2_cmp_update_trig_time(pwm_base, cmp_index1, pwm_shadow_register_update_on_reload);
	pwmv2_cmp_update_trig_time(pwm_base, cmp_index2, pwm_shadow_register_update_on_reload);

	/* Set output polarity */
	if (invert_output) {
		pwmv2_enable_output_invert(pwm_base, (pwm_channel_t)channel);
	} else {
		pwmv2_disable_output_invert(pwm_base, (pwm_channel_t)channel);
	}

	/* Set dead zone if configured */
	if (config->dead_zone_in_half_cycle > 0) {
		pwmv2_set_dead_area(pwm_base, (pwm_channel_t)channel, config->dead_zone_in_half_cycle);
	}

	/* Lock shadow registers to apply changes */
	pwmv2_shadow_register_lock(pwm_base);

	/*
	 * TRIG_SEL4 lives on even channels only. Zephyr uses 2-cmp duty PWM
	 * (SDK pwm_output / rgb_led / sdm): always clear it on the even floor
	 * so odd channels use CMP N*2+2..N*2+3. Four-cmp is unused here.
	 */
	pwmv2_disable_four_cmp(pwm_base, (pwm_channel_t)(channel & ~1U));
	/* Enable channel output */
	pwmv2_channel_enable_output(pwm_base, (pwm_channel_t)channel);
	
	/* Enable and start counter */
	pwmv2_enable_counter(pwm_base, counter);
	pwmv2_start_pwm_output(pwm_base, counter);

	return 0;
}
#endif /* CONFIG_DT_HAS_HPMICRO_HPM_PWMV2_ENABLED */

static int hpmicro_pwm_set_cycles(const struct device *dev, uint32_t channel,
		       uint32_t period_cycles, uint32_t pulse_cycles,
		       pwm_flags_t flags)
{
#if CONFIG_DT_HAS_HPMICRO_HPM_PWM_ENABLED
	return hpmicro_pwm_v1_set_cycles(dev, channel, period_cycles, pulse_cycles, flags);
#elif CONFIG_DT_HAS_HPMICRO_HPM_PWMV2_ENABLED  
	return hpmicro_pwm_v2_set_cycles(dev, channel, period_cycles, pulse_cycles, flags);
#endif
}

#if CONFIG_DT_HAS_HPMICRO_HPM_PWM_ENABLED
static int hpmicro_pwm_v1_get_cycles_per_sec(const struct device *dev,
			       uint32_t channel, uint64_t *cycles)
{
	const struct pwm_hpmicro_config *config = dev->config;
	uint32_t freqc;

	freqc = clock_get_frequency(config->clock_name);
	*cycles = freqc;

	return 0;
}
#endif /* CONFIG_DT_HAS_HPMICRO_HPM_PWM_ENABLED */

#if CONFIG_DT_HAS_HPMICRO_HPM_PWMV2_ENABLED
static int hpmicro_pwm_v2_get_cycles_per_sec(const struct device *dev,
			       uint32_t channel, uint64_t *cycles)
{
	const struct pwm_hpmicro_config *config = dev->config;
	uint32_t freqc;

	freqc = clock_get_frequency(config->clock_name);
	*cycles = freqc;

	return 0;
}
#endif /* CONFIG_DT_HAS_HPMICRO_HPM_PWMV2_ENABLED */

static int hpmicro_pwm_get_cycles_per_sec(const struct device *dev,
			       uint32_t channel, uint64_t *cycles)
{
#if CONFIG_DT_HAS_HPMICRO_HPM_PWM_ENABLED
	return hpmicro_pwm_v1_get_cycles_per_sec(dev, channel, cycles);
#elif CONFIG_DT_HAS_HPMICRO_HPM_PWMV2_ENABLED  
	return hpmicro_pwm_v2_get_cycles_per_sec(dev, channel, cycles);
#endif
}

#if CONFIG_DT_HAS_HPMICRO_HPM_PWM_ENABLED
static int pwm_hpmicro_v1_init(const struct device *dev)
{
	const struct pwm_hpmicro_config *config = dev->config;
	int err;

	err = pinctrl_apply_state(config->pincfg, PINCTRL_STATE_DEFAULT);
	if (err < 0) {
		return err;
	}

	clock_enable_name((clock_name_t)config->clock_name);

	return 0;
}
#endif /* CONFIG_DT_HAS_HPMICRO_HPM_PWM_ENABLED */

#if CONFIG_DT_HAS_HPMICRO_HPM_PWMV2_ENABLED
static int pwm_hpmicro_v2_init(const struct device *dev)
{
	const struct pwm_hpmicro_config *config = dev->config;
	HPM_PWM_BASE_TYPE *pwm_base = config->base;
	int err;

	/* Apply pin configuration */
	err = pinctrl_apply_state(config->pincfg, PINCTRL_STATE_DEFAULT);
	if (err < 0) {
		return err;
	}

	clock_enable_name((clock_name_t)config->clock_name);

	/* Deinitialize PWMv2 to reset to known state */
	pwmv2_deinit(pwm_base);

	return 0;
}
#endif /* CONFIG_DT_HAS_HPMICRO_HPM_PWMV2_ENABLED */

static int pwm_hpmicro_init(const struct device *dev)
{
#if CONFIG_DT_HAS_HPMICRO_HPM_PWM_ENABLED
	return pwm_hpmicro_v1_init(dev);
#elif CONFIG_DT_HAS_HPMICRO_HPM_PWMV2_ENABLED  
	return pwm_hpmicro_v2_init(dev);
#endif
}

static const struct pwm_driver_api pwm_hpmicro_driver_api = {
	.set_cycles = hpmicro_pwm_set_cycles,
	.get_cycles_per_sec = hpmicro_pwm_get_cycles_per_sec,
};

#define PWM_DEVICE_INIT_HPMICRO(n)			  \
	static struct pwm_hpmicro_data pwm_hpmicro_data_##n;		  \
	PINCTRL_DT_INST_DEFINE(n);					  \
									  \
	static const struct pwm_hpmicro_config pwm_hpmicro_config_##n = {     \
		.base = (HPM_PWM_BASE_TYPE *)DT_INST_REG_ADDR(n),	  \
		.clock_name = DT_INST_CLOCKS_CELL(n, name),		\
		.pincfg = PINCTRL_DT_INST_DEV_CONFIG_GET(n),		  \
		.dead_zone_in_half_cycle = DT_INST_PROP_OR(n, dead_zone_in_half_cycle, 0),	\
	};								  \
									  \
	DEVICE_DT_INST_DEFINE(n,					  \
			    pwm_hpmicro_init,				  \
			    NULL,					  \
			    &pwm_hpmicro_data_##n,			  \
			    &pwm_hpmicro_config_##n,			  \
			    POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEVICE,\
			    &pwm_hpmicro_driver_api);

DT_INST_FOREACH_STATUS_OKAY(PWM_DEVICE_INIT_HPMICRO)
