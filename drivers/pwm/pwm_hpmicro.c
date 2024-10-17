/*
 * Copyright (c) 2022 hpmicro
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 */

#define DT_DRV_COMPAT hpmicro_hpm_pwm

#include <errno.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/drivers/clock_control.h>
#include <soc.h>
#include <hpm_pwm_drv.h>
#include <zephyr/drivers/pinctrl.h>
#include "hpm_clock_drv.h"
#include "dt-bindings/pwm/hpmicro-pwm-common.h"
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(pwm_hpmicro, CONFIG_PWM_LOG_LEVEL);

struct pwm_hpmicro_config {
	PWM_Type *base;
	uint32_t clock_name;
	uint32_t period;
	uint32_t dead_zone_in_half_cycle;
	const struct pinctrl_dev_config *pincfg;
};

struct pwm_hpmicro_data {
	uint32_t period_cycles[1];
};

static int hpmicro_pwm_set_cycles(const struct device *dev, uint32_t channel,
			       uint32_t period_cycles, uint32_t pulse_cycles,
			       pwm_flags_t flags)
{
	const struct pwm_hpmicro_config *config = dev->config;
	pwm_config_t pwm_config;
	pwm_cmp_config_t cmp_config[2] = {0};
	PWM_Type *pwm_base = config->base;
	uint32_t rld = 0, xrld = 0, prld = 0;
	uint32_t rld_cmp = 0, xrld_cmp = 0;
	uint16_t i, j;

	pwm_get_default_pwm_config(pwm_base, &pwm_config);
	if (flags == PWM_POLARITY_INVERTED) {
		pwm_config.invert_output = true;
	} else if (flags == PWM_POLARITY_NORMAL) {
		pwm_config.invert_output = false;
	} else if (flags == PWM_TRIG_ENABLE) {
		cmp_config[0].enable_ex_cmp  = false;
		cmp_config[0].mode = pwm_cmp_mode_output_compare;
		cmp_config[0].cmp = period_cycles + 1;
		cmp_config[0].update_trigger = pwm_shadow_register_update_on_hw_event;

		cmp_config[1].mode = pwm_cmp_mode_output_compare;
		cmp_config[1].cmp = period_cycles;
		cmp_config[1].update_trigger = pwm_shadow_register_update_on_modify;
		pwm_config_cmp(pwm_base, channel, &cmp_config[0]);
		pwm_load_cmp_shadow_on_capture(pwm_base, 23, 0);
		pwm_config_cmp(pwm_base, 23, &cmp_config[1]);
	
		pwm_start_counter(pwm_base);
		pwm_issue_shadow_register_lock_event(pwm_base);
		return 0;
	} else {
		return -ENOTSUP;
	}

	if (period_cycles >  0xffffff) {
		for (i = 1; i <= 16; i++) {
			if ((period_cycles / (i + 1)) <= 0xffffff) {
				rld = period_cycles / (i + 1);
				xrld = i;
				prld = (xrld << 24) | rld;
				for (j = 0; j <= 16; j++) {
					if (((period_cycles - pulse_cycles) / (j + 1)) <= 0xffffff) {
						rld_cmp = (period_cycles - pulse_cycles) / (j + 1);
						xrld_cmp = j;
						break;
					} else if (j >= 16) {
						return -ENOTSUP;
					}
				}
				break;
			} else if (i >= 16) {
				return -ENOTSUP;
			}
		}
	} else {
		rld = period_cycles;
		xrld = 0;
		prld = period_cycles;
		rld_cmp = period_cycles - pulse_cycles;
		xrld_cmp = 0;
	}

	if (prld != ((PWM_RLD_XRLD_GET(pwm_base->RLD) << 24) | PWM_RLD_RLD_GET(pwm_base->RLD))) {

		pwm_config.enable_output = true;
		pwm_config.dead_zone_in_half_cycle = config->dead_zone_in_half_cycle;
		pwm_set_reload(pwm_base, xrld, rld);
		pwm_set_start_count(pwm_base, 0, 0);

		cmp_config[0].enable_ex_cmp  = true;
		cmp_config[0].mode = pwm_cmp_mode_output_compare;
		cmp_config[0].cmp = rld + 1;
		cmp_config[0].ex_cmp = xrld;
		cmp_config[0].update_trigger = pwm_shadow_register_update_on_modify;

		cmp_config[1].enable_ex_cmp  = true;
		cmp_config[1].mode = pwm_cmp_mode_output_compare;
		cmp_config[1].cmp = rld;
		cmp_config[1].ex_cmp = xrld;
		cmp_config[1].update_trigger = pwm_shadow_register_update_on_modify;

		if (status_success != pwm_setup_waveform(pwm_base, channel, &pwm_config, channel, &cmp_config[0], 1)) {
			LOG_ERR("failed to setup waveform\n");
			return -ENOTSUP;
		}
		pwm_load_cmp_shadow_on_capture(pwm_base, 23, 0);
		pwm_config_cmp(pwm_base, 23, &cmp_config[1]);
	
		pwm_start_counter(pwm_base);
		pwm_issue_shadow_register_lock_event(pwm_base);
	}

	pwm_shadow_register_unlock(pwm_base);
    pwm_cmp_update_cmp_value(pwm_base, channel, rld_cmp, xrld_cmp);
	pwm_issue_shadow_register_lock_event(pwm_base);
	return 0;
}

static int hpmicro_pwm_get_cycles_per_sec(const struct device *dev,
				       uint32_t channel, uint64_t *cycles)
{
	const struct pwm_hpmicro_config *config = dev->config;
	uint32_t freqc;

	freqc = clock_get_frequency(config->clock_name);
	*cycles = freqc;

	return 0;
}

static int pwm_hpmicro_init(const struct device *dev)
{
	const struct pwm_hpmicro_config *config = dev->config;
	pwm_config_t pwm_config;
	uint32_t freqc;
	PWM_Type *pwm_base = config->base;
	int err;

	err = pinctrl_apply_state(config->pincfg, PINCTRL_STATE_DEFAULT);
	if (err < 0) {
		return err;
	}

	pwm_get_default_pwm_config(pwm_base, &pwm_config);
	pwm_config.enable_output = true;
    pwm_config.dead_zone_in_half_cycle = config->dead_zone_in_half_cycle;
    pwm_config.invert_output = false;
	freqc = clock_get_frequency(config->clock_name);
	pwm_set_reload(pwm_base, 0, freqc / config->period);
    pwm_set_start_count(pwm_base, 0, 0);

	return 0;
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
		.base = (PWM_Type *)DT_INST_REG_ADDR(n),	  \
		.clock_name = DT_INST_CLOCKS_CELL(n, name),		\
		.pincfg = PINCTRL_DT_INST_DEV_CONFIG_GET(n),		  \
		.period = DT_INST_PROP(n, period_init),			\
		.dead_zone_in_half_cycle = DT_INST_PROP(n, dead_zone_in_half_cycle),	\
	};								  \
									  \
	DEVICE_DT_INST_DEFINE(n,					  \
			    &pwm_hpmicro_init,				  \
			    NULL,					  \
			    &pwm_hpmicro_data_##n,			  \
			    &pwm_hpmicro_config_##n,			  \
			    POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEVICE,\
			    &pwm_hpmicro_driver_api);

DT_INST_FOREACH_STATUS_OKAY(PWM_DEVICE_INIT_HPMICRO)
