/*
 * Copyright (c) 2022 HPMicro
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 */
#ifndef ZEPHYR_INCLUDE_DT_BINDINGS_PINCTRL_HPMICRO_PWM_COMMON_H_
#define ZEPHYR_INCLUDE_DT_BINDINGS_PINCTRL_HPMICRO_PWM_COMMON_H_

/*
 * SoC-specific pwm_flags_t bits (Zephyr reserves the upper 8 bits).
 * When set, pwm_set_cycles() configures CMP[channel] as a trigger compare
 * source (CHxREF); pulse_cycles is the compare value (trigger point).
 */
#define PWM_TRIG_SHIFT	8U
#define PWM_TRIG_MASK	(1U << PWM_TRIG_SHIFT)
#define PWM_TRIG_ENABLE	(1U << PWM_TRIG_SHIFT)

#endif /* ZEPHYR_INCLUDE_DT_BINDINGS_PWM_PWM_H_ */
