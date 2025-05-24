/*
 * Copyright (c) 2022-2025 HPMicro
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 */

#ifndef ZEPHYR_INCLUDE_DT_BINDINGS_PINCTRL_HPMICRO_PINCTRL_COMMON_H_
#define ZEPHYR_INCLUDE_DT_BINDINGS_PINCTRL_HPMICRO_PINCTRL_COMMON_H_

/**
 * @brief Pin numbers of gpio
 *
 */
#define HPMICRO_PIN_NUM_SHIFT      0U
#define HPMICRO_PIN_NUM_MASK       0x3FFU

/**
 * @brief io peripheral function
 *
 */
#define HPMICRO_PIN_ANALOG_MASK      1U
#define HPMICRO_PIN_ANALOG_SHIFT     10U
#define HPMICRO_PIN_ALT_MASK         0x1FU
#define HPMICRO_PIN_ALT_SHIFT        11U
#define HPMICRO_PIN_IOC_MASK         0x03U
#define HPMICRO_PIN_IOC_SHIFT        30U

/**
 * @brief numerical ioc_pad for IO ports
 */

#define	HPMICRO_PORTA 0 	/* IO port A */
#define	HPMICRO_PORTB 32	/* .. */
#define	HPMICRO_PORTC 64
#define	HPMICRO_PORTD 96
#define	HPMICRO_PORTE 128
#define	HPMICRO_PORTF 160
#define	HPMICRO_PORTX 416
#define	HPMICRO_PORTY 448
#define	HPMICRO_PORTZ 480	/* IO port Z */

/**
 * @brief ioc controller mode
 *
 */
#define IOC_TYPE_IOC 0
#define IOC_TYPE_BIOC 1
#define IOC_TYPE_PIOC 2

/**
 * @brief helper macro to encode an IO port pin in a numerical format
 */

#define HPMICRO_PIN(_port, _pin) \
	(_port + _pin)

/**
 * @brief macro generation codes to select pin functions
 * pin: HPMICRO_PIN(port, pin)
 * is_analog: 1: enable analog, 0: disable analog
 * alt: 0 - 31 Function Selection alt0-alt31
 * ioc: IOC_TYPE_IOC IOC_TYPE_BIOC IOC_TYPE_PIOC
 */
#define HPMICRO_PINMUX(pin, ioc, is_analog, alt)						\
		(((pin & HPMICRO_PIN_NUM_MASK) << HPMICRO_PIN_NUM_SHIFT) |		\
		((is_analog & HPMICRO_PIN_ANALOG_MASK) << HPMICRO_PIN_ANALOG_SHIFT) |	\
		((alt & HPMICRO_PIN_ALT_MASK) << HPMICRO_PIN_ALT_SHIFT) | \
		((ioc & HPMICRO_PIN_IOC_MASK) << HPMICRO_PIN_IOC_SHIFT))

/**
 * @brief Location of configuration items in the code
 *
 */
#define HPMICRO_OPEN_DRAIN			1U
#define HPMICRO_OPEN_DRAIN_SHIFT	0U

#define HPMICRO_NO_PULL				1U
#define HPMICRO_NO_PULL_SHIFT		1U

#define HPMICRO_PULL_DOWN			1U
#define HPMICRO_PULL_DOWN_SHIFT		2U

#define HPMICRO_PULL_UP				1U
#define HPMICRO_PULL_UP_SHIFT		3U

#define HPMICRO_FORCE_INPUT				1U
#define HPMICRO_FORCE_INPUT_SHIFT		4U

#define HPMICRO_DRIVER_STRENGTH				7U
#define HPMICRO_DRIVER_STRENGTH_SHIFT		5U

#define HPMICRO_SLEW_PD				3U
#define HPMICRO_SLEW_PD_SHIFT		8U

#define HPMICRO_PULL_STRENGTH		3U
#define HPMICRO_PULL_STRENGTH_SHIFT	10U

#define HPMICRO_SCHMITT_ENABLE				1U
#define HPMICRO_SCHMITT_ENABLE_SHIFT		12U

#define HPMICRO_POWER			1U
#define HPMICRO_POWER_SHIFT		13U

#define HPMICRO_SLEW_RATE			1U
#define HPMICRO_SLEW_RATE_SHIFT		14U

#define HPMICRO_KEEPER_DISABLE  1U
#define HPMICRO_KEEPER_DISABLE_SHIFT   15U

#endif	/* ZEPHYR_INCLUDE_DT_BINDINGS_PINCTRL_HPMICRO_PINCTRL_COMMON_H_ */
