/*
 * Copyright (c) 2026 Freqchip
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * Freqchip SoC specific helpers for pinctrl driver
 */

#ifndef ZEPHYR_SOC_FREQCHIP_FREQ_COMMON_PINCTRL_SOC_H_
#define ZEPHYR_SOC_FREQCHIP_FREQ_COMMON_PINCTRL_SOC_H_

#include <stdint.h>
#include <zephyr/devicetree.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * pincfg field bit layout:
 *   bit 0: bias-pull-up
 *   bit 1: bias-pull-down
 *   bit 2: drive-open-drain  (bit cleared = push-pull)
 */
#define FREQ_PINCFG_PULL_UP      (1U << 0)
#define FREQ_PINCFG_PULL_DOWN    (1U << 1)
#define FREQ_PINCFG_OPEN_DRAIN   (1U << 2)

/** Type for Freqchip pin (pinmux + pincfg). */
struct pinctrl_soc_pin {
	/** Pinmux value (port / pin / function packed per FR_PINMUX). */
	uint32_t pinmux;
	/** Pincfg flags (bias / drive). */
	uint32_t pincfg;
};

typedef struct pinctrl_soc_pin pinctrl_soc_pin_t;

/**
 * @brief Build pincfg flags from a pinctrl child node's devicetree properties.
 */
#define Z_PINCTRL_FREQ_PINCFG_INIT(child_id)                          \
	(((FREQ_PINCFG_PULL_UP * DT_PROP(child_id, bias_pull_up)) |   \
	  (FREQ_PINCFG_PULL_DOWN * DT_PROP(child_id, bias_pull_down)) | \
	  (FREQ_PINCFG_OPEN_DRAIN * DT_PROP(child_id, drive_open_drain))))

/**
 * @brief Initialize a single pin from pinctrl-<k>.
 *
 * @param node_id  device node (e.g. usart2)
 * @param prop     pinctrl_0 (the current state's pinctrl property token)
 * @param idx      index into the phandle array
 */
#define Z_PINCTRL_STATE_PIN_INIT(node_id, prop, idx)                   \
	{                                                              \
		.pinmux = DT_PROP(DT_PINCTRL_BY_IDX(node_id, 0, idx),   \
				  pinmux),                              \
		.pincfg = Z_PINCTRL_FREQ_PINCFG_INIT(                   \
			DT_PINCTRL_BY_IDX(node_id, 0, idx)),           \
	},

/**
 * @brief Initialize state pins for the current pinctrl-<k> property.
 *
 * @param node_id  device node
 * @param prop     pinctrl-0 / pinctrl-1 / ... property token
 */
#define Z_PINCTRL_STATE_PINS_INIT(node_id, prop)                       \
	{DT_FOREACH_PROP_ELEM(node_id, pinctrl_0, Z_PINCTRL_STATE_PIN_INIT)}

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_SOC_FREQCHIP_FREQ_COMMON_PINCTRL_SOC_H_ */
