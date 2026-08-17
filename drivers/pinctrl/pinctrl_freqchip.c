/*
 * Copyright (c) 2026 Freqchip
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/drivers/pinctrl.h>
#include <driver_gpio.h>

/*
 * FR_PINMUX bit layout (from freq.h):
 *   bits [15:12] = port (0=A, 1=B, 2=C, 3=D)
 *   bits [11:4]  = pin  (0-15)
 *   bits [3:0]   = alternate function (0-15)
 */
#define FR_PORT_GET(pinmux)  (((pinmux) >> 12) & 0x3)
#define FR_PIN_GET(pinmux)   (((pinmux) >> 4) & 0xff)
#define FR_FUNC_GET(pinmux)  ((pinmux) & 0xf)

static GPIO_TypeDef *get_gpio_port(uint8_t port)
{
	switch (port) {
	case 0:  return GPIOA;
	case 1:  return GPIOB;
	case 2:  return GPIOC;
	default: return NULL;
	}
}

/*
 * Map pincfg flags to HAL pull enum.
 * Priority: pull-up > pull-down > no-pull.
 */
static enum_Pull_t pincfg_to_pull(uint32_t pincfg)
{
	if (pincfg & FREQ_PINCFG_PULL_UP) {
		return GPIO_PULLUP;
	}
	if (pincfg & FREQ_PINCFG_PULL_DOWN) {
		return GPIO_PULLDOWN;
	}
	return GPIO_NOPULL;
}

int pinctrl_configure_pins(const pinctrl_soc_pin_t *pins, uint8_t pin_cnt,uintptr_t reg)
{
	ARG_UNUSED(reg);

	for (uint8_t i = 0U; i < pin_cnt; i++) {
		uint32_t pinmux = pins[i].pinmux;
		uint32_t pincfg = pins[i].pincfg;

		uint8_t port = FR_PORT_GET(pinmux);
		uint8_t pin  = FR_PIN_GET(pinmux);
		uint8_t func = FR_FUNC_GET(pinmux);

		GPIO_TypeDef *GPIOx = get_gpio_port(port);
		if (GPIOx == NULL) {
			continue;
		}

		GPIO_InitTypeDef GPIO_Init = {
			.Pin       = (uint16_t)(1U << pin),
			.Mode      = GPIO_MODE_AF_PP,
			.Pull      = pincfg_to_pull(pincfg),
			.Alternate = func,
		};
		gpio_init(GPIOx, &GPIO_Init);
	}

	return 0;
}
