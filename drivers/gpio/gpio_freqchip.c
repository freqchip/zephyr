/*
 * Copyright (c) 2026 Freqchip
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT freqchip_freqchip_gpio

#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/gpio/gpio_utils.h>
#include <zephyr/irq.h>

#include <driver_gpio.h>

struct gpio_freqchip_config {
	struct gpio_driver_config common;
	uint32_t *base;
};

struct gpio_freqchip_data {
	struct gpio_driver_data common;
	sys_slist_t callbacks;
	GPIO_TypeDef *GPIOx;
};

static const struct device *gpioa_isr_dev;
static const struct device *gpiob_isr_dev;
static const struct device *gpioc_isr_dev;

static void gpioa_freqchip_isr(const void *arg)
{
	const struct device *dev = gpioa_isr_dev;
	struct gpio_freqchip_data *data = dev->data;
	uint32_t int_status;

	while ((int_status = data->GPIOx->GPIO_INTS) != 0) {
		data->GPIOx->GPIO_INTS = int_status;
		gpio_fire_callbacks(&data->callbacks, dev, int_status);
	}
	(void)arg;
}
static void gpiob_freqchip_isr(const void *arg)
{
	const struct device *dev = gpiob_isr_dev;
	struct gpio_freqchip_data *data = dev->data;
	uint32_t int_status;

	while ((int_status = data->GPIOx->GPIO_INTS) != 0) {
		data->GPIOx->GPIO_INTS = int_status;
		gpio_fire_callbacks(&data->callbacks, dev, int_status);
	}
	(void)arg;
}
static void gpioc_freqchip_isr(const void *arg)
{
	const struct device *dev = gpioc_isr_dev;
	struct gpio_freqchip_data *data = dev->data;
	uint32_t int_status;

	while ((int_status = data->GPIOx->GPIO_INTS) != 0) {
		data->GPIOx->GPIO_INTS = int_status;
		gpio_fire_callbacks(&data->callbacks, dev, int_status);
	}
	(void)arg;
}

static int gpio_freqchip_configure(const struct device *port, gpio_pin_t pin, gpio_flags_t flags)
{
	GPIO_TypeDef *GPIOx = ((const struct gpio_freqchip_data *)port->data)->GPIOx;
#ifdef CONFIG_SOC_SERIES_FR802X
	GPIO_InitTypeDef GPIO_Handle;
	if (flags & GPIO_OUTPUT)
	{
		GPIO_Handle.Pin  = BIT(pin);
		GPIO_Handle.Mode = GPIO_MODE_OUTPUT_PP;
		GPIO_Handle.Pull = GPIO_NOPULL;
		gpio_init(GPIOx, &GPIO_Handle);

		if (flags & GPIO_OUTPUT_INIT_HIGH){
			gpio_write_pin(GPIOx, BIT(pin), GPIO_PIN_SET);
		}
		else{
			gpio_write_pin(GPIOx, BIT(pin), GPIO_PIN_CLEAR);
		}
	}
	else if (flags & GPIO_INPUT)
	{
		GPIO_Handle.Pin  = BIT(pin);

		if (flags & GPIO_PULL_UP) {
			GPIO_Handle.Pull = GPIO_PULLUP;
		} else if (flags & GPIO_PULL_DOWN) {
			GPIO_Handle.Pull = GPIO_PULLDOWN;
		} else {
			GPIO_Handle.Pull = GPIO_NOPULL;
		}

		if (flags & GPIO_INT_ENABLE)
		{
			if (flags & GPIO_INT_EDGE)
			{//沿触发
				if((flags & (GPIO_INT_LOW_0|GPIO_INT_HIGH_1)) == (GPIO_INT_LOW_0|GPIO_INT_HIGH_1))
				{
					GPIO_Handle.Mode = GPIO_MODE_GPIO_IT_RISING_FALLING;
					gpio_init(GPIOx, &GPIO_Handle);
				}
				else if (flags & GPIO_INT_LOW_0)
				{
					GPIO_Handle.Mode = GPIO_MODE_GPIO_IT_FALLING;
					gpio_init(GPIOx, &GPIO_Handle);
				}
				else if (flags & GPIO_INT_HIGH_1)
				{
					GPIO_Handle.Mode = GPIO_MODE_GPIO_IT_RISING;
					gpio_init(GPIOx, &GPIO_Handle);
				}
			}
			else
			{//电平触发
				if (flags & GPIO_INT_LOW_0)
				{
					GPIO_Handle.Mode = GPIO_MODE_GPIO_IT_LOW_LEVEL;
					gpio_init(GPIOx, &GPIO_Handle);
				}
				else if (flags & GPIO_INT_HIGH_1)
				{
					GPIO_Handle.Mode = GPIO_MODE_GPIO_IT_HIGH_LEVEL;
					gpio_init(GPIOx, &GPIO_Handle);
				}
			}

			gpio_int_enable(GPIOx,GPIO_Handle.Pin);
            gpio_clear_int_Status(GPIOx,GPIO_Handle.Pin);
		}
		else
		{
			GPIO_Handle.Mode = GPIO_MODE_INPUT;
			gpio_init(GPIOx, &GPIO_Handle);
		}
	}
#endif
	return 0;
}

static int gpio_freqchip_port_get_raw(const struct device *port, uint32_t *value)
{
	GPIO_TypeDef *GPIOx = ((const struct gpio_freqchip_data *)port->data)->GPIOx;
#ifdef CONFIG_SOC_SERIES_FR802X
	*value = gpio_read_group(GPIOx);
#endif
	return 0;
}

static int gpio_freqchip_port_set_masked_raw(const struct device *port, gpio_port_pins_t mask, gpio_port_value_t value)
{
	GPIO_TypeDef *GPIOx = ((const struct gpio_freqchip_data *)port->data)->GPIOx;
	uint32_t set_value = (value & mask);
#ifdef CONFIG_SOC_SERIES_FR802X
	gpio_write_group(GPIOx, set_value);
#endif
	return 0;
}

static int gpio_freqchip_port_set_bits_raw(const struct device *port, gpio_port_pins_t pins)
{
	GPIO_TypeDef *GPIOx = ((const struct gpio_freqchip_data *)port->data)->GPIOx;
#ifdef CONFIG_SOC_SERIES_FR802X
	gpio_write_pin(GPIOx, pins, GPIO_PIN_SET);
#endif
	return 0;
}

static int gpio_freqchip_port_clear_bits_raw(const struct device *port, gpio_port_pins_t pins)
{
	GPIO_TypeDef *GPIOx = ((const struct gpio_freqchip_data *)port->data)->GPIOx;
#ifdef CONFIG_SOC_SERIES_FR802X
	gpio_write_pin(GPIOx, pins, GPIO_PIN_CLEAR);
#endif
	return 0;
}

static int gpio_freqchip_port_toggle_bits(const struct device *port, gpio_port_pins_t pins)
{
	GPIO_TypeDef *GPIOx = ((const struct gpio_freqchip_data *)port->data)->GPIOx;
#ifdef CONFIG_SOC_SERIES_FR802X
	gpio_toggle_pin(GPIOx, pins);
#endif
	
	return 0;
}

static int gpio_freqchip_pin_interrupt_configure(const struct device *port, gpio_pin_t pin, enum gpio_int_mode mode, enum gpio_int_trig trig)
{
	GPIO_TypeDef *GPIOx = ((const struct gpio_freqchip_data *)port->data)->GPIOx;
#ifdef CONFIG_SOC_SERIES_FR802X
	if (mode == GPIO_INT_MODE_DISABLED){
		gpio_int_disable(GPIOx, BIT(pin));
	}
	else{
		uint32_t int_mode = 0;
		if (mode == GPIO_INT_MODE_LEVEL){
			if (trig == GPIO_INT_TRIG_LOW){
				int_mode = GPIO_MODE_GPIO_IT_LOW_LEVEL;
			}
			else{
				int_mode = GPIO_MODE_GPIO_IT_HIGH_LEVEL;
			}
		}
		else if (mode == GPIO_INT_MODE_EDGE){
			if (trig == GPIO_INT_TRIG_LOW){
				int_mode = GPIO_MODE_GPIO_IT_FALLING;
			}
			else if (trig == GPIO_INT_TRIG_HIGH){
				int_mode = GPIO_MODE_GPIO_IT_RISING;
			}
			else if (trig == GPIO_INT_TRIG_BOTH){
				int_mode = GPIO_MODE_GPIO_IT_RISING_FALLING;
			}
			else if (trig == GPIO_INT_TRIG_WAKE){
				;
			}
			else{
				return -1;
			}
		}
		
		/* Port 0 ~ 7 */
		if (pin < 8) {
			GPIOx->GPIO_INTTL = (GPIOx->GPIO_INTTL & ~(0xF << (pin * 4))) | ((int_mode & 0xF) << (pin * 4));
		}
		/* Port 8 ~ 15 */
		else {
			GPIOx->GPIO_INTTH = (GPIOx->GPIO_INTTH & ~(0xF << ((pin - 8) * 4))) | ((int_mode & 0xF) << ((pin - 8) * 4));
		}
		gpio_int_enable(GPIOx, BIT(pin));
	}

#endif
	return 0;
}

static int gpio_freqchip_manage_callback(const struct device *dev, struct gpio_callback *callback, bool set)
{
	struct gpio_freqchip_data *data = dev->data;

	return gpio_manage_callback(&data->callbacks, callback, set);
}

static int gpio_freqchip_init(const struct device *dev)
{
	const struct gpio_freqchip_config *cfg = dev->config;
	struct gpio_freqchip_data *data = dev->data;

#ifdef CONFIG_SOC_SERIES_FR802X
	if (cfg->base == (uint32_t *)DT_REG_ADDR(DT_NODELABEL(gpioa))) {
		data->GPIOx = GPIOA;
		__SYSTEM_GPIOA_CLK_ENABLE();
		gpioa_isr_dev = dev;
	} else if (cfg->base == (uint32_t *)DT_REG_ADDR(DT_NODELABEL(gpiob))) {
		data->GPIOx = GPIOB;
		__SYSTEM_GPIOB_CLK_ENABLE();
		gpiob_isr_dev = dev;
	} else if (cfg->base == (uint32_t *)DT_REG_ADDR(DT_NODELABEL(gpioc))) {
		data->GPIOx = GPIOC;
		__SYSTEM_GPIOC_CLK_ENABLE();
		gpioc_isr_dev = dev;
	}
	IRQ_CONNECT(DT_INST_IRQN(0), DT_INST_IRQ(0, priority), gpioa_freqchip_isr, NULL, 0);
	IRQ_CONNECT(DT_INST_IRQN(1), DT_INST_IRQ(1, priority), gpiob_freqchip_isr, NULL, 0);
	IRQ_CONNECT(DT_INST_IRQN(2), DT_INST_IRQ(2, priority), gpioc_freqchip_isr, NULL, 0);
	irq_enable(DT_INST_IRQN(0));
	irq_enable(DT_INST_IRQN(1));
	irq_enable(DT_INST_IRQN(2));
#endif

	return 0;
}

static const struct gpio_driver_api gpio_freqchip_api = {
	.pin_configure = gpio_freqchip_configure,
	.port_get_raw = gpio_freqchip_port_get_raw,
	.port_set_masked_raw = gpio_freqchip_port_set_masked_raw,
	.port_set_bits_raw = gpio_freqchip_port_set_bits_raw,
	.port_clear_bits_raw = gpio_freqchip_port_clear_bits_raw,
	.port_toggle_bits = gpio_freqchip_port_toggle_bits,
	.pin_interrupt_configure = gpio_freqchip_pin_interrupt_configure,
	.manage_callback = gpio_freqchip_manage_callback,
};

#define GPIO_FREQCHIP_INIT(n)						                           \
	static const struct gpio_freqchip_config gpio_freqchip_config##n = {       \
		.common = {						                                       \
			.port_pin_mask = GPIO_PORT_PIN_MASK_FROM_DT_INST(n),               \
		},							                                           \
		.base = (uint32_t *)DT_INST_REG_ADDR(n), 		       			       \
	};								                                           \
									                                           \
	static struct gpio_freqchip_data gpio_freqchip_data##n;			           \
									                                           \
	DEVICE_DT_INST_DEFINE(n, gpio_freqchip_init, NULL, &gpio_freqchip_data##n, \
			             &gpio_freqchip_config##n, PRE_KERNEL_1,	           \
			             CONFIG_GPIO_INIT_PRIORITY, &gpio_freqchip_api);

DT_INST_FOREACH_STATUS_OKAY(GPIO_FREQCHIP_INIT)
