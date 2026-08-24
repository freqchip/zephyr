/*
 * Copyright (c) 2026 Freqchip
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "system_fr802x.h"
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#define DT_DRV_COMPAT freqchip_freqchip_gpio

#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/gpio/gpio_utils.h>
#include <zephyr/irq.h>
#include <zephyr/pm/device.h>

#include <driver_gpio.h>

struct gpio_freqchip_config {
	struct gpio_driver_config common;
	uint32_t *base;
    uint8_t irq_num;
    uint8_t irq_prio;
};

struct gpio_freqchip_data {
	struct gpio_driver_data common;
	sys_slist_t callbacks;
	GPIO_TypeDef *GPIOx;
};

static void gpioa_freqchip_isr(const void *arg)
{
	const struct device *dev = (const struct device *)arg;
	struct gpio_freqchip_data *data = dev->data;
	uint32_t int_status;

	while ((int_status = data->GPIOx->GPIO_INTS) != 0) {
		data->GPIOx->GPIO_INTS = int_status;
		gpio_fire_callbacks(&data->callbacks, dev, int_status);
	}
}
static void gpiob_freqchip_isr(const void *arg)
{
	const struct device *dev = (const struct device *)arg;
	struct gpio_freqchip_data *data = dev->data;
	uint32_t int_status;

	while ((int_status = data->GPIOx->GPIO_INTS) != 0) {
		data->GPIOx->GPIO_INTS = int_status;
		gpio_fire_callbacks(&data->callbacks, dev, int_status);
	}
}
static void gpioc_freqchip_isr(const void *arg)
{
	const struct device *dev = (const struct device *)arg;
	struct gpio_freqchip_data *data = dev->data;
	uint32_t int_status;

	while ((int_status = data->GPIOx->GPIO_INTS) != 0) {
		data->GPIOx->GPIO_INTS = int_status;
		gpio_fire_callbacks(&data->callbacks, dev, int_status);
	}
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

#ifdef CONFIG_PM_DEVICE

extern char freqchiip_deep_sleep;

struct gpio_reg_backup_t {
    uint32_t GPIOA_REG[6];
    uint32_t GPIOA_CTRL[12];
    uint32_t GPIOB_REG[6];
    uint32_t GPIOB_CTRL[12];
    uint32_t GPIOC_REG[6];
    uint32_t GPIOC_CTRL[12];
    uint8_t  GPIOA_IRQ_PRIO;
    uint8_t  GPIOB_IRQ_PRIO;
    uint8_t  GPIOC_IRQ_PRIO;
    bool reg_valid[3];   /* 仅真实挂起备份过才恢复寄存器，避免初始化时用空数据覆盖 */
};
static struct gpio_reg_backup_t reg_backup;

static int gpio_freqchip_pm_action(const struct device *dev, enum pm_device_action action)
{
    const struct gpio_freqchip_config *cfg = dev->config;
	struct gpio_freqchip_data *data = dev->data;
    uint32_t *P;
    int i;

    switch (action) 
    {
        case PM_DEVICE_ACTION_SUSPEND:
        {
            if (data->GPIOx == GPIOA){
                reg_backup.GPIOA_REG[0] = GPIOA->GPIO_OEN;
                reg_backup.GPIOA_REG[1] = GPIOA->GPIO_ODR;
                reg_backup.GPIOA_REG[2] = GPIOA->GPIO_OLCKR;
                reg_backup.GPIOA_REG[3] = GPIOA->GPIO_INTE;
                reg_backup.GPIOA_REG[4] = GPIOA->GPIO_INTTL;
                reg_backup.GPIOA_REG[5] = GPIOA->GPIO_INTTH;

                P = (uint32_t *)&SYSTEM->GPIOA_CTRL;
                for (i = 0; i < 12; i++){
                    reg_backup.GPIOA_CTRL[i] = P[i];
                }
                reg_backup.GPIOA_IRQ_PRIO = NVIC_GetPriority(cfg->irq_num);
                reg_backup.reg_valid[0] = true;
            }
            else if (data->GPIOx == GPIOB){
                reg_backup.GPIOB_REG[0] = GPIOB->GPIO_OEN;
                reg_backup.GPIOB_REG[1] = GPIOB->GPIO_ODR;
                reg_backup.GPIOB_REG[2] = GPIOB->GPIO_OLCKR;
                reg_backup.GPIOB_REG[3] = GPIOB->GPIO_INTE;
                reg_backup.GPIOB_REG[4] = GPIOB->GPIO_INTTL;
                reg_backup.GPIOB_REG[5] = GPIOB->GPIO_INTTH;

                P = (uint32_t *)&SYSTEM->GPIOB_CTRL;
                for (i = 0; i < 12; i++){
                    reg_backup.GPIOB_CTRL[i] = P[i];
                }
                reg_backup.GPIOB_IRQ_PRIO = NVIC_GetPriority(cfg->irq_num);
                reg_backup.reg_valid[1] = true;
            }
            else if (data->GPIOx == GPIOC){
                reg_backup.GPIOC_REG[0] = GPIOC->GPIO_OEN;
                reg_backup.GPIOC_REG[1] = GPIOC->GPIO_ODR;
                reg_backup.GPIOC_REG[2] = GPIOC->GPIO_OLCKR;
                reg_backup.GPIOC_REG[3] = GPIOC->GPIO_INTE;
                reg_backup.GPIOC_REG[4] = GPIOC->GPIO_INTTL;
                reg_backup.GPIOC_REG[5] = GPIOC->GPIO_INTTH;

                P = (uint32_t *)&SYSTEM->GPIOC_CTRL;
                for (i = 0; i < 12; i++){
                    reg_backup.GPIOC_CTRL[i] = P[i];
                }
                reg_backup.GPIOC_IRQ_PRIO = NVIC_GetPriority(cfg->irq_num);
                reg_backup.reg_valid[2] = true;
            }
        }break;

        case PM_DEVICE_ACTION_RESUME:
        {
            if (freqchiip_deep_sleep == false)
                return 0;

            if (data->GPIOx == GPIOA) {
                if (reg_backup.reg_valid[0]){
                    reg_backup.reg_valid[0] = false;

                    __SYSTEM_GPIOA_CLK_ENABLE();
                    GPIOA->GPIO_ODR   = reg_backup.GPIOA_REG[1];
                    GPIOA->GPIO_OEN   = reg_backup.GPIOA_REG[0];
                    GPIOA->GPIO_INTTL = reg_backup.GPIOA_REG[4];
                    GPIOA->GPIO_INTTH = reg_backup.GPIOA_REG[5];
                    GPIOA->GPIO_INTE  = reg_backup.GPIOA_REG[3];
                    GPIOA->GPIO_OLCKR = reg_backup.GPIOA_REG[2];

                    P = (uint32_t *)&SYSTEM->GPIOA_CTRL;
                    for (i = 0; i < 12; i++){
                        P[i] = reg_backup.GPIOA_CTRL[i];
                    }
                    NVIC_SetPriority(cfg->irq_num, reg_backup.GPIOA_IRQ_PRIO);
                    irq_enable(cfg->irq_num);
                }
            } 
            else if (data->GPIOx == GPIOB) {
                if (reg_backup.reg_valid[1]){
                    reg_backup.reg_valid[1] = false;

                    __SYSTEM_GPIOB_CLK_ENABLE();
                    GPIOB->GPIO_ODR   = reg_backup.GPIOB_REG[1];
                    GPIOB->GPIO_OEN   = reg_backup.GPIOB_REG[0];
                    GPIOB->GPIO_INTTL = reg_backup.GPIOB_REG[4];
                    GPIOB->GPIO_INTTH = reg_backup.GPIOB_REG[5];
                    GPIOB->GPIO_INTE  = reg_backup.GPIOB_REG[3];
                    GPIOB->GPIO_OLCKR = reg_backup.GPIOB_REG[2];

                    P = (uint32_t *)&SYSTEM->GPIOB_CTRL;
                    for (i = 0; i < 12; i++){
                        P[i] = reg_backup.GPIOB_CTRL[i];
                    }
                    NVIC_SetPriority(cfg->irq_num, reg_backup.GPIOB_IRQ_PRIO);
                    irq_enable(cfg->irq_num);
                }
            } 
            else if (data->GPIOx == GPIOC) {
                if (reg_backup.reg_valid[2]){
                        reg_backup.reg_valid[2] = false;

                    __SYSTEM_GPIOC_CLK_ENABLE();
                    GPIOC->GPIO_ODR   = reg_backup.GPIOC_REG[1];
                    GPIOC->GPIO_OEN   = reg_backup.GPIOC_REG[0];
                    GPIOC->GPIO_INTTL = reg_backup.GPIOC_REG[4];
                    GPIOC->GPIO_INTTH = reg_backup.GPIOC_REG[5];
                    GPIOC->GPIO_INTE  = reg_backup.GPIOC_REG[3];
                    GPIOC->GPIO_OLCKR = reg_backup.GPIOC_REG[2];

                    P = (uint32_t *)&SYSTEM->GPIOC_CTRL;
                    for (i = 0; i < 12; i++){
                        P[i] = reg_backup.GPIOC_CTRL[i];
                    }
                    NVIC_SetPriority(cfg->irq_num, reg_backup.GPIOC_IRQ_PRIO);
                    irq_enable(cfg->irq_num);
                }
            }
        }break;

        case PM_DEVICE_ACTION_TURN_ON:
        case PM_DEVICE_ACTION_TURN_OFF:
            return 0;

        default: return -ENOTSUP;
	}
	return 0;
}
#endif /* CONFIG_PM_DEVICE */

static int gpio_freqchip_init(const struct device *dev)
{
	const struct gpio_freqchip_config *cfg = dev->config;
	struct gpio_freqchip_data *data = dev->data;
	int ret = 0;

#ifdef CONFIG_SOC_SERIES_FR802X
    if (cfg->base == (uint32_t *)DT_REG_ADDR(DT_NODELABEL(gpioa))) {
        data->GPIOx = GPIOA;
        __SYSTEM_GPIOA_CLK_ENABLE();
        irq_connect_dynamic(cfg->irq_num, cfg->irq_prio, gpioa_freqchip_isr, dev, 0);
    } 
    else if (cfg->base == (uint32_t *)DT_REG_ADDR(DT_NODELABEL(gpiob))) {
        data->GPIOx = GPIOB;
        __SYSTEM_GPIOB_CLK_ENABLE();
        irq_connect_dynamic(cfg->irq_num, cfg->irq_prio, gpiob_freqchip_isr, dev, 0);
    } 
    else if (cfg->base == (uint32_t *)DT_REG_ADDR(DT_NODELABEL(gpioc))) {
        data->GPIOx = GPIOC;
        __SYSTEM_GPIOC_CLK_ENABLE();
        irq_connect_dynamic(cfg->irq_num, cfg->irq_prio, gpioc_freqchip_isr, dev, 0);
    }
    irq_enable(cfg->irq_num);
#endif

#ifdef CONFIG_PM_DEVICE
	/* 初始化设备 PM 状态 */
	ret = pm_device_driver_init(dev, gpio_freqchip_pm_action);
#endif

    gpio_status_auto_latch_enable(true);

	return ret;
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
		.base = (uint32_t *)DT_INST_REG_ADDR(n),                               \
        .irq_num  = DT_INST_IRQN(n),                                           \
        .irq_prio = DT_INST_IRQ(n, priority),                                  \
	};								                                           \
									                                           \
	static struct gpio_freqchip_data gpio_freqchip_data##n;			           \
									                                           \
	PM_DEVICE_DT_INST_DEFINE(n, gpio_freqchip_pm_action);		               \
									                                           \
	DEVICE_DT_INST_DEFINE(n, gpio_freqchip_init, PM_DEVICE_DT_INST_GET(n),     \
			             &gpio_freqchip_data##n,	                           \
			             &gpio_freqchip_config##n, PRE_KERNEL_1,	           \
			             CONFIG_GPIO_INIT_PRIORITY, &gpio_freqchip_api);

DT_INST_FOREACH_STATUS_OKAY(GPIO_FREQCHIP_INIT)
