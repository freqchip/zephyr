/*
 * Copyright (c) 2026 Freqchip
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT freqchip_freqchip_wdt

#include <errno.h>

#include <zephyr/drivers/watchdog.h>
#include <zephyr/devicetree.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys_clock.h>

LOG_MODULE_REGISTER(wdt_freqchip, CONFIG_WDT_LOG_LEVEL);

/* WDT Control Register */
typedef struct
{
    uint32_t WDT_EN : 1;
    uint32_t RMOD   : 1;
    uint32_t RPL    : 3;
    uint32_t rsv_0  : 27;
}wdt_ctrl_t;

typedef struct 
{
    volatile wdt_ctrl_t      wdt_CR;            /* Offset 0x00 */
    volatile uint32_t        wdt_CCVR;          /* Offset 0x04 */
    volatile uint32_t        wdt_CRR;           /* Offset 0x08 */
    volatile uint32_t        wdt_STAT;          /* Offset 0x0C */
    volatile uint32_t        wdt_CNT;           /* Offset 0x10 */
}struct_WDT_t;

/* 获取设备树地址 */
#define WDT_BASE_ADDR           DT_INST_REG_ADDR(0)

#define WDT        ((struct_WDT_t *)(WDT_BASE_ADDR))

struct wdt_freqchip_dev_data {
	uint32_t timeout;
};

static int wdt_freqchip_setup(const struct device *dev, uint8_t options)
{
    if (options){
        WDT->wdt_CR.WDT_EN = 0;
    }

	return 0;
}

static int wdt_freqchip_disable(const struct device *dev)
{
	ARG_UNUSED(dev);

    WDT->wdt_CR.WDT_EN = 0;

	return 0;
}

static int wdt_freqchip_install_timeout(const struct device *dev, const struct wdt_timeout_cfg *config)
{

    /* init */
    WDT->wdt_CR.RMOD = 0;
    WDT->wdt_CNT = 1000*(config->window.max);
    WDT->wdt_CRR = 0x76;
    /* wdt count start */
    WDT->wdt_CR.WDT_EN = 1;

	return 0;
}

static int wdt_freqchip_feed(const struct device *dev, int channel_id)
{
	ARG_UNUSED(channel_id);

	WDT->wdt_CRR = 0x76;

	return 0;
}

static const struct wdt_driver_api wdt_freqchip_api = {
	.setup = wdt_freqchip_setup,
	.disable = wdt_freqchip_disable,
	.install_timeout = wdt_freqchip_install_timeout,
	.feed = wdt_freqchip_feed,
};

static int wdt_freqchip_init(const struct device *dev)
{
    /* WDT clock enable and select 1MHz clock source  */
    __SYSTEM_WDT_CLK_ENABLE();
    __SYSTEM_WDT_CLK_SELECT_1M();

	return 0;
}

static struct wdt_freqchip_dev_data wdt_freqchip_data;

DEVICE_DT_INST_DEFINE(0, wdt_freqchip_init, NULL, &wdt_freqchip_data, NULL,
		              POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEVICE,
		              &wdt_freqchip_api);
