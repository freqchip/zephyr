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
#include <zephyr/pm/device.h>

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

#ifdef CONFIG_PM_DEVICE

extern char freqchiip_deep_sleep;
#define WDT_REG_RD(reg)    (*(volatile uint32_t *)&(reg))
#define WDT_REG_WR(reg, v) (*(volatile uint32_t *)&(reg) = (v))

struct wdt_reg_backup_t {
    uint32_t WDT_REG[3];
    bool reg_valid;
};
static struct wdt_reg_backup_t reg_backup;

static int wdt_freqchip_pm_action(const struct device *dev, enum pm_device_action action)
{
	switch (action) 
    {
        case PM_DEVICE_ACTION_SUSPEND:
        {
            reg_backup.WDT_REG[0] = WDT_REG_RD(WDT->wdt_CR) & ~(1 << 0);
            reg_backup.WDT_REG[1] = WDT->wdt_CNT;
            reg_backup.WDT_REG[2] = WDT->wdt_CR.WDT_EN;
            reg_backup.reg_valid = true;
        }break;

        case PM_DEVICE_ACTION_RESUME:
        {
            if (freqchiip_deep_sleep == false)
                return 0;

            if (reg_backup.reg_valid) {
                reg_backup.reg_valid = false;

                __SYSTEM_WDT_CLK_ENABLE();
                __SYSTEM_WDT_CLK_SELECT_1M();

                WDT_REG_WR(WDT->wdt_CR, reg_backup.WDT_REG[0]);
                WDT->wdt_CNT =          reg_backup.WDT_REG[1];
                WDT->wdt_CRR = 0x76;
                if (reg_backup.WDT_REG[2]){
                    WDT->wdt_CR.WDT_EN = 1;
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

static int wdt_freqchip_init(const struct device *dev)
{
    /* WDT clock enable and select 1MHz clock source  */
    __SYSTEM_WDT_CLK_ENABLE();
    __SYSTEM_WDT_CLK_SELECT_1M();

#ifdef CONFIG_PM_DEVICE
    return pm_device_driver_init(dev, wdt_freqchip_pm_action);
#else
    return 0;
#endif
}

static struct wdt_freqchip_dev_data wdt_freqchip_data;

PM_DEVICE_DT_INST_DEFINE(0, wdt_freqchip_pm_action);

DEVICE_DT_INST_DEFINE(0, wdt_freqchip_init, PM_DEVICE_DT_INST_GET(0), &wdt_freqchip_data, NULL,
		              POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEVICE,
		              &wdt_freqchip_api);
