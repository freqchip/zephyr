/*
 * Copyright (c) 2026 Freqchip
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT freqchip_freqchip_i2c

#include <errno.h>

#include <zephyr/kernel.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/logging/log.h>
#include <zephyr/irq.h>

#include <driver_i2c.h>

LOG_MODULE_REGISTER(i2c_freqchip, CONFIG_I2C_LOG_LEVEL);

#include "i2c-priv.h"

struct i2c_freqchip_config {
	struct_I2C_t *base;
	uint32_t bitrate;
	const struct pinctrl_dev_config *pcfg;
};

struct i2c_freqchip_data {
    I2C_HandleTypeDef handle;
	struct k_sem bus_mutex;
};

static int i2c_freqchip_configure(const struct device *dev, uint32_t dev_config)
{
	struct i2c_freqchip_data *data = dev->data;
	const struct i2c_freqchip_config *cfg = dev->config;

	k_sem_take(&data->bus_mutex, K_FOREVER);

	/* Only support controller mode */
	if ((dev_config & I2C_MODE_CONTROLLER) == 0) {
		k_sem_give(&data->bus_mutex);
		return -ENOTSUP;
	}

    if (cfg->base == I2C0)
        __SYSTEM_I2C0_RESET();
    else if (cfg->base == I2C1)
        __SYSTEM_I2C1_RESET();
    else
        return -ENOTSUP;

    /* Configure HAL handle */
    data->handle.I2Cx = cfg->base;
	data->handle.Init.I2C_Mode = I2C_MODE_MASTER_7BIT;
	data->handle.Init.Slave_Address = 0;

    uint32_t i2c_clk = system_get_peripheral_clock(PER_CLK_I2Cx);
    // 2. 提取速率 (bit[3:1] 右移 1 位)
    switch (I2C_SPEED_GET(dev_config)) 
    {
        // 0x1, 100kHz
        case I2C_SPEED_STANDARD:{data->handle.Init.SCL_PCNT = i2c_clk / 100000;}break;
		// 0x2, 400kHz
		case I2C_SPEED_FAST:{data->handle.Init.SCL_PCNT = i2c_clk / 400000;}break;
		// 0x3, 1MHz
		case I2C_SPEED_FAST_PLUS:{data->handle.Init.SCL_PCNT = i2c_clk / 1000000;}break;
		// 0x4, 3.4MHz
		case I2C_SPEED_HIGH:{data->handle.Init.SCL_PCNT = i2c_clk / 3400000;}break;
		// 0x5, 5MHz
		case I2C_SPEED_ULTRA:{data->handle.Init.SCL_PCNT = i2c_clk / 5000000;}break;

		default:{data->handle.Init.SCL_PCNT = i2c_clk / 100000;}break;
    }
	i2c_init(&data->handle);

	k_sem_give(&data->bus_mutex);

	return 0;
}

static int i2c_master_restart_receive(I2C_HandleTypeDef *hi2c, uint16_t fu16_DevAddress, uint8_t *fp_Data, uint32_t fu32_Size)
{
    /* start transaction */
    hi2c->I2Cx->CTRL0 = PHASE_START | PHASE_STOP | PHASE_ADDR| PHASE_DATA | DIR_RECEIVER_MASTER | (fu32_Size << 16);
    hi2c->I2Cx->CMD = CMD_TRANSACTION;

    /* Waiting for the address to be sent successfully */    
    while (!(__I2C_GET_STATUS(hi2c->I2Cx) & INT_ADDR_SENDCOMPL));
    __I2C_CLEAR_STATUS(hi2c->I2Cx, INT_ADDR_SENDCOMPL);

    if(!(__I2C_GET_STATUS(hi2c->I2Cx) & INT_ADDRHIT))
    {
        /* DevAddress NACK */
        while (__I2C_GET_STATUS(hi2c->I2Cx) & GEN_BUS_BUSY);
        __I2C_CLEAR_STATUS(hi2c->I2Cx, INT_COMPL_ACTIVE);

        __I2C_DISABLE(hi2c->I2Cx);
    
        return I2C_RET_CODES_ADDR_NAK;  
    }
    __I2C_CLEAR_STATUS(hi2c->I2Cx, INT_ADDRHIT);    
    
    while (fu32_Size)
    {
        if (!(__I2C_GET_STATUS(hi2c->I2Cx) & INT_FIFO_EMPTY))
        {
            *fp_Data++ =  hi2c->I2Cx->DATA;

            fu32_Size--;
        }
    }
    
    while (__I2C_GET_STATUS(hi2c->I2Cx) & GEN_BUS_BUSY); 
    __I2C_CLEAR_STATUS(hi2c->I2Cx, INT_COMPL_ACTIVE);  

    return I2C_RET_CODES_OK;
}

static int i2c_master_transmit_without_stop(I2C_HandleTypeDef *hi2c, uint16_t fu16_DevAddress, uint8_t *fp_Data, uint32_t fu32_Size)
{
    if (fu32_Size == 0)  return I2C_RET_CODES_PARAM_ERROR;
    if (hi2c->b_TxBusy)  return I2C_RET_CODES_TX_BUSY;
    if (hi2c->b_RxBusy)  return I2C_RET_CODES_RX_BUSY;

    while (__I2C_GET_STATUS(hi2c->I2Cx) & GEN_BUS_BUSY);
    __I2C_CLEAR_STATUS(hi2c->I2Cx, INT_COMPL_ACTIVE);
    __I2C_DISABLE(hi2c->I2Cx);
    
    /* transaction sends all complete phases */ 
    hi2c->I2Cx->ADDR  = fu16_DevAddress >> 1; 
    hi2c->I2Cx->CTRL0 = PHASE_START | PHASE_ADDR | PHASE_DATA | (0x20000);

    /* start transaction */
    __I2C_ENABLE(hi2c->I2Cx);
    hi2c->I2Cx->CMD = CMD_TRANSACTION;

    /* Waiting for the address to be sent successfully */
    while (!(__I2C_GET_STATUS(hi2c->I2Cx) & INT_ADDR_SENDCOMPL)); 
    __I2C_CLEAR_STATUS(hi2c->I2Cx, INT_ADDR_SENDCOMPL);

    if(!(__I2C_GET_STATUS(hi2c->I2Cx) & INT_ADDRHIT))
    {
        /* send stop condition */
        hi2c->I2Cx->CTRL0 = PHASE_STOP;
        hi2c->I2Cx->CMD = CMD_TRANSACTION;
        /* DevAddress NACK */
        while (__I2C_GET_STATUS(hi2c->I2Cx) & GEN_BUS_BUSY);
        __I2C_CLEAR_STATUS(hi2c->I2Cx, INT_COMPL_ACTIVE);

        __I2C_DISABLE(hi2c->I2Cx);

        return I2C_RET_CODES_ADDR_NAK;
    }
    __I2C_CLEAR_STATUS(hi2c->I2Cx, INT_ADDRHIT);

    while (fu32_Size)
    {
        if (!(__I2C_GET_STATUS(hi2c->I2Cx) & INT_FIFO_FULL))
        {
            hi2c->I2Cx->DATA = *fp_Data++;
    
            fu32_Size--;
        }
    
        if (__I2C_GET_STATUS(hi2c->I2Cx) & INT_COMPL_ACTIVE)
        {
            while (__I2C_GET_STATUS(hi2c->I2Cx) & GEN_BUS_BUSY);            
            __I2C_CLEAR_STATUS(hi2c->I2Cx, INT_COMPL_ACTIVE);
            __I2C_DISABLE(hi2c->I2Cx);
    
            return I2C_RET_CODES_ADDR_NAK;
        }
    } 

    //Waiting for transaction completion    
    while (!(__I2C_GET_STATUS(hi2c->I2Cx) & INT_COMPL_ACTIVE)); 
    __I2C_CLEAR_STATUS(hi2c->I2Cx, INT_COMPL_ACTIVE); 

    return I2C_RET_CODES_OK;
}

/*
    纯写:         I2C_MSG_WRITE | I2C_MSG_STOP                  → flag = 0x02
    纯读:         I2C_MSG_READ  | I2C_MSG_STOP                  → flag = 0x03
    写+无STOP:    I2C_MSG_WRITE                                 → flag = 0x00
    读+RESTART:   I2C_MSG_RESTART | I2C_MSG_READ | I2C_MSG_STOP → flag = 0x07
*/
static int i2c_freqchip_transfer(const struct device *dev, struct i2c_msg *msgs, uint8_t num_msgs, uint16_t addr)
{
    struct i2c_freqchip_data *data = dev->data;
	int ret = I2C_RET_CODES_OK;

	/* 获取 I2C 总线互斥锁，保证多线程安全 */
	k_sem_take(&data->bus_mutex, K_FOREVER);

	/* 遍历所有消息段 */
	for (int i = 0; i < num_msgs; i++) 
    {
		/* 不支持 10 位地址的设备 */
		if (I2C_MSG_ADDR_10_BITS & msgs[i].flags) {
			ret = -ENOTSUP;
			break;
		}

        /* 本条消息后需要发 STOP */
        if (msgs[i].flags & I2C_MSG_STOP)
        {
            if (msgs[i].flags & I2C_MSG_RESTART)
            {/* RESTART + STOP */
                if ((msgs[i].flags & I2C_MSG_RW_MASK) == I2C_MSG_READ) {
                    ret = i2c_master_restart_receive(&data->handle, addr, msgs[i].buf, msgs[i].len);
                } 
            }
            else
            {/* START + STOP */
                if ((msgs[i].flags & I2C_MSG_RW_MASK) == I2C_MSG_READ) {
                    ret = i2c_master_receive(&data->handle, addr, msgs[i].buf, msgs[i].len);
                } 
                else {
                    ret = i2c_master_transmit(&data->handle, addr, msgs[i].buf, msgs[i].len);
                }
            }
        }
        /* 本条消息后不发 STOP: 保留总线，供后续消息组合（如写寄存器+读数据） */
        else
        {
            ret = i2c_master_transmit_without_stop(&data->handle, addr, msgs[i].buf, msgs[i].len);
        }

        /* error */
		if (ret < 0) {
			break;
		}
	}

	/* 释放总线互斥锁 */
	k_sem_give(&data->bus_mutex);

	return ret;
}

static const struct i2c_driver_api i2c_freqchip_driver_api = {
	.configure = i2c_freqchip_configure,
	.transfer = i2c_freqchip_transfer,
};

static int i2c_freqchip_init(const struct device *dev)
{
	struct i2c_freqchip_data *data = dev->data;
	const struct i2c_freqchip_config *cfg = dev->config;
	int err;

	/* 1. Enable I2C0/I2C1 peripheral clocks */
	__SYSTEM_I2C0_CLK_ENABLE();
	__SYSTEM_I2C1_CLK_ENABLE();

	/* 2. Apply DTS pinctrl: configure SCL/SDA pins (function, pull-up, open-drain) */
	err = pinctrl_apply_state(cfg->pcfg, PINCTRL_STATE_DEFAULT);
	if (err < 0) {
		return err;
	}

	k_sem_init(&data->bus_mutex, 1, 1);

	err = i2c_freqchip_configure(dev, I2C_MODE_CONTROLLER | i2c_map_dt_bitrate(cfg->bitrate));
	if (err) {
		return err;
	}

	return 0;
}

#define I2C_FREQCHIP_INIT(inst)                                              \
	PINCTRL_DT_INST_DEFINE(inst);                                            \
	static const struct i2c_freqchip_config i2c_freqchip_cfg_##inst = {      \
		.base = (struct_I2C_t *)DT_INST_REG_ADDR(inst),                      \
		.bitrate = DT_INST_PROP(inst, clock_frequency),                      \
		.pcfg = PINCTRL_DT_INST_DEV_CONFIG_GET(inst),                        \
	};                                                                       \
	static struct i2c_freqchip_data i2c_freqchip_data_##inst;                \
                                                                             \
	I2C_DEVICE_DT_INST_DEFINE(inst,                                          \
				              i2c_freqchip_init, NULL,                       \
				              &i2c_freqchip_data_##inst,                     \
				              &i2c_freqchip_cfg_##inst,                      \
				              POST_KERNEL, CONFIG_I2C_INIT_PRIORITY,         \
				              &i2c_freqchip_driver_api);

DT_INST_FOREACH_STATUS_OKAY(I2C_FREQCHIP_INIT)
