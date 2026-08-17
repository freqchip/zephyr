/*
 * Copyright (c) 2026 Freqchip
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT freqchip_freqchip_usart

#include <zephyr/irq.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/drivers/reset.h>
#include <zephyr/drivers/uart.h>

#include <driver_usart.h>

struct freq_usart_config {
	const struct pinctrl_dev_config *pcfg;
#ifdef CONFIG_UART_INTERRUPT_DRIVEN
	unsigned int irq_num;
	unsigned int irq_prio;
#endif /* CONFIG_UART_INTERRUPT_DRIVEN */
};

struct freq_usart_data {
	USART_HandleTypeDef hUSARTx;
	struct device *dev;
#ifdef CONFIG_UART_INTERRUPT_DRIVEN
	uart_irq_callback_user_data_t user_cb;
	void *user_data;
#endif /* CONFIG_UART_INTERRUPT_DRIVEN */
};


#ifdef CONFIG_UART_INTERRUPT_DRIVEN

static int usart_freq_fifo_fill(const struct device *dev, const uint8_t *tx_data, int size)
{
	uint32_t txcount = 0;
	USART_HandleTypeDef *USART_handle = &((struct freq_usart_data *)dev->data)->hUSARTx;

	/* Until TxFIFO full */
	while((txcount < size) && __USART_GET_SR_STATUS(USART_handle->USARTx, USART_STATUS_TXFIFO_NOFULL)){
		USART_handle->USARTx->DR = tx_data[txcount++];        
	}            
	return txcount;
}

static int usart_freq_fifo_read(const struct device *dev, uint8_t *rx_data, int size)
{
	uint32_t rxcount = 0;
	USART_HandleTypeDef *USART_handle = &((struct freq_usart_data *)dev->data)->hUSARTx;

	/* Rx ready */        
	while((rxcount < size) && __USART_GET_SR_STATUS(USART_handle->USARTx, USART_STATUS_DR)){
		rx_data[rxcount++] = USART_handle->USARTx->DR;
	}            
	return rxcount;
}

static void usart_freq_irq_tx_enable(const struct device *dev)
{
	USART_HandleTypeDef *USARTx = &((struct freq_usart_data *)dev->data)->hUSARTx;
	__USART_INT_STATUS_ENABLE(USARTx->USARTx, USART_INT_TX);
}

static void usart_freq_irq_tx_disable(const struct device *dev)
{
	USART_HandleTypeDef *USARTx = &((struct freq_usart_data *)dev->data)->hUSARTx;
	__USART_INT_STATUS_DISABLE(USARTx->USARTx, USART_INT_TX);
}

static int usart_freq_irq_tx_ready(const struct device *dev)
{
	USART_HandleTypeDef *USARTx = &((struct freq_usart_data *)dev->data)->hUSARTx;
	return __USART_GET_SR_STATUS(USARTx->USARTx, USART_STATUS_TXFIFO_NOFULL) ? 1 : 0;
}

static int usart_freq_irq_tx_complete(const struct device *dev)
{
	USART_HandleTypeDef *USARTx = &((struct freq_usart_data *)dev->data)->hUSARTx;
	return __USART_GET_SR_STATUS(USARTx->USARTx, USART_STATUS_TXFIFO_EMPTY) ? 1 : 0;
}

static void usart_freq_irq_rx_enable(const struct device *dev)
{
	USART_HandleTypeDef *USARTx = &((struct freq_usart_data *)dev->data)->hUSARTx;
	__USART_INT_STATUS_ENABLE(USARTx->USARTx, USART_INT_RX);
	__USART_INT_STATUS_ENABLE(USARTx->USARTx, USART_INT_RFTO);
}

static void usart_freq_irq_rx_disable(const struct device *dev)
{
	USART_HandleTypeDef *USARTx = &((struct freq_usart_data *)dev->data)->hUSARTx;
	__USART_INT_STATUS_DISABLE(USARTx->USARTx, USART_INT_RX);
	__USART_INT_STATUS_DISABLE(USARTx->USARTx, USART_INT_RFTO);
}

static int usart_freq_irq_rx_ready(const struct device *dev)
{
	USART_HandleTypeDef *USARTx = &((struct freq_usart_data *)dev->data)->hUSARTx;
	return __USART_GET_SR_STATUS(USARTx->USARTx, USART_STATUS_RXFIFO_NOEMPTY) ? 1 : 0;
}

static void usart_freq_irq_err_enable(const struct device *dev)
{
	USART_HandleTypeDef *USARTx = &((struct freq_usart_data *)dev->data)->hUSARTx;
	__USART_INT_STATUS_ENABLE(USARTx->USARTx, USART_INT_ROV | USART_INT_FE | USART_INT_PE);
}

static void usart_freq_irq_err_disable(const struct device *dev)
{
	USART_HandleTypeDef *USARTx = &((struct freq_usart_data *)dev->data)->hUSARTx;
	__USART_INT_STATUS_DISABLE(USARTx->USARTx, USART_INT_ROV | USART_INT_FE | USART_INT_PE);
}

static int usart_freq_irq_is_pending(const struct device *dev)
{
	USART_HandleTypeDef *USARTx = &((struct freq_usart_data *)dev->data)->hUSARTx;
	if (__USART_GET_INT_STATUS(USARTx->USARTx, USART_INT_RX) && __USART_GET_SR_STATUS(USARTx->USARTx, USART_STATUS_RXFIFO_NOEMPTY)) return 1;
	if (__USART_GET_INT_STATUS(USARTx->USARTx, USART_INT_TX) && __USART_GET_SR_STATUS(USARTx->USARTx, USART_STATUS_TXFIFO_NOFULL)) return 1;
	if (__USART_GET_INT_STATUS(USARTx->USARTx, USART_INT_ROV | USART_INT_FE | USART_INT_PE)) return 1;
	return 0;
}

static int usart_freq_irq_update(const struct device *dev)
{
	USART_HandleTypeDef *USARTx = &((struct freq_usart_data *)dev->data)->hUSARTx;

	if (__USART_GET_INT_STATUS(USARTx->USARTx, USART_INT_RX) && __USART_GET_SR_STATUS(USARTx->USARTx, USART_STATUS_RXFIFO_NOEMPTY)) {return 1;}
	if (__USART_GET_INT_STATUS(USARTx->USARTx, USART_INT_TX) && __USART_GET_SR_STATUS(USARTx->USARTx, USART_STATUS_TXFIFO_NOFULL)) {return 1;}
	if (__USART_GET_INT_STATUS(USARTx->USARTx, USART_INT_ROV | USART_INT_FE | USART_INT_PE))
	{
		__USART_CLEAN_INT_STATUS(USARTx->USARTx, USART_INT_ROV | USART_INT_FE | USART_INT_PE);
		return 1;
	}

	return 0;
}

static void usart_freq_irq_callback_set(const struct device *dev, uart_irq_callback_user_data_t cb, void *user_data)
{
	struct freq_usart_data *data = dev->data;
	data->user_cb = cb;
	data->user_data = user_data;
}

static void usart0_freq_isr(const void *arg)
{
	const struct device *dev = (const struct device *)arg;
	struct freq_usart_data *data = dev->data;

	if (data->user_cb) 
		data->user_cb(dev, data->user_data);
}
static void usart1_freq_isr(const void *arg)
{
	const struct device *dev = (const struct device *)arg;
	struct freq_usart_data *data = dev->data;

	if (data->user_cb) 
		data->user_cb(dev, data->user_data);
}
static void usart2_freq_isr(const void *arg)
{
	const struct device *dev = (const struct device *)arg;
	struct freq_usart_data *data = dev->data;

	if (data->user_cb) 
		data->user_cb(dev, data->user_data);
}
static void usart3_freq_isr(const void *arg)
{
	const struct device *dev = (const struct device *)arg;
	struct freq_usart_data *data = dev->data;

	if (data->user_cb) 
		data->user_cb(dev, data->user_data);
}
#endif /* CONFIG_UART_INTERRUPT_DRIVEN */

static int usart_freq_init(const struct device *dev)
{
	int ret;

	const struct freq_usart_config *cfg = (const struct freq_usart_config *)dev->config;
	USART_HandleTypeDef *USART_handle = &((struct freq_usart_data *)dev->data)->hUSARTx;

	((struct freq_usart_data *)dev->data)->dev = (struct device *)dev;

	/* 配置引脚复用 (pinctrl) */
	ret = pinctrl_apply_state(cfg->pcfg, PINCTRL_STATE_DEFAULT);
	if (ret < 0) {
		return ret;
	}

	USART_handle->Init.TxFIFOEmpty_Threshold = 1;
	USART_handle->Init.RxFIFOFull_Threshold  = 1;

	if (USART_handle->Init.DataLength == 8)	
		USART_handle->Init.DataLength = USART_DATA_LENGTH_8BIT;
	else if (USART_handle->Init.DataLength == 7)
		USART_handle->Init.DataLength = USART_DATA_LENGTH_7BIT;
	else if (USART_handle->Init.DataLength == 9)
		USART_handle->Init.DataLength = USART_DATA_LENGTH_9BIT;

	if (USART_handle->USARTx == USART0)
		__SYSTEM_USART0_CLK_ENABLE();
	if (USART_handle->USARTx == USART1)
		__SYSTEM_USART1_CLK_ENABLE();
	if (USART_handle->USARTx == USART2)
		__SYSTEM_USART2_CLK_ENABLE();
	if (USART_handle->USARTx == USART3)
		__SYSTEM_USART3_CLK_ENABLE();
    /* init uart */   
	usart_init(USART_handle);

#ifdef CONFIG_UART_INTERRUPT_DRIVEN
	if (USART_handle->USARTx == USART0){irq_connect_dynamic(cfg->irq_num, cfg->irq_prio, usart0_freq_isr, (void *)dev, 0);}
	if (USART_handle->USARTx == USART1){irq_connect_dynamic(cfg->irq_num, cfg->irq_prio, usart1_freq_isr, (void *)dev, 0);}
	if (USART_handle->USARTx == USART2){irq_connect_dynamic(cfg->irq_num, cfg->irq_prio, usart2_freq_isr, (void *)dev, 0);}
	if (USART_handle->USARTx == USART3){irq_connect_dynamic(cfg->irq_num, cfg->irq_prio, usart3_freq_isr, (void *)dev, 0);}

	irq_enable(cfg->irq_num);
#endif /* CONFIG_UART_INTERRUPT_DRIVEN */

	return 0;
}

static int usart_freq_poll_in(const struct device *dev, unsigned char *c)
{
	USART_HandleTypeDef *USARTx = &((struct freq_usart_data *)dev->data)->hUSARTx;
	usart_receive(USARTx, c, 1);
	return 0;
}

static void usart_freq_poll_out(const struct device *dev, unsigned char c)
{
	USART_HandleTypeDef *USARTx = &((struct freq_usart_data *)dev->data)->hUSARTx;
	uint8_t data = c;
	usart_transmit(USARTx, &data, 1);
}

static int usart_freq_err_check(const struct device *dev)
{
	return 0;
}


static const struct uart_driver_api usart_freq_driver_api = {
	.poll_in = usart_freq_poll_in,
	.poll_out = usart_freq_poll_out,
	.err_check = usart_freq_err_check,
#ifdef CONFIG_UART_INTERRUPT_DRIVEN
	.fifo_fill = usart_freq_fifo_fill,
	.fifo_read = usart_freq_fifo_read,
	.irq_tx_enable = usart_freq_irq_tx_enable,
	.irq_tx_disable = usart_freq_irq_tx_disable,
	.irq_tx_ready = usart_freq_irq_tx_ready,
	.irq_tx_complete = usart_freq_irq_tx_complete,
	.irq_rx_enable = usart_freq_irq_rx_enable,
	.irq_rx_disable = usart_freq_irq_rx_disable,
	.irq_rx_ready = usart_freq_irq_rx_ready,
	.irq_err_enable = usart_freq_irq_err_enable,
	.irq_err_disable = usart_freq_irq_err_disable,
	.irq_is_pending = usart_freq_irq_is_pending,
	.irq_update = usart_freq_irq_update,
	.irq_callback_set = usart_freq_irq_callback_set,
#endif /* CONFIG_UART_INTERRUPT_DRIVEN */
};

#ifdef CONFIG_UART_INTERRUPT_DRIVEN
#define FREQ_USART_IRQ_CFG(n)   \
	.irq_num = DT_INST_IRQN(n), \
	.irq_prio = DT_INST_IRQ(n, priority),
#else
#define FREQ_USART_IRQ_CFG(n)
#endif /* CONFIG_UART_INTERRUPT_DRIVEN */

#define FREQ_USART_INIT(n)							                                \
	PINCTRL_DT_INST_DEFINE(n);							                            \
	static const struct freq_usart_config usart_freq_config_##n = {		            \
		.pcfg = PINCTRL_DT_INST_DEV_CONFIG_GET(n),					                \
		FREQ_USART_IRQ_CFG(n)                                                       \
	};																	            \
	static struct freq_usart_data usart_freq_data_##n = {			                \
		.hUSARTx = {													            \
			.USARTx = (struct_USART_t *)DT_INST_REG_ADDR(n),			            \
			.Init.BaudRate = DT_INST_PROP(n, current_speed),			            \
			.Init.DataLength = DT_INST_PROP(n, data_bits),                          \
			.Init.StopBits = DT_INST_ENUM_IDX_OR(n, stop_bit, 0),                   \
			.Init.Parity = DT_INST_ENUM_IDX_OR(n, parity, 0),                       \
			.Init.TxFIFOEmpty_Threshold = DT_INST_PROP(n, tx_fifo_empty_threshold), \
			.Init.RxFIFOFull_Threshold = DT_INST_PROP(n, rx_fifo_full_threshold),   \
			.Init.AUTO_FLOW = DT_INST_PROP(n, hw_flow_control),                     \
		},							                                                \
	};									                                            \
																					\
	DEVICE_DT_INST_DEFINE(n, &usart_freq_init,				                        \
			              NULL,						                                \
			              &usart_freq_data_##n,				                        \
			              &usart_freq_config_##n, PRE_KERNEL_1,                     \
			              CONFIG_SERIAL_INIT_PRIORITY,			                    \
			              &usart_freq_driver_api);

DT_INST_FOREACH_STATUS_OKAY(FREQ_USART_INIT)
