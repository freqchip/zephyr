/*
 * Copyright (c) 2024 Freqchip
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * FR802x (FREQCHIP MACAN) CAN driver for Zephyr.
 *
 * Modelled on the STM32 FDCAN driver (zephyr/drivers/can/can_stm32_fdcan.c):
 * it reuses the Zephyr CAN subsystem's generic layer
 * (can_driver_api / can_frame / can_filter) and only implements the hardware
 * back-end on top of the FREQCHIP MACAN HAL
 * (modules/hal/freqchip/fr802x/components/driver/peripheral/driver_can.h).
 *
 * Unlike STM32 FDCAN (which is Bosch M_CAN compatible and therefore reuses
 * Zephyr's can_mcan common code), the FR802x MACAN is a proprietary
 * peripheral, so this file wires the Zephyr CAN API directly to the MACAN HAL.
 */

#include <errno.h>

#include <zephyr/kernel.h>
#include <zephyr/drivers/can.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/irq.h>
#include <zephyr/sys/__assert.h>
#include <zephyr/sys/util.h>

#include "driver_can.h"
#include "system_fr802x.h"

#define DT_DRV_COMPAT freqchip_freqchip_can

/* Forward declaration: notify_state_change() calls can_fr802x_get_state(). */
static int can_fr802x_get_state(const struct device *dev, enum can_state *state, struct can_bus_err_cnt *err_cnt);
static int can_fr802x_init(const struct device *dev);

/* Message RAM layout. Keep within the MACAN limits:
 *  - up to 128 standard / 64 extended ID filter elements
 *  - up to 32 Tx elements (FIFO + dedicated buffers)
 *  - up to 64 Rx FIFO0 / FIFO1 elements
 */
#define FR802X_STD_FILTERS	128U
#define FR802X_EXT_FILTERS	64U
#define FR802X_TX_FIFO		32U
#define FR802X_RX_FIFO0		64U
#define FR802X_RX_FIFO1		64U

struct can_fr802x_config {
	/* Must be first. */
	struct can_driver_config common;

	uintptr_t base;
	const struct pinctrl_dev_config *pcfg;

	uint8_t irq0;
	uint8_t irq1;
	uint8_t irq0_prio;
	uint8_t irq1_prio;
};

struct can_fr802x_data {
	/* Must be first. */
	struct can_driver_data common;
	struct device *dev;

	CAN_HandleTypeDef hcan;
	uint32_t CanBuffer[4352];
	bool std_filters_used[FR802X_STD_FILTERS];
	bool ext_filters_used[FR802X_EXT_FILTERS];

	/* Per Tx-FIFO-slot completion tracking, indexed by put index. */
	uint32_t tx_slot_used;
	can_tx_callback_t tx_cb[FR802X_TX_FIFO];
	void *tx_cb_arg[FR802X_TX_FIFO];

	/* Per hardware-filter-index Rx callbacks. */
	can_rx_callback_t rx_cb_std[FR802X_STD_FILTERS];
	void *rx_cb_arg_std[FR802X_STD_FILTERS];
	can_rx_callback_t rx_cb_ext[FR802X_EXT_FILTERS];
	void *rx_cb_arg_ext[FR802X_EXT_FILTERS];

	bool bus_off;
	enum can_state prev_state;

	struct k_mutex tx_mutex;
	struct k_sem tx_sem;
};

/* ---------------------------------------------------------------------------
 * Helpers
 * ------------------------------------------------------------------------- */

/* Convert a CAN FD data byte length (0..64) into the DLC code (0..15). */
static uint8_t freq_fdcan_bytes_to_dlc(uint8_t len)
{
	if (len <= 8) {
		return len;
	}
	if (len <= 12) {
		return 9;
	}
	if (len <= 16) {
		return 10;
	}
	if (len <= 20) {
		return 11;
	}
	if (len <= 24) {
		return 12;
	}
	if (len <= 32) {
		return 13;
	}
	if (len <= 48) {
		return 14;
	}
	return 15;
}

static void notify_state_change(struct can_fr802x_data *data)
{
	enum can_state state;
	struct can_bus_err_cnt err;

	if (data->common.state_change_cb == NULL) {
		return;
	}

	can_fr802x_get_state(data->dev, &state, &err);
	if (state != data->prev_state) 
	{
		data->prev_state = state;
		data->common.state_change_cb(data->dev, state, err, data->common.state_change_cb_user_data);
	}
}

/* ---------------------------------------------------------------------------
 * HAL callbacks (run in interrupt context via can_IntHandler())
 * ------------------------------------------------------------------------- */

static void rx_fifo0_cb(CAN_HandleTypeDef *hcan)
{
	struct can_fr802x_data *data = CONTAINER_OF(hcan, struct can_fr802x_data, hcan);
	const struct device *dev = data->dev;

	struct_CANRxHeaderDef_t rxh;
	struct can_frame frame;
	uint8_t buf[64];
	can_rx_callback_t cb;
	void *arg;

	while (can_get_rxfifo0_fill_level(hcan)) 
	{
		can_get_rxfifo0_message(hcan, &rxh, buf);

		memset(&frame, 0, sizeof(frame));

		frame.id = rxh.Identifier;
		if (rxh.IdType == CAN_ID_EXTENDED) {
			frame.flags |= CAN_FRAME_IDE;
		}
		if (rxh.FrameType == CAN_REMOTE_FRAM) {
			frame.flags |= CAN_FRAME_RTR;
		}
		if (rxh.FormatMode == CAN_FD_FRAME) {
			frame.flags |= CAN_FRAME_FDF;
		}
		if (rxh.BitRateSwitch) {
			frame.flags |= CAN_FRAME_BRS;
		}
		frame.dlc = rxh.DLC;
		if (rxh.FrameType == CAN_DATA_FRAME) {
			memcpy(frame.data, buf, rxh.DLC);
		}

		if (rxh.IdType == CAN_ID_EXTENDED) 
		{
			cb = data->rx_cb_ext[rxh.FilterMatchIndex];
			arg = data->rx_cb_arg_ext[rxh.FilterMatchIndex];
		} 
		else 
		{
			cb = data->rx_cb_std[rxh.FilterMatchIndex];
			arg = data->rx_cb_arg_std[rxh.FilterMatchIndex];
		}

		if (cb != NULL) {
			cb(dev, &frame, arg);
		}
	}
}

static void tx_done_cb(CAN_HandleTypeDef *hcan)
{
	struct can_fr802x_data *data = CONTAINER_OF(hcan, struct can_fr802x_data, hcan);
	int i;

	for (i = 0; i < FR802X_TX_FIFO; i++) 
	{
		if (data->tx_slot_used & (1 << i))
		{//used
			if (__CAN_GET_Tx_OCCURRED(hcan->CANx) & (1 << i))
			{//Tx_OCCURRED
				if (data->tx_cb[i] != NULL) 
				{
					can_tx_callback_t cb = data->tx_cb[i];
					void *arg = data->tx_cb_arg[i];

					data->tx_cb[i] = NULL;
					cb(data->dev, 0, arg);
				}
				data->tx_slot_used &= ~(1 << i);

				k_sem_give(&data->tx_sem);
			}
		}
	}
}

static void bus_off_cb(CAN_HandleTypeDef *hcan)
{
	struct can_fr802x_data *data = CONTAINER_OF(hcan, struct can_fr802x_data, hcan);

	data->bus_off = true;
	notify_state_change(data);
}

static void err_passive_cb(CAN_HandleTypeDef *hcan)
{
	struct can_fr802x_data *data = CONTAINER_OF(hcan, struct can_fr802x_data, hcan);

	notify_state_change(data);
}

static void warning_cb(CAN_HandleTypeDef *hcan)
{
	struct can_fr802x_data *data = CONTAINER_OF(hcan, struct can_fr802x_data, hcan);

	notify_state_change(data);
}

static void can_fr802x_isr(const void *arg)
{
	const struct device *dev = (const struct device *)arg;
	struct can_fr802x_data *data = dev->data;

	can_IntHandler(&data->hcan);
}

/* ---------------------------------------------------------------------------
 * Zephyr CAN driver API
 * ------------------------------------------------------------------------- */

static int can_fr802x_get_capabilities(const struct device *dev, can_mode_t *cap) 
{ 
	// can_mode_t modes = CAN_MODE_NORMAL | CAN_MODE_LOOPBACK | CAN_MODE_LISTENONLY;

	can_mode_t modes = CAN_MODE_NORMAL;
#ifdef CONFIG_CAN_FD_MODE
	modes |= CAN_MODE_FD;
#endif
	*cap = modes;
	return 0;
}

static int can_fr802x_start(const struct device *dev) 
{ 
	struct can_fr802x_data *data = dev->data;
	if (data->common.started) {
		return 0;
	}
	data->common.started = true;
	data->bus_off = false;
	data->prev_state = CAN_STATE_ERROR_ACTIVE;
	// notify_state_change(data);
	return 0;
}

static int can_fr802x_stop(const struct device *dev) 
{ 
	struct can_fr802x_data *data = dev->data;
	if (!data->common.started) {
		return 0;
	}
	data->common.started = false;
	return 0;
}

static int can_fr802x_set_mode(const struct device *dev, can_mode_t mode) 
{ 
	struct can_fr802x_data *data = dev->data;

	if (data->common.started) 
		return -EBUSY;

/*
	switch (mode) 
	{
		case CAN_MODE_NORMAL: can_quit_test_mode(hcan); break;
		case CAN_MODE_LOOPBACK: can_enter_test_mode(hcan, CAN_INTERNAL_LOOP_BACK_TEST_MODE); break;
		case CAN_MODE_LISTENONLY: can_enter_bus_monitoring_mode(hcan); break;
	#ifdef CONFIG_CAN_FD_MODE
		case CAN_MODE_FD: can_quit_test_mode(hcan); break;
	#endif
		default: return -ENOTSUP;
	}
*/
	data->common.mode = mode;

	return 0;
}

static int can_fr802x_set_timing(const struct device *dev, const struct can_timing *timing) 
{ 
	struct can_fr802x_data *data = dev->data;
	CAN_HandleTypeDef *hcan = &data->hcan;
	if (data->common.started) 
		return -EBUSY;

	if (timing->sjw < 1|| timing->sjw > 127 || 
		timing->prop_seg < 1   || timing->prop_seg > 255 || 
		timing->phase_seg1 < 1 || timing->phase_seg1 > 255 || 
		timing->phase_seg2 < 1 || timing->phase_seg2 > 127 || 
		timing->prescaler < 1  || timing->prescaler > 256) { return -EINVAL; }

	/* Bit timing registers may only be written while in INIT mode. */
	hcan->Init.Prescaler     = timing->prescaler;
	hcan->Init.SyncJumpWidth = timing->sjw;
	hcan->Init.TimeSeg1      = timing->phase_seg1 - 1;
	hcan->Init.TimeSeg2      = timing->phase_seg2 - 1;
	can_init(hcan);

	return 0;
}

#ifdef CONFIG_CAN_FD_MODE
static int can_fr802x_set_timing_data(const struct device *dev, const struct can_timing *timing) 
{ 
	struct can_fr802x_data *data = dev->data;
	CAN_HandleTypeDef *hcan = &data->hcan;
	if (data->common.started) 
		return -EBUSY;

	if (timing->sjw < 1 || timing->sjw > 15 || 
		timing->prop_seg < 1   || timing->prop_seg > 15 || 
		timing->phase_seg1 < 1 || timing->phase_seg1 > 31 || 
		timing->phase_seg2 < 1 || timing->phase_seg2 > 15 || 
		timing->prescaler < 1  || timing->prescaler > 32) { return -EINVAL; }
		
	hcan->Init.DataBit_Prescaler = timing->prescaler;
	hcan->Init.DataBit_SyncJumpWidth = timing->sjw;
	hcan->Init.DataBit_TimeSeg1 = timing->phase_seg1 - 1;
	hcan->Init.DataBit_TimeSeg2 = timing->phase_seg2 - 1;
	hcan->Init.DataBit_RateSwitch = 1;
	can_init(hcan);
	return 0;
}
#endif /* CONFIG_CAN_FD_MODE */

static int can_fr802x_send(const struct device *dev, const struct can_frame *frame, k_timeout_t timeout, can_tx_callback_t cb, void *user_data) 
{ 
	struct can_fr802x_data *data = dev->data;
	CAN_HandleTypeDef *hcan = &data->hcan;
	struct_CANTxHeaderDef_t CANTxHeader;
	int32_t idx;

	if (!data->common.started) return -ENETDOWN;
	if (frame == NULL || frame->dlc > CAN_MAX_DLEN) return -EINVAL;

	CANTxHeader.IdType        = (frame->flags & CAN_FRAME_IDE) ? CAN_ID_EXTENDED : CAN_ID_STANDARD;
	CANTxHeader.FrameType     = (frame->flags & CAN_FRAME_RTR) ? CAN_REMOTE_FRAM : CAN_DATA_FRAME;
	CANTxHeader.FormatMode    = (frame->flags & CAN_FRAME_FDF) ? CAN_FD_FRAME : CAN_CLASSICAL_FRAME;
	CANTxHeader.Identifier    = (frame->id);
	CANTxHeader.BitRateSwitch = (frame->flags & CAN_FRAME_BRS) ? 1 : 0;
	CANTxHeader.DLC           = (CANTxHeader.FormatMode == CAN_FD_FRAME) ? freq_fdcan_bytes_to_dlc(frame->dlc) : frame->dlc;

	k_mutex_lock(&data->tx_mutex, K_FOREVER);

	idx = can_add_tx_message(hcan, CANTxHeader, (uint8_t *)frame->data);
	if (idx < 0) {
		if (K_TIMEOUT_EQ(timeout, K_NO_WAIT)) {
			k_mutex_unlock(&data->tx_mutex);
			return -EAGAIN;
		}
		k_mutex_unlock(&data->tx_mutex);
		if (k_sem_take(&data->tx_sem, timeout) != 0) {
			k_mutex_unlock(&data->tx_mutex);
			return -EAGAIN;
		}
		k_mutex_lock(&data->tx_mutex, K_FOREVER);
		if (!data->common.started) {
			k_mutex_unlock(&data->tx_mutex);
			return -ENETDOWN;
		}
		idx = can_add_tx_message(hcan, CANTxHeader, (uint8_t *)frame->data);
		if (idx < 0)  {
			k_mutex_unlock(&data->tx_mutex);
			return -EAGAIN;
		}
	}
	data->tx_slot_used |= (1 << idx);
	data->tx_cb[idx] = cb;
	data->tx_cb_arg[idx] = user_data;

	k_mutex_unlock(&data->tx_mutex);

	return 0;
}

static int can_fr802x_add_rx_filter(const struct device *dev, can_rx_callback_t cb, void *cb_arg, const struct can_filter *filter) 
{ 
	int std_filter_index = FR802X_STD_FILTERS; 
	int ext_filter_index = FR802X_EXT_FILTERS; 
	struct can_fr802x_data *data = dev->data;
	CAN_HandleTypeDef *hcan = &data->hcan;
	struct_FilterCfg_t fcfg;

	bool ext = (filter->flags & CAN_FILTER_IDE) != 0;

	if (cb == NULL) 
		return -EINVAL;

	fcfg.FilterType  = CAN_FILTER_CLASSIC_FILTER;
	fcfg.ProcessMode = FILTER_PROCESS_STORE_IN_RxFIFO0;
	fcfg.FilterID_1  = filter->id;
	fcfg.FilterID_2  = filter->mask;
	fcfg.RxBufferIndex = 0;

	if (ext)
	{//扩展帧过滤
		for (int i = 0; i < FR802X_EXT_FILTERS; i++){
			if (data->ext_filters_used[i] == 0){
				ext_filter_index = i;
				break;
			}
		}
		if (ext_filter_index >= FR802X_EXT_FILTERS)
			return -ENOSPC;

		data->ext_filters_used[ext_filter_index] = true;

		can_add_extended_filter(hcan, fcfg, (uint32_t)ext_filter_index);
		data->rx_cb_ext[ext_filter_index] = cb;
		data->rx_cb_arg_ext[ext_filter_index] = cb_arg;
		return ext_filter_index + FR802X_STD_FILTERS;
	}
	else
	{//标准帧过滤
		for (int i = 0; i < FR802X_STD_FILTERS; i++){
			if (data->std_filters_used[i] == 0){
				std_filter_index = i;
				break;
			}
		}
		if (std_filter_index >= FR802X_STD_FILTERS)
			return -ENOSPC;

		data->std_filters_used[std_filter_index] = true;

		can_add_standard_filter(hcan, fcfg, (uint32_t)std_filter_index);
		data->rx_cb_std[std_filter_index] = cb;
		data->rx_cb_arg_std[std_filter_index] = cb_arg;
		return std_filter_index;
	}
}

static void can_fr802x_remove_rx_filter(const struct device *dev, int filter_id) 
{ 
	struct can_fr802x_data *data = dev->data;
	CAN_HandleTypeDef *hcan = &data->hcan;
	int idx;

	if (filter_id < (int)FR802X_STD_FILTERS) 
	{
		data->rx_cb_std[filter_id] = NULL; 
		data->rx_cb_arg_std[filter_id] = NULL;
		can_remove_standard_filter(hcan, (uint32_t)filter_id);

		data->std_filters_used[filter_id] = false;
	} 
	else 
	{
		idx = filter_id - (int)FR802X_STD_FILTERS;
		if (idx < (int)FR802X_EXT_FILTERS) 
		{
			data->rx_cb_ext[idx] = NULL; 
			data->rx_cb_arg_ext[idx] = NULL;
			can_remove_extended_filter(hcan, (uint32_t)idx);

			data->ext_filters_used[idx] = false;
		}
	}
}

#if defined(CONFIG_CAN_MANUAL_RECOVERY_MODE)
static int can_fr802x_recover(const struct device *dev, k_timeout_t timeout) 
{
	// Reset MACAN IP, re-init the controller, then poll until bus-off state is cleared or timeout expires.
	__SYSTEM_MCAN_RESET();
	can_fr802x_init(dev);

	return 0;
}
#endif /* CONFIG_CAN_MANUAL_RECOVERY_MODE */

static int can_fr802x_get_state(const struct device *dev, enum can_state *state, struct can_bus_err_cnt *err_cnt) 
{ 
	struct can_fr802x_data *data = dev->data;
	CAN_HandleTypeDef *hcan = &data->hcan;

	uint8_t tec = can_get_transmit_error_counter(hcan);
	uint8_t rec = can_get_receive_error_counter(hcan);

	if (err_cnt != NULL) { 
		err_cnt->tx_err_cnt = tec; 
		err_cnt->rx_err_cnt = rec; 
	}
	if (!data->common.started) { 
		*state = CAN_STATE_STOPPED; 
	}
	else if (data->bus_off) { 
		*state = CAN_STATE_BUS_OFF; 
	}
	else if (tec >= 128 || rec >= 128) { 
		*state = CAN_STATE_ERROR_PASSIVE; 
	}
	else if (tec >= 96 || rec >= 96) { 
		*state = CAN_STATE_ERROR_WARNING; 
	}
	else { 
		*state = CAN_STATE_ERROR_ACTIVE; 
	}

	return 0;
}

static void can_fr802x_set_state_change_callback(const struct device *dev, can_state_change_callback_t cb, void *user_data) 
{ 
	struct can_fr802x_data *data = dev->data;
	data->common.state_change_cb = cb;
	data->common.state_change_cb_user_data = user_data;
}

static int can_fr802x_get_core_clock(const struct device *dev, uint32_t *rate) 
{ 
	*rate = system_get_peripheral_clock(PER_CLK_CAN);
	return 0;
}

static int can_fr802x_get_max_filters(const struct device *dev, bool ide) 
{ 
	return ide ? FR802X_EXT_FILTERS : FR802X_STD_FILTERS;
}

/* ---------------------------------------------------------------------------
 * Initialisation
 * ------------------------------------------------------------------------- */

static int can_fr802x_init(const struct device *dev)
{
	const struct can_fr802x_config *cfg = dev->config;
	struct can_fr802x_data *data = dev->data;
	CAN_HandleTypeDef *hcan = &data->hcan;
	int err, ret;

	data->dev = (struct device *)dev;

	/* 1. Enable and reset the MACAN peripheral clock. */
    __SYSTEM_MCAN_CLK_ENABLE();
    __SYSTEM_MCAN_CLK_SELECT_24M();

	/* 2. Apply pinctrl (TX/RX pins). */
	err = pinctrl_apply_state(cfg->pcfg, PINCTRL_STATE_DEFAULT);
	if (err < 0) {
		printk("can io err\n");
		return err;
	}

	hcan->CANx = (struct_CAN_t *)cfg->base;

	/* 3. Calculate bit timing from device tree config. */
	struct can_timing timing;
	ret = can_calc_timing(dev, &timing, cfg->common.bitrate, cfg->common.sample_point);
	if (ret < 0) {
		printk("can_calc_timing fail ret=%d\n", ret);
		return ret;
	}
	hcan->Init.Prescaler     = timing.prescaler;
	hcan->Init.SyncJumpWidth = timing.sjw;
	hcan->Init.TimeSeg1      = timing.prop_seg + timing.phase_seg1 - 1;
	hcan->Init.TimeSeg2      = timing.phase_seg2 - 1;
	hcan->Init.DataBit_RateSwitch = 1;
#ifdef CONFIG_CAN_FD_MODE
	ret = can_calc_timing_data(dev, &timing, DT_INST_PROP_OR(0, bitrate_data, 0),DT_INST_PROP_OR(0, sample_point_data, 0));
	if (ret < 0) {
		printk("can_calc_timing_data fail ret=%d\n", ret);
		return ret;
	}
	hcan->Init.DataBit_Prescaler = timing.prescaler;
	hcan->Init.DataBit_SyncJumpWidth = timing.sjw;
	hcan->Init.DataBit_TimeSeg1 = timing.prop_seg + timing.phase_seg1 - 1;
	hcan->Init.DataBit_TimeSeg2 = timing.phase_seg2 - 1;
	hcan->Init.DataBit_RateSwitch = 1;
#endif
    /* CAN init */
	can_init(hcan);
	/* Configure the message RAM layout. */
	hcan->RAMConfig.StartAddress = (uint32_t)data->CanBuffer;
	hcan->RAMConfig.StandardIDFilterNums = FR802X_STD_FILTERS;
	hcan->RAMConfig.ExtendedIDFilterNums = FR802X_EXT_FILTERS;
	hcan->RAMConfig.TxFIFOQueueNums = FR802X_TX_FIFO;
	hcan->RAMConfig.TxDedicatedBufferNums = 0;
	hcan->RAMConfig.RxFIFO0Nums = FR802X_RX_FIFO0;
	hcan->RAMConfig.RxFIFO1Nums = FR802X_RX_FIFO0;
#ifdef CAN_TX_EVENT_FUNCTION_EXIST
	hcan->RAMConfig.TxEventNums = 0;
#endif
#ifdef CONFIG_CAN_FD_MODE
	hcan->RAMConfig.DataBufferSize = CAN_DATA_BUFFER_SIZE_64_BYTE;
#else
	hcan->RAMConfig.DataBufferSize = CAN_DATA_BUFFER_SIZE_8_BYTE;
#endif
    uint32_t ram_size = can_message_ram_init(hcan);
    printk("can use ram size %d \n", ram_size);

	/* Install HAL interrupt callbacks. */
	hcan->RxFIFO0_New_Message_Callback    = rx_fifo0_cb;
	hcan->Transmission_Completed_Callback = tx_done_cb;
	hcan->Bus_Off_Callback = bus_off_cb;
	hcan->Warning_Callback = warning_cb;
	hcan->Error_Passive_Callback = err_passive_cb;

	/* Synchronisation primitives. */
	k_mutex_init(&data->tx_mutex);
	k_sem_init(&data->tx_sem, 0, FR802X_TX_FIFO);

	for (int i = 0; i < FR802X_STD_FILTERS; i++)
		data->std_filters_used[i] = 0;
	for (int i = 0; i < FR802X_EXT_FILTERS; i++)
		data->ext_filters_used[i] = 0;

	/* Route interrupts to line 0 and enable them. */
    can_int_select_line(hcan, INT_RxFIFO0_NEW_MESSAGE, CAN_INT_LINE0);
    can_int_select_line(hcan, INT_TRANSMISSION_COMPLETED, CAN_INT_LINE0);
	can_int_select_line(hcan, INT_BUS_OFF_STATUS, CAN_INT_LINE0);
    can_int_enable(hcan, INT_RxFIFO0_NEW_MESSAGE);
    can_int_enable(hcan, INT_TRANSMISSION_COMPLETED);
	can_int_enable(hcan, INT_BUS_OFF_STATUS);

	/* Connect and enable the interrupt lines (dynamic, FR802x port). */
	irq_connect_dynamic(cfg->irq0, cfg->irq0_prio, can_fr802x_isr, (void *)dev, 0);
	irq_enable(cfg->irq0);

	data->common.started = false;
	data->bus_off        = false;
	data->prev_state = CAN_STATE_STOPPED;

	return 0;
}

/* ---------------------------------------------------------------------------
 * Driver API
 * ------------------------------------------------------------------------- */

static const struct can_driver_api can_fr802x_driver_api = {
	.get_capabilities = can_fr802x_get_capabilities,
	.start = can_fr802x_start,
	.stop = can_fr802x_stop,
	.set_mode = can_fr802x_set_mode,
	.set_timing = can_fr802x_set_timing,
	.send = can_fr802x_send,
	.add_rx_filter = can_fr802x_add_rx_filter,
	.remove_rx_filter = can_fr802x_remove_rx_filter,
#if defined(CONFIG_CAN_MANUAL_RECOVERY_MODE)
	.recover = can_fr802x_recover,
#endif
	.get_state = can_fr802x_get_state,
	.set_state_change_callback = can_fr802x_set_state_change_callback,
	.get_core_clock = can_fr802x_get_core_clock,
	.get_max_filters = can_fr802x_get_max_filters,
	.timing_min = {
		.sjw = 1,
		.prop_seg = 1,
		.phase_seg1 = 1,
		.phase_seg2 = 1,
		.prescaler = 1,
	},
	.timing_max = {
		.sjw = 127,
		.prop_seg = 255,
		.phase_seg1 = 255,
		.phase_seg2 = 127,
		.prescaler = 256,
	},
#ifdef CONFIG_CAN_FD_MODE
	.set_timing_data = can_fr802x_set_timing_data,
	.timing_data_min = {
		.sjw = 1,
		.prop_seg = 1,
		.phase_seg1 = 1,
		.phase_seg2 = 1,
		.prescaler = 1,
	},
	.timing_data_max = {
		.sjw = 15,
		.prop_seg = 15,
		.phase_seg1 = 31,
		.phase_seg2 = 15,
		.prescaler = 32,
	},
#endif /* CONFIG_CAN_FD_MODE */
};

/* ---------------------------------------------------------------------------
 * Instance definition
 * ------------------------------------------------------------------------- */

#define CAN_FREQCHIP_INIT(inst)						                \
	PINCTRL_DT_INST_DEFINE(inst);					                \
	static const struct can_fr802x_config can_fr802x_cfg_##inst = {	\
		.common = CAN_DT_DRIVER_CONFIG_GET(DT_DRV_INST(inst) ,0, DT_CAN_TRANSCEIVER_MAX_BITRATE(DT_DRV_INST(inst), 1000000)), \
		.base = DT_INST_REG_ADDR(inst),			                    \
		.pcfg = PINCTRL_DT_INST_DEV_CONFIG_GET(inst),		        \
		.irq0 = DT_INST_IRQN_BY_IDX(inst, 0),			            \
		.irq1 = DT_INST_IRQN_BY_IDX(inst, 1),			            \
		.irq0_prio = DT_INST_IRQ_BY_IDX(inst, 0, priority),	        \
		.irq1_prio = DT_INST_IRQ_BY_IDX(inst, 1, priority),	        \
	};											                    \
	static struct can_fr802x_data can_fr802x_data_##inst;		    \
	DEVICE_DT_INST_DEFINE(inst, can_fr802x_init, NULL,		        \
			              &can_fr802x_data_##inst,			        \
			              &can_fr802x_cfg_##inst,			        \
			              POST_KERNEL, CONFIG_CAN_INIT_PRIORITY,    \
			              &can_fr802x_driver_api)

DT_INST_FOREACH_STATUS_OKAY(CAN_FREQCHIP_INIT)
