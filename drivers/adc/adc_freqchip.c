/*
 * Copyright (c) 2026 Freqchip
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/*
 * SARADC channel mapping reference (enum_saradc_Channel_Map_t):
 *
 *   ADC_CH_MAP_PORTB_7  =  2    PortB_7
 *   ADC_CH_MAP_PORTB_8  =  3    PortB_8
 *   ADC_CH_MAP_PORTB_9  =  4    PortB_9
 *   ADC_CH_MAP_PORTB_10 =  5    PortB_10
 *   ADC_CH_MAP_PORTB_11 =  6    PortB_11
 *   ADC_CH_MAP_PORTB_12 =  7    PortB_12
 *   ADC_CH_MAP_VBATN    =  8    Vbat negative
 *   ADC_CH_MAP_REFINT   =  9    ADC reference voltage
 *   ADC_CH_MAP_VBEN     = 10    Core temperature negative
 *   ADC_CH_MAP_VBEP     = 11    Core temperature positive
 *   ADC_CH_MAP_PGA_OUTN = 12    PGA out negative
 *   ADC_CH_MAP_PGA_OUTP = 13    PGA out positive
 *   ADC_CH_MAP_PORTB_14 = 16    PortB_14
 *   ADC_CH_MAP_PORTB_15 = 17    PortB_15
 *   ADC_CH_MAP_PORTC_0  = 18    PortC_0
 *   ADC_CH_MAP_PORTC_1  = 19    PortC_1
 *   ADC_CH_MAP_PORTC_2  = 20    PortC_2
 *   ADC_CH_MAP_PORTC_3  = 21    PortC_3
 *   ADC_CH_MAP_PORTC_4  = 22    PortC_4
 *   ADC_CH_MAP_PORTC_5  = 23    PortC_5
 *   ADC_CH_MAP_PMU_TEST = 30    PMU test
 *   ADC_CH_MAP_VBATP    = 31    Vbat positive
 */

#define DT_DRV_COMPAT freqchip_freqchip_saradc

#include <errno.h>

#include <zephyr/drivers/adc.h>
#include <zephyr/devicetree.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include <driver_saradc.h>

#define ADC_CONTEXT_USES_KERNEL_TIMER
#include "adc_context.h"

LOG_MODULE_REGISTER(adc_freqchip, CONFIG_ADC_LOG_LEVEL);

#define ADC_FREQCHIP_MAX_CHANNELS	32
#define ADC_FREQCHIP_RESOLUTION		12
#define ADC_FREQCHIP_REF_INTERNAL_MV	1200

/* Polling timeout per channel (us) */
#define ADC_FREQCHIP_CHANNEL_TIMEOUT_US	100000

struct adc_freqchip_config {
	saradc_InitConfig_t InitConfig;
	saradc_LoopConfig_t LoopConfig;
};

struct adc_freqchip_data {
	struct adc_context ctx;
	saradc_ChannelConfig_t ch_cfg[ADC_FREQCHIP_MAX_CHANNELS];
};

static void adc_context_start_sampling(struct adc_context *ctx)
{
	/* Start loop conversion, data read happens in adc_freqchip_read() */
	saradc_loop_convert_start();
}

static void adc_context_update_buffer_pointer(struct adc_context *ctx,
					      bool repeat_sampling)
{
	/* Not used in polling mode */
}

static int adc_freqchip_channel_setup(const struct device *dev, const struct adc_channel_cfg *chan_cfg)
{
	/* Channel initialization is already done in adc_freqchip_init()
	 * from DTS child nodes. This function is provided for runtime
	 * reconfiguration only and currently serves as a placeholder.
	 */

	return 0;
}

static int adc_freqchip_read(const struct device *dev, const struct adc_sequence *sequence)
{
	struct adc_freqchip_data *data = dev->data;
	uint16_t *adc_data;
	uint32_t channels;
	int error = 0;

	if (sequence->resolution != ADC_FREQCHIP_RESOLUTION) {
		LOG_ERR("Only %u-bit resolution supported", ADC_FREQCHIP_RESOLUTION);
		return -ENOTSUP;
	}

	if (sequence->channels == 0) {
		LOG_ERR("No channels selected");
		return -EINVAL;
	}

	/*
	 * Acquire the ADC mutex to serialize hardware access.
	 * Parameters: ctx, asynchronous=false (synchronous mode),
	 *             signal=NULL (no async notification needed).
	 */
	adc_context_lock(&data->ctx, false, NULL);

	/* Start loop conversion */
	saradc_loop_convert_start();

	adc_data = sequence->buffer;
	channels = sequence->channels;

	/* Poll for each channel's data */
	for (uint8_t ch = 0; ch < ADC_FREQCHIP_MAX_CHANNELS; ch++) 
	{
		if (channels & BIT(ch)) 
		{
			uint32_t timeout = ADC_FREQCHIP_CHANNEL_TIMEOUT_US;

			while (!saradc_get_channel_valid_status((enum_saradc_channel_t)ch)) 
			{
				if (--timeout == 0) 
				{
					LOG_ERR("Channel %u conversion timeout", ch);
					error = -EIO;
				}
				system_delay_us(1);
			}

			adc_data[ch] = saradc_get_channel_data((enum_saradc_channel_t)ch);
		}
	}

	adc_context_release(&data->ctx, error);

	return error;
}
#ifdef CONFIG_ADC_ASYNC
static int adc_freqchip_read_async(const struct device *dev,
				    const struct adc_sequence *sequence,
				    struct k_poll_signal *async)
{
	struct adc_freqchip_data *data = dev->data;
	int error;

	adc_context_lock(&data->ctx, true, async);
	error = adc_freqchip_start_read(dev, sequence);
	adc_context_release(&data->ctx, error);

	return error;
}
#endif /* CONFIG_ADC_ASYNC */

static const struct adc_driver_api adc_freqchip_driver_api = {
	.channel_setup = adc_freqchip_channel_setup,
	.read = adc_freqchip_read,
#ifdef CONFIG_ADC_ASYNC
	.read_async = adc_gd32_read_async,
#endif /* CONFIG_ADC_ASYNC */
	.ref_internal = ADC_FREQCHIP_REF_INTERNAL_MV,
};

static int adc_freqchip_init(const struct device *dev)
{
	const struct adc_freqchip_config *cfg = dev->config;
	struct adc_freqchip_data *data = dev->data;

	/* ADC Power on */
    pmu_saradc_power_ctrl(true);
    /* ADC clock enable */
    __SYSTEM_SARADC_CLK_ENABLE();
    __SYSTEM_ASP_CLK_ENABLE();

	struct adc_freqchip_config adc_config;
	adc_config.InitConfig.saradc_reference      = cfg->InitConfig.saradc_reference;
	adc_config.InitConfig.saradc_sampling_cycle = cfg->InitConfig.saradc_sampling_cycle;
	adc_config.InitConfig.saradc_clock_div      = cfg->InitConfig.saradc_clock_div; 
	adc_config.InitConfig.saradc_interval_div   = 1; 
	adc_config.InitConfig.saradc_mode           = cfg->InitConfig.saradc_mode;
	saradc_init(&adc_config.InitConfig);

	adc_config.LoopConfig.loop_max_channel = cfg->LoopConfig.loop_max_channel;
	adc_config.LoopConfig.loop_ch_interval = cfg->LoopConfig.loop_ch_interval;
	adc_config.LoopConfig.loop_FIFO_enable = cfg->LoopConfig.loop_FIFO_enable;
	adc_config.LoopConfig.loop_FIFO_channel_sel = cfg->LoopConfig.loop_FIFO_channel_sel;
	adc_config.LoopConfig.loop_FIFO_almost_threshold = cfg->LoopConfig.loop_FIFO_almost_threshold;            
	saradc_loop_config(&adc_config.LoopConfig);

	saradc_ChannelConfig_t ChannelConfig;
	for (int i = 0; i < adc_config.LoopConfig.loop_max_channel; i++)
	{
		ChannelConfig.ch_mode  = data->ch_cfg[i].ch_mode;
		ChannelConfig.ch_map_p = data->ch_cfg[i].ch_map_p;
		ChannelConfig.ch_map_n = data->ch_cfg[i].ch_map_n;            
		ChannelConfig.ch_voltage_divider = data->ch_cfg[i].ch_voltage_divider;
		ChannelConfig.ch_source_follower_en = data->ch_cfg[i].ch_source_follower_en;

		saradc_channel_config(i, &ChannelConfig);

		if (ChannelConfig.ch_map_p == ADC_CH_MAP_VBEP){
			saradc_vbe_measure_enable();
		}
		else if (ChannelConfig.ch_map_p == ADC_CH_MAP_VBATP){
			saradc_vbat_measure_enable(SARADC_VABT_DIVIDER_0P25);
		}
	}

	adc_context_unlock_unconditionally(&data->ctx);

	return 0;
}

/* 从 DTS 通道子节点生成指定初始化器 */
#define FREQCHIP_CH_INIT(node_id)                                                                                       \
	[DT_PROP_OR(node_id, reg, 0)] = {                                                                                   \
		.ch_mode = DT_PROP(node_id, zephyr_differential) ?  SARADC_CH_MODE_DIFFERENTIAL : SARADC_CH_MODE_SINGLE,        \
		.ch_map_p = DT_PROP_OR(node_id, zephyr_input_positive, 0),                                                      \
		.ch_map_n = DT_PROP_OR(node_id, zephyr_input_negative, 0),                                                      \
		.ch_voltage_divider = DT_PROP_OR(node_id, freqchip_voltage_divider, SARADC_VOLTAGE_DIVIDER_BYPASS),             \
		.ch_source_follower_en = DT_PROP(node_id, freqchip_source_follower) ? SARADC_FUNC_ENABLE : SARADC_FUNC_DISABLE, \
	},

#define ADC_FREQCHIP_INIT(n)						                             \
	static struct adc_freqchip_data adc_freqchip_data_##n = {	                 \
		ADC_CONTEXT_INIT_TIMER(adc_freqchip_data_##n, ctx),                      \
		ADC_CONTEXT_INIT_LOCK(adc_freqchip_data_##n, ctx),                       \
		ADC_CONTEXT_INIT_SYNC(adc_freqchip_data_##n, ctx),                       \
		.ch_cfg = {                                                              \
			DT_INST_FOREACH_CHILD(n, FREQCHIP_CH_INIT)                           \
		},                                                                       \
	};								                                             \
	static const struct adc_freqchip_config adc_freqchip_config_##n = {          \
		.InitConfig = {                                                          \
			.saradc_mode = SARADC_LOOP_MODE,                                     \
			.saradc_reference = DT_INST_PROP_OR(n, reference, 0),                \
			.saradc_clock_div = DT_INST_PROP_OR(n, clk_div, 12),                 \
			.saradc_sampling_cycle = DT_INST_PROP_OR(n, sampling_cycle, 16),     \
			.saradc_interval_div = 1,                                            \
		},                                                                       \
		.LoopConfig = {                                                          \
			.loop_max_channel = DT_INST_PROP_OR(n, channels, 0),                 \
			.loop_ch_interval = 1,                                               \
			.loop_FIFO_enable = 0,                                               \
			.loop_FIFO_channel_sel = 0,                                          \
			.loop_FIFO_almost_threshold = 0,                                     \
		},                                                                       \
	};                                                                           \
	DEVICE_DT_INST_DEFINE(n,					                                 \
			              &adc_freqchip_init, NULL,			                     \
			              &adc_freqchip_data_##n, &adc_freqchip_config_##n,      \
			              POST_KERNEL, CONFIG_ADC_INIT_PRIORITY,	             \
			              &adc_freqchip_driver_api);

DT_INST_FOREACH_STATUS_OKAY(ADC_FREQCHIP_INIT)
