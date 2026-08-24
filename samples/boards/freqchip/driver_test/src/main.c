/*
 * Copyright (c) 2012-2014 Wind River Systems, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */


#include <stdint.h>
#include <stdio.h>

#include <zephyr/kernel.h>
#include "zephyr/pm/pm.h"
#include <zephyr/irq.h>
#include <zephyr/drivers/adc.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/led.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/drivers/can.h>
#include <zephyr/drivers/watchdog.h>
#include <zephyr/shell/shell.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/gap.h>

//#include <fr802x.h>
//#include <system_fr802x.h>
#include <driver_usart.h>
#include <driver_cali.h>
//#include <driver_saradc.h>

/* Zephyr 内核函数，无公共头文件，需要手动声明 */
extern void __stdout_hook_install(int (*hook)(int c));

/*
 * stdout hook - 等价于 KEIL 中重定义 fputc
 * picolibc 的 printf 会通过 stdout -> zephyr_fputc -> _stdout_hook 最终调到这里
 */
static int uart_putchar(int c)
{
	while(!(__USART_GET_SR_STATUS(USART2, USART_STATUS_TXFIFO_NOFULL)));
	USART2->DR = c;
	return c;
}

//gpioa 是 GPIO 控制器节点 → 用 DEVICE_DT_GET 直接获取设备指针
static const struct device *gpioa_dev  = DEVICE_DT_GET(DT_NODELABEL(gpioa));
static const struct device *gpiob_dev  = DEVICE_DT_GET(DT_NODELABEL(gpiob));
static const struct device *gpioc_dev  = DEVICE_DT_GET(DT_NODELABEL(gpioc));
static const struct device *led_dev    = DEVICE_DT_GET(DT_NODELABEL(leds));
static const struct device *usart0_dev = DEVICE_DT_GET(DT_NODELABEL(usart0));
static const struct device *usart1_dev = DEVICE_DT_GET(DT_NODELABEL(usart1));
static const struct device *usart2_dev = DEVICE_DT_GET(DT_NODELABEL(usart2));
static const struct device *adc        = DEVICE_DT_GET(DT_NODELABEL(adc));
static const struct device *i2c0       = DEVICE_DT_GET(DT_NODELABEL(i2c0));
static const struct device *can_dev    = DEVICE_DT_GET(DT_NODELABEL(can0));
static const struct device *wdt        = DEVICE_DT_GET(DT_NODELABEL(wdt));
// 获取子节点索引
#define LED_RED_IDX DT_NODE_CHILD_IDX(DT_NODELABEL(red_led))

static struct gpio_callback button_cb_data;
void button_pressed_callback(const struct device *dev, struct gpio_callback *cb, gpio_port_pins_t pins)
{
	// printf("button pressed\n");
}

void PMU_IRQHandler(void);
void PMU_IRQHandler_dynamic(const void *dev)
{
    PMU_IRQHandler();
}

static void on_state_entry(enum pm_state state)
{
    // if (state == PM_STATE_SUSPEND_TO_RAM)
    // {
    //     USART2->DR = 's';
    //     system_delay_us(10);

    //     // gpio_status_latch_enable();
    // }
}
static void on_state_exit(enum pm_state state)
{
    // gpio_status_latch_disable();
    if (state == PM_STATE_SUSPEND_TO_RAM)
    {
        // __SYSTEM_USART2_CLK_ENABLE();
        // *(uint32_t *)0xE0052080 = 0x1100;
        // uint32_t *P = (uint32_t *)UART2_BASE;
        // P[1] = 0x03;
        // P[2] = 0x02;
        // P[3] = 0x1C;
        // P[4] = 0x01;
        // P[5] = 0x0C;
        // P[9] = 0x0101;
        // // gpio_status_latch_disable();

        // USART2->DR = 'w';
        // system_delay_us(10);
    }
}
static struct pm_notifier app_pm_notifier = {
	.state_entry = on_state_entry,
	.state_exit  = on_state_exit,
};

void can_tx_cb(const struct device *dev, int error, void *user_data)
{
	printf("TX done, error=%d\n", error);
}
void can_rx_cb(const struct device *dev, struct can_frame *frame, void *user_data)
{
	printf("RX: ID=0x%X DLC=%d data=", frame->id, frame->dlc);
	for (int i = 0; i < frame->dlc; i++) printf("%02X ", frame->data[i]);
	printf("\n");
}

/* LED 闪烁: 定时器回调直接翻转 */
static void led_timer_cb(struct k_timer *timer)
{
	static bool led_state;
GPIOC->GPIO_TOG = 1 << 14;
GPIOC->GPIO_TOG = 1 << 14;
	led_state = !led_state;
	if (led_state) {
		led_on(led_dev, LED_RED_IDX);
	} else {
		led_off(led_dev, LED_RED_IDX);
	}
}

extern struct __USART_HandleTypeDef hci_handle;
extern void freq_controller_task(void);

static void bluetooth_adv_demo(void);

/* UART1 接收回调: 中断驱动，每收一个字节触发 */
static void uart1_rx_cb(const struct device *dev, void *user_data)
{
	// usart_IntHandler(&hci_handle);
}

uint32_t GPIOC_DATA;

void user_entry_before_sleep(void)
{
    USART2->DR = 's';
    system_delay_us(10);
    // gpio_status_latch_enable();
    // GPIOC_DATA = *(uint32_t *)0x50210008;
}
void user_entry_after_sleep_device_ready(void)
{
    // __SYSTEM_USART2_CLK_ENABLE();
    // // __SYSTEM_GPIOC_CLK_ENABLE();
    // // *(uint32_t *)0x50210008 = GPIOC_DATA | 0x00000001;
    // // *(uint32_t *)0x50210014 = 1;
    // // *(uint32_t *)0x50210000 &= ~((1 << 13) | 1);


    // *(uint32_t *)0xE0052080 = 0x1100;
    // uint32_t *P = (uint32_t *)UART2_BASE;
    // P[1] = 0x03;
    // P[2] = 0x02;
    // P[3] = 0x1C;
    // P[4] = 0x01;
    // P[5] = 0x0C;
    // P[9] = 0x0101;

    // // gpio_status_latch_disable();
    // USART2->DR = 'w';
    // system_delay_us(10);

    // __SYSTEM_USART2_CLK_ENABLE();
    // *(uint32_t *)0xE0052080 = 0x1100;
    // uint32_t *P = (uint32_t *)UART2_BASE;
    // P[1] = 0x03;
    // P[2] = 0x02;
    // P[3] = 0x1C;
    // P[4] = 0x01;
    // P[5] = 0x0C;
    // P[9] = 0x0101;
    // gpio_status_latch_disable();

    USART2->DR = 'w';
    system_delay_us(10);
}

int main(void)
{
	__stdout_hook_install(uart_putchar);
    
    pm_notifier_register(&app_pm_notifier);

	printf("\n");
	printf("Driver Test! %s\n", CONFIG_BOARD_TARGET);

    if (!device_is_ready(gpioa_dev)){
		printf("Error: gpioa_dev device not ready\n");
	}
	else{
		printf("gpioa_dev ready!\n");
	}
    if (!device_is_ready(gpiob_dev)){
		printf("Error: gpiob_dev device not ready\n");
	}
	else{
		printf("gpiob_dev ready!\n");
	}
    if (!device_is_ready(gpioc_dev)){
		printf("Error: gpioc_dev device not ready\n");
	}
	else{
		printf("gpioc_dev ready!\n");
	}

	if (!device_is_ready(adc)){
		printf("Error: adc device not ready\n");
	}
	else{
		printf("ADC ready!\n");
	}

	if (!device_is_ready(i2c0)){
		printf("Error: i2c0 device not ready\n");
	}
	else{
		printf("I2C0 ready!\n");
	}

	/* UART1 中断接收 */
	if (device_is_ready(usart1_dev)) {
		uart_irq_callback_set(usart1_dev, uart1_rx_cb);
		///uart_irq_rx_enable(usart1_dev);
		printf("UART1 RX callback registered\n");
	}

    /* do calibration, get current RC frequency */
    __SYSTEM_CALI_CLK_ENABLE();
    __SYSTEM_CALI_SRC_SEL(CALI_SRC_CLK_SEL_LPRC);
    CALI_HandleTypeDef cali_handle;
    cali_handle.mode = CALI_UP_MODE_NORMAL;
    cali_handle.rc_cnt = 300;
    cali_init(&cali_handle);
    system_set_LPRCCLK(cali_calc_rc_freq(&cali_handle, cali_start(&cali_handle)));
    __SYSTEM_CALI_CLK_DISABLE();

	struct k_timer led_timer;
	k_timer_init(&led_timer, led_timer_cb, NULL);
	k_timer_start(&led_timer, K_MSEC(200), K_MSEC(200));

    gpio_pin_configure(gpioc_dev, 14, GPIO_OUTPUT);

    void gpio_demo(void);
    gpio_demo();
	/* ====== BLE 广播例程 ====== */
	bluetooth_adv_demo();

    irq_connect_dynamic(PMU_IRQn, 2, PMU_IRQHandler_dynamic, NULL, 0);
    irq_enable(PMU_IRQn);

    system_sleep_enable();
    // system_sleep_disable();

    uint32_t count = 0;
    printf("NVIC:%d, count=%d\n", NVIC_GetPriority(GPIOA_IRQn), count);
	while (1)
	{
        count++;
        if (count == 60)
        {
            system_sleep_disable();
            printf("system_sleep_disable");
        }
		k_sleep(K_MSEC(1000));
	}

	return 0;
}

/*
 * BLE 广播数据（文件级静态，确保编译期常量）
 */
static const char adv_name[] = "Zephyr_BLE_ADV";

static const struct bt_data ad[] = {
	BT_DATA_BYTES(BT_DATA_FLAGS, BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR),
	BT_DATA(BT_DATA_NAME_COMPLETE, adv_name, sizeof(adv_name) - 1),
};

static const struct bt_le_adv_param adv_param = {
	.options = BT_LE_ADV_OPT_CONNECTABLE | BT_LE_ADV_OPT_USE_IDENTITY,
	.interval_min = BT_GAP_ADV_FAST_INT_MIN_2,
	.interval_max = BT_GAP_ADV_FAST_INT_MAX_2,
};

/*
 * BLE 广播例程：使能 Host，启动可连接广播
 */
static void bluetooth_adv_demo(void)
{
	int err;

	err = bt_enable(NULL);
	if (err) {
		printf("bt_enable failed: %d\n", err);
		return;
	}
	printf("Bluetooth enabled\n");

	err = bt_le_adv_start(&adv_param, ad, ARRAY_SIZE(ad), NULL, 0);
	if (err) {
		printf("Advertising failed: %d\n", err);
		return;
	}

	printf("Advertising started\n");
}

void gpio_demo(void)
{
	gpio_pin_configure(gpioa_dev, 0, GPIO_INPUT | GPIO_PULL_UP);
	gpio_pin_interrupt_configure(gpioa_dev, 0, GPIO_INT_LEVEL_LOW);
    
	gpio_init_callback(&button_cb_data, button_pressed_callback, BIT(0));
	gpio_add_callback(gpioa_dev, &button_cb_data);
}

void adc_demo(void)
{
	uint16_t adc_buf[2];

	struct adc_sequence seq = {
		.channels    = BIT(0) | BIT(1),
		.buffer      = adc_buf,
		.buffer_size = sizeof(adc_buf),
		.resolution  = 12,
	};


	int ret = adc_read(adc, &seq);
	if (ret == 0) {
		printf("ADC: PC0=%u, 1/4Vbat=%u\n", adc_buf[0], adc_buf[1]);
	} else {
		printf("ADC read error: %d\n", ret);
	}
}

void wdt_demo(void)
{
	// 2. 配置超时时间
    struct wdt_timeout_cfg cfg = {
        .window.min = 0,              // 窗口看门狗最小值（0=普通看门狗）
        .window.max = 5000,           // 超时时间 1000ms
        .callback = NULL,      // 超时回调（可为 NULL）
        .flags = WDT_FLAG_RESET_SOC,  // 超时后复位整个系统
    };

    // 3. 安装超时配置（启动看门狗）
    int channel = wdt_install_timeout(wdt, &cfg);
}

void i2c_demo(void)
{
	int ret;

	/* I2C0: 先发 2 字节(寄存器地址)，再 RESTART + 读 10 字节 */
	uint8_t tx_buf[20] = {0};
	uint8_t rx_buf[10] = {0};

	struct i2c_msg msgs[2];

	tx_buf[0] = 0x00;
	tx_buf[1] = 0x00;
	for (int i = 0; i < 10; i++)
	{
		tx_buf[i + 2] = i;
	}
	msgs[0].buf   = tx_buf;
	msgs[0].len   = 12;
	msgs[0].flags = I2C_MSG_WRITE|I2C_MSG_STOP;
	ret = i2c_transfer(i2c0, msgs, 1, 0xA0);
	if (ret >= 0) {
		printf("write success: %d\n", ret);
	} else {
		printf("write error: %d\n", ret);
	}
	system_delay_ms(500);

	msgs[0].buf   = tx_buf;
	msgs[0].len   = 2;
	msgs[0].flags = I2C_MSG_WRITE;                                /* 写，不发 STOP */

	msgs[1].buf   = rx_buf;
	msgs[1].len   = 10;
	msgs[1].flags = I2C_MSG_RESTART | I2C_MSG_READ | I2C_MSG_STOP; /* RESTART + 读 + STOP */

	ret = i2c_transfer(i2c0, msgs, 2, 0xA0);
	if (ret >= 0) {
		for (int i = 0; i < 10; i++)
			printf("I2C RX: %02x\n", rx_buf[i]);
	} else {
		printf("I2C transfer error: %d\n", ret);
	}
}

void can_demo(void)
{
	can_start(can_dev);

	struct can_filter rx_filter1 = {
		.flags = 0,           // 标准帧
		.id    = 0x111,       // 匹配 ID
		.mask  = 0x7FF,       // 全匹配
	};
	int ret = can_add_rx_filter(can_dev, can_rx_cb, NULL, &rx_filter1);
	printf("rx_filter ret:%d\n", ret);

	struct can_filter rx_filter2 = {
		.flags = 0,           // 标准帧
		.id    = 0x222,       // 匹配 ID
		.mask  = 0x7FF,       // 全匹配
	};
	ret = can_add_rx_filter(can_dev, can_rx_cb, NULL, &rx_filter2);
	printf("rx_filter ret:%d\n", ret);

	struct can_frame frame = {0};
	frame.id = 0x123;
	frame.dlc = 8;
	frame.flags = 0;
	frame.data[0] = 0x11;
	frame.data[1] = 0x22;
	frame.data[2] = 0x33;
	frame.data[3] = 0x44;
	frame.data[4] = 0x55;
	frame.data[5] = 0x66;
	frame.data[6] = 0x77;
	frame.data[7] = 0x88;
	ret = can_send(can_dev, &frame, K_MSEC(100), can_tx_cb, NULL);
	printf("can_send ret:%d\n", ret);
}

