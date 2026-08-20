/*
 * Copyright (c) 2026 Freqchip
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include <zephyr/drivers/timer/system_timer.h>
#include "system_fr802x.h"
#include "zephyr/kernel_structs.h"
#include "zephyr/sys_clock.h"
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <zephyr/pm/pm.h>
#include <zephyr/irq.h>
#include <cmsis_core.h>
#include <soc.h>

#include <driver_usart.h>

#define UART_PUT_CHAR(c)     USART2->DR = c
#define MINUS_DEEP_SLEEP_TIME                   3

#define PM_NVIC_SHPR3_REG                    ( *( ( volatile uint32_t * ) 0xe000ed20 ) )

uint8_t calc_sleep_time_controller(int32_t *sleep_duration, uint32_t *base, uint16_t *fine);
/* base: BIT0~27 is valid, unit is 312.5us; fine: 0~624, unit is 0.5us */
void bb_counter_get(uint32_t *base, uint16_t *fine);
void low_power_save(void);
void low_power_restore(void);
void bb_wakeup_start(void);
bool bb_wakeup_is_ongoing(void);

__RAM_CODE __WEAK bool user_deep_sleep_check(void)
{
    return true;
}

__RAM_CODE __WEAK void user_entry_before_sleep(void)
{
}

__RAM_CODE __WEAK void user_entry_after_sleep(void)
{
}

__RAM_CODE __WEAK void user_entry_after_sleep_user(void)
{
}

static uint32_t event_notify_counter = 0;
static void (*event_notify_record)(void) = NULL;
extern void (*ke_event_notify)(void);
__RAM_CODE static void event_notify_slince(void)
{
    event_notify_counter++;
}
uint32_t data[10];

static uint32_t start_basecnt;
static uint16_t start_finecnt;
static uint32_t end_basecnt;
static uint16_t end_finecnt;
static char     deep_sleep = false;
/*
 * Zephyr 电源管理入口。该函数在 idle 线程中由 PM 子系统调用，
 * 调用时中断已关闭。必须在 RAM 中执行，因为深度睡眠时 Flash 可能被断电。
 *
 * 支持的模式:
 *   PM_STATE_ACTIVE          - 正常运行，不做任何操作
 *   PM_STATE_RUNTIME_IDLE    - CPU WFI 休眠，外设保持运行
 *   PM_STATE_SUSPEND_TO_IDLE - 同上，PMU 管理 CPU 时钟门控
 *   PM_STATE_SUSPEND_TO_RAM  - 深度睡眠，SRAM0/1 保持供电，Flash 断电,
 *                              唤醒后从 WAKEUP 向量恢复
 */
__RAM_CODE void pm_state_set(enum pm_state state, uint8_t substate_id)
{
	ARG_UNUSED(substate_id);

    uint8_t baseband_sleep_allowed;
    int32_t sleep_duration_hus; // unit 0.5us
    uint32_t xExpectedIdleTime;

    deep_sleep = false;
    int64_t ticks = k_sec_to_ticks_floor64(1000);

	switch (state) 
    {
        case PM_STATE_ACTIVE:
            /* 正常运行，无需操作 */
            break;

        // case PM_STATE_RUNTIME_IDLE:
        // case PM_STATE_SUSPEND_TO_IDLE:
        // 	/*
        // 	 * CPU 空闲: 通知 PMU CPU 即将休眠，然后执行 WFI。
        // 	 * PMU 在 WFI 期间门控 CPU 时钟以节省功耗，
        // 	 * 任何中断/事件均可唤醒。
        // 	 */
        // 	break;

        case PM_STATE_SUSPEND_TO_RAM:
        {
            if(_kernel.idle > ticks){
                xExpectedIdleTime = ticks;
            }
            else{
                xExpectedIdleTime = _kernel.idle;
            }

            /* convert unit of sleep duration from SYSTEM TICK to half us */
            sleep_duration_hus = k_ticks_to_us_floor32(xExpectedIdleTime) * 2;
            /* check whether enter sleep mode is allowed by controller */
            baseband_sleep_allowed = calc_sleep_time_controller(&sleep_duration_hus, &start_basecnt, &start_finecnt);

            /* BLE 控制器不允许, 能睡的时间太短 */
            if ((baseband_sleep_allowed == 0) || (sleep_duration_hus < 2000)){
                return;
            }
            else {
                uint8_t sleep_mode = 0; // 0: light sleep, 1: deep sleep
                int ms = sleep_duration_hus / 2000;

                if ((ms < MINUS_DEEP_SLEEP_TIME)
                    || (baseband_sleep_allowed == 1)
                    || (system_prevent_sleep_get() != 0)
                    || (user_deep_sleep_check()==false)) {   // enter light sleep mode

                    deep_sleep = false;

                    __set_BASEPRI(0);
                    __DSB();
                    __WFI();
                    __ISB();
                }
                else {    // enter deep sleep mode
                    deep_sleep = true;
                    /* notice APP layer that system will enter deep sleep mode */
                    user_entry_before_sleep();
                    /* 
                    * put flash into deep sleep mode as soon as possible, so no more time
                    */
#if FLASH_POWER_DOWN
                    ool_write(PMU_REG_FLASH_VDD_CTRL, 0);
#else
                    flash_enter_deep_sleep(QSPI0);
#endif
                    /* remove time spent in APP layer for sleep preparation */
                    bb_counter_get(&end_basecnt, &end_finecnt);
                    if (end_finecnt >= start_finecnt) {
                        uint32_t diff = (((end_basecnt - start_basecnt) & 0xfffffff) * 625);
                        diff += (end_finecnt - start_finecnt);
                        sleep_duration_hus -= diff;
                    }
                    else {
                        uint32_t diff = (((end_basecnt - 1 - start_basecnt) & 0xfffffff) * 625);
                        diff += (end_finecnt + 625 - start_finecnt);
                        sleep_duration_hus -= diff;
                    }

                    /* 
                    * subtract consumed time in the following process to enter deep sleep mode, 
                    * wakeup_lp signal to clock recover done during wakeup process and prefetch time
                    */
                    sleep_duration_hus -= 1300;
                    /* the time should not smaller than wakeup_lp duration */
                    if (sleep_duration_hus < 3300) {
                        while(1);
                    }

                    /* prepare for deep sleep mode */
                    uint32_t reset_vector = SCB->VTOR;
                    uint32_t rtos_priorities = PM_NVIC_SHPR3_REG;

                    static bool first_sleep = true;
                    uint8_t lcd_reg_tmp;
                    
                    if (first_sleep) {
                        /* record origin lcd configuration */
                        lcd_reg_tmp = ool_read(PMU_REG_LCD_COM_MAP3);
                    }

                    ool_write(PMU_REG_STATUS, PMU_STATUS_DEEP_SLEEP);
                    /* enable access to PWR_Registers */
                    ool_write(PMU_REG_CPI_SLP_ON, 0x80);
                    pmu_adjust_onoff_timing(1000, 1350, sleep_duration_hus>>1);

                    /* 
                    * triger deep sleep state machine early to save time, system will enter deep sleep procedure
                    * in 3~4 lprc clock after this configuration.
                    */
                    *(volatile uint32_t *)0x40000030 |= 0x07;
#if FLASH_POWER_DOWN
                    SYSTEM->QSPIIO_CTRL.MUX |= 0x00000FFF;
                    SYSTEM->QSPIIO_CTRL.IOCCTRL = 0x00;
#endif
                    /* keep the status of QSPI IO */
                    ool_write(PMU_REG_BLOCKS_GATE_CTRL_2, ool_read(PMU_REG_BLOCKS_GATE_CTRL_2) | 0x40);
                    if (first_sleep) {
                        /* enable dynamic control signals */
                        ool_write(PMU_REG_DYN_CTRL, PMU_DYN_OSC_FINE_CAP_BIT 
                                                        | PMU_DYN_OSC_COARSE_CAP_BIT
                                                        | PMU_DYN_OSC_DRV_BIT);
                    }
                    /* disable access to PWR_Registers */
                    ool_write(PMU_REG_CPI_SLP_ON, 0x00);
                    
                    if (first_sleep) {
                        /* FIX BUG: recover PMU_REG_LCD_COM_MAP3 */
                        ool_write(PMU_REG_LCD_COM_MAP3, lcd_reg_tmp);
                    }
                    
                    first_sleep = false;

                    /* remove SPLL control signal from OOL logic */
                    ool_write(PMU_REG_SPLL_MISC, (ool_read(PMU_REG_SPLL_MISC) & (~0x20)) | (0x10)); 

                    /* enter deep sleep */
                    low_power_save();

                    /* the following process is similar to SystemInit */
                    /* FPU settings */
#if (__FPU_PRESENT == 1) && (__FPU_USED == 1)
                    SCB->CPACR |= ((3UL << 10*2) | (3UL << 11*2));    /* set CP10 and CP11 Full Access */
#endif
                    SYSTEM->QSPIIO_CTRL.MUX &= 0xFFFFF000;
                    SYSTEM->QSPIIO_CTRL.IOCCTRL = 0x3f;

                    system_cache_enable(false);
                    flash_init_controller(QSPI0, FLASH_RD_TYPE_QUAD_FAST, FLASH_WR_TYPE_SINGLE);
                    flash_set_baudrate(QSPI0, QSPI_BAUDRATE_DIV_2);

                    /* enable access to PWR_Registers */
                    ool_write(PMU_REG_CPI_SLP_ON, 0x80);
                    /* release the right of control of QSPI IO to qspi controller */
                    ool_write(PMU_REG_BLOCKS_GATE_CTRL_2, ool_read(PMU_REG_BLOCKS_GATE_CTRL_2) & (~0x40));
                    /* disable access to PWR_Registers */
                    ool_write(PMU_REG_CPI_SLP_ON, 0);
#if FLASH_POWER_DOWN
                    ool_write(PMU_REG_FLASH_VDD_CTRL, 0x01);
#else
                    flash_exit_deep_sleep(QSPI0);
#endif

                    /* restore system */
                    bb_wakeup_start();
                    low_power_restore();
                    ool_write(PMU_REG_STATUS, PMU_STATUS_NORAML);
                    SCB->VTOR = reset_vector;
                    PM_NVIC_SHPR3_REG = rtos_priorities;

                    /* reinit RF */
                    void rf_init_custom(void *);
                    extern char rf_api;
                    rf_init_custom(&rf_api);

                    /* ★ 提前清，让 isr_wrapper 跳过 pm_system_resume */
                    _kernel.idle = 0;

                    /*
                    * FIX BUG: some events may be generated by the controller duaring wakeup process, 
                    * real ke_event_notify contains some flash code which may cause longer wake up time, 
                    * event_notify_slince is used to count the events. These events will be triggered 
                    * after ke_event_notify is restored to original function.
                    */
                    event_notify_counter = 0;
                    event_notify_record = ke_event_notify;
                    ke_event_notify = event_notify_slince;

                    /* wait for baseband wake up end */
                    __set_BASEPRI(0);
                    __enable_irq();
                    while(bb_wakeup_is_ongoing());
                    __disable_irq();

                    /* sleep and wake up is controlled by BT sleep timer, this register is modified during wakeup procedure */
                    ool_write(PMU_REG_SLP_WK_SRC, PMU_SLP_BT_TIM_EN_BIT | PMU_WK_BT_TIM_EN_BIT);

                    /* notice APP layer that system has recovery from deep sleep mode */
                    system_nvic_setpriority_default();
                    user_entry_after_sleep();

                    /*
                    * restore ke_event_notify to original function, and re-trigger the events
                    * generated during wakeup procedure.
                    */
                    ke_event_notify = event_notify_record;
                    while(event_notify_counter--) {
                        if (ke_event_notify) {
                            ke_event_notify();
                        }
                    }

                    user_entry_after_sleep_user();
                }
            }
        }break;

	    default:break;
	}
}

/*
 * 从低功耗状态唤醒后的处理。
 * Zephyr 会在中断上下文中（pm_system_resume）调用本函数，
 * 需要在这里恢复中断使能，否则内核无法继续调度。
 */
__RAM_CODE void pm_state_exit_post_ops(enum pm_state state, uint8_t substate_id)
{
    ARG_UNUSED(state);
    ARG_UNUSED(substate_id);

    uint32_t us;
    if (deep_sleep == true)
    {
        deep_sleep = false;
        /* get current baseband counter, used to recover system tick. */
        bb_counter_get(&end_basecnt, &end_finecnt);
        if (start_finecnt > end_finecnt) {
            us = (((end_basecnt - 1 - start_basecnt) & 0xfffffff) * 625 + (end_finecnt + 625 - start_finecnt));
        }
        else {
            us = (((end_basecnt - start_basecnt) & 0xfffffff) * 625 + (end_finecnt - start_finecnt));
        }
        us >>= 1;
        /* 补偿深睡期间丢失的内核时间 */
        uint32_t dticks = us * CONFIG_SYS_CLOCK_TICKS_PER_SEC / USEC_PER_SEC;
        sys_clock_announce(dticks);
        /* 恢复系统tick */
        SysTick->CTRL = SysTick_CTRL_ENABLE_Msk | SysTick_CTRL_TICKINT_Msk | SysTick_CTRL_CLKSOURCE_Msk;
    }

    /* Clear PRIMASK */
    __enable_irq();
}
