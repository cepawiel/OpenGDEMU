/**
 * @file sleep_mgr.c
 *
 * @brief
 *
 * Copyright (c) 2018 - 2019 Microchip Technology Inc. and its subsidiaries.
 *
 * \asf_license_start
 *
 * \page License
 *
 * Subject to your compliance with these terms, you may use Microchip
 * software and any derivatives exclusively with Microchip products.
 * It is your responsibility to comply with third party license terms applicable
 * to your use of third party software (including open source software) that
 * may accompany Microchip software.
 *
 * THIS SOFTWARE IS SUPPLIED BY MICROCHIP "AS IS". NO WARRANTIES,
 * WHETHER EXPRESS, IMPLIED OR STATUTORY, APPLY TO THIS SOFTWARE,
 * INCLUDING ANY IMPLIED WARRANTIES OF NON-INFRINGEMENT, MERCHANTABILITY,
 * AND FITNESS FOR A PARTICULAR PURPOSE. IN NO EVENT WILL MICROCHIP BE
 * LIABLE FOR ANY INDIRECT, SPECIAL, PUNITIVE, INCIDENTAL OR CONSEQUENTIAL
 * LOSS, DAMAGE, COST OR EXPENSE OF ANY KIND WHATSOEVER RELATED TO THE
 * SOFTWARE, HOWEVER CAUSED, EVEN IF MICROCHIP HAS BEEN ADVISED OF THE
 * POSSIBILITY OR THE DAMAGES ARE FORESEEABLE.  TO THE FULLEST EXTENT
 * ALLOWED BY LAW, MICROCHIP'S TOTAL LIABILITY ON ALL CLAIMS IN ANY WAY
 * RELATED TO THIS SOFTWARE WILL NOT EXCEED THE AMOUNT OF FEES, IF ANY,
 * THAT YOU HAVE PAID DIRECTLY TO MICROCHIP FOR THIS SOFTWARE.
 *
 * \asf_license_stop
 *
 *
 */

/*
 * Copyright (c) 2014-2018 Microchip Technology Inc. and its subsidiaries.
 *
 * Licensed under Atmel's Limited License Agreement --> EULA.txt
 */

#include "sleep_mgr.h"
#include "rtc_count.h"
#include "system.h"
#include "rtc_count_interrupt.h"
#include "asf.h"
#include "trx_access.h"
#include "sysTimer.h"

/* Minimum sleep interval in milliseconds */
#define MIN_SLEEP_INTERVAL     (1000)

struct rtc_module rtc_instance;

/**
 * @brief Configuring RTC Callback Function on Overflow
 *
 * @param void
 */
static void configure_rtc_callbacks(void);

/**
 * @brief Callback Function indicating RTC Overflow
 *
 * @param void
 */
static void rtc_overflow_callback(void);

/**
 * @brief Sleep Preparation procedures
 *
 * @param void
 */
static void sleepPreparation(void);

/**
 * @brief Sleep Exit procedures
 *
 * @param sleepTime
 */
static void sleepExit(uint32_t sleepTime);

/**
 * \brief This function Initializes the Sleep functions
 * Enable RTC Clock in conf_clocks.h
 */
void sleepMgr_init(void)
{
    struct rtc_count_config config_rtc_count;

    rtc_count_get_config_defaults(&config_rtc_count);
    config_rtc_count.prescaler           = RTC_COUNT_PRESCALER_DIV_1;
    config_rtc_count.mode                = RTC_COUNT_MODE_32BIT;

#ifdef FEATURE_RTC_CONTINUOUSLY_UPDATED
    /** Continuously update the counter value so no synchronization is
     *  needed for reading. */
    config_rtc_count.continuously_update = true;
#endif

    /* Clear the timer on match to generate the overflow interrupt*/
    config_rtc_count.clear_on_match = true;

    /* Initialize RTC Counter */
    rtc_count_init(&rtc_instance, RTC, &config_rtc_count);
    configure_rtc_callbacks();
}

/**
 * @brief Sleep Preparation procedures
 *
 * @param void
 */
static void sleepPreparation(void)
{
    /* Disable Transceiver SPI */
    trx_spi_disable();
}

/**
 * @brief Sleep Exit procedures
 *
 * @param sleepTime
 */
static void sleepExit(uint32_t sleepTime)
{
    /* Enable Transceiver SPI */
    trx_spi_enable();

    /* Synchronize Timers */
    SYS_TimerAdjust_SleptTime(sleepTime);
}

/**
 * \brief This function puts the transceiver and device to sleep
 * \Parameter interval - the time to sleep in milliseconds
 */
bool sleepMgr_sleep(uint32_t interval)
{
    if (interval < MIN_SLEEP_INTERVAL)
    {
        return false;
    }

    /*Set the timeout for compare mode and enable the RTC*/
    rtc_count_set_compare(&rtc_instance, interval, RTC_COUNT_COMPARE_0);

    /* Configure RTC Callbacks */
    configure_rtc_callbacks();

    /* Enable RTC */
    rtc_count_enable(&rtc_instance);

    /* Preparing to go for sleep */
    sleepPreparation();

    /*put the MCU in standby mode with RTC as wakeup source*/
    system_set_sleepmode(SYSTEM_SLEEPMODE_STANDBY);
    system_sleep();

    /* Exit procedure after wakeup */
    sleepExit(interval);
    return true;
}

static void configure_rtc_callbacks(void)
{
    /*Register rtc callback*/
    rtc_count_register_callback(
            &rtc_instance, rtc_overflow_callback,
            RTC_COUNT_CALLBACK_OVERFLOW);
    rtc_count_enable_callback(&rtc_instance, RTC_COUNT_CALLBACK_OVERFLOW);
}

static void rtc_overflow_callback(void)
{
    /* Disable RTC upon interrupt */
    rtc_count_disable(&rtc_instance);
}
