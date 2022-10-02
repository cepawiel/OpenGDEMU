/**
 * \file
 *
 * \brief Timer management for the lwIP Raw HTTP basic example.
 *
 * Copyright (c) 2013-2018 Microchip Technology Inc. and its subsidiaries.
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
 */

#include "board.h"
#include "tc.h"
#include "timer_mgt.h"
#include "lwip/init.h"
#include "lwip/sys.h"

/* Clock tick count. */
static volatile uint32_t gs_ul_clk_tick;

#if SAMD20
#include "tc_interrupt.h"
struct tc_module tc_instance;

static void tc_callback(struct tc_module *const module_inst)
{
	/* Increase tick. */
	gs_ul_clk_tick++;
}

void sys_init_timing(void)
{
	struct tc_config config_tc;
	tc_get_config_defaults(&config_tc);

	config_tc.counter_size    = TC_COUNTER_SIZE_16BIT;
	config_tc.wave_generation = TC_WAVE_GENERATION_MATCH_FREQ;
	config_tc.counter_16_bit.compare_capture_channel[0] = 0x5DC0;
	config_tc.clock_source = GCLK_GENERATOR_0;
	config_tc.clock_prescaler = TC_CLOCK_PRESCALER_DIV2;

	tc_init(&tc_instance, TC0, &config_tc);
	tc_enable(&tc_instance);
	tc_register_callback(&tc_instance, tc_callback, TC_CALLBACK_CC_CHANNEL0);
	tc_enable_callback(&tc_instance, TC_CALLBACK_CC_CHANNEL0);

	/* Enable system interrupts. */
	system_interrupt_enable_global();
}
#else
#include "pmc.h"
#include "sysclk.h"

/**
 * TC0 Interrupt handler.
 */
void TC0_Handler(void)
{
	/* Remove warnings. */
	volatile uint32_t ul_dummy;

	/* Clear status bit to acknowledge interrupt. */
	ul_dummy = TC0->TC_CHANNEL[0].TC_SR;

	/* Increase tick. */
	gs_ul_clk_tick++;
}

/**
 * \brief Initialize the timer counter (TC0).
 */
void sys_init_timing(void)
{
	uint32_t ul_div;
	uint32_t ul_tcclks;

	/* Clear tick value. */
	gs_ul_clk_tick = 0;

	/* Configure PMC. */
	pmc_enable_periph_clk(ID_TC0);

	/* Configure TC for a 1kHz frequency and trigger on RC compare. */
	tc_find_mck_divisor(1000,
			sysclk_get_main_hz(), &ul_div, &ul_tcclks,
			sysclk_get_main_hz());
	tc_init(TC0, 0, ul_tcclks | TC_CMR_CPCTRG);
	tc_write_rc(TC0, 0, (sysclk_get_main_hz() / ul_div) / 1000);

	/* Configure and enable interrupt on RC compare. */
	NVIC_EnableIRQ((IRQn_Type)ID_TC0);
	tc_enable_interrupt(TC0, 0, TC_IER_CPCS);

	/* Start timer. */
	tc_start(TC0, 0);
}
#endif

/**
 * \brief Return the number of timer ticks (ms).
 */
uint32_t sys_get_ms(void)
{
	return gs_ul_clk_tick;
}

#if ((LWIP_VERSION) != ((1U << 24) | (3U << 16) | (2U << 8) | (LWIP_VERSION_RC)))
u32_t sys_now(void)
{
	return (sys_get_ms());
}
#endif
