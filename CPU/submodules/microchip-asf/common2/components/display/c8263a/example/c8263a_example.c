/**
 * \file
 *
 * \brief C8263A LCD Glass component example for SAM.
 *
 * Copyright (c) 2015-2018 Microchip Technology Inc. and its subsidiaries.
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

/**
 * \mainpage
 * \section intro Introduction
 * This is the documentation for the data structures, functions, variables,
 * defines, enums, and typedefs for the C8263A LCD Glass component example.
 * It also comes bundled with an application-example of usage.
 *
 * This example demonstrates how to use the C8263A LCD Glass driver.
 *
 * The supported board list:
 *    - SAM L22 Xplained Pro B
 *
 * \section compilinfo Compilation Information
 * This software is written for GNU GCC and IAR Embedded Workbench
 * for Atmel. Other compilers may or may not work.
 *
 * \section deviceinfo Device Information
 * The TSLCD1 Xplained Pro extension board must be connected to SAM devices.
 *
 * \section configinfo Configuration Information
 * - PC terminal settings:
 *   - 38400 bps,
 *   - 8 data bits,
 *   - no parity bit,
 *   - 1 stop bit,
 *   - no flow control.
 *
 * \section contactinfo Contact Information
 * For further information, visit
 * <A href="http://www.microchip.com">Microchip</A>.\n
 * Support and FAQ: https://www.microchip.com/support/
 */
/*
 * Support and FAQ: visit <a href="https://www.microchip.com/support/">Microchip Support</a>
 */

#include <asf.h>
#include <string.h>
#include "config_c8263a.h"
struct usart_module usart_instance;

/**
 *  Configure serial console.
 */
static void configure_console(void)
{
	struct usart_config config_usart;
	usart_get_config_defaults(&config_usart);
	config_usart.baudrate = 38400;
	config_usart.mux_setting = EDBG_CDC_SERCOM_MUX_SETTING;
	config_usart.pinmux_pad0 = EDBG_CDC_SERCOM_PINMUX_PAD0;
	config_usart.pinmux_pad1 = EDBG_CDC_SERCOM_PINMUX_PAD1;
	config_usart.pinmux_pad2 = EDBG_CDC_SERCOM_PINMUX_PAD2;
	config_usart.pinmux_pad3 = EDBG_CDC_SERCOM_PINMUX_PAD3;
	stdio_serial_init(&usart_instance, EDBG_CDC_MODULE, &config_usart);
	usart_enable(&usart_instance);
}

static void c8263a_icon_test(void)
{
	c8263a_show_icon(C8263A_ICON_COLON);
	delay_s(1);
	c8263a_show_battery(C8263A_BATTERY_THREE);
	delay_s(1);
	c8263a_show_battery(C8263A_BATTERY_ONE);
	delay_s(1);
	c8263a_show_wireless(C8263A_WIRELESS_THREE);
	delay_s(1);
	c8263a_show_wireless(C8263A_WIRELESS_TWO);
	delay_s(1);
	c8263a_show_icon(C8263A_ICON_ARROW_UP);
	delay_s(1);
	c8263a_show_icon(C8263A_ICON_ARROW_DWON);
	delay_s(1);
	c8263a_show_icon(C8263A_ICON_ARROW_LEFT);
	delay_s(1);
	c8263a_show_icon(C8263A_ICON_ARROW_RIGHT);
	delay_s(1);
	c8263a_show_icon(C8263A_ICON_ARROW_CORNER);
	delay_s(1);
	c8263a_clear_icon(C8263A_ICON_COLON);
}
static void c8263a_blink_test(void)
{
	c8263a_blink_icon_start(C8263A_ICON_USB);
	c8263a_blink_icon_start(C8263A_ICON_ATMEL);
	delay_s(1);
	c8263a_blink_icon_stop(C8263A_ICON_USB);
}
static void c8263a_text_test(void)
{
	c8263a_clear_text();
	c8263a_show_text((const uint8_t *)"Hi");
	delay_s(1);
	c8263a_clear_text();
	c8263a_show_text((const uint8_t *)"Hello");
	delay_s(1);
	c8263a_clear_text();
	c8263a_show_text((const uint8_t *)"Atmel");
}

static void c8263a_num_dec_test(void)
{
	c8263a_clear_numeric_dec();
	c8263a_show_numeric_dec(12);
	delay_s(1);
	c8263a_clear_numeric_dec();
	c8263a_show_numeric_dec(345);
	delay_s(1);
	c8263a_clear_numeric_dec();
	c8263a_show_numeric_dec(6789);
	delay_s(1);
	c8263a_clear_numeric_dec();
	c8263a_show_numeric_dec(-98765);
	delay_s(1);
	c8263a_clear_numeric_dec();
	c8263a_show_numeric_dec(-198765);
}

static void c8263a_animation_test(void)
{
	c8263a_circular_animation_start(11,0x0f);
}

/**
 * \brief main function : do init and loop
 */
int main(void)
{
	system_init();
	configure_console();
	delay_init();

	/* Turn on the backlight. */
	port_pin_set_output_level(SLCD_BACLKLIGHT,true);

	printf("Start SLCD test\r\n");
	/* Initialize the C8263A LCD glass component. */
	c8263a_init();

	c8263a_show_all();
	c8263a_set_contrast(0xc);
	delay_s(1);
	c8263a_clear_all();

	c8263a_icon_test();
	delay_s(1);
	c8263a_blink_test();
	delay_s(1);
	c8263a_text_test();
	delay_s(1);
	c8263a_num_dec_test();
	delay_s(1);
	c8263a_animation_test();
	while (1) {
	}
}

