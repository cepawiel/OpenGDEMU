/**
* \file  oled1_support.h
*
* \brief LORAWAN Getting Started  [Button OLED1] Demo Application
* OLED1 Xplained Pro OLED, LED and button driver
*
*
* Copyright (c) 2019 Microchip Technology Inc. and its subsidiaries.
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
* THIS SOFTWARE IS SUPPLIED BY MICROCHIP "AS IS".  NO WARRANTIES,
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
/*
* Support and FAQ: visit <a href="https://www.microchip.com/support/">Microchip Support</a>
*/
#ifndef OLED1_SUPPORT_H_
#define OLED1_SUPPORT_H_

#include <board.h>
#include <compiler.h>
#include <port.h>

#include "gfx_mono_menu.h"




/**
 * \defgroup oled1_xpro_io_group OLED1 Xplained Pro LED and button driver
 *
 * This is a simple driver for manipulating LEDs and reading buttons on any
 * number of OLED1 Xplained Pro wing boards.
 *
 * The display is not supported by this driver. To operate the display, see
 * \ref asfdoc_common2_gfx_mono.
 *
 * @{
 */

//! \name Constants
//@{

//! Active level for LEDs
#define OLED1_LED_ACTIVE     false

//! Active level for buttons
#define OLED1_BUTTON_ACTIVE  false

//@}


/**
 * \brief Create a driver instance
 *
 * This macro creates a \ref oled1_instance struct with the specified name, and
 * populates it with pin identifiers that correspond to the specified
 * extension header.
 *
 * This macro is created for Xplained Pro-style board definitions as it makes
 * use of the EXT_PIN_x macros.
 *
 * To initialize the hardware, the function \ref oled1_init() must be called on
 * each driver instance.
 *
 * \param name Name to use for driver instance.
 * \param ext_header Extension header to use, on the form EXTn (f.ex. EXT1).
 */
#define OLED1_CREATE_INSTANCE(name, ext_header)  \
	_OLED1_CREATE_INSTANCE(name, ext_header)

#define _OLED1_CREATE_INSTANCE(name, ext_header)  \
	const struct oled1_instance name = {          \
		.led0_pin = ext_header ## _PIN_7,         \
		.led1_pin = ext_header ## _PIN_8,         \
		.led2_pin = ext_header ## _PIN_6,         \
		.button0_pin = ext_header ## _PIN_9,      \
		.button1_pin = ext_header ## _PIN_3,      \
		.button2_pin = ext_header ## _PIN_4,      \
	}


/**
 * \brief Driver instance struct
 *
 * This struct contains identifiers for the LED and button pins. It should be
 * created with the macro \ref OLED1_CREATE_INSTANCE(), and the hardware be
 * initialized with the function \ref oled1_init().
 */
struct oled1_instance {
	//! Pin identifier for first LED
	uint8_t led0_pin;
	//! Pin identifier for second LED
	uint8_t led1_pin;
	//! Pin identifier for third LED
	uint8_t led2_pin;
	//! Pin identifier for first button
	uint8_t button0_pin;
	//! Pin identifier for second button
	uint8_t button1_pin;
	//! Pin identifier for third button
	uint8_t button2_pin;
};

//! IDs for each of the on-board LEDs
enum oled1_led_id {
	OLED1_LED1_ID,
	OLED1_LED2_ID,
	OLED1_LED3_ID,
};

//! IDs for each of the on-board buttons
enum oled1_button_id {
	OLED1_BUTTON1_ID,
	OLED1_BUTTON2_ID,
	OLED1_BUTTON3_ID,
};





/**
 * \brief Initialize hardware for driver instance
 *
 * This function initializes the hardware (pins) that belong to the specified
 * driver instance.
 *
 * \param oled1 Address of driver instance to initialize hardware for.
 */
static inline void oled1_init(const struct oled1_instance *const oled1)
{
	struct port_config pin_conf;

	port_get_config_defaults(&pin_conf);

	port_pin_set_output_level(oled1->led0_pin, !OLED1_LED_ACTIVE);
	port_pin_set_output_level(oled1->led1_pin, !OLED1_LED_ACTIVE);
	port_pin_set_output_level(oled1->led2_pin, !OLED1_LED_ACTIVE);

	pin_conf.direction  = PORT_PIN_DIR_OUTPUT_WTH_READBACK;
	pin_conf.input_pull = PORT_PIN_PULL_NONE;
	port_pin_set_config(oled1->led0_pin, &pin_conf);
	port_pin_set_config(oled1->led1_pin, &pin_conf);
	port_pin_set_config(oled1->led2_pin, &pin_conf);

	pin_conf.direction  = PORT_PIN_DIR_INPUT;
	pin_conf.input_pull = PORT_PIN_PULL_UP;
	port_pin_set_config(oled1->button0_pin, &pin_conf);
	port_pin_set_config(oled1->button1_pin, &pin_conf);
	port_pin_set_config(oled1->button2_pin, &pin_conf);
}

//! \name LED manipulation
//@{

/**
 * \internal
 * \brief Get the pin identifier for a given LED number in instance
 *
 * \param oled1 Address of driver instance to fetch pin identifier from.
 * \param led_id ID of the LED to fetch pin identifier for.
 */
static inline uint8_t _oled1_get_led_pin(
		const struct oled1_instance *const oled1,
		const enum oled1_led_id led_id)
{
	uint8_t pin;

	switch (led_id) {
	case OLED1_LED1_ID:
		pin = oled1->led0_pin;
		break;

	case OLED1_LED2_ID:
		pin = oled1->led1_pin;
		break;

	default:
	case OLED1_LED3_ID:
		pin = oled1->led2_pin;
		break;
	}

	return pin;
}

/**
 * \brief Get the state of a LED
 *
 * \param oled1 Address of driver instance to operate on.
 * \param led_id ID of the LED to get state of.
 *
 * \retval true if LED is active, i.e., lit up.
 * \retval false if LED is not active.
 */
static inline bool oled1_get_led_state(const struct oled1_instance *const oled1,
		const enum oled1_led_id led_id)
{
	uint8_t pin = _oled1_get_led_pin(oled1, led_id);
	return port_pin_get_input_level(pin) == OLED1_LED_ACTIVE;
}

/**
 * \brief Set the state of a LED
 *
 * \param oled1 Address of driver instance to operate on.
 * \param led_id ID of the LED to set state of.
 * \param state State to set LED to:
 * \arg true to activate, i.e., light it up.
 * \arg false to deactivate.
 */
static inline void oled1_set_led_state(const struct oled1_instance *const oled1,
		const enum oled1_led_id led_id, const bool state)
{
	uint8_t pin = _oled1_get_led_pin(oled1, led_id);
	port_pin_set_output_level(pin, state == OLED1_LED_ACTIVE);
}

/**
 * \brief Set the state of a LED
 *
 * \param oled1 Address of driver instance to operate on.
 * \param led_id ID of the LED to toggle state of.
 */
static inline void oled1_toggle_led_state(
		const struct oled1_instance *const oled1,
		const enum oled1_led_id led_id)
{
	uint8_t pin = _oled1_get_led_pin(oled1, led_id);
	port_pin_toggle_output_level(pin);
}

//@}

//! \name Button manipulation
//@{

/**
 * \internal
 * \brief Get the pin identifier for a given button number in instance
 *
 * \param oled1_instance Driver instance to fetch pin identifier from.
 * \param led_id ID of the button to fetch pin identifier for.
 *
 * \return Identifier of pin for specified button.
 */
static inline uint8_t _oled1_get_button_pin(
		const struct oled1_instance *const oled1,
		const enum oled1_button_id button_id)
{
	uint8_t pin;

	switch (button_id) {
	case OLED1_BUTTON1_ID:
		pin = oled1->button0_pin;
		break;

	case OLED1_BUTTON2_ID:
		pin = oled1->button1_pin;
		break;

	default:
	case OLED1_BUTTON3_ID:
		pin = oled1->button2_pin;
		break;
	}

	return pin;
}

/**
 * \brief Get the state of a button
 *
 * \param oled1 Address of driver instance to operate on.
 * \param led_id ID of the button to get state of.
 *
 * \retval true if button is active, i.e., pressed.
 * \retval false if button is not active.
 */
static inline bool oled1_get_button_state(
		const struct oled1_instance *const oled1,
		const enum oled1_button_id button_id)
{
	uint8_t pin = _oled1_get_button_pin(oled1, button_id);
	return port_pin_get_input_level(pin) == OLED1_BUTTON_ACTIVE;
}



void oled1_print_array(uint8_t *array, uint8_t length, uint8_t x, uint8_t y) ;

void button_splash(void);

typedef struct gfx_mono_menu Bselmenu;
Bselmenu Bsel_menu;

typedef struct gfx_mono_menu Mmenu;
Mmenu main_menu;



#endif // OLED1_SUPPORT_H
