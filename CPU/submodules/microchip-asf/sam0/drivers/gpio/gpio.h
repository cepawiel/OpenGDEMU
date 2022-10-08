/**
 * \file
 *
 * \brief SAM GPIO Driver for SAMB11
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
/*
 * Support and FAQ: visit <a href="https://www.microchip.com/support/">Microchip Support</a>
 */
#ifndef GPIO_H_INCLUDED
#define GPIO_H_INCLUDED

/**
 * \defgroup asfdoc_samb_gpio_group SAM GPIO Driver (GPIO)
 *
 * This driver for Atmel&reg; | SMART SAM devices provides an interface for the
 * configuration and management of the device's General Purpose Input/Output
 * (GPIO) pin functionality, for manual pin state reading and writing.
 *
 * The following peripherals are used by this module:
 *  - GPIO (GPIO Management)
 *
 * The following devices can use this module:
 *  - Atmel | SMART SAM B11
 *
 * The outline of this documentation is as follows:
 *  - \ref asfdoc_samb_gpio_prerequisites
 *  - \ref asfdoc_samb_gpio_module_overview
 *  - \ref asfdoc_samb_gpio_special_considerations
 *  - \ref asfdoc_samb_gpio_extra_info
 *  - \ref asfdoc_samb_gpio_examples
 *  - \ref asfdoc_samb_gpio_api_overview
 *
 *
 * \section asfdoc_samb_gpio_prerequisites Prerequisites
 *
 * There are no prerequisites for this module.
 *
 *
 * \section asfdoc_samb_gpio_module_overview Module Overview
 *
 * The device GPIO module provides an interface between the user application
 * logic and external hardware peripherals, when general pin state manipulation
 * is required. This driver provides an easy-to-use interface to the physical
 * pin input samplers and output drivers, so that pins can be read from or
 * written to for general purpose external hardware control.
 * The GPIO I/O lines are differentiated based on whether digital or analog 
 * functionalities are enabled, whether the pins are capable of waking-up the core
 * from sleep into 3 groups called LP_GPIO_x, AO_GPIO_z, GPIO_MSy
 *
 * The GPIO module also allows all GPIO pins (LP_GPIO_x, GPIO_MSy, AO_GPIO_z) to be
 * configured as interrupt lines. Each interrupt line can be individually 
 * masked and can generate an interrupt to CPU on rising, falling, or on high or 
 * low levels.
 *
 * Only AO_GPIO_z pin can wakeup the ARM Subsystem as well as BLE Subsystem from ULP mode. 
 * Along with wakeup configuration enabling as an external interrupt will wakeup ARM and
 * generate interrupt request
 *
 * There are different peripheral functions that are Software selectable
 * on a per pin basis. This allows for maximum flexibility of mapping desired
 * interfaces on GPIO pins. MUX1 option allows for any MEGAMUX option to be
 * assigned to a GPIO.
 *
 * An example is to illustrate the available options for pin LP_GPIO_3, depending
 * on the pin-MUX option selected:
 * - MUX0: the pin will function as bit 3 of the GPIO bus and is controlled by
 *         the GPIO controller in the ARM subsystem
 * - MUX1: any option from the MEGAMUX table can be selected, for example it
 *         can be a quad_dec, pwm, or any of the other functions listed in the
 *         MEGAMUX table
 * - MUX2: the pin will function as UART1 TXD; this can be also achieved with
 *         the MUX1 option via MEGAMUX, but the MUX2 option allows a shortcut
 *         for the recommended pinout
 * - MUX3: this option is not used and thus defaults to the GPIO option (same
 *         as MUX0)
 * - MUX4: the pin will function as SPI1 MOSI (this option is not available
 *         through MEGAMUX)
 * - MUX5: the pin will function as SPI0 MOSI (this option is not available
 *         through MEGAMUX)
 * - MUX6: the pin will function as SPI FLASH SCK (this option is not available
 *         through MEGAMUX)
 * - MUX7: the pin will function as bit 3 of the test output bus, giving access
 *         to various debug signals
 *
 * \section asfdoc_samb_gpio_special_considerations Special Considerations
 *
 * There are no special considerations for this module.
 *
 * \section asfdoc_samb_gpio_extra_info Extra Information
 *
 * For extra information, see \ref asfdoc_samb_gpio_extra. This includes:
 *  - \ref asfdoc_samb_gpio_extra_acronyms
 *  - \ref asfdoc_samb_gpio_extra_dependencies
 *  - \ref asfdoc_samb_gpio_extra_errata
 *  - \ref asfdoc_samb_gpio_extra_history
 *
 *
 * \section asfdoc_samb_gpio_examples Examples
 *
 * For a list of examples related to this driver, see
 * \ref asfdoc_samb_gpio_exqsg.
 *
 *
 * \section asfdoc_samb_gpio_api_overview API Overview
 * @{
 */

#include <compiler.h>
#include <system_sam_b.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 *  \brief GPIO pin direction configuration enum.
 *
 *  Enum for the possible pin direction settings of the gpio pin configuration
 *  structure, to indicate the direction the pin should use.
 */
enum gpio_pin_dir {
	/** The pin's input buffer should be enabled, so that the pin state can
	 *  be read. */
	GPIO_PIN_DIR_INPUT,
	/** The pin's output buffer should be enabled, so that the pin state can
	 *  be set. */
	GPIO_PIN_DIR_OUTPUT,
};

/**
 *  \brief GPIO pin input pull configuration enum.
 *
 *  Enum for the possible pin pull settings of the GPIO pin configuration
 *  structure, to indicate the type of logic level pull the pin should use.
 */
enum gpio_pin_pull {
	/** No logical pull should be applied to the pin */
	GPIO_PIN_PULL_NONE,
	/** Pin should be pulled up when idle */
	GPIO_PIN_PULL_UP ,
	/** Pin should be pulled down when idle */
	GPIO_PIN_PULL_DOWN,
};

/**
 *  \brief GPIO pinmux selection enum.
 *
 *  Enum for the pinmux settings of the GPIO pin configuration.
 *  PINMUX = 0 to 3 is for AO_GPIO_z, PINMUX = 0 to 7 for LP_GPIO_x
 *  PINMUX = 0,1 for GPIO_MSy
 */
enum gpio_pinmux_sel {
	/** PINMUX selection 0 */
	GPIO_PINMUX_SEL_0 = 0,
	/** PINMUX selection 1 */
	GPIO_PINMUX_SEL_1,
	/** PINMUX selection 2 */
	GPIO_PINMUX_SEL_2,
	/** PINMUX selection 3 */
	GPIO_PINMUX_SEL_3,
	/** PINMUX selection 4 */
	GPIO_PINMUX_SEL_4,
	/** PINMUX selection 5 */
	GPIO_PINMUX_SEL_5,
	/** PINMUX selection 6 */
	GPIO_PINMUX_SEL_6,
	/** PINMUX selection 7 */
	GPIO_PINMUX_SEL_7,
};

/**
 * \brief GPIO module instance
 *
 * Forward Declaration for the device instance.
 */
struct gpio_module;

/**
 * \brief GPIO callback type
 *
 * Type of the callback functions.
 */
typedef void (*gpio_callback_t)(void);

/**
 * \brief GPIO Callback enum
 *
 * Callbacks for the GPIO driver.
 */
enum gpio_callback {
	/** Callback for low level */
	GPIO_CALLBACK_LOW,
	/** Callback for high level */
	GPIO_CALLBACK_HIGH,
	/** Callback for rising edge */
	GPIO_CALLBACK_RISING,
	/** Callback for falling edge */
	GPIO_CALLBACK_FALLING,
	/** Number of available callbacks */
	GPIO_CALLBACK_N,
};

/**
 *  \brief GPIO pin configuration structure.
 *
 *  Configuration structure for a GPIO pin instance. This structure should be
 *  initialized by the \ref GPIO_get_config_defaults() function before being
 *  modified by the user application.
 */
struct gpio_config {
	/** GPIO buffer input/output direction */
	enum gpio_pin_dir  direction;

	/** GPIO pull-up/pull-down for input pins */
	enum gpio_pin_pull input_pull;

	/** Enable AON_GPIOs to wakeup MCU from ULP mode 
	 *
	 *  \note Only AO_GPIO_0, could enable this feature as per current ROM firmware code
	 *		Restrict to use only AO_GPIO_0 for waking up BLE and MCU
	 */
	bool aon_wakeup;
};

/**
 * \brief GPIO driver software device instance structure.
 *
 * GPIO driver software instance structure, used to retain software
 * state information of an associated hardware module instance.
 *
 * \note The fields of this structure should not be altered by the user
 *       application; they are reserved for module-internal use only.
 */
struct gpio_module {
#if !defined(__DOXYGEN__)
	/** Pointer to the hardware instance */
	Gpio *hw;
	/** Array to store callback function pointers in */
	gpio_callback_t callback[16];
	/** Bit mask for callbacks registered */
	uint16_t callback_reg_mask;
	/** Bit mask for callbacks enabled */
	uint16_t callback_enable_mask;
#endif
};

/** \name Configuration and initialization
 * @{
 */

void gpio_get_config_defaults(struct gpio_config *const config);
enum status_code gpio_pin_set_config(const uint8_t gpio_pin,
		const struct gpio_config *config);

/** @} */

/** \name State reading/writing (logical pin orientated)
 * @{
 */

bool gpio_pin_get_input_level(const uint8_t gpio_pin);
bool gpio_pin_get_output_level(const uint8_t gpio_pin);
void gpio_pin_set_output_level(const uint8_t gpio_pin, const bool level);
void gpio_pin_toggle_output_level(const uint8_t gpio_pin);
enum status_code gpio_pin_invert_output_level(const uint8_t gpio_pin, const bool invert);
/** @} */

/** \name PINMUX and MEGAMUX selection configuration.
 * @{
 */
void gpio_pinmux_cofiguration(const uint8_t gpio_pin, uint16_t pinmux_megamux_sel);
/** @}*/

/** \name GPIO callback config
 * @{
 */
void gpio_register_callback(uint8_t gpio_pin, gpio_callback_t callback_func,
				enum gpio_callback callback_type);
void gpio_unregister_callback(uint8_t gpio_pin,
				enum gpio_callback callback_type);
void gpio_enable_callback(uint8_t gpio_pin);
void gpio_disable_callback(uint8_t gpio_pin);
void gpio_init(void);
/** @}*/

/** @}*/

#ifdef __cplusplus
}
#endif


/**
 * \page asfdoc_samb_gpio_extra Extra Information for GPIO Driver
 *
 * \section asfdoc_samb_gpio_extra_acronyms Acronyms
 * Below is a table listing the acronyms used in this module, along with their
 * intended meanings.
 *
 * <table>
 *	<tr>
 *		<th>Acronym</th>
 *		<th>Description</th>
 *	</tr>
 *	<tr>
 *		<td>GPIO</td>
 *		<td>General Purpose Input/Output</td>
 *	</tr>
 * </table>
 *
 *
 * \section asfdoc_samb_gpio_extra_dependencies Dependencies
 * There are no dependencies related to this driver.
 *
 *
 * \section asfdoc_samb_gpio_extra_errata Errata
 * There are no errata related to this driver.
 *
 *
 * \section asfdoc_samb_gpio_extra_history Module History
 * An overview of the module history is presented in the table below, with
 * details on the enhancements and fixes made to the module since its first
 * release. The current version of this corresponds to the newest version in
 * the table.
 *
 * <table>
 *	<tr>
 *		<th>Changelog</th>
 *	</tr>
 *	<tr>
 *		<td>Initial Release</td>
 *	</tr>
 * </table>
 */

/**
 * \page asfdoc_samb_gpio_exqsg Examples for GPIO Driver
 *
 * This is a list of the available Quick Start guides (QSGs) and example
 * applications for \ref asfdoc_samb_gpio_group. QSGs are simple examples with
 * step-by-step instructions to configure and use this driver in a selection of
 * use cases. Note that QSGs can be compiled as a standalone application or be
 * added to the user application.
 *
 *  - \subpage asfdoc_samb_gpio_basic_use_case
 *  - \subpage asfdoc_samb_gpio_callback_use_case
 *
 * \page asfdoc_samb_gpio_document_revision_history Document Revision History
 *
 * <table>
 *	<tr>
 *		<th>Doc. Rev.</td>
 *		<th>Date</td>
 *		<th>Comments</td>
 *	</tr>
 *	<tr>
 *		<td>A</td>
 *		<td>09/2015</td>
 *		<td>Initial release</td>
 *	</tr>
 * </table>
 */

#endif
