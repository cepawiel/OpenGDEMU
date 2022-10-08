/**
 * \file
 *
 * \brief SAM Always ON Sleep Timer Driver for SAMB11
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
#ifndef AON_SLEEP_TIMER_H_INCLUDED
#define AON_SLEEP_TIMER_H_INCLUDED

/**
 * \defgroup asfdoc_samb_aon_sleep_timer_group SAM AON SLEEP TIMER
 *
 * This driver for Atmel&reg; | SMART SAM devices provides an interface for the
 * configuration and management of the device's Always-On Sleep Timer functionality,
 * for the setup the period of wakeup the MCU from the sleep state.
 *
 * The following peripherals are used by this module:
 *  - AON SLEEP TIMER
 *
 * The following devices can use this module:
 *  - Atmel | SMART SAM B11
 *
 * The outline of this documentation is as follows:
 *  - \ref asfdoc_samb_aon_sleep_timer_prerequisites
 *  - \ref asfdoc_samb_aon_sleep_timer_module_overview
 *  - \ref asfdoc_samb_aon_sleep_timer_special_considerations
 *  - \ref asfdoc_samb_aon_sleep_timer_extra_info
 *  - \ref asfdoc_samb_aon_sleep_timer_examples
 *  - \ref asfdoc_samb_aon_sleep_timer_api_overview
 *
 *
 * \section asfdoc_samb_aon_sleep_timer_prerequisites Prerequisites
 *
 * There are no prerequisites for this module.
 *
 *
 * \section asfdoc_samb_aon_sleep_timer_module_overview Module Overview
 * This driver provides an interrupt for the MCU which in the sleep state
 * when the timer counter is decline to 0.
 *
 * \subsection asfdoc_samb_aon_sleep_timer_module_overview_wakeup_control Wakeup Control
 * You can control wakeup domain, there'er three choices:
 * - Disable wakeup
 * - Wakeup arm
 * - Wakeup arm and blue-tooth
 *
 * \subsection asfdoc_samb_aon_sleep_timer_module_overview_load_mode Counter Load Mode
 * There are two counter load mode:
 * - Single mode
 *
 *   The timer will trigger interrupt when the counter is decline to 0, then the counter is
 *   always 0.
 * - Reload mode
 *
 *   The timer will trigger interrupt when the counter is decline to 0, then the counter is
 *   reload to the initial count.
 *
 * \section asfdoc_samb_aon_sleep_timer_special_considerations Special Considerations
 *
 * There are no special considerations for this module.
 *
 * \section asfdoc_samb_aon_sleep_timer_extra_info Extra Information
 *
 * For extra information, see \ref asfdoc_samb_aon_sleep_timer_extra. This includes:
 *  - \ref asfdoc_samb_aon_sleep_timer_extra_acronyms
 *  - \ref asfdoc_samb_aon_sleep_timer_extra_dependencies
 *  - \ref asfdoc_samb_aon_sleep_timer_extra_errata
 *  - \ref asfdoc_samb_aon_sleep_timer_extra_history
 *
 *
 * \section asfdoc_samb_aon_sleep_timer_examples Examples
 *
 * For a list of examples related to this driver, see
 * \ref asfdoc_samb_aon_sleep_timer_exqsg.
 *
 *
 * \section asfdoc_samb_aon_sleep_timer_api_overview API Overview
 * @{
 */

#include <compiler.h>
#include <system_sam_b.h>

#ifdef __cplusplus
extern "C" {
#endif

/** Type definition for a AON SLEEP TIMER module callback function. */
typedef void (*aon_sleep_timer_callback_t)(void);

/**
 * \brief Specifies the counter mode is reload or single mode.
 *
 * This enum specifies counter with reload or single mode.
 */
enum aon_sleep_timer_mode {
	/** Counter in reload mode */
	AON_SLEEP_TIMER_RELOAD_MODE = 0,
	/** Counter in single mode */
	AON_SLEEP_TIMER_SINGLE_MODE,
};

/**
 * \brief Enable or disable arm wakeup.
 *
 * This enum specifies arm wakeup enable or disable.
 */
enum aon_sleep_timer_wakeup {
	/** Wakeup disable */
	AON_SLEEP_TIMER_WAKEUP_DIS = 0,
	/** Arm wakeup */
	AON_SLEEP_TIMER_WAKEUP_ARM,
	/** Arm and blue-tooth wakeup */
	AON_SLEEP_TIMER_WAKEUP_ARM_BLE,
};

/**
 *  \brief AON Sleep Timer configuration structure.
 *
 *  Configuration structure for a AON Sleep Timer instance. This structure should be
 *  initialized by the \ref aon_sleep_timer_get_config_defaults() function before being
 *  modified by the user application.
 */
struct aon_sleep_timer_config {
	/** Enable wakeup arm or wakeup arm and blue-tooth 
	  * or disable arm wakeup */
	enum aon_sleep_timer_wakeup wakeup;
	/** Counter is reload or single mode */
	enum aon_sleep_timer_mode mode;
	/** Counter value */
	uint32_t counter;
};

/**
 * \name Configuration and Initialization
 * @{
 */
void aon_sleep_timer_get_config_defaults(struct aon_sleep_timer_config *config);
void aon_sleep_timer_init(const struct aon_sleep_timer_config *config);
void aon_sleep_timer_disable(void);
/** @} */

/**
 * \name Get value
 * @{
 */
uint32_t aon_sleep_timer_get_current_value(void);
/** @} */

/**
 * \name Callback
 * @{
 */
void aon_sleep_timer_clear_interrupt(void);
bool aon_sleep_timer_sleep_timer_active(void);
void aon_sleep_timer_register_callback(aon_sleep_timer_callback_t fun);
void aon_sleep_timer_unregister_callback(void);
/** @} */

/** @}*/

#ifdef __cplusplus
}
#endif

/**
 * \page asfdoc_samb_aon_sleep_timer_extra Extra Information for TIMER Driver
 *
 * \section asfdoc_samb_aon_sleep_timer_extra_acronyms Acronyms
 * Below is a table listing the acronyms used in this module, along with their
 * intended meanings.
 * <table>
 *	<tr>
 *		<th>Acronym</th>
 *		<th>Description</th>
 *	</tr>
 *	<tr>
 *		<td>AON SLEEP TIMER</td>
 *		<td>    </td>
 *	</tr>
 * </table>
 *
 * \section asfdoc_samb_aon_sleep_timer_extra_dependencies Dependencies
 * There are no dependencies related to this driver.
 *
 *
 * \section asfdoc_samb_aon_sleep_timer_extra_errata Errata
 * There are no errata related to this driver.
 *
 *
 * \section asfdoc_samb_aon_sleep_timer_extra_history Module History
 * An overview of the module history is presented in the table below, with
 * details on the enhancements and fixes made to the module since its first
 * release. The current version of this corresponds to the newest version in
 * the table.
 *
 * <table>
 *  <tr>
 *      <th>Changelog</th>
 *  </tr>
 *  <tr>
 *      <td>Initial Release</td>
 *  </tr>
 * </table>
 */

/**
 * \page asfdoc_samb_aon_sleep_timer_exqsg Examples for AON SLEEP TIMER Driver
 *
 * This is a list of the available Quick Start guides (QSGs) and example
 * applications for \ref asfdoc_samb_aon_sleep_timer_group. QSGs are simple examples with
 * step-by-step instructions to configure and use this driver in a selection of
 * use cases. Note that QSGs can be compiled as a standalone application or be
 * added to the user application.
 *
 *  - \subpage asfdoc_samb_aon_sleep_timer_basic_use_case
 *
 * \page asfdoc_samb_aon_sleep_timer_document_revision_history Document Revision History
 *
 * <table>
 *  <tr>
 *    <th>Doc. Rev.</td>
 *    <th>Date</td>
 *    <th>Comments</td>
 *  </tr>
 *  <tr>
 *    <td>A</td>
 *    <td>09/2015</td>
 *    <td>Initial release</td>
 *  </tr>
 * </table>
 */

#endif