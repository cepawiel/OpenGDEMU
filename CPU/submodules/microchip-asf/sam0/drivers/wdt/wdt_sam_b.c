/**
 * \file
 *
 * \brief SAM Watchdog Driver for SAMB
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
#include "wdt_sam_b.h"

struct wdt_module *_wdt_instances[WDT_INST_NUM];

static void wdt_isr_handler(void)
{
	struct wdt_module *module = NULL;
	
	if (WDT0->WDOGMIS.reg) {
		module = _wdt_instances[0];
		if (!(module->hw->WDOGCONTROL.reg & WDT_WDOGCONTROL_RESEN)) {
			module->hw->WDOGINTCLR.reg = 0x01;
		}
		if ((module->callback_enable_mask & (1 << WDT_CALLBACK_EARLY_WARNING)) &&
			(module->callback_reg_mask & (1 << WDT_CALLBACK_EARLY_WARNING))) {
			(module->callback[WDT_CALLBACK_EARLY_WARNING])();
		}
	}
	if (WDT1->WDOGMIS.reg) {
		module = _wdt_instances[1];
		if (!(module->hw->WDOGCONTROL.reg & WDT_WDOGCONTROL_RESEN)) {
			module->hw->WDOGINTCLR.reg = 0x01;
		}
		if ((module->callback_enable_mask & (1 << WDT_CALLBACK_EARLY_WARNING)) &&
			(module->callback_reg_mask & (1 << WDT_CALLBACK_EARLY_WARNING))) {
			(module->callback[WDT_CALLBACK_EARLY_WARNING])();
		}
	}
}

/**
 * \brief Initializes a Watchdog Timer configuration structure to defaults.
 *
 *  Initializes a given Watchdog Timer configuration structure to a set of
 *  known default values. This function should be called on all new
 *  instances of these configuration structures before being modified by the
 *  user application.
 *
 *  The default configuration is as follows:
 *   \li Load register value
 *   \li Enable reset output
 *   \li Open write access
 *
 *  \param[out] config  Configuration structure to initialize to default values
 */
void wdt_get_config_defaults(struct wdt_config *const config)
{
	/* Sanity check arguments */
	Assert(config);

	config->load_value = 0xFFFFFFFF;
	config->enable_reset = true;
	config->write_access = true;
}

/**
 * \brief Sets up the WDT hardware module based on the configuration.
 *
 * Writes a given configuration of a WDT configuration to the
 * hardware module, and initializes the internal device struct.
 *
 * \param[in]  module  Pointer to the software instance struct
 * \param[in]  hw      Pointer to WDT hardware instance
 * \param[in]  config  Pointer to configuration struct
 *
 * \return Status of the initialization.
 *
 * \retval STATUS_OK            The initialization was successful
 * \retval STATUS_ERR_BAD_DATA  If the value isn't available
 */
enum status_code wdt_set_config(struct wdt_module *const module, Wdt * const hw,
		const struct wdt_config *const config)
{
	/* Sanity check arguments */
	Assert(module);
	Assert(hw);
	Assert(config);

	/* Assign module pointer to software instance struct */
	module->hw = hw;

	if (config->load_value == 0) {
		return STATUS_ERR_BAD_DATA;
	}
	
	if (module->hw == WDT0) {
		system_clock_peripheral_disable(PERIPHERAL_WDT0);
	} else if (module->hw ==WDT1) {
		system_clock_peripheral_disable(PERIPHERAL_WDT1);
	}

	/* Unlock register */
	module->hw->WDOGLOCK.reg = WDT_WRITE_ACCESS_KEY;

	module->hw->WDOGLOAD.reg = config->load_value;

	if (config->enable_reset) {
		module->hw->WDOGCONTROL.reg |= WDT_WDOGCONTROL_RESEN;
	}
	module->hw->WDOGCONTROL.reg |= WDT_WDOGCONTROL_INTEN;
	
	/* Lock register */
	if (config->write_access == false) {
		module->hw->WDOGLOCK.reg = WDT_WDOGLOCK_ENABLE_STATUS;
	}
	
	system_register_isr(RAM_ISR_TABLE_NMI_INDEX, (uint32_t)wdt_isr_handler);
	
	/* Enable WDT clock */
	if (module->hw == WDT0) {
		_wdt_instances[0] = module;
		system_clock_peripheral_enable(PERIPHERAL_WDT0);
	} else if (module->hw == WDT1) {
		_wdt_instances[1] = module;
		system_clock_peripheral_enable(PERIPHERAL_WDT1);
	}

	return STATUS_OK;
}

/**
 * \brief Reset WDT module.
 *
 * Reset WDT module.
 *
 * \param[in]  module  Pointer to the software instance struct
 */
void wdt_reset(struct wdt_module *const module)
{
	if (module->hw == WDT0) {
		system_peripheral_reset(PERIPHERAL_WDT0);
	} else if (module->hw == WDT1) {
		system_peripheral_reset(PERIPHERAL_WDT1);
	}
}

/**
 * \brief Get WDT interrupt status.
 *
 * Get WDT interrupt status.
 *
 * \param[in]  module  Pointer to the software instance struct
 */
uint8_t wdt_get_interrupt_status(struct wdt_module *const module)
{
	return module->hw->WDOGMIS.reg;
}

/**
 * \brief Get WDT raw interrupt status.
 *
 * Get WDT raw interrupt status.
 *
 * \param[in]  module  Pointer to the software instance struct
 */
uint8_t wdt_get_status(struct wdt_module *const module)
{
	return module->hw->WDOGRIS.reg;
}

/**
 * \brief Clear WDT interrupt status.
 *
 * Clear WDT interrupt status.
 *
 * \param[in]  module  Pointer to the software instance struct
 */
void wdt_clear_status(struct wdt_module *const module)
{
	module->hw->WDOGINTCLR.reg = 0x01;
}

/**
 * \brief Reload the count of the running Watchdog Timer.
 *
 * Reload the value of WDT Load register. When this register is written to,
 * the count is immediately restarted from the new value.
 *
 * \param[in]  module      Pointer to the software instance struct
 * \param[in]  load_value  Reload value
 *
 * \return Status of the operation.
 * \retval STATUS_OK            If the operation was completed
 * \retval STATUS_ERR_BAD_DATA  If the value isn't available
 */
enum status_code wdt_set_reload_count(struct wdt_module *const module, uint32_t load_value)
{
	if (load_value == 0) {
		return STATUS_ERR_BAD_DATA;
	} else {
		if (module->hw->WDOGLOCK.bit.ENABLE_STATUS) {
			module->hw->WDOGLOCK.reg = WDT_WRITE_ACCESS_KEY;
			module->hw->WDOGLOAD.reg = load_value;
			module->hw->WDOGLOCK.reg = WDT_WDOGLOCK_ENABLE_STATUS;
		} else {
			module->hw->WDOGLOAD.reg = load_value;
		}
	}

	return STATUS_OK;
}

/**
 * \brief Get the current count value of the running Watchdog Timer.
 *
 * Get the current count value of the running Watchdog Timer.
 *
 * \param[in]      module       Pointer to the software instance struct
 * \param[in,out]  count_value  Pointer to store the current count value
 *
 */
void wdt_get_current_count(struct wdt_module *const module, \
			uint32_t * count_value)
{
	*count_value = module->hw->WDOGVALUE.reg;
}

/**
 * \brief Registers a callback
 *
 * Registers a callback function which is implemented by the user.
 *
 * \note The callback must be enabled by \ref wdt_enable_callback,
 *       in order for the interrupt handler to call it when the conditions for
 *       the callback type are met.
 *
 * \param[in]  module         Pointer to WDT software instance struct
 * \param[in]  callback_func  Pointer to callback function
 * \param[in]  callback_type  Callback type given by an enum
 *
 */
void wdt_register_callback(struct wdt_module *const module,
		wdt_callback_t callback_func,
		enum wdt_callback callback_type)
{
	/* Sanity check arguments */
	Assert(module);
	Assert(callback_func);

	/* Register callback function */
	module->callback[callback_type] = callback_func;
	/* Set the bit corresponding to the callback_type */
	module->callback_reg_mask |= (1 << callback_type);
}

/**
 * \brief Unregisters a callback
 *
 * Unregisters a callback function which is implemented by the user.
 *
 * \param[in,out]  module         Pointer to WDT software instance struct
 * \param[in]      callback_type  Callback type given by an enum
 *
 */
void wdt_unregister_callback(struct wdt_module *module,
		enum wdt_callback callback_type)
{
	/* Sanity check arguments */
	Assert(module);

	/* Unregister callback function */
	module->callback[callback_type] = NULL;
	/* Clear the bit corresponding to the callback_type */
	module->callback_reg_mask &= ~(1 << callback_type);
}

/**
 * \brief Enables callback
 *
 * Enables the callback function registered by the \ref usart_register_callback.
 * The callback function will be called from the interrupt handler when the
 * conditions for the callback type are met.
 *
 * \param[in]  module         Pointer to WDT software instance struct
 * \param[in]  callback_type  Callback type given by an enum
 */
void wdt_enable_callback(struct wdt_module *const module,
		enum wdt_callback callback_type)
{
	/* Sanity check arguments */
	Assert(module);

	/* Enable callback */
	module->callback_enable_mask |= (1 << callback_type);
}

/**
 * \brief Disable callback
 *
 * Disables the callback function registered by the \ref usart_register_callback,
 * and the callback will not be called from the interrupt routine.
 *
 * \param[in]  module         Pointer to WDT software instance struct
 * \param[in]  callback_type  Callback type given by an enum
 */
void wdt_disable_callback(struct wdt_module *const module,
		enum wdt_callback callback_type)
{
	/* Sanity check arguments */
	Assert(module);

	/* Disable callback */
	module->callback_enable_mask &= ~(1 << callback_type);
}