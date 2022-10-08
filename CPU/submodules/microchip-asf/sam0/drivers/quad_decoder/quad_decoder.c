/**
 * \file
 *
 * \brief SAM QUAD DECODER Driver for SAMB11
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
#include "quad_decoder.h"

static quad_decoder_callback_t quad_decoder0_callback = NULL;
static quad_decoder_callback_t quad_decoder1_callback = NULL;
static quad_decoder_callback_t quad_decoder2_callback = NULL;

/**
 * \brief Initializes config with predefined default values.
 *
 * This function will initialize a given QUAD DECODER configuration structure to
 * a set of known default values. This function should be called on
 * any new instance of the configuration structures before being
 * modified by the user application.
 *
 * The default configuration is as follows:
 *  \li Select input clock as 26MHz
 *  \li Set counter upper threshold as 32767
 *  \li Set counter lower threshold as -32768
 *  \li Set pinmux_pad[0] as 0
 *  \li Set pinmux_pad[1] as 0
 *
 * \param[out]  config  Pointer to a QUAD DECODER module configuration structure to set
 */
void quad_decoder_get_config_defaults(struct quad_decoder_config *config)
{
	/* Axis X */
	config->qdec0.clock_sel = QDEC_CLK_INPUT_0;
	config->qdec0.threshold_upper = 32767;
	config->qdec0.threshold_lower = -32768;
	config->qdec0.pin_number_pad[0] = 0;
	config->qdec0.pin_number_pad[1] = 0;
	config->qdec0.pinmux_sel_pad[0] = 0;
	config->qdec0.pinmux_sel_pad[1] = 0;
	config->qdec_enalbe = (1 << QDEC_AXIS_X);
	/* Axis Y */
	config->qdec1.clock_sel = QDEC_CLK_INPUT_0;
	config->qdec1.threshold_upper = 32767;
	config->qdec1.threshold_lower = -32768;
	config->qdec1.pin_number_pad[0] = 0;
	config->qdec1.pin_number_pad[1] = 0;
	config->qdec1.pinmux_sel_pad[0] = 0;
	config->qdec1.pinmux_sel_pad[1] = 0;
	config->qdec_enalbe |= (1 << QDEC_AXIS_Y);
	/* Axis Z */
	config->qdec2.clock_sel = QDEC_CLK_INPUT_0;
	config->qdec2.threshold_upper = 32767;
	config->qdec2.threshold_lower = -32768;
	config->qdec2.pin_number_pad[0] = 0;
	config->qdec2.pin_number_pad[1] = 0;
	config->qdec2.pinmux_sel_pad[0] = 0;
	config->qdec2.pinmux_sel_pad[1] = 0;
	config->qdec_enalbe |= (1 << QDEC_AXIS_Z);
}

/**
 * \brief Quad Decoder ISR handler.
 *
 * Quad Decoder ISR handler.
 *
 */
static void quad_decoder_isr_handler(void)
{
	uint8_t status = LPMCU_MISC_REGS0->QUAD_DEC_IRQS.reg;
	
	if (status & LPMCU_MISC_REGS_QUAD_DEC_IRQS_QUAD_DEC0_IRQ) {
		LPMCU_MISC_REGS0->QUAD_DEC0_CTRL.reg |=
				LPMCU_MISC_REGS_QUAD_DEC0_CTRL_CLR_IRQ;
		if (quad_decoder0_callback) {
			quad_decoder0_callback();
		}
	}
	
	if (status & LPMCU_MISC_REGS_QUAD_DEC_IRQS_QUAD_DEC1_IRQ) {
		LPMCU_MISC_REGS0->QUAD_DEC1_CTRL.reg |=
				LPMCU_MISC_REGS_QUAD_DEC1_CTRL_CLR_IRQ;
		if (quad_decoder1_callback) {
			quad_decoder1_callback();
		}
	}

	if (status & LPMCU_MISC_REGS_QUAD_DEC_IRQS_QUAD_DEC2_IRQ) {
		LPMCU_MISC_REGS0->QUAD_DEC2_CTRL.reg |=
				LPMCU_MISC_REGS_QUAD_DEC2_CTRL_CLR_IRQ;
		if (quad_decoder2_callback) {
			quad_decoder2_callback();
		}
	}
}

/**
 * \brief Initializes QUAD DECODER module instance.
 *
 * Initializes the QUAD DECODER module, based on the given
 * configuration values.
 *
 * \param[in]     qdec         Axis(x,y,z) of QUAD DECODER instance
 * \param[in]     config       Pointer to the QAUD DECODER configuration options struct
 */
void quad_decoder_init(const struct quad_decoder_config *config)
{
	if (config->qdec_enalbe & (1 << QDEC_AXIS_X)) {
		system_peripheral_reset(PERIPHERAL_QDEC0);
		LPMCU_MISC_REGS0->QUAD_DEC0_CTRL.bit.CLOCK_SEL  = config->qdec0.clock_sel;
		LPMCU_MISC_REGS0->QUAD_DEC0_THRESHOLD.bit.UPPER = config->qdec0.threshold_upper;
		LPMCU_MISC_REGS0->QUAD_DEC0_THRESHOLD.bit.LOWER = config->qdec0.threshold_lower;
		gpio_pinmux_cofiguration(config->qdec0.pin_number_pad[0],
				(uint16_t)(config->qdec0.pinmux_sel_pad[0]));
		gpio_pinmux_cofiguration(config->qdec0.pin_number_pad[1],
				(uint16_t)(config->qdec0.pinmux_sel_pad[1]));
		quad_decoder_enable(QDEC_AXIS_X);
	}
	if (config->qdec_enalbe & (1 << QDEC_AXIS_Y)) {
		system_peripheral_reset(PERIPHERAL_QDEC1);
		LPMCU_MISC_REGS0->QUAD_DEC1_CTRL.bit.CLOCK_SEL  = config->qdec1.clock_sel;
		LPMCU_MISC_REGS0->QUAD_DEC1_THRESHOLD.bit.UPPER = config->qdec1.threshold_upper;
		LPMCU_MISC_REGS0->QUAD_DEC1_THRESHOLD.bit.LOWER = config->qdec1.threshold_lower;
		gpio_pinmux_cofiguration(config->qdec1.pin_number_pad[0],
				(uint16_t)(config->qdec1.pinmux_sel_pad[0]));
		gpio_pinmux_cofiguration(config->qdec1.pin_number_pad[1],
				(uint16_t)(config->qdec1.pinmux_sel_pad[1]));
		quad_decoder_enable(QDEC_AXIS_Y);
	}
	if (config->qdec_enalbe & (1 << QDEC_AXIS_Z)) {
		system_peripheral_reset(PERIPHERAL_QDEC2);
		LPMCU_MISC_REGS0->QUAD_DEC2_CTRL.bit.CLOCK_SEL  = config->qdec2.clock_sel;
		LPMCU_MISC_REGS0->QUAD_DEC2_THRESHOLD.bit.UPPER = config->qdec2.threshold_upper;
		LPMCU_MISC_REGS0->QUAD_DEC2_THRESHOLD.bit.LOWER = config->qdec2.threshold_lower;
		gpio_pinmux_cofiguration(config->qdec2.pin_number_pad[0],
				(uint16_t)(config->qdec2.pinmux_sel_pad[0]));
		gpio_pinmux_cofiguration(config->qdec2.pinmux_sel_pad[1],
				(uint16_t)(config->qdec2.pinmux_sel_pad[1]));
		quad_decoder_enable(QDEC_AXIS_Z);
	}
	LPMCU_MISC_REGS0->IRQ_MUX_IO_SEL_4.bit.MUX_18 =  LPMCU_MISC_REGS_IRQ_MUX_IO_SEL_4_MUX_18_19_Val;
	system_register_isr(33, (uint32_t)quad_decoder_isr_handler);
}

/**
 * \brief Enable QUAD DECODER module instance.
 *
 * Enable the QUAD DECODER module instance, based on the given axis
 *
 * \param[in]     qdec         Axis(x,y,z) of QUAD DECODER instance
 */
void quad_decoder_enable(enum quad_decoder_axis qdec)
{
	switch (qdec) {
	case QDEC_AXIS_X:
		LPMCU_MISC_REGS0->LPMCU_CLOCK_ENABLES_0.reg |=
				LPMCU_MISC_REGS_LPMCU_CLOCK_ENABLES_0_QUAD_DEC0_CLK_EN;
		LPMCU_MISC_REGS0->QUAD_DEC0_CTRL.reg |=
				LPMCU_MISC_REGS_QUAD_DEC0_CTRL_ENABLE;
		break;
	case QDEC_AXIS_Y:
		LPMCU_MISC_REGS0->LPMCU_CLOCK_ENABLES_0.reg |=
				LPMCU_MISC_REGS_LPMCU_CLOCK_ENABLES_0_QUAD_DEC1_CLK_EN;
		LPMCU_MISC_REGS0->QUAD_DEC1_CTRL.reg |=
				LPMCU_MISC_REGS_QUAD_DEC1_CTRL_ENABLE;
		break;
	case QDEC_AXIS_Z:
		LPMCU_MISC_REGS0->LPMCU_CLOCK_ENABLES_0.reg |=
				LPMCU_MISC_REGS_LPMCU_CLOCK_ENABLES_0_QUAD_DEC2_CLK_EN;
		LPMCU_MISC_REGS0->QUAD_DEC2_CTRL.reg |=
				LPMCU_MISC_REGS_QUAD_DEC2_CTRL_ENABLE;
		break;
	}
}

/**
 * \brief Disable QUAD DECODER module instance.
 *
 * Disable the QUAD DECODER module instance, based on the given axis
 *
 * \param[in]     qdec         Axis(x,y,z) of QUAD DECODER instance
 */
void quad_decoder_disable(enum quad_decoder_axis qdec)
{
	switch (qdec) {
	case QDEC_AXIS_X:
		LPMCU_MISC_REGS0->LPMCU_CLOCK_ENABLES_0.reg &=
				~LPMCU_MISC_REGS_LPMCU_CLOCK_ENABLES_0_QUAD_DEC0_CLK_EN;
		LPMCU_MISC_REGS0->QUAD_DEC0_CTRL.reg &=
				~LPMCU_MISC_REGS_QUAD_DEC0_CTRL_ENABLE;
		break;
	case QDEC_AXIS_Y:
		LPMCU_MISC_REGS0->LPMCU_CLOCK_ENABLES_0.reg &=
				~LPMCU_MISC_REGS_LPMCU_CLOCK_ENABLES_0_QUAD_DEC1_CLK_EN;
		LPMCU_MISC_REGS0->QUAD_DEC1_CTRL.reg &=
				~LPMCU_MISC_REGS_QUAD_DEC1_CTRL_ENABLE;
		break;
	case QDEC_AXIS_Z:
		LPMCU_MISC_REGS0->LPMCU_CLOCK_ENABLES_0.reg &=
				~LPMCU_MISC_REGS_LPMCU_CLOCK_ENABLES_0_QUAD_DEC2_CLK_EN;
		LPMCU_MISC_REGS0->QUAD_DEC2_CTRL.reg &=
				~LPMCU_MISC_REGS_QUAD_DEC2_CTRL_ENABLE;
		break;
	}
}

/**
 * \brief Get QUAD DECODER current counter
 *
 * Get the current counter of quad decoder, based on the given axis
 *
 * \param[in]     qdec         Axis(x,y,z) of QUAD DECODER instance
 *
 * \return The current counter of quad decoder instance
 */
int16_t quad_decoder_get_counter(enum quad_decoder_axis qdec)
{
	switch (qdec) {
	case QDEC_AXIS_X:
		return LPMCU_MISC_REGS0->QUAD_DEC0_STATUS.bit.COUNT;
	case QDEC_AXIS_Y:
		return LPMCU_MISC_REGS0->QUAD_DEC1_STATUS.bit.COUNT;
	case QDEC_AXIS_Z:
		return LPMCU_MISC_REGS0->QUAD_DEC2_STATUS.bit.COUNT;
	}
	return 0;
}

/**
 * \brief Registers a callback.
 *
 * Registers and enable a callback function which is implemented by the user.
 *
 * \param[in]     callback_func Pointer to callback function
 */
void quad_decoder_register_callback(enum quad_decoder_axis qdec, quad_decoder_callback_t fun)
{
	switch (qdec) {
	case QDEC_AXIS_X:
		quad_decoder0_callback = fun;
		break;
	case QDEC_AXIS_Y:
		quad_decoder1_callback = fun;
		break;
	case QDEC_AXIS_Z:
		quad_decoder2_callback = fun;
		break;
	}
}

/**
 * \brief Unregisters a callback.
 *
 * Unregisters and disable a callback function implemented by the user.
 *
 */
void quad_decoder_unregister_callback(enum quad_decoder_axis qdec)
{
	switch (qdec) {
	case QDEC_AXIS_X:
		quad_decoder0_callback = NULL;
		break;
	case QDEC_AXIS_Y:
		quad_decoder1_callback = NULL;
		break;
	case QDEC_AXIS_Z:
		quad_decoder2_callback = NULL;
		break;
	}
}