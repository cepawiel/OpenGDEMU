/**
 * \file tfa.c
 *
 * \brief Implementation of Transceiver Feature Access (TFA) functionality.
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

/*
 * Copyright (c) 2013-2018 Microchip Technology Inc. and its subsidiaries.
 *
 * Licensed under Atmel's Limited License Agreement --> EULA.txt
 */

#if (defined ENABLE_TFA) || (defined TFA_BAT_MON)

/* === INCLUDES ============================================================ */

#include <stdint.h>
#include <string.h>
#include <stdbool.h>
#include <stdlib.h>
#include "pal.h"
#include "return_val.h"
#include "tal.h"
#include "tal_irq_handler.h"
#include "tal_constants.h"
#include "tal_internal.h"
#include "ieee_const.h"
#include "atmega256rfr2.h"
#include "tfa.h"

/* === TYPES =============================================================== */

/* === MACROS ============================================================== */

/* Constant define for the ED scaling: register value at -35dBm */
#define CLIP_VALUE_REG                  (55)

/* Number of ADC samples used for temperature measurement averaging */
#define NUM_SAMPLES                     16

/* === GLOBALS ============================================================= */

#ifdef ENABLE_TFA

/**
 * TFA PIB attribute to reduce the Rx sensitivity.
 * Represents the Rx sensitivity value in dBm; example: -52
 */
static int8_t tfa_pib_rx_sens;

/**
 * Pin State
 */
/* Please dont change this type. */
typedef enum pin_state_tag {
	/** Pin state low */
	LOW,
	/** Pin state high */
	HIGH
} SHORTENUM pin_state_t;

#endif

/* === PROTOTYPES ========================================================== */

#ifdef ENABLE_TFA
static void init_tfa_pib(void);
static void write_all_tfa_pibs_to_trx(void);

#endif

/* === IMPLEMENTATION ====================================================== */

#ifdef ENABLE_TFA

/*
 * \brief Gets a TFA PIB attribute
 *
 * This function is called to retrieve the transceiver information base
 * attributes.
 *
 * \param[in] tfa_pib_attribute TAL infobase attribute ID
 * \param[out] value TFA infobase attribute value
 *
 * \return MAC_UNSUPPORTED_ATTRIBUTE if the TFA infobase attribute is not found
 *         MAC_SUCCESS otherwise
 */
retval_t tfa_pib_get(tfa_pib_t tfa_pib_attribute, void *value)
{
	switch (tfa_pib_attribute) {
	case TFA_PIB_RX_SENS:
		*(uint8_t *)value = tfa_pib_rx_sens;
		break;

	default:
		/* Invalid attribute id */
		return MAC_UNSUPPORTED_ATTRIBUTE;
	}

	return MAC_SUCCESS;
}

#endif

#ifdef ENABLE_TFA

/*
 * \brief Sets a TFA PIB attribute
 *
 * This function is called to set the transceiver information base
 * attributes.
 *
 * \param[in] tfa_pib_attribute TFA infobase attribute ID
 * \param[in] value TFA infobase attribute value to be set
 *
 * \return MAC_UNSUPPORTED_ATTRIBUTE if the TFA info base attribute is not found
 *         TAL_BUSY if the TAL is not in TAL_IDLE state.
 *         MAC_SUCCESS if the attempt to set the PIB attribute was successful
 */
retval_t tfa_pib_set(tfa_pib_t tfa_pib_attribute, void *value)
{
	switch (tfa_pib_attribute) {
	case TFA_PIB_RX_SENS:
	{
		uint8_t reg_val;

		tfa_pib_rx_sens = *((int8_t *)value);
		if (tfa_pib_rx_sens > -49) {
			reg_val = 0xF;
			tfa_pib_rx_sens = -49;
		} else if (tfa_pib_rx_sens <= RSSI_BASE_VAL_DBM) {
			reg_val = 0x0;
			tfa_pib_rx_sens = RSSI_BASE_VAL_DBM;
		} else {
			reg_val
				= ((tfa_pib_rx_sens -
					(RSSI_BASE_VAL_DBM)) / 3) + 1;
		}

		trx_bit_write(SR_RX_PDT_LEVEL, reg_val);
	}
	break;

	default:
		/* Invalid attribute id */
		return MAC_UNSUPPORTED_ATTRIBUTE;
	}

	return MAC_SUCCESS;
}

#endif

#ifdef ENABLE_TFA

/*
 * \brief Initializes the TFA
 *
 * This function is called to initialize the TFA.
 *
 * \return MAC_SUCCESS if everything went correct;
 *         FAILURE otherwise
 */
retval_t tfa_init(void)
{
	init_tfa_pib();
	write_all_tfa_pibs_to_trx();
	sysclk_enable_peripheral_clock(&ADC);
	return MAC_SUCCESS;
}

#endif

#ifdef ENABLE_TFA

/*
 * \brief Reset the TFA
 *
 * This function is called to reset the TFA.
 *
 * \param set_default_pib Defines whether PIB values need to be set
 *                        to its default values
 */
void tfa_reset(bool set_default_pib)
{
	if (set_default_pib) {
		init_tfa_pib();
	}

	write_all_tfa_pibs_to_trx();
}

#endif

#ifdef ENABLE_TFA

/*
 * \brief Perform a CCA
 *
 * This function performs a CCA request.
 *
 * \return phy_enum_t PHY_IDLE or PHY_BUSY
 */
phy_enum_t tfa_cca_perform(void)
{
	tal_trx_status_t trx_status;
	uint8_t cca_status;
	uint8_t cca_done;

	/* Ensure that trx is not in SLEEP for register access */
	do {
		trx_status = set_trx_state(CMD_TRX_OFF);
	} while (trx_status != TRX_OFF);

	/* no interest in receiving frames while doing CCA */
	trx_bit_write(SR_RX_PDT_DIS, RX_DISABLE); /* disable frame reception
	                                           * indication */

	/* Set trx to rx mode. */
	do {
		trx_status = set_trx_state(CMD_RX_ON);
	} while (trx_status != RX_ON);

	/* Start CCA */
	trx_bit_write(SR_CCA_REQUEST, CCA_START);

	/* wait until CCA is done */
	pal_timer_delay(TAL_CONVERT_SYMBOLS_TO_US(CCA_DURATION_SYM));
	do {
		/* poll until CCA is really done */
		cca_done = trx_bit_read(SR_CCA_DONE);
	} while (cca_done != CCA_COMPLETED);

	set_trx_state(CMD_TRX_OFF);

	/* Check if channel was idle or busy. */
	if (trx_bit_read(SR_CCA_STATUS) == CCA_CH_IDLE) {
		cca_status = PHY_IDLE;
	} else {
		cca_status = PHY_BUSY;
	}

	/* Enable frame reception again. */
	trx_bit_write(SR_RX_PDT_DIS, RX_ENABLE);

	return (phy_enum_t)cca_status;
}

#endif

#ifdef ENABLE_TFA

/*
 * \brief Perform a single ED measurement
 *
 * \return ed_value Result of the measurement
 *         If the build switch TRX_REG_RAW_VALUE is defined, the transceiver's
 *         register value is returned.
 */
uint8_t tfa_ed_sample(void)
{
	trx_irq_reason_t trx_irq_cause;
	uint8_t ed_value;
	tal_trx_status_t trx_status;

	/* Make sure that receiver is switched on. */
	do {
		trx_status = set_trx_state(CMD_RX_ON);
	} while (trx_status != RX_ON);

	/*
	 * Disable the transceiver interrupts to prevent frame reception
	 * while performing ED scan.
	 */
	trx_bit_write(SR_RX_PDT_DIS, RX_DISABLE);

	/* Write dummy value to start measurement. */
	trx_reg_write(RG_PHY_ED_LEVEL, 0xFF);

	/* Wait for ED measurement completion. */
	pal_timer_delay(TAL_CONVERT_SYMBOLS_TO_US(ED_SAMPLE_DURATION_SYM));
	do {
		trx_irq_cause
			= (trx_irq_reason_t)trx_reg_read(RG_IRQ_STATUS);
	} while ((trx_irq_cause & TRX_IRQ_CCA_ED_READY) !=
			TRX_IRQ_CCA_ED_READY);

	/* Read the ED Value. */
	ed_value = trx_reg_read(RG_PHY_ED_LEVEL);

#ifndef TRX_REG_RAW_VALUE

	/*
	 * Scale ED result.
	 * Clip values to 0xFF if > -35dBm
	 */
	if (ed_value > CLIP_VALUE_REG) {
		ed_value = 0xFF;
	} else {
		ed_value
			= (uint8_t)(((uint16_t)ed_value *
				0xFF) / CLIP_VALUE_REG);
	}
#endif

	/* Clear IRQ register */
	trx_reg_read(RG_IRQ_STATUS);
	/* Enable reception agian */
	trx_bit_write(SR_RX_PDT_DIS, RX_ENABLE);
	/* Switch receiver off again */
	set_trx_state(CMD_TRX_OFF);

	return ed_value;
}

#endif

#if (defined ENABLE_TFA) || (defined TFA_BAT_MON)

/*
 * \brief Get the transceiver's supply voltage
 *
 * \return mv Milli Volt; 0 if below threshold, 0xFFFF if above threshold
 */
uint16_t tfa_get_batmon_voltage(void)
{
	tal_trx_status_t previous_trx_status;
	uint8_t vth_val;
	uint8_t i;
	uint16_t mv = 1; /* 1 used as indicator flag */
	bool range;

	previous_trx_status = tal_trx_status;
	if (tal_trx_status == TRX_SLEEP) {
		set_trx_state(CMD_TRX_OFF);
	}

	/*
	 * Disable all trx interrupts.
	 * This needs to be done AFTER the transceiver has been woken up.
	 */
	trx_reg_write(RG_IRQ_MASK, TRX_IRQ_NONE);

	/* Check if supply voltage is within lower range */
	trx_bit_write(SR_BATMON_HR, BATMON_LOW_RANGE);
	trx_bit_write(SR_BATMON_VTH, 0x0F);
	pal_timer_delay(5); /* Wait until Batmon has been settled. */
	if (trx_bit_read(SR_BATMON_OK) == BATMON_BELOW_THRES) {
		/* Lower range */
		/* Check if supply voltage is below lower limit */
		trx_bit_write(SR_BATMON_VTH, 0);
		pal_timer_delay(2); /* Wait until Batmon has been settled. */
		if (trx_bit_read(SR_BATMON_OK) == BATMON_BELOW_THRES) {
			/* below lower limit */
			mv = SUPPLY_VOLTAGE_BELOW_LOWER_LIMIT;
		}

		range = LOW;
	} else {
		/* Higher range */
		trx_bit_write(SR_BATMON_HR, BATMON_HIGH_RANGE);
		/* Check if supply voltage is above upper limit */
		trx_bit_write(SR_BATMON_VTH, 0x0F);
		pal_timer_delay(5); /* Wait until Batmon has been settled. */
		if (trx_bit_read(SR_BATMON_OK) == BATMON_ABOVE_THRES) {
			/* above upper limit */
			mv = SUPPLY_VOLTAGE_ABOVE_UPPER_LIMIT;
		}

		range = HIGH;
	}

	/* Scan through the current range for the matching threshold. */
	if (mv == 1) {
		vth_val = 0x0F;
		for (i = 0; i < 16; i++) {
			trx_bit_write(SR_BATMON_VTH, i);
			pal_timer_delay(2); /* Wait until Batmon has been
			                     * settled. */
			if (trx_bit_read(SR_BATMON_OK) ==
					BATMON_BELOW_THRES) {
				if (i > 0) {
					vth_val = i - 1;
				} else {
					vth_val = i;
				}

				break;
			}
		}

		if (range == HIGH) {
			mv = 2550 + (75 * vth_val);
		} else {
			mv = 1700 + (50 * vth_val);
		}
	}

	trx_reg_read(RG_IRQ_STATUS);

	/* Clear all pending interrupts. */
	pal_trx_irq_flag_clr_rx_end();
	pal_trx_irq_flag_clr_tx_end();
	pal_trx_irq_flag_clr_tstamp();

	/*
	 * Enable all trx interrupts.
	 * This needs to be done BEFORE putting the transceiver back to slee.
	 */
	trx_reg_write(RG_IRQ_MASK, TRX_IRQ_DEFAULT);

	if (previous_trx_status == TRX_SLEEP) {
		set_trx_state(CMD_SLEEP);
	}

	return mv;
}

#endif  /* #if (defined ENABLE_TFA) || (defined TFA_BAT_MON) */

#ifdef ENABLE_TFA

/**
 * \brief Initialize the TFA PIB
 *
 * This function initializes the TFA information base attributes
 * to their default values.
 * \ingroup group_tfa
 */
static void init_tfa_pib(void)
{
	tfa_pib_rx_sens = TFA_PIB_RX_SENS_DEF;
}

#endif

#ifdef ENABLE_TFA

/**
 * \brief Write all shadow PIB variables to the transceiver
 *
 * This function writes all shadow PIB variables to the transceiver.
 * It is assumed that the radio does not sleep.
 * \ingroup group_tfa
 */
static void write_all_tfa_pibs_to_trx(void)
{
	tfa_pib_set(TFA_PIB_RX_SENS, (void *)&tfa_pib_rx_sens);
}

#endif

#ifdef ENABLE_TFA

/*
 * \brief Get the temperature value from the integrated sensor
 *
 * \return temperature value in degree Celsius
 */
double tfa_get_temperature(void)
{
	uint32_t F_CPU_VAL;
	F_CPU_VAL = F_CPU;
	float temp_result;
	float offset;
	double result;
#if NUM_SAMPLES > 255
	uint16_t i;
#else
	uint8_t i;
#endif
	uint16_t adc_value;

	uint32_t temp_result32 = 0;
	int32_t offset32 = 0;

	ADCSRA = (1 << ADEN); /* Enable ADC */

	/*
	 * Analog channel and gain selection
	 * The MUX5 bit has to written first followed by a
	 * write access to the MUX4:0 bits which triggers the update of the
	 * internal buffer.
	 */
	ADCSRB = (1 << MUX5);

	/*
	 * Select internal 1.6V reference voltage
	 * Select temperature sensor
	 */
	ADMUX = (1 << REFS1) | (1 << REFS0) | (1 << MUX3) | (1 << MUX0);

	/* Dummy conversion to clear PGA */
	if ((F_CPU_VAL == 16000000UL) || (F_CPU_VAL == 15384600UL)) {
		ADCSRA
			= (1 <<
				ADEN) |
				(1 <<
				ADSC) |
				(1 << ADIF) | (1 << ADPS2) | (1 << ADPS0);
	} else if (F_CPU_VAL == 8000000UL) {
		ADCSRA = (1 << ADEN) | (1 << ADSC) | (1 << ADIF) | (1 << ADPS2);
	} else if (F_CPU_VAL == 4000000UL) {
		ADCSRA
			= (1 <<
				ADEN) |
				(1 <<
				ADSC) |
				(1 << ADIF) | (1 << ADPS1) | (1 << ADPS0);
	} else {
	}

	/* Wait for conversion to be completed */
	do {
	} while ((ADCSRA & (1 << ADIF)) == 0x00);

	/* Sample */
	for (i = 0; i < NUM_SAMPLES; i++) {
		ADCSRA |= (1 << ADEN) | (1 << ADSC) | (1 << ADIF);

		/*
		 * ADC Control and Status Register A:
		 * ADC Enable
		 * ADC Start Conversion
		 * Clear ADIF
		 * Prescaler = 32 (500kHz) for 16 MHz main clock
		 */

		/* Wait for conversion to be completed */
		do {
		} while ((ADCSRA & (1 << ADIF)) == 0x00);

		adc_value = ADC;

		/* Averaging */
		temp_result32 += adc_value;
	}
	temp_result = (float)temp_result32 / NUM_SAMPLES;

	ADCSRA ^= (1 << ADEN); /* Disable ADC for channel change */

	/* Get offset value */
	ADCSRB = 0x00;
	ADMUX = (1 << REFS1) | (1 << REFS0) | (1 << MUX3);

	for (i = 0; i < NUM_SAMPLES; i++) {
		ADCSRA |= (1 << ADEN) | (1 << ADSC) | (1 << ADIF);

		/* Wait for conversion to be completed */
		do {
		} while ((ADCSRA & (1 << ADIF)) == 0x00);

		adc_value = ADC;

		/* Averaging */
		if (adc_value > 0x1FF) {
			offset32 -= 0x400 - adc_value;
		} else {
			offset32 += adc_value;
		}
	}
	offset = (float)offset32 / NUM_SAMPLES;

	ADCSRA &= ~(1 << ADEN); /* Disable ADC */

	temp_result -= offset;
	result = ((double)temp_result * 1.13) - 272.8;
	F_CPU_VAL = F_CPU_VAL; /* Ignore Warnings */
	return result;
}

#endif

#ifdef ENABLE_TFA

/*
 * \brief Starts continuous transmission on current channel
 *
 * \param tx_mode Mode of continuous transmission (CW or PRBS)
 * \param random_content Use random content if true
 */
void tfa_continuous_tx_start(continuous_tx_mode_t tx_mode, bool random_content)
{
	uint8_t txcwdata[128];

	trx_bit_write(SR_TX_AUTO_CRC_ON, TX_AUTO_CRC_DISABLE);
	trx_reg_write(RG_TRX_STATE, CMD_TRX_OFF);
	trx_reg_write(0x0176, 0x0F); /*TST_CTRL_DIGI*/

	/* Here: use 2MBPS mode for PSD measurements.
	 * Omit the two following lines, if 250k mode is desired for PRBS mode.
	 **/
	trx_reg_write(RG_TRX_CTRL_2, 0x03);
	trx_reg_write(RG_RX_CTRL, 0xA7);
	if (tx_mode == CW_MODE) {
		txcwdata[0] = 1; /* length */
		/* Step 11 - frame buffer write access */
		txcwdata[1] = 0x00; /* f=fch-0.5 MHz; set value to 0xFF for
		                     * f=fch+0.5MHz */
		trx_frame_write(txcwdata, 2);
	} else { /* PRBS mode */
		txcwdata[0] = 127; /* = max length */
		for (uint8_t i = 1; i < 128; i++) {
			if (random_content) {
				txcwdata[i] = (uint8_t)rand();
			} else {
				txcwdata[i] = 0;
			}
		}
		trx_frame_write(txcwdata, 128);
	}

	trx_reg_write(RG_PART_NUM, 0x54);
	trx_reg_write(RG_PART_NUM, 0x46);
	set_trx_state(CMD_PLL_ON);
	TRX_SLP_TR_HIGH();
	TRX_SLP_TR_LOW();
}

#endif

#ifdef ENABLE_TFA

/*
 * \brief Stops continuous transmission
 */
void tfa_continuous_tx_stop(void)
{
	tal_reset(false);
}

#endif

#endif /* #if (defined ENABLE_TFA) || (defined TFA_BAT_MON) */

/* EOF */
