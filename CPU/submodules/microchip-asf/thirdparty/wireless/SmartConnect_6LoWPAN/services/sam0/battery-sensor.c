/**
 * \file
 *
 * \brief 
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
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include "contiki.h"
#include "battery-sensor.h"
#include "sys/timer.h"
#include "asf.h"
#include "extint.h"
#include "trx_access.h"
//#include "samr21_xplained_pro.h"
// #include ""

/*---------------------------------------------------------------------------*/

extern void battery_sensor_init(void);

static uint16_t get_batmon_voltage(void);

/** Access parameters for sub-register BATMON_HR in register @ref RG_BATMON */
#define SR_BATMON_HR   0x11, 0x10, 4

/** Constant BATMON_HR_HIGH for sub-register @ref SR_BATMON_HR in register
 * BATMON */
#define BATMON_HR_HIGH   (1)

/** Constant BATMON_HR_LOW for sub-register @ref SR_BATMON_HR in register BATMON
**/
#define BATMON_HR_LOW   (0)

/** Constant BATMON_NOT_VALID for sub-register @ref SR_BATMON_OK in register
 * BATMON */
#define BATMON_NOT_VALID   (0)

/** Constant BATMON_VALID for sub-register @ref SR_BATMON_OK in register BATMON
**/
#define BATMON_VALID   (1)


/** Access parameters for sub-register BATMON_OK in register @ref RG_BATMON */
#define SR_BATMON_OK   0x11, 0x20, 5

/** Access parameters for sub-register BATMON_VTH in register @ref RG_BATMON */
#define SR_BATMON_VTH   0x11, 0x0F, 0

/**
 * Supply voltage above upper limit.
 */
#define SUPPLY_VOLTAGE_ABOVE_UPPER_LIMIT    (0xFFFF)

/**
 * Supply voltage below lower limit.
 */
#define SUPPLY_VOLTAGE_BELOW_LOWER_LIMIT    (0)

static int enabled = 0;
/*---------------------------------------------------------------------------*/
static int
battery_sensor_value(int type)
{
	if(enabled)
	{
	return(get_batmon_voltage());
	}else
	{
	return 0xFF;
	}
	
  }
/*---------------------------------------------------------------------------*/

static int
battery_sensor_configure(int type, int value)
{

  if(value == 1) {
	  enabled = 1;
	  } else {
	  /* Disable the sensor */
	  enabled = 0;
  }
  return 0;
}
/*---------------------------------------------------------------------------*/
void
battery_sensor_init(void)
{

  /* on by default */
  battery_sensor_configure(0, 1);

}
/*---------------------------------------------------------------------------*/
static int
battery_sensor_status(int type)
{
  return enabled;
}
/*---------------------------------------------------------------------------*/
SENSORS_SENSOR(battery_sensor, "Battery sensor",
               battery_sensor_value, battery_sensor_configure, battery_sensor_status);
/*---------------------------------------------------------------------------*/


static uint16_t get_batmon_voltage(void)
{
	
	uint8_t vth_val;
	uint16_t mv = 1; /* 1 used as indicator flag */
	bool range;
	DISABLE_TRX_IRQ();
	/* Check if supply voltage is within upper or lower range. */
	trx_bit_write(SR_BATMON_HR, BATMON_HR_HIGH);
	trx_bit_write(SR_BATMON_VTH, 0x00);
	delay_us(5); /* Wait until Batmon has been settled. */
	/* Check if supply voltage is within lower range */
	if (trx_bit_read(SR_BATMON_OK) == BATMON_NOT_VALID) {
		/* Lower range */
		/* Check if supply voltage is below lower limit */
		trx_bit_write(SR_BATMON_HR, BATMON_HR_LOW);
		delay_us(2); /* Wait until Batmon has been settled. */
		if (trx_bit_read(SR_BATMON_OK) == BATMON_NOT_VALID) {
			/* below lower limit */
			mv = SUPPLY_VOLTAGE_BELOW_LOWER_LIMIT;
		}

		range = LOW;
	} else {
		/* Higher range */
		/* Check if supply voltage is above upper limit */
		trx_bit_write(SR_BATMON_VTH, 0x0F);
		delay_us(5); /* Wait until Batmon has been settled. */
		if (trx_bit_read(SR_BATMON_OK) == BATMON_VALID) {
			/* above upper limit */
			mv = SUPPLY_VOLTAGE_ABOVE_UPPER_LIMIT;
		}

		range = HIGH;
	}

	/* Scan through the current range for the matching threshold. */
	if (mv == 1) { /* 1 = indicates that voltage is within supported range
		        **/
		vth_val = 0x0F;
		for (uint8_t i = 0; i < 16; i++) {
			trx_bit_write(SR_BATMON_VTH, vth_val);
			delay_us(2); /* Wait until Batmon has been
			                     * settled. */
			if (trx_bit_read(SR_BATMON_OK) == BATMON_VALID) {
				break;
			}

			vth_val--;
		}

		/* Calculate voltage based on register value and range. */
		if (range == HIGH) {
			mv = 2550 + (75 * vth_val);
		} else {
			mv = 1700 + (50 * vth_val);
		}
	}

	trx_reg_read(0x0F); //Read IRQ Status to clear flag

ENABLE_TRX_IRQ();

	return mv;
}