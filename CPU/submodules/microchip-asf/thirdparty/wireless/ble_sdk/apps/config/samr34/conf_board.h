/**
 * \file  conf_board.h
 *
 * \brief SAM R34 Xplained Pro board configuration.
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
 * Support and FAQ: visit <a href="http://www.atmel.com/design-support/">Atmel Support</a>
 */

#ifndef CONF_BOARD_H_INCLUDED
#define CONF_BOARD_H_INCLUDED


/*********** DIO Config **********/
/*Enable the required DIO's*/
#define ENABLE_DIO0
#define ENABLE_DIO1
#define ENABLE_DIO2
/*********************************/

/* RF Switch Enable */
#define RFSWITCH_ENABLE

/******** TCXO Changes***********/
/* Possible Radio clock sources */
#define TCXO_OSC                0
#define XTAL_OSC                1

/* Clock source for SX1276 radio */
#define RADIO_CLK_SRC        TCXO_OSC

#if (RADIO_CLK_SRC == TCXO_OSC)
#define TCXO_ENABLE
#endif

/* Delay in ms for TCXO to stabilize */
#define RADIO_CLK_STABILITATION_DELAY       2
/**********************************/

/************** RSSI Config *******/
// For freq above 862Mhz the RSSI offset to be used
#define RSSI_HF_OFFSET				-157

// For freq below 862Mhz the RSSI offset to be used
#define RSSI_LF_OFFSET				-164
/**********************************/


/* Enable if OLED1 Xplained Pro is connected */
//#define EXT_BOARD_OLED1_XPLAINED_PRO

//#define CONF_BOARD_AT86RFX

//#define MCU_SOC_NAME        "ATSAMR30G18A"

//#define AT86RFX_SPI_BAUDRATE             4000000UL

#if (defined EXT_BOARD_OLED1_XPLAINED_PRO)

#define ENABLE_LCD

#define WING_BUTTON_1 EXT1_PIN_9
#define WING_BUTTON_2 EXT1_PIN_3
#define WING_BUTTON_3 EXT1_PIN_4

#endif

#if defined (ENABLE_LCD)
#include "lcd.h"
#endif

#endif /* CONF_BOARD_H_INCLUDED */
