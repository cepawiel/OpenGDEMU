/**
 *
 * \file
 *
 * \brief SAM D21 WINC3400 configuration.
 *
 * Copyright (c) 2017-2019 Microchip Technology Inc. and its subsidiaries.
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

#ifndef CONF_WINC_H_INCLUDED
#define CONF_WINC_H_INCLUDED

#ifdef __cplusplus
extern "C" {
#endif

#include "board.h"

/*
   ---------------------------------
   ---------- PIN settings ---------
   ---------------------------------
*/

#define CONF_WINC_PIN_RESET			PIN_PB01
#define CONF_WINC_PIN_CHIP_ENABLE		PIN_PB05
#define CONF_WINC_PIN_WAKE				PIN_PB07

/*
   ---------------------------------
   ---------- SPI settings ---------
   ---------------------------------
*/

#define CONF_WINC_USE_SPI				(1)
#define CONF_WINC_USE_EDGE_INTERRUPT	(1)

/** SPI pin and instance settings. */
#define CONF_WINC_SPI_MODULE			EXT1_SPI_MODULE
#define CONF_WINC_SPI_SERCOM_MUX		EXT1_SPI_SERCOM_MUX_SETTING
#define CONF_WINC_SPI_PINMUX_PAD0		EXT1_SPI_SERCOM_PINMUX_PAD0 /* in */
#define CONF_WINC_SPI_PINMUX_PAD1		PINMUX_UNUSED /* cs driven from software */
#define CONF_WINC_SPI_PINMUX_PAD2		EXT1_SPI_SERCOM_PINMUX_PAD2 /* out */
#define CONF_WINC_SPI_PINMUX_PAD3		EXT1_SPI_SERCOM_PINMUX_PAD3 /* sck */
#define CONF_WINC_SPI_CS_PIN			EXT1_PIN_SPI_SS_0

#define CONF_WINC_SPI_MISO				EXT1_PIN_SPI_MISO
#define CONF_WINC_SPI_MOSI				EXT1_PIN_SPI_MOSI
#define CONF_WINC_SPI_SCK				EXT1_PIN_SPI_SCK
#define CONF_WINC_SPI_SS				EXT1_PIN_SPI_SS_0

/*
   ---------------------------------
   ---------- I2C settings ---------
   ---------------------------------
*/
#define CONF_WIFI_M2M_I2C                    (0)

#define CONF_WINC_I2C_SCL                  EXT1_PIN_I2C_SCL
#define CONF_WINC_I2C_SDA                  EXT1_PIN_I2C_SDA

/** WINC interrupt pin. */
#define CONF_WINC_SPI_INT_PIN			EXT1_IRQ_PIN
#define CONF_WINC_SPI_INT_MUX			EXT1_IRQ_MUX
#define CONF_WINC_SPI_INT_EIC			(4)

/** SPI clock. */
#define CONF_WINC_SPI_CLOCK				(10000000)

/*
   ---------------------------------
   --------- Debug Options ---------
   ---------------------------------
*/

#define CONF_WINC_DEBUG					(1)
#define CONF_WINC_PRINTF				printf

#ifdef __cplusplus
}
#endif

#endif /* CONF_WINC_H_INCLUDED */
