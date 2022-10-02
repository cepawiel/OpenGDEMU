/**
 * \file
 *
 * \brief ILI9341 display controller component driver default config header
 *
 * Copyright (c) 2014-2018 Microchip Technology Inc. and its subsidiaries.
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
#ifndef CONF_ILI9341_H_INCLUDED
#define CONF_ILI9341_H_INCLUDED

#include <compiler.h>

/**
 * \brief Select a SPI clock speed
 *
 * This selects the clock speed for the SPI clock used to communicate with the
 * display controller. Higher clock speeds allow for higher frame rates.
 * \note That the clock speed may be limited by the speed of the
 * microcontroller a normal limitation would be CPUclk/2. For more details
 * please refer to the device datasheet.
 */
#define CONF_ILI9341_CLOCK_SPEED   8000000UL

/**
 * \name UC3 typical configurations
 * @{
 */

/**
 * \brief Select the correct hardware interface
 *
 * Currently supported interfaces are the SPI interface and the USART Master SPI
 * interface.
 */
#if (AVR32_SPI_NUM == 1)
#  define CONF_ILI9341_SPI         &AVR32_SPI
#else
#  define CONF_ILI9341_SPI         &AVR32_SPI0
#endif
/* #define CONF_ILI9341_USART_SPI    &AVR32_USART0 */

/** \brief Define what MCU pin the ILI9341 chip select pin is connected to */
#define CONF_ILI9341_CS_PIN        AVR32_PIN_PA08

/** \brief Define what MCU pin the ILI9341 DC pin is connected to */
#define CONF_ILI9341_DC_PIN        AVR32_PIN_PB10

/** \brief Define what MCU pin the ILI9341 back light pin is connected to */
#define CONF_ILI9341_BACKLIGHT_PIN AVR32_PIN_PB07

/** \brief Define what MCU pin the ILI9341 reset is connected to */
#define CONF_ILI9341_RESET_PIN     AVR32_PIN_PB06

/** \brief Define what peripheral DMA channel to use for the display */
#define CONF_ILI9341_PDCA_CHANNEL  0

/** \brief Define what peripheral ID to use for the display output */
#if defined(AVR32_PDCA_PID_SPI0_TX)
#  define CONF_ILI9341_PDCA_PID    AVR32_PDCA_PID_SPI0_TX
#else
#  define CONF_ILI9341_PDCA_PID    AVR32_PDCA_PID_SPI_TX
#endif

/** @} */

#endif /* CONF_ILI9341_H_INCLUDED */
