/**
 * \file
 *
 * \brief Atmel Development Board Clock Configuration (ASF)
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
#ifndef CONF_CLOCK_H_INCLUDED
#define CONF_CLOCK_H_INCLUDED

#include <board.h>

#if (BOARD == EVK1100)
#   define USE_PLL
#   define OSC      (0)
#   define PLL_MUL  (10)
#elif (BOARD == EVK1101)
#   define USE_PLL
#   define OSC      (0)
#   define PLL_MUL  (9)
#elif (BOARD == EVK1104)
#   define USE_PLL
#   define OSC      (0)
#   define PLL_MUL  (9)     /* for some reason we can't use 66 Mhz */
#elif (BOARD == EVK1105)
#   define USE_PLL
#   define OSC      (1)
#   define PLL_MUL  (10)
#endif

/**
 * @name System Clock Frequencies
 * @{
 */
#if (OSC == 0)
#   define FOSC     (FOSC0) /* 12 MHz */
#else
#   define FOSC     (FOSC1) /* 11.2896 MHz */
#endif

#ifdef USE_PLL
#   define FMCK_HZ  ((FOSC * (PLL_MUL + 1)) / 2)
#else
#   define FMCK_HZ  (FOSC)
#endif

#define FCPU_HZ     (FMCK_HZ)
#define FHSB_HZ     (FCPU_HZ)
#define FPBB_HZ     (FMCK_HZ)
#define FPBA_HZ     (FMCK_HZ)
//! @}

#define F_CPU       (FPBA_HZ)

#endif /* CONF_CLOCK_H_INCLUDED */
