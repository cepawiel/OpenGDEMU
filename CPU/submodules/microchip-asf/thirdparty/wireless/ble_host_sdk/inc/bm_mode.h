/**
 * \file bm_mode.h
 *
 * \brief BM mode
 *
 * Copyright (c) 2018 Microchip Technology Inc. and its subsidiaries.
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
 */
/*
 * Support and FAQ: visit <a href="https://www.microchip.com/support/">Microchip Support</a>
 */

#ifndef __BM_MODE_H__
#define __BM_MODE_H__

/** \file bm_mode.h
 * Defines BM mode APIs to switch between different
 * BM operation modes such as EEPROM Mode and Application Mode.
 * Defines BM operation mode enumerations.
 */

#include "stdint.h"
 
/*! \enum BM_MODE
 * Enumeration of the BM operation modes.
 */
typedef enum
{
    /*! Flash update mode. */
    BM_MODE_PROGRAM = 0,
    /*! Application mode. */
    BM_MODE_APPLICATION
}BM_MODE;

/*! \fn void BM_MODE_Init(void)
 *  \brief Initializes BM mode configuration library.
 *  \pre None.
 *  \return None.
 */
void BM_MODE_Init(void);

/*! \fn void BM_MODE_Set(BM_MODE mode)
 *  \brief Sets the BM operation mode.
 *  \param mode The BM operation mode to be set.
 *  \pre BM mode initalize function should be invoked prior.
 *  \return None.
 */
void BM_MODE_Set(BM_MODE mode);

/*! \fn void BM_MODE BM_MODE_Get(BM_MODE mode)
 *  \brief Gets the curent BM operation mode.
 *  \param mode Variable to load the current BM operation mode.
 *  \pre BM mode initalize function should be invoked prior.
 *  \return None.
 */
void BM_MODE_Get(BM_MODE mode);

/*! \fn uint16_t BM_ConfigurationLibraryVersion(void)
 *  \brief Gets the version of the BM configuration library.
 *  \pre None.
 *  \return BM Configuration Library version (0xMMmm).
 */
uint16_t BM_ConfigurationLibraryVersion(void);

/* @cond */
#define BM_MODE_Init()                BM_MODULE_ControlInit();

#define BM_MODE_Get(mode)             {                                               \
                                            if(BM_MODULE_ProgramModeGet())             \
                                                mode = BM_MODE_PROGRAM;                \
                                            else if(BM_MODULE_ApplicationModeGet())             \
                                                mode = BM_MODE_APPLICATION;                \
                                        }                                               \

#define BM_MODE_Set(mode)             {                                           \
                                            if(mode == BM_MODE_PROGRAM)            \
                                                BM_MODULE_ProgramModeSet();        \
                                            if(mode == BM_MODE_APPLICATION)       \
                                                BM_MODULE_ApplicationModeSet();   \
                                        }                                           \

/* @endcond */

#endif /* __BM_MODE_H__ */