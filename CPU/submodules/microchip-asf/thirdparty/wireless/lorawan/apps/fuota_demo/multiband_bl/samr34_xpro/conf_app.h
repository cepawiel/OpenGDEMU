/**
* \file  conf_app.h
*
* \brief LORAWAN Sample Application config file
*
*
* Copyright (c) 2020 Microchip Technology Inc. and its subsidiaries.
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
* THIS SOFTWARE IS SUPPLIED BY MICROCHIP "AS IS".  NO WARRANTIES,
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


#ifndef APP_CONFIG_H_
#define APP_CONFIG_H_

/****************************** INCLUDES **************************************/

/****************************** MACROS **************************************/
/* Select the band for this app */
#define APP_ISMBAND                             (ISM_EU868)
//#define APP_ISMBAND                             (ISM_NA915)
//#define APP_ISMBAND                             (ISM_NZ923)
//#define APP_ISMBAND                             (ISM_IND865)

/* Select the Type of Transmission - Confirmed(CNF) / Unconfirmed(UNCNF) */
#define APP_TRANSMISSION_TYPE                   LORAWAN_UNCNF
//#define APP_TRANSMISSION_TYPE                   LORAWAN_CNF

#define APP_EDCLASS                             (CLASS_A)
//#define APP_EDCLASS                             (CLASS_C)

/* FPORT Value (1-255) */
#define APP_FPORT                               (1)

/* OTAA Join Parameters */
#define DEVICE_EUI                              {0x00, 0x04, 0x25, 0x19, 0x18, 0x01, 0xD7, 0x7F}
#define APPLICATION_EUI                         {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}
#define APPLICATION_KEY                         {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}

/* Other security parameters */
#define APP_GENAPPKEY                           {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}

/* This macro defines the application's uplink timer interval in milliseconds */
#define APP_UPLINKINTERVAL_MS                   (10000)

/* This macro defines the application's default sleep duration in milliseconds */
#define APP_SLEEPTIME_MS                        (PMM_SLEEPTIME_MAX_MS)

/* This macro defines the number of retries to be done for each failed send request */
#define APP_SENDRETRY_COUNT                     (3)

/* This macro defines the timeout in milliseconds to elapse between send retries */
#define APP_SENDRETRYINTERVAL_MS                (6000)

/* This macro defines the custom FUOTA descriptor to be used by FTMPackage */
#define APP_FTM_FUOTA_DESCRIPTOR                (0x2)

#endif /* APP_CONFIG_H_ */
