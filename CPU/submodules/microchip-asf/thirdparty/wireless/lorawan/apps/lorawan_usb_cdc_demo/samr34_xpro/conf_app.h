/**
* \file  conf_app.h
*
* \brief Getting Started LoRAWAN [USB_CDC] Demo Application
*
*
* Copyright (c) 2019-2020 Microchip Technology Inc. and its subsidiaries.
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
/* Number of software timers */
#define TOTAL_NUMBER_OF_TIMERS            (25u)


/*Define the Sub band of Channels to be enabled by default for the application*/
/* Only necessary for regions that have defined subbands like NA915 and AU915 */
#define SUBBAND 2
#if ((SUBBAND < 1 ) || (SUBBAND > 8 ) )
#error " Invalid Value of Subband"
#endif


/* Activation method constants */
#define OVER_THE_AIR_ACTIVATION           LORAWAN_OTAA
#define ACTIVATION_BY_PERSONALIZATION     LORAWAN_ABP

/* Message Type constants */
#define UNCONFIRMED                       LORAWAN_UNCNF
#define CONFIRMED                         LORAWAN_CNF

/* Enable one of the activation methods */
#define DEMO_APP_ACTIVATION_TYPE               OVER_THE_AIR_ACTIVATION
//#define DEMO_APP_ACTIVATION_TYPE               ACTIVATION_BY_PERSONALIZATION

/* Select the Type of Transmission - Confirmed(CNF) / Unconfirmed(UNCNF) */
#define DEMO_APP_TRANSMISSION_TYPE              UNCONFIRMED
//#define DEMO_APP_TRANSMISSION_TYPE            CONFIRMED

/* FPORT Value (1-255) */
#define DEMO_APP_FPORT                           1

/* Device Class - Class of the device (CLASS_A/CLASS_C) */
#define DEMO_APP_ENDDEVICE_CLASS                 CLASS_A
//#define DEMO_APP_ENDDEVICE_CLASS                 CLASS_C

/* Adaptive Data Rate ( 1 = enable, 0 = disable ) */
//#define  ADRenable									1
#define  ADRenable								0

/* ABP Join Parameters */

#define DEMO_DEVICE_ADDRESS                     0x26000B37
#define DEMO_APPLICATION_SESSION_KEY            { 0x00, 0x00, 0x00, 0x7B, 0xC5, 0x55, 0xB2, 0x01, 0x89, 0x9E, 0x4B, 0x03, 0x3B, 0xC0, 0x63, 0xB7 }
#define DEMO_NETWORK_SESSION_KEY                { 0x00, 0x00, 0x00, 0x0E, 0x50, 0x60, 0x58, 0x92, 0xE4, 0x6F, 0x04, 0xC2, 0xB3, 0x1D, 0x0D, 0xC5 }


/* OTAA Join Parameters */

#define DEMO_DEVICE_EUI                         {0xde, 0xaf, 0xfa, 0xce, 0xde, 0xaf, 0xfa, 0xce}
#define DEMO_JOIN_EUI                    {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x05}
#define DEMO_APPLICATION_KEY                    {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x05}



//NOT USED in this example
/* Multicast Parameters */
#define DEMO_APP_MCAST_GROUPID                  0
#define DEMO_APP_MCAST_ENABLE                   true
#define DEMO_APP_MCAST_GROUP_ADDRESS            0x0037CC56
#define DEMO_APP_MCAST_APP_SESSION_KEY          {0x2B, 0x7E, 0x15, 0x16, 0x28, 0xAE, 0xD2, 0xA6, 0x2B, 0x7E, 0x15, 0x16, 0x28, 0xAE, 0xD2, 0xA6}
#define DEMO_APP_MCAST_NWK_SESSION_KEY          {0x3C, 0x8F, 0x26, 0x27, 0x39, 0xBF, 0xE3, 0xB7, 0xBC, 0x08, 0x26, 0x99, 0x1A, 0xD0, 0x50, 0x4D}


/* this defines the sensor periodic sample / transmission time */
#define SENSOR_SAMPLE_TIME  30000		// time in ms

#endif /* APP_CONFIG_H_ */

