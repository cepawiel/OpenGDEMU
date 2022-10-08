/**
* \file  enddevice_demo.h
*
* \brief LORAWAN Demo Application
*		
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
 
#ifndef DEMO_APP_H_
#define DEMO_APP_H_

/*********************************************************************//**
 \brief      Macro to check the LoRaWAN stack status attribute
             network joined bit
*************************************************************************/
#define LORAWAN_NW_JOINED	0x01
//#define EDBG_EUI_READ 1
typedef enum _AppTaskIds_t
{
	DISPLAY_TASK_HANDLER,
	PROCESS_TASK_HANDLER,
	APP_TASKS_COUNT
}AppTaskIds_t;

typedef enum _AppTaskState_t
{
	RESTORE_BAND_STATE,
	DEMO_CERT_APP_STATE,
	DEMO_APP_STATE,
	JOIN_SEND_STATE
}AppTaskState_t;

/*********************************************************************//**
 \brief      Function to Initialize the Demo application
*************************************************************************/
void mote_demo_init(void);

/*********************************************************************//*
 \brief      Set join parameters function
 \param[in]  activation type - notifies the activation type (OTAA/ABP)
 \return     LORAWAN_SUCCESS, if successfully set the join parameters
             LORAWAN_INVALID_PARAMETER, otherwise
 ************************************************************************/
StackRetStatus_t set_join_parameters(ActivationType_t activation_type);

/*********************************************************************//*
 \brief      Function to Initialize the device type
 \param[in]  ed_class - notifies the device class (CLASS_A/CLASS_C)
 \return     LORAWAN_SUCCESS, if successfully set the device class
             LORAWAN_INVALID_PARAMETER, otherwise
 ************************************************************************/
StackRetStatus_t set_device_type(EdClass_t ed_class);

/***********************************************************************
 \brief      Function to Initialize set default parameters
 \param[in]  void
 \return     LORAWAN_SUCCESS, if successfully set all the parameters
             LORAWAN_INVALID_PARAMETER, otherwise
 ************************************************************************/
StackRetStatus_t mote_set_parameters(IsmBand_t ismBand, const uint16_t index);

/*********************************************************************//*
 \brief      Function to Initialize the Multicast parameters
 ************************************************************************/
void set_multicast_params (void);

/*********************************************************************//**
 \brief      Callback function for the ending of Bidirectional communication of
       Application data
 *************************************************************************/
void demo_appdata_callback(void *appHandle, appCbParams_t *data);

/*********************************************************************//*
 \brief      Callback function for the ending of Activation procedure
 ************************************************************************/
void demo_joindata_callback(StackRetStatus_t status);

/*********************************************************************//*
 \brief      Function to Print stack default parameters
 ************************************************************************/
void print_default_parameters(void);

/*********************************************************************//*
 \brief      Function to Print application configuration
 ************************************************************************/
void  print_application_config (void);

/*********************************************************************//*
 \brief      Function to Print array of characters
 \param[in]  *array  - Pointer of the array to be printed
 \param[in]   length - Length of the array
 ************************************************************************/
void print_array (uint8_t *data, uint8_t length);

/*********************************************************************//*
 \brief      Function to Print stack return status
 \param[in]  status - Status from the stack
 ************************************************************************/
void print_stack_status(StackRetStatus_t status);

/*********************************************************************//*
 \brief      Function to Check stack return status
 \param[in]  Void
 ************************************************************************/
void check_stack_status(void);

/*********************************************************************//**
\brief    Pulls the data from UART when activated
*************************************************************************/
void serial_data_handler(void);

/*********************************************************************//**
\brief    Reads the MAC EUI from the EDBG Controller of the XPro Board
*************************************************************************/
void dev_eui_read(void);

#endif /* DEMO_APP_H_ */

