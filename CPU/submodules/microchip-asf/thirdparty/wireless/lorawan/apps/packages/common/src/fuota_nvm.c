/**
* \file  fuota_nvm.c
*
* \brief Interface for accessing NVM to perform FUOTA image related storage
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

/************************************************************************/
/* INCLUDES                                                             */
/************************************************************************/
#include <asf.h>
#include "nvm.h"
#include "common_nvm.h"
#include "fuota_nvm.h"

/************************************************************************/
/* FUNCTION DEFINITIONS                                                 */
/************************************************************************/
status_code_t FUOTA_NVM_Init( void )
{
    status_code_t status = STATUS_OK;
#if (FUOTA_NVM_TYPE == TYPE_INTERNAL)
    status = nvm_init( INT_FLASH ); 
#endif /* #if (FUOTA_NVM_TYPE == TYPE_INTERNAL) */
    return status;
}

status_code_t FUOTA_NVM_Read(uint32_t address, void *buffer, uint32_t len)
{
    status_code_t status = STATUS_OK;
#if (FUOTA_NVM_TYPE == TYPE_INTERNAL)
    do
    {
        status = nvm_read( INT_FLASH, address, buffer, len );
    } while ( (status_code_genare_t) status == STATUS_BUSY );
#endif /* #if (FUOTA_NVM_TYPE == TYPE_INTERNAL) */
    return status;
}

status_code_t FUOTA_NVM_Write(uint32_t address, void *buffer, uint32_t len)
{
    status_code_t status = STATUS_OK;
#if (FUOTA_NVM_TYPE == TYPE_INTERNAL)
    status = nvm_write( INT_FLASH, address, buffer, len );
#endif /* #if (FUOTA_NVM_TYPE == TYPE_INTERNAL) */
    return status;
}

/* eof fuota_nvm.h */
