/**
* \file log.c
*
* \brief Implementation of logging module
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
#include <stdarg.h>
#include "log.h"
#include "sio2host.h"

#define  WARN_PREFIX_STR        ((uint8_t*)"\r\n[WARN] ")
#define  WARN_PREFIX_STR_LEN    (9)
#define  ERROR_PREFIX_STR       ((uint8_t*)"\r\n[ERROR] ")
#define  ERROR_PREFIX_STR_LEN   (10)
#define  DEBUG_PREFIX_STR       ((uint8_t*)"\r\n[DEBUG] ")
#define  DEBUG_PREFIX_STR_LEN   (10)
#define  NEW_LINE_STR           ((uint8_t*)"\r\n")
#define  NEW_LINE_STR_LEN       (2)

/************************************************************************/
/* FUNCTION DEFINITIONS                                                 */
/************************************************************************/
void log_output(uint8_t level, bool newline, const char *format, ...)
{
#if (LOG_LEVEL > NONE)
    if (level <= LOG_LEVEL)
    {
        uint8_t str[LOGSTR_BUFFER_SIZE];
        uint8_t length;
        va_list argList;
        uint8_t prefixLengths[6] = {0, 2, 2, WARN_PREFIX_STR_LEN, ERROR_PREFIX_STR_LEN, DEBUG_PREFIX_STR_LEN};
        uint8_t curPrefixLen = newline ? prefixLengths[level] : 0;
        
        va_start(argList, format);
        length = vsnprintf((char *)(str + curPrefixLen), LOGSTR_BUFFER_SIZE - curPrefixLen, format, argList);
        va_end(argList);        
        
        if (newline)
        {
            switch ( level )
            {
                case NONE:
                case INFO:
                memcpy(str, NEW_LINE_STR, NEW_LINE_STR_LEN);
                break;
                
                case WARN:
                memcpy(str, WARN_PREFIX_STR, WARN_PREFIX_STR_LEN);
                break;
                
                case ERROR:
                memcpy(str, ERROR_PREFIX_STR, ERROR_PREFIX_STR_LEN);
                break;
                
                case DEBUG:
                memcpy(str, DEBUG_PREFIX_STR, DEBUG_PREFIX_STR_LEN);
                break;
            }
        }
        
        sio2host_tx(str, length + curPrefixLen);
    }
    else
    {
        // nothing
    }
#endif
}

/* eof log.c */
