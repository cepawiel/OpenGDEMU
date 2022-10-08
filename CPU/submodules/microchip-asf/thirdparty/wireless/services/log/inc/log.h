/**
* \file log.h
*
* \brief Headers containing interfaces, types and macros for logging module
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

#ifndef _LOG_H_
#define _LOG_H_

/************************************************************************/
/* INCLUDES                                                             */
/************************************************************************/
#include <asf.h>

/************************************************************************/
/* DEFINES                                                              */
/************************************************************************/
/* Format of printf */
#define PRINTF_FORMAT           __attribute__ ((format (printf, 3, 4)))

/* Levels of logging */
#define NONE                    (1)
#define INFO                    (2)
#define WARN                    (3)
#define ERROR                   (4)
#define DEBUG                   (5)

/* Size of the temporary buffer */
#define LOGSTR_BUFFER_SIZE      (256)

/* If LOG_LEVEL symbol is not defined during build, then set to NONE */
#if !defined(LOG_LEVEL)
#define LOG_LEVEL               NONE
#endif

#if (LOG_LEVEL == NONE)
#define LOGLINE_INFO(...)
#define LOGLINE_WARN(...)
#define LOGLINE_ERROR(...)
#define LOGLINE_DEBUG(...)
#define LOGMSG_INFO(...)
#define LOGMSG_WARN(...)
#define LOGMSG_ERROR(...)
#define LOGMSG_DEBUG(...)
#endif

#if (LOG_LEVEL == INFO)
#define LOGLINE_INFO(...)   log_output(INFO, true, __VA_ARGS__)
#define LOGLINE_WARN(...)                   
#define LOGLINE_ERROR(...)                  
#define LOGLINE_DEBUG(...)      
#define LOGMSG_INFO(...)   log_output(INFO, false, __VA_ARGS__)
#define LOGMSG_WARN(...)
#define LOGMSG_ERROR(...)
#define LOGMSG_DEBUG(...)            
#endif                                  
                                        
#if (LOG_LEVEL == WARN)                 
#define LOGLINE_INFO(...)   log_output(INFO, true, __VA_ARGS__)
#define LOGLINE_WARN(...)   log_output(WARN, true, __VA_ARGS__)
#define LOGLINE_ERROR(...)                  
#define LOGLINE_DEBUG(...)                  
#define LOGMSG_INFO(...)   log_output(INFO, false, __VA_ARGS__)
#define LOGMSG_WARN(...)   log_output(WARN, false, __VA_ARGS__)
#define LOGMSG_ERROR(...)
#define LOGMSG_DEBUG(...)
#endif                                  
                                        
#if (LOG_LEVEL == ERROR)                
#define LOGLINE_INFO(...)   log_output(INFO, true, __VA_ARGS__)
#define LOGLINE_WARN(...)   log_output(WARN, true, __VA_ARGS__)
#define LOGLINE_ERROR(...)  log_output(ERROR, true, __VA_ARGS__)
#define LOGLINE_DEBUG(...)                  
#define LOGMSG_INFO(...)   log_output(INFO, false, __VA_ARGS__)
#define LOGMSG_WARN(...)   log_output(WARN, false, __VA_ARGS__)
#define LOGMSG_ERROR(...)  log_output(ERROR, false, __VA_ARGS__)
#define LOGMSG_DEBUG(...)
#endif                                  
                                        
#if (LOG_LEVEL == DEBUG)                
#define LOGLINE_INFO(...)   log_output(INFO, true, __VA_ARGS__)
#define LOGLINE_WARN(...)   log_output(WARN, true, __VA_ARGS__)
#define LOGLINE_ERROR(...)  log_output(ERROR, true, __VA_ARGS__)
#define LOGLINE_DEBUG(...)  log_output(DEBUG, true, __VA_ARGS__)
#define LOGMSG_INFO(...)   log_output(INFO, false, __VA_ARGS__)
#define LOGMSG_WARN(...)   log_output(WARN, false, __VA_ARGS__)
#define LOGMSG_ERROR(...)  log_output(ERROR, false, __VA_ARGS__)
#define LOGMSG_DEBUG(...)  log_output(DEBUG, false, __VA_ARGS__)
#endif

/************************************************************************/
/* PROTOTYPES                                                           */
/************************************************************************/
/*
 * \brief Outputs formated string to serial interface
 * 
 * \param[in] level   - logging level for the message
 * \param[in] newline - add log message in a new line
 * \param[in] format  - formatted string pointer
 */
PRINTF_FORMAT void log_output(uint8_t level, bool newline, const char *format, ...);

#endif /* #ifndef _LOG_H_ */

/* eof log.h */
