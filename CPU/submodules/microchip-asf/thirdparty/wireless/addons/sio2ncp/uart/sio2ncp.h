/**
 * \file sio2ncp.h
 *
 * \brief Handles Serial I/O  Functionalities
 *
 * Copyright (c) 2013-2018 Microchip Technology Inc. and its subsidiaries.
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

#ifndef SIO2NCP_H
#define SIO2NCP_H

/* === INCLUDES ============================================================ */

#include "compiler.h"
#include "status_codes.h"

/**
 * \defgroup group_sio2ncp_uart SIO2NCP - UART
 * This module performs serial input/output functionalities via UART from and to
 * the HOST
 * @{
 */

#define SERIAL_RX_BUF_SIZE_NCP    156

/* === PROTOTYPES ============================================================
**/

/**
 * \brief Initializes the Serial IO Module of the NCP Device
 * \return STATUS_OK for successful initialization and FAILURE incase the IO is
 * not initialized
 */
void sio2ncp_init(void);

/**
 * \brief Transmits data via UART
 * \param data Pointer to the buffer where the data to be transmitted is present
 * \param length Number of bytes to be transmitted
 *
 * \return Number of bytes actually transmitted
 */
uint8_t sio2ncp_tx(uint8_t *data, uint8_t length);

/**
 * \brief Receives data from UART
 *
 * \param data pointer to the buffer where the received data is to be stored
 * \param max_length maximum length of data to be received
 *
 * \return actual number of bytes received
 */
uint8_t sio2ncp_rx(uint8_t *data, uint8_t max_length);

/**
 * \brief This function performs a non-blocking character receive functionality
 * \return '-1' if no data is recieved or returns the data if a character is
 * received
 */
int sio2ncp_getchar_nowait(void);

/**
 * \brief This function performs a blocking character receive functionality
 * \return returns the data which is received
 */
uint8_t sio2ncp_getchar(void);

#if SAMD || SAMR21
void USART_NCP_ISR_VECT(uint8_t instance);

#endif

#endif /* SIO2NCP_H */
