/**
 *
 * \file
 *
 * \brief SAM UART API Declaration Implementations.
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

#ifndef BUFFERED_UART_H_INCLUDED
#define BUFFERED_UART_H_INCLUDED

#include "compiler.h"
#include "status_codes.h"

/**
 * \brief check if the uart buffer is empty or not.
 * \return non zero if empy.
 * not initialized
 */
int buffered_uart_is_empty(void);

/**
 * \brief Initializes the Buffered Serial IO Module
 * \return STATUS_OK for successful initialization and FAILURE in case the IO is
 * not initialized
 */
void buffered_uart_init(uint32_t baudrate);

/**
 * \brief Terminate the Buffered Serial IO Module
 * \return STATUS_OK for successful initialization and FAILURE in case the IO is
 * not initialized
 */
void buffered_uart_term(void);

/**
 * \brief Transmits data via UART
 * \param data Pointer to the buffer where the data to be transmitted is present
 * \param length Number of bytes to be transmitted
 *
 * \return Number of bytes actually transmitted
 */
uint8_t buffered_uart_tx(uint8_t *data, uint32_t length);

/**
 * \brief Receives data from UART
 *
 * \param data pointer to the buffer where the received data is to be stored
 * \param max_length maximum length of data to be received
 *
 * \return actual number of bytes received
 */
uint8_t buffered_uart_rx(uint8_t *data, uint8_t max_length);

/**
 * \brief This function performs a blocking character receive functionality
 * \return returns the data which is received
 */
uint8_t buffered_uart_getchar(void);

/**
 * \brief This function prints the character to the host
 * \return void
 */

void buffered_uart_putchar(uint8_t);

/**
 * \brief This function performs a non-blocking character receive functionality
 * \return '-1' if no data is received or returns the data if a character is
 * received
 */
int buffered_uart_getchar_nowait(void);

#endif/* BUFFERED_UART_H_INCLUDED */
