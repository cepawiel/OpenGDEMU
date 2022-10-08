/**
 *
 * \file
 *
 * \brief FreeRTOS+CLI task implementation example
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

/* Standard includes. */
#include <stdint.h>
#include <string.h>

/* FreeRTOS includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

/* FreeRTOS+ includes. */
#include "FreeRTOS_CLI.h"

/* Atmel library includes. */
#include <freertos_uart_serial.h>
#include "sysclk.h"

/* Demo includes. */
#include "demo-tasks.h"

#if (defined confINCLUDE_UART_CLI)

/* Dimensions the buffer into which input characters are placed. */
#define MAX_INPUT_SIZE          50

/* The size of the buffer provided to the UART driver for storage of received
 * bytes. */
#define RX_BUFFER_SIZE_BYTES    (50)

/* Baud rate to use. */
#define CLI_BAUD_RATE           115200

/*-----------------------------------------------------------*/

/*
 * The task that implements the command console processing.
 */
static void uart_command_console_task(void *pvParameters);

/*-----------------------------------------------------------*/

/* Const strings used by the CLI interface. */
static const uint8_t *const welcome_message = (uint8_t *) "FreeRTOS command server.\r\nType Help to view a list of registered commands.\r\n\r\n>";
static const uint8_t *const new_line = (uint8_t *) "\r\n";
static const uint8_t *const line_separator = (uint8_t *) "\r\n[Press ENTER to execute the previous command again]\r\n>";

/* The buffer provided to the UART driver to store incoming character in. */
static uint8_t receive_buffer[RX_BUFFER_SIZE_BYTES] = {0};

/* The UART instance used for input and output. */
static freertos_uart_if cli_uart = NULL;

/*-----------------------------------------------------------*/

void create_uart_cli_task(Uart *uart_base, uint16_t stack_depth_words,
		unsigned portBASE_TYPE task_priority)
{
	freertos_uart_if freertos_uart;
	freertos_peripheral_options_t driver_options = {
		receive_buffer,									/* The buffer used internally by the UART driver to store incoming characters. */
		RX_BUFFER_SIZE_BYTES,							/* The size of the buffer provided to the UART driver to store incoming characters. */
		configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY,	/* The priority used by the UART interrupts. */
		UART_RS232,									/* Configure the UART for RS232 operation. */
		(WAIT_TX_COMPLETE | USE_TX_ACCESS_SEM)		/* Use access mutex on Tx (as more than one task transmits) but not Rx. Wait for a Tx to complete before returning from send functions. */
	};

	sam_uart_opt_t uart_settings;
	uart_settings.ul_mck = sysclk_get_peripheral_hz();
	uart_settings.ul_baudrate = CLI_BAUD_RATE;
	uart_settings.ul_mode = UART_MR_PAR_NO;

	/* Initialise the UART interface. */
	freertos_uart = freertos_uart_serial_init(uart_base,
			&uart_settings,
			&driver_options);
	configASSERT(freertos_uart);

	/* Register the default CLI commands. */
	vRegisterCLICommands();

	/* Create the UART CLI task. */
	xTaskCreate(	uart_command_console_task,			/* The task that implements the command console. */
					(const int8_t *const) "U_CLI",	/* Text name assigned to the task.  This is just to assist debugging.  The kernel does not use this name itself. */
					stack_depth_words,					/* The size of the stack allocated to the task. */
					(void *) freertos_uart,			/* The parameter is used to pass the already configured UART port into the task. */
					task_priority,						/* The priority allocated to the task. */
					NULL);								/* Used to store the handle to the created task - in this case the handle is not required. */
}

/*-----------------------------------------------------------*/

static void uart_command_console_task(void *pvParameters)
{
	uint8_t received_char, input_index = 0, *output_string;
	static int8_t input_string[MAX_INPUT_SIZE],
			last_input_string[MAX_INPUT_SIZE];
	portBASE_TYPE returned_value;
	portTickType max_block_time_ticks = 200UL / portTICK_RATE_MS;

	cli_uart = (freertos_uart_if) pvParameters;
	configASSERT(cli_uart);

	/* Obtain the address of the output buffer.  Note there is no mutual
	exclusion on this buffer as it is assumed only one command console
	interface will be used at any one time. */
	output_string = (uint8_t *) FreeRTOS_CLIGetOutputBuffer();

	/* Send the welcome message.  The message is copied into RAM first as the
	DMA cannot send from Flash addresses. */
	strcpy((char *) output_string, (char *) welcome_message);
	freertos_uart_write_packet(cli_uart, output_string,
			strlen((char *) welcome_message),
			max_block_time_ticks);

	for (;;) {
		/* Only interested in reading one character at a time. */
		if (freertos_uart_serial_read_packet(cli_uart,
				&received_char, sizeof(received_char),
				portMAX_DELAY) == sizeof(received_char)) {
			/* Echo the character. */
			freertos_uart_write_packet(cli_uart, &received_char,
					sizeof(received_char),
					max_block_time_ticks);

			if (received_char == '\r') {
				/* Start to transmit a line separator, just to make the output
				easier to read. */
				strcpy((char *) output_string,
						(char *) new_line);
				freertos_uart_write_packet(cli_uart,
						output_string, strlen(
						(char *) new_line),
						max_block_time_ticks);

				/* See if the command is empty, indicating that the last command
				is to be executed again. */
				if (input_index == 0) {
					strcpy((char *) input_string,
							(char *) last_input_string);
				}

				/* Pass the received command to the command interpreter.  The
				command interpreter is called repeatedly until it returns pdFALSE as
				it might generate more than one string. */
				do {
					/* Get the string to write to the UART from the command
					interpreter. */
					returned_value = FreeRTOS_CLIProcessCommand(
							input_string,
							(int8_t *) output_string,
							configCOMMAND_INT_MAX_OUTPUT_SIZE);

					/* Start the UART transmitting the generated string. */
					freertos_uart_write_packet(cli_uart,
							output_string,
							strlen((char *)
							output_string),
							max_block_time_ticks);
				} while (returned_value != pdFALSE);

				/* All the strings generated by the input command have been sent.
				Clear the input	string ready to receive the next command.
				Remember the command that was just processed first in case it is
				to be processed again. */
				strcpy((char *) last_input_string,
						(char *) input_string);
				input_index = 0;
				memset(input_string, 0x00, MAX_INPUT_SIZE);

				/* Start to transmit a line separator, just to make the output
				easier to read. */
				strcpy((char *) output_string,
						(char *) line_separator);
				freertos_uart_write_packet(cli_uart,
						output_string, strlen(
						(char *) line_separator),
						max_block_time_ticks);
			} else {
				if (received_char == '\n') {
					/* Ignore the character. */
				} else if (received_char == '\b') {
					/* Backspace was pressed.  Erase the last character in the
					string - if any. */
					if (input_index > 0) {
						input_index--;
						input_string[input_index] = '\0';
					}
				} else {
					/* A character was entered.  Add it to the string
					entered so far.  When a \n is entered the complete
					string will be passed to the command interpreter. */
					if (input_index < MAX_INPUT_SIZE) {
						input_string[input_index] = received_char;
						input_index++;
					}
				}
			}
		}
	}
}

/*-----------------------------------------------------------*/

void uart_cli_output(const uint8_t *message_string)
{
	const portTickType max_block_time_ticks = 200UL / portTICK_RATE_MS;

	/* The UART is configured to use a mutex on Tx, so can be safely written
	to directly. */
	if (cli_uart != NULL) {
		freertos_uart_write_packet(cli_uart, message_string,
				strlen((const char *) message_string), max_block_time_ticks);
	}
}

/*-----------------------------------------------------------*/

#endif
