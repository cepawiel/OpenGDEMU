/**
 * \file
 *
 * \brief USART hardware handshaking example for SAM.
 *
 * Copyright (c) 2011-2018 Microchip Technology Inc. and its subsidiaries.
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

/**
 *  \mainpage USART Hardware Handshaking Example
 *
 *  \par Purpose
 *
 *  This example demonstrates the hardware handshaking mode (i.e., RTS/CTS)
 *  provided by the USART peripherals on SAM microcontrollers. The practical
 *  use of hardware handshaking is that it allows to stop transfer on the USART
 *  without losing any data in the process. This is very useful for applications
 *  that need to program slow memories for example.
 *
 *  \par Requirements
 *
 *  This example can be used on all SAM EK. 
 *  It requires a serial line with hardware control support (TXD and RXD cross 
 *  over, RTS and CTS cross over) to connect the board and PC.
 *  
 *  \par Description
 *
 *  The provided program uses hardware handshaking mode to regulate the data
 *  rate of an incoming file transfer. A terminal application, such as
 *  HyperTerminal, is used to send a text file to the device (without any
 *  protocol such as X-modem). The device will enforce the configured
 *  bytes per second (bps) rate with its Request To Send (RTS) line.
 *
 *  Whenever the data rate meets or exceeds the configurable threshold, the device
 *  stops receiving data on the USART. Since no buffer is provided to the PDC,
 *  this will set the RTS line, telling the computer to stop sending data. Each
 *  second, the current data rate and total number of bytes received are
 *  displayed; the transfer is also restarted.
 *
 *  Note that the device may receive slightly less bytes than the actual file
 *  size, depending on the nature of the file. This does NOT mean that bytes
 *  have been lost: this is simply an issue with how line breaks are transmitted
 *  by the terminal. It is therefore better to use binary files, as they most
 *  often do not contain line breaks. For example, send one of the object files
 *  generated by the compiler.
 *
 *  \par Usage
 *
 *  -# Build the program and download it into the evaluation board.
 *  -# Connect a serial cable to the USART port on the evaluation kit.
 *  -# On the computer, open and configure a terminal application (e.g.,
 *     HyperTerminal on Microsoft Windows) with these settings:
 *        - 115200 bauds
 *        - 8 data bits
 *        - No parity
 *        - 1 stop bit
 *        - Hardware flow control (RTS/CTS)
 *  -# Start the application. The following traces shall appear on the terminal:
 *     \code
	-- USART Hardware Handshaking Example --
	-- xxxxxx-xx
	-- Compiled: xxx xx xxxx xx:xx:xx --
	Bps:    0; Tot:      0
\endcode
 *  -# Send a file in text format to the device. On HyperTerminal, this is done
 *     by selecting "Transfer -> Send Text File" (this does not prevent you from
 *     sending binary files). The transfer will start and the device will update
 *     the bps and total counts on the terminal.
 *  -# Whenever the transfer is complete, the total number of bytes received
 *     should match the size of the sent file (unless it is a text file, see
 *     explanation in description section).
 *
 */
/*
 * Support and FAQ: visit <a href="https://www.microchip.com/support/">Microchip Support</a>
 */

#include <string.h>
#include "asf.h"
#include "stdio_serial.h"
#include "conf_board.h"
#include "conf_clock.h"
#include "conf_example.h"

/** Maximum Bytes Per Second (BPS) rate that will be forced using the CTS pin. */
#define MAX_BPS             500

/** Size of the receive buffer used by the PDC, in bytes. */
#define BUFFER_SIZE         125

/** All interrupt mask. */
#define ALL_INTERRUPT_MASK  0xffffffff

/** Timer counter frequency in Hz. */
#define TC_FREQ             1

#define STRING_EOL    "\r"
#define STRING_HEADER "--USART Hardware Handshaking Example --\r\n" \
		"-- "BOARD_NAME" --\r\n" \
		"-- Compiled: "__DATE__" "__TIME__" --"STRING_EOL

/** Number of bytes received between two timer ticks. */
volatile uint32_t g_ul_bytes_received = 0;

/** Receive buffer. */
uint8_t g_uc_buffer;

/** PDC data packet. */
pdc_packet_t g_st_packet, g_st_nextpacket;

/** Pointer to PDC register base. */
Pdc *g_p_pdc;

/** String g_uc_buffer. */
uint8_t g_puc_string[BUFFER_SIZE];

/** Receive buffer. */
static uint8_t gs_puc_buffer[BUFFER_SIZE],gs_puc_nextbuffer[BUFFER_SIZE];

/** Dump buffer. */
static uint8_t gs_dump_buffer[BUFFER_SIZE];

/**
 *  \brief Interrupt handler for USART.
 *
 * Increment the number of bytes received in the current second and start
 * another transfer if the desired bps has not been met yet.
 *
 */
void USART_Handler(void)
{
	uint32_t ul_status;

	tc_stop(TC0, 0);

	/* Read USART status. */
	ul_status = usart_get_status(BOARD_USART);

	/* Receive buffer is full. */
	if (ul_status & US_CSR_RXBUFF) {
		g_ul_bytes_received += 2 * BUFFER_SIZE;
		if (g_ul_bytes_received < MAX_BPS) {
			/* Restart transfer if BPS is not high enough. */
			g_st_packet.ul_addr = (uint32_t)gs_puc_buffer;
			g_st_packet.ul_size = BUFFER_SIZE;
			g_st_nextpacket.ul_addr = (uint32_t)gs_puc_nextbuffer;
			g_st_nextpacket.ul_size = BUFFER_SIZE;
			pdc_rx_init(g_p_pdc, &g_st_packet, &g_st_nextpacket);
		} else {
			/* Otherwise disable interrupt. */
			usart_disable_interrupt(BOARD_USART, US_IDR_RXBUFF);
		}
		memcpy(gs_dump_buffer, gs_puc_buffer, BUFFER_SIZE);
	}
	tc_start(TC0, 0);
}

/**
 *  \brief Interrupt handler for TC0.
 *
 * Display the number of bytes received during the last second and the total
 * number of bytes received, and then restart a read transfer on the USART if it
 * was stopped.
 */
void TC0_Handler(void)
{
	uint32_t ul_status;
	static uint32_t bytes_total = 0;

	/* Read TC0 status. */
	ul_status = tc_get_status(TC0, 0);

	/* RC compare. */
	if ((ul_status & TC_SR_CPCS) == TC_SR_CPCS) {
		if((pdc_read_rx_counter(g_p_pdc) != 0) &&
				(pdc_read_rx_counter(g_p_pdc) != BUFFER_SIZE)) {
			if(pdc_read_rx_next_counter(g_p_pdc) == 0) {
				g_ul_bytes_received += (2 * BUFFER_SIZE - pdc_read_rx_counter(g_p_pdc));
			} else {
				g_ul_bytes_received += (BUFFER_SIZE - pdc_read_rx_counter(g_p_pdc));
			}
			memset(gs_dump_buffer, 0, BUFFER_SIZE);
			memcpy(gs_dump_buffer, gs_puc_buffer, BUFFER_SIZE -
					pdc_read_rx_counter(g_p_pdc));
		} else if((pdc_read_rx_counter(g_p_pdc) == BUFFER_SIZE) &&
				(pdc_read_rx_next_counter(g_p_pdc) == 0)) {
			g_ul_bytes_received += BUFFER_SIZE;
			memcpy(gs_dump_buffer, gs_puc_buffer, BUFFER_SIZE);
		}

		/* Display info. */
		bytes_total += g_ul_bytes_received;
		memset(g_puc_string, 0, BUFFER_SIZE);
		sprintf((char *)g_puc_string, "Bps: %4lu; Tot: %6lu\r\n",
				(unsigned long)g_ul_bytes_received,
				(unsigned long)bytes_total);
		usart_write_line(BOARD_USART, (char *)g_puc_string);
		/* Resume transfer */
		g_st_packet.ul_addr = (uint32_t)gs_puc_buffer;
		g_st_packet.ul_size = BUFFER_SIZE;
		g_st_nextpacket.ul_addr = (uint32_t)gs_puc_nextbuffer;
		g_st_nextpacket.ul_size = BUFFER_SIZE;
		pdc_rx_init(g_p_pdc, &g_st_packet, &g_st_nextpacket);
			usart_enable_interrupt(BOARD_USART, US_IER_RXBUFF);

		g_ul_bytes_received = 0;
	}
}

/**
 *  Configure Timer Counter 0 to generate an interrupt every second.
 */
static void configure_tc(void)
{
	uint32_t ul_div;
	uint32_t ul_tcclks;
	uint32_t ul_sysclk;

	/* Get system clock. */
	ul_sysclk = sysclk_get_cpu_hz();

	/* Configure PMC. */
	pmc_enable_periph_clk(ID_TC0);

    /** Configure TC for a 1Hz frequency and trigger on RC compare. */
	tc_find_mck_divisor(TC_FREQ, ul_sysclk, &ul_div, &ul_tcclks, ul_sysclk);
	tc_init(TC0, 0, ul_tcclks | TC_CMR_CPCTRG);
	tc_write_rc(TC0, 0, (ul_sysclk / ul_div) / TC_FREQ);

	/* Configure and enable interrupt on RC compare. */
	NVIC_EnableIRQ((IRQn_Type) ID_TC0);
	tc_enable_interrupt(TC0, 0, TC_IER_CPCS);
}

/**
 *  Configure board USART communication with PC or other terminal.
 */
static void configure_usart(void)
{
	static uint32_t ul_sysclk;
	const sam_usart_opt_t usart_console_settings = {
		BOARD_USART_BAUDRATE,
		US_MR_CHRL_8_BIT,
		US_MR_PAR_NO,
		US_MR_NBSTOP_1_BIT,
		US_MR_CHMODE_NORMAL,
		/* This field is only used in IrDA mode. */
		0
	};

	/* Get peripheral clock. */
	ul_sysclk = sysclk_get_peripheral_hz();

	/* Enable peripheral clock. */
	sysclk_enable_peripheral_clock(BOARD_ID_USART);

	/* Configure USART. */
	usart_init_hw_handshaking(BOARD_USART, &usart_console_settings, ul_sysclk);

	/* Disable all the interrupts. */
	usart_disable_interrupt(BOARD_USART, ALL_INTERRUPT_MASK);
	
	/* Enable TX & RX function. */
	usart_enable_tx(BOARD_USART);
	usart_enable_rx(BOARD_USART);

	/* Specify that stdout should not be buffered. */
#if defined(__GNUC__)
	setbuf(stdout, NULL);
#else
	/* Already the case in IAR's Normal DLIB default configuration: printf()
	 * emits one character at a time.
	 */
#endif

	/* Configure and enable interrupt of USART. */
	NVIC_EnableIRQ(USART_IRQn);
}

/**
 *  Configure UART for debug message output.
 */
static void configure_console(void)
{
	const usart_serial_options_t uart_serial_options = {
		.baudrate = CONF_UART_BAUDRATE,
		.paritytype = CONF_UART_PARITY
	};
	
	/* Configure console UART. */
	sysclk_enable_peripheral_clock(CONSOLE_UART_ID);
	stdio_serial_init(CONF_UART, &uart_serial_options);
}

/**
 *  \brief usart_hard_handshaking_example application entry point.
 *
 *  \return Unused (ANSI-C compatibility).
 */
int main(void)
{
	uint8_t uc_char;
	uint8_t uc_flag;

	/* Initialize the system. */
	sysclk_init();
	board_init();

	/* Configure UART for debug message output. */
	configure_console();

	/* Output example information. */
	puts(STRING_HEADER);

	/* Initialize board USART. */
	configure_usart();

	/* Initialize TC0. */
	configure_tc();

	/* Get board USART PDC base address and enable receiver. */
	g_p_pdc = usart_get_pdc_base(BOARD_USART);
	g_st_packet.ul_addr = (uint32_t)gs_puc_buffer;
	g_st_packet.ul_size = BUFFER_SIZE;
	g_st_nextpacket.ul_addr = (uint32_t)gs_puc_nextbuffer;
	g_st_nextpacket.ul_size = BUFFER_SIZE;
	pdc_rx_init(g_p_pdc, &g_st_packet, &g_st_nextpacket);
	pdc_enable_transfer(g_p_pdc, PERIPH_PTCR_RXTEN);

	usart_enable_interrupt(BOARD_USART, US_IER_RXBUFF);

	/* Start the Timer counter. */
	tc_start(TC0, 0);

	while (1) {
		uc_char = 0;
		uc_flag = uart_read(CONSOLE_UART, &uc_char);
		if (!uc_flag) {
			switch (uc_char) {
			case 'd':
			case 'D':
				usart_write_line(BOARD_USART, (char *)gs_dump_buffer);
				break;
			default:
				break;
			}
		}
	}
}