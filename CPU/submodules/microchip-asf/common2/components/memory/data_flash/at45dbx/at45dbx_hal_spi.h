/**
 * \file
 *
 * \brief Connection of the AT45DBX DataFlash to SPI interface driver.
 *
 * This file manages the connection of the AT45dbx DataFlash driver to
 * SPI driver. The SPI module selection on hardware depends on AT45DBX_SPI
 * in conf_at45dbx.h
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
/*
 * Support and FAQ: visit <a href="https://www.microchip.com/support/">Microchip Support</a>
 */

#ifndef _AT45DBX_HAL_SPI_H_
#define _AT45DBX_HAL_SPI_H_

#include "conf_at45dbx.h"

#ifdef __cplusplus
extern "C" {
#endif

struct spi_module at45dbx_master;
struct spi_slave_inst at45dbx_slave;

/*! \name Functions to connect the DataFlash driver with the SPI Multiple-Sector Access Functions
 */
//! @{

#  include "spi.h"

#define DUMMY          0xFF

/* Create as many slave instances as required... */
#define AT45DBX_CS0    AT45DBX_CS  // To keep compliance

struct spi_slave_inst_config slave_configs[AT45DBX_MEM_CNT];

uint8_t ss_pins[] = {
# define AT45DBX_CS_ID(slot, unused) \
		AT45DBX_CS##slot,
		MREPEAT(AT45DBX_MEM_CNT, AT45DBX_CS_ID, ~)
# undef AT45DBX_CS_ID
};

/*! \brief Initialize SPI external resource for AT45dbx DataFlash driver.
 *
 *  This function initializes the DataFlash component as well as the SPI service
 *  the memory is connected to.
 *  \pre The conf_at45dbx.h file should provide the following information
 *  - AT45DBX_MEM_SIZE: The DataFlash type the driver is connected to.
 *  - AT45DBX_MEM_CNT: The number of DataFlash components (number of chipselect)
 *  - AT45DBX_SPI_MODULE:  The name of the SPI module used by the DataFlash driver
 *  - AT45DBX_SPI_MASTER_SPEED: The SPI bus speed.
 *  - AT45DBX_SPI_PINMUX_SETTING: The pin mux setting for SPI module
 *  - AT45DBX_SPI_PINMUX_PAD0-3: The pad selection for SPI
 *  - AT45DBX_CS: The chip select line
 *  - AT45DBX_CLOCK_SPEED: The SPI baudrate used
 */
inline void at45dbx_spi_init(void)
{
	struct spi_config config;

	spi_get_config_defaults(&config);

	config.mux_setting = AT45DBX_SPI_PINMUX_SETTING;
	config.pinmux_pad0 = AT45DBX_SPI_PINMUX_PAD0;
	config.pinmux_pad1 = AT45DBX_SPI_PINMUX_PAD1;
	config.pinmux_pad2 = AT45DBX_SPI_PINMUX_PAD2;
	config.pinmux_pad3 = AT45DBX_SPI_PINMUX_PAD3;
	config.mode_specific.master.baudrate = AT45DBX_CLOCK_SPEED;

	spi_init(&at45dbx_master, AT45DBX_SPI, &config);
	spi_enable(&at45dbx_master);
}

/*! \brief Select one external DataFlash component
 *
 * \param mem_id  The DataFlash index number.
 */
inline void at45dbx_spi_select_device(uint8_t mem_id)
{
	switch(mem_id) {
#define AT45DBX_SELECT(slot, unused) \
	case slot:\
		spi_slave_inst_get_config_defaults(&slave_configs[slot]);\
		slave_configs[slot].ss_pin = ss_pins[slot];\
		spi_attach_slave(&at45dbx_slave, &slave_configs[slot]);\
		spi_select_slave(&at45dbx_master, &at45dbx_slave, true);\
		break;
		MREPEAT(AT45DBX_MEM_CNT, AT45DBX_SELECT, ~)
#undef AT45DBX_SELECT
	default:
		/* unhandled_case(id); */
		return;
	}
}

/*! \brief Unselect one external DataFlash component
 *
 * \param mem_id  The DataFlash index number.
 */
inline void at45dbx_spi_deselect_device(uint8_t mem_id)
{
	switch(mem_id) {
#define AT45DBX_DESELECT(slot, unused) \
	case slot:\
		spi_select_slave(&at45dbx_master, &at45dbx_slave, false);\
		break;
		MREPEAT(AT45DBX_MEM_CNT, AT45DBX_DESELECT, ~)
#undef AT45DBX_DESELECT
	default:
		/* unhandled_case(id); */
		return;
	}
}

/*! \brief Send one byte to the DataFlash.
 *
 * \param data The data byte to send.
 * \pre The DataFlash should be selected first using at45dbx_spi_select_device
 */
void at45dbx_spi_write_byte(uint8_t data)
{
	uint16_t temp = 0;
	while (!spi_is_ready_to_write(&at45dbx_master));
	spi_write(&at45dbx_master,data);
	while (!spi_is_ready_to_read(&at45dbx_master));
	spi_read(&at45dbx_master,&temp);
}

/*! \brief Get one byte (read) from the DataFlash.
 *
 * \return The received byte.
 * \pre The DataFlash should be selected first using at45dbx_spi_select_device
 */
void at45dbx_spi_read_byte(uint8_t *data)
{
	uint16_t temp = 0;

	while (!spi_is_ready_to_write(&at45dbx_master));
	spi_write(&at45dbx_master, DUMMY);
	while (!spi_is_ready_to_read(&at45dbx_master));
	spi_read(&at45dbx_master,&temp);

	*data = (uint8_t)temp;
}


/**
 * \brief Receive a sequence of bytes from a DataFlash
 *
 *
 * \param data   data buffer to read
 * \param len    Length of data
 * \pre The DataFlash should be selected first using at45dbx_spi_select_device
 */
inline void at45dbx_spi_read_packet(void const *data, size_t len)
{
	spi_read_buffer_wait(&at45dbx_master, (uint8_t *)data, len,
		(uint16_t)DUMMY);
}

/**
 * \brief Send a sequence of bytes to a DataFlash from
 *
 *
 * \param data   data buffer to write
 * \param len    Length of data
 * \pre The DataFlash should be selected first using at45dbx_spi_select_device
 *
 */
inline void at45dbx_spi_write_packet(void const *data, size_t len)
{
	spi_write_buffer_wait(&at45dbx_master,(const uint8_t *)data, len);
}

//! @}

#ifdef __cplusplus
}
#endif

#endif  // _AT45DBX_HAL_SPI_H_
