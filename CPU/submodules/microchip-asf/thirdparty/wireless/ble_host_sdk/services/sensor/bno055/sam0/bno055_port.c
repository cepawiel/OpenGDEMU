/**
 * \file bno055_port.c
 *
 * \brief BNO055 extension board example
 *
 * Copyright (c) 2018 Microchip Technology Inc. and its subsidiaries.
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
/*
 * Support and FAQ: visit <a href="https://www.microchip.com/support/">Microchip Support</a>
 */

#include "bno055_port.h"
#include "i2c_master.h"
#include "ioport.h"
#include "conf_board.h"
#include "conf_bno055.h"

/* Instantiates a BNO055 software instance structure which retains
* chip ID, internal sensors IDs, I2C address and pointers
* to required functions (bus read/write and delay functions) */
struct bno055_t bno055_sensor;

struct i2c_master_module i2c_master_module;
struct i2c_master_packet bno055_write_pkt;
struct i2c_master_packet bno055_read_pkt;

#if 0	//Just to avoid compiler warning
void i2c_write_complete_callback(struct i2c_master_module *const module)
{
	
}
#endif //#if 0

void bno055_initialize(void)
{
	struct bno055_accel_t accel_xyz;
	int16_t accel_datax = 0, accel_datay = 0, accel_dataz = 0;
	uint8_t power_mode = BNO055_POWER_MODE_NORMAL;
	
	bno055_sensor.bus_write = bno055_i2c_bus_write;
	bno055_sensor.bus_read = bno055_i2c_bus_read;
	bno055_sensor.delay_msec = bno055_delay_msek;
	bno055_sensor.dev_addr = BNO055_I2C_SLAVE_ADDRESS;
	
	bno055_gpio_config();
#if BNO055_I2C_SLAVE_ADDRESS == BNO055_I2C_ADDR1
	ioport_set_pin_level(BNO055_PIN_SLAVE_ADDR_SELECT, false);
#else
	ioport_set_pin_level(BNO055_PIN_SLAVE_ADDR_SELECT, true);
#endif
	
	bno055_i2c_bus_init();
	bno055_reset();
	bno055_init(&bno055_sensor);
	
	/* set the power mode as NORMAL*/
	bno055_set_power_mode(power_mode);
	bno055_set_operation_mode(BNO055_OPERATION_MODE_AMG);
	bno055_set_accel_range(BNO055_ACCEL_RANGE_2G);	//
	bno055_set_gyro_range(BNO055_GYRO_RANGE_500DPS);	//
	
	bno055_read_accel_x(&accel_datax);
	bno055_delay_msek(5);
	bno055_read_accel_y(&accel_datay);
	bno055_delay_msek(5);
	bno055_read_accel_z(&accel_dataz);
	bno055_delay_msek(5);
	bno055_read_accel_xyz(&accel_xyz);
	bno055_delay_msek(10);
}

/*	
 *  \Brief: The function is used as I2C bus init
 */
void bno055_i2c_bus_init(void)
{
	struct i2c_master_config config_i2c_master;
	
	bno055_write_pkt.address = (uint16_t)BNO055_I2C_SLAVE_ADDRESS;
	bno055_write_pkt.ten_bit_address = false;
	bno055_read_pkt.address = (uint16_t)BNO055_I2C_SLAVE_ADDRESS;
	bno055_read_pkt.ten_bit_address = false;
	
	i2c_master_get_config_defaults(&config_i2c_master);
	while(i2c_master_init(&i2c_master_module, CONF_I2C_MASTER_MODULE, &config_i2c_master)	!= STATUS_OK);
	i2c_master_enable(&i2c_master_module);
}

/*	\Brief: The function is used as I2C bus write
 *	\Return : Status of the I2C write
 *	\param dev_addr : The device address of the sensor
 *	\param reg_addr : Address of the first register, will data is going to be written
 *	\param reg_data : It is a value hold in the array,
 *		will be used for write the value into the register
 *	\param cnt : The no of byte of data to be write
 */
int8_t bno055_i2c_bus_write(uint8_t dev_addr, uint8_t reg_addr, uint8_t *reg_data, uint8_t cnt)
{
	int8_t ierror;
	uint8_t i2c_data_buf[10] = {0};
	
	if((!reg_data) && (cnt))
	{
		return (s8)BNO055_E_NULL_PTR;
	}
	
	i2c_data_buf[BNO055_REG_ADDR_START] = reg_addr;
	for(uint8_t index = 0; index < cnt; index++)
	{
		i2c_data_buf[BNO055_WRITE_DATA_START + index] = *(reg_data + index);
	}
	
	bno055_write_pkt.data_length = cnt + 1;
	bno055_write_pkt.data = i2c_data_buf;
	ierror = i2c_master_write_packet_wait(&i2c_master_module, &bno055_write_pkt);
	
	return ierror;
}

#if 0	//Just to avoid compiler warnings
static int8_t bno055_i2c_bus_write_no_stop(uint8_t dev_addr, uint8_t reg_addr, uint8_t *reg_data, uint8_t cnt)
{
	int8_t ierror;
	uint8_t i2c_data_buf[10] = {0};
	
	if((!reg_data) && (cnt))
	{
		return (s8)BNO055_E_NULL_PTR;
	}
	
	i2c_data_buf[BNO055_REG_ADDR_START] = reg_addr;
	for(uint8_t index = 0; index < cnt; index++)
	{
		i2c_data_buf[BNO055_WRITE_DATA_START + index] = *(reg_data + index);
	}
	
	bno055_write_pkt.data_length = cnt + 1;
	bno055_write_pkt.data = i2c_data_buf;
	ierror = i2c_master_read_packet_wait_no_stop(&i2c_master_module, &bno055_write_pkt);

	return ierror;
}
#endif //#if 0

 /*	\Brief: The function is used as I2C bus read
 *	\Return : Status of the I2C read
 *	\param dev_addr : The device address of the sensor
 *	\param reg_addr : Address of the first register, will data is going to be read
 *	\param reg_data : This data read from the sensor, which is hold in an array
 *	\param cnt : The no of byte of data to be read
 */
int8_t bno055_i2c_bus_read(uint8_t dev_addr, uint8_t reg_addr, uint8_t *reg_data, uint8_t cnt)
{
	int8_t ierror;
	
	if((!reg_data) && (cnt))
	{
		return (s8)BNO055_E_NULL_PTR;
	}
	
	bno055_i2c_bus_write(dev_addr, reg_addr, NULL, 0);
	bno055_delay_msek(40);
	
	bno055_read_pkt.data_length = cnt;
	bno055_read_pkt.data = reg_data;
	ierror = i2c_master_read_packet_wait(&i2c_master_module, &bno055_read_pkt);
	
	return ierror;
}
/*	Brief : The delay routine
 *	\param : delay in ms
*/
void bno055_delay_msek(BNO055_MDELAY_DATA_TYPE msek)
{
	/*Here you can write your own delay routine*/
	delay_ms(msek);
}

void bno055_reset(void)
{
	ioport_set_pin_level(BNO055_PIN_RESET,  BNO055_RESET_ACTIVE);
	delay_ms(BNO055_RESET_DELAY_MS);
	ioport_set_pin_level(BNO055_PIN_RESET, !BNO055_RESET_ACTIVE);
	delay_ms(BNO055_RESET_DELAY_MS);
}

void extint_initialize(void (*handler_function)(void))
{

}

void bno055_gpio_config(void)
{
	/* Slave address */
	ioport_set_pin_dir(BNO055_PIN_SLAVE_ADDR_SELECT, IOPORT_DIR_OUTPUT);
	/* Reset */
	ioport_set_pin_dir(BNO055_PIN_RESET, IOPORT_DIR_OUTPUT);
	/* Interrupt */
	ioport_set_pin_dir(BNO055_PIN_INT, IOPORT_DIR_INPUT);
}