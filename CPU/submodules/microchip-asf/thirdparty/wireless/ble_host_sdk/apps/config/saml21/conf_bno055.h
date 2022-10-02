/**
 * \file conf_bno055.h
 *
 * \brief BNO055 interface config for SAM L21
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

#ifndef __CONF_BNO055_H_INCLUDED__
#define __CONF_BNO055_H_INCLUDED__

/* master sercom pinmux setting */
#define CONF_I2C_MASTER_MODULE    SERCOM2

#define BNO055_I2C_SLAVE_ADDRESS  BNO055_I2C_ADDR2

/* Slave address select pin */
#define BNO055_PIN_SLAVE_ADDR_SELECT  EXT2_PIN_5

/* Reset */
#define BNO055_PIN_RESET              EXT2_PIN_5
#define BNO055_PIN_INT                EXT2_PIN_9

/* Sleep State Macros */
#define SLEEP_STATE_SLEEP    UINT8_C(1)
#define SLEEP_STATE_AWAKE    UINT8_C(0)

/* determines the active state of BNO055 reset */
#define BNO055_RESET_ACTIVE       false
/* the delay required to wait for BNO055 chip to reset */
#define BNO055_RESET_DELAY_MS     UINT32_C(650)

#endif /* __CONF_BNO055_H_INCLUDED__ */
