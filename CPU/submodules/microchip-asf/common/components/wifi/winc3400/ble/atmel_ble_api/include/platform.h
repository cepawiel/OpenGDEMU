/**
 *
 * \file platform.h
 *
 * \brief WINC3400 BLE platform APIs declarations
 *
 * Copyright (c) 2017-2019 Microchip Technology Inc. and its subsidiaries.
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

#ifndef __PLATFORM_H__
#define __PLATFORM_H__

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "at_ble_api.h"

#define VERSION_FIELD_VALID (0x5A5A5A5A)
typedef struct {
    int (*ble_write_cb)(uint8_t*, uint32_t);
    void (*plf_wait_cb)(void);
	uint32_t mv;
	struct fw_version {
		uint8_t major;
		uint8_t minor;
		uint8_t patch;
	} fw_ver;
}plf_params_t;

/**
@defgroup platform Platform API

@{
*/

 /** @brief implements platform-specific initialization
  *
  * @param[in] platform_params platform specific params, this pointer is passed from the
  * at_ble_init function and interpreted by the platform
  *
  * @return AT_BLE_SUCCESS operation completed successfully
  * @return AT_BLE_FAILURE Generic error.
  */
at_ble_status_t platform_init(plf_params_t* platform_params);

 /** @brief sends a message over the platform-specific bus
  *
  * Sends a message over the platform-specific bus that might be UART, SPI or other
  * if the BTLC1000 external wakeup signal is used, it is up to this function implementation
  * to assert/deassert it appropriately
  *
  * @param[in] data data to send over the interface
  * @param[in] len length of data
  * @return Zero if operation completed successfully
  * @return Non zero for Generic error.
  *
  */
at_ble_status_t platform_interface_send(uint8_t* data, uint32_t len);

void platform_receive(uint8_t* data, uint32_t len);

 /** @brief the callback to upper layers to process received packets
  *
  * This function is implemented by the upper layers (the event loop) and it is up
  * to the platform implementation to call it whenever data are received from the interface
  *
  * @param[in] data data received from the interface
  * @param[in] len length of data
  *
  */
void platform_interface_callback(uint8_t* data, uint32_t len);

 /** @brief fires the command-complete signal
  *  @note more details at the platform porting guide
  *
  */
void platform_cmd_cmpl_signal(void);

 /** @brief blocks until the command-complete signal is fired
  *  @note more details at the platform porting guide
  *
  * @return non-zero if Timeout
  * @return zero if Operation completed successfully
  */
at_ble_status_t platform_cmd_cmpl_wait(void);

 /** @brief fires the event signal
  *  @note more details at the platform porting guide
  *
  */
void platform_event_signal(void);

 /** @brief blocks until the event signal is fired
  *  @note more details at the platform porting guide
  *
  * @param[in] timeout timeout in ms passed by user
  *
  */
at_ble_status_t platform_event_wait(uint32_t timeout);

 /** @brief set timeout value used for wait functions.
  *  @note more details at the platform porting guide
  *
  * @param[in] timeout timeout in ms passed by user
  *
  */
void platform_set_timeout(uint32_t timeout);
 /** @}*/

#endif // __PLATFORM_H__
