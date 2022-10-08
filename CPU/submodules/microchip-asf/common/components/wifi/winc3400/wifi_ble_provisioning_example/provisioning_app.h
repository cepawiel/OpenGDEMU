/**
 *
 * \file
 *
 * \brief WiFi Provisioning Declarations
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

#ifndef __PROVISIONING_APP_H__
#define __PROVISIONING_APP_H__
#include <asf.h>
#include "ble_manager.h"
#include "wifi_provisioning.h"
#include "common/include/nm_common.h"
uint8_t retrieve_credentials(credentials *cred);
void send_scan_result(tstrM2mWifiscanResult* sr,uint8_t remain);
void initialise_provisioning_app(void);
at_ble_status_t start_provisioning_app(void);
uint8_t provisioning_app_processing(void);
void provision_app_ble_disconnect(void);
void wifi_state_update(uint8_t);
at_ble_status_t wifi_provision_app_scanning_handler(void);
at_ble_status_t wifi_provision_app_credentials_update(credentials *c);
void ble_app_connected_update(at_ble_handle_t connection_handle);
void ble_app_paired_update(at_ble_handle_t paired_handle);
void ble_app_disconnected_update(at_ble_handle_t connection_handle);
#endif /* __PROVISIONING_APP_H__ */
