/**
 * \file
 *
 * \brief EMAC (Ethernet MAC) driver configuration file.
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

#ifndef CONF_EMAC_H_INCLUDED
#define CONF_EMAC_H_INCLUDED

/** Number of buffer for RX */
#define EMAC_RX_BUFFERS  16

/** Number of buffer for TX */
#define EMAC_TX_BUFFERS  8

/** MAC PHY operation max retry count */
#define MAC_PHY_RETRY_MAX 1000000

/** MAC address definition.  The MAC address must be unique on the network. */
#define ETHERNET_CONF_ETHADDR0                        0x00
#define ETHERNET_CONF_ETHADDR1                        0x04
#define ETHERNET_CONF_ETHADDR2                        0x25
#define ETHERNET_CONF_ETHADDR3                        0x1C
#define ETHERNET_CONF_ETHADDR4                        0xA0
#define ETHERNET_CONF_ETHADDR5                        0x02

/** WAN Address: 192.168.0.2 */
/* The IP address being used. */
#define ETHERNET_CONF_IPADDR0                         10
#define ETHERNET_CONF_IPADDR1                         217
#define ETHERNET_CONF_IPADDR2                         2
#define ETHERNET_CONF_IPADDR3                         145

/** WAN gateway: 192.168.0.1 */
/** The gateway address being used. */
#define ETHERNET_CONF_GATEWAY_ADDR0                   10
#define ETHERNET_CONF_GATEWAY_ADDR1                   217
#define ETHERNET_CONF_GATEWAY_ADDR2                   2
#define ETHERNET_CONF_GATEWAY_ADDR3                   250

/** The network mask being used. */
#define ETHERNET_CONF_NET_MASK0                       255
#define ETHERNET_CONF_NET_MASK1                       255
#define ETHERNET_CONF_NET_MASK2                       255
#define ETHERNET_CONF_NET_MASK3                       0

/** Ethernet MII/RMII mode */
#define ETH_PHY_MODE  BOARD_EMAC_MODE_RMII

#endif /* CONF_EMAC_H_INCLUDED */
