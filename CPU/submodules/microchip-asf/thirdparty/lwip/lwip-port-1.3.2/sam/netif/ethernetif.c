/**
 *
 * \file
 *
 * \brief This file is a skeleton for developing Ethernet network interface
 * drivers for lwIP. Add code to the low_level functions and do a
 * search-and-replace for the word "ethernetif" to replace it with
 * something that better describes your network interface.
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

#include "lwip/opt.h"

#include "lwip/def.h"
#include "lwip/mem.h"
#include "lwip/pbuf.h"
#include "lwip/sys.h"
#include <lwip/stats.h>
#include <lwip/snmp.h>
#include "netif/etharp.h"
#include "netif/ppp_oe.h"

#include "pmc.h"
#include "emac.h"
#include "rstc.h"
#include "ethernet_phy.h"
#include "netif/ethernetif.h"
#include "sysclk.h"
#include <string.h>
#include "conf_eth.h"

#include "gpio.h"

/** Define those to better describe your network interface */
#define IFNAME0 'e'
#define IFNAME1 'n'

#ifdef FREERTOS_USED
#define netifGUARD_BLOCK_NBTICKS       (250)
#endif

/** Maximum transfer unit */
#define NET_MTU  1500

/** Network link speed */
#define NET_LINK_SPEED  100000000

/** Read/write buffer size for lwIP */
#define NET_RW_BUFF_SIZE 1536

/** The MAC address used for the test */
static uint8_t gs_uc_mac_address[] =
{ ETHERNET_CONF_ETHADDR0, ETHERNET_CONF_ETHADDR1, ETHERNET_CONF_ETHADDR2,
  ETHERNET_CONF_ETHADDR3, ETHERNET_CONF_ETHADDR4, ETHERNET_CONF_ETHADDR5};

/** The EMAC driver instance */
static emac_device_t gs_emac_dev;

/**
 * Helper struct to hold private data used to operate your ethernet interface.
 * Keeping the ethernet address of the MAC in this struct is not necessary
 * as it is already kept in the struct netif.
 * But this is only an example, anyway...
 */
struct ethernetif {
	struct eth_addr *ethaddr;
	/* Add whatever per-interface state that is needed here. */
};

/**
 * \brief EMAC interrupt handler.
 */
void EMAC_Handler(void)
{
	emac_handler(&gs_emac_dev);
}

/**
 * \brief In this function, the hardware should be initialized.
 * Called from ethernetif_init().
 *
 * \param netif the already initialized lwip network interface structure
 *        for this ethernetif
 */
static void low_level_init(struct netif *netif)
{
	volatile uint32_t ul_dealy;
	emac_options_t emac_option;

#ifdef FREERTOS_USED
	unsigned portBASE_TYPE uxPriority;
#endif

	/* Set MAC hardware address length */
	netif->hwaddr_len = sizeof(gs_uc_mac_address);
	/* Set MAC hardware address */
	netif->hwaddr[0] = gs_uc_mac_address[0];
	netif->hwaddr[1] = gs_uc_mac_address[1];
	netif->hwaddr[2] = gs_uc_mac_address[2];
	netif->hwaddr[3] = gs_uc_mac_address[3];
	netif->hwaddr[4] = gs_uc_mac_address[4];
	netif->hwaddr[5] = gs_uc_mac_address[5];

	/* Maximum transfer unit */
	netif->mtu = NET_MTU;

	/* Configure EMAC pins */
	gpio_configure_pin(PIN_EEMAC_EREFCK, PIN_EMAC_FLAGS);
	gpio_configure_pin(PIN_EMAC_ETX0, PIN_EMAC_FLAGS);
	gpio_configure_pin(PIN_EMAC_ETX1, PIN_EMAC_FLAGS);
	gpio_configure_pin(PIN_EMAC_ETXEN, PIN_EMAC_FLAGS);
	gpio_configure_pin(PIN_EMAC_ECRSDV, PIN_EMAC_FLAGS);
	gpio_configure_pin(PIN_EMAC_ERX0, PIN_EMAC_FLAGS);
	gpio_configure_pin(PIN_EMAC_ERX1, PIN_EMAC_FLAGS);
	gpio_configure_pin(PIN_EMAC_ERXER, PIN_EMAC_FLAGS);
	gpio_configure_pin(PIN_EMAC_EMDC, PIN_EMAC_FLAGS);
	gpio_configure_pin(PIN_EMAC_EMDIO, PIN_EMAC_FLAGS);

	/* device capabilities */
	/* don't set NETIF_FLAG_ETHARP if this device is not an ethernet one */
	netif->flags |= NETIF_FLAG_BROADCAST | NETIF_FLAG_ETHARP
#if defined(DHCP_USED)
			| NETIF_FLAG_DHCP
#endif
	;

#ifdef FREERTOS_USED
	/*
	* NOTE: This routine contains code that polls status bits. If the Ethernet
	* cable is not plugged in then this can take a considerable time.  To prevent
	* this from starving lower priority tasks of processing time we lower our
	* priority prior to the call, then raise it back again once the initialization
	* is complete.
	*/

	/* Read the priority of the current task. */
	uxPriority = uxTaskPriorityGet( NULL );
	/* Set the priority of the current task to the lowest possible. */
	vTaskPrioritySet( NULL, tskIDLE_PRIORITY );
#endif

	/* Reset PHY */
	rstc_set_external_reset(RSTC, 13);      /* (2^(13+1))/32768 */
	rstc_reset_extern(RSTC);
	while (rstc_get_status(RSTC) & RSTC_SR_NRSTL) {
	}

	/* Wait for PHY to be ready (CAT811: Max400ms) */
	ul_dealy = sysclk_get_cpu_hz() / 6;
	while (ul_dealy--) {
	}

	/* Enable EMAC clock */
	pmc_enable_periph_clk(ID_EMAC);

	/* Fill in EMAC options */
	emac_option.uc_copy_all_frame = 0;
	emac_option.uc_no_boardcast = 0;

	memcpy(emac_option.uc_mac_addr, gs_uc_mac_address,
			sizeof(gs_uc_mac_address));

	gs_emac_dev.p_hw = EMAC;

	/* Init EMAC driver structure */
	emac_dev_init(EMAC, &gs_emac_dev, &emac_option);

	/* Enable Interrupt */
	NVIC_EnableIRQ(EMAC_IRQn);

	/* Init MAC PHY driver */
	if (ethernet_phy_init(EMAC, BOARD_EMAC_PHY_ADDR,
			sysclk_get_cpu_hz()) != EMAC_OK) {
		LWIP_DEBUGF(LWIP_DBG_TRACE, "PHY Initialize ERROR!\r");
		return;
	}

	/* Auto Negotiate, work in RMII mode */
	if (ethernet_phy_auto_negotiate(EMAC, BOARD_EMAC_PHY_ADDR) != EMAC_OK) {
		LWIP_DEBUGF(LWIP_DBG_TRACE, "Auto Negotiate ERROR!\r");
		return;
	}

	/* Establish ethernet link */
	while (ethernet_phy_set_link(EMAC, BOARD_EMAC_PHY_ADDR, 1) != EMAC_OK) {
		LWIP_DEBUGF(LWIP_DBG_TRACE, "Set link ERROR!\r");
	}

#ifdef FREERTOS_USED
	/* Restore the priority of the current task. */
	vTaskPrioritySet( NULL, uxPriority );

	/* Create the task that handles the EMAC input packets. */
	sys_thread_new( "ETHINT", ethernetif_input, netif,
			netifINTERFACE_TASK_STACK_SIZE,
			netifINTERFACE_TASK_PRIORITY );
#endif
}

/**
 * \brief This function should do the actual transmission of the packet. The
 * packet is contained in the pbuf that is passed to the function. This pbuf
 * might be chained.
 * note: Returning ERR_MEM here if a DMA queue of your MAC is full can lead to
 *       strange results. You might consider waiting for space in the DMA queue
 *       to become available since the stack doesn't retry to send a packet
 *       dropped because of memory failure (except for the TCP timers).
 *
 * \param netif the lwip network interface structure for this ethernetif
 * \param p the MAC packet to send (e.g. IP packet including MAC addresses and type)
 *
 * \return ERR_OK if the packet could be sent
 *         an err_t value if the packet couldn't be sent.
 */
static err_t low_level_output(struct netif *netif, struct pbuf *p)
{
	struct pbuf *q = NULL;
	int8_t pc_buf[NET_RW_BUFF_SIZE];
	int8_t *bufptr = &pc_buf[0];
	uint8_t uc_rc;

#if ETH_PAD_SIZE
	pbuf_header(p, -ETH_PAD_SIZE);    /* Drop the padding word */
#endif

	/* Check the buffer boundary */
	if (p->tot_len > NET_RW_BUFF_SIZE) {
		return ERR_BUF;
	}

	/* Clear the output buffer */
	memset(bufptr, 0x0, NET_RW_BUFF_SIZE);

	for (q = p; q != NULL; q = q->next) {
		/* Send the data from the pbuf to the interface, one pbuf at a
		 * time. The size of the data in each pbuf is kept in the ->len
		 * variable. */

		/* Send data from(q->payload, q->len); */
		memcpy(bufptr, q->payload, q->len);
		bufptr += q->len;
	}

	/* Signal that packet should be sent(); */
	uc_rc = emac_dev_write(&gs_emac_dev, pc_buf, p->tot_len, NULL);
	if (uc_rc != EMAC_OK) {
		return ERR_BUF;
	}

#if ETH_PAD_SIZE
	pbuf_header(p, ETH_PAD_SIZE);    /* Reclaim the padding word */
#endif

	LINK_STATS_INC(link.xmit);

	return ERR_OK;
}

/**
 * \brief Should allocate a pbuf and transfer the bytes of the incoming
 * packet from the interface into the pbuf.
 *
 * \param netif the lwip network interface structure for this ethernetif.
 *
 * \return a pbuf filled with the received packet (including MAC header)
 *         NULL on memory error.
 */
static struct pbuf *low_level_input(struct netif *netif)
{
	struct pbuf *p = NULL, *q = NULL;
	u16_t s_len;
	uint8_t pc_buf[NET_RW_BUFF_SIZE];
	int8_t *bufptr = (int8_t *)&pc_buf[0];

	uint32_t ul_frmlen;
	uint8_t uc_rc;

	/* Obtain the size of the packet and put it into the "len"
	 * variable. */
	uc_rc = emac_dev_read(&gs_emac_dev, pc_buf, sizeof(pc_buf), &ul_frmlen);
	if (uc_rc != EMAC_OK) {
		return NULL;
	}

	s_len = ul_frmlen;

#if ETH_PAD_SIZE
	s_len += ETH_PAD_SIZE;    /* allow room for Ethernet padding */
#endif

	/* We allocate a pbuf chain of pbufs from the pool. */
	p = pbuf_alloc(PBUF_RAW, s_len, PBUF_POOL);

	if (p != NULL) {
#if ETH_PAD_SIZE
		pbuf_header(p, -ETH_PAD_SIZE);  /* drop the padding word */
#endif

		/* Iterate over the pbuf chain until we have read the entire
		 * packet into the pbuf. */
		for (q = p; q != NULL; q = q->next) {
			/* Read enough bytes to fill this pbuf in the chain. The
			 * available data in the pbuf is given by the q->len
			 * variable. */
			/* read data into(q->payload, q->len); */
			memcpy(q->payload, bufptr, q->len);
			bufptr += q->len;
		}
		/* Acknowledge that packet has been read(); */

#if ETH_PAD_SIZE
		pbuf_header(p, ETH_PAD_SIZE);    /* Reclaim the padding word */
#endif

		LINK_STATS_INC(link.recv);
	} else {
		/* Drop packet(); */
		LINK_STATS_INC(link.memerr);
		LINK_STATS_INC(link.drop);
	}

	return p;
}

/**
 * \brief This function should be called when a packet is ready to be
 * read from the interface. It uses the function low_level_input()
 * that should handle the actual reception of bytes from the network
 * interface. Then the type of the received packet is determined and
 * the appropriate input function is called.
 *
 * \param pv_parameters the lwip network interface structure for this
 * ethernetif.
 */
void ethernetif_input(void * pvParameters)
{
	struct netif      *netif = (struct netif *)pvParameters;
	struct pbuf       *p;

#ifdef FREERTOS_USED
	for( ;; ) {
		do {
#endif
			/* move received packet into a new pbuf */
			p = low_level_input( netif );
			if( p == NULL ) {
#ifdef FREERTOS_USED
				/* No packet could be read.  Wait a for an interrupt to tell us
				there is more data available. */
				vTaskDelay(100);
			}
		}while( p == NULL );
#else
				return;
			}
#endif

		if( ERR_OK != netif->input( p, netif ) ) {
			pbuf_free(p);
			p = NULL;
		}
#ifdef FREERTOS_USED
	}
#endif
}

/**
 * \brief Should be called at the beginning of the program to set up the
 * network interface. It calls the function low_level_init() to do the
 * actual setup of the hardware.
 *
 * This function should be passed as a parameter to netif_add().
 *
 * \param netif the lwip network interface structure for this ethernetif.
 *
 * \return ERR_OK if the loopif is initialized
 *         ERR_MEM if private data couldn't be allocated
 *         any other err_t on error.
 */
err_t ethernetif_init(struct netif *netif)
{
	LWIP_ASSERT("netif != NULL", (netif != NULL));

#if LWIP_NETIF_HOSTNAME
	/* Initialize interface hostname */
	netif->hostname = "lwip";
#endif /* LWIP_NETIF_HOSTNAME */

	/*
	 * Initialize the snmp variables and counters inside the struct netif.
	 * The last argument should be replaced with your link speed, in units
	 * of bits per second.
	 */
#if LWIP_SNMP
	NETIF_INIT_SNMP(netif, snmp_ifType_ethernet_csmacd, NET_LINK_SPEED);
#endif /* LWIP_SNMP */

	netif->state = NULL;
	netif->name[0] = IFNAME0;
	netif->name[1] = IFNAME1;

	/* We directly use etharp_output() here to save a function call.
	 * You can instead declare your own function an call etharp_output()
	 * from it if you have to do some checks before sending (e.g. if link
	 * is available...) */
	netif->output = etharp_output;
	netif->linkoutput = low_level_output;

	/* Initialize the hardware */
	low_level_init(netif);

	return ERR_OK;
}
