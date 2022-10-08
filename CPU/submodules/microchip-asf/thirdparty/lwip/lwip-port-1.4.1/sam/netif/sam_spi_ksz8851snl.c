/**
 * \file
 *
 * \brief KSZ8851SNL SAM driver for lwIP.
 *
 * Copyright (c) 2013-2018 Microchip Technology Inc. and its subsidiaries.
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
#include "lwip/sys.h"
#include "lwip/def.h"
#include "lwip/mem.h"
#include "lwip/pbuf.h"
#include "lwip/stats.h"
#include "lwip/snmp.h"
#include "netif/etharp.h"
#include "netif/ppp_oe.h"

#include <string.h>
#include "ksz8851snl.h"
#include "ksz8851snl_reg.h"
#include "netif/sam_spi_ksz8851snl.h"
#include "sysclk.h"
#include "spi.h"
#include "pdc.h"
#include "conf_eth.h"

/** Network interface identifier. */
#define IFNAME0								'e'
#define IFNAME1								'n'

/** Maximum transfer unit. */
#define NET_MTU								1500

/** Network link speed. */
#define NET_LINK_SPEED						100000000

/**
 * ksz8851snl driver structure.
 */
struct ksz8851snl_device {
	/** Set to 1 when owner is software (ready to read), 0 for Micrel. */
	uint32_t rx_desc[NETIF_RX_BUFFERS];
	/** Set to 1 when owner is Micrel, 0 for software. */
	uint32_t tx_desc[NETIF_TX_BUFFERS];
	/** RX pbuf pointer list */
	struct pbuf *rx_pbuf[NETIF_RX_BUFFERS];
	/** TX pbuf pointer list */
	struct pbuf *tx_pbuf[NETIF_TX_BUFFERS];
	struct pbuf *tx_cur_pbuf;

	/** Circular buffer head pointer for packet received. */
	uint32_t us_rx_head;
	/** Circular buffer tail pointer for packet to be read. */
	uint32_t us_rx_tail;
	/** Circular buffer head pointer by upper layer (buffer to be sent). */
	uint32_t us_tx_head;
	/** Circular buffer tail pointer incremented by handlers (buffer sent). */
	uint32_t us_tx_tail;

	/** Reference to lwIP netif structure. */
	struct netif *netif;

#if NO_SYS == 0
	/** RX task notification semaphore. */
	sys_sem_t sync_sem;
#endif
};

/**
 * ksz8851snl driver instance.
 */
COMPILER_ALIGNED(8)
static struct ksz8851snl_device gs_ksz8851snl_dev;

static uint16_t pending_frame = 0;

/**
 * MAC address to use.
 */
static uint8_t gs_uc_mac_address[] =
{
	ETHERNET_CONF_ETHADDR0,
	ETHERNET_CONF_ETHADDR1,
	ETHERNET_CONF_ETHADDR2,
	ETHERNET_CONF_ETHADDR3,
	ETHERNET_CONF_ETHADDR4,
	ETHERNET_CONF_ETHADDR5
};

#if LWIP_STATS
/** Used to compute lwIP bandwidth. */
uint32_t lwip_tx_count = 0;
uint32_t lwip_rx_count = 0;
uint32_t lwip_tx_rate = 0;
uint32_t lwip_rx_rate = 0;
#endif

volatile uint32_t g_intn_flag = 0;
volatile uint32_t g_spi_pdc_flag = 0;
extern Pdc *g_p_spi_pdc;

#define SPI_PDC_IDLE		0
#define SPI_PDC_RX_START	1
#define SPI_PDC_RX_COMPLETE	2
#define SPI_PDC_TX_START	3
#define SPI_PDC_TX_COMPLETE	4

extern uint8_t tmpbuf[];

/**
 * \brief Handler for SPI interrupt.
 */
void SPI_Handler(void)
{
	pdc_packet_t g_pdc_spi_tx_npacket;
	pdc_packet_t g_pdc_spi_rx_npacket;
	uint32_t status;

	status = spi_read_status(KSZ8851SNL_SPI);
	status &= spi_read_interrupt_mask(KSZ8851SNL_SPI);

	if (SPI_PDC_RX_START == g_spi_pdc_flag) {
		/* Transfer complete, disable SPI interrupt. */
		if (status & SPI_SR_RXBUFF) {
			spi_disable_interrupt(KSZ8851SNL_SPI, SPI_IDR_RXBUFF);
			g_spi_pdc_flag = SPI_PDC_RX_COMPLETE;
		}
	}
	else if (SPI_PDC_TX_START == g_spi_pdc_flag) {
		/* Middle of TX. */
		if (status & SPI_SR_ENDRX) {
			/* Fetch next pbuf in the pbuf chain. */
			if (gs_ksz8851snl_dev.tx_cur_pbuf)
				gs_ksz8851snl_dev.tx_cur_pbuf = gs_ksz8851snl_dev.tx_cur_pbuf->next;

			/* If next pbuf is available, enqueue for transfer. */
			if (gs_ksz8851snl_dev.tx_cur_pbuf) {
				g_pdc_spi_tx_npacket.ul_addr = (uint32_t) gs_ksz8851snl_dev.tx_cur_pbuf->payload;
				g_pdc_spi_tx_npacket.ul_size = gs_ksz8851snl_dev.tx_cur_pbuf->len;
				g_pdc_spi_rx_npacket.ul_addr = (uint32_t) tmpbuf;
				g_pdc_spi_rx_npacket.ul_size = gs_ksz8851snl_dev.tx_cur_pbuf->len;

				pdc_tx_init(g_p_spi_pdc, NULL, &g_pdc_spi_tx_npacket);
				pdc_rx_init(g_p_spi_pdc, NULL, &g_pdc_spi_rx_npacket);
			}
			else {
				/* Enable RX complete interrupt. */
				spi_disable_interrupt(KSZ8851SNL_SPI, SPI_IDR_ENDRX);
				spi_enable_interrupt(KSZ8851SNL_SPI, SPI_IER_RXBUFF);
			}
		}
		/* End of TX. */
		if (status & SPI_SR_RXBUFF) {
			/* Transfer complete, disable SPI interrupt. */
			spi_disable_interrupt(KSZ8851SNL_SPI, SPI_IDR_RXBUFF);
			g_spi_pdc_flag = SPI_PDC_TX_COMPLETE;
		}
	}
}

/**
 * \brief Handler for INTN falling edge interrupt.
 */
static void INTN_Handler(uint32_t id, uint32_t mask)
{
#if NO_SYS == 0
	portBASE_TYPE xKSZTaskWoken = pdFALSE;
#endif

	if ((INTN_ID == id) && (INTN_PIN_MSK == mask)) {
		/* Clear the PIO interrupt flags. */
		pio_get_interrupt_status(INTN_PIO);

		/* Set the INTN flag. */
		g_intn_flag = 1;

#if NO_SYS == 0
		xSemaphoreGiveFromISR(gs_ksz8851snl_dev.sync_sem, &xKSZTaskWoken);
		portEND_SWITCHING_ISR(xKSZTaskWoken);
#endif
	}
}

/**
 * \brief Populate the RX descriptor ring buffers with pbufs.
 *
 * \param p_ksz8851snl_dev Pointer to driver data structure.
 */
static void ksz8851snl_rx_populate_queue(struct ksz8851snl_device *p_ksz8851snl_dev)
{
	uint32_t ul_index = 0;
	struct pbuf *p = 0;

	/* Set up the RX descriptors */
	for (ul_index = 0; ul_index < NETIF_RX_BUFFERS; ul_index++) {
		if (p_ksz8851snl_dev->rx_pbuf[ul_index] == 0) {

			/* Allocate a new pbuf with the maximum size. */
			p = pbuf_alloc(PBUF_RAW, PBUF_POOL_BUFSIZE, PBUF_POOL);
			if (0 == p) {
				LWIP_DEBUGF(NETIF_DEBUG, ("ksz8851snl_rx_populate_queue: pbuf allocation failure\n"));
			}

			/* Make sure lwIP is well configured so one pbuf can contain the maximum packet size. */
			LWIP_ASSERT("ksz8851snl_rx_populate_queue: pbuf size too small!", pbuf_clen(p) <= 1);

			/* Set owner as Micrel. */
			p_ksz8851snl_dev->rx_desc[ul_index] = 0;

			/* Save pbuf pointer to be sent to lwIP upper layer. */
			p_ksz8851snl_dev->rx_pbuf[ul_index] = p;

			LWIP_DEBUGF(NETIF_DEBUG,
					("ksz8851snl_rx_populate_queue: new pbuf allocated with size %d: 0x%p [pos=%u]\n",
					PBUF_POOL_BUFSIZE, p, ul_index));
		}
	}
}

/**
 * \brief Update Micrel state machine and perform required actions.
 *
 * \param netif the lwIP network interface structure for this ethernetif.
 */
static void ksz8851snl_update(struct netif *netif)
{
	struct ksz8851snl_device *ps_ksz8851snl_dev = netif->state;
	uint16_t status = 0;
	uint16_t len = 0;
	uint16_t txmir = 0;

	/* Check for free PDC. */
	if (SPI_PDC_IDLE == g_spi_pdc_flag) {

		/* Handle TX. */
		/* Fetch next packet marked as owned by Micrel. */
		if (ps_ksz8851snl_dev->tx_desc[ps_ksz8851snl_dev->us_tx_tail]
				&& (pending_frame == 0)) {
			len = ps_ksz8851snl_dev->tx_pbuf[ps_ksz8851snl_dev->us_tx_tail]->tot_len;

			/* TX step1: check if TXQ memory size is available for transmit. */
			txmir = ksz8851_reg_read(REG_TX_MEM_INFO) & TX_MEM_AVAILABLE_MASK;
			if (txmir < len + 8) {
				LWIP_DEBUGF(NETIF_DEBUG,
						("ksz8851snl_update: TX not enough memory in queue: %d required %d\n",
						txmir, len + 8));
				return;
			}

			/* TX step2: disable all interrupts. */
			ksz8851_reg_write(REG_INT_MASK, 0);

			LWIP_DEBUGF(NETIF_DEBUG,
					("ksz8851snl_update: TX start packet transmit len=%d [tail=%u head=%u]\n",
					len,
					ps_ksz8851snl_dev->us_tx_tail, ps_ksz8851snl_dev->us_tx_head));

			/* TX step3: enable TXQ write access. */
			ksz8851_reg_setbits(REG_RXQ_CMD, RXQ_START);

			/* TX step4-8: perform FIFO write operation. */
			g_spi_pdc_flag = SPI_PDC_TX_START;
			ps_ksz8851snl_dev->tx_cur_pbuf = ps_ksz8851snl_dev->tx_pbuf[ps_ksz8851snl_dev->us_tx_tail];
			gpio_set_pin_low(KSZ8851SNL_CSN_GPIO);
			ksz8851_fifo_write(ps_ksz8851snl_dev->tx_cur_pbuf->payload,
					ps_ksz8851snl_dev->tx_cur_pbuf->tot_len,
					ps_ksz8851snl_dev->tx_cur_pbuf->len);
		}

		/* Handle RX. */
		else if (g_intn_flag || pending_frame > 0)
		{
			g_intn_flag = 0;

			if (0 == pending_frame) {
				/* RX step1: read interrupt status for INT_RX flag. */
				status = ksz8851_reg_read(REG_INT_STATUS);
				if (!(status & INT_RX)) {
					return;
				}

				/* RX step2: disable all interrupts. */
				ksz8851_reg_write(REG_INT_MASK, 0);

				/* RX step3: clear INT_RX flag. */
				ksz8851_reg_setbits(REG_INT_STATUS, INT_RX);

				/* RX step4-5: check for received frames. */
				pending_frame = ksz8851_reg_read(REG_RX_FRAME_CNT_THRES) >> 8;
				if (0 == pending_frame) {
					/* RX step24: enable INT_RX flag. */
					ksz8851_reg_write(REG_INT_MASK, INT_RX);
					return;
				}
			}

			/* Don't break Micrel state machine, wait for a free descriptor first! */
			if (ps_ksz8851snl_dev->rx_desc[ps_ksz8851snl_dev->us_rx_head]) {
				LWIP_DEBUGF(NETIF_DEBUG,
						("ksz8851snl_update: out of free descriptor! [tail=%u head=%u]\n",
						ps_ksz8851snl_dev->us_rx_tail, ps_ksz8851snl_dev->us_rx_head));
				return;
			}
			if (0 == ps_ksz8851snl_dev->rx_pbuf[ps_ksz8851snl_dev->us_rx_head]) {
				ksz8851snl_rx_populate_queue(ps_ksz8851snl_dev);
				LWIP_DEBUGF(NETIF_DEBUG,
						("ksz8851snl_update: descriptor with NULL pbuf! [head=%u]\n",
						ps_ksz8851snl_dev->us_rx_head));
				return;
			}

			/* RX step6: get RX packet status. */
			status = ksz8851_reg_read(REG_RX_FHR_STATUS);
			if (((status & RX_VALID) == 0) || (status & RX_ERRORS)) {
				ksz8851_reg_setbits(REG_RXQ_CMD, RXQ_CMD_FREE_PACKET);
				LWIP_DEBUGF(NETIF_DEBUG, ("ksz8851snl_update: RX packet error!\n"));
			}
			else {
				/* RX step7: read frame length. */
				len = ksz8851_reg_read(REG_RX_FHR_BYTE_CNT) & RX_BYTE_CNT_MASK;

				/* RX step8: Drop packet if len is invalid or no descriptor available. */
				if (0 == len) {
					ksz8851_reg_setbits(REG_RXQ_CMD, RXQ_CMD_FREE_PACKET);
					LWIP_DEBUGF(NETIF_DEBUG, ("ksz8851snl_update: RX bad len!\n"));
				}
				else {
					LWIP_DEBUGF(NETIF_DEBUG,
							("ksz8851snl_update: RX start packet receive len=%d [tail=%u head=%u]\n",
							len,
							ps_ksz8851snl_dev->us_rx_tail, ps_ksz8851snl_dev->us_rx_head));

					/* RX step9: reset RX frame pointer. */
					ksz8851_reg_clrbits(REG_RX_ADDR_PTR, ADDR_PTR_MASK);

					/* RX step10: start RXQ read access. */
					ksz8851_reg_setbits(REG_RXQ_CMD, RXQ_START);

					/* RX step11-17: start asynchronous FIFO read operation. */
					g_spi_pdc_flag = SPI_PDC_RX_START;
					gpio_set_pin_low(KSZ8851SNL_CSN_GPIO);
					ksz8851_fifo_read(ps_ksz8851snl_dev->rx_pbuf[ps_ksz8851snl_dev->us_rx_head]->payload, len);

					/* Remove CRC and update pbuf length. */
					len -= 4;
					ps_ksz8851snl_dev->rx_pbuf[ps_ksz8851snl_dev->us_rx_head]->len = len;
					ps_ksz8851snl_dev->rx_pbuf[ps_ksz8851snl_dev->us_rx_head]->tot_len = len;
				}
			}
		}
	}
	else if (SPI_PDC_RX_COMPLETE == g_spi_pdc_flag) {
		/* RX step18-19: pad with dummy data to keep dword alignment. */
		len = ps_ksz8851snl_dev->rx_pbuf[ps_ksz8851snl_dev->us_rx_head]->tot_len & 3;
		if (len) {
			ksz8851_fifo_dummy(4 - len);
		}

		/* RX step20: end RX transfer. */
		gpio_set_pin_high(KSZ8851SNL_CSN_GPIO);

		/* Disable asynchronous transfer mode. */
		g_spi_pdc_flag = SPI_PDC_IDLE;

		/* RX step21: end RXQ read access. */
		ksz8851_reg_clrbits(REG_RXQ_CMD, RXQ_START);

		/* RX step22-23: update frame count to be read. */
		pending_frame -= 1;

		/* RX step24: enable INT_RX flag if transfer complete. */
		if (0 == pending_frame) {
			ksz8851_reg_write(REG_INT_MASK, INT_RX);
		}

		/* Mark descriptor ready to be read. */
		ps_ksz8851snl_dev->rx_desc[ps_ksz8851snl_dev->us_rx_head] = 1;
		ps_ksz8851snl_dev->us_rx_head = (ps_ksz8851snl_dev->us_rx_head + 1) % NETIF_RX_BUFFERS;
	}
	else if (SPI_PDC_TX_COMPLETE == g_spi_pdc_flag) {
		/* TX step9-10: pad with dummy data to keep dword alignment. */
		len = ps_ksz8851snl_dev->tx_pbuf[ps_ksz8851snl_dev->us_tx_tail]->tot_len & 3;
		if (len) {
			ksz8851_fifo_dummy(4 - len);
		}

		/* TX step11: end TX transfer. */
		gpio_set_pin_high(KSZ8851SNL_CSN_GPIO);

		/* Disable asynchronous transfer mode. */
		g_spi_pdc_flag = SPI_PDC_IDLE;

		/* TX step12: disable TXQ write access. */
		ksz8851_reg_clrbits(REG_RXQ_CMD, RXQ_START);

		/* TX step12.1: enqueue frame in TXQ. */
		ksz8851_reg_setbits(REG_TXQ_CMD, TXQ_ENQUEUE);

		/* RX step13: enable INT_RX flag. */
		ksz8851_reg_write(REG_INT_MASK, INT_RX);

		/* Buffer sent, free the corresponding buffer and mark descriptor as owned by software. */
		pbuf_free(ps_ksz8851snl_dev->tx_pbuf[ps_ksz8851snl_dev->us_tx_tail]);
		ps_ksz8851snl_dev->tx_pbuf[ps_ksz8851snl_dev->us_tx_tail] = NULL;
		ps_ksz8851snl_dev->tx_desc[ps_ksz8851snl_dev->us_tx_tail] = 0;
		ps_ksz8851snl_dev->us_tx_tail = (ps_ksz8851snl_dev->us_tx_tail + 1) % NETIF_TX_BUFFERS;
	}
}

/**
 * \brief Set up the RX descriptor ring buffers.
 *
 * This function sets up the descriptor list used for RX packets.
 *
 * \param ps_ksz8851snl_dev Pointer to driver data structure.
 */
static void ksz8851snl_rx_init(struct ksz8851snl_device *ps_ksz8851snl_dev)
{
	uint32_t ul_index = 0;

	/* Init pointer index. */
	ps_ksz8851snl_dev->us_rx_head = 0;
	ps_ksz8851snl_dev->us_rx_tail = 0;

	/* Set up the RX descriptors. */
	for (ul_index = 0; ul_index < NETIF_RX_BUFFERS; ul_index++) {
		ps_ksz8851snl_dev->rx_pbuf[ul_index] = 0;
		ps_ksz8851snl_dev->rx_desc[ul_index] = 0;
	}

	/* Build RX buffer and descriptors. */
	ksz8851snl_rx_populate_queue(ps_ksz8851snl_dev);
}

/**
 * \brief Set up the TX descriptor ring buffers.
 *
 * This function sets up the descriptor list used for TX packets.
 *
 * \param ps_ksz8851snl_dev Pointer to driver data structure.
 */
static void ksz8851snl_tx_init(struct ksz8851snl_device *ps_ksz8851snl_dev)
{
	uint32_t ul_index = 0;

	/* Init TX index pointer. */
	ps_ksz8851snl_dev->us_tx_head = 0;
	ps_ksz8851snl_dev->us_tx_tail = 0;

	/* Set up the TX descriptors */
	for (ul_index = 0; ul_index < NETIF_TX_BUFFERS; ul_index++) {
		ps_ksz8851snl_dev->tx_desc[ul_index] = 0;
	}
}

/**
 * \brief Initialize ksz8851snl ethernet controller.
 *
 * \note Called from ethernetif_init().
 *
 * \param netif the lwIP network interface structure for this ethernetif.
 */
static void ksz8851snl_low_level_init(struct netif *netif)
{
	/* Set MAC hardware address length. */
	netif->hwaddr_len = sizeof(gs_uc_mac_address);
	/* Set MAC hardware address. */
	netif->hwaddr[0] = gs_uc_mac_address[0];
	netif->hwaddr[1] = gs_uc_mac_address[1];
	netif->hwaddr[2] = gs_uc_mac_address[2];
	netif->hwaddr[3] = gs_uc_mac_address[3];
	netif->hwaddr[4] = gs_uc_mac_address[4];
	netif->hwaddr[5] = gs_uc_mac_address[5];

	/* Set maximum transfer unit. */
	netif->mtu = NET_MTU;

	/* Device capabilities. */
	netif->flags |= NETIF_FLAG_BROADCAST | NETIF_FLAG_ETHARP
#if defined(DHCP_USED)
			| NETIF_FLAG_DHCP
#endif
	;

	ksz8851snl_rx_init(&gs_ksz8851snl_dev);
	ksz8851snl_tx_init(&gs_ksz8851snl_dev);

	/* Enable NVIC interrupts. */
	NVIC_SetPriority(SPI_IRQn, INT_PRIORITY_SPI);
	NVIC_EnableIRQ(SPI_IRQn);

	/* Initialize SPI link. */
	if (0 != ksz8851snl_init()) {
		LWIP_DEBUGF(NETIF_DEBUG,
				("ksz8851snl_low_level_init: failed to initialize the Micrel driver!\n"));
		LWIP_ASSERT("SPI communication issue", 1);
	}

	/* Initialize interrupt line INTN. */
	configure_intn(INTN_Handler);
}

/**
 * \brief This function should do the actual transmission of the packet. The
 * packet is contained in the pbuf that is passed to the function. This pbuf
 * might be chained.
 *
 * \param netif the lwIP network interface structure for this ethernetif.
 * \param p the MAC packet to send (e.g. IP packet including MAC addresses and type).
 *
 * \return ERR_OK if the packet could be sent.
 * an err_t value if the packet couldn't be sent.
 */
static err_t ksz8851snl_low_level_output(struct netif *netif, struct pbuf *p)
{
	struct ksz8851snl_device *ps_ksz8851snl_dev = netif->state;

	/* Make sure the next descriptor is free. */
	if (ps_ksz8851snl_dev->tx_desc[ps_ksz8851snl_dev->us_tx_head]) {
#if NO_SYS
		LWIP_DEBUGF(NETIF_DEBUG,
				("ksz8851snl_low_level_output: out of free descriptor! [tail=%u head=%u]\n",
				ps_ksz8851snl_dev->us_tx_tail, ps_ksz8851snl_dev->us_tx_head));
#endif
		return ERR_IF;
	}

	/* Ensure lwIP won't free this pbuf before the Micrel actually sends it. */
	pbuf_ref(p);

	/* Mark descriptor has owned by Micrel. Enqueue pbuf packet. */
	ps_ksz8851snl_dev->tx_desc[ps_ksz8851snl_dev->us_tx_head] = 1;
	ps_ksz8851snl_dev->tx_pbuf[ps_ksz8851snl_dev->us_tx_head] = p;

#if NO_SYS
	LWIP_DEBUGF(NETIF_DEBUG,
			("ksz8851snl_low_level_output: DMA buffer 0x%p sent, size=%u [tail=%u head=%u]\n",
			p->payload, p->tot_len, ps_ksz8851snl_dev->us_tx_tail, ps_ksz8851snl_dev->us_tx_head));
#endif

	ps_ksz8851snl_dev->us_tx_head = (ps_ksz8851snl_dev->us_tx_head + 1) % NETIF_TX_BUFFERS;

#if LWIP_STATS
	lwip_tx_count += p->tot_len;
#endif
	LINK_STATS_INC(link.xmit);

#if NO_SYS == 0
	/* Release KSZ task to perform packet transfer. */
	xSemaphoreGive(ps_ksz8851snl_dev->sync_sem);
#endif

	return ERR_OK;
}

/**
 * \brief Use pre-allocated pbuf as DMA source and return the incoming packet.
 *
 * \param netif the lwIP network interface structure for this ethernetif.
 *
 * \return a pbuf filled with the received packet (including MAC header).
 * 0 on memory error.
 */
static struct pbuf *ksz8851snl_low_level_input(struct netif *netif)
{
	struct ksz8851snl_device *ps_ksz8851snl_dev = netif->state;
	struct pbuf *p = 0;

	/* Check that descriptor is owned by software (ie packet received). */
	if (ps_ksz8851snl_dev->rx_desc[ps_ksz8851snl_dev->us_rx_tail]) {

		/* Fetch pre-allocated pbuf. */
		p = ps_ksz8851snl_dev->rx_pbuf[ps_ksz8851snl_dev->us_rx_tail];

		/* Remove this pbuf from its descriptor. */
		ps_ksz8851snl_dev->rx_pbuf[ps_ksz8851snl_dev->us_rx_tail] = 0;

		LWIP_DEBUGF(NETIF_DEBUG,
				("ksz8851snl_low_level_input: DMA buffer 0x%p received, size=%u [tail=%u head=%u]\n",
				p->payload, p->tot_len, ps_ksz8851snl_dev->us_rx_tail, ps_ksz8851snl_dev->us_rx_head));

		/* Set pbuf total packet size. */
		LINK_STATS_INC(link.recv);

		/* Fill empty descriptors with new pbufs. */
		ksz8851snl_rx_populate_queue(ps_ksz8851snl_dev);

		ps_ksz8851snl_dev->us_rx_tail = (ps_ksz8851snl_dev->us_rx_tail + 1) % NETIF_RX_BUFFERS;

#if LWIP_STATS
		lwip_rx_count += p->tot_len;
#endif
	}

	return p;
}

#if NO_SYS == 0
/**
 * \brief This function waits for the notification
 * semaphore from the interrupt, processes the incoming packet and then
 * passes it to the lwIP stack.
 *
 * \param pvParameters A pointer to the ksz8851snl_device instance.
 */
static void ksz8851snl_task(void *pvParameters)
{
	struct ksz8851snl_device *ps_ksz8851snl_dev = pvParameters;

	while (1) {
		/* Block if no transfer pending. */
		if ((SPI_PDC_IDLE == g_spi_pdc_flag) &&
				(0 == pending_frame) &&
				(0 == ps_ksz8851snl_dev->tx_desc[ps_ksz8851snl_dev->us_tx_tail])) {
			/* Wait for the RX notification semaphore. */
			sys_arch_sem_wait(&ps_ksz8851snl_dev->sync_sem, 0);
		}

		/* Process the incoming packet. */
		ethernetif_input(ps_ksz8851snl_dev->netif);
	}
}
#endif

/**
 * \brief This function should be called when a packet is ready to be
 * read from the interface. It uses the function ksz8851snl_low_level_input()
 * that handles the actual reception of bytes from the network interface.
 * Then the type of the received packet is determined and the appropriate
 * input function is called.
 *
 * \param netif the lwIP network interface structure for this ethernetif.
 */
void ethernetif_input(struct netif *netif)
{
	struct eth_hdr *ethhdr;
	struct pbuf *p;

	/* Update driver state machine. */
	ksz8851snl_update(netif);

	/* Move received packet into a new pbuf. */
	p = ksz8851snl_low_level_input(netif);
	if (p == NULL)
		return;

	/* Points to packet payload, which starts with an Ethernet header. */
	ethhdr = p->payload;

	switch (htons(ethhdr->type)) {
		case ETHTYPE_IP:
		case ETHTYPE_ARP:
#if PPPOE_SUPPORT
		case ETHTYPE_PPPOEDISC:
		case ETHTYPE_PPPOE:
#endif /* PPPOE_SUPPORT */
			/* Send packet to lwIP for processing. */
			if (netif->input(p, netif) != ERR_OK) {
				LWIP_DEBUGF(NETIF_DEBUG, ("ethernetif_input: IP input error\n"));
				/* Free buffer. */
				pbuf_free(p);
			}
			break;

		default:
			/* Free buffer. */
			pbuf_free(p);
			break;
	}
}

/**
 * \brief Should be called at the beginning of the program to set up the
 * network interface. It calls the function ksz8851snl_low_level_init() to do the
 * actual setup of the hardware.
 *
 * \param netif the lwIP network interface structure for this ethernetif.
 *
 * \return ERR_OK if the loopif is initialized.
 * ERR_MEM if private data couldn't be allocated.
 * any other err_t on error.
 */
err_t ethernetif_init(struct netif *netif)
{
	LWIP_ASSERT("netif != NULL", (netif != NULL));

	gs_ksz8851snl_dev.netif = netif;

#if LWIP_NETIF_HOSTNAME
	/* Initialize interface hostname */
	netif->hostname = "ksz8851snldev";
#endif /* LWIP_NETIF_HOSTNAME */

	/*
	 * Initialize the snmp variables and counters inside the struct netif.
	 * The last argument should be replaced with your link speed, in units
	 * of bits per second.
	 */
#if LWIP_SNMP
	NETIF_INIT_SNMP(netif, snmp_ifType_ethernet_csmacd, NET_LINK_SPEED);
#endif /* LWIP_SNMP */

	netif->state = &gs_ksz8851snl_dev;
	netif->name[0] = IFNAME0;
	netif->name[1] = IFNAME1;

	/* We directly use etharp_output() here to save a function call.
	 * You can instead declare your own function an call etharp_output()
	 * from it if you have to do some checks before sending (e.g. if link
	 * is available...) */
	netif->output = etharp_output;
	netif->linkoutput = ksz8851snl_low_level_output;
	/* Initialize the hardware. */
	ksz8851snl_low_level_init(netif);

#if NO_SYS == 0
	err_t err;
	sys_thread_t id;

	/* Incoming packet notification semaphore. */
	err = sys_sem_new(&gs_ksz8851snl_dev.sync_sem, 0);
	LWIP_ASSERT("ethernetif_init: ksz8851snl RX semaphore allocation ERROR!\n",
			(err == ERR_OK));
	if (err == ERR_MEM)
		return ERR_MEM;

	id = sys_thread_new("ksz8851", ksz8851snl_task, &gs_ksz8851snl_dev,
			netifINTERFACE_TASK_STACK_SIZE, netifINTERFACE_TASK_PRIORITY);
	LWIP_ASSERT("ethernetif_init: ksz8851snl Task allocation ERROR!\n",
			(id != 0));
	if (id == 0)
		return ERR_MEM;
#endif

	return ERR_OK;
}
