/*****************************************************************************
*
* \file
*
* \brief lwIP configuration for SAM.
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
*****************************************************************************/

#ifndef __LWIPOPTS_H__
#define __LWIPOPTS_H__

/* Include user defined options first */
// #include "conf_eth.h"
#include "conf_lwip_threads.h"

#include "lwip/debug.h"

/* Define default values for unconfigured parameters. */
#define LWIP_NOASSERT 1 /* To suppress some errors for now (no debug output) */

/* These two control is reclaimer functions should be compiled
 * in. Should always be turned on (1). */
#define MEM_RECLAIM             1
#define MEMP_RECLAIM            1


/* Platform specific locking */

/*
 * enable SYS_LIGHTWEIGHT_PROT in lwipopts.h if you want inter-task protection
 * for certain critical regions during buffer allocation, deallocation and memory
 * allocation and deallocation.
 */
#define SYS_LIGHTWEIGHT_PROT            1

/* ---------- Memory options ---------- */

/* MEM_ALIGNMENT: should be set to the alignment of the CPU for which
 * lwIP is compiled. 4 byte alignment -> define MEM_ALIGNMENT to 4, 2
 * byte alignment -> define MEM_ALIGNMENT to 2. */
#define MEM_ALIGNMENT           4

/* MEM_SIZE: the size of the heap memory. If the application will send
 * a lot of data that needs to be copied, this should be set high. */
#define MEM_SIZE                3 * 1024

/* MEMP_NUM_PBUF: the number of memp struct pbufs. If the application
 * sends a lot of data out of ROM (or other static memory), this
 * should be set high. */
#define MEMP_NUM_PBUF           6

/* Number of raw connection PCBs */
#define MEMP_NUM_RAW_PCB                1

/* ---------- UDP options ---------- */
  #define LWIP_UDP                1
  #define UDP_TTL                 255

/* MEMP_NUM_UDP_PCB: the number of UDP protocol control blocks. One
 * per active UDP "connection". */

/* required by DHCP (because DNS is used) */
  #define MEMP_NUM_UDP_PCB        1

#if (TFTP_USED == 1)

/* one PCB for DHCP (DNS used), one for TFTP */
  #undef MEMP_NUM_UDP_PCB
  #define MEMP_NUM_UDP_PCB        2

#endif

/* MEMP_NUM_TCP_PCB: the number of simultaneously active TCP connections. */
#define MEMP_NUM_TCP_PCB        2
/* MEMP_NUM_TCP_PCB_LISTEN: the number of listening TCP connections. */
#define MEMP_NUM_TCP_PCB_LISTEN 1
/* MEMP_NUM_TCP_SEG: the number of simultaneously queued TCP segments. */
#define MEMP_NUM_TCP_SEG        9
/* MEMP_NUM_SYS_TIMEOUT: the number of simulateously active timeouts. */
#define MEMP_NUM_SYS_TIMEOUT    6

/* The following four are used only with the sequential API and can be
 * set to 0 if the application only will use the raw API. */
/* MEMP_NUM_NETBUF: the number of struct netbufs. */
#define MEMP_NUM_NETBUF         3
/* MEMP_NUM_NETCONN: the number of struct netconns. */
#define MEMP_NUM_NETCONN        4

/* ---------- Pbuf options ---------- */
/* PBUF_POOL_SIZE: the number of buffers in the pbuf pool. */

#define PBUF_POOL_SIZE          6

/* PBUF_POOL_BUFSIZE: the size of each pbuf in the pbuf pool. */

#define PBUF_POOL_BUFSIZE       500

/** ETH_PAD_SIZE: number of bytes added before the ethernet header to ensure
 * alignment of payload after that header. Since the header is 14 bytes long,
 * without this padding e.g. addresses in the IP header will not be aligned
 * on a 32-bit boundary, so setting this to 2 can speed up 32-bit-platforms.
 */
#define ETH_PAD_SIZE                    0

/* PBUF_LINK_HLEN: the number of bytes that should be allocated for a
 * link level header. */
#define PBUF_LINK_HLEN          (16 + ETH_PAD_SIZE)

/* ---------- TCP options ---------- */
#define LWIP_TCP                1
#define TCP_TTL                 255
/* TCP receive window. */
#define TCP_WND                 1500

/* Controls if TCP should queue segments that arrive out of
 * order. Define to 0 if your device is low on memory. */
#define TCP_QUEUE_OOSEQ         1

/* TCP Maximum segment size. */
#define TCP_MSS                 1500

/* TCP sender buffer space (bytes). */
#define TCP_SND_BUF             2150

/* TCP sender buffer space (pbufs). This must be at least = 2 *
   TCP_SND_BUF/TCP_MSS for things to work. */
#define TCP_SND_QUEUELEN        ((6 * (TCP_SND_BUF) + (TCP_MSS - 1))/(TCP_MSS))



/* Maximum number of retransmissions of data segments. */
#define TCP_MAXRTX              12

/* Maximum number of retransmissions of SYN segments. */
#define TCP_SYNMAXRTX           4

/**
 * DEFAULT_RAW_RECVMBOX_SIZE: The mailbox size for the incoming packets on a
 * NETCONN_RAW. The queue size value itself is platform-dependent, but is passed
 * to sys_mbox_new() when the recvmbox is created.
 */
#define DEFAULT_RAW_RECVMBOX_SIZE       6

/**
 * DEFAULT_UDP_RECVMBOX_SIZE: The mailbox size for the incoming packets on a
 * NETCONN_UDP. The queue size value itself is platform-dependent, but is passed
 * to sys_mbox_new() when the recvmbox is created.
 */
#define DEFAULT_UDP_RECVMBOX_SIZE       6

/**
 * DEFAULT_TCP_RECVMBOX_SIZE: The mailbox size for the incoming packets on a
 * NETCONN_TCP. The queue size value itself is platform-dependent, but is passed
 * to sys_mbox_new() when the recvmbox is created.
 */
#define DEFAULT_TCP_RECVMBOX_SIZE       6

/**
 * DEFAULT_ACCEPTMBOX_SIZE: The mailbox size for the incoming connections.
 * The queue size value itself is platform-dependent, but is passed to
 * sys_mbox_new() when the acceptmbox is created.
 */
#define DEFAULT_ACCEPTMBOX_SIZE         6

/* ---------- ARP options ---------- */
#define ARP_TABLE_SIZE 10
#define ARP_QUEUEING 0

/* ---------- IP options ---------- */

/* Define IP_FORWARD to 1 if you wish to have the ability to forward
 * IP packets across network interfaces. If you are going to run lwIP
 * on a device with only one network interface, define this to 0. */
#define IP_FORWARD              0

/* If defined to 1, IP options are allowed (but not parsed). If
 * defined to 0, all packets with IP options are dropped. */
#define IP_OPTIONS              1

/* ---------- ICMP options ---------- */
#define ICMP_TTL                255

/* ---------- DHCP options ---------- */

/* Define LWIP_DHCP to 1 if you want DHCP configuration of
 * interfaces. DHCP is not implemented in lwIP 0.5.1, however, so
 * turning this on does currently not work. */
#define LWIP_DHCP               1

#ifdef LWIP_DHCP
#define DHCP_USED
#endif

/* 1 if you want to do an ARP check on the offered address
 * (recommended). */
/* #define DHCP_DOES_ARP_CHECK     1 */

/*
 * ------------------------------------
 * ---------- Thread options ----------
 * ------------------------------------
 */

/**
 * TCPIP_THREAD_NAME: The name assigned to the main tcpip thread.
 */
#define TCPIP_THREAD_NAME              "TCP/IP"

/**
 * TCPIP_THREAD_STACKSIZE: The stack size used by the main tcpip thread.
 * The stack size value itself is platform-dependent, but is passed to
 * sys_thread_new() when the thread is created.
 */
#define TCPIP_THREAD_STACKSIZE          lwipINTERFACE_STACK_SIZE

/**
 * TCPIP_THREAD_PRIO: The priority assigned to the main tcpip thread.
 * The priority value itself is platform-dependent, but is passed to
 * sys_thread_new() when the thread is created.
 */
#define TCPIP_THREAD_PRIO               lwipINTERFACE_TASK_PRIORITY

/**
 * TCPIP_MBOX_SIZE: The mailbox size for the tcpip thread messages
 * The queue size value itself is platform-dependent, but is passed to
 * sys_mbox_new() when tcpip_init is called.
 */
#define TCPIP_MBOX_SIZE                 6

/**
 * SLIPIF_THREAD_NAME: The name assigned to the slipif_loop thread.
 */
#define SLIPIF_THREAD_NAME             "slipif"

/**
 * SLIP_THREAD_STACKSIZE: The stack size used by the slipif_loop thread.
 * The stack size value itself is platform-dependent, but is passed to
 * sys_thread_new() when the thread is created.
 */
#define SLIPIF_THREAD_STACKSIZE         configMINIMAL_STACK_SIZE

/**
 * SLIPIF_THREAD_PRIO: The priority assigned to the slipif_loop thread.
 * The priority value itself is platform-dependent, but is passed to
 * sys_thread_new() when the thread is created.
 */
#define SLIPIF_THREAD_PRIO              1

/**
 * PPP_THREAD_NAME: The name assigned to the pppMain thread.
 */
#define PPP_THREAD_NAME                "pppInputThread"

/**
 * PPP_THREAD_STACKSIZE: The stack size used by the pppMain thread.
 * The stack size value itself is platform-dependent, but is passed to
 * sys_thread_new() when the thread is created.
 */
#define PPP_THREAD_STACKSIZE            configMINIMAL_STACK_SIZE

/**
 * PPP_THREAD_PRIO: The priority assigned to the pppMain thread.
 * The priority value itself is platform-dependent, but is passed to
 * sys_thread_new() when the thread is created.
 */
#define PPP_THREAD_PRIO                 1

/**
 * DEFAULT_THREAD_NAME: The name assigned to any other lwIP thread.
 */
#define DEFAULT_THREAD_NAME            "lwIP"

/**
 * DEFAULT_THREAD_STACKSIZE: The stack size used by any other lwIP thread.
 * The stack size value itself is platform-dependent, but is passed to
 * sys_thread_new() when the thread is created.
 */
#define DEFAULT_THREAD_STACKSIZE        configMINIMAL_STACK_SIZE

/**
 * DEFAULT_THREAD_PRIO: The priority assigned to any other lwIP thread.
 * The priority value itself is platform-dependent, but is passed to
 * sys_thread_new() when the thread is created.
 */
#define DEFAULT_THREAD_PRIO             1

#define LWIP_NETIF_STATUS_CALLBACK      1

/* ---------- Socket Options ---------- */

#ifdef __GNUC__
// Do not use the lwip timeval structure.
#define LWIP_TIMEVAL_PRIVATE            0
// Use the gnu libc struct timeval instead.
//#include <sys/time.h>
#endif

/* ---------- Statistics options ---------- */
#define LWIP_STATS 1

#define LWIP_STATS_DISPLAY 0

#if LWIP_STATS
#define LINK_STATS 1
#define IP_STATS   1
#define ICMP_STATS 1
#define UDP_STATS  1
#define TCP_STATS  1
#define MEM_STATS  1
#define MEMP_STATS 1
#define PBUF_STATS 1
#define SYS_STATS  1
#endif /* STATS */

/* ---------- Lwip Debug options ---------- */

#define DBG_TYPES_ON                    0xff

#define ETHARP_DEBUG                    LWIP_DBG_OFF

#define NETIF_DEBUG                     LWIP_DBG_OFF

#define PBUF_DEBUG                      LWIP_DBG_ON

#define API_LIB_DEBUG                   LWIP_DBG_OFF

#define API_MSG_DEBUG                   LWIP_DBG_ON

#define SOCKETS_DEBUG                   LWIP_DBG_OFF

#define ICMP_DEBUG                      LWIP_DBG_OFF

#define INET_DEBUG                      LWIP_DBG_OFF

#define IP_DEBUG                        LWIP_DBG_OFF

#define IP_REASS_DEBUG                  LWIP_DBG_OFF

#define RAW_DEBUG                       LWIP_DBG_OFF

#define MEM_DEBUG                       LWIP_DBG_OFF

#define MEMP_DEBUG                      LWIP_DBG_OFF

#define SYS_DEBUG                       LWIP_DBG_OFF

#define TCP_DEBUG                       LWIP_DBG_ON

#define TCP_INPUT_DEBUG                 LWIP_DBG_OFF

#define TCP_FR_DEBUG                    LWIP_DBG_OFF

#define TCP_RTO_DEBUG                   LWIP_DBG_OFF

#define TCP_CWND_DEBUG                  LWIP_DBG_OFF

#define TCP_WND_DEBUG                   LWIP_DBG_OFF

#define TCP_OUTPUT_DEBUG                LWIP_DBG_OFF

#define TCP_RST_DEBUG                   LWIP_DBG_OFF

#define TCP_QLEN_DEBUG                  LWIP_DBG_OFF

#define UDP_DEBUG                       LWIP_DBG_OFF

#define TCPIP_DEBUG                     LWIP_DBG_OFF

#define DBG_MIN_LEVEL                   LWIP_DBG_LEVEL_SEVERE

/* \note For a list of all possible lwIP configurations, check
 * http://lwip.wikia.com/wiki/Lwipopts.h */

#endif /* __LWIPOPTS_H__ */
