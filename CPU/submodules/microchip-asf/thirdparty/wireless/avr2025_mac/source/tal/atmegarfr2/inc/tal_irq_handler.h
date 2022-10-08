/**
 * \file tal_irq_handler.h
 *
 * \brief This header file contains the interrupt handling definitions
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

/*
 * Copyright (c) 2013-2018 Microchip Technology Inc. and its subsidiaries.
 *
 * Licensed under Atmel's Limited License Agreement --> EULA.txt
 */

/* Prevent double inclusion */
#ifndef TAL_IRQ_HANDLER_H
#define TAL_IRQ_HANDLER_H

/* === INCLUDES ============================================================ */

/* === EXTERNALS =========================================================== */

#if (defined BEACON_SUPPORT) || (defined ENABLE_TSTAMP) || (defined __DOXYGEN__)
extern uint32_t pal_tx_timestamp;
#endif

/* === TYPES =============================================================== */

/* === MACROS ============================================================== */

/**
 * \brief Clears the transceiver RX_END interrupt
 *
 */
#define pal_trx_irq_flag_clr_rx_end()   (CLEAR_TRX_IRQ_RX_END())

/**
 * \brief Clears the transceiver TX_END interrupt
 *
 */
#define pal_trx_irq_flag_clr_tx_end()   (CLEAR_TRX_IRQ_TX_END())

/**
 * \brief Clears the transceiver CCA_ED_END interrupt
 *
 */
#define pal_trx_irq_flag_clr_cca_ed()   (CLEAR_TRX_IRQ_CCA_ED())

/**
 * \brief Clears the transceiver AWAKE interrupt
 *
 */
#define pal_trx_irq_flag_clr_awake()    (CLEAR_TRX_IRQ_AWAKE())

#ifdef ENABLE_ALL_TRX_IRQS

/**
 * \brief Clears the transceiver AMI interrupt
 *
 */
#define pal_trx_irq_flag_clr_ami()      (CLEAR_TRX_IRQ_AMI())

/**
 * \brief Clears the transceiver BATMON interrupt
 *
 */
#define pal_trx_irq_flag_clr_batmon()   (CLEAR_TRX_IRQ_BATMON())

/**
 * \brief Clears the transceiver PLL_LOCK interrupt
 *
 */
#define pal_trx_irq_flag_clr_pll_lock() (CLEAR_TRX_IRQ_PLL_LOCK())

/**
 * \brief Clears the transceiver PLL_UNLOCK interrupt
 *
 */
#define pal_trx_irq_flag_clr_pll_unlock()   (CLEAR_TRX_IRQ_PLL_UNLOCK())

/**
 * \brief Clears the transceiver TX_START interrupt
 *
 */
#define pal_trx_irq_flag_clr_tx_start()     (CLEAR_TRX_IRQ_TX_START())

/**
 * \brief Clears the transceiver MAF AMI0 interrupt
 *
 */
#define pal_trx_irq_flag_clr_maf_0_ami()    (CLEAR_TRX_IRQ_MAF_0_AMI())

/**
 * \brief Clears the transceiver MAF AMI1 interrupt
 *
 */
#define pal_trx_irq_flag_clr_maf_1_ami()    (CLEAR_TRX_IRQ_MAF_1_AMI())

/**
 * \brief Clears the transceiver MAF AMI2 interrupt
 *
 */
#define pal_trx_irq_flag_clr_maf_2_ami()    (CLEAR_TRX_IRQ_MAF_2_AMI())

/**
 * \brief Clears the transceiver MAF AMI3 interrupt
 *
 */
#define pal_trx_irq_flag_clr_maf_3_ami()    (CLEAR_TRX_IRQ_MAF_3_AMI())

#endif  /* ENABLE_ALL_TRX_IRQS */

/* === PROTOTYPES ========================================================== */

/**
 * \brief Initializes the transceiver RX END interrupt
 *
 * This function sets the microcontroller specific registers
 * responsible for handling the transceiver RX END interrupt
 *
 * \param trx_irq_cb Callback function for the transceiver RX END interrupt
 * \ingroup group_pal_interrupt
 */
void pal_trx_irq_init_rx_end(FUNC_PTR trx_irq_cb);

/**
 * \brief Initializes the transceiver TX END interrupt
 *
 * This function sets the microcontroller specific registers
 * responsible for handling the transceiver TX END interrupt
 *
 * \param trx_irq_cb Callback function for the transceiver TX END interrupt
 * \ingroup group_pal_interrupt
 */
void pal_trx_irq_init_tx_end(FUNC_PTR trx_irq_cb);

/**
 * \brief Initializes the transceiver CCA ED END interrupt
 *
 * This function sets the microcontroller specific registers
 * responsible for handling the transceiver CCA ED END interrupt
 *
 * \param trx_irq_cb Callback function for the transceiver CCA ED END interrupt
 * \ingroup group_pal_interrupt
 */
void pal_trx_irq_init_cca_ed(FUNC_PTR trx_irq_cb);

/**
 * \brief Initializes the transceiver AWAKE interrupt
 *
 * This function sets the microcontroller specific registers
 * responsible for handling the transceiver AWAKE interrupt
 *
 * \param trx_irq_cb Callback function for the transceiver AWAKE interrupt
 * \ingroup group_pal_interrupt
 */
void pal_trx_irq_init_awake(FUNC_PTR trx_irq_cb);

#ifdef ENABLE_ALL_TRX_IRQS

/**
 * \brief Initializes the transceiver AMI interrupt
 *
 * This function sets the microcontroller specific registers
 * responsible for handling the transceiver AMI interrupt
 *
 * \param trx_irq_cb Callback function for the transceiver AMI interrupt
 * \ingroup group_pal_interrupt
 */
void pal_trx_irq_init_ami(FUNC_PTR trx_irq_cb);

/**
 * \brief Initializes the transceiver BATMON interrupt
 *
 * This function sets the microcontroller specific registers
 * responsible for handling the transceiver BATMON interrupt
 *
 * \param trx_irq_cb Callback function for the transceiver BATMON interrupt
 * \ingroup group_pal_interrupt
 */
void pal_trx_irq_init_batmon(FUNC_PTR trx_irq_cb);

/**
 * \brief Initializes the transceiver PLL_LOCK interrupt
 *
 * This function sets the microcontroller specific registers
 * responsible for handling the transceiver PLL_LOCK interrupt
 *
 * \param trx_irq_cb Callback function for the transceiver PLL_LOCK interrupt
 * \ingroup group_pal_interrupt
 */
void pal_trx_irq_init_pll_lock(FUNC_PTR trx_irq_cb);

/**
 * \brief Initializes the transceiver PLL_UNLOCK interrupt
 *
 * This function sets the microcontroller specific registers
 * responsible for handling the transceiver PLL_UNLOCK interrupt
 *
 * \param trx_irq_cb Callback function for the transceiver PLL_UNLOCK interrupt
 * \ingroup group_pal_interrupt
 */
void pal_trx_irq_init_pll_unlock(FUNC_PTR trx_irq_cb);

/**
 * \brief Initializes the transceiver AES_READY interrupt
 *
 * This function sets the microcontroller specific registers
 * responsible for handling the transceiver AES_READY interrupt
 *
 * \param trx_irq_cb Callback function for the transceiver AES_READY interrupt
 * \ingroup group_pal_interrupt
 */
void pal_trx_irq_init_aes_ready(FUNC_PTR trx_irq_cb);

/**
 * \brief Initializes the transceiver TX_START interrupt
 *
 * This function sets the microcontroller specific registers
 * responsible for handling the transceiver TX_START interrupt
 *
 * \param trx_irq_cb Callback function for the transceiver TX_START interrupt
 * \ingroup group_pal_interrupt
 */
void pal_trx_irq_init_tx_start(FUNC_PTR trx_irq_cb);

/**
 * \brief Initializes the transceiver MAF AMI0 interrupt
 *
 * This function sets the microcontroller specific registers
 * responsible for handling the transceiver MAF AMI0 interrupt
 *
 * \param trx_irq_cb Callback function for the transceiver MAF AMI0 interrupt
 * \ingroup group_pal_interrupt
 */
void pal_trx_irq_init_maf_0_ami(FUNC_PTR trx_irq_cb);

/**
 * \brief Initializes the transceiver MAF AMI1 interrupt
 *
 * This function sets the microcontroller specific registers
 * responsible for handling the transceiver MAF AMI1 interrupt
 *
 * \param trx_irq_cb Callback function for the transceiver MAF AMI1 interrupt
 * \ingroup group_pal_interrupt
 */
void pal_trx_irq_init_maf_1_ami(FUNC_PTR trx_irq_cb);

/**
 * \brief Initializes the transceiver MAF AMI2 interrupt
 *
 * This function sets the microcontroller specific registers
 * responsible for handling the transceiver MAF AMI2 interrupt
 *
 * \param trx_irq_cb Callback function for the transceiver MAF AMI2 interrupt
 * \ingroup group_pal_interrupt
 */
void pal_trx_irq_init_maf_2_ami(FUNC_PTR trx_irq_cb);

/**
 * \brief Initializes the transceiver MAF AMI3 interrupt
 *
 * This function sets the microcontroller specific registers
 * responsible for handling the transceiver MAF AMI3 interrupt
 *
 * \param trx_irq_cb Callback function for the transceiver MAF AMI3 interrupt
 * \ingroup group_pal_interrupt
 */
void pal_trx_irq_init_maf_3_ami(FUNC_PTR trx_irq_cb);

#endif  /* ENABLE_ALL_TRX_IRQS */

#ifdef PAL_XTD_IRQ_API

/**
 * \brief Returns the current callback function for the transceiver RX END
 * interrupt
 *
 * \return Current callback function for the transceiver RX END interrupt
 * \ingroup group_pal_interrupt
 */
FUNC_PTR pal_trx_irq_get_hdlr_rx_end(void);

/**
 * \brief Returns the current callback function for the transceiver TX END
 * interrupt
 *
 * \return Current callback function for the transceiver TX END interrupt
 * \ingroup group_pal_interrupt
 */
FUNC_PTR pal_trx_irq_get_hdlr_tx_end(void);

#endif  /* PAL_XTD_IRQ_API */

typedef void (*irq_handler_t)(void);

#ifdef __cplusplus
extern "C" {
#endif

/**
 * \addtogroup group_tal_irq_rfr2
 * @{
 */

/**
 * \brief Transceiver interrupt handler
 *
 * This function handles the transceiver generated interrupts for RX end.
 */

void trx_rx_end_handler_cb(void);

/**
 * \brief Transceiver interrupt handler
 *
 * This function handles the transceiver generated interrupts for TX end.
 */

void trx_tx_end_handler_cb(void);

/*
 * \brief Transceiver interrupt handler for awake end IRQ
 *
 * This function handles the transceiver awake end interrupt.
 */
void trx_awake_handler_cb(void);

#if (defined BEACON_SUPPORT) || (defined ENABLE_TSTAMP) || (defined __DOXYGEN__)

/**
 * \brief Timestamp interrupt handler
 *
 * This function handles the interrupts handling the timestamp.
 *
 * The timestamping is only required for
 * - beaconing networks or if timestamping is explicitly enabled,
 * - and if antenna diversity is NOT enabled,
 * - and if the time stamp interrupt is not explicitly disabled.
 */
void trx_irq_timestamp_handler_cb(void);

#endif

/* ! @} */
#ifdef __cplusplus
} /* extern "C" */
#endif

#endif /* TAL_IRQ_HANDLER_H */

/* EOF */
