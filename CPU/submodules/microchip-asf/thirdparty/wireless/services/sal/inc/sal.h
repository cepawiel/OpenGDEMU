/**
 * @file sal.h
 *
 * @brief Declarations for low-level security API
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
#ifndef SAL_H
#define SAL_H

/* === Includes =========================================================== */

#include "sal_types.h"
#include "sal_generic.h"
#include "stdbool.h"
#include <stdint.h>

/* === Macros ============================================================= */

#if (SAL_TYPE != AT86RF2xx)

/*
 * Values for SPI based systems are defined in the corresponding header files
 * for these transceivers.
 */

/* Values for SR_AES_DIR */
#ifndef AES_DIR_ENCRYPT

/**
 * Defines AES direction as encryption
 */
#define AES_DIR_ENCRYPT              (0)
#endif
#ifndef AES_DIR_DECRYPT

/**
 * Defines AES direction as decryption
 */
#define AES_DIR_DECRYPT              (1)
#endif

/* Values for SR_AES_MODE */
#ifndef AES_MODE_ECB

/**
 * Defines AES mode as ECB
 */
#define AES_MODE_ECB                 (0)
#endif

/**
 * Defines AES mode as CBC
 */
#ifndef AES_MODE_CBC
#if (SAL_TYPE == ATXMEGA_SAL)
#define AES_MODE_CBC                 (2)
#else /*MegaRF*/
#define AES_MODE_CBC                 (1)
#endif
#endif
#else

/** Access parameters for sub-register AES_DIR in register @ref RG_AES_CTRL */
#define SR_AES_DIR   0x03, 0x08, 3

/** Access parameters for sub-register AES_MODE in register @ref RG_AES_CTRL */
#define SR_AES_MODE   0x03, 0x70, 4

/** Access parameters for sub-register AES_REQUEST in register @ref RG_AES_CTRL
**/
#define SR_AES_REQUEST   0x03, 0x80, 7

/** Base address for Transceiver AES address space **/
#define AES_BASE_ADDR   (0x80)

/** Offset for register AES_STATUS */
#define RG_AES_STATUS   (0x02)

/** Offset for register AES_CTRL */
#define RG_AES_CTRL   (0x03)

/** Offset for register AES_STATE_KEY_0 */
#define RG_AES_STATE_KEY_0   (0x04)

/** AES core operation direction: Decryption (ECB) */
#define AES_DIR_DECRYPT   (1)

/** AES core operation direction: Encryption (ECB, CBC) */
#define AES_DIR_ENCRYPT   (0)

/** Set CBC mode */
#define AES_MODE_CBC   (2)

/** Set ECB mode */
#define AES_MODE_ECB   (0)

/** Set key mode */
#define AES_MODE_KEY   (1)

/** Initiate an AES operation */
#define AES_REQUEST   (1)

/** AES core operation status: AES module finished */
#define AES_DONE   (1)

/** AES core operation status: AES module did not finish */
#define AES_NOT_DONE   (0)

#endif  /* (SAL_TYPE != AT86RF2xx) */

/* === Types ============================================================== */

/* === Externals ========================================================== */

/* === Prototypes ========================================================= */

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Initialization of SAL.
 *
 * This functions initializes the SAL.
 *
 * @ingroup group_SalApi
 */
void sal_init(void);

/**
 * @brief Reads the result of previous AES en/decryption
 *
 * This function returns the result of the previous AES operation,
 * so this function is needed in order to get the last result
 * of a series of sal_aes_wrrd() calls.
 *
 * @param[out] data     - result of previous operation
 *
 * @ingroup group_SalApi
 */
void sal_aes_read(uint8_t *data);

#if defined(__DOXYGEN__)

/**
 * @brief En/decrypt one AES block.
 *
 * The function returns after the AES operation is finished.
 *
 * @param[in]  data  AES block to be en/decrypted
 *
 * @ingroup group_SalApi
 */
void sal_aes_exec(uint8_t *data);

/**
 * @brief Writes data, reads previous result and does the AES en/decryption
 *
 * The function returns after the AES operation is finished.
 *
 * When sal_aes_wrrd() is called several times in sequence, from the
 * second call onwards, odata contains the result of the previous operation.
 * To obtain the last result, you must call sal_aes_read() at the end.
 * Please note that any call of sal_aes_setup() as well as putting
 * the transceiver to sleep state destroys the SRAM contents,
 * i.e. the next call of sal_aes_wrrd() yields no meaningful result.
 *
 * @param[in]  idata  AES block to be en/decrypted
 * @param[out] odata  Result of previous operation
 *                    (odata may be NULL or equal to idata)
 *
 * @ingroup group_SalApi
 */
void sal_aes_wrrd(uint8_t *idata, uint8_t *odata);

#else   /* !defined(__DOXYGEN__) */

#if (SAL_TYPE == ATMEGARF_SAL) || (SAL_TYPE == SW_AES_SAL) || \
	(SAL_TYPE == ATXMEGA_SAL)
void sal_aes_exec(uint8_t *data);

#elif (SAL_TYPE == AT86RF2xx)
void sal_aes_wrrd(uint8_t *idata, uint8_t *odata);

#else
#error "unknown SAL_TYPE"
#endif  /* SAL_TYPE */

#endif  /* __DOXYGEN__ */

/**
 * @brief Re-inits key and state after a sleep or TRX reset
 *
 * This function re-initializes the AES key and the state of the
 * AES engine after TRX sleep or reset.
 * The contents of AES register AES_CON is restored,
 * the next AES operation started with sal_aes_exec()
 * will be executed correctly.
 *
 * @ingroup group_SalApi
 */
void sal_aes_restart(void);

/**
 * @brief Cleans up the SAL/AES after STB has been completed
 *
 * This function puts the radio to SLEEP if it has been in SLEEP
 * before sal_aes_restart().
 *
 * @ingroup group_SalApi
 */
#if (SAL_TYPE == AT86RF2xx) || (SAL_TYPE == ATMEGARF_SAL) || \
	(defined __DOXYGEN__)
void _sal_aes_clean_up(void);

/** Route function macro to the corresponding function. */
#define sal_aes_clean_up()      _sal_aes_clean_up()
#endif

/**
 * @brief Setup AES unit
 *
 * This function perform the following tasks as part of the setup of the
 * AES unit: key initialization, set encryption mode.
 *
 * @param[in] key AES key or NULL (NULL: use last key)
 * @param[in] enc_mode  AES_MODE_ECB or AES_MODE_CBC
 * @param[in] dir must be AES_DIR_ENCRYPT
 *
 * @return  False if some parameter was illegal, true else
 *
 * @ingroup group_SalApi
 */
bool sal_aes_setup(uint8_t *key,
		uint8_t enc_mode,
		uint8_t dir);

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif /* SAL_H */
/* EOF */
