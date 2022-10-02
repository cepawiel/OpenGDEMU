/**
 *
 * \file
 *
 * \brief WINC3400 Flash Interface.
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

/**@defgroup FLASHAPI Flash
*/

#ifndef __M2M_FLASH_H__
#define __M2M_FLASH_H__

/*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*
INCLUDES
*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*/
#include "common/include/nm_common.h"
#include "driver/include/m2m_types.h"

/**@defgroup  FlashEnums Enumeration/Typedefs
 * @ingroup FLASHAPI
 * @{*/

/*!
@enum   tenuWincFlashRegion

@brief  The regions of WINC flash available for accessing.
*/
typedef enum
{
    /*! Access to the entire flash address space. */
    WINC_FLASH_REGION_RAW,
    /*! Access to the active firmware region. */
    WINC_FLASH_REGION_FIRMWARE_ACTIVE,
    /*! Access to the inactive firmware region. */
    WINC_FLASH_REGION_FIRMWARE_INACTIVE,
    /*! Access to PLL table region. Erase is not supported for this region. */
    WINC_FLASH_REGION_PLL_TABLE,
    /*! Access to gain table region. Erase is not supported for this region. */
    WINC_FLASH_REGION_GAIN_TABLE,
    /*! Access to PLL and gain tables region. Erase is supported for this region. */
    WINC_FLASH_REGION_PLL_AND_GAIN_TABLES,
    /*! Access to TLS root certificate region. */
    WINC_FLASH_REGION_ROOT_CERTS,
    /*! Access to TLS local certificate region. */
    WINC_FLASH_REGION_LOCAL_CERTS,
    /*! Access to connection parameters region. */
    WINC_FLASH_REGION_CONN_PARAM,
    /*! Access to HTTP files region. This region is also erased/written by erase/write of firmware regions. */
    WINC_FLASH_REGION_HTTP_FILES,
    /*! Number of regions. */
    WINC_FLASH_NUM_REGIONS
} tenuWincFlashRegion;

/**@}
*/

/*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*
FUNCTION PROTOTYPES
*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*/
/** \defgroup FLASHFUNCTIONS Functions
*  @ingroup FLASHAPI
*/
/**@{*/

/*!
@fn \
    sint8 m2m_flash_erase_sector(tenuWincFlashRegion enuRegion, uint8 u8StartSector, uint8 u8NumSectors);

@brief  Erase one or more sectors within a WINC flash region.

@param [in]     enuRegion
                    Region of flash in which to erase.
@param [in]     u8StartSector
                    Sector offset (within region) of first sector to erase.
@param [in]     u8NumSectors
                    Number of sectors to erase.

@return     M2M_SUCCESS            - The erase completed successfully.
            M2M_ERR_INVALID_ARG    - The parameters were invalid.
            M2M_ERR_FAIL           - The request encountered an error.

@pre        The WINC must have been initialised but must not be running. This can be done via
            @ref m2m_wifi_init_hold, @ref m2m_wifi_reinit_hold or @ref m2m_wifi_download_mode.

*/
sint8 m2m_flash_erase_sector(tenuWincFlashRegion enuRegion, uint8 u8StartSector, uint8 u8NumSectors);

/*!
@fn \
    sint8 m2m_flash_write(tenuWincFlashRegion enuRegion, void *pvBuffer, uint32 u32Offset, uint32 u32Size);

@brief  Write the number of bytes specified to a WINC flash location.

@param [in]     enuRegion
                    Region of flash in which to write.
@param [in]     pvBuffer
                    Pointer to buffer containing the data to write.
@param [in]     u32Offset
                    Byte offset (within region) of write location.
@param [in]     u32Size
                    Number of bytes to write.

@return     M2M_SUCCESS            - The write completed successfully.
            M2M_ERR_INVALID_ARG    - The parameters were invalid.
            M2M_ERR_FAIL           - The request encountered an error.

@pre        The WINC must have been initialised but must not be running. This can be done via
            @ref m2m_wifi_init_hold, @ref m2m_wifi_reinit_hold or @ref m2m_wifi_download_mode.

@warning    The write operation does not erase the flash device first, therefore the user
            must ensure the flash region has been erased first.
*/
sint8 m2m_flash_write(tenuWincFlashRegion enuRegion, void *pvBuffer, uint32 u32Offset, uint32 u32Size);

/*!
@fn \
    sint8 m2m_flash_read(tenuWincFlashRegion enuRegion, void *pvBuffer, uint32 u32Offset, uint32 u32Size);

@brief  Read the number of bytes specified from a WINC flash location.

@param [in]     enuRegion
                    Region of flash from which to read.
@param [out]    pvBuffer
                    Pointer to buffer to populate with the data being read.
@param [in]     u32Offset
                    Byte offset (within region) of read location.
@param [in]     u32Size
                    Number of bytes to read.

@return     M2M_SUCCESS            - The read completed successfully.
            M2M_ERR_INVALID_ARG    - The parameters were invalid.
            M2M_ERR_FAIL           - The request encountered an error.

@pre        The WINC must have been initialised but must not be running. This can be done via
            @ref m2m_wifi_init_hold, @ref m2m_wifi_reinit_hold or @ref m2m_wifi_download_mode.
*/
sint8 m2m_flash_read(tenuWincFlashRegion enuRegion, void *pvBuffer, uint32 u32Offset, uint32 u32Size);

/*!
@fn \
    sint8 m2m_flash_switch_firmware(void);

@brief  Switch the active and inactive firmware images.

@return     M2M_SUCCESS            - The firmware switch completed successfully.
            M2M_ERR_FAIL           - The request encountered an error.

@pre        The WINC must have been initialised but must not be running. This can be done via
            @ref m2m_wifi_init_hold, @ref m2m_wifi_reinit_hold or @ref m2m_wifi_download_mode.

*/
sint8 m2m_flash_switch_firmware(void);

/**@}*/
#endif /* __M2M_FLASH_H__ */
