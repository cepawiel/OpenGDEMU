/**
 *
 * \file
 *
 * \brief WINC Flash Interface.
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



/*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*
INCLUDES
*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*/
#include "driver/include/m2m_flash.h"
#include "spi_flash/include/spi_flash.h"
#include "spi_flash/include/spi_flash_map.h"
#include "driver/source/nmdrv.h"

/*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*
TYPEDEFS
*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*/

typedef struct
{
    uint32 address;
    uint32 size;
} tstrFlashMapEntry;

/*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*
GLOBALS
*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*/

static const tstrFlashMapEntry flashMap[] =
{
    {0                              , 0                                 },  // WINC_FLASH_REGION_RAW,
    {0                              , OTA_IMAGE_SIZE                    },  // WINC_FLASH_REGION_FIRMWARE_ACTIVE,
    {0                              , OTA_IMAGE_SIZE                    },  // WINC_FLASH_REGION_FIRMWARE_INACTIVE,
    {M2M_PLL_FLASH_OFFSET           , M2M_PLL_FLASH_SZ                  },  // WINC_FLASH_REGION_PLL_TABLE,
    {M2M_GAIN_FLASH_OFFSET          , M2M_GAIN_FLASH_SZ                 },  // WINC_FLASH_REGION_GAIN_TABLE,
    {M2M_PLL_FLASH_OFFSET           , M2M_PLL_FLASH_SZ+M2M_GAIN_FLASH_SZ},  // WINC_FLASH_REGION_PLL_AND_GAIN_TABLES,
    {M2M_TLS_ROOTCER_FLASH_OFFSET   , M2M_TLS_ROOTCER_FLASH_SZ          },  // WINC_FLASH_REGION_ROOT_CERTS,
    {M2M_TLS_SERVER_FLASH_OFFSET    , M2M_TLS_SERVER_FLASH_SZ           },  // WINC_FLASH_REGION_LOCAL_CERTS,
    {M2M_CACHED_CONNS_FLASH_OFFSET  , M2M_CACHED_CONNS_FLASH_SZ         },  // WINC_FLASH_REGION_CONN_PARAM,
    {0                              , M2M_HTTP_MEM_FLASH_SZ             },  // WINC_FLASH_REGION_HTTP_FILES,
};

/*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*
FUNCTIONS
*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*=*/

static sint8 winc_flash_compare(uint8 *pu8Buf, uint32 u32Offset, uint32 u32Size)
{
    sint8 ret = M2M_SUCCESS;
    uint8 buf[128];
    uint32 offset = 0;

    while(offset < u32Size)
    {
        uint32 chunk_sz = sizeof(buf);
        if(chunk_sz > u32Size - offset)
            chunk_sz = u32Size - offset;
        ret = spi_flash_read(buf, u32Offset + offset, chunk_sz);
        if(ret != M2M_SUCCESS)
            break;
        ret = m2m_memcmp(buf, pu8Buf + offset, chunk_sz);
        if(ret != 0)
            break;
        offset += chunk_sz;
    }
    return ret;
}
static sint8 winc_flash_write_verify(uint8 *pu8Buf, uint32 u32Offset, uint32 u32Size)
{
    sint8   ret = M2M_ERR_FAIL;
    uint8   count = 20;

    while((ret != M2M_SUCCESS) && (count-- > 0))
    {
        ret = spi_flash_write(pu8Buf, u32Offset, u32Size);
        if(ret == M2M_SUCCESS)
            ret = winc_flash_compare(pu8Buf, u32Offset, u32Size);
    }
    return ret;
}

/* Some internal functions for accessing the control structure. */
static uint8 crc7(uint8 crc, const uint8 *buff, uint16 len)
{
    uint8 reg = crc;
    uint16 i;
    for(i = 0; i < len; i++)
    {
        uint16 g;
        for(g = 0; g < 8; g++)
        {
            uint8 inv = (((buff[i] << g) & 0x80) >> 7) ^ ((reg >> 6) & 1);
            reg = ((reg << 1) & 0x7f) ^ (9 * inv);
        }
    }
    return reg;
}
static sint8 verify_control_structure(tstrOtaControlSec *pstrControlSec)
{
    sint8   s8Ret = M2M_SUCCESS;

    if(pstrControlSec->u32OtaMagicValue != OTA_MAGIC_VALUE)
        s8Ret = M2M_ERR_FAIL;
    if(pstrControlSec->u32OtaControlSecCrc != crc7(0x7f, (uint8 *)pstrControlSec, sizeof(tstrOtaControlSec) - 4))
        s8Ret = M2M_ERR_FAIL;

    return s8Ret;
}
static sint8 read_control_structure(tstrOtaControlSec *pstrControlSec)
{
    sint8   s8Ret;

    s8Ret = spi_flash_read((uint8 *)pstrControlSec, M2M_CONTROL_FLASH_OFFSET, sizeof(tstrOtaControlSec));
    if(s8Ret == M2M_SUCCESS)
        s8Ret = verify_control_structure(pstrControlSec);

    if(s8Ret != M2M_SUCCESS)
    {
        s8Ret = spi_flash_read((uint8 *)pstrControlSec, M2M_BACKUP_FLASH_OFFSET, sizeof(tstrOtaControlSec));
        if(s8Ret == M2M_SUCCESS)
            s8Ret = verify_control_structure(pstrControlSec);
    }

    return s8Ret;
}
static sint8 update_control_structure(tstrOtaControlSec *pstrControlSec)
{
    sint8 ret = M2M_ERR_FAIL;

    ret = spi_flash_erase(M2M_BACKUP_FLASH_OFFSET, M2M_BACKUP_FLASH_SZ);
    if(ret == M2M_SUCCESS)
    {
        pstrControlSec->u32OtaSequenceNumber++;
        pstrControlSec->u32OtaControlSecCrc = crc7(0x7f, (uint8 *)pstrControlSec, sizeof(tstrOtaControlSec) - 4);
        ret = winc_flash_write_verify((uint8 *)pstrControlSec, M2M_BACKUP_FLASH_OFFSET, sizeof(tstrOtaControlSec));
        if(ret == M2M_SUCCESS)
        {
            ret = spi_flash_erase(M2M_CONTROL_FLASH_OFFSET, M2M_CONTROL_FLASH_SZ);
            if(ret == M2M_SUCCESS)
            {
                pstrControlSec->u32OtaSequenceNumber++;
                pstrControlSec->u32OtaControlSecCrc = crc7(0x7f, (uint8 *)pstrControlSec, sizeof(tstrOtaControlSec) - 4);
                ret = winc_flash_write_verify((uint8 *)pstrControlSec, M2M_CONTROL_FLASH_OFFSET, sizeof(tstrOtaControlSec));
            }
        }
    }
    return ret;
}

static bool find_flash_section(tenuWincFlashRegion enuRegion, uint32 *pu32StartAddr, uint32 *pu32Size)
{
    /* Ensure the pointers and region are valid. */
    if((NULL == pu32StartAddr) || (NULL == pu32Size) || (enuRegion >= WINC_FLASH_NUM_REGIONS))
        return false;

    /* For the raw region resolve the full flash space, otherwise lookup
       the region location and size from the flexible flash map. */
    switch(enuRegion)
    {
    case WINC_FLASH_REGION_RAW:
        *pu32StartAddr = 0;
        *pu32Size = spi_flash_get_size() << 17;
        break;
    case WINC_FLASH_REGION_FIRMWARE_ACTIVE:
    case WINC_FLASH_REGION_FIRMWARE_INACTIVE:
    case WINC_FLASH_REGION_HTTP_FILES:
    {
        /* In these cases we need to read the control structure to find the appropriate flash address. */
        tstrOtaControlSec strControl;

        /* Check the WINC is initialised and not running. */
        if (NM_STATE_INIT != nm_get_state())
            return false;

        /* Read control structure from flash. */
        if (M2M_SUCCESS != read_control_structure(&strControl))
            return false;

        if (WINC_FLASH_REGION_FIRMWARE_INACTIVE == enuRegion)
            *pu32StartAddr = strControl.u32OtaRollbackImageOffset;
        else if (WINC_FLASH_REGION_FIRMWARE_ACTIVE == enuRegion)
            *pu32StartAddr = strControl.u32OtaCurrentWorkingImagOffset;
        else if (WINC_FLASH_REGION_HTTP_FILES == enuRegion)
            *pu32StartAddr = strControl.u32OtaCurrentWorkingImagOffset + (M2M_HTTP_MEM_FLASH_OFFSET - M2M_OTA_IMAGE1_OFFSET);

        *pu32Size      = flashMap[enuRegion].size;
        break;
    }
    default:
        *pu32StartAddr = flashMap[enuRegion].address;
        *pu32Size      = flashMap[enuRegion].size;
        break;
    }

    M2M_INFO("Flash lookup %2d: 0x%06lx %0ld\r\n", enuRegion, *pu32StartAddr, *pu32Size);

    return true;
}

sint8 m2m_flash_erase_sector(tenuWincFlashRegion enuRegion, uint8 u8StartSector, uint8 u8NumSectors)
{
    uint32 flashAddress;
    uint32 flashRegionSize;

    /* Check the WINC is initialised and not running. */
    if(NM_STATE_INIT != nm_get_state())
        return M2M_ERR_FAIL;

    /* Check the region is valid. */
    if(enuRegion >= WINC_FLASH_NUM_REGIONS)
        return M2M_ERR_INVALID_ARG;

    /* Find region address and size. */
    if(false == find_flash_section(enuRegion, &flashAddress, &flashRegionSize))
        return M2M_ERR_FAIL;

    /* Erase is only supported for regions which begin on a sector boundary. */
    if(flashAddress & (FLASH_SECTOR_SZ-1))
        return M2M_ERR_INVALID_ARG;

    /* Check requested size fits within region size. */
    if((((uint32)u8StartSector + u8NumSectors) * FLASH_SECTOR_SZ) > flashRegionSize)
        return M2M_ERR_FAIL;

    /* Find start address of area within requested region. */
    flashAddress += (u8StartSector * FLASH_SECTOR_SZ);

    /* Erase the requested sectors. */
    if(M2M_SUCCESS != spi_flash_erase(flashAddress, u8NumSectors * FLASH_SECTOR_SZ))
        return M2M_ERR_FAIL;

    return M2M_SUCCESS;
}

sint8 m2m_flash_write(tenuWincFlashRegion enuRegion, void *pvBuffer, uint32 u32Offset, uint32 u32Size)
{
    uint32 flashAddress;
    uint32 flashRegionSize;

    /* Check the WINC is initialised and not running. */
    if(NM_STATE_INIT != nm_get_state())
        return M2M_ERR_FAIL;

    /* Check the buffer pointer and region are valid. */
    if((NULL == pvBuffer) || (enuRegion >= WINC_FLASH_NUM_REGIONS))
        return M2M_ERR_INVALID_ARG;

    /* Find region address and size. */
    if(false == find_flash_section(enuRegion, &flashAddress, &flashRegionSize))
        return M2M_ERR_FAIL;

    /* Check requested size fits within region size. Also check for wraparound. */
    if(((u32Offset + u32Size) > flashRegionSize) || ((uint32)(u32Offset + u32Size) < u32Offset))
        return M2M_ERR_FAIL;

    /* Find start address of area within requested region. */
    flashAddress += u32Offset;

    /* Write data to flash. */
    if(M2M_SUCCESS != spi_flash_write(pvBuffer, flashAddress, u32Size))
        return M2M_ERR_FAIL;

    return M2M_SUCCESS;
}

sint8 m2m_flash_read(tenuWincFlashRegion enuRegion, void *pvBuffer, uint32 u32Offset, uint32 u32Size)
{
    uint32 flashAddress;
    uint32 flashRegionSize;

    /* Check the WINC is initialised and not running. */
    if(NM_STATE_INIT != nm_get_state())
        return M2M_ERR_FAIL;

    /* Check the buffer pointer and region are valid. */
    if((NULL == pvBuffer) || (enuRegion >= WINC_FLASH_NUM_REGIONS))
        return M2M_ERR_INVALID_ARG;

    /* Find region address and size. */
    if(false == find_flash_section(enuRegion, &flashAddress, &flashRegionSize))
        return M2M_ERR_FAIL;

    /* Check requested size fits within region size. Also check for wraparound. */
    if(((u32Offset + u32Size) > flashRegionSize) || ((uint32)(u32Offset + u32Size) < u32Offset))
        return M2M_ERR_FAIL;

    /* Find start address of area within requested region. */
    flashAddress += u32Offset;

    /* Read data from flash. */
    if(M2M_SUCCESS != spi_flash_read(pvBuffer, flashAddress, u32Size))
        return M2M_ERR_FAIL;

    return M2M_SUCCESS;
}

sint8 m2m_flash_switch_firmware(void)
{
    tstrOtaControlSec   strControl;
    uint32              u32Tmp;

    /* Check the WINC is initialised and not running. */
    if(NM_STATE_INIT != nm_get_state())
        return M2M_ERR_FAIL;

    /* Read control structure from flash. */
    if(M2M_SUCCESS != read_control_structure(&strControl))
        return M2M_ERR_FAIL;

    /* Switch active and inactive. */
    u32Tmp = strControl.u32OtaRollbackImageOffset;
    strControl.u32OtaRollbackImageOffset = strControl.u32OtaCurrentWorkingImagOffset;
    strControl.u32OtaCurrentWorkingImagOffset = u32Tmp;

    /* Ensure the inactive image is marked as invalid. This protects m2m_ota_switch_firmware from
       switching to an image whose validity is unknown. Switching remains possible via this API. */
    strControl.u32OtaRollbackImageValidStatus = OTA_STATUS_INVALID;

    if(M2M_SUCCESS != update_control_structure(&strControl))
        return M2M_ERR_FAIL;

    return M2M_SUCCESS;
}
