/**
 *
 * \file
 *
 * \brief This module contains M2M host interface APIs implementation.
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

#include "common/include/nm_common.h"
#include "driver/source/nmbus.h"
#include "driver/source/nmdrv.h"
#include "bsp/include/nm_bsp.h"
#include "m2m_hif.h"
#include "driver/include/m2m_types.h"
#include "driver/source/nmasic.h"
#include "driver/include/m2m_periph.h"
#include "socket/include/m2m_socket_host_if.h"

#if (defined NM_EDGE_INTERRUPT)&&(defined NM_LEVEL_INTERRUPT)
#error "only one type of interrupt NM_EDGE_INTERRUPT,NM_LEVEL_INTERRUPT"
#endif

#if !((defined NM_EDGE_INTERRUPT)||(defined NM_LEVEL_INTERRUPT))
#error "define interrupt type NM_EDGE_INTERRUPT,NM_LEVEL_INTERRUPT"
#endif

#ifndef CORTUS_APP
#define NMI_AHB_DATA_MEM_BASE  0x30000
#define NMI_AHB_SHARE_MEM_BASE 0xd0000

#define WIFI_HOST_RCV_CTRL_0    (0x1070)
#define WIFI_HOST_RCV_CTRL_1    (0x1084)
#define WIFI_HOST_RCV_CTRL_2    (0x1078)
#define WIFI_HOST_RCV_CTRL_3    (0x106c)
#define WIFI_HOST_RCV_CTRL_4    (0x150400)


#define INTERRUPT_CORTUS_0_3000D0   (0x10a8)
#define INTERRUPT_CORTUS_1_3000D0   (0x10ac)
#define INTERRUPT_CORTUS_2_3000D0   (0x10b0)
#define INTERRUPT_CORTUS_3_3000D0   (0x10b4)

typedef struct {
    uint8 u8ChipMode;
    uint8 u8ChipSleep;
    uint8 u8HifRXDone;
    uint8 u8Interrupt;
    uint8 u8Yield;
    uint32 u32RxAddr;
    uint32 u32RxSize;
    tpfHifCallBack pfWifiCb;
    tpfHifCallBack pfIpCb;
    tpfHifCallBack pfOtaCb;
    tpfHifCallBack pfSigmaCb;
    tpfHifCallBack pfHifCb;
    //tpfHifCallBack pfCryptoCb;
    tpfHifCallBack pfSslCb;
} tstrHifContext;

volatile tstrHifContext gstrHifCxt;

#ifdef ETH_MODE
extern void os_hook_isr(void);
#endif

/*
    Special codes for managing HIF restriction to OTA rollback/switch only
*/
#define HIF_OTA_RB_ONLY             0xFFFF
#define HIFCODE_OTA_RB              ((M2M_REQ_GROUP_OTA << 8) | M2M_OTA_REQ_ROLLBACK)
#define HIFCODE_OTA_SW              ((M2M_REQ_GROUP_OTA << 8) | M2M_OTA_REQ_SWITCH_FIRMWARE)
/*
    Codes for new HIF messages (since last HIF major increase).
    Only need ones which are host->winc.
    Each entry is formed of ((GroupId << 8) | OpCode)
*/
#define HIFCODE_SSL_WRITECERT       ((M2M_REQ_GROUP_SSL << 8) | M2M_SSL_REQ_WRITE_OWN_CERTS)
#define HIFCODE_WIFI_PASSIVESCAN    ((M2M_REQ_GROUP_WIFI << 8) | M2M_WIFI_REQ_PASSIVE_SCAN)
#define HIFCODE_WIFI_CONN           ((M2M_REQ_GROUP_WIFI << 8) | M2M_WIFI_REQ_CONN)
#define HIFCODE_WIFI_CONN_PARAM     ((M2M_REQ_GROUP_WIFI << 8) | M2M_WIFI_IND_CONN_PARAM)
#define HIFCODE_WIFI_DELETE_CRED    ((M2M_REQ_GROUP_WIFI << 8) | M2M_WIFI_REQRSP_DELETE_APID)
#define HIFCODE_WIFI_START_PROV_MODE ((M2M_REQ_GROUP_WIFI << 8) | M2M_WIFI_REQ_START_PROVISION_MODE)
#define HIFCODE_WIFI_ENABLE_AP      ((M2M_REQ_GROUP_WIFI << 8) | M2M_WIFI_REQ_ENABLE_AP)
#define HIFCODE_IP_RAW_SOCK_OPT     ((M2M_REQ_GROUP_IP << 8)   | SOCKET_CMD_RAW_SET_SOCK_OPT)

/*
    List of new HIF messages (since last HIF major increase).
    Only need to list ones which are host->winc.
    Additionally, entry 0 used to indicate OTA RB/SW only.
*/
#define NEW_HIF_LIST \
    HIF_OTA_RB_ONLY, \
    HIFCODE_SSL_WRITECERT, \
    HIFCODE_WIFI_PASSIVESCAN, \
    HIFCODE_WIFI_CONN, \
    HIFCODE_WIFI_CONN_PARAM, \
    HIFCODE_WIFI_DELETE_CRED, \
    HIFCODE_WIFI_START_PROV_MODE, \
    HIFCODE_WIFI_ENABLE_AP, \
    HIFCODE_IP_RAW_SOCK_OPT
/*
    Array of HIF messages which are not supported by Firmware.
    During hif_init() this array is rebased using an offset determined by Firmware HIF level.
*/
static uint16 gau16HifBlacklist[] = {NEW_HIF_LIST};
#define HIF_BLACKLIST_SZ (sizeof(gau16HifBlacklist)/sizeof(gau16HifBlacklist[0]))
static uint8 gu8HifBlOffset = 0;

static void isr(void)
{
    gstrHifCxt.u8Interrupt++;
#ifdef NM_LEVEL_INTERRUPT
    nm_bsp_interrupt_ctrl(0);
#endif
#ifdef ETH_MODE
	os_hook_isr();
#endif
}

static sint8 hif_set_rx_done(void)
{
    uint32 reg;
    sint8 ret = M2M_SUCCESS;
    gstrHifCxt.u8HifRXDone = 0;
#ifdef NM_EDGE_INTERRUPT
    nm_bsp_interrupt_ctrl(1);
#endif
    if(ISNMC3400(nmi_get_chipid())) {
        ret = nm_write_reg(INTERRUPT_CORTUS_0_3000D0, 1);
        if(ret != M2M_SUCCESS)goto ERR1;
    } else  {
        ret = nm_read_reg_with_ret(WIFI_HOST_RCV_CTRL_0, &reg);
        if(ret != M2M_SUCCESS)goto ERR1;

        /* Set RX Done */
        reg |= NBIT1;
        ret = nm_write_reg(WIFI_HOST_RCV_CTRL_0, reg);
        if(ret != M2M_SUCCESS)goto ERR1;
    }

#ifdef NM_LEVEL_INTERRUPT
    nm_bsp_interrupt_ctrl(1);
#endif
ERR1:
    return ret;

}
/**
 *  @fn         static void m2m_hif_cb(uint8 u8OpCode, uint16 u16DataSize, uint32 u32Addr)
 *  @brief      WiFi call back function
 *  @param[in]  u8OpCode
 *                  HIF Opcode type.
 *  @param[in]  u16DataSize
 *                  HIF data length.
 *  @param[in]  u32Addr
 *                  HIF address.
 */
static void m2m_hif_cb(uint8 u8OpCode, uint16 u16DataSize, uint32 u32Addr)
{

}

/**
 *  @fn         NMI_API sint8 hif_chip_wake(void);
 *  @brief      To Wakeup the chip.
 *  @return     The function shall return ZERO for successful operation and a negative value otherwise.
 */
sint8 hif_chip_wake(void)
{
    sint8 ret = M2M_SUCCESS;
    if(gstrHifCxt.u8HifRXDone)
    {
        /* chip already wake for the rx not done no need to send wake request */
        return ret;
    }
    if(gstrHifCxt.u8ChipSleep == 0)
    {
        if(gstrHifCxt.u8ChipMode != M2M_NO_PS)
        {
            ret = chip_wake();
            if(ret != M2M_SUCCESS)goto ERR1;
        }
    }
    gstrHifCxt.u8ChipSleep++;
ERR1:
    return ret;
}
/*!
@fn \
    NMI_API void hif_set_sleep_mode(uint8 u8Pstype);

@brief
    Set the sleep mode of the HIF layer.

@param [in] u8Pstype
                Sleep mode.

@return
    The function SHALL return 0 for success and a negative value otherwise.
*/

void hif_set_sleep_mode(uint8 u8Pstype)
{
    gstrHifCxt.u8ChipMode = u8Pstype;
}

/*!
@fn \
    NMI_API uint8 hif_get_sleep_mode(void);

@brief
    Get the sleep mode of the HIF layer.

@return
    The function SHALL return the sleep mode of the HIF layer.
*/
uint8 hif_get_sleep_mode(void)
{
    return gstrHifCxt.u8ChipMode;
}

/**
 *  @fn         NMI_API sint8 hif_chip_sleep_sc(void);
 *  @brief      To clear the chip sleep but keep the chip sleep
 *  @return     The function shall return ZERO for successful operation and a negative value otherwise.
 */
static sint8 hif_chip_sleep_sc(void)
{
    if(gstrHifCxt.u8ChipSleep >= 1)
    {
        gstrHifCxt.u8ChipSleep--;
    }
    return M2M_SUCCESS;
}


/**
 *  @fn         NMI_API sint8 hif_chip_sleep(void);
 *  @brief      To make the chip sleep.
 *  @return     The function shall return ZERO for successful operation and a negative value otherwise.
 */
sint8 hif_chip_sleep(void)
{
    sint8 ret = M2M_SUCCESS;

    if(gstrHifCxt.u8ChipSleep >= 1)
    {
        gstrHifCxt.u8ChipSleep--;
    }

    if(gstrHifCxt.u8ChipSleep == 0)
    {
        if(gstrHifCxt.u8ChipMode != M2M_NO_PS)
            ret = chip_sleep();
    }
//ERR1:
    return ret;
}

/**
 *  @fn         NMI_API sint8 hif_init(void * arg);
 *  @brief      To initialize HIF layer.
 *  @param[in]  arg
 *                  Pointer to the arguments.
 *  @return     The function shall return ZERO for successful operation and a negative value otherwise.
 */
sint8 hif_init(void *arg)
{
    sint8 ret = M2M_SUCCESS;
    m2m_memset((uint8*)&gstrHifCxt, 0, sizeof(tstrHifContext));
    nm_bsp_register_isr(isr);

    hif_register_cb(M2M_REQ_GROUP_HIF, m2m_hif_cb);
    ret = hif_chip_sleep();

    return ret;
}

/**
 *  @fn         NMI_API sint8 hif_deinit(void * arg);
 *  @brief      To Deinitialize HIF layer.
 *  @param[in]  arg
 *                  Pointer to the arguments.
 *   @return    The function shall return ZERO for successful operation and a negative value otherwise.
 */
sint8 hif_deinit(void *arg)
{
    sint8 ret = M2M_SUCCESS;
    ret = hif_chip_wake();
    m2m_memset((uint8*)&gstrHifCxt, 0, sizeof(tstrHifContext));
    return ret;
}

/**
 *  @fn         sint8 hif_check_compatibility(uint16 u16HifInfo);
 *  @brief
 *              To check the compatibility of an image with the current driver.
 *  @param [in] u16HifInfo
 *                  HIF info of image to be checked.
 *  @return     The function shall return ZERO for compatible image and a negative value otherwise.
 */
sint8 hif_check_compatibility(uint16 u16HifInfo)
{
    sint8 ret = M2M_ERR_FW_VER_MISMATCH;
    if((M2M_GET_HIF_BLOCK(u16HifInfo) == M2M_HIF_BLOCK_VALUE) && (M2M_GET_HIF_MAJOR(u16HifInfo) == M2M_HIF_MAJOR_VALUE))
    {
        ret = M2M_SUCCESS;
    }
    return ret;
}

/**
 *  @fn         sint8 hif_enable_access(void);
 *  @brief
 *              To enable access to HIF layer, based on HIF level of Firmware.
 *              This function reads HIF level directly from a register written by Firmware.
 *  @return     The function shall return ZERO for full match operation and a negative value if operation is restricted.
 */
sint8 hif_enable_access(void)
{
    sint8 ret = M2M_SUCCESS;
    uint16 fw_hif_info = 0;

    ret = nm_get_hif_info(&fw_hif_info, NULL);
    if(ret == M2M_SUCCESS)
    {
        ret = hif_check_compatibility(fw_hif_info);
        if(ret == M2M_SUCCESS)
        {
            switch(M2M_GET_HIF_MINOR(fw_hif_info))
            {
            case 0:
                gu8HifBlOffset = 1;
                break;
            case 1:
                gu8HifBlOffset = 2;
                break;
            case 2:
                gu8HifBlOffset = 2;
                break;
            case 3:
                gu8HifBlOffset = 3;
                break;
            case 4:
                gu8HifBlOffset = 9;
                break;
            // Additional case to be added each time hif minor increments.
            // All additional cases to be removed in the event of a hif major increment.
            // Default catches all cases in which hif minor is greater in Firmware than in Driver.
            default:
                gu8HifBlOffset = HIF_BLACKLIST_SZ;
                break;
            }
        }
        else
        {
            gu8HifBlOffset = 0;
            M2M_ERR("HIF access limited to OTA Switch/Rollback only\n");
        }
    }
    return ret;
}

/**
 *  @fn         sint8 hif_check_code(uint8 u8Gid, uint8 u8OpCode);
 *  @brief
 *              To check that a particular hif message is supported with the current driver/firmware pair.
 *  @param[in]  u8Gid
 *                  Group ID.
 *  @param[in]  u8Opcode
 *                  Operation ID.
 *  @return     The function shall return @ref M2M_SUCCESS for success and a negative value otherwise.
 */
sint8 hif_check_code(uint8 u8Gid, uint8 u8OpCode)
{
    uint8 u8BlId;
    uint16 u16HifCode = ((uint16)u8Gid<<8) | u8OpCode;
    if((u16HifCode == HIFCODE_OTA_RB) || (u16HifCode == HIFCODE_OTA_SW))
    {
        return M2M_SUCCESS;
    }
    if(gu8HifBlOffset == 0)
    {
        M2M_ERR("HIF OTA rb/sw only\n");
        return M2M_ERR_SEND;
    }
    for(u8BlId = gu8HifBlOffset; u8BlId < HIF_BLACKLIST_SZ; u8BlId++)
    {
        if(u16HifCode == gau16HifBlacklist[u8BlId])
        {
            M2M_ERR("HIF message unsupported\n");
            return M2M_ERR_SEND;
        }
    }
    return M2M_SUCCESS;
}

/**
 *  @fn         NMI_API sint8 hif_send(uint8 u8Gid,uint8 u8Opcode,uint8 *pu8CtrlBuf,uint16 u16CtrlBufSize,
 *                                     uint8 *pu8DataBuf,uint16 u16DataSize, uint16 u16DataOffset)
 *  @brief      Send packet using host interface.
 *
 *  @param[in]  u8Gid
 *                  Group ID.
 *  @param[in]  u8Opcode
 *                  Operation ID.
 *  @param[in]  pu8CtrlBuf
 *                  Pointer to the Control buffer.
 *  @param[in]  u16CtrlBufSize
 *                  Control buffer size.
 *  @param[in]  u16DataOffset
 *                  Packet Data offset.
 *  @param[in]  pu8DataBuf
 *                  Packet buffer Allocated by the caller.
 *  @param[in]  u16DataSize
 *                  Packet buffer size (including the HIF header).
 *   @return    The function shall return @ref M2M_SUCCESS for successful operation and a negative value otherwise.
 */
sint8 hif_send(uint8 u8Gid, uint8 u8Opcode, uint8 *pu8CtrlBuf, uint16 u16CtrlBufSize,
               uint8 *pu8DataBuf, uint16 u16DataSize, uint16 u16DataOffset)
{
    sint8       ret = M2M_ERR_SEND;
    tstrHifHdr  strHif;
    uint32      u32CtrlDataGap = u16DataOffset;

    strHif.u8Opcode     = u8Opcode&(~NBIT7);
    strHif.u8Gid        = u8Gid;
    strHif.u16Length    = M2M_HIF_HDR_OFFSET;

    if(pu8CtrlBuf != NULL)
    {
        if(u16CtrlBufSize > M2M_HIF_MAX_PACKET_SIZE-M2M_HIF_HDR_OFFSET)
        {
            M2M_ERR("HIF %s (%dB) exceeds max (%dB)\n", "Ctrl", u16CtrlBufSize, M2M_HIF_MAX_PACKET_SIZE-M2M_HIF_HDR_OFFSET);
            goto ERR2;
        }
        strHif.u16Length += u16CtrlBufSize;
        u32CtrlDataGap -= u16CtrlBufSize;
    }
    if(pu8DataBuf != NULL)
    {
        if((uint32)u16DataOffset + u16DataSize > M2M_HIF_MAX_PACKET_SIZE-M2M_HIF_HDR_OFFSET)
        {
            M2M_ERR("HIF %s (%luB) exceeds max (%luB)\n", "Data", (uint32)u16DataOffset + u16DataSize, (uint32)M2M_HIF_MAX_PACKET_SIZE-M2M_HIF_HDR_OFFSET);
            goto ERR2;
        }
        strHif.u16Length += u32CtrlDataGap + u16DataSize;
    }

    ret = hif_check_code(strHif.u8Gid, strHif.u8Opcode);
    if(ret != M2M_SUCCESS)
    {
        goto ERR2;
    }

    if(strHif.u16Length <= M2M_HIF_MAX_PACKET_SIZE)
    {
        ret = hif_chip_wake();
        if(ret == M2M_SUCCESS)
        {
            volatile uint32 reg, dma_addr = 0;
            volatile uint16 cnt = 0;

            reg = 0UL;
            reg |= (uint32)u8Gid;
            reg |= ((uint32)u8Opcode<<8);
            reg |= ((uint32)strHif.u16Length<<16);
            ret = nm_write_reg(NMI_STATE_REG, reg);
            if(M2M_SUCCESS != ret) goto ERR1;

            reg = 0UL;
            reg |= NBIT1;
            ret = nm_write_reg(WIFI_HOST_RCV_CTRL_2, reg);
            if(M2M_SUCCESS != ret) goto ERR1;


            if(ISNMC3400(nmi_get_chipid())) {

                ret = nm_write_reg(INTERRUPT_CORTUS_1_3000D0, 1);
                if(M2M_SUCCESS != ret) goto ERR1;
            }

            dma_addr = 0;

            for(cnt = 0; cnt < 1000*5; cnt ++)
            {
                ret = nm_read_reg_with_ret(WIFI_HOST_RCV_CTRL_2, (uint32 *)&reg);
                if(ret != M2M_SUCCESS) break;
                /*
                 * If it takes too long to get a response, the slow down to
                 * avoid back-to-back register read operations.
                 */
                if(cnt >= 1000) {
                    if(cnt == 1000) {
                        M2M_INFO("Slowing down...\n");
                    }
                    nm_bsp_sleep(5);
                }
                if(!(reg & NBIT1))
                {
                    ret = nm_read_reg_with_ret(WIFI_HOST_RCV_CTRL_4, (uint32 *)&dma_addr);
                    if(ret != M2M_SUCCESS) {
                        /*in case of read error clear the DMA address and return error*/
                        dma_addr = 0;
                        goto ERR1;
                    }
                    /*in case of success break */
                    break;
                }
            }

            if(dma_addr != 0)
            {
                volatile uint32 u32CurrAddr;
                u32CurrAddr = dma_addr;
                strHif.u16Length=NM_BSP_B_L_16(strHif.u16Length);
                M2M_DBG("Writing into %lx %ld\n", dma_addr, strHif.u16Length);
                ret = nm_write_block(u32CurrAddr, (uint8 *)&strHif, M2M_HIF_HDR_OFFSET);
                if(M2M_SUCCESS != ret) goto ERR1;
                u32CurrAddr += M2M_HIF_HDR_OFFSET;
                if(pu8CtrlBuf != NULL)
                {
                    ret = nm_write_block(u32CurrAddr, pu8CtrlBuf, u16CtrlBufSize);
                    if(M2M_SUCCESS != ret) goto ERR1;
                    u32CurrAddr += u16CtrlBufSize;
                }
                if(pu8DataBuf != NULL)
                {
                    u32CurrAddr += u32CtrlDataGap;
                    ret = nm_write_block(u32CurrAddr, pu8DataBuf, u16DataSize);
                    if(M2M_SUCCESS != ret) goto ERR1;
                    u32CurrAddr += u16DataSize;
                }

                reg = dma_addr << 2;

                /* Following line of code is to generate the interrupt which is not strictly needed for 3400,
                 * but has no noticeable side effects
                 */
                reg |= NBIT1;
                ret = nm_write_reg(WIFI_HOST_RCV_CTRL_3, reg);
                if(M2M_SUCCESS != ret) goto ERR1;

                if(ISNMC3400(nmi_get_chipid())) {
                    ret = nm_write_reg(INTERRUPT_CORTUS_2_3000D0, 1);
                    if(M2M_SUCCESS != ret) goto ERR1;
                }
            }
            else
            {
                ret = hif_chip_sleep();
                M2M_INFO("Failed to alloc rx size\n");
                ret =  M2M_ERR_MEM_ALLOC;
                goto ERR2;
            }
        }
        else
        {
            M2M_ERR("(HIF)Failed to wakeup the chip\n");
            goto ERR2;
        }
    }
    else
    {
        M2M_ERR("HIF message length (%d) exceeds max length (%d)\n", strHif.u16Length, M2M_HIF_MAX_PACKET_SIZE);
        ret = M2M_ERR_SEND;
        goto ERR2;
    }
    /*actual sleep ret = M2M_SUCCESS*/
    ret = hif_chip_sleep();
    return ret;
ERR1:
    /*reset the count but no actual sleep as it already bus error*/
    hif_chip_sleep_sc();
ERR2:
    /*logical error*/
    return ret;
}
/**
 *  @fn         hif_isr
 *  @brief      Host interface interrupt service routine
 *  @return     @ref M2M_SUCCESS in case of success or a negative vale otherwise
 */
static sint8 hif_isr(void)
{
    sint8 ret = M2M_SUCCESS;
    uint32 reg;
    volatile tstrHifHdr strHif;

    ret = nm_read_reg_with_ret(WIFI_HOST_RCV_CTRL_0, &reg);
    if(M2M_SUCCESS == ret)
    {
        if(reg & 0x1)   /* New interrupt has been received */
        {
            uint16 size;

            /*Clearing RX interrupt*/
            ret = nm_read_reg_with_ret(WIFI_HOST_RCV_CTRL_0, &reg);
            if(ret != M2M_SUCCESS)goto ERR1;
            reg &= ~NBIT0;
            ret = nm_write_reg(WIFI_HOST_RCV_CTRL_0, reg);
            if(ret != M2M_SUCCESS)goto ERR1;
            /* read the rx size */
            ret = nm_read_reg_with_ret(WIFI_HOST_RCV_CTRL_0, &reg);
            if(M2M_SUCCESS != ret)
            {
                M2M_ERR("(hif) WIFI_HOST_RCV_CTRL_0 bus fail\n");
                goto ERR1;
            }
            gstrHifCxt.u8HifRXDone = 1;
            size = (uint16)((reg >> 2) & 0xfff);
            if(size > 0) {
                uint32 address = 0;
                /**
                start bus transfer
                **/
                ret = nm_read_reg_with_ret(WIFI_HOST_RCV_CTRL_1, &address);
                if(M2M_SUCCESS != ret)
                {
                    M2M_ERR("(hif) WIFI_HOST_RCV_CTRL_1 bus fail\n");
                    goto ERR1;
                }
                gstrHifCxt.u32RxAddr = address;
                gstrHifCxt.u32RxSize = size;
                ret = nm_read_block(address, (uint8 *)&strHif, sizeof(tstrHifHdr));
                strHif.u16Length = NM_BSP_B_L_16(strHif.u16Length);
                if(M2M_SUCCESS != ret)
                {
                    M2M_ERR("(hif) address bus fail\n");
                    goto ERR1;
                }
                if(strHif.u16Length != size)
                {
                    if((size - strHif.u16Length) > 4)
                    {
                        M2M_ERR("(hif) Corrupted packet Size = %u <L = %u, G = %u, OP = %02X>\n",
                                size, strHif.u16Length, strHif.u8Gid, strHif.u8Opcode);
                        ret = M2M_ERR_BUS_FAIL;
                        goto ERR1;
                    }
                }

                if(M2M_REQ_GROUP_WIFI == strHif.u8Gid)
                {
                    if(gstrHifCxt.pfWifiCb)
                        gstrHifCxt.pfWifiCb(strHif.u8Opcode, strHif.u16Length - M2M_HIF_HDR_OFFSET, address + M2M_HIF_HDR_OFFSET);
                    else
                        M2M_ERR("WIFI callback is not registered\n");
                }
                else if(M2M_REQ_GROUP_IP == strHif.u8Gid)
                {
                    if(gstrHifCxt.pfIpCb)
                        gstrHifCxt.pfIpCb(strHif.u8Opcode, strHif.u16Length - M2M_HIF_HDR_OFFSET, address + M2M_HIF_HDR_OFFSET);
                    else
                        M2M_ERR("Socket callback is not registered\n");
                }
                else if(M2M_REQ_GROUP_OTA == strHif.u8Gid)
                {
                    if(gstrHifCxt.pfOtaCb)
                        gstrHifCxt.pfOtaCb(strHif.u8Opcode, strHif.u16Length - M2M_HIF_HDR_OFFSET, address + M2M_HIF_HDR_OFFSET);
                    else
                        M2M_ERR("OTA callback is not registered\n");
                }
                else if(M2M_REQ_GROUP_SIGMA == strHif.u8Gid)
                {
                    if(gstrHifCxt.pfSigmaCb)
                        gstrHifCxt.pfSigmaCb(strHif.u8Opcode, strHif.u16Length - M2M_HIF_HDR_OFFSET, address + M2M_HIF_HDR_OFFSET);
                    else
                        M2M_ERR("Sigma callback is not registered\n");
                }
                else if(M2M_REQ_GROUP_SSL == strHif.u8Gid)
                {
                    if(gstrHifCxt.pfSslCb)
                        gstrHifCxt.pfSslCb(strHif.u8Opcode, strHif.u16Length - M2M_HIF_HDR_OFFSET, address + M2M_HIF_HDR_OFFSET);
                    else
                        M2M_ERR("SSL callback is not registered\n");
                }
                else
                {
                    M2M_ERR("(hif) invalid group ID\n");
                    ret = M2M_ERR_BUS_FAIL;
                    goto ERR1;
                }
                if(gstrHifCxt.u8HifRXDone)
                {
                    M2M_ERR("(hif) host app didn't set RX Done <%u><%X>\n", strHif.u8Gid, strHif.u8Opcode);
                    ret = hif_set_rx_done();
                    if(ret != M2M_SUCCESS) goto ERR1;
                }
            }
            else
            {
                ret = M2M_ERR_RCV;
                M2M_ERR("(hif) Wrong Size\n");
                goto ERR1;
            }
        }
        else
        {
#ifndef WIN32
            M2M_ERR("(hif) False interrupt %lx", reg);
            goto ERR1;
#endif
        }
    }
    else
    {
        M2M_ERR("(hif) Failed to Read interrupt reg\n");
    }

ERR1:
    return ret;
}

/**
 *  @fn         hif_yield(void)
 *  @brief      Yields control from interrupt event handler
 */

void hif_yield(void)
{
    gstrHifCxt.u8Yield = 1;
}

sint8 hif_handle_isr(void)
{
    sint8 ret = M2M_SUCCESS;

    gstrHifCxt.u8Yield = 0;
    while(gstrHifCxt.u8Interrupt && !gstrHifCxt.u8Yield)
    {
        /* Atomic decrement u8Interrupt since it takes multiple instructions to load, decrement and store,
         * during which the ISR could fire again.
         * If LEVEL interrupt is used instead of EDGE then the atomicity isn't needed since the interrupt
         * is turned off in the ISR and back on again only after the interrupt has been serviced in hif_isr(). */

#ifndef NM_LEVEL_INTERRUPT
        nm_bsp_interrupt_ctrl(0);
#endif
        gstrHifCxt.u8Interrupt--;
#ifndef NM_LEVEL_INTERRUPT
        nm_bsp_interrupt_ctrl(1);
#endif

        uint8 retries = 5;
        while(1)
        {
            ret = hif_isr();
            if(ret == M2M_SUCCESS) {
                /*we will try forever until we get that interrupt*/
                /*Fail return errors here due to bus errors (reading expected values)*/
                break;
            } else {
                retries--;
                if(!retries)
                {
                    M2M_ERR("(HIF) Failed to handle interrupt %d, aborting due to too many retries\n", ret);
                    break;
                }
                else
                    M2M_ERR("(HIF) Failed to handle interrupt %d try again... (%u)\n", ret, retries);
            }
        }
    }

    return ret;
}

/**
 *  @fn         hif_receive
 *  @brief      Host interface interrupt service routine
 *  @param[in]  u32Addr
 *                  Receive start address
 *  @param[out] pu8Buf
 *                  Pointer to receive buffer. Allocated by the caller
 *  @param[in]  u16Sz
 *                  Receive buffer size
 *  @param[in]  isDone
 *                  If you don't need any more packets send True otherwise send false
 *   @return    The function shall return ZERO for successful operation and a negative value otherwise.
 */
sint8 hif_receive(uint32 u32Addr, uint8 *pu8Buf, uint16 u16Sz, uint8 isDone)
{
    sint8 ret = M2M_SUCCESS;

    if((u32Addr == 0) || (pu8Buf == NULL) || (u16Sz == 0))
    {
        if(isDone)
        {
            ret = hif_set_rx_done();
        }
        else
        {
            ret = M2M_ERR_FAIL;
            M2M_ERR(" hif_receive: Invalid argument\n");
        }
        goto ERR1;
    }

    if(u16Sz > gstrHifCxt.u32RxSize)
    {
        ret = M2M_ERR_FAIL;
        M2M_ERR("APP Requested Size is larger than the received buffer size <%d> <%lu>\n", u16Sz, gstrHifCxt.u32RxSize);
        goto ERR1;
    }
    if((u32Addr < gstrHifCxt.u32RxAddr)||((u32Addr + u16Sz)>(gstrHifCxt.u32RxAddr+gstrHifCxt.u32RxSize)))
    {
        ret = M2M_ERR_FAIL;
        M2M_ERR("APP Requested Address beyond the received buffer address and length\n");
        goto ERR1;
    }

    /* Receive the payload */
    ret = nm_read_block(u32Addr, pu8Buf, u16Sz);
    if(ret != M2M_SUCCESS)goto ERR1;

    /* check if this is the last packet */
    if(isDone || (((gstrHifCxt.u32RxAddr+gstrHifCxt.u32RxSize) - (u32Addr+u16Sz)) <= 0) ||
            ((4 - ((u32Addr+u16Sz) & 3)) == ((gstrHifCxt.u32RxAddr+gstrHifCxt.u32RxSize) - (u32Addr+u16Sz))))   /* Length in the RCV CTRL 0 register is rounded off to 4 by the firmware,
                                                                                   but length inside the HIF header is not, Hence consider done if number
                                                                                   of rounding bytes equal to length left to read */
    {
        /* set RX done */
        ret = hif_set_rx_done();
    }
ERR1:
    return ret;
}

/**
 *  @fn         hif_register_cb
 *  @brief      To set Callback function for every component
 *  @param[in]  u8Grp
 *                  Group to which the Callback function should be set.
 *  @param[in]  fn
 *                  function to be set
 *   @return    The function shall return ZERO for successful operation and a negative value otherwise.
 */

sint8 hif_register_cb(uint8 u8Grp, tpfHifCallBack fn)
{
    sint8 ret = M2M_SUCCESS;
    switch(u8Grp)
    {
        case M2M_REQ_GROUP_IP:
            gstrHifCxt.pfIpCb = fn;
            break;
        case M2M_REQ_GROUP_WIFI:
            gstrHifCxt.pfWifiCb = fn;
            break;
        case M2M_REQ_GROUP_OTA:
            gstrHifCxt.pfOtaCb = fn;
            break;
        case M2M_REQ_GROUP_HIF:
            gstrHifCxt.pfHifCb = fn;
            break;
        case M2M_REQ_GROUP_SIGMA:
            gstrHifCxt.pfSigmaCb = fn;
            break;
        case M2M_REQ_GROUP_SSL:
            gstrHifCxt.pfSslCb = fn;
            break;
        default:
            M2M_ERR("GRp ? %d\n", u8Grp);
            ret = M2M_ERR_FAIL;
            break;
    }
    return ret;
}

#endif
