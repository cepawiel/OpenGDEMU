/**
 * \file bm_program_mode.c
 *
 * \brief BM program mode definitions
 *
 * Copyright (c) 2018 Microchip Technology Inc. and its subsidiaries.
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
 */
/*
 * Support and FAQ: visit <a href="https://www.microchip.com/support/">Microchip Support</a>
 */

#include "platform_files.h"
#include "bm_program_mode.h"
#include "event_mem.h"

static BM_PROGRAM_CMDPKT            pgmCmdPkt;
static uint8_t                      pgmDataCmdPkt[BM_PGM_PKT_LEN_MAX];
static uint8_t                      pgmRspData[BM_PGM_PKT_LEN_MAX];
static uint16_t                     pgmDataIndex = 0;
static BM_PROGRAM_CMD_STATE         pgmCmdState = BM_PGM_HCI_EVENT;
extern event_t						appEvent;

/*! \fn BM_PROGRAM_CMDPKT* BM_PROGRAM_Init(void)
 *  \brief Initializes program mode.
 *  \param None.
 *  \pre None.
 *  \return Handle to the program mode command packet.
 */
BM_PROGRAM_CMDPKT* BM_PROGRAM_Init()
{
    BM_PROGRAM_CMDPKT* programCmdPkt = &pgmCmdPkt;
    programCmdPkt->cmdPkt = pgmDataCmdPkt;
    programCmdPkt->length = 0;
    appEvent.event_id = BM_PROGRAM_HCI_ERROR;
    appEvent.event_msg.data = NULL;
    appEvent.event_msg.data_len = 0;
    return programCmdPkt;
}

/*! \fn void BM_PROGRAM_DeInit(BM_PROGRAM_CMDPKT* programCmdPkt)
 *  \brief De-initializes program mode.
 *  \param programCmdPkt Handle to the program mode command packet.
 *  \pre None.
 *  \return None.
 */
void BM_PROGRAM_DeInit(BM_PROGRAM_CMDPKT* programCmdPkt)
{

}

/*! \fn void BM_PROGRAM_Connect(BM_PROGRAM_CMDPKT* programCmdPkt)
 *  \brief Builds a program mode command packet t0 connect to program memory to start the program mode.
 *  \param programCmdPkt Handle to the program mode command packet.
 *  \pre Program mode initialization function mut be called to initialize the programCmdPkt.
 *  \return None.
 */
void BM_PROGRAM_Connect(BM_PROGRAM_CMDPKT* programCmdPkt)
{
    BM_PROGRAM_CONNECT_PKT* connect_pkt = (BM_PROGRAM_CONNECT_PKT*)(programCmdPkt->cmdPkt);
    connect_pkt->hci_command_pkt.type = COMMAND_PKT_TYPE;
    connect_pkt->hci_command_pkt.opcode = 0x0405;
    connect_pkt->hci_command_pkt.length = sizeof(BM_PROGRAM_CONNECT_PKT)-sizeof(HCI_COMMAND_PKT);
    memset(connect_pkt->addr, 0, 6);
    connect_pkt->type = 0;
    connect_pkt->mode = 0;
    connect_pkt->offset = 0;
    connect_pkt->role = 0;
	programCmdPkt->length = sizeof(BM_PROGRAM_CONNECT_PKT);
}

/*! \fn void BM_PROGRAM_Disconnect(BM_PROGRAM_CMDPKT* programCmdPkt)
 *  \brief Builds a program mode command packet to disconnect from program memory to stop the program mode.
 *  \param programCmdPkt Handle to the program mode command packet.
 *  \pre Program mode connect function mut be called.
 *  \return None.
 */
void BM_PROGRAM_Disconnect(BM_PROGRAM_CMDPKT* programCmdPkt)
{
    BM_PROGRAM_DISCONNECT_PKT* disconnect_pkt = (BM_PROGRAM_DISCONNECT_PKT*)(programCmdPkt->cmdPkt);
    disconnect_pkt->hci_command_pkt.type = COMMAND_PKT_TYPE;
    disconnect_pkt->hci_command_pkt.opcode = 0x0406;
    disconnect_pkt->hci_command_pkt.length = sizeof(BM_PROGRAM_DISCONNECT_PKT)-sizeof(HCI_COMMAND_PKT);
    disconnect_pkt->conn_handle = 0x0FFF;   /* TBD: Update with received flash connection handle */
    disconnect_pkt->reason = 0;
	programCmdPkt->length = sizeof(BM_PROGRAM_DISCONNECT_PKT);
}

/*! \fn void BM_PROGRAM_Erase(BM_PROGRAM_CMDPKT* programCmdPkt, BM_PROGRAM_CMD* pgm_cmd)
 *  \brief Builds a program mode command packet to erase the program memory.
 *  \param programCmdPkt Handle to the program mode command packet.
 *  \param pgm_cmd Handle to the program mode data packet.
 *  \pre Program mode connect function mut be called.
 *  \return None.
 */
void BM_PROGRAM_Erase(BM_PROGRAM_CMDPKT* programCmdPkt, BM_PROGRAM_CMD* pgm_cmd)
{
    BM_PROGRAM_DATA_PKT* erase_pkt = (BM_PROGRAM_DATA_PKT*)(programCmdPkt->cmdPkt);
    erase_pkt->acl_data_pkt.type = DATA_ACL_PKT_TYPE;
    erase_pkt->acl_data_pkt.conn_handle = pgm_cmd->handle;
    erase_pkt->acl_data_pkt.length = sizeof(BM_PROGRAM_DATA_PKT)-sizeof(HCI_DATA_PKT);
    erase_pkt->pgm_cmd_id = 0x0112;
    erase_pkt->pgm_data_len = erase_pkt->acl_data_pkt.length-4; 
    erase_pkt->pgm_mem_type = 0x03;
    erase_pkt->pgm_mem_subtype = 0x00;
    erase_pkt->address = pgm_cmd->address;
    erase_pkt->size = pgm_cmd->length;
	programCmdPkt->length = sizeof(BM_PROGRAM_DATA_PKT);
}

/*! \fn void BM_PROGRAM_Write(BM_PROGRAM_CMDPKT* programCmdPkt, BM_PROGRAM_CMD* pgm_cmd)
 *  \brief Builds a program mode command packet to write the program memory.
 *  \param programCmdPkt Handle to the program mode command packet.
 *  \param pgm_cmd Handle to the program mode data packet.
 *  \pre Program mode connect function mut be called.
 *  \return None.
 */
void BM_PROGRAM_Write(BM_PROGRAM_CMDPKT* programCmdPkt, BM_PROGRAM_CMD* pgm_cmd)
{    
    BM_PROGRAM_DATA_PKT* write_pkt = (BM_PROGRAM_DATA_PKT*)(programCmdPkt->cmdPkt);
    write_pkt->acl_data_pkt.type = DATA_ACL_PKT_TYPE;
    write_pkt->acl_data_pkt.conn_handle = pgm_cmd->handle;
    write_pkt->acl_data_pkt.length = sizeof(BM_PROGRAM_DATA_PKT)-sizeof(HCI_DATA_PKT)+pgm_cmd->length;
    write_pkt->pgm_cmd_id = 0x0111;
    write_pkt->pgm_data_len = write_pkt->acl_data_pkt.length-4; 
    if (pgm_cmd->write_continue)
    {
        write_pkt->pgm_data_len |= 0x8000;
    }
    write_pkt->pgm_mem_type = 0x03;
    write_pkt->pgm_mem_subtype = 0x00;
    write_pkt->address = pgm_cmd->address;
    if (pgm_cmd->write_continue)
    {
        write_pkt->size = pgm_cmd->write_continue_length;
    }
    else
    {
        write_pkt->size = pgm_cmd->length;
    }
    memcpy(programCmdPkt->cmdPkt+sizeof(BM_PROGRAM_DATA_PKT), pgm_cmd->data, pgm_cmd->length);
    programCmdPkt->length = sizeof(BM_PROGRAM_DATA_PKT)+pgm_cmd->length;
}	

/*! \fn void BM_PROGRAM_Write_Continue(BM_PROGRAM_CMDPKT* programCmdPkt, BM_PROGRAM_CMD* pgm_cmd)
 *  \brief Builds a program mode command packet to continue writing the program memory.
 *  \param programCmdPkt Handle to the program mode command packet.
 *  \param pgm_cmd Handle to the program mode data packet.
 *  \pre Program mode connect function mut be called.
 *  \return None.
 */
void BM_PROGRAM_Write_Continue(BM_PROGRAM_CMDPKT* programCmdPkt, BM_PROGRAM_CMD* pgm_cmd)
{    
    BM_PROGRAM_DATA_ALT_PKT* write_pkt = (BM_PROGRAM_DATA_ALT_PKT*)(programCmdPkt->cmdPkt);
    write_pkt->acl_data_pkt.type = DATA_ACL_PKT_TYPE;
    write_pkt->acl_data_pkt.conn_handle = pgm_cmd->handle;
    write_pkt->acl_data_pkt.length = sizeof(BM_PROGRAM_DATA_ALT_PKT)-sizeof(HCI_DATA_PKT)+pgm_cmd->length;
    write_pkt->pgm_cmd_id = 0x0001;
    write_pkt->pgm_data_len = write_pkt->acl_data_pkt.length-4; 
    if (pgm_cmd->write_continue)
    {
        write_pkt->pgm_data_len |= 0x8000;
    }
    memcpy(programCmdPkt->cmdPkt+sizeof(BM_PROGRAM_DATA_ALT_PKT), pgm_cmd->data, pgm_cmd->length);
    programCmdPkt->length = sizeof(BM_PROGRAM_DATA_ALT_PKT)+pgm_cmd->length;
}

/*! \fn void BM_PROGRAM_Read(BM_PROGRAM_CMDPKT* programCmdPkt, BM_PROGRAM_CMD* pgm_cmd)
 *  \brief Builds a program mode command packet to read the program memory.
 *  \param programCmdPkt Handle to the program mode command packet.
 *  \param pgm_cmd Handle to the program mode data packet.
 *  \pre Program mode connect function mut be called.
 *  \return None.
 */
void BM_PROGRAM_Read(BM_PROGRAM_CMDPKT* programCmdPkt, BM_PROGRAM_CMD* pgm_cmd)
{  
    BM_PROGRAM_DATA_PKT* read_pkt = (BM_PROGRAM_DATA_PKT*)(programCmdPkt->cmdPkt);
    read_pkt->acl_data_pkt.type = DATA_ACL_PKT_TYPE;
    read_pkt->acl_data_pkt.conn_handle = pgm_cmd->handle;
    read_pkt->acl_data_pkt.length = sizeof(BM_PROGRAM_DATA_PKT)-sizeof(HCI_DATA_PKT);
    read_pkt->pgm_cmd_id = 0x0110;
    read_pkt->pgm_data_len = read_pkt->acl_data_pkt.length-4; 
    read_pkt->pgm_mem_type = 0x03;
    read_pkt->pgm_mem_subtype = 0x00;
    read_pkt->address = pgm_cmd->address;
    read_pkt->size = pgm_cmd->length;
    programCmdPkt->length = sizeof(BM_PROGRAM_DATA_PKT);
}

/*! \fn void BM_PROGRAM_CRC_Get(BM_PROGRAM_CMDPKT* programCmdPkt, BM_PROGRAM_CMD* pgm_cmd)
 *  \brief Builds a program mode command packet to get CRC of the program memory.
 *  \param programCmdPkt Handle to the program mode command packet.
 *  \param pgm_cmd Handle to the program mode data packet.
 *  \pre Program mode connect function mut be called.
 *  \return None.
 */
void BM_PROGRAM_CRC_Get(BM_PROGRAM_CMDPKT* programCmdPkt, BM_PROGRAM_CMD* pgm_cmd)
{  
    BM_PROGRAM_CRC_PKT* crc_pkt = (BM_PROGRAM_CRC_PKT*)(programCmdPkt->cmdPkt);
    crc_pkt->acl_data_pkt.type = DATA_ACL_PKT_TYPE;
    crc_pkt->acl_data_pkt.conn_handle = pgm_cmd->handle;
    crc_pkt->acl_data_pkt.length = sizeof(BM_PROGRAM_CRC_PKT)-sizeof(HCI_DATA_PKT);
    crc_pkt->pgm_cmd_id = 0x0110;
    crc_pkt->pgm_data_len = crc_pkt->acl_data_pkt.length-4; 
    crc_pkt->pgm_mem_type = 0x03;
    crc_pkt->pgm_mem_subtype = 0x00;
	crc_pkt->crc_algo = 0x01;
    crc_pkt->address = pgm_cmd->address;
    crc_pkt->size = pgm_cmd->length;
    programCmdPkt->length = sizeof(BM_PROGRAM_CRC_PKT);
}

/*! \fn bool BM_PROGRAM_ResponseCheck(uint8_t* data, uint8_t len)
 *  \brief Parses the event packet and provides the handle to the event.
 *  \param data Handle to the data containing the response.
 *  \param len Length of the data containing the response.
 *  \pre Program mode connect function mut be called.
 *  \return Returns true if there is valid event; false, otherwise.
 */
bool BM_PROGRAM_ResponseCheck(uint8_t* data, uint8_t len)
{
    
    while(len--)
    {
        switch(pgmCmdState)
        {
            case BM_PGM_HCI_EVENT:
                if(*data == BM_PGM_HCI_EVENT_VAL || *data == BM_PGM_HCI_ACL_VAL)
                {
                    pgmRspData[BM_PGM_HCI_EVENT_INDEX] = *data;
                    
                    if (*data == BM_PGM_HCI_EVENT_VAL)
                        pgmCmdState = BM_PGM_HCI_EVENT_TYPE;
                    else if (*data == BM_PGM_HCI_ACL_VAL)
                        pgmCmdState = BM_PGM_HCI_ACL_HANDLEL;
					appEvent.event_id = BM_PROGRAM_HCI_ERROR;
					appEvent.event_msg.data = NULL;
					appEvent.event_msg.data_len = 0;
                }
                break;

            case BM_PGM_HCI_EVENT_TYPE:
                {
                    pgmRspData[BM_PGM_HCI_EVENT_TYPE_INDEX] = *data;
                    pgmCmdState = BM_PGM_HCI_EVENT_LEN;
                }
                break;

            case BM_PGM_HCI_EVENT_LEN:
                {
                    pgmRspData[BM_PGM_HCI_EVENT_LEN_INDEX] = *data;
                    pgmCmdState = BM_PGM_HCI_EVENT_DATA;
                }
                break;

            case BM_PGM_HCI_EVENT_DATA:
                pgmRspData[BM_PGM_HCI_EVENT_DATA_INDEX+pgmDataIndex] = *data;
                pgmDataIndex++;
                if(pgmDataIndex >= pgmRspData[BM_PGM_HCI_EVENT_LEN_INDEX])
                {
                    pgmDataIndex = 0;
                    pgmCmdState = BM_PGM_HCI_EVENT;                  
                    appEvent.event_id = pgmRspData[BM_PGM_HCI_EVENT_INDEX];
                    appEvent.event_msg.data = &pgmRspData[BM_PGM_HCI_EVENT_INDEX];
                    appEvent.event_msg.data_len = pgmRspData[BM_PGM_HCI_EVENT_LEN_INDEX];
                    return true;                    
                }
                break;

            case BM_PGM_HCI_ACL_HANDLEL:
                {
                    pgmRspData[BM_PGM_HCI_ACL_HANDLEL_INDEX] = *data;
                    pgmCmdState = BM_PGM_HCI_ACL_HANDLEH;
                }
                break;
            
            case BM_PGM_HCI_ACL_HANDLEH:
                {
                    pgmRspData[BM_PGM_HCI_ACL_HANDLEH_INDEX] = *data;
                    pgmCmdState = BM_PGM_HCI_ACL_LENL;
                }
                break;
            
            case BM_PGM_HCI_ACL_LENL:
                {
                    pgmRspData[BM_PGM_HCI_ACL_LENL_INDEX] = *data;
                    pgmCmdState = BM_PGM_HCI_ACL_LENH;
                }
                break;
                
            case BM_PGM_HCI_ACL_LENH:
                {
                    pgmRspData[BM_PGM_HCI_ACL_LENH_INDEX] = *data;
                    pgmCmdState = BM_PGM_HCI_ACL_DATA;
                }
                break;
                
            case BM_PGM_HCI_ACL_DATA:
                {
                    pgmRspData[BM_PGM_HCI_ACL_DATA_INDEX+pgmDataIndex] = *data;
                    pgmDataIndex++;
                    if(pgmDataIndex >= (pgmRspData[BM_PGM_HCI_ACL_LENH_INDEX]<<8 | pgmRspData[BM_PGM_HCI_ACL_LENL_INDEX]))
                    {
                        pgmDataIndex = 0;
                        pgmCmdState = BM_PGM_HCI_EVENT;                                      
						appEvent.event_id = pgmRspData[BM_PGM_HCI_EVENT_INDEX];
						appEvent.event_msg.data = &pgmRspData[BM_PGM_HCI_EVENT_INDEX];
						appEvent.event_msg.data_len = (pgmRspData[BM_PGM_HCI_ACL_LENH_INDEX]<<8 | pgmRspData[BM_PGM_HCI_ACL_LENL_INDEX]);
                     
						return true;
                    }
                }
                break;
                
            default:
                break;

        }
        if(len != 0)
            data++;
    }

	return false;
    
}
