/**
 * \file dfu_api.c
 *
 * \brief DFU API definitions
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
#include "stdlib.h"
#include "string.h"
#include "bm_mode.h"
#include "bm_program_mode.h"
#include "bm_application_mode.h"
#include "bm_utils.h"
#include "event_mem.h"
#include "ble_api.h"
#include "dfu_api.h"
#include "interface.h"

BM_PROGRAM_CMDPKT* program_cmdpkt = NULL;
static uint16_t program_handle;
static dfu_status_t dfu_status_report_parser(BM_PROGRAM_HCI_EVENT_STATUS status_id);
static dfu_status_t dfu_response_report_parser(BM_PROGRAM_HCI_EVENT_STATUS status_id);
static uint16_t dfu_status_handle_parser(BM_PROGRAM_HCI_EVENT_STATUS status_id);
static uint8_t* dfu_response_read_parser(BM_PROGRAM_HCI_EVENT_STATUS status_id);

dfu_status_t dfu_init(platform_init_t *platform_init)
{ 
    dfu_status_t status = DFU_SUCCESS;
	status = interface_init(platform_init);
	if(DFU_SUCCESS != status)
	{
		return status;
	}
    
	program_cmdpkt = (BM_PROGRAM_CMDPKT*)BM_PROGRAM_Init();
	
	/* Set operating mode */
	platform_init->platform_api_list.mode_set(BM_MODE_PROGRAM);
                     
	BM_PROGRAM_Connect(program_cmdpkt);
	status = interface_cmd_send_wait(program_cmdpkt->cmdPkt, (uint32_t) program_cmdpkt->length, EVENT_PKT_TYPE);
	if(DFU_SUCCESS == status)
	{
		/* Check the status in status-report event */
		status = dfu_status_report_parser(BM_PROGRAM_HCI_SUCCESS);
        
        interface_event_wait(EVENT_PKT_TYPE);
        status = dfu_status_report_parser(BM_PROGRAM_HCI_SUCCESS);
        program_handle = dfu_status_handle_parser(BM_PROGRAM_HCI_SUCCESS);
	}
		
	return status;
}

dfu_status_t dfu_deinit(platform_init_t *platform_init)
{ 
    dfu_status_t status = DFU_SUCCESS;
	
	BM_PROGRAM_Disconnect(program_cmdpkt);
	status = interface_cmd_send_wait(program_cmdpkt->cmdPkt, (uint32_t) program_cmdpkt->length, EVENT_PKT_TYPE);
	
    platform_init->platform_api_list.mode_set(BM_MODE_APPLICATION);
	return status;
}


dfu_status_t dfu_program_memory_erase(uint32_t address, uint8_t length)
{
    dfu_status_t status = DFU_SUCCESS;
    
    BM_PROGRAM_CMD pgm_cmd;
    pgm_cmd.handle = program_handle;
    pgm_cmd.address = address;
    pgm_cmd.data = NULL;
    pgm_cmd.length = length;
    pgm_cmd.write_continue = 0;
    
    BM_PROGRAM_Erase(program_cmdpkt, &pgm_cmd);
	status = interface_cmd_send_wait(program_cmdpkt->cmdPkt, (uint32_t) program_cmdpkt->length, EVENT_PKT_TYPE);
	if(DFU_SUCCESS == status)
	{
		/* Check the status in status-report event */
		status = dfu_status_report_parser(BM_PROGRAM_HCI_HANDLE);
        
        interface_event_wait(DATA_ACL_PKT_TYPE);
        status = dfu_response_report_parser(BM_PROGRAM_HCI_SUCCESS);
	}
		
	return status;   
}

dfu_status_t dfu_program_memory_write(uint32_t address, uint8_t* data, uint8_t length, bool write_continue)
{
    dfu_status_t status = DFU_SUCCESS;
    
    BM_PROGRAM_CMD pgm_cmd;
    pgm_cmd.handle = program_handle;
    pgm_cmd.address = address;
    pgm_cmd.data = data;
    pgm_cmd.length = length;
    pgm_cmd.write_continue = write_continue;
    
    BM_PROGRAM_Write(program_cmdpkt, &pgm_cmd);
	status = interface_cmd_send_wait(program_cmdpkt->cmdPkt, (uint32_t) program_cmdpkt->length, EVENT_PKT_TYPE);
	if(DFU_SUCCESS == status)
	{
		/* Check the status in status-report event */
		status = dfu_status_report_parser(BM_PROGRAM_HCI_SUCCESS);
        
        interface_event_wait(DATA_ACL_PKT_TYPE);
        status = dfu_response_report_parser(BM_PROGRAM_HCI_SUCCESS);
	}
		
	return status;   
}


dfu_status_t dfu_program_memory_write_continue(uint32_t address, uint8_t* data, uint8_t length, uint8_t continue_mode, uint32_t continue_length)
{
    dfu_status_t status = DFU_SUCCESS;
    
    BM_PROGRAM_CMD pgm_cmd;
    pgm_cmd.handle = program_handle;
    pgm_cmd.address = 0x00000000;
    pgm_cmd.data = data;
    pgm_cmd.length = length;
    pgm_cmd.write_continue = true;
    
    if (continue_mode == DFU_WRITE_CONTINUE_START)
    {
        pgm_cmd.address = address;
        pgm_cmd.write_continue_length = continue_length;
        BM_PROGRAM_Write(program_cmdpkt, &pgm_cmd);
    }
    else if (continue_mode == DFU_WRITE_CONTINUE_PROCEED)
    {
        BM_PROGRAM_Write_Continue(program_cmdpkt, &pgm_cmd);
    }
    else if (continue_mode == DFU_WRITE_CONTINUE_STOP)
    {
        pgm_cmd.write_continue = false;
        BM_PROGRAM_Write_Continue(program_cmdpkt, &pgm_cmd);
    }
    
	status = interface_cmd_send_wait(program_cmdpkt->cmdPkt, (uint32_t) program_cmdpkt->length, EVENT_PKT_TYPE);
	if(DFU_SUCCESS == status)
	{
		/* Check the status in status-report event */
		status = dfu_status_report_parser(BM_PROGRAM_HCI_HANDLE);
        
        interface_event_wait(DATA_ACL_PKT_TYPE);
        status = dfu_response_report_parser(BM_PROGRAM_HCI_SUCCESS);
	}
		
	return status;   
}

dfu_status_t dfu_program_memory_read(uint32_t address, uint8_t* data, uint8_t length)
{
    dfu_status_t status = DFU_SUCCESS;
    
    BM_PROGRAM_CMD pgm_cmd;
    pgm_cmd.handle = program_handle;
    pgm_cmd.address = address;
    pgm_cmd.data = data;
    pgm_cmd.length = length;
    
    BM_PROGRAM_Read(program_cmdpkt, &pgm_cmd);
	status = interface_cmd_send_wait(program_cmdpkt->cmdPkt, (uint32_t) program_cmdpkt->length, EVENT_PKT_TYPE);
	if(DFU_SUCCESS == status)
	{
        uint8_t* buff; 
		/* Check the status in status-report event */
		status = dfu_status_report_parser(BM_PROGRAM_HCI_HANDLE);
        
        interface_event_wait(DATA_ACL_PKT_TYPE);
        status = dfu_response_report_parser(BM_PROGRAM_HCI_SUCCESS);
        
        buff = dfu_response_read_parser(BM_PROGRAM_HCI_SUCCESS);
        if(buff != NULL)
        {
            memcpy(data, buff, length);
        }
	}
		
	return status;   
}

static dfu_status_t dfu_status_report_parser(BM_PROGRAM_HCI_EVENT_STATUS status_id)
{
	event_t *cmd_frame = (event_t *) get_received_cmd_frame();
    
    
    HCI_EVENT_PKT* hci_event_pkt = (HCI_EVENT_PKT*)(cmd_frame->event_msg.data);
    
    if(hci_event_pkt->event == BM_PROGRAM_HCI_COMPLETE)
    {
        BM_PROGRAM_EVENT_COMPLETE_PKT* event_pkt = (BM_PROGRAM_EVENT_COMPLETE_PKT*)(cmd_frame->event_msg.data);

        if(event_pkt->error == status_id) 
        {
            return DFU_SUCCESS;
        }
    }            
    else if(hci_event_pkt->event == BM_PROGRAM_HCI_STATUS)
    {
        BM_PROGRAM_EVENT_STATUS_PKT* event_pkt = (BM_PROGRAM_EVENT_STATUS_PKT*)(cmd_frame->event_msg.data);

        if(event_pkt->status == status_id) 
        {
            return DFU_SUCCESS;
        }
    }   
    else if(hci_event_pkt->event == BM_PROGRAM_HCI_NOCP)
    {
        BM_PROGRAM_EVENT_NOCP_PKT* event_pkt = (BM_PROGRAM_EVENT_NOCP_PKT*)(cmd_frame->event_msg.data);

        if((event_pkt->conn_num == 0x01) && 
           (event_pkt->conn_handle == program_handle) && 
           (event_pkt->pkt_num == 0x0001)) 
        {
            return DFU_SUCCESS;
        }
    }
    
	return DFU_FAIL;
}

static dfu_status_t dfu_response_report_parser(BM_PROGRAM_HCI_EVENT_STATUS status_id)
{
	event_t *cmd_frame = (event_t *) get_received_cmd_frame();
    
    if(cmd_frame->event_id == BM_PGM_HCI_EVENT_VAL)
    {
        HCI_EVENT_PKT* hci_event_pkt = (HCI_EVENT_PKT*)(cmd_frame->event_msg.data);


        if(hci_event_pkt->event == BM_PROGRAM_HCI_CONN)
        {
            BM_PROGRAM_EVENT_CONN_PKT* event_pkt = (BM_PROGRAM_EVENT_CONN_PKT*)(cmd_frame->event_msg.data);

            if(event_pkt->status == status_id) 
            {
                return DFU_SUCCESS;
            }
        }
        else if(hci_event_pkt->event == BM_PROGRAM_HCI_DISCONN)
        {
            BM_PROGRAM_EVENT_DISCONN_PKT* event_pkt = (BM_PROGRAM_EVENT_DISCONN_PKT*)(cmd_frame->event_msg.data);

            if(event_pkt->status == status_id) 
            {
                return DFU_SUCCESS;
            }
        }
    }
    else if(cmd_frame->event_id == BM_PGM_HCI_ACL_VAL)
    {
        BM_PROGRAM_EVENT_DATA_PKT* event_pkt = (BM_PROGRAM_EVENT_DATA_PKT*)(cmd_frame->event_msg.data);

        if(event_pkt->status == status_id) 
        {
            return DFU_SUCCESS;
        }
    }
    
	return DFU_FAIL;
}

static uint16_t dfu_status_handle_parser(BM_PROGRAM_HCI_EVENT_STATUS status_id)
{
	event_t *cmd_frame = (event_t *) get_received_cmd_frame();
    HCI_EVENT_PKT* hci_event_pkt = (HCI_EVENT_PKT*)(cmd_frame->event_msg.data);

    if(hci_event_pkt->event == BM_PROGRAM_HCI_CONN)
    {
        BM_PROGRAM_EVENT_CONN_PKT* event_pkt = (BM_PROGRAM_EVENT_CONN_PKT*)(cmd_frame->event_msg.data);

        if(event_pkt->status == status_id) 
        {
            return event_pkt->conn_handle;
        }
    }
            
	return 0;
}

static uint8_t* dfu_response_read_parser(BM_PROGRAM_HCI_EVENT_STATUS status_id)
{
	event_t *cmd_frame = (event_t *) get_received_cmd_frame();

    if(cmd_frame->event_id == BM_PGM_HCI_ACL_VAL)
    {
        BM_PROGRAM_EVENT_DATA_PKT* event_pkt = (BM_PROGRAM_EVENT_DATA_PKT*)(cmd_frame->event_msg.data);

        if(event_pkt->status == status_id) 
        {
            return event_pkt->data;
        }
    }
    
	return NULL;
}

bool dfu_response_check(uint8_t byte)
{
    return (BM_PROGRAM_ResponseCheck(&byte, 1));
}