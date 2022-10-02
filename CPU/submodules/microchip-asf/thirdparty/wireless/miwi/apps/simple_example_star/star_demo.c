/**
* \file  star_demo.c
*
* \brief Demo Application for MiWi Star Implementation
*
* Copyright (c) 2018 - 2019 Microchip Technology Inc. and its subsidiaries.
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
* THIS SOFTWARE IS SUPPLIED BY MICROCHIP "AS IS".  NO WARRANTIES,
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
* Support and FAQ: visit <a href="https://www.microchip.com/support/">Microchip Support</a>
*/

/************************ HEADERS ****************************************/
#include "miwi_api.h"
#include "miwi_p2p_star.h"
#include "task.h"
#include "star_demo.h"
#include "mimem.h"
#include "asf.h"
#if defined(ENABLE_SLEEP_FEATURE)
#include "sleep_mgr.h"
#endif
#if defined (ENABLE_CONSOLE)
#include "sio2host.h"
#endif
#if defined(ENABLE_NETWORK_FREEZER)
#include "pdsDataServer.h"
#include "wlPdsTaskManager.h"
#endif

#if defined(PROTOCOL_STAR)
/************************ LOCAL VARIABLES ****************************************/
uint8_t i;
uint8_t TxSynCount = 0;    // Maintain the Count on TX's Done
uint8_t TxSynCount2 = 0; // Maintain the Count on TX's Done
uint8_t TxNum = 0;         // Maintain the Count on TX's Done
uint8_t RxNum = 0;         // Maintain the Count on RX's Done
/* check for selections made by USER */
bool chk_sel_status;
uint8_t NumOfActiveScanResponse;
bool update_ed;
uint8_t select_ed;
uint8_t msghandledemo = 0;
extern uint8_t myChannel;
/* Connection Table Memory */
extern CONNECTION_ENTRY connectionTable[CONNECTION_SIZE];
bool display_connections;
MIWI_TICK t1 , t2;

/************************ FUNCTION DEFINITIONS ****************************************/
/*********************************************************************
* Function: static void dataConfcb(uint8_t handle, miwi_status_t status)
*
* Overview: Confirmation Callback for MiApp_SendData
*
* Parameters:  handle - message handle, miwi_status_t status of data send
****************************************************************************/
static void dataConfcb(uint8_t handle, miwi_status_t status, uint8_t* msgPointer)
{
    if (SUCCESS == status)
    {
        /* Update the TX NUM and Display it on the LCD */
        DemoOutput_UpdateTxRx(++TxNum, RxNum);
        /* Delay for Display */
        delay_ms(100);
    }
    /* After Displaying TX and RX Counts , Switch back to showing Demo Instructions */
    STAR_DEMO_OPTIONS_MESSAGE (role);
}

/*********************************************************************
* Function: void run_star_demo(void)
*
* Overview: runs the demo based on input
*
* Parameters: None
*********************************************************************/
void run_star_demo(void)
{
    t1.Val = MiWi_TickGet ();
    display_connections = true;
    while(display_connections)
    {
        P2PTasks();
#if defined(ENABLE_NETWORK_FREEZER)
#if PDS_ENABLE_WEAR_LEVELING
        PDS_TaskHandler();
#endif
#endif
#if defined(ENABLE_SLEEP_FEATURE)
        if ((role == END_DEVICE) && Total_Connections())
        {
            uint32_t timeToSleep = 0;
            /* Check whether the stack allows to sleep, if yes, put the device
            into sleep */
            if(MiApp_ReadyToSleep(&timeToSleep))
            {
#if defined (ENABLE_CONSOLE)
                /* Disable UART */
                sio2host_disable();
#endif
                /* Put the MCU into sleep */
                sleepMgr_sleep(timeToSleep);
                //printf("\r\nDevice is sleeping");
#if defined (ENABLE_CONSOLE)
                /* Enable UART */
                sio2host_enable();
#endif
            }
        }
#endif
            /*******************************************************************/
            // If no packet received, now we can check if we want to send out
            // any information.
            // Function ButtonPressed will return if any of the two buttons
            // has been pushed.
            /*******************************************************************/
#if defined (CONF_BOARD_JOYSTICK)
        if(update_ed == true)
        {
#if defined (ENABLE_LCD)
            LCD_Erase();
            if (myConnectionIndex_in_PanCo  == select_ed)
            {   /* if END_device displays itself , "me" is added in display to denote itself*/
                snprintf(LCDText, sizeof(LCDText), "UP:%02d-%02x%02x%02x-me \nDOWN: Change node", END_DEVICES_Short_Address[select_ed].connection_slot,END_DEVICES_Short_Address[select_ed].Address[0],
                END_DEVICES_Short_Address[select_ed].Address[1],END_DEVICES_Short_Address[select_ed].Address[2] );
                LCD_Update();
            }
            else
            {
                snprintf(LCDText, sizeof(LCDText), "UP:%02d-%02x%02x%02x \nDOWN: Change node", END_DEVICES_Short_Address[select_ed].connection_slot,END_DEVICES_Short_Address[select_ed].Address[0],
                END_DEVICES_Short_Address[select_ed].Address[1],END_DEVICES_Short_Address[select_ed].Address[2] );
                LCD_Update();
            }
            snprintf(LCDText, sizeof(LCDText), "UP:%02d-%02x%02x%02x \nDOWN: Change node", END_DEVICES_Short_Address[select_ed].connection_slot,END_DEVICES_Short_Address[select_ed].Address[0],
            END_DEVICES_Short_Address[select_ed].Address[1],END_DEVICES_Short_Address[select_ed].Address[2]  );
            LCD_Update();
#endif
        }
        uint8_t JoyStickAction = JoystickPressed();
        switch( JoyStickAction )
        {
            case JOYSTICK_CENTER:
            {
                chk_sel_status = true;
                select_ed = 0;
                update_ed = true;
                /* User Selected Change end device */
                LCD_Erase();
                if (myConnectionIndex_in_PanCo  == select_ed)
                {   // if END_device displays itself , "me" is added in display to denote itself
                    snprintf(LCDText, sizeof(LCDText), "UP:%02d-%02x%02x%02x-me \nDOWN: Change node", END_DEVICES_Short_Address[select_ed].connection_slot,END_DEVICES_Short_Address[select_ed].Address[0],
                    END_DEVICES_Short_Address[select_ed].Address[1],END_DEVICES_Short_Address[select_ed].Address[2] );
                    LCD_Update();
                }
                else
                {
                    snprintf(LCDText, sizeof(LCDText), "UP:%02d-%02x%02x%02x \nDOWN: Change node", END_DEVICES_Short_Address[select_ed].connection_slot,END_DEVICES_Short_Address[select_ed].Address[0],
                    END_DEVICES_Short_Address[select_ed].Address[1],END_DEVICES_Short_Address[select_ed].Address[2] );
                    LCD_Update();
                }
                /* Display another Peer Device Address */
                chk_sel_status = true;
            }
            break;

            case JOYSTICK_UP:
            {
                if(chk_sel_status)
                {
                    bool mac_ack_status;
                    update_ed = false;    // No Display of peer Device Info
                    chk_sel_status = false;   // Selection Mode off

                    if (myConnectionIndex_in_PanCo == select_ed)
                    {
                        /* IF on the demo , a END_Device displays its own Connection Detail
                             We unicast data packet to just PAN COR , No forwarding */
                        mac_ack_status = MiApp_SendData(LONG_ADDR_LEN, connectionTable[0].Address, MIWI_TEXT_LEN,
                                                    (uint8_t*)&MiWi[(TxSynCount2%6)][0], msghandledemo++, true, dataConfcb);
                        if (mac_ack_status)
                        {
                            TxSynCount2++;
                        }
                    }
                    else
                    {
                        /* Data can be sent at a time from one END_DEVICE_TO_ANOTHER
                           Edx --> Pan CO --> EDy
                           To forward a Packet from one ED to another ED , address should be specified with length as 3
                           and address as end dest device short address (3 bytes)
                        */
                        mac_ack_status = MiApp_SendData(3, END_DEVICES_Short_Address[select_ed].Address,
                        MIWI_TEXT_LEN, (uint8_t *)&MiWi[(TxSynCount%6)][0], msghandledemo++, true, dataConfcb);

                        if (mac_ack_status)
                        {
                            TxSynCount++;
                        }
                    }
                }
            }

            break;

            case JOYSTICK_DOWN:
            {
                if(chk_sel_status)
                {
                      /* Reset Peer Device Info */
                    if (select_ed > end_nodes-1)
                    {
                        /* If end of Peer Device Info reset the count */
                        select_ed = 0;
                    }
                    else
                    {
                        /* New device Information */
                        select_ed = select_ed+1;
                    }
                }
            }
            break;

            default:
            break;
        }
#endif
            uint8_t PressedButton = ButtonPressed();
            if ( PressedButton == 1 || PressedButton == 2)
            {

                if (role == PAN_COORD)
                {
                    /*******************************************************************/
                    // Button 1 pressed. We need to send out the bitmap of word "MiWi".
                    /*******************************************************************/
                    uint16_t broadcastAddress = 0xFFFF;
                    bool mac_ack_status;

                    /* Function MiApp_SendData is used to broadcast a message with address as 0xFFFF */
                    mac_ack_status = MiApp_SendData(SHORT_ADDR_LEN, (uint8_t *)&broadcastAddress,
                        MIWI_TEXT_LEN, (uint8_t *)&MiWi[(TxSynCount%6)][0], msghandledemo++, true, dataConfcb);

                    if (mac_ack_status)
                    {
                        /* Update the bitmap count */
                        TxSynCount++;
                    }
                }
                else
                {

                    select_ed = 0;
                    chk_sel_status = true;
                    update_ed = true;
                    while(update_ed == true)
                    {
                        /* User Selected Change end device */
#if defined (ENABLE_LCD)
                        LCD_Erase();
                        if (myConnectionIndex_in_PanCo  == select_ed)
                        {        /* if END_device displays itself , "me" is added in display to denote itself */
                                snprintf(LCDText, sizeof(LCDText), "SW0:%02d-%02x%02x%02x-me \nBUTTON1: Change node", END_DEVICES_Short_Address[select_ed].connection_slot,END_DEVICES_Short_Address[select_ed].Address[0],
                                    END_DEVICES_Short_Address[select_ed].Address[1],END_DEVICES_Short_Address[select_ed].Address[2] );
                                LCD_Update();
                        }
                        else
                        {
                                snprintf(LCDText, sizeof(LCDText), "SW0:%02d-%02x%02x%02x \nBUTTON1: Change node", END_DEVICES_Short_Address[select_ed].connection_slot,END_DEVICES_Short_Address[select_ed].Address[0],
                                    END_DEVICES_Short_Address[select_ed].Address[1],END_DEVICES_Short_Address[select_ed].Address[2] );
                                LCD_Update();
                        }
                        LCD_Update();
#endif
                        chk_sel_status = true;

                        while(chk_sel_status)
                        {
                                bool mac_ack_status;
                                /* Check for Button Press */
                                uint8_t switch_val = ButtonPressed();

                                /* While waiting in TX , RX will process if any message was available */
                                P2PTasks ();

                                /* Button 1 is pressed, Initiate the Packet transmission */
                                if(switch_val == 1)
                                {
                                    update_ed = false;    // No Display of peer Device Info
                                    chk_sel_status = false;   // Selection Mode off

                                    if (myConnectionIndex_in_PanCo == select_ed)
                                    {
                                        /* IF on the demo , a END_Device displays its own Connection Detail
                                           unicast data packet to just PAN COR , No forwarding */
                                        mac_ack_status = MiApp_SendData(LONG_ADDR_LEN, connectionTable[0].Address,
                                        MIWI_TEXT_LEN, (uint8_t*)&MiWi[(TxSynCount%6)][0], msghandledemo++, true, dataConfcb);

                                        if (mac_ack_status)
                                        {
                                            TxSynCount++;
                                        }
                                    }

                                    else
                                    {
                                        /* Data can be sent at a time from one END_DEVICE_TO_ANOTHER
                                           Edx --> Pan CO --> EDy
                                           To forward a Packet from one ED to another ED , address should be specified with length as 3
                                           and address as end dest device short address (3 bytes)
                                        */
                                        mac_ack_status = MiApp_SendData(3, END_DEVICES_Short_Address[select_ed].Address,
                                        MIWI_TEXT_LEN, (uint8_t*)&MiWi[(TxSynCount%6)][0], msghandledemo++, true, dataConfcb);

                                        if (mac_ack_status)
                                        {
                                            TxSynCount++;
                                        }
                                    }
                                }
                                else if(switch_val == 2)   /* SWITCH to new peer Device Info */
                                {
                                    if (select_ed > end_nodes-1)  /* Reset Peer Device Info */
                                    {
                                        /* If end of Peer Device Info reset the count */
                                        select_ed = 0;
                                    }
                                    else
                                    {
                                        /* New device Information */
                                        select_ed = select_ed+1;
                                    }
                                    chk_sel_status = false;

                                }  /* end of SWITCH to new peer Device INFO */
                            } /* end of Selection of Peer Device */

                    } /* END of Display of PEER DEVICE Info */

                }/* end of END_DEVICE send packet option */
            } /* end of options on button press */

        t2.Val = MiWi_TickGet ();
        /* Display the no of End devices in Network */
        if( MiWi_TickGetDiff(t2, t1) > (15 * ONE_SECOND) && (role == PAN_COORD))
        {
#if defined (ENABLE_LCD)
            LCD_Erase();
            snprintf(LCDText, sizeof(LCDText), "END_DEVICES :%02d",conn_size);
            LCD_Update();
#endif
            delay_ms(500);
            display_connections = false;
            STAR_DEMO_OPTIONS_MESSAGE (true);

        }

    }
}
#endif

/*********************************************************************
* Function: void ReceivedDataIndication (RECEIVED_MESSAGE *ind)
*
* Overview: Process a Received Message
*
* PreCondition: MiApp_ProtocolInit
*
* Input:  RECEIVED_MESSAGE *ind - Indication structure
********************************************************************/
void ReceivedDataIndication (RECEIVED_MESSAGE *ind)
{
#if defined(ENABLE_CONSOLE)
    /* Print the received information via Console */
    DemoOutput_HandleMessage();
#endif

    /* Update the TX AND RX Counts on the display */
    DemoOutput_UpdateTxRx(TxNum, ++RxNum);

    /* Delay for Showing the contents on the display before showing instructions */
    delay_ms(500);

#if !defined(ENABLE_SLEEP_FEATURE)
    /* Toggle LED2 to indicate receiving a packet */
    LED_Toggle(LED0);
#endif

    /* Display the Source End Device Info on reception msg, Do not display if it is
       a PAN CO or if the message received was a broadcast packet */
    if ((role == END_DEVICE) && !rxMessage.flags.bits.broadcast)
    {
        Source_END_DEVICE_INFO(rxMessage.Payload);
    }
    /* Display the Instructions message */
    STAR_DEMO_OPTIONS_MESSAGE (role);
}