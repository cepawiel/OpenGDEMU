/**
* \file  p2p_demo.c
*
* \brief Demo Application for MiWi P2P Implementation
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
#include "p2p_demo.h"
#include "mimem.h"
#include "asf.h"
#if defined(ENABLE_SLEEP_FEATURE)
#include "sleep_mgr.h"
#endif
#if defined (ENABLE_CONSOLE)
#include "sio2host.h"
#endif

#if defined(ENABLE_NETWORK_FREEZER)
#include "pdsMemIds.h"
#include "pdsDataServer.h"
#include "wlPdsTaskManager.h"
#endif

#if defined(PROTOCOL_P2P)
/************************ LOCAL VARIABLES ****************************************/
uint8_t i;
uint8_t TxSynCount = 0;
uint8_t TxSynCount2 = 0;
uint8_t TxNum = 0;
uint8_t RxNum = 0;
bool chk_sel_status = true;
uint8_t NumOfActiveScanResponse;
bool update_ed;
uint8_t select_ed;
uint8_t msghandledemo = 0;
/* Connection Table Memory */
extern CONNECTION_ENTRY connectionTable[CONNECTION_SIZE];

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
    DemoOutput_Instruction ();
}

/*********************************************************************
* Function: void run_p2p_demo(void)
*
* Overview: runs the demo based on input
*
* Parameters: None
*********************************************************************/
void run_p2p_demo(void)
{
#if defined(ENABLE_SLEEP_FEATURE)
    if (Total_Connections())
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
    {
        /*******************************************************************/
        // If no packet received, now we can check if we want to send out
        // any information.
        // Function ButtonPressed will return if any of the two buttons
        // has been pushed.
        /*******************************************************************/
#if defined (CONF_BOARD_JOYSTICK)
        uint8_t JoyStickAction = JoystickPressed();
        switch( JoyStickAction )
        {
            case JOYSTICK_CENTER:
                chk_sel_status = true;
                select_ed = 0;
                update_ed = true;
                //Peer Device Info
                #if defined (ENABLE_LCD)
                LCD_Erase();
                snprintf(LCDText, sizeof(LCDText), "UP  : %02d-%02x%02x%02x \nDOWN: Change node", select_ed,connectionTable[select_ed].Address[0],
                connectionTable[select_ed].Address[1],connectionTable[select_ed].Address[2]);
                LCD_Update();
                #endif
                // Display another Peer Device Address
                chk_sel_status = true;
            break;

            case JOYSTICK_UP:
                if(chk_sel_status)
                {
                    update_ed = false;
                    chk_sel_status = false;

                    /* IF on the demo , a END_Device displays its own Connection Detail
                             We unicast data packet to just PAN COR*/
                    if( MiApp_SendData(LONG_ADDR_LEN, connectionTable[select_ed].Address, DE_LEN, (uint8_t*)&DE[(TxSynCount2%6)][0], msghandledemo++, true, dataConfcb) == false )
                    {
                        DemoOutput_UnicastFail();
                    }
                    else
                    {
                        /* Successful Transmission */
                        TxSynCount2++;
                    }
                    select_ed = 0;
                }
            break;

            case JOYSTICK_DOWN:
                if(chk_sel_status)
                {
                    if (select_ed > conn_size-2)
                    {
                        // Last Peer Device
                        select_ed = 0;
                    }
                    else
                    {
                        // Update the Display
                        select_ed = select_ed+1;
                    }
#if defined (ENABLE_LCD)
                    LCD_Erase();
                    snprintf(LCDText, sizeof(LCDText), "UP  : %02d-%02x%02x%02x \nDOWN: Change node", select_ed, connectionTable[select_ed].Address[0],
                    connectionTable[select_ed].Address[1],connectionTable[select_ed].Address[2]);
                    LCD_Update();
#endif
                }
            break;

            default:
            break;
        }
#endif

        uint8_t PressedButton = ButtonPressed();
        switch( PressedButton )
        {
            case 1:
            {
                /*******************************************************************/
                // Button 1 pressed. We need to send out the bitmap of word "MiWi".
                /*******************************************************************/
                uint16_t broadcastAddress = 0xFFFF;
                bool mac_ack_status;

                /* Function MiApp_SendData is used to broadcast a message with address as 0xFFFF */
                mac_ack_status = MiApp_SendData(SHORT_ADDR_LEN, (uint8_t *)&broadcastAddress, MIWI_TEXT_LEN, (uint8_t *)&MiWi[(TxSynCount%6)][0], msghandledemo++, true, dataConfcb);
                if (mac_ack_status)
                {
                    /* Update the bitmap count */
                    TxSynCount++;
                }
            }
            break;

#if !defined (CONF_BOARD_JOYSTICK)
            case 2:
                chk_sel_status = true;
                select_ed =0;
                update_ed = true;
                while(update_ed == true)
                {
                    //Peer Device Info
#if defined (ENABLE_LCD)
                    LCD_Erase();
                    snprintf(LCDText, sizeof(LCDText),(char*)"SW:%02d-%02x%02x%02x \nBUTTON1: Change node",select_ed,connectionTable[select_ed].Address[0],
                            connectionTable[select_ed].Address[1],connectionTable[select_ed].Address[2]);
                    LCD_Update();
#endif
                    // Display another Peer Device Address
                    chk_sel_status = true;

                    while(chk_sel_status)
                    {
                        uint8_t switch_val = ButtonPressed();
                        //// While waiting in TX , RX will process if any message was available
                        P2PTasks();
#if defined(ENABLE_NETWORK_FREEZER)
#if PDS_ENABLE_WEAR_LEVELING
                        PDS_TaskHandler();
#endif
#endif
                        if(switch_val == 1)
                        {
                            update_ed = false;
                            chk_sel_status = false;

                            if( MiApp_SendData(LONG_ADDR_LEN, connectionTable[select_ed].Address, DE_LEN, (uint8_t*)&DE[(TxSynCount2%6)][i], msghandledemo++, 1, dataConfcb) == false)
                            {
                                DemoOutput_UnicastFail();
                            }
                            else
                            {
                                // Successful Transmission
                                TxSynCount2++;
                            }
                            break;
                        }   // end switch_val = 1

                        else if(switch_val == 2)
                        {
                            if (select_ed > conn_size-2)
                            {
                                // Last Peer Device
                                select_ed = 0;
                            }
                            else
                            {
                                // Update the Display
                                select_ed = select_ed+1;
                            }
                            chk_sel_status = false;
                        }   // end switch_val = 2
                    }  // end of Peer Device selection

                } // End of Display


                break;
#endif
            default:
                break;
        }

    }

}

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

#if !defined(ENABLE_SLEEP_FEATURE)
    /* Toggle LED2 to indicate receiving a packet */
    LED_Toggle(LED0);
#endif

    /* Display the Instructions message */
    DemoOutput_Instruction();
}
#endif