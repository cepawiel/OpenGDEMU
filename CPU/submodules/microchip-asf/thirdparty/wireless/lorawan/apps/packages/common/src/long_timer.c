/**
* \file  long_timer.c
*
* \brief Timer implementation that can elapse duration greater than SW_TIMER_MAX_TIMEOUT
*		
*
* Copyright (c) 2020 Microchip Technology Inc. and its subsidiaries. 
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
 
/************************************************************************/
/* INCLUDES                                                             */
/************************************************************************/
#include "long_timer.h"
#include "sw_timer.h"
#include "log.h"

/************************************************************************/
/* TYPE DEFINES                                                         */
/************************************************************************/
typedef void (*LongTimerCallback_t)(void);

/************************************************************************/
/* STATIC FUNCTION PROTOTYPES                                           */
/************************************************************************/
static void LongTimerCallback(void);

/************************************************************************/
/* STATIC VARIABLES                                                     */
/************************************************************************/
static uint8_t pkgCommonLongTimerId = SWTIMER_INVALID;
static uint64_t longTimerRemInterval;
static LongTimerCallback_t longTimerCb;

/************************************************************************/
/* FUNCTION DEFINITIONS                                                 */
/************************************************************************/
void LongTimerInit(void)
{
    if (LORAWAN_SUCCESS != SwTimerCreate(&pkgCommonLongTimerId))
    {
        LOGLINE_ERROR("%s: pkgCommonLongTimerId creation failed",
            __FUNCTION__);
    }
}

static void LongTimerCallback(void)
{
    LOGLINE_DEBUG("%s", __FUNCTION__);
    
    if ( longTimerRemInterval )
    {        
        if (longTimerRemInterval > ((uint64_t)LONG_TIMER_LAP_DURATION_US))
        {
            longTimerRemInterval -= LONG_TIMER_LAP_DURATION_US;
            SwTimerStart(pkgCommonLongTimerId,
                LONG_TIMER_LAP_DURATION_US,
                SW_TIMEOUT_RELATIVE,
                LongTimerCallback,
                NULL);
            LOGLINE_DEBUG("Next lap started in LongTimer, remaining=%lums",
                (uint32_t)(longTimerRemInterval/1000));
        }
        else
        {
            SwTimerStart(pkgCommonLongTimerId,
                (uint32_t)longTimerRemInterval,
                SW_TIMEOUT_RELATIVE,
                longTimerCb,
                NULL);
            longTimerRemInterval ^= longTimerRemInterval;
            LOGLINE_DEBUG("Final lap of LongTimer started");
        }
    }
}

void LongTimerStart(uint64_t intervalMicros, void (*cb)(void))
{
    if (SWTIMER_INVALID == pkgCommonLongTimerId)
    {
      return;
    }

    if (intervalMicros > ((uint64_t)LONG_TIMER_LAP_DURATION_US))
    {
        longTimerRemInterval = intervalMicros - LONG_TIMER_LAP_DURATION_US;
        longTimerCb = cb;
        SwTimerStart(pkgCommonLongTimerId,
            LONG_TIMER_LAP_DURATION_US,
            SW_TIMEOUT_RELATIVE,
            LongTimerCallback,
            NULL);
    }
    else
    {
        uint32_t timeout = intervalMicros;
        longTimerCb = NULL;
        longTimerRemInterval ^= longTimerRemInterval;
        SwTimerStart(pkgCommonLongTimerId,
            timeout,
            SW_TIMEOUT_RELATIVE,
            cb,
            NULL);
    }
    LOGLINE_DEBUG("LongTimer started, interval=%lums",
        (uint32_t)(intervalMicros/1000));
}

void LongTimerReset(void)
{
    if (SWTIMER_INVALID == pkgCommonLongTimerId)
    {
        return;
    }
    
    if (SwTimerIsRunning(pkgCommonLongTimerId))
    {
        SwTimerStop(pkgCommonLongTimerId);
    }

    longTimerRemInterval ^= longTimerRemInterval;
    longTimerCb = NULL;
}

/* eof pkg_common.c */
