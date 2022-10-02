/**
* \file  moveIntVector.h
*
* \brief Moving the interrupt vector from app to boot section and vice versa.
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
#ifndef _MOVEINTVECTOR_H
#define _MOVEINTVECTOR_H

/******************************************************************************
                   Prototypes section
******************************************************************************/
#if defined(RFR2_SPM)
/**************************************************************************//**
\brief Move Interrupt vectors from application section to boot section
******************************************************************************/
void moveInterrupToBootSection(void);

/**************************************************************************//**
\brief Move Interrupt vectors from boot section to application section
******************************************************************************/
void moveInterrupToAppSection(void);
#endif

/******************************************************************************
                     Inline static functions section
******************************************************************************/
/**************************************************************************//**
\brief This routine moves the interrupt vector from app to boot section. \n
******************************************************************************/
static inline void moveIntVectToBootSection(void)
{
  #if defined(RFR2_SPM)
  // Enable change of Interrupt Vectors
  moveInterrupToBootSection();
  sei();
  #endif
}

/**************************************************************************//**
\brief This routine moves the interrupt vector from boot to app section. \n
******************************************************************************/
static inline void moveIntVectToAppSection(void)
{
  #if defined(RFR2_SPM)
  // disable global interrupt
  cli();
  // Change interrupt vector to apps section
  moveInterrupToAppSection();
  #endif
}

#endif // _MOVEINTVECTOR_H

// eof moveIntVector.h
