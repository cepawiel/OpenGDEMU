/**
 * \file
 *
 * \brief Instance description for PM
 *
 * Copyright (c) 2016-2018 Microchip Technology Inc. and its subsidiaries.
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

#ifndef _SAMHA1_PM_INSTANCE_
#define _SAMHA1_PM_INSTANCE_

/* ========== Register definition for PM peripheral ========== */
#if (defined(__ASSEMBLY__) || defined(__IAR_SYSTEMS_ASM__))
#define REG_PM_CTRL                (0x40000400) /**< \brief (PM) Control */
#define REG_PM_SLEEP               (0x40000401) /**< \brief (PM) Sleep Mode */
#define REG_PM_EXTCTRL             (0x40000402) /**< \brief (PM) External Reset Controller */
#define REG_PM_CPUSEL              (0x40000408) /**< \brief (PM) CPU Clock Select */
#define REG_PM_APBASEL             (0x40000409) /**< \brief (PM) APBA Clock Select */
#define REG_PM_APBBSEL             (0x4000040A) /**< \brief (PM) APBB Clock Select */
#define REG_PM_APBCSEL             (0x4000040B) /**< \brief (PM) APBC Clock Select */
#define REG_PM_AHBMASK             (0x40000414) /**< \brief (PM) AHB Mask */
#define REG_PM_APBAMASK            (0x40000418) /**< \brief (PM) APBA Mask */
#define REG_PM_APBBMASK            (0x4000041C) /**< \brief (PM) APBB Mask */
#define REG_PM_APBCMASK            (0x40000420) /**< \brief (PM) APBC Mask */
#define REG_PM_INTENCLR            (0x40000434) /**< \brief (PM) Interrupt Enable Clear */
#define REG_PM_INTENSET            (0x40000435) /**< \brief (PM) Interrupt Enable Set */
#define REG_PM_INTFLAG             (0x40000436) /**< \brief (PM) Interrupt Flag Status and Clear */
#define REG_PM_RCAUSE              (0x40000438) /**< \brief (PM) Reset Cause */
#else
#define REG_PM_CTRL                (*(RwReg8 *)0x40000400UL) /**< \brief (PM) Control */
#define REG_PM_SLEEP               (*(RwReg8 *)0x40000401UL) /**< \brief (PM) Sleep Mode */
#define REG_PM_EXTCTRL             (*(RwReg8 *)0x40000402UL) /**< \brief (PM) External Reset Controller */
#define REG_PM_CPUSEL              (*(RwReg8 *)0x40000408UL) /**< \brief (PM) CPU Clock Select */
#define REG_PM_APBASEL             (*(RwReg8 *)0x40000409UL) /**< \brief (PM) APBA Clock Select */
#define REG_PM_APBBSEL             (*(RwReg8 *)0x4000040AUL) /**< \brief (PM) APBB Clock Select */
#define REG_PM_APBCSEL             (*(RwReg8 *)0x4000040BUL) /**< \brief (PM) APBC Clock Select */
#define REG_PM_AHBMASK             (*(RwReg  *)0x40000414UL) /**< \brief (PM) AHB Mask */
#define REG_PM_APBAMASK            (*(RwReg  *)0x40000418UL) /**< \brief (PM) APBA Mask */
#define REG_PM_APBBMASK            (*(RwReg  *)0x4000041CUL) /**< \brief (PM) APBB Mask */
#define REG_PM_APBCMASK            (*(RwReg  *)0x40000420UL) /**< \brief (PM) APBC Mask */
#define REG_PM_INTENCLR            (*(RwReg8 *)0x40000434UL) /**< \brief (PM) Interrupt Enable Clear */
#define REG_PM_INTENSET            (*(RwReg8 *)0x40000435UL) /**< \brief (PM) Interrupt Enable Set */
#define REG_PM_INTFLAG             (*(RwReg8 *)0x40000436UL) /**< \brief (PM) Interrupt Flag Status and Clear */
#define REG_PM_RCAUSE              (*(RoReg8 *)0x40000438UL) /**< \brief (PM) Reset Cause */
#endif /* (defined(__ASSEMBLY__) || defined(__IAR_SYSTEMS_ASM__)) */

/* ========== Instance parameters for PM peripheral ========== */
#define PM_CTRL_MCSEL_DFLL48M       3       
#define PM_CTRL_MCSEL_GCLK          0       
#define PM_CTRL_MCSEL_OSC8M         1       
#define PM_CTRL_MCSEL_XOSC          2       
#define PM_PM_CLK_APB_NUM           2       

#endif /* _SAMHA1_PM_INSTANCE_ */
