/**
 * \file config.h
 *
 * \brief BM mode configuration
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

#ifndef __CONFIG_H__
#define __CONFIG_H__

#if defined(__PIC32MX__) || defined(__C32__)
#define UART1M UART2
#define UART2M UART1
#define SYS_FREQ                        (80000000L)
#define	GetPeripheralClock()		(SYS_FREQ/(1 << OSCCONbits.PBDIV))
#define	GetInstructionClock()		(SYS_FREQ)

#define GetUARTBaudrate()               (115200)


#elif defined(__18CXX) || defined(__XC8)
#define UART1M 1
#define UART2M 0

/* Specify the System clock frequency in Hz */
#define SYS_FREQ                        (40000000ul)
/* Specify the Peripheral clock frequency in Hz */
#define GetPeripheralClock()            (SYS_FREQ/4)

#elif defined ( __C30__ ) || defined ( __XC16__ )

#define UART1M 1
#define UART2M 0

/* Specify the System clock frequency in Hz */
#define SYS_FREQ                        (16000000ul)
/* Specify the Peripheral clock frequency in Hz */
#define GetPeripheralClock()            (SYS_FREQ)
#define	GetInstructionClock()		(SYS_FREQ)

#endif

#define BUFFER_LEN_MAX                  64

#define DELAY_4S 4000

#if defined(__18CXX) || defined(__XC8)
    #define BM_RST_DIR    TRISAbits.TRISA2
    #define BM_P20_DIR    TRISAbits.TRISA5
    #define BM_P24_DIR    TRISEbits.TRISE3
    #define BM_EAN_DIR    TRISAbits.TRISA4

    #define BM_RST        LATAbits.LATA2
    #define BM_P20        LATAbits.LATA5
    #define BM_P24        LATEbits.LATE3
    #define BM_EAN        LATAbits.LATA4

    #define BM_CTRL_INIT()    { BM_RST_DIR = 0; BM_P20_DIR = 1; BM_P24_DIR = 1; BM_EAN_DIR = 1; BM_RST = 0; }

#elif defined(__PIC32MX__) || defined(__XC32__) || defined ( __C30__ ) || defined ( __XC16__ )
    #define BM_RST_DIR    TRISGbits.TRISG1
    #define BM_P20_DIR    TRISBbits.TRISB1
    #define BM_P24_DIR    TRISBbits.TRISB3
    #define BM_EAN_DIR    TRISBbits.TRISB4

    #define BM_RST        LATGbits.LATG1
    #define BM_P20        LATBbits.LATB1
    #define BM_P24        LATBbits.LATB3
    #define BM_EAN        LATBbits.LATB4

    #define BM_CTRL_INIT()    { AD1PCFG = 0xFFFF; BM_RST_DIR = 0; BM_P20_DIR = 1; BM_P24_DIR = 1; BM_EAN_DIR = 1; BM_RST = 0; }
#endif

#if (HOST_MCU_DEVICE == HOST_MCU_SAML21)

#define BM_MODULE_ControlInit()			ble_configure_control_pin()

#define BM_MODULE_ApplicationModeSet()	{   ble_configure_reset_pin_dir_output();						\
											bm7x_reset_pin_set_low();                                   \
											delay_ms(5);												\
											bm7x_mode_pin_set_high();									\
											delay_ms(3);												\
											ble_configure_reset_pin_dir_input();						\
											bm7x_wakeup_pin_set_high();	}

#define BM_MODULE_ApplicationModeGet() ((bm7x_mode_pin_level() == 1) ? 1 : 0)

#define BM_MODULE_ProgramModeSet()		{   ble_configure_reset_pin_dir_output();						\
											bm7x_reset_pin_set_low();                                   \
											delay_ms(5);												\
											bm7x_mode_pin_set_low();									\
											delay_ms(3);												\
											ble_configure_reset_pin_dir_input();						\
											bm7x_wakeup_pin_set_high();}

#define BM_MODULE_ProgramModeGet()		((bm7x_mode_pin_level() == 1) ? 0 : 1)

#define BM_RESET()						{	ble_configure_reset_pin_dir_output();						\
											bm7x_reset_pin_set_low();  									\
											delay_ms(5);												\
											ble_configure_reset_pin_dir_input(); }                                        \

#endif /* #if (HOST_MCU_DEVICE == HOST_MCU_SAML21) */

#endif /* __CONFIG_H__ */
