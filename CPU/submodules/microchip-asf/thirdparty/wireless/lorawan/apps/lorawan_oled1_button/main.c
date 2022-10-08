/**
* \file  main.c
*
* \brief LORAWAN Getting Started  OLED1 [Button] Demo Application main file
*
*
* Copyright (c) 2019 Microchip Technology Inc. and its subsidiaries.
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

/****************************** INCLUDES **************************************/
#include "system_low_power.h"
#include "radio_driver_hal.h"
#include "lorawan.h"
#include "sys.h"
#include "system_init.h"
#include "system_assert.h"
#include "aes_engine.h"
#include "enddevice_demo.h"
#include "sio2host.h"
#include "extint.h"
#include "conf_app.h"
#include "sw_timer.h"
#ifdef CONF_PMM_ENABLE
#include "pmm.h"
#include  "conf_pmm.h"
#include "sleep_timer.h"
#include "sleep.h"
#endif
#include "conf_sio2host.h"
#if (ENABLE_PDS == 1)
#include "pds_interface.h"
#endif
#include "ssd1306.h"
#include "gfx_mono_text.h"
#include "sysfont.h"
#include "conf_menu.h"
#include "oled1_support.h"
#include "gfx_mono_menu.h"


/************************** Macro definition ***********************************/
/* Button debounce time in ms */
#define APP_DEBOUNCE_TIME       50

/************************** Global variables ***********************************/

uint8_t AppTimerId = 0xFF;
uint8_t demoITimerId = 0xFF;
uint8_t lTimerId = 0xFF;
struct user_gflags app_gflags;		// application global flags

uint8_t keycode;

char str[80];

#ifdef CONF_PMM_ENABLE
bool deviceResetsForWakeup = false;
#endif
/************************** Extern variables ***********************************/

/************************** Function Prototypes ********************************/
static void mdriver_init(void);
extern void button_read (void);
extern void periodicSend(void);


#if (_DEBUG_ == 1)
static void assertHandler(SystemAssertLevel_t level, uint16_t code);
#endif /* #if (_DEBUG_ == 1) */

/*********************************************************************//**
 \brief      Un-initialize app resources before going to low power mode
*************************************************************************/
#ifdef CONF_PMM_ENABLE
static void app_resources_uninit(void);
#endif

/****************************** FUNCTIONS **************************************/

/** Configures and registers the External Interrupt callback function
 *  To handle the SW0 push button on the XPRO board.
 *  upon a push button interrupt, the function button_read   will be called
 */
static void configure_eic_callback(void)
{
	extint_register_callback(button_read,
			BUTTON_0_EIC_LINE,
			EXTINT_CALLBACK_TYPE_DETECT);
	extint_chan_enable_callback(BUTTON_0_EIC_LINE,
			EXTINT_CALLBACK_TYPE_DETECT);
}

/** Configures the External Interrupt Controller to detect changes in the board
 *  SW0 button state.
 */
static void configure_extint(void)
{
	struct extint_chan_conf eint_chan_conf;
	extint_chan_get_config_defaults(&eint_chan_conf);

	eint_chan_conf.gpio_pin           = BUTTON_0_EIC_PIN;
	eint_chan_conf.gpio_pin_mux       = BUTTON_0_EIC_MUX;
	eint_chan_conf.detection_criteria = EXTINT_DETECT_FALLING;
	eint_chan_conf.filter_input_signal = true;
	extint_chan_set_config(BUTTON_0_EIC_LINE, &eint_chan_conf);
}



static void print_reset_causes(void)
{
    enum system_reset_cause rcause = system_get_reset_cause();
    printf("Last reset cause: ");
    if(rcause & (1 << 6)) {
        printf("System Reset Request\r\n");
    }
    if(rcause & (1 << 5)) {
        printf("Watchdog Reset\r\n");
    }
    if(rcause & (1 << 4)) {
        printf("External Reset\r\n");
    }
    if(rcause & (1 << 2)) {
        printf("Brown Out 33 Detector Reset\r\n");
    }
    if(rcause & (1 << 1)) {
        printf("Brown Out 12 Detector Reset\r\n");
    }
    if(rcause & (1 << 0)) {
        printf("Power-On Reset\r\n");
    }
}

#ifdef CONF_PMM_ENABLE
static void appWakeup(uint32_t sleptDuration)
{
    HAL_Radio_resources_init();
    sio2host_init();
    printf("\r\nwakeup from sleep_ok %ld ms\r\n", sleptDuration);
	// upon awake from sleep -- post task for periodic Sending of data ( will only arrive here if already joined to network )
	periodicSend();

}
#endif

#if (_DEBUG_ == 1)
static void assertHandler(SystemAssertLevel_t level, uint16_t code)
{
    printf("\r\n%04x\r\n", code);
    (void)level;
}
#endif /* #if (_DEBUG_ == 1) */

/**
 * \mainpage
 * \section preface Preface
 * This is the reference code for the LORAWAN Getting Started LoRaWAN [Button OLED1] Demo Application main file
 */
int main(void)
{
    /* System Initialization */
    system_init();
    /* Initialize the delay driver */
    delay_init();
    /* Initialize the board target resources */
    board_init();
	/*Configures the External Interrupt*/
	configure_extint();
	/*Configures the External Interrupt callback*/
	configure_eic_callback();

    INTERRUPT_GlobalInterruptEnable();

    /* Initialize Hardware and Software drivers */
    mdriver_init();
    /* Initialize the Serial Interface */
    sio2host_init();
    print_reset_causes();

#if (_DEBUG_ == 1)
    SYSTEM_AssertSubscribe(assertHandler);
#endif
    /* Initialize demo application */
    Stack_Init();

    SwTimerCreate(&demoITimerId);	// create demo user input timer
    SwTimerCreate(&lTimerId);		// create blink led timer
	SwTimerCreate(&AppTimerId);		// create application timer

    mote_demo_init();  // Configure demo and set active task to  SELECT_BAND_STATE
	keycode = 0;
    while (1)
    {
		serial_data_handler();
        SYSTEM_RunTasks();
#ifdef CONF_PMM_ENABLE
        if(app_gflags.joined == true)
        {
			if(app_gflags.sleep_en == true)
			{
	            PMM_SleepReq_t sleepReq;
	            /* Put the application to sleep */
	            sleepReq.sleepTimeMs = DEMO_CONF_DEFAULT_APP_SLEEP_TIME_MS;
	            sleepReq.pmmWakeupCallback = appWakeup;
	            sleepReq.sleep_mode = CONF_PMM_SLEEPMODE_WHEN_IDLE;
	            if (CONF_PMM_SLEEPMODE_WHEN_IDLE == SLEEP_MODE_STANDBY)
	            {
	                deviceResetsForWakeup = false;
	            }
	            if (true == LORAWAN_ReadyToSleep(deviceResetsForWakeup))
	            {
	                app_resources_uninit();
					app_gflags.oled_button_chk = false;
					/*Configures the External Interrupt*/
					configure_extint();
	                if (PMM_SLEEP_REQ_DENIED == PMM_Sleep(&sleepReq))
	                {
	                    HAL_Radio_resources_init();
						app_gflags.oled_button_chk = true;
	                    sio2host_init();
	                }
	            }
        	}
        }
#endif
    }
}

/* Initializes all the hardware and software modules used for Stack operation */
static void mdriver_init(void)
{
    /* Initialize the Radio Hardware */
    HAL_RadioInit();
    /* Initialize the AES Hardware Engine */
    AESInit();
    /* Initialize the Software Timer Module */
    SystemTimerInit();
#ifdef CONF_PMM_ENABLE
    /* Initialize the Sleep Timer Module */
    SleepTimerInit();
#endif
#if (ENABLE_PDS == 1)
    /* PDS Module Init */
    PDS_Init();
#endif
}



#ifdef CONF_PMM_ENABLE
static void app_resources_uninit(void)
{
	/* Disable USART TX and RX Pins */
	struct port_config pin_conf;
	port_get_config_defaults(&pin_conf);
	pin_conf.powersave  = true;
	#ifdef HOST_SERCOM_PAD0_PIN
	port_pin_set_config(HOST_SERCOM_PAD0_PIN, &pin_conf);
	#endif
	#ifdef HOST_SERCOM_PAD1_PIN
	port_pin_set_config(HOST_SERCOM_PAD1_PIN, &pin_conf);
	#endif
	/* Disable UART module */
	sio2host_deinit();
	/* Disable Transceiver SPI Module */
	HAL_RadioDeInit();
}
#endif


/**
 End of File
 */
