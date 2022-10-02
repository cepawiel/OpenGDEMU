/**
* \file  main.c
*
* \brief Getting Started  [USB_CDC] Demo Application main file
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

#include <stdarg.h>
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

#include "conf_sio2host.h"
#if (ENABLE_PDS == 1)
#include "pds_interface.h"
#endif

#include "conf_usb.h"


/************************** Macro definition ***********************************/
/* Button debounce time in ms */
#define APP_DEBOUNCE_TIME       50

/************************** Global variables ***********************************/
bool factory_reset = false;
bool bandSelected = false;
uint8_t AppTimerId = 0xFF;
uint8_t demoITimerId = 0xFF;
uint8_t lTimerId = 0xFF;


bool usb_task_idle = true;

static volatile bool main_b_cdc_enable = false;
char pstr1[80];		// buffer for usb cdc string creation

// added for USB CDC
uint8_t usb_rx_databuf[80];

/************************** Extern variables ***********************************/

/************************** Function Prototypes ********************************/
static void driver_init(void);
extern void button_read (void);
void serialUSBsend(const char* str);
SYSTEM_TaskStatus_t USB_TaskHandler(void);
int USB_sprintf(char * str, const char * format, ... ) __attribute__ ((format (gnu_printf, 2, 3)));


#if (_DEBUG_ == 1)
static void assertHandler(SystemAssertLevel_t level, uint16_t code);
#endif /* #if (_DEBUG_ == 1) */

/*********************************************************************//**
 \brief      Un-initialize app resources before going to low power mode
*************************************************************************/

/****************************** FUNCTIONS **************************************/
// Send string thru SAMR34 Target USB port
void serialUSBsend(const char* str)
{
	udi_cdc_multi_write_buf(1,str,strlen(str));
}

// sprintf version for directing output to SAMR34 Target USB_CDC port
int USB_sprintf(char * str, const char * format, ... )
{
	va_list args;
	va_start(args, format);
	vsprintf(str,format, args);
	va_end(args);
	udi_cdc_multi_write_buf(1,str,strlen(str));
	return 0;
}

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


// Reset causes  output directed to EDBG serial console port
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
 * This is the reference code for the LORAWAN Getting Started LoRaWAN [USB_CDC] Demo Application main file
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
    driver_init();
    /* Initialize the Serial Interface */
    sio2host_init();

	// Start USB stack to authorize VBus monitoring
	udc_start();

    print_reset_causes();

#if (_DEBUG_ == 1)
    SYSTEM_AssertSubscribe(assertHandler);
#endif
    /* Initialize demo application */
    Stack_Init();

    SwTimerCreate(&demoITimerId);	// create demo user input timer
    SwTimerCreate(&lTimerId);		// create blink led timer
	SwTimerCreate(&AppTimerId);		// create periodic msg transmission timer

	printf("\r\n***********************************************\r\n");
	printf("Connect Terminal Application to Target USB port of SAMR34 XPRO\r\n");
	printf("Select 115.2k baud N 8 1 and make USB connection\r\n");
	printf("Then press SW0 on SAMR34 XPRO board to begin\r\n");
	printf("\r\n***********************************************\r\n");

// wait for button press to continue to test usb  send
	while (port_pin_get_input_level(BUTTON_0_PIN) == BUTTON_0_INACTIVE);

	printf("Console output now directed to Target USB port\r\n");

	// send USB CDC start up prompt
	USB_sprintf(pstr1,"Lorawan_USB_integration example \r\n");

    mote_demo_init();  // Configure demo and set active task to  SELECT_BAND_STATE

    while (1)
    {
        serial_data_handler();
        SYSTEM_RunTasks();
    }
}

/* Initializes all the hardware and software modules used for Stack operation */
static void driver_init(void)
{
    /* Initialize the Radio Hardware */
    HAL_RadioInit();
    /* Initialize the AES Hardware Engine */
    AESInit();
    /* Initialize the Software Timer Module */
    SystemTimerInit();

#if (ENABLE_PDS == 1)
    /* PDS Module Init */
    PDS_Init();
#endif
}


/*********************************************************************//*
 \brief  USB Task Handler   RX characters from USB port
 \return processing status
 ************************************************************************/
SYSTEM_TaskStatus_t USB_TaskHandler(void)
{
	uint32_t cnt;

	if ((cnt = udi_cdc_get_nb_received_data())) {
		/* get usb rx data */
		//ui_com_rx_start();
		cnt = udi_cdc_read_no_polling(usb_rx_databuf,79);
		usb_rx_databuf[cnt] = 0x00;
		udi_cdc_multi_write_buf(1, usb_rx_databuf, strlen((const char*)usb_rx_databuf));
	} else
	{
		/* Fifo empty  */
		//ui_com_rx_stop();
	}


	usb_task_idle = true;

	return SYSTEM_TASK_SUCCESS;

}



void main_callback_suspend_action(void)
{
	// Add code here
}

void main_callback_resume_action(void)
{
	// Add code here
}

void main_callback_sof_action(void)
{
	if (!main_b_cdc_enable)
	return;
	//Add code here
}

#ifdef USB_DEVICE_LPM_SUPPORT
void main_suspend_lpm_action(void)
{
	// Add code here
}

void main_remotewakeup_lpm_disable(void)
{
	// Add code here
}

void main_remotewakeup_lpm_enable(void)
{
	// Add code here
}
#endif

bool main_cdc_enable(uint8_t port)
{
	main_b_cdc_enable = true;
	// Add code here
	return true;
}

void main_cdc_disable(uint8_t port)
{
	main_b_cdc_enable = false;
	// Close communication
	// Add code here
}

void main_cdc_set_dtr(uint8_t port, bool b_enable)
{
	if (b_enable) {
		// Host terminal has open COM
		// Add code here
		}else{
		// Host terminal has close COM
		// Add code here
	}
}


/**
 End of File
 */
