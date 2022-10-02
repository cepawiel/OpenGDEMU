/**
 * \file conf_gpio.h
 *
 * \brief SAM L21 GPIO configuration and utility functions
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

#ifndef __CONF_GPIO_H_INCLUDED__
#define __CONF_GPIO_H_INCLUDED__

/* set port pin high */
#define IOPORT_PIN_LEVEL_HIGH		(true)
/* Set port pin low */
#define IOPORT_PIN_LEVEL_LOW		(false)

/* BM7x Reset Pin */
#define BM7X_RESET_PIN				(EXT1_PIN_4)
/* BM7x RX_IND Pin. This RX_IND will indicate the UART communication in advance to BM7x. 
	This pin will be drove by Host MCU and it help to wakeup BM7x */
#define BM7X_RX_IND_PIN				(EXT1_PIN_10)
/* BM7x Mode Pin. 	This MODE pin will decide whether the BM7x device should be in 
	Programming mode or in Application mode*/
#define BM7X_MODE_PIN				(EXT1_PIN_3)

/* BM7x TX_IND Pin. This TX_IND will indicate the UART communication in advance to Host MCU. 
	This is configured as external interrupt pin. This can be used to wakeup host from sleep. */
#define BMXX_TX_IND_PIN					EXT1_PIN_9
#define BMXX_TX_IND_EIC_PIN				PIN_PB04A_EIC_EXTINT4
#define BMXX_TX_IND_EIC_MUX				MUX_PB04A_EIC_EXTINT4
#define BMXX_TX_IND_EIC_LINE			4

void platform_host_wake_interrupt_handler(void);
static inline void bm_host_wakeup_config(void);
static inline void bm_host_wakeup_handler(void);

/* BM7x Host Wakeup Initialization */
static inline void bm_host_wakeup_config(void)
{
	struct extint_chan_conf eint_chan_conf;
	extint_chan_get_config_defaults(&eint_chan_conf);

	eint_chan_conf.gpio_pin           = BMXX_TX_IND_EIC_PIN;
	eint_chan_conf.gpio_pin_pull      = EXTINT_PULL_UP;
	eint_chan_conf.gpio_pin_mux       = BMXX_TX_IND_EIC_MUX;
	eint_chan_conf.detection_criteria = EXTINT_DETECT_FALLING;
	eint_chan_conf.filter_input_signal = true;
	eint_chan_conf.enable_async_edge_detection = true;
	
	extint_chan_set_config(BMXX_TX_IND_EIC_LINE, &eint_chan_conf);
	
	extint_register_callback(bm_host_wakeup_handler,
	BMXX_TX_IND_EIC_LINE, EXTINT_CALLBACK_TYPE_DETECT);
	
	extint_chan_enable_callback(BMXX_TX_IND_EIC_LINE,
	EXTINT_CALLBACK_TYPE_DETECT);
	
}

static inline void bm_host_wakeup_handler(void)
{
	platform_host_wake_interrupt_handler();
}

/* Check host wakeup pin level */
static inline bool host_wakeup_pin_level(void)
{
	return (port_pin_get_input_level(BMXX_TX_IND_PIN));
}

/* Check BM7x wakeup pin level */
static inline bool bm7x_wakeup_pin_level(void)
{
	return (port_pin_get_output_level(BM7X_RX_IND_PIN));
}

/* Set BM7x wakeup pin to low */
static inline void bm7x_wakeup_pin_set_low(void)
{
	port_pin_set_output_level(BM7X_RX_IND_PIN, IOPORT_PIN_LEVEL_LOW);
}

/* Set BM7x wakeup pin to high */
static inline void bm7x_wakeup_pin_set_high(void)
{
	port_pin_set_output_level(BM7X_RX_IND_PIN, IOPORT_PIN_LEVEL_HIGH);
}


/* Check BM7x reset pin level */
static inline bool bm7x_reset_pin_level(void)
{
	return (port_pin_get_output_level(BM7X_RESET_PIN));
}

/* Set BM7x reset pin to low */
static inline void bm7x_reset_pin_set_low(void)
{
	port_pin_set_output_level(BM7X_RESET_PIN, IOPORT_PIN_LEVEL_LOW);
}

/* Set BM7x reset pin to high */
static inline void bm7x_reset_pin_set_high(void)
{
	port_pin_set_output_level(BM7X_RESET_PIN, IOPORT_PIN_LEVEL_HIGH);
}

/* Check BM7x mode pin level */
static inline bool bm7x_mode_pin_level(void)
{
	return (port_pin_get_output_level(BM7X_MODE_PIN));
}

/* Set BM7x mode pin to low */
static inline void bm7x_mode_pin_set_low(void)
{
	port_pin_set_output_level(BM7X_MODE_PIN, IOPORT_PIN_LEVEL_LOW);
}

/* Set BM7x mode pin to high */
static inline void bm7x_mode_pin_set_high(void)
{
	port_pin_set_output_level(BM7X_MODE_PIN, IOPORT_PIN_LEVEL_HIGH);
}


/* Configure the BTLC1000 control(chip_enable, wakeup) pins */
static inline void ble_configure_control_pin(void)
{
	struct port_config pin_conf;
	
	/* initialize the delay before use */
	delay_init();
	
	/* get the default values for port pin configuration */
	port_get_config_defaults(&pin_conf);

	/* Configure control pins as output */
	pin_conf.direction  = PORT_PIN_DIR_OUTPUT;
	pin_conf.input_pull = PORT_PIN_PULL_DOWN;
	
	port_pin_set_config(BM7X_RESET_PIN, &pin_conf);
	port_pin_set_config(BM7X_MODE_PIN, &pin_conf);
	port_pin_set_config(BM7X_RX_IND_PIN, &pin_conf);
}

/* Configure the BM7x reset pin as input */
static inline void ble_configure_reset_pin_dir_input(void)
{
	struct port_config pin_conf;
	
	/* initialize the delay before use */
	delay_init();
	
	/* get the default values for port pin configuration */
	port_get_config_defaults(&pin_conf);

	/* Configure control pins as output */
	pin_conf.direction  = PORT_PIN_DIR_INPUT;
	pin_conf.input_pull = PORT_PIN_PULL_UP;
	
	port_pin_set_config(BM7X_RESET_PIN, &pin_conf);
}

/* Configure the BM7x reset pin as output */
static inline void ble_configure_reset_pin_dir_output(void)
{
	struct port_config pin_conf;
	
	/* initialize the delay before use */
	delay_init();
	
	/* get the default values for port pin configuration */
	port_get_config_defaults(&pin_conf);

	/* Configure control pins as output */
	pin_conf.direction  = PORT_PIN_DIR_OUTPUT;
	pin_conf.input_pull = PORT_PIN_PULL_DOWN;
	
	port_pin_set_config(BM7X_RESET_PIN, &pin_conf);
}

#endif /* __CONF_GPIO_H_INCLUDED__ */