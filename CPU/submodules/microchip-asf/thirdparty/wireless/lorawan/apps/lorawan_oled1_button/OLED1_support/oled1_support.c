/**
* \file  oled1_support.c
*
* \brief LORAWAN Getting Started  [Button OLED1] Demo Application
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
#include "asf.h"
#include "oled1_support.h"

#define LINES 8

// Regional Band Select  Menu options
PROGMEM_DECLARE(char const, bandsel_menu_title[]) = "Select Region";
PROGMEM_DECLARE(char const, bandsel_menu_1[]) = "  EU868";
PROGMEM_DECLARE(char const, bandsel_menu_2[]) = "  NA915";
PROGMEM_DECLARE(char const, bandsel_menu_3[]) = "  AU915";
PROGMEM_DECLARE(char const, bandsel_menu_4[]) = "  AS923";
PROGMEM_DECLARE(char const, bandsel_menu_5[]) = "  JPN923";
PROGMEM_DECLARE(char const, bandsel_menu_6[]) = "  KR920";
PROGMEM_DECLARE(char const, bandsel_menu_7[]) = "  IND865";
PROGMEM_DECLARE(char const, bandsel_menu_8[]) = "  Clear PDS";
PROGMEM_DECLARE(char const, bandsel_menu_9[]) = "  Reset Board";

PROGMEM_STRING_T bandsel_menu_strings[] = {
	bandsel_menu_1,
	bandsel_menu_2,
	bandsel_menu_3,
	bandsel_menu_4,
	bandsel_menu_5,
	bandsel_menu_6,
	bandsel_menu_7,
	bandsel_menu_8,
	bandsel_menu_9,
};

struct gfx_mono_menu Bsel_menu = {
	// Title
	bandsel_menu_title,
	// Array with menu strings
	bandsel_menu_strings,
	// Number of menu elements
	9,
	// Initial selection
	0
};

void oled1_print_array(uint8_t *array, uint8_t length, uint8_t x, uint8_t y)
{
	char c[3] ;
	uint8_t pos = x ;
	for (uint8_t i = 0; i < length; i++)
	{
		sprintf(c, "%02X", array[i]) ;
		gfx_mono_draw_char((char)c[0], pos, y, &sysfont) ;
		pos += 6 ;	//+6 per char displayed
		gfx_mono_draw_char((char)c[1], pos, y, &sysfont) ;
		pos += 6 ;
	}
	c[2] = '\0' ;
}

void button_splash(void)
{

	// Clear screen
	gfx_mono_draw_filled_rect(0, 0, 128, 32, GFX_PIXEL_CLR);

	gfx_mono_draw_filled_circle(10, 16, 4, GFX_PIXEL_SET, GFX_WHOLE);
	gfx_mono_draw_filled_circle(64, 16, 4, GFX_PIXEL_SET, GFX_WHOLE);
	gfx_mono_draw_filled_circle(120, 16, 4, GFX_PIXEL_SET, GFX_WHOLE);


	gfx_mono_draw_string("Up", 116, 1, &sysfont);
	gfx_mono_draw_string("Down", 2, 1, &sysfont);
	gfx_mono_draw_string("Select", 50, 24, &sysfont);


}



