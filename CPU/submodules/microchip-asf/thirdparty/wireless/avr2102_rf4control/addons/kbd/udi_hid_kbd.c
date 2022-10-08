/**
 * \file
 *
 * \brief USB Device Human Interface Device (HID) keyboard interface.
 *
 * Copyright (c) 2014-2018 Microchip Technology Inc. and its subsidiaries.
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

#include "conf_usb.h"
#include "usb_protocol.h"
#include "udd.h"
#include "udc.h"
#include "udi_hid.h"
#include "udi_hid_kbd.h"
#include <string.h>

/**
 * \ingroup udi_hid_keyboard_group
 * \defgroup udi_hid_keyboard_group_udc Interface with USB Device Core (UDC)
 *
 * Structures and functions required by UDC.
 *
 * @{
 */

bool udi_hid_kbd_enable(void);
void udi_hid_kbd_disable(void);
bool udi_hid_kbd_setup(void);
uint8_t udi_hid_kbd_getsetting(void);

/* ! Global structure which contains standard UDI interface for UDC */
UDC_DESC_STORAGE udi_api_t udi_api_hid_kbd = {
	.enable = (bool (*)(void))udi_hid_kbd_enable,
	.disable = (void (*)(void))udi_hid_kbd_disable,
	.setup = (bool (*)(void))udi_hid_kbd_setup,
	.getsetting = (uint8_t (*)(void))udi_hid_kbd_getsetting,
	.sof_notify = NULL,
};
/* @} */

/**
 * \ingroup udi_hid_keyboard_group
 * \defgroup udi_hid_keyboard_group_internal Implementation of UDI HID keyboard
 *
 * Class internal implementation
 * @{
 */

/**
 * \name Internal defines and variables to manage HID keyboard
 */
/* @{ */

/* ! Size of report for standard HID keyboard */

#define UDI_HID_KBD_REPORT_SIZE  9
#define UDI_HID_MKBD_REPORT_SIZE  3

/* ! To store current rate of HID keyboard */
static uint8_t udi_hid_kbd_rate;
/* ! To store current protocol of HID keyboard */
static uint8_t udi_hid_kbd_protocol;
/* ! To store report feedback from USB host */
static uint8_t udi_hid_kbd_report_set;
/* ! To signal if a valid report is ready to send */
static bool udi_hid_kbd_b_report_valid;
/* ! Report ready to send */
static uint8_t udi_hid_kbd_report[UDI_HID_KBD_REPORT_SIZE];
/* ! Signal if a report transfer is on going */
static uint8_t report_size;
static bool udi_hid_kbd_b_report_trans_ongoing;
/* ! Buffer used to send report */
COMPILER_WORD_ALIGNED
static uint8_t
		udi_hid_kbd_report_trans[UDI_HID_KBD_REPORT_SIZE];

/* @} */

/* ! HID report descriptor for standard HID keyboard */
UDC_DESC_STORAGE udi_hid_kbd_report_desc_t udi_hid_kbd_report_desc = {
	{                       0x05, 0x01,     /* Usage Page (Generic Desktop)
		                                 *     */
				0x09, 0x06,     /* Usage (Keyboard)
		                                 *                 */
				0xA1, 0x01,     /* Collection (Application)
		                                 *         */
				0x85, 0x01,
				0x05, 0x07,     /* Usage Page (Keyboard)
		                                 *            */
				0x19, 224,      /* Usage Minimum (224)
		                                 *              */
				0x29, 231,      /* Usage Maximum (231)
		                                 *              */
				0x15, 0x00,     /* Logical Minimum (0)
		                                 *              */
				0x25, 0x01,     /* Logical Maximum (1)
		                                 *              */
				0x75, 0x01,     /* Report Size (1)
		                                 *                  */
				0x95, 0x08,     /* Report Count (8)
		                                 *                 */
				0x81, 0x02,     /* Input (Data, Variable,
		                                 * Absolute)  */
				0x81, 0x01,     /* Input (Constant)
		                                 *                 */
				0x19, 0x00,     /* Usage Minimum (0)
		                                 *                */
				0x29, 101,      /* Usage Maximum (101)
		                                 *              */
				0x15, 0x00,     /* Logical Minimum (0)
		                                 *              */
				0x25, 101,      /* Logical Maximum (101)
		                                 *            */
				0x75, 0x08,     /* Report Size (8)
		                                 *                  */
				0x95, 0x06,     /* Report Count (6)
		                                 *                 */
				0x81, 0x00,     /* Input (Data, Array)
		                                 *              */
				0x05, 0x08,     /* Usage Page (LED)
		                                 *                 */
				0x19, 0x01,     /* Usage Minimum (1)
		                                 *                */
				0x29, 0x05,     /* Usage Maximum (5)
		                                 *                */
				0x15, 0x00,     /* Logical Minimum (0)
		                                 *              */
				0x25, 0x01,     /* Logical Maximum (1)
		                                 *              */
				0x75, 0x01,     /* Report Size (1)
		                                 *                  */
				0x95, 0x05,     /* Report Count (5)
		                                 *                 */
				0x91, 0x02,     /* Output (Data, Variable,
		                                 * Absolute) */
				0x95, 0x03,     /* Report Count (3)
		                                 *                 */
				0x91, 0x01,     /* Output (Constant)
		                                 *                */
				0xC0,   /* End Collection                    */
				0x05, 0x0c,                     /* USAGE_PAGE
		                                                * (Consumer
		                                                * Devices) */
				0x09, 0x01,                     /* USAGE
		                                                 * (Consumer
		                                                 * Control) */
				0xa1, 0x01,
				0x85, 0x02,
				0x09, 0xe2,                     /*   USAGE
		                                                 * (Mute) 0x01
		                                                 **/
				0x09, 0xe9,                     /*   USAGE
		                                                 * (Volume Up)
		                                                 * 0x02 */
				0x09, 0xea,                     /*   USAGE
		                                                 * (Volume Down)
		                                                 * 0x03 */
				0x0a, 0x83, 0x01,               /*   USAGE
		                                                 * (Play/Pause)
		                                                 * 0x04 */
				0x09, 0xb7,                     /*   USAGE
		                                                 * (Stop) 0x05
		                                                 **/
				0x09, 0xb6,                     /*  USAGE (Scan
		                                                 * Previous
		                                                 * Track) 0x06
		                                                 **/
				0x09, 0xb5,                     /*   USAGE (Scan
		                                                 * Next Track)
		                                                 * 0x07 */
				0x09, 0xb0,                     /*   USAGE
		                                                 * (play) 0x08
		                                                 **/
				0x09, 0xb1,                     /*   USAGE
		                                                 * (pause) 0x08
		                                                 **/
				0x09, 0xbc,                     /*   USAGE
		                                                 * (repeat) 0x09
		                                                 **/
				0x0a, 0x21, 0x02,               /*   USAGE (www
		                                                 * search) 0x0a
		                                                 **/
				0x0a, 0x23, 0x02,               /*   USAGE (www
		                                                * home) 0x0b */
				0x0a, 0x2a, 0x02,               /*   USAGE (www
		                                                 * favorites)
		                                                 * 0x0c */
				0x0a, 0x27, 0x02,               /*   USAGE (www
		                                                 * refresh) 0x0d
		                                                 *     0x0a,
		                                                 * 0x26, 0x02,
		                                                 **/
				0x0a, 0x25, 0x02,               /*   USAGE (www
		                                                 * forward) 0x0f
		                                                 **/
				0x0a, 0x24, 0x02,               /*   USAGE (www
		                                                * back) 0x10 */
				0x15, 0x00,                     /*
		                                                 *
		                                                 *
		                                                 *
		                                                 *LOGICAL_MINIMUM
		                                                 * (0) */
				0x25, 0x01,                     /*
		                                                 *
		                                                 *
		                                                 *
		                                                 *LOGICAL_MAXIMUM
		                                                 * (1) */
				0x75, 0x01,                     /*   REPORT_SIZE
		                                                 * (1) */
				0x95, 0x10,                     /*
		                                                 *
		                                                 *  REPORT_COUNT
		                                                 * (16) */
				0x81, 0x62,                     /*   INPUT
		                                                 *
		                                                 *(Data,Var,Abs,NPrf,Null)
		                                                 **/
				0xc0}
};

/**
 * \name Internal routines
 */
/* @{ */

/**
 * \brief Changes keyboard report states (like LEDs)
 *
 * \param rate       New rate value
 *
 */
static bool udi_hid_kbd_setreport(void);

/**
 * \brief Send the report
 *
 * \return \c 1 if send on going, \c 0 if delay.
 */
static bool udi_hid_kbd_send_report(void);

/**
 * \brief Callback called when the report is sent
 *
 * \param status     UDD_EP_TRANSFER_OK, if transfer is completed
 * \param status     UDD_EP_TRANSFER_ABORT, if transfer is aborted
 * \param nb_sent    number of data transfered
 *
 * \return \c 1 if function was successfully done, otherwise \c 0.
 */
static void udi_hid_kbd_report_sent(udd_ep_status_t status, iram_size_t nb_sent,
		udd_ep_id_t ep);

/**
 * \brief Callback called to update report from USB host
 * udi_hid_kbd_report_set is updated before callback execution
 */
static void udi_hid_kbd_setreport_valid(void);

/* @} */

/* -------------------------------------------- */
/* ------ Interface for UDI HID level */

bool udi_hid_kbd_enable(void)
{
	/* Initialize internal values */
	udi_hid_kbd_rate = 0;
	udi_hid_kbd_protocol = 0;
	udi_hid_kbd_b_report_trans_ongoing = false;
	memset(udi_hid_kbd_report, 0, UDI_HID_KBD_REPORT_SIZE);
	udi_hid_kbd_b_report_valid = false;
	return UDI_HID_KBD_ENABLE_EXT();
}

void udi_hid_kbd_disable(void)
{
	UDI_HID_KBD_DISABLE_EXT();
}

bool udi_hid_kbd_setup(void)
{
	return udi_hid_setup(&udi_hid_kbd_rate,
			&udi_hid_kbd_protocol,
			(uint8_t *)&udi_hid_kbd_report_desc,
			udi_hid_kbd_setreport);
}

uint8_t udi_hid_kbd_getsetting(void)
{
	return 0;
}

static bool udi_hid_kbd_setreport(void)
{
	if ((USB_HID_REPORT_TYPE_OUTPUT == (udd_g_ctrlreq.req.wValue >> 8)) &&
			(0 == (0xFF & udd_g_ctrlreq.req.wValue)) &&
			(1 == udd_g_ctrlreq.req.wLength)) {
		/* Report OUT type on report ID 0 from USB Host */
		udd_g_ctrlreq.payload = &udi_hid_kbd_report_set;
		udd_g_ctrlreq.callback = udi_hid_kbd_setreport_valid;
		udd_g_ctrlreq.payload_size = 1;
		return true;
	}

	return false;
}

/* -------------------------------------------- */
/* ------ Interface for application */

bool udi_hid_mkbd_modifier_up(uint16_t modifier_id)
{
	irqflags_t flags = cpu_irq_save();

	/* Fill report */
	udi_hid_kbd_report[0] = 0x02;
	udi_hid_kbd_report[1] &= ~(uint8_t)modifier_id;
	udi_hid_kbd_report[2] &= ~(uint8_t)(modifier_id >> 8);
	udi_hid_kbd_b_report_valid = true;
	report_size = UDI_HID_MKBD_REPORT_SIZE;
	/* Send report */
	udi_hid_kbd_send_report();

	cpu_irq_restore(flags);
	return true;
}

bool udi_hid_kbd_modifier_up(uint8_t modifier_id)
{
	irqflags_t flags = cpu_irq_save();

	/* Fill report */

	udi_hid_kbd_report[0] = 0x01;
	udi_hid_kbd_report[1] &= ~(unsigned)modifier_id;
	udi_hid_kbd_b_report_valid = true;
	report_size = UDI_HID_KBD_REPORT_SIZE;
	/* Send report */
	udi_hid_kbd_send_report();

	cpu_irq_restore(flags);
	return true;
}

bool udi_hid_kbd_modifier_down(uint8_t modifier_id)
{
	irqflags_t flags = cpu_irq_save();

	/* Fill report */

	udi_hid_kbd_report[0] = 0x01;
	udi_hid_kbd_report[1] |= modifier_id;
	udi_hid_kbd_b_report_valid = true;
	report_size = UDI_HID_KBD_REPORT_SIZE;
	/* Send report */
	udi_hid_kbd_send_report();

	cpu_irq_restore(flags);
	return true;
}

bool udi_hid_mkbd_modifier_down(uint16_t modifier_id)
{
	irqflags_t flags = cpu_irq_save();

	/* Fill report */
	udi_hid_kbd_report[0] = 0x02;
	udi_hid_kbd_report[1] = (uint8_t)modifier_id;
	udi_hid_kbd_report[2] = (uint8_t)(modifier_id >> 8);
	report_size = UDI_HID_MKBD_REPORT_SIZE;
	udi_hid_kbd_b_report_valid = true;

	/* Send report */
	udi_hid_kbd_send_report();

	cpu_irq_restore(flags);
	return true;
}

bool udi_hid_kbd_up(uint8_t key_id)
{
	uint8_t i;

	irqflags_t flags = cpu_irq_save();
	udi_hid_kbd_report[0] = 0x01;
	/* Fill report */
	for (i = 3; i < UDI_HID_KBD_REPORT_SIZE; i++) {
		if (0 == udi_hid_kbd_report[i]) {
			/* Already removed */
			cpu_irq_restore(flags);
			return true;
		}

		if (key_id == udi_hid_kbd_report[i]) {
			break;
		}
	}
	if (UDI_HID_KBD_REPORT_SIZE == i) {
		/* Already removed */
		cpu_irq_restore(flags);
		return true;
	}

	/* Remove key and shift */
	while (i < (UDI_HID_KBD_REPORT_SIZE - 1)) {
		udi_hid_kbd_report[i] = udi_hid_kbd_report[i + 1];
		i++;
	}
	udi_hid_kbd_report[UDI_HID_KBD_REPORT_SIZE - 1] = 0x00;
	report_size = UDI_HID_KBD_REPORT_SIZE;
	udi_hid_kbd_b_report_valid = true;

	/* Send report */
	udi_hid_kbd_send_report();

	cpu_irq_restore(flags);
	return true;
}

bool udi_hid_kbd_down(uint8_t key_id)
{
	uint8_t i;

	irqflags_t flags = cpu_irq_save();
	udi_hid_kbd_report[0] = 0x01;
	/* Fill report */
	for (i = 3; i < UDI_HID_KBD_REPORT_SIZE; i++) {
		if (0 == udi_hid_kbd_report[i]) {
			break;
		}

		if (key_id == udi_hid_kbd_report[i]) {
			/* Already in array */
			cpu_irq_restore(flags);
			return true;
		}
	}

	if (UDI_HID_KBD_REPORT_SIZE == i) {
		/* Array full */

		/* TODO manage more than UDI_HID_KBD_REPORT_SIZE key pressed in
		 * same time */
		cpu_irq_restore(flags);
		return false;
	}

	/* Add key at the end of array */
	udi_hid_kbd_report[i] = key_id;
	report_size = UDI_HID_KBD_REPORT_SIZE;
	udi_hid_kbd_b_report_valid = true;

	/* Send report */
	udi_hid_kbd_send_report();

	/* Enable IT */
	cpu_irq_restore(flags);
	return true;
}

/* -------------------------------------------- */
/* ------ Internal routines */

static bool udi_hid_kbd_send_report(void)
{
	if (udi_hid_kbd_b_report_trans_ongoing) {
		return false;
	}

	memcpy(udi_hid_kbd_report_trans, udi_hid_kbd_report,
			report_size);
	udi_hid_kbd_b_report_valid = false;
	udi_hid_kbd_b_report_trans_ongoing
		= udd_ep_run(     UDI_HID_KBD_EP_IN,
			false,
			udi_hid_kbd_report_trans,
			report_size,
			udi_hid_kbd_report_sent);
	return udi_hid_kbd_b_report_trans_ongoing;
}

static void udi_hid_kbd_report_sent(udd_ep_status_t status, iram_size_t nb_sent,
		udd_ep_id_t ep)
{
	UNUSED(status);
	UNUSED(nb_sent);
	UNUSED(ep);
	udi_hid_kbd_b_report_trans_ongoing = false;
	if (udi_hid_kbd_b_report_valid) {
		udi_hid_kbd_send_report();
	}
}

static void udi_hid_kbd_setreport_valid(void)
{
	UDI_HID_KBD_CHANGE_LED(udi_hid_kbd_report_set);
}

/* @} */
