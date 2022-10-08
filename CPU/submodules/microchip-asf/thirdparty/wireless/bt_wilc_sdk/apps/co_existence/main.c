/**
 *
 * \file
 *
 * \brief main.c file.
 *
 * Copyright (c) 2018-2020 Atmel Corporation. All rights reserved.
 *
 * \asf_license_start
 *
 * \page License
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. The name of Atmel may not be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY ATMEL "AS IS" AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT ARE
 * EXPRESSLY AND SPECIFICALLY DISCLAIMED. IN NO EVENT SHALL ATMEL BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * \asf_license_stop
 *
 */
#define __BTSTACK_FILE__ "main.c"

#include "asf.h"
#include "console_serial.h"
#include "conf_board.h"
#include "conf_clock.h"

// BTstack
#include "btstack_chipset_atwilc3000.h"
#include "btstack_debug.h"
#include "btstack_memory.h"
#include "btstack_run_loop.h"
#include "btstack_run_loop_freertos.h" 
#include "classic/btstack_link_key_db.h"
#include "hci.h"
#include "hci_dump.h"

#ifdef __cplusplus
extern "C" {
#endif

#include "osprintf.h"
#include "driver/include/m2m_wifi.h"
#include "sta.h"
#include "ap.h"

#define TASK_STA_STACK_SIZE		(4096 / sizeof(portSTACK_TYPE))
#define TASK_STA_PRIORITY			(tskIDLE_PRIORITY + 1)
#define TASK_AP_STACK_SIZE		(1024 / sizeof(portSTACK_TYPE))
#define TASK_AP_PRIORITY			(tskIDLE_PRIORITY + 1)

extern int btstack_main(int argc, const char * argv[]);

#define TASK_BT_STACK_SIZE				   ((2048*2)/sizeof(portSTACK_TYPE))
#define TASK_BT_STACK_PRIORITY			   (2)//(tskIDLE_PRIORITY)

extern TaskHandle_t btstack_run_loop_task;
extern void vApplicationStackOverflowHook(xTaskHandle *pxTask,
		signed char *pcTaskName);
extern void vApplicationIdleHook(void);
extern void vApplicationTickHook(void);
extern void vApplicationMallocFailedHook(void);
extern void xPortSysTickHandler(void);

TaskHandle_t xHandle = NULL;
TaskHandle_t xHandle1 = NULL;
TaskHandle_t xHandle2 = NULL;

static const btstack_uart_block_t * uart_driver;
static btstack_uart_config_t uart_config;


static hci_transport_config_uart_t transport_config = {
	HCI_TRANSPORT_CONFIG_UART,
	115200, 
	0, 		  
	1,        
	NULL,
};

void SysTick_Handler(void)
{
	xPortSysTickHandler();
}
//#endif

/**
 * \brief Called if stack overflow during execution
 */
extern void vApplicationStackOverflowHook(xTaskHandle *pxTask,
		signed char *pcTaskName)
{
	printf("stack overflow %x %s\r\n", pxTask, (portCHAR *)pcTaskName);
	/* If the parameters have been corrupted then inspect pxCurrentTCB to
	 * identify which task has overflowed its stack.
	 */
	for (;;) {
	}
}

/**
 * \brief This function is called by FreeRTOS idle task
 */
extern void vApplicationIdleHook(void)
{
}

/**
 * \brief This function is called by FreeRTOS each tick
 */
extern void vApplicationTickHook(void)
{
}

extern void vApplicationMallocFailedHook(void)
{
	/* Called if a call to pvPortMalloc() fails because there is insufficient
	free memory available in the FreeRTOS heap.  pvPortMalloc() is called
	internally by FreeRTOS API functions that create tasks, queues, software
	timers, and semaphores.  The size of the FreeRTOS heap is set by the
	configTOTAL_HEAP_SIZE configuration constant in FreeRTOSConfig.h. */

	/* Force an assert. */
	configASSERT( ( volatile void * ) NULL );
}

// [main_console_configure]



static void bt_app_init(int status){

    if (status){
        printf("\r\nFW download fail\n");
        return;
    }

    // init HCI
    const hci_transport_t * transport = hci_transport_h4_instance(uart_driver);
    hci_init(transport, (void*) &transport_config);
    hci_set_chipset(btstack_chipset_atwilc3000_instance());
    hci_set_link_key_db(btstack_link_key_db_memory_instance());
    
     //setup app
    btstack_main(0, NULL);
	
}


static void bt_task(void *arg)
{
	vTaskDelay(10);
	xSemaphoreTake(wifi_bt_if_sem, portMAX_DELAY); /* Semaphore to synchronize BT &WiFi FW download */
	 btstack_run_loop_task = xHandle; 
	// start with BTstack init - especially configure HCI Transport
	btstack_memory_init();

	btstack_run_loop_init(btstack_run_loop_freertos_get_instance());
	
	// setup UART HAL + Run Loop integration
	uart_driver = btstack_uart_block_freertos_instance();    // RTOS

	// extract UART config from transport config, but disable flow control and use default baudrate
	uart_config.baudrate    = HCI_DEFAULT_BAUDRATE;
	uart_config.flowcontrol = 0;
	uart_config.device_name = transport_config.device_name;
	uart_driver->init(&uart_config);	 
	btstack_chipset_atwilc3000_download_firmware(uart_driver, transport_config.baudrate_init, transport_config.flowcontrol, NULL, 0, &bt_app_init);
	btstack_run_loop_execute();
	xSemaphoreGive(wifi_bt_if_sem); /* Semaphore to synchronize BT &WiFi FW download */
}


/**
 *  \brief getting-started Application entry point.
 *
 *  \return Unused (ANSI-C compatibility).
 */
// [main]
int main(void)
{
	/* Initialize the SAM system */
	sysclk_init();
	board_init();

	/* Initialize serial console */
	serial_console_init();

	/* Output boot info */
	printf("\r\nBTstack on SAMV71 Xplained Ultra with ATWILC3000\n");
	printf("\r\nCPU %lu hz, peripheral clock %lu hz\n", sysclk_get_cpu_hz(), sysclk_get_peripheral_hz());
#ifdef USE_XDMAC_FOR_USART
	printf("\r\nUsing XDMA for Bluetooth UART\n");
#else
	printf("\r\nUsing IRQ driver for Bluetooth UART\n");
#endif
	printf("--\n");

	//hci_dump_open(NULL, 2);

	/* Semaphore to synchronize BT &WiFi FW download */
	vSemaphoreCreateBinary(wifi_bt_if_sem);	

 	/* Create main task. */
 	xTaskCreate(sta_task, (const void *)"WiFiSTA", TASK_STA_STACK_SIZE, 0,	TASK_STA_PRIORITY+1, &xHandle1);
 		
 	/* Create main task. */
	xTaskCreate(ap_task, (const void *)"WiFiAP", TASK_AP_STACK_SIZE, 0,	TASK_AP_PRIORITY, &xHandle2);
 	
	/* Create task to monitor processor activity */
	if (xTaskCreate(bt_task, "bt_task", TASK_BT_STACK_SIZE, NULL, TASK_BT_STACK_PRIORITY, &xHandle) != pdPASS)
	{
		printf("\r\nFailed to create bt task\n");
	}
	
	/* Start the scheduler. */
	vTaskStartScheduler();
	
    // compiler happy
	while(1);
}
#ifdef __cplusplus
}
#endif
