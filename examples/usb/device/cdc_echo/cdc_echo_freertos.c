/*
 *  Copyright (C) 2021-2023 Texas Instruments Incorporated
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *    Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 *    Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the
 *    distribution.
 *
 *    Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 *  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 *  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 *  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/* Adapted by TI for running on its platform and SDK */

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <ctype.h>

#if defined(SOC_AM64X) || defined (SOC_AM243X)
#include <usb/cdn/include/usb_init.h>
#endif

#include "tusb.h"

#include "FreeRTOS.h"
#include <kernel/dpl/TaskP.h>

#include "ti_drivers_config.h"
#include "ti_drivers_open_close.h"
#include "ti_board_open_close.h"

#if defined(SOC_AM64X) || defined (SOC_AM243X)
#ifdef TINYUSB_INTEGRATION

#define DSR_TASK_PRI  (TaskP_PRIORITY_HIGHEST-2)
#define DSR_TASK_SIZE (1024U)
uint8_t gDsrTaskStack[DSR_TASK_SIZE] __attribute__((aligned(32)));
TaskP_Object gDsrTaskObj;
TaskP_Params gDsrTaskParams;

#endif /* TINYUSB_INTEGRATION */


#define TUD_TASK_PRI  (TaskP_PRIORITY_HIGHEST-3)
#define TUD_TASK_SIZE (1024U)
uint8_t gTudTaskStack[TUD_TASK_SIZE] __attribute__((aligned(32)));
TaskP_Object gTudTaskObj;
TaskP_Params gTudTaskParams;


#define CDC_TASK_PRI  (TaskP_PRIORITY_HIGHEST-4)
#define CDC_TASK_SIZE (1024U)
uint8_t gCdcTaskStack[CDC_TASK_SIZE] __attribute__((aligned(32)));
TaskP_Object gCdcTaskObj;
TaskP_Params gCdcTaskParams;

#ifdef TINYUSB_INTEGRATION
void dsr_task_loop(void *args);
#endif /* TINYUSB_INTEGRATION */

void tud_task_loop(void *args);
void cdc_task_loop(void *args);

#else /* AM261X */

#define USB_TASK_PRI  (TaskP_PRIORITY_HIGHEST-2)
#define USB_TASK_SIZE (1024U)
uint8_t gUsbTaskStack[USB_TASK_SIZE] __attribute__((aligned(32)));
TaskP_Object gUsbTaskObj;
TaskP_Params gUsbTaskParams;

void usb_task_loop(void *args);

#endif

int cdc_echo_main(void)
{
    int32_t status;

    Drivers_open();
    Board_driversOpen();
	
#if defined(SOC_AM64X) || defined (SOC_AM243X)

#ifdef TINYUSB_INTEGRATION

    /* Cadence DSR task is to handle the USB device events */
    TaskP_Params_init(&gDsrTaskParams);
    gDsrTaskParams.name = "dsr_task";                /**< Pointer to task name */
    gDsrTaskParams.stackSize = DSR_TASK_SIZE;        /**< Size of stack in units of bytes */
    gDsrTaskParams.stack = gDsrTaskStack;            /**< Pointer to stack memory, MUST be aligned based on CPU architecture, typically atleast 32b on 32b systems */
    gDsrTaskParams.priority = DSR_TASK_PRI;          /**< Task priority, MUST be between \ref TaskP_PRIORITY_LOWEST and TaskP_PRIORITY_HIGHEST */
    gDsrTaskParams.args = NULL;                      /**< User arguments that are passed back as parater to task main */
    gDsrTaskParams.taskMain = dsr_task_loop;         /**< Entry point function to the task */
    /* create the task */
    status = TaskP_construct(&gDsrTaskObj, &gDsrTaskParams);
    DebugP_assert(status == SystemP_SUCCESS);
#endif
	
	/* Enable debug for cdn module */ 
	DbgMsgEnableModule(USBSSP_DBG_CUSBD);
	DbgMsgEnableModule(USBSSP_DBG_CUSBD_ISR);
	DbgMsgSetLvl(100); 
	
    /* TUD task is to handle the USB device events */
    TaskP_Params_init(&gTudTaskParams);
    gTudTaskParams.name = "tud_task";                /**< Pointer to task name */
    gTudTaskParams.stackSize = TUD_TASK_SIZE;        /**< Size of stack in units of bytes */
    gTudTaskParams.stack = gTudTaskStack;            /**< Pointer to stack memory, MUST be aligned based on CPU architecture, typically atleast 32b on 32b systems */
    gTudTaskParams.priority = TUD_TASK_PRI;          /**< Task priority, MUST be between \ref TaskP_PRIORITY_LOWEST and TaskP_PRIORITY_HIGHEST */
    gTudTaskParams.args = NULL;                      /**< User arguments that are passed back as parater to task main */
    gTudTaskParams.taskMain = tud_task_loop;         /**< Entry point function to the task */
    /* create the task */
    status = TaskP_construct(&gTudTaskObj, &gTudTaskParams);
    DebugP_assert(status == SystemP_SUCCESS);

    /* CDC task is to handle the CDC class events */
    TaskP_Params_init(&gCdcTaskParams);
    gCdcTaskParams.name = "cdc_task";                /**< Pointer to task name */
    gCdcTaskParams.stackSize = CDC_TASK_SIZE;        /**< Size of stack in units of bytes */
    gCdcTaskParams.stack = gCdcTaskStack;            /**< Pointer to stack memory, MUST be aligned based on CPU architecture, typically atleast 32b on 32b systems */
    gCdcTaskParams.priority = CDC_TASK_PRI;          /**< Task priority, MUST be between \ref TaskP_PRIORITY_LOWEST and TaskP_PRIORITY_HIGHEST */
    gCdcTaskParams.args = NULL;                      /**< User arguments that are passed back as parater to task main */
    gCdcTaskParams.taskMain = cdc_task_loop;         /**< Entry point function to the task */
    /* create the task */
    status = TaskP_construct(&gCdcTaskObj, &gCdcTaskParams);
    DebugP_assert(status == SystemP_SUCCESS);

#else /* AM261X */

    TaskP_Params_init(&gUsbTaskParams);
    gUsbTaskParams.name = "usb_task";                /**< Pointer to task name */
    gUsbTaskParams.stackSize = USB_TASK_SIZE;        /**< Size of stack in units of bytes */
    gUsbTaskParams.stack = gUsbTaskStack;            /**< Pointer to stack memory, MUST be aligned based on CPU architecture, typically atleast 32b on 32b systems */
    gUsbTaskParams.priority = USB_TASK_PRI;          /**< Task priority, MUST be between \ref TaskP_PRIORITY_LOWEST and TaskP_PRIORITY_HIGHEST */
    gUsbTaskParams.args = NULL;                      /**< User arguments that are passed back as parater to task main */
    gUsbTaskParams.taskMain = usb_task_loop;         /**< Entry point function to the task */
    /* create the task */
    status = TaskP_construct(&gUsbTaskObj, &gUsbTaskParams);
    DebugP_assert(status == SystemP_SUCCESS);

#endif
    return 0;
}

/* echo to either Serial0 or Serial1
   with Serial0 as all lower case, Serial1 as all upper case
 */
static void echo_serial_port(uint8_t itf, uint8_t buf[], uint32_t count)
{
    for(uint32_t i=0; i<count; i++)
    {
        if (itf == 0)
        {
            /* echo back 1st port as lower case */
            if (isupper(buf[i])) buf[i] += 'a' - 'A';
        }
        else
        {
            /* echo back additional ports as upper case */
            if (islower(buf[i])) buf[i] -= 'a' - 'A';
        }

        tud_cdc_n_write_char(itf, buf[i]);

        if ( buf[i] == '\r' ) tud_cdc_n_write_char(itf, '\n');
    }
    tud_cdc_n_write_flush(itf);
}

static void cdc_task(void)
{
    uint8_t itf;

    for (itf = 0; itf < CFG_TUD_CDC; itf++)
    {
        /* connected() check for DTR bit
           Most but not all terminal client set this when making connection
         */
        /* if ( tud_cdc_n_connected(itf) ) */
        {
            if ( tud_cdc_n_available(itf) )
            {

                uint8_t buf[64];

                uint32_t count = tud_cdc_n_read(itf, buf, sizeof(buf));

                /* echo back to both serial ports */
                echo_serial_port(0, buf, count);
                echo_serial_port(1, buf, count);
            }
        }
    }
}

#if defined(SOC_AM64X) || defined (SOC_AM243X)

void dsr_task_loop(void *args)
{
    while (1)
    {
	    cusbd_dsr();
    }
}
void tud_task_loop(void *args)
{
    while (1)
    {
        tud_task();
    }
}

void cdc_task_loop(void *args)
{
    while (1)
    {
        cdc_task();
    }
}

#else /* AM261X */

void usb_task_loop(void *args)
{
    while (1)
    {
        USB_dwcTask(); /* Synopsis DWC task */

        tud_task();

        cdc_task();
    }
}
#endif