/*
 * The MIT License (MIT)
 *
 * Copyright (c) 2019 Ha Thach (tinyusb.org)
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 *
 */
/*
 *  Copyright (C) 2021 Texas Instruments Incorporated
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

#include <usb/cdn/include/usb_init.h>
#include "tusb.h"

#include "FreeRTOS.h"
#include "TaskP.h"

#include "ti_drivers_config.h"
#include "ti_drivers_open_close.h"
#include "ti_board_open_close.h"

#define TUD_TASK_PRI  (TaskP_PRIORITY_HIGHEST-2)
#define TUD_TASK_SIZE (16384U)
uint8_t gTudTaskStack[TUD_TASK_SIZE] __attribute__((aligned(32)));
TaskP_Object gTudTaskObj;
TaskP_Params gTudTaskParams;

#define CDC_TASK_PRI  (TaskP_PRIORITY_HIGHEST-3)
#define CDC_TASK_SIZE (16384U)
uint8_t gCdcTaskStack[CDC_TASK_SIZE] __attribute__((aligned(32)));
TaskP_Object gCdcTaskObj;
TaskP_Params gCdcTaskParams;

void tud_task_loop(void *args);
void cdc_task_loop(void *args);

int cdc_echo_main(void)
{
    int32_t status;

    Drivers_open();
    Board_driversOpen();

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
