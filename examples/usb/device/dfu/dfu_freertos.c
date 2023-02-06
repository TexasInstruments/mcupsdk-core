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

#define UART_TASK_PRI  (TaskP_PRIORITY_HIGHEST-4)
#define UART_STACK_SIZE (1024U)
uint8_t gUartTaskStack[UART_STACK_SIZE] __attribute__((aligned(32)));
TaskP_Object gUartTaskObj;
TaskP_Params gUartTaskParams;


#ifdef TINYUSB_INTEGRATION
void dsr_task_loop(void *args);
#endif /* TINYUSB_INTEGRATION */

void tud_task_loop(void *args);

void uart_send_img(void *args); 

#define MANIFEST_DONE     (0x1) 
#define MANIFEST_PENDING  (0x00) 

/* max filesize is 4KB */ 
#define MAX_FILE_SIZE     (4096) 
uint8_t gFileBuf[MAX_FILE_SIZE] ;

uint8_t *FileBufPtr = gFileBuf ; 

/* global variable to track received File size */ 
uint32_t recvFileSize = 0 ; 

/* flag to indicate manifest done or not */ 
uint8_t gManifestDone = MANIFEST_PENDING ; 

const char* upload_image[2]=
{
  "Hello world from TinyUSB DFU! - Partition 0",
  "Hello world from TinyUSB DFU! - Partition 1"
};

int dfu_main(void)
{
    int32_t status;

    Drivers_open();
    Board_driversOpen();

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

	/* Task to send file received through USB DFU via uart */ 
    TaskP_Params_init(&gUartTaskParams);
    gUartTaskParams.name = "uart_sendImg";                /**< Pointer to task name */
    gUartTaskParams.stackSize = UART_STACK_SIZE;        /**< Size of stack in units of bytes */
    gUartTaskParams.stack = gUartTaskStack;            /**< Pointer to stack memory, MUST be aligned based on CPU architecture, typically atleast 32b on 32b systems */
    gUartTaskParams.priority = UART_TASK_PRI - 1;          /**< Task priority, MUST be between \ref TaskP_PRIORITY_LOWEST and TaskP_PRIORITY_HIGHEST */
    gUartTaskParams.args = NULL;                      /**< User arguments that are passed back as parater to task main */
    gUartTaskParams.taskMain = uart_send_img;         /**< Entry point function to the task */
    /* create the task */
    status = TaskP_construct(&gUartTaskObj, &gUartTaskParams);
    DebugP_assert(status == SystemP_SUCCESS);

    return 0;
}

/* Task that will send received image via UART */ 
void uart_send_img(void *args)
{
	if( gManifestDone == MANIFEST_DONE)
	{
		/* send the received file to uart as string */ 
		DebugP_log("\r\n%s",(char*)gFileBuf);
		gManifestDone = MANIFEST_PENDING ; 
		/* reset the buffer */ 
		memset((void*)gFileBuf,0,MAX_FILE_SIZE); 
	}
}

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

/*--------------------------------------------------------------------+
* DFU callbacks
* Note: alt is used as the partition number, in order to support multiple partitions like FLASH, EEPROM, etc.
*--------------------------------------------------------------------+

* Invoked right before tud_dfu_download_cb() (state=DFU_DNBUSY) or tud_dfu_manifest_cb() (state=DFU_MANIFEST)
* Application return timeout in milliseconds (bwPollTimeout) for the next download/manifest operation.
* During this period, USB host won't try to communicate with us. */ 
uint32_t tud_dfu_get_timeout_cb(uint8_t alt, uint8_t state)
{
  if ( state == DFU_DNBUSY )
  {
	/* return 1ms timeout */ 
    return (alt == 0) ? 1 : 100;
  }

  return 0;
}

/* Invoked when received DFU_DNLOAD (wLength>0) following by DFU_GETSTATUS (state=DFU_DNBUSY) requests
* This callback could be returned before flashing op is complete (async).
* Once finished flashing, application must call tud_dfu_finish_flashing() */ 
void tud_dfu_download_cb(uint8_t alt, uint16_t block_num, uint8_t const* data, uint16_t length)
{
  (void) alt;
  (void) block_num;

 /* Buffer overflow check */ 
  if((recvFileSize + length) <= MAX_FILE_SIZE)
  {
	  memcpy((void*)FileBufPtr,(void*)data,length); 
	  FileBufPtr += length ; 
	  recvFileSize += length ; 
	  tud_dfu_finish_flashing(DFU_STATUS_OK);
  }
  else
  {
	  recvFileSize = 0 ;  
	  tud_dfu_finish_flashing(DFU_STATUS_ERR_FILE);
  }
}

/* Invoked when download process is complete, received DFU_DNLOAD (wLength=0) following by DFU_GETSTATUS (state=Manifest)
* Application can do checksum, or actual flashing if buffered entire image previously.
* Once finished flashing, application must call tud_dfu_finish_flashing() */ 
void tud_dfu_manifest_cb(uint8_t alt)
{
  (void) alt;

  /* Indicate the flag that manifest stage has reached */ 
  gManifestDone = MANIFEST_DONE ; 	
  /* Reinitialize the same buffer to accept another file */
  FileBufPtr = gFileBuf ;
  recvFileSize = 0 ;

  tud_dfu_finish_flashing(DFU_STATUS_OK);
}

/* Invoked when received DFU_UPLOAD request
* Application must populate data with up to length bytes and
* Return the number of written bytes */ 
uint16_t tud_dfu_upload_cb(uint8_t alt, uint16_t block_num, uint8_t* data, uint16_t length)
{
  (void) block_num;
  (void) length;

  uint16_t const xfer_len = (uint16_t) strlen(upload_image[alt]);
  memcpy(data, upload_image[alt], xfer_len);

  return xfer_len;
}

