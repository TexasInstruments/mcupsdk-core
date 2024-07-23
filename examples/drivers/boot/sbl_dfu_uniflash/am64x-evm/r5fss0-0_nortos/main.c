/*
 *  Copyright (C) 2018-2021 Texas Instruments Incorporated
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


#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <ctype.h>
#include "ti_drivers_config.h"
#include "ti_drivers_open_close.h"
#include "ti_board_open_close.h"
#include <drivers/sciclient.h>
#include <drivers/bootloader.h>
#include <drivers/bootloader/bootloader_xmodem.h>
#include <drivers/bootloader/bootloader_uniflash/bootloader_uniflash.h>

#include <usb/cdn/include/usb_init.h>
#include "tusb.h"

#define BOOTLOADER_UNIFLASH_MAX_FILE_SIZE (0x150000) /* This has to match the size of MSRAM_2 section in linker.cmd */
uint8_t gUniflashFileBuf[BOOTLOADER_UNIFLASH_MAX_FILE_SIZE] __attribute__((aligned(128), section(".bss.filebuf")));


#define BOOTLOADER_UNIFLASH_VERIFY_BUF_MAX_SIZE (32*1024)
uint8_t gUniflashVerifyBuf[BOOTLOADER_UNIFLASH_VERIFY_BUF_MAX_SIZE] __attribute__((aligned(128), section(".bss")));

/* global pointer to point the filebuf */ 
uint8_t *FileBufPtr = gUniflashFileBuf ;

/* global variable to track received File size */ 
uint32_t recvFileSize = 0 ; 

/** @brief private function that will parse the response message and 
 * send appropriate DFU status back 
 *  
 *  @param respHeader Flash response Header type 
 *
 *  @return DFU status code 
 */ 
inline uint8_t parseUniRespHeader(Bootloader_UniflashResponseHeader* respHeader); 

/* call this API to stop the booting process and spin, do that you can connect
 * debugger, load symbols and then make the 'loop' variable as 0 to continue execution
 * with debugger connected.
 */

void loop_forever(void)
{
    volatile uint32_t loop = 1;
    while(loop)
        ;
}

/* Main DFU task */ 
int dfu_task(void)
{
    while (1) 
    {
	    cusbd_dsr(); /* Cadence DSR task */
        tud_task(); /* tinyusb device task */
    }
    return 0;
}

int main(void)
{
    int32_t status = SystemP_SUCCESS;

    Bootloader_socWaitForFWBoot();
    Bootloader_socOpenFirewalls();

    Bootloader_socNotifyFirewallOpen();

    System_init();
    Drivers_open();

    status = Board_driversOpen();
    DebugP_assert(status == SystemP_SUCCESS);
	
	dfu_task() ; 

    Drivers_close();
    System_deinit();

    return 0;
}

/*--------------------------------------------------------------------+
* DFU callbacks
* Note: alt is used as the partition number, in order to support multiple partitions like FLASH, EEPROM, etc.
*--------------------------------------------------------------------+ */ 

/* Invoked right before tud_dfu_download_cb() (state=DFU_DNBUSY) or tud_dfu_manifest_cb() (state=DFU_MANIFEST)
* Application return timeout in milliseconds (bwPollTimeout) for the next download/manifest operation.
* During this period, USB host won't try to communicate with us. */ 

uint32_t tud_dfu_get_timeout_cb(uint8_t alt, uint8_t state)
{
  if ( state == DFU_DNBUSY )
  {
	  /* as of now we support only flash memory */ 
	  return 1 ; 
  }
  return 0;
}

/* Invoked when received DFU_DNLOAD (wLength>0) following by DFU_GETSTATUS (state=DFU_DNBUSY) requests
* This callback could be returned before flashing op is complete (async).
* Once finished flashing, application must call tud_dfu_finish_flashing() */ 

void tud_dfu_download_cb(uint8_t alt, uint16_t block_num, uint8_t const* data, uint16_t length)
{
  /* buffer overflow check */ 
  if((recvFileSize + length) <= BOOTLOADER_UNIFLASH_MAX_FILE_SIZE)
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
    uint8_t dfu_status = DFU_STATUS_OK;
    Bootloader_UniflashConfig uniflashConfig;
    Bootloader_UniflashResponseHeader respHeader;
	
	/* Parse file buffer */ 
	uniflashConfig.flashIndex = CONFIG_FLASH0;
	uniflashConfig.buf = gUniflashFileBuf;
	uniflashConfig.bufSize = 0; /* Actual fileSize will be parsed from the header */
	uniflashConfig.verifyBuf = gUniflashVerifyBuf;
	uniflashConfig.verifyBufSize = BOOTLOADER_UNIFLASH_VERIFY_BUF_MAX_SIZE;

	/* Process the flash commands and return a response */
	Bootloader_uniflashProcessFlashCommands(&uniflashConfig, &respHeader);

	/* Parse response header */ 
	dfu_status = parseUniRespHeader(&respHeader);

	/* Reinitialize the same buffer to accept another file */
	FileBufPtr = gUniflashFileBuf ;
	recvFileSize = 0 ; 

	/* update status */ 
	tud_dfu_finish_flashing(dfu_status);
}

uint8_t parseUniRespHeader(Bootloader_UniflashResponseHeader* respHeader)
{
	uint8_t dfu_status = DFU_STATUS_OK ; 
	
	/* Map error codes from Uniflash to DFU status codes */ 
	switch(respHeader->statusCode)
	{
		case BOOTLOADER_UNIFLASH_STATUSCODE_SUCCESS: 
			dfu_status = DFU_STATUS_OK ; 
			break ; 

		case BOOTLOADER_UNIFLASH_STATUSCODE_MAGIC_ERROR: 
			dfu_status = DFU_STATUS_ERR_UNKNOWN ; 
			break ;

		case BOOTLOADER_UNIFLASH_STATUSCODE_OPTYPE_ERROR: 
			dfu_status = DFU_STATUS_ERR_UNKNOWN ; 
			break ; 

		case BOOTLOADER_UNIFLASH_STATUSCODE_FLASH_ERROR: 
			dfu_status = DFU_STATUS_ERR_WRITE ; 
			break ; 

		case BOOTLOADER_UNIFLASH_STATUSCODE_FLASH_VERIFY_ERROR: 
			dfu_status = DFU_STATUS_ERR_VERIFY ; 
			break ; 

		case BOOTLOADER_UNIFLASH_STATUSCODE_FLASH_ERASE_ERROR: 
			dfu_status = DFU_STATUS_ERR_ERASE ; 
			break ;

		default:
			dfu_status = DFU_STATUS_ERR_UNKNOWN;
			break ; 
	}
	return dfu_status ; 
}
