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
#include <kernel/dpl/CycleCounterP.h> 

#include <usb/cdn/include/usb_init.h>
#include "tusb.h"
#include "usbd.h"


#define MANIFEST_DONE     (0xA1) 
#define MANIFEST_PENDING  (0xB2) 
#define MANIFEST_ERROR    (0xC3) 
#define DETACH_NOT_RECV   (0x55) 
#define DETACH_RECV       (0xAA) 

#define BOOTLOADER_APPIMAGE_MAX_FILE_SIZE (0x60000) /* Size of section MSRAM_2 specified in linker.cmd */
uint8_t gAppImageBuf[BOOTLOADER_APPIMAGE_MAX_FILE_SIZE] __attribute__((aligned(128), section(".bss.filebuf")));

/* global pointer to point the filebuf */ 
uint8_t *FileBufPtr = gAppImageBuf ;

/* global variable to track received File size */ 
uint32_t recvFileSize = 0 ; 

/* flag to indicate manifest done or not */ 
uint8_t gManifestDone = MANIFEST_PENDING; 

uint8_t gDetachFlag = DETACH_NOT_RECV ; 
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
	/* 1s delay */ 
	uint32_t maxTics = 800000000;
	uint32_t startTicks = CycleCounterP_getCount32(); 
	uint32_t endTicks = CycleCounterP_getCount32() ;

    while ( (endTicks - startTicks) < maxTics) 
    {
	    cusbd_dsr(); /* Cadence DSR task */
        tud_task(); /* tinyusb device task */
		
		/* If this condition occurs wait for 10ms to complete the reset sequence */ 
		if(gManifestDone == MANIFEST_DONE && gDetachFlag == DETACH_RECV)
		{
			endTicks = CycleCounterP_getCount32(); 
		}
		else 
		{
			/* prevent overflow */ 
			startTicks = CycleCounterP_getCount32(); 
			endTicks = CycleCounterP_getCount32() ;  	
		}
    }
    return 0;
}

int main(void)
{
    int32_t status = SystemP_SUCCESS;
    Bootloader_socWaitForFWBoot();

    if (!Bootloader_socIsMCUResetIsoEnabled())
    {
        /* Update devGrp to ALL to initialize MCU domain when reset isolation is
        not enabled */
        Sciclient_BoardCfgPrms_t boardCfgPrms_pm =
            {
                .boardConfigLow = (uint32_t)0,
                .boardConfigHigh = 0,
                .boardConfigSize = 0,
                .devGrp = DEVGRP_ALL,
            };

        status = Sciclient_boardCfgPm(&boardCfgPrms_pm);

        /* Enable MCU PLL. MCU PLL will not be enabled by DMSC when devGrp is set
        to Main in boardCfg */
        Bootloader_enableMCUPLL();
    }

    Bootloader_socOpenFirewalls();

    Bootloader_socNotifyFirewallOpen();

    System_init();
    Drivers_open();

    status = Board_driversOpen();
    DebugP_assert(status == SystemP_SUCCESS);
	

	/* exit once DFU download is completed */ 
	dfu_task() ; 
	tud_disconnect(); 
	tud_connect(); 
	Bootloader_BootImageInfo bootImageInfo;
	Bootloader_Params bootParams;
	Bootloader_Handle bootHandle;

	Bootloader_Params_init(&bootParams);
	Bootloader_BootImageInfo_init(&bootImageInfo);

	bootParams.bufIoTempBuf     = gAppImageBuf;
	bootParams.bufIoTempBufSize = BOOTLOADER_APPIMAGE_MAX_FILE_SIZE;
	bootParams.memArgsAppImageBaseAddr = (uintptr_t)gAppImageBuf;

	bootHandle = Bootloader_open(CONFIG_BOOTLOADER_0, &bootParams);

	/* Boot media == MEMBOOT loader is selected */ 
	/* Appimage is in memory buffer */ 
	if((bootHandle != NULL) && (SystemP_SUCCESS == status) && ( BOOTLOADER_MEDIA_MEM == Bootloader_getBootMedia(bootHandle))) 
	{
		status = Bootloader_parseMultiCoreAppImage(bootHandle, &bootImageInfo);
		/* Load CPUs */
		/* Do not load M4 when MCU domain is reset isolated */
		if (!Bootloader_socIsMCUResetIsoEnabled())
		{
			if(status == SystemP_SUCCESS && (TRUE == Bootloader_isCorePresent(bootHandle, CSL_CORE_ID_M4FSS0_0)))
			{
				bootImageInfo.cpuInfo[CSL_CORE_ID_M4FSS0_0].clkHz = Bootloader_socCpuGetClkDefault(CSL_CORE_ID_M4FSS0_0);
				status = Bootloader_loadCpu(bootHandle, &bootImageInfo.cpuInfo[CSL_CORE_ID_M4FSS0_0]);
			}
		}
		if(status == SystemP_SUCCESS && (TRUE == Bootloader_isCorePresent(bootHandle, CSL_CORE_ID_R5FSS1_0)))
		{
			bootImageInfo.cpuInfo[CSL_CORE_ID_R5FSS1_0].clkHz = Bootloader_socCpuGetClkDefault(CSL_CORE_ID_R5FSS1_0);
			status = Bootloader_loadCpu(bootHandle, &bootImageInfo.cpuInfo[CSL_CORE_ID_R5FSS1_0]);
		}
		if(status == SystemP_SUCCESS && (TRUE == Bootloader_isCorePresent(bootHandle, CSL_CORE_ID_R5FSS1_1)))
		{
			bootImageInfo.cpuInfo[CSL_CORE_ID_R5FSS1_1].clkHz = Bootloader_socCpuGetClkDefault(CSL_CORE_ID_R5FSS1_1);
			status = Bootloader_loadCpu(bootHandle, &bootImageInfo.cpuInfo[CSL_CORE_ID_R5FSS1_1]);
		}

		if(status == SystemP_SUCCESS && ((Bootloader_isCorePresent(bootHandle, CSL_CORE_ID_R5FSS0_0) == TRUE) || (Bootloader_isCorePresent(bootHandle, CSL_CORE_ID_R5FSS0_1) == TRUE)))
		{
			/* Set clocks for self cluster */
			bootImageInfo.cpuInfo[CSL_CORE_ID_R5FSS0_0].clkHz = Bootloader_socCpuGetClkDefault(CSL_CORE_ID_R5FSS0_0);
			bootImageInfo.cpuInfo[CSL_CORE_ID_R5FSS0_1].clkHz = Bootloader_socCpuGetClkDefault(CSL_CORE_ID_R5FSS0_1);

			/* Reset self cluster, both Core0 and Core 1. Init RAMs and load the app  */
			status = Bootloader_loadSelfCpu(bootHandle, &bootImageInfo.cpuInfo[CSL_CORE_ID_R5FSS0_0]);
			if((status == SystemP_SUCCESS) && (TRUE == Bootloader_socIsR5FSSDual(BOOTLOADER_R5FSS0)))
			{
				status = Bootloader_loadSelfCpu(bootHandle, &bootImageInfo.cpuInfo[CSL_CORE_ID_R5FSS0_1]);
			}
		}

		/* Run CPUs */
		/* Do not run M4 when MCU domain is reset isolated */
		if (!Bootloader_socIsMCUResetIsoEnabled())
		{
			if(status == SystemP_SUCCESS && (TRUE == Bootloader_isCorePresent(bootHandle, CSL_CORE_ID_M4FSS0_0)))
			{
				status = Bootloader_runCpu(bootHandle, &bootImageInfo.cpuInfo[CSL_CORE_ID_M4FSS0_0]);
			}
		}
		if(status == SystemP_SUCCESS && (TRUE == Bootloader_isCorePresent(bootHandle, CSL_CORE_ID_R5FSS1_0)))
		{
			status = Bootloader_runCpu(bootHandle, &bootImageInfo.cpuInfo[CSL_CORE_ID_R5FSS1_0]);
		}
		if(status == SystemP_SUCCESS && (TRUE == Bootloader_isCorePresent(bootHandle, CSL_CORE_ID_R5FSS1_1)))
		{
			status = Bootloader_runCpu(bootHandle, &bootImageInfo.cpuInfo[CSL_CORE_ID_R5FSS1_1]);
		}
		if(status == SystemP_SUCCESS && ((Bootloader_isCorePresent(bootHandle, CSL_CORE_ID_R5FSS0_0) == TRUE) || (Bootloader_isCorePresent(bootHandle, CSL_CORE_ID_R5FSS0_1) == TRUE)))
		{
			/* Reset self cluster, both Core0 and Core 1. Init RAMs and run the app  */
			status = Bootloader_runSelfCpu(bootHandle, &bootImageInfo);
		}

		/* it should not return here, if it does, then there was some error */
		Bootloader_close(bootHandle);
        
    }
    if(status != SystemP_SUCCESS)
    {
        DebugP_log("Some tests have failed!!\r\n");
    }
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
  if((recvFileSize + length) <= BOOTLOADER_APPIMAGE_MAX_FILE_SIZE ) 
  {
	  memcpy((void*)FileBufPtr,(void*)data,length); 
	  FileBufPtr += length ; 
	  recvFileSize += length ; 
	  tud_dfu_finish_flashing(DFU_STATUS_OK);
  }
  else
  {
	  /* Set flag for error condition */ 
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

	 /* If manifest stage reached then it means that correct appimage was recieved in the filebuf */ 
	 /* Nothing has to be done in manifest stage */ 
		/*Set global flag to start boot process */ 
	gManifestDone = MANIFEST_DONE ; 
	/* always return manifest status ok */ 
	tud_dfu_finish_flashing(DFU_STATUS_OK);
}

/* Invoked when a DFU_DETACH request is received */ 
void tud_dfu_detach_cb(void)
{
	/* Set a flag to indicate that detach is received */ 
	/* so that SBL and go ahead and boot the received appimage */ 
	gDetachFlag = DETACH_RECV ; 
	
}
