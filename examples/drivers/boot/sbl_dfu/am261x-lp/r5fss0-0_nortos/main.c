/*
 *  Copyright (C) 2018-2024 Texas Instruments Incorporated
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
#include "ti_drivers_config.h"
#include "ti_drivers_open_close.h"
#include "ti_board_open_close.h"
#include <drivers/bootloader.h>
#include <security/security_common/drivers/hsmclient/hsmclient.h>
#include <drivers/bootloader/bootloader_xmodem.h>
#include <security/security_common/drivers/hsmclient/soc/am261x/hsmRtImg.h> /* hsmRt bin   header file */
#include <usb/synp/soc/usb_init.h>
#include "tusb.h"
#include "usbd.h"

#define MANIFEST_DONE     (0xA1)
#define MANIFEST_PENDING  (0xB2)

#define BOOTLOADER_APPIMAGE_MAX_FILE_SIZE (0x80000) /* Size of section MSRAM_2 specified in linker.cmd */
uint8_t gAppImageBuf[BOOTLOADER_APPIMAGE_MAX_FILE_SIZE] __attribute__((aligned(128), section(".bss.filebuf")));

const uint8_t gHsmRtFw[HSMRT_IMG_SIZE_IN_BYTES]__attribute__((section(".rodata.hsmrt"))) = HSMRT_IMG;

extern HsmClient_t gHSMClient ;

/* global pointer to point the filebuf */
uint8_t *FileBufPtr = gAppImageBuf ;

/* global variable to track received File size */
uint32_t recvFileSize = 0 ;

/* flag to indicate manifest done or not */
uint8_t gManifestDone = MANIFEST_PENDING;

extern char __DFU_CTX_START, __DFU_CTX_END ;

/* TinyUSB dfu context struct */
typedef struct
{
    uint8_t attrs;
    uint8_t alt;

    dfu_state_t state;
    dfu_status_t status;

    bool flashing_in_progress;
    uint16_t block;
    uint16_t length;

    CFG_TUSB_MEM_ALIGN uint8_t transfer_buf[CFG_TUD_DFU_XFER_BUFSIZE];
} dfu_state_ctx_t;

/* This pointer will point to the DFU state machine context variable
 * The _dfu_ctx defined in TinyUSB DFU class driver is a static variable
 * and we need to identify the state of DFU state machine after the manifest stage
 * is completed. */
dfu_state_ctx_t *dfu_ctx = (dfu_state_ctx_t*)(&__DFU_CTX_START);

/* call this API to stop the booting process and spin, do that you can connect
 * debugger, load symbols and then make the 'loop' variable as 0 to continue execution
 * with debugger connected.
 */
void loop_forever(void)
{
    volatile uint32_t loop = 1;
    while(loop);
}

/* Main DFU task */
int dfu_task(void)
{
    while (1)
    {
        USB_dwcTask(); /* Synopsis DWC task */
        tud_task();    /* tinyusb device task */

        /* Remain in this forever loop till DFU manifest stage is complete
         * and then the device return DFU state = DFU_IDLE and status = DFU_STATUS_OK */
        if(gManifestDone == MANIFEST_DONE && dfu_ctx->state == DFU_IDLE )
        {
            /* Serve the last occurred events */
            USB_dwcTask(); 
            tud_task();
            break;
        }

    }
    return 0;
}

/*  this API is a weak function definition for keyring_init function
    which is defined in generated files if keyring module is enabled
    in syscfg
*/
__attribute__((weak)) int32_t Keyring_init(HsmClient_t *gHSMClient)
{
    return SystemP_SUCCESS;
}

int main(void)
{
    int32_t status;
    
    Bootloader_socConfigurePll();
    Bootloader_socSetAutoClock();

    System_init();
    Drivers_open();
    // Bootloader_socLoadHsmRtFw(&gHSMClient, gHsmRtFw, HSMRT_IMG_SIZE_IN_BYTES);
    Bootloader_socInitL2MailBoxMemory();

    // status = Keyring_init(&gHSMClient);
    // DebugP_assert(status == SystemP_SUCCESS);

    status = Board_driversOpen();
    DebugP_assert(status == SystemP_SUCCESS);

    /* exit once DFU download is completed */
    dfu_task();

    if(SystemP_SUCCESS == status)
    {
        Bootloader_BootImageInfo bootImageInfo;
        Bootloader_Params bootParams;
        Bootloader_Handle bootHandle;

        Bootloader_Params_init(&bootParams);
        Bootloader_BootImageInfo_init(&bootImageInfo);

        bootParams.bufIoTempBuf     = gAppImageBuf;
        bootParams.bufIoTempBufSize = BOOTLOADER_APPIMAGE_MAX_FILE_SIZE;
        bootParams.bufIoDeviceIndex = CONFIG_UART0;
        bootParams.memArgsAppImageBaseAddr = (uintptr_t)gAppImageBuf;

        bootHandle = Bootloader_open(CONFIG_BOOTLOADER_0, &bootParams);

        if((bootHandle != NULL) && (SystemP_SUCCESS == status) && BOOTLOADER_MEDIA_MEM == Bootloader_getBootMedia(bootHandle))
        {

            status = Bootloader_parseMultiCoreAppImage(bootHandle, &bootImageInfo);

            /* Initialize CPUs and Load RPRC Image */
            if(status == SystemP_SUCCESS && ((TRUE == Bootloader_isCorePresent(bootHandle, CSL_CORE_ID_R5FSS0_0)) || (TRUE == Bootloader_isCorePresent(bootHandle, CSL_CORE_ID_R5FSS0_1))))
            {
                /* Set clocks for self cluster */
                bootImageInfo.cpuInfo[CSL_CORE_ID_R5FSS0_0].clkHz = Bootloader_socCpuGetClkDefault(CSL_CORE_ID_R5FSS0_0);
                bootImageInfo.cpuInfo[CSL_CORE_ID_R5FSS0_1].clkHz = Bootloader_socCpuGetClkDefault(CSL_CORE_ID_R5FSS0_1);

                /* Reset self cluster, both Core0 and Core 1. Init RAMs and load the app  */
                status = Bootloader_initCpu(bootHandle, &bootImageInfo.cpuInfo[CSL_CORE_ID_R5FSS0_1]);

                if(bootImageInfo.cpuInfo[CSL_CORE_ID_R5FSS0_1].rprcOffset != BOOTLOADER_INVALID_ID) {
                    status = Bootloader_rprcImageLoad(bootHandle, &bootImageInfo.cpuInfo[CSL_CORE_ID_R5FSS0_1]);
                }

                /* Skip the image load by passing TRUE, so that image load on self core doesnt corrupt the SBLs IVT. Load the image later before the reset release of the self core  */
                if(status == SystemP_SUCCESS)
                {
                    status = Bootloader_loadSelfCpu(bootHandle, &bootImageInfo.cpuInfo[CSL_CORE_ID_R5FSS0_0], TRUE);
                }
            }
            /* Run CPUs */
            if(status == SystemP_SUCCESS)
            {
                status = Bootloader_runCpu(bootHandle, &bootImageInfo.cpuInfo[CSL_CORE_ID_R5FSS0_1]);
            }
            if(status == SystemP_SUCCESS)
            {
                /* Load the RPRC image on self core now */
                if(bootImageInfo.cpuInfo[CSL_CORE_ID_R5FSS0_0].rprcOffset != BOOTLOADER_INVALID_ID)
                {
                    status = Bootloader_rprcImageLoad(bootHandle, &bootImageInfo.cpuInfo[CSL_CORE_ID_R5FSS0_0]);
                }
                /* Reset self cluster, both Core0 and Core 1. Init RAMs and run the app  */
                status = Bootloader_runSelfCpu(bootHandle, &bootImageInfo);
            }

            /* it should not return here, if it does, then there was some error */
            Bootloader_close(bootHandle);
        }
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

