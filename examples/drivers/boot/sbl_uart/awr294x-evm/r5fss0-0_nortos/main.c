/*
 *  Copyright (C) 2018-2023 Texas Instruments Incorporated
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
#include <drivers/bootloader/bootloader_xmodem.h>
#include <drivers/hsmclient.h>
#include <drivers/hsmclient/soc/awr294x/hsmRtImg.h> /* hsmRt bin   header file */

#define BOOTLOADER_UART_STATUS_LOAD_SUCCESS           (0x53554343) /* SUCC */
#define BOOTLOADER_UART_STATUS_LOAD_CPU_FAIL          (0x4641494C) /* FAIL */
#define BOOTLOADER_UART_STATUS_APPIMAGE_SIZE_EXCEEDED (0x45584344) /* EXCD */

#define BOOTLOADER_UART_CPU_RUN_WAIT_SECONDS (2)
#define BOOTLOADER_SBL_SCRATCH_MEM_ADDR                (0x88100000)

#define BOOTLOADER_APPIMAGE_MAX_FILE_SIZE (0x100000) /* Size of section DSS L3 RAM specified in linker.cmd */
uint8_t gAppImageBuf[BOOTLOADER_APPIMAGE_MAX_FILE_SIZE] __attribute__((aligned(128), section(".bss.sbl_scratch")));

const uint8_t gHsmRtFw[HSMRT_IMG_SIZE_IN_BYTES]__attribute__((section(".rodata.hsmrt")))
    = HSMRT_IMG;

extern HsmClient_t gHSMClient ;

/*  this API is a weak function definition for keyring_init function
    which is defined in generated files if keyring module is enabled
    in syscfg
*/
__attribute__((weak)) int32_t Keyring_init(HsmClient_t *gHSMClient)
{
    return SystemP_SUCCESS;
}

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

int main(void)
{
    int32_t status;

    Bootloader_Params bootParams;
    Bootloader_BootImageInfo bootImageInfo;
    Bootloader_Handle bootHandle = NULL;

    Bootloader_socConfigurePll();

    System_init();
    Drivers_open();
    Bootloader_socLoadHsmRtFw(&gHSMClient, gHsmRtFw, HSMRT_IMG_SIZE_IN_BYTES);

    /* Initialize the DSP Core and DSS L3 Memory. */
    Bootloader_BootImageInfo_init(&bootImageInfo);
    status = Bootloader_loadCpu(bootHandle, &bootImageInfo.cpuInfo[CSL_CORE_ID_C66SS0]);
    DebugP_assert(status == SystemP_SUCCESS);

    status = Board_driversOpen();
    DebugP_assert(status == SystemP_SUCCESS);
    Bootloader_xmodemSendAck(CONFIG_UART0);

    if(SystemP_SUCCESS == status)
    {
        Bootloader_Params_init(&bootParams);
        Bootloader_BootImageInfo_init(&bootImageInfo);

        bootParams.bufIoTempBuf     = gAppImageBuf;
        bootParams.bufIoTempBufSize = BOOTLOADER_APPIMAGE_MAX_FILE_SIZE;
        bootParams.bufIoDeviceIndex = CONFIG_UART0;
        bootParams.memArgsAppImageBaseAddr = (uintptr_t)gAppImageBuf;

        bootHandle = Bootloader_open(CONFIG_BOOTLOADER_0, &bootParams);

        if(BOOTLOADER_MEDIA_MEM == Bootloader_getBootMedia(bootHandle))
        {
            uint32_t fileSize;
            /* Xmodem Receive */
            status = Bootloader_xmodemReceive(CONFIG_UART0, gAppImageBuf, BOOTLOADER_APPIMAGE_MAX_FILE_SIZE, &fileSize);

            if(SystemP_SUCCESS == status && fileSize == BOOTLOADER_APPIMAGE_MAX_FILE_SIZE)
            {
                /* A file larger than 384 KB was sent, and xmodem probably dropped bytes */
                status = SystemP_FAILURE;

                /* Send response to the script that file size exceeded */
                uint32_t response;
                response = BOOTLOADER_UART_STATUS_APPIMAGE_SIZE_EXCEEDED;

                Bootloader_xmodemTransmit(CONFIG_UART0, (uint8_t *)&response, 4);
            }
        }

        if((bootHandle != NULL) && (SystemP_SUCCESS == status))
        {
            status = Bootloader_parseMultiCoreAppImage(bootHandle, &bootImageInfo);
            /* Load CPUs */
            if((status == SystemP_SUCCESS) && (TRUE == Bootloader_isCorePresent(bootHandle, CSL_CORE_ID_RSS_R4)))
            {
                bootImageInfo.cpuInfo[CSL_CORE_ID_RSS_R4].clkHz = Bootloader_socCpuGetClkDefault(CSL_CORE_ID_RSS_R4);
                status = Bootloader_loadCpu(bootHandle, &bootImageInfo.cpuInfo[CSL_CORE_ID_RSS_R4]);
            }
            if((status == SystemP_SUCCESS) && (TRUE == Bootloader_isCorePresent(bootHandle, CSL_CORE_ID_C66SS0)))
            {
                bootImageInfo.cpuInfo[CSL_CORE_ID_C66SS0].clkHz = Bootloader_socCpuGetClkDefault(CSL_CORE_ID_C66SS0);
                status = Bootloader_loadCpu(bootHandle, &bootImageInfo.cpuInfo[CSL_CORE_ID_C66SS0]);
            }
            if ((status == SystemP_SUCCESS) && (TRUE == Bootloader_isCorePresent(bootHandle, CSL_CORE_ID_R5FSS0_1)))
            {
                bootImageInfo.cpuInfo[CSL_CORE_ID_R5FSS0_1].clkHz = Bootloader_socCpuGetClkDefault(CSL_CORE_ID_R5FSS0_1);
                status = Bootloader_loadCpu(bootHandle, &bootImageInfo.cpuInfo[CSL_CORE_ID_R5FSS0_1]);
            }
            if((status == SystemP_SUCCESS) && (TRUE == Bootloader_isCorePresent(bootHandle, CSL_CORE_ID_R5FSS0_0)))
            {
                bootImageInfo.cpuInfo[CSL_CORE_ID_R5FSS0_0].clkHz = Bootloader_socCpuGetClkDefault(CSL_CORE_ID_R5FSS0_0);
                /* Skip the image load by passing TRUE, so that image load on self core doesnt corrupt the SBLs IVT. Load the image later before the reset release of the self core  */
                status = Bootloader_loadSelfCpu(bootHandle, &bootImageInfo.cpuInfo[CSL_CORE_ID_R5FSS0_0], TRUE);
            }
            if(BOOTLOADER_MEDIA_BUFIO == Bootloader_getBootMedia(bootHandle))
            {
                BufIo_sendTransferComplete(CONFIG_UART0);
            }
            else if(BOOTLOADER_MEDIA_MEM == Bootloader_getBootMedia(bootHandle))
            {
                uint32_t response = BOOTLOADER_UART_STATUS_LOAD_SUCCESS;

                if(status != SystemP_SUCCESS)
                {
                    response = BOOTLOADER_UART_STATUS_LOAD_CPU_FAIL;
                }

                Bootloader_xmodemTransmit(CONFIG_UART0, (uint8_t *)&response, 4);
            }
            else
            {
                /* do nothing */
            }

            /* Delay 2 seconds for the user to connect to UART before the CPUs start running*/
            ClockP_sleep(BOOTLOADER_UART_CPU_RUN_WAIT_SECONDS);

            /* Run CPUs */
            if((status == SystemP_SUCCESS) && (TRUE == Bootloader_isCorePresent(bootHandle, CSL_CORE_ID_RSS_R4)))
            {
                status = Bootloader_runCpu(bootHandle, &bootImageInfo.cpuInfo[CSL_CORE_ID_RSS_R4]);
                Bootloader_socConfigurePllPostApllSwitch();
            }
            if((status == SystemP_SUCCESS) && (TRUE == Bootloader_isCorePresent(bootHandle, CSL_CORE_ID_C66SS0)))
            {
                status = Bootloader_runCpu(bootHandle, &bootImageInfo.cpuInfo[CSL_CORE_ID_C66SS0]);
            }
            if((status == SystemP_SUCCESS) && (TRUE == Bootloader_isCorePresent(bootHandle, CSL_CORE_ID_R5FSS0_1)))
            {
                status = Bootloader_runCpu(bootHandle, &bootImageInfo.cpuInfo[CSL_CORE_ID_R5FSS0_1]);
            }

            /*
             * Check if RSS_R4 firmware is booted successfully.
             * RSS_R4 boot sequence doesnot have any impact on loading and unhalting of other cores (C666SS0, R5FSS0_0 and R5FSS0_1)
             * To reduce SBL power-Up time checking of RSS_R4 firmware boot success should be done just before unhalting R5FSS0_0 / R5FSS0_1.
             */
            if((status == SystemP_SUCCESS) && (TRUE == Bootloader_isCorePresent(bootHandle, CSL_CORE_ID_RSS_R4)))
            {
                SOC_rcmWaitBSSBootComplete();
            }

            if(status == SystemP_SUCCESS && ((TRUE == Bootloader_isCorePresent(bootHandle, CSL_CORE_ID_R5FSS0_0)) || (TRUE == Bootloader_isCorePresent(bootHandle, CSL_CORE_ID_R5FSS0_1))))
            {
                /* Load the image on self core now */
                if( bootImageInfo.cpuInfo[CSL_CORE_ID_R5FSS0_0].rprcOffset != BOOTLOADER_INVALID_ID)
                {
                    status = Bootloader_rprcImageLoad(bootHandle, &bootImageInfo.cpuInfo[CSL_CORE_ID_R5FSS0_0]);
                }
                /* If any of the R5 core 0 /core 1 have valid image reset the R5 core. */
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


