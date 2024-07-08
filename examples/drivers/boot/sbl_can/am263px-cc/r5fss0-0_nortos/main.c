/*
 *  Copyright (C) 2023-24 Texas Instruments Incorporated
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
#include "ti_drivers_config.h"
#include "ti_drivers_open_close.h"
#include "ti_board_open_close.h"
#include <drivers/bootloader.h>
#include <drivers/hsmclient.h>
#include <drivers/bootloader/bootloader_can.h>
#include <drivers/hsmclient/soc/am263px/hsmRtImg.h> /* hsmRt bin   header file */

#define BOOTLOADER_CAN_STATUS_LOAD_SUCCESS           (0x53554343) /* SUCC */
#define BOOTLOADER_CAN_STATUS_LOAD_CPU_FAIL          (0x4641494C) /* FAIL */
#define BOOTLOADER_CAN_STATUS_APPIMAGE_SIZE_EXCEEDED (0x45584344) /* EXCD */

#define BOOTLOADER_APPIMAGE_MAX_FILE_SIZE (0x80000) /* Size of section MSRAM_2 specified in linker.cmd */
uint8_t gAppImageBuf[BOOTLOADER_APPIMAGE_MAX_FILE_SIZE] __attribute__((aligned(128), section(".bss.filebuf")));

// const uint8_t gHsmRtFw[HSMRT_IMG_SIZE_IN_BYTES]__attribute__((section(".rodata.hsmrt")))
//     = HSMRT_IMG;

extern HsmClient_t gHSMClient ;

uint32_t gRunApp;

void mcanEnableTransceiver(void);

/* call this API to stop the booting process and spin, do that you can connect
 * debugger, load symbols and then make the 'loop' variable as 0 to continue execution
 * with debugger connected.
 */
void loop_forever()
{
    volatile uint32_t loop = 1;
    while(loop)
        ;
}

/*  this API is a weak function definition for keyring_init function
    which is defined in generated files if keyring module is enabled
    in syscfg
*/
__attribute__((weak)) int32_t Keyring_init(HsmClient_t *gHSMClient)
{
    return SystemP_SUCCESS;
}

int main()
{
    int32_t status;

    Bootloader_profileReset();
    Bootloader_socConfigurePll();
    Bootloader_socSetAutoClock();

    System_init();
    Bootloader_profileAddProfilePoint("System_init");

    Drivers_open();
    Bootloader_profileAddProfilePoint("Drivers_open");

    DebugP_log("\r\n");
    Bootloader_socInitL2MailBoxMemory();
    // Bootloader_socLoadHsmRtFw(&gHSMClient, gHsmRtFw, HSMRT_IMG_SIZE_IN_BYTES);
    Bootloader_profileAddProfilePoint("LoadHsmRtFw");

    status = Keyring_init(&gHSMClient);
    DebugP_assert(status == SystemP_SUCCESS);

    status = Board_driversOpen();
    DebugP_assert(status == SystemP_SUCCESS);
    Bootloader_profileAddProfilePoint("Board_driversOpen");

    mcanEnableTransceiver();

    DebugP_log("Starting CAN Bootloader...\r\n");
    Bootloader_CANInit(CONFIG_MCAN0_BASE_ADDR);

    if(SystemP_SUCCESS == status)
    {
        Bootloader_BootImageInfo bootImageInfo;
        Bootloader_Params bootParams;
        Bootloader_Handle bootHandle;

        Bootloader_Params_init(&bootParams);
        Bootloader_BootImageInfo_init(&bootImageInfo);

        bootParams.bufIoTempBuf     = gAppImageBuf;
        bootParams.bufIoTempBufSize = BOOTLOADER_APPIMAGE_MAX_FILE_SIZE;
        bootParams.bufIoDeviceIndex = CONFIG_MCAN0;
        bootParams.memArgsAppImageBaseAddr = (uintptr_t)gAppImageBuf;

        bootHandle = Bootloader_open(CONFIG_BOOTLOADER_0, &bootParams);
		Bootloader_Config *config = (Bootloader_Config *)bootHandle;
        uint32_t imageFormat = config->imageFormat;

        if(BOOTLOADER_MEDIA_MEM == Bootloader_getBootMedia(bootHandle))
        {
            uint32_t fileSize = 0U;
            /* CAN Receive */
            status = Bootloader_CANReceiveFile(&fileSize, gAppImageBuf, &gRunApp);

            if(SystemP_SUCCESS == status && fileSize == BOOTLOADER_APPIMAGE_MAX_FILE_SIZE)
            {
                /* A file larger than 384 KB was sent, and xmodem probably dropped bytes */
                status = SystemP_FAILURE;

                /* Send response to the script that file size exceeded */
                uint32_t response;
                response = BOOTLOADER_CAN_STATUS_APPIMAGE_SIZE_EXCEEDED;

                Bootloader_CANTransmitResp((uint8_t *)&response);
            }
        }

        if((bootHandle != NULL) && (SystemP_SUCCESS == status) && (gRunApp == CSL_TRUE))
        {
            if(imageFormat == BOOTLOADER_IMAGE_RPRC)
            {
                status = Bootloader_parseMultiCoreAppImage(bootHandle, &bootImageInfo);
            }
            /* Load CPUs */
            if(status == SystemP_SUCCESS)
            {
                bootImageInfo.cpuInfo[CSL_CORE_ID_R5FSS1_1].clkHz = Bootloader_socCpuGetClkDefault(CSL_CORE_ID_R5FSS1_1);
                Bootloader_profileAddCore(CSL_CORE_ID_R5FSS1_1);
                status = Bootloader_loadCpu(bootHandle, &bootImageInfo.cpuInfo[CSL_CORE_ID_R5FSS1_1], imageFormat);
			}
            if(status == SystemP_SUCCESS)
            {
                bootImageInfo.cpuInfo[CSL_CORE_ID_R5FSS1_0].clkHz = Bootloader_socCpuGetClkDefault(CSL_CORE_ID_R5FSS1_0);
                Bootloader_profileAddCore(CSL_CORE_ID_R5FSS1_0);
                status = Bootloader_loadCpu(bootHandle, &bootImageInfo.cpuInfo[CSL_CORE_ID_R5FSS1_0], imageFormat);
            }
            if(status == SystemP_SUCCESS)
            {
                /* Set clocks for self cluster */
                bootImageInfo.cpuInfo[CSL_CORE_ID_R5FSS0_0].clkHz = Bootloader_socCpuGetClkDefault(CSL_CORE_ID_R5FSS0_0);
                bootImageInfo.cpuInfo[CSL_CORE_ID_R5FSS0_1].clkHz = Bootloader_socCpuGetClkDefault(CSL_CORE_ID_R5FSS0_1);
                Bootloader_profileAddCore(CSL_CORE_ID_R5FSS0_0);
                Bootloader_profileAddCore(CSL_CORE_ID_R5FSS0_1);

                /* Reset self cluster, both Core0 and Core 1. Init RAMs and load the app  */
                status = Bootloader_loadCpu(bootHandle, &bootImageInfo.cpuInfo[CSL_CORE_ID_R5FSS0_1], imageFormat);
                Bootloader_profileAddProfilePoint("CPU load");

                /* Skip the image load by passing TRUE, so that image load on self core doesnt corrupt the SBLs IVT. Load the image later before the reset release of the self core  */
                if(status == SystemP_SUCCESS)
                {
                    status = Bootloader_loadSelfCpu(bootHandle, &bootImageInfo.cpuInfo[CSL_CORE_ID_R5FSS0_0], imageFormat);
                }
            }
			if(imageFormat == BOOTLOADER_IMAGE_MCELF)
            {
                status = Bootloader_parseAndLoadMultiCoreELF(bootHandle, &bootImageInfo);
            }
            if(BOOTLOADER_MEDIA_BUFIO == Bootloader_getBootMedia(bootHandle))
            {
                BufIo_sendTransferComplete(CONFIG_MCAN0);
            }
            else if(BOOTLOADER_MEDIA_MEM == Bootloader_getBootMedia(bootHandle))
            {
                uint32_t response = BOOTLOADER_CAN_STATUS_LOAD_SUCCESS;

                if(status != SystemP_SUCCESS)
                {
                    response = BOOTLOADER_CAN_STATUS_LOAD_CPU_FAIL;
                }

                Bootloader_CANTransmitResp((uint8_t *)&response);
            }
            else
            {
                /* do nothing */
            }

            /* Run CPUs */
            if(status == SystemP_SUCCESS)
            {
                status = Bootloader_runCpu(bootHandle, &bootImageInfo.cpuInfo[CSL_CORE_ID_R5FSS1_1]);
            }
            if(status == SystemP_SUCCESS)
            {
                status = Bootloader_runCpu(bootHandle, &bootImageInfo.cpuInfo[CSL_CORE_ID_R5FSS1_0]);
            }
            if(status == SystemP_SUCCESS)
            {
                status = Bootloader_runCpu(bootHandle, &bootImageInfo.cpuInfo[CSL_CORE_ID_R5FSS0_1]);
            }
            if(status == SystemP_SUCCESS)
            {
                /* Load the image on self core now */
                if((imageFormat == BOOTLOADER_IMAGE_RPRC) && bootImageInfo.cpuInfo[CSL_CORE_ID_R5FSS0_0].rprcOffset != BOOTLOADER_INVALID_ID)
                {
                    status = Bootloader_rprcImageLoad(bootHandle, &bootImageInfo.cpuInfo[CSL_CORE_ID_R5FSS0_0]);
                }
                if(status == SystemP_SUCCESS)
                {
                    Bootloader_profileAddProfilePoint("SBL End");
                    Bootloader_profilePrintProfileLog();
                    DebugP_log("Image loading done, switching to application ...\r\n");
                    UART_flushTxFifo(gUartHandle[CONFIG_UART0]);
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