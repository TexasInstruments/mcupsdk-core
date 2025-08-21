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
#include "ti_clocktree_pll_config.h"
#include <drivers/bootloader.h>
#include <security/security_common/drivers/hsmclient/hsmclient.h>
#include <drivers/bootloader/bootloader_can.h>
#include <drivers/bootloader/bootloader_uniflash/bootloader_uniflash.h>
#include <security/security_common/drivers/hsmclient/soc/am263px/hsmRtImg.h> /* hsmRt bin   header file */

const uint8_t gHsmRtFw[HSMRT_IMG_SIZE_IN_BYTES] __attribute__((section(".rodata.hsmrt"))) = HSMRT_IMG;

extern HsmClient_t gHSMClient ;

#define BOOTLOADER_UNIFLASH_MAX_FILE_SIZE (0x170000) /* This has to match the size of MSRAM1 section in linker.cmd */
uint8_t gUniflashFileBuf[BOOTLOADER_UNIFLASH_MAX_FILE_SIZE] __attribute__((aligned(128), section(".bss.filebuf")));

#define BOOTLOADER_UNIFLASH_VERIFY_BUF_MAX_SIZE (32*1024)
uint8_t gUniflashVerifyBuf[BOOTLOADER_UNIFLASH_VERIFY_BUF_MAX_SIZE] __attribute__((aligned(128), section(".bss")));

uint32_t gRunApp;
extern Flash_Config gFlashConfig[CONFIG_FLASH_NUM_INSTANCES];

void flashFixUpOspiBoot(OSPI_Handle oHandle);
void board_flash_reset(OSPI_Handle oHandle);
void mcanEnableTransceiver(void);

/* call this API to stop the booting process and spin, do that you can connect
 * debugger, load symbols and then make the 'loop' variable as 0 to continue execution
 * with debugger connected.
 */
void loop_forever(void)
{
    volatile uint32_t loop = 1;
    while (loop)
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

int main(void)
{
    int32_t status;
    uint32_t done = 0U;
    uint32_t fileSize = 0U;
    Bootloader_UniflashConfig uniflashConfig;
    Bootloader_UniflashResponseHeader respHeader;

    Bootloader_profileReset();
    Bootloader_socConfigurePll();
    Bootloader_socSetAutoClock();

    System_init();
    Bootloader_profileAddProfilePoint("System_init");

    Drivers_open();
    Bootloader_profileAddProfilePoint("Drivers_open");

    DebugP_log("\r\n");
    Bootloader_socLoadHsmRtFw(&gHSMClient, gHsmRtFw, HSMRT_IMG_SIZE_IN_BYTES);
    Bootloader_socInitL2MailBoxMemory();
    Bootloader_profileAddProfilePoint("LoadHsmRtFw");

    status = Keyring_init(&gHSMClient);
    DebugP_assert(status == SystemP_SUCCESS);

    flashFixUpOspiBoot(gOspiHandle[CONFIG_OSPI0]);

    status = Board_driversOpen();
    DebugP_assert(status == SystemP_SUCCESS);
    Bootloader_profileAddProfilePoint("Board_driversOpen");

    mcanEnableTransceiver();
    Bootloader_CANInit(CONFIG_MCAN0_BASE_ADDR);

    DebugP_log("\r\n");
    DebugP_log("Starting CAN Flashwriter...\r\n");

    while(!done)
    {
        /* CAN Receive */
        status = Bootloader_CANReceiveFile(&fileSize, gUniflashFileBuf, &gRunApp);

        if(fileSize >= BOOTLOADER_UNIFLASH_MAX_FILE_SIZE)
        {
            /* Possible overflow, send error to host side */
            status = SystemP_FAILURE;

            respHeader.magicNumber = BOOTLOADER_UNIFLASH_RESP_HEADER_MAGIC_NUMBER;
            respHeader.statusCode = BOOTLOADER_UNIFLASH_STATUSCODE_FLASH_ERROR;

            Bootloader_CANTransmitResp((uint8_t *)&respHeader);
        }

        if(status == SystemP_SUCCESS)
        {
            uniflashConfig.flashIndex = CONFIG_FLASH0;
            uniflashConfig.buf = gUniflashFileBuf;
            uniflashConfig.bufSize = 0; /* Actual fileSize will be parsed from the header */
            uniflashConfig.verifyBuf = gUniflashVerifyBuf;
            uniflashConfig.verifyBufSize = BOOTLOADER_UNIFLASH_VERIFY_BUF_MAX_SIZE;

            /* Process the flash commands and return a response */
            Bootloader_uniflashProcessFlashCommands(&uniflashConfig, &respHeader);
            status = Bootloader_CANTransmitResp((uint8_t *)&respHeader);
            done = 1U;
        }
    }

    DebugP_log("\r\n");

    if(SystemP_SUCCESS == status && gRunApp == CSL_TRUE)
    {
        Bootloader_BootImageInfo bootImageInfo;
        Bootloader_Params bootParams;
        Bootloader_Handle bootHandle;

        Bootloader_Params_init(&bootParams);
        Bootloader_BootImageInfo_init(&bootImageInfo);

        bootHandle = Bootloader_open(CONFIG_BOOTLOADER0, &bootParams);

        if(bootHandle != NULL)
        {
            status = Bootloader_parseAndLoadMultiCoreELF(bootHandle, &bootImageInfo);

			Bootloader_profileAddProfilePoint("CPU load");
            OSPI_Handle ospiHandle = OSPI_getHandle(CONFIG_OSPI0);

            if (status == SystemP_SUCCESS)
            {
                /* enable Phy and Phy pipeline for XIP execution */
                if (OSPI_isPhyEnable(gOspiHandle[CONFIG_OSPI0]))
                {
                    status = OSPI_enablePhy(gOspiHandle[CONFIG_OSPI0]);
                    DebugP_assert(status == SystemP_SUCCESS);

                    status = OSPI_enablePhyPipeline(gOspiHandle[CONFIG_OSPI0]);
                    DebugP_assert(status == SystemP_SUCCESS);

                    status = OSPI_enableDacMode(gOspiHandle[CONFIG_OSPI0]);
                    DebugP_assert(status == SystemP_SUCCESS);
                }
            }

            /* Run CPUs */
            if(status == SystemP_SUCCESS && (TRUE == Bootloader_isCorePresent(bootHandle, CSL_CORE_ID_R5FSS1_1)))
            {
                status = Bootloader_runCpu(bootHandle, &bootImageInfo.cpuInfo[CSL_CORE_ID_R5FSS1_1]);
            }
            if(status == SystemP_SUCCESS && (TRUE == Bootloader_isCorePresent(bootHandle, CSL_CORE_ID_R5FSS1_0)))
            {
                status = Bootloader_runCpu(bootHandle, &bootImageInfo.cpuInfo[CSL_CORE_ID_R5FSS1_0]);
            }
            if(status == SystemP_SUCCESS && (TRUE == Bootloader_isCorePresent(bootHandle, CSL_CORE_ID_R5FSS0_1)))
            {
                status = Bootloader_runCpu(bootHandle, &bootImageInfo.cpuInfo[CSL_CORE_ID_R5FSS0_1]);
            }
            if(status == SystemP_SUCCESS)
            {
                /* enable Phy and Phy pipeline for XIP execution */
                if (OSPI_isPhyEnable(gOspiHandle[CONFIG_OSPI0]))
                {
                    status = OSPI_enablePhy(gOspiHandle[CONFIG_OSPI0]);
                    DebugP_assert(status == SystemP_SUCCESS);

                    status = OSPI_enablePhyPipeline(gOspiHandle[CONFIG_OSPI0]);
                    DebugP_assert(status == SystemP_SUCCESS);

                    status = OSPI_enableDacMode(gOspiHandle[CONFIG_OSPI0]);
                    DebugP_assert(status == SystemP_SUCCESS);
                }
                if(status == SystemP_SUCCESS)
                {
                    Bootloader_profileUpdateAppimageSize(Bootloader_getMulticoreImageSize(bootHandle));
                    Bootloader_profileUpdateMediaAndClk(BOOTLOADER_MEDIA_FLASH, OSPI_getInputClk(ospiHandle));
                    Bootloader_profileAddProfilePoint("SBL End");
                    Bootloader_profilePrintProfileLog();
                    DebugP_log("Image loading done, switching to application ...\r\n");
                    UART_flushTxFifo(gUartHandle[CONFIG_UART0]);
                }
                /* If any of the R5 core 0 have valid image reset the R5 core. */
                status = Bootloader_runSelfCpu(bootHandle, &bootImageInfo);
            }

            /* it should not return here, if it does, then there was some error */
            Bootloader_close(bootHandle);
        }
    }
    if (status != SystemP_SUCCESS)
    {
        DebugP_log("Some tests have failed!!\r\n");
    }

    Drivers_close();
    System_deinit();

    return 0;
}

void flashFixUpOspiBoot(OSPI_Handle oHandle)
{
    board_flash_reset(oHandle);
    OSPI_enableSDR(oHandle);
    OSPI_clearDualOpCodeMode(oHandle);
    OSPI_setProtocol(oHandle, OSPI_NOR_PROTOCOL(1,1,1,0));
}
