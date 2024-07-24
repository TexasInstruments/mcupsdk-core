/*
 *  Copyright (C) 2023 Texas Instruments Incorporated
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

/**
 *
 * This bootloader does SOC initializations in addition to providing an
 * option to receive an application image via UDP over ethernet and flashing
 * the received application image to 0xA0000 location in the QSPI Flash and
 * attempts to boot the same multicore appimage present at 0xA0000 location
 * in the QSPI Flash after successful completion of the image transferred
 * over ethernet.
 *
 */

#include <stdlib.h>
#include <string.h>
#include "ti_drivers_config.h"
#include "ti_drivers_open_close.h"
#include "ti_board_open_close.h"
#include "ti_board_config.h"
#include <drivers/bootloader.h>
#include <kernel/dpl/DebugP.h>
#include <drivers/bootloader.h>
#include <drivers/bootloader/bootloader_uniflash/bootloader_uniflash.h>
#include "sbl_enet.h"

uint32_t gGpioBaseAddr = ENET_TRANSFER_START_BTN_BASE_ADDR;
uint32_t pinNum = ENET_TRANSFER_START_BTN_PIN;

void receiveAppImgOverEnet();

/* call this API to stop the booting process and spin, do that you can connect
 * debugger, load symbols and then make the 'loop' variable as 0 to continue execution
 * with debugger connected.
 */
void loop_forever(void)
{
    volatile uint32_t loop = 1;
    while(loop);
}

int main(void)
{
    int32_t status;

    Bootloader_profileReset();
    Bootloader_socConfigurePll();

    System_init();
    Bootloader_profileAddProfilePoint("System_init");

    Board_init();

    Drivers_open();
    Bootloader_profileAddProfilePoint("Drivers_open");

    status = Board_driversOpen();
    DebugP_assert(status == SystemP_SUCCESS);
    Bootloader_profileAddProfilePoint("Board_driversOpen");

    /* Receive application image via UDP over ethernet */
    receiveAppImgOverEnet();

    DebugP_log("\r\n");
    DebugP_log("Starting QSPI Bootloader ... \r\n");

    if(SystemP_SUCCESS == status)
    {
        Bootloader_BootImageInfo bootImageInfo;
        Bootloader_Params bootParams;
        Bootloader_Handle bootHandle;

        Bootloader_Params_init(&bootParams);
        Bootloader_BootImageInfo_init(&bootImageInfo);

        bootHandle = Bootloader_open(CONFIG_BOOTLOADER0, &bootParams);
        if(bootHandle != NULL)
        {
            status = Bootloader_parseMultiCoreAppImage(bootHandle, &bootImageInfo);
            /* Load CPUs */
            if((status == SystemP_SUCCESS) && (TRUE == Bootloader_isCorePresent(bootHandle, CSL_CORE_ID_RSS_R4)))
            {
                bootImageInfo.cpuInfo[CSL_CORE_ID_RSS_R4].clkHz = Bootloader_socCpuGetClkDefault(CSL_CORE_ID_RSS_R4);
                Bootloader_profileAddCore(CSL_CORE_ID_RSS_R4);
                status = Bootloader_loadCpu(bootHandle, &bootImageInfo.cpuInfo[CSL_CORE_ID_RSS_R4]);
            }
            if((status == SystemP_SUCCESS) && (TRUE == Bootloader_isCorePresent(bootHandle, CSL_CORE_ID_C66SS0)))
            {
                bootImageInfo.cpuInfo[CSL_CORE_ID_C66SS0].clkHz = Bootloader_socCpuGetClkDefault(CSL_CORE_ID_C66SS0);
                Bootloader_profileAddCore(CSL_CORE_ID_C66SS0);
                status = Bootloader_loadCpu(bootHandle, &bootImageInfo.cpuInfo[CSL_CORE_ID_C66SS0]);
            }
            if ((status == SystemP_SUCCESS) && (TRUE == Bootloader_isCorePresent(bootHandle, CSL_CORE_ID_R5FSS0_1)))
            {
                bootImageInfo.cpuInfo[CSL_CORE_ID_R5FSS0_1].clkHz = Bootloader_socCpuGetClkDefault(CSL_CORE_ID_R5FSS0_1);
                Bootloader_profileAddCore(CSL_CORE_ID_R5FSS0_1);
                status = Bootloader_loadCpu(bootHandle, &bootImageInfo.cpuInfo[CSL_CORE_ID_R5FSS0_1]);
            }
            if((status == SystemP_SUCCESS) && (TRUE == Bootloader_isCorePresent(bootHandle, CSL_CORE_ID_R5FSS0_0)))
            {
                bootImageInfo.cpuInfo[CSL_CORE_ID_R5FSS0_0].clkHz = Bootloader_socCpuGetClkDefault(CSL_CORE_ID_R5FSS0_0);
                Bootloader_profileAddCore(CSL_CORE_ID_R5FSS0_0);
                status = Bootloader_loadCpu(bootHandle, &bootImageInfo.cpuInfo[CSL_CORE_ID_R5FSS0_0]);
            }
            Bootloader_profileAddProfilePoint("CPU load");
            Bootloader_profileUpdateAppimageSize(Bootloader_getMulticoreImageSize(bootHandle));
            QSPI_Handle qspiHandle = QSPI_getHandle(CONFIG_QSPI0);
            Bootloader_profileUpdateMediaAndClk(BOOTLOADER_MEDIA_FLASH, QSPI_getInputClk(qspiHandle));

            if(status == SystemP_SUCCESS)
            {
                Bootloader_profileAddProfilePoint("SBL End");
                Bootloader_profilePrintProfileLog();
                DebugP_log("Image loading done, switching to application ...\r\n");
                UART_flushTxFifo(gUartHandle[CONFIG_UART0]);
            }

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

            if((TRUE == Bootloader_isCorePresent(bootHandle, CSL_CORE_ID_R5FSS0_0)) ||
                (TRUE == Bootloader_isCorePresent(bootHandle, CSL_CORE_ID_R5FSS0_1)))
            {
                /* If any of the R5 core 0 /core 1 have valid image reset the R5 core. */
                status = Bootloader_runSelfCpu(bootHandle, &bootImageInfo);
            }

            /* it should not return here, if it does, then there was some error */
            Bootloader_close(bootHandle);
        }
    }
    if(status != SystemP_SUCCESS )
    {
        DebugP_log("Some tests have failed!!\r\n");
    }
    Drivers_close();
    System_deinit();

    return 0;
}

void receiveAppImgOverEnet()
{
    int32_t status = SystemP_SUCCESS;
    uint8_t done = false;
    Bootloader_UniflashConfig uniflashConfig;
    Bootloader_UniflashResponseHeader respHeader;
    Bootloader_UniflashFileHeader *pktInfo;

#if (ENETSBL_TRANSFER_START_MODE == ENETSBL_BUTTON_MODE)
    /* Check if SW2 is pressed, if not skip the ethernet transfer */
    if(GPIO_pinRead(gGpioBaseAddr,pinNum) == GPIO_INTR_LEVEL_LOW)
    {
        DebugP_log("[ ENETSBL SKIP ] Skipping enet transfer.\r\n");
        done = true;
    }
#endif

    if(!done)
    {
        DebugP_log("\r\n[ ENETSBL ] Starting Ethernet Transfer ...\r\n");

        /* Initialize the C66x subsystem as the DSS_L3 memory is used to store the file to flash */
        Bootloader_socCpuPowerOnReset(CSL_CORE_ID_C66SS0,NULL_PTR);

        /* Initialize sbl_enet config and setup ethernet peripheral */
        memset(&gEnetSBL_LLDObj, 0, sizeof(gEnetSBL_LLDObj));
        memset(&gEnetSBL_MetaObj, 0, sizeof(gEnetSBL_MetaObj));
        memset(&respHeader, 0, sizeof(respHeader));
        memset(&uniflashConfig, 0, sizeof(uniflashConfig));

        status = EnetSBL_setup();

        if(status == ENET_SOK)
        {
            /* Send ACK packet to let host know that EVM is linked up */
            respHeader.magicNumber = ENETSBL_HEADER_MGC_NUMBER;
            respHeader.statusCode = ENETSBL_HEADER_ACK;
            EnetSBL_txFlashResp(respHeader);
        }
        else if(status == ENET_ETIMEOUT)
        {
            DebugP_log("[ ENETSBL TIMEOUT ] Link Up Timeout. Please check ethernet cable connections.\r\n");
            done = true;
        }

        while (!done)
        {
            /* Run SBL application */
            status = EnetSBL_transferAppimage();
            if(gFlashFileSize >= BOOTLOADER_MAX_FILE_SIZE)
            {
                /* Possible overflow, send error to host side */
                status = SystemP_FAILURE;

                respHeader.magicNumber = BOOTLOADER_UNIFLASH_RESP_HEADER_MAGIC_NUMBER;
                respHeader.statusCode = BOOTLOADER_UNIFLASH_STATUSCODE_FLASH_ERROR;

                EnetSBL_txFlashResp(respHeader);

                /* Exit due to possible error */
                done = 1U;
                DebugP_log("[ ENETSBL ERROR ] Overflow detected.\r\n");
                break;
            }

            if(status == ENET_SOK)
            {
                uniflashConfig.flashIndex = CONFIG_FLASH0;
                uniflashConfig.buf = gFlashFileBuf;
                /* Actual fileSize will be parsed from the header */
                uniflashConfig.bufSize = 0;
                uniflashConfig.verifyBuf = gFlashVerifyBuf;
                uniflashConfig.verifyBufSize = BOOTLOADER_VERIFY_MAX_SIZE;

                /* Process the flash commands and return a response */
                status = Bootloader_uniflashProcessFlashCommands(&uniflashConfig, &respHeader);

                /* Exit if error or timeout; Send response to host */
                if (status != SystemP_SUCCESS)
                {
                    DebugP_log("[ ENETSBL ERROR ] Uniflash timeout error.\r\n");
                    done = 1U;
                }
                else
                {
                    pktInfo = (Bootloader_UniflashFileHeader*) &gFlashFileBuf;
                    status = EnetSBL_txFlashResp(respHeader);
                    DebugP_log("[ ENETSBL SUCCESS ] Ethernet Transfer Done.\r\n");
                    DebugP_log("[ ENETSBL ] Packets Received   :  %d \r\n",pktInfo->rsv1);
                    DebugP_log("[ ENETSBL ] Total File Size    :  %d Bytes\r\n",pktInfo->actualFileSize);
                    DebugP_log("[ ENETSBL ] Flash Offset       :  0x%X\r\n\n",pktInfo->offset);
                    break;
                }
            }
            else
            {
                DebugP_log("[ ENETSBL TIMEOUT ] Skipping enet transfer.\r\n");
                break;
            }
        }

        /* Close */
        EnetSBL_destruct();
    }
}
