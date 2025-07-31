
/*
 *  Copyright (C) 2018-2025 Texas Instruments Incorporated
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
 * the received application image to 0x80000 location in the QSPI Flash and
 * attempts to boot the same multicore appimage present at 0x80000 location
 * in the QSPI Flash after successful completion of the image transferred
 * over ethernet.
 *
 */
#include <stdlib.h>
#include <string.h>
#include "ti_drivers_config.h"
#include "ti_drivers_open_close.h"
#include "ti_board_open_close.h"
#include "ti_clocktree_pll_config.h"
#include <drivers/bootloader.h>
#include <drivers/bootloader/bootloader_uniflash/bootloader_uniflash.h>
#include <kernel/dpl/DebugP.h>
#include "sbl_enet.h"
#include <security/security_common/drivers/hsmclient/hsmclient.h>
#include <security/security_common/drivers/hsmclient/soc/am263px/hsmRtImg.h> /* hsmRt bin   header file */
#include <drivers/ospi.h>
#include <drivers/fss.h>

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

#define BOOTLOADER_UNIFLASH_FLASH_REMAP     (0xABCDABCD)

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */

uint32_t gGpioBaseAddr = ENET_TRANSFER_START_BTN_BASE_ADDR;
uint32_t pinNum = ENET_TRANSFER_START_BTN_PIN;

const uint8_t gHsmRtFw[HSMRT_IMG_SIZE_IN_BYTES] __attribute__((section(".rodata.hsmrt"))) = HSMRT_IMG;

extern HsmClient_t gHSMClient;

/*
 * This function will receive app image from enet_uniflash.py script over Ethernet
 */
uint32_t receiveAppImgOverEnet();

/**
 * @brief Reset that flash to start from known default state.
 * 
 * @param oHandle OSPI handle
 */
void flashFixUpOspiBoot(OSPI_Handle oHandle);

/**
 * @brief Does the actual reset of the flash on board
 * 
 */
void board_flash_reset(OSPI_Handle oHandle);

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
    Bootloader_profileReset();
    Bootloader_socConfigurePll();
    Bootloader_socSetAutoClock();

    System_init();
    Bootloader_profileAddProfilePoint("System_init");
    Drivers_open();
    Bootloader_profileAddProfilePoint("Drivers_open");
    
    Bootloader_socLoadHsmRtFw(&gHSMClient, gHsmRtFw, HSMRT_IMG_SIZE_IN_BYTES);
    Bootloader_socInitL2MailBoxMemory();
    Bootloader_profileAddProfilePoint("LoadHsmRtFw");

    status = Keyring_init(&gHSMClient);
    DebugP_assert(status == SystemP_SUCCESS);

    /* ROM doesn't reset the OSPI flash. This can make the flash initialization
    troublesome because sequences are very different in Octal DDR mode. So for a
    moment switch OSPI controller to 8D mode and do a flash reset. */
    flashFixUpOspiBoot(gOspiHandle[CONFIG_OSPI0]);

    status = Board_driversOpen();
    DebugP_assert(status == SystemP_SUCCESS); 
    Bootloader_profileAddProfilePoint("Board_driversOpen");

    DebugP_log("\r\nStarting OSPI Bootloader ... \r\n");

    /* Receive application image via UDP over ethernet */
    status = receiveAppImgOverEnet();

    if (SystemP_SUCCESS == status)
    {
        Bootloader_BootImageInfo bootImageInfo;
        Bootloader_Params bootParams;
        Bootloader_Handle bootHandle;

        Bootloader_Params_init(&bootParams);
        Bootloader_BootImageInfo_init(&bootImageInfo);

        bootHandle = Bootloader_open(CONFIG_BOOTLOADER0, &bootParams);

        if (bootHandle != NULL)
        {
            status = Bootloader_parseMultiCoreAppImage(bootHandle, &bootImageInfo);
            OSPI_enableDacMode(gOspiHandle[CONFIG_OSPI0]);


            /* Initialize CPUs and Load RPRC Image */
            if ((status == SystemP_SUCCESS) && (TRUE == Bootloader_isCorePresent(bootHandle, CSL_CORE_ID_R5FSS1_1)))
            {
                bootImageInfo.cpuInfo[CSL_CORE_ID_R5FSS1_1].clkHz = Bootloader_socCpuGetClkDefault(CSL_CORE_ID_R5FSS1_1);
                Bootloader_profileAddCore(CSL_CORE_ID_R5FSS1_1);
                status = Bootloader_initCpu(bootHandle, &bootImageInfo.cpuInfo[CSL_CORE_ID_R5FSS1_1]);

				if ((status == SystemP_SUCCESS) && (bootImageInfo.cpuInfo[CSL_CORE_ID_R5FSS1_1].rprcOffset != BOOTLOADER_INVALID_ID)) {
					status = Bootloader_rprcImageLoad(bootHandle, &bootImageInfo.cpuInfo[CSL_CORE_ID_R5FSS1_1]);
				}
            }
            if ((status == SystemP_SUCCESS) && (TRUE == Bootloader_isCorePresent(bootHandle, CSL_CORE_ID_R5FSS1_0)))
            {
                bootImageInfo.cpuInfo[CSL_CORE_ID_R5FSS1_0].clkHz = Bootloader_socCpuGetClkDefault(CSL_CORE_ID_R5FSS1_0);
                Bootloader_profileAddCore(CSL_CORE_ID_R5FSS1_0);
                status = Bootloader_initCpu(bootHandle, &bootImageInfo.cpuInfo[CSL_CORE_ID_R5FSS1_0]);

				if ((status == SystemP_SUCCESS) && (bootImageInfo.cpuInfo[CSL_CORE_ID_R5FSS1_0].rprcOffset != BOOTLOADER_INVALID_ID)) {
					status = Bootloader_rprcImageLoad(bootHandle, &bootImageInfo.cpuInfo[CSL_CORE_ID_R5FSS1_0]);
				}
            }
            if ((status == SystemP_SUCCESS) && (TRUE == Bootloader_isCorePresent(bootHandle, CSL_CORE_ID_R5FSS0_1)))
            {
                bootImageInfo.cpuInfo[CSL_CORE_ID_R5FSS0_1].clkHz = Bootloader_socCpuGetClkDefault(CSL_CORE_ID_R5FSS0_1);
                Bootloader_profileAddCore(CSL_CORE_ID_R5FSS0_1);
                status = Bootloader_initCpu(bootHandle, &bootImageInfo.cpuInfo[CSL_CORE_ID_R5FSS0_1]);

				if ((status == SystemP_SUCCESS) && (bootImageInfo.cpuInfo[CSL_CORE_ID_R5FSS0_1].rprcOffset != BOOTLOADER_INVALID_ID)) {
					status = Bootloader_rprcImageLoad(bootHandle, &bootImageInfo.cpuInfo[CSL_CORE_ID_R5FSS0_1]);
				}
            }
            if ((status == SystemP_SUCCESS) && (TRUE == Bootloader_isCorePresent(bootHandle, CSL_CORE_ID_R5FSS0_0)))
            {
                bootImageInfo.cpuInfo[CSL_CORE_ID_R5FSS0_0].clkHz = Bootloader_socCpuGetClkDefault(CSL_CORE_ID_R5FSS0_0);
                Bootloader_profileAddCore(CSL_CORE_ID_R5FSS0_0);
                status = Bootloader_loadSelfCpu(bootHandle, &bootImageInfo.cpuInfo[CSL_CORE_ID_R5FSS0_0], FALSE);
            }
            Bootloader_profileAddProfilePoint("CPU load");
            OSPI_Handle ospiHandle = OSPI_getHandle(CONFIG_OSPI0);
            Bootloader_profileUpdateAppimageSize(Bootloader_getMulticoreImageSize(bootHandle));
            Bootloader_profileUpdateMediaAndClk(BOOTLOADER_MEDIA_FLASH, OSPI_getInputClk(ospiHandle));
            
            if(status == SystemP_SUCCESS)
            {
                
                Bootloader_profileAddProfilePoint("SBL End");
                Bootloader_profilePrintProfileLog();

                DebugP_log("Image loading done, switching to application ...\r\n");
                UART_flushTxFifo(gUartHandle[CONFIG_UART0]);
            }
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
            if((status == SystemP_SUCCESS) && (TRUE == Bootloader_isCorePresent(bootHandle, CSL_CORE_ID_R5FSS1_1)))
            {
                status = Bootloader_runCpu(bootHandle, &bootImageInfo.cpuInfo[CSL_CORE_ID_R5FSS1_1]);
            }
            if((status == SystemP_SUCCESS) && (TRUE == Bootloader_isCorePresent(bootHandle, CSL_CORE_ID_R5FSS1_0)))
            {
                status = Bootloader_runCpu(bootHandle, &bootImageInfo.cpuInfo[CSL_CORE_ID_R5FSS1_0]);
            }
            if((status == SystemP_SUCCESS) && (TRUE == Bootloader_isCorePresent(bootHandle, CSL_CORE_ID_R5FSS0_1)))
            {
                status = Bootloader_runCpu(bootHandle, &bootImageInfo.cpuInfo[CSL_CORE_ID_R5FSS0_1]);
            }
            if((status == SystemP_SUCCESS) && (TRUE == Bootloader_isCorePresent(bootHandle, CSL_CORE_ID_R5FSS0_0)))
            {
                /* Load the RPRC image on self core now */
                
                status = Bootloader_runSelfCpu(bootHandle, &bootImageInfo);
            }

            /* it should not return here, if it does, then there was some error */
            Bootloader_close(bootHandle);
        }
    }
    if (status != SystemP_SUCCESS)
    {
        DebugP_log("SBL failed!!\r\n");
    }
    Drivers_close();
    System_deinit();

    return 0;
}

/* SEND A UDP PACKET OVER ETHERNET */
uint32_t receiveAppImgOverEnet()
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
        status = SystemP_FAILURE;
    }
#endif

    if(!done)
    {
        DebugP_log("\r\n[ ENETSBL ] Starting Ethernet Transfer ...\r\n");

        /* Initialize the C66x subsystem as the DSS_L3 memory is used to store the file to flash */
        Bootloader_socCpuPowerOnReset(CSL_CORE_ID_R5FSS0_0,NULL_PTR);

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
            status = SystemP_TIMEOUT;
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
                    status = SystemP_FAILURE;
                }
                else
                {
                    pktInfo = (Bootloader_UniflashFileHeader*) &gFlashFileBuf;
                    status = EnetSBL_txFlashResp(respHeader);
                    DebugP_log("[ ENETSBL SUCCESS ] Ethernet Transfer Done.\r\n");
                    DebugP_log("[ ENETSBL ] Packets Received   :  %d \r\n",pktInfo->rsv1);
                    DebugP_log("[ ENETSBL ] Total File Size    :  %d Bytes\r\n",pktInfo->actualFileSize);
                    DebugP_log("[ ENETSBL ] Flash Offset       :  0x%X\r\n\n",pktInfo->offset);
                    status = SystemP_SUCCESS;
                    break;
                }
            }
            else
            {
                DebugP_log("[ ENETSBL TIMEOUT ] Skipping enet transfer.\r\n");
                status = SystemP_FAILURE;
                break;
            }
        }

        /* Close */
        EnetSBL_destruct();
    }
    return status;
}

void flashFixUpOspiBoot(OSPI_Handle oHandle)
{
    board_flash_reset(oHandle);
    OSPI_enableSDR(oHandle);
    OSPI_clearDualOpCodeMode(oHandle);
    OSPI_setProtocol(oHandle, OSPI_NOR_PROTOCOL(1,1,1,0));
}
