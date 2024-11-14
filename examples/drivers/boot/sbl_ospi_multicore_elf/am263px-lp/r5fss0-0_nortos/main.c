
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
#include "ti_drivers_config.h"
#include "ti_drivers_open_close.h"
#include "ti_board_open_close.h"
#include <drivers/bootloader.h>
#include <security/security_common/drivers/hsmclient/hsmclient.h>
#include <security/security_common/drivers/hsmclient/soc/am263px/hsmRtImg.h> /* hsmRt bin   header file */
#include <drivers/ospi.h>
#include <drivers/fss.h>
#include <drivers/bootloader/bootloader_elf.h>



const uint8_t gHsmRtFw[HSMRT_IMG_SIZE_IN_BYTES] __attribute__((section(".rodata.hsmrt"))) = HSMRT_IMG;

extern HsmClient_t gHSMClient ;

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
void board_flash_reset(void);

/**
 * @brief Configure OTFA and ECCM module
 * 
 * @param config bootloader config
 */
void OTFAECCM_Config(Bootloader_OtfaConfig *config);

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
    
            OSPI_Handle ospiHandle = OSPI_getHandle(CONFIG_OSPI0);
            status = Bootloader_parseAndLoadMultiCoreELF(bootHandle, &bootImageInfo);
            OSPI_enableDacMode(ospiHandle);
            if(SystemP_SUCCESS == status)
            {
                Bootloader_OtfaConfig otfaConfig;
                status = Bootloader_getOTFAConfigFromNoteSegment(bootHandle, ELF_NOTE_SEGMENT_MAX_SIZE, &otfaConfig);

                if(SystemP_SUCCESS == status)
                {
                    /* OTFA configuration is in NOTE section. */
                    if(TRUE == otfaConfig.isOTFAECCMEnabled)
                    {
                        OTFAECCM_Config(&otfaConfig);
                    }
                }
            }

            Bootloader_profileAddProfilePoint("CPU load");
            if (status == SystemP_SUCCESS)
            {
                /* enable Phy and Phy pipeline for XIP execution */
                if (OSPI_isPhyEnable(gOspiHandle[CONFIG_OSPI0]))
                {
                    status = OSPI_enablePhy(gOspiHandle[CONFIG_OSPI0]);
                    DebugP_assert(status == SystemP_SUCCESS);

                    status = OSPI_enablePhyPipeline(gOspiHandle[CONFIG_OSPI0]);
                    DebugP_assert(status == SystemP_SUCCESS);
                }
            }

            /* Run CPUs */
            if (status == SystemP_SUCCESS)
            {
                status = Bootloader_runCpu(bootHandle, &bootImageInfo.cpuInfo[CSL_CORE_ID_R5FSS1_1]);
            }
            if (status == SystemP_SUCCESS)
            {
                status = Bootloader_runCpu(bootHandle, &bootImageInfo.cpuInfo[CSL_CORE_ID_R5FSS1_0]);
            }
            if (status == SystemP_SUCCESS)
            {
                status = Bootloader_runCpu(bootHandle, &bootImageInfo.cpuInfo[CSL_CORE_ID_R5FSS0_1]);
            }
            if (status == SystemP_SUCCESS)
            {
                /* Load the RPRC image on self core now */
                if(status == SystemP_SUCCESS)
                {
                    OSPI_Handle ospiHandle = OSPI_getHandle(CONFIG_OSPI0);
                    Bootloader_profileUpdateAppimageSize(Bootloader_getMulticoreImageSize(bootHandle));
                    Bootloader_profileUpdateMediaAndClk(BOOTLOADER_MEDIA_FLASH, OSPI_getInputClk(ospiHandle));
                    Bootloader_profileAddProfilePoint("SBL End");
                    Bootloader_profilePrintProfileLog();

                    DebugP_log("Image loading done, switching to application ...\r\n");
                    UART_flushTxFifo(gUartHandle[CONFIG_UART0]);
                }
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

void flashFixUpOspiBoot(OSPI_Handle oHandle)
{
    board_flash_reset();
    OSPI_enableSDR(oHandle);
    OSPI_clearDualOpCodeMode(oHandle);
    OSPI_setProtocol(oHandle, OSPI_NOR_PROTOCOL(1,1,1,0));
}

void OTFAECCM_Config(Bootloader_OtfaConfig *config)
{
    if(NULL != config)
    {
        uint8_t doEnableECC = FALSE;
        FSS_disableECC();
        for(unsigned int i = 0; i < config->regionLen; i++)
        {   
            if(config->region[i].eccEnable == TRUE)
            {
                OSPI_Handle ospiHandle = OSPI_getHandle(CONFIG_OSPI0);
                uint32_t dataBaseAddress = OSPI_getFlashDataBaseAddr(ospiHandle);
                doEnableECC = TRUE;
                FSS_ECCRegionConfig rc;
                rc.size = config->region[i].size;
                rc.startAddress = config->region[i].startAddress - dataBaseAddress;
                rc.regionIndex = i;
                FSS_configECCMRegion(&rc);
            }
        }
        
        if(doEnableECC == TRUE)
        {
            FSS_enableECC();
        }
    }    
}

