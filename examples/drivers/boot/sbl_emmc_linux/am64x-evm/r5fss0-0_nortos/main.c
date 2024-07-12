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
#include <drivers/sciclient.h>
#include <drivers/bootloader.h>

/*  In this sample bootloader, we load appimages for RTOS/Baremetal and Linux at different offset
    i.e the appimage for Linux (for A53) and RTOS/Baremetal (for R5, M4) is flashed at different offset in eMMC

    Here at one eMMC offset, there is a multi-core .appimage that holds RPRC for M4 and R5
    and another .appimage that holds the linux binaries(ATF, OPTEE, A53-SPL) at another offset.

    When flashing make sure to flash images to below offset using the flash tool.

    RTOS/Baremetal appimage (for R5, M4 cores) flash at offset 0x800000 of eMMC
    Linux appimage (for A53) flash at offset 0xA00000 of eMMC
*/

/* This buffer needs to be defined for eMMC boot in case of HS device for
   image authentication
   The size of the buffer should be large enough to accomodate the appimage */
uint8_t gAppimage[0x800000] __attribute__ ((section (".app"), aligned (4096)));

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

int32_t App_loadImages(Bootloader_Handle bootHandle, Bootloader_BootImageInfo *bootImageInfo)
{
    int32_t status = SystemP_FAILURE;

    if(bootHandle != NULL)
    {
        status = Bootloader_parseMultiCoreAppImage(bootHandle, bootImageInfo);

        /* Load CPUs */
        /* Do not load M4 when MCU domain is reset isolated */
        if (!Bootloader_socIsMCUResetIsoEnabled())
        {
            if(status == SystemP_SUCCESS && (TRUE == Bootloader_isCorePresent(bootHandle, CSL_CORE_ID_M4FSS0_0)))
            {
                bootImageInfo->cpuInfo[CSL_CORE_ID_M4FSS0_0].clkHz = Bootloader_socCpuGetClkDefault(CSL_CORE_ID_M4FSS0_0);
                Bootloader_profileAddCore(CSL_CORE_ID_M4FSS0_0);
                status = Bootloader_loadCpu(bootHandle, &bootImageInfo->cpuInfo[CSL_CORE_ID_M4FSS0_0]);
            }
        }
        if(status == SystemP_SUCCESS && (TRUE == Bootloader_isCorePresent(bootHandle, CSL_CORE_ID_R5FSS1_0)))
        {
            bootImageInfo->cpuInfo[CSL_CORE_ID_R5FSS1_0].clkHz = Bootloader_socCpuGetClkDefault(CSL_CORE_ID_R5FSS1_0);
            status = Bootloader_loadCpu(bootHandle, &(bootImageInfo->cpuInfo[CSL_CORE_ID_R5FSS1_0]));
        }
        if(status == SystemP_SUCCESS && (TRUE == Bootloader_isCorePresent(bootHandle, CSL_CORE_ID_R5FSS1_1)))
        {
            bootImageInfo->cpuInfo[CSL_CORE_ID_R5FSS1_1].clkHz = Bootloader_socCpuGetClkDefault(CSL_CORE_ID_R5FSS1_1);
            status = Bootloader_loadCpu(bootHandle, &(bootImageInfo->cpuInfo[CSL_CORE_ID_R5FSS1_1]));
        }

        /* Assume self boot for either of the cores of R50 cluster */
        uint32_t isSelfBoot = FALSE;
        if(TRUE == Bootloader_isCorePresent(bootHandle, CSL_CORE_ID_R5FSS0_0))
        {
            isSelfBoot = TRUE;
        }

        if(TRUE == Bootloader_isCorePresent(bootHandle, CSL_CORE_ID_R5FSS0_1))
        {
            isSelfBoot = TRUE;
        }

        /* Self cores has to be reset together, so check for both */
        if(status == SystemP_SUCCESS && (TRUE == isSelfBoot))
        {
            /* Set clocks for self cluster */
            bootImageInfo->cpuInfo[CSL_CORE_ID_R5FSS0_0].clkHz = Bootloader_socCpuGetClkDefault(CSL_CORE_ID_R5FSS0_0);
            bootImageInfo->cpuInfo[CSL_CORE_ID_R5FSS0_1].clkHz = Bootloader_socCpuGetClkDefault(CSL_CORE_ID_R5FSS0_1);

            /* Reset self cluster, both Core0 and Core 1. Init RAMs and load the app  */
            /* Skip the image load by passing TRUE, so that image load on self core doesnt corrupt the SBLs IVT. Load the image later before the reset release of the self core  */
            status = Bootloader_loadSelfCpu(bootHandle, &bootImageInfo->cpuInfo[CSL_CORE_ID_R5FSS0_0], TRUE);
            if((status == SystemP_SUCCESS) && (TRUE == Bootloader_socIsR5FSSDual(BOOTLOADER_R5FSS0)))
            {
                status = Bootloader_loadSelfCpu(bootHandle, &bootImageInfo->cpuInfo[CSL_CORE_ID_R5FSS0_1], FALSE);
            }
        }
    }

    return status;
}

int32_t App_loadLinuxImages(Bootloader_Handle bootHandle, Bootloader_BootImageInfo *bootImageInfo)
{
    int32_t status = SystemP_FAILURE;

    if(bootHandle != NULL)
    {
        status = Bootloader_parseAndLoadLinuxAppImage(bootHandle, bootImageInfo);

        if(status == SystemP_SUCCESS)
        {
            bootImageInfo->cpuInfo[CSL_CORE_ID_A53SS0_0].clkHz = Bootloader_socCpuGetClkDefault(CSL_CORE_ID_A53SS0_0);
            status = Bootloader_loadCpu(bootHandle, &(bootImageInfo->cpuInfo[CSL_CORE_ID_A53SS0_0]));
        }
    }

    return status;
}

int32_t App_runCpus(Bootloader_Handle bootHandle, Bootloader_BootImageInfo *bootImageInfo)
{
    int32_t status = SystemP_SUCCESS;

    /* Do not run M4 when MCU domain is reset isolated */
    if (!Bootloader_socIsMCUResetIsoEnabled())
    {
        if(TRUE == Bootloader_isCorePresent(bootHandle, CSL_CORE_ID_M4FSS0_0))
        {
            status = Bootloader_runCpu(bootHandle, &bootImageInfo->cpuInfo[CSL_CORE_ID_M4FSS0_0]);
        }
    }

    if(status == SystemP_SUCCESS && (TRUE == Bootloader_isCorePresent(bootHandle, CSL_CORE_ID_R5FSS1_0)))
    {
        status = Bootloader_runCpu(bootHandle, &(bootImageInfo->cpuInfo[CSL_CORE_ID_R5FSS1_0]));
    }
    if(status == SystemP_SUCCESS && (TRUE == Bootloader_isCorePresent(bootHandle, CSL_CORE_ID_R5FSS1_1)))
	{
        status = Bootloader_runCpu(bootHandle, &(bootImageInfo->cpuInfo[CSL_CORE_ID_R5FSS1_1]));
    }
    return status;
}

int32_t App_runLinuxCpu(Bootloader_Handle bootHandle, Bootloader_BootImageInfo *bootImageInfo)
{
    int32_t status = SystemP_FAILURE;

    /* Initialize GTC by enabling using Syscfg */

    /* Unlock all the control MMRs. Linux/U-boot expects all the MMRs to be unlocked */
    SOC_unlockAllMMR();

    status = Bootloader_runCpu(bootHandle, &(bootImageInfo->cpuInfo[CSL_CORE_ID_A53SS0_0]));

    return status;
}

int main(void)
{
    int32_t status;

    Bootloader_profileReset();

    Bootloader_socWaitForFWBoot();

    /* On a warm reset in eMMC boot mode ROM is not booting the image from eMMC again */
    /* Hence removing the workaround for eMMC boot */
#ifndef DISABLE_WARM_REST_WA
    /* Warm Reset Workaround to prevent CPSW register lockup */
    if (!Bootloader_socIsMCUResetIsoEnabled())
    {
        Bootloader_socResetWorkaround();
    }
#endif

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
    Bootloader_profileAddProfilePoint("System_init");

    Drivers_open();
    Bootloader_profileAddProfilePoint("Drivers_open");

    status = Board_driversOpen();
    DebugP_assert(status == SystemP_SUCCESS);
    Bootloader_profileAddProfilePoint("Board_driversOpen");

    status = Sciclient_getVersionCheck(1);

    if(SystemP_SUCCESS == status)
    {
        Bootloader_BootImageInfo bootImageInfo;
        Bootloader_Params bootParams;
        Bootloader_Handle bootHandle;
        Bootloader_Config *bootConfig;

        Bootloader_BootImageInfo bootImageInfoLinux;
        Bootloader_Params bootParamsLinux;
        Bootloader_Handle bootHandleLinux;
        Bootloader_Config *bootConfigLinux;

        Bootloader_Params_init(&bootParams);
        Bootloader_Params_init(&bootParamsLinux);

        Bootloader_BootImageInfo_init(&bootImageInfo);
        Bootloader_BootImageInfo_init(&bootImageInfoLinux);

        bootHandle = Bootloader_open(CONFIG_BOOTLOADER_EMMC0, &bootParams);
        bootHandleLinux = Bootloader_open(CONFIG_BOOTLOADER_EMMMC_LINUX, &bootParamsLinux);

        if(bootHandleLinux != NULL)
        {
            bootConfigLinux = (Bootloader_Config *)bootHandleLinux;
            bootConfigLinux->scratchMemPtr = gAppimage;

            status = App_loadLinuxImages(bootHandleLinux, &bootImageInfoLinux);
            Bootloader_profileAddProfilePoint("App_loadLinuxImages");
        }

        if(SystemP_SUCCESS == status)
        {
            if(bootHandle != NULL)
            {
                /* Initialize PRU Cores if applicable */
                Bootloader_Config *cfg = (Bootloader_Config *)bootHandle;
                if(TRUE == cfg->initICSSCores)
                {
                    status = Bootloader_socEnableICSSCores(BOOTLOADER_ICSS_CORE_DEFAULT_FREQUENCY);
                    DebugP_assert(status == SystemP_SUCCESS);
                }

                bootConfig = (Bootloader_Config *)bootHandle;
                bootConfig->scratchMemPtr = gAppimage;

                status = App_loadImages(bootHandle, &bootImageInfo);
                Bootloader_profileAddProfilePoint("App_loadImages");
            }

        }

        if(SystemP_SUCCESS == status)
        {
            /* Print SBL log as Linux prints log to the same UART port */
            Bootloader_profilePrintProfileLog();
            DebugP_log("Image loading done, switching to application ...\r\n");
            DebugP_log("Starting linux and RTOS/Baremetal applications\r\n\n");
            UART_flushTxFifo(gUartHandle[CONFIG_UART0]);
        }

        if(SystemP_SUCCESS == status)
        {
            status = App_runLinuxCpu(bootHandleLinux, &bootImageInfoLinux);
        }

        Bootloader_close(bootHandleLinux);

        if(SystemP_SUCCESS == status)
        {
            status = App_runCpus(bootHandle, &bootImageInfo);
        }

        if(SystemP_SUCCESS == status)
        {
            /* Load the image on self core now */
            if( bootImageInfo.cpuInfo[CSL_CORE_ID_R5FSS0_0].rprcOffset != BOOTLOADER_INVALID_ID)
            {
                status = Bootloader_rprcImageLoad(bootHandle, &bootImageInfo.cpuInfo[CSL_CORE_ID_R5FSS0_0]);
            }
            status = Bootloader_runSelfCpuWithLinux();
        }

        Bootloader_close(bootHandle);
    }

    if(status != SystemP_SUCCESS )
    {
        DebugP_log("Some tests have failed!!\r\n");
    }

    Drivers_close();
    System_deinit();

    return 0;
}
