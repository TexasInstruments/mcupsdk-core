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
#include "ti_drivers_config.h"
#include "ti_board_config.h"
#include "ti_drivers_open_close.h"
#include "ti_board_open_close.h"
#include <drivers/bootloader/soc/bootloader_soc.h>
#include <drivers/bootloader.h>

extern int32_t sciclient_set_boardcfg(void);
extern int32_t sciclient_load_sysfw(void);

int main(void)
{
    int32_t status = SystemP_FAILURE;
    Bootloader_Params bootParams;
    Bootloader_BootImageInfo bootImageInfo;
    Bootloader_Handle bootHandle;

    status = sciclient_load_sysfw();
    DebugP_assertNoLog(status == SystemP_SUCCESS);

    status = sciclient_set_boardcfg();
    DebugP_assertNoLog(status == SystemP_SUCCESS);

    System_init();
    Board_init();

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

    /* Initialize the CPUs */
    Bootloader_socOpenFirewalls();

    Bootloader_socNotifyFirewallOpen();

    Bootloader_Params_init(&bootParams);
    /* set default which will basically allow to simply power on reset the CPU and run a while(1) loop */
    Bootloader_BootImageInfo_init(&bootImageInfo);

    DebugP_log("Starting SOC Initialization ...\r\n");

    bootHandle = Bootloader_open(0, &bootParams);

    if(bootHandle != NULL)
    {
        /* Initialize PRU Cores if applicable */
        Bootloader_Config *cfg = (Bootloader_Config *)bootHandle;
        if(TRUE == cfg->initICSSCores)
        {
            status = Bootloader_socEnableICSSCores(BOOTLOADER_ICSS_CORE_DEFAULT_FREQUENCY);
            DebugP_assert(status == SystemP_SUCCESS);
        }

        uint32_t coreVariant = Bootloader_socGetCoreVariant();
        if((coreVariant == BOOTLOADER_DEVICE_VARIANT_QUAD_CORE) || (coreVariant == BOOTLOADER_DEVICE_VARIANT_DUAL_CORE))
        {
            if(status == SystemP_SUCCESS)
            {
                status = Bootloader_bootCpu(bootHandle, &bootImageInfo.cpuInfo[CSL_CORE_ID_R5FSS1_0]);
            }
            if((Bootloader_socIsR5FSSDual(BOOTLOADER_R5FSS1) == TRUE) && (status == SystemP_SUCCESS))
            {
                status = Bootloader_bootCpu(bootHandle, &bootImageInfo.cpuInfo[CSL_CORE_ID_R5FSS1_1]);
            }
        }
        /* Do not boot M4 when MCU domain is reset isolated */
        if (!Bootloader_socIsMCUResetIsoEnabled())
        {
            if(status == SystemP_SUCCESS)
            {
                status = Bootloader_bootCpu(bootHandle, &bootImageInfo.cpuInfo[CSL_CORE_ID_M4FSS0_0]);
            }
        }
        if(status == SystemP_SUCCESS)
        {
            status = Bootloader_bootCpu(bootHandle, &bootImageInfo.cpuInfo[CSL_CORE_ID_A53SS0_0]);
        }
        if(status == SystemP_SUCCESS)
        {
            status = Bootloader_bootCpu(bootHandle, &bootImageInfo.cpuInfo[CSL_CORE_ID_A53SS0_1]);
        }
        DebugP_log("Resetting self cluster ...\r\n");
        if(status == SystemP_SUCCESS)
        {
            /* Reset self cluster, both Core0 and Core 1. Init RAMs and run dummy loop  */
            status = Bootloader_bootSelfCpu(bootHandle, &bootImageInfo);
        }
        /* it should not return here, if it does, then there was some error */
        Bootloader_close(bootHandle);
    }

    Board_driversClose();
    Drivers_close();

    Board_deinit();
    System_deinit();

    return 0;
}
