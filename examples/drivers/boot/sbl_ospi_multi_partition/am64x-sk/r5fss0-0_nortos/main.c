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
#include "ti_drivers_config.h"
#include "ti_drivers_open_close.h"
#include "ti_board_open_close.h"
#include <drivers/sciclient.h>
#include <drivers/bootloader.h>

/*  In this sample bootloader, we load each CPU from different flash offsets instead of a single offset.
    i.e each CPU is flash at different segments or partitions of the flash.

    Note, here, partition refers to different offsets, NOT file system or such partitions.

    Here at each flash offset, there is a multi-core .appimage but it holds RPRC
    for only one CPU vs holding RPRC for all CPUs.

    This allows end user to independently update each CPU's binary without having to
    update other CPUs.

    When flashing make sure to flash images to below offset using the flash tool.
    Note, these offsets can be change via SysConfig.

    R5FSS0-0 flash at offset 0x80000
    R5FSS0-1 flash at offset 0x100000
    R5FSS1-0 flash at offset 0x180000
    R5FSS1-1 flash at offset 0x200000
    M4FSS0-0 flash at offset 0x280000
    A53SS0-0 flash at offset 0x300000
 */

void flashFixUpOspiBoot(OSPI_Handle oHandle, Flash_Handle fHandle);

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

/* boot, i.e load and run CPUs not in self CPU cluster */
int32_t App_bootCpu(uint32_t bootDrvInstanceId, uint32_t cpuId)
{
    int32_t status = SystemP_FAILURE;
    Bootloader_BootImageInfo bootImageInfo;
    Bootloader_Params bootParams;
    Bootloader_Handle bootHandle;

    Bootloader_Params_init(&bootParams);
    Bootloader_BootImageInfo_init(&bootImageInfo);

    bootHandle = Bootloader_open(bootDrvInstanceId, &bootParams);
    if(bootHandle != NULL)
    {
        status = Bootloader_parseMultiCoreAppImage(bootHandle, &bootImageInfo);
        if(status == SystemP_SUCCESS)
        {
            bootImageInfo.cpuInfo[cpuId].clkHz = Bootloader_socCpuGetClkDefault(cpuId);
            status = Bootloader_bootCpu(bootHandle, &bootImageInfo.cpuInfo[cpuId]);
        }
        Bootloader_close(bootHandle);
    }
    return status;
}

/* load but DONT run CPU in self CPU cluster, i.e R5FSS0-0 */
int32_t App_bootLoadSelfCpu(uint32_t bootDrvInstanceId, uint32_t cpuId)
{
    int32_t status = SystemP_FAILURE;
    Bootloader_BootImageInfo bootImageInfo;
    Bootloader_Handle bootHandle;
    Bootloader_Params bootParams;

    Bootloader_Params_init(&bootParams);
    Bootloader_BootImageInfo_init(&bootImageInfo);

    bootHandle = Bootloader_open(bootDrvInstanceId, &bootParams);
    if(bootHandle != NULL)
    {
        status = Bootloader_parseMultiCoreAppImage(bootHandle, &bootImageInfo);
        if(status == SystemP_SUCCESS)
        {
            bootImageInfo.cpuInfo[cpuId].clkHz = Bootloader_socCpuGetClkDefault(cpuId);
            status = Bootloader_loadSelfCpu( bootHandle, &bootImageInfo.cpuInfo[cpuId], FALSE);
        }
        Bootloader_close(bootHandle);
    }
    return status;
}

int main(void)
{
    int32_t status;

    Bootloader_socWaitForFWBoot();

#ifndef DISABLE_WARM_REST_WA
    /* Warm Reset Workaround to prevent CPSW register lockup */
    if (!Bootloader_socIsMCUResetIsoEnabled())
    {
        Bootloader_socResetWorkaround();
    }
#endif

    Bootloader_profileAddProfilePoint("SYSFW init");

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

    DebugP_log("\r\n");
    DebugP_log("Starting OSPI Multi-Partition Bootloader ... \r\n");

    /* ROM doesn't reset the OSPI flash. This can make the flash initialization
    troublesome because sequences are very different in Octal DDR mode. So for a
    moment switch OSPI controller to 8D mode and do a flash reset. */
    flashFixUpOspiBoot(gOspiHandle[CONFIG_OSPI0], gFlashHandle[CONFIG_FLASH0]);

    status = Board_driversOpen();
    DebugP_assert(status == SystemP_SUCCESS);

    status = Sciclient_getVersionCheck(1);
    if(SystemP_SUCCESS == status)
    {
        /* load and run all CPUs, if a application is not found, the core is run with a while(1); loop */
        uint32_t coreVariant = Bootloader_socGetCoreVariant();
        if((coreVariant == BOOTLOADER_DEVICE_VARIANT_QUAD_CORE) || (coreVariant == BOOTLOADER_DEVICE_VARIANT_DUAL_CORE))
        {
            if(status == SystemP_SUCCESS)
            {
                status = App_bootCpu( CONFIG_BOOTLOADER_FLASH_R5FSS1_0, CSL_CORE_ID_R5FSS1_0 );
            }
            if((Bootloader_socIsR5FSSDual(BOOTLOADER_R5FSS1) == TRUE) && (status == SystemP_SUCCESS))
            {
                status = App_bootCpu( CONFIG_BOOTLOADER_FLASH_R5FSS1_1, CSL_CORE_ID_R5FSS1_1 );
            }
        }
        if (!Bootloader_socIsMCUResetIsoEnabled())
        {
            if(SystemP_SUCCESS == status)
            {
                status = App_bootCpu( CONFIG_BOOTLOADER_FLASH_M4FSS0_0, CSL_CORE_ID_M4FSS0_0 );
            }
        }
        if(SystemP_SUCCESS == status)
        {
            status = App_bootCpu( CONFIG_BOOTLOADER_FLASH_A53SS0_0, CSL_CORE_ID_A53SS0_0 );
        }
        if(SystemP_SUCCESS == status)
        {
            status = App_bootLoadSelfCpu( CONFIG_BOOTLOADER_FLASH_R5FSS0_0, CSL_CORE_ID_R5FSS0_0 );
        }
        if(SystemP_SUCCESS == status)
        {
            status = App_bootLoadSelfCpu( CONFIG_BOOTLOADER_FLASH_R5FSS0_1, CSL_CORE_ID_R5FSS0_1 );
        }
        if(SystemP_SUCCESS == status)
        {
            /* Reset self cluster and run both the CPUs */
            status = Bootloader_socCpuResetReleaseSelf();
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

void flashFixUpOspiBoot(OSPI_Handle oHandle, Flash_Handle fHandle)
{
    OSPI_setProtocol(oHandle, OSPI_NOR_PROTOCOL(8,8,8,1));
    OSPI_enableDDR(oHandle);
    OSPI_setDualOpCodeMode(oHandle);
    Flash_reset(fHandle);
    OSPI_enableSDR(oHandle);
    OSPI_clearDualOpCodeMode(oHandle);
    OSPI_setProtocol(oHandle, OSPI_NOR_PROTOCOL(1,1,1,0));
}

