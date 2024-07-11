/*
 *  Copyright (C) 2024 Texas Instruments Incorporated
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

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */
#include <stdlib.h>
#include "ti_drivers_config.h"
#include "ti_drivers_open_close.h"
#include "ti_board_open_close.h"
#include <drivers/sciclient.h>
#include <drivers/bootloader/soc/bootloader_soc.h>
#include <drivers/bootloader.h>
#include <drivers/bootloader/bootloader_xmodem.h>
#include <drivers/bootloader/bootloader_uniflash/bootloader_uniflash.h>
#include <board/flash.h>

#ifdef HS
#include <drivers/sciclient/sciclient_default_boardcfg/am65x/sciclient_boardcfg_hs.h>
#else
#include <drivers/sciclient/sciclient_default_boardcfg/am65x/sciclient_boardcfg.h>
#endif

/* ========================================================================== */
/*                             Macros & Typedefs                              */
/* ========================================================================== */
#define OSPI_OFFSET_SYSFW           (0x80000U)
#define SBL_SYSFW_MAX_SIZE          (0x42000U)

uint8_t gSysImageBuf[SBL_SYSFW_MAX_SIZE] __attribute__((aligned(128), section(".firmware")));

/* ========================================================================== */
/*                 Internal Function Declarations                             */
/* ========================================================================== */
int32_t Bootloader_ReadSysfwImage();
int32_t Bootloader_loadFirmware();
static void Bootloader_sciclientBoardCfg();

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */
/* call this API to stop the booting process and spin, do that you can connect
 * debugger, load symbols and then make the 'loop' variable as 0 to continue execution
 * with debugger connected.
 */
void loop_forever(void)
{
    volatile uint32_t loop = 1;
    while(loop){
        ;
    }
    }

int main(void)
{
    int32_t status = SystemP_FAILURE;

    System_init();
    Drivers_open();

    status = Board_driversOpen();
    DebugP_assert(status == SystemP_SUCCESS);

    status = Bootloader_ReadSysfwImage();
    if (SystemP_SUCCESS == status)
    {
        status = Bootloader_loadFirmware();
    }
    if (SystemP_SUCCESS == status)
    {
        Bootloader_sciclientBoardCfg();
    }

    System_lateInit();

    DebugP_log("\r\n");
    DebugP_log("Starting NULL Bootloader ... \r\n");

    status = Sciclient_getVersionCheck(1);
    if(status == SystemP_SUCCESS)
    {
        Bootloader_Params bootParams;
        Bootloader_BootImageInfo bootImageInfo;
        Bootloader_Handle bootHandle;

        Bootloader_Params_init(&bootParams);
        /* set default which will basically allow to simply power on reset the CPU and run a while(1) loop */
        Bootloader_BootImageInfo_init(&bootImageInfo);

        bootHandle = Bootloader_open(CONFIG_BOOTLOADER_0, &bootParams);
        if(bootHandle != NULL)
        {
            if(status == SystemP_SUCCESS)
            {
                /* Reset self cluster, both Core0 and Core 1. Init RAMs and run dummy loop  */
                status = Bootloader_bootSelfCpu(bootHandle, &bootImageInfo);
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

int32_t Bootloader_ReadSysfwImage()
{
    int32_t status;

    status = Flash_read(gFlashHandle[CONFIG_FLASH0], OSPI_OFFSET_SYSFW, gSysImageBuf, SBL_SYSFW_MAX_SIZE);
    if(SystemP_SUCCESS != status)
    {
        status = SystemP_FAILURE;
    }

    return status;
}

int32_t Bootloader_loadFirmware()
{
    int32_t status = SystemP_FAILURE;
    uint32_t size = SBL_SYSFW_MAX_SIZE;

    size = BOOTLOADER_ALIGN_SIZE(SBL_SYSFW_MAX_SIZE);

    CacheP_wbInv((uint8_t *)gSysImageBuf, size, CacheP_TYPE_ALL);
    status = Sciclient_loadFirmware((const uint32_t *)gSysImageBuf);

    return status;
}

static void Bootloader_sciclientBoardCfg()
{
    int32_t status = SystemP_FAILURE;

    static uint8_t boardCfgLow[] = SCICLIENT_BOARDCFG;
    Sciclient_BoardCfgPrms_t boardCfgPrms =
    {
        .boardConfigLow = (uint32_t) &boardCfgLow,
        .boardConfigHigh = 0,
        .boardConfigSize = SCICLIENT_BOARDCFG_SIZE_IN_BYTES,
        .devGrp = DEVGRP_ALL,
    };
    status = Sciclient_boardCfg(&boardCfgPrms);
    if (status != SystemP_SUCCESS)
    {
        DebugP_logError("[SCICLIENT] Sciclient Common Board Configuration has failed \r\n");
    }

    if(SystemP_SUCCESS == status)
    {
         static uint8_t boardCfgLow[] = SCICLIENT_BOARDCFG_PM;
         Sciclient_BoardCfgPrms_t boardCfgPrms_pm =
         {
            .boardConfigLow = (uint32_t)boardCfgLow,
            .boardConfigHigh = 0,
            .boardConfigSize = SCICLIENT_BOARDCFG_PM_SIZE_IN_BYTES,
            .devGrp = DEVGRP_ALL,
         };
         status = Sciclient_boardCfgPm(&boardCfgPrms_pm);
         if (status != SystemP_SUCCESS)
         {
             DebugP_logError("[SCICLIENT] PM Board Configuration has failed \r\n");
         }
    }

    if (status == SystemP_SUCCESS)
    {
        static uint8_t boardCfgLow[] = SCICLIENT_BOARDCFG_SECURITY;
        Sciclient_BoardCfgPrms_t boardCfgPrms_sec =
        {
            .boardConfigLow = (uint32_t)boardCfgLow,
            .boardConfigHigh = 0,
            .boardConfigSize = SCICLIENT_BOARDCFG_SECURITY_SIZE_IN_BYTES,
            .devGrp = DEVGRP_ALL,
        };
        status = Sciclient_boardCfgSec(&boardCfgPrms_sec) ;
        if (status != SystemP_SUCCESS)
        {
            DebugP_logError("[SCICLIENT] Security Board Configuration has failed \r\n");
        }
    }
    status = Bootloader_socOpenFirewalls();

    Bootloader_socNotifyFirewallOpen();

    DebugP_assertNoLog(status == SystemP_SUCCESS);

    if (SystemP_SUCCESS == status)
    {
        static uint8_t boardCfgLow[] = SCICLIENT_BOARDCFG_RM;
        Sciclient_BoardCfgPrms_t boardCfgPrms_rm =
        {
            .boardConfigLow = (uint32_t)boardCfgLow,
            .boardConfigHigh = 0,
            .boardConfigSize = SCICLIENT_BOARDCFG_RM_SIZE_IN_BYTES,
            .devGrp = DEVGRP_ALL,
        };
        status = Sciclient_boardCfgRm(&boardCfgPrms_rm);
        if (status != SystemP_SUCCESS)
        {
            DebugP_logError("[SCICLIENT] RM Board Configuration has failed \r\n");
        }
    }

    /* RTI seems to be turned on by ROM. Turning it off so that Power domain can transition */
    Sciclient_pmSetModuleState(134, TISCI_MSG_VALUE_DEVICE_SW_STATE_AUTO_OFF, TISCI_MSG_FLAG_AOP, SystemP_WAIT_FOREVER);
    Sciclient_pmSetModuleState(135, TISCI_MSG_VALUE_DEVICE_SW_STATE_AUTO_OFF, TISCI_MSG_FLAG_AOP, SystemP_WAIT_FOREVER);
}

