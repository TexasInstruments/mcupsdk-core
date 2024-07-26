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
#include <drivers/bootloader.h>
#include <drivers/pinmux.h>
#include <drivers/gtc.h>

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
/* This buffer needs to be defined for Flash boot in case of HS device for
   image authentication
   The size of the buffer should be large enough to accomodate the appimage */
uint8_t gAppimage[0x800000] __attribute__ ((section (".app"), aligned (4096)));

/* Workaround to initialize MMC SD Pinmux (Later will be done in SPL) */
static Pinmux_PerCfg_t gPinMuxMMCSDCfg[] = {
    /* MMC1_CMD -> MMC1_CMD (C28) */
    {
        PIN_MMC1_CMD,
        ( PIN_MODE(0) | PIN_INPUT_ENABLE | PIN_PULL_DIRECTION )
    },
    /* MMC1_CLK -> MMC1_CLK (C27) */
    {
        PIN_MMC1_CLK,
        ( PIN_MODE(0) | PIN_INPUT_ENABLE | PIN_PULL_DISABLE )
    },
    /* MMC1_DAT0 -> MMC1_DAT0 (D28) */
    {
        PIN_MMC1_DAT0,
        ( PIN_MODE(0) | PIN_INPUT_ENABLE | PIN_PULL_DIRECTION )
    },
    /* MMC1_DAT1 -> MMC1_DAT1 (E27) */
    {
        PIN_MMC1_DAT1,
        ( PIN_MODE(0) | PIN_INPUT_ENABLE | PIN_PULL_DIRECTION )
    },
    /* MMC1_DAT2 -> MMC1_DAT2 (D26) */
    {
        PIN_MMC1_DAT2,
        ( PIN_MODE(0) | PIN_INPUT_ENABLE | PIN_PULL_DIRECTION )
    },
    /* MMC1_DAT3 -> MMC1_DAT3 (D27) */
    {
        PIN_MMC1_DAT3,
        ( PIN_MODE(0) | PIN_INPUT_ENABLE | PIN_PULL_DIRECTION )
    },
    /* MMC1_SDCD -> MMC1_SDCD (B24) */
    {
        PIN_MMC1_SDCD,
        ( PIN_MODE(0) | PIN_INPUT_ENABLE | PIN_PULL_DIRECTION )
    },

    {PINMUX_END, PINMUX_END}
};

/* ========================================================================== */
/*                 Internal Function Declarations                             */
/* ========================================================================== */
int32_t Bootloader_ReadSysfwImage();
int32_t Bootloader_loadFirmware();
static void Bootloader_sciclientBoardCfg();

/*  In this sample bootloader, we load appimages for RTO/Baremetal and Linux at different offset
    i.e the appimage for Linux (for A53) and RTOS/Baremetal (for R5) is flashed at different offset in flash

    Here at one flash offset, there is a multi-core .appimage that holds RPRC for R5
    and another .appimage that holds the linux binaries(ATF, OPTEE, A53-SPL) at another offset.

    When flashing make sure to flash images to below offset using the flash tool.

    RTOS/Baremetal appimage (for R5) flash at offset 0x100000
    Linux appimage (for A53) flash at offset 0x800000
*/

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
    while(loop)
        ;
}

int32_t App_loadImages(Bootloader_Handle bootHandle, Bootloader_BootImageInfo *bootImageInfo)
{
    int32_t status = SystemP_FAILURE;
    Bootloader_Config *bootConfig;

    if(bootHandle != NULL)
    {
        bootConfig = (Bootloader_Config *)bootHandle;
        bootConfig->scratchMemPtr = gAppimage;
        status = Bootloader_parseMultiCoreAppImage(bootHandle, bootImageInfo);

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

        /* Load CPUs */
        if(status == SystemP_SUCCESS && (TRUE == isSelfBoot))
        {
            /* Set clocks for self cluster */
            bootImageInfo->cpuInfo[CSL_CORE_ID_R5FSS0_0].clkHz = Bootloader_socCpuGetClkDefault(CSL_CORE_ID_R5FSS0_0);
            bootImageInfo->cpuInfo[CSL_CORE_ID_R5FSS0_1].clkHz = Bootloader_socCpuGetClkDefault(CSL_CORE_ID_R5FSS0_1);

            /* Reset self cluster, both Core0 and Core 1. Init RAMs and load the app  */
            status = Bootloader_loadSelfCpu(bootHandle, &(bootImageInfo->cpuInfo[CSL_CORE_ID_R5FSS0_0]), FALSE);
            if(status == SystemP_SUCCESS && (TRUE == Bootloader_socIsR5FSSDual(BOOTLOADER_R5FSS0)))
            {
                status = Bootloader_loadSelfCpu(bootHandle, &(bootImageInfo->cpuInfo[CSL_CORE_ID_R5FSS0_1]), FALSE);
            }
        }
    }

    return status;
}

int32_t App_loadLinuxImages(Bootloader_Handle bootHandle, Bootloader_BootImageInfo *bootImageInfo)
{
    int32_t status = SystemP_FAILURE;
    Bootloader_Config *bootConfigLinux;

    if(bootHandle != NULL)
    {
        bootConfigLinux = (Bootloader_Config *)bootHandle;
        bootConfigLinux->scratchMemPtr = gAppimage;
        status = Bootloader_parseAndLoadLinuxAppImage(bootHandle, bootImageInfo);

		if(status == SystemP_SUCCESS)
		{
			bootImageInfo->cpuInfo[CSL_CORE_ID_A53SS0_0].clkHz = Bootloader_socCpuGetClkDefault(CSL_CORE_ID_A53SS0_0);
			status = Bootloader_loadCpu(bootHandle, &(bootImageInfo->cpuInfo[CSL_CORE_ID_A53SS0_0]));
		}
	}

	return status;
}

int32_t App_runLinuxCpu(Bootloader_Handle bootHandle, Bootloader_BootImageInfo *bootImageInfo)
{
	int32_t status = SystemP_FAILURE;

    /* Initialize GTC by enabling using Syscfg */

    /* Change the dev stat register to SD card bootmode so that SPL loads uBoot and linux kernel from SD card */
	SOC_setDevStat(SOC_BOOTMODE_MMCSD);

    /* Enable pinmux for MMCSD (Workaround as MMC SD pinmux is not initialized in A53 SPL) */
    Pinmux_config(gPinMuxMMCSDCfg, PINMUX_DOMAIN_ID_MAIN);

	status = Bootloader_runCpu(bootHandle, &(bootImageInfo->cpuInfo[CSL_CORE_ID_A53SS0_0]));

	return status;
}

int main(void)
{
    int32_t status = SystemP_FAILURE;

    Bootloader_profileReset();

    System_init();
	Bootloader_profileAddProfilePoint("System_init");

    Drivers_open();
    Bootloader_profileAddProfilePoint("Drivers_open");

    status = Board_driversOpen();
    DebugP_assert(status == SystemP_SUCCESS);
    Bootloader_profileAddProfilePoint("Board_driversOpen");

    status = Bootloader_ReadSysfwImage();
    if (SystemP_SUCCESS == status)
    {
        status = Bootloader_loadFirmware();
    }
    Bootloader_profileAddProfilePoint("SYSFW init");

    if (SystemP_SUCCESS == status)
    {
        Bootloader_sciclientBoardCfg();
    }

    System_lateInit();

    GTC_init();

    status = Sciclient_getVersionCheck(1);

    if(SystemP_SUCCESS == status)
    {
        Bootloader_BootImageInfo bootImageInfo;
		Bootloader_Params bootParams;
        Bootloader_Handle bootHandle;

		Bootloader_BootImageInfo bootImageInfoLinux;
		Bootloader_Params bootParamsLinux;
        Bootloader_Handle bootHandleLinux;

        Bootloader_Params_init(&bootParams);
		Bootloader_Params_init(&bootParamsLinux);

		Bootloader_BootImageInfo_init(&bootImageInfo);
		Bootloader_BootImageInfo_init(&bootImageInfoLinux);

        bootHandle = Bootloader_open(CONFIG_BOOTLOADER_FLASH0, &bootParams);
		bootHandleLinux = Bootloader_open(CONFIG_BOOTLOADER_FLASH_LINUX, &bootParamsLinux);
        if(bootHandle != NULL)
        {
			status = App_loadImages(bootHandle, &bootImageInfo);
            Bootloader_profileAddProfilePoint("App_loadImages");
        }

		if(SystemP_SUCCESS == status)
		{
			if(bootHandleLinux != NULL)
			{
				status = App_loadLinuxImages(bootHandleLinux, &bootImageInfoLinux);
                Bootloader_profileAddProfilePoint("App_loadLinuxImages");
			}
		}

		if(SystemP_SUCCESS == status)
		{
			/* Print SBL log as Linux prints log to the same UART port */
			Bootloader_profilePrintProfileLog();
			DebugP_log("Image loading done, switching to application ...\r\n");
			DebugP_log("Starting linux and RTOS/Baremetal applications\r\n");
			//UART_flushTxFifo(gUartHandle[CONFIG_UART0]);
		}

		if(SystemP_SUCCESS == status)
		{
			status = App_runLinuxCpu(bootHandleLinux, &bootImageInfoLinux);
		}

        Bootloader_close(bootHandleLinux);

        uint32_t isDualSelfR5F = FALSE;
        if(Bootloader_socIsR5FSSDual(BOOTLOADER_R5FSS0) == TRUE)
        {
            isDualSelfR5F = TRUE;
        }

        status = Bootloader_cpuSetAppEntryPoint(&bootImageInfo, isDualSelfR5F);
		if(SystemP_SUCCESS == status)
		{
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