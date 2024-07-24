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
#include <string.h>
#include "ti_drivers_config.h"
#include "ti_board_config.h"
#include "ti_drivers_open_close.h"
#include "ti_board_open_close.h"
#include <drivers/sciclient.h>
#include <drivers/bootloader.h>
#include <drivers/pcie.h>
#include <drivers/gtc/v0/gtc.h>

#include <drivers/ddr/v1/soc/am65x/board_ddr_config.h>
#include <drivers/sciclient/sciclient_default_boardcfg/am65x/sciclient_defaultBoardcfg_hex.h>
#include <drivers/sciclient/sciclient_default_boardcfg/am65x/sciclient_defaultBoardcfg_rm_hex.h>
#include <drivers/sciclient/sciclient_default_boardcfg/am65x/sciclient_defaultBoardcfg_pm_hex.h>
#include <drivers/sciclient/sciclient_default_boardcfg/am65x/sciclient_defaultBoardcfg_security_hex.h>
/******************************************************************************/
/* ROM in PCIe boot mode expects image transfer via PCIe.                     */
/*    - The image is can be moved to any location ROM is able to load to      */
/*      (SRAM in am65x).                                               */
/*    - The image transfer is marked as completed on marking the boot data    */
/*      address in SRAM at 0x71BCFE0. For example, if the image is loaded at  */
/*      the offset 0x1000, then when it is done it would write 0x70001000     */
/*      (Internal RAM memory base + offset) to 0x701BCFE0.                    */
/*    - Then ROM authenticates and boots the SBL from the address.            */
/* Care must be taken so that the the address where the SBL image is loaded   */
/* to does not overlap with the SBL load address.                             */

/* SBL nevertheless adds an inbound ATU region so that the application        */
/* appimage can be loaded to DDR. This helps in receiving appimages of larger */
/* sizes.                                                                     */
/* An outbound ATU is also added so that SBL can notify the host side         */
/* application with a handshake signal to start image transfer on sending out */
/* a "MAGIC WORD" of choice.                                                  */

/* The completion of image transfer by host sided application is marked by a  */
/* "MAGIC WORD" of choice.                                                    */
/* The host side application also marks the relative offset to which the      */
/* image is loaded to.                                                        */
/* On receiving the "MAGIC WORD", SBL loads and boot the application image    */
/******************************************************************************/

/* ========================================================================== */
/*                             Macros & Typedefs                              */
/* ========================================================================== */
#define BOARD_DDR_PLL_CLK_FREQ              (400000000U)

#define SBL_SYSFW_MAX_SIZE                  (0x42000U)
/* Maximum image size supported */
/* The last 8 bytes is used for the "MAGIC WORD" and the relative offset to */
/* which image is transferred to. */
#define BOOTLOADER_APPIMAGE_MAX_FILE_SIZE   (0x60000)

/* Hand shake "MAGIC WORD" to indicate SBL is ready to accept images */
#define HANDSHAKE_MAGIC_WORD                (0xFEEDC0DE)

/* The "MAGIC WORD" used to indicate image transfer complete by the HOST side */
#define APPIMAGE_MAGIC_WORD                 (0xC0DEFEED)

#define SYSFW_MAGIC_WORD                    (0xC0DEC0DE)
/* PCIe Instance Macros */
#define CONFIG_PCIE0 (0U)
#define CONFIG_PCIE_NUM_INSTANCES (1U)

#define CONFIG_PCIE_OB_REGION_INDEX             (0x1U)
#define CONFIG_PCIE0_OB_REGION0_LOWER           (0x11000000UL)
#define CONFIG_PCIE0_OB_REGION0_UPPER           (0x0U)
#define CONFIG_PCIE0_OB_REGION0_LOWER_TARGET    (0x90000000UL)
#define CONFIG_PCIE0_OB_REGION0_UPPER_TARGET    (0x0U)
#define CONFIG_PCIE_OB_REGION0_WINDOW           (0x8U)  /* 4 Bytes (uint32_t) */

#define CONFIG_PCIE_IB_REGION0_INDEX            (0x2U)
#define CONFIG_PCIE0_IB_REGION0_LOWER           (0x70000000UL)
#define CONFIG_PCIE0_IB_REGION0_UPPER           (0x0U)
#define CONFIG_PCIE_IB_REGION0_WINDOW           (0x800000U)  /* 8MB */

Pcie_Handle gPcieHandle[CONFIG_PCIE_NUM_INSTANCES];
/* Pcie objects - initialized by the driver */
static Pcie_Object gPcieObjects[CONFIG_PCIE_NUM_INSTANCES];
/* PCIe Driver Attributes */
Pcie_Attrs gPcieAttrs[CONFIG_PCIE_NUM_INSTANCES] =
{
    {
        .deviceNum = 0,
        .operationMode = PCIE_EP_MODE,
        .gen = PCIE_GEN2,
        .numLanes = 2,
        .obAtu = NULL,
        .obAtuNum = 0,
        .ibAtu = NULL,
        .ibAtuNum = 0,
        .msiGlobalEventNum = 0,
        .msiRingNum = 0,
        .msiIntNum = 0,
        .msiIrqEnableFlag = 0,
        .msiIsrCtrl = NULL,
        .msiRingMem = NULL,

        .msixGlobalEventNum = 0,
        .msixRingNum = 0,
        .msixIntNum = 0,
        .msixIrqEnableFlag = 0,
        .epMsixTbl = NULL,
        .msixIsrCtrl = NULL,
        .msixRingMem = NULL,
    },
};

Pcie_Config gPcieConfig[CONFIG_PCIE_NUM_INSTANCES] =
{
    {
        &gPcieObjects[CONFIG_PCIE0],
        &gPcieAttrs[CONFIG_PCIE0],
    },
};

uint32_t gPcieConfigNum = CONFIG_PCIE_NUM_INSTANCES;
/* Buffer to receive Sysfw image */
uint8_t gSysImageBuf[SBL_SYSFW_MAX_SIZE] __attribute__((aligned(4096), section(".firmware")));
/* Buffer to receive appimage */
uint8_t gAppImageBuf[BOOTLOADER_APPIMAGE_MAX_FILE_SIZE] __attribute__((aligned(4096), section(".imagebuf")));

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
    while(loop)
        ;
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

#ifdef CONFIG_BOOTLOADER_PCIE_USEROMCFG
    /* Pcie_open is not called again in SBL */
    /* SBL makes use of ROM configuration. Link training is not done again */
    Pcie_init();
#endif

    /* Outbound region to send the Handshake Signal to Host Device */
    {
        Pcie_AtuRegionParams regionParams;
        memset(&regionParams, 0, sizeof(regionParams));

        regionParams.regionDir = PCIE_ATU_REGION_DIR_OUTBOUND;
        regionParams.tlpType   = PCIE_TLP_TYPE_MEM;

        regionParams.lowerBaseAddr    = CONFIG_PCIE0_OB_REGION0_LOWER;
        regionParams.upperBaseAddr    = 0x0;

        regionParams.regionWindowSize = CONFIG_PCIE_OB_REGION0_WINDOW-1;

        regionParams.lowerTargetAddr    = CONFIG_PCIE0_OB_REGION0_LOWER_TARGET;
        regionParams.upperTargetAddr    = 0x0UL;

        regionParams.enableRegion = 1;

        status = Pcie_atuRegionConfig (&gPcieConfig[0], PCIE_LOCATION_LOCAL,
                    CONFIG_PCIE_OB_REGION_INDEX, &regionParams);

        DebugP_assert(SystemP_SUCCESS == status);
    }

    /* Set inbound ATU for receiving the sysfw image */
    {
        Pcie_BarCfg barCfg;
        Pcie_AtuRegionParams regionParams;

        memset (&barCfg, 0, sizeof(Pcie_BarCfg));

        barCfg.location = PCIE_LOCATION_LOCAL;
        barCfg.mode     = PCIE_EP_MODE;
        barCfg.barxc    = PCIE_BARC_32B_MEM_BAR_NON_PREFETCH;
        barCfg.barxa    = PCIE_EPBARA_8M;
        barCfg.idx      = CONFIG_PCIE_IB_REGION0_INDEX;

        status = Pcie_cfgBar (&gPcieConfig[0], &barCfg);

        DebugP_assert(SystemP_SUCCESS == status);

        memset(&regionParams, 0, sizeof(regionParams));

        regionParams.regionDir = PCIE_ATU_REGION_DIR_INBOUND;
        regionParams.tlpType   = PCIE_TLP_TYPE_MEM;

        regionParams.lowerBaseAddr    = CONFIG_PCIE0_IB_REGION0_LOWER;
        regionParams.upperBaseAddr    = 0x0;

        regionParams.regionWindowSize = CONFIG_PCIE_IB_REGION0_WINDOW-1;

        regionParams.lowerTargetAddr    = (uint32_t)gSysImageBuf;
        regionParams.upperTargetAddr    = 0x0;

        regionParams.enableRegion = 1;

        status = Pcie_atuRegionConfig (&gPcieConfig[0], PCIE_LOCATION_LOCAL,
                    CONFIG_PCIE_IB_REGION0_INDEX, &regionParams);

        DebugP_assert(SystemP_SUCCESS == status);
    }
    {

    /* Send out handshake message to HOST side application to signal SBL
       is ready to accept SYSFW image */
        volatile uint32_t *handShakePtr = (uint32_t *)CONFIG_PCIE0_OB_REGION0_LOWER;
        *handShakePtr = SYSFW_MAGIC_WORD;

        /* Wait for image reception via PCIe */

        volatile uint32_t *magicPtr  = (uint32_t *)(&gSysImageBuf[0] + SBL_SYSFW_MAX_SIZE - sizeof(uint32_t));
        /* Wait for image to receive */
        do {
            CacheP_inv((void *)magicPtr, SBL_SYSFW_MAX_SIZE * sizeof(uint32_t), CacheP_TYPE_ALL);
        }while(*magicPtr != SYSFW_MAGIC_WORD);

    }

    if (SystemP_SUCCESS == status)
    {
        status = Bootloader_loadFirmware();
    }
    Bootloader_profileAddProfilePoint("SYSFW init");

    status = Sciclient_init(CSL_CORE_ID_R5FSS0_0);
    status = Sciclient_getVersionCheck(1);
    Bootloader_profileAddProfilePoint("Sciclient Get Version");

    if (SystemP_SUCCESS == status)
    {
        Bootloader_sciclientBoardCfg();
    }
    System_lateInit();
    /* Re-establish the PCIe link */
    Pcie_open(CONFIG_PCIE0);
    memset((void*)CONFIG_PCIE0_IB_REGION0_LOWER, 0, 0x40000);

    if(SystemP_SUCCESS == status)
    {
        Bootloader_BootImageInfo bootImageInfo;
        Bootloader_Params bootParams;
        Bootloader_Handle bootHandle;

        Bootloader_Params_init(&bootParams);
        Bootloader_BootImageInfo_init(&bootImageInfo);

        bootParams.memArgsAppImageBaseAddr = (uintptr_t)gAppImageBuf;

        bootHandle = Bootloader_open(CONFIG_BOOTLOADER_PCIE, &bootParams);

        if(bootHandle != NULL)
        {
           /* Outbound region to send the Handshake Signal to Host Device */

            Pcie_AtuRegionParams regionParams;
            memset(&regionParams, 0, sizeof(regionParams));

            regionParams.regionDir = PCIE_ATU_REGION_DIR_OUTBOUND;
            regionParams.tlpType   = PCIE_TLP_TYPE_MEM;

            regionParams.lowerBaseAddr    = CONFIG_PCIE0_OB_REGION0_LOWER;
            regionParams.upperBaseAddr    = 0x0;

            regionParams.regionWindowSize = CONFIG_PCIE_OB_REGION0_WINDOW-1;

            regionParams.lowerTargetAddr    = CONFIG_PCIE0_OB_REGION0_LOWER_TARGET;
            regionParams.upperTargetAddr    = 0x0UL;

            regionParams.enableRegion = 1;

            status = Pcie_atuRegionConfig (&gPcieConfig[0], PCIE_LOCATION_LOCAL,
                        CONFIG_PCIE_OB_REGION_INDEX, &regionParams);

            DebugP_assert(SystemP_SUCCESS == status);

            /* configure inbound region to receive application image*/

            memset(&regionParams, 0, sizeof(regionParams));

            regionParams.regionDir = PCIE_ATU_REGION_DIR_INBOUND;
            regionParams.tlpType   = PCIE_TLP_TYPE_MEM;

            regionParams.lowerBaseAddr    = CONFIG_PCIE0_IB_REGION0_LOWER;
            regionParams.upperBaseAddr    = 0x0;

            regionParams.regionWindowSize = CONFIG_PCIE_IB_REGION0_WINDOW-1;

            regionParams.lowerTargetAddr    = (uint32_t)gAppImageBuf;
            regionParams.upperTargetAddr    = 0x0;

            regionParams.enableRegion = 1;

            status = Pcie_atuRegionConfig (&gPcieConfig[0], PCIE_LOCATION_LOCAL,
                        CONFIG_PCIE_IB_REGION0_INDEX, &regionParams);

            DebugP_assert(SystemP_SUCCESS == status);
                /* Send out handshake message to HOST side application to signal SBL
                    is ready to accept application image */
            volatile uint32_t *handShakePtr = (uint32_t *)CONFIG_PCIE0_OB_REGION0_LOWER;
            *handShakePtr = HANDSHAKE_MAGIC_WORD;

                 /* Wait for image reception via PCIe */

            volatile uint32_t *magicPtr  = (uint32_t *)(&gAppImageBuf[0] + BOOTLOADER_APPIMAGE_MAX_FILE_SIZE + sizeof(uint32_t));
                /* Wait for image to receive */
        do {
            CacheP_inv((void *)magicPtr, 128 , CacheP_TYPE_ALL);
        }while(*magicPtr != APPIMAGE_MAGIC_WORD);

        }

        Bootloader_profileAddProfilePoint("Appimage reception via PCIE");

        if((bootHandle != NULL) && (SystemP_SUCCESS == status))
        {
            status = Bootloader_parseMultiCoreAppImage(bootHandle, &bootImageInfo);
            /* Load CPUs */

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
                bootImageInfo.cpuInfo[CSL_CORE_ID_R5FSS0_0].clkHz = Bootloader_socCpuGetClkDefault(CSL_CORE_ID_R5FSS0_0);
                bootImageInfo.cpuInfo[CSL_CORE_ID_R5FSS0_1].clkHz = Bootloader_socCpuGetClkDefault(CSL_CORE_ID_R5FSS0_1);

                /* Reset self cluster, both Core0 and Core 1. Init RAMs and load the app  */
                status = Bootloader_loadSelfCpu(bootHandle, &bootImageInfo.cpuInfo[CSL_CORE_ID_R5FSS0_0], FALSE);
                if((status == SystemP_SUCCESS) && (TRUE == Bootloader_socIsR5FSSDual(BOOTLOADER_R5FSS0)))
                {
                    status = Bootloader_loadSelfCpu(bootHandle, &bootImageInfo.cpuInfo[CSL_CORE_ID_R5FSS0_1], FALSE);
                }
            }
            Bootloader_profileAddProfilePoint("CPU Load");
            Bootloader_profileUpdateAppimageSize(Bootloader_getMulticoreImageSize(bootHandle));
            Bootloader_profileUpdateMediaAndClk(BOOTLOADER_MEDIA_PCIE, 0);

            if(status == SystemP_SUCCESS)
            {
                /* Print SBL Profiling logs to UART as other cores may use the UART for logging */
                Bootloader_profileAddProfilePoint("SBL End");
                Bootloader_profilePrintProfileLog();
                DebugP_log("Image loading done, switching to application ...\r\n");
                UART_flushTxFifo(gUartHandle[CONFIG_UART0]);
            }

            uint32_t isDualSelfR5F = FALSE;
            if(Bootloader_socIsR5FSSDual(BOOTLOADER_R5FSS0) == TRUE)
            {
                isDualSelfR5F = TRUE;
            }

            /* Run CPUs */
            if(status == SystemP_SUCCESS)
            {
                status = Bootloader_cpuSetAppEntryPoint(&bootImageInfo, isDualSelfR5F);
                if(status == SystemP_SUCCESS){
                /* Reset self cluster, both Core0 and Core 1. Init RAMs and run the app  */
                status = Bootloader_runSelfCpu(bootHandle, &bootImageInfo);
                }
            }

            /* it should not return here, if it does, then there was some error */
            Bootloader_close(bootHandle);
        }

    }
    if(status != SystemP_SUCCESS)
    {
        DebugP_log("Some tests have failed!!\r\n");
    }

    Board_deinit();
    System_deinit();

    return 0;
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
