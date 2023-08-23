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

#include <stdlib.h>
#include <string.h>
#include "ti_drivers_config.h"
#include "ti_board_config.h"
#include "ti_drivers_open_close.h"
#include "ti_board_open_close.h"
#include <drivers/sciclient.h>
#include <drivers/bootloader.h>
#include <drivers/pcie.h>

/******************************************************************************/
/* ROM in PCIe boot mode expects image transfer via PCIe.                     */
/*    - The image is can be moved to any location ROM is able to load to      */
/*      (SRAM in am64x/am243x).                                               */
/*    - The image transfer is marked as completed on marking the boot data    */
/*      address in SRAM at 0x71BCFE0. For example, if the image is loaded at  */
/*      the offset 0x1000, then when it is done it would write 0x70001000     */
/*      (Internal RAM memory base + offset) to 0x701BCFE0.                    */
/*    - Then ROM authenticates and boots the SBL from the address.            */
/* Care must be taken so that the the address where the SBL image is loaded   */
/* to does not overlap with the SBL load address.                             */

/* In SBL there is no need of reinitialization of PCIe and link training      */
/* again. SBL can reuse the ROM configuration does by ROM.                    */
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

/* Maximum image size supported */
/* The last 8 bytes is used for the "MAGIC WORD" and the relative offset to */
/* which image is transferred to. */
#define BOOTLOADER_APPIMAGE_MAX_FILE_SIZE   (0x800000U - (2*sizeof(uint32_t)))

/* Hand shake "MAGIC WORD" to indicate SBL is ready to accept images */
#define HANDSHAKE_MAGIC_WORD                (0xFEEDC0DE)

/* The "MAGIC WORD" used to indicate image transfer complete by the HOST side */
#define APPIMAGE_MAGIC_WORD                 (0xC0DEFEED)

/* PCIe Instance Macros */
#define CONFIG_PCIE0 (0U)
#define CONFIG_PCIE_NUM_INSTANCES (1U)

#define CONFIG_PCIE_OB_REGION_INDEX             (0x0U)
#define CONFIG_PCIE0_OB_REGION0_LOWER           (0x68200000UL)
#define CONFIG_PCIE0_OB_REGION0_UPPER           (0x0U)
#define CONFIG_PCIE0_OB_REGION0_LOWER_TAEGET    (0x80000000UL)
#define CONFIG_PCIE0_OB_REGION0_UPPER_TAEGET    (0x0U)
#define CONFIG_PCIE_OB_REGION0_WINDOW           (0x4U)  /* 4 Bytes (uint32_t) */

#define CONFIG_PCIE_IB_REGION_INDEX             (0x2U)
#define CONFIG_PCIE0_IB_REGION0_LOWER           (0x70000000UL)
#define CONFIG_PCIE0_IB_REGION0_UPPER           (0x0U)
#define CONFIG_PCIE_IB_REGION0_WINDOW           (0x800000U)  /* 8MB */

/* Buffer to receive appimage */
uint8_t gAppimage[BOOTLOADER_APPIMAGE_MAX_FILE_SIZE] __attribute__ ((section (".app"), aligned (4096)));

#ifdef CONFIG_BOOTLOADER_PCIE_USEROMCFG
/* PCIe Driver Attributes */
Pcie_Attrs gPcieAttrs[CONFIG_PCIE_NUM_INSTANCES] =
{
    {
        .deviceNum = 0,
        .operationMode = PCIE_EP_MODE,
        .gen = PCIE_GEN2,
        .numLanes = 1,
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

/* Pcie objects - initialized by the driver */
static Pcie_Object gPcieObjects[CONFIG_PCIE_NUM_INSTANCES];

/* Pcie driver configuration */
Pcie_Config gPcieConfig[CONFIG_PCIE_NUM_INSTANCES] =
{
    {
        &gPcieObjects[CONFIG_PCIE0],
        &gPcieAttrs[CONFIG_PCIE0],
    },
};

/* Number of PCIe instances */
uint32_t gPcieConfigNum = CONFIG_PCIE_NUM_INSTANCES;
/******************************************************************************/
#endif

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
    int32_t status;

    Bootloader_profileReset();

    Bootloader_socWaitForFWBoot();

    /* On a warm reset in PCIe boot mode ROM is not booting image again from PCIe */
    /* Hence removing the workaround for eMMC boot */
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

        Sciclient_BoardCfgPrms_t boardCfgPrms_rm =
        {
            .boardConfigLow = (uint32_t)0,
            .boardConfigHigh = 0,
            .boardConfigSize = 0,
            .devGrp = DEVGRP_ALL,
        };

        status = Sciclient_boardCfgRm(&boardCfgPrms_rm);

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

    status = Sciclient_getVersionCheck(1);
    Bootloader_profileAddProfilePoint("Sciclient Get Version");

    /* If not using ROM configurations for PCIe, Pcie_init is called from the
       syscfg generated files */
#ifdef CONFIG_BOOTLOADER_PCIE_USEROMCFG
    /* Pcie_open is not called again in SBL */
    /* SBL makes use of ROM configuration. Link training is not done again */
    Pcie_init();
#endif
    /* Set outbound ATU for sending the handshake signal */
    {
        Pcie_AtuRegionParams regionParams;
        memset(&regionParams, 0, sizeof(regionParams));

        regionParams.regionDir = PCIE_ATU_REGION_DIR_OUTBOUND;
        regionParams.tlpType   = PCIE_TLP_TYPE_MEM;

        regionParams.lowerBaseAddr    = CONFIG_PCIE0_OB_REGION0_LOWER;
        regionParams.upperBaseAddr    = 0x0;

        regionParams.regionWindowSize = CONFIG_PCIE_OB_REGION0_WINDOW-1;

        regionParams.lowerTargetAddr    = CONFIG_PCIE0_OB_REGION0_LOWER_TAEGET;
        regionParams.upperTargetAddr    = 0x0UL;

        status = Pcie_atuRegionConfig (&gPcieConfig[0], PCIE_LOCATION_LOCAL,
                    CONFIG_PCIE_OB_REGION_INDEX, &regionParams);

        DebugP_assert(SystemP_SUCCESS == status);
    }

    /* Set inbound ATU for receiving the application image */
    {
        Pcie_BarCfg barCfg;
        Pcie_AtuRegionParams regionParams;

        memset (&barCfg, 0, sizeof(Pcie_BarCfg));

        barCfg.location = PCIE_LOCATION_LOCAL;
        barCfg.mode     = PCIE_EP_MODE;
        barCfg.barxc    = PCIE_BARC_32B_MEM_BAR_NON_PREFETCH;
        barCfg.barxa    = PCIE_EPBARA_8M;
        barCfg.idx      = CONFIG_PCIE_IB_REGION_INDEX;

        status = Pcie_cfgBar (&gPcieConfig[0], &barCfg);

        DebugP_assert(SystemP_SUCCESS == status);

        memset(&regionParams, 0, sizeof(regionParams));

        regionParams.regionDir = PCIE_ATU_REGION_DIR_INBOUND;
        regionParams.tlpType   = PCIE_TLP_TYPE_MEM;

        regionParams.lowerBaseAddr    = CONFIG_PCIE0_IB_REGION0_LOWER;
        regionParams.upperBaseAddr    = 0x0;

        regionParams.regionWindowSize = CONFIG_PCIE_IB_REGION0_WINDOW-1;

        regionParams.lowerTargetAddr    = (uint32_t)gAppimage;
        regionParams.upperTargetAddr    = 0x0;

        status = Pcie_atuRegionConfig (&gPcieConfig[0], PCIE_LOCATION_LOCAL,
                    CONFIG_PCIE_IB_REGION_INDEX, &regionParams);

        DebugP_assert(SystemP_SUCCESS == status);
    }

    /* Send out handshake message to HOST side application to signal SBL
       is ready to accept application image */
    volatile uint32_t *handShakePtr = (uint32_t *)CONFIG_PCIE0_OB_REGION0_LOWER;
    *handShakePtr = HANDSHAKE_MAGIC_WORD;

    Bootloader_BootImageInfo bootImageInfo;
    Bootloader_Params bootParams;
    Bootloader_Handle bootHandle;

    /* Wait for image reception via PCIe */
    {
        volatile uint32_t *magicPtr  = (uint32_t *)(&gAppimage[0] + BOOTLOADER_APPIMAGE_MAX_FILE_SIZE + (1*sizeof(uint32_t)));
        /* Wait for image to receive */
        do {
            CacheP_inv((void *)magicPtr, 128, CacheP_TYPE_ALL);
        }while(*magicPtr != APPIMAGE_MAGIC_WORD);

        volatile uint32_t *imageOffsetPtr = (uint32_t *)(&gAppimage[0] + BOOTLOADER_APPIMAGE_MAX_FILE_SIZE);
        uintptr_t imgOffset = *imageOffsetPtr;

        /* Initialize bootloader instance */
        {
            Bootloader_Params_init(&bootParams);
            Bootloader_BootImageInfo_init(&bootImageInfo);

            bootParams.memArgsAppImageBaseAddr = (uintptr_t)(gAppimage + imgOffset);
            bootHandle = Bootloader_open(CONFIG_BOOTLOADER_PCIE, &bootParams);
        }

        if((bootHandle != NULL) && (SystemP_SUCCESS == status))
        {
            /* Initialize PRU Cores if applicable */
            Bootloader_Config *cfg = (Bootloader_Config *)bootHandle;
            if(TRUE == cfg->initICSSCores)
            {
                status = Bootloader_socEnableICSSCores(BOOTLOADER_ICSS_CORE_DEFAULT_FREQUENCY);
                DebugP_assert(status == SystemP_SUCCESS);
            }

            status = Bootloader_parseMultiCoreAppImage(bootHandle, &bootImageInfo);
        }

    }

    Bootloader_profileAddProfilePoint("Appimage reception via PCIE");

    /* Load appimage */
    {
        /* Load CPUs */
        /* Do not load M4 when MCU domain is reset isolated */
        if (!Bootloader_socIsMCUResetIsoEnabled())
        {
            if(status == SystemP_SUCCESS && (TRUE == Bootloader_isCorePresent(bootHandle, CSL_CORE_ID_M4FSS0_0)))
            {
                bootImageInfo.cpuInfo[CSL_CORE_ID_M4FSS0_0].clkHz = Bootloader_socCpuGetClkDefault(CSL_CORE_ID_M4FSS0_0);
                status = Bootloader_loadCpu(bootHandle, &bootImageInfo.cpuInfo[CSL_CORE_ID_M4FSS0_0]);
            }
        }
        if(status == SystemP_SUCCESS && (TRUE == Bootloader_isCorePresent(bootHandle, CSL_CORE_ID_R5FSS1_0)))
        {
            bootImageInfo.cpuInfo[CSL_CORE_ID_R5FSS1_0].clkHz = Bootloader_socCpuGetClkDefault(CSL_CORE_ID_R5FSS1_0);
            status = Bootloader_loadCpu(bootHandle, &bootImageInfo.cpuInfo[CSL_CORE_ID_R5FSS1_0]);
        }
        if(status == SystemP_SUCCESS && (TRUE == Bootloader_isCorePresent(bootHandle, CSL_CORE_ID_R5FSS1_1)))
        {
            bootImageInfo.cpuInfo[CSL_CORE_ID_R5FSS1_1].clkHz = Bootloader_socCpuGetClkDefault(CSL_CORE_ID_R5FSS1_1);
            status = Bootloader_loadCpu(bootHandle, &bootImageInfo.cpuInfo[CSL_CORE_ID_R5FSS1_1]);
        }
        if(status == SystemP_SUCCESS && (TRUE == Bootloader_isCorePresent(bootHandle, CSL_CORE_ID_A53SS0_0)))
        {
            bootImageInfo.cpuInfo[CSL_CORE_ID_A53SS0_0].clkHz = Bootloader_socCpuGetClkDefault(CSL_CORE_ID_A53SS0_0);
            status = Bootloader_loadCpu(bootHandle, &bootImageInfo.cpuInfo[CSL_CORE_ID_A53SS0_0]);
        }
        if(status == SystemP_SUCCESS && (TRUE == Bootloader_isCorePresent(bootHandle, CSL_CORE_ID_A53SS0_1)))
        {
            bootImageInfo.cpuInfo[CSL_CORE_ID_A53SS0_1].clkHz = Bootloader_socCpuGetClkDefault(CSL_CORE_ID_A53SS0_1);
            status = Bootloader_loadCpu(bootHandle, &bootImageInfo.cpuInfo[CSL_CORE_ID_A53SS0_1]);
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
            bootImageInfo.cpuInfo[CSL_CORE_ID_R5FSS0_0].clkHz = Bootloader_socCpuGetClkDefault(CSL_CORE_ID_R5FSS0_0);
            bootImageInfo.cpuInfo[CSL_CORE_ID_R5FSS0_1].clkHz = Bootloader_socCpuGetClkDefault(CSL_CORE_ID_R5FSS0_1);

            /* Reset self cluster, both Core0 and Core 1. Init RAMs and load the app  */
            /* Skip the image load by passing TRUE, so that image load on self core doesnt corrupt the SBLs IVT. Load the image later before the reset release of the self core  */
            status = Bootloader_loadSelfCpu(bootHandle, &bootImageInfo.cpuInfo[CSL_CORE_ID_R5FSS0_0], TRUE);
            if((status == SystemP_SUCCESS) && (TRUE == Bootloader_socIsR5FSSDual(BOOTLOADER_R5FSS0)))
            {
                status = Bootloader_loadSelfCpu(bootHandle, &bootImageInfo.cpuInfo[CSL_CORE_ID_R5FSS0_1], FALSE);
            }
        }

        Bootloader_profileAddProfilePoint("CPU Load");
        Bootloader_profileUpdateAppimageSize(Bootloader_getMulticoreImageSize(bootHandle));
        Bootloader_profileUpdateMediaAndClk(BOOTLOADER_MEDIA_PCIE, 0);
    }

    if(status == SystemP_SUCCESS)
    {
        /* Print SBL Profiling logs to UART as other cores may use the UART for logging */
        Bootloader_profileAddProfilePoint("SBL End");
        Bootloader_profilePrintProfileLog();
        DebugP_log("Image loading done, switching to application ...\r\n");
        UART_flushTxFifo(gUartHandle[CONFIG_UART_CONSOLE]);
    }

    /* Run CPUs */
    {
        /* Do not run M4 when MCU domain is reset isolated */
        if (!Bootloader_socIsMCUResetIsoEnabled())
        {
            if(status == SystemP_SUCCESS && (TRUE == Bootloader_isCorePresent(bootHandle, CSL_CORE_ID_M4FSS0_0)))
            {
                status = Bootloader_runCpu(bootHandle, &bootImageInfo.cpuInfo[CSL_CORE_ID_M4FSS0_0]);
            }
        }
        if(status == SystemP_SUCCESS && (TRUE == Bootloader_isCorePresent(bootHandle, CSL_CORE_ID_R5FSS1_0)))
        {
            status = Bootloader_runCpu(bootHandle, &bootImageInfo.cpuInfo[CSL_CORE_ID_R5FSS1_0]);
        }
        if(status == SystemP_SUCCESS && (TRUE == Bootloader_isCorePresent(bootHandle, CSL_CORE_ID_R5FSS1_1)))
        {
            status = Bootloader_runCpu(bootHandle, &bootImageInfo.cpuInfo[CSL_CORE_ID_R5FSS1_1]);
        }
        if(status == SystemP_SUCCESS && (TRUE == Bootloader_isCorePresent(bootHandle, CSL_CORE_ID_A53SS0_0)))
        {
            status = Bootloader_runCpu(bootHandle, &bootImageInfo.cpuInfo[CSL_CORE_ID_A53SS0_0]);
        }
        if(status == SystemP_SUCCESS && (TRUE == Bootloader_isCorePresent(bootHandle, CSL_CORE_ID_A53SS0_1)))
        {
            status = Bootloader_runCpu(bootHandle, &bootImageInfo.cpuInfo[CSL_CORE_ID_A53SS0_1]);
        }
        if(status == SystemP_SUCCESS)
        {
            /* Load the image on self core now */
            if( bootImageInfo.cpuInfo[CSL_CORE_ID_R5FSS0_0].rprcOffset != BOOTLOADER_INVALID_ID)
            {
                status = Bootloader_rprcImageLoad(bootHandle, &bootImageInfo.cpuInfo[CSL_CORE_ID_R5FSS0_0]);
            }
            /* Reset self cluster, both Core0 and Core 1. Init RAMs and run the app  */
            status = Bootloader_runSelfCpu(bootHandle, &bootImageInfo);
        }

        /* it should not return here, if it does, then there was some error */
        Bootloader_close(bootHandle);
    }

    if(status != SystemP_SUCCESS)
    {
        DebugP_log("Some tests have failed!!\r\n");
    }

    Board_deinit();
    System_deinit();

    return 0;
}
