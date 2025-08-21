/*
 *  Copyright (C) 2021-2024 Texas Instruments Incorporated
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

#include <string.h>
#include <drivers/bootloader.h>
#include <drivers/bootloader/bootloader_priv.h>
#include <drivers/bootloader/soc/bootloader_soc.h>
#include <kernel/dpl/CacheP.h>
#include <kernel/dpl/HwiP.h>
#include <security_common/drivers/hsmclient/hsmclient.h>

#define BOOTLOADER_R5SS_FREQ_200MHz (1U)

extern SecureBoot_Stream_t gSecureBootStreamArray[];
extern HsmClient_t gHSMClient ;

uint32_t gStreamId = 0;

Bootloader_resMemSections gResMemSection =
{
    .numSections    = 1,
    .memSection[0].memStart   = 0x70002000,
    .memSection[0].memEnd     = 0x70040000,
};

Bootloader_CoreBootInfo gCoreBootInfo[]                                                                             =
{
    {
        .defaultClockHz = (uint32_t)(400*1000000),
        .coreName       = "r5f0-0",
    },

    {
        .defaultClockHz = (uint32_t)(400*1000000),
        .coreName       = "r5f0-1",
    },
    {
        .defaultClockHz = (uint32_t)(400*1000000),
        .coreName       = "r5f1-0",
    },

    {
        .defaultClockHz = (uint32_t)(400*1000000),
        .coreName       = "r5f1-1",
    },
};

/* list the R5F cluster where this bootloader runs, this is fixed to R5FSS0-0, R5FSS0-1 in this SOC */
uint32_t gBootloaderSelfCpuList[] = {
    CSL_CORE_ID_R5FSS0_0,
    BOOTLOADER_INVALID_ID,
};

Bootloader_CoreAddrTranslateInfo gAddrTranslateInfo[] =
{
    /* CSL_CORE_ID_R5FSS0_0 */
    {
        .numRegions = 2,
        .addrRegionInfo =
        {
            {
                .cpuLocalAddr = CSL_MSS_TCMA_RAM_BASE,
                .socAddr      = CSL_MSS_TCMA_RAM_BASE,
                .regionSize   = CSL_MSS_TCMA_RAM_SIZE * 2U,
            },
            {
                .cpuLocalAddr = CSL_MSS_TCMB_RAM_BASE,
                .socAddr      = CSL_MSS_TCMB_RAM_BASE,
                .regionSize   = CSL_MSS_TCMB_RAM_SIZE * 2U,
            },
        },
    },

    /* CSL_CORE_ID_R5FSS0_1 */
    {
        .numRegions = 2,
        .addrRegionInfo =
        {
            {
                .cpuLocalAddr = CSL_MSS_TCMA_RAM_BASE,
                .socAddr      = CSL_MSS_TCMA_RAM_BASE + CSL_MSS_TCMA_RAM_SIZE,
                .regionSize   = CSL_MSS_TCMA_RAM_SIZE,
            },
            {
                .cpuLocalAddr = CSL_MSS_TCMB_RAM_BASE,
                .socAddr      = CSL_MSS_TCMB_RAM_BASE + CSL_MSS_TCMB_RAM_SIZE,
                .regionSize   = CSL_MSS_TCMB_RAM_SIZE,
            },
        },
    },

    /* CSL_CORE_ID_R5FSS1_0 */
    {
        .numRegions = 2,
        .addrRegionInfo =
        {
            {
                .cpuLocalAddr = CSL_MSS_TCMA_RAM_BASE,
                .socAddr      = CSL_R5SS1_CORE0_TCMA_U_BASE,
                .regionSize   = CSL_MSS_TCMA_RAM_SIZE * 2U,
            },
            {
                .cpuLocalAddr = CSL_MSS_TCMB_RAM_BASE,
                .socAddr      = CSL_R5SS1_CORE0_TCMB_U_BASE,
                .regionSize   = CSL_MSS_TCMB_RAM_SIZE * 2U,
            },
        },
    },

    /* CSL_CORE_ID_R5FSS1_1 */
    {
        .numRegions = 2,
        .addrRegionInfo =
        {
            {
                .cpuLocalAddr = CSL_MSS_TCMA_RAM_BASE,
                .socAddr      = CSL_R5SS1_CORE1_TCMA_U_BASE,
                .regionSize   = CSL_MSS_TCMA_RAM_SIZE,
            },
            {
                .cpuLocalAddr = CSL_MSS_TCMB_RAM_BASE,
                .socAddr      = CSL_R5SS1_CORE1_TCMB_U_BASE,
                .regionSize   = CSL_MSS_TCMB_RAM_SIZE,
            },
        },
    },
};

CSL_top_ctrlRegs * ptrTopCtrlRegs = (CSL_top_ctrlRegs *)CSL_TOP_CTRL_U_BASE;

volatile uint32_t gR5ss0MemInitDone = FALSE, gR5ss1MemInitDone = FALSE, gR5ss1PORDone = FALSE;
volatile uint32_t gR5ss0Core1ImagePresent = FALSE, gR5ss1Core1ImagePresent = FALSE;

uint32_t Bootloader_socRprcToCslCoreId(uint32_t rprcCoreId)
{
    uint32_t cslCoreId = CSL_CORE_ID_MAX;
    uint32_t rprcCoreIds[CSL_CORE_ID_MAX] =
    {
        0U, 1U, 2U, 3U
    };
    uint32_t i;

    for(i = 0U; i < CSL_CORE_ID_MAX; i++)
    {
        if(rprcCoreId == rprcCoreIds[i])
        {
            cslCoreId = i;
            break;
        }
    }

    return cslCoreId;
}

uint32_t Bootloader_socCpuGetClkDefault(uint32_t cpuId)
{
    uint32_t defClock = 0U;

    if(cpuId < CSL_CORE_ID_MAX)
    {
        defClock = gCoreBootInfo[cpuId].defaultClockHz;
    }

    return defClock;
}

char* Bootloader_socGetCoreName(uint32_t cpuId)
{
    char *pName = NULL;

    if(cpuId < CSL_CORE_ID_MAX)
    {
        pName = gCoreBootInfo[cpuId].coreName;
    }

    return pName;
}

uint64_t Bootloader_socCpuGetClock(uint32_t cpuId)
{
    int32_t status = SystemP_FAILURE;
    uint64_t clkRate = 0;
    if ((cpuId == CSL_CORE_ID_R5FSS0_0) || (cpuId == CSL_CORE_ID_R5FSS0_1) ||
        (cpuId == CSL_CORE_ID_R5FSS1_0) || (cpuId == CSL_CORE_ID_R5FSS1_1))
    {
        clkRate = SOC_rcmGetR5Clock(cpuId);
    }
    if (clkRate != 0)
    {
        status = SystemP_SUCCESS;
    }
    if(status != SystemP_SUCCESS)
    {
        DebugP_logError("CPU clock get failed for %s\r\n", Bootloader_socGetCoreName(cpuId));
    }
    return clkRate;
}

int32_t Bootloader_socCpuRequest(uint32_t cpuId)
{
    return SystemP_SUCCESS;
}

int32_t Bootloader_socCpuRelease(uint32_t cpuId)
{
    return SystemP_SUCCESS;
}

int32_t Bootloader_socCpuSetClock(uint32_t cpuId, uint32_t cpuHz)
{
    int32_t status = SystemP_FAILURE;
    switch (cpuId)
    {
        case CSL_CORE_ID_R5FSS0_0:
        case CSL_CORE_ID_R5FSS0_1:
        case CSL_CORE_ID_R5FSS1_0:
        case CSL_CORE_ID_R5FSS1_1:
            status = SOC_rcmSetR5Clock(cpuHz, cpuHz/2, cpuId);
            break;
    }
    return status;
}

int32_t Bootloader_socCpuPowerOnReset(uint32_t cpuId,void *socCoreOpMode)
{
    int32_t status = SystemP_SUCCESS;
    switch (cpuId)
    {
        case CSL_CORE_ID_R5FSS0_0:
            if (gR5ss0Core1ImagePresent == FALSE)
            {
            /* Core 1 image is not present or not booted yet.
               ConfigureR5 in lock step mode. */
                if (socCoreOpMode != NULL)
                {
                    Bootloader_socCoreOpModeConfig *config = (Bootloader_socCoreOpModeConfig *)socCoreOpMode;
                    /* Check for operating mode configuration as set in syscfg */
                    if (config->r5fss0_opMode == BOOTLOADER_OPMODE_LOCKSTEP){
                        SOC_rcmR5ConfigLockStep(cpuId);
                        Bootloader_socMemInitCpu(cpuId);
                    }
                    else
                    {
                    /* ConfigureR5 in Standalone (Dual Core) mode as set in syscfg*/
                        SOC_rcmR5ConfigDualCore(CSL_CORE_ID_R5FSS0_1);
                        Bootloader_socMemInitCpu(CSL_CORE_ID_R5FSS0_1);
                    }
                }
                else
                {
                    SOC_rcmR5ConfigLockStep(cpuId);
                    Bootloader_socMemInitCpu(cpuId);
                }
            }
            break;
        case CSL_CORE_ID_R5FSS0_1:
            /* Configure the R5 to dual core boot. Actual switch happens
               At the end of SBL execution when the core reset is done. */
            SOC_rcmR5ConfigDualCore(cpuId);
            gR5ss0Core1ImagePresent = TRUE;
            Bootloader_socMemInitCpu(cpuId);
            break;
        case CSL_CORE_ID_R5FSS1_0:
            if (gR5ss1Core1ImagePresent == FALSE)
            {
            /* Core 1 image is not present or not booted yet.
               ConfigureR5 in lock step mode. */
                if (socCoreOpMode != NULL){
                    Bootloader_socCoreOpModeConfig *config = (Bootloader_socCoreOpModeConfig *)socCoreOpMode;
                    if (config->r5fss1_opMode == BOOTLOADER_OPMODE_LOCKSTEP){
                        SOC_rcmR5ConfigLockStep(cpuId);
                        Bootloader_socMemInitCpu(cpuId);
                    }
                    else
                    {
                    /* ConfigureR5 in Standalone mode as set in syscfg*/
                        SOC_rcmR5ConfigDualCore(CSL_CORE_ID_R5FSS1_1);
                        if (gR5ss1PORDone == FALSE)
                        {
                            SOC_rcmR5SS1PowerOnReset();
                            SOC_rcmR5SS1TriggerReset();
                            gR5ss1PORDone = TRUE;
                        }
                        Bootloader_socMemInitCpu(CSL_CORE_ID_R5FSS1_1);
                    }
                }
                else{
                    SOC_rcmR5ConfigLockStep(cpuId);
                    Bootloader_socMemInitCpu(cpuId);
                }
            }
        break;
        case CSL_CORE_ID_R5FSS1_1:
            SOC_rcmR5ConfigDualCore(cpuId);
            gR5ss1Core1ImagePresent = TRUE;
            if (gR5ss1PORDone == FALSE)
            {
                SOC_rcmR5SS1PowerOnReset();
                SOC_rcmR5SS1TriggerReset();
                gR5ss1PORDone = TRUE;
            }
            Bootloader_socMemInitCpu(cpuId);
            break;
    }
    return status;
}

int32_t Bootloader_socMemInitCpu(uint32_t cpuId)
{
    int32_t status = SystemP_SUCCESS;

    switch(cpuId) {
        case CSL_CORE_ID_R5FSS0_0:
        case CSL_CORE_ID_R5FSS0_1:
            if (gR5ss0MemInitDone == FALSE)
            {
                SOC_rcmStartMemInitTCMA(cpuId);
                SOC_rcmWaitMemInitTCMA(cpuId);
                SOC_rcmStartMemInitTCMB(cpuId);
                SOC_rcmWaitMemInitTCMB(cpuId);
                gR5ss0MemInitDone = TRUE;
            }
            break;
        case CSL_CORE_ID_R5FSS1_0:
        case CSL_CORE_ID_R5FSS1_1:
            if (gR5ss1MemInitDone == FALSE)
            {
                SOC_rcmStartMemInitTCMA(cpuId);
                SOC_rcmWaitMemInitTCMA(cpuId);
                SOC_rcmStartMemInitTCMB(cpuId);
                SOC_rcmWaitMemInitTCMB(cpuId);
                gR5ss1MemInitDone = TRUE;
            }
            break;
        default:
            break;
    }

    return status;
}

int32_t Bootloader_socCpuResetRelease(uint32_t cpuId, uintptr_t entryPoint)
{
    int32_t status = SystemP_SUCCESS;

    switch (cpuId)
    {
        case CSL_CORE_ID_R5FSS0_1:
            SOC_rcmCoreR5FUnhalt(cpuId);
            break;
        case CSL_CORE_ID_R5FSS1_0:
        case CSL_CORE_ID_R5FSS1_1:
            SOC_rcmCoreR5FUnhalt(cpuId);
            break;
        default:
            break;
    }
    return status;
}

int32_t Bootloader_socCpuResetReleaseSelf(void)
{
    SOC_rcmR5SS0PowerOnReset();
    SOC_rcmR5SS0TriggerReset();

    return SystemP_SUCCESS;
}

int32_t Bootloader_socCpuSetEntryPoint(uint32_t cpuId, uintptr_t entryPoint)
{
    return SystemP_SUCCESS;
}

uint32_t* Bootloader_socGetSelfCpuList(void)
{
    return &gBootloaderSelfCpuList[0];
}

int32_t Bootloader_socSecHandover(void)
{
    return SystemP_SUCCESS;
}

uint32_t Bootloader_socTranslateSectionAddr(uint32_t cslCoreId, uint32_t addr)
{
    uint32_t outputAddr = addr;

    Bootloader_CoreAddrTranslateInfo *addrTranslateInfo = &gAddrTranslateInfo[cslCoreId];

    uint32_t i;

    for(i=0; i<addrTranslateInfo->numRegions; i++)
    {
        uint32_t cpuLocalAddr = addrTranslateInfo->addrRegionInfo[i].cpuLocalAddr;
        uint32_t socAddr      = addrTranslateInfo->addrRegionInfo[i].socAddr;
        uint32_t regionSize   = addrTranslateInfo->addrRegionInfo[i].regionSize;

        if((addr >= cpuLocalAddr) && (addr <  cpuLocalAddr + regionSize))
        {
            uint32_t offset = addr - cpuLocalAddr;
            outputAddr = socAddr + offset;
            break;
        }
    }

    return outputAddr;
}

// void Bootloader_socConfigurePll(void)
// {
//     SOC_RcmPllHsDivOutConfig hsDivCfg;
//     uint32_t r5ClkSrc_restore;

//     /* Pre Requisite Sequence to relock core pll needs to be done */
//     r5ClkSrc_restore = SOC_rcmCoreApllRelockPreRequisite();

//     hsDivCfg.hsdivOutEnMask = (RCM_PLL_HSDIV_OUTPUT_ENABLE_0 |
//                               RCM_PLL_HSDIV_OUTPUT_ENABLE_1 |
//                               RCM_PLL_HSDIV_OUTPUT_ENABLE_2);
//     hsDivCfg.hsDivOutFreqHz[0] = SOC_RCM_FREQ_MHZ2HZ(400U);
//     hsDivCfg.hsDivOutFreqHz[1] = SOC_RCM_FREQ_MHZ2HZ(500U);
//     hsDivCfg.hsDivOutFreqHz[2] = SOC_RCM_FREQ_MHZ2HZ(400U);
//     SOC_rcmCoreApllConfig(RCM_PLL_FOUT_FREQID_CLK_2000MHZ, &hsDivCfg);

//     hsDivCfg.hsdivOutEnMask = (RCM_PLL_HSDIV_OUTPUT_ENABLE_0 |
//                               RCM_PLL_HSDIV_OUTPUT_ENABLE_1);
//     hsDivCfg.hsDivOutFreqHz[0] = SOC_RCM_FREQ_MHZ2HZ(160U);
//     hsDivCfg.hsDivOutFreqHz[1] = SOC_RCM_FREQ_MHZ2HZ(192U);
//     SOC_rcmPerApllConfig(RCM_PLL_FOUT_FREQID_CLK_1920MHZ, &hsDivCfg);

//     /* Restore R5F source clock*/
//     SOC_rcmSetR5ClockSource(r5ClkSrc_restore);
// }

void Bootloader_socInitL2MailBoxMemory(void)
{
    SOC_rcmMemInitL2Memory();
    SOC_rcmMemInitMailboxMemory();
}

Bootloader_resMemSections* Bootloader_socGetSBLMem(void)
{
    return &gResMemSection;
}

int32_t Bootloader_socAuthImage(uint32_t certLoadAddr)
{
    int32_t status = SystemP_FAILURE;
    HsmClient_t client ;
    status = HsmClient_register(&client, BOOTLOADER_CLIENT_ID);

    /* Request TIFS-MCU to authenticate (and decrypt if mentioned in the x509 cert) the image */
    status = HsmClient_procAuthBoot(&client, (uint8_t *)certLoadAddr, BOOTLOADER_SOC_APP_CERT_SIZE, SystemP_WAIT_FOREVER);

    return status;
}

uint32_t Bootloader_socIsAuthRequired(void)
{
    uint32_t isAuthRequired = TRUE;

    if(ptrTopCtrlRegs->EFUSE_DEVICE_TYPE == BOOTLOADER_DEVTYPE_HSSE)
    {
        isAuthRequired = TRUE;
    }
    else
    {
        isAuthRequired = FALSE;
    }

    return isAuthRequired;
}

void Bootloader_socGetBootSeqOid(uint8_t* boot_seq_oid){
    uint8_t boot_seq[] = {0x06, 0x09, 0x2B, 0x06, 0x01, 0x04, 0x01, 0x82, 0x26, 0x01, 0x01};
    memcpy(boot_seq_oid, boot_seq, sizeof(boot_seq));
}

void Bootloader_socSetAutoClock()
{
    uint32_t eFuseFreq = (((ptrTopCtrlRegs->EFUSE1_ROW_12) & CSL_TOP_CTRL_EFUSE1_ROW_12_EFUSE1_ROW_12_R5SS_FREQ_MASK) >>
                          CSL_TOP_CTRL_EFUSE1_ROW_12_EFUSE1_ROW_12_R5SS_FREQ_SHIFT);
    if (eFuseFreq == BOOTLOADER_R5SS_FREQ_200MHz)
    {

        uint32_t cpuFreq = 200 * 1000000, sysClkFreq = 200 * 1000000;
        uint32_t testCpuFreq;

        SOC_rcmsetR5SysClock(cpuFreq, sysClkFreq, CSL_CORE_ID_R5FSS0_0);
        testCpuFreq = SOC_rcmGetR5Clock(CSL_CORE_ID_R5FSS0_0);

        DebugP_assert(testCpuFreq == 200 * 1000000);
    }
    else
    {
        Bootloader_socCpuSetClock(CSL_CORE_ID_R5FSS0_0, (uint32_t)(400*1000000));
    }
}

int32_t Bootloader_socCpuSetAppEntryPoint(uint32_t cpuId, uintptr_t entryPoint)
{
   int32_t status = SystemP_SUCCESS;
   /* dummy api call */

    return status;
}

int32_t Bootloader_authStart(uintptr_t startAddr, uint32_t size)
{
    int32_t status = SystemP_FAILURE;

    gSecureBootStreamArray[gStreamId].dataIn = (uint8_t *)SOC_virtToPhy((void *)startAddr);
    gSecureBootStreamArray[gStreamId].dataLen = size;
    gSecureBootStreamArray[gStreamId].canBeEncrypted = BOOTLOADER_APP_SEGMENT_CANNOTBE_ENCRYPTED;

    status = HsmClient_procAuthBootStart(&gHSMClient, &gSecureBootStreamArray[gStreamId]);

    gStreamId++;

    return status;
}

int32_t Bootloader_authUpdate(uintptr_t startAddr, uint32_t size, uint8_t enc)
{
	int32_t status = SystemP_FAILURE;

    gSecureBootStreamArray[gStreamId].dataIn = (uint8_t *)SOC_virtToPhy((void *)startAddr);
    gSecureBootStreamArray[gStreamId].dataLen = size;
    gSecureBootStreamArray[gStreamId].canBeEncrypted = enc;

    status = HsmClient_procAuthBootUpdate(&gHSMClient, &gSecureBootStreamArray[gStreamId]);

    gStreamId++;

    return status;
}

int32_t Bootloader_authFinish(void)
{
    int32_t status = SystemP_FAILURE;

    gSecureBootStreamArray[gStreamId].dataIn = 0x0U;
    gSecureBootStreamArray[gStreamId].dataLen = 0x0U;
    gSecureBootStreamArray[gStreamId].canBeEncrypted = BOOTLOADER_APP_SEGMENT_CANNOTBE_ENCRYPTED;

    status = HsmClient_procAuthBootFinish(&gHSMClient, &gSecureBootStreamArray[gStreamId]);

    return status;
}