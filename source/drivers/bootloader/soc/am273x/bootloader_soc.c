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

#define BOOTLOADER_SOC_RSV_MEM_START   (0x10200000)
#define BOOTLOADER_SOC_RSV_MEM_END     (0x10220000)

#define BOOTLOADER_SOC_APP_CERT_SIZE   (0x1000)

#define BOOTLOADER_SOC_CLK_FREQ_400MHZ (uint32_t)(400*1000000)
#define BOOTLOADER_SOC_CLK_FREQ_200MHZ (uint32_t)(200*1000000)
#define BOOTLOADER_SOC_CLK_FREQ_450MHZ (uint32_t)(450*1000000)
#define BOOTLOADER_SOC_CLK_FREQ_550MHZ (uint32_t)(550*1000000)
#define BOOTLOADER_SOC_CLK_FREQ_300MHZ (uint32_t)(300*1000000)
#define BOOTLOADER_SOC_CLK_FREQ_600MHZ (uint32_t)(600*1000000)

Bootloader_resMemSections gResMemSection =
{
    .numSections    = 2,
    .memSection[0].memStart   = 0x10200000,
    .memSection[0].memEnd     = 0x10220000,
    .memSection[1].memStart   = 0x88100000,
    .memSection[1].memEnd     = 0x88200000
};

Bootloader_CoreBootInfo gCoreBootInfo[] =
{
    {
        .defaultClockHz = BOOTLOADER_SOC_CLK_FREQ_400MHZ,
        .coreName       = "r5f0-0",
    },

    {
        .defaultClockHz = BOOTLOADER_SOC_CLK_FREQ_400MHZ,
        .coreName       = "r5f0-1",
    },

    {
        .defaultClockHz = BOOTLOADER_SOC_CLK_FREQ_450MHZ,
        .coreName       = "c66ss0",
    },
};

/* list the R5F cluster where this bootloader runs, this is fixed to R5FSS0-0, R5FSS0-1 in this SOC */
uint32_t gBootloaderSelfCpuList[] = {
    CSL_CORE_ID_R5FSS0_0,
    CSL_CORE_ID_R5FSS0_1,
    BOOTLOADER_INVALID_ID,
};

uint32_t gR5ss0MemInitDone = FALSE;
uint32_t gDssMemInitDone = FALSE;
uint32_t gR5ss0Core1ImagePresent = FALSE;

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
    if ((cpuId == CSL_CORE_ID_R5FSS0_0) || (cpuId == CSL_CORE_ID_R5FSS0_1))
    {
        clkRate = SOC_rcmGetR5Clock();
    }
    if (cpuId == CSL_CORE_ID_C66SS0)
    {
        clkRate = SOC_rcmGetDspClock();
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
            status = SOC_rcmSetR5Clock(cpuHz, cpuHz/2);
            break;
        case CSL_CORE_ID_C66SS0:
            status = SOC_rcmSetDspClock(SOC_RcmDspClockSource_DPLL_DSP_HSDIV0_CLKOUT1, cpuHz);
            break;
    }
    return status;
}

CSL_top_ctrlRegs * ptrTopCtrlRegs = (CSL_top_ctrlRegs *)CSL_TOP_CTRL_U_BASE;

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
                        SOC_rcmR5ConfigLockStep();
                        Bootloader_socMemInitCpu(cpuId);
                    }
                    else
                    {
                    /* ConfigureR5 in Standalone (Dual Core) mode as set in syscfg*/
                        SOC_rcmR5ConfigDualCore();
                        Bootloader_socMemInitCpu(CSL_CORE_ID_R5FSS0_1);
                        Bootloader_socCpuResetRelease(CSL_CORE_ID_R5FSS0_1, 0);
                    }
                }
                else
                {
                    SOC_rcmR5ConfigLockStep();
                    Bootloader_socMemInitCpu(cpuId);
                }
            }
            break;
        case CSL_CORE_ID_R5FSS0_1:
            /* Configure the R5 to dual core boot. Actual switch happens
               At the end of SBL execution when the core reset is done. */
            SOC_rcmR5ConfigDualCore();
            gR5ss0Core1ImagePresent = TRUE;
            Bootloader_socMemInitCpu(cpuId);
            break;
        case CSL_CORE_ID_C66SS0:
            SOC_rcmDspPowerOnReset();
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
                SOC_rcmStartMemInitTCMA();
                SOC_rcmWaitMemInitTCMA();
                SOC_rcmStartMemInitTCMB();
                SOC_rcmWaitMemInitTCMB();
                SOC_rcmMemInitMssMailboxMemory();
                gR5ss0MemInitDone = TRUE;
            }
            break;
        case CSL_CORE_ID_C66SS0:
            if (gDssMemInitDone == FALSE)
            {
                SOC_rcmStartMemInitDSSL3(SOC_RCM_MEMINIT_DSSL3_MEMBANK_ALL);
                SOC_rcmWaitMemInitDSSL3(SOC_RCM_MEMINIT_DSSL3_MEMBANK_ALL);
                SOC_rcmMemInitDssMailboxMemory();
                SOC_rcmStartMemInitDSSL2(SOC_RCM_MEMINIT_DSSL2_MEMBANK_ALL);
                SOC_rcmWaitMemInitDSSL2(SOC_RCM_MEMINIT_DSSL2_MEMBANK_ALL);
                gDssMemInitDone = TRUE;
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
            SOC_rcmCr5bUnhalt();
            break;
        case CSL_CORE_ID_C66SS0:
            SOC_rcmC66xStart();
            break;
        default:
            break;
    }
    return status;
}

int32_t Bootloader_socCpuResetReleaseSelf(void)
{
    SOC_rcmR5PowerOnReset();
    SOC_rcmR5TriggerReset();
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

void Bootloader_socConfigurePll(void)
{
    SOC_RcmPllHsDivOutConfig hsDivCfg;

    uint32_t eFuseRow11 = CSL_REG32_RD(BOOTLOADER_SOC_EFUSE_REG_ROW11);

    hsDivCfg.hsdivOutEnMask = SOC_RCM_PLL_HSDIV_OUTPUT_ENABLE_1 |
                                SOC_RCM_PLL_HSDIV_OUTPUT_ENABLE_2;
    /* Configure CLKOUT1 to DSS PLL Fout/2. Divider is hsDivOut + 1 so set 1 */
    uint32_t r5FreqEfuse = (eFuseRow11 & BOOTLOADER_SOC_R5_FREQ_MASK);
    hsDivCfg.hsDivOutFreqHz[1] = SOC_RCM_FREQ_MHZ2HZ(200U);
    if(r5FreqEfuse == BOOTLOADER_SOC_R5_PART_400MHZ)
    {
        gCoreBootInfo[CSL_CORE_ID_R5FSS0_0].defaultClockHz = BOOTLOADER_SOC_CLK_FREQ_400MHZ;
        gCoreBootInfo[CSL_CORE_ID_R5FSS0_1].defaultClockHz = BOOTLOADER_SOC_CLK_FREQ_400MHZ;
        hsDivCfg.hsDivOutFreqHz[2] = SOC_RCM_FREQ_MHZ2HZ(400U);
    }
    else
    {
        gCoreBootInfo[CSL_CORE_ID_R5FSS0_0].defaultClockHz = BOOTLOADER_SOC_CLK_FREQ_200MHZ;
        gCoreBootInfo[CSL_CORE_ID_R5FSS0_1].defaultClockHz = BOOTLOADER_SOC_CLK_FREQ_200MHZ;
        hsDivCfg.hsDivOutFreqHz[2] = SOC_RCM_FREQ_MHZ2HZ(200U);
    }
    SOC_rcmCoreApllHSDivConfig(&hsDivCfg);

    hsDivCfg.hsdivOutEnMask = SOC_RCM_PLL_HSDIV_OUTPUT_ENABLE_1;
    /* Configure CLKOUT1 to DSS PLL Fout/2. Divider is hsDivOut + 1 so set 1 */
    /* Check the DSP frequency eFUSE register to find out frequency supported by the part */

    uint8_t dspFreqEfuse = (eFuseRow11 & (BOOTLOADER_SOC_DSP_PART_MASK));

    switch(dspFreqEfuse)
    {
        case BOOTLOADER_SOC_DSP_PART_300MHZ:
            gCoreBootInfo[CSL_CORE_ID_C66SS0].defaultClockHz = BOOTLOADER_SOC_CLK_FREQ_300MHZ;
            hsDivCfg.hsDivOutFreqHz[1] = gCoreBootInfo[CSL_CORE_ID_C66SS0].defaultClockHz;
            SOC_rcmDspPllConfig(SOC_RcmPllFoutFreqId_CLK_900MHZ, &hsDivCfg);
            break;

        case BOOTLOADER_SOC_DSP_PART_550MHZ:
            gCoreBootInfo[CSL_CORE_ID_C66SS0].defaultClockHz = BOOTLOADER_SOC_CLK_FREQ_550MHZ;
            hsDivCfg.hsDivOutFreqHz[1] = gCoreBootInfo[CSL_CORE_ID_C66SS0].defaultClockHz;
            SOC_rcmDspPllConfig(SOC_RcmPllFoutFreqId_CLK_1650MHZ, &hsDivCfg);
            break;

        case BOOTLOADER_SOC_DSP_PART_450MHZ:
            gCoreBootInfo[CSL_CORE_ID_C66SS0].defaultClockHz = BOOTLOADER_SOC_CLK_FREQ_450MHZ;
            hsDivCfg.hsDivOutFreqHz[1] = gCoreBootInfo[CSL_CORE_ID_C66SS0].defaultClockHz;
            SOC_rcmDspPllConfig(SOC_RcmPllFoutFreqId_CLK_900MHZ, &hsDivCfg);
            break;

        default:
            gCoreBootInfo[CSL_CORE_ID_C66SS0].defaultClockHz = BOOTLOADER_SOC_CLK_FREQ_450MHZ;
            hsDivCfg.hsDivOutFreqHz[1] = gCoreBootInfo[CSL_CORE_ID_C66SS0].defaultClockHz;
            SOC_rcmDspPllConfig(SOC_RcmPllFoutFreqId_CLK_900MHZ, &hsDivCfg);
            break;
    }

    hsDivCfg.hsdivOutEnMask = (SOC_RCM_PLL_HSDIV_OUTPUT_ENABLE_1 |
                                SOC_RCM_PLL_HSDIV_OUTPUT_ENABLE_2 |
                                SOC_RCM_PLL_HSDIV_OUTPUT_ENABLE_3);
    /* Configure CLKOUT1 to DSS PLL Fout/2. Divider is hsDivOut + 1 so set 1 */
    hsDivCfg.hsDivOutFreqHz[1] = SOC_RCM_FREQ_MHZ2HZ(192U);
    hsDivCfg.hsDivOutFreqHz[2] = SOC_RCM_FREQ_MHZ2HZ(96U);
    hsDivCfg.hsDivOutFreqHz[3] = SOC_RCM_FREQ_MHZ2HZ(172.8);
    SOC_rcmPerPllConfig(SOC_RcmPllFoutFreqId_CLK_1728MHZ, &hsDivCfg);
}

uint32_t Bootloader_socRprcToCslCoreId(uint32_t rprcCoreId)
{
    /* both are same for am273x/awr294x. */
    return rprcCoreId;
}

Bootloader_resMemSections* Bootloader_socGetSBLMem(void)
{
    return &gResMemSection;
}

uint32_t Bootloader_socGetCoreVariant(void)
{
    uint32_t coreVariant = (CSL_REG32_RD(BOOTLOADER_SOC_EFUSE_REG_ROW11) & (BOOTLOADER_SOC_EFUSE_DUAL_CORE_DIS_MASK));
    return coreVariant;
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

int32_t Bootloader_socCpuSetAppEntryPoint(uint32_t cpuId, uintptr_t entryPoint)
{
   int32_t status = SystemP_SUCCESS;
   /* dummy api call */

    return status;
}

int32_t Bootloader_authStart(uintptr_t startAddr, uint32_t size)
{
    return SystemP_SUCCESS;
}

int32_t Bootloader_authUpdate(uintptr_t startAddr, uint32_t size, uint8_t enc)
{
    return SystemP_SUCCESS;
}

int32_t Bootloader_authFinish()
{
    return SystemP_SUCCESS;
}
