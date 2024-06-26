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

#define BOOTLOADER_SOC_APP_CERT_SIZE (0x1000)

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
        .defaultClockHz = (uint32_t)(300*1000000),
        .coreName       = "r5f0-0",
    },

    {
        .defaultClockHz = (uint32_t)(300*1000000),
        .coreName       = "r5f0-1",
    },

    {
        .defaultClockHz = (uint32_t)(360*1000000),
        .coreName       = "c66ss0",
    },

    {
        .defaultClockHz = (uint32_t)(200*1000000),
        .coreName       = "r4",
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
uint32_t gR5TopPbistRinfol = 0U;
uint32_t gR5TopPBISTRinfou = 0U;

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
    if (cpuId == CSL_CORE_ID_RSS_R4)
    {
        SOC_rcmGetRssClkFreq((uint32_t *)&clkRate);
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
    if(cpuId == CSL_CORE_ID_C66SS0)
    {
        SOC_rcmSetDspClock(SOC_RcmDspClockSource_DPLL_DSP_HSDIV0_CLKOUT1, cpuHz);
    }
    return SystemP_SUCCESS;
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
                SOC_rcmR5ConfigLockStep();
            }
            Bootloader_socMemInitCpu(cpuId);
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
        case CSL_CORE_ID_RSS_R4:
            SOC_rcmBSSControl();
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
                SOC_rcmStartMemInitDSSL2(SOC_RCM_MEMINIT_DSSL2_MEMBANK_ALL);
                SOC_rcmStartMemInitDSSL3(SOC_RCM_MEMINIT_DSSL3_MEMBANK_ALL);
                SOC_rcmWaitMemInitDSSL2(SOC_RCM_MEMINIT_DSSL2_MEMBANK_ALL);
                SOC_rcmWaitMemInitDSSL3(SOC_RCM_MEMINIT_DSSL3_MEMBANK_ALL);
                SOC_rcmMemInitDssMailboxMemory();
                gDssMemInitDone = TRUE;
            }
            break;
        case CSL_CORE_ID_RSS_R4:
            SOC_rcmStartMeminitTCMBSS();
            SOC_rcmWaitMeminitTCMBSS();
            SOC_rcmStartMeminitStaticBSS();
            SOC_rcmWaitMeminitStaticBSS();
            /* Exclude Shared memory init for ES2.0 devices. */
            if(SOC_rcmGetEfusePGVer() != SOC_RCM_ES2_PG_VER)
            {
                SOC_rcmStartMeminitSharedBSS();
                SOC_rcmWaitMeminitSharedBSS();
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
        case CSL_CORE_ID_RSS_R4:
            SOC_rcmPopulateBSSControl();
            /* Clear MSS_CTRL.MSS_PBIST_KEY_RST[3:0] before unhalting BSS Core. */
            if(SOC_rcmGetEfusePGVer() == SOC_RCM_ES2_PG_VER)
            {
                /* Save and publish TOP PBIST peripheral memories executed by RBL. */
                gR5TopPbistRinfol = CSL_REG32_RD((CSL_TOP_PBIST_U_BASE + CSL_PBIST_PBIST_RINFOL));
                gR5TopPBISTRinfou = CSL_REG32_RD((CSL_TOP_PBIST_U_BASE + CSL_PBIST_PBIST_RINFOU));
                DebugP_logInfo("PBIST memory tests exeucted by RBL, RINFOL : 0x%X and RINFOU : 0x%X\r\n",
                                (uint32_t)gR5TopPbistRinfol, (uint32_t)gR5TopPBISTRinfou);

                /* Disable the Top PBIST Self-Test Key */
                SOC_rcmDisableTopPbist();
            }
            SOC_rcmBSSR4Unhalt();

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
    SOC_RcmEfuseBootFreqConfig bootFreqEfuseCfg;
    int32_t retVal;
    uint32_t defaultR5Clk;

    retVal = SOC_rcmSetHSDivMux(SOC_RcmHSDIVClkOutMuxId_DPLL_CORE_OUT2,
                                SOC_RcmHSDIVClkOutMuxClockSource_DPLL_CORE_HSDIV0_CLKOUT2_PreMux);
    DebugP_assert(retVal == SystemP_SUCCESS);

    hsDivCfg.hsdivOutEnMask = (SOC_RCM_PLL_HSDIV_OUTPUT_ENABLE_1 |
                                SOC_RCM_PLL_HSDIV_OUTPUT_ENABLE_2);
    /* Configure CLKOUT1 to DSS PLL Fout/2. Divider is hsDivOut + 1 so set 1 */
    hsDivCfg.hsDivOutFreqHz[1] = SOC_RCM_FREQ_MHZ2HZ(360U);
    hsDivCfg.hsDivOutFreqHz[2] = SOC_RCM_FREQ_MHZ2HZ(240U);
    SOC_rcmDspPllConfig(SOC_RcmPllFoutFreqId_CLK_720MHZ, &hsDivCfg);
    retVal = SOC_rcmSetHSDivMux(SOC_RcmHSDIVClkOutMuxId_DPLL_DSP_OUT1,
                        SOC_RcmHSDIVClkOutMuxClockSource_DPLL_DSP_HSDIV0_CLKOUT1_PreMux);
    DebugP_assert(retVal == SystemP_SUCCESS);
    retVal = SOC_rcmSetHSDivMux(SOC_RcmHSDIVClkOutMuxId_DPLL_DSP_OUT2,
                        SOC_RcmHSDIVClkOutMuxClockSource_DPLL_DSP_HSDIV0_CLKOUT2_PreMux);
    DebugP_assert(retVal == SystemP_SUCCESS);

    hsDivCfg.hsdivOutEnMask = SOC_RCM_PLL_HSDIV_OUTPUT_ENABLE_1;
    /* Configure CLKOUT1 to DSS PLL Fout/2. Divider is hsDivOut + 1 so set 1 */
    hsDivCfg.hsDivOutFreqHz[1] = SOC_RCM_FREQ_MHZ2HZ(200U);
    SOC_rcmPerPllConfig(SOC_RcmPllFoutFreqId_CLK_1800MHZ, &hsDivCfg);
    retVal = SOC_rcmSetHSDivMux(SOC_RcmHSDIVClkOutMuxId_DPLL_PER_OUT1,
                        SOC_RcmHSDIVClkOutMuxClockSource_DPLL_PER_HSDIV0_CLKOUT1_PreMux);
    DebugP_assert(retVal == SystemP_SUCCESS);

    SOC_rcmGetEfuseBootFrequency(&bootFreqEfuseCfg);

    SOC_rcmSetR5Clock(bootFreqEfuseCfg.r5FreqHz, bootFreqEfuseCfg.sysClkFreqHz);

    SOC_rcmSetRssClkFreq(SOC_RcmRssClkSrcId_DPLL_PER_HSDIV0_CLKOUT1_MUXED, SOC_RCM_FREQ_MHZ2HZ(200U));

    /* For AWR change the UART clock to PER HSDIV CLKOUT1 insted of SYSCLK.
     * R5 clock and Sysclk is updated during boot up */
    SOC_rcmSetPeripheralClock(SOC_RcmPeripheralId_MSS_SCIA,
                              SOC_RcmPeripheralClockSource_DPLL_PER_HSDIV0_CLKOUT1,
                              SOC_RCM_FREQ_MHZ2HZ(200U));

    /* Switch R5F clock source to XTAL */
    SOC_rcmSwitchR5Clock(SOC_RcmR5ClockSource_XTAL_CLK, 0U);

    /* CORE ADPLL reconfiguration. */
    hsDivCfg.hsdivOutEnMask = (SOC_RCM_PLL_HSDIV_OUTPUT_ENABLE_1 |
                               SOC_RCM_PLL_HSDIV_OUTPUT_ENABLE_2 |
                               SOC_RCM_PLL_HSDIV_OUTPUT_ENABLE_3);
    hsDivCfg.hsDivOutFreqHz[1] = SOC_RCM_FREQ_MHZ2HZ(200);
    hsDivCfg.hsDivOutFreqHz[2] = gCoreBootInfo[CSL_CORE_ID_R5FSS0_0].defaultClockHz;
    hsDivCfg.hsDivOutFreqHz[3] = SOC_RCM_FREQ_MHZ2HZ(200);

    SOC_rcmCoreDpllConfig(SOC_RcmPllFoutFreqId_CLK_600MHZ, &hsDivCfg);

    /* Change the CORE clock source back to DPLL CORE HSDIV0 CLKOUT2. */
    retVal = SOC_rcmSetHSDivMux(SOC_RcmHSDIVClkOutMuxId_DPLL_CORE_OUT2,
                                SOC_RcmHSDIVClkOutMuxClockSource_DPLL_CORE_HSDIV0_CLKOUT2_PreMux);
    DebugP_assert(retVal == SystemP_SUCCESS);

    /* Switch R5F clock source to COREDPLL CLKOUT2 */
    SOC_rcmSwitchR5Clock(SOC_RcmR5ClockSource_DPLL_CORE_HSDIV0_CLKOUT2, 0U);

    defaultR5Clk = gCoreBootInfo[CSL_CORE_ID_R5FSS0_0].defaultClockHz;
    SOC_rcmSetR5Clock(defaultR5Clk, defaultR5Clk/2);
}

void Bootloader_socConfigurePllPostApllSwitch(void)
{
    int32_t retVal;

    /* Set Fast Charge Bias */
    SOC_rcmSetFastchargeBias();

    /*Configure CBUFF/Aurora*/
    /*Enable the clock to the HSI Clock mux :*/
    SOC_rcmCoreDpllHSDivOutEnable(0, 0x2);
    /* Switch HSI clock source to SOC_RcmHSIClockSource_PLL_PER_CLK */
    retVal = SOC_rcmSetHSIClock (SOC_RcmHSIClockSource_PLL_PER_CLK,
                                    SOC_RCM_FREQ_MHZ2HZ(600U));
    DebugP_assert(retVal == SystemP_SUCCESS);

    SOC_rcmConfigEthMacIf();

    /* APLL 1.2 */
    SOC_rcmApllHSDivDisableOutput(SOC_RcmAPLLID_1P2G, SOC_RCM_PLL_HSDIV_OUTPUT_IDX0);
    SOC_rcmApllHSDivDisableOutput(SOC_RcmAPLLID_1P2G, SOC_RCM_PLL_HSDIV_OUTPUT_IDX1);
    SOC_rcmApllHSDivDisableOutput(SOC_RcmAPLLID_1P2G, SOC_RCM_PLL_HSDIV_OUTPUT_IDX2);

    /* APLL 1.8 */
    SOC_rcmApllHSDivDisableOutput(SOC_RcmAPLLID_1P8G, SOC_RCM_PLL_HSDIV_OUTPUT_IDX0);
    SOC_rcmApllHSDivDisableOutput(SOC_RcmAPLLID_1P8G, SOC_RCM_PLL_HSDIV_OUTPUT_IDX1);
    SOC_rcmApllHSDivDisableOutput(SOC_RcmAPLLID_1P8G, SOC_RCM_PLL_HSDIV_OUTPUT_IDX2);
    SOC_rcmApllHSDivDisableOutput(SOC_RcmAPLLID_1P8G, SOC_RCM_PLL_HSDIV_OUTPUT_IDX3);

    /* COREDPLL CLOCKOUT0 */
    SOC_rcmDpllHSDivDisableOutput(SOC_RCM_DPLL_CORE, SOC_RCM_PLL_HSDIV_OUTPUT_IDX0);

    /* PER PLL */
    SOC_rcmDpllHSDivDisableOutput(SOC_RCM_DPLL_PER, SOC_RCM_PLL_HSDIV_OUTPUT_IDX0);
    SOC_rcmDpllHSDivDisableOutput(SOC_RCM_DPLL_PER, SOC_RCM_PLL_HSDIV_OUTPUT_IDX2);
    SOC_rcmDpllHSDivDisableOutput(SOC_RCM_DPLL_PER, SOC_RCM_PLL_HSDIV_OUTPUT_IDX3);

    /* DSP PLL */
    SOC_rcmDpllHSDivDisableOutput(SOC_RCM_DPLL_DSS, SOC_RCM_PLL_HSDIV_OUTPUT_IDX0);
    SOC_rcmDpllHSDivDisableOutput(SOC_RCM_DPLL_DSS, SOC_RCM_PLL_HSDIV_OUTPUT_IDX3);
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