/******************************************************************************
 * Copyright (c) 2021-2022 Texas Instruments Incorporated - http://www.ti.com
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
 *
 *****************************************************************************/

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <stdint.h>
#include <string.h>
#include <kernel/dpl/DebugP.h>
#include <drivers/hw_include/hw_types.h>
#include <drivers/soc.h>
#include <drivers/ddr.h>
#include <drivers/ddr/v0/cdn_drv.h>
#include <drivers/ddr/v0/csl_emif.h>
#include <kernel/dpl/ClockP.h>
#include <kernel/dpl/CacheP.h>

#if defined (SOC_AM64X) || defined (SOC_AM243X)
#include <drivers/ddr/v0/soc/am64x_am243x/ddr_soc.h>
#endif

/* ========================================================================== */
/*                             Macros & Typedefs                              */
/* ========================================================================== */

#define DDR_CTL_REG_OFFSET              (0)
#define DDR_SRAM_MAX                    (512U)
#define DDR_TYPE_MASK                   (0xF00U)
#define DDR4_MEMORY                     (0xA00U)
#define LPDDR4_MEMORY                   (0xB00U)
/*FSP MAIN_CTRL_MMR register offsets*/
/* LPDDR4 FSP Clock Change Request Register - DDR Controller to request the DDR PLL clock frequency change (read only)*/
#define DDR_FSP_CLKCHNG_REQ    0x14080U
/* LPDDR4 FSP Clock Change Acknowledge Register - acknowledge a DDR PLL clock frequency change to the DDR Controller (r/w) */
#define DDR_FSP_CLKCHNG_ACK    0x140C0U

#define DDR_CTL_REG_BASE        0x0F308000U
#define DDR_PI_REG_BLOCK_OFFS   0x2000U
#define DDR_PI_0_SFR_OFFS       0x0U
#define DDR_CTL_0_SFR_OFFS      0x0U
#define DDR_PI_83_SFR_OFFS      0x14CU
#define DDR_CTL_342_SFR_OFFS    0x558U
/* Trigger Start bit*/
#define TRIGGER_START_BIT        0x00000B01U

#define KEY0                    0x68EF3490U
#define KEY1                    0xD172BC5AU

#define DDR_REQ_TYPE_0    0
#define DDR_REQ_TYPE_1    1
#define DDR_REQ_TYPE_2    2

/* ========================================================================== */
/*                 Internal Function Declarations                             */
/* ========================================================================== */


/* ========================================================================== */
/*                         Global Variables Declarations                      */
/* ========================================================================== */

static LPDDR4_Config gLpddrCfg;
static LPDDR4_PrivateData gLpddrPd;

/* ========================================================================== */
/*                         Extern Function declerations                       */
/* ========================================================================== */
extern void DDR_socEnableVttRegulator(void);

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */
static void DDR_setFreq(uint64_t freq)
{
    uint32_t module = TISCI_DEV_DDR16SS0;
    uint32_t clock = TISCI_DEV_DDR16SS0_DDRSS_DDR_PLL_CLK;
    int32_t status = SystemP_SUCCESS;

    status = SOC_moduleSetClockFrequency(module, clock, freq);
    if(status != SystemP_SUCCESS)
    {
        DebugP_logError("SOC_moduleSetClockFrequency failed !!!\n");
    }
}

static void DDR_changeFreqAck(DDR_Params *prms)
{
    uint32_t req, req_type, counter;

    /*unlock MMR reg 5*/
    /*Partition5 lockkey0*/
    HW_WR_REG32((CSL_CTRL_MMR0_CFG0_BASE + CSL_MAIN_CTRL_MMR_CFG0_LOCK5_KICK0), KEY0);
    /*Partition5 lockkey1*/
    HW_WR_REG32(( CSL_CTRL_MMR0_CFG0_BASE + CSL_MAIN_CTRL_MMR_CFG0_LOCK5_KICK1), KEY1);

    ClockP_usleep(500);

    for(counter = 0; counter < prms->fshcount; counter++)
    {
        req = (HW_RD_REG32(CSL_CTRL_MMR0_CFG0_BASE + DDR_FSP_CLKCHNG_REQ) & 0x80);
        while((req ) == 0x0)
        {
            req = (HW_RD_REG32(CSL_CTRL_MMR0_CFG0_BASE + DDR_FSP_CLKCHNG_REQ) & 0x80);
        }

        req_type = HW_RD_REG32(CSL_CTRL_MMR0_CFG0_BASE + DDR_FSP_CLKCHNG_REQ) & 0x03;

        if(req_type == DDR_REQ_TYPE_1)
        {
            DDR_setFreq(prms->clk1Freq);
        }
        else if(req_type == DDR_REQ_TYPE_2)
        {
            DDR_setFreq(prms->clk2Freq);
        }
        else if(req_type == DDR_REQ_TYPE_0)
        {
            DDR_setFreq(25000000);
        }

        /*set the ack bit*/
        HW_WR_REG32((CSL_CTRL_MMR0_CFG0_BASE + DDR_FSP_CLKCHNG_ACK), 0x1);
        while(((HW_RD_REG32((CSL_CTRL_MMR0_CFG0_BASE + DDR_FSP_CLKCHNG_REQ))) & 0x80) == 0x80);
        /*clear the ack bit */
        HW_WR_REG32((CSL_CTRL_MMR0_CFG0_BASE + DDR_FSP_CLKCHNG_ACK), 0x0);
    }
}

static int32_t DDR_setClock(DDR_Params *prms)
{
    int32_t status = SystemP_SUCCESS;

    if((HW_RD_REG32(DDR_CTL_REG_BASE) & DDR_TYPE_MASK) == DDR4_MEMORY)
    {
        /* Type is DDR4*/
        DDR_setFreq(prms->clk1Freq);
    }
    else
    {
        /* Type is LPDDR4. Start Frequency handshake.*/
        DDR_setFreq(25000000);

        ClockP_usleep(5000);

        /*trigger the start bit (from PI)*/
        HW_WR_REG32(DDR_CTL_REG_BASE + DDR_PI_REG_BLOCK_OFFS + DDR_PI_0_SFR_OFFS,   TRIGGER_START_BIT);

        ClockP_usleep(500);

        /*trigger the start bit (from CTL)*/
        HW_WR_REG32(DDR_CTL_REG_BASE + DDR_CTL_0_SFR_OFFS,   TRIGGER_START_BIT);

        ClockP_usleep(1000);

        DDR_changeFreqAck(prms);

        ClockP_usleep(500);

        /*PI INT STATUS*/
        while(((HW_RD_REG32(DDR_CTL_REG_BASE + DDR_PI_REG_BLOCK_OFFS + DDR_PI_83_SFR_OFFS)) & 0x1) != 0x1);
        /*CTL_342[25] = int status init[1] = 1 - The MC initialization has been completed.*/
        while((HW_RD_REG32(DDR_CTL_REG_BASE + DDR_CTL_342_SFR_OFFS)&0x02000000)!= 0x02000000);
    }

    return status;
}

static int32_t DDR_probe(void)
{
    uint32_t status = 0U;
    uint16_t configsize = 0U;
    int32_t ret = SystemP_SUCCESS;

    status = LPDDR4_Probe(&gLpddrCfg, &configsize);
    if ((status != 0) || (configsize != sizeof(LPDDR4_PrivateData)) ||
        (configsize > DDR_SRAM_MAX))
    {
        DebugP_logError("DDR4/LPDDR4_Probe failed !!!\n");
        ret = SystemP_FAILURE;
    }

    return ret;
}

static int32_t DDR_initDrv(void)
{
    uint32_t status = 0U;
    int32_t ret = SystemP_SUCCESS;

    if ((sizeof(gLpddrPd) != sizeof(LPDDR4_PrivateData)) ||
        (sizeof(gLpddrPd) > DDR_SRAM_MAX))
    {
        DebugP_logError("Invalid parameters !!!\n");
        ret = SystemP_FAILURE;
    }

    if(ret == SystemP_SUCCESS)
    {
        gLpddrCfg.ctlBase = (struct LPDDR4_CtlRegs_s *)CSL_DDR16SS0_CTL_CFG_BASE;
        gLpddrCfg.infoHandler = NULL;

        status = LPDDR4_Init(&gLpddrPd, &gLpddrCfg);
        if ((status > 0U) ||
            (gLpddrPd.ctlBase != (struct LPDDR4_CtlRegs_s *)gLpddrCfg.ctlBase) ||
            (gLpddrPd.ctlInterruptHandler != gLpddrCfg.ctlInterruptHandler) ||
            (gLpddrPd.phyIndepInterruptHandler != gLpddrCfg.phyIndepInterruptHandler))
        {
            DebugP_logError("DDR4/LPDDR4_Init failed !!!\n");
            ret = SystemP_FAILURE;
        }
    }

    return ret;
}

static int32_t DDR_initHwRegs(DDR_Params *prms)
{
    uint32_t status = 0U;
    int32_t ret = SystemP_SUCCESS;

    status = LPDDR4_WriteCtlConfig(&gLpddrPd,
                                    prms->ddrssCtlReg,
                                    prms->ddrssCtlRegNum,
                                    prms->ddrssCtlRegCount);
    if (status == SystemP_SUCCESS)
    {
        status = LPDDR4_WritePhyIndepConfig(&gLpddrPd,
                                            prms->ddrssPhyIndepReg,
                                            prms->ddrssPhyIndepRegNum,
                                            prms->ddrssPhyIndepRegCount);
    }
    if (status == SystemP_SUCCESS)
    {
        status = LPDDR4_WritePhyConfig(&gLpddrPd,
                                        prms->ddrssPhyReg,
                                        prms->ddrssPhyRegNum,
                                        prms->ddrssPhyRegCount);
    }

    if (status != SystemP_SUCCESS)
    {
        DebugP_logError("DDR config write failed !!!\r\n");
        ret = SystemP_FAILURE;
    }
    return ret;
}

static int32_t DDR_start(void)
{
    uint32_t status = 0U;
    uint32_t regval = 0U;
    uint32_t offset = 0U;
    int32_t ret = SystemP_SUCCESS;

    offset = DDR_CTL_REG_OFFSET;

    status = LPDDR4_ReadReg(&gLpddrPd, LPDDR4_CTL_REGS, offset, &regval);

    if((HW_RD_REG32(DDR_CTL_REG_BASE) & DDR_TYPE_MASK) == DDR4_MEMORY)
    {
        /* DDR4 memory */
        if ((status > 0U) || ((regval & 0x1U) != 0U))
        {
            DebugP_logError("DDR4_ReadReg failed !!!\r\n");
            ret = SystemP_FAILURE;
        }
    }
    else
    {
        /* LPDDR4 memory */
        if ((status > 0U) || ((regval & 0x1U) != 1U))
        {
            DebugP_logError("LPDDR4_ReadReg failed !!!\r\n");
            ret = SystemP_FAILURE;
        }
    }

    if(ret == SystemP_SUCCESS)
    {
        status = LPDDR4_Start(&gLpddrPd);
        if (status > 0U)
        {
            DebugP_logError("DDR4/LPDDR4_Start failed !!!\r\n");
            ret = SystemP_FAILURE;
        }

        if(ret == SystemP_SUCCESS)
        {
            status = LPDDR4_ReadReg(&gLpddrPd, LPDDR4_CTL_REGS, offset, &regval);
            if ((status > 0U) || ((regval & 0x1U) != 1U))
            {
                DebugP_logError("DDR4/LPDDR4_ReadReg failed !!!\r\n");
                ret = SystemP_FAILURE;
            }
        }
    }

    return ret;
}

static uint32_t DDR_isEnabled (DDR_Params *prm)
{
    uint32_t isEnabled = 0;
    struct LPDDR4_CtlRegs_s * ctlRegBase = (struct LPDDR4_CtlRegs_s *)gLpddrCfg.ctlBase;

    if ((HW_RD_REG32(&(ctlRegBase->LPDDR4__PI_START__REG)) & 0x1) &&
            (HW_RD_REG32(&(ctlRegBase->LPDDR4__START__REG)) & 0x1))
    {
        isEnabled = 1;
    }

    return isEnabled;
}

static int32_t DDR_inlineECCCfg (DDR_Params *prm)
{
    int32_t status = SystemP_SUCCESS;

    if (prm->eccRegion != NULL)
    {
        /* Disable inline ECC */
        DDR_enableInlineECC (0U);

        CSL_EmifConfig emifCfg;
        uintptr_t      memPtr;
        memset(&emifCfg, 0, sizeof(emifCfg));

        emifCfg.bEnableMemoryECC = TRUE;
        emifCfg.bReadModifyWriteEnable = TRUE;
        emifCfg.bECCCheck = TRUE;
        emifCfg.bWriteAlloc = TRUE;
        emifCfg.ECCThreshold = 1U;

        emifCfg.pMemEccCfg.startAddr[0] = prm->eccRegion->ddrEccStart0;
        emifCfg.pMemEccCfg.endAddr[0] = prm->eccRegion->ddrEccEnd0;
        emifCfg.pMemEccCfg.startAddr[1] = prm->eccRegion->ddrEccStart1;
        emifCfg.pMemEccCfg.endAddr[1] = prm->eccRegion->ddrEccEnd1;
        emifCfg.pMemEccCfg.startAddr[2] = prm->eccRegion->ddrEccStart2;
        emifCfg.pMemEccCfg.endAddr[2] = prm->eccRegion->ddrEccEnd2;

        status = CSL_emifConfig((CSL_emif_sscfgRegs *)CSL_DDR16SS0_SS_CFG_BASE,
                            &emifCfg);

        if (status == SystemP_SUCCESS)
        {
            /* Prime memory section 1 with known pattern */
            if (prm->eccRegion->ddrEccStart0 < prm->eccRegion->ddrEccEnd0)
            {
                for (memPtr = prm->eccRegion->ddrEccStart0;
                        memPtr < prm->eccRegion->ddrEccEnd0; memPtr += 4)
                {
                    *((uint32_t *) (memPtr + DDR_DRAM_START_ADDR)) = memPtr + DDR_DRAM_START_ADDR;
                }

                CacheP_wbInv((void *)(prm->eccRegion->ddrEccStart0 + DDR_DRAM_START_ADDR),
                    prm->eccRegion->ddrEccEnd0 - prm->eccRegion->ddrEccStart0,
                                                            CacheP_TYPE_ALL);
            }

            /* Prime memory section 2 with known pattern */
            if (prm->eccRegion->ddrEccStart1 < prm->eccRegion->ddrEccEnd1)
            {
                for (memPtr = prm->eccRegion->ddrEccStart1;
                        memPtr < prm->eccRegion->ddrEccEnd1; memPtr += 4)
                {
                    *((uint32_t *) (memPtr + DDR_DRAM_START_ADDR)) = memPtr + DDR_DRAM_START_ADDR;
                }

                CacheP_wbInv((void *)(prm->eccRegion->ddrEccStart1 + DDR_DRAM_START_ADDR),
                    prm->eccRegion->ddrEccEnd1 - prm->eccRegion->ddrEccStart1,
                                                            CacheP_TYPE_ALL);
            }

            /* Prime memory section 3 with known pattern */
            if (prm->eccRegion->ddrEccStart2 < prm->eccRegion->ddrEccEnd2)
            {
                for (memPtr = prm->eccRegion->ddrEccStart2;
                        memPtr < prm->eccRegion->ddrEccEnd2; memPtr += 4)
                {
                    *((uint32_t *) (memPtr + DDR_DRAM_START_ADDR)) = memPtr + DDR_DRAM_START_ADDR;
                }

                CacheP_wbInv((void *)(prm->eccRegion->ddrEccStart2 + DDR_DRAM_START_ADDR),
                    prm->eccRegion->ddrEccEnd2 - prm->eccRegion->ddrEccStart2,
                                                            CacheP_TYPE_ALL);
            }
        }

        if (status == SystemP_SUCCESS)
        {
            status = CSL_emifClearECCInterruptStatus((CSL_emif_sscfgRegs *)CSL_DDR16SS0_SS_CFG_BASE,
                                                    CSL_EMIF_SSCFG_V2A_INT_SET_REG_ECC1BERR_EN_MASK
                                                    | CSL_EMIF_SSCFG_V2A_INT_SET_REG_ECCM1BERR_EN_MASK
                                                    | CSL_EMIF_SSCFG_V2A_INT_SET_REG_ECC2BERR_EN_MASK);

            if (status == SystemP_SUCCESS)
            {
                status = CSL_emifEnableECCInterrupts((CSL_emif_sscfgRegs *)CSL_DDR16SS0_SS_CFG_BASE,
                                                    CSL_EMIF_SSCFG_V2A_INT_SET_REG_ECC1BERR_EN_MASK
                                                    | CSL_EMIF_SSCFG_V2A_INT_SET_REG_ECCM1BERR_EN_MASK
                                                    | CSL_EMIF_SSCFG_V2A_INT_SET_REG_ECC2BERR_EN_MASK);
            }
        }

        DDR_enableInlineECC (1U);
    }
    else
    {
        status = SystemP_FAILURE;
    }

    return status;
}

int32_t DDR_init(DDR_Params *prm)
{
    uint32_t isEnabled = 0;
    int32_t status = SystemP_SUCCESS;

    DDR_socEnableVttRegulator();

    /* power and clock to DDR and EMIF is done form outside using SysConfig */

    /* Configure MSMC2DDR Bridge Control register. Configure REGION_IDX, SDRAM_IDX and SDRAM_3QT.*/
    HW_WR_REG32((CSL_DDR16SS0_SS_CFG_BASE + 0x20), 0x1EF);

    /* Configure DDRSS_ECC_CTRL_REG register. Disable ECC. */
    HW_WR_REG32((CSL_DDR16SS0_SS_CFG_BASE + 0x120), 0x00);

    status = DDR_probe();
    if(status == SystemP_SUCCESS)
    {
        status = DDR_initDrv();
    }
    isEnabled = DDR_isEnabled(prm);
    if (!isEnabled)
    {
        if(status == SystemP_SUCCESS)
        {
            status = DDR_initHwRegs(prm);
        }
        if(status == SystemP_SUCCESS)
        {
            status = DDR_setClock(prm);
        }
        if(status == SystemP_SUCCESS)
        {
            status = DDR_start();
        }
    }

    if (status == SystemP_SUCCESS)
    {
        if (prm->enableEccFlag)
        {
            status = DDR_inlineECCCfg (prm);
        }
    }

    return status;
}


void DDR_Params_init(DDR_Params *prms)
{
    memset((void*)prms, 0, sizeof(DDR_Params));
}

void DDR_enableInlineECC (uint8_t enableFlag)
{
    if (1U == enableFlag)
    {
        CSL_emifEnableECC ((CSL_emif_sscfgRegs *)CSL_DDR16SS0_SS_CFG_BASE);
    }
    else
    {
        /* Disable inline ECC */
        CSL_emifDisableECC ((CSL_emif_sscfgRegs *)CSL_DDR16SS0_SS_CFG_BASE);
    }
}

int32_t DDR_clearECCError (uint8_t errorType)
{
    int32_t status = SystemP_SUCCESS;

    if (DDR_ECC_1B_ERROR == errorType)
    {
        /* Clear Specific ECC error */
        status = CSL_emifClearECCError((CSL_emif_sscfgRegs *)CSL_DDR16SS0_SS_CFG_BASE,
                                           CSL_EMIF_ECC_ERROR_TYPE_SINGLE_BIT);

        if (status == SystemP_SUCCESS)
        {
            /* Clear ECC interrupt bits */
            status = CSL_emifClearECCInterruptStatus((CSL_emif_sscfgRegs *)CSL_DDR16SS0_SS_CFG_BASE,
                                                CSL_EMIF_SSCFG_V2A_INT_SET_REG_ECC1BERR_EN_MASK
                                                | CSL_EMIF_SSCFG_V2A_INT_SET_REG_ECCM1BERR_EN_MASK);
        }
    }
    else if (DDR_ECC_2B_ERROR == errorType)
    {
        /* Clear specific error */
        status = CSL_emifClearECCError((CSL_emif_sscfgRegs *)CSL_DDR16SS0_SS_CFG_BASE,
                                           CSL_EMIF_ECC_ERROR_TYPE_DOUBLE_BIT);

        if (status == SystemP_SUCCESS)
        {
            /* Clear ECC interrupt bits */
            status = CSL_emifClearECCInterruptStatus((CSL_emif_sscfgRegs *)CSL_DDR16SS0_SS_CFG_BASE,
                                                     CSL_EMIF_SSCFG_V2A_INT_SET_REG_ECC2BERR_EN_MASK);
        }
    }
    else
    {
        /* Clear all errors */
        status = CSL_emifClearAllECCErrors ((CSL_emif_sscfgRegs *)CSL_DDR16SS0_SS_CFG_BASE);

        if (status == SystemP_SUCCESS)
        {
            status = CSL_emifClearECCInterruptStatus((CSL_emif_sscfgRegs *)CSL_DDR16SS0_SS_CFG_BASE,
                                                CSL_EMIF_SSCFG_V2A_INT_SET_REG_ECC1BERR_EN_MASK
                                                | CSL_EMIF_SSCFG_V2A_INT_SET_REG_ECCM1BERR_EN_MASK
                                                | CSL_EMIF_SSCFG_V2A_INT_SET_REG_ECC2BERR_EN_MASK);
        }
    }

    return status;
}

int32_t DDR_getECCErrorInfo (DDR_ECCErrorInfo *ECCErrorInfo)
{
    int32_t status = SystemP_SUCCESS;
    CSL_ECCErrorInfo csl_ECCErrorInfo = {0};

    if (ECCErrorInfo != NULL)
    {
        status = CSL_emifGetECCErrorInfo ((CSL_emif_sscfgRegs *)CSL_DDR16SS0_SS_CFG_BASE, &csl_ECCErrorInfo);

        memcpy ((void *)ECCErrorInfo, (void *)&csl_ECCErrorInfo, sizeof(csl_ECCErrorInfo));
    }
    else
    {
        status = SystemP_FAILURE;
    }

    return status;
}
