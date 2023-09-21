/*
 *  Copyright (C) 2021-23 Texas Instruments Incorporated
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

#include <drivers/hw_include/cslr_soc.h>
#include <drivers/hw_include/hw_types.h>
#include <drivers/soc.h>
#if defined(__ARM_ARCH_7R__)
#ifdef __ARM_ACLE
#include <arm_acle.h>
#endif /* __ARM_ACLE */
#endif

#define SOC_RCM_L3MEMCONFIG_START_BIT               (7U)
#define SOC_RCM_L3MEMCONFIG_END_BIT                 (10U)
#define SOC_RCM_FLASHMODECONFIG_START_BIT           (15U)
#define SOC_RCM_FLASHMODECONFIG_END_BIT             (16U)
#define SOC_RCM_QSPIFREQCONFIG_START_BIT            (17U)
#define SOC_RCM_QSPIFREQCONFIG_END_BIT              (18U)
#define SOC_RCM_BISTENABLECONFIG_START_BIT          (1U)
#define SOC_RCM_BISTENABLECONFIG_END_BIT            (9U)
#define SOC_RCM_COREADPLLTRIMCONFIG_START_BIT       (0U)
#define SOC_RCM_COREADPLLTRIMCONFIG_END_BIT         (11U)
#define SOC_RCM_DSPADPLLTRIMCONFIG_START_BIT        (12U)
#define SOC_RCM_DSPADPLLTRIMCONFIG_END_BIT          (23U)
#define SOC_RCM_PERADPLLTRIMCONFIG_START_BIT        (0U)
#define SOC_RCM_PERADPLLTRIMCONFIG_END_BIT          (11U)
#define SOC_RCM_COREADPLLTRIMVALIDCONFIG_START_BIT  (12U)
#define SOC_RCM_COREADPLLTRIMVALIDCONFIG_END_BIT    (12U)
#define SOC_RCM_DSPADPLLTRIMVALIDCONFIG_START_BIT   (13U)
#define SOC_RCM_DSPADPLLTRIMVALIDCONFIG_END_BIT     (13U)
#define SOC_RCM_PERADPLLTRIMVALIDCONFIG_START_BIT   (14U)
#define SOC_RCM_PERADPLLTRIMVALIDCONFIG_END_BIT     (14U)
#define SOC_RCM_PARTID_START_BIT                    (13U)
#define SOC_RCM_PARTID_END_BIT                      (24U)
#define SOC_RCM_PARTNUMBER_START_BIT                (0U)
#define SOC_RCM_PARTNUMBER_END_BIT                  (2U)
#define SOC_RCM_PGVERSION_START_BIT                 (0U)
#define SOC_RCM_PGVERSION_END_BIT                   (3U)
#define SOC_RCM_ROMVERSION_START_BIT                (4U)
#define SOC_RCM_ROMVERSION_END_BIT                  (6U)
#define SOC_RCM_METALVERSION_START_BIT              (7U)
#define SOC_RCM_METALVERSION_END_BIT                (9U)
#define SOC_RCM_XTAL_FREQ_SCALE_START_BIT           (25U)
#define SOC_RCM_XTAL_FREQ_SCALE_END_BIT             (25U)

#define SOC_RCM_CORE_ADPLL_DEFAULT_VALUE            (0x9U)
#define SOC_RCM_DSP_ADPLL_DEFAULT_VALUE             (0x9U)
#define SOC_RCM_PER_ADPLL_DEFAULT_VALUE             (0x9U)
#define SOC_RCM_PLL_SYS_CLK_FREQUENCY_HZ            (200000000U)
#define SOC_RCM_PLL_HSDIV_CLKOUT2_FREQUENCY_HZ      (400000000U)
#define SOC_RCM_XTAL_CLK_40MHZ                      (40000000U)
#define SOC_RCM_XTAL_CLK_50MHZ                      (50000000U)
#define SOC_RCM_XTAL_CLK_49p152MHZ                  (49152000U)
#define SOC_RCM_XTAL_CLK_45p1584MHZ                 (45158400U)
#define SOC_RCM_XTAL_CLK_20MHZ                      (20000000U)
#define SOC_RCM_XTAL_CLK_25MHZ                      (25000000U)
#define SOC_RCM_XTAL_CLK_24p576MHZ                  (24576000U)
#define SOC_RCM_XTAL_CLK_22p5792MHZ                 (22579200U)

#define SOC_RCM_UTILS_ARRAYSIZE(x)                  (sizeof(x)/sizeof(x[0]))

#define SOC_RCM_FREQ_1GHZ                           (1000*1000*1000)
#define SOC_RCM_FREQ_550MHZ                         (uint32_t)(550*1000*1000)

typedef enum SOC_RcmXtalFreqId_e
{
    SOC_RcmXtalFreqId_CLK_40MHZ,
    SOC_RcmXtalFreqId_CLK_50MHZ,
    SOC_RcmXtalFreqId_CLK_49p152MHZ,
    SOC_RcmXtalFreqId_CLK_45p1584MHZ,
    SOC_RcmXtalFreqId_CLK_20MHZ,
    SOC_RcmXtalFreqId_CLK_25MHZ,
    SOC_RcmXtalFreqId_CLK_24p576MHZ,
    SOC_RcmXtalFreqId_CLK_22p5792MHZ
} SOC_RcmXtalFreqId;

typedef enum SOC_RcmPllId_e
{
    SOC_RcmPllId_CORE,
    SOC_RcmPllId_DSS,
    SOC_RcmPllId_PER,
    SOC_RcmPllId_XTALCLK,
    SOC_RcmPllId_WUCPUCLK,
    SOC_RcmPllId_RCCLK32K,
    SOC_RcmPllId_RCCLK10M,
} SOC_RcmPllId;

typedef enum SOC_RcmPllHsDivOutId_e
{
    SOC_RcmPllHsDivOutId_0,
    SOC_RcmPllHsDivOutId_1,
    SOC_RcmPllHsDivOutId_2,
    SOC_RcmPllHsDivOutId_3,
    SOC_RcmPllHsDivOutId_NONE,
} SOC_RcmPllHsDivOutId;

typedef struct SOC_RcmClkSrcInfo_s
{
    SOC_RcmPllId pllId;
    SOC_RcmPllHsDivOutId hsDivOut;
} SOC_RcmClkSrcInfo;

typedef struct SOC_RcmXtalInfo_s
{
    uint32_t Finp;
    uint32_t div2flag;
} SOC_RcmXtalInfo;

typedef struct SOC_RcmADPLLJConfig_s
{
    uint32_t N; /* Input Clock divider/Pre-divider (N) */
    uint32_t M2; /* Post divider (M2) */
    uint32_t M;  /* Multiplier integer (M) */
    uint32_t FracM; /* Multiplier fractional (M) */
    uint32_t Fout; /* Output frequency of PLL */
    uint32_t Finp; /* Output frequency of PLL */
} SOC_RcmADPLLJConfig;

/* Table populated from TPR12_ADPLLJ_Settings_1p0.xlsx.
 * Each entry corresponds to tab in the excel sheet
 */
static const SOC_RcmADPLLJConfig gSocRcmADPLLJConfigTbl[] =
{
    /*DSP_1100_40MHz*/
    {
        .Finp = 40U,
        .N = 19U,
        .Fout = 1100U,
        .M2 = 1U,
        .M = 550U,
        .FracM = 0U,
    },
    /*DSP_1650_40MHz*/
    {
        .Finp = 40U,
        .N = 39U,
        .Fout = 1650U,
        .M2 = 1U,
        .M = 1650U,
        .FracM = 0U,
    },
    /* DSP_900_40MHz  */
    {
        .Finp = 40U,
        .N    = 39U,
        .Fout = 900U,
        .M2   = 1U,
        .M    = 900U,
        .FracM = 0U,
    },
    /* CORE_DSP_800_40MHz */
    {
        .Finp = 40U,
        .N = 39U,
        .Fout = 800U,
        .M2 = 1U,
        .M = 800U,
        .FracM = 0U,
    },
    /* CORE_DSP_800_40MHz */
    {
        .Finp = 40U,
        .N = 19U,
        .Fout = 800U,
        .M2 = 1U,
        .M = 400U,
        .FracM = 0U,
    },
    /* CORE_DSP_2000_40MHz */
    {
        .Finp = 40U,
        .N = 39U,
        .Fout = 2000U,
        .M2 = 1U,
        .M = 2000U,
        .FracM = 0U,
    },
    /* CORE_DSP_2000_40MHz */
    {
        .Finp = 40U,
        .N = 19U,
        .Fout = 2000U,
        .M2 = 1U,
        .M = 1000U,
        .FracM = 0U,
    },
    /* CORE_PER_1728_40MHz */
    {
        .Finp = 40U,
        .N   = 39U,
        .Fout = 1728U,
        .M2  = 1U,
        .M = 1728U,
        .FracM = 0U,
    },
    /* DSP_900_40MHz  */
    {
        .Finp = 40U,
        .N = 19U,
        .Fout = 900U,
        .M2 = 1U,
        .M  = 450U,
        .FracM = 0U,
    },
    /* DSP_1800_40MHz  */
    {
        .Finp = 40U,
        .N    = 19U,
        .Fout = 1800U,
        .M2   = 1U,
        .M    = 900U,
        .FracM = 0U,
    },
    /* PER_1920_40MHz  */
    {
        .Finp = 40U,
        .N = 19U,
        .Fout = 1920U,
        .M2   = 1U,
        .M    = 960U,
        .FracM = 0U,
    },
    /* CORE_DSP_800_50MHz  */
    {
        .Finp = 50U,
        .N = 24U,
        .Fout = 800U,
        .M2 = 1U,
        .M = 400U,
        .FracM = 0U,
    },
    /* CORE_DSP_2000_50MHz  */
    {
       .Finp = 50U,
       .N = 24U,
       .Fout = 2000U,
       .M2 = 1U,
       .M = 1000U,
       .FracM = 0U,
    },
    /* DSP_1800_50MHz  */
    {
       .Finp = 50U,
       .N = 24U,
       .Fout = 1800U,
       .M2 = 1U,
       .M = 900U,
       .FracM = 0U,
    },
    /* PER_1920_50MHz  */
    {
       .Finp = 50U,
       .N = 24U,
       .Fout = 1920U,
       .M2 = 1U,
       .M = 960U,
       .FracM = 0U,
    },
    /* CORE_800_49152  */
    {
       .Finp = 49U,
       .N = 23U,
       .Fout = 800U,
       .M2 = 1U,
       .M = 390U,
       .FracM = 163840U,
    },
    /* CORE_2000_49152  */
    {
       .Finp = 49U,
       .N = 21U,
       .Fout = 2000U,
       .M2 = 1U,
       .M = 895U,
       .FracM = 47787U,
    },
    /* DSP_1800_49152  */
    {
       .Finp = 49U,
       .N = 20U,
       .Fout = 1800U,
       .M2 = 1U,
       .M = 769U,
       .FracM = 11264U,
    },
    /* DSP_1728_49152  */
    {
       .Finp = 49U,
       .N = 19U,
       .Fout = 1728U,
       .M2 = 1U,
       .M = 703U,
       .FracM = 32768U,
    },
    /* PER_1966p08_49152  */
    {
       .Finp = 49U,
       .N = 19U,
       .Fout = 1966U,
       .M2 = 1U,
       .M = 800U,
       .FracM = 209715200U,
    },
    /* CORE_800_451584  */
    {
       .Finp = 45U,
       .N = 20U,
       .Fout = 800U,
       .M2 = 1U,
       .M = 372U,
       .FracM = 6242U,
    },
    /* CORE_2000_451584  */
    {
       .Finp = 45U,
       .N = 20U,
       .Fout = 2000U,
       .M2 = 1U,
       .M = 930U,
       .FracM = 15604U,
    },
    /* DSP_1800_451584  */
    {
       .Finp = 45U,
       .N = 20U,
       .Fout = 1800U,
       .M2 = 1U,
       .M = 837U,
       .FracM = 14043U,
    },
    /* DSP_1728_451584  */
    {
       .Finp = 45U,
       .N = 18U,
       .Fout = 1728U,
       .M2 = 1U,
       .M = 727U,
       .FracM = 10700U,
    },
    /* PER_1806p336_451584  */
    {
       .Finp = 45U,
       .N = 18U,
       .Fout = 1806U,
       .M2 = 1U,
       .M = 760U,
       .FracM = 0U,
    },
    /* PER_1699_40MHz  */
    {
       .Finp = 40U,
       .N = 16U,
       .Fout = 1699U,
       .M2 = 1U,
       .M = 722U,
       .FracM = 44032U,
    },
};

static const SOC_RcmXtalInfo gSocRcmXtalInfo[] =
{
    [SOC_RcmXtalFreqId_CLK_40MHZ]      = {.Finp = SOC_RCM_FREQ_HZ2MHZ(SOC_RCM_XTAL_CLK_40MHZ),      .div2flag = 0 },
    [SOC_RcmXtalFreqId_CLK_50MHZ]      = {.Finp = SOC_RCM_FREQ_HZ2MHZ(SOC_RCM_XTAL_CLK_50MHZ),      .div2flag = 0 },
    [SOC_RcmXtalFreqId_CLK_49p152MHZ]  = {.Finp = SOC_RCM_FREQ_HZ2MHZ(SOC_RCM_XTAL_CLK_49p152MHZ),  .div2flag = 0 },
    [SOC_RcmXtalFreqId_CLK_45p1584MHZ] = {.Finp = SOC_RCM_FREQ_HZ2MHZ(SOC_RCM_XTAL_CLK_45p1584MHZ), .div2flag = 0 },
    [SOC_RcmXtalFreqId_CLK_20MHZ]      = {.Finp = SOC_RCM_FREQ_HZ2MHZ(SOC_RCM_XTAL_CLK_40MHZ),      .div2flag = 1 },
    [SOC_RcmXtalFreqId_CLK_25MHZ]      = {.Finp = SOC_RCM_FREQ_HZ2MHZ(SOC_RCM_XTAL_CLK_50MHZ),      .div2flag = 1 },
    [SOC_RcmXtalFreqId_CLK_24p576MHZ]  = {.Finp = SOC_RCM_FREQ_HZ2MHZ(SOC_RCM_XTAL_CLK_49p152MHZ),  .div2flag = 1 },
    [SOC_RcmXtalFreqId_CLK_22p5792MHZ] = {.Finp = SOC_RCM_FREQ_HZ2MHZ(SOC_RCM_XTAL_CLK_50MHZ),      .div2flag = 1 },
};

static const uint32_t gSocRcmPllFreqId2FOutMap[] =
{
    [SOC_RcmPllFoutFreqId_CLK_1100MHZ]        = 1100U,
    [SOC_RcmPllFoutFreqId_CLK_1650MHZ]        = 1650U,
    [SOC_RcmPllFoutFreqId_CLK_800MHZ]        = 800U,
    [SOC_RcmPllFoutFreqId_CLK_900MHZ]        = 900U,
    [SOC_RcmPllFoutFreqId_CLK_2000MHZ]       = 2000U,
    [SOC_RcmPllFoutFreqId_CLK_1800MHZ]       = 1800U,
    [SOC_RcmPllFoutFreqId_CLK_1920MHZ]       = 1920U,
    [SOC_RcmPllFoutFreqId_CLK_1699p21875MHZ] = 1699U,
    [SOC_RcmPllFoutFreqId_CLK_1728MHZ]       = 1728U,
    [SOC_RcmPllFoutFreqId_CLK_1966p08MHZ]    = 1966U,
    [SOC_RcmPllFoutFreqId_CLK_1806p336MHZ]   = 1806U,
};

static const SOC_RcmClkSrcInfo gSocRcmDspClkSrcInfoMap[] =
{
    [SOC_RcmDspClockSource_XTAL_CLK] =
    {
        .pllId = SOC_RcmPllId_XTALCLK,
        .hsDivOut = SOC_RcmPllHsDivOutId_NONE,
    },
    [SOC_RcmDspClockSource_DPLL_DSP_HSDIV0_CLKOUT1] =
    {
        .pllId = SOC_RcmPllId_DSS,
        .hsDivOut = SOC_RcmPllHsDivOutId_1,
    },
    [SOC_RcmDspClockSource_DPLL_CORE_HSDIV0_CLKOUT1] =
    {
        .pllId = SOC_RcmPllId_CORE,
        .hsDivOut = SOC_RcmPllHsDivOutId_1,
    },
};

const SOC_RcmClkSrcInfo gSocRcmPeripheralClkSrcInfoMap[] =
{
    [SOC_RcmPeripheralClockSource_XTAL_CLK] =
    {
        .pllId = SOC_RcmPllId_XTALCLK,
        .hsDivOut = SOC_RcmPllHsDivOutId_NONE,
    },
    [SOC_RcmPeripheralClockSource_SYS_CLK] =
    {
        .pllId = SOC_RcmPllId_CORE,
        .hsDivOut = SOC_RcmPllHsDivOutId_2,
    },
    [SOC_RcmPeripheralClockSource_DPLL_CORE_HSDIV0_CLKOUT1] =
    {
        .pllId = SOC_RcmPllId_CORE,
        .hsDivOut = SOC_RcmPllHsDivOutId_1,
    },
    [SOC_RcmPeripheralClockSource_DPLL_CORE_HSDIV0_CLKOUT2] =
    {
        .pllId = SOC_RcmPllId_CORE,
        .hsDivOut = SOC_RcmPllHsDivOutId_2,
    },
    [SOC_RcmPeripheralClockSource_DPLL_PER_HSDIV0_CLKOUT1] =
    {
        .pllId = SOC_RcmPllId_PER,
        .hsDivOut = SOC_RcmPllHsDivOutId_1,
    },
    [SOC_RcmPeripheralClockSource_DPLL_PER_HSDIV0_CLKOUT2] =
    {
        .pllId = SOC_RcmPllId_PER,
        .hsDivOut = SOC_RcmPllHsDivOutId_2,
    },
    [SOC_RcmPeripheralClockSource_DPLL_PER_HSDIV0_CLKOUT3] =
    {
        .pllId = SOC_RcmPllId_PER,
        .hsDivOut = SOC_RcmPllHsDivOutId_3,
    },
    [SOC_RcmPeripheralClockSource_WUCPU_CLK] =
    {
        .pllId = SOC_RcmPllId_WUCPUCLK,
        .hsDivOut = SOC_RcmPllHsDivOutId_NONE,
    },
};

static const SOC_RcmClkSrcInfo gSocRcmR5ClkSrcInfoMap =
{
    .pllId = SOC_RcmPllId_CORE,
    .hsDivOut = SOC_RcmPllHsDivOutId_2,
};

/*
 *  Mapping Array between Clock mode and Clock Mode Value for RTI and WDG
 */
static const uint16_t gSocRcmRtiClkSrcValMap[] =
{
    [SOC_RcmPeripheralClockSource_XTAL_CLK] = 0x000U,
    [SOC_RcmPeripheralClockSource_SYS_CLK]  = 0x222U,
    [SOC_RcmPeripheralClockSource_DPLL_CORE_HSDIV0_CLKOUT1] = 0x888U, /* Set unsupported clock source to 0x888 which indicates invalid value */
    [SOC_RcmPeripheralClockSource_DPLL_CORE_HSDIV0_CLKOUT2] = 0x888U, /* Set unsupported clock source to 0x888 which indicates invalid value */
    [SOC_RcmPeripheralClockSource_DPLL_PER_HSDIV0_CLKOUT1] = 0x333U,
    [SOC_RcmPeripheralClockSource_DPLL_PER_HSDIV0_CLKOUT2]  = 0x888U, /* Set unsupported clock source to 0x888 which indicates invalid value */
    [SOC_RcmPeripheralClockSource_DPLL_PER_HSDIV0_CLKOUT3] = 0x888U, /* Set unsupported clock source to 0x888 which indicates invalid value */
};

/*
 *  Mapping Array between Clock mode and Clock Mode Value for SPI
 */
static const uint16_t gSocRcmSpiClkSrcValMap[] =
{
    [SOC_RcmPeripheralClockSource_XTAL_CLK] = 0x111U,
    [SOC_RcmPeripheralClockSource_SYS_CLK]  = 0x222U,
    [SOC_RcmPeripheralClockSource_DPLL_CORE_HSDIV0_CLKOUT1] = 0x888U, /* Set unsupported clock source to 0x888 which indicates invalid value */
    [SOC_RcmPeripheralClockSource_DPLL_CORE_HSDIV0_CLKOUT2] = 0x333U,
    [SOC_RcmPeripheralClockSource_DPLL_PER_HSDIV0_CLKOUT1] = 0x444U,
    [SOC_RcmPeripheralClockSource_DPLL_PER_HSDIV0_CLKOUT2]  = 0x888U, /* Set unsupported clock source to 0x888 which indicates invalid value */
    [SOC_RcmPeripheralClockSource_DPLL_PER_HSDIV0_CLKOUT3] = 0x888U, /* Set unsupported clock source to 0x888 which indicates invalid value */
};


/*
 *  Mapping Array between Clock mode and Clock Mode Value for I2C
 */
static const uint16_t gSocRcmI2cClkSrcValMap[] =
{
    [SOC_RcmPeripheralClockSource_XTAL_CLK] = 0x111U,
    [SOC_RcmPeripheralClockSource_SYS_CLK] = 0x222U,
    [SOC_RcmPeripheralClockSource_DPLL_CORE_HSDIV0_CLKOUT1] = 0x888U, /* Set unsupported clock source to 0x888 which indicates invalid value */
    [SOC_RcmPeripheralClockSource_DPLL_CORE_HSDIV0_CLKOUT2] = 0x444U,
    [SOC_RcmPeripheralClockSource_DPLL_PER_HSDIV0_CLKOUT1] = 0x333U,
    [SOC_RcmPeripheralClockSource_DPLL_PER_HSDIV0_CLKOUT2]  = 0x888U, /* Set unsupported clock source to 0x888 which indicates invalid value */
    [SOC_RcmPeripheralClockSource_DPLL_PER_HSDIV0_CLKOUT3] = 0x888U, /* Set unsupported clock source to 0x888 which indicates invalid value */

};


/*
 *  Mapping Array between Clock mode and Clock Mode Value for QSPI
 */
static const uint16_t gSocRcmQspiClkSrcValMap[] =
{
    [SOC_RcmPeripheralClockSource_XTAL_CLK] = 0x000U,
    [SOC_RcmPeripheralClockSource_SYS_CLK]  = 0x222U,
    [SOC_RcmPeripheralClockSource_DPLL_CORE_HSDIV0_CLKOUT1] = 0x888U, /* Set unsupported clock source to 0x888 which indicates invalid value */
    [SOC_RcmPeripheralClockSource_DPLL_CORE_HSDIV0_CLKOUT2] = 0x444U,
    [SOC_RcmPeripheralClockSource_DPLL_PER_HSDIV0_CLKOUT1] = 0x333,
    [SOC_RcmPeripheralClockSource_DPLL_PER_HSDIV0_CLKOUT2]  = 0x888U, /* Set unsupported clock source to 0x888 which indicates invalid value */
    [SOC_RcmPeripheralClockSource_DPLL_PER_HSDIV0_CLKOUT3] = 0x888U, /* Set unsupported clock source to 0x888 which indicates invalid value */

};

/*
 *  Mapping Array between Clock mode and Clock Mode Value for SCI UART
 */
static const uint16_t gSocRcmSciClkSrcValMap[] =
{
    [SOC_RcmPeripheralClockSource_XTAL_CLK] = 0x111U,
    [SOC_RcmPeripheralClockSource_SYS_CLK] = 0x222U,
    [SOC_RcmPeripheralClockSource_DPLL_CORE_HSDIV0_CLKOUT1] = 0x888U, /* Set unsupported clock source to 0x888 which indicates invalid value */
    [SOC_RcmPeripheralClockSource_DPLL_CORE_HSDIV0_CLKOUT2] = 0x444U,
    [SOC_RcmPeripheralClockSource_DPLL_PER_HSDIV0_CLKOUT1] = 0x333U,
    [SOC_RcmPeripheralClockSource_DPLL_PER_HSDIV0_CLKOUT2]  = 0x888U, /* Set unsupported clock source to 0x888 which indicates invalid value */
    [SOC_RcmPeripheralClockSource_DPLL_PER_HSDIV0_CLKOUT3] = 0x888U, /* Set unsupported clock source to 0x888 which indicates invalid value */

};

static const uint16_t gSocRcmMcanClkSrcValMap[] =
{
    [SOC_RcmPeripheralClockSource_XTAL_CLK] = 0x888U, /* Set unsupported clock source to 0x888 which indicates invalid value */
    [SOC_RcmPeripheralClockSource_SYS_CLK]  = 0x222U,
    [SOC_RcmPeripheralClockSource_DPLL_CORE_HSDIV0_CLKOUT1] = 0x888U, /* Set unsupported clock source to 0x888 which indicates invalid value */
    [SOC_RcmPeripheralClockSource_DPLL_CORE_HSDIV0_CLKOUT2] = 0x444U,
    [SOC_RcmPeripheralClockSource_DPLL_PER_HSDIV0_CLKOUT1] = 0x888U, /* Set unsupported clock source to 0x888 which indicates invalid value */
    [SOC_RcmPeripheralClockSource_DPLL_PER_HSDIV0_CLKOUT2]  = 0x888U, /* Set unsupported clock source to 0x888 which indicates invalid value */
    [SOC_RcmPeripheralClockSource_DPLL_PER_HSDIV0_CLKOUT3] = 0x888U, /* Set unsupported clock source to 0x888 which indicates invalid value */

};

static const uint16_t gSocRcmCsiRxClkSrcValMap[] =
{
    [SOC_RcmPeripheralClockSource_XTAL_CLK] = 0x666U,
    [SOC_RcmPeripheralClockSource_SYS_CLK]  = 0x888U, /* Set unsupported clock source to 0x888 which indicates invalid value */
    [SOC_RcmPeripheralClockSource_DPLL_CORE_HSDIV0_CLKOUT1] = 0x888U, /* Set unsupported clock source to 0x888 which indicates invalid value */
    [SOC_RcmPeripheralClockSource_DPLL_CORE_HSDIV0_CLKOUT2] = 0x888U, /* Set unsupported clock source to 0x888 which indicates invalid value */
    [SOC_RcmPeripheralClockSource_DPLL_PER_HSDIV0_CLKOUT1] = 0x888U, /* Set unsupported clock source to 0x888 which indicates invalid value */
    [SOC_RcmPeripheralClockSource_DPLL_PER_HSDIV0_CLKOUT2]  = 0x222U,
    [SOC_RcmPeripheralClockSource_DPLL_PER_HSDIV0_CLKOUT3] = 0x888U, /* Set unsupported clock source to 0x888 which indicates invalid value */
};

static const uint16_t gSocRcmDssRtiClkSrcValMap[] =
{
    [SOC_RcmPeripheralClockSource_XTAL_CLK] = 0x111U,
    [SOC_RcmPeripheralClockSource_SYS_CLK] = 0x222U,
    [SOC_RcmPeripheralClockSource_DPLL_CORE_HSDIV0_CLKOUT1] = 0x888U, /* Set unsupported clock source to 0x888 which indicates invalid value */
    [SOC_RcmPeripheralClockSource_DPLL_CORE_HSDIV0_CLKOUT2] = 0x888U, /* Set unsupported clock source to 0x888 which indicates invalid value */
    [SOC_RcmPeripheralClockSource_DPLL_PER_HSDIV0_CLKOUT1] = 0x333U,
    [SOC_RcmPeripheralClockSource_DPLL_PER_HSDIV0_CLKOUT2]  = 0x888U, /* Set unsupported clock source to 0x888 which indicates invalid value */
    [SOC_RcmPeripheralClockSource_DPLL_PER_HSDIV0_CLKOUT3] = 0x888U,  /* Set unsupported clock source to 0x888 which indicates invalid value */
};

static const uint16_t gSocRcmDssSciClkSrcValMap[] =
{
    [SOC_RcmPeripheralClockSource_XTAL_CLK] = 0x111U,
    [SOC_RcmPeripheralClockSource_SYS_CLK] = 0x222U,
    [SOC_RcmPeripheralClockSource_DPLL_CORE_HSDIV0_CLKOUT1] = 0x888U, /* Set unsupported clock source to 0x888 which indicates invalid value */
    [SOC_RcmPeripheralClockSource_DPLL_CORE_HSDIV0_CLKOUT2] = 0x888U, /* Set unsupported clock source to 0x888 which indicates invalid value */
    [SOC_RcmPeripheralClockSource_DPLL_PER_HSDIV0_CLKOUT1] = 0x666U,
    [SOC_RcmPeripheralClockSource_DPLL_PER_HSDIV0_CLKOUT2]  = 0x888U, /* Set unsupported clock source to 0x888 which indicates invalid value */
    [SOC_RcmPeripheralClockSource_DPLL_PER_HSDIV0_CLKOUT3] = 0x888U,  /* Set unsupported clock source to 0x888 which indicates invalid value */
};

static const uint16_t gSocRcmRcssSciClkSrcValMap[] =
{
    [SOC_RcmPeripheralClockSource_XTAL_CLK] = 0x111U,
    [SOC_RcmPeripheralClockSource_SYS_CLK] = 0x222U,
    [SOC_RcmPeripheralClockSource_DPLL_CORE_HSDIV0_CLKOUT1] = 0x888U, /* Set unsupported clock source to 0x888 which indicates invalid value */
    [SOC_RcmPeripheralClockSource_DPLL_CORE_HSDIV0_CLKOUT2] = 0x888U, /* Set unsupported clock source to 0x888 which indicates invalid value */
    [SOC_RcmPeripheralClockSource_DPLL_PER_HSDIV0_CLKOUT1] = 0x333U,
    [SOC_RcmPeripheralClockSource_DPLL_PER_HSDIV0_CLKOUT2]  = 0x888U, /* Set unsupported clock source to 0x888 which indicates invalid value */
    [SOC_RcmPeripheralClockSource_DPLL_PER_HSDIV0_CLKOUT3] = 0x888U,  /* Set unsupported clock source to 0x888 which indicates invalid value */
};

static const uint16_t gSocRcmRcssSpiClkSrcValMap[] =
{
    [SOC_RcmPeripheralClockSource_XTAL_CLK] = 0x111U,
    [SOC_RcmPeripheralClockSource_SYS_CLK] = 0x222U,
    [SOC_RcmPeripheralClockSource_DPLL_CORE_HSDIV0_CLKOUT1] = 0x888U, /* Set unsupported clock source to 0x888 which indicates invalid value */
    [SOC_RcmPeripheralClockSource_DPLL_CORE_HSDIV0_CLKOUT2] = 0x888U, /* Set unsupported clock source to 0x888 which indicates invalid value */
    [SOC_RcmPeripheralClockSource_DPLL_PER_HSDIV0_CLKOUT1] = 0x333U,
    [SOC_RcmPeripheralClockSource_DPLL_PER_HSDIV0_CLKOUT2]  = 0x888U, /* Set unsupported clock source to 0x888 which indicates invalid value */
    [SOC_RcmPeripheralClockSource_DPLL_PER_HSDIV0_CLKOUT3] = 0x888U, /* Set unsupported clock source to 0x888 which indicates invalid value */
};

static const uint16_t gSocRcmRcssI2cClkSrcValMap[] =
{
    [SOC_RcmPeripheralClockSource_XTAL_CLK] = 0x111U,
    [SOC_RcmPeripheralClockSource_SYS_CLK] = 0x222U,
    [SOC_RcmPeripheralClockSource_DPLL_CORE_HSDIV0_CLKOUT1] = 0x888U, /* Set unsupported clock source to 0x888 which indicates invalid value */
    [SOC_RcmPeripheralClockSource_DPLL_CORE_HSDIV0_CLKOUT2] = 0x888U, /* Set unsupported clock source to 0x888 which indicates invalid value */
    [SOC_RcmPeripheralClockSource_DPLL_PER_HSDIV0_CLKOUT1] = 0x333U,
    [SOC_RcmPeripheralClockSource_DPLL_PER_HSDIV0_CLKOUT2]  = 0x888U, /* Set unsupported clock source to 0x888 which indicates invalid value */
    [SOC_RcmPeripheralClockSource_DPLL_PER_HSDIV0_CLKOUT3] = 0x888U, /* Set unsupported clock source to 0x888 which indicates invalid value */
};

static const uint16_t gSocRcmRcssAtlClkSrcValMap[] =
{
    [SOC_RcmPeripheralClockSource_XTAL_CLK] = 0x111U,
    [SOC_RcmPeripheralClockSource_SYS_CLK] = 0x222U,
    [SOC_RcmPeripheralClockSource_DPLL_CORE_HSDIV0_CLKOUT1] = 0x000U, /* This is as such not supported and it is expected that clock source would be changed when using ATL */
    [SOC_RcmPeripheralClockSource_DPLL_CORE_HSDIV0_CLKOUT2] = 0x888U, /* Set unsupported clock source to 0x888 which indicates invalid value */
    [SOC_RcmPeripheralClockSource_DPLL_PER_HSDIV0_CLKOUT1] = 0x333U,
    [SOC_RcmPeripheralClockSource_DPLL_PER_HSDIV0_CLKOUT2] = 0x444U,
    [SOC_RcmPeripheralClockSource_DPLL_PER_HSDIV0_CLKOUT3] = 0x666U,
};

static const uint16_t gSocRcmRcssMcaspAuxClkSrcValMap[] =
{
    [SOC_RcmPeripheralClockSource_XTAL_CLK] = 0x888U, /* Set unsupported clock source to 0x888 which indicates invalid value */
    [SOC_RcmPeripheralClockSource_SYS_CLK] = 0x888U,  /* Set unsupported clock source to 0x888 which indicates invalid value */
    [SOC_RcmPeripheralClockSource_DPLL_CORE_HSDIV0_CLKOUT1] = 0x888U, /* Set unsupported clock source to 0x888 which indicates invalid value */
    [SOC_RcmPeripheralClockSource_DPLL_CORE_HSDIV0_CLKOUT2] = 0x888U, /* Set unsupported clock source to 0x888 which indicates invalid value */
    [SOC_RcmPeripheralClockSource_DPLL_PER_HSDIV0_CLKOUT1] = 0x222U,
    [SOC_RcmPeripheralClockSource_DPLL_PER_HSDIV0_CLKOUT2] = 0x333U,
    [SOC_RcmPeripheralClockSource_DPLL_PER_HSDIV0_CLKOUT3] = 0x444U,
    [SOC_RcmPeripheralClockSource_WUCPU_CLK] = 0x111U,
    [SOC_RcmPeripheralClockSource_XREF_CLK0] = 0x666U,
    [SOC_RcmPeripheralClockSource_XREF_CLK1] = 0x777U,
};

static const uint16_t gSocRcmCptsClkSrcValMap[] =
{
    [SOC_RcmPeripheralClockSource_XTAL_CLK] = 0x111U,
    [SOC_RcmPeripheralClockSource_SYS_CLK] = 0x222U,
    [SOC_RcmPeripheralClockSource_DPLL_CORE_HSDIV0_CLKOUT1] = 0x333U,
    [SOC_RcmPeripheralClockSource_DPLL_CORE_HSDIV0_CLKOUT2] = 0x444U,
    [SOC_RcmPeripheralClockSource_DPLL_PER_HSDIV0_CLKOUT1] = 0x888U, /* Set unsupported clock source to 0x888 which indicates invalid value */
    [SOC_RcmPeripheralClockSource_DPLL_PER_HSDIV0_CLKOUT2]  = 0x888U, /* Set unsupported clock source to 0x888 which indicates invalid value */
    [SOC_RcmPeripheralClockSource_DPLL_PER_HSDIV0_CLKOUT3] = 0x888U, /* Set unsupported clock source to 0x888 which indicates invalid value */
};

static const uint16_t gSocRcmCpswClkSrcValMap[] =
{
    [SOC_RcmPeripheralClockSource_XTAL_CLK] = 0x111U,
    [SOC_RcmPeripheralClockSource_SYS_CLK] = 0x222U,
    [SOC_RcmPeripheralClockSource_DPLL_CORE_HSDIV0_CLKOUT1] = 0x888U, /* Set unsupported clock source to 0x888 which indicates invalid value */
    [SOC_RcmPeripheralClockSource_DPLL_CORE_HSDIV0_CLKOUT2] = 0x444U,
    [SOC_RcmPeripheralClockSource_DPLL_PER_HSDIV0_CLKOUT1] = 0x888U, /* Set unsupported clock source to 0x888 which indicates invalid value */
    [SOC_RcmPeripheralClockSource_DPLL_PER_HSDIV0_CLKOUT2]  = 0x888U, /* Set unsupported clock source to 0x888 which indicates invalid value */
    [SOC_RcmPeripheralClockSource_DPLL_PER_HSDIV0_CLKOUT3] = 0x888U, /* Set unsupported clock source to 0x888 which indicates invalid value */
};

static const uint16_t gSocRcmDspCoreClkSrcValMap[] =
{
    [SOC_RcmDspClockSource_XTAL_CLK] = 0x111U,
    [SOC_RcmDspClockSource_DPLL_DSP_HSDIV0_CLKOUT1] = 0x222U,
    [SOC_RcmDspClockSource_DPLL_CORE_HSDIV0_CLKOUT1] = 0x444U
};

static const uint16_t gSocRcmR5ClkSrcValMap[] =
{
    [SOC_RcmR5ClockSource_DPLL_CORE_HSDIV0_CLKOUT2] = 0x222U,
};

/*
 *  Mapping Array between Reset Cause Bit and Reset Cause
 */
static const SOC_RcmResetCause gSocRcmResetBitToResetCause[11U] =
{
    SOC_RcmResetCause_POWER_ON_RESET,
    SOC_RcmResetCause_WARM_RESET,
    SOC_RcmResetCause_STC_RESET,
    SOC_RcmResetCause_MMR_CPU0_VIM0_RESET,
    SOC_RcmResetCause_MMR_CPU1_VIM1_RESET,
    SOC_RcmResetCause_MMR_CPU0_RESET,
    SOC_RcmResetCause_MMR_CPU1_RESET,
    SOC_RcmResetCause_DBG_CPU0_RESET,
    SOC_RcmResetCause_DBG_CPU1_RESET,
    SOC_RcmResetCause_FSM_TRIGGER_RESET,
    SOC_RcmResetCause_RST_CAUSE_UNKNOWN,
};

static uint32_t SOC_rcmMake8 (uint8_t msb, uint8_t lsb, uint8_t val)
{
    uint32_t    mask;
    uint8_t     bits;
    uint32_t    newVal;

    bits = (msb - lsb + 1U);
    mask = (uint32_t)((uint32_t)1U << bits);
    mask = mask - 1U;

    newVal = (uint32_t)val & mask;

    return (newVal << lsb);
}

static uint32_t SOC_rcmMake16 (uint8_t msb, uint8_t lsb, uint16_t val)
{
    uint32_t    mask;
    uint8_t     bits;
    uint32_t    newVal;

    /* Compute the mask: */
    bits = (msb - lsb + 1U);
    mask = (uint32_t)((uint32_t)1U << bits);
    mask = mask - 1U;

    newVal = (uint32_t)val & mask;

    return (newVal << lsb);
}

static uint32_t SOC_rcmMake32 (uint8_t msb, uint8_t lsb, uint32_t val)
{
    uint32_t    mask;
    uint8_t     bits;
    uint32_t    newVal;

    /* Compute the mask: */
    bits = (msb - lsb + 1U);
    mask = (uint32_t)((uint32_t)1U << bits);
    mask = mask - 1U;

    newVal = val & mask;

    return (newVal << lsb);
}

static uint32_t SOC_rcmInsert8 (volatile uint32_t reg, uint8_t msb, uint8_t lsb, uint8_t val)
{
    uint32_t    mask;
    uint8_t     bits;
    uint32_t    value;
    uint32_t    tmp;

    /* Compute the mask: */
    bits = (msb - lsb + 1U);
    mask = (uint32_t)((uint32_t)1U << bits);
    mask = mask - 1U;

    value = (mask << lsb);
    tmp   = (reg & ~value);
    return (tmp | SOC_rcmMake8(msb, lsb, val));
}

static uint32_t SOC_rcmInsert16 (volatile uint32_t reg, uint8_t msb, uint8_t lsb, uint16_t val)
{
    uint32_t    mask;
    uint8_t     bits;
    uint32_t    value;
    uint32_t    tmp;

    /* Compute the mask: */
    bits = (msb - lsb + 1U);
    mask = (uint32_t)((uint32_t)1U << bits);
    mask = mask - 1U;

    value = (mask << lsb);
    tmp   = (reg & ~value);
    return (tmp | SOC_rcmMake16(msb, lsb, val));
}

static uint32_t SOC_rcmInsert32 (volatile uint32_t reg, uint8_t msb, uint8_t lsb, uint32_t val)
{
    uint32_t    mask;
    uint8_t     bits;
    uint32_t    value;
    uint32_t    tmp;

    /* Compute the mask: */
    bits = (msb - lsb + 1U);
    mask = (uint32_t)((uint32_t)1U << bits);
    mask = mask - 1U;

    value = (mask << lsb);
    tmp   = (reg & ~value);
    return (tmp | SOC_rcmMake32(msb, lsb, val));
}

static uint8_t SOC_rcmExtract8 (volatile uint32_t reg, uint8_t msb, uint8_t lsb)
{
    uint32_t    mask;
    uint8_t     bits;
    uint8_t     value;

    /* Compute the mask: */
    bits = (msb - lsb + 1U);
    mask = (uint32_t)((uint32_t)1U << bits);
    mask = mask - 1U;

    value = (uint8_t)((reg >> lsb) & mask);
    return value;
}

static uint16_t SOC_rcmExtract16 (volatile uint32_t reg, uint8_t msb, uint8_t lsb)
{
    uint32_t    mask;
    uint8_t     bits;
    uint16_t    value;

    /* Compute the mask: */
    bits = (msb - lsb + 1U);
    mask = (uint32_t)((uint32_t)1U << bits);
    mask = mask - 1U;

    value = (uint16_t)((reg >> lsb) & mask);
    return value;
}

static uint32_t SOC_rcmExtract32 (volatile uint32_t reg, uint8_t msb, uint8_t lsb)
{
    uint32_t    mask;
    uint8_t     bits;
    uint32_t    value;

    /* Compute the mask: */
    bits = (msb - lsb + 1U);
    mask = (uint32_t)((uint32_t)1U << bits);
    mask = mask - 1U;

    value = (reg >> lsb) & mask;
    return value;
}

static CSL_top_ctrlRegs* SOC_rcmGetBaseAddressTOPCTRL (void)
{
    return (CSL_top_ctrlRegs*)CSL_TOP_CTRL_U_BASE;
}

static CSL_mss_ctrlRegs* SOC_rcmGetBaseAddressMSSCTRL (void)
{
    return (CSL_mss_ctrlRegs*) CSL_MSS_CTRL_U_BASE;
}

static CSL_dss_ctrlRegs* SOC_rcmGetBaseAddressDSSCTRL (void)
{
    return (CSL_dss_ctrlRegs *) CSL_DSS_CTRL_U_BASE;
}

static CSL_mss_toprcmRegs* SOC_rcmGetBaseAddressTOPRCM (void)
{
    return (CSL_mss_toprcmRegs*) CSL_MSS_TOPRCM_U_BASE;
}

static CSL_mss_rcmRegs* SOC_rcmGetBaseAddressMSSRCM (void)
{
    return (CSL_mss_rcmRegs*) CSL_MSS_RCM_U_BASE;
}

static CSL_dss_rcmRegs* SOC_rcmGetBaseAddressDSSRCM (void)
{
    return (CSL_dss_rcmRegs*) CSL_DSS_RCM_U_BASE;
}

static CSL_rcss_rcmRegs* SOC_rcmGetBaseAddressRCSSRCM (void)
{
    return (CSL_rcss_rcmRegs*) CSL_RCSS_RCM_U_BASE;
}

static uint8_t SOC_rcmReadXtalFreqScale (const CSL_top_ctrlRegs* ptrTopCtrlRegs)
{
    return (SOC_rcmExtract8 (ptrTopCtrlRegs->EFUSE1_ROW_10, \
                          SOC_RCM_XTAL_FREQ_SCALE_END_BIT, \
                          SOC_RCM_XTAL_FREQ_SCALE_START_BIT));
}

static uint8_t SOC_rcmReadCoreADPLLTrimValidEfuse (const CSL_top_ctrlRegs* ptrTopCtrlRegs)
{
    return (SOC_rcmExtract8 (ptrTopCtrlRegs->EFUSE1_ROW_41, \
                          SOC_RCM_COREADPLLTRIMVALIDCONFIG_END_BIT, \
                          SOC_RCM_COREADPLLTRIMVALIDCONFIG_START_BIT));
}

static uint8_t SOC_rcmReadDspADPLLTrimValidEfuse (const CSL_top_ctrlRegs* ptrTopCtrlRegs)
{
    return (SOC_rcmExtract8 (ptrTopCtrlRegs->EFUSE1_ROW_41, \
                          SOC_RCM_DSPADPLLTRIMVALIDCONFIG_END_BIT, \
                          SOC_RCM_DSPADPLLTRIMVALIDCONFIG_START_BIT));
}

static uint8_t SOC_rcmReadPerADPLLTrimValidEfuse (const CSL_top_ctrlRegs* ptrTopCtrlRegs)
{
    return (SOC_rcmExtract8 (ptrTopCtrlRegs->EFUSE1_ROW_41, \
                          SOC_RCM_PERADPLLTRIMVALIDCONFIG_END_BIT, \
                          SOC_RCM_PERADPLLTRIMVALIDCONFIG_START_BIT));
}

static uint8_t SOC_rcmReadQspiClkFreqEfuse (const CSL_top_ctrlRegs * ptrTopCtrlRegs)
{
    return (SOC_rcmExtract8 (ptrTopCtrlRegs->EFUSE1_ROW_11, \
                          SOC_RCM_QSPIFREQCONFIG_END_BIT, \
                          SOC_RCM_QSPIFREQCONFIG_START_BIT));
}

static uint8_t SOC_rcmReadFlashModeEfuse (const CSL_top_ctrlRegs * ptrTopCtrlRegs)
{
    return (SOC_rcmExtract8 (ptrTopCtrlRegs->EFUSE1_ROW_11, \
                          SOC_RCM_FLASHMODECONFIG_END_BIT, \
                          SOC_RCM_FLASHMODECONFIG_START_BIT));
}

static uint16_t SOC_rcmReadCoreADPLLTrimEfuse (const CSL_top_ctrlRegs* ptrTopCtrlRegs)
{
    return (SOC_rcmExtract16 (ptrTopCtrlRegs->EFUSE1_ROW_40, \
                           SOC_RCM_COREADPLLTRIMCONFIG_END_BIT, \
                           SOC_RCM_COREADPLLTRIMCONFIG_START_BIT));
}

static uint16_t SOC_rcmReadDspADPLLTrimEfuse (const CSL_top_ctrlRegs* ptrTopCtrlRegs)
{
    return (SOC_rcmExtract16 (ptrTopCtrlRegs->EFUSE1_ROW_40, \
                           SOC_RCM_DSPADPLLTRIMCONFIG_END_BIT, \
                           SOC_RCM_DSPADPLLTRIMCONFIG_START_BIT));
}

static uint16_t SOC_rcmReadPerADPLLTrimEfuse (const CSL_top_ctrlRegs* ptrTopCtrlRegs)
{
    return (SOC_rcmExtract16 (ptrTopCtrlRegs->EFUSE1_ROW_41,
                           SOC_RCM_PERADPLLTRIMCONFIG_END_BIT,
                           SOC_RCM_PERADPLLTRIMCONFIG_START_BIT));
}

static uint16_t SOC_rcmGetCoreTrimVal(void)
{
    CSL_top_ctrlRegs *ptrTopCtrlRegs;
    uint8_t coreADPLLValid;
    uint16_t coreADPLLTrimVal;

    ptrTopCtrlRegs = SOC_rcmGetBaseAddressTOPCTRL ();

    coreADPLLValid = SOC_rcmReadCoreADPLLTrimValidEfuse (ptrTopCtrlRegs);

    if(coreADPLLValid == 1U)
    {
        coreADPLLTrimVal = SOC_rcmReadCoreADPLLTrimEfuse (ptrTopCtrlRegs);
    }
    else
    {
        coreADPLLTrimVal = (uint16_t) SOC_RCM_CORE_ADPLL_DEFAULT_VALUE;
    }
    return (coreADPLLTrimVal);
}

static uint16_t SOC_rcmGetDspTrimVal(void)
{
    CSL_top_ctrlRegs *ptrTopCtrlRegs;
    uint8_t dspADPLLValid;
    uint16_t dspADPLLTrimVal;

    ptrTopCtrlRegs = SOC_rcmGetBaseAddressTOPCTRL ();

    dspADPLLValid = SOC_rcmReadDspADPLLTrimValidEfuse (ptrTopCtrlRegs);

    if(dspADPLLValid == 1U)
    {
        dspADPLLTrimVal = SOC_rcmReadDspADPLLTrimEfuse (ptrTopCtrlRegs);
    }
    else
    {
        dspADPLLTrimVal = (uint16_t) SOC_RCM_DSP_ADPLL_DEFAULT_VALUE;
    }
    return (dspADPLLTrimVal);
}

static uint16_t SOC_rcmGetPerTrimVal(void)
{
    CSL_top_ctrlRegs *ptrTopCtrlRegs;
    uint8_t perADPLLValid;
    uint16_t perADPLLTrimVal;

    ptrTopCtrlRegs = SOC_rcmGetBaseAddressTOPCTRL ();

    perADPLLValid = SOC_rcmReadPerADPLLTrimValidEfuse (ptrTopCtrlRegs);

    if(perADPLLValid == 1U)
    {
        perADPLLTrimVal = SOC_rcmReadPerADPLLTrimEfuse (ptrTopCtrlRegs);
    }
    else
    {
        perADPLLTrimVal = (uint16_t) SOC_RCM_PER_ADPLL_DEFAULT_VALUE;
    }
    return (perADPLLTrimVal);
}

static uint32_t SOC_rcmGetClkSrcFromClkSelVal(const uint16_t *clkSelTbl, uint32_t numEntries, uint32_t clkSelMatchVal)
{
    uint32_t i;
    uint32_t clkSource;

    for (i = 0; i < numEntries; i++)
    {
        if (clkSelMatchVal == clkSelTbl[i])
        {
            break;
        }
    }
    if (i < numEntries)
    {
        clkSource = i;
    }
    else
    {
        clkSource = ~0U;
    }
    return clkSource;
}

static void SOC_rcmGetClkSrcAndDivReg (SOC_RcmPeripheralId periphId,
                                SOC_RcmPeripheralClockSource clkSource,
                                uint16_t *clkSrcVal,
                                volatile uint32_t **clkSrcReg,
                                volatile uint32_t **clkdDivReg)
{
    CSL_mss_rcmRegs *ptrMSSRCMRegs;
    CSL_dss_rcmRegs *ptrDSSRCMRegs;
    CSL_rcss_rcmRegs *ptrRCSSRCMRegs;
    CSL_mss_toprcmRegs *ptrTOPRCMRegs;

    ptrMSSRCMRegs = SOC_rcmGetBaseAddressMSSRCM ();
    ptrDSSRCMRegs = SOC_rcmGetBaseAddressDSSRCM ();
    ptrTOPRCMRegs = SOC_rcmGetBaseAddressTOPRCM ();
    ptrRCSSRCMRegs = SOC_rcmGetBaseAddressRCSSRCM ();

    switch (periphId)
    {
        case SOC_RcmPeripheralId_MSS_RTIA:
        {
            *clkSrcReg  = &(ptrMSSRCMRegs->MSS_RTIA_CLK_SRC_SEL);
            *clkdDivReg = &(ptrMSSRCMRegs->MSS_RTIA_CLK_DIV_VAL);
            *clkSrcVal = gSocRcmRtiClkSrcValMap[clkSource];
            break;
        }
        case SOC_RcmPeripheralId_MSS_RTIB:
        {
            *clkSrcReg  = &(ptrMSSRCMRegs->MSS_RTIB_CLK_SRC_SEL);
            *clkdDivReg = &(ptrMSSRCMRegs->MSS_RTIB_CLK_DIV_VAL);
            *clkSrcVal = gSocRcmRtiClkSrcValMap[clkSource];
            break;
        }
        case SOC_RcmPeripheralId_MSS_RTIC:
        {
            *clkSrcReg  = &(ptrMSSRCMRegs->MSS_RTIC_CLK_SRC_SEL);
            *clkdDivReg = &(ptrMSSRCMRegs->MSS_RTIC_CLK_DIV_VAL);
            *clkSrcVal = gSocRcmRtiClkSrcValMap[clkSource];
            break;
        }
        case SOC_RcmPeripheralId_MSS_WDT:
        {
            *clkSrcReg  = &(ptrMSSRCMRegs->MSS_WDT_CLK_SRC_SEL);
            *clkdDivReg = &(ptrMSSRCMRegs->MSS_WDT_CLK_DIV_VAL);
            *clkSrcVal = gSocRcmRtiClkSrcValMap[clkSource];
            break;
        }
        case SOC_RcmPeripheralId_MSS_SCIA:
        {
            *clkSrcReg  = &(ptrMSSRCMRegs->MSS_SCIA_CLK_SRC_SEL);
            *clkdDivReg = &(ptrMSSRCMRegs->MSS_SCIA_CLK_DIV_VAL);
            *clkSrcVal = gSocRcmSciClkSrcValMap[clkSource];
            break;
        }
        case SOC_RcmPeripheralId_MSS_SCIB:
        {
            *clkSrcReg  = &(ptrMSSRCMRegs->MSS_SCIB_CLK_SRC_SEL);
            *clkdDivReg = &(ptrMSSRCMRegs->MSS_SCIB_CLK_DIV_VAL);
            *clkSrcVal = gSocRcmSciClkSrcValMap[clkSource];
            break;
        }
        case SOC_RcmPeripheralId_MSS_SPIA:
        {
            *clkSrcReg  = &(ptrMSSRCMRegs->MSS_SPIA_CLK_SRC_SEL);
            *clkdDivReg = &(ptrMSSRCMRegs->MSS_SPIA_CLK_DIV_VAL);
            *clkSrcVal = gSocRcmSpiClkSrcValMap[clkSource];
            break;
        }
        case SOC_RcmPeripheralId_MSS_SPIB:
        {
            *clkSrcReg  = &(ptrMSSRCMRegs->MSS_SPIB_CLK_SRC_SEL);
            *clkdDivReg = &(ptrMSSRCMRegs->MSS_SPIB_CLK_DIV_VAL);
            *clkSrcVal = gSocRcmSpiClkSrcValMap[clkSource];
            break;
        }
        case SOC_RcmPeripheralId_MSS_I2C:
        {
            *clkSrcReg  = &(ptrMSSRCMRegs->MSS_I2C_CLK_SRC_SEL);
            *clkdDivReg = &(ptrMSSRCMRegs->MSS_I2C_CLK_DIV_VAL);
            *clkSrcVal = gSocRcmI2cClkSrcValMap[clkSource];
            break;
        }
        case SOC_RcmPeripheralId_MSS_QSPI:
        {
            *clkSrcReg  = &(ptrMSSRCMRegs->MSS_QSPI_CLK_SRC_SEL);
            *clkdDivReg = &(ptrMSSRCMRegs->MSS_QSPI_CLK_DIV_VAL);
            *clkSrcVal = gSocRcmQspiClkSrcValMap[clkSource];
            break;
        }
        case SOC_RcmPeripheralId_MSS_MCANA:
        {
            *clkSrcReg  = &(ptrMSSRCMRegs->MSS_MCANA_CLK_SRC_SEL);
            *clkdDivReg = &(ptrMSSRCMRegs->MSS_MCANA_CLK_DIV_VAL);
            *clkSrcVal = gSocRcmMcanClkSrcValMap[clkSource];
            break;
        }
        case SOC_RcmPeripheralId_MSS_MCANB:
        {
            *clkSrcReg  = &(ptrMSSRCMRegs->MSS_MCANB_CLK_SRC_SEL);
            *clkdDivReg = &(ptrMSSRCMRegs->MSS_MCANB_CLK_DIV_VAL);
            *clkSrcVal = gSocRcmMcanClkSrcValMap[clkSource];
            break;
        }
        case SOC_RcmPeripheralId_MSS_CPSW:
        {
            *clkSrcReg  = &(ptrMSSRCMRegs->MSS_CPSW_CLK_SRC_SEL);
            *clkdDivReg = &(ptrMSSRCMRegs->MSS_CPSW_CLK_DIV_VAL);
            *clkSrcVal = gSocRcmCpswClkSrcValMap[clkSource];
            break;
        }
        case SOC_RcmPeripheralId_MSS_CPTS:
        {
            *clkSrcReg  = &(ptrMSSRCMRegs->MSS_CPTS_CLK_SRC_SEL);
            *clkdDivReg = &(ptrMSSRCMRegs->MSS_CPTS_CLK_DIV_VAL);
            *clkSrcVal = gSocRcmCptsClkSrcValMap[clkSource];
            break;
        }
        case SOC_RcmPeripheralId_CSIRX:
        {
            *clkSrcReg  = &(ptrTOPRCMRegs->CSIRX_CLK_SRC_SEL);
            *clkdDivReg = &(ptrTOPRCMRegs->CSIRX_DIV_VAL);
            *clkSrcVal = gSocRcmCsiRxClkSrcValMap[clkSource];
            break;
        }
        case SOC_RcmPeripheralId_DSS_RTIA:
        {
            *clkSrcReg  = &(ptrDSSRCMRegs->DSS_RTIA_CLK_SRC_SEL);
            *clkdDivReg = &(ptrDSSRCMRegs->DSS_RTIA_CLK_DIV_VAL);
            *clkSrcVal = gSocRcmDssRtiClkSrcValMap[clkSource];
            break;
        }
        case SOC_RcmPeripheralId_DSS_RTIB:
        {
            *clkSrcReg  = &(ptrDSSRCMRegs->DSS_RTIB_CLK_SRC_SEL);
            *clkdDivReg = &(ptrDSSRCMRegs->DSS_RTIB_CLK_DIV_VAL);
            *clkSrcVal = gSocRcmDssRtiClkSrcValMap[clkSource];
            break;
        }
        case SOC_RcmPeripheralId_DSS_WDT:
        {
            *clkSrcReg  = &(ptrDSSRCMRegs->DSS_WDT_CLK_SRC_SEL);
            *clkdDivReg = &(ptrDSSRCMRegs->DSS_WDT_CLK_DIV_VAL);
            *clkSrcVal = gSocRcmDssRtiClkSrcValMap[clkSource];
            break;
        }
        case SOC_RcmPeripheralId_DSS_SCIA:
        {
            *clkSrcReg  = &(ptrDSSRCMRegs->DSS_SCIA_CLK_SRC_SEL);
            *clkdDivReg = &(ptrDSSRCMRegs->DSS_SCIA_CLK_DIV_VAL);
            *clkSrcVal = gSocRcmDssSciClkSrcValMap[clkSource];
            break;
        }
        case SOC_RcmPeripheralId_RCSS_SCIA:
        {
            *clkSrcReg  = &(ptrRCSSRCMRegs->RCSS_SCIA_CLK_SRC_SEL);
            *clkdDivReg = &(ptrRCSSRCMRegs->RCSS_SCIA_CLK_DIV_VAL);
            *clkSrcVal = gSocRcmRcssSciClkSrcValMap[clkSource];
            break;
        }
        case SOC_RcmPeripheralId_RCSS_SPIA:
        {
            *clkSrcReg  = &(ptrRCSSRCMRegs->RCSS_SPIA_CLK_SRC_SEL);
            *clkdDivReg = &(ptrRCSSRCMRegs->RCSS_SPIA_CLK_DIV_VAL);
            *clkSrcVal = gSocRcmRcssSpiClkSrcValMap[clkSource];
            break;
        }
        case SOC_RcmPeripheralId_RCSS_SPIB:
        {
            *clkSrcReg  = &(ptrRCSSRCMRegs->RCSS_SPIB_CLK_SRC_SEL);
            *clkdDivReg = &(ptrRCSSRCMRegs->RCSS_SPIB_CLK_DIV_VAL);
            *clkSrcVal = gSocRcmRcssSpiClkSrcValMap[clkSource];
            break;
        }
        case SOC_RcmPeripheralId_RCSS_I2CA:
        {
            *clkSrcReg  = &(ptrRCSSRCMRegs->RCSS_I2CA_CLK_SRC_SEL);
            *clkdDivReg = &(ptrRCSSRCMRegs->RCSS_I2CA_CLK_DIV_VAL);
            *clkSrcVal = gSocRcmRcssI2cClkSrcValMap[clkSource];
            break;
        }
        case SOC_RcmPeripheralId_RCSS_I2CB:
        {
            *clkSrcReg  = &(ptrRCSSRCMRegs->RCSS_I2CB_CLK_SRC_SEL);
            *clkdDivReg = &(ptrRCSSRCMRegs->RCSS_I2CB_CLK_DIV_VAL);
            *clkSrcVal = gSocRcmRcssI2cClkSrcValMap[clkSource];
            break;
        }
        case SOC_RcmPeripheralId_RCSS_ATL:
        {
            *clkSrcReg  = &(ptrRCSSRCMRegs->RCSS_ATL_CLK_SRC_SEL);
            *clkdDivReg = &(ptrRCSSRCMRegs->RCSS_ATL_CLK_DIV_VAL);
            *clkSrcVal = gSocRcmRcssAtlClkSrcValMap[clkSource];
            break;
        }
        case SOC_RcmPeripheralId_RCSS_MCASPA_AUX:
        {
            *clkSrcReg  = &(ptrRCSSRCMRegs->RCSS_MCASPA_AUX_CLK_SRC_SEL);
            *clkdDivReg = &(ptrRCSSRCMRegs->RCSS_MCASPA_AUX_CLK_DIV_VAL);
            *clkSrcVal = gSocRcmRcssMcaspAuxClkSrcValMap[clkSource];
            break;
        }
        case SOC_RcmPeripheralId_RCSS_MCASPB_AUX:
        {
            *clkSrcReg  = &(ptrRCSSRCMRegs->RCSS_MCASPB_AUX_CLK_SRC_SEL);
            *clkdDivReg = &(ptrRCSSRCMRegs->RCSS_MCASPB_AUX_CLK_DIV_VAL);
            *clkSrcVal = gSocRcmRcssMcaspAuxClkSrcValMap[clkSource];
            break;
        }
        case SOC_RcmPeripheralId_RCSS_MCASPC_AUX:
        {
            *clkSrcReg  = &(ptrRCSSRCMRegs->RCSS_MCASPC_AUX_CLK_SRC_SEL);
            *clkdDivReg = &(ptrRCSSRCMRegs->RCSS_MCASPC_AUX_CLK_DIV_VAL);
            *clkSrcVal = gSocRcmRcssMcaspAuxClkSrcValMap[clkSource];
            break;
        }
        default:
        {
            *clkSrcReg  = NULL;
            *clkdDivReg = NULL;
            *clkSrcVal = 0x888U;
        }
    }
    return;
}

static int32_t SOC_rcmGetClkSrcAndDivValue (SOC_RcmPeripheralId periphId,
                                        SOC_RcmPeripheralClockSource *clkSource,
                                        volatile uint32_t *clkDiv)
{
    CSL_mss_rcmRegs *ptrMSSRCMRegs;
    CSL_dss_rcmRegs *ptrDSSRCMRegs;
    CSL_rcss_rcmRegs *ptrRCSSRCMRegs;
    CSL_mss_toprcmRegs *ptrTOPRCMRegs;
    uint32_t clkSrc;
    uint32_t clkSrcId;
    int32_t retVal = SystemP_SUCCESS;

    ptrMSSRCMRegs = SOC_rcmGetBaseAddressMSSRCM ();
    ptrDSSRCMRegs = SOC_rcmGetBaseAddressDSSRCM ();
    ptrTOPRCMRegs = SOC_rcmGetBaseAddressTOPRCM ();
    ptrRCSSRCMRegs = SOC_rcmGetBaseAddressRCSSRCM ();

    switch (periphId)
    {
        case SOC_RcmPeripheralId_MSS_RTIA:
        {
            clkSrc  = ptrMSSRCMRegs->MSS_RTIA_CLK_SRC_SEL;
            *clkDiv = ptrMSSRCMRegs->MSS_RTIA_CLK_DIV_VAL;
            clkSrcId = SOC_rcmGetClkSrcFromClkSelVal(gSocRcmRtiClkSrcValMap, SOC_RCM_UTILS_ARRAYSIZE(gSocRcmRtiClkSrcValMap), clkSrc);
            DebugP_assert(clkSrcId != ~0U);
            *clkSource = (SOC_RcmPeripheralClockSource) clkSrcId;
            break;
        }
        case SOC_RcmPeripheralId_MSS_RTIB:
        {
            clkSrc  = ptrMSSRCMRegs->MSS_RTIB_CLK_SRC_SEL;
            *clkDiv = ptrMSSRCMRegs->MSS_RTIB_CLK_DIV_VAL;
            clkSrcId = SOC_rcmGetClkSrcFromClkSelVal(gSocRcmRtiClkSrcValMap, SOC_RCM_UTILS_ARRAYSIZE(gSocRcmRtiClkSrcValMap), clkSrc);
            DebugP_assert(clkSrcId != ~0U);
            *clkSource = (SOC_RcmPeripheralClockSource) clkSrcId;
            break;
        }
        case SOC_RcmPeripheralId_MSS_RTIC:
        {
            clkSrc  = ptrMSSRCMRegs->MSS_RTIC_CLK_SRC_SEL;
            *clkDiv = ptrMSSRCMRegs->MSS_RTIC_CLK_DIV_VAL;
            clkSrcId = SOC_rcmGetClkSrcFromClkSelVal(gSocRcmRtiClkSrcValMap, SOC_RCM_UTILS_ARRAYSIZE(gSocRcmRtiClkSrcValMap), clkSrc);
            DebugP_assert(clkSrcId != ~0U);
            *clkSource = (SOC_RcmPeripheralClockSource) clkSrcId;

            break;
        }
        case SOC_RcmPeripheralId_MSS_WDT:
        {
            clkSrc  = ptrMSSRCMRegs->MSS_WDT_CLK_SRC_SEL;
            *clkDiv = ptrMSSRCMRegs->MSS_WDT_CLK_DIV_VAL;
            clkSrcId = SOC_rcmGetClkSrcFromClkSelVal(gSocRcmRtiClkSrcValMap, SOC_RCM_UTILS_ARRAYSIZE(gSocRcmRtiClkSrcValMap), clkSrc);
            DebugP_assert(clkSrcId != ~0U);
            *clkSource = (SOC_RcmPeripheralClockSource) clkSrcId;
            break;
        }
        case SOC_RcmPeripheralId_MSS_SCIA:
        {
            clkSrc  = ptrMSSRCMRegs->MSS_SCIA_CLK_SRC_SEL;
            *clkDiv = ptrMSSRCMRegs->MSS_SCIA_CLK_DIV_VAL;
            clkSrcId = SOC_rcmGetClkSrcFromClkSelVal(gSocRcmSciClkSrcValMap, SOC_RCM_UTILS_ARRAYSIZE(gSocRcmSciClkSrcValMap), clkSrc);
            DebugP_assert(clkSrcId != ~0U);
            *clkSource = (SOC_RcmPeripheralClockSource) clkSrcId;
            break;
        }
        case SOC_RcmPeripheralId_MSS_SCIB:
        {
            clkSrc  = ptrMSSRCMRegs->MSS_SCIB_CLK_SRC_SEL;
            *clkDiv = ptrMSSRCMRegs->MSS_SCIB_CLK_DIV_VAL;
            clkSrcId = SOC_rcmGetClkSrcFromClkSelVal(gSocRcmSciClkSrcValMap, SOC_RCM_UTILS_ARRAYSIZE(gSocRcmSciClkSrcValMap), clkSrc);
            DebugP_assert(clkSrcId != ~0U);
            *clkSource = (SOC_RcmPeripheralClockSource) clkSrcId;
            break;
        }
        case SOC_RcmPeripheralId_MSS_SPIA:
        {
            clkSrc  = ptrMSSRCMRegs->MSS_SPIA_CLK_SRC_SEL;
            *clkDiv = ptrMSSRCMRegs->MSS_SPIA_CLK_DIV_VAL;
            clkSrcId = SOC_rcmGetClkSrcFromClkSelVal(gSocRcmSpiClkSrcValMap, SOC_RCM_UTILS_ARRAYSIZE(gSocRcmSpiClkSrcValMap), clkSrc);
            DebugP_assert(clkSrcId != ~0U);
            *clkSource = (SOC_RcmPeripheralClockSource) clkSrcId;
            break;
        }
        case SOC_RcmPeripheralId_MSS_SPIB:
        {
            clkSrc  = ptrMSSRCMRegs->MSS_SPIB_CLK_SRC_SEL;
            *clkDiv = ptrMSSRCMRegs->MSS_SPIB_CLK_DIV_VAL;
            clkSrcId = SOC_rcmGetClkSrcFromClkSelVal(gSocRcmSpiClkSrcValMap, SOC_RCM_UTILS_ARRAYSIZE(gSocRcmSpiClkSrcValMap), clkSrc);
            DebugP_assert(clkSrcId != ~0U);
            *clkSource = (SOC_RcmPeripheralClockSource) clkSrcId;
            break;
        }
        case SOC_RcmPeripheralId_MSS_I2C:
        {
            clkSrc  = ptrMSSRCMRegs->MSS_I2C_CLK_SRC_SEL;
            *clkDiv = ptrMSSRCMRegs->MSS_I2C_CLK_DIV_VAL;
            clkSrcId = SOC_rcmGetClkSrcFromClkSelVal(gSocRcmI2cClkSrcValMap, SOC_RCM_UTILS_ARRAYSIZE(gSocRcmI2cClkSrcValMap), clkSrc);
            DebugP_assert(clkSrcId != ~0U);
            *clkSource = (SOC_RcmPeripheralClockSource) clkSrcId;
            break;
        }
        case SOC_RcmPeripheralId_MSS_QSPI:
        {
            clkSrc  = ptrMSSRCMRegs->MSS_QSPI_CLK_SRC_SEL;
            *clkDiv = ptrMSSRCMRegs->MSS_QSPI_CLK_DIV_VAL;
            clkSrcId = SOC_rcmGetClkSrcFromClkSelVal(gSocRcmQspiClkSrcValMap, SOC_RCM_UTILS_ARRAYSIZE(gSocRcmQspiClkSrcValMap), clkSrc);
            DebugP_assert(clkSrcId != ~0U);
            *clkSource = (SOC_RcmPeripheralClockSource) clkSrcId;
            break;
        }
        case SOC_RcmPeripheralId_MSS_MCANA:
        {
            clkSrc  = ptrMSSRCMRegs->MSS_MCANA_CLK_SRC_SEL;
            *clkDiv = ptrMSSRCMRegs->MSS_MCANA_CLK_DIV_VAL;
            clkSrcId = SOC_rcmGetClkSrcFromClkSelVal(gSocRcmMcanClkSrcValMap, SOC_RCM_UTILS_ARRAYSIZE(gSocRcmMcanClkSrcValMap), clkSrc);
            DebugP_assert(clkSrcId != ~0U);
            *clkSource = (SOC_RcmPeripheralClockSource) clkSrcId;
            break;
        }
        case SOC_RcmPeripheralId_MSS_MCANB:
        {
            clkSrc  = ptrMSSRCMRegs->MSS_MCANB_CLK_SRC_SEL;
            *clkDiv = ptrMSSRCMRegs->MSS_MCANB_CLK_DIV_VAL;
            clkSrcId = SOC_rcmGetClkSrcFromClkSelVal(gSocRcmMcanClkSrcValMap, SOC_RCM_UTILS_ARRAYSIZE(gSocRcmMcanClkSrcValMap), clkSrc);
            DebugP_assert(clkSrcId != ~0U);
            *clkSource = (SOC_RcmPeripheralClockSource) clkSrcId;
            break;
        }
        case SOC_RcmPeripheralId_MSS_CPSW:
        {
            clkSrc  = ptrMSSRCMRegs->MSS_CPSW_CLK_SRC_SEL;
            *clkDiv = ptrMSSRCMRegs->MSS_CPSW_CLK_DIV_VAL;
            clkSrcId = SOC_rcmGetClkSrcFromClkSelVal(gSocRcmCpswClkSrcValMap, SOC_RCM_UTILS_ARRAYSIZE(gSocRcmCpswClkSrcValMap), clkSrc);
            DebugP_assert(clkSrcId != ~0U);
            *clkSource = (SOC_RcmPeripheralClockSource) clkSrcId;
            break;
        }
        case SOC_RcmPeripheralId_MSS_CPTS:
        {
            clkSrc  = ptrMSSRCMRegs->MSS_CPTS_CLK_SRC_SEL;
            *clkDiv = ptrMSSRCMRegs->MSS_CPTS_CLK_DIV_VAL;
            clkSrcId = SOC_rcmGetClkSrcFromClkSelVal(gSocRcmCptsClkSrcValMap, SOC_RCM_UTILS_ARRAYSIZE(gSocRcmCptsClkSrcValMap), clkSrc);
            DebugP_assert(clkSrcId != ~0U);
            *clkSource = (SOC_RcmPeripheralClockSource) clkSrcId;
            break;
        }
        case SOC_RcmPeripheralId_CSIRX:
        {
            clkSrc  = ptrTOPRCMRegs->CSIRX_CLK_SRC_SEL;
            *clkDiv = ptrTOPRCMRegs->CSIRX_DIV_VAL;
            clkSrcId = SOC_rcmGetClkSrcFromClkSelVal(gSocRcmCsiRxClkSrcValMap, SOC_RCM_UTILS_ARRAYSIZE(gSocRcmCsiRxClkSrcValMap), clkSrc);
            DebugP_assert(clkSrcId != ~0U);
            *clkSource = (SOC_RcmPeripheralClockSource) clkSrcId;
            break;
        }
        case SOC_RcmPeripheralId_DSS_RTIA:
        {
            clkSrc  = ptrDSSRCMRegs->DSS_RTIA_CLK_SRC_SEL;
            *clkDiv = ptrDSSRCMRegs->DSS_RTIA_CLK_DIV_VAL;
            clkSrcId = SOC_rcmGetClkSrcFromClkSelVal(gSocRcmDssRtiClkSrcValMap, SOC_RCM_UTILS_ARRAYSIZE(gSocRcmDssRtiClkSrcValMap), clkSrc);
            DebugP_assert(clkSrcId != ~0U);
            *clkSource = (SOC_RcmPeripheralClockSource) clkSrcId;
            break;
        }
        case SOC_RcmPeripheralId_DSS_RTIB:
        {
            clkSrc  = ptrDSSRCMRegs->DSS_RTIB_CLK_SRC_SEL;
            *clkDiv = ptrDSSRCMRegs->DSS_RTIB_CLK_DIV_VAL;
            clkSrcId = SOC_rcmGetClkSrcFromClkSelVal(gSocRcmDssRtiClkSrcValMap, SOC_RCM_UTILS_ARRAYSIZE(gSocRcmDssRtiClkSrcValMap), clkSrc);
            DebugP_assert(clkSrcId != ~0U);
            *clkSource = (SOC_RcmPeripheralClockSource) clkSrcId;
            break;
        }
        case SOC_RcmPeripheralId_DSS_WDT:
        {
            clkSrc  = ptrDSSRCMRegs->DSS_WDT_CLK_SRC_SEL;
            *clkDiv = ptrDSSRCMRegs->DSS_WDT_CLK_DIV_VAL;
            clkSrcId = SOC_rcmGetClkSrcFromClkSelVal(gSocRcmDssRtiClkSrcValMap, SOC_RCM_UTILS_ARRAYSIZE(gSocRcmDssRtiClkSrcValMap), clkSrc);
            DebugP_assert(clkSrcId != ~0U);
            *clkSource = (SOC_RcmPeripheralClockSource) clkSrcId;
            break;
        }
        case SOC_RcmPeripheralId_DSS_SCIA:
        {
            clkSrc  = ptrDSSRCMRegs->DSS_SCIA_CLK_SRC_SEL;
            *clkDiv = ptrDSSRCMRegs->DSS_SCIA_CLK_DIV_VAL;
            clkSrcId = SOC_rcmGetClkSrcFromClkSelVal(gSocRcmDssSciClkSrcValMap, SOC_RCM_UTILS_ARRAYSIZE(gSocRcmDssSciClkSrcValMap), clkSrc);
            DebugP_assert(clkSrcId != ~0U);
            *clkSource = (SOC_RcmPeripheralClockSource) clkSrcId;
            break;
        }
        case SOC_RcmPeripheralId_RCSS_SCIA:
        {
            clkSrc  = ptrRCSSRCMRegs->RCSS_SCIA_CLK_SRC_SEL;
            *clkDiv = ptrRCSSRCMRegs->RCSS_SCIA_CLK_DIV_VAL;
            clkSrcId = SOC_rcmGetClkSrcFromClkSelVal(gSocRcmRcssSciClkSrcValMap, SOC_RCM_UTILS_ARRAYSIZE(gSocRcmRcssSciClkSrcValMap), clkSrc);
            DebugP_assert(clkSrcId != ~0U);
            *clkSource = (SOC_RcmPeripheralClockSource) clkSrcId;
            break;
        }
        case SOC_RcmPeripheralId_RCSS_SPIA:
        {
            clkSrc  = ptrRCSSRCMRegs->RCSS_SPIA_CLK_SRC_SEL;
            *clkDiv = ptrRCSSRCMRegs->RCSS_SPIA_CLK_DIV_VAL;
            clkSrcId = SOC_rcmGetClkSrcFromClkSelVal(gSocRcmRcssSciClkSrcValMap, SOC_RCM_UTILS_ARRAYSIZE(gSocRcmRcssSciClkSrcValMap), clkSrc);
            DebugP_assert(clkSrcId != ~0U);
            *clkSource = (SOC_RcmPeripheralClockSource) clkSrcId;
            break;
        }
        case SOC_RcmPeripheralId_RCSS_SPIB:
        {
            clkSrc  = ptrRCSSRCMRegs->RCSS_SPIB_CLK_SRC_SEL;
            *clkDiv = ptrRCSSRCMRegs->RCSS_SPIB_CLK_DIV_VAL;
            clkSrcId = SOC_rcmGetClkSrcFromClkSelVal(gSocRcmRcssSciClkSrcValMap, SOC_RCM_UTILS_ARRAYSIZE(gSocRcmRcssSciClkSrcValMap), clkSrc);
            DebugP_assert(clkSrcId != ~0U);
            *clkSource = (SOC_RcmPeripheralClockSource) clkSrcId;
            break;
        }
        case SOC_RcmPeripheralId_RCSS_I2CA:
        {
            clkSrc  = ptrRCSSRCMRegs->RCSS_I2CA_CLK_SRC_SEL;
            *clkDiv = ptrRCSSRCMRegs->RCSS_I2CA_CLK_DIV_VAL;
            clkSrcId = SOC_rcmGetClkSrcFromClkSelVal(gSocRcmRcssI2cClkSrcValMap, SOC_RCM_UTILS_ARRAYSIZE(gSocRcmRcssI2cClkSrcValMap), clkSrc);
            DebugP_assert(clkSrcId != ~0U);
            *clkSource = (SOC_RcmPeripheralClockSource) clkSrcId;
            break;
        }
        case SOC_RcmPeripheralId_RCSS_I2CB:
        {
            clkSrc  = ptrRCSSRCMRegs->RCSS_I2CB_CLK_SRC_SEL;
            *clkDiv = ptrRCSSRCMRegs->RCSS_I2CB_CLK_DIV_VAL;
            clkSrcId = SOC_rcmGetClkSrcFromClkSelVal(gSocRcmRcssI2cClkSrcValMap, SOC_RCM_UTILS_ARRAYSIZE(gSocRcmRcssI2cClkSrcValMap), clkSrc);
            DebugP_assert(clkSrcId != ~0U);
            *clkSource = (SOC_RcmPeripheralClockSource) clkSrcId;
            break;
        }
        case SOC_RcmPeripheralId_RCSS_ATL:
        {
            clkSrc  = ptrRCSSRCMRegs->RCSS_ATL_CLK_SRC_SEL;
            *clkDiv = ptrRCSSRCMRegs->RCSS_ATL_CLK_DIV_VAL;
            clkSrcId = SOC_rcmGetClkSrcFromClkSelVal(gSocRcmRcssAtlClkSrcValMap, SOC_RCM_UTILS_ARRAYSIZE(gSocRcmRcssAtlClkSrcValMap), clkSrc);
            DebugP_assert(clkSrcId != ~0U);
            *clkSource = (SOC_RcmPeripheralClockSource) clkSrcId;
            break;
        }
        case SOC_RcmPeripheralId_RCSS_MCASPA_AUX:
        {
            clkSrc  = ptrRCSSRCMRegs->RCSS_MCASPA_AUX_CLK_SRC_SEL;
            *clkDiv = ptrRCSSRCMRegs->RCSS_MCASPA_AUX_CLK_DIV_VAL;
            clkSrcId = SOC_rcmGetClkSrcFromClkSelVal(gSocRcmRcssMcaspAuxClkSrcValMap, SOC_RCM_UTILS_ARRAYSIZE(gSocRcmRcssMcaspAuxClkSrcValMap), clkSrc);
            DebugP_assert(clkSrcId != ~0U);
            *clkSource = (SOC_RcmPeripheralClockSource) clkSrcId;
            break;
        }
        case SOC_RcmPeripheralId_RCSS_MCASPB_AUX:
        {
            clkSrc  = ptrRCSSRCMRegs->RCSS_MCASPB_AUX_CLK_SRC_SEL;
            *clkDiv = ptrRCSSRCMRegs->RCSS_MCASPB_AUX_CLK_DIV_VAL;
            clkSrcId = SOC_rcmGetClkSrcFromClkSelVal(gSocRcmRcssMcaspAuxClkSrcValMap, SOC_RCM_UTILS_ARRAYSIZE(gSocRcmRcssMcaspAuxClkSrcValMap), clkSrc);
            DebugP_assert(clkSrcId != ~0U);
            *clkSource = (SOC_RcmPeripheralClockSource) clkSrcId;
            break;
        }
        case SOC_RcmPeripheralId_RCSS_MCASPC_AUX:
        {
            clkSrc  = ptrRCSSRCMRegs->RCSS_MCASPC_AUX_CLK_SRC_SEL;
            *clkDiv = ptrRCSSRCMRegs->RCSS_MCASPC_AUX_CLK_DIV_VAL;
            clkSrcId = SOC_rcmGetClkSrcFromClkSelVal(gSocRcmRcssMcaspAuxClkSrcValMap, SOC_RCM_UTILS_ARRAYSIZE(gSocRcmRcssMcaspAuxClkSrcValMap), clkSrc);
            DebugP_assert(clkSrcId != ~0U);
            *clkSource = (SOC_RcmPeripheralClockSource) clkSrcId;
            break;
        }
        default:
        {
            *clkDiv = 0U;
            retVal = SystemP_FAILURE;
        }
    }
    return retVal;
}

static void SOC_rcmGetDspClkSrcAndDivReg (SOC_RcmDspClockSource clkSource,
                                   uint16_t *clkSrcVal,
                                   volatile uint32_t **clkSrcReg,
                                   volatile uint32_t **clkdDivReg)
{
    CSL_dss_rcmRegs *ptrDSSRCMRegs;

    ptrDSSRCMRegs = SOC_rcmGetBaseAddressDSSRCM ();

    *clkSrcReg  = &(ptrDSSRCMRegs->DSS_DSP_CLK_SRC_SEL);
    *clkdDivReg = &(ptrDSSRCMRegs->DSS_DSP_CLK_DIV_VAL);
    *clkSrcVal = gSocRcmDspCoreClkSrcValMap[clkSource];

    return;
}


static int32_t SOC_rcmGetDspClkSrcAndDivValue(SOC_RcmDspClockSource *clkSource,
                                          uint32_t *clkDiv)
{
    uint32_t clkSrc, clkSrcId;
    int32_t retVal = SystemP_SUCCESS;
    CSL_dss_rcmRegs *ptrDSSRCMRegs;

    ptrDSSRCMRegs = SOC_rcmGetBaseAddressDSSRCM ();
    clkSrc  = ptrDSSRCMRegs->DSS_DSP_CLK_SRC_SEL;
    *clkDiv = ptrDSSRCMRegs->DSS_DSP_CLK_DIV_VAL;
    clkSrcId = SOC_rcmGetClkSrcFromClkSelVal(gSocRcmDspCoreClkSrcValMap, SOC_RCM_UTILS_ARRAYSIZE(gSocRcmDspCoreClkSrcValMap), clkSrc);
    DebugP_assert(clkSrcId != ~0U);
    if (clkSrcId != ~0U)
    {
        *clkSource = (SOC_RcmDspClockSource) clkSrcId;
    }
    else
    {
        retVal = SystemP_FAILURE;
    }
    return retVal;
}

static uint32_t SOC_rcmGetModuleClkDivVal(uint32_t inFreq, uint32_t outFreq)
{
    uint32_t moduleClkDivVal;

    DebugP_assert((inFreq % outFreq) == 0);
    moduleClkDivVal = inFreq / outFreq;
    moduleClkDivVal--;
    return moduleClkDivVal;
}

static uint32_t SOC_rcmGetModuleClkDivRegVal(uint32_t moduleClkDivVal)
{
    uint32_t moduleClkDivRegVal;

    moduleClkDivRegVal = (moduleClkDivVal & 0xF) | ((moduleClkDivVal & 0xF) << 4) | ((moduleClkDivVal & 0xF) << 8);
    return moduleClkDivRegVal;
}

static uint32_t SOC_rcmGetModuleClkDivFromRegVal(uint32_t moduleClkDivRegVal)
{
    uint32_t moduleClkDivVal;

    moduleClkDivVal = ((moduleClkDivRegVal & 0xF) + 1);
    return moduleClkDivVal;
}

static SOC_RcmADPLLJConfig const * SOC_rcmGetADPLLJConfig(uint32_t Finp, SOC_RcmPllFoutFreqId foutFreqId)
{
    uint32_t i;
    SOC_RcmADPLLJConfig const *adplljCfg;
    uint32_t Fout;

    Fout = gSocRcmPllFreqId2FOutMap[foutFreqId];
    for (i = 0; i < SOC_RCM_UTILS_ARRAYSIZE(gSocRcmADPLLJConfigTbl); i++)
    {
        if ((gSocRcmADPLLJConfigTbl[i].Finp == Finp) && (gSocRcmADPLLJConfigTbl[i].Fout == Fout))
        {
            break;
        }
    }
    if (i < SOC_RCM_UTILS_ARRAYSIZE(gSocRcmADPLLJConfigTbl))
    {
        adplljCfg = &gSocRcmADPLLJConfigTbl[i];
    }
    else
    {
        adplljCfg = (SOC_RcmADPLLJConfig const *)NULL;
    }
    return adplljCfg;
}

static uint32_t SOC_rcmGetADPLLJFout(uint32_t Finp, uint32_t N, uint32_t M, uint32_t M2, uint32_t FracM, uint32_t div2flag)
{
    uint32_t i;
    uint32_t FOut;
    uint32_t Nmatch;

    if (div2flag)
    {
        Nmatch = ((N + 1) * 2) - 1;
    }
    else
    {
        Nmatch = N;
    }
    for (i = 0; i < SOC_RCM_UTILS_ARRAYSIZE(gSocRcmADPLLJConfigTbl); i++)
    {
        if ((gSocRcmADPLLJConfigTbl[i].Finp == Finp)   &&
            (gSocRcmADPLLJConfigTbl[i].FracM == FracM) &&
            (gSocRcmADPLLJConfigTbl[i].M == M)         &&
            (gSocRcmADPLLJConfigTbl[i].M2 == M2)       &&
            (gSocRcmADPLLJConfigTbl[i].N  == Nmatch))
        {
            break;
        }
    }
    if (i < SOC_RCM_UTILS_ARRAYSIZE(gSocRcmADPLLJConfigTbl))
    {
        FOut = SOC_RCM_FREQ_MHZ2HZ(gSocRcmADPLLJConfigTbl[i].Fout);
    }
    else
    {
        FOut = 0;
    }
    return FOut;
}

static uint32_t SOC_rcmGetDspFout(uint32_t Finp, uint32_t div2flag)
{
    uint8_t pllSwitchFlag;
    CSL_mss_toprcmRegs *ptrTopRCMRegs;
    uint32_t FOut;

    ptrTopRCMRegs = SOC_rcmGetBaseAddressTOPRCM ();
    /* read the Core PLL Lock status */
    pllSwitchFlag = SOC_rcmExtract8 (ptrTopRCMRegs->PLL_DSP_STATUS, 10U, 10U);
    if (pllSwitchFlag)
    {

        uint32_t M, N, M2, FracM;

        N  = CSL_FEXT(ptrTopRCMRegs->PLL_DSP_M2NDIV, MSS_TOPRCM_PLL_DSP_M2NDIV_PLL_DSP_M2NDIV_N);
        M2 = CSL_FEXT(ptrTopRCMRegs->PLL_DSP_M2NDIV, MSS_TOPRCM_PLL_DSP_M2NDIV_PLL_DSP_M2NDIV_M2);
        M  = CSL_FEXT(ptrTopRCMRegs->PLL_DSP_MN2DIV, MSS_TOPRCM_PLL_DSP_MN2DIV_PLL_DSP_MN2DIV_M);
        FracM = CSL_FEXT(ptrTopRCMRegs->PLL_DSP_FRACDIV,MSS_TOPRCM_PLL_DSP_FRACDIV_PLL_DSP_FRACDIV_FRACTIONALM);
        FOut = SOC_rcmGetADPLLJFout(Finp, N, M, M2, FracM, div2flag);
        DebugP_assert(FOut != 0);
    }
    else
    {
        uint32_t ULOWCLKEN = CSL_FEXT(ptrTopRCMRegs->PLL_DSP_CLKCTRL, MSS_TOPRCM_PLL_DSP_CLKCTRL_PLL_DSP_CLKCTRL_ULOWCLKEN);
        if (ULOWCLKEN == 0)
        {
            uint32_t N2;

            N2  = CSL_FEXT(ptrTopRCMRegs->PLL_DSP_MN2DIV, MSS_TOPRCM_PLL_DSP_MN2DIV_PLL_DSP_MN2DIV_N2);
            FOut = Finp/(N2 + 1);
        }
        else
        {
            FOut = Finp;
        }
    }
    return FOut;
}

static uint32_t SOC_rcmGetDspHsDivOut(uint32_t Finp, uint32_t div2flag, SOC_RcmPllHsDivOutId hsDivOut)
{
    CSL_mss_toprcmRegs *ptrTopRCMRegs;
    uint32_t FOut;
    uint32_t clkDiv;

    FOut = SOC_rcmGetDspFout(Finp, div2flag);
    ptrTopRCMRegs = SOC_rcmGetBaseAddressTOPRCM ();
    switch(hsDivOut)
    {
        case SOC_RcmPllHsDivOutId_0:
        {
            clkDiv = CSL_FEXT(ptrTopRCMRegs->PLL_DSP_HSDIVIDER_CLKOUT0, MSS_TOPRCM_PLL_DSP_HSDIVIDER_CLKOUT0_PLL_DSP_HSDIVIDER_CLKOUT0_DIV);
            break;
        }
        case SOC_RcmPllHsDivOutId_1:
        {
            clkDiv = CSL_FEXT(ptrTopRCMRegs->PLL_DSP_HSDIVIDER_CLKOUT1, MSS_TOPRCM_PLL_DSP_HSDIVIDER_CLKOUT1_PLL_DSP_HSDIVIDER_CLKOUT1_DIV);
            break;
        }
        case SOC_RcmPllHsDivOutId_2:
        {
            clkDiv = CSL_FEXT(ptrTopRCMRegs->PLL_DSP_HSDIVIDER_CLKOUT2, MSS_TOPRCM_PLL_DSP_HSDIVIDER_CLKOUT2_PLL_DSP_HSDIVIDER_CLKOUT2_DIV);
            break;
        }
        case SOC_RcmPllHsDivOutId_3:
        {
            clkDiv = CSL_FEXT(ptrTopRCMRegs->PLL_DSP_HSDIVIDER_CLKOUT3, MSS_TOPRCM_PLL_DSP_HSDIVIDER_CLKOUT3_PLL_DSP_HSDIVIDER_CLKOUT3_DIV);
            break;
        }
        default:
        {
            DebugP_assert(0);
            clkDiv = 0;
            break;
        }
    }
    return (FOut/(clkDiv + 1));
}

static uint32_t SOC_rcmGetCoreFout(uint32_t Finp, uint32_t div2flag)
{
    uint8_t pllSwitchFlag;
    CSL_mss_toprcmRegs *ptrTopRCMRegs;
    uint32_t FOut;

    ptrTopRCMRegs = SOC_rcmGetBaseAddressTOPRCM ();
    /* read the Core PLL Lock status */
    pllSwitchFlag = SOC_rcmExtract8 (ptrTopRCMRegs->PLL_CORE_STATUS, 10U, 10U);
    if (pllSwitchFlag)
    {
        uint32_t M, N, M2, FracM;

        N  = CSL_FEXT(ptrTopRCMRegs->PLL_CORE_M2NDIV, MSS_TOPRCM_PLL_CORE_M2NDIV_PLL_CORE_M2NDIV_N);
        M2 = CSL_FEXT(ptrTopRCMRegs->PLL_CORE_M2NDIV, MSS_TOPRCM_PLL_CORE_M2NDIV_PLL_CORE_M2NDIV_M2);
        M  = CSL_FEXT(ptrTopRCMRegs->PLL_CORE_MN2DIV, MSS_TOPRCM_PLL_CORE_MN2DIV_PLL_CORE_MN2DIV_M);
        FracM = CSL_FEXT(ptrTopRCMRegs->PLL_CORE_FRACDIV,MSS_TOPRCM_PLL_CORE_FRACDIV_PLL_CORE_FRACDIV_FRACTIONALM);
        FOut = SOC_rcmGetADPLLJFout(Finp, N, M, M2, FracM,div2flag);
        DebugP_assert(FOut != 0);
    }
    else
    {
        uint32_t ULOWCLKEN = CSL_FEXT(ptrTopRCMRegs->PLL_CORE_CLKCTRL, MSS_TOPRCM_PLL_CORE_CLKCTRL_PLL_CORE_CLKCTRL_ULOWCLKEN);
        if (ULOWCLKEN == 0)
        {
            uint32_t N2;

            N2  = CSL_FEXT(ptrTopRCMRegs->PLL_CORE_MN2DIV, MSS_TOPRCM_PLL_CORE_MN2DIV_PLL_CORE_MN2DIV_N2);
            FOut = Finp/(N2 + 1);
        }
        else
        {
            FOut = Finp;
        }
    }
    return FOut;
}

static uint32_t SOC_rcmGetCoreHsDivOut(uint32_t Finp, uint32_t div2flag, SOC_RcmPllHsDivOutId hsDivOut)
{
    uint32_t FOut;
    uint32_t clkDiv;
    CSL_mss_toprcmRegs *ptrTopRCMRegs;

    ptrTopRCMRegs = SOC_rcmGetBaseAddressTOPRCM ();
    FOut = SOC_rcmGetCoreFout(Finp, div2flag);
    switch(hsDivOut)
    {
        case SOC_RcmPllHsDivOutId_0:
        {
            clkDiv = CSL_FEXT(ptrTopRCMRegs->PLL_CORE_HSDIVIDER_CLKOUT0, MSS_TOPRCM_PLL_CORE_HSDIVIDER_CLKOUT0_PLL_CORE_HSDIVIDER_CLKOUT0_DIV);
            break;
        }
        case SOC_RcmPllHsDivOutId_1:
        {
            clkDiv = CSL_FEXT(ptrTopRCMRegs->PLL_CORE_HSDIVIDER_CLKOUT1, MSS_TOPRCM_PLL_CORE_HSDIVIDER_CLKOUT1_PLL_CORE_HSDIVIDER_CLKOUT1_DIV);
            break;
        }
        case SOC_RcmPllHsDivOutId_2:
        {
            clkDiv = CSL_FEXT(ptrTopRCMRegs->PLL_CORE_HSDIVIDER_CLKOUT2, MSS_TOPRCM_PLL_CORE_HSDIVIDER_CLKOUT2_PLL_CORE_HSDIVIDER_CLKOUT2_DIV);
            break;
        }
        case SOC_RcmPllHsDivOutId_3:
        {
            clkDiv = CSL_FEXT(ptrTopRCMRegs->PLL_CORE_HSDIVIDER_CLKOUT3, MSS_TOPRCM_PLL_CORE_HSDIVIDER_CLKOUT3_PLL_CORE_HSDIVIDER_CLKOUT3_DIV);
            break;
        }
        default:
        {
            DebugP_assert(0);
            clkDiv = 0;
            break;
        }
    }
    return (FOut/(clkDiv + 1));
}

static uint32_t SOC_rcmGetPerFout(uint32_t Finp, uint32_t div2flag)
{
    uint8_t pllSwitchFlag;
    CSL_mss_toprcmRegs *ptrTopRCMRegs;
    uint32_t FOut;

    ptrTopRCMRegs = SOC_rcmGetBaseAddressTOPRCM ();
    /* read the Core PLL Lock status */
    pllSwitchFlag = SOC_rcmExtract8 (ptrTopRCMRegs->PLL_PER_STATUS, 10U, 10U);
    if (pllSwitchFlag)
    {
        uint32_t M, N, M2, FracM;

        N  = CSL_FEXT(ptrTopRCMRegs->PLL_PER_M2NDIV, MSS_TOPRCM_PLL_PER_M2NDIV_PLL_PER_M2NDIV_N);
        M2 = CSL_FEXT(ptrTopRCMRegs->PLL_PER_M2NDIV, MSS_TOPRCM_PLL_PER_M2NDIV_PLL_PER_M2NDIV_M2);
        M  = CSL_FEXT(ptrTopRCMRegs->PLL_PER_MN2DIV, MSS_TOPRCM_PLL_PER_MN2DIV_PLL_PER_MN2DIV_M);
        FracM = CSL_FEXT(ptrTopRCMRegs->PLL_PER_FRACDIV,MSS_TOPRCM_PLL_PER_FRACDIV_PLL_PER_FRACDIV_FRACTIONALM);
        FOut = SOC_rcmGetADPLLJFout(Finp, N, M, M2, FracM, div2flag);
        DebugP_assert(FOut != 0);
    }
    else
    {
        uint32_t ULOWCLKEN = CSL_FEXT(ptrTopRCMRegs->PLL_PER_CLKCTRL, MSS_TOPRCM_PLL_PER_CLKCTRL_PLL_PER_CLKCTRL_ULOWCLKEN);
        if (ULOWCLKEN == 0)
        {
            uint32_t N2;

            N2  = CSL_FEXT(ptrTopRCMRegs->PLL_PER_MN2DIV, MSS_TOPRCM_PLL_PER_MN2DIV_PLL_PER_MN2DIV_N2);
            FOut = Finp/(N2 + 1);
        }
        else
        {
            FOut = Finp;
        }
    }
    return FOut;
}

static uint32_t SOC_rcmGetPerHsDivOut(uint32_t Finp, uint32_t div2flag, SOC_RcmPllHsDivOutId hsDivOut)
{
    uint32_t FOut;
    uint32_t clkDiv;
    CSL_mss_toprcmRegs *ptrTopRCMRegs;

    ptrTopRCMRegs = SOC_rcmGetBaseAddressTOPRCM ();

    FOut = SOC_rcmGetPerFout(Finp, div2flag);
    switch(hsDivOut)
    {
        case SOC_RcmPllHsDivOutId_0:
        {
            clkDiv = CSL_FEXT(ptrTopRCMRegs->PLL_PER_HSDIVIDER_CLKOUT0, MSS_TOPRCM_PLL_PER_HSDIVIDER_CLKOUT0_PLL_PER_HSDIVIDER_CLKOUT0_DIV);
            break;
        }
        case SOC_RcmPllHsDivOutId_1:
        {
            clkDiv = CSL_FEXT(ptrTopRCMRegs->PLL_PER_HSDIVIDER_CLKOUT1, MSS_TOPRCM_PLL_PER_HSDIVIDER_CLKOUT1_PLL_PER_HSDIVIDER_CLKOUT1_DIV);
            break;
        }
        case SOC_RcmPllHsDivOutId_2:
        {
            clkDiv = CSL_FEXT(ptrTopRCMRegs->PLL_PER_HSDIVIDER_CLKOUT2, MSS_TOPRCM_PLL_PER_HSDIVIDER_CLKOUT2_PLL_PER_HSDIVIDER_CLKOUT2_DIV);
            break;
        }
        case SOC_RcmPllHsDivOutId_3:
        {
            clkDiv = CSL_FEXT(ptrTopRCMRegs->PLL_PER_HSDIVIDER_CLKOUT3, MSS_TOPRCM_PLL_PER_HSDIVIDER_CLKOUT3_PLL_PER_HSDIVIDER_CLKOUT3_DIV);
            break;
        }
        default:
        {
            DebugP_assert(0);
            clkDiv = 0;
            break;
        }
    }

    return (FOut/(clkDiv  + 1));
}

static void SOC_rcmProgPllCoreDivider (uint8_t inputClockDiv , uint8_t divider,
                                uint16_t multiplier, uint8_t postDivider,
                                uint32_t fracMultiplier)
{
    volatile uint32_t *ptrM2NReg, *ptrMN2Reg, *ptrFracMReg;
    CSL_mss_toprcmRegs *ptrTopRCMRegs;

    ptrTopRCMRegs = SOC_rcmGetBaseAddressTOPRCM ();

    ptrM2NReg   = &(ptrTopRCMRegs->PLL_CORE_M2NDIV);
    ptrMN2Reg   = &(ptrTopRCMRegs->PLL_CORE_MN2DIV);
    ptrFracMReg = &(ptrTopRCMRegs->PLL_CORE_FRACDIV);

    /* Initialization sequence referred from ADPLLLJ_GS70_v0.8-02 */
    /* program M2 (post divider) */
    // GEL file equivalent
    // APPLJ-1 Setting
    // CLOCKOUT = M/(N+1) * CLKINP * (1/M2)  =  0x7d0/(39+1) * 40 * (1/1) = 2G
    // GEL file equivalent
    // HW_WR_REG32(CSL_MSS_TOPRCM_U_BASE+PLL_CORE_M2NDIV     , 0x10027);      //M2NDIV_M2[22:16] = 1 , M2NDIV_N[7:0] = 0x27
    // HW_WR_REG32(CSL_MSS_TOPRCM_U_BASE+PLL_CORE_MN2DIV     , 0x107d0);      //MN2DIV_N2[19:16] = 1 , MN2DIV_M[11:0] = 0x7d0

    *ptrM2NReg = SOC_rcmInsert8 (*ptrM2NReg, 22U, 16U, postDivider);

    /* program N (input clock divider) */
    *ptrM2NReg = SOC_rcmInsert8 (*ptrM2NReg, 7U, 0U, inputClockDiv);

    /* program M (multiplier) */
    *ptrMN2Reg = SOC_rcmInsert16 (*ptrMN2Reg, 11U, 0U, multiplier);

    /* program N2 (divider) */
    *ptrMN2Reg = SOC_rcmInsert8 (*ptrMN2Reg, 19U, 16U, divider);

    /* program Fractional Multiplier */
    *ptrFracMReg = SOC_rcmInsert32 (*ptrFracMReg, 17U, 0U, fracMultiplier);
}

static void SOC_rcmProgPllDspDivider (uint8_t inputClockDiv , uint8_t divider,
                               uint16_t multiplier, uint8_t postDivider,
                               uint32_t fracMultiplier)
{
    volatile uint32_t *ptrM2NReg, *ptrMN2Reg, *ptrFracMReg;
    CSL_mss_toprcmRegs *ptrTopRCMRegs;

    ptrTopRCMRegs = SOC_rcmGetBaseAddressTOPRCM ();

    ptrM2NReg   = &(ptrTopRCMRegs->PLL_DSP_M2NDIV);
    ptrMN2Reg   = &(ptrTopRCMRegs->PLL_DSP_MN2DIV);
    ptrFracMReg = &(ptrTopRCMRegs->PLL_DSP_FRACDIV);

    /* Initialization sequence referred from ADPLLLJ_GS70_v0.8-02 */
    /* program M2 (post divider) */
    // GEL file equivalent
    // APPLJ-2 Setting
    // CLOCKOUT = M/(N+1) * CLKINP * (1/M2)  =  0x708/(39+1) * 40 * (1/1) = 1.8G
    // HW_WR_REG32(CSL_MSS_TOPRCM_U_BASE+PLL_DSP_M2NDIV     , 0x10027);      //M2NDIV_M2[22:16] = 1 , M2NDIV_N[7:0] = 0x27
    // HW_WR_REG32(CSL_MSS_TOPRCM_U_BASE+PLL_DSP_MN2DIV     , 0x10708);      //MN2DIV_N2[19:16] = 1 , MN2DIV_M[11:0] = 0x708
    *ptrM2NReg = SOC_rcmInsert8 (*ptrM2NReg, 22U, 16U, postDivider);

    /* program N (input clock divider) */
    *ptrM2NReg = SOC_rcmInsert8 (*ptrM2NReg, 7U, 0U, inputClockDiv);

    /* program M (multiplier) */
    *ptrMN2Reg = SOC_rcmInsert16 (*ptrMN2Reg, 11U, 0U, multiplier);

    /* program N2 (divider) */
    *ptrMN2Reg = SOC_rcmInsert8 (*ptrMN2Reg, 19U, 16U, divider);

    /* program Fractional Multiplier */
    *ptrFracMReg = SOC_rcmInsert32 (*ptrFracMReg, 17U, 0U, fracMultiplier);

    if(multiplier == 1650)
    {
        *ptrFracMReg = SOC_rcmInsert32 (*ptrFracMReg, 31U, 24U, 7U);
    }
}

static void SOC_rcmProgPllPerDivider (uint8_t inputClockDiv , uint8_t divider,
                               uint16_t multiplier, uint8_t postDivider,
                               uint32_t fracMultiplier)
{
    volatile uint32_t *ptrM2NReg, *ptrMN2Reg, *ptrFracMReg;
    CSL_mss_toprcmRegs *ptrTopRCMRegs;

    ptrTopRCMRegs = SOC_rcmGetBaseAddressTOPRCM ();

    ptrM2NReg   = &(ptrTopRCMRegs->PLL_PER_M2NDIV);
    ptrMN2Reg   = &(ptrTopRCMRegs->PLL_PER_MN2DIV);
    ptrFracMReg = &(ptrTopRCMRegs->PLL_PER_FRACDIV);

    /* Initialization sequence referred from ADPLLLJ_GS70_v0.8-02 */
    /* program M2 (post divider) */
    // GEL file equivalent
    // APPLJ-3 Setting
    // CLOCKOUT = M/(N+1) * CLKINP * (1/M2)  =  0x6C0/(39+1) * 40 * (1/1) = 1.728G
    // HW_WR_REG32(CSL_MSS_TOPRCM_U_BASE+PLL_PER_M2NDIV     , 0x10027);      //M2NDIV_M2[22:16] = 1 , M2NDIV_N[7:0] = 0x27
    // HW_WR_REG32(CSL_MSS_TOPRCM_U_BASE+PLL_PER_MN2DIV     , 0x106C0);      //MN2DIV_N2[19:16] = 1 , MN2DIV_M[11:0] = 0x6C0
    *ptrM2NReg = SOC_rcmInsert8 (*ptrM2NReg, 22U, 16U, postDivider);

    /* program N (input clock divider) */
    *ptrM2NReg = SOC_rcmInsert8 (*ptrM2NReg, 7U, 0U, inputClockDiv);

    /* program M (multiplier) */
    *ptrMN2Reg = SOC_rcmInsert16 (*ptrMN2Reg, 11U, 0U, multiplier);

    /* program N2 (divider) */
    *ptrMN2Reg = SOC_rcmInsert8 (*ptrMN2Reg, 19U, 16U, divider);

    /* program Fractional Multiplier */
    *ptrFracMReg = SOC_rcmInsert32 (*ptrFracMReg, 17U, 0U, fracMultiplier);
}


static void SOC_rcmConfigurePllCore (uint16_t trimVal)
{
    volatile uint32_t *ptrClkCtrl, *ptrTenable, *ptrTenableDiv, *ptrPllStatus;
    CSL_mss_toprcmRegs *ptrTopRCMRegs;
    uint8_t phaseLockStatus;

    /* Core PLL settings */
    ptrTopRCMRegs = SOC_rcmGetBaseAddressTOPRCM ();

    ptrClkCtrl    = &(ptrTopRCMRegs->PLL_CORE_CLKCTRL);
    ptrTenable    = &(ptrTopRCMRegs->PLL_CORE_TENABLE);
    ptrTenableDiv = &(ptrTopRCMRegs->PLL_CORE_TENABLEDIV);
    ptrPllStatus  = &(ptrTopRCMRegs->PLL_CORE_STATUS);

    /* update the Clock control setting */
    // APPLJ-1 Setting
    // CLOCKOUT = M/(N+1) * CLKINP * (1/M2)  =  0x7d0/(39+1) * 40 * (1/1) = 2G
    // GEL file equivalent
    // HW_WR_REG32(CSL_MSS_TOPRCM_U_BASE+PLL_CORE_M2NDIV     , 0x10027);      //M2NDIV_M2[22:16] = 1 , M2NDIV_N[7:0] = 0x27
    // HW_WR_REG32(CSL_MSS_TOPRCM_U_BASE+PLL_CORE_MN2DIV     , 0x107d0);      //MN2DIV_N2[19:16] = 1 , MN2DIV_M[11:0] = 0x7d0
    // HW_WR_REG32(CSL_MSS_TOPRCM_U_BASE+PLL_CORE_CLKCTRL    , 0x29021000);   //CLKDCOLDOEN[29] = 1,NWELLTRIM[28:24] = 9 IDLE[23] = 0 CLKDCOLDOPWDNZ[17] = 1 SELFREQDCO[12:10] = 4

    // HW_WR_REG32(CSL_MSS_TOPRCM_U_BASE+PLL_CORE_TENABLE    , 0x1);          // TENABLE    = 1
    // HW_WR_REG32(CSL_MSS_TOPRCM_U_BASE+PLL_CORE_CLKCTRL    , 0x29021001);   //+TINTZ[0]   = 1
    // HW_WR_REG32(CSL_MSS_TOPRCM_U_BASE+PLL_CORE_TENABLE    , 0x0);          // TENABLE    = 0
    // HW_WR_REG32(CSL_MSS_TOPRCM_U_BASE+PLL_CORE_TENABLEDIV , 0x1);          // TENABLEDIV = 1
    // HW_WR_REG32(CSL_MSS_TOPRCM_U_BASE+PLL_CORE_TENABLEDIV , 0x0);          // TENABLEDIV = 0

    /* program CLKDCOLDOEN[29] = 1, IDLE[23] = 0, CLKDCOLDOPWDNZ[17] = 1, SELFREQDCO[12:10] = 4 */
    *ptrClkCtrl = SOC_rcmInsert8 (*ptrClkCtrl, 29U, 29U, 0x1U);
    *ptrClkCtrl = SOC_rcmInsert8 (*ptrClkCtrl, 23U, 23U, 0x0U);
    *ptrClkCtrl = SOC_rcmInsert8 (*ptrClkCtrl, 17U, 17U, 0x1U);
    *ptrClkCtrl = SOC_rcmInsert8 (*ptrClkCtrl, 12U, 10U, 0x4U);

    /* Write Nwell Trim Value */
    *ptrClkCtrl = SOC_rcmInsert8 (*ptrClkCtrl, 28U, 24U, ((uint8_t) trimVal & 0x1FU));

    /* Soft reset Pll */
    *ptrClkCtrl = SOC_rcmInsert8 (*ptrClkCtrl, 0U, 0U, 0x0U);

    /* TENABLE High */
    *ptrTenable = SOC_rcmInsert8 (*ptrTenable, 0U, 0U, 0x1U);

    /* out of reset Pll */
    *ptrClkCtrl = SOC_rcmInsert8 (*ptrClkCtrl, 0U, 0U, 0x1U);

    /* TENABLE Low */
    *ptrTenable = SOC_rcmInsert8 (*ptrTenable, 0U, 0U, 0x0U);

    /* TENABLEDIV High */
    *ptrTenableDiv = SOC_rcmInsert8 (*ptrTenableDiv, 0U, 0U, 0x1U);

    /* TENABLEDIV Low */
    *ptrTenableDiv = SOC_rcmInsert8 (*ptrTenableDiv, 0U, 0U, 0x0U);

    /* wait for the Phase lock for Core/DSP PLL */
    // GEL File equivalent
    // APPLJ-1  :  loop check to PLLLOCK DONE
    // lock_status = HW_RD_REG32(CSL_MSS_TOPRCM_U_BASE+PLL_CORE_STATUS); //PHASELOCK[10]
    // while(0x400 != (lock_status & 0x400)) {
    //   lock_status = HW_RD_REG32(CSL_MSS_TOPRCM_U_BASE+PLL_CORE_STATUS); //PHASELOCK[10]
    // }

    do
    {
        phaseLockStatus = SOC_rcmExtract8 (*ptrPllStatus, 10U, 10U);
    }while(phaseLockStatus != 1U);
}

static void SOC_rcmConfigurePllDsp (uint16_t trimVal)
{
    volatile uint32_t *ptrClkCtrl, *ptrTenable, *ptrTenableDiv, *ptrPllStatus;
    CSL_mss_toprcmRegs *ptrTopRCMRegs;
    uint8_t phaseLockStatus;

    /* Core PLL settings */
    ptrTopRCMRegs = SOC_rcmGetBaseAddressTOPRCM ();

    ptrClkCtrl    = &(ptrTopRCMRegs->PLL_DSP_CLKCTRL);
    ptrTenable    = &(ptrTopRCMRegs->PLL_DSP_TENABLE);
    ptrTenableDiv = &(ptrTopRCMRegs->PLL_DSP_TENABLEDIV);
    ptrPllStatus  = &(ptrTopRCMRegs->PLL_DSP_STATUS);

    /* update the Clock control setting */
    /* program CLKDCOLDOEN[29] = 1, IDLE[23] = 0, CLKDCOLDOPWDNZ[17] = 1, SELFREQDCO[12:10] = 4 */
    /* GEL file equivalent */
    // HW_WR_REG32(CSL_MSS_TOPRCM_U_BASE+PLL_DSP_CLKCTRL    , 0x29021000);   //CLKDCOLDOEN[29] = 1,NWELLTRIM[28:24] = 9 IDLE[23] = 0 CLKDCOLDOPWDNZ[17] = 1 SELFREQDCO[12:10] = 4

    // HW_WR_REG32(CSL_MSS_TOPRCM_U_BASE+PLL_DSP_TENABLE    , 0x1);          // TENABLE    = 1
    // HW_WR_REG32(CSL_MSS_TOPRCM_U_BASE+PLL_DSP_CLKCTRL    , 0x29021001);   //+TINTZ[0]   = 1
    // HW_WR_REG32(CSL_MSS_TOPRCM_U_BASE+PLL_DSP_TENABLE    , 0x0);          // TENABLE    = 0
    // HW_WR_REG32(CSL_MSS_TOPRCM_U_BASE+PLL_DSP_TENABLEDIV , 0x1);          // TENABLEDIV = 1
    // HW_WR_REG32(CSL_MSS_TOPRCM_U_BASE+PLL_DSP_TENABLEDIV , 0x0);          // TENABLEDIV = 0

    *ptrClkCtrl = SOC_rcmInsert8 (*ptrClkCtrl, 29U, 29U, 0x1U);
    *ptrClkCtrl = SOC_rcmInsert8 (*ptrClkCtrl, 23U, 23U, 0x0U);
    *ptrClkCtrl = SOC_rcmInsert8 (*ptrClkCtrl, 17U, 17U, 0x1U);
    *ptrClkCtrl = SOC_rcmInsert8 (*ptrClkCtrl, 12U, 10U, 0x4U);

    /* Write Nwell Trim Value */
    *ptrClkCtrl = SOC_rcmInsert8 (*ptrClkCtrl, 28U, 24U, ((uint8_t) trimVal & 0x1FU));

    /* Soft reset Pll */
    *ptrClkCtrl = SOC_rcmInsert8 (*ptrClkCtrl, 0U, 0U, 0x0U);

    /* TENABLE High */
    *ptrTenable = SOC_rcmInsert8 (*ptrTenable, 0U, 0U, 0x1U);

    /* out of reset Pll */
    *ptrClkCtrl = SOC_rcmInsert8 (*ptrClkCtrl, 0U, 0U, 0x1U);

    /* TENABLE Low */
    *ptrTenable = SOC_rcmInsert8 (*ptrTenable, 0U, 0U, 0x0U);

    /* TENABLEDIV High */
    *ptrTenableDiv = SOC_rcmInsert8 (*ptrTenableDiv, 0U, 0U, 0x1U);

    /* TENABLEDIV Low */
    *ptrTenableDiv = SOC_rcmInsert8 (*ptrTenableDiv, 0U, 0U, 0x0U);

    /* wait for the Phase lock for Core/DSP PLL */
    // GEL File equivalent
    // APPLJ-2 : loop check to PLLLOCK DONE
    // lock_status = HW_RD_REG32(CSL_MSS_TOPRCM_U_BASE+PLL_DSP_STATUS); //PHASELOCK[10]
    // while(0x400 != (lock_status & 0x400)) {
    //   lock_status = HW_RD_REG32(CSL_MSS_TOPRCM_U_BASE+PLL_DSP_STATUS); //PHASELOCK[10]
    // }

    do
    {
        phaseLockStatus = SOC_rcmExtract8 (*ptrPllStatus, 10U, 10U);
    }while(phaseLockStatus != 1U);
}

static void SOC_rcmConfigurePllPer (uint16_t trimVal)
{
    volatile uint32_t *ptrClkCtrl, *ptrTenable, *ptrTenableDiv, *ptrPllStatus;
    CSL_mss_toprcmRegs *ptrTopRCMRegs;
    uint8_t phaseLockStatus;

    /* Core PLL settings */
    ptrTopRCMRegs = SOC_rcmGetBaseAddressTOPRCM ();

    ptrClkCtrl    = &(ptrTopRCMRegs->PLL_PER_CLKCTRL);
    ptrTenable    = &(ptrTopRCMRegs->PLL_PER_TENABLE);
    ptrTenableDiv = &(ptrTopRCMRegs->PLL_PER_TENABLEDIV);
    ptrPllStatus  = &(ptrTopRCMRegs->PLL_PER_STATUS);

    /* update the Clock control setting */
    /* program CLKDCOLDOEN[29] = 1, IDLE[23] = 0, CLKDCOLDOPWDNZ[17] = 1, SELFREQDCO[12:10] = 4 */
    /* GEL file equivalent */
    // HW_WR_REG32(CSL_MSS_TOPRCM_U_BASE+PLL_PER_CLKCTRL    , 0x29021000);   //CLKDCOLDOEN[29] = 1,NWELLTRIM[28:24] = 9 IDLE[23] = 0 CLKDCOLDOPWDNZ[17] = 1 SELFREQDCO[12:10] = 4

    // HW_WR_REG32(CSL_MSS_TOPRCM_U_BASE+PLL_PER_TENABLE    , 0x1);          // TENABLE    = 1
    // HW_WR_REG32(CSL_MSS_TOPRCM_U_BASE+PLL_PER_CLKCTRL    , 0x29021001);   //+TINTZ[0]   = 1
    // HW_WR_REG32(CSL_MSS_TOPRCM_U_BASE+PLL_PER_TENABLE    , 0x0);          // TENABLE    = 0
    // HW_WR_REG32(CSL_MSS_TOPRCM_U_BASE+PLL_PER_TENABLEDIV , 0x1);          // TENABLEDIV = 1
    // HW_WR_REG32(CSL_MSS_TOPRCM_U_BASE+PLL_PER_TENABLEDIV , 0x0);          // TENABLEDIV = 0

    *ptrClkCtrl = SOC_rcmInsert8 (*ptrClkCtrl, 29U, 29U, 0x1U);
    *ptrClkCtrl = SOC_rcmInsert8 (*ptrClkCtrl, 23U, 23U, 0x0U);
    *ptrClkCtrl = SOC_rcmInsert8 (*ptrClkCtrl, 17U, 17U, 0x1U);
    *ptrClkCtrl = SOC_rcmInsert8 (*ptrClkCtrl, 12U, 10U, 0x4U);

    /* Write Nwell Trim Value */
    *ptrClkCtrl = SOC_rcmInsert8 (*ptrClkCtrl, 28U, 24U, ((uint8_t) trimVal & 0x1FU));

    /* Soft reset Pll */
    *ptrClkCtrl = SOC_rcmInsert8 (*ptrClkCtrl, 0U, 0U, 0x0U);

    /* TENABLE High */
    *ptrTenable = SOC_rcmInsert8 (*ptrTenable, 0U, 0U, 0x1U);

    /* out of reset Pll */
    *ptrClkCtrl = SOC_rcmInsert8 (*ptrClkCtrl, 0U, 0U, 0x1U);

    /* TENABLE Low */
    *ptrTenable = SOC_rcmInsert8 (*ptrTenable, 0U, 0U, 0x0U);

    /* TENABLEDIV High */
    *ptrTenableDiv = SOC_rcmInsert8 (*ptrTenableDiv, 0U, 0U, 0x1U);

    /* TENABLEDIV Low */
    *ptrTenableDiv = SOC_rcmInsert8 (*ptrTenableDiv, 0U, 0U, 0x0U);

    /* wait for the Phase lock for Core/DSP PLL */
    // GEL File equivalent
    // APPLJ-3 : loop check to PLLLOCK DONE
    // lock_status = HW_RD_REG32(CSL_MSS_TOPRCM_U_BASE+PLL_PER_STATUS); //PHASELOCK[10]
    // while(0x400 != (lock_status & 0x400)) {
    //   lock_status = HW_RD_REG32(CSL_MSS_TOPRCM_U_BASE+PLL_PER_STATUS); //PHASELOCK[10]
    // }

    do
    {
        phaseLockStatus = SOC_rcmExtract8 (*ptrPllStatus, 10U, 10U);
    }while(phaseLockStatus != 1U);
}

static SOC_RcmXtalFreqId SOC_rcmGetXtalFrequency(void)
{
    CSL_mss_toprcmRegs *ptrTopRCMRegs;
    CSL_top_ctrlRegs* ptrTopCtrlRegs;
    uint32_t xtalRegVal;
    SOC_RcmXtalFreqId freq;
    uint8_t xtalFreqScale;

    ptrTopRCMRegs = SOC_rcmGetBaseAddressTOPRCM ();
    ptrTopCtrlRegs = SOC_rcmGetBaseAddressTOPCTRL ();

    /* read the register bits corresponding to XTAL Frequency */
    xtalRegVal = SOC_rcmExtract32 (ptrTopRCMRegs->ANA_REG_WU_MODE_REG_LOWV, 6U, 5U);
    /* read the register bits corresponding to XTAL Frequency Scale */
    xtalFreqScale = SOC_rcmReadXtalFreqScale (ptrTopCtrlRegs);

    if(xtalFreqScale == 1U)
    {
        if (xtalRegVal == 0U)
        {
            freq = SOC_RcmXtalFreqId_CLK_20MHZ;
        }
        else if (xtalRegVal == 1U)
        {
            freq = SOC_RcmXtalFreqId_CLK_22p5792MHZ;
        }
        else if (xtalRegVal == 2U)
        {
            freq = SOC_RcmXtalFreqId_CLK_24p576MHZ;
        }
        else
        {
            freq = SOC_RcmXtalFreqId_CLK_25MHZ;
        }
    }
    else
    {
        if (xtalRegVal == 0U)
        {
            freq = SOC_RcmXtalFreqId_CLK_40MHZ;
        }
        else if (xtalRegVal == 1U)
        {
            freq = SOC_RcmXtalFreqId_CLK_45p1584MHZ;
        }
        else if (xtalRegVal == 2U)
        {
            freq = SOC_RcmXtalFreqId_CLK_49p152MHZ;
        }
        else
        {
            freq = SOC_RcmXtalFreqId_CLK_50MHZ;
        }
    }

    return (freq);

}

uint32_t SOC_rcmIsR5FInLockStepMode(uint32_t r5fClusterGroupId)
{
    uint32_t retVal = FALSE;
    CSL_mss_ctrlRegs *mssCtrl = SOC_rcmGetBaseAddressMSSCTRL();

    if (r5fClusterGroupId == CSL_ARM_R5_CLUSTER_GROUP_ID_0)
    {
		if (CSL_FEXT(mssCtrl->R5_STATUS_REG, MSS_CTRL_R5_STATUS_REG_R5_STATUS_REG_LOCK_STEP) == 1U)
        {
            /* Lockstep Mode */
            retVal = TRUE;
        }
        else
        {
            /* Dualcore Mode */
            retVal = FALSE;
        }
    }

    return retVal;
}

/*
 * This API Sets up the core PLL configuration switches the R5 to PLL clock.
 */
void SOC_rcmCoreApllConfig(SOC_RcmPllFoutFreqId outFreqId, SOC_RcmPllHsDivOutConfig *hsDivCfg)
{
    SOC_RcmXtalFreqId XTALFreq;
    uint16_t coreTrimVal;
    SOC_RcmADPLLJConfig const * adplljCfg;

    /* read the Core ADPLL trim value */
    coreTrimVal = SOC_rcmGetCoreTrimVal ();

    /* read the XTAL Frequency */
    XTALFreq = SOC_rcmGetXtalFrequency ();

    /* program PLL dividers and multipliers. The value are taken from  TPR12_ADPLLJ_Settings_1p0.xlsx */
    adplljCfg = SOC_rcmGetADPLLJConfig(gSocRcmXtalInfo[XTALFreq].Finp, outFreqId);
    DebugP_assert(adplljCfg != NULL);

    if (adplljCfg != NULL)
    {
        if (gSocRcmXtalInfo[XTALFreq].div2flag == 0)
        {
            SOC_rcmProgPllCoreDivider (adplljCfg->N,
                                0U /* N2 divider for bypass */,
                                adplljCfg->M,
                                adplljCfg->M2,
                                adplljCfg->FracM);
        }
        else
        {
            uint32_t N;

            DebugP_assert(((adplljCfg->N + 1) % 2) == 0);
            /* Input XTAL freq is half. Divide input divider by 2 to get same output freq */
            N = ((adplljCfg->N + 1) / 2) - 1;
            SOC_rcmProgPllCoreDivider (N,
                                0U /* N2 divider for bypass */,
                                adplljCfg->M,
                                adplljCfg->M2,
                                adplljCfg->FracM);
        }
        /* Configure and Lock Core PLL */
        SOC_rcmConfigurePllCore (coreTrimVal);

        SOC_rcmCoreApllHSDivConfig(hsDivCfg);
    }
}

void SOC_rcmCoreApllHSDivConfig(SOC_RcmPllHsDivOutConfig *hsDivCfg)
{
    CSL_mss_toprcmRegs *ptrTopRCMRegs;
    uint32_t hsDivOutRegVal;
    uint32_t Fout;
    uint32_t Finp;
    SOC_RcmXtalFreqId clkFreqId;
    volatile uint32_t *ptrFracMReg;
    volatile uint32_t *ptrCoreClkCtrlReg;

    /* Core PLL settings */
    ptrTopRCMRegs = SOC_rcmGetBaseAddressTOPRCM ();
    clkFreqId = SOC_rcmGetXtalFrequency ();

    Finp = gSocRcmXtalInfo[clkFreqId].Finp;
    Fout = SOC_rcmGetCoreFout(Finp, gSocRcmXtalInfo[clkFreqId].div2flag);

    /* Derive Clocks */
    /* TPR12_Ch08_Clock_Arch_0p91 is used as reference for below settings */
    /* Set clock divider values from Core PLL*/
    /* 400Mhz */
    if (hsDivCfg->hsdivOutEnMask & SOC_RCM_PLL_HSDIV_OUTPUT_ENABLE_0)
    {
        DebugP_assert((Fout % hsDivCfg->hsDivOutFreqHz[0]) == 0);
        hsDivOutRegVal = Fout / hsDivCfg->hsDivOutFreqHz[0];
        hsDivOutRegVal--;
        ptrTopRCMRegs->PLL_CORE_HSDIVIDER_CLKOUT0 = SOC_rcmInsert8 (ptrTopRCMRegs->PLL_CORE_HSDIVIDER_CLKOUT0, 4U, 0U, hsDivOutRegVal);
    }
    if (hsDivCfg->hsdivOutEnMask & SOC_RCM_PLL_HSDIV_OUTPUT_ENABLE_1)
    {
        DebugP_assert((Fout % hsDivCfg->hsDivOutFreqHz[1]) == 0);
        hsDivOutRegVal = Fout / hsDivCfg->hsDivOutFreqHz[1];
        hsDivOutRegVal--;
        ptrTopRCMRegs->PLL_CORE_HSDIVIDER_CLKOUT1 = SOC_rcmInsert8 (ptrTopRCMRegs->PLL_CORE_HSDIVIDER_CLKOUT1, 4U, 0U, hsDivOutRegVal);
    }
    if (hsDivCfg->hsdivOutEnMask & SOC_RCM_PLL_HSDIV_OUTPUT_ENABLE_2)
    {
        DebugP_assert((Fout % hsDivCfg->hsDivOutFreqHz[2]) == 0);
        hsDivOutRegVal = Fout / hsDivCfg->hsDivOutFreqHz[2];
        hsDivOutRegVal--;
        ptrTopRCMRegs->PLL_CORE_HSDIVIDER_CLKOUT2 = SOC_rcmInsert8 (ptrTopRCMRegs->PLL_CORE_HSDIVIDER_CLKOUT2, 4U, 0U, hsDivOutRegVal);
    }
    /* Core PLL output 3 not used.WIll not configure */
    DebugP_assert((hsDivCfg->hsdivOutEnMask & SOC_RCM_PLL_HSDIV_OUTPUT_ENABLE_3) == 0);
    /* Generate Trigger to latch these values */
    ptrTopRCMRegs->PLL_CORE_HSDIVIDER         = SOC_rcmInsert8 (ptrTopRCMRegs->PLL_CORE_HSDIVIDER, 2U, 2U, 0x1U);
    ptrTopRCMRegs->PLL_CORE_HSDIVIDER         = SOC_rcmInsert8 (ptrTopRCMRegs->PLL_CORE_HSDIVIDER, 2U, 2U, 0x0U);

    /* Ungate the clocks */
    if (hsDivCfg->hsdivOutEnMask & SOC_RCM_PLL_HSDIV_OUTPUT_ENABLE_0)
    {
        ptrTopRCMRegs->PLL_CORE_HSDIVIDER_CLKOUT0 = SOC_rcmInsert8 (ptrTopRCMRegs->PLL_CORE_HSDIVIDER_CLKOUT0, 8U, 8U, 0x1U);
    }
    if (hsDivCfg->hsdivOutEnMask & SOC_RCM_PLL_HSDIV_OUTPUT_ENABLE_1)
    {
        ptrTopRCMRegs->PLL_CORE_HSDIVIDER_CLKOUT1 = SOC_rcmInsert8 (ptrTopRCMRegs->PLL_CORE_HSDIVIDER_CLKOUT1, 8U, 8U, 0x1U);
    }
    if (hsDivCfg->hsdivOutEnMask & SOC_RCM_PLL_HSDIV_OUTPUT_ENABLE_2)
    {
        ptrTopRCMRegs->PLL_CORE_HSDIVIDER_CLKOUT2 = SOC_rcmInsert8 (ptrTopRCMRegs->PLL_CORE_HSDIVIDER_CLKOUT2, 8U, 8U, 0x1U);
    }
    /* If the PLL lock frequency is less than 1GHz, update the sigma delta divider and DCO frequency. Errata: i2389 */
    if(Fout < SOC_RCM_FREQ_1GHZ)
    {
        ptrFracMReg = &(ptrTopRCMRegs->PLL_CORE_FRACDIV);
        /* PLL_CORE_FRACDIV_REGSD_SHIFT, PLL_CORE_FRACDIV_REGSD_MASK */
        *ptrFracMReg = SOC_rcmInsert8 (*ptrFracMReg, 31U, 24U, 0x4);

        ptrCoreClkCtrlReg = &(ptrTopRCMRegs->PLL_CORE_CLKCTRL);
        /* PLL_CORE_CLKCTRL_SELFREQDCO_SHIFT, PLL_CORE_CLKCTRL_SELFREQDCO_MASK */
        *ptrCoreClkCtrlReg = SOC_rcmInsert8 (*ptrCoreClkCtrlReg, 12U, 10U, 0x2);
    }

    return;
}

void SOC_rcmDspPllConfig(SOC_RcmPllFoutFreqId outFreqId, SOC_RcmPllHsDivOutConfig *hsDivCfg)
{
    SOC_RcmXtalFreqId XTALFreq;
    uint16_t dspTrimVal;
    CSL_mss_toprcmRegs *ptrTopRCMRegs;
    SOC_RcmADPLLJConfig const * adplljCfg;
    uint32_t hsDivOutRegVal;
    uint32_t Fout;

    Fout = SOC_RCM_FREQ_MHZ2HZ(gSocRcmPllFreqId2FOutMap[outFreqId]);
    /* read the Core ADPLL trim value */
    dspTrimVal = SOC_rcmGetDspTrimVal ();

    /* read the XTAL Frequency */
    XTALFreq = SOC_rcmGetXtalFrequency ();

    /* program PLL dividers and multipliers. The value are taken from  TPR12_ADPLLJ_Settings_1p0.xlsx */
    adplljCfg = SOC_rcmGetADPLLJConfig(gSocRcmXtalInfo[XTALFreq].Finp, outFreqId);
    DebugP_assert(adplljCfg != NULL);

    if (adplljCfg != NULL)
    {
        if (gSocRcmXtalInfo[XTALFreq].div2flag == 0)
        {
            SOC_rcmProgPllDspDivider (adplljCfg->N,
                               0U /* N2 divider for bypass */,
                               adplljCfg->M,
                               adplljCfg->M2,
                               adplljCfg->FracM);
        }
        else
        {
            uint32_t N;

            DebugP_assert(((adplljCfg->N + 1) % 2) == 0);
            /* Input XTAL freq is half. Divide input divider by 2 to get same output freq */
            N = ((adplljCfg->N + 1) / 2) - 1;
            SOC_rcmProgPllDspDivider (N,
                               0U /* N2 divider for bypass */,
                               adplljCfg->M,
                               adplljCfg->M2,
                               adplljCfg->FracM);
        }
        /* Configure and Lock Core PLL */
        SOC_rcmConfigurePllDsp (dspTrimVal);

        /* Derive Clocks */
        // HSDIV-2 Settings
        /* Core PLL settings */
        ptrTopRCMRegs = SOC_rcmGetBaseAddressTOPRCM ();

        if (hsDivCfg->hsdivOutEnMask & SOC_RCM_PLL_HSDIV_OUTPUT_ENABLE_0)
        {
            DebugP_assert((Fout % hsDivCfg->hsDivOutFreqHz[0]) == 0);
            hsDivOutRegVal = Fout / hsDivCfg->hsDivOutFreqHz[0];
            hsDivOutRegVal--;
            //HW_WR_REG32(CSL_MSS_TOPRCM_U_BASE+PLL_DSP_HSDIVIDER_CLKOUT0, 0x0);    // CLKOUT0_DIV[4:0] = 4  -- 900M/(4+1) = 225MHz
            ptrTopRCMRegs->PLL_DSP_HSDIVIDER_CLKOUT0 = SOC_rcmInsert8 (ptrTopRCMRegs->PLL_DSP_HSDIVIDER_CLKOUT0, 4U, 0U, hsDivOutRegVal);
        }
        if (hsDivCfg->hsdivOutEnMask & SOC_RCM_PLL_HSDIV_OUTPUT_ENABLE_1)
        {
            DebugP_assert((Fout % hsDivCfg->hsDivOutFreqHz[1]) == 0);
            hsDivOutRegVal = Fout / hsDivCfg->hsDivOutFreqHz[1];
            hsDivOutRegVal--;
            //HW_WR_REG32(CSL_MSS_TOPRCM_U_BASE+PLL_DSP_HSDIVIDER_CLKOUT1, 0x3);    // CLKOUT1_DIV[4:0] = 1  -- 900M/(1+1) = 450MHz
            ptrTopRCMRegs->PLL_DSP_HSDIVIDER_CLKOUT1 = SOC_rcmInsert8 (ptrTopRCMRegs->PLL_DSP_HSDIVIDER_CLKOUT1, 4U, 0U, hsDivOutRegVal);
        }
        if (hsDivCfg->hsdivOutEnMask & SOC_RCM_PLL_HSDIV_OUTPUT_ENABLE_2)
        {
            DebugP_assert((Fout % hsDivCfg->hsDivOutFreqHz[2]) == 0);
            hsDivOutRegVal = Fout / hsDivCfg->hsDivOutFreqHz[2];
            hsDivOutRegVal--;
            //HW_WR_REG32(CSL_MSS_TOPRCM_U_BASE+PLL_DSP_HSDIVIDER_CLKOUT2, 0x4);    // CLKOUT2_DIV[4:0] = 8  -- 900M/(8+1) = 100MHz
            ptrTopRCMRegs->PLL_DSP_HSDIVIDER_CLKOUT2 = SOC_rcmInsert8 (ptrTopRCMRegs->PLL_DSP_HSDIVIDER_CLKOUT2, 4U, 0U, hsDivOutRegVal);
        }
        if (hsDivCfg->hsdivOutEnMask & SOC_RCM_PLL_HSDIV_OUTPUT_ENABLE_3)
        {
            DebugP_assert((Fout % hsDivCfg->hsDivOutFreqHz[3]) == 0);
            hsDivOutRegVal = Fout / hsDivCfg->hsDivOutFreqHz[3];
            hsDivOutRegVal--;
            //UnsedHW_WR_REG32(CSL_MSS_TOPRCM_U_BASE+PLL_DSP_HSDIVIDER_CLKOUT3, 0x109);  //+CLKOUT3_GATE[8]  = 1
            //Unsed HW_WR_REG32(CSL_MSS_TOPRCM_U_BASE+PLL_DSP_HSDIVIDER_CLKOUT3, 0x9);    // CLKOUT3_DIV[4:0] = 9  -- 900M/(9+1) = 90MHz unused
            ptrTopRCMRegs->PLL_DSP_HSDIVIDER_CLKOUT3 = SOC_rcmInsert8 (ptrTopRCMRegs->PLL_DSP_HSDIVIDER_CLKOUT3, 4U, 0U, hsDivOutRegVal);
        }

        /* Generate Trigger to latch these values */
        //HW_WR_REG32(CSL_MSS_TOPRCM_U_BASE+PLL_DSP_HSDIVIDER , 0x4);    // HSDIVIDER[2]     = 1
        ptrTopRCMRegs->PLL_DSP_HSDIVIDER         = SOC_rcmInsert8 (ptrTopRCMRegs->PLL_DSP_HSDIVIDER, 2U, 2U, 0x1U);
        //HW_WR_REG32(CSL_MSS_TOPRCM_U_BASE+PLL_DSP_HSDIVIDER        , 0x0);    // HSDIVIDER[2]     = 0
        ptrTopRCMRegs->PLL_DSP_HSDIVIDER         = SOC_rcmInsert8 (ptrTopRCMRegs->PLL_DSP_HSDIVIDER, 2U, 2U, 0x0U);

        /* Ungate the clocks */
        if (hsDivCfg->hsdivOutEnMask & SOC_RCM_PLL_HSDIV_OUTPUT_ENABLE_0)
        {
            //HW_WR_REG32(CSL_MSS_TOPRCM_U_BASE+PLL_DSP_HSDIVIDER_CLKOUT0, 0x100);  //+CLKOUT0_GATE[8]  = 1
            ptrTopRCMRegs->PLL_DSP_HSDIVIDER_CLKOUT0 = SOC_rcmInsert8 (ptrTopRCMRegs->PLL_DSP_HSDIVIDER_CLKOUT0, 8U, 8U, 0x1U);
        }
        if (hsDivCfg->hsdivOutEnMask & SOC_RCM_PLL_HSDIV_OUTPUT_ENABLE_1)
        {
            //HW_WR_REG32(CSL_MSS_TOPRCM_U_BASE+PLL_DSP_HSDIVIDER_CLKOUT1, 0x103);  //+CLKOUT1_GATE[8]  = 1
            ptrTopRCMRegs->PLL_DSP_HSDIVIDER_CLKOUT1 = SOC_rcmInsert8 (ptrTopRCMRegs->PLL_DSP_HSDIVIDER_CLKOUT1, 8U, 8U, 0x1U);
        }
        if (hsDivCfg->hsdivOutEnMask & SOC_RCM_PLL_HSDIV_OUTPUT_ENABLE_2)
        {
            //HW_WR_REG32(CSL_MSS_TOPRCM_U_BASE+PLL_DSP_HSDIVIDER_CLKOUT2, 0x104);  //+CLKOUT2_GATE[8]  = 1
            ptrTopRCMRegs->PLL_DSP_HSDIVIDER_CLKOUT2 = SOC_rcmInsert8 (ptrTopRCMRegs->PLL_DSP_HSDIVIDER_CLKOUT2, 8U, 8U, 0x1U);
        }
        if (hsDivCfg->hsdivOutEnMask & SOC_RCM_PLL_HSDIV_OUTPUT_ENABLE_3)
        {
            //HW_WR_REG32(CSL_MSS_TOPRCM_U_BASE+PLL_DSP_HSDIVIDER_CLKOUT2, 0x104);  //+CLKOUT2_GATE[8]  = 1
            ptrTopRCMRegs->PLL_DSP_HSDIVIDER_CLKOUT2 = SOC_rcmInsert8 (ptrTopRCMRegs->PLL_DSP_HSDIVIDER_CLKOUT3, 8U, 8U, 0x1U);
        }
    }

    return;
}

void SOC_rcmPerPllConfig(SOC_RcmPllFoutFreqId outFreqId, SOC_RcmPllHsDivOutConfig *hsDivCfg)
{
    SOC_RcmXtalFreqId XTALFreq;
    uint16_t perTrimVal;
    CSL_mss_toprcmRegs *ptrTopRCMRegs;
    SOC_RcmADPLLJConfig const * adplljCfg;
    uint32_t hsDivOutRegVal;
    uint32_t Fout;

    Fout = SOC_RCM_FREQ_MHZ2HZ(gSocRcmPllFreqId2FOutMap[outFreqId]);
    /* read the Core ADPLL trim value */
    perTrimVal = SOC_rcmGetPerTrimVal ();
    /* Core PLL settings */
    ptrTopRCMRegs = SOC_rcmGetBaseAddressTOPRCM ();

    /* read the XTAL Frequency */
    XTALFreq = SOC_rcmGetXtalFrequency ();

   /* program PLL dividers and multipliers. The value are taken from  TPR12_ADPLLJ_Settings_1p0.xlsx */
    adplljCfg = SOC_rcmGetADPLLJConfig(gSocRcmXtalInfo[XTALFreq].Finp, outFreqId);
    DebugP_assert(adplljCfg != NULL);

    if (adplljCfg != NULL)
    {
        if (gSocRcmXtalInfo[XTALFreq].div2flag == 0)
        {
            SOC_rcmProgPllPerDivider (adplljCfg->N, 0U, adplljCfg->M, adplljCfg->M2, adplljCfg->FracM);
        }
        else
        {
            uint32_t N;

            DebugP_assert(((adplljCfg->N + 1) % 2) == 0);
            /* Input XTAL freq is half. Divide input divider by 2 to get same output freq */
            N = ((adplljCfg->N + 1) / 2) - 1;

            SOC_rcmProgPllPerDivider (N, 0U, adplljCfg->M, adplljCfg->M2, adplljCfg->FracM);

        }
        /* Configure and Lock Core PLL */
        SOC_rcmConfigurePllPer (perTrimVal);

        /* Derive Clocks */
        /* TPR12_Ch08_Clock_Arch_0p91 is used as reference for below settings */
        /* Set clock divider values from PER PLL*/
        /* 1728 Mhz */
        // HSDIV-2 Settings
        if (hsDivCfg->hsdivOutEnMask & SOC_RCM_PLL_HSDIV_OUTPUT_ENABLE_0)
        {
            DebugP_assert((Fout % hsDivCfg->hsDivOutFreqHz[0]) == 0);
            hsDivOutRegVal = Fout / hsDivCfg->hsDivOutFreqHz[0];
            hsDivOutRegVal--;
            //HW_WR_REG32(CSL_MSS_TOPRCM_U_BASE+PLL_PER_HSDIVIDER_CLKOUT0, 0x0);    // CLKOUT0_DIV[4:0] = 0  -- 1.728G/(0+1) = 1.728GHz
            ptrTopRCMRegs->PLL_PER_HSDIVIDER_CLKOUT0 = SOC_rcmInsert8 (ptrTopRCMRegs->PLL_PER_HSDIVIDER_CLKOUT0, 4U, 0U, hsDivOutRegVal);
        }
        if (hsDivCfg->hsdivOutEnMask & SOC_RCM_PLL_HSDIV_OUTPUT_ENABLE_1)
        {
            DebugP_assert((Fout % hsDivCfg->hsDivOutFreqHz[1]) == 0);
            hsDivOutRegVal = Fout / hsDivCfg->hsDivOutFreqHz[1];
            hsDivOutRegVal--;
            //HW_WR_REG32(CSL_MSS_TOPRCM_U_BASE+PLL_PER_HSDIVIDER_CLKOUT1, 0x8);    // CLKOUT1_DIV[4:0] = 8  -- 1.728G/(8+1) = 192 MHz
            ptrTopRCMRegs->PLL_PER_HSDIVIDER_CLKOUT1 = SOC_rcmInsert8 (ptrTopRCMRegs->PLL_PER_HSDIVIDER_CLKOUT1, 4U, 0U, hsDivOutRegVal);
        }
        if (hsDivCfg->hsdivOutEnMask & SOC_RCM_PLL_HSDIV_OUTPUT_ENABLE_2)
        {
            DebugP_assert((Fout % hsDivCfg->hsDivOutFreqHz[2]) == 0);
            hsDivOutRegVal = Fout / hsDivCfg->hsDivOutFreqHz[2];
            hsDivOutRegVal--;
            //HW_WR_REG32(CSL_MSS_TOPRCM_U_BASE+PLL_PER_HSDIVIDER_CLKOUT2, 0x11);    // CLKOUT2_DIV[4:0] = 11  -- 1.8G/(17+1) = 96 MHz
            ptrTopRCMRegs->PLL_PER_HSDIVIDER_CLKOUT2 = SOC_rcmInsert8 (ptrTopRCMRegs->PLL_PER_HSDIVIDER_CLKOUT2, 4U, 0U, hsDivOutRegVal);
        }
        if (hsDivCfg->hsdivOutEnMask & SOC_RCM_PLL_HSDIV_OUTPUT_ENABLE_3)
        {
            DebugP_assert((Fout % hsDivCfg->hsDivOutFreqHz[3]) == 0);
            hsDivOutRegVal = Fout / hsDivCfg->hsDivOutFreqHz[3];
            hsDivOutRegVal--;
            //Unsed HW_WR_REG32(CSL_MSS_TOPRCM_U_BASE+PLL_DSP_HSDIVIDER_CLKOUT3, 0x9);    //  unused
            ptrTopRCMRegs->PLL_PER_HSDIVIDER_CLKOUT3 = SOC_rcmInsert8 (ptrTopRCMRegs->PLL_PER_HSDIVIDER_CLKOUT3, 4U, 0U, hsDivOutRegVal);
        }
        /* Generate Trigger to latch these values */
        //HW_WR_REG32(CSL_MSS_TOPRCM_U_BASE+PLL_PER_HSDIVIDER        , 0x4);    // HSDIVIDER[2]     = 1
        ptrTopRCMRegs->PLL_PER_HSDIVIDER         = SOC_rcmInsert8 (ptrTopRCMRegs->PLL_PER_HSDIVIDER, 2U, 2U, 0x1U);
        //HW_WR_REG32(CSL_MSS_TOPRCM_U_BASE+PLL_PER_HSDIVIDER        , 0x0);    // HSDIVIDER[2]     = 0
        ptrTopRCMRegs->PLL_PER_HSDIVIDER         = SOC_rcmInsert8 (ptrTopRCMRegs->PLL_PER_HSDIVIDER, 2U, 2U, 0x0U);

        /* Ungate the clocks */
        if (hsDivCfg->hsdivOutEnMask & SOC_RCM_PLL_HSDIV_OUTPUT_ENABLE_0)
        {
            //HW_WR_REG32(CSL_MSS_TOPRCM_U_BASE+PLL_PER_HSDIVIDER_CLKOUT0, 0x100);  //+CLKOUT0_GATE[8]  = 1
            ptrTopRCMRegs->PLL_PER_HSDIVIDER_CLKOUT0 = SOC_rcmInsert8 (ptrTopRCMRegs->PLL_PER_HSDIVIDER_CLKOUT0, 8U, 8U, 0x1U);
        }
        if (hsDivCfg->hsdivOutEnMask & SOC_RCM_PLL_HSDIV_OUTPUT_ENABLE_1)
        {
            //HW_WR_REG32(CSL_MSS_TOPRCM_U_BASE+PLL_PER_HSDIVIDER_CLKOUT1, 0x108);  //+CLKOUT1_GATE[8]  = 1
            ptrTopRCMRegs->PLL_PER_HSDIVIDER_CLKOUT1 = SOC_rcmInsert8 (ptrTopRCMRegs->PLL_PER_HSDIVIDER_CLKOUT1, 8U, 8U, 0x1U);
        }
        if (hsDivCfg->hsdivOutEnMask & SOC_RCM_PLL_HSDIV_OUTPUT_ENABLE_2)
        {
            //HW_WR_REG32(CSL_MSS_TOPRCM_U_BASE+PLL_PER_HSDIVIDER_CLKOUT2, 0x111);  //+CLKOUT2_GATE[8]  = 1
            ptrTopRCMRegs->PLL_PER_HSDIVIDER_CLKOUT2 = SOC_rcmInsert8 (ptrTopRCMRegs->PLL_PER_HSDIVIDER_CLKOUT2, 8U, 8U, 0x1U);
        }
        if (hsDivCfg->hsdivOutEnMask & SOC_RCM_PLL_HSDIV_OUTPUT_ENABLE_3)
        {
            //UnusedHW_WR_REG32(CSL_MSS_TOPRCM_U_BASE+PLL_DSP_HSDIVIDER_CLKOUT3, 0x109);  //+CLKOUT3_GATE[8]  = 1
            ptrTopRCMRegs->PLL_PER_HSDIVIDER_CLKOUT3 = SOC_rcmInsert8 (ptrTopRCMRegs->PLL_PER_HSDIVIDER_CLKOUT3, 8U, 8U, 0x1U);
        }
    }

    return;
}

static uint32_t SOC_rcmGetR5ClockFrequency(void)
{
    uint32_t clkFreq = 0U;
    SOC_RcmClkSrcInfo const * clkSrcInfo;
    SOC_RcmXtalFreqId clkFreqId;
    uint32_t Finp;


    clkSrcInfo = &gSocRcmR5ClkSrcInfoMap;
    clkFreqId = SOC_rcmGetXtalFrequency ();

    Finp = gSocRcmXtalInfo[clkFreqId].Finp;
    clkFreq = SOC_rcmGetCoreHsDivOut(Finp, gSocRcmXtalInfo[clkFreqId].div2flag, clkSrcInfo->hsDivOut);

    return (clkFreq);

}

int32_t SOC_rcmSetR5Clock(uint32_t r5FreqHz, uint32_t sysClkFreqHz)
{
    CSL_mss_toprcmRegs *ptrTopRCMRegs;
    uint32_t Finp;
    uint32_t moduleClkDivVal;

    Finp = SOC_rcmGetR5ClockFrequency();
    ptrTopRCMRegs = SOC_rcmGetBaseAddressTOPRCM ();
    moduleClkDivVal = SOC_rcmGetModuleClkDivVal(Finp, r5FreqHz);

    /* Errata i2387 fix: set staggered PLL config to nullify the GCM circuit glitch in case of 200 MHz and 400 MHz */
    if(r5FreqHz == 400*1000000)
    {
        /* Switch back to XTAL */
        ptrTopRCMRegs->MSS_CR5_CLK_SRC_SEL = 0U;
        ptrTopRCMRegs->MSS_CR5_DIV_VAL = 0U;
        ptrTopRCMRegs->SYS_CLK_DIV_VAL = 0U;

        /* Suppress the glitch */
        ptrTopRCMRegs->MSS_CR5_DIV_VAL = 0x111;

        /* R5F and SYS clocks are in the ratio 1:2 */
        ptrTopRCMRegs->SYS_CLK_DIV_VAL = 0x111;

        /* Switch to PLL clock */
        ptrTopRCMRegs->MSS_CR5_CLK_SRC_SEL = 0x222;

        /* Switch back to 400 MHz */
        ptrTopRCMRegs->MSS_CR5_DIV_VAL = 0U;
    }
    else if(r5FreqHz == 200*1000000)
    {
        /* Switch back to XTAL */
        ptrTopRCMRegs->MSS_CR5_CLK_SRC_SEL = 0U;
        ptrTopRCMRegs->MSS_CR5_DIV_VAL = 0U;
        ptrTopRCMRegs->SYS_CLK_DIV_VAL = 0U;

        /* Suppress the glitch */
        ptrTopRCMRegs->MSS_CR5_DIV_VAL = 0x111;

        /* R5F and SYS clocks are in the ratio 1:1 */
        ptrTopRCMRegs->SYS_CLK_DIV_VAL = 0U;

        /* Switch to PLL clock */
        ptrTopRCMRegs->MSS_CR5_CLK_SRC_SEL = 0x222;

        /* Switch back to 200 MHz */
        ptrTopRCMRegs->MSS_CR5_DIV_VAL = 0U;
    }
    else
    {
        /* Divide by 1 to get the R5 Core Clock */
        ptrTopRCMRegs->MSS_CR5_DIV_VAL = SOC_rcmInsert16 (ptrTopRCMRegs->MSS_CR5_DIV_VAL, 11U, 0U, SOC_rcmGetModuleClkDivRegVal(moduleClkDivVal));

        /* Divide by 2 to get the VCLK */
        moduleClkDivVal = SOC_rcmGetModuleClkDivVal(Finp, sysClkFreqHz);

        ptrTopRCMRegs->SYS_CLK_DIV_VAL = SOC_rcmInsert16 (ptrTopRCMRegs->SYS_CLK_DIV_VAL, 11U, 0U, SOC_rcmGetModuleClkDivRegVal(moduleClkDivVal));

        /* Select CLKOUT2 as clock for R5 Core */
        ptrTopRCMRegs->MSS_CR5_CLK_SRC_SEL = SOC_rcmInsert16 (ptrTopRCMRegs->MSS_CR5_CLK_SRC_SEL, 11U, 0U, gSocRcmR5ClkSrcValMap[SOC_RcmR5ClockSource_DPLL_CORE_HSDIV0_CLKOUT2]);
    }

    return SystemP_SUCCESS;
}

uint32_t SOC_rcmGetR5Clock(void)
{
    uint32_t Finp;
    uint32_t moduleClkDivRegVal;
    uint32_t clkDivVal;
    CSL_mss_toprcmRegs *ptrTopRCMRegs;

    ptrTopRCMRegs = SOC_rcmGetBaseAddressTOPRCM ();
    Finp = SOC_rcmGetR5ClockFrequency();
    moduleClkDivRegVal = ptrTopRCMRegs->MSS_CR5_DIV_VAL;
    clkDivVal = SOC_rcmGetModuleClkDivFromRegVal(moduleClkDivRegVal);
    return (Finp / clkDivVal);
}

static uint32_t SOC_rcmGetPeripheralClockFrequency(SOC_RcmPeripheralClockSource clkSource)
{
    uint32_t clkFreq = 0U;
    SOC_RcmClkSrcInfo const * clkSrcInfo;
    SOC_RcmXtalFreqId clkFreqId;
    uint32_t Finp;

    clkSrcInfo = &gSocRcmPeripheralClkSrcInfoMap[clkSource];
    clkFreqId = SOC_rcmGetXtalFrequency ();
    switch (clkSrcInfo->pllId)
    {
        case SOC_RcmPllId_CORE:
        {
            Finp = gSocRcmXtalInfo[clkFreqId].Finp;
            clkFreq = SOC_rcmGetCoreHsDivOut(Finp, gSocRcmXtalInfo[clkFreqId].div2flag, clkSrcInfo->hsDivOut);
            if (clkSource == SOC_RcmPeripheralClockSource_SYS_CLK)
            {
                CSL_mss_toprcmRegs *ptrTopRCMRegs;
                uint32_t sysClkDiv;

                /* Core PLL settings */
                ptrTopRCMRegs = SOC_rcmGetBaseAddressTOPRCM ();
                sysClkDiv = CSL_FEXT(ptrTopRCMRegs->SYS_CLK_DIV_VAL, MSS_TOPRCM_SYS_CLK_DIV_VAL_SYS_CLK_DIV_VAL_CLKDIV);
                sysClkDiv = (sysClkDiv & 0xF) + 1;
                clkFreq = clkFreq / sysClkDiv;
            }
            break;
        }
        case SOC_RcmPllId_DSS:
        {
            Finp = gSocRcmXtalInfo[clkFreqId].Finp;
            clkFreq = SOC_rcmGetDspHsDivOut(Finp, gSocRcmXtalInfo[clkFreqId].div2flag, clkSrcInfo->hsDivOut);
            break;
        }
        case SOC_RcmPllId_PER:
        {
            Finp = gSocRcmXtalInfo[clkFreqId].Finp;
            clkFreq = SOC_rcmGetPerHsDivOut(Finp, gSocRcmXtalInfo[clkFreqId].div2flag, clkSrcInfo->hsDivOut);
            break;
        }
        case SOC_RcmPllId_XTALCLK:
        case SOC_RcmPllId_WUCPUCLK:
        {
            Finp = gSocRcmXtalInfo[clkFreqId].Finp;
            if (gSocRcmXtalInfo[clkFreqId].div2flag)
            {
                clkFreq = Finp/2;
            }
            else
            {
                clkFreq = Finp;
            }
            clkFreq = SOC_RCM_FREQ_MHZ2HZ(clkFreq);
            break;
        }
        default:
        {
            DebugP_assert(FALSE);
            break;
        }
    }
    return (clkFreq);

}

int32_t SOC_rcmSetPeripheralClock (SOC_RcmPeripheralId periphID,
                                      SOC_RcmPeripheralClockSource clkSource,
                                      uint32_t freqHz)
{
    volatile uint32_t   *ptrClkSrcReg, *ptrClkDivReg;
    uint16_t            clkSrcVal;
    uint32_t            clkDivisor;
    int32_t             retVal;
    uint32_t            Finp;

    if ((SOC_RcmPeripheralClockSource_XREF_CLK0 == clkSource) || (SOC_RcmPeripheralClockSource_XREF_CLK1 == clkSource))
    {
        /* For now, external clock sources from IOs use same input<->output value */
        Finp = freqHz;
    }
    else
    {
        Finp = SOC_rcmGetPeripheralClockFrequency(clkSource);
    }
    clkDivisor = SOC_rcmGetModuleClkDivVal(Finp, freqHz);
    SOC_rcmGetClkSrcAndDivReg (periphID, clkSource, &clkSrcVal, &ptrClkSrcReg, &ptrClkDivReg);

    if ((ptrClkSrcReg != NULL) && (ptrClkDivReg != NULL) && (clkSrcVal != 0x888U))
    {
        uint16_t            clkDivVal;

        /* Create the Divider Value to be programmed */
        clkDivVal = ((uint16_t)clkDivisor & 0xFU);
        clkDivVal = (clkDivVal | (clkDivVal << 4U) | (clkDivVal << 8U));

        /* Write the Divider Value */
        *ptrClkDivReg = SOC_rcmInsert16 (*ptrClkDivReg, 11U, 0U, clkDivVal);

        /* Write the Clock Source Selection Value */
        *ptrClkSrcReg = SOC_rcmInsert16 (*ptrClkSrcReg, 11U, 0U, clkSrcVal);
        retVal = SystemP_SUCCESS;
    }
    else
    {
        /* Error */
        retVal = SystemP_FAILURE;
    }

    return (retVal);
}


uint32_t SOC_rcmGetPeripheralClock (SOC_RcmPeripheralId periphID)
{
    uint32_t            clkDivisorRegVal;
    uint32_t            clkDivisor;
    int32_t             retVal;
    uint32_t            Finp;
    uint32_t            freqHz = 0;
    SOC_RcmPeripheralClockSource clkSource;

    retVal = SOC_rcmGetClkSrcAndDivValue(periphID, &clkSource, &clkDivisorRegVal);
    DebugP_assert(retVal == SystemP_SUCCESS);
    if (SystemP_SUCCESS == retVal)
    {
        Finp = SOC_rcmGetPeripheralClockFrequency(clkSource);
        clkDivisor = SOC_rcmGetModuleClkDivFromRegVal(clkDivisorRegVal);
        freqHz = Finp / clkDivisor;
    }
    return (freqHz);
}

static uint32_t SOC_rcmGetDspClockFrequency(SOC_RcmDspClockSource clkSource)
{
    uint32_t clkFreq = 0U;
    SOC_RcmClkSrcInfo const * clkSrcInfo;
    SOC_RcmXtalFreqId clkFreqId;
    uint32_t Finp;


    clkSrcInfo = &gSocRcmDspClkSrcInfoMap[clkSource];
    clkFreqId = SOC_rcmGetXtalFrequency ();
    switch (clkSrcInfo->pllId)
    {
        case SOC_RcmPllId_CORE:
        {
            Finp = gSocRcmXtalInfo[clkFreqId].Finp;
            clkFreq = SOC_rcmGetCoreHsDivOut(Finp, gSocRcmXtalInfo[clkFreqId].div2flag, clkSrcInfo->hsDivOut);
            break;
        }
        case SOC_RcmPllId_DSS:
        {
            Finp = gSocRcmXtalInfo[clkFreqId].Finp;
            clkFreq = SOC_rcmGetDspHsDivOut(Finp, gSocRcmXtalInfo[clkFreqId].div2flag, clkSrcInfo->hsDivOut);
            break;
        }
        case SOC_RcmPllId_PER:
        {
            Finp = gSocRcmXtalInfo[clkFreqId].Finp;
            clkFreq = SOC_rcmGetPerHsDivOut(Finp, gSocRcmXtalInfo[clkFreqId].div2flag, clkSrcInfo->hsDivOut);
            break;
        }
        case SOC_RcmPllId_XTALCLK:
        {
            Finp = gSocRcmXtalInfo[clkFreqId].Finp;
            if (gSocRcmXtalInfo[clkFreqId].div2flag)
            {
                clkFreq = Finp/2;
            }
            else
            {
                clkFreq = Finp;
            }
            break;
        }
        default:
        {
            DebugP_assert(FALSE);
            break;
        }
    }
    return (clkFreq);
}

int32_t SOC_rcmSetDspClock (SOC_RcmDspClockSource clkSource,
                                   uint32_t freqHz)
{
    volatile uint32_t   *ptrClkSrcReg, *ptrClkDivReg;
    uint16_t            clkSrcVal;
    int32_t             retVal;
    uint32_t            pllFout;
    uint32_t            clkDivisor;


    SOC_controlModuleUnlockMMR(SOC_DOMAIN_ID_DSS_RCM, 0);
    pllFout = SOC_rcmGetDspClockFrequency(clkSource);
    SOC_rcmGetDspClkSrcAndDivReg (clkSource, &clkSrcVal, &ptrClkSrcReg, &ptrClkDivReg);
    clkDivisor = SOC_rcmGetModuleClkDivVal(pllFout, freqHz);

    if ((ptrClkSrcReg != NULL) && (ptrClkDivReg != NULL) && (clkSrcVal != 0x888U))
    {
        uint16_t            clkDivVal;

        /* Create the Divider Value to be programmed */
        clkDivVal = ((uint16_t)clkDivisor & 0xFU);
        clkDivVal = (clkDivVal | (clkDivVal << 4U) | (clkDivVal << 8U));

        /* Write the Divider Value */
        *ptrClkDivReg = SOC_rcmInsert16 (*ptrClkDivReg, 11U, 0U, clkDivVal);

        /* Write the Clock Source Selection Value */
        *ptrClkSrcReg = SOC_rcmInsert16 (*ptrClkSrcReg, 11U, 0U, clkSrcVal);
        retVal = SystemP_SUCCESS;
    }
    else
    {
        /* Error */
        retVal = SystemP_FAILURE;
    }

    return (retVal);
}

uint32_t SOC_rcmGetDspClock (void)
{
    int32_t         retVal;
    uint32_t        pllFout;
    uint32_t        clkDivisorRegVal;
    uint32_t        clkDiv;
    uint32_t        freqHz = 0;
    SOC_RcmDspClockSource  clkSource;


    SOC_controlModuleUnlockMMR(SOC_DOMAIN_ID_DSS_RCM, 0);
    retVal = SOC_rcmGetDspClkSrcAndDivValue (&clkSource, &clkDivisorRegVal);

    DebugP_assert(retVal == SystemP_SUCCESS);
    if (SystemP_SUCCESS == retVal)
    {
        pllFout = SOC_rcmGetDspClockFrequency(clkSource);
        clkDiv = SOC_rcmGetModuleClkDivFromRegVal(clkDivisorRegVal);
        freqHz = (pllFout / clkDiv);
    }
    return (freqHz);
}


SOC_RcmResetCause SOC_rcmGetResetCause (void)
{
    CSL_mss_rcmRegs *ptrRCMRegs;
    uint16_t    resetCauseBits;
    uint8_t     resetCause = 0U;

    ptrRCMRegs = SOC_rcmGetBaseAddressMSSRCM ();

    /* Read the Reset Cause Register bits */
    resetCauseBits = SOC_rcmExtract16 (ptrRCMRegs->MSS_RST_STATUS, 9U, 0U);

    if (resetCauseBits == 0x0U)
    {
        resetCause = 10U;
    }
    else
    {
        while ((resetCauseBits & 0x1U) != 0x1U)
        {
            resetCauseBits = resetCauseBits >> 1U;
            resetCause = resetCause + 1U;
        }

        /* clear the reset cause */
        ptrRCMRegs->MSS_RST_CAUSE_CLR = SOC_rcmInsert8 (ptrRCMRegs->MSS_RST_CAUSE_CLR, 2U, 0U, 0x7U);
    }

    return (SOC_RcmResetCause) gSocRcmResetBitToResetCause[resetCause];
}

void SOC_rcmDspPowerOnReset(void)
{
    CSL_dss_rcmRegs *ptrRCMRegs;
    ptrRCMRegs = SOC_rcmGetBaseAddressDSSRCM ();
    CSL_FINS(ptrRCMRegs->DSS_DSP_RST_CTRL, DSS_RCM_DSS_DSP_RST_CTRL_DSS_DSP_RST_CTRL_ASSERT_POR, 0x0);
    CSL_FINS(ptrRCMRegs->DSS_DSP_RST_CTRL, DSS_RCM_DSS_DSP_RST_CTRL_DSS_DSP_RST_CTRL_ASSERT_GLOBAL, 0x0);
    CSL_FINS(ptrRCMRegs->DSS_DSP_RST_CTRL, DSS_RCM_DSS_DSP_RST_CTRL_DSS_DSP_RST_CTRL_ASSERT_LOCAL, 0x0);

    CSL_FINS(ptrRCMRegs->DSS_DSP_CLK_GATE, DSS_RCM_DSS_DSP_CLK_GATE_DSS_DSP_CLK_GATE_GATED, 0x0);

    CSL_FINS(ptrRCMRegs->DSP_PD_WAKEUP_MASK0, DSS_RCM_DSP_PD_WAKEUP_MASK0_DSP_PD_WAKEUP_MASK0_WAKEUP_MASK0, 0xFFFEFFFF);
    CSL_FINS(ptrRCMRegs->DSP_PD_TRIGGER_WAKUP, DSS_RCM_DSP_PD_TRIGGER_WAKUP_DSP_PD_TRIGGER_WAKUP_WAKEUP_TRIGGER, 0x1);

    while((CSL_FEXT(ptrRCMRegs->DSP_PD_STATUS, DSS_RCM_DSP_PD_STATUS_DSP_PD_STATUS_PROC_HALTED) & 0x1) != 1U);
}

void SOC_rcmR5PowerOnReset(void)
{
    uint32_t regVal;
    CSL_mss_rcmRegs *ptrRCMRegs;
    ptrRCMRegs = SOC_rcmGetBaseAddressMSSRCM ();
    regVal = ptrRCMRegs->RST2ASSERTDLY;
    CSL_FINS(regVal, MSS_RCM_RST2ASSERTDLY_RST2ASSERTDLY_R5SSA, 0x0);
    CSL_FINS(regVal, MSS_RCM_RST2ASSERTDLY_RST2ASSERTDLY_R5SSB, 0x0);
    CSL_FINS(regVal, MSS_RCM_RST2ASSERTDLY_RST2ASSERTDLY_R5A, 0x0);
    CSL_FINS(regVal, MSS_RCM_RST2ASSERTDLY_RST2ASSERTDLY_R5B, 0x0);
    ptrRCMRegs->RST2ASSERTDLY = regVal;

    /* WR_MEM_32(MSS_RCM_U_BASE+RST_WFICHECK, 0x00000707); */ //RSTWFI CHECK
    regVal = ptrRCMRegs->RST_WFICHECK;
    CSL_FINS(regVal, MSS_RCM_RST_WFICHECK_RST_WFICHECK_R5SSA, 0x7);
    CSL_FINS(regVal, MSS_RCM_RST_WFICHECK_RST_WFICHECK_R5SSB, 0x7);
    CSL_FINS(regVal, MSS_RCM_RST_WFICHECK_RST_WFICHECK_R5A, 0x7);
    CSL_FINS(regVal, MSS_RCM_RST_WFICHECK_RST_WFICHECK_R5B, 0x7);
    ptrRCMRegs->RST_WFICHECK = regVal;
}

void SOC_rcmR5TriggerReset(void)
{
    uint32_t regVal;
    CSL_mss_ctrlRegs *mssCtrl = SOC_rcmGetBaseAddressMSSCTRL ();

    regVal = mssCtrl->R5_CONTROL;
    CSL_FINS(regVal, MSS_CTRL_R5_CONTROL_R5_CONTROL_RESET_FSM_TRIGGER, 0x7);
    mssCtrl->R5_CONTROL = regVal;
#if defined(__ARM_ARCH_7R__)
    __wfi();
#endif
}

void SOC_rcmCr5bUnhalt(void)
{
    CSL_mss_ctrlRegs *mssCtrl = SOC_rcmGetBaseAddressMSSCTRL ();
    //Core B unhalt
    CSL_FINS(mssCtrl->R5_COREB_HALT, MSS_CTRL_R5_COREB_HALT_R5_COREB_HALT_HALT, 0x0);
}

void SOC_rcmC66xStart(void)
{
    CSL_dss_rcmRegs *ptrRCMRegs;
    ptrRCMRegs = SOC_rcmGetBaseAddressDSSRCM ();

    CSL_FINS(ptrRCMRegs->DSP_PD_CTRL, DSS_RCM_DSP_PD_CTRL_DSP_PD_CTRL_INTERRUPT_MASK, 0x1);
    CSL_FINS(ptrRCMRegs->DSP_PD_CTRL, DSS_RCM_DSP_PD_CTRL_DSP_PD_CTRL_INTERRUPT_MASK, 0x0);
    CSL_FINS(ptrRCMRegs->DSP_PD_CTRL, DSS_RCM_DSP_PD_CTRL_DSP_PD_CTRL_PROC_HALT, 0x0);
}

void SOC_rcmR5ConfigLockStep(void)
{
    CSL_mss_ctrlRegs *mssCtrl = SOC_rcmGetBaseAddressMSSCTRL ();
    uint32_t regVal;
    regVal = mssCtrl->R5_CONTROL;
    CSL_FINS(regVal, MSS_CTRL_R5_CONTROL_R5_CONTROL_LOCK_STEP, 0x7);
    CSL_FINS(regVal, MSS_CTRL_R5_CONTROL_R5_CONTROL_LOCK_STEP_SWITCH_WAIT, 0x7);
    mssCtrl->R5_CONTROL = regVal;
}

#define DUALCOREBOOTENABLE_START_BIT        (13U)
#define DUALCOREBOOTENABLE_END_BIT          (13U)
#define DUALCORESWITCHDISABLE_START_BIT     (14U)
#define DUALCORESWITCHDISABLE_END_BIT       (14U)

uint8_t CSL_extract8 (volatile uint32_t reg, uint8_t msb, uint8_t lsb)
{
    uint32_t    mask;
    uint8_t     bits;
    uint8_t     value;

    /* Compute the mask: */
    bits = (msb - lsb + 1U);
    mask = (uint32_t)((uint32_t)1U << bits);
    mask = mask - 1U;

    value = (uint8_t)((reg >> lsb) & mask);
    return value;
}

uint8_t CSL_TopCtrl_dualCoreBootEnableEfuse (const CSL_top_ctrlRegs * ptrTopCtrlRegs)
{
    return (CSL_extract8 (ptrTopCtrlRegs->EFUSE1_ROW_11, \
                          DUALCOREBOOTENABLE_END_BIT, \
                          DUALCOREBOOTENABLE_START_BIT));
}

uint8_t CSL_TopCtrl_dualCoreSwitchDisableEfuse (const CSL_top_ctrlRegs * ptrTopCtrlRegs)
{
    return (CSL_extract8 (ptrTopCtrlRegs->EFUSE1_ROW_11, \
                          DUALCORESWITCHDISABLE_END_BIT, \
                          DUALCORESWITCHDISABLE_START_BIT));
}

uint32_t SOC_rcmIsDualCoreSwitchSupported(void)
{
    CSL_top_ctrlRegs* ptrTopCtrlRegs;
    uint8_t dualCoreBootEnable, dualCoreSwitchDisable;
    uint32_t retVal = FALSE;

    ptrTopCtrlRegs = SOC_rcmGetBaseAddressTOPCTRL ();
    dualCoreBootEnable = CSL_TopCtrl_dualCoreBootEnableEfuse(ptrTopCtrlRegs);
    dualCoreSwitchDisable = CSL_TopCtrl_dualCoreSwitchDisableEfuse(ptrTopCtrlRegs);
    if ((dualCoreBootEnable == 0) && (dualCoreSwitchDisable == 0))
    {
        retVal = TRUE;
    }
    return retVal;
}

void SOC_rcmR5ConfigDualCore(void)
{
    uint32_t regVal;
    CSL_mss_ctrlRegs *mssCtrl = SOC_rcmGetBaseAddressMSSCTRL ();

    if (SOC_rcmIsDualCoreSwitchSupported() == TRUE)
    {
        regVal = mssCtrl->R5_CONTROL;
        if ((regVal & CSL_MSS_CTRL_R5_CONTROL_R5_CONTROL_LOCK_STEP_MASK) == 0x0)
        {
            /* It is already in dual core mode. Skip programming LOCK STEP Bit. */
        }
        else
        {
            CSL_FINS(regVal, MSS_CTRL_R5_CONTROL_R5_CONTROL_LOCK_STEP, 0x0);
            CSL_FINS(regVal, MSS_CTRL_R5_CONTROL_R5_CONTROL_LOCK_STEP_SWITCH_WAIT, 0x7);
            mssCtrl->R5_CONTROL = regVal;
        }
    }
}

void SOC_rcmGetEfuseQSPIConfig(SOC_RcmEfuseQspiConfig *qspiEfuseCfg)
{
    CSL_top_ctrlRegs* ptrTopCtrlRegs;

    ptrTopCtrlRegs = SOC_rcmGetBaseAddressTOPCTRL ();

    qspiEfuseCfg->qspiClockFreqId  = (SOC_RcmQspiClockFreqId) SOC_rcmReadQspiClkFreqEfuse(ptrTopCtrlRegs);
    qspiEfuseCfg->flashClockModeId = (SOC_RcmEfuseFlashClkModeId) SOC_rcmReadFlashModeEfuse(ptrTopCtrlRegs);

}

void SOC_rcmStartMemInitTCMA(void)
{
    CSL_mss_ctrlRegs *mssCtrl = SOC_rcmGetBaseAddressMSSCTRL ();

    /* Check MEMINIT STATUS is zero to confirm no inprogress MEM INIT */
    while (CSL_FEXT(mssCtrl->MSS_ATCM_MEM_INIT_STATUS, MSS_CTRL_MSS_ATCM_MEM_INIT_STATUS_MSS_ATCM_MEM_INIT_STATUS_MEM_STATUS) != 0);

    /* Clear MEMINIT DONE before initiating MEMINIT */
    CSL_FINS(mssCtrl->MSS_ATCM_MEM_INIT_DONE, MSS_CTRL_MSS_ATCM_MEM_INIT_DONE_MSS_ATCM_MEM_INIT_DONE_MEM_INIT_DONE, 1);
    while (CSL_FEXT(mssCtrl->MSS_ATCM_MEM_INIT_DONE, MSS_CTRL_MSS_ATCM_MEM_INIT_DONE_MSS_ATCM_MEM_INIT_DONE_MEM_INIT_DONE) != 0);

    CSL_FINS(mssCtrl->MSS_ATCM_MEM_INIT, MSS_CTRL_MSS_ATCM_MEM_INIT_MSS_ATCM_MEM_INIT_MEM_INIT, 1);
}

void SOC_rcmWaitMemInitTCMA(void)
{
    CSL_mss_ctrlRegs*        mssCtrl;

    mssCtrl = SOC_rcmGetBaseAddressMSSCTRL ();

    while (CSL_FEXT(mssCtrl->MSS_ATCM_MEM_INIT_DONE, MSS_CTRL_MSS_ATCM_MEM_INIT_DONE_MSS_ATCM_MEM_INIT_DONE_MEM_INIT_DONE) != 1);
    CSL_FINS(mssCtrl->MSS_ATCM_MEM_INIT_DONE, MSS_CTRL_MSS_ATCM_MEM_INIT_DONE_MSS_ATCM_MEM_INIT_DONE_MEM_INIT_DONE, 1);
    /* Check MEMINIT STATUS is zero to confirm no inprogress MEM INIT */
    while (CSL_FEXT(mssCtrl->MSS_ATCM_MEM_INIT_STATUS, MSS_CTRL_MSS_ATCM_MEM_INIT_STATUS_MSS_ATCM_MEM_INIT_STATUS_MEM_STATUS) != 0);
    while (CSL_FEXT(mssCtrl->MSS_ATCM_MEM_INIT_DONE, MSS_CTRL_MSS_ATCM_MEM_INIT_DONE_MSS_ATCM_MEM_INIT_DONE_MEM_INIT_DONE) != 0);
}

void SOC_rcmStartMemInitTCMB(void)
{
    CSL_mss_ctrlRegs *mssCtrl = SOC_rcmGetBaseAddressMSSCTRL();

    /* Check MEMINIT STATUS is zero to confirm no inprogress MEM INIT */
    while (CSL_FEXT(mssCtrl->MSS_BTCM_MEM_INIT_STATUS, MSS_CTRL_MSS_BTCM_MEM_INIT_STATUS_MSS_BTCM_MEM_INIT_STATUS_MEM_STATUS) != 0);

    /* Clear MEMINIT DONE before initiating MEMINIT */
    CSL_FINS(mssCtrl->MSS_BTCM_MEM_INIT_DONE, MSS_CTRL_MSS_BTCM_MEM_INIT_DONE_MSS_BTCM_MEM_INIT_DONE_MEM_INIT_DONE, 1);
    while (CSL_FEXT(mssCtrl->MSS_BTCM_MEM_INIT_DONE, MSS_CTRL_MSS_BTCM_MEM_INIT_DONE_MSS_BTCM_MEM_INIT_DONE_MEM_INIT_DONE) != 0);

    CSL_FINS(mssCtrl->MSS_BTCM_MEM_INIT, MSS_CTRL_MSS_BTCM_MEM_INIT_MSS_BTCM_MEM_INIT_MEM_INIT, 1);
}


void SOC_rcmWaitMemInitTCMB(void)
{
    CSL_mss_ctrlRegs *mssCtrl = SOC_rcmGetBaseAddressMSSCTRL();

    while (CSL_FEXT(mssCtrl->MSS_BTCM_MEM_INIT_DONE, MSS_CTRL_MSS_BTCM_MEM_INIT_DONE_MSS_BTCM_MEM_INIT_DONE_MEM_INIT_DONE) != 1);
    CSL_FINS(mssCtrl->MSS_BTCM_MEM_INIT_DONE, MSS_CTRL_MSS_BTCM_MEM_INIT_DONE_MSS_BTCM_MEM_INIT_DONE_MEM_INIT_DONE, 1);

    /* Check MEMINIT STATUS is zero to confirm no inprogress MEM INIT */
    while (CSL_FEXT(mssCtrl->MSS_BTCM_MEM_INIT_STATUS, MSS_CTRL_MSS_BTCM_MEM_INIT_STATUS_MSS_BTCM_MEM_INIT_STATUS_MEM_STATUS) != 0);
    while (CSL_FEXT(mssCtrl->MSS_BTCM_MEM_INIT_DONE, MSS_CTRL_MSS_BTCM_MEM_INIT_DONE_MSS_BTCM_MEM_INIT_DONE_MEM_INIT_DONE) != 0);
}

void SOC_rcmMemInitMssMailboxMemory(void)
{
    CSL_mss_ctrlRegs *mssCtrl = SOC_rcmGetBaseAddressMSSCTRL();
    CSL_FINS(mssCtrl->MSS_MAILBOX_MEM_INIT, MSS_CTRL_MSS_MAILBOX_MEM_INIT_MSS_MAILBOX_MEM_INIT_MEM0_INIT, 1);
    while (CSL_FEXT(mssCtrl->MSS_MAILBOX_MEM_INIT_DONE, MSS_CTRL_MSS_MAILBOX_MEM_INIT_DONE_MSS_MAILBOX_MEM_INIT_DONE_MEM0_DONE) != 1);
}

void SOC_rcmMemInitDssMailboxMemory(void)
{
    CSL_dss_ctrlRegs *dssCtrl = SOC_rcmGetBaseAddressDSSCTRL();
    CSL_FINS(dssCtrl->DSS_MAILBOX_MEMINIT_START, DSS_CTRL_DSS_MAILBOX_MEMINIT_START_DSS_MAILBOX_MEMINIT_START_MEMINIT_START, 1);
    while (CSL_FEXT(dssCtrl->DSS_MAILBOX_MEMINIT_DONE, DSS_CTRL_DSS_MAILBOX_MEMINIT_DONE_DSS_MAILBOX_MEMINIT_DONE_MEMINIT_DONE) != 1);
}

void SOC_rcmStartMemInitMSSL2(void)
{
    CSL_mss_ctrlRegs*        ptrMSSCtrlRegs;

    ptrMSSCtrlRegs = SOC_rcmGetBaseAddressMSSCTRL ();

    /* Start the Initialization of L2 Memory */
    ptrMSSCtrlRegs->MSS_L2_MEM_INIT = SOC_rcmInsert8 (ptrMSSCtrlRegs->MSS_L2_MEM_INIT, 1U, 0U, 0x3U);

}

void SOC_rcmWaitMemInitMSSL2(void)
{
    CSL_mss_ctrlRegs*        ptrMSSCtrlRegs;

    ptrMSSCtrlRegs = SOC_rcmGetBaseAddressMSSCTRL ();

     /* wait for the L2 memory init to complete */
    while((ptrMSSCtrlRegs->MSS_L2_MEM_INIT_DONE & 0x3U) != 0x3U)
    {
        /* TBD - handle time out */
    }

    /* clear the status */
    ptrMSSCtrlRegs->MSS_L2_MEM_INIT_DONE = SOC_rcmInsert8 (ptrMSSCtrlRegs->MSS_L2_MEM_INIT_DONE, 1U, 0U, 0x3U);
}

void SOC_rcmStartMemInitDSSL2(uint32_t l2bankMask)
{
    CSL_dss_ctrlRegs *dssCtrl = SOC_rcmGetBaseAddressDSSCTRL ();
    uint32_t memBankInit = 0;

    SOC_controlModuleUnlockMMR(SOC_DOMAIN_ID_DSS_CTRL, 0);
    if (l2bankMask & SOC_RCM_MEMINIT_DSSL2_MEMBANK_VB00)
    {
        memBankInit |= CSL_DSS_CTRL_DSS_DSP_L2RAM_MEMINIT_START_DSS_DSP_L2RAM_MEMINIT_START_VB00_MASK;
    }
    if (l2bankMask & SOC_RCM_MEMINIT_DSSL2_MEMBANK_VB01)
    {
        memBankInit |= CSL_DSS_CTRL_DSS_DSP_L2RAM_MEMINIT_START_DSS_DSP_L2RAM_MEMINIT_START_VB01_MASK;
    }
    if (l2bankMask & SOC_RCM_MEMINIT_DSSL2_MEMBANK_VB10)
    {
        memBankInit |= CSL_DSS_CTRL_DSS_DSP_L2RAM_MEMINIT_START_DSS_DSP_L2RAM_MEMINIT_START_VB10_MASK;
    }
    if (l2bankMask & SOC_RCM_MEMINIT_DSSL2_MEMBANK_VB11)
    {
        memBankInit |= CSL_DSS_CTRL_DSS_DSP_L2RAM_MEMINIT_START_DSS_DSP_L2RAM_MEMINIT_START_VB11_MASK;
    }
    if (l2bankMask & SOC_RCM_MEMINIT_DSSL2_MEMBANK_VB20)
    {
        memBankInit |= CSL_DSS_CTRL_DSS_DSP_L2RAM_MEMINIT_START_DSS_DSP_L2RAM_MEMINIT_START_VB20_MASK;
    }
    if (l2bankMask & SOC_RCM_MEMINIT_DSSL2_MEMBANK_VB21)
    {
        memBankInit |= CSL_DSS_CTRL_DSS_DSP_L2RAM_MEMINIT_START_DSS_DSP_L2RAM_MEMINIT_START_VB21_MASK;
    }
    if (l2bankMask & SOC_RCM_MEMINIT_DSSL2_MEMBANK_VB30)
    {
        memBankInit |= CSL_DSS_CTRL_DSS_DSP_L2RAM_MEMINIT_START_DSS_DSP_L2RAM_MEMINIT_START_VB30_MASK;
    }
    if (l2bankMask & SOC_RCM_MEMINIT_DSSL2_MEMBANK_VB31)
    {
        memBankInit |= CSL_DSS_CTRL_DSS_DSP_L2RAM_MEMINIT_START_DSS_DSP_L2RAM_MEMINIT_START_VB31_MASK;
    }

    /* Start the Initialization of L2 Memory */
    dssCtrl->DSS_DSP_L2RAM_MEMINIT_START = SOC_rcmInsert8 (dssCtrl->DSS_DSP_L2RAM_MEMINIT_START, 7U, 0U, memBankInit);
}

void SOC_rcmWaitMemInitDSSL2(uint32_t l2bankMask)
{
    CSL_dss_ctrlRegs *dssCtrl = SOC_rcmGetBaseAddressDSSCTRL();
    uint32_t          clearMemInitMask = 0;

    SOC_controlModuleUnlockMMR(SOC_DOMAIN_ID_DSS_CTRL, 0);

    if (l2bankMask & SOC_RCM_MEMINIT_DSSL2_MEMBANK_VB00)
    {
        while (CSL_FEXT(dssCtrl->DSS_DSP_L2RAM_MEMINIT_DONE, DSS_CTRL_DSS_DSP_L2RAM_MEMINIT_DONE_DSS_DSP_L2RAM_MEMINIT_DONE_VB00) != 1);
        clearMemInitMask |= CSL_DSS_CTRL_DSS_DSP_L2RAM_MEMINIT_DONE_DSS_DSP_L2RAM_MEMINIT_DONE_VB00_MASK;
    }

    if (l2bankMask & SOC_RCM_MEMINIT_DSSL2_MEMBANK_VB01)
    {
        while (CSL_FEXT(dssCtrl->DSS_DSP_L2RAM_MEMINIT_DONE, DSS_CTRL_DSS_DSP_L2RAM_MEMINIT_DONE_DSS_DSP_L2RAM_MEMINIT_DONE_VB01) != 1);
        clearMemInitMask |= CSL_DSS_CTRL_DSS_DSP_L2RAM_MEMINIT_DONE_DSS_DSP_L2RAM_MEMINIT_DONE_VB01_MASK;
    }

    if (l2bankMask & SOC_RCM_MEMINIT_DSSL2_MEMBANK_VB10)
    {
        while (CSL_FEXT(dssCtrl->DSS_DSP_L2RAM_MEMINIT_DONE, DSS_CTRL_DSS_DSP_L2RAM_MEMINIT_DONE_DSS_DSP_L2RAM_MEMINIT_DONE_VB10) != 1);
        clearMemInitMask |= CSL_DSS_CTRL_DSS_DSP_L2RAM_MEMINIT_DONE_DSS_DSP_L2RAM_MEMINIT_DONE_VB10_MASK;
    }

    if (l2bankMask & SOC_RCM_MEMINIT_DSSL2_MEMBANK_VB11)
    {
        while (CSL_FEXT(dssCtrl->DSS_DSP_L2RAM_MEMINIT_DONE, DSS_CTRL_DSS_DSP_L2RAM_MEMINIT_DONE_DSS_DSP_L2RAM_MEMINIT_DONE_VB11) != 1);
        clearMemInitMask |= CSL_DSS_CTRL_DSS_DSP_L2RAM_MEMINIT_DONE_DSS_DSP_L2RAM_MEMINIT_DONE_VB11_MASK;
    }

    if (l2bankMask & SOC_RCM_MEMINIT_DSSL2_MEMBANK_VB20)
    {
        while (CSL_FEXT(dssCtrl->DSS_DSP_L2RAM_MEMINIT_DONE, DSS_CTRL_DSS_DSP_L2RAM_MEMINIT_DONE_DSS_DSP_L2RAM_MEMINIT_DONE_VB20) != 1);
        clearMemInitMask |= CSL_DSS_CTRL_DSS_DSP_L2RAM_MEMINIT_DONE_DSS_DSP_L2RAM_MEMINIT_DONE_VB20_MASK;
    }

    if (l2bankMask & SOC_RCM_MEMINIT_DSSL2_MEMBANK_VB21)
    {
        while (CSL_FEXT(dssCtrl->DSS_DSP_L2RAM_MEMINIT_DONE, DSS_CTRL_DSS_DSP_L2RAM_MEMINIT_DONE_DSS_DSP_L2RAM_MEMINIT_DONE_VB21) != 1);
        clearMemInitMask |= CSL_DSS_CTRL_DSS_DSP_L2RAM_MEMINIT_DONE_DSS_DSP_L2RAM_MEMINIT_DONE_VB21_MASK;
    }

    if (l2bankMask & SOC_RCM_MEMINIT_DSSL2_MEMBANK_VB30)
    {
        while (CSL_FEXT(dssCtrl->DSS_DSP_L2RAM_MEMINIT_DONE, DSS_CTRL_DSS_DSP_L2RAM_MEMINIT_DONE_DSS_DSP_L2RAM_MEMINIT_DONE_VB30) != 1);
        clearMemInitMask |= CSL_DSS_CTRL_DSS_DSP_L2RAM_MEMINIT_DONE_DSS_DSP_L2RAM_MEMINIT_DONE_VB30_MASK;
    }

    if (l2bankMask & SOC_RCM_MEMINIT_DSSL2_MEMBANK_VB31)
    {
        while (CSL_FEXT(dssCtrl->DSS_DSP_L2RAM_MEMINIT_DONE, DSS_CTRL_DSS_DSP_L2RAM_MEMINIT_DONE_DSS_DSP_L2RAM_MEMINIT_DONE_VB31) != 1);
        clearMemInitMask |= CSL_DSS_CTRL_DSS_DSP_L2RAM_MEMINIT_DONE_DSS_DSP_L2RAM_MEMINIT_DONE_VB31_MASK;
    }

    dssCtrl->DSS_DSP_L2RAM_MEMINIT_DONE = SOC_rcmInsert8 (dssCtrl->DSS_DSP_L2RAM_MEMINIT_DONE, 7U, 0U, clearMemInitMask);

    if (l2bankMask & SOC_RCM_MEMINIT_DSSL2_MEMBANK_VB00)
    {
        /* Check MEMINIT STATUS is zero to confirm no inprogress MEM INIT */
        while (CSL_FEXT(dssCtrl->DSS_DSP_L2RAM_MEMINIT_STATUS, DSS_CTRL_DSS_DSP_L2RAM_MEMINIT_STATUS_DSS_DSP_L2RAM_MEMINIT_STATUS_VB00) != 0);
        while (CSL_FEXT(dssCtrl->DSS_DSP_L2RAM_MEMINIT_DONE, DSS_CTRL_DSS_DSP_L2RAM_MEMINIT_DONE_DSS_DSP_L2RAM_MEMINIT_DONE_VB00) != 0);
    }

    if (l2bankMask & SOC_RCM_MEMINIT_DSSL2_MEMBANK_VB01)
    {
        /* Check MEMINIT STATUS is zero to confirm no inprogress MEM INIT */
        while (CSL_FEXT(dssCtrl->DSS_DSP_L2RAM_MEMINIT_STATUS, DSS_CTRL_DSS_DSP_L2RAM_MEMINIT_STATUS_DSS_DSP_L2RAM_MEMINIT_STATUS_VB01) != 0);
        while (CSL_FEXT(dssCtrl->DSS_DSP_L2RAM_MEMINIT_DONE, DSS_CTRL_DSS_DSP_L2RAM_MEMINIT_DONE_DSS_DSP_L2RAM_MEMINIT_DONE_VB01) != 0);
    }

    if (l2bankMask & SOC_RCM_MEMINIT_DSSL2_MEMBANK_VB10)
    {
        /* Check MEMINIT STATUS is zero to confirm no inprogress MEM INIT */
        while (CSL_FEXT(dssCtrl->DSS_DSP_L2RAM_MEMINIT_STATUS, DSS_CTRL_DSS_DSP_L2RAM_MEMINIT_STATUS_DSS_DSP_L2RAM_MEMINIT_STATUS_VB10) != 0);
        while (CSL_FEXT(dssCtrl->DSS_DSP_L2RAM_MEMINIT_DONE, DSS_CTRL_DSS_DSP_L2RAM_MEMINIT_DONE_DSS_DSP_L2RAM_MEMINIT_DONE_VB10) != 0);
    }

    if (l2bankMask & SOC_RCM_MEMINIT_DSSL2_MEMBANK_VB11)
    {
        /* Check MEMINIT STATUS is zero to confirm no inprogress MEM INIT */
        while (CSL_FEXT(dssCtrl->DSS_DSP_L2RAM_MEMINIT_STATUS, DSS_CTRL_DSS_DSP_L2RAM_MEMINIT_STATUS_DSS_DSP_L2RAM_MEMINIT_STATUS_VB11) != 0);
        while (CSL_FEXT(dssCtrl->DSS_DSP_L2RAM_MEMINIT_DONE, DSS_CTRL_DSS_DSP_L2RAM_MEMINIT_DONE_DSS_DSP_L2RAM_MEMINIT_DONE_VB11) != 0);
    }

    if (l2bankMask & SOC_RCM_MEMINIT_DSSL2_MEMBANK_VB20)
    {
        /* Check MEMINIT STATUS is zero to confirm no inprogress MEM INIT */
        while (CSL_FEXT(dssCtrl->DSS_DSP_L2RAM_MEMINIT_STATUS, DSS_CTRL_DSS_DSP_L2RAM_MEMINIT_STATUS_DSS_DSP_L2RAM_MEMINIT_STATUS_VB20) != 0);
        while (CSL_FEXT(dssCtrl->DSS_DSP_L2RAM_MEMINIT_DONE, DSS_CTRL_DSS_DSP_L2RAM_MEMINIT_DONE_DSS_DSP_L2RAM_MEMINIT_DONE_VB20) != 0);
    }

    if (l2bankMask & SOC_RCM_MEMINIT_DSSL2_MEMBANK_VB21)
    {
        /* Check MEMINIT STATUS is zero to confirm no inprogress MEM INIT */
        while (CSL_FEXT(dssCtrl->DSS_DSP_L2RAM_MEMINIT_STATUS, DSS_CTRL_DSS_DSP_L2RAM_MEMINIT_STATUS_DSS_DSP_L2RAM_MEMINIT_STATUS_VB21) != 0);
        while (CSL_FEXT(dssCtrl->DSS_DSP_L2RAM_MEMINIT_DONE, DSS_CTRL_DSS_DSP_L2RAM_MEMINIT_DONE_DSS_DSP_L2RAM_MEMINIT_DONE_VB21) != 0);
    }

    if (l2bankMask & SOC_RCM_MEMINIT_DSSL2_MEMBANK_VB30)
    {
        /* Check MEMINIT STATUS is zero to confirm no inprogress MEM INIT */
        while (CSL_FEXT(dssCtrl->DSS_DSP_L2RAM_MEMINIT_STATUS, DSS_CTRL_DSS_DSP_L2RAM_MEMINIT_STATUS_DSS_DSP_L2RAM_MEMINIT_STATUS_VB30) != 0);
        while (CSL_FEXT(dssCtrl->DSS_DSP_L2RAM_MEMINIT_DONE, DSS_CTRL_DSS_DSP_L2RAM_MEMINIT_DONE_DSS_DSP_L2RAM_MEMINIT_DONE_VB30) != 0);
    }

    if (l2bankMask & SOC_RCM_MEMINIT_DSSL2_MEMBANK_VB31)
    {
        /* Check MEMINIT STATUS is zero to confirm no inprogress MEM INIT */
        while (CSL_FEXT(dssCtrl->DSS_DSP_L2RAM_MEMINIT_STATUS, DSS_CTRL_DSS_DSP_L2RAM_MEMINIT_STATUS_DSS_DSP_L2RAM_MEMINIT_STATUS_VB31) != 0);
        while (CSL_FEXT(dssCtrl->DSS_DSP_L2RAM_MEMINIT_DONE, DSS_CTRL_DSS_DSP_L2RAM_MEMINIT_DONE_DSS_DSP_L2RAM_MEMINIT_DONE_VB31) != 0);
    }
}

void SOC_rcmStartMemInitDSSL3(uint32_t l3bankMask)
{
    CSL_dss_ctrlRegs *dssCtrl = SOC_rcmGetBaseAddressDSSCTRL ();
    uint32_t memBankInit = 0;

    SOC_controlModuleUnlockMMR(SOC_DOMAIN_ID_DSS_CTRL, 0);
    if (l3bankMask & SOC_RCM_MEMINIT_DSSL3_MEMBANK_RAM0)
    {
        memBankInit |= CSL_DSS_CTRL_DSS_L3RAM_MEMINIT_START_DSS_L3RAM_MEMINIT_START_L3RAM0_MEMINIT_START_MASK;
    }

    if (l3bankMask & SOC_RCM_MEMINIT_DSSL3_MEMBANK_RAM1)
    {
        memBankInit |= CSL_DSS_CTRL_DSS_L3RAM_MEMINIT_START_DSS_L3RAM_MEMINIT_START_L3RAM1_MEMINIT_START_MASK;
    }

    if (l3bankMask & SOC_RCM_MEMINIT_DSSL3_MEMBANK_RAM2)
    {
        memBankInit |= CSL_DSS_CTRL_DSS_L3RAM_MEMINIT_START_DSS_L3RAM_MEMINIT_START_L3RAM2_MEMINIT_START_MASK;
    }

    if (l3bankMask & SOC_RCM_MEMINIT_DSSL3_MEMBANK_RAM3)
    {
        memBankInit |= CSL_DSS_CTRL_DSS_L3RAM_MEMINIT_START_DSS_L3RAM_MEMINIT_START_L3RAM3_MEMINIT_START_MASK;
    }

    /* Start the Initialization of L2 Memory */
    dssCtrl->DSS_L3RAM_MEMINIT_START = SOC_rcmInsert8 (dssCtrl->DSS_L3RAM_MEMINIT_START, 3U, 0U, memBankInit);
}

void SOC_rcmWaitMemInitDSSL3(uint32_t l3bankMask)
{
    CSL_dss_ctrlRegs *dssCtrl = SOC_rcmGetBaseAddressDSSCTRL ();
    uint32_t          clearMemInitMask = 0;

    SOC_controlModuleUnlockMMR(SOC_DOMAIN_ID_DSS_CTRL, 0);
    if (l3bankMask & SOC_RCM_MEMINIT_DSSL3_MEMBANK_RAM0)
    {
        while (CSL_FEXT(dssCtrl->DSS_L3RAM_MEMINIT_DONE, DSS_CTRL_DSS_L3RAM_MEMINIT_DONE_DSS_L3RAM_MEMINIT_DONE_L3RAM0_MEMINIT_DONE) != 1);
        clearMemInitMask |= CSL_DSS_CTRL_DSS_L3RAM_MEMINIT_DONE_DSS_L3RAM_MEMINIT_DONE_L3RAM0_MEMINIT_DONE_MASK;
    }

    if (l3bankMask & SOC_RCM_MEMINIT_DSSL3_MEMBANK_RAM1)
    {
        while (CSL_FEXT(dssCtrl->DSS_L3RAM_MEMINIT_DONE, DSS_CTRL_DSS_L3RAM_MEMINIT_DONE_DSS_L3RAM_MEMINIT_DONE_L3RAM1_MEMINIT_DONE) != 1);
        clearMemInitMask |= CSL_DSS_CTRL_DSS_L3RAM_MEMINIT_DONE_DSS_L3RAM_MEMINIT_DONE_L3RAM1_MEMINIT_DONE_MASK;
    }

    if (l3bankMask & SOC_RCM_MEMINIT_DSSL3_MEMBANK_RAM2)
    {
        while (CSL_FEXT(dssCtrl->DSS_L3RAM_MEMINIT_DONE, DSS_CTRL_DSS_L3RAM_MEMINIT_DONE_DSS_L3RAM_MEMINIT_DONE_L3RAM2_MEMINIT_DONE) != 1);
        clearMemInitMask |= CSL_DSS_CTRL_DSS_L3RAM_MEMINIT_DONE_DSS_L3RAM_MEMINIT_DONE_L3RAM2_MEMINIT_DONE_MASK;
    }

    if (l3bankMask & SOC_RCM_MEMINIT_DSSL3_MEMBANK_RAM3)
    {
        while (CSL_FEXT(dssCtrl->DSS_L3RAM_MEMINIT_DONE, DSS_CTRL_DSS_L3RAM_MEMINIT_DONE_DSS_L3RAM_MEMINIT_DONE_L3RAM3_MEMINIT_DONE) != 1);
        clearMemInitMask |= CSL_DSS_CTRL_DSS_L3RAM_MEMINIT_DONE_DSS_L3RAM_MEMINIT_DONE_L3RAM3_MEMINIT_DONE_MASK;
    }
    dssCtrl->DSS_L3RAM_MEMINIT_DONE = SOC_rcmInsert8 (dssCtrl->DSS_L3RAM_MEMINIT_DONE, 3U, 0U, clearMemInitMask);

    if (l3bankMask & SOC_RCM_MEMINIT_DSSL3_MEMBANK_RAM0)
    {
        /* Check MEMINIT STATUS is zero to confirm no inprogress MEM INIT */
        while (CSL_FEXT(dssCtrl->DSS_L3RAM_MEMINIT_STATUS, DSS_CTRL_DSS_L3RAM_MEMINIT_STATUS_DSS_L3RAM_MEMINIT_STATUS_L3RAM0_MEMINIT_STATUS) != 0);
        while (CSL_FEXT(dssCtrl->DSS_L3RAM_MEMINIT_DONE, DSS_CTRL_DSS_L3RAM_MEMINIT_DONE_DSS_L3RAM_MEMINIT_DONE_L3RAM0_MEMINIT_DONE) != 0);
    }

    if (l3bankMask & SOC_RCM_MEMINIT_DSSL3_MEMBANK_RAM1)
    {
        /* Check MEMINIT STATUS is zero to confirm no inprogress MEM INIT */
        while (CSL_FEXT(dssCtrl->DSS_L3RAM_MEMINIT_STATUS, DSS_CTRL_DSS_L3RAM_MEMINIT_STATUS_DSS_L3RAM_MEMINIT_STATUS_L3RAM1_MEMINIT_STATUS) != 0);
        while (CSL_FEXT(dssCtrl->DSS_L3RAM_MEMINIT_DONE, DSS_CTRL_DSS_L3RAM_MEMINIT_DONE_DSS_L3RAM_MEMINIT_DONE_L3RAM1_MEMINIT_DONE) != 0);
    }

    if (l3bankMask & SOC_RCM_MEMINIT_DSSL3_MEMBANK_RAM2)
    {
        /* Check MEMINIT STATUS is zero to confirm no inprogress MEM INIT */
        while (CSL_FEXT(dssCtrl->DSS_L3RAM_MEMINIT_STATUS, DSS_CTRL_DSS_L3RAM_MEMINIT_STATUS_DSS_L3RAM_MEMINIT_STATUS_L3RAM2_MEMINIT_STATUS) != 0);
        while (CSL_FEXT(dssCtrl->DSS_L3RAM_MEMINIT_DONE, DSS_CTRL_DSS_L3RAM_MEMINIT_DONE_DSS_L3RAM_MEMINIT_DONE_L3RAM2_MEMINIT_DONE) != 0);
    }

    if (l3bankMask & SOC_RCM_MEMINIT_DSSL3_MEMBANK_RAM3)
    {
        /* Check MEMINIT STATUS is zero to confirm no inprogress MEM INIT */
        while (CSL_FEXT(dssCtrl->DSS_L3RAM_MEMINIT_STATUS, DSS_CTRL_DSS_L3RAM_MEMINIT_STATUS_DSS_L3RAM_MEMINIT_STATUS_L3RAM3_MEMINIT_STATUS) != 0);
        while (CSL_FEXT(dssCtrl->DSS_L3RAM_MEMINIT_DONE, DSS_CTRL_DSS_L3RAM_MEMINIT_DONE_DSS_L3RAM_MEMINIT_DONE_L3RAM3_MEMINIT_DONE) != 0);
    }
}

static inline CSL_mss_rcmRegs* CSL_RCM_getBaseAddress (void)
{
    return (CSL_mss_rcmRegs*) CSL_MSS_RCM_U_BASE;
}

void SOC_rcmConfigEthMacIf(void)
{
    CSL_mss_rcmRegs *ptrMSSRCMRegs;
    uint32_t clkFreq = 0U;
    SOC_RcmXtalFreqId clkFreqId;
    uint32_t Finp;
    uint32_t clkDivisor;
    uint32_t mii10ClkDivVal;


    clkFreqId = SOC_rcmGetXtalFrequency ();
    Finp = gSocRcmXtalInfo[clkFreqId].Finp;

    ptrMSSRCMRegs = CSL_RCM_getBaseAddress ();
    clkFreq = SOC_rcmGetCoreHsDivOut(Finp, gSocRcmXtalInfo[clkFreqId].div2flag, SOC_RcmPllHsDivOutId_1);
    clkDivisor = SOC_rcmGetModuleClkDivVal(clkFreq, SOC_RCM_FREQ_MHZ2HZ(50U));
    ptrMSSRCMRegs->MSS_MII100_CLK_DIV_VAL = SOC_rcmInsert16 (ptrMSSRCMRegs->MSS_MII100_CLK_DIV_VAL, 11U, 0U, SOC_rcmGetModuleClkDivRegVal(clkDivisor));
    clkDivisor = SOC_rcmGetModuleClkDivVal(clkFreq, SOC_RCM_FREQ_MHZ2HZ(5U));
    mii10ClkDivVal = (clkDivisor & 0xFF) | ((clkDivisor & 0xFF) << 8) | ((clkDivisor & 0xFF) << 16);
    ptrMSSRCMRegs->MSS_MII10_CLK_DIV_VAL  = SOC_rcmInsert32 (ptrMSSRCMRegs->MSS_MII10_CLK_DIV_VAL, 23U, 0U, mii10ClkDivVal);
    clkDivisor = SOC_rcmGetModuleClkDivVal(clkFreq, SOC_RCM_FREQ_MHZ2HZ(50U));
    ptrMSSRCMRegs->MSS_RGMII_CLK_DIV_VAL = SOC_rcmInsert16 (ptrMSSRCMRegs->MSS_RGMII_CLK_DIV_VAL, 11U, 0U, SOC_rcmGetModuleClkDivRegVal(clkDivisor));
}

void SOC_generateSwWarmReset(void)
{
    CSL_mss_toprcmRegs *ptrTOPRCMRegs;

    ptrTOPRCMRegs = SOC_rcmGetBaseAddressTOPRCM();

    /* Unlock CONTROLSS_CTRL registers */
    SOC_controlModuleUnlockMMR(SOC_DOMAIN_ID_MSS_TOP_RCM, 0);

    /* Assert SW Warm Reset */
    CSL_FINS(ptrTOPRCMRegs->WARM_RESET_CONFIG, MSS_TOPRCM_WARM_RESET_CONFIG_WARM_RESET_CONFIG_SW_RST, 0x0);

    /* Lock CONTROLSS_CTRL registers */
    SOC_controlModuleLockMMR(SOC_DOMAIN_ID_MSS_TOP_RCM, 0);

    /* execute wfi */
#if defined(__ARM_ARCH_7R__)
    __wfi();
#endif
}

SOC_WarmResetCause SOC_getWarmResetCause(void)
{
    CSL_mss_toprcmRegs *ptrTOPRCMRegs;
    uint16_t     resetCause = 0U;

    ptrTOPRCMRegs = SOC_rcmGetBaseAddressTOPRCM();

    /* Unlock CONTROLSS_CTRL registers */
    SOC_controlModuleUnlockMMR(SOC_DOMAIN_ID_MSS_TOP_RCM, 0);

    /* Read the Reset Cause Register bits */
    resetCause = SOC_rcmExtract8 (ptrTOPRCMRegs->SYS_RST_CAUSE, 4U, 0U);

    /* clear the reset cause */
    CSL_FINS(ptrTOPRCMRegs->SYS_RST_CAUSE_CLR, MSS_TOPRCM_SYS_RST_CAUSE_CLR_SYS_RST_CAUSE_CLR_CLEAR, 0x1);

    /* Lock CONTROLSS_CTRL registers */
    SOC_controlModuleLockMMR(SOC_DOMAIN_ID_MSS_TOP_RCM, 0);

    return (SOC_WarmResetCause) resetCause;
}

void SOC_clearWarmResetCause(void)
{
    CSL_mss_toprcmRegs *ptrTOPRCMRegs;

    ptrTOPRCMRegs = SOC_rcmGetBaseAddressTOPRCM();

    /* Unlock CONTROLSS_CTRL registers */
    SOC_controlModuleUnlockMMR(SOC_DOMAIN_ID_MSS_TOP_RCM, 0);

    CSL_FINS(ptrTOPRCMRegs->SYS_RST_CAUSE_CLR, MSS_TOPRCM_SYS_RST_CAUSE_CLR_SYS_RST_CAUSE_CLR_CLEAR, 0x1);

    /* Lock CONTROLSS_CTRL registers */
    SOC_controlModuleLockMMR(SOC_DOMAIN_ID_MSS_TOP_RCM, 0);

}
