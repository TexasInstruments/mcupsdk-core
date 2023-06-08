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

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <drivers/hw_include/cslr_soc.h>
#include <drivers/soc.h>
#include <kernel/dpl/ClockP.h>
#if defined(__ARM_ARCH_7R__)
#ifdef __ARM_ACLE
#include <arm_acle.h>
#endif /* __ARM_ACLE */
#endif

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

#define SOC_RCM_FREQ_HZ2MHZ(hz)     ((hz)/(1000000U))
#define SOC_RCM_FREQ_MHZ2HZ(mhz)    ((mhz)*(1000000U))

/**
* FROM     ROW   Fields               Start Bit   End Bit  # of bits
* FROM1    10    XTAL_FREQ_SCALE        25          25       1
* FROM1    11    PKG_TYPE                0           2       3
* FROM1    11    L3_MEM_SIZE             7          10       4
* FROM1    11    DUALCORE_BOOT_ENABLE   13          13       1
* FROM1    11    DUALCORE_SWITCH_DISABLE 14         14      1
* FROM1    11    FLASH_MODE             15          16       2
* FROM1    11    QSPI_CLK_FREQ          17          18       2
* FROM1    14    PG_VER                  0           3       4
* FROM1    40    CORE_ADPLL_TRIM         0          11       12
* FROM1    40    DSP_ADPLL_TRIM         12          23       12
* FROM1    41    PER_ADPLL_TRIM          0          11       12
* FROM1    41    CORE_ADPLL_TRIM_VALID  12          12       1
* FROM1    41    DSP_ADPLL_TRIM_VALID   13          13       1
* FROM1    41    PER_ADPLL_TRIM_VALID   14          14       1
* FROM1    43    BOOT_FREQUENCY_SELECT   8          11       4
*/
#define SOC_RCM_EFUSE_FIELD_EXTRACT(topCtrlReg, fromNum, row, startBit, endBit)  SOC_rcmExtract8((topCtrlReg)->EFUSE##fromNum##_ROW_##row, endBit, startBit)
#define SOC_RCM_EFUSE_FIELD_EXTRACT_SIZE16(topCtrlReg, fromNum, row, startBit, endBit)  SOC_rcmExtract16((topCtrlReg)->EFUSE##fromNum##_ROW_##row, endBit, startBit)
#define SOC_RCM_EFUSE_FIELD_EXTRACT_XTAL_FREQ_SCALE(topCtrlReg)          SOC_RCM_EFUSE_FIELD_EXTRACT(topCtrlReg, 1, 10, 25,  25)
#define SOC_RCM_EFUSE_FIELD_EXTRACT_PKG_TYPE(topCtrlReg)                 SOC_RCM_EFUSE_FIELD_EXTRACT(topCtrlReg, 1, 11, 0,   2)
#define SOC_RCM_EFUSE_FIELD_EXTRACT_L3_MEM_SIZE(topCtrlReg)              SOC_RCM_EFUSE_FIELD_EXTRACT(topCtrlReg, 1, 11, 7,  10)
#define SOC_RCM_EFUSE_FIELD_EXTRACT_FLASH_MODE(topCtrlReg)               SOC_RCM_EFUSE_FIELD_EXTRACT(topCtrlReg, 1, 11, 15, 16)
#define SOC_RCM_EFUSE_FIELD_EXTRACT_QSPI_CLK_FREQ(topCtrlReg)            SOC_RCM_EFUSE_FIELD_EXTRACT(topCtrlReg, 1, 11, 17, 18)
#define SOC_RCM_EFUSE_FIELD_EXTRACT_PG_VER(topCtrlReg)                   SOC_RCM_EFUSE_FIELD_EXTRACT(topCtrlReg, 1, 14,  0,  3)
#define SOC_RCM_EFUSE_FIELD_EXTRACT_BOOT_FREQ(topCtrlReg)                SOC_RCM_EFUSE_FIELD_EXTRACT(topCtrlReg, 1, 43,  8, 11)
#define SOC_RCM_EFUSE_FIELD_EXTRACT_CORE_ADPLL_TRIM(topCtrlReg)          SOC_RCM_EFUSE_FIELD_EXTRACT(topCtrlReg, 1, 40, 0,  11)
#define SOC_RCM_EFUSE_FIELD_EXTRACT_DSP_ADPLL_TRIM(topCtrlReg)           SOC_RCM_EFUSE_FIELD_EXTRACT(topCtrlReg, 1, 40, 12, 23)
#define SOC_RCM_EFUSE_FIELD_EXTRACT_PER_ADPLL_TRIM(topCtrlReg)           SOC_RCM_EFUSE_FIELD_EXTRACT(topCtrlReg, 1, 41, 0,  11)
#define SOC_RCM_EFUSE_FIELD_EXTRACT_CORE_ADPLL_TRIM_VALID(topCtrlReg)    SOC_RCM_EFUSE_FIELD_EXTRACT(topCtrlReg, 1, 41, 12, 12)
#define SOC_RCM_EFUSE_FIELD_EXTRACT_DSP_ADPLL_TRIM_VALID(topCtrlReg)     SOC_RCM_EFUSE_FIELD_EXTRACT(topCtrlReg, 1, 41, 13, 13)
#define SOC_RCM_EFUSE_FIELD_EXTRACT_PER_ADPLL_TRIM_VALID(topCtrlReg)     SOC_RCM_EFUSE_FIELD_EXTRACT(topCtrlReg, 1, 41, 14, 14)
#define SOC_RCM_EFUSE_FIELD_EXTRACT_DUALCORE_BOOT_ENABLE(topCtrlReg)     SOC_RCM_EFUSE_FIELD_EXTRACT(topCtrlReg, 1, 11, 13, 13)
#define SOC_RCM_EFUSE_FIELD_EXTRACT_DUALCORE_SWITCH_DISABLE(topCtrlReg)  SOC_RCM_EFUSE_FIELD_EXTRACT(topCtrlReg, 1, 11, 14, 14)

#define SOC_RCM_CORE_ADPLL_DEFAULT_VALUE            (0x9U)
#define SOC_RCM_DSP_ADPLL_DEFAULT_VALUE             (0x9U)
#define SOC_RCM_PER_ADPLL_DEFAULT_VALUE             (0x9U)

#define SOC_RCM_XTAL_CLK_40MHZ                      (SOC_RCM_FREQ_MHZ2HZ(40U))
#define SOC_RCM_XTAL_CLK_50MHZ                      (SOC_RCM_FREQ_MHZ2HZ(50U))
#define SOC_RCM_XTAL_CLK_49p152MHZ                  (49152000U)
#define SOC_RCM_XTAL_CLK_45p1584MHZ                 (45158400U)
#define SOC_RCM_XTAL_CLK_20MHZ                      (SOC_RCM_FREQ_MHZ2HZ(20U))
#define SOC_RCM_XTAL_CLK_25MHZ                      (SOC_RCM_FREQ_MHZ2HZ(25U))
#define SOC_RCM_XTAL_CLK_24p576MHZ                  (24576000U)
#define SOC_RCM_XTAL_CLK_22p5792MHZ                 (22579200U)

#define SOC_RCM_UTILS_ARRAYSIZE(x)                  (sizeof(x)/sizeof(x[0]))

#define SOC_RCM_EFUSE_DEVICE_PKG_TYPE_ETS_VAL       (0x2)

#define SOC_RCM_DEVICE_PKG_TYPE_DSP_FREQ_ETS        (SOC_RCM_FREQ_MHZ2HZ(360U))
#define SOC_RCM_DEVICE_PKG_TYPE_R5_FREQ_ETS         (SOC_RCM_FREQ_MHZ2HZ(300U))
#define SOC_RCM_DEVICE_PKG_TYPE_SYSCLK_FREQ_ETS     (SOC_RCM_DEVICE_PKG_TYPE_R5_FREQ_ETS / 2U)

typedef enum SOC_RcmXtalFreqId_e
{
    SOC_RCM_XTAL_FREQID_CLK_40MHZ,
    SOC_RCM_XTAL_FREQID_CLK_50MHZ,
    SOC_RCM_XTAL_FREQID_CLK_49p152MHZ,
    SOC_RCM_XTAL_FREQID_CLK_45p1584MHZ,
    SOC_RCM_XTAL_FREQID_CLK_20MHZ,
    SOC_RCM_XTAL_FREQID_CLK_25MHZ,
    SOC_RCM_XTAL_FREQID_CLK_24p576MHZ,
    SOC_RCM_XTAL_FREQID_CLK_22p5792MHZ
} SOC_RcmXtalFreqId;

typedef enum SOC_RcmFixedClockId_e
{
    SOC_RCM_FIXEDCLKID_ANA_HSI_CLK,
    SOC_RCM_FIXEDCLKID_FE1_REF_CLK,
    SOC_RCM_FIXEDCLKID_DSS_DSP_DITHERED_CLK,
    SOC_RCM_FIXEDCLKID_XREF_CLK0,
    SOC_RCM_FIXEDCLKID_XREF_CLK1,
    SOC_RCM_FIXEDCLKID_WUCPU_CLK,
    SOC_RCM_FIXEDCLKID_RCCLK32K,
    SOC_RCM_FIXEDCLKID_RC_CLK_10M,
    SOC_RCM_FIXEDCLKID_APLL1P8GHZ,
    SOC_RCM_FIXEDCLKID_APLL1P2GHZ,
} SOC_RcmFixedClockId;

typedef enum SOC_RcmClockSrcId_e
{
    SOC_RCM_CLKSRCID_APLL_1p2G_HSDIV0_CLKOUT0,
    SOC_RCM_CLKSRCID_APLL_1p2G_HSDIV0_CLKOUT1,
    SOC_RCM_CLKSRCID_APLL_1p2G_HSDIV0_CLKOUT2,
    SOC_RCM_CLKSRCID_APLL_1p2G_HSDIV0_CLKOUT3,
    SOC_RCM_CLKSRCID_APLL_1p8G_HSDIV0_CLKOUT0,
    SOC_RCM_CLKSRCID_APLL_1p8G_HSDIV0_CLKOUT1,
    SOC_RCM_CLKSRCID_APLL_1p8G_HSDIV0_CLKOUT2,
    SOC_RCM_CLKSRCID_APLL_1p8GHZ,
    SOC_RCM_CLKSRCID_APLL_400MHZ,
    SOC_RCM_CLKSRCID_APLL_CLK_HSI,
    SOC_RCM_CLKSRCID_DPLL_CORE_HSDIV0_CLKOUT0,
    SOC_RCM_CLKSRCID_DPLL_CORE_HSDIV0_CLKOUT1,
    SOC_RCM_CLKSRCID_DPLL_CORE_HSDIV0_CLKOUT2,
    SOC_RCM_CLKSRCID_DPLL_CORE_HSDIV0_CLKOUT2_PreMux,
    SOC_RCM_CLKSRCID_DPLL_DSP_HSDIV0_CLKOUT0,
    SOC_RCM_CLKSRCID_DPLL_DSP_HSDIV0_CLKOUT1,
    SOC_RCM_CLKSRCID_DPLL_DSP_HSDIV0_CLKOUT1_DITH,
    SOC_RCM_CLKSRCID_DPLL_DSP_HSDIV0_CLKOUT1_PreMux,
    SOC_RCM_CLKSRCID_DPLL_DSP_HSDIV0_CLKOUT2,
    SOC_RCM_CLKSRCID_DPLL_DSP_HSDIV0_CLKOUT2_PreMux,
    SOC_RCM_CLKSRCID_DPLL_PER_HSDIV0_CLKOUT0,
    SOC_RCM_CLKSRCID_DPLL_PER_HSDIV0_CLKOUT1,
    SOC_RCM_CLKSRCID_DPLL_PER_HSDIV0_CLKOUT1_PreMux,
    SOC_RCM_CLKSRCID_DPLL_PER_HSDIV0_CLKOUT2,
    SOC_RCM_CLKSRCID_DPLL_PER_HSDIV0_CLKOUT3,
    SOC_RCM_CLKSRCID_FE1_REF_CLK,
    SOC_RCM_CLKSRCID_PLL_CORE_CLK,
    SOC_RCM_CLKSRCID_PLL_PER_CLK,
    SOC_RCM_CLKSRCID_RC_CLK_10M,
    SOC_RCM_CLKSRCID_RCCLK32K,
    SOC_RCM_CLKSRCID_SYS_CLK,
    SOC_RCM_CLKSRCID_WUCPU_CLK,
    SOC_RCM_CLKSRCID_XREF_CLK0,
    SOC_RCM_CLKSRCID_XREF_CLK1,
    SOC_RCM_CLKSRCID_XTAL_CLK,
} SOC_RcmClockSrcId;

typedef enum SOC_RcmClockSrcType_e
{
    SOC_RCM_CLKSRCTYPE_XTAL,
    SOC_RCM_CLKSRCTYPE_FIXED,
    SOC_RCM_CLKSRCTYPE_APLL_HSDIVOUT,
    SOC_RCM_CLKSRCTYPE_DPLL_HSDIVOUT,
    SOC_RCM_CLKSRCTYPE_DPLL_HSDIVOUTMUX,
} SOC_RcmClockSrcType;

typedef enum SOC_RcmPllHSDIVOutId_e
{
    SOC_RCM_PLLHSDIV_OUT_0,
    SOC_RCM_PLLHSDIV_OUT_1,
    SOC_RCM_PLLHSDIV_OUT_2,
    SOC_RCM_PLLHSDIV_OUT_3,
    SOC_RCM_PLLHSDIV_OUT_NONE,
} SOC_RcmPllHSDIVOutId;

typedef enum SOC_RcmCpswMiiClockSourceId_e
{
    SOC_RcmCpswMiiClockSource_DPLL_CORE_HSDIV0_CLKOUT1,
    SOC_RcmCpswMiiClockSource_DPLL_PER_HSDIV0_CLKOUT1_MUXED,
    SOC_RcmCpswMiiClockSource_SYS_CLK,
    SOC_RcmCpswMiiClockSource_RC_CLK_10M,
} SOC_RcmCpswMiiClockSourceId;

/* ========================================================================== */
/*                         Structure Declarations                             */
/* ========================================================================== */

typedef struct SOC_RcmDpllHsDivInfo_s
{
    SOC_RcmDpllId_e dpllId;
    SOC_RcmPllHSDIVOutId hsdivOut;
} SOC_RcmDpllHsDivInfo;

typedef struct SOC_RcmApllHsDivInfo_s
{
    SOC_RcmApllId apllId;
    SOC_RcmPllHSDIVOutId hsdivOut;
} SOC_RcmApllHsDivInfo;

typedef struct SOC_RcmHsDivOutMuxInfo_s
{
    SOC_RcmHSDIVClkOutMuxId hsdivMuxId;
} SOC_RcmHsDivOutMuxInfo;

typedef struct SOC_RcmFixedClkInfo_s
{
    SOC_RcmFixedClockId fixedClkId;
} SOC_RcmFixedClkInfo;

typedef struct SOC_RcmClkSrcInfo_s
{
    SOC_RcmClockSrcType clkSrcType;
    union {
        SOC_RcmFixedClkInfo  fixedClkInfo;
        SOC_RcmApllHsDivInfo apllHsDivInfo;
        SOC_RcmDpllHsDivInfo dpllHsDivInfo;
        SOC_RcmHsDivOutMuxInfo hsdivMuxInfo;
    } u;
} SOC_RcmClkSrcInfo;

typedef struct SOC_RcmXTALInfo_s
{
    uint32_t Finp;
    uint32_t div2flag;
} SOC_RcmXTALInfo;

typedef struct SOC_RcmFixedClocks_s
{
    uint32_t       fOut;
} SOC_RcmFixedClocks_t;

typedef struct SOC_RcmADPLLJConfig_s
{
    uint32_t N; /* Input Clock divider/Pre-divider (N) */
    uint32_t M2; /* Post divider (M2) */
    uint32_t M;  /* Multiplier integer (M) */
    uint32_t FracM; /* Multiplier fractional (M) */
    uint32_t Fout; /* Output frequency of PLL */
    uint32_t Finp; /* Output frequency of PLL */
    uint32_t sdDiv; /* Sigma Delta Divider Value */
} SOC_RcmADPLLJConfig_t;

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */

static const SOC_RcmDeviceFreqConfig gSocRcmDeviceFreqConfigTbl[] =
{
    [SOC_RCM_EFUSE_DEVICE_PKG_TYPE_ETS] =
    {
        .dspFreqHz    = SOC_RCM_DEVICE_PKG_TYPE_DSP_FREQ_ETS,
        .r5FreqHz     = SOC_RCM_DEVICE_PKG_TYPE_R5_FREQ_ETS,
        .sysClkFreqHz = SOC_RCM_DEVICE_PKG_TYPE_SYSCLK_FREQ_ETS,
    },
};

/* Table populated from AWR294x_ADPLLJ_Settings_1p0.xlsx.
 * Each entry corresponds to tab in the excel sheet
 */
static const SOC_RcmADPLLJConfig_t gSocRcmADPLLJConfigTbl[] =
{
    /* CORE_DSP_800_40MHz */
    {
        .Finp = 40U,
        .N = 19U,
        .Fout = 800U,
        .M2 = 1U,
        .M = 400U,
        .FracM = 0U,
        .sdDiv = 4U,
    },
    /* CORE_DSP_2000_40MHz */
    {
        .Finp = 40U,
        .N = 19U,
        .Fout = 2000U,
        .M2 = 1U,
        .M = 1000U,
        .FracM = 0U,
        .sdDiv = 8U,
    },
    /* DSP_900_40MHz */
    {
        .Finp = 40U,
        .N = 19U,
        .Fout = 900U,
        .M2 = 1U,
        .M = 450U,
        .FracM = 0U,
        .sdDiv = 4U,
    },
    /* DSP_1800_40MHz */
    {
        .Finp = 40U,
        .N = 19U,
        .Fout = 1800U,
        .M2 = 1U,
        .M = 900U,
        .FracM = 0U,
        .sdDiv = 8U,
    },
    /* PER_1920_40MHz */
    {
        .Finp = 40U,
        .N = 19U,
        .Fout = 1920U,
        .M2 = 1U,
        .M = 960U,
        .FracM = 0U,
        .sdDiv = 8U,
    },
    /* CORE_DSP_800_50MHz */
    {
        .Finp = 50U,
        .N = 24U,
        .Fout = 800U,
        .M2 = 1U,
        .M = 400U,
        .FracM = 0U,
        .sdDiv = 4U,
    },
    /* CORE_DSP_2000_50MHz */
    {
        .Finp = 50U,
        .N = 24U,
        .Fout = 2000U,
        .M2 = 1U,
        .M = 1000U,
        .FracM = 0U,
        .sdDiv = 8U,
    },
    /* DSP_1800_50MHz */
    {
        .Finp = 50U,
        .N = 24U,
        .Fout = 1800U,
        .M2 = 1U,
        .M = 900U,
        .FracM = 0U,
        .sdDiv = 8U,
    },
    /* PER_1699_40MHz */
    {
        .Finp = 40U,
        .N = 16U,
        .Fout = 1699U,
        .M2 = 1U,
        .M = 722U,
        .FracM = 44032U,
        .sdDiv = 7U,
    },
    /* PER_1920_50MHz */
    {
        .Finp = 50U,
        .N = 24U,
        .Fout = 1920U,
        .M2 = 1U,
        .M = 960U,
        .FracM = 0U,
        .sdDiv = 8U,
    },
    /* CORE_800_49152 */
    {
        .Finp = 49U,
        .N = 23U,
        .Fout = 800U,
        .M2 = 1U,
        .M = 390U,
        .FracM = 163840U,
        .sdDiv = 4U,
    },
    /* CORE_2000_49152 */
    {
        .Finp = 49U,
        .N = 21U,
        .Fout = 2000U,
        .M2 = 1U,
        .M = 895U,
        .FracM = 47787U,
        .sdDiv = 8U,
    },
    /* DSP_1800_49152 */
    {
        .Finp = 49U,
        .N = 20U,
        .Fout = 1800U,
        .M2 = 1U,
        .M = 769U,
        .FracM = 11264U,
        .sdDiv = 8U,
    },
    /* DSP_1728_49152 */
    {
        .Finp = 49U,
        .N = 19U,
        .Fout = 1728U,
        .M2 = 1U,
        .M = 703U,
        .FracM = 32768U,
        .sdDiv = 7U,
    },
    /* PER_1966p08_49152 */
    {
        .Finp = 49U,
        .N = 19U,
        .Fout = 1966U,
        .M2 = 1U,
        .M = 800U,
        .FracM = 209715200U,
        .sdDiv = 8U,
    },
    /* CORE_800_451584 */
    {
        .Finp = 45U,
        .N = 20U,
        .Fout = 800U,
        .M2 = 1U,
        .M = 372U,
        .FracM = 6242U,
        .sdDiv = 4U,
    },
    /* CORE_2000_451584 */
    {
        .Finp = 45U,
        .N = 20U,
        .Fout = 2000U,
        .M2 = 1U,
        .M = 930U,
        .FracM = 15604U,
        .sdDiv = 8U,
    },
    /* DSP_1800_451584 */
    {
        .Finp = 45U,
        .N = 20U,
        .Fout = 1800U,
        .M2 = 1U,
        .M = 837U,
        .FracM = 14043U,
        .sdDiv = 8U,
    },
    /* DSP_1728_451584 */
    {
        .Finp = 45U,
        .N = 18U,
        .Fout = 1728U,
        .M2 = 1U,
        .M = 727U,
        .FracM = 10700U,
        .sdDiv = 7U,
    },
    /* PER_1806p336_451584 */
    {
        .Finp = 45U,
        .N = 18U,
        .Fout = 1806U,
        .M2 = 1U,
        .M = 760U,
        .FracM = 0U,
        .sdDiv = 8U,
    },
    /* CORE_600_40MHz */
    {
        .Finp = 40U,
        .N = 19U,
        .Fout = 600U,
        .M2 = 1U,
        .M = 300U,
        .FracM = 0U,
        .sdDiv = 3U,
    },
    /* CORE_600_50MHz */
    {
        .Finp = 50U,
        .N = 24U,
        .Fout = 600U,
        .M2 = 1U,
        .M = 300U,
        .FracM = 0U,
        .sdDiv = 3U,
    },
    /* CORE_600_49152 */
    {
        .Finp = 49U,
        .N = 23U,
        .Fout = 600U,
        .M2 = 1U,
        .M = 292U,
        .FracM = 253952U,
        .sdDiv = 3U,
    },
    /* CORE_600_451584 */
    {
        .Finp = 45U,
        .N = 20U,
        .Fout = 600U,
        .M2 = 1U,
        .M = 279U,
        .FracM = 4681U,
        .sdDiv = 3U,
    },
    /* CORE_720_40MHz */
    {
        .Finp = 40U,
        .N = 19U,
        .Fout = 720U,
        .M2 = 1U,
        .M = 360U,
        .FracM = 0U,
        .sdDiv = 3U,
    },
    /* DSP_1500_40MHz */
    {
        .Finp = 40U,
        .N = 19U,
        .Fout = 1500U,
        .M2 = 1U,
        .M = 750,
        .FracM = 0U,
        .sdDiv = 6U,
    }
};

static const SOC_RcmFixedClocks_t gSocRcmFixedClocksTbl[] =
{
    [SOC_RCM_FIXEDCLKID_ANA_HSI_CLK] =
    {
        /* ANA_HSI_CLK_TO_DIG	400	400	400	400.000 */
        .fOut = SOC_RCM_FREQ_MHZ2HZ(400U),
    },
    [SOC_RCM_FIXEDCLKID_FE1_REF_CLK] =
    {
        /* FE1_REF_CLK	0.000	0.000	0.000	0.000	Tied OFF. Not used in 29xx */
        .fOut = SOC_RCM_FREQ_MHZ2HZ(0U),
    },
    [SOC_RCM_FIXEDCLKID_DSS_DSP_DITHERED_CLK] =
    {
        /* DSS_DSP_DITHERED_CLK	500.000	500.000	500.000	500.000 */
        .fOut = SOC_RCM_FREQ_MHZ2HZ(500U),
    },
    [SOC_RCM_FIXEDCLKID_XREF_CLK0] =
    {
        /* XREF_CLK0	64.100	64.100	64.100	64.000 */
        .fOut = 64100000U,
    },
    [SOC_RCM_FIXEDCLKID_XREF_CLK1] =
    {
        /* XREF_CLK1	64.100	64.100	64.100	64.000 */
        .fOut = 64100000U,
    },
    [SOC_RCM_FIXEDCLKID_WUCPU_CLK] =
    {
        /* WUCPU_CLK	50.000	50.000	50.000	50.000 */
        .fOut = SOC_RCM_FREQ_MHZ2HZ(50),
    },
    [SOC_RCM_FIXEDCLKID_RCCLK32K] =
    {
        /* RCCLK32K	0.032	0.032	0.032	0.032 */
        .fOut = 32000U,
    },
    [SOC_RCM_FIXEDCLKID_RC_CLK_10M] =
    {
        /* RC_CLK_10M	10.000	10.000	10.000	10.000 */
        .fOut = SOC_RCM_FREQ_MHZ2HZ(10U),
    },
    [SOC_RCM_FIXEDCLKID_APLL1P8GHZ] =
    {
        /*ANA_PLL_1800	1800	1800	1800	1800 */
        .fOut = SOC_RCM_FREQ_MHZ2HZ(1800U),
    },
    [SOC_RCM_FIXEDCLKID_APLL1P2GHZ] =
    {
        /*ANA_PLL_1200	1200	1200	1200	1200 */
        .fOut = SOC_RCM_FREQ_MHZ2HZ(1200U),
    },
};

static const SOC_RcmXTALInfo gSocRcmXTALInfo[] =
{
    [SOC_RCM_XTAL_FREQID_CLK_40MHZ]      = {.Finp = SOC_RCM_FREQ_HZ2MHZ(SOC_RCM_XTAL_CLK_40MHZ),      .div2flag = FALSE},
    [SOC_RCM_XTAL_FREQID_CLK_50MHZ]      = {.Finp = SOC_RCM_FREQ_HZ2MHZ(SOC_RCM_XTAL_CLK_50MHZ),      .div2flag = FALSE},
    [SOC_RCM_XTAL_FREQID_CLK_49p152MHZ]  = {.Finp = SOC_RCM_FREQ_HZ2MHZ(SOC_RCM_XTAL_CLK_49p152MHZ),  .div2flag = FALSE},
    [SOC_RCM_XTAL_FREQID_CLK_45p1584MHZ] = {.Finp = SOC_RCM_FREQ_HZ2MHZ(SOC_RCM_XTAL_CLK_45p1584MHZ), .div2flag = FALSE},
    [SOC_RCM_XTAL_FREQID_CLK_20MHZ]      = {.Finp = SOC_RCM_FREQ_HZ2MHZ(SOC_RCM_XTAL_CLK_40MHZ),      .div2flag = TRUE},
    [SOC_RCM_XTAL_FREQID_CLK_25MHZ]      = {.Finp = SOC_RCM_FREQ_HZ2MHZ(SOC_RCM_XTAL_CLK_50MHZ),      .div2flag = TRUE},
    [SOC_RCM_XTAL_FREQID_CLK_24p576MHZ]  = {.Finp = SOC_RCM_FREQ_HZ2MHZ(SOC_RCM_XTAL_CLK_49p152MHZ),  .div2flag = TRUE},
    [SOC_RCM_XTAL_FREQID_CLK_22p5792MHZ] = {.Finp = SOC_RCM_FREQ_HZ2MHZ(SOC_RCM_XTAL_CLK_50MHZ),      .div2flag = TRUE},
};

static const uint32_t gSocRcmXTALFreqTbl[] =
{
    [SOC_RCM_XTAL_FREQID_CLK_40MHZ]      = SOC_RCM_XTAL_CLK_40MHZ,
    [SOC_RCM_XTAL_FREQID_CLK_50MHZ]      = SOC_RCM_XTAL_CLK_50MHZ,
    [SOC_RCM_XTAL_FREQID_CLK_49p152MHZ]  = SOC_RCM_XTAL_CLK_49p152MHZ,
    [SOC_RCM_XTAL_FREQID_CLK_45p1584MHZ] = SOC_RCM_XTAL_CLK_45p1584MHZ,
    [SOC_RCM_XTAL_FREQID_CLK_20MHZ]      = SOC_RCM_XTAL_CLK_40MHZ,
    [SOC_RCM_XTAL_FREQID_CLK_25MHZ]      = SOC_RCM_XTAL_CLK_50MHZ,
    [SOC_RCM_XTAL_FREQID_CLK_24p576MHZ]  = SOC_RCM_XTAL_CLK_49p152MHZ,
    [SOC_RCM_XTAL_FREQID_CLK_22p5792MHZ] = SOC_RCM_XTAL_CLK_50MHZ,
};

static const uint32_t gSocRcmPLLFreqId2FOutMap[] =
{
    [SOC_RcmPllFoutFreqId_CLK_800MHZ]        = 800U,
    [SOC_RcmPllFoutFreqId_CLK_900MHZ]        = 900U,
    [SOC_RcmPllFoutFreqId_CLK_2000MHZ]       = 2000U,
    [SOC_RcmPllFoutFreqId_CLK_1800MHZ]       = 1800U,
    [SOC_RcmPllFoutFreqId_CLK_1920MHZ]       = 1920U,
    [SOC_RcmPllFoutFreqId_CLK_1699p21875MHZ] = 1699U,
    [SOC_RcmPllFoutFreqId_CLK_1728MHZ]       = 1728U,
    [SOC_RcmPllFoutFreqId_CLK_1966p08MHZ]    = 1966U,
    [SOC_RcmPllFoutFreqId_CLK_1806p336MHZ]   = 1806U,
    [SOC_RcmPllFoutFreqId_CLK_600MHZ]        = 600U,
    [SOC_RcmPllFoutFreqId_CLK_720MHZ]        = 720U,
    [SOC_RcmPllFoutFreqId_CLK_1500MHZ]       = 1500U,
};

static const SOC_RcmClkSrcInfo gSocRcmClkSrcInfoMap[] =
{
    [SOC_RCM_CLKSRCID_APLL_1p2G_HSDIV0_CLKOUT0] =
    {
        .clkSrcType = SOC_RCM_CLKSRCTYPE_APLL_HSDIVOUT,
        .u.apllHsDivInfo =
        {
            .apllId = SOC_RcmAPLLID_1P2G,
            .hsdivOut = SOC_RCM_PLLHSDIV_OUT_0,
        },
    },
    [SOC_RCM_CLKSRCID_APLL_1p2G_HSDIV0_CLKOUT1] =
    {
        .clkSrcType = SOC_RCM_CLKSRCTYPE_APLL_HSDIVOUT,
        .u.apllHsDivInfo =
        {
            .apllId = SOC_RcmAPLLID_1P2G,
            .hsdivOut = SOC_RCM_PLLHSDIV_OUT_1,
        },
    },
    [SOC_RCM_CLKSRCID_APLL_1p2G_HSDIV0_CLKOUT2] =
    {
        .clkSrcType = SOC_RCM_CLKSRCTYPE_APLL_HSDIVOUT,
        .u.apllHsDivInfo =
        {
            .apllId = SOC_RcmAPLLID_1P2G,
            .hsdivOut = SOC_RCM_PLLHSDIV_OUT_2,
        },
    },
    [SOC_RCM_CLKSRCID_APLL_1p2G_HSDIV0_CLKOUT3] =
    {
        .clkSrcType = SOC_RCM_CLKSRCTYPE_APLL_HSDIVOUT,
        .u.apllHsDivInfo =
        {
            .apllId = SOC_RcmAPLLID_1P2G,
            .hsdivOut = SOC_RCM_PLLHSDIV_OUT_3,
        },
    },
    [SOC_RCM_CLKSRCID_APLL_1p8G_HSDIV0_CLKOUT0] =
    {
        .clkSrcType = SOC_RCM_CLKSRCTYPE_APLL_HSDIVOUT,
        .u.apllHsDivInfo =
        {
            .apllId = SOC_RcmAPLLID_1P8G,
            .hsdivOut = SOC_RCM_PLLHSDIV_OUT_0,
        },
    },
    [SOC_RCM_CLKSRCID_APLL_1p8G_HSDIV0_CLKOUT1] =
    {
        .clkSrcType = SOC_RCM_CLKSRCTYPE_APLL_HSDIVOUT,
        .u.apllHsDivInfo =
        {
            .apllId = SOC_RcmAPLLID_1P8G,
            .hsdivOut = SOC_RCM_PLLHSDIV_OUT_1,
        },
    },
    [SOC_RCM_CLKSRCID_APLL_1p8G_HSDIV0_CLKOUT2] =
    {
        .clkSrcType = SOC_RCM_CLKSRCTYPE_APLL_HSDIVOUT,
        .u.apllHsDivInfo =
        {
            .apllId = SOC_RcmAPLLID_1P8G,
            .hsdivOut = SOC_RCM_PLLHSDIV_OUT_2,
        },
    },
    [SOC_RCM_CLKSRCID_DPLL_CORE_HSDIV0_CLKOUT0] =
    {
        .clkSrcType = SOC_RCM_CLKSRCTYPE_DPLL_HSDIVOUT,
        .u.dpllHsDivInfo =
        {
            .dpllId = SOC_RCM_DPLL_CORE,
            .hsdivOut = SOC_RCM_PLLHSDIV_OUT_0,
        },
    },
    [SOC_RCM_CLKSRCID_DPLL_CORE_HSDIV0_CLKOUT1] =
    {
        .clkSrcType = SOC_RCM_CLKSRCTYPE_DPLL_HSDIVOUT,
        .u.dpllHsDivInfo =
        {
            .dpllId = SOC_RCM_DPLL_CORE,
            .hsdivOut = SOC_RCM_PLLHSDIV_OUT_1,
        },
    },
    [SOC_RCM_CLKSRCID_DPLL_CORE_HSDIV0_CLKOUT2_PreMux] =
    {
        .clkSrcType = SOC_RCM_CLKSRCTYPE_DPLL_HSDIVOUT,
        .u.dpllHsDivInfo =
        {
            .dpllId = SOC_RCM_DPLL_CORE,
            .hsdivOut = SOC_RCM_PLLHSDIV_OUT_2,
        },
    },
    [SOC_RCM_CLKSRCID_PLL_CORE_CLK] =
    {
        .clkSrcType = SOC_RCM_CLKSRCTYPE_DPLL_HSDIVOUT,
        .u.dpllHsDivInfo =
        {
            .dpllId = SOC_RCM_DPLL_CORE,
            .hsdivOut = SOC_RCM_PLLHSDIV_OUT_NONE,
        },
    },
    [SOC_RCM_CLKSRCID_DPLL_DSP_HSDIV0_CLKOUT0] =
    {
        .clkSrcType = SOC_RCM_CLKSRCTYPE_DPLL_HSDIVOUT,
        .u.dpllHsDivInfo =
        {
            .dpllId = SOC_RCM_DPLL_DSS,
            .hsdivOut = SOC_RCM_PLLHSDIV_OUT_0,
        },
    },
    [SOC_RCM_CLKSRCID_DPLL_DSP_HSDIV0_CLKOUT1_PreMux] =
    {
        .clkSrcType = SOC_RCM_CLKSRCTYPE_DPLL_HSDIVOUT,
        .u.dpllHsDivInfo =
        {
            .dpllId = SOC_RCM_DPLL_DSS,
            .hsdivOut = SOC_RCM_PLLHSDIV_OUT_1,
        },
    },
    [SOC_RCM_CLKSRCID_DPLL_DSP_HSDIV0_CLKOUT2_PreMux] =
    {
        .clkSrcType = SOC_RCM_CLKSRCTYPE_DPLL_HSDIVOUT,
        .u.dpllHsDivInfo =
        {
            .dpllId = SOC_RCM_DPLL_DSS,
            .hsdivOut = SOC_RCM_PLLHSDIV_OUT_2,
        },
    },
    [SOC_RCM_CLKSRCID_PLL_PER_CLK] =
    {
        .clkSrcType = SOC_RCM_CLKSRCTYPE_DPLL_HSDIVOUT,
        .u.dpllHsDivInfo =
        {
            .dpllId = SOC_RCM_DPLL_PER,
            .hsdivOut = SOC_RCM_PLLHSDIV_OUT_NONE,
        },
    },
    [SOC_RCM_CLKSRCID_DPLL_PER_HSDIV0_CLKOUT0] =
    {
        .clkSrcType = SOC_RCM_CLKSRCTYPE_DPLL_HSDIVOUT,
        .u.dpllHsDivInfo =
        {
            .dpllId = SOC_RCM_DPLL_PER,
            .hsdivOut = SOC_RCM_PLLHSDIV_OUT_0,
        },
    },
    [SOC_RCM_CLKSRCID_DPLL_PER_HSDIV0_CLKOUT1_PreMux] =
    {
        .clkSrcType = SOC_RCM_CLKSRCTYPE_DPLL_HSDIVOUT,
        .u.dpllHsDivInfo =
        {
            .dpllId = SOC_RCM_DPLL_PER,
            .hsdivOut = SOC_RCM_PLLHSDIV_OUT_1,
        },
    },
    [SOC_RCM_CLKSRCID_DPLL_PER_HSDIV0_CLKOUT2] =
    {
        .clkSrcType = SOC_RCM_CLKSRCTYPE_DPLL_HSDIVOUT,
        .u.dpllHsDivInfo =
        {
            .dpllId = SOC_RCM_DPLL_PER,
            .hsdivOut = SOC_RCM_PLLHSDIV_OUT_2,
        },
    },
    [SOC_RCM_CLKSRCID_DPLL_PER_HSDIV0_CLKOUT3] =
    {
        .clkSrcType = SOC_RCM_CLKSRCTYPE_DPLL_HSDIVOUT,
        .u.dpllHsDivInfo =
        {
            .dpllId = SOC_RCM_DPLL_PER,
            .hsdivOut = SOC_RCM_PLLHSDIV_OUT_3,
        },
    },
    [SOC_RCM_CLKSRCID_APLL_1p8GHZ] =
    {
        .clkSrcType = SOC_RCM_CLKSRCTYPE_FIXED,
        .u.fixedClkInfo =
        {
            .fixedClkId = SOC_RCM_FIXEDCLKID_APLL1P8GHZ,
        }
    },
    [SOC_RCM_CLKSRCID_APLL_400MHZ] =
    {
        .clkSrcType = SOC_RCM_CLKSRCTYPE_FIXED,
        .u.fixedClkInfo =
        {
            .fixedClkId = SOC_RCM_FIXEDCLKID_ANA_HSI_CLK,
        }
    },
    [SOC_RCM_CLKSRCID_DPLL_DSP_HSDIV0_CLKOUT1_DITH] =
    {
        .clkSrcType = SOC_RCM_CLKSRCTYPE_FIXED,
        .u.fixedClkInfo =
        {
            .fixedClkId = SOC_RCM_FIXEDCLKID_DSS_DSP_DITHERED_CLK,
        }
    },
    [SOC_RCM_CLKSRCID_FE1_REF_CLK] =
    {
        .clkSrcType = SOC_RCM_CLKSRCTYPE_FIXED,
        .u.fixedClkInfo =
        {
            .fixedClkId = SOC_RCM_FIXEDCLKID_FE1_REF_CLK,
        }
    },
    [SOC_RCM_CLKSRCID_RC_CLK_10M] =
    {
        .clkSrcType = SOC_RCM_CLKSRCTYPE_FIXED,
        .u.fixedClkInfo =
        {
            .fixedClkId = SOC_RCM_FIXEDCLKID_RC_CLK_10M,
        }
    },
    [SOC_RCM_CLKSRCID_RCCLK32K] =
    {
        .clkSrcType = SOC_RCM_CLKSRCTYPE_FIXED,
        .u.fixedClkInfo =
        {
            .fixedClkId = SOC_RCM_FIXEDCLKID_RCCLK32K,
        }
    },
    [SOC_RCM_CLKSRCID_XREF_CLK0] =
    {
        .clkSrcType = SOC_RCM_CLKSRCTYPE_FIXED,
        .u.fixedClkInfo =
        {
            .fixedClkId = SOC_RCM_FIXEDCLKID_XREF_CLK0,
        }
    },
    [SOC_RCM_CLKSRCID_XREF_CLK1] =
    {
        .clkSrcType = SOC_RCM_CLKSRCTYPE_FIXED,
        .u.fixedClkInfo =
        {
            .fixedClkId = SOC_RCM_FIXEDCLKID_XREF_CLK1,
        }
    },
    [SOC_RCM_CLKSRCID_WUCPU_CLK] =
    {
        .clkSrcType = SOC_RCM_CLKSRCTYPE_FIXED,
        .u.fixedClkInfo =
        {
            .fixedClkId = SOC_RCM_FIXEDCLKID_WUCPU_CLK,
        }
    },
    [SOC_RCM_CLKSRCID_APLL_CLK_HSI] =
    {
        .clkSrcType = SOC_RCM_CLKSRCTYPE_FIXED,
        .u.fixedClkInfo =
        {
            .fixedClkId = SOC_RCM_FIXEDCLKID_ANA_HSI_CLK,
        }
    },

    [SOC_RCM_CLKSRCID_DPLL_CORE_HSDIV0_CLKOUT2] =
    {
        .clkSrcType = SOC_RCM_CLKSRCTYPE_DPLL_HSDIVOUTMUX,
        .u.hsdivMuxInfo =
        {
            .hsdivMuxId = SOC_RcmHSDIVClkOutMuxId_DPLL_CORE_OUT2,
        }
    },
    [SOC_RCM_CLKSRCID_DPLL_DSP_HSDIV0_CLKOUT1] =
    {
        .clkSrcType = SOC_RCM_CLKSRCTYPE_DPLL_HSDIVOUTMUX,
        .u.hsdivMuxInfo =
        {
            .hsdivMuxId = SOC_RcmHSDIVClkOutMuxId_DPLL_DSP_OUT1,
        }
    },
    [SOC_RCM_CLKSRCID_DPLL_DSP_HSDIV0_CLKOUT2] =
    {
        .clkSrcType = SOC_RCM_CLKSRCTYPE_DPLL_HSDIVOUTMUX,
        .u.hsdivMuxInfo =
        {
            .hsdivMuxId = SOC_RcmHSDIVClkOutMuxId_DPLL_DSP_OUT2,
        }
    },
    [SOC_RCM_CLKSRCID_DPLL_PER_HSDIV0_CLKOUT1] =
    {
        .clkSrcType = SOC_RCM_CLKSRCTYPE_DPLL_HSDIVOUTMUX,
        .u.hsdivMuxInfo =
        {
            .hsdivMuxId = SOC_RcmHSDIVClkOutMuxId_DPLL_PER_OUT1,
        }
    },
    [SOC_RCM_CLKSRCID_SYS_CLK] =
    {
        .clkSrcType = SOC_RCM_CLKSRCTYPE_DPLL_HSDIVOUTMUX,
        .u.hsdivMuxInfo =
        {
            .hsdivMuxId = SOC_RcmHSDIVClkOutMuxId_DPLL_CORE_OUT2,
        }
    },
    [SOC_RCM_CLKSRCID_XTAL_CLK] =
    {
        .clkSrcType = SOC_RCM_CLKSRCTYPE_XTAL,
    },
};

static const SOC_RcmClockSrcId gSocRcmCpswMiiClkSrcInfoMap[] =
{
   [SOC_RcmCpswMiiClockSource_DPLL_CORE_HSDIV0_CLKOUT1]      = SOC_RCM_CLKSRCID_DPLL_CORE_HSDIV0_CLKOUT1,
   [SOC_RcmCpswMiiClockSource_DPLL_PER_HSDIV0_CLKOUT1_MUXED] = SOC_RCM_CLKSRCID_DPLL_PER_HSDIV0_CLKOUT1,
   [SOC_RcmCpswMiiClockSource_SYS_CLK]  = SOC_RCM_CLKSRCID_SYS_CLK,
   [SOC_RcmCpswMiiClockSource_RC_CLK_10M] = SOC_RCM_CLKSRCID_RC_CLK_10M,
};

static const SOC_RcmClockSrcId gSocRcmDspClkSrcInfoMap[] =
{
    [SOC_RcmDspClockSource_XTAL_CLK]  = SOC_RCM_CLKSRCID_XTAL_CLK,
    [SOC_RcmDspClockSource_RC_CLK_10M] = SOC_RCM_CLKSRCID_RC_CLK_10M,
    [SOC_RcmDspClockSource_DPLL_DSP_HSDIV0_CLKOUT1] = SOC_RCM_CLKSRCID_DPLL_DSP_HSDIV0_CLKOUT1,
    [SOC_RcmDspClockSource_DPLL_DSP_HSDIV0_CLKOUT1_DITH] = SOC_RCM_CLKSRCID_DPLL_DSP_HSDIV0_CLKOUT1_DITH,
    [SOC_RcmDspClockSource_DPLL_CORE_HSDIV0_CLKOUT1] = SOC_RCM_CLKSRCID_DPLL_CORE_HSDIV0_CLKOUT1,
    [SOC_RcmDspClockSource_DPLL_PER_HSDIV0_CLKOUT3] = SOC_RCM_CLKSRCID_DPLL_PER_HSDIV0_CLKOUT3,
};

static const SOC_RcmClockSrcId gSocRcmPeripheralClkSrcInfoMap[] =
{
    [SOC_RcmPeripheralClockSource_DPLL_CORE_HSDIV0_CLKOUT1] = SOC_RCM_CLKSRCID_DPLL_CORE_HSDIV0_CLKOUT1,
    [SOC_RcmPeripheralClockSource_DPLL_CORE_HSDIV0_CLKOUT2] = SOC_RCM_CLKSRCID_DPLL_CORE_HSDIV0_CLKOUT2,
    [SOC_RcmPeripheralClockSource_DPLL_DSP_HSDIV0_CLKOUT2] = SOC_RCM_CLKSRCID_DPLL_DSP_HSDIV0_CLKOUT2,
    [SOC_RcmPeripheralClockSource_DPLL_PER_HSDIV0_CLKOUT1] = SOC_RCM_CLKSRCID_DPLL_PER_HSDIV0_CLKOUT1,
    [SOC_RcmPeripheralClockSource_DPLL_PER_HSDIV0_CLKOUT2] = SOC_RCM_CLKSRCID_DPLL_PER_HSDIV0_CLKOUT2,
    [SOC_RcmPeripheralClockSource_FE1_REF_CLK] = SOC_RCM_CLKSRCID_FE1_REF_CLK,
    [SOC_RcmPeripheralClockSource_RC_CLK_10M] = SOC_RCM_CLKSRCID_RC_CLK_10M,
    [SOC_RcmPeripheralClockSource_RCCLK32K] = SOC_RCM_CLKSRCID_RCCLK32K,
    [SOC_RcmPeripheralClockSource_SYS_CLK] = SOC_RCM_CLKSRCID_SYS_CLK,
    [SOC_RcmPeripheralClockSource_WUCPU_CLK] = SOC_RCM_CLKSRCID_WUCPU_CLK,
    [SOC_RcmPeripheralClockSource_XREF_CLK0] = SOC_RCM_CLKSRCID_XREF_CLK0,
    [SOC_RcmPeripheralClockSource_XREF_CLK1] = SOC_RCM_CLKSRCID_XREF_CLK1,
    [SOC_RcmPeripheralClockSource_XTAL_CLK] = SOC_RCM_CLKSRCID_XTAL_CLK,
};

static const SOC_RcmClockSrcId gSocRcmR5ClkSrcInfoMap[] =
{
    [SOC_RcmR5ClockSource_XTAL_CLK] = SOC_RCM_CLKSRCID_XTAL_CLK,
    [SOC_RcmR5ClockSource_RC_CLK_10M] = SOC_RCM_CLKSRCID_RC_CLK_10M,
    [SOC_RcmR5ClockSource_DPLL_CORE_HSDIV0_CLKOUT2] = SOC_RCM_CLKSRCID_DPLL_CORE_HSDIV0_CLKOUT2,
    [SOC_RcmR5ClockSource_SYS_CLK] = SOC_RCM_CLKSRCID_SYS_CLK,
    [SOC_RcmR5ClockSource_WUCPU_CLK] = SOC_RCM_CLKSRCID_WUCPU_CLK,
};

static const SOC_RcmClockSrcId gSocRcmRssBssFrcClkSrcInfoMap[] =
{
    [SOC_RcmRssBssFrcClockSource_XTAL_CLK] = SOC_RCM_CLKSRCID_XTAL_CLK,
    [SOC_RcmRssBssFrcClockSource_WUCPU_CLK] = SOC_RCM_CLKSRCID_WUCPU_CLK,
    [SOC_RcmRssBssFrcClockSource_SYS_CLK] = SOC_RCM_CLKSRCID_SYS_CLK,
    [SOC_RcmRssBssFrcClockSource_DPLL_PER_HSDIV0_CLKOUT1] = SOC_RCM_CLKSRCID_DPLL_PER_HSDIV0_CLKOUT1,
    [SOC_RcmRssBssFrcClockSource_APLL_1p8G_HSDIV0_CLKOUT2] = SOC_RCM_CLKSRCID_APLL_1p8G_HSDIV0_CLKOUT2,
    [SOC_RcmRssBssFrcClockSource_RC_CLK_10M] = SOC_RCM_CLKSRCID_RC_CLK_10M,
    [SOC_RcmRssBssFrcClockSource_XREF_CLK0] = SOC_RCM_CLKSRCID_XREF_CLK0,
    [SOC_RcmRssBssFrcClockSource_RCCLK32K] = SOC_RCM_CLKSRCID_RCCLK32K,
};

static const SOC_RcmClockSrcId gSocRcmHsiClkSrcInfoMap[] =
{
    [SOC_RcmHSIClockSource_PLL_CORE_CLK] = SOC_RCM_CLKSRCID_PLL_CORE_CLK,
    [SOC_RcmHSIClockSource_APLL_CLK_HSI] = SOC_RCM_CLKSRCID_APLL_CLK_HSI,
    [SOC_RcmHSIClockSource_APLL_1p8GHZ] = SOC_RCM_CLKSRCID_APLL_1p8GHZ,
    [SOC_RcmHSIClockSource_PLL_PER_CLK] = SOC_RCM_CLKSRCID_PLL_PER_CLK,
    [SOC_RcmHSIClockSource_DPLL_CORE_HSDIV0_CLKOUT0] = SOC_RCM_CLKSRCID_DPLL_CORE_HSDIV0_CLKOUT0,
    [SOC_RcmHSIClockSource_RC_CLK_10M] = SOC_RCM_CLKSRCID_RC_CLK_10M,
    [SOC_RcmHSIClockSource_DPLL_DSP_HSDIV0_CLKOUT0] = SOC_RCM_CLKSRCID_DPLL_DSP_HSDIV0_CLKOUT0,
    [SOC_RcmHSIClockSource_DPLL_PER_HSDIV0_CLKOUT0] = SOC_RCM_CLKSRCID_DPLL_PER_HSDIV0_CLKOUT0,
};

static const SOC_RcmClockSrcId gSocRcmHsDivMuxClkSrcInfoMap[] =
{
    [SOC_RcmHSDIVClkOutMuxClockSource_DPLL_CORE_HSDIV0_CLKOUT2_PreMux] = SOC_RCM_CLKSRCID_DPLL_CORE_HSDIV0_CLKOUT2_PreMux,
    [SOC_RcmHSDIVClkOutMuxClockSource_DPLL_DSP_HSDIV0_CLKOUT1_PreMux] = SOC_RCM_CLKSRCID_DPLL_DSP_HSDIV0_CLKOUT1_PreMux,
    [SOC_RcmHSDIVClkOutMuxClockSource_DPLL_DSP_HSDIV0_CLKOUT2_PreMux] = SOC_RCM_CLKSRCID_DPLL_DSP_HSDIV0_CLKOUT2_PreMux,
    [SOC_RcmHSDIVClkOutMuxClockSource_DPLL_PER_HSDIV0_CLKOUT1_PreMux] = SOC_RCM_CLKSRCID_DPLL_PER_HSDIV0_CLKOUT1_PreMux,
    [SOC_RcmHSDIVClkOutMuxClockSource_APLL_1p2G_HSDIV0_CLKOUT0] = SOC_RCM_CLKSRCID_APLL_1p2G_HSDIV0_CLKOUT0,
    [SOC_RcmHSDIVClkOutMuxClockSource_APLL_1p2G_HSDIV0_CLKOUT1] = SOC_RCM_CLKSRCID_APLL_1p2G_HSDIV0_CLKOUT1,
    [SOC_RcmHSDIVClkOutMuxClockSource_APLL_1p2G_HSDIV0_CLKOUT2] = SOC_RCM_CLKSRCID_APLL_1p2G_HSDIV0_CLKOUT2,
    [SOC_RcmHSDIVClkOutMuxClockSource_APLL_1p2G_HSDIV0_CLKOUT3] = SOC_RCM_CLKSRCID_APLL_1p2G_HSDIV0_CLKOUT3,
    [SOC_RcmHSDIVClkOutMuxClockSource_APLL_1p8G_HSDIV0_CLKOUT0] = SOC_RCM_CLKSRCID_APLL_1p8G_HSDIV0_CLKOUT0,
    [SOC_RcmHSDIVClkOutMuxClockSource_APLL_1p8G_HSDIV0_CLKOUT1] = SOC_RCM_CLKSRCID_APLL_1p8G_HSDIV0_CLKOUT1,
    [SOC_RcmHSDIVClkOutMuxClockSource_APLL_400MHZ] = SOC_RCM_CLKSRCID_APLL_400MHZ,
};

static const SOC_RcmClockSrcId gSocRcmRssClkSrcInfoMap[] =
{
    [SOC_RcmRssClkSrcId_WUCPU_CLK] = SOC_RCM_CLKSRCID_WUCPU_CLK,
    [SOC_RcmRssClkSrcId_XTAL_CLK]  = SOC_RCM_CLKSRCID_XTAL_CLK,
    [SOC_RcmRssClkSrcId_DPLL_CORE_HSDIV0_CLKOUT2_MUXED] = SOC_RCM_CLKSRCID_DPLL_CORE_HSDIV0_CLKOUT2,
    [SOC_RcmRssClkSrcId_DPLL_PER_HSDIV0_CLKOUT1_MUXED] = SOC_RCM_CLKSRCID_DPLL_PER_HSDIV0_CLKOUT1,
    [SOC_RcmRssClkSrcId_APLL_1P8_HSDIV0_CLKOUT2] = SOC_RCM_CLKSRCID_APLL_1p8G_HSDIV0_CLKOUT2,
    [SOC_RcmRssClkSrcId_RC_CLK_10M] = SOC_RCM_CLKSRCID_RC_CLK_10M,
    [SOC_RcmRssClkSrcId_SYS_CLK] = SOC_RCM_CLKSRCID_SYS_CLK,
};

static const uint32_t gSocRcmCpswMiiClkSrcValMap[] =
{
   [SOC_RcmCpswMiiClockSource_DPLL_CORE_HSDIV0_CLKOUT1]       = 0x000U,
   [SOC_RcmCpswMiiClockSource_DPLL_PER_HSDIV0_CLKOUT1_MUXED]  = 0x111U,
   [SOC_RcmCpswMiiClockSource_SYS_CLK]                        = 0x222U,
   [SOC_RcmCpswMiiClockSource_RC_CLK_10M]                     = 0x333U,
};

static const uint16_t gSocRcmRssClkSrcValMap[] =
{
    [SOC_RcmRssClkSrcId_WUCPU_CLK]                           =  0x000U,
    [SOC_RcmRssClkSrcId_XTAL_CLK]                            =  0x111U,
    [SOC_RcmRssClkSrcId_DPLL_CORE_HSDIV0_CLKOUT2_MUXED]     =  0x222U,
    [SOC_RcmRssClkSrcId_DPLL_PER_HSDIV0_CLKOUT1_MUXED]      =  0x333U,
    [SOC_RcmRssClkSrcId_APLL_1P8_HSDIV0_CLKOUT2]            =  0x444U,
    [SOC_RcmRssClkSrcId_RC_CLK_10M]                           =  0x555U,
    [SOC_RcmRssClkSrcId_SYS_CLK]                            =  0x666U,
};

static const uint16_t gSocRcmCsiRxClkSrcValMap[] =
{
    [SOC_RcmPeripheralClockSource_XTAL_CLK]                  = 0x000U,
    [SOC_RcmPeripheralClockSource_RC_CLK_10M]                 = 0x111U,
    [SOC_RcmPeripheralClockSource_DPLL_PER_HSDIV0_CLKOUT2]  = 0x222U,
    [SOC_RcmPeripheralClockSource_DPLL_PER_HSDIV0_CLKOUT1]  = 0x333U,
    [SOC_RcmPeripheralClockSource_WUCPU_CLK]                 = 0x666U,

    [SOC_RcmPeripheralClockSource_DPLL_CORE_HSDIV0_CLKOUT1] = 0x888U,
    [SOC_RcmPeripheralClockSource_DPLL_CORE_HSDIV0_CLKOUT2] = 0x888U,
    [SOC_RcmPeripheralClockSource_DPLL_DSP_HSDIV0_CLKOUT2]  = 0x888U,
    [SOC_RcmPeripheralClockSource_FE1_REF_CLK]              = 0x888U,
    [SOC_RcmPeripheralClockSource_RCCLK32K]                 = 0x888U,
    [SOC_RcmPeripheralClockSource_SYS_CLK]                  = 0x888U,
    [SOC_RcmPeripheralClockSource_XREF_CLK0]                = 0x888U,
    [SOC_RcmPeripheralClockSource_XREF_CLK1]                = 0x888U,
};

static const uint16_t gSocRcmRtiClkSrcValMap[] =
{
    [SOC_RcmPeripheralClockSource_XTAL_CLK]                    =    0x000U,
    [SOC_RcmPeripheralClockSource_FE1_REF_CLK]                =    0x111U,
    [SOC_RcmPeripheralClockSource_SYS_CLK]                    =    0x222U,
    [SOC_RcmPeripheralClockSource_DPLL_PER_HSDIV0_CLKOUT1]    =    0x333U,
    [SOC_RcmPeripheralClockSource_RC_CLK_10M]                   =    0x444U,
    [SOC_RcmPeripheralClockSource_XREF_CLK0]                  =    0x666U,
    [SOC_RcmPeripheralClockSource_RCCLK32K]                   =    0x777U,

    [SOC_RcmPeripheralClockSource_DPLL_CORE_HSDIV0_CLKOUT1]   =    0x888U, /* Set unsupported clock source to 0x888 which indicates invalid value */
    [SOC_RcmPeripheralClockSource_DPLL_CORE_HSDIV0_CLKOUT2]   =    0x888U, /* Set unsupported clock source to 0x888 which indicates invalid value */
    [SOC_RcmPeripheralClockSource_DPLL_DSP_HSDIV0_CLKOUT2]    =    0x888U, /* Set unsupported clock source to 0x888 which indicates invalid value */
    [SOC_RcmPeripheralClockSource_DPLL_PER_HSDIV0_CLKOUT2]    =    0x888U, /* Set unsupported clock source to 0x888 which indicates invalid value */
    [SOC_RcmPeripheralClockSource_WUCPU_CLK]                   =    0x888U, /* Set unsupported clock source to 0x888 which indicates invalid value */
    [SOC_RcmPeripheralClockSource_XREF_CLK1]                  =    0x888U, /* Set unsupported clock source to 0x888 which indicates invalid value */
};

static const uint16_t gSocRcmQspiClkSrcValMap[] =
{
    [SOC_RcmPeripheralClockSource_XTAL_CLK]                  =  0x000,
    [SOC_RcmPeripheralClockSource_DPLL_DSP_HSDIV0_CLKOUT2]  =  0x111,
    [SOC_RcmPeripheralClockSource_SYS_CLK]                  =  0x222,
    [SOC_RcmPeripheralClockSource_DPLL_PER_HSDIV0_CLKOUT1]  =  0x333,
    [SOC_RcmPeripheralClockSource_DPLL_CORE_HSDIV0_CLKOUT2] =  0x444,
    [SOC_RcmPeripheralClockSource_RC_CLK_10M]                 =  0x555,
    [SOC_RcmPeripheralClockSource_XREF_CLK0]                =  0x666,

    [SOC_RcmPeripheralClockSource_DPLL_CORE_HSDIV0_CLKOUT1] =  0x888,
    [SOC_RcmPeripheralClockSource_DPLL_PER_HSDIV0_CLKOUT2]  =  0x888,
    [SOC_RcmPeripheralClockSource_FE1_REF_CLK]              =  0x888,
    [SOC_RcmPeripheralClockSource_RCCLK32K]                 =  0x888,
    [SOC_RcmPeripheralClockSource_WUCPU_CLK]                 =  0x888,
    [SOC_RcmPeripheralClockSource_XREF_CLK1]                =  0x888,
};

static const uint16_t gSocRcmSpiClkSrcValMap[] =
{
    [SOC_RcmPeripheralClockSource_XTAL_CLK]                    = 0x000,
    [SOC_RcmPeripheralClockSource_WUCPU_CLK]                   = 0x111,
    [SOC_RcmPeripheralClockSource_SYS_CLK]                    = 0x222,
    [SOC_RcmPeripheralClockSource_DPLL_PER_HSDIV0_CLKOUT1]    = 0x333,
    [SOC_RcmPeripheralClockSource_DPLL_CORE_HSDIV0_CLKOUT2]   = 0x444,
    [SOC_RcmPeripheralClockSource_RC_CLK_10M]                   = 0x555,
    [SOC_RcmPeripheralClockSource_XREF_CLK0]                  = 0x666,

    [SOC_RcmPeripheralClockSource_DPLL_CORE_HSDIV0_CLKOUT1]   = 0x888,
    [SOC_RcmPeripheralClockSource_DPLL_DSP_HSDIV0_CLKOUT2]    = 0x888,
    [SOC_RcmPeripheralClockSource_DPLL_PER_HSDIV0_CLKOUT2]    = 0x888,
    [SOC_RcmPeripheralClockSource_FE1_REF_CLK]                = 0x888,
    [SOC_RcmPeripheralClockSource_RCCLK32K]                   = 0x888,
    [SOC_RcmPeripheralClockSource_XREF_CLK1]                  = 0x888,
};

static const uint16_t gSocRcmI2CClkSrcValMap[] =
{
    [SOC_RcmPeripheralClockSource_XTAL_CLK]                    = 0x000U,
    [SOC_RcmPeripheralClockSource_WUCPU_CLK]                   = 0x111U,
    [SOC_RcmPeripheralClockSource_SYS_CLK]                    = 0x222U,
    [SOC_RcmPeripheralClockSource_DPLL_PER_HSDIV0_CLKOUT1]    = 0x333U,
    [SOC_RcmPeripheralClockSource_DPLL_CORE_HSDIV0_CLKOUT2]   = 0x444U,
    [SOC_RcmPeripheralClockSource_RC_CLK_10M]                   = 0x555U,
    [SOC_RcmPeripheralClockSource_XREF_CLK0]                  = 0x666U,

    [SOC_RcmPeripheralClockSource_DPLL_CORE_HSDIV0_CLKOUT1]   = 0x888U,
    [SOC_RcmPeripheralClockSource_DPLL_DSP_HSDIV0_CLKOUT2]    = 0x888U,
    [SOC_RcmPeripheralClockSource_DPLL_PER_HSDIV0_CLKOUT2]    = 0x888U,
    [SOC_RcmPeripheralClockSource_FE1_REF_CLK]                = 0x888U,
    [SOC_RcmPeripheralClockSource_RCCLK32K]                   = 0x888U,
    [SOC_RcmPeripheralClockSource_XREF_CLK1]                  = 0x888U,
};

static const uint16_t gSocRcmSciClkSrcValMap[] =
{
    [SOC_RcmPeripheralClockSource_XTAL_CLK]                    = 0x000U,
    [SOC_RcmPeripheralClockSource_WUCPU_CLK]                   = 0x111U,
    [SOC_RcmPeripheralClockSource_SYS_CLK]                    = 0x222U,
    [SOC_RcmPeripheralClockSource_DPLL_PER_HSDIV0_CLKOUT1]    = 0x333U,
    [SOC_RcmPeripheralClockSource_DPLL_CORE_HSDIV0_CLKOUT2]   = 0x444U,
    [SOC_RcmPeripheralClockSource_RC_CLK_10M]                   = 0x555U,
    [SOC_RcmPeripheralClockSource_XREF_CLK0]                  = 0x666U,

    [SOC_RcmPeripheralClockSource_DPLL_CORE_HSDIV0_CLKOUT1]   = 0x888U,
    [SOC_RcmPeripheralClockSource_DPLL_DSP_HSDIV0_CLKOUT2]    = 0x888U,
    [SOC_RcmPeripheralClockSource_DPLL_PER_HSDIV0_CLKOUT2]    = 0x888U,
    [SOC_RcmPeripheralClockSource_FE1_REF_CLK]                = 0x888U,
    [SOC_RcmPeripheralClockSource_RCCLK32K]                   = 0x888U,
    [SOC_RcmPeripheralClockSource_XREF_CLK1]                  = 0x888U,
};

static const uint16_t gSocRcmMcanClkSrcValMap[] =
{
    [SOC_RcmPeripheralClockSource_XTAL_CLK]                    = 0x000U,
    [SOC_RcmPeripheralClockSource_DPLL_DSP_HSDIV0_CLKOUT2]    = 0x111U,
    [SOC_RcmPeripheralClockSource_SYS_CLK]                    = 0x222U,
    [SOC_RcmPeripheralClockSource_DPLL_PER_HSDIV0_CLKOUT1]    = 0x333U,
    [SOC_RcmPeripheralClockSource_DPLL_CORE_HSDIV0_CLKOUT2]   = 0x444U,
    [SOC_RcmPeripheralClockSource_RC_CLK_10M]                   = 0x555U,
    [SOC_RcmPeripheralClockSource_XREF_CLK0]                  = 0x666U,
    [SOC_RcmPeripheralClockSource_XREF_CLK1]                  = 0x777U,

    [SOC_RcmPeripheralClockSource_DPLL_CORE_HSDIV0_CLKOUT1]   = 0x888U,
    [SOC_RcmPeripheralClockSource_DPLL_PER_HSDIV0_CLKOUT2]    = 0x888U,
    [SOC_RcmPeripheralClockSource_FE1_REF_CLK]                = 0x888U,
    [SOC_RcmPeripheralClockSource_RCCLK32K]                   = 0x888U,
    [SOC_RcmPeripheralClockSource_WUCPU_CLK]                   = 0x888U,
};

static const uint16_t gSocRcmCptsClkSrcValMap[] =
{
    [SOC_RcmPeripheralClockSource_XTAL_CLK]                    = 0x000U,
    [SOC_RcmPeripheralClockSource_WUCPU_CLK]                   = 0x111U,
    [SOC_RcmPeripheralClockSource_SYS_CLK]                    = 0x222U,
    [SOC_RcmPeripheralClockSource_DPLL_CORE_HSDIV0_CLKOUT1]   = 0x333U,
    [SOC_RcmPeripheralClockSource_DPLL_PER_HSDIV0_CLKOUT1]    = 0x444U,
    [SOC_RcmPeripheralClockSource_RC_CLK_10M]                   = 0x555U,

    [SOC_RcmPeripheralClockSource_DPLL_CORE_HSDIV0_CLKOUT2]   = 0x888U,
    [SOC_RcmPeripheralClockSource_DPLL_DSP_HSDIV0_CLKOUT2]    = 0x888U,
    [SOC_RcmPeripheralClockSource_DPLL_PER_HSDIV0_CLKOUT2]    = 0x888U,
    [SOC_RcmPeripheralClockSource_FE1_REF_CLK]                = 0x888U,
    [SOC_RcmPeripheralClockSource_RCCLK32K]                   = 0x888U,
    [SOC_RcmPeripheralClockSource_XREF_CLK0]                  = 0x888U,
    [SOC_RcmPeripheralClockSource_XREF_CLK1]                  = 0x888U,
};

static const uint16_t gSocRcmCpswClkSrcValMap[] =
{
    [SOC_RcmPeripheralClockSource_XTAL_CLK]                   = 0x000U,
    [SOC_RcmPeripheralClockSource_WUCPU_CLK]                  = 0x111U,
    [SOC_RcmPeripheralClockSource_SYS_CLK]                   = 0x222U,
    [SOC_RcmPeripheralClockSource_DPLL_PER_HSDIV0_CLKOUT1]   = 0x333U,
    [SOC_RcmPeripheralClockSource_DPLL_CORE_HSDIV0_CLKOUT2]  = 0x444U,
    [SOC_RcmPeripheralClockSource_RC_CLK_10M]                  = 0x555U,

    [SOC_RcmPeripheralClockSource_DPLL_CORE_HSDIV0_CLKOUT1]  = 0x888U,
    [SOC_RcmPeripheralClockSource_DPLL_DSP_HSDIV0_CLKOUT2]   = 0x888U,
    [SOC_RcmPeripheralClockSource_DPLL_PER_HSDIV0_CLKOUT2]   = 0x888U,
    [SOC_RcmPeripheralClockSource_FE1_REF_CLK]               = 0x888U,
    [SOC_RcmPeripheralClockSource_RCCLK32K]                  = 0x888U,
    [SOC_RcmPeripheralClockSource_XREF_CLK0]                 = 0x888U,
    [SOC_RcmPeripheralClockSource_XREF_CLK1]                 = 0x888U,
};

static const uint16_t gSocRcmDssHwaClkSrcValMap[] =
{
    [SOC_RcmPeripheralClockSource_DPLL_CORE_HSDIV0_CLKOUT2]  = 0x000U,
    [SOC_RcmPeripheralClockSource_SYS_CLK]                   = 0x111U,

    [SOC_RcmPeripheralClockSource_DPLL_CORE_HSDIV0_CLKOUT1]  = 0x888U,
    [SOC_RcmPeripheralClockSource_DPLL_DSP_HSDIV0_CLKOUT2]   = 0x888U,
    [SOC_RcmPeripheralClockSource_DPLL_PER_HSDIV0_CLKOUT1]   = 0x888U,
    [SOC_RcmPeripheralClockSource_DPLL_PER_HSDIV0_CLKOUT2]   = 0x888U,
    [SOC_RcmPeripheralClockSource_FE1_REF_CLK]               = 0x888U,
    [SOC_RcmPeripheralClockSource_RC_CLK_10M]                  = 0x888U,
    [SOC_RcmPeripheralClockSource_RCCLK32K]                  = 0x888U,
    [SOC_RcmPeripheralClockSource_WUCPU_CLK]                  = 0x888U,
    [SOC_RcmPeripheralClockSource_XREF_CLK0]                 = 0x888U,
    [SOC_RcmPeripheralClockSource_XREF_CLK1]                 = 0x888U,
    [SOC_RcmPeripheralClockSource_XTAL_CLK]                   = 0x888U,
};

static const uint16_t gSocRcmDssRtiClkSrcValMap[] =
{
    [SOC_RcmPeripheralClockSource_XTAL_CLK]                   = 0x000U,
    [SOC_RcmPeripheralClockSource_WUCPU_CLK]                  = 0x111U,
    [SOC_RcmPeripheralClockSource_SYS_CLK]                   = 0x222U,
    [SOC_RcmPeripheralClockSource_DPLL_PER_HSDIV0_CLKOUT1]   = 0x333U,
    [SOC_RcmPeripheralClockSource_RC_CLK_10M]                  = 0x444U,
    [SOC_RcmPeripheralClockSource_XREF_CLK0]                 = 0x666U,
    [SOC_RcmPeripheralClockSource_RCCLK32K]                  = 0x777U,

    [SOC_RcmPeripheralClockSource_DPLL_CORE_HSDIV0_CLKOUT1]  = 0x888U,
    [SOC_RcmPeripheralClockSource_DPLL_CORE_HSDIV0_CLKOUT2]  = 0x888U,
    [SOC_RcmPeripheralClockSource_DPLL_DSP_HSDIV0_CLKOUT2]   = 0x888U,
    [SOC_RcmPeripheralClockSource_DPLL_PER_HSDIV0_CLKOUT2]   = 0x888U,
    [SOC_RcmPeripheralClockSource_FE1_REF_CLK]               = 0x888U,
    [SOC_RcmPeripheralClockSource_XREF_CLK1]                 = 0x888U,
};

static const uint16_t gSocRcmDssSciClkSrcValMap[] =
{
    [SOC_RcmPeripheralClockSource_XTAL_CLK]                   = 0x000U,
    [SOC_RcmPeripheralClockSource_WUCPU_CLK]                  = 0x111U,
    [SOC_RcmPeripheralClockSource_SYS_CLK]                   = 0x222U,
    [SOC_RcmPeripheralClockSource_RCCLK32K]                  = 0x333U,
    [SOC_RcmPeripheralClockSource_RC_CLK_10M]                  = 0x444U,
    [SOC_RcmPeripheralClockSource_DPLL_PER_HSDIV0_CLKOUT1]   = 0x666U,

    [SOC_RcmPeripheralClockSource_DPLL_CORE_HSDIV0_CLKOUT1]  = 0x888U,
    [SOC_RcmPeripheralClockSource_DPLL_CORE_HSDIV0_CLKOUT2]  = 0x888U,
    [SOC_RcmPeripheralClockSource_DPLL_DSP_HSDIV0_CLKOUT2]   = 0x888U,
    [SOC_RcmPeripheralClockSource_DPLL_PER_HSDIV0_CLKOUT2]   = 0x888U,
    [SOC_RcmPeripheralClockSource_FE1_REF_CLK]               = 0x888U,
    [SOC_RcmPeripheralClockSource_XREF_CLK0]                 = 0x888U,
    [SOC_RcmPeripheralClockSource_XREF_CLK1]                 = 0x888U,
};

static const uint16_t gSocRcmRssBssFrcClkSrcValMap[] =
{
    [SOC_RcmRssBssFrcClockSource_XTAL_CLK]                    = 0x000U,
    [SOC_RcmRssBssFrcClockSource_WUCPU_CLK]                   = 0x111U,
    [SOC_RcmRssBssFrcClockSource_SYS_CLK]                    = 0x222U,
    [SOC_RcmRssBssFrcClockSource_DPLL_PER_HSDIV0_CLKOUT1]    = 0x333U,
    [SOC_RcmRssBssFrcClockSource_APLL_1p8G_HSDIV0_CLKOUT2]   = 0x444U,
    [SOC_RcmRssBssFrcClockSource_RC_CLK_10M]                   = 0x555U,
    [SOC_RcmRssBssFrcClockSource_XREF_CLK0]                  = 0x666U,
    [SOC_RcmRssBssFrcClockSource_RCCLK32K]                   = 0x777U,
};

static const uint16_t gSocRcmDspCoreClkSrcValMap[] =
{
    [SOC_RcmDspClockSource_XTAL_CLK]                      =  0x000,
    [SOC_RcmDspClockSource_RC_CLK_10M]                    =  0x111,
    [SOC_RcmDspClockSource_DPLL_DSP_HSDIV0_CLKOUT1]       =  0x222,
    [SOC_RcmDspClockSource_DPLL_DSP_HSDIV0_CLKOUT1_DITH]  =  0x333,
    [SOC_RcmDspClockSource_DPLL_CORE_HSDIV0_CLKOUT1]      =  0x444,
    [SOC_RcmDspClockSource_DPLL_PER_HSDIV0_CLKOUT3]       =  0x666,
};

static const uint16_t gSocRcmR5ClkSrcValMap[] =
{
    [SOC_RcmR5ClockSource_XTAL_CLK]                       = 0x000U,
    [SOC_RcmR5ClockSource_RC_CLK_10M]                      = 0x111U,
    [SOC_RcmR5ClockSource_DPLL_CORE_HSDIV0_CLKOUT2]      = 0x222U,
    [SOC_RcmR5ClockSource_SYS_CLK]                       = 0x333U,
    [SOC_RcmR5ClockSource_WUCPU_CLK]                      = 0x666U,
};

static const uint16_t gSocRcmHSIClkSrcValMap[] =
{
    [SOC_RcmHSIClockSource_PLL_CORE_CLK]                  = 0x000U,
    [SOC_RcmHSIClockSource_APLL_CLK_HSI]                  = 0x111U,
    [SOC_RcmHSIClockSource_APLL_1p8GHZ]                   = 0x222U,
    [SOC_RcmHSIClockSource_PLL_PER_CLK]                   = 0x333U,
    [SOC_RcmHSIClockSource_DPLL_CORE_HSDIV0_CLKOUT0]      = 0x444U,
    [SOC_RcmHSIClockSource_RC_CLK_10M]                      = 0x555U,
    [SOC_RcmHSIClockSource_DPLL_DSP_HSDIV0_CLKOUT0]       = 0x666U,
    [SOC_RcmHSIClockSource_DPLL_PER_HSDIV0_CLKOUT0]       = 0x777U,
};

static const uint16_t gSocRcmPllCoreOut2HSDivClkSrcValMap[] =
{
    [SOC_RcmHSDIVClkOutMuxClockSource_DPLL_CORE_HSDIV0_CLKOUT2_PreMux]  = 0x000U,
    [SOC_RcmHSDIVClkOutMuxClockSource_APLL_1p2G_HSDIV0_CLKOUT0]         = 0x111U,
    [SOC_RcmHSDIVClkOutMuxClockSource_APLL_1p8G_HSDIV0_CLKOUT0]         = 0x222U,
    [SOC_RcmHSDIVClkOutMuxClockSource_APLL_400MHZ]                      = 0x333U,

    [SOC_RcmHSDIVClkOutMuxClockSource_DPLL_DSP_HSDIV0_CLKOUT1_PreMux]   = 0x888U,
    [SOC_RcmHSDIVClkOutMuxClockSource_DPLL_DSP_HSDIV0_CLKOUT2_PreMux]   = 0x888U,
    [SOC_RcmHSDIVClkOutMuxClockSource_DPLL_PER_HSDIV0_CLKOUT1_PreMux]   = 0x888U,
    [SOC_RcmHSDIVClkOutMuxClockSource_APLL_1p2G_HSDIV0_CLKOUT1]         = 0x888U,
    [SOC_RcmHSDIVClkOutMuxClockSource_APLL_1p2G_HSDIV0_CLKOUT2]         = 0x888U,
    [SOC_RcmHSDIVClkOutMuxClockSource_APLL_1p2G_HSDIV0_CLKOUT3]         = 0x888U,
    [SOC_RcmHSDIVClkOutMuxClockSource_APLL_1p8G_HSDIV0_CLKOUT1]         = 0x888U,
};

static const uint16_t gSocRcmPllDspOut1HSDivClkSrcValMap[] =
{
    [SOC_RcmHSDIVClkOutMuxClockSource_DPLL_DSP_HSDIV0_CLKOUT1_PreMux]   = 0x000U,
    [SOC_RcmHSDIVClkOutMuxClockSource_APLL_1p2G_HSDIV0_CLKOUT1]         = 0x111U,
    [SOC_RcmHSDIVClkOutMuxClockSource_APLL_1p8G_HSDIV0_CLKOUT1]         = 0x222U,
    [SOC_RcmHSDIVClkOutMuxClockSource_APLL_400MHZ]                      = 0x333U,

    [SOC_RcmHSDIVClkOutMuxClockSource_DPLL_CORE_HSDIV0_CLKOUT2_PreMux]  = 0x888U,
    [SOC_RcmHSDIVClkOutMuxClockSource_DPLL_DSP_HSDIV0_CLKOUT2_PreMux]   = 0x888U,
    [SOC_RcmHSDIVClkOutMuxClockSource_DPLL_PER_HSDIV0_CLKOUT1_PreMux]   = 0x888U,
    [SOC_RcmHSDIVClkOutMuxClockSource_APLL_1p2G_HSDIV0_CLKOUT0]         = 0x888U,
    [SOC_RcmHSDIVClkOutMuxClockSource_APLL_1p2G_HSDIV0_CLKOUT2]         = 0x888U,
    [SOC_RcmHSDIVClkOutMuxClockSource_APLL_1p2G_HSDIV0_CLKOUT3]         = 0x888U,
    [SOC_RcmHSDIVClkOutMuxClockSource_APLL_1p8G_HSDIV0_CLKOUT0]         = 0x888U,
};

static const uint16_t gSocRcmPllDspOut2HSDivClkSrcValMap[] =
{
    [SOC_RcmHSDIVClkOutMuxClockSource_DPLL_DSP_HSDIV0_CLKOUT2_PreMux]   = 0x000U,
    [SOC_RcmHSDIVClkOutMuxClockSource_APLL_1p2G_HSDIV0_CLKOUT2]         = 0x111U,

    [SOC_RcmHSDIVClkOutMuxClockSource_DPLL_CORE_HSDIV0_CLKOUT2_PreMux]  = 0x888U,
    [SOC_RcmHSDIVClkOutMuxClockSource_DPLL_DSP_HSDIV0_CLKOUT1_PreMux]   = 0x888U,
    [SOC_RcmHSDIVClkOutMuxClockSource_DPLL_PER_HSDIV0_CLKOUT1_PreMux]   = 0x888U,
    [SOC_RcmHSDIVClkOutMuxClockSource_APLL_1p2G_HSDIV0_CLKOUT0]         = 0x888U,
    [SOC_RcmHSDIVClkOutMuxClockSource_APLL_1p2G_HSDIV0_CLKOUT1]         = 0x888U,
    [SOC_RcmHSDIVClkOutMuxClockSource_APLL_1p2G_HSDIV0_CLKOUT3]         = 0x888U,
    [SOC_RcmHSDIVClkOutMuxClockSource_APLL_1p8G_HSDIV0_CLKOUT0]         = 0x888U,
    [SOC_RcmHSDIVClkOutMuxClockSource_APLL_1p8G_HSDIV0_CLKOUT1]         = 0x888U,
    [SOC_RcmHSDIVClkOutMuxClockSource_APLL_400MHZ]                      = 0x888U,
};

static const uint16_t gSocRcmPllPerOut1HSDivClkSrcValMap[] =
{
    [SOC_RcmHSDIVClkOutMuxClockSource_DPLL_PER_HSDIV0_CLKOUT1_PreMux]   = 0x000U,
    [SOC_RcmHSDIVClkOutMuxClockSource_APLL_1p2G_HSDIV0_CLKOUT3]         = 0x111U,

    [SOC_RcmHSDIVClkOutMuxClockSource_DPLL_CORE_HSDIV0_CLKOUT2_PreMux]  = 0x888U,
    [SOC_RcmHSDIVClkOutMuxClockSource_DPLL_DSP_HSDIV0_CLKOUT1_PreMux]   = 0x888U,
    [SOC_RcmHSDIVClkOutMuxClockSource_DPLL_DSP_HSDIV0_CLKOUT2_PreMux]   = 0x888U,
    [SOC_RcmHSDIVClkOutMuxClockSource_APLL_1p2G_HSDIV0_CLKOUT0]         = 0x888U,
    [SOC_RcmHSDIVClkOutMuxClockSource_APLL_1p2G_HSDIV0_CLKOUT1]         = 0x888U,
    [SOC_RcmHSDIVClkOutMuxClockSource_APLL_1p2G_HSDIV0_CLKOUT2]         = 0x888U,
    [SOC_RcmHSDIVClkOutMuxClockSource_APLL_1p8G_HSDIV0_CLKOUT0]         = 0x888U,
    [SOC_RcmHSDIVClkOutMuxClockSource_APLL_1p8G_HSDIV0_CLKOUT1]         = 0x888U,
    [SOC_RcmHSDIVClkOutMuxClockSource_APLL_400MHZ]                      = 0x888U,
};

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

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

static void SOC_rcmGetClkSrcAndDivReg(SOC_RcmPeripheralId PeriphID,
                               SOC_RcmPeripheralClockSource clkSource, uint16_t *clkSrcVal,
                               volatile uint32_t **clkSrcReg,
                               volatile uint32_t **clkdDivReg);
static void SOC_rcmProgPllCoreDivider(uint8_t inputClockDiv , uint8_t divider,
                               uint16_t multiplier, uint8_t postDivider,
                               uint32_t fracMultiplier, uint32_t sdDiv);
static void SOC_rcmProgPllDspDivider(uint8_t inputClockDiv , uint8_t divider,
                              uint16_t multiplier, uint8_t postDivider,
                              uint32_t fracMultiplier, uint32_t sdDiv);
static void SOC_rcmProgPllPerDivider(uint8_t inputClockDiv , uint8_t divider,
                              uint16_t multiplier, uint8_t postDivider,
                              uint32_t fracMultiplier, uint32_t sdDiv);

static void SOC_rcmConfigurePllCore(uint16_t trimVal);
static void SOC_rcmConfigurePllDsp(uint16_t trimVal);
static void SOC_rcmConfigurePllPer(uint16_t trimVal);
static SOC_RcmXtalFreqId SOC_rcmGetXTALFrequency(void);
static uint16_t SOC_rcmGetCoreTrimVal(void);
static uint16_t SOC_rcmGetDspTrimVal(void);
static uint16_t SOC_rcmGetPerTrimVal(void);
static SOC_RcmADPLLJConfig_t const *SOC_rcmGetADPLLJConfig(uint32_t Finp,
                                                      SOC_RcmPllFoutFreqId foutFreqId);
static uint32_t SOC_rcmADPLLJGetFOut(uint32_t Finp, uint32_t N, uint32_t M,
                                     uint32_t M2, uint32_t FracM,
                                     uint32_t div2flag);
static uint32_t SOC_rcmGetDSSFout(uint32_t Finp, uint32_t div2flag);
static uint32_t SOC_rcmGetCoreFout(uint32_t Finp, uint32_t div2flag);
static uint32_t SOC_rcmGetPerFout(uint32_t Finp, uint32_t div2flag);
static uint32_t SOC_rcmGetCoreHSDivOut(uint32_t Finp, uint32_t div2flag, SOC_RcmPllHSDIVOutId hsDivOut);
static uint32_t SOC_rcmGetDSSHSDivOut(uint32_t Finp, uint32_t div2flag, SOC_RcmPllHSDIVOutId hsDivOut);
static uint32_t SOC_rcmGetPerHSDivOut(uint32_t Finp, uint32_t div2flag, SOC_RcmPllHSDIVOutId hsDivOut);
static uint32_t SOC_rcmGetModuleClkDivFromRegVal(uint32_t moduleClkDivRegVal);
static uint32_t SOC_rcmGetApllHSDivOutFreq(SOC_RcmApllId apllId, SOC_RcmPllHSDIVOutId hsDivOut);
static void SOC_rcmGetHSDivClkSrc(SOC_RcmHSDIVClkOutMuxId clkOutMuxId, SOC_RcmClockSrcId * clkSrcId);
static void SOC_rcmGetRssClkSrc(SOC_RcmClockSrcId * clkSrcId);
static uint32_t SOC_rcmGetPeripheralClockFrequency(SOC_RcmPeripheralClockSource clkSource);
static uint32_t SOC_rcmGetDspClockFrequency(SOC_RcmDspClockSource clkSource);

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

static inline uint32_t SOC_rcmMake8 (uint8_t msb, uint8_t lsb, uint8_t val)
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

static inline uint32_t SOC_rcmMake16 (uint8_t msb, uint8_t lsb, uint16_t val)
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

static inline uint32_t SOC_rcmMake32 (uint8_t msb, uint8_t lsb, uint32_t val)
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

static inline uint32_t SOC_rcmInsert8 (volatile uint32_t reg, uint8_t msb, uint8_t lsb, uint8_t val)
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

static inline uint32_t SOC_rcmInsert16 (volatile uint32_t reg, uint8_t msb, uint8_t lsb, uint16_t val)
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

static inline uint32_t SOC_rcmInsert32 (volatile uint32_t reg, uint8_t msb, uint8_t lsb, uint32_t val)
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

static inline uint8_t SOC_rcmExtract8 (volatile uint32_t reg, uint8_t msb, uint8_t lsb)
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

static inline uint16_t SOC_rcmExtract16 (volatile uint32_t reg, uint8_t msb, uint8_t lsb)
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

static inline uint32_t SOC_rcmExtract32 (volatile uint32_t reg, uint8_t msb, uint8_t lsb)
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

static inline CSL_mss_ctrlRegs* CSL_MSS_CTRL_getBaseAddress (void)
{
    return (CSL_mss_ctrlRegs*) CSL_MSS_CTRL_U_BASE;
}

static inline CSL_dss_ctrlRegs* CSL_DSS_CTRL_getBaseAddress (void)
{
    return (CSL_dss_ctrlRegs *) CSL_DSS_CTRL_U_BASE;
}

static inline CSL_rss_ctrlRegs* CSL_RSS_CTRL_getBaseAddress (void)
{
    return (CSL_rss_ctrlRegs *) CSL_RSS_CTRL_U_BASE;
}

static inline CSL_rss_proc_ctrlRegs* CSL_RSS_PROC_CTRL_getBaseAddress (void)
{
    return (CSL_rss_proc_ctrlRegs *) CSL_RSS_PROC_CTRL_U_BASE;
}

static inline CSL_rss_rcmRegs* CSL_RSS_RCM_getBaseAddress (void)
{
    return (CSL_rss_rcmRegs*) CSL_RSS_RCM_U_BASE;
}

static inline CSL_mss_rcmRegs* CSL_RCM_getBaseAddress (void)
{
    return (CSL_mss_rcmRegs*) CSL_MSS_RCM_U_BASE;
}

static inline CSL_dss_rcmRegs* CSL_DSSRCM_getBaseAddress (void)
{
    return (CSL_dss_rcmRegs*) CSL_DSS_RCM_U_BASE;
}

static inline CSL_mss_toprcmRegs* CSL_TopRCM_getBaseAddress (void)
{
    return (CSL_mss_toprcmRegs*) CSL_MSS_TOPRCM_U_BASE;
}

static inline CSL_top_ctrlRegs* CSL_TopCtrl_getBaseAddress (void)
{
    return (CSL_top_ctrlRegs*)CSL_TOP_CTRL_U_BASE;
}

static inline uint8_t CSL_TopCtrl_readXtalFreqScale (const CSL_top_ctrlRegs* ptrTopCtrlRegs)
{
    return (SOC_RCM_EFUSE_FIELD_EXTRACT_XTAL_FREQ_SCALE(ptrTopCtrlRegs));
}

static inline uint8_t CSL_TopCtrl_readCoreADPLLTrimValidEfuse (const CSL_top_ctrlRegs* ptrTopCtrlRegs)
{
    return (SOC_RCM_EFUSE_FIELD_EXTRACT_CORE_ADPLL_TRIM_VALID(ptrTopCtrlRegs));
}

static inline uint8_t CSL_TopCtrl_readDspADPLLTrimValidEfuse (const CSL_top_ctrlRegs* ptrTopCtrlRegs)
{
    return (SOC_RCM_EFUSE_FIELD_EXTRACT_DSP_ADPLL_TRIM_VALID(ptrTopCtrlRegs));
}

static inline uint8_t CSL_TopCtrl_readPerADPLLTrimValidEfuse (const CSL_top_ctrlRegs* ptrTopCtrlRegs)
{
    return (SOC_RCM_EFUSE_FIELD_EXTRACT_PER_ADPLL_TRIM_VALID(ptrTopCtrlRegs));
}

static inline uint8_t CSL_TopCtrl_readQSPIClkFreqEfuse (const CSL_top_ctrlRegs * ptrTopCtrlRegs)
{
    return (SOC_RCM_EFUSE_FIELD_EXTRACT_QSPI_CLK_FREQ(ptrTopCtrlRegs));
}

static inline uint8_t CSL_TopCtrl_readPgVerEfuse (const CSL_top_ctrlRegs * ptrTopCtrlRegs)
{
    return (SOC_RCM_EFUSE_FIELD_EXTRACT_PG_VER(ptrTopCtrlRegs));
}

static inline uint8_t CSL_TopCtrl_readFlashModeEfuse (const CSL_top_ctrlRegs * ptrTopCtrlRegs)
{
    return (SOC_RCM_EFUSE_FIELD_EXTRACT_FLASH_MODE(ptrTopCtrlRegs));
}

static inline uint8_t CSL_TopCtrl_readR5ClkFreqEfuse(const CSL_top_ctrlRegs * ptrTopCtrlRegs)
{
    return (SOC_RCM_EFUSE_FIELD_EXTRACT_BOOT_FREQ(ptrTopCtrlRegs));
}

static inline uint16_t CSL_TopCtrl_readCoreADPLLTrimEfuse(const CSL_top_ctrlRegs* ptrTopCtrlRegs)
{
    return (SOC_RCM_EFUSE_FIELD_EXTRACT_CORE_ADPLL_TRIM(ptrTopCtrlRegs));
}

static inline uint16_t CSL_TopCtrl_readDspADPLLTrimEfuse(const CSL_top_ctrlRegs* ptrTopCtrlRegs)
{
    return (SOC_RCM_EFUSE_FIELD_EXTRACT_DSP_ADPLL_TRIM(ptrTopCtrlRegs));
}

static inline uint16_t CSL_TopCtrl_readPerADPLLTrimEfuse(const CSL_top_ctrlRegs* ptrTopCtrlRegs)
{
    return (SOC_RCM_EFUSE_FIELD_EXTRACT_PER_ADPLL_TRIM(ptrTopCtrlRegs));
}

static inline uint8_t CSL_TopCtrl_dualCoreBootEnableEfuse(const CSL_top_ctrlRegs * ptrTopCtrlRegs)
{
    return (SOC_RCM_EFUSE_FIELD_EXTRACT_DUALCORE_BOOT_ENABLE(ptrTopCtrlRegs));
}

static inline uint8_t CSL_TopCtrl_dualCoreSwitchDisableEfuse(const CSL_top_ctrlRegs * ptrTopCtrlRegs)
{
    return (SOC_RCM_EFUSE_FIELD_EXTRACT_DUALCORE_SWITCH_DISABLE(ptrTopCtrlRegs));
}

static SOC_RcmEfusePkgType CSL_TopCtrl_readDeviceTypeEfuse(const CSL_top_ctrlRegs * ptrTopCtrlRegs)
{
    uint8_t pkgTypeEfuseVal = SOC_RCM_EFUSE_FIELD_EXTRACT_PKG_TYPE(ptrTopCtrlRegs);
    SOC_RcmEfusePkgType pkgType = SOC_RCM_EFUSE_DEVICE_PKG_TYPE_ETS;

    switch(pkgTypeEfuseVal)
    {
        case SOC_RCM_EFUSE_DEVICE_PKG_TYPE_ETS_VAL:
            pkgType = SOC_RCM_EFUSE_DEVICE_PKG_TYPE_ETS;
            break;
        default:
            /* To work with untrimmed samples.Assume ETS */
            pkgType = SOC_RCM_EFUSE_DEVICE_PKG_TYPE_ETS;
    }

    return pkgType;
}

static uint32_t SOC_rcmGetClkSrcFromClkSelVal(const uint16_t *clkSelTbl, uint32_t numEntries, uint32_t clkSelMatchVal)
{
    uint32_t i;
    uint32_t clkSource;

    for (i = 0; i < numEntries; i++)
    {
        if(clkSelMatchVal == clkSelTbl[i])
        {
            break;
        }
    }
    if(i < numEntries)
    {
        clkSource = i;
    }
    else
    {
        clkSource = SOC_RcmR5ClockSource_MAX_VALUE;
    }
    return clkSource;
}

static void SOC_rcmGetHSDIVClkOutMuxClkSrcAndDivReg(SOC_RcmHSDIVClkOutMuxId hsDivMuxId,
                                             SOC_RcmHSDIVClkOutMuxClockSource clkSource,
                                             uint16_t *muxSelVal,
                                             volatile uint32_t **muxSelReg)
{
    CSL_mss_toprcmRegs *ptrTOPRCMRegs;

    ptrTOPRCMRegs = CSL_TopRCM_getBaseAddress();
    switch(hsDivMuxId)
    {
        case  SOC_RcmHSDIVClkOutMuxId_DPLL_CORE_OUT2:
            *muxSelReg = &ptrTOPRCMRegs->PLLC_CLK2_SRC_SEL;
            *muxSelVal = gSocRcmPllCoreOut2HSDivClkSrcValMap[clkSource];
            break;
        case  SOC_RcmHSDIVClkOutMuxId_DPLL_DSP_OUT1:
            *muxSelReg = &ptrTOPRCMRegs->PLLD_CLK1_SRC_SEL;
            *muxSelVal = gSocRcmPllDspOut1HSDivClkSrcValMap[clkSource];
            break;
        case  SOC_RcmHSDIVClkOutMuxId_DPLL_DSP_OUT2:
            *muxSelReg = &ptrTOPRCMRegs->PLLD_CLK2_SRC_SEL;
            *muxSelVal = gSocRcmPllDspOut2HSDivClkSrcValMap[clkSource];
            break;
        case  SOC_RcmHSDIVClkOutMuxId_DPLL_PER_OUT1:
            *muxSelReg = &ptrTOPRCMRegs->PLLP_CLK1_SRC_SEL;
            *muxSelVal = gSocRcmPllPerOut1HSDivClkSrcValMap[clkSource];
            break;
        default:
            DebugP_assert(FALSE);
    }
}

static int32_t SOC_rcmGetFixedRootClkFout(SOC_RcmFixedClockId fixedClkId, uint32_t *fOut)
{
    *fOut = gSocRcmFixedClocksTbl[fixedClkId].fOut;
    return SystemP_SUCCESS;
}

static void SOC_rcmGetHSDIVClkOutMuxClkSrc(SOC_RcmHSDIVClkOutMuxId hsDivMuxId,
                                           SOC_RcmHSDIVClkOutMuxClockSource *clkSource)
{
    uint32_t clkSrcId;
    CSL_mss_toprcmRegs *ptrTOPRCMRegs;
    uint32_t clkSrcRegVal;

    ptrTOPRCMRegs = CSL_TopRCM_getBaseAddress();
    switch(hsDivMuxId)
    {
        case  SOC_RcmHSDIVClkOutMuxId_DPLL_CORE_OUT2:
            clkSrcRegVal  = ptrTOPRCMRegs->PLLC_CLK2_SRC_SEL;
            clkSrcId = SOC_rcmGetClkSrcFromClkSelVal(gSocRcmPllCoreOut2HSDivClkSrcValMap, SOC_RCM_UTILS_ARRAYSIZE(gSocRcmPllCoreOut2HSDivClkSrcValMap), clkSrcRegVal);
            DebugP_assert(clkSrcId != ~0U);
            *clkSource = (SOC_RcmHSDIVClkOutMuxClockSource) clkSrcId;
            break;
        case  SOC_RcmHSDIVClkOutMuxId_DPLL_DSP_OUT1:
            clkSrcRegVal  = ptrTOPRCMRegs->PLLD_CLK1_SRC_SEL;
            clkSrcId = SOC_rcmGetClkSrcFromClkSelVal(gSocRcmPllDspOut1HSDivClkSrcValMap, SOC_RCM_UTILS_ARRAYSIZE(gSocRcmPllDspOut1HSDivClkSrcValMap), clkSrcRegVal);
            DebugP_assert(clkSrcId != ~0U);
            *clkSource = (SOC_RcmHSDIVClkOutMuxClockSource) clkSrcId;
            break;
        case  SOC_RcmHSDIVClkOutMuxId_DPLL_DSP_OUT2:
            clkSrcRegVal  = ptrTOPRCMRegs->PLLD_CLK2_SRC_SEL;
            clkSrcId = SOC_rcmGetClkSrcFromClkSelVal(gSocRcmPllDspOut2HSDivClkSrcValMap, SOC_RCM_UTILS_ARRAYSIZE(gSocRcmPllDspOut2HSDivClkSrcValMap), clkSrcRegVal);
            DebugP_assert(clkSrcId != ~0U);
            *clkSource = (SOC_RcmHSDIVClkOutMuxClockSource) clkSrcId;
            break;
        case  SOC_RcmHSDIVClkOutMuxId_DPLL_PER_OUT1:
            clkSrcRegVal  = ptrTOPRCMRegs->PLLP_CLK1_SRC_SEL;
            clkSrcId = SOC_rcmGetClkSrcFromClkSelVal(gSocRcmPllPerOut1HSDivClkSrcValMap, SOC_RCM_UTILS_ARRAYSIZE(gSocRcmPllPerOut1HSDivClkSrcValMap), clkSrcRegVal);
            DebugP_assert(clkSrcId != ~0U);
            *clkSource = (SOC_RcmHSDIVClkOutMuxClockSource) clkSrcId;
            break;
        default:
            DebugP_assert(FALSE);
    }

    return;
}

static void SOC_rcmGetRssBssFrcClkSrcAndDivReg(SOC_RcmRssBssFrcClockSource clkSource,
                                        uint16_t *clkSrcVal,
                                        volatile uint32_t **clkSrcReg,
                                        volatile uint32_t **clkdDivReg)
{
    CSL_rss_rcmRegs *ptrRSSRCMRegs;

    ptrRSSRCMRegs = CSL_RSS_RCM_getBaseAddress();

    *clkSrcReg  = &(ptrRSSRCMRegs->RSS_FRC_CLK_SRC_SEL);
    *clkdDivReg = &(ptrRSSRCMRegs->RSS_FRC_CLK_DIV_VAL);
    *clkSrcVal = gSocRcmRssBssFrcClkSrcValMap[clkSource];
}

static void SOC_rcmGetRssBssFrcClkSrcAndDivValue(SOC_RcmRssBssFrcClockSource *clkSource,
                                                 volatile uint32_t *clkDiv)
{
    uint32_t clkSrc;
    uint32_t clkSrcId;
    CSL_rss_rcmRegs *ptrRSSRCMRegs;

    ptrRSSRCMRegs = CSL_RSS_RCM_getBaseAddress();

    clkSrc  = ptrRSSRCMRegs->RSS_FRC_CLK_SRC_SEL;
    *clkDiv = ptrRSSRCMRegs->RSS_FRC_CLK_DIV_VAL;
    clkSrcId = SOC_rcmGetClkSrcFromClkSelVal(gSocRcmRssBssFrcClkSrcValMap, SOC_RCM_UTILS_ARRAYSIZE(gSocRcmRssBssFrcClkSrcValMap), clkSrc);
    DebugP_assert(clkSrcId != ~0U);
    *clkSource = (SOC_RcmRssBssFrcClockSource) clkSrcId;

    return;
}

static void SOC_rcmGetRssClkSrcAndDivRegInfo(SOC_RcmRssClkSrcId clkSource,
                                      uint16_t *clkSrcVal,
                                      volatile uint32_t **clkSrcReg,
                                      volatile uint32_t **clkDivReg)
{
    CSL_mss_toprcmRegs *ptrTOPRCMRegs;

    ptrTOPRCMRegs = CSL_TopRCM_getBaseAddress();

    *clkSrcReg  = &(ptrTOPRCMRegs->RSS_CLK_SRC_SEL);
    *clkSrcVal = gSocRcmRssClkSrcValMap[clkSource];
    *clkDivReg = &(ptrTOPRCMRegs->RSS_DIV_VAL);
}

static void SOC_rcmGetRssClkDivValue(volatile uint32_t *clkDivRegVal)
{
    CSL_mss_toprcmRegs *ptrTOPRCMRegs;

    ptrTOPRCMRegs = CSL_TopRCM_getBaseAddress();
    *clkDivRegVal = ptrTOPRCMRegs->RSS_DIV_VAL;
}

static void SOC_rcmGetRssClkSrcLocal(SOC_RcmRssClkSrcId *clkSource)
{
    uint32_t clkSrc;
    uint32_t clkSrcId;
    CSL_mss_toprcmRegs *ptrTOPRCMRegs;

    ptrTOPRCMRegs = CSL_TopRCM_getBaseAddress();

    clkSrc  = ptrTOPRCMRegs->RSS_CLK_SRC_SEL;
    clkSrcId = SOC_rcmGetClkSrcFromClkSelVal(gSocRcmRssClkSrcValMap, SOC_RCM_UTILS_ARRAYSIZE(gSocRcmRssClkSrcValMap), clkSrc);
    DebugP_assert(clkSrcId != ~0U);
    *clkSource = (SOC_RcmRssClkSrcId) clkSrcId;
    return;
}

static void SOC_rcmGetHSIClkSrcAndDivReg(SOC_RcmHSIClockSource clkSource,
                                  uint16_t *clkSrcVal,
                                  volatile uint32_t **clkSrcReg,
                                  volatile uint32_t **clkdDivReg)
{
    CSL_mss_toprcmRegs *ptrTOPRCMRegs;

    ptrTOPRCMRegs = CSL_TopRCM_getBaseAddress();

    *clkSrcReg  = &(ptrTOPRCMRegs->HSI_CLK_SRC_SEL);
    *clkdDivReg = &(ptrTOPRCMRegs->HSI_DIV_VAL);
    *clkSrcVal = gSocRcmHSIClkSrcValMap[clkSource];
}

static void SOC_rcmGetHSIClkSrcAndDivValue(SOC_RcmHSIClockSource *clkSource,
                                           volatile uint32_t *clkDiv)
{
    uint32_t clkSrc;
    uint32_t clkSrcId;
    CSL_mss_toprcmRegs *ptrTOPRCMRegs;

    ptrTOPRCMRegs = CSL_TopRCM_getBaseAddress();

    clkSrc  = ptrTOPRCMRegs->HSI_CLK_SRC_SEL;
    *clkDiv = ptrTOPRCMRegs->HSI_DIV_VAL;
    clkSrcId = SOC_rcmGetClkSrcFromClkSelVal(gSocRcmHSIClkSrcValMap, SOC_RCM_UTILS_ARRAYSIZE(gSocRcmHSIClkSrcValMap), clkSrc);
    DebugP_assert(clkSrcId != ~0U);
    *clkSource = (SOC_RcmHSIClockSource) clkSrcId;

    return;
}

static uint32_t SOC_rcmGetDpllHSDivOutFreq(SOC_RcmDpllId_e dpllId, SOC_RcmPllHSDIVOutId hsDivOut)
{
    SOC_RcmXtalFreqId clkFreqId;
    uint32_t Finp, FOut;

    clkFreqId = SOC_rcmGetXTALFrequency();
    switch(dpllId)
    {
        case SOC_RCM_DPLL_CORE:
        {
            Finp = gSocRcmXTALInfo[clkFreqId].Finp;

            if(hsDivOut != SOC_RCM_PLLHSDIV_OUT_NONE)
            {
                FOut = SOC_rcmGetCoreHSDivOut(Finp, gSocRcmXTALInfo[clkFreqId].div2flag, hsDivOut);
            }
            else
            {
                FOut = SOC_rcmGetCoreFout(Finp, gSocRcmXTALInfo[clkFreqId].div2flag);
            }
            break;
        }
        case SOC_RCM_DPLL_PER:
        {
            Finp = gSocRcmXTALInfo[clkFreqId].Finp;
            if(hsDivOut != SOC_RCM_PLLHSDIV_OUT_NONE)
            {
                FOut = SOC_rcmGetPerHSDivOut(Finp, gSocRcmXTALInfo[clkFreqId].div2flag, hsDivOut);
            }
            else
            {
                FOut = SOC_rcmGetPerFout(Finp, gSocRcmXTALInfo[clkFreqId].div2flag);
            }
            break;
        }
        default:
        case SOC_RCM_DPLL_DSS:
        {
            Finp = gSocRcmXTALInfo[clkFreqId].Finp;
            if(hsDivOut != SOC_RCM_PLLHSDIV_OUT_NONE)
            {
                FOut = SOC_rcmGetDSSHSDivOut(Finp, gSocRcmXTALInfo[clkFreqId].div2flag, hsDivOut);
            }
            else
            {
                FOut = SOC_rcmGetDSSFout(Finp, gSocRcmXTALInfo[clkFreqId].div2flag);
            }
            break;
        }
    }
    return FOut;
}

static uint32_t SOC_rcmGetFreqLeafNode(SOC_RcmClockSrcId clkSrcId)
{
    SOC_RcmXtalFreqId clkFreqId;
    uint32_t FOut = 1U;
    const SOC_RcmClkSrcInfo *clkSrcInfo;

    clkSrcInfo = &gSocRcmClkSrcInfoMap[clkSrcId];
    switch (clkSrcInfo->clkSrcType)
    {
        case SOC_RCM_CLKSRCTYPE_XTAL:
        {
            clkFreqId = SOC_rcmGetXTALFrequency();
            FOut = gSocRcmXTALFreqTbl[clkFreqId];
            break;
        }
        case SOC_RCM_CLKSRCTYPE_FIXED:
        {
            SOC_rcmGetFixedRootClkFout(clkSrcInfo->u.fixedClkInfo.fixedClkId ,&FOut);
            break;
        }
        case SOC_RCM_CLKSRCTYPE_APLL_HSDIVOUT:
        {
            FOut = SOC_rcmGetApllHSDivOutFreq(clkSrcInfo->u.apllHsDivInfo.apllId,
                                   clkSrcInfo->u.apllHsDivInfo.hsdivOut);

            break;
        }
        case SOC_RCM_CLKSRCTYPE_DPLL_HSDIVOUT:
        {
            FOut = SOC_rcmGetDpllHSDivOutFreq(clkSrcInfo->u.dpllHsDivInfo.dpllId,
                                              clkSrcInfo->u.dpllHsDivInfo.hsdivOut);
            break;
        }
        case SOC_RCM_CLKSRCTYPE_DPLL_HSDIVOUTMUX:
        {
            /* SOC_RCM_CLKSRCTYPE_DPLL_HSDIVOUTMUX is not a leaf node */
            DebugP_assert(FALSE);
            break;
        }
    }
    return (FOut);
}

static uint32_t SOC_rcmGetFreq(SOC_RcmClockSrcId clkSrcId)
{
    uint32_t FOut;
    const SOC_RcmClkSrcInfo *clkSrcInfo;

    clkSrcInfo = &gSocRcmClkSrcInfoMap[clkSrcId];
    if(clkSrcInfo->clkSrcType == SOC_RCM_CLKSRCTYPE_DPLL_HSDIVOUTMUX)
    {
        SOC_RcmClockSrcId hsMuxSrcClkId;
        SOC_rcmGetHSDivClkSrc(clkSrcInfo->u.hsdivMuxInfo.hsdivMuxId, &hsMuxSrcClkId);
        FOut = SOC_rcmGetFreqLeafNode(hsMuxSrcClkId);
        /* If SYS_CLKD clock source additional divide Fout of CORE_CLK_OUT_2 by SYS_CLK_DIV */
        if(clkSrcId == SOC_RCM_CLKSRCID_SYS_CLK)
        {
            CSL_mss_toprcmRegs *ptrTopRCMRegs;
            uint32_t sysClkDiv;

            /* Core PLL settings */
            ptrTopRCMRegs = CSL_TopRCM_getBaseAddress();
            sysClkDiv = CSL_FEXT(ptrTopRCMRegs->SYS_CLK_DIV_VAL, MSS_TOPRCM_SYS_CLK_DIV_VAL_SYS_CLK_DIV_VAL_CLKDIV);
            sysClkDiv = (sysClkDiv & 0xF) + 1;
            FOut = FOut / sysClkDiv;
        }
    }
    else
    {
        FOut = SOC_rcmGetFreqLeafNode(clkSrcId);
    }

    return (FOut);
}

static uint32_t SOC_rcmGetHSIInFreq(SOC_RcmHSIClockSource  clkSource)
{
    SOC_RcmClockSrcId clkSrcId;
    uint32_t FOut;

    clkSrcId = gSocRcmHsiClkSrcInfoMap[clkSource];
    FOut = SOC_rcmGetFreq(clkSrcId);
    return (FOut);
}

int32_t SOC_rcmGetHSIFreq(uint32_t *freqHz)
{
    uint32_t            clkDivisorRegVal;
    uint32_t            clkDivisor;
    int32_t             retVal = SystemP_SUCCESS;
    uint32_t            Finp;
    SOC_RcmHSIClockSource  clkSource;

    SOC_rcmGetHSIClkSrcAndDivValue(&clkSource, &clkDivisorRegVal);
    Finp = SOC_rcmGetHSIInFreq(clkSource);
    clkDivisor = SOC_rcmGetModuleClkDivFromRegVal(clkDivisorRegVal);
    *freqHz = Finp / clkDivisor;

    return (retVal);
}

int32_t SOC_rcmGetRssBssFrcFreq(uint32_t *freqHz)
{
    uint32_t            clkDivisorRegVal;
    uint32_t            clkDivisor;
    int32_t             retVal = SystemP_SUCCESS;
    uint32_t            Finp;
    SOC_RcmRssBssFrcClockSource  clkSource;
    SOC_RcmClockSrcId rootClkId;

    SOC_rcmGetRssBssFrcClkSrcAndDivValue(&clkSource, &clkDivisorRegVal);
    rootClkId = gSocRcmRssBssFrcClkSrcInfoMap[clkSource];
    Finp = SOC_rcmGetFreq(rootClkId);
    clkDivisor = SOC_rcmGetModuleClkDivFromRegVal(clkDivisorRegVal);
    *freqHz = Finp / clkDivisor;

    return (retVal);
}

int32_t SOC_rcmGetRssClkFreq(uint32_t *freqHz)
{
    int32_t             retVal = SystemP_SUCCESS;
    SOC_RcmClockSrcId   rootClkId;
    uint32_t            Finp;
    uint32_t            clkDivisorRegVal;
    uint32_t            clkDivisor;

    SOC_rcmGetRssClkSrc(&rootClkId);
    SOC_rcmGetRssClkDivValue(&clkDivisorRegVal);
    Finp = SOC_rcmGetFreq(rootClkId);
    clkDivisor = SOC_rcmGetModuleClkDivFromRegVal(clkDivisorRegVal);
    *freqHz = Finp / clkDivisor;

    return (retVal);
}

static void SOC_rcmGetClkSrcAndDivReg(SOC_RcmPeripheralId PeriphID,
                                SOC_RcmPeripheralClockSource clkSource,
                                uint16_t *clkSrcVal,
                                volatile uint32_t **clkSrcReg,
                                volatile uint32_t **clkdDivReg)
{
    CSL_mss_rcmRegs *ptrMSSRCMRegs;
    CSL_dss_rcmRegs *ptrDSSRCMRegs;
    CSL_mss_toprcmRegs *ptrTOPRCMRegs;

    ptrMSSRCMRegs = CSL_RCM_getBaseAddress();
    ptrDSSRCMRegs = CSL_DSSRCM_getBaseAddress();
    ptrTOPRCMRegs = CSL_TopRCM_getBaseAddress();

    switch (PeriphID)
    {
        case SOC_RcmPeripheralId_CSIRX:
        {
            *clkSrcReg  = &(ptrTOPRCMRegs->CSIRX_CLK_SRC_SEL);
            *clkdDivReg = &(ptrTOPRCMRegs->CSIRX_DIV_VAL);
            *clkSrcVal = gSocRcmCsiRxClkSrcValMap[clkSource];
            break;
        }

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
        case SOC_RcmPeripheralId_MSS_QSPI:
        {
            *clkSrcReg  = &(ptrMSSRCMRegs->MSS_QSPI_CLK_SRC_SEL);
            *clkdDivReg = &(ptrMSSRCMRegs->MSS_QSPI_CLK_DIV_VAL);
            *clkSrcVal = gSocRcmQspiClkSrcValMap[clkSource];
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
            *clkSrcVal = gSocRcmI2CClkSrcValMap[clkSource];
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
        case SOC_RcmPeripheralId_MSS_CPTS:
        {
            *clkSrcReg  = &(ptrMSSRCMRegs->MSS_CPTS_CLK_SRC_SEL);
            *clkdDivReg = &(ptrMSSRCMRegs->MSS_CPTS_CLK_DIV_VAL);
            *clkSrcVal = gSocRcmCptsClkSrcValMap[clkSource];
            break;
        }
        case SOC_RcmPeripheralId_MSS_CPSW:
        {
            *clkSrcReg  = &(ptrMSSRCMRegs->MSS_CPSW_CLK_SRC_SEL);
            *clkdDivReg = &(ptrMSSRCMRegs->MSS_CPSW_CLK_DIV_VAL);
            *clkSrcVal = gSocRcmCpswClkSrcValMap[clkSource];
            break;
        }
        case SOC_RcmPeripheralId_DSS_HWA:
        {
            *clkSrcReg  = &(ptrDSSRCMRegs->DSS_HWA_CLK_SRC_SEL);
            *clkdDivReg = (volatile uint32_t *)NULL;
            *clkSrcVal = gSocRcmDssHwaClkSrcValMap[clkSource];
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
        default:
        {
            *clkSrcReg  = NULL;
            *clkdDivReg = NULL;
            *clkSrcVal = 0x888U;
        }
    }
    return;
}

static int32_t SOC_rcmGetClkSrcAndDivValue(SOC_RcmPeripheralId PeriphID,
                                        SOC_RcmPeripheralClockSource *clkSource,
                                        volatile uint32_t *clkDiv)
{
    CSL_mss_rcmRegs *ptrMSSRCMRegs;
    CSL_dss_rcmRegs *ptrDSSRCMRegs;
    CSL_mss_toprcmRegs *ptrTOPRCMRegs;
    uint32_t clkSrc;
    uint32_t clkSrcId;
    int32_t retVal = SystemP_SUCCESS;

    ptrMSSRCMRegs = CSL_RCM_getBaseAddress();
    ptrDSSRCMRegs = CSL_DSSRCM_getBaseAddress();
    ptrTOPRCMRegs = CSL_TopRCM_getBaseAddress();

    switch (PeriphID)
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
            clkSrcId = SOC_rcmGetClkSrcFromClkSelVal(gSocRcmI2CClkSrcValMap, SOC_RCM_UTILS_ARRAYSIZE(gSocRcmI2CClkSrcValMap), clkSrc);
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
        case SOC_RcmPeripheralId_DSS_HWA:
        {
            clkSrc  = ptrDSSRCMRegs->DSS_HWA_CLK_SRC_SEL;
            /* HWA does not have a clk div register and divider is always 1 */
            *clkDiv = 1U;
            clkSrcId = SOC_rcmGetClkSrcFromClkSelVal(gSocRcmDssHwaClkSrcValMap, SOC_RCM_UTILS_ARRAYSIZE(gSocRcmDssHwaClkSrcValMap), clkSrc);
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

static void SOC_rcmGetDSPClkSrcAndDivReg(SOC_RcmDspClockSource clkSource,
                                   uint16_t *clkSrcVal,
                                   volatile uint32_t **clkSrcReg,
                                   volatile uint32_t **clkdDivReg)
{
    CSL_dss_rcmRegs *ptrDSSRCMRegs;

    ptrDSSRCMRegs = CSL_DSSRCM_getBaseAddress();

    *clkSrcReg  = &(ptrDSSRCMRegs->DSS_DSP_CLK_SRC_SEL);
    *clkdDivReg = &(ptrDSSRCMRegs->DSS_DSP_CLK_DIV_VAL);
    *clkSrcVal = gSocRcmDspCoreClkSrcValMap[clkSource];

    return;
}

static int32_t SOC_rcmGetDSPClkSrcAndDivValue(SOC_RcmDspClockSource *clkSource, uint32_t *clkDiv)
{
    uint32_t clkSrc, clkSrcId;
    int32_t retVal = SystemP_SUCCESS;
    CSL_dss_rcmRegs *ptrDSSRCMRegs;

    ptrDSSRCMRegs = CSL_DSSRCM_getBaseAddress();
    clkSrc  = ptrDSSRCMRegs->DSS_DSP_CLK_SRC_SEL;
    *clkDiv = ptrDSSRCMRegs->DSS_DSP_CLK_DIV_VAL;
    clkSrcId = SOC_rcmGetClkSrcFromClkSelVal(gSocRcmDspCoreClkSrcValMap, SOC_RCM_UTILS_ARRAYSIZE(gSocRcmDspCoreClkSrcValMap), clkSrc);
    if(clkSrcId != ~0U)
    {
        *clkSource = (SOC_RcmDspClockSource) clkSrcId;
    }
    else
    {
        retVal = SystemP_FAILURE;
    }
    return retVal;
}

static void SOC_rcmProgPllCoreDivider(uint8_t inputClockDiv , uint8_t divider,
                                uint16_t multiplier, uint8_t postDivider,
                                uint32_t fracMultiplier, uint32_t sdDiv)
{
    volatile uint32_t *ptrM2NReg, *ptrMN2Reg, *ptrFracMReg;
    CSL_mss_toprcmRegs *ptrTopRCMRegs;

    ptrTopRCMRegs = CSL_TopRCM_getBaseAddress();

    ptrM2NReg   = &(ptrTopRCMRegs->PLL_CORE_M2NDIV);
    ptrMN2Reg   = &(ptrTopRCMRegs->PLL_CORE_MN2DIV);
    ptrFracMReg = &(ptrTopRCMRegs->PLL_CORE_FRACDIV);

    /* Initialization sequence referred from ADPLLLJ_GS70_v0.8-02 */
    /* program M2 (post divider) */
    // APPLJ-1 Setting
    // CLOCKOUT = M/(N+1) * CLKINP * (1/M2)  =  0x7d0/(39+1) * 40 * (1/1) = 2G
    //HW_WR_REG32(CSL_MSS_TOPRCM_U_BASE+PLL_CORE_M2NDIV     , 0x10027);      //M2NDIV_M2[22:16] = 1 , M2NDIV_N[7:0] = 0x27
    //HW_WR_REG32(CSL_MSS_TOPRCM_U_BASE+PLL_CORE_MN2DIV     , 0x107d0);      //MN2DIV_N2[19:16] = 1 , MN2DIV_M[11:0] = 0x7d0

    *ptrM2NReg = SOC_rcmInsert8 (*ptrM2NReg, 22U, 16U, postDivider);

    /* program N (input clock divider) */
    *ptrM2NReg = SOC_rcmInsert8 (*ptrM2NReg, 7U, 0U, inputClockDiv);

    /* program M (multiplier) */
    *ptrMN2Reg = SOC_rcmInsert16 (*ptrMN2Reg, 11U, 0U, multiplier);

    /* program N2 (divider) */
    *ptrMN2Reg = SOC_rcmInsert8 (*ptrMN2Reg, 19U, 16U, divider);

    /* program Fractional Multiplier */
    *ptrFracMReg = SOC_rcmInsert32 (*ptrFracMReg, 17U, 0U, fracMultiplier);

    /* program sigma Delta divider value. This is part of Frac Div register */
    *ptrFracMReg = SOC_rcmInsert32 (*ptrFracMReg, 31U, 24U, sdDiv);
}

static void SOC_rcmProgPllDspDivider(uint8_t inputClockDiv , uint8_t divider,
                               uint16_t multiplier, uint8_t postDivider,
                               uint32_t fracMultiplier, uint32_t sdDiv)
{
    volatile uint32_t *ptrM2NReg, *ptrMN2Reg, *ptrFracMReg;
    CSL_mss_toprcmRegs *ptrTopRCMRegs;

    ptrTopRCMRegs = CSL_TopRCM_getBaseAddress();

    ptrM2NReg   = &(ptrTopRCMRegs->PLL_DSP_M2NDIV);
    ptrMN2Reg   = &(ptrTopRCMRegs->PLL_DSP_MN2DIV);
    ptrFracMReg = &(ptrTopRCMRegs->PLL_DSP_FRACDIV);

    /* Initialization sequence referred from ADPLLLJ_GS70_v0.8-02 */
    /* program M2 (post divider) */
    // APPLJ-2 Setting
    // CLOCKOUT = M/(N+1) * CLKINP * (1/M2)  =  0x708/(39+1) * 40 * (1/1) = 1.8G
    //HW_WR_REG32(CSL_MSS_TOPRCM_U_BASE+PLL_DSP_M2NDIV     , 0x10027);      //M2NDIV_M2[22:16] = 1 , M2NDIV_N[7:0] = 0x27
    //HW_WR_REG32(CSL_MSS_TOPRCM_U_BASE+PLL_DSP_MN2DIV     , 0x10708);      //MN2DIV_N2[19:16] = 1 , MN2DIV_M[11:0] = 0x708
    *ptrM2NReg = SOC_rcmInsert8 (*ptrM2NReg, 22U, 16U, postDivider);

    /* program N (input clock divider) */
    *ptrM2NReg = SOC_rcmInsert8 (*ptrM2NReg, 7U, 0U, inputClockDiv);

    /* program M (multiplier) */
    *ptrMN2Reg = SOC_rcmInsert16 (*ptrMN2Reg, 11U, 0U, multiplier);

    /* program N2 (divider) */
    *ptrMN2Reg = SOC_rcmInsert8 (*ptrMN2Reg, 19U, 16U, divider);

    /* program Fractional Multiplier */
    *ptrFracMReg = SOC_rcmInsert32 (*ptrFracMReg, 17U, 0U, fracMultiplier);

    /* program sigma Delta divider value. This is part of Frac Div register */
    *ptrFracMReg = SOC_rcmInsert32 (*ptrFracMReg, 31U, 24U, sdDiv);
}

static void SOC_rcmProgPllPerDivider(uint8_t inputClockDiv , uint8_t divider,
                               uint16_t multiplier, uint8_t postDivider,
                               uint32_t fracMultiplier, uint32_t sdDiv)
{
    volatile uint32_t *ptrM2NReg, *ptrMN2Reg, *ptrFracMReg;
    CSL_mss_toprcmRegs *ptrTopRCMRegs;

    ptrTopRCMRegs = CSL_TopRCM_getBaseAddress();

    ptrM2NReg   = &(ptrTopRCMRegs->PLL_PER_M2NDIV);
    ptrMN2Reg   = &(ptrTopRCMRegs->PLL_PER_MN2DIV);
    ptrFracMReg = &(ptrTopRCMRegs->PLL_PER_FRACDIV);

    /* Initialization sequence referred from ADPLLLJ_GS70_v0.8-02 */
    /* program M2 (post divider) */
    // APPLJ-3 Setting
    // CLOCKOUT = M/(N+1) * CLKINP * (1/M2)  =  0x6C0/(39+1) * 40 * (1/1) = 1.728G
    //HW_WR_REG32(CSL_MSS_TOPRCM_U_BASE+PLL_PER_M2NDIV     , 0x10027);      //M2NDIV_M2[22:16] = 1 , M2NDIV_N[7:0] = 0x27
    //HW_WR_REG32(CSL_MSS_TOPRCM_U_BASE+PLL_PER_MN2DIV     , 0x106C0);      //MN2DIV_N2[19:16] = 1 , MN2DIV_M[11:0] = 0x6C0
    *ptrM2NReg = SOC_rcmInsert8 (*ptrM2NReg, 22U, 16U, postDivider);

    /* program N (input clock divider) */
    *ptrM2NReg = SOC_rcmInsert8 (*ptrM2NReg, 7U, 0U, inputClockDiv);

    /* program M (multiplier) */
    *ptrMN2Reg = SOC_rcmInsert16 (*ptrMN2Reg, 11U, 0U, multiplier);

    /* program N2 (divider) */
    *ptrMN2Reg = SOC_rcmInsert8 (*ptrMN2Reg, 19U, 16U, divider);

    /* program Fractional Multiplier */
    *ptrFracMReg = SOC_rcmInsert32 (*ptrFracMReg, 17U, 0U, fracMultiplier);

    /* program sigma Delta divider value. This is part of Frac Div register */
    *ptrFracMReg = SOC_rcmInsert32 (*ptrFracMReg, 31U, 24U, sdDiv);
}

static void SOC_rcmConfigurePllCore(uint16_t trimVal)
{
    volatile uint32_t *ptrClkCtrl, *ptrTenable, *ptrTenableDiv, *ptrPllStatus;
    CSL_mss_toprcmRegs *ptrTopRCMRegs;
    uint8_t phaseLockStatus;

    /* Core PLL settings */
    ptrTopRCMRegs = CSL_TopRCM_getBaseAddress();

    ptrClkCtrl    = &(ptrTopRCMRegs->PLL_CORE_CLKCTRL);
    ptrTenable    = &(ptrTopRCMRegs->PLL_CORE_TENABLE);
    ptrTenableDiv = &(ptrTopRCMRegs->PLL_CORE_TENABLEDIV);
    ptrPllStatus  = &(ptrTopRCMRegs->PLL_CORE_STATUS);

    /* update the Clock control setting */
    // APPLJ-1 Setting
    // CLOCKOUT = M/(N+1) * CLKINP * (1/M2)  =  0x7d0/(39+1) * 40 * (1/1) = 2G
    //HW_WR_REG32(CSL_MSS_TOPRCM_U_BASE+PLL_CORE_M2NDIV     , 0x10027);      //M2NDIV_M2[22:16] = 1 , M2NDIV_N[7:0] = 0x27
    //HW_WR_REG32(CSL_MSS_TOPRCM_U_BASE+PLL_CORE_MN2DIV     , 0x107d0);      //MN2DIV_N2[19:16] = 1 , MN2DIV_M[11:0] = 0x7d0
    //HW_WR_REG32(CSL_MSS_TOPRCM_U_BASE+PLL_CORE_CLKCTRL    , 0x29021000);   //CLKDCOLDOEN[29] = 1,NWELLTRIM[28:24] = 9 IDLE[23] = 0 CLKDCOLDOPWDNZ[17] = 1 SELFREQDCO[12:10] = 4

    //HW_WR_REG32(CSL_MSS_TOPRCM_U_BASE+PLL_CORE_TENABLE    , 0x1);          // TENABLE    = 1
    //HW_WR_REG32(CSL_MSS_TOPRCM_U_BASE+PLL_CORE_CLKCTRL    , 0x29021001);   //+TINTZ[0]   = 1
    //HW_WR_REG32(CSL_MSS_TOPRCM_U_BASE+PLL_CORE_TENABLE    , 0x0);          // TENABLE    = 0
    //HW_WR_REG32(CSL_MSS_TOPRCM_U_BASE+PLL_CORE_TENABLEDIV , 0x1);          // TENABLEDIV = 1
    //HW_WR_REG32(CSL_MSS_TOPRCM_U_BASE+PLL_CORE_TENABLEDIV , 0x0);          // TENABLEDIV = 0

    /* program CLKDCOLDOEN[29] = 1, IDLE[23] = 0, CLKDCOLDOPWDNZ[17] = 1, SELFREQDCO[12:10] = 4 */
    *ptrClkCtrl = SOC_rcmInsert8 (*ptrClkCtrl, 29U, 29U, 0x1U);
    *ptrClkCtrl = SOC_rcmInsert8 (*ptrClkCtrl, 23U, 23U, 0x0U);
    *ptrClkCtrl = SOC_rcmInsert8 (*ptrClkCtrl, 17U, 17U, 0x1U);
    *ptrClkCtrl = SOC_rcmInsert8 (*ptrClkCtrl, 12U, 10U, 0x2U);

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
    // APPLJ-1  :  loop check to PLLLOCK DONE
    //lock_status = HW_RD_REG32(CSL_MSS_TOPRCM_U_BASE+PLL_CORE_STATUS); //PHASELOCK[10]
    //while(0x400 != (lock_status & 0x400)) {
    //   lock_status = HW_RD_REG32(CSL_MSS_TOPRCM_U_BASE+PLL_CORE_STATUS); //PHASELOCK[10]
    //}

    do
    {
        phaseLockStatus = SOC_rcmExtract8 (*ptrPllStatus, 10U, 10U);
    }while(phaseLockStatus != 1U);
}

static void SOC_rcmConfigurePllDsp(uint16_t trimVal)
{
    volatile uint32_t *ptrClkCtrl, *ptrTenable, *ptrTenableDiv, *ptrPllStatus;
    CSL_mss_toprcmRegs *ptrTopRCMRegs;
    uint8_t phaseLockStatus;

    /* Core PLL settings */
    ptrTopRCMRegs = CSL_TopRCM_getBaseAddress();

    ptrClkCtrl    = &(ptrTopRCMRegs->PLL_DSP_CLKCTRL);
    ptrTenable    = &(ptrTopRCMRegs->PLL_DSP_TENABLE);
    ptrTenableDiv = &(ptrTopRCMRegs->PLL_DSP_TENABLEDIV);
    ptrPllStatus  = &(ptrTopRCMRegs->PLL_DSP_STATUS);

    /* update the Clock control setting */
    /* program CLKDCOLDOEN[29] = 1, IDLE[23] = 0, CLKDCOLDOPWDNZ[17] = 1, SELFREQDCO[12:10] = 4 */
    //HW_WR_REG32(CSL_MSS_TOPRCM_U_BASE+PLL_DSP_CLKCTRL    , 0x29021000);   //CLKDCOLDOEN[29] = 1,NWELLTRIM[28:24] = 9 IDLE[23] = 0 CLKDCOLDOPWDNZ[17] = 1 SELFREQDCO[12:10] = 4

    //HW_WR_REG32(CSL_MSS_TOPRCM_U_BASE+PLL_DSP_TENABLE    , 0x1);          // TENABLE    = 1
    //HW_WR_REG32(CSL_MSS_TOPRCM_U_BASE+PLL_DSP_CLKCTRL    , 0x29021001);   //+TINTZ[0]   = 1
    //HW_WR_REG32(CSL_MSS_TOPRCM_U_BASE+PLL_DSP_TENABLE    , 0x0);          // TENABLE    = 0
    //HW_WR_REG32(CSL_MSS_TOPRCM_U_BASE+PLL_DSP_TENABLEDIV , 0x1);          // TENABLEDIV = 1
    //HW_WR_REG32(CSL_MSS_TOPRCM_U_BASE+PLL_DSP_TENABLEDIV , 0x0);          // TENABLEDIV = 0

    *ptrClkCtrl = SOC_rcmInsert8 (*ptrClkCtrl, 29U, 29U, 0x1U);
    *ptrClkCtrl = SOC_rcmInsert8 (*ptrClkCtrl, 23U, 23U, 0x0U);
    *ptrClkCtrl = SOC_rcmInsert8 (*ptrClkCtrl, 17U, 17U, 0x1U);
    *ptrClkCtrl = SOC_rcmInsert8 (*ptrClkCtrl, 12U, 10U, 0x2U);

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
    // APPLJ-2 : loop check to PLLLOCK DONE
    //lock_status = HW_RD_REG32(CSL_MSS_TOPRCM_U_BASE+PLL_DSP_STATUS); //PHASELOCK[10]
    //while(0x400 != (lock_status & 0x400)) {
    //   lock_status = HW_RD_REG32(CSL_MSS_TOPRCM_U_BASE+PLL_DSP_STATUS); //PHASELOCK[10]
    //}

    do
    {
        phaseLockStatus = SOC_rcmExtract8 (*ptrPllStatus, 10U, 10U);
    }while(phaseLockStatus != 1U);
}

static void SOC_rcmConfigurePllPer(uint16_t trimVal)
{
    volatile uint32_t *ptrClkCtrl, *ptrTenable, *ptrTenableDiv, *ptrPllStatus;
    CSL_mss_toprcmRegs *ptrTopRCMRegs;
    uint8_t phaseLockStatus;

    /* Core PLL settings */
    ptrTopRCMRegs = CSL_TopRCM_getBaseAddress();

    ptrClkCtrl    = &(ptrTopRCMRegs->PLL_PER_CLKCTRL);
    ptrTenable    = &(ptrTopRCMRegs->PLL_PER_TENABLE);
    ptrTenableDiv = &(ptrTopRCMRegs->PLL_PER_TENABLEDIV);
    ptrPllStatus  = &(ptrTopRCMRegs->PLL_PER_STATUS);

    /* update the Clock control setting */
    /* program CLKDCOLDOEN[29] = 1, IDLE[23] = 0, CLKDCOLDOPWDNZ[17] = 1, SELFREQDCO[12:10] = 4 */
    //HW_WR_REG32(CSL_MSS_TOPRCM_U_BASE+PLL_PER_CLKCTRL    , 0x29021000);   //CLKDCOLDOEN[29] = 1,NWELLTRIM[28:24] = 9 IDLE[23] = 0 CLKDCOLDOPWDNZ[17] = 1 SELFREQDCO[12:10] = 4

    //HW_WR_REG32(CSL_MSS_TOPRCM_U_BASE+PLL_PER_TENABLE    , 0x1);          // TENABLE    = 1
    //HW_WR_REG32(CSL_MSS_TOPRCM_U_BASE+PLL_PER_CLKCTRL    , 0x29021001);   //+TINTZ[0]   = 1
    //HW_WR_REG32(CSL_MSS_TOPRCM_U_BASE+PLL_PER_TENABLE    , 0x0);          // TENABLE    = 0
    //HW_WR_REG32(CSL_MSS_TOPRCM_U_BASE+PLL_PER_TENABLEDIV , 0x1);          // TENABLEDIV = 1
    //HW_WR_REG32(CSL_MSS_TOPRCM_U_BASE+PLL_PER_TENABLEDIV , 0x0);          // TENABLEDIV = 0

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
    // APPLJ-3 : loop check to PLLLOCK DONE
    //lock_status = HW_RD_REG32(CSL_MSS_TOPRCM_U_BASE+PLL_PER_STATUS); //PHASELOCK[10]
    //while(0x400 != (lock_status & 0x400)) {
    //   lock_status = HW_RD_REG32(CSL_MSS_TOPRCM_U_BASE+PLL_PER_STATUS); //PHASELOCK[10]
    //}

    do
    {
        phaseLockStatus = SOC_rcmExtract8 (*ptrPllStatus, 10U, 10U);
    }while(phaseLockStatus != 1U);
}

static SOC_RcmXtalFreqId SOC_rcmGetXTALFrequency(void)
{
    CSL_mss_toprcmRegs *ptrTopRCMRegs;
    CSL_top_ctrlRegs* ptrTopCtrlRegs;
    uint32_t xtalRegVal;
    SOC_RcmXtalFreqId freq;
    uint8_t xtalFreqScale;

    ptrTopRCMRegs = CSL_TopRCM_getBaseAddress();
    ptrTopCtrlRegs = CSL_TopCtrl_getBaseAddress();

    /* read the register bits corresponding to XTAL Frequency */
    xtalRegVal = SOC_rcmExtract32 (ptrTopRCMRegs->ANA_REG_WU_MODE_REG_LOWV, 6U, 5U);
    /* read the register bits corresponding to XTAL Frequency Scale */
    xtalFreqScale = CSL_TopCtrl_readXtalFreqScale (ptrTopCtrlRegs);

    if(xtalFreqScale == 1U)
    {
        if(xtalRegVal == 0U)
        {
            freq = SOC_RCM_XTAL_FREQID_CLK_20MHZ;
        }
        else if(xtalRegVal == 1U)
        {
            freq = SOC_RCM_XTAL_FREQID_CLK_22p5792MHZ;
        }
        else if(xtalRegVal == 2U)
        {
            freq = SOC_RCM_XTAL_FREQID_CLK_24p576MHZ;
        }
        else
        {
            freq = SOC_RCM_XTAL_FREQID_CLK_25MHZ;
        }
    }
    else
    {
        if(xtalRegVal == 0U)
        {
            freq = SOC_RCM_XTAL_FREQID_CLK_40MHZ;
        }
        else if(xtalRegVal == 1U)
        {
            freq = SOC_RCM_XTAL_FREQID_CLK_45p1584MHZ;
        }
        else if(xtalRegVal == 2U)
        {
            freq = SOC_RCM_XTAL_FREQID_CLK_49p152MHZ;
        }
        else
        {
            freq = SOC_RCM_XTAL_FREQID_CLK_50MHZ;
        }
    }

    return (freq);
}

static uint16_t SOC_rcmGetCoreTrimVal(void)
{
    CSL_top_ctrlRegs *ptrTopCtrlRegs;
    uint8_t coreADPLLValid;
    uint16_t coreADPLLTrimVal;

    ptrTopCtrlRegs = CSL_TopCtrl_getBaseAddress();
    coreADPLLValid = CSL_TopCtrl_readCoreADPLLTrimValidEfuse(ptrTopCtrlRegs);
    if(coreADPLLValid == 1U)
    {
        coreADPLLTrimVal = CSL_TopCtrl_readCoreADPLLTrimEfuse(ptrTopCtrlRegs);
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

    ptrTopCtrlRegs = CSL_TopCtrl_getBaseAddress();
    dspADPLLValid = CSL_TopCtrl_readDspADPLLTrimValidEfuse(ptrTopCtrlRegs);
    if(dspADPLLValid == 1U)
    {
        dspADPLLTrimVal = CSL_TopCtrl_readDspADPLLTrimEfuse(ptrTopCtrlRegs);
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

    ptrTopCtrlRegs = CSL_TopCtrl_getBaseAddress();
    perADPLLValid = CSL_TopCtrl_readPerADPLLTrimValidEfuse(ptrTopCtrlRegs);
    if(perADPLLValid == 1U)
    {
        perADPLLTrimVal = CSL_TopCtrl_readPerADPLLTrimEfuse(ptrTopCtrlRegs);
    }
    else
    {
        perADPLLTrimVal = (uint16_t) SOC_RCM_PER_ADPLL_DEFAULT_VALUE;
    }
    return (perADPLLTrimVal);
}

__attribute__((weak)) uint32_t SOC_rcmIsR5FInLockStepMode(uint32_t r5fClusterGroupId)
{
    uint32_t retVal = FALSE;
    CSL_mss_ctrlRegs *mssCtrl = CSL_MSS_CTRL_getBaseAddress();

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

void SOC_rcmCoreApllHSDivConfig(SOC_RcmPllHsDivOutConfig *hsDivCfg)
{
    CSL_mss_toprcmRegs *ptrTopRCMRegs;
    uint32_t hsDivOutRegVal;
    uint32_t Fout;
    uint32_t Finp;
    SOC_RcmXtalFreqId clkFreqId;

    /* Core PLL settings */
    ptrTopRCMRegs = CSL_TopRCM_getBaseAddress();
    clkFreqId = SOC_rcmGetXTALFrequency();
    Finp = gSocRcmXTALInfo[clkFreqId].Finp;
    Fout = SOC_rcmGetCoreFout(Finp, gSocRcmXTALInfo[clkFreqId].div2flag);

    /* Derive Clocks */
    /* TPR12_Ch08_Clock_Arch_0p91 is used as reference for below settings */
    /* Set clock divider values from Core PLL*/
    /* 400Mhz */
    if(hsDivCfg->hsdivOutEnMask & SOC_RCM_PLL_HSDIV_OUTPUT_ENABLE_0)
    {
        DebugP_assert((Fout % hsDivCfg->hsDivOutFreqHz[SOC_RCM_PLL_HSDIV_OUTPUT_IDX0]) == 0);
        hsDivOutRegVal = Fout / hsDivCfg->hsDivOutFreqHz[SOC_RCM_PLL_HSDIV_OUTPUT_IDX0];
        hsDivOutRegVal--;
        ptrTopRCMRegs->PLL_CORE_HSDIVIDER_CLKOUT0 = SOC_rcmInsert8 (ptrTopRCMRegs->PLL_CORE_HSDIVIDER_CLKOUT0, 4U, 0U, hsDivOutRegVal);
    }
    if(hsDivCfg->hsdivOutEnMask & SOC_RCM_PLL_HSDIV_OUTPUT_ENABLE_1)
    {
        DebugP_assert((Fout % hsDivCfg->hsDivOutFreqHz[SOC_RCM_PLL_HSDIV_OUTPUT_IDX1]) == 0);
        hsDivOutRegVal = Fout / hsDivCfg->hsDivOutFreqHz[SOC_RCM_PLL_HSDIV_OUTPUT_IDX1];
        hsDivOutRegVal--;
        ptrTopRCMRegs->PLL_CORE_HSDIVIDER_CLKOUT1 = SOC_rcmInsert8 (ptrTopRCMRegs->PLL_CORE_HSDIVIDER_CLKOUT1, 4U, 0U, hsDivOutRegVal);
    }
    if(hsDivCfg->hsdivOutEnMask & SOC_RCM_PLL_HSDIV_OUTPUT_ENABLE_2)
    {
        DebugP_assert((Fout % hsDivCfg->hsDivOutFreqHz[SOC_RCM_PLL_HSDIV_OUTPUT_IDX2]) == 0);
        hsDivOutRegVal = Fout / hsDivCfg->hsDivOutFreqHz[SOC_RCM_PLL_HSDIV_OUTPUT_IDX2];
        hsDivOutRegVal--;
        ptrTopRCMRegs->PLL_CORE_HSDIVIDER_CLKOUT2 = SOC_rcmInsert8 (ptrTopRCMRegs->PLL_CORE_HSDIVIDER_CLKOUT2, 4U, 0U, hsDivOutRegVal);
    }
    if(hsDivCfg->hsdivOutEnMask & SOC_RCM_PLL_HSDIV_OUTPUT_ENABLE_3)
    {
        DebugP_assert((Fout % hsDivCfg->hsDivOutFreqHz[SOC_RCM_PLL_HSDIV_OUTPUT_IDX3]) == 0);
        hsDivOutRegVal = Fout / hsDivCfg->hsDivOutFreqHz[SOC_RCM_PLL_HSDIV_OUTPUT_IDX3];
        hsDivOutRegVal--;
        ptrTopRCMRegs->PLL_CORE_HSDIVIDER_CLKOUT3 = SOC_rcmInsert8 (ptrTopRCMRegs->PLL_CORE_HSDIVIDER_CLKOUT3, 4U, 0U, hsDivOutRegVal);
    }
    /* Generate Trigger to latch these values */
    ptrTopRCMRegs->PLL_CORE_HSDIVIDER         = SOC_rcmInsert8 (ptrTopRCMRegs->PLL_CORE_HSDIVIDER, 2U, 2U, 0x1U);
    ptrTopRCMRegs->PLL_CORE_HSDIVIDER         = SOC_rcmInsert8 (ptrTopRCMRegs->PLL_CORE_HSDIVIDER, 2U, 2U, 0x0U);

    /* Ungate the clocks */
    if(hsDivCfg->hsdivOutEnMask & SOC_RCM_PLL_HSDIV_OUTPUT_ENABLE_0)
    {
        ptrTopRCMRegs->PLL_CORE_HSDIVIDER_CLKOUT0 = SOC_rcmInsert8 (ptrTopRCMRegs->PLL_CORE_HSDIVIDER_CLKOUT0, 8U, 8U, 0x1U);
    }
    if(hsDivCfg->hsdivOutEnMask & SOC_RCM_PLL_HSDIV_OUTPUT_ENABLE_1)
    {
        ptrTopRCMRegs->PLL_CORE_HSDIVIDER_CLKOUT1 = SOC_rcmInsert8 (ptrTopRCMRegs->PLL_CORE_HSDIVIDER_CLKOUT1, 8U, 8U, 0x1U);
    }
    if(hsDivCfg->hsdivOutEnMask & SOC_RCM_PLL_HSDIV_OUTPUT_ENABLE_2)
    {
        ptrTopRCMRegs->PLL_CORE_HSDIVIDER_CLKOUT2 = SOC_rcmInsert8 (ptrTopRCMRegs->PLL_CORE_HSDIVIDER_CLKOUT2, 8U, 8U, 0x1U);
    }
    if(hsDivCfg->hsdivOutEnMask & SOC_RCM_PLL_HSDIV_OUTPUT_ENABLE_3)
    {
        ptrTopRCMRegs->PLL_CORE_HSDIVIDER_CLKOUT3 = SOC_rcmInsert8 (ptrTopRCMRegs->PLL_CORE_HSDIVIDER_CLKOUT3, 8U, 8U, 0x1U);
    }
}

/**
 *  @b Description
 *  @n
 *      This API Sets up the core PLL configuration switches the R5 to PLL clock.
 *
 *  \ingroup DRIVER_RCM_FUNCTIONS
 *
 *  @retval     None
 */
void SOC_rcmCoreDpllConfig(SOC_RcmPllFoutFreqId outFreqId, SOC_RcmPllHsDivOutConfig *hsDivCfg)
{
    SOC_RcmXtalFreqId XTALFreq;
    uint16_t coreTrimVal;
    SOC_RcmADPLLJConfig_t const * adplljCfg;

    /* read the Core ADPLL trim value */
    coreTrimVal = SOC_rcmGetCoreTrimVal();

    /* read the XTAL Frequency */
    XTALFreq = SOC_rcmGetXTALFrequency();

    /* program PLL dividers and multipliers. The value are taken from  TPR12_ADPLLJ_Settings_1p0.xlsx */
    adplljCfg = SOC_rcmGetADPLLJConfig(gSocRcmXTALInfo[XTALFreq].Finp, outFreqId);
    DebugP_assert(adplljCfg != NULL);

    if(adplljCfg != NULL)
    {
        if(gSocRcmXTALInfo[XTALFreq].div2flag == FALSE)
        {
            SOC_rcmProgPllCoreDivider (adplljCfg->N,
                                0U /* N2 divider for bypass */,
                                adplljCfg->M,
                                adplljCfg->M2,
                                adplljCfg->FracM,
                                adplljCfg->sdDiv);
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
                                adplljCfg->FracM,
                                adplljCfg->sdDiv);
        }
        /* Configure and Lock Core PLL */
        SOC_rcmConfigurePllCore(coreTrimVal);
        SOC_rcmCoreApllHSDivConfig(hsDivCfg);
    }

    return;
}

void SOC_rcmApllHSDivDisableOutput(SOC_RcmApllId apllId, uint32_t  hsDivIdx)
{
    CSL_mss_toprcmRegs *ptrTopRCMRegs;
    volatile uint32_t *regHsDivClkOut = NULL;

    ptrTopRCMRegs = CSL_TopRCM_getBaseAddress();
    if(apllId == SOC_RcmAPLLID_1P2G)
    {
        switch (hsDivIdx)
        {
            case SOC_RCM_PLL_HSDIV_OUTPUT_IDX0:
                regHsDivClkOut = &ptrTopRCMRegs->PLL_1P2_HSDIVIDER_CLKOUT0;
                break;
            case SOC_RCM_PLL_HSDIV_OUTPUT_IDX1:
                regHsDivClkOut = &ptrTopRCMRegs->PLL_1P2_HSDIVIDER_CLKOUT1;
                break;
            case SOC_RCM_PLL_HSDIV_OUTPUT_IDX2:
                regHsDivClkOut = &ptrTopRCMRegs->PLL_1P2_HSDIVIDER_CLKOUT2;
                break;
            case SOC_RCM_PLL_HSDIV_OUTPUT_IDX3:
                regHsDivClkOut = &ptrTopRCMRegs->PLL_1P2_HSDIVIDER_CLKOUT3;
                break;
            default:
                regHsDivClkOut = NULL;
                break;
        }
    }
    if(apllId == SOC_RcmAPLLID_1P8G)
    {
        switch (hsDivIdx)
        {
            case SOC_RCM_PLL_HSDIV_OUTPUT_IDX0:
                regHsDivClkOut = &ptrTopRCMRegs->PLL_1P8_HSDIVIDER_CLKOUT0;
                break;
            case SOC_RCM_PLL_HSDIV_OUTPUT_IDX1:
                regHsDivClkOut = &ptrTopRCMRegs->PLL_1P8_HSDIVIDER_CLKOUT1;
                break;
            case SOC_RCM_PLL_HSDIV_OUTPUT_IDX2:
                regHsDivClkOut = &ptrTopRCMRegs->PLL_1P8_HSDIVIDER_CLKOUT2;
                break;
            case SOC_RCM_PLL_HSDIV_OUTPUT_IDX3:
                regHsDivClkOut = &ptrTopRCMRegs->PLL_1P8_HSDIVIDER_CLKOUT3;
                break;
            default:
                regHsDivClkOut = NULL;
                break;
        }
    }
    if (regHsDivClkOut != NULL)
    {
        *regHsDivClkOut = SOC_rcmInsert32 (*regHsDivClkOut, 8U, 8U, 0);
        while(SOC_rcmExtract32(*regHsDivClkOut, 9U, 9U) != 0);
        *regHsDivClkOut = SOC_rcmInsert32 (*regHsDivClkOut, 8U, 8U, 0);
    }
}

void SOC_rcmDpllHSDivDisableOutput(SOC_RcmDpllId_e dpllId, uint32_t  hsDivIdx)
{
    CSL_mss_toprcmRegs *ptrTopRCMRegs;
    volatile uint32_t *regHsDivClkOut = NULL;

    ptrTopRCMRegs = CSL_TopRCM_getBaseAddress();
    if(dpllId == SOC_RCM_DPLL_CORE)
    {
        switch (hsDivIdx)
        {
            case SOC_RCM_PLL_HSDIV_OUTPUT_IDX0:
                regHsDivClkOut = &ptrTopRCMRegs->PLL_CORE_HSDIVIDER_CLKOUT0;
                break;
            case SOC_RCM_PLL_HSDIV_OUTPUT_IDX1:
                regHsDivClkOut = &ptrTopRCMRegs->PLL_CORE_HSDIVIDER_CLKOUT1;
                break;
            case SOC_RCM_PLL_HSDIV_OUTPUT_IDX2:
                regHsDivClkOut = &ptrTopRCMRegs->PLL_CORE_HSDIVIDER_CLKOUT2;
                break;
            case SOC_RCM_PLL_HSDIV_OUTPUT_IDX3:
                regHsDivClkOut = &ptrTopRCMRegs->PLL_CORE_HSDIVIDER_CLKOUT3;
                break;
            default:
                regHsDivClkOut = NULL;
                break;
        }
    }
    if(dpllId == SOC_RCM_DPLL_DSS)
    {
        switch (hsDivIdx)
        {
            case SOC_RCM_PLL_HSDIV_OUTPUT_IDX0:
                regHsDivClkOut = &ptrTopRCMRegs->PLL_DSP_HSDIVIDER_CLKOUT0;
                break;
            case SOC_RCM_PLL_HSDIV_OUTPUT_IDX1:
                regHsDivClkOut = &ptrTopRCMRegs->PLL_DSP_HSDIVIDER_CLKOUT1;
                break;
            case SOC_RCM_PLL_HSDIV_OUTPUT_IDX2:
                regHsDivClkOut = &ptrTopRCMRegs->PLL_DSP_HSDIVIDER_CLKOUT2;
                break;
            case SOC_RCM_PLL_HSDIV_OUTPUT_IDX3:
                regHsDivClkOut = &ptrTopRCMRegs->PLL_DSP_HSDIVIDER_CLKOUT3;
                break;
            default:
                regHsDivClkOut = NULL;
                break;
        }
    }
    if(dpllId == SOC_RCM_DPLL_PER)
    {
        switch (hsDivIdx)
        {
            case SOC_RCM_PLL_HSDIV_OUTPUT_IDX0:
                regHsDivClkOut = &ptrTopRCMRegs->PLL_PER_HSDIVIDER_CLKOUT0;
                break;
            case SOC_RCM_PLL_HSDIV_OUTPUT_IDX1:
                regHsDivClkOut = &ptrTopRCMRegs->PLL_PER_HSDIVIDER_CLKOUT1;
                break;
            case SOC_RCM_PLL_HSDIV_OUTPUT_IDX2:
                regHsDivClkOut = &ptrTopRCMRegs->PLL_PER_HSDIVIDER_CLKOUT2;
                break;
            case SOC_RCM_PLL_HSDIV_OUTPUT_IDX3:
                regHsDivClkOut = &ptrTopRCMRegs->PLL_PER_HSDIVIDER_CLKOUT3;
                break;
            default:
                regHsDivClkOut = NULL;
                break;
        }
    }
    if (regHsDivClkOut != NULL)
    {
        *regHsDivClkOut = SOC_rcmInsert32 (*regHsDivClkOut, 8U, 8U, 0);
        while(SOC_rcmExtract32(*regHsDivClkOut, 9U, 9U) != 0);
        *regHsDivClkOut = SOC_rcmInsert32 (*regHsDivClkOut, 12U, 12U, 1);
    }
}

void SOC_rcmApllHSDivConfig(SOC_RcmApllId apllId, SOC_RcmPllHsDivOutConfig *hsDivCfg)
{
    CSL_mss_toprcmRegs *ptrTopRCMRegs;
    uint32_t hsDivOutRegVal;
    uint32_t FOut;
    volatile uint32_t *regHsDivClkOut0;
    volatile uint32_t *regHsDivClkOut1;
    volatile uint32_t *regHsDivClkOut2;
    volatile uint32_t *regHsDivClkOut3;
    volatile uint32_t *regHsDiv;

    ptrTopRCMRegs = CSL_TopRCM_getBaseAddress();
    if(apllId == SOC_RcmAPLLID_1P2G)
    {
        FOut = gSocRcmFixedClocksTbl[SOC_RCM_FIXEDCLKID_APLL1P2GHZ].fOut;
        regHsDivClkOut0 = &ptrTopRCMRegs->PLL_1P2_HSDIVIDER_CLKOUT0;
        regHsDivClkOut1 = &ptrTopRCMRegs->PLL_1P2_HSDIVIDER_CLKOUT1;
        regHsDivClkOut2 = &ptrTopRCMRegs->PLL_1P2_HSDIVIDER_CLKOUT2;
        regHsDivClkOut3 = &ptrTopRCMRegs->PLL_1P2_HSDIVIDER_CLKOUT3;
        regHsDiv        = &ptrTopRCMRegs->PLL_1P2_HSDIVIDER;
    }
    else
    {
        FOut = gSocRcmFixedClocksTbl[SOC_RCM_FIXEDCLKID_APLL1P8GHZ].fOut;
        regHsDivClkOut0 = &ptrTopRCMRegs->PLL_1P8_HSDIVIDER_CLKOUT0;
        regHsDivClkOut1 = &ptrTopRCMRegs->PLL_1P8_HSDIVIDER_CLKOUT1;
        regHsDivClkOut2 = &ptrTopRCMRegs->PLL_1P8_HSDIVIDER_CLKOUT2;
        regHsDivClkOut3 = &ptrTopRCMRegs->PLL_1P8_HSDIVIDER_CLKOUT3;
        regHsDiv        = &ptrTopRCMRegs->PLL_1P8_HSDIVIDER;
    }

    if(hsDivCfg->hsdivOutEnMask & SOC_RCM_PLL_HSDIV_OUTPUT_ENABLE_0)
    {
        DebugP_assert((FOut % hsDivCfg->hsDivOutFreqHz[SOC_RCM_PLL_HSDIV_OUTPUT_IDX0]) == 0);
        hsDivOutRegVal = FOut / hsDivCfg->hsDivOutFreqHz[SOC_RCM_PLL_HSDIV_OUTPUT_IDX0];
        hsDivOutRegVal--;
        *regHsDivClkOut0 = SOC_rcmInsert8 (*regHsDivClkOut0, 4U, 0U, hsDivOutRegVal);
    }
    if(hsDivCfg->hsdivOutEnMask & SOC_RCM_PLL_HSDIV_OUTPUT_ENABLE_1)
    {
        DebugP_assert((FOut % hsDivCfg->hsDivOutFreqHz[SOC_RCM_PLL_HSDIV_OUTPUT_IDX1]) == 0);
        hsDivOutRegVal = FOut / hsDivCfg->hsDivOutFreqHz[SOC_RCM_PLL_HSDIV_OUTPUT_IDX1];
        hsDivOutRegVal--;
        *regHsDivClkOut1 = SOC_rcmInsert8 (*regHsDivClkOut1, 4U, 0U, hsDivOutRegVal);
    }
    if(hsDivCfg->hsdivOutEnMask & SOC_RCM_PLL_HSDIV_OUTPUT_ENABLE_2)
    {
        DebugP_assert((FOut % hsDivCfg->hsDivOutFreqHz[SOC_RCM_PLL_HSDIV_OUTPUT_IDX2]) == 0);
        hsDivOutRegVal = FOut / hsDivCfg->hsDivOutFreqHz[SOC_RCM_PLL_HSDIV_OUTPUT_IDX2];
        hsDivOutRegVal--;
        *regHsDivClkOut2 = SOC_rcmInsert8 (*regHsDivClkOut2, 4U, 0U, hsDivOutRegVal);
    }
    if(hsDivCfg->hsdivOutEnMask & SOC_RCM_PLL_HSDIV_OUTPUT_ENABLE_3)
    {
        DebugP_assert((FOut % hsDivCfg->hsDivOutFreqHz[SOC_RCM_PLL_HSDIV_OUTPUT_IDX3]) == 0);
        hsDivOutRegVal = FOut / hsDivCfg->hsDivOutFreqHz[SOC_RCM_PLL_HSDIV_OUTPUT_IDX3];
        hsDivOutRegVal--;
        *regHsDivClkOut3 = SOC_rcmInsert8 (*regHsDivClkOut3, 4U, 0U, hsDivOutRegVal);
    }
    /* Generate Trigger to latch these values */
    *regHsDiv = SOC_rcmInsert8 (*regHsDiv, 2U, 2U, 0x1U);
    *regHsDiv = SOC_rcmInsert8 (*regHsDiv, 2U, 2U, 0x0U);

    /* Ungate the clocks */
    if(hsDivCfg->hsdivOutEnMask & SOC_RCM_PLL_HSDIV_OUTPUT_ENABLE_0)
    {
        *regHsDivClkOut0 = SOC_rcmInsert8 (*regHsDivClkOut0, 8U, 8U, 0x1U);
    }
    if(hsDivCfg->hsdivOutEnMask & SOC_RCM_PLL_HSDIV_OUTPUT_ENABLE_1)
    {
        *regHsDivClkOut1 = SOC_rcmInsert8 (*regHsDivClkOut1, 8U, 8U, 0x1U);
    }
    if(hsDivCfg->hsdivOutEnMask & SOC_RCM_PLL_HSDIV_OUTPUT_ENABLE_2)
    {
        *regHsDivClkOut2 = SOC_rcmInsert8 (*regHsDivClkOut2, 8U, 8U, 0x1U);
    }
    if(hsDivCfg->hsdivOutEnMask & SOC_RCM_PLL_HSDIV_OUTPUT_ENABLE_3)
    {
        *regHsDivClkOut3 = SOC_rcmInsert8 (*regHsDivClkOut3, 8U, 8U, 0x1U);
    }
    return;
}

static uint32_t SOC_rcmGetR5InFrequency(void)
{
    uint32_t clkFreq = 0U;
    SOC_RcmClockSrcId rootClkSrcId;
    SOC_RcmR5ClockSource r5ClkSrcId;
    CSL_mss_toprcmRegs *ptrTopRCMRegs;
    uint32_t clkSrcVal;

    /* Core PLL settings */
    ptrTopRCMRegs = CSL_TopRCM_getBaseAddress();
    /* Select CLKOUT2 as clock for R5 Core */
    clkSrcVal = SOC_rcmExtract16 (ptrTopRCMRegs->MSS_CR5_CLK_SRC_SEL, 11U, 0U);
    r5ClkSrcId = (SOC_RcmR5ClockSource) SOC_rcmGetClkSrcFromClkSelVal(gSocRcmR5ClkSrcValMap, SOC_RCM_UTILS_ARRAYSIZE(gSocRcmR5ClkSrcValMap), clkSrcVal);
    DebugP_assert(r5ClkSrcId != SOC_RcmR5ClockSource_MAX_VALUE);
    rootClkSrcId = gSocRcmR5ClkSrcInfoMap[r5ClkSrcId];
    clkFreq= SOC_rcmGetFreq(rootClkSrcId);

    return (clkFreq);
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

int32_t SOC_rcmSetR5Clock(uint32_t r5FreqHz, uint32_t sysClkFreqHz)
{
    CSL_mss_toprcmRegs *ptrTopRCMRegs;
    uint32_t Finp;
    uint32_t moduleClkDivVal;

    Finp = SOC_rcmGetR5InFrequency();
    moduleClkDivVal = SOC_rcmGetModuleClkDivVal(Finp, r5FreqHz);

    ptrTopRCMRegs = CSL_TopRCM_getBaseAddress();

    /* Divide by 1 to get the R5 Core Clock */
    ptrTopRCMRegs->MSS_CR5_DIV_VAL = SOC_rcmInsert16 (ptrTopRCMRegs->MSS_CR5_DIV_VAL, 11U, 0U, SOC_rcmGetModuleClkDivRegVal(moduleClkDivVal));

    /* Divide by 2 to get the VCLK */
    moduleClkDivVal = SOC_rcmGetModuleClkDivVal(Finp, sysClkFreqHz);

    ptrTopRCMRegs->SYS_CLK_DIV_VAL = SOC_rcmInsert16 (ptrTopRCMRegs->SYS_CLK_DIV_VAL, 11U, 0U, SOC_rcmGetModuleClkDivRegVal(moduleClkDivVal));

    /* Select CLKOUT2 as clock for R5 Core */
    ptrTopRCMRegs->MSS_CR5_CLK_SRC_SEL = SOC_rcmInsert16 (ptrTopRCMRegs->MSS_CR5_CLK_SRC_SEL, 11U, 0U, gSocRcmR5ClkSrcValMap[SOC_RcmR5ClockSource_DPLL_CORE_HSDIV0_CLKOUT2]);

    return SystemP_SUCCESS;
}

uint32_t SOC_rcmGetR5Clock(void)
{
    uint32_t Finp;
    uint32_t moduleClkDivRegVal;
    uint32_t clkDivVal;
    CSL_mss_toprcmRegs *ptrTopRCMRegs;

    ptrTopRCMRegs = CSL_TopRCM_getBaseAddress();
    Finp = SOC_rcmGetR5InFrequency();
    moduleClkDivRegVal = ptrTopRCMRegs->MSS_CR5_DIV_VAL;
    clkDivVal = SOC_rcmGetModuleClkDivFromRegVal(moduleClkDivRegVal);
    return (Finp / clkDivVal);
}

int32_t SOC_rcmSetPeripheralClock(SOC_RcmPeripheralId periphID,
                                  SOC_RcmPeripheralClockSource clkSource,
                                  uint32_t freqHz)
{
    volatile uint32_t   *ptrClkSrcReg, *ptrClkDivReg;
    uint16_t            clkSrcVal;
    uint32_t            clkDivisor;
    int32_t          retVal;
    uint32_t            Finp;

    Finp = SOC_rcmGetPeripheralClockFrequency(clkSource);
    clkDivisor = SOC_rcmGetModuleClkDivVal(Finp, freqHz);
    SOC_rcmGetClkSrcAndDivReg (periphID, clkSource, &clkSrcVal, &ptrClkSrcReg, &ptrClkDivReg);

    if((ptrClkSrcReg != NULL)  && (clkSrcVal != 0x888U))
    {
        uint16_t            clkDivVal;

        /* Create the Divider Value to be programmed */
        clkDivVal = ((uint16_t)clkDivisor & 0xFU);
        clkDivVal = (clkDivVal | (clkDivVal << 4U) | (clkDivVal << 8U));

        /* Some registers like HWA do not have ClkDiv reg */
        if(ptrClkDivReg != NULL)
        {
            /* Write the Divider Value */
            *ptrClkDivReg = SOC_rcmInsert16 (*ptrClkDivReg, 11U, 0U, clkDivVal);
        }
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

uint32_t SOC_rcmGetPeripheralClock(SOC_RcmPeripheralId periphID)
{
    uint32_t            clkDivisorRegVal;
    uint32_t            clkDivisor;
    int32_t             retVal;
    uint32_t            Finp;
    SOC_RcmPeripheralClockSource clkSource;
    uint32_t            freqHz = 1U;

    retVal = SOC_rcmGetClkSrcAndDivValue(periphID, &clkSource, &clkDivisorRegVal);
    if(SystemP_SUCCESS == retVal)
    {
        Finp = SOC_rcmGetPeripheralClockFrequency(clkSource);
        clkDivisor = SOC_rcmGetModuleClkDivFromRegVal(clkDivisorRegVal);
        freqHz = Finp / clkDivisor;
    }
    return (freqHz);
}

int32_t SOC_rcmSetHSIClock(SOC_RcmHSIClockSource clkSource,
                           uint32_t freqHz)
{
    volatile uint32_t   *ptrClkSrcReg, *ptrClkDivReg;
    uint16_t            clkSrcVal;
    uint32_t            clkDivisor;
    int32_t          retVal;
    uint32_t            Finp;
    SOC_RcmClockSrcId       rootClkId;

    rootClkId = gSocRcmHsiClkSrcInfoMap[clkSource];
    Finp = SOC_rcmGetFreq(rootClkId);
    clkDivisor = SOC_rcmGetModuleClkDivVal(Finp, freqHz);
    SOC_rcmGetHSIClkSrcAndDivReg (clkSource, &clkSrcVal, &ptrClkSrcReg, &ptrClkDivReg);

    if((ptrClkSrcReg != NULL)  && (clkSrcVal != 0x888U))
    {
        uint16_t            clkDivVal;

        /* Create the Divider Value to be programmed */
        clkDivVal = ((uint16_t)clkDivisor & 0xFU);
        clkDivVal = (clkDivVal | (clkDivVal << 4U) | (clkDivVal << 8U));

        /* Some registers like HWA do not have ClkDiv reg */
        if(ptrClkDivReg != NULL)
        {
            /* Write the Divider Value */
            *ptrClkDivReg = SOC_rcmInsert16 (*ptrClkDivReg, 11U, 0U, clkDivVal);
        }
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

int32_t SOC_rcmSetRssBssFrcClock(SOC_RcmRssBssFrcClockSource clkSource, uint32_t freqHz)
{
    volatile uint32_t   *ptrClkSrcReg, *ptrClkDivReg;
    uint16_t            clkSrcVal;
    uint32_t            clkDivisor;
    int32_t          retVal;
    uint32_t            Finp;
    SOC_RcmClockSrcId       rootClkId;

    rootClkId = gSocRcmRssBssFrcClkSrcInfoMap[clkSource];
    Finp = SOC_rcmGetFreq(rootClkId);
    clkDivisor = SOC_rcmGetModuleClkDivVal(Finp, freqHz);
    SOC_rcmGetRssBssFrcClkSrcAndDivReg (clkSource, &clkSrcVal, &ptrClkSrcReg, &ptrClkDivReg);

    if((ptrClkSrcReg != NULL)  && (clkSrcVal != 0x888U))
    {
        uint16_t            clkDivVal;

        /* Create the Divider Value to be programmed */
        clkDivVal = ((uint16_t)clkDivisor & 0xFU);
        clkDivVal = (clkDivVal | (clkDivVal << 4U) | (clkDivVal << 8U));

        /* Some registers like HWA do not have ClkDiv reg */
        if(ptrClkDivReg != NULL)
        {
            /* Write the Divider Value */
            *ptrClkDivReg = SOC_rcmInsert16 (*ptrClkDivReg, 11U, 0U, clkDivVal);
        }
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

SOC_RcmResetCause SOC_rcmGetResetCause(void)
{
    CSL_mss_rcmRegs *ptrRCMRegs;
    uint16_t    resetCauseBits;
    uint8_t     resetCause = 0U;

    ptrRCMRegs = CSL_RCM_getBaseAddress();

    /* Read the Reset Cause Register bits */
    resetCauseBits = SOC_rcmExtract16 (ptrRCMRegs->MSS_RST_STATUS, 9U, 0U);

    if(resetCauseBits == 0x0U)
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

static uint32_t SOC_rcmGetPeripheralClockFrequency(SOC_RcmPeripheralClockSource clkSource)
{
    uint32_t clkFreq = 0U;
    SOC_RcmClockSrcId rootClkSrcId;

    rootClkSrcId = gSocRcmPeripheralClkSrcInfoMap[clkSource];
    clkFreq = SOC_rcmGetFreq(rootClkSrcId);
    return (clkFreq);
}

void SOC_rcmStartMemInitTCMA(void)
{
    CSL_mss_ctrlRegs *mssCtrl = CSL_MSS_CTRL_getBaseAddress();

    /* Check MEMINIT STATUS is zero to confirm no inprogress MEM INIT */
    while (CSL_FEXT(mssCtrl->MSS_ATCM_MEM_INIT_STATUS, MSS_CTRL_MSS_ATCM_MEM_INIT_STATUS_MSS_ATCM_MEM_INIT_STATUS_MEM_STATUS) != 0);

    /* Clear MEMINIT DONE before initiating MEMINIT */
    CSL_FINS(mssCtrl->MSS_ATCM_MEM_INIT_DONE, MSS_CTRL_MSS_ATCM_MEM_INIT_DONE_MSS_ATCM_MEM_INIT_DONE_MEM_INIT_DONE, 1);
    while (CSL_FEXT(mssCtrl->MSS_ATCM_MEM_INIT_DONE, MSS_CTRL_MSS_ATCM_MEM_INIT_DONE_MSS_ATCM_MEM_INIT_DONE_MEM_INIT_DONE) != 0);

    CSL_FINS(mssCtrl->MSS_ATCM_MEM_INIT, MSS_CTRL_MSS_ATCM_MEM_INIT_MSS_ATCM_MEM_INIT_MEM_INIT, 1);

}

void SOC_rcmStartMeminitTCMBSS(void)
{
    CSL_rss_ctrlRegs *rssCtrl = CSL_RSS_CTRL_getBaseAddress();

    /* Check MEMINIT STATUS is zero to confirm no inprogress MEM INIT */
    while (CSL_FEXT(rssCtrl->BSS_TCM_MEMINIT_STATUS, MSS_CTRL_MSS_ATCM_MEM_INIT_STATUS_MSS_ATCM_MEM_INIT_STATUS_MEM_STATUS) != 0);

    /* Clear MEMINIT DONE before initiating MEMINIT */
    CSL_FINS(rssCtrl->BSS_TCM_MEMINIT_DONE, MSS_CTRL_MSS_ATCM_MEM_INIT_DONE_MSS_ATCM_MEM_INIT_DONE_MEM_INIT_DONE, 1);
    while (CSL_FEXT(rssCtrl->BSS_TCM_MEMINIT_DONE, MSS_CTRL_MSS_ATCM_MEM_INIT_DONE_MSS_ATCM_MEM_INIT_DONE_MEM_INIT_DONE) != 0);

    CSL_FINS(rssCtrl->BSS_TCM_MEMINIT, MSS_CTRL_MSS_ATCM_MEM_INIT_MSS_ATCM_MEM_INIT_MEM_INIT, 1);
}

void SOC_rcmStartMeminitStaticBSS(void)
{
    CSL_rss_proc_ctrlRegs *rssProcCtrl = CSL_RSS_PROC_CTRL_getBaseAddress();

    /* Check MEMINIT STATUS is zero to confirm no inprogress MEM INIT */
    while (CSL_FEXT(rssProcCtrl->RSS_STATIC_MEM_MEMINIT_STATUS, MSS_CTRL_MSS_ATCM_MEM_INIT_STATUS_MSS_ATCM_MEM_INIT_STATUS_MEM_STATUS) != 0);

    /* Clear MEMINIT DONE before initiating MEMINIT */
    CSL_FINS(rssProcCtrl->RSS_STATIC_MEM_MEMINIT_DONE, MSS_CTRL_MSS_ATCM_MEM_INIT_DONE_MSS_ATCM_MEM_INIT_DONE_MEM_INIT_DONE, 1);
    while (CSL_FEXT(rssProcCtrl->RSS_STATIC_MEM_MEMINIT_DONE, MSS_CTRL_MSS_ATCM_MEM_INIT_DONE_MSS_ATCM_MEM_INIT_DONE_MEM_INIT_DONE) != 0);

    CSL_FINS(rssProcCtrl->RSS_STATIC_MEM_MEMINIT, MSS_CTRL_MSS_ATCM_MEM_INIT_MSS_ATCM_MEM_INIT_MEM_INIT, 1);
}

void SOC_rcmStartMeminitSharedBSS(void)
{
    CSL_rss_ctrlRegs *rssCtrl = CSL_RSS_CTRL_getBaseAddress();

    /* Check MEMINIT STATUS is zero to confirm no inprogress MEM INIT */
    while (CSL_FEXT(rssCtrl->RSS_SHARED_MEM_MEMINIT_STATUS, MSS_CTRL_MSS_ATCM_MEM_INIT_STATUS_MSS_ATCM_MEM_INIT_STATUS_MEM_STATUS) != 0);

    /* Clear MEMINIT DONE before initiating MEMINIT */
    CSL_FINS(rssCtrl->RSS_SHARED_MEM_MEMINIT_DONE, MSS_CTRL_MSS_ATCM_MEM_INIT_DONE_MSS_ATCM_MEM_INIT_DONE_MEM_INIT_DONE, 1);
    while (CSL_FEXT(rssCtrl->RSS_SHARED_MEM_MEMINIT_DONE, MSS_CTRL_MSS_ATCM_MEM_INIT_DONE_MSS_ATCM_MEM_INIT_DONE_MEM_INIT_DONE) != 0);

    CSL_FINS(rssCtrl->RSS_SHARED_MEM_MEMINIT, MSS_CTRL_MSS_ATCM_MEM_INIT_MSS_ATCM_MEM_INIT_MEM_INIT, 1);
}

void SOC_rcmBSSControl(void)
{
    CSL_rss_ctrlRegs *rssCtrl = CSL_RSS_CTRL_getBaseAddress();
    CSL_dss_rcmRegs  *ptrDSSRcmRegs = CSL_DSSRCM_getBaseAddress();
    CSL_top_ctrlRegs* ptrTopCtrlRegs = CSL_TopCtrl_getBaseAddress();

    /* Check if the device is ES1.0 or ES2.0 */
    if(CSL_TopCtrl_readPgVerEfuse(ptrTopCtrlRegs) == SOC_RCM_ES2_PG_VER)
    {
        /* Check if the device is ES2.0 */

        /* Do not allocate DSS L3 Bank as TCM for BSS */
        CSL_FINSR(rssCtrl->BSS_CONTROL, 30, 28, 0x0U);
        /* Sets normal boot mode for CR4 */
        CSL_FINSR(rssCtrl->BSS_CONTROL, 11, 0, 0x0U);
    }
    else
    {
        /* Allocate DSS L3 Bank as TCM for BSS */
        CSL_FINSR(rssCtrl->BSS_CONTROL, 30, 28, 0x7U);
        /* Sets FW development mode */
        CSL_FINSR(rssCtrl->BSS_CONTROL, 11, 0, 0x111U);

        /* add 1msec delay. */
        ClockP_usleep(1000);

        /* workaround for AWR2943 memory allocation issue */
        CSL_FINSR(ptrDSSRcmRegs->DSS_L3_BANKD1_PD_CTRL, 2, 0, 0x0U);
    }
}

void SOC_rcmPopulateBSSControl(void)
{
    CSL_rss_proc_ctrlRegs *rssProcCtrl = CSL_RSS_PROC_CTRL_getBaseAddress();

    /* Clear RSS Boot info registers. */
    rssProcCtrl->RSS_CR4_BOOT_INFO_REG0 = 0x0U;
    rssProcCtrl->RSS_CR4_BOOT_INFO_REG1 = 0x0U;
    rssProcCtrl->RSS_CR4_BOOT_INFO_REG2 = 0x0U;
    rssProcCtrl->RSS_CR4_BOOT_INFO_REG3 = 0x0U;
    rssProcCtrl->RSS_CR4_BOOT_INFO_REG4 = 0x0U;
    rssProcCtrl->RSS_CR4_BOOT_INFO_REG5 = 0x0U;
    rssProcCtrl->RSS_CR4_BOOT_INFO_REG6 = 0x0U;
    rssProcCtrl->RSS_CR4_BOOT_INFO_REG7 = 0x0U;

    /*  Boot status register
        Bits 15:0 -> XTAL frequency in MHz as an unsigned number */
    CSL_FINSR(rssProcCtrl->RSS_CR4_BOOT_INFO_REG0, 15, 0, 0x28U);
    /*  Redundant configuration as above */
    CSL_FINSR(rssProcCtrl->RSS_CR4_BOOT_INFO_REG1, 15, 0, 0x28U);
    /*
        Boot time Mailbox Memory configuration
        Bits 7:0 -> MSS
        Bits 15:8 -> DSS
                              MSS CR5 A	  DSS DSP
        Tx Buffer Size        512 B	      512 B
        Tx Buffer Offset      0x0000      0x0000
    */
    CSL_FINSR(rssProcCtrl->RSS_CR4_BOOT_INFO_REG3, 15, 0, 0x8080U);
    /*  Redundant configuration as above */
    CSL_FINSR(rssProcCtrl->RSS_CR4_BOOT_INFO_REG4, 15, 0, 0x8080U);

    /* Configure BSS Logger: current configuration is Disabled. */
	/* To Enable the BSS logger set 2:0 field of RSS_CR4_BOOT_INFO_REG5 register to 0x1. */
	/* The debug data is transferred to a programmable MSS L2 memory buffer with a size of 2KB. */
	/* The BSS splits this 2KB into two halves (ping and pong) to copy the debug logger data. */
    CSL_FINSR(rssProcCtrl->RSS_CR4_BOOT_INFO_REG5, 2, 0, 0);

    /* Configure MSS L2 offset for BSS Logger. */
	/* 0xC0260000 is the transaled address of MSS L2 memory 0x10260000. */
	/* 2KB of memory is needed for BSS to transfer the BSS logger data. */
	/* Applications are required to make sure the 2KB starting from 0x10260000 is reserved for BSS logger. */
	CSL_REG_WR(&rssProcCtrl->RSS_CR4_BOOT_INFO_REG6, 0xC0260000);

    /* BSS dynamic frequency switching feature control - Enable */
    /*  0b0 - BSS CPU clock is always kept at 200MHz. */
    /*  0b1 - BSS CPU clock frequency will be switched to 40MHz when */
    /*        going to sleep and will be switched back to 200MHz on waking up. */
    CSL_FINSR(rssProcCtrl->RSS_CR4_BOOT_INFO_REG5, 3, 3, 1);

    /* BSS CPU clock source select configuration.*/
    /* 12-bit RSS CPU clock source selection at MSS_TOPRCM→RSS_CLK_SRC_SEL. */
    /* SBL configures DPLL_PER_HSDIV0_CLKOUT1_MUXED (0x333U) as the clock source for RSS */
    CSL_FINSR(rssProcCtrl->RSS_CR4_BOOT_INFO_REG5, 15, 4, 0x333U);

    SOC_rcmSetPeripheralClock(SOC_RcmPeripheralId_MSS_RTIC, SOC_RcmPeripheralClockSource_XTAL_CLK, 40000000);

}

void SOC_rcmStartMeminitL2(void)
{
    CSL_mss_ctrlRegs*        ptrMSSCtrlRegs;

    ptrMSSCtrlRegs = CSL_MSS_CTRL_getBaseAddress();

    /* Start the Initialization of L2 Memory */
    ptrMSSCtrlRegs->MSS_L2_MEM_INIT = SOC_rcmInsert8 (ptrMSSCtrlRegs->MSS_L2_MEM_INIT, 1U, 0U, 0x3U);

}

void SOC_rcmWaitMemInitTCMA(void)
{
    CSL_mss_ctrlRegs*        mssCtrl;

    mssCtrl = CSL_MSS_CTRL_getBaseAddress();

    while (CSL_FEXT(mssCtrl->MSS_ATCM_MEM_INIT_DONE, MSS_CTRL_MSS_ATCM_MEM_INIT_DONE_MSS_ATCM_MEM_INIT_DONE_MEM_INIT_DONE) != 1);
    CSL_FINS(mssCtrl->MSS_ATCM_MEM_INIT_DONE, MSS_CTRL_MSS_ATCM_MEM_INIT_DONE_MSS_ATCM_MEM_INIT_DONE_MEM_INIT_DONE, 1);
    /* Check MEMINIT STATUS is zero to confirm no inprogress MEM INIT */
    while (CSL_FEXT(mssCtrl->MSS_ATCM_MEM_INIT_STATUS, MSS_CTRL_MSS_ATCM_MEM_INIT_STATUS_MSS_ATCM_MEM_INIT_STATUS_MEM_STATUS) != 0);
    while (CSL_FEXT(mssCtrl->MSS_ATCM_MEM_INIT_DONE, MSS_CTRL_MSS_ATCM_MEM_INIT_DONE_MSS_ATCM_MEM_INIT_DONE_MEM_INIT_DONE) != 0);
}

void SOC_rcmWaitMeminitTCMBSS(void)
{
    CSL_rss_ctrlRegs *rssCtrl = CSL_RSS_CTRL_getBaseAddress();

    while (CSL_FEXT(rssCtrl->BSS_TCM_MEMINIT_DONE, MSS_CTRL_MSS_ATCM_MEM_INIT_DONE_MSS_ATCM_MEM_INIT_DONE_MEM_INIT_DONE) != 1);
    CSL_FINS(rssCtrl->BSS_TCM_MEMINIT_DONE, MSS_CTRL_MSS_ATCM_MEM_INIT_DONE_MSS_ATCM_MEM_INIT_DONE_MEM_INIT_DONE, 1);

    /* Check MEMINIT STATUS is zero to confirm no inprogress MEM INIT */
    while (CSL_FEXT(rssCtrl->BSS_TCM_MEMINIT_STATUS, MSS_CTRL_MSS_ATCM_MEM_INIT_STATUS_MSS_ATCM_MEM_INIT_STATUS_MEM_STATUS) != 0);
    while (CSL_FEXT(rssCtrl->BSS_TCM_MEMINIT_DONE, MSS_CTRL_MSS_ATCM_MEM_INIT_DONE_MSS_ATCM_MEM_INIT_DONE_MEM_INIT_DONE) != 0);
}

void SOC_rcmWaitMeminitStaticBSS(void)
{
    CSL_rss_proc_ctrlRegs *rssProcCtrl = CSL_RSS_PROC_CTRL_getBaseAddress();

    while (CSL_FEXT(rssProcCtrl->RSS_STATIC_MEM_MEMINIT_DONE, MSS_CTRL_MSS_ATCM_MEM_INIT_DONE_MSS_ATCM_MEM_INIT_DONE_MEM_INIT_DONE) != 1);
    CSL_FINS(rssProcCtrl->RSS_STATIC_MEM_MEMINIT_DONE, MSS_CTRL_MSS_ATCM_MEM_INIT_DONE_MSS_ATCM_MEM_INIT_DONE_MEM_INIT_DONE, 1);

    /* Check MEMINIT STATUS is zero to confirm no inprogress MEM INIT */
    while (CSL_FEXT(rssProcCtrl->RSS_STATIC_MEM_MEMINIT_STATUS, MSS_CTRL_MSS_ATCM_MEM_INIT_STATUS_MSS_ATCM_MEM_INIT_STATUS_MEM_STATUS) != 0);
    while (CSL_FEXT(rssProcCtrl->RSS_STATIC_MEM_MEMINIT_DONE, MSS_CTRL_MSS_ATCM_MEM_INIT_DONE_MSS_ATCM_MEM_INIT_DONE_MEM_INIT_DONE) != 0);
    /* Write 0 after mem init is completed */
    CSL_FINS(rssProcCtrl->RSS_STATIC_MEM_MEMINIT, MSS_CTRL_MSS_ATCM_MEM_INIT_MSS_ATCM_MEM_INIT_MEM_INIT, 0);
}

void SOC_rcmWaitMeminitSharedBSS(void)
{
    CSL_rss_ctrlRegs *rssCtrl = CSL_RSS_CTRL_getBaseAddress();

    while (CSL_FEXT(rssCtrl->RSS_SHARED_MEM_MEMINIT_DONE, MSS_CTRL_MSS_ATCM_MEM_INIT_DONE_MSS_ATCM_MEM_INIT_DONE_MEM_INIT_DONE) != 1);
    CSL_FINS(rssCtrl->RSS_SHARED_MEM_MEMINIT_DONE, MSS_CTRL_MSS_ATCM_MEM_INIT_DONE_MSS_ATCM_MEM_INIT_DONE_MEM_INIT_DONE, 1);

    /* Check MEMINIT STATUS is zero to confirm no inprogress MEM INIT */
    while (CSL_FEXT(rssCtrl->RSS_SHARED_MEM_MEMINIT_STATUS, MSS_CTRL_MSS_ATCM_MEM_INIT_STATUS_MSS_ATCM_MEM_INIT_STATUS_MEM_STATUS) != 0);
    while (CSL_FEXT(rssCtrl->RSS_SHARED_MEM_MEMINIT_DONE, MSS_CTRL_MSS_ATCM_MEM_INIT_DONE_MSS_ATCM_MEM_INIT_DONE_MEM_INIT_DONE) != 0);
    /* Write 0 after mem init is completed */
    CSL_FINS(rssCtrl->RSS_SHARED_MEM_MEMINIT, MSS_CTRL_MSS_ATCM_MEM_INIT_MSS_ATCM_MEM_INIT_MEM_INIT, 0);
}

void SOC_rcmWaitMeminitL2(void)
{
    CSL_mss_ctrlRegs*        ptrMSSCtrlRegs;

    ptrMSSCtrlRegs = CSL_MSS_CTRL_getBaseAddress();

     /* wait for the L2 memory init to complete */
    while((ptrMSSCtrlRegs->MSS_L2_MEM_INIT_DONE & 0x3U) != 0x3U)
    {
        /* TBD - handle time out */
    }

    /* clear the status */
    ptrMSSCtrlRegs->MSS_L2_MEM_INIT_DONE = SOC_rcmInsert8 (ptrMSSCtrlRegs->MSS_L2_MEM_INIT_DONE, 1U, 0U, 0x3U);
}

void SOC_rcmDspPllConfig(SOC_RcmPllFoutFreqId outFreqId, SOC_RcmPllHsDivOutConfig *hsDivCfg)
{
    SOC_RcmXtalFreqId XTALFreq;
    uint16_t dspTrimVal;
    CSL_mss_toprcmRegs *ptrTopRCMRegs;
    SOC_RcmADPLLJConfig_t const * adplljCfg;
    uint32_t hsDivOutRegVal;
    uint32_t Fout;

    Fout = SOC_RCM_FREQ_MHZ2HZ(gSocRcmPLLFreqId2FOutMap[outFreqId]);
    /* read the Core ADPLL trim value */
    dspTrimVal = SOC_rcmGetDspTrimVal();

    /* read the XTAL Frequency */
    XTALFreq = SOC_rcmGetXTALFrequency();

   /* program PLL dividers and multipliers. The value are taken from  TPR12_ADPLLJ_Settings_1p0.xlsx */
    adplljCfg = SOC_rcmGetADPLLJConfig(gSocRcmXTALInfo[XTALFreq].Finp, outFreqId);
    DebugP_assert(adplljCfg != NULL);

    if(adplljCfg != NULL)
    {
        if(gSocRcmXTALInfo[XTALFreq].div2flag == FALSE)
        {
            SOC_rcmProgPllDspDivider (adplljCfg->N,
                               0U /* N2 divider for bypass */,
                               adplljCfg->M,
                               adplljCfg->M2,
                               adplljCfg->FracM,
                               adplljCfg->sdDiv);
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
                               adplljCfg->FracM,
                               adplljCfg->sdDiv);
        }
        /* Configure and Lock Core PLL */
        SOC_rcmConfigurePllDsp(dspTrimVal);

        /* Derive Clocks */
        // HSDIV-2 Settings
        /* Core PLL settings */
        ptrTopRCMRegs = CSL_TopRCM_getBaseAddress();

        if(hsDivCfg->hsdivOutEnMask & SOC_RCM_PLL_HSDIV_OUTPUT_ENABLE_0)
        {
            DebugP_assert((Fout % hsDivCfg->hsDivOutFreqHz[SOC_RCM_PLL_HSDIV_OUTPUT_IDX0]) == 0);
            hsDivOutRegVal = Fout / hsDivCfg->hsDivOutFreqHz[SOC_RCM_PLL_HSDIV_OUTPUT_IDX0];
            hsDivOutRegVal--;
            //HW_WR_REG32(CSL_MSS_TOPRCM_U_BASE+PLL_DSP_HSDIVIDER_CLKOUT0, 0x0);    // CLKOUT0_DIV[4:0] = 4  -- 900M/(4+1) = 225MHz
            ptrTopRCMRegs->PLL_DSP_HSDIVIDER_CLKOUT0 = SOC_rcmInsert8 (ptrTopRCMRegs->PLL_DSP_HSDIVIDER_CLKOUT0, 4U, 0U, hsDivOutRegVal);
        }
        if(hsDivCfg->hsdivOutEnMask & SOC_RCM_PLL_HSDIV_OUTPUT_ENABLE_1)
        {
            DebugP_assert((Fout % hsDivCfg->hsDivOutFreqHz[SOC_RCM_PLL_HSDIV_OUTPUT_IDX1]) == 0);
            hsDivOutRegVal = Fout / hsDivCfg->hsDivOutFreqHz[SOC_RCM_PLL_HSDIV_OUTPUT_IDX1];
            hsDivOutRegVal--;
            //HW_WR_REG32(CSL_MSS_TOPRCM_U_BASE+PLL_DSP_HSDIVIDER_CLKOUT1, 0x3);    // CLKOUT1_DIV[4:0] = 1  -- 900M/(1+1) = 450MHz
            ptrTopRCMRegs->PLL_DSP_HSDIVIDER_CLKOUT1 = SOC_rcmInsert8 (ptrTopRCMRegs->PLL_DSP_HSDIVIDER_CLKOUT1, 4U, 0U, hsDivOutRegVal);
        }
        if(hsDivCfg->hsdivOutEnMask & SOC_RCM_PLL_HSDIV_OUTPUT_ENABLE_2)
        {
            DebugP_assert((Fout % hsDivCfg->hsDivOutFreqHz[SOC_RCM_PLL_HSDIV_OUTPUT_IDX2]) == 0);
            hsDivOutRegVal = Fout / hsDivCfg->hsDivOutFreqHz[SOC_RCM_PLL_HSDIV_OUTPUT_IDX2];
            hsDivOutRegVal--;
            //HW_WR_REG32(CSL_MSS_TOPRCM_U_BASE+PLL_DSP_HSDIVIDER_CLKOUT2, 0x4);    // CLKOUT2_DIV[4:0] = 8  -- 900M/(8+1) = 100MHz
            ptrTopRCMRegs->PLL_DSP_HSDIVIDER_CLKOUT2 = SOC_rcmInsert8 (ptrTopRCMRegs->PLL_DSP_HSDIVIDER_CLKOUT2, 4U, 0U, hsDivOutRegVal);
        }
        if(hsDivCfg->hsdivOutEnMask & SOC_RCM_PLL_HSDIV_OUTPUT_ENABLE_3)
        {
            DebugP_assert((Fout % hsDivCfg->hsDivOutFreqHz[SOC_RCM_PLL_HSDIV_OUTPUT_IDX3]) == 0);
            hsDivOutRegVal = Fout / hsDivCfg->hsDivOutFreqHz[SOC_RCM_PLL_HSDIV_OUTPUT_IDX3];
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
        if(hsDivCfg->hsdivOutEnMask & SOC_RCM_PLL_HSDIV_OUTPUT_ENABLE_0)
        {
            //HW_WR_REG32(CSL_MSS_TOPRCM_U_BASE+PLL_DSP_HSDIVIDER_CLKOUT0, 0x100);  //+CLKOUT0_GATE[8]  = 1
            ptrTopRCMRegs->PLL_DSP_HSDIVIDER_CLKOUT0 = SOC_rcmInsert8 (ptrTopRCMRegs->PLL_DSP_HSDIVIDER_CLKOUT0, 8U, 8U, 0x1U);
        }
        if(hsDivCfg->hsdivOutEnMask & SOC_RCM_PLL_HSDIV_OUTPUT_ENABLE_1)
        {
            //HW_WR_REG32(CSL_MSS_TOPRCM_U_BASE+PLL_DSP_HSDIVIDER_CLKOUT1, 0x103);  //+CLKOUT1_GATE[8]  = 1
            ptrTopRCMRegs->PLL_DSP_HSDIVIDER_CLKOUT1 = SOC_rcmInsert8 (ptrTopRCMRegs->PLL_DSP_HSDIVIDER_CLKOUT1, 8U, 8U, 0x1U);
        }
        if(hsDivCfg->hsdivOutEnMask & SOC_RCM_PLL_HSDIV_OUTPUT_ENABLE_2)
        {
            //HW_WR_REG32(CSL_MSS_TOPRCM_U_BASE+PLL_DSP_HSDIVIDER_CLKOUT2, 0x104);  //+CLKOUT2_GATE[8]  = 1
            ptrTopRCMRegs->PLL_DSP_HSDIVIDER_CLKOUT2 = SOC_rcmInsert8 (ptrTopRCMRegs->PLL_DSP_HSDIVIDER_CLKOUT2, 8U, 8U, 0x1U);
        }
        if(hsDivCfg->hsdivOutEnMask & SOC_RCM_PLL_HSDIV_OUTPUT_ENABLE_3)
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
    SOC_RcmADPLLJConfig_t const * adplljCfg;
    uint32_t hsDivOutRegVal;
    uint32_t Fout;

    Fout = SOC_RCM_FREQ_MHZ2HZ(gSocRcmPLLFreqId2FOutMap[outFreqId]);
    /* read the Core ADPLL trim value */
    perTrimVal = SOC_rcmGetPerTrimVal();
    /* Core PLL settings */
    ptrTopRCMRegs = CSL_TopRCM_getBaseAddress();

    /* read the XTAL Frequency */
    XTALFreq = SOC_rcmGetXTALFrequency();

   /* program PLL dividers and multipliers. The value are taken from  TPR12_ADPLLJ_Settings_1p0.xlsx */
    adplljCfg = SOC_rcmGetADPLLJConfig(gSocRcmXTALInfo[XTALFreq].Finp, outFreqId);
    DebugP_assert(adplljCfg != NULL);

    if(adplljCfg != NULL)
    {
        if(gSocRcmXTALInfo[XTALFreq].div2flag == FALSE)
        {
            SOC_rcmProgPllPerDivider (adplljCfg->N, 0U, adplljCfg->M, adplljCfg->M2, adplljCfg->FracM, adplljCfg->sdDiv);
        }
        else
        {
            uint32_t N;

            DebugP_assert(((adplljCfg->N + 1) % 2) == 0);
            /* Input XTAL freq is half. Divide input divider by 2 to get same output freq */
            N = ((adplljCfg->N + 1) / 2) - 1;

            SOC_rcmProgPllPerDivider (N, 0U, adplljCfg->M, adplljCfg->M2, adplljCfg->FracM, adplljCfg->sdDiv);

        }
        /* Configure and Lock Core PLL */
        SOC_rcmConfigurePllPer (perTrimVal);

        /* Derive Clocks */
        /* TPR12_Ch08_Clock_Arch_0p91 is used as reference for below settings */
        /* Set clock divider values from PER PLL*/
        /* 1728 Mhz */
        // HSDIV-2 Settings
        if(hsDivCfg->hsdivOutEnMask & SOC_RCM_PLL_HSDIV_OUTPUT_ENABLE_0)
        {
            DebugP_assert((Fout % hsDivCfg->hsDivOutFreqHz[SOC_RCM_PLL_HSDIV_OUTPUT_IDX0]) == 0);
            hsDivOutRegVal = Fout / hsDivCfg->hsDivOutFreqHz[SOC_RCM_PLL_HSDIV_OUTPUT_IDX0];
            hsDivOutRegVal--;
            //HW_WR_REG32(CSL_MSS_TOPRCM_U_BASE+PLL_PER_HSDIVIDER_CLKOUT0, 0x0);    // CLKOUT0_DIV[4:0] = 0  -- 1.728G/(0+1) = 1.728GHz
            ptrTopRCMRegs->PLL_PER_HSDIVIDER_CLKOUT0 = SOC_rcmInsert8(ptrTopRCMRegs->PLL_PER_HSDIVIDER_CLKOUT0, 4U, 0U, hsDivOutRegVal);
        }
        if(hsDivCfg->hsdivOutEnMask & SOC_RCM_PLL_HSDIV_OUTPUT_ENABLE_1)
        {
            DebugP_assert((Fout % hsDivCfg->hsDivOutFreqHz[SOC_RCM_PLL_HSDIV_OUTPUT_IDX1]) == 0);
            hsDivOutRegVal = Fout / hsDivCfg->hsDivOutFreqHz[SOC_RCM_PLL_HSDIV_OUTPUT_IDX1];
            hsDivOutRegVal--;
            //HW_WR_REG32(CSL_MSS_TOPRCM_U_BASE+PLL_PER_HSDIVIDER_CLKOUT1, 0x8);    // CLKOUT1_DIV[4:0] = 8  -- 1.728G/(8+1) = 192 MHz
            ptrTopRCMRegs->PLL_PER_HSDIVIDER_CLKOUT1 = SOC_rcmInsert8(ptrTopRCMRegs->PLL_PER_HSDIVIDER_CLKOUT1, 4U, 0U, hsDivOutRegVal);
        }
        if(hsDivCfg->hsdivOutEnMask & SOC_RCM_PLL_HSDIV_OUTPUT_ENABLE_2)
        {
            DebugP_assert((Fout % hsDivCfg->hsDivOutFreqHz[SOC_RCM_PLL_HSDIV_OUTPUT_IDX2]) == 0);
            hsDivOutRegVal = Fout / hsDivCfg->hsDivOutFreqHz[SOC_RCM_PLL_HSDIV_OUTPUT_IDX2];
            hsDivOutRegVal--;
            //HW_WR_REG32(CSL_MSS_TOPRCM_U_BASE+PLL_PER_HSDIVIDER_CLKOUT2, 0x11);    // CLKOUT2_DIV[4:0] = 11  -- 1.8G/(17+1) = 96 MHz
            ptrTopRCMRegs->PLL_PER_HSDIVIDER_CLKOUT2 = SOC_rcmInsert8(ptrTopRCMRegs->PLL_PER_HSDIVIDER_CLKOUT2, 4U, 0U, hsDivOutRegVal);
        }
        if(hsDivCfg->hsdivOutEnMask & SOC_RCM_PLL_HSDIV_OUTPUT_ENABLE_3)
        {
            DebugP_assert((Fout % hsDivCfg->hsDivOutFreqHz[SOC_RCM_PLL_HSDIV_OUTPUT_IDX3]) == 0);
            hsDivOutRegVal = Fout / hsDivCfg->hsDivOutFreqHz[SOC_RCM_PLL_HSDIV_OUTPUT_IDX3];
            hsDivOutRegVal--;
            //Unsed HW_WR_REG32(CSL_MSS_TOPRCM_U_BASE+PLL_DSP_HSDIVIDER_CLKOUT3, 0x9);    //  unused
            ptrTopRCMRegs->PLL_PER_HSDIVIDER_CLKOUT3 = SOC_rcmInsert8(ptrTopRCMRegs->PLL_PER_HSDIVIDER_CLKOUT3, 4U, 0U, hsDivOutRegVal);
        }
        /* Generate Trigger to latch these values */
        //HW_WR_REG32(CSL_MSS_TOPRCM_U_BASE+PLL_PER_HSDIVIDER        , 0x4);    // HSDIVIDER[2]     = 1
        ptrTopRCMRegs->PLL_PER_HSDIVIDER         = SOC_rcmInsert8(ptrTopRCMRegs->PLL_PER_HSDIVIDER, 2U, 2U, 0x1U);
        //HW_WR_REG32(CSL_MSS_TOPRCM_U_BASE+PLL_PER_HSDIVIDER        , 0x0);    // HSDIVIDER[2]     = 0
        ptrTopRCMRegs->PLL_PER_HSDIVIDER         = SOC_rcmInsert8(ptrTopRCMRegs->PLL_PER_HSDIVIDER, 2U, 2U, 0x0U);

        /* Ungate the clocks */
        if(hsDivCfg->hsdivOutEnMask & SOC_RCM_PLL_HSDIV_OUTPUT_ENABLE_0)
        {
            //HW_WR_REG32(CSL_MSS_TOPRCM_U_BASE+PLL_PER_HSDIVIDER_CLKOUT0, 0x100);  //+CLKOUT0_GATE[8]  = 1
            ptrTopRCMRegs->PLL_PER_HSDIVIDER_CLKOUT0 = SOC_rcmInsert8(ptrTopRCMRegs->PLL_PER_HSDIVIDER_CLKOUT0, 8U, 8U, 0x1U);
        }
        if(hsDivCfg->hsdivOutEnMask & SOC_RCM_PLL_HSDIV_OUTPUT_ENABLE_1)
        {
            //HW_WR_REG32(CSL_MSS_TOPRCM_U_BASE+PLL_PER_HSDIVIDER_CLKOUT1, 0x108);  //+CLKOUT1_GATE[8]  = 1
            ptrTopRCMRegs->PLL_PER_HSDIVIDER_CLKOUT1 = SOC_rcmInsert8(ptrTopRCMRegs->PLL_PER_HSDIVIDER_CLKOUT1, 8U, 8U, 0x1U);
        }
        if(hsDivCfg->hsdivOutEnMask & SOC_RCM_PLL_HSDIV_OUTPUT_ENABLE_2)
        {
            //HW_WR_REG32(CSL_MSS_TOPRCM_U_BASE+PLL_PER_HSDIVIDER_CLKOUT2, 0x111);  //+CLKOUT2_GATE[8]  = 1
            ptrTopRCMRegs->PLL_PER_HSDIVIDER_CLKOUT2 = SOC_rcmInsert8(ptrTopRCMRegs->PLL_PER_HSDIVIDER_CLKOUT2, 8U, 8U, 0x1U);
        }
        if(hsDivCfg->hsdivOutEnMask & SOC_RCM_PLL_HSDIV_OUTPUT_ENABLE_3)
        {
            //UnusedHW_WR_REG32(CSL_MSS_TOPRCM_U_BASE+PLL_DSP_HSDIVIDER_CLKOUT3, 0x109);  //+CLKOUT3_GATE[8]  = 1
            ptrTopRCMRegs->PLL_PER_HSDIVIDER_CLKOUT3 = SOC_rcmInsert8(ptrTopRCMRegs->PLL_PER_HSDIVIDER_CLKOUT3, 8U, 8U, 0x1U);
        }
    }

    return;
}

int32_t SOC_rcmSetDspClock(SOC_RcmDspClockSource clkSource, uint32_t freqHz)
{
    volatile uint32_t   *ptrClkSrcReg, *ptrClkDivReg;
    uint16_t            clkSrcVal;
    int32_t             retVal;
    uint32_t            pllFout;
    uint32_t            clkDivisor;

    SOC_controlModuleUnlockMMR(SOC_DOMAIN_ID_DSS_RCM, 0);
    pllFout = SOC_rcmGetDspClockFrequency(clkSource);
    SOC_rcmGetDSPClkSrcAndDivReg(clkSource, &clkSrcVal, &ptrClkSrcReg, &ptrClkDivReg);
    clkDivisor = SOC_rcmGetModuleClkDivVal(pllFout, freqHz);

    if((ptrClkSrcReg != NULL) && (ptrClkDivReg != NULL) && (clkSrcVal != 0x888U))
    {
        uint16_t            clkDivVal;

        /* Create the Divider Value to be programmed */
        clkDivVal = ((uint16_t)clkDivisor & 0xFU);
        clkDivVal = (clkDivVal | (clkDivVal << 4U) | (clkDivVal << 8U));

        /* Write the Divider Value */
        *ptrClkDivReg = SOC_rcmInsert16(*ptrClkDivReg, 11U, 0U, clkDivVal);

        /* Write the Clock Source Selection Value */
        *ptrClkSrcReg = SOC_rcmInsert16(*ptrClkSrcReg, 11U, 0U, clkSrcVal);
        retVal = SystemP_SUCCESS;
    }
    else
    {
        /* Error */
        retVal = SystemP_FAILURE;
    }

    return (retVal);
}

int32_t SOC_rcmSetHSDivMux(SOC_RcmHSDIVClkOutMuxId clkOutMuxId,
                           SOC_RcmHSDIVClkOutMuxClockSource muxSource)
{
    volatile uint32_t   *ptrClkSrcReg;
    uint16_t            clkSrcVal;
    int32_t          retVal;

    SOC_rcmGetHSDIVClkOutMuxClkSrcAndDivReg (clkOutMuxId, muxSource, &clkSrcVal, &ptrClkSrcReg);
    if((ptrClkSrcReg != NULL) && (clkSrcVal != 0x888U))
    {
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

int32_t SOC_rcmSetRssClkFreq(SOC_RcmRssClkSrcId rssClkSrcId, uint32_t freqHz)
{
    volatile uint32_t   *ptrClkSrcReg;
    volatile uint32_t   *ptrClkDivReg;
    uint16_t            clkSrcVal, clkDivisor;
    int32_t          retVal;
    uint32_t            fInp;
    SOC_RcmClockSrcId       rootClkId;

    SOC_rcmGetRssClkSrcAndDivRegInfo (rssClkSrcId, &clkSrcVal, &ptrClkSrcReg, &ptrClkDivReg);
    rootClkId = gSocRcmRssClkSrcInfoMap[rssClkSrcId];
    fInp = SOC_rcmGetFreq(rootClkId);
    clkDivisor = SOC_rcmGetModuleClkDivVal(fInp, freqHz);

    if((ptrClkSrcReg != NULL) && (clkSrcVal != 0x888U))
    {
        uint16_t            clkDivVal;

        /* Create the Divider Value to be programmed */
        clkDivVal = ((uint16_t)clkDivisor & 0xFU);
        clkDivVal = (clkDivVal | (clkDivVal << 4U) | (clkDivVal << 8U));

        /* Some registers like HWA do not have ClkDiv reg */
        if(ptrClkDivReg != NULL)
        {
            /* Write the Divider Value */
            *ptrClkDivReg = SOC_rcmInsert16 (*ptrClkDivReg, 11U, 0U, clkDivVal);
        }
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

static void SOC_rcmGetRssClkSrc(SOC_RcmClockSrcId *clkSrcId)
{
    SOC_RcmRssClkSrcId  rssClkSrcId;

    SOC_rcmGetRssClkSrcLocal(&rssClkSrcId);
    /* Write the Clock Source Selection Value */
    *clkSrcId = gSocRcmRssClkSrcInfoMap[rssClkSrcId];

    return;
}

static void SOC_rcmGetHSDivClkSrc(SOC_RcmHSDIVClkOutMuxId clkOutMuxId, SOC_RcmClockSrcId * clkSrcId)
{
    SOC_RcmHSDIVClkOutMuxClockSource muxSource;

    SOC_rcmGetHSDIVClkOutMuxClkSrc(clkOutMuxId, &muxSource);
    /* Write the Clock Source Selection Value */
    *clkSrcId = gSocRcmHsDivMuxClkSrcInfoMap[muxSource];

    return;
}

static uint32_t SOC_rcmGetDspClockFrequency(SOC_RcmDspClockSource clkSource)
{
    uint32_t clkFreq = 0U;
    SOC_RcmClockSrcId clkSrcId;

    clkSrcId = gSocRcmDspClkSrcInfoMap[clkSource];
    clkFreq = SOC_rcmGetFreq(clkSrcId);
    return (clkFreq);
}

uint32_t SOC_rcmGetDspClock(void)
{
    int32_t             retVal;
    uint32_t            pllFout;
    uint32_t            clkDivisorRegVal;
    uint32_t            clkDiv;
    SOC_RcmDspClockSource  clkSource;
    uint32_t            freqHz = 1U;

    SOC_controlModuleUnlockMMR(SOC_DOMAIN_ID_DSS_RCM, 0);

    retVal = SOC_rcmGetDSPClkSrcAndDivValue(&clkSource, &clkDivisorRegVal);
    if(SystemP_SUCCESS == retVal)
    {
        pllFout = SOC_rcmGetDspClockFrequency(clkSource);
        clkDiv = SOC_rcmGetModuleClkDivFromRegVal(clkDivisorRegVal);
        freqHz = (pllFout / clkDiv);
    }
    return (freqHz);
}

static SOC_RcmADPLLJConfig_t const *SOC_rcmGetADPLLJConfig(uint32_t Finp, SOC_RcmPllFoutFreqId foutFreqId)
{
    uint32_t i;
    SOC_RcmADPLLJConfig_t const *adplljCfg;
    uint32_t Fout;

    Fout = gSocRcmPLLFreqId2FOutMap[foutFreqId];
    for (i = 0; i < SOC_RCM_UTILS_ARRAYSIZE(gSocRcmADPLLJConfigTbl); i++)
    {
        if((gSocRcmADPLLJConfigTbl[i].Finp == Finp) && (gSocRcmADPLLJConfigTbl[i].Fout == Fout))
        {
            break;
        }
    }
    if(i < SOC_RCM_UTILS_ARRAYSIZE(gSocRcmADPLLJConfigTbl))
    {
        adplljCfg = &gSocRcmADPLLJConfigTbl[i];
    }
    else
    {
        adplljCfg = (SOC_RcmADPLLJConfig_t const *)NULL;
    }
    return adplljCfg;
}

void SOC_rcmWaitMemInitDSSL2(uint32_t l2bankMask)
{
    CSL_dss_ctrlRegs *dssCtrl = CSL_DSS_CTRL_getBaseAddress();
    uint32_t          clearMemInitMask = 0;

    SOC_controlModuleUnlockMMR(SOC_DOMAIN_ID_DSS_CTRL, 0);

    if(l2bankMask & SOC_RCM_MEMINIT_DSSL2_MEMBANK_VB00)
    {
        while (CSL_FEXT(dssCtrl->DSS_DSP_L2RAM_MEMINIT_DONE, DSS_CTRL_DSS_DSP_L2RAM_MEMINIT_DONE_DSS_DSP_L2RAM_MEMINIT_DONE_VB00) != 1);
        clearMemInitMask |= CSL_DSS_CTRL_DSS_DSP_L2RAM_MEMINIT_DONE_DSS_DSP_L2RAM_MEMINIT_DONE_VB00_MASK;
    }

    if(l2bankMask & SOC_RCM_MEMINIT_DSSL2_MEMBANK_VB01)
    {
        while (CSL_FEXT(dssCtrl->DSS_DSP_L2RAM_MEMINIT_DONE, DSS_CTRL_DSS_DSP_L2RAM_MEMINIT_DONE_DSS_DSP_L2RAM_MEMINIT_DONE_VB01) != 1);
        clearMemInitMask |= CSL_DSS_CTRL_DSS_DSP_L2RAM_MEMINIT_DONE_DSS_DSP_L2RAM_MEMINIT_DONE_VB01_MASK;
    }

    if(l2bankMask & SOC_RCM_MEMINIT_DSSL2_MEMBANK_VB10)
    {
        while (CSL_FEXT(dssCtrl->DSS_DSP_L2RAM_MEMINIT_DONE, DSS_CTRL_DSS_DSP_L2RAM_MEMINIT_DONE_DSS_DSP_L2RAM_MEMINIT_DONE_VB10) != 1);
        clearMemInitMask |= CSL_DSS_CTRL_DSS_DSP_L2RAM_MEMINIT_DONE_DSS_DSP_L2RAM_MEMINIT_DONE_VB10_MASK;
    }

    if(l2bankMask & SOC_RCM_MEMINIT_DSSL2_MEMBANK_VB11)
    {
        while (CSL_FEXT(dssCtrl->DSS_DSP_L2RAM_MEMINIT_DONE, DSS_CTRL_DSS_DSP_L2RAM_MEMINIT_DONE_DSS_DSP_L2RAM_MEMINIT_DONE_VB11) != 1);
        clearMemInitMask |= CSL_DSS_CTRL_DSS_DSP_L2RAM_MEMINIT_DONE_DSS_DSP_L2RAM_MEMINIT_DONE_VB11_MASK;
    }

    if(l2bankMask & SOC_RCM_MEMINIT_DSSL2_MEMBANK_VB20)
    {
        while (CSL_FEXT(dssCtrl->DSS_DSP_L2RAM_MEMINIT_DONE, DSS_CTRL_DSS_DSP_L2RAM_MEMINIT_DONE_DSS_DSP_L2RAM_MEMINIT_DONE_VB20) != 1);
        clearMemInitMask |= CSL_DSS_CTRL_DSS_DSP_L2RAM_MEMINIT_DONE_DSS_DSP_L2RAM_MEMINIT_DONE_VB20_MASK;
    }

    if(l2bankMask & SOC_RCM_MEMINIT_DSSL2_MEMBANK_VB21)
    {
        while (CSL_FEXT(dssCtrl->DSS_DSP_L2RAM_MEMINIT_DONE, DSS_CTRL_DSS_DSP_L2RAM_MEMINIT_DONE_DSS_DSP_L2RAM_MEMINIT_DONE_VB21) != 1);
        clearMemInitMask |= CSL_DSS_CTRL_DSS_DSP_L2RAM_MEMINIT_DONE_DSS_DSP_L2RAM_MEMINIT_DONE_VB21_MASK;
    }

    if(l2bankMask & SOC_RCM_MEMINIT_DSSL2_MEMBANK_VB30)
    {
        while (CSL_FEXT(dssCtrl->DSS_DSP_L2RAM_MEMINIT_DONE, DSS_CTRL_DSS_DSP_L2RAM_MEMINIT_DONE_DSS_DSP_L2RAM_MEMINIT_DONE_VB30) != 1);
        clearMemInitMask |= CSL_DSS_CTRL_DSS_DSP_L2RAM_MEMINIT_DONE_DSS_DSP_L2RAM_MEMINIT_DONE_VB30_MASK;
    }

    if(l2bankMask & SOC_RCM_MEMINIT_DSSL2_MEMBANK_VB31)
    {
        while (CSL_FEXT(dssCtrl->DSS_DSP_L2RAM_MEMINIT_DONE, DSS_CTRL_DSS_DSP_L2RAM_MEMINIT_DONE_DSS_DSP_L2RAM_MEMINIT_DONE_VB31) != 1);
        clearMemInitMask |= CSL_DSS_CTRL_DSS_DSP_L2RAM_MEMINIT_DONE_DSS_DSP_L2RAM_MEMINIT_DONE_VB31_MASK;
    }

    dssCtrl->DSS_DSP_L2RAM_MEMINIT_DONE = SOC_rcmInsert8 (dssCtrl->DSS_DSP_L2RAM_MEMINIT_DONE, 7U, 0U, clearMemInitMask);

    if(l2bankMask & SOC_RCM_MEMINIT_DSSL2_MEMBANK_VB00)
    {
        /* Check MEMINIT STATUS is zero to confirm no inprogress MEM INIT */
        while (CSL_FEXT(dssCtrl->DSS_DSP_L2RAM_MEMINIT_STATUS, DSS_CTRL_DSS_DSP_L2RAM_MEMINIT_STATUS_DSS_DSP_L2RAM_MEMINIT_STATUS_VB00) != 0);
        while (CSL_FEXT(dssCtrl->DSS_DSP_L2RAM_MEMINIT_DONE, DSS_CTRL_DSS_DSP_L2RAM_MEMINIT_DONE_DSS_DSP_L2RAM_MEMINIT_DONE_VB00) != 0);
    }

    if(l2bankMask & SOC_RCM_MEMINIT_DSSL2_MEMBANK_VB01)
    {
        /* Check MEMINIT STATUS is zero to confirm no inprogress MEM INIT */
        while (CSL_FEXT(dssCtrl->DSS_DSP_L2RAM_MEMINIT_STATUS, DSS_CTRL_DSS_DSP_L2RAM_MEMINIT_STATUS_DSS_DSP_L2RAM_MEMINIT_STATUS_VB01) != 0);
        while (CSL_FEXT(dssCtrl->DSS_DSP_L2RAM_MEMINIT_DONE, DSS_CTRL_DSS_DSP_L2RAM_MEMINIT_DONE_DSS_DSP_L2RAM_MEMINIT_DONE_VB01) != 0);
    }

    if(l2bankMask & SOC_RCM_MEMINIT_DSSL2_MEMBANK_VB10)
    {
        /* Check MEMINIT STATUS is zero to confirm no inprogress MEM INIT */
        while (CSL_FEXT(dssCtrl->DSS_DSP_L2RAM_MEMINIT_STATUS, DSS_CTRL_DSS_DSP_L2RAM_MEMINIT_STATUS_DSS_DSP_L2RAM_MEMINIT_STATUS_VB10) != 0);
        while (CSL_FEXT(dssCtrl->DSS_DSP_L2RAM_MEMINIT_DONE, DSS_CTRL_DSS_DSP_L2RAM_MEMINIT_DONE_DSS_DSP_L2RAM_MEMINIT_DONE_VB10) != 0);
    }

    if(l2bankMask & SOC_RCM_MEMINIT_DSSL2_MEMBANK_VB11)
    {
        /* Check MEMINIT STATUS is zero to confirm no inprogress MEM INIT */
        while (CSL_FEXT(dssCtrl->DSS_DSP_L2RAM_MEMINIT_STATUS, DSS_CTRL_DSS_DSP_L2RAM_MEMINIT_STATUS_DSS_DSP_L2RAM_MEMINIT_STATUS_VB11) != 0);
        while (CSL_FEXT(dssCtrl->DSS_DSP_L2RAM_MEMINIT_DONE, DSS_CTRL_DSS_DSP_L2RAM_MEMINIT_DONE_DSS_DSP_L2RAM_MEMINIT_DONE_VB11) != 0);
    }

    if(l2bankMask & SOC_RCM_MEMINIT_DSSL2_MEMBANK_VB20)
    {
        /* Check MEMINIT STATUS is zero to confirm no inprogress MEM INIT */
        while (CSL_FEXT(dssCtrl->DSS_DSP_L2RAM_MEMINIT_STATUS, DSS_CTRL_DSS_DSP_L2RAM_MEMINIT_STATUS_DSS_DSP_L2RAM_MEMINIT_STATUS_VB20) != 0);
        while (CSL_FEXT(dssCtrl->DSS_DSP_L2RAM_MEMINIT_DONE, DSS_CTRL_DSS_DSP_L2RAM_MEMINIT_DONE_DSS_DSP_L2RAM_MEMINIT_DONE_VB20) != 0);
    }

    if(l2bankMask & SOC_RCM_MEMINIT_DSSL2_MEMBANK_VB21)
    {
        /* Check MEMINIT STATUS is zero to confirm no inprogress MEM INIT */
        while (CSL_FEXT(dssCtrl->DSS_DSP_L2RAM_MEMINIT_STATUS, DSS_CTRL_DSS_DSP_L2RAM_MEMINIT_STATUS_DSS_DSP_L2RAM_MEMINIT_STATUS_VB21) != 0);
        while (CSL_FEXT(dssCtrl->DSS_DSP_L2RAM_MEMINIT_DONE, DSS_CTRL_DSS_DSP_L2RAM_MEMINIT_DONE_DSS_DSP_L2RAM_MEMINIT_DONE_VB21) != 0);
    }

    if(l2bankMask & SOC_RCM_MEMINIT_DSSL2_MEMBANK_VB30)
    {
        /* Check MEMINIT STATUS is zero to confirm no inprogress MEM INIT */
        while (CSL_FEXT(dssCtrl->DSS_DSP_L2RAM_MEMINIT_STATUS, DSS_CTRL_DSS_DSP_L2RAM_MEMINIT_STATUS_DSS_DSP_L2RAM_MEMINIT_STATUS_VB30) != 0);
        while (CSL_FEXT(dssCtrl->DSS_DSP_L2RAM_MEMINIT_DONE, DSS_CTRL_DSS_DSP_L2RAM_MEMINIT_DONE_DSS_DSP_L2RAM_MEMINIT_DONE_VB30) != 0);
    }

    if(l2bankMask & SOC_RCM_MEMINIT_DSSL2_MEMBANK_VB31)
    {
        /* Check MEMINIT STATUS is zero to confirm no inprogress MEM INIT */
        while (CSL_FEXT(dssCtrl->DSS_DSP_L2RAM_MEMINIT_STATUS, DSS_CTRL_DSS_DSP_L2RAM_MEMINIT_STATUS_DSS_DSP_L2RAM_MEMINIT_STATUS_VB31) != 0);
        while (CSL_FEXT(dssCtrl->DSS_DSP_L2RAM_MEMINIT_DONE, DSS_CTRL_DSS_DSP_L2RAM_MEMINIT_DONE_DSS_DSP_L2RAM_MEMINIT_DONE_VB31) != 0);
    }
}

void SOC_rcmStartMemInitDSSL2(uint32_t l2bankMask)
{
    CSL_dss_ctrlRegs *dssCtrl = CSL_DSS_CTRL_getBaseAddress();
    uint32_t memBankInit = 0;

    SOC_controlModuleUnlockMMR(SOC_DOMAIN_ID_DSS_CTRL, 0);

    if(l2bankMask & SOC_RCM_MEMINIT_DSSL2_MEMBANK_VB00)
    {
        memBankInit |= CSL_DSS_CTRL_DSS_DSP_L2RAM_MEMINIT_START_DSS_DSP_L2RAM_MEMINIT_START_VB00_MASK;
    }
    if(l2bankMask & SOC_RCM_MEMINIT_DSSL2_MEMBANK_VB01)
    {
        memBankInit |= CSL_DSS_CTRL_DSS_DSP_L2RAM_MEMINIT_START_DSS_DSP_L2RAM_MEMINIT_START_VB01_MASK;
    }
    if(l2bankMask & SOC_RCM_MEMINIT_DSSL2_MEMBANK_VB10)
    {
        memBankInit |= CSL_DSS_CTRL_DSS_DSP_L2RAM_MEMINIT_START_DSS_DSP_L2RAM_MEMINIT_START_VB10_MASK;
    }
    if(l2bankMask & SOC_RCM_MEMINIT_DSSL2_MEMBANK_VB11)
    {
        memBankInit |= CSL_DSS_CTRL_DSS_DSP_L2RAM_MEMINIT_START_DSS_DSP_L2RAM_MEMINIT_START_VB11_MASK;
    }
    if(l2bankMask & SOC_RCM_MEMINIT_DSSL2_MEMBANK_VB20)
    {
        memBankInit |= CSL_DSS_CTRL_DSS_DSP_L2RAM_MEMINIT_START_DSS_DSP_L2RAM_MEMINIT_START_VB20_MASK;
    }
    if(l2bankMask & SOC_RCM_MEMINIT_DSSL2_MEMBANK_VB21)
    {
        memBankInit |= CSL_DSS_CTRL_DSS_DSP_L2RAM_MEMINIT_START_DSS_DSP_L2RAM_MEMINIT_START_VB21_MASK;
    }
    if(l2bankMask & SOC_RCM_MEMINIT_DSSL2_MEMBANK_VB30)
    {
        memBankInit |= CSL_DSS_CTRL_DSS_DSP_L2RAM_MEMINIT_START_DSS_DSP_L2RAM_MEMINIT_START_VB30_MASK;
    }
    if(l2bankMask & SOC_RCM_MEMINIT_DSSL2_MEMBANK_VB31)
    {
        memBankInit |= CSL_DSS_CTRL_DSS_DSP_L2RAM_MEMINIT_START_DSS_DSP_L2RAM_MEMINIT_START_VB31_MASK;
    }

    /* Start the Initialization of L2 Memory */
    dssCtrl->DSS_DSP_L2RAM_MEMINIT_START = SOC_rcmInsert8 (dssCtrl->DSS_DSP_L2RAM_MEMINIT_START, 7U, 0U, memBankInit);
}

void SOC_rcmWaitMemInitDSSL3(uint32_t l3bankMask)
{
    CSL_dss_ctrlRegs *dssCtrl = CSL_DSS_CTRL_getBaseAddress();
    uint32_t          clearMemInitMask = 0;

    SOC_controlModuleUnlockMMR(SOC_DOMAIN_ID_DSS_CTRL, 0);

    if(l3bankMask & SOC_RCM_MEMINIT_DSSL3_MEMBANK_RAM0)
    {
        while (CSL_FEXT(dssCtrl->DSS_L3RAM_MEMINIT_DONE, DSS_CTRL_DSS_L3RAM_MEMINIT_DONE_DSS_L3RAM_MEMINIT_DONE_L3RAM0_MEMINIT_DONE) != 1);
        clearMemInitMask |= CSL_DSS_CTRL_DSS_L3RAM_MEMINIT_DONE_DSS_L3RAM_MEMINIT_DONE_L3RAM0_MEMINIT_DONE_MASK;
    }

    if(l3bankMask & SOC_RCM_MEMINIT_DSSL3_MEMBANK_RAM1)
    {
        while (CSL_FEXT(dssCtrl->DSS_L3RAM_MEMINIT_DONE, DSS_CTRL_DSS_L3RAM_MEMINIT_DONE_DSS_L3RAM_MEMINIT_DONE_L3RAM1_MEMINIT_DONE) != 1);
        clearMemInitMask |= CSL_DSS_CTRL_DSS_L3RAM_MEMINIT_DONE_DSS_L3RAM_MEMINIT_DONE_L3RAM1_MEMINIT_DONE_MASK;
    }

    if(l3bankMask & SOC_RCM_MEMINIT_DSSL3_MEMBANK_RAM2)
    {
        while (CSL_FEXT(dssCtrl->DSS_L3RAM_MEMINIT_DONE, DSS_CTRL_DSS_L3RAM_MEMINIT_DONE_DSS_L3RAM_MEMINIT_DONE_L3RAM2_MEMINIT_DONE) != 1);
        clearMemInitMask |= CSL_DSS_CTRL_DSS_L3RAM_MEMINIT_DONE_DSS_L3RAM_MEMINIT_DONE_L3RAM2_MEMINIT_DONE_MASK;
    }

    if(l3bankMask & SOC_RCM_MEMINIT_DSSL3_MEMBANK_RAM3)
    {
        while (CSL_FEXT(dssCtrl->DSS_L3RAM_MEMINIT_DONE, DSS_CTRL_DSS_L3RAM_MEMINIT_DONE_DSS_L3RAM_MEMINIT_DONE_L3RAM3_MEMINIT_DONE) != 1);
        clearMemInitMask |= CSL_DSS_CTRL_DSS_L3RAM_MEMINIT_DONE_DSS_L3RAM_MEMINIT_DONE_L3RAM3_MEMINIT_DONE_MASK;
    }
    dssCtrl->DSS_L3RAM_MEMINIT_DONE = SOC_rcmInsert8 (dssCtrl->DSS_L3RAM_MEMINIT_DONE, 3U, 0U, clearMemInitMask);

    if(l3bankMask & SOC_RCM_MEMINIT_DSSL3_MEMBANK_RAM0)
    {
        /* Check MEMINIT STATUS is zero to confirm no inprogress MEM INIT */
        while (CSL_FEXT(dssCtrl->DSS_L3RAM_MEMINIT_STATUS, DSS_CTRL_DSS_L3RAM_MEMINIT_STATUS_DSS_L3RAM_MEMINIT_STATUS_L3RAM0_MEMINIT_STATUS) != 0);
        while (CSL_FEXT(dssCtrl->DSS_L3RAM_MEMINIT_DONE, DSS_CTRL_DSS_L3RAM_MEMINIT_DONE_DSS_L3RAM_MEMINIT_DONE_L3RAM0_MEMINIT_DONE) != 0);
    }

    if(l3bankMask & SOC_RCM_MEMINIT_DSSL3_MEMBANK_RAM1)
    {
        /* Check MEMINIT STATUS is zero to confirm no inprogress MEM INIT */
        while (CSL_FEXT(dssCtrl->DSS_L3RAM_MEMINIT_STATUS, DSS_CTRL_DSS_L3RAM_MEMINIT_STATUS_DSS_L3RAM_MEMINIT_STATUS_L3RAM1_MEMINIT_STATUS) != 0);
        while (CSL_FEXT(dssCtrl->DSS_L3RAM_MEMINIT_DONE, DSS_CTRL_DSS_L3RAM_MEMINIT_DONE_DSS_L3RAM_MEMINIT_DONE_L3RAM1_MEMINIT_DONE) != 0);
    }

    if(l3bankMask & SOC_RCM_MEMINIT_DSSL3_MEMBANK_RAM2)
    {
        /* Check MEMINIT STATUS is zero to confirm no inprogress MEM INIT */
        while (CSL_FEXT(dssCtrl->DSS_L3RAM_MEMINIT_STATUS, DSS_CTRL_DSS_L3RAM_MEMINIT_STATUS_DSS_L3RAM_MEMINIT_STATUS_L3RAM2_MEMINIT_STATUS) != 0);
        while (CSL_FEXT(dssCtrl->DSS_L3RAM_MEMINIT_DONE, DSS_CTRL_DSS_L3RAM_MEMINIT_DONE_DSS_L3RAM_MEMINIT_DONE_L3RAM2_MEMINIT_DONE) != 0);
    }

    if(l3bankMask & SOC_RCM_MEMINIT_DSSL3_MEMBANK_RAM3)
    {
        /* Check MEMINIT STATUS is zero to confirm no inprogress MEM INIT */
        while (CSL_FEXT(dssCtrl->DSS_L3RAM_MEMINIT_STATUS, DSS_CTRL_DSS_L3RAM_MEMINIT_STATUS_DSS_L3RAM_MEMINIT_STATUS_L3RAM3_MEMINIT_STATUS) != 0);
        while (CSL_FEXT(dssCtrl->DSS_L3RAM_MEMINIT_DONE, DSS_CTRL_DSS_L3RAM_MEMINIT_DONE_DSS_L3RAM_MEMINIT_DONE_L3RAM3_MEMINIT_DONE) != 0);
    }
}

void SOC_rcmStartMemInitDSSL3(uint32_t l3bankMask)
{
    CSL_dss_ctrlRegs *dssCtrl = CSL_DSS_CTRL_getBaseAddress();
    uint32_t memBankInit = 0;

    SOC_controlModuleUnlockMMR(SOC_DOMAIN_ID_DSS_CTRL, 0);

    if(l3bankMask & SOC_RCM_MEMINIT_DSSL3_MEMBANK_RAM0)
    {
        memBankInit |= CSL_DSS_CTRL_DSS_L3RAM_MEMINIT_START_DSS_L3RAM_MEMINIT_START_L3RAM0_MEMINIT_START_MASK;
    }

    if(l3bankMask & SOC_RCM_MEMINIT_DSSL3_MEMBANK_RAM1)
    {
        memBankInit |= CSL_DSS_CTRL_DSS_L3RAM_MEMINIT_START_DSS_L3RAM_MEMINIT_START_L3RAM1_MEMINIT_START_MASK;
    }

    if(l3bankMask & SOC_RCM_MEMINIT_DSSL3_MEMBANK_RAM2)
    {
        memBankInit |= CSL_DSS_CTRL_DSS_L3RAM_MEMINIT_START_DSS_L3RAM_MEMINIT_START_L3RAM2_MEMINIT_START_MASK;
    }

    if(l3bankMask & SOC_RCM_MEMINIT_DSSL3_MEMBANK_RAM3)
    {
        memBankInit |= CSL_DSS_CTRL_DSS_L3RAM_MEMINIT_START_DSS_L3RAM_MEMINIT_START_L3RAM3_MEMINIT_START_MASK;
    }

    /* Start the Initialization of L2 Memory */
    dssCtrl->DSS_L3RAM_MEMINIT_START = SOC_rcmInsert8 (dssCtrl->DSS_L3RAM_MEMINIT_START, 3U, 0U, memBankInit);
}

void SOC_rcmStartMemInitTCMB(void)
{
    CSL_mss_ctrlRegs *mssCtrl = CSL_MSS_CTRL_getBaseAddress();

    /* Check MEMINIT STATUS is zero to confirm no inprogress MEM INIT */
    while (CSL_FEXT(mssCtrl->MSS_BTCM_MEM_INIT_STATUS, MSS_CTRL_MSS_BTCM_MEM_INIT_STATUS_MSS_BTCM_MEM_INIT_STATUS_MEM_STATUS) != 0);

    /* Clear MEMINIT DONE before initiating MEMINIT */
    CSL_FINS(mssCtrl->MSS_BTCM_MEM_INIT_DONE, MSS_CTRL_MSS_BTCM_MEM_INIT_DONE_MSS_BTCM_MEM_INIT_DONE_MEM_INIT_DONE, 1);
    while (CSL_FEXT(mssCtrl->MSS_BTCM_MEM_INIT_DONE, MSS_CTRL_MSS_BTCM_MEM_INIT_DONE_MSS_BTCM_MEM_INIT_DONE_MEM_INIT_DONE) != 0);

    CSL_FINS(mssCtrl->MSS_BTCM_MEM_INIT, MSS_CTRL_MSS_BTCM_MEM_INIT_MSS_BTCM_MEM_INIT_MEM_INIT, 1);
}

void SOC_rcmWaitMemInitTCMB(void)
{
    CSL_mss_ctrlRegs *mssCtrl = CSL_MSS_CTRL_getBaseAddress();

    while (CSL_FEXT(mssCtrl->MSS_BTCM_MEM_INIT_DONE, MSS_CTRL_MSS_BTCM_MEM_INIT_DONE_MSS_BTCM_MEM_INIT_DONE_MEM_INIT_DONE) != 1);
    CSL_FINS(mssCtrl->MSS_BTCM_MEM_INIT_DONE, MSS_CTRL_MSS_BTCM_MEM_INIT_DONE_MSS_BTCM_MEM_INIT_DONE_MEM_INIT_DONE, 1);

    /* Check MEMINIT STATUS is zero to confirm no inprogress MEM INIT */
    while (CSL_FEXT(mssCtrl->MSS_BTCM_MEM_INIT_STATUS, MSS_CTRL_MSS_BTCM_MEM_INIT_STATUS_MSS_BTCM_MEM_INIT_STATUS_MEM_STATUS) != 0);
    while (CSL_FEXT(mssCtrl->MSS_BTCM_MEM_INIT_DONE, MSS_CTRL_MSS_BTCM_MEM_INIT_DONE_MSS_BTCM_MEM_INIT_DONE_MEM_INIT_DONE) != 0);
}

void SOC_rcmMemInitMssMailboxMemory(void)
{
    CSL_mss_ctrlRegs *mssCtrl = CSL_MSS_CTRL_getBaseAddress();
    CSL_FINS(mssCtrl->MSS_MAILBOX_MEM_INIT, MSS_CTRL_MSS_MAILBOX_MEM_INIT_MSS_MAILBOX_MEM_INIT_MEM0_INIT, 1);
    while (CSL_FEXT(mssCtrl->MSS_MAILBOX_MEM_INIT_DONE, MSS_CTRL_MSS_MAILBOX_MEM_INIT_DONE_MSS_MAILBOX_MEM_INIT_DONE_MEM0_DONE) != 1);
}

void SOC_rcmMemInitDssMailboxMemory(void)
{
    CSL_dss_ctrlRegs *dssCtrl = CSL_DSS_CTRL_getBaseAddress();
    CSL_FINS(dssCtrl->DSS_MAILBOX_MEMINIT_START, DSS_CTRL_DSS_MAILBOX_MEMINIT_START_DSS_MAILBOX_MEMINIT_START_MEMINIT_START, 1);
    while (CSL_FEXT(dssCtrl->DSS_MAILBOX_MEMINIT_DONE, DSS_CTRL_DSS_MAILBOX_MEMINIT_DONE_DSS_MAILBOX_MEMINIT_DONE_MEMINIT_DONE) != 1);
}

static uint32_t SOC_rcmGetDSSFout(uint32_t Finp, uint32_t div2flag)
{
    uint8_t pllSwitchFlag;
    CSL_mss_toprcmRegs *ptrTopRCMRegs;
    uint32_t FOut;

    ptrTopRCMRegs = CSL_TopRCM_getBaseAddress();
    /* read the Core PLL Lock status */
    pllSwitchFlag = SOC_rcmExtract8 (ptrTopRCMRegs->PLL_DSP_STATUS, 10U, 10U);
    if(pllSwitchFlag)
    {

        uint32_t M, N, M2, FracM;

        N  = CSL_FEXT(ptrTopRCMRegs->PLL_DSP_M2NDIV, MSS_TOPRCM_PLL_DSP_M2NDIV_PLL_DSP_M2NDIV_N);
        M2 = CSL_FEXT(ptrTopRCMRegs->PLL_DSP_M2NDIV, MSS_TOPRCM_PLL_DSP_M2NDIV_PLL_DSP_M2NDIV_M2);
        M  = CSL_FEXT(ptrTopRCMRegs->PLL_DSP_MN2DIV, MSS_TOPRCM_PLL_DSP_MN2DIV_PLL_DSP_MN2DIV_M);
        FracM = CSL_FEXT(ptrTopRCMRegs->PLL_DSP_FRACDIV,MSS_TOPRCM_PLL_DSP_FRACDIV_PLL_DSP_FRACDIV_FRACTIONALM);
        FOut = SOC_rcmADPLLJGetFOut(Finp, N, M, M2, FracM, div2flag);
        DebugP_assert(FOut != 0);
    }
    else
    {
        uint32_t ULOWCLKEN = CSL_FEXT(ptrTopRCMRegs->PLL_DSP_CLKCTRL, MSS_TOPRCM_PLL_DSP_CLKCTRL_PLL_DSP_CLKCTRL_ULOWCLKEN);
        if(ULOWCLKEN == 0)
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

static uint32_t SOC_rcmGetDSSHSDivOut(uint32_t Finp, uint32_t div2flag, SOC_RcmPllHSDIVOutId hsDivOut)
{
    CSL_mss_toprcmRegs *ptrTopRCMRegs;
    uint32_t FOut;
    uint32_t clkDiv;

    FOut = SOC_rcmGetDSSFout(Finp, div2flag);
    ptrTopRCMRegs = CSL_TopRCM_getBaseAddress();
    switch(hsDivOut)
    {
        case SOC_RCM_PLLHSDIV_OUT_0:
        {
            clkDiv = CSL_FEXT(ptrTopRCMRegs->PLL_DSP_HSDIVIDER_CLKOUT0, MSS_TOPRCM_PLL_DSP_HSDIVIDER_CLKOUT0_PLL_DSP_HSDIVIDER_CLKOUT0_DIV);
            break;
        }
        case SOC_RCM_PLLHSDIV_OUT_1:
        {
            clkDiv = CSL_FEXT(ptrTopRCMRegs->PLL_DSP_HSDIVIDER_CLKOUT1, MSS_TOPRCM_PLL_DSP_HSDIVIDER_CLKOUT1_PLL_DSP_HSDIVIDER_CLKOUT1_DIV);
            break;
        }
        case SOC_RCM_PLLHSDIV_OUT_2:
        {
            clkDiv = CSL_FEXT(ptrTopRCMRegs->PLL_DSP_HSDIVIDER_CLKOUT2, MSS_TOPRCM_PLL_DSP_HSDIVIDER_CLKOUT2_PLL_DSP_HSDIVIDER_CLKOUT2_DIV);
            break;
        }
        case SOC_RCM_PLLHSDIV_OUT_3:
        {
            clkDiv = CSL_FEXT(ptrTopRCMRegs->PLL_DSP_HSDIVIDER_CLKOUT3, MSS_TOPRCM_PLL_DSP_HSDIVIDER_CLKOUT3_PLL_DSP_HSDIVIDER_CLKOUT3_DIV);
            break;
        }
        case SOC_RCM_PLLHSDIV_OUT_NONE:
        {
            DebugP_assert(FALSE);
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

    ptrTopRCMRegs = CSL_TopRCM_getBaseAddress();
    /* read the Core PLL Lock status */
    pllSwitchFlag = SOC_rcmExtract8 (ptrTopRCMRegs->PLL_CORE_STATUS, 10U, 10U);
    if(pllSwitchFlag)
    {
        uint32_t M, N, M2, FracM;

        N  = CSL_FEXT(ptrTopRCMRegs->PLL_CORE_M2NDIV, MSS_TOPRCM_PLL_CORE_M2NDIV_PLL_CORE_M2NDIV_N);
        M2 = CSL_FEXT(ptrTopRCMRegs->PLL_CORE_M2NDIV, MSS_TOPRCM_PLL_CORE_M2NDIV_PLL_CORE_M2NDIV_M2);
        M  = CSL_FEXT(ptrTopRCMRegs->PLL_CORE_MN2DIV, MSS_TOPRCM_PLL_CORE_MN2DIV_PLL_CORE_MN2DIV_M);
        FracM = CSL_FEXT(ptrTopRCMRegs->PLL_CORE_FRACDIV,MSS_TOPRCM_PLL_CORE_FRACDIV_PLL_CORE_FRACDIV_FRACTIONALM);
        FOut = SOC_rcmADPLLJGetFOut(Finp, N, M, M2, FracM,div2flag);
        DebugP_assert(FOut != 0);
    }
    else
    {
        uint32_t ULOWCLKEN = CSL_FEXT(ptrTopRCMRegs->PLL_CORE_CLKCTRL, MSS_TOPRCM_PLL_CORE_CLKCTRL_PLL_CORE_CLKCTRL_ULOWCLKEN);
        if(ULOWCLKEN == 0)
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

static uint32_t SOC_rcmGetCoreHSDivOut(uint32_t Finp, uint32_t div2flag, SOC_RcmPllHSDIVOutId hsDivOut)
{
    uint32_t FOut;
    uint32_t clkDiv;
    CSL_mss_toprcmRegs *ptrTopRCMRegs;

    ptrTopRCMRegs = CSL_TopRCM_getBaseAddress();
    FOut = SOC_rcmGetCoreFout(Finp, div2flag);
    switch(hsDivOut)
    {
        case SOC_RCM_PLLHSDIV_OUT_0:
        {
            clkDiv = CSL_FEXT(ptrTopRCMRegs->PLL_CORE_HSDIVIDER_CLKOUT0, MSS_TOPRCM_PLL_CORE_HSDIVIDER_CLKOUT0_PLL_CORE_HSDIVIDER_CLKOUT0_DIV);
            break;
        }
        case SOC_RCM_PLLHSDIV_OUT_1:
        {
            clkDiv = CSL_FEXT(ptrTopRCMRegs->PLL_CORE_HSDIVIDER_CLKOUT1, MSS_TOPRCM_PLL_CORE_HSDIVIDER_CLKOUT1_PLL_CORE_HSDIVIDER_CLKOUT1_DIV);
            break;
        }
        case SOC_RCM_PLLHSDIV_OUT_2:
        {
            clkDiv = CSL_FEXT(ptrTopRCMRegs->PLL_CORE_HSDIVIDER_CLKOUT2, MSS_TOPRCM_PLL_CORE_HSDIVIDER_CLKOUT2_PLL_CORE_HSDIVIDER_CLKOUT2_DIV);
            break;
        }
        case SOC_RCM_PLLHSDIV_OUT_3:
        {
            clkDiv = CSL_FEXT(ptrTopRCMRegs->PLL_CORE_HSDIVIDER_CLKOUT3, MSS_TOPRCM_PLL_CORE_HSDIVIDER_CLKOUT3_PLL_CORE_HSDIVIDER_CLKOUT3_DIV);
            break;
        }
        case SOC_RCM_PLLHSDIV_OUT_NONE:
        {
            DebugP_assert(FALSE);
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

    ptrTopRCMRegs = CSL_TopRCM_getBaseAddress();
    /* read the Core PLL Lock status */
    pllSwitchFlag = SOC_rcmExtract8 (ptrTopRCMRegs->PLL_PER_STATUS, 10U, 10U);
    if(pllSwitchFlag)
    {
        uint32_t M, N, M2, FracM;

        N  = CSL_FEXT(ptrTopRCMRegs->PLL_PER_M2NDIV, MSS_TOPRCM_PLL_PER_M2NDIV_PLL_PER_M2NDIV_N);
        M2 = CSL_FEXT(ptrTopRCMRegs->PLL_PER_M2NDIV, MSS_TOPRCM_PLL_PER_M2NDIV_PLL_PER_M2NDIV_M2);
        M  = CSL_FEXT(ptrTopRCMRegs->PLL_PER_MN2DIV, MSS_TOPRCM_PLL_PER_MN2DIV_PLL_PER_MN2DIV_M);
        FracM = CSL_FEXT(ptrTopRCMRegs->PLL_PER_FRACDIV,MSS_TOPRCM_PLL_PER_FRACDIV_PLL_PER_FRACDIV_FRACTIONALM);
        FOut = SOC_rcmADPLLJGetFOut(Finp, N, M, M2, FracM, div2flag);
        DebugP_assert(FOut != 0);
    }
    else
    {
        uint32_t ULOWCLKEN = CSL_FEXT(ptrTopRCMRegs->PLL_PER_CLKCTRL, MSS_TOPRCM_PLL_PER_CLKCTRL_PLL_PER_CLKCTRL_ULOWCLKEN);
        if(ULOWCLKEN == 0)
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

static uint32_t SOC_rcmGetPerHSDivOut(uint32_t Finp, uint32_t div2flag, SOC_RcmPllHSDIVOutId hsDivOut)
{
    uint32_t FOut;
    uint32_t clkDiv;
    CSL_mss_toprcmRegs *ptrTopRCMRegs;

    ptrTopRCMRegs = CSL_TopRCM_getBaseAddress();

    FOut = SOC_rcmGetPerFout(Finp, div2flag);
    switch(hsDivOut)
    {
        case SOC_RCM_PLLHSDIV_OUT_0:
        {
            clkDiv = CSL_FEXT(ptrTopRCMRegs->PLL_PER_HSDIVIDER_CLKOUT0, MSS_TOPRCM_PLL_PER_HSDIVIDER_CLKOUT0_PLL_PER_HSDIVIDER_CLKOUT0_DIV);
            break;
        }
        case SOC_RCM_PLLHSDIV_OUT_1:
        {
            clkDiv = CSL_FEXT(ptrTopRCMRegs->PLL_PER_HSDIVIDER_CLKOUT1, MSS_TOPRCM_PLL_PER_HSDIVIDER_CLKOUT1_PLL_PER_HSDIVIDER_CLKOUT1_DIV);
            break;
        }
        case SOC_RCM_PLLHSDIV_OUT_2:
        {
            clkDiv = CSL_FEXT(ptrTopRCMRegs->PLL_PER_HSDIVIDER_CLKOUT2, MSS_TOPRCM_PLL_PER_HSDIVIDER_CLKOUT2_PLL_PER_HSDIVIDER_CLKOUT2_DIV);
            break;
        }
        case SOC_RCM_PLLHSDIV_OUT_3:
        {
            clkDiv = CSL_FEXT(ptrTopRCMRegs->PLL_PER_HSDIVIDER_CLKOUT3, MSS_TOPRCM_PLL_PER_HSDIVIDER_CLKOUT3_PLL_PER_HSDIVIDER_CLKOUT3_DIV);
            break;
        }
        case SOC_RCM_PLLHSDIV_OUT_NONE:
        {
            DebugP_assert(FALSE);
            clkDiv = 0;
            break;
        }
    }

    return (FOut/(clkDiv  + 1));
}

static uint32_t SOC_rcmGetApllHSDivOutFreq(SOC_RcmApllId apllId, SOC_RcmPllHSDIVOutId hsDivOut)
{
    CSL_mss_toprcmRegs *ptrTopRCMRegs;
    uint32_t FOut;
    uint32_t clkDiv;

    if(apllId == SOC_RcmAPLLID_1P2G)
    {
        FOut = gSocRcmFixedClocksTbl[SOC_RCM_FIXEDCLKID_APLL1P2GHZ].fOut;
    }
    else
    {
        FOut = gSocRcmFixedClocksTbl[SOC_RCM_FIXEDCLKID_APLL1P8GHZ].fOut;
    }

    ptrTopRCMRegs = CSL_TopRCM_getBaseAddress();
    switch(hsDivOut)
    {
        case SOC_RCM_PLLHSDIV_OUT_0:
        {
            if(apllId == SOC_RcmAPLLID_1P2G)
            {
                clkDiv = CSL_FEXT(ptrTopRCMRegs->PLL_1P2_HSDIVIDER_CLKOUT0, MSS_TOPRCM_PLL_1P2_HSDIVIDER_CLKOUT0_PLL_1P2_HSDIVIDER_CLKOUT0_DIV);
            }
            else
            {
                clkDiv = CSL_FEXT(ptrTopRCMRegs->PLL_1P8_HSDIVIDER_CLKOUT0, MSS_TOPRCM_PLL_1P8_HSDIVIDER_CLKOUT0_PLL_1P8_HSDIVIDER_CLKOUT0_DIV);
            }
            break;
        }
        case SOC_RCM_PLLHSDIV_OUT_1:
        {
            if(apllId == SOC_RcmAPLLID_1P2G)
            {
                clkDiv = CSL_FEXT(ptrTopRCMRegs->PLL_1P2_HSDIVIDER_CLKOUT1, MSS_TOPRCM_PLL_1P2_HSDIVIDER_CLKOUT1_PLL_1P2_HSDIVIDER_CLKOUT1_DIV);
            }
            else
            {
                clkDiv = CSL_FEXT(ptrTopRCMRegs->PLL_1P8_HSDIVIDER_CLKOUT1, MSS_TOPRCM_PLL_1P8_HSDIVIDER_CLKOUT1_PLL_1P8_HSDIVIDER_CLKOUT1_DIV);
            }
            break;
        }
        case SOC_RCM_PLLHSDIV_OUT_2:
        {
            if(apllId == SOC_RcmAPLLID_1P2G)
            {
                clkDiv = CSL_FEXT(ptrTopRCMRegs->PLL_1P2_HSDIVIDER_CLKOUT2, MSS_TOPRCM_PLL_1P2_HSDIVIDER_CLKOUT2_PLL_1P2_HSDIVIDER_CLKOUT2_DIV);
            }
            else
            {
                clkDiv = CSL_FEXT(ptrTopRCMRegs->PLL_1P8_HSDIVIDER_CLKOUT2, MSS_TOPRCM_PLL_1P8_HSDIVIDER_CLKOUT2_PLL_1P8_HSDIVIDER_CLKOUT2_DIV);
            }
            break;
        }
        case SOC_RCM_PLLHSDIV_OUT_3:
        {
            if(apllId == SOC_RcmAPLLID_1P2G)
            {
                clkDiv = CSL_FEXT(ptrTopRCMRegs->PLL_1P2_HSDIVIDER_CLKOUT3, MSS_TOPRCM_PLL_1P2_HSDIVIDER_CLKOUT3_PLL_1P2_HSDIVIDER_CLKOUT3_DIV);
            }
            else
            {
                clkDiv = CSL_FEXT(ptrTopRCMRegs->PLL_1P8_HSDIVIDER_CLKOUT3, MSS_TOPRCM_PLL_1P8_HSDIVIDER_CLKOUT3_PLL_1P8_HSDIVIDER_CLKOUT3_DIV);
            }
            break;
        }
        case SOC_RCM_PLLHSDIV_OUT_NONE:
        {
            DebugP_assert(FALSE);
            clkDiv = 0;
            break;
        }
    }
    return (FOut/(clkDiv + 1));
}

static uint32_t SOC_rcmADPLLJGetFOut(uint32_t Finp, uint32_t N, uint32_t M, uint32_t M2, uint32_t FracM, uint32_t div2flag)
{
    uint32_t i;
    uint32_t FOut;
    uint32_t Nmatch;

    if(div2flag)
    {
        Nmatch = ((N + 1) * 2) - 1;
    }
    else
    {
        Nmatch = N;
    }
    for (i = 0; i < SOC_RCM_UTILS_ARRAYSIZE(gSocRcmADPLLJConfigTbl); i++)
    {
        if((gSocRcmADPLLJConfigTbl[i].Finp == Finp)   &&
            (gSocRcmADPLLJConfigTbl[i].FracM == FracM) &&
            (gSocRcmADPLLJConfigTbl[i].M == M)         &&
            (gSocRcmADPLLJConfigTbl[i].M2 == M2)       &&
            (gSocRcmADPLLJConfigTbl[i].N  == Nmatch))
        {
            break;
        }
    }
    if(i < SOC_RCM_UTILS_ARRAYSIZE(gSocRcmADPLLJConfigTbl))
    {
        FOut = SOC_RCM_FREQ_MHZ2HZ(gSocRcmADPLLJConfigTbl[i].Fout);
    }
    else
    {
        FOut = 0;
    }
    return FOut;
}

void SOC_rcmGetEfuseQSPIConfig(SOC_RcmEfuseQspiConfig *qspiEfuseCfg)
{
    CSL_top_ctrlRegs* ptrTopCtrlRegs;

    ptrTopCtrlRegs = CSL_TopCtrl_getBaseAddress();

    qspiEfuseCfg->qspiClockFreqId  = (SOC_RcmQspiClockFreqId) CSL_TopCtrl_readQSPIClkFreqEfuse(ptrTopCtrlRegs);
    qspiEfuseCfg->flashClockModeId = (SOC_RcmEfuseFlashClkModeId) CSL_TopCtrl_readFlashModeEfuse(ptrTopCtrlRegs);
}

void SOC_rcmBSSR4Unhalt(void)
{
    CSL_rss_ctrlRegs *rssCtrl = CSL_RSS_CTRL_getBaseAddress();
    volatile uint32_t unhaltStatus;

    unhaltStatus = CSL_FEXT(rssCtrl->BSS_CONTROL, RSS_CTRL_BSS_CONTROL_BSS_CONTROL_HALT);
    if(unhaltStatus != 0x0)
    {
        /* Bring the BSS out of the HALT state */
        CSL_FINS(rssCtrl->BSS_CONTROL, RSS_CTRL_BSS_CONTROL_BSS_CONTROL_HALT, 0U);
    }
    return;
}

/*****************************************************************************
 *  The register RSS_PROC_CTRL:RSS_CR4_BOOT_INFO_REG0 contains the boot status
 *  information, which includes the following fields
 *
 *   XTAL Frequency    15:0    XTAL frequency in MHz as an unsigned number
 *   APLL Calibration Done    16
 *   APLL Calibration Status    17
 *   BSS Boot Done    18    The bit will be set once the BSS boots up before entering the idle task loop
 *   BSS Boot Status    19    The bit indicated that all the boot-time monitors have passed
 *   BSS Fault Status    22:20
 *   Firmware Fault Status Number
 *
 *   0    No-Fault
 *   1    BSS Firmware ASSERT
 *   2    BSS Firmware CPU Abort
 *   3    ESM Group 1 Error
 *   4    ESM Group 2 Error
 *   5    ANA LDO WU fault (Deprecated)
 *   6    ANA LDO SC fault
 *
 *   Mailbox Boot Config Status    26:23
 *   The set bits indicate the successful configuration of the mailbox system
 *   23    MSS configuration Success
 *   24    DSS configuration Success
 *   26:25    Reserved
 *   Reserved    31:27
 *
 *****************************************************************************/
#define CSL_RSS_PROC_CTRL_RSS_CR4_BOOT_INFO_REG0_RSS_CR4_BOOT_INFO_REG0_BOOTSTATUS_MASK  (0x007F0000U)
#define CSL_RSS_PROC_CTRL_RSS_CR4_BOOT_INFO_REG0_RSS_CR4_BOOT_INFO_REG0_BOOTSTATUS_SHIFT (16U)

void SOC_rcmWaitBSSBootComplete(void)
{
    CSL_rss_proc_ctrlRegs *rssProcCtrl = CSL_RSS_PROC_CTRL_getBaseAddress();
    volatile uint32_t bssStatus;

    do
    {
        /* Wait until the value in the Spare reg0 [22:16] becomes 0xF.
         * SPARE0[16] : 0 indicates APLL calibration is ongoing
         *              1 indicates APLL calibration is complete
         * SPARE0[17] : 0 indicates APLL calibration did not succeed
         *              1 indicates APLL calibration was successful
         * SPARE0[18] : 0 indicates BSS powerup is ongoing
         *              1 indicates BSS powerup is complete
         * SPARE0[19] : 0 indicates BSS powerup did not succeed
         *              1 indicates BSS powerup was successful
         * SPARE0[22:20] : Firmware Fault Status
         *              0 indicates No-Fault
         *              1 indicates BSS Firmware ASSERT
         *              2 indicates BSS Firmware CPU Abort
         *              3 indicates ESM Group 1 Error
         *              4 indicates ESM Group 2 Error
         *              5 indicates ANA LDO WU fault (Deprecated)
         *              6 indicates ANA LDO SC fault
         */
        bssStatus = CSL_FEXT(rssProcCtrl->RSS_CR4_BOOT_INFO_REG0,
                             RSS_PROC_CTRL_RSS_CR4_BOOT_INFO_REG0_RSS_CR4_BOOT_INFO_REG0_BOOTSTATUS);
    } while(bssStatus != 0xFU);
}

void SOC_rcmSetFastchargeBias(void)
{
    CSL_mss_toprcmRegs *ptrTopRCMRegs;

    /* Core PLL settings */
    ptrTopRCMRegs = CSL_TopRCM_getBaseAddress();

    ptrTopRCMRegs->ANA_REG_CLK_CTRL_REG1_XO_SLICER  =
        SOC_rcmInsert32 (ptrTopRCMRegs->ANA_REG_CLK_CTRL_REG1_XO_SLICER,
                        CSL_MSS_TOPRCM_ANA_REG_CLK_CTRL_REG1_XO_SLICER_FASTCHARGEZ_BIAS_XO_SLICER_SHIFT,
                        CSL_MSS_TOPRCM_ANA_REG_CLK_CTRL_REG1_XO_SLICER_FASTCHARGEZ_BIAS_XO_SLICER_SHIFT,
                        CSL_MSS_TOPRCM_ANA_REG_CLK_CTRL_REG1_XO_SLICER_FASTCHARGEZ_BIAS_XO_SLICER_MAX);

}

uint8_t SOC_rcmGetEfusePGVer(void)
{
    CSL_top_ctrlRegs* ptrTopCtrlRegs = CSL_TopCtrl_getBaseAddress();

    return CSL_TopCtrl_readPgVerEfuse(ptrTopCtrlRegs);
}

void SOC_rcmGetEfuseBootFrequency(SOC_RcmEfuseBootFreqConfig *bootFreqEfuseCfg)
{
    CSL_top_ctrlRegs* ptrTopCtrlRegs;
    uint8_t bootFreqValEfuse;

    ptrTopCtrlRegs = CSL_TopCtrl_getBaseAddress();

    bootFreqValEfuse = CSL_TopCtrl_readR5ClkFreqEfuse (ptrTopCtrlRegs);
    switch (bootFreqValEfuse)
    {
        case SOC_RCM_EFUSE_R5_CLK_300_SYS_CLK_150MHz:
        {
            bootFreqEfuseCfg->r5FreqHz     = SOC_RCM_FREQ_MHZ2HZ(300U);
            bootFreqEfuseCfg->sysClkFreqHz = SOC_RCM_FREQ_MHZ2HZ(150U);
            break;
        }
        case SOC_RCM_EFUSE_R5_CLK_200_SYS_CLK_200MHz:
        {
            bootFreqEfuseCfg->r5FreqHz     = SOC_RCM_FREQ_MHZ2HZ(200U);
            bootFreqEfuseCfg->sysClkFreqHz = SOC_RCM_FREQ_MHZ2HZ(200U);
            break;
        }
        case SOC_RCM_EFUSE_R5_CLK_400_SYS_CLK_200MHz:
        {
            bootFreqEfuseCfg->r5FreqHz     = SOC_RCM_FREQ_MHZ2HZ(400U);
            bootFreqEfuseCfg->sysClkFreqHz = SOC_RCM_FREQ_MHZ2HZ(200U);
            break;
        }
        default:
            DebugP_assert(FALSE);
    }
}

uint32_t SOC_rcmIsDualCoreSwitchSupported(void)
{
    CSL_top_ctrlRegs* ptrTopCtrlRegs;
    uint8_t dualCoreBootEnable, dualCoreSwitchDisable;
    uint32_t retVal = FALSE;

    ptrTopCtrlRegs = CSL_TopCtrl_getBaseAddress();
    dualCoreBootEnable = CSL_TopCtrl_dualCoreBootEnableEfuse(ptrTopCtrlRegs);
    dualCoreSwitchDisable = CSL_TopCtrl_dualCoreSwitchDisableEfuse(ptrTopCtrlRegs);
    if((dualCoreBootEnable == 0) && (dualCoreSwitchDisable == 0))
    {
        retVal = TRUE;
    }
    return retVal;
}

void SOC_rcmGetDeviceFrequency(SOC_RcmDeviceFreqConfig *deviceFreqEfuseCfg)
{
    CSL_top_ctrlRegs* ptrTopCtrlRegs;
    SOC_RcmEfusePkgType deviceTypeEfuse;

    ptrTopCtrlRegs = CSL_TopCtrl_getBaseAddress();

    deviceTypeEfuse = CSL_TopCtrl_readDeviceTypeEfuse (ptrTopCtrlRegs);
    *deviceFreqEfuseCfg = gSocRcmDeviceFreqConfigTbl[deviceTypeEfuse];
}

void SOC_rcmGetPackageType(SOC_RcmEfusePkgType *deviceTypeEfuse)
{
    CSL_top_ctrlRegs* ptrTopCtrlRegs;

    ptrTopCtrlRegs = CSL_TopCtrl_getBaseAddress();

    *deviceTypeEfuse = CSL_TopCtrl_readDeviceTypeEfuse (ptrTopCtrlRegs);
}

void SOC_rcmConfigEthMacIf(void)
{
    CSL_mss_rcmRegs *ptrMSSRCMRegs;
    uint32_t clkFreq = 0U;
    uint32_t clkDivisor;
    uint32_t mii10ClkDivVal;
    uint32_t clkSrcVal;

    ptrMSSRCMRegs = CSL_RCM_getBaseAddress();
    clkSrcVal = gSocRcmCpswMiiClkSrcValMap[SOC_RcmCpswMiiClockSource_DPLL_CORE_HSDIV0_CLKOUT1];
    CSL_FINS(ptrMSSRCMRegs->MSS_CPSW_MII_CLK_SRC_SEL, MSS_RCM_MSS_CPSW_MII_CLK_SRC_SEL_MSS_CPSW_MII_CLK_SRC_SEL_CLKSRCSEL,  clkSrcVal);

    clkFreq = SOC_rcmGetFreq(gSocRcmCpswMiiClkSrcInfoMap[SOC_RcmCpswMiiClockSource_DPLL_CORE_HSDIV0_CLKOUT1]);
    clkDivisor = SOC_rcmGetModuleClkDivVal(clkFreq, SOC_RCM_FREQ_MHZ2HZ(50U));
    ptrMSSRCMRegs->MSS_MII100_CLK_DIV_VAL = SOC_rcmInsert16 (ptrMSSRCMRegs->MSS_MII100_CLK_DIV_VAL, 11U, 0U, SOC_rcmGetModuleClkDivRegVal(clkDivisor));
    clkDivisor = SOC_rcmGetModuleClkDivVal(clkFreq, SOC_RCM_FREQ_MHZ2HZ(5U));
    mii10ClkDivVal = (clkDivisor & 0xFF) | ((clkDivisor & 0xFF) << 8) | ((clkDivisor & 0xFF) << 16);
    ptrMSSRCMRegs->MSS_MII10_CLK_DIV_VAL  = SOC_rcmInsert32 (ptrMSSRCMRegs->MSS_MII10_CLK_DIV_VAL, 23U, 0U, mii10ClkDivVal);
    clkDivisor = SOC_rcmGetModuleClkDivVal(clkFreq, SOC_RCM_FREQ_MHZ2HZ(50U));
    ptrMSSRCMRegs->MSS_RGMII_CLK_DIV_VAL = SOC_rcmInsert16 (ptrMSSRCMRegs->MSS_RGMII_CLK_DIV_VAL, 11U, 0U, SOC_rcmGetModuleClkDivRegVal(clkDivisor));
}

void SOC_rcmCoreDpllHSDivOutEnable(uint32_t hsDivOutIdx, uint32_t divVal)
{
    CSL_mss_toprcmRegs *ptrTopRCMRegs;

    /* Core PLL settings */
    ptrTopRCMRegs = CSL_TopRCM_getBaseAddress();

    switch (hsDivOutIdx)
    {
        case SOC_RCM_PLL_HSDIV_OUTPUT_IDX0:
            ptrTopRCMRegs->PLL_CORE_HSDIVIDER_CLKOUT0 = SOC_rcmInsert8 (ptrTopRCMRegs->PLL_CORE_HSDIVIDER_CLKOUT0, 4U, 0U, (divVal - 1));

            /* Generate Trigger to latch these values */
            ptrTopRCMRegs->PLL_CORE_HSDIVIDER         = SOC_rcmInsert8 (ptrTopRCMRegs->PLL_CORE_HSDIVIDER, 2U, 2U, 0x1U);
            ptrTopRCMRegs->PLL_CORE_HSDIVIDER         = SOC_rcmInsert8 (ptrTopRCMRegs->PLL_CORE_HSDIVIDER, 2U, 2U, 0x0U);

            ptrTopRCMRegs->PLL_CORE_HSDIVIDER_CLKOUT0 = SOC_rcmInsert8 (ptrTopRCMRegs->PLL_CORE_HSDIVIDER_CLKOUT0, 8U, 8U, 0x1U);
            while(CSL_FEXT(ptrTopRCMRegs->PLL_CORE_HSDIVIDER_CLKOUT0,MSS_TOPRCM_PLL_CORE_HSDIVIDER_CLKOUT0_PLL_CORE_HSDIVIDER_CLKOUT0_STATUS) != 1);
            break;
        case SOC_RCM_PLL_HSDIV_OUTPUT_IDX1:
            ptrTopRCMRegs->PLL_CORE_HSDIVIDER_CLKOUT1 = SOC_rcmInsert8 (ptrTopRCMRegs->PLL_CORE_HSDIVIDER_CLKOUT1, 4U, 0U, (divVal - 1));

            /* Generate Trigger to latch these values */
            ptrTopRCMRegs->PLL_CORE_HSDIVIDER         = SOC_rcmInsert8 (ptrTopRCMRegs->PLL_CORE_HSDIVIDER, 2U, 2U, 0x1U);
            ptrTopRCMRegs->PLL_CORE_HSDIVIDER         = SOC_rcmInsert8 (ptrTopRCMRegs->PLL_CORE_HSDIVIDER, 2U, 2U, 0x0U);

            ptrTopRCMRegs->PLL_CORE_HSDIVIDER_CLKOUT1 = SOC_rcmInsert8 (ptrTopRCMRegs->PLL_CORE_HSDIVIDER_CLKOUT1, 8U, 8U, 0x1U);
            while(CSL_FEXT(ptrTopRCMRegs->PLL_CORE_HSDIVIDER_CLKOUT1,MSS_TOPRCM_PLL_CORE_HSDIVIDER_CLKOUT1_PLL_CORE_HSDIVIDER_CLKOUT1_STATUS) != 1);
            break;
        case SOC_RCM_PLL_HSDIV_OUTPUT_IDX2:
            ptrTopRCMRegs->PLL_CORE_HSDIVIDER_CLKOUT2 = SOC_rcmInsert8 (ptrTopRCMRegs->PLL_CORE_HSDIVIDER_CLKOUT2, 4U, 0U, (divVal - 1));

            /* Generate Trigger to latch these values */
            ptrTopRCMRegs->PLL_CORE_HSDIVIDER         = SOC_rcmInsert8 (ptrTopRCMRegs->PLL_CORE_HSDIVIDER, 2U, 2U, 0x1U);
            ptrTopRCMRegs->PLL_CORE_HSDIVIDER         = SOC_rcmInsert8 (ptrTopRCMRegs->PLL_CORE_HSDIVIDER, 2U, 2U, 0x0U);

            ptrTopRCMRegs->PLL_CORE_HSDIVIDER_CLKOUT2 = SOC_rcmInsert8 (ptrTopRCMRegs->PLL_CORE_HSDIVIDER_CLKOUT2, 8U, 8U, 0x1U);
            while(CSL_FEXT(ptrTopRCMRegs->PLL_CORE_HSDIVIDER_CLKOUT2,MSS_TOPRCM_PLL_CORE_HSDIVIDER_CLKOUT2_PLL_CORE_HSDIVIDER_CLKOUT2_STATUS) != 1);
            break;
        case SOC_RCM_PLL_HSDIV_OUTPUT_IDX3:
            ptrTopRCMRegs->PLL_CORE_HSDIVIDER_CLKOUT3 = SOC_rcmInsert8 (ptrTopRCMRegs->PLL_CORE_HSDIVIDER_CLKOUT3, 4U, 0U, (divVal - 1));

            /* Generate Trigger to latch these values */
            ptrTopRCMRegs->PLL_CORE_HSDIVIDER         = SOC_rcmInsert8 (ptrTopRCMRegs->PLL_CORE_HSDIVIDER, 2U, 2U, 0x1U);
            ptrTopRCMRegs->PLL_CORE_HSDIVIDER         = SOC_rcmInsert8 (ptrTopRCMRegs->PLL_CORE_HSDIVIDER, 2U, 2U, 0x0U);

            ptrTopRCMRegs->PLL_CORE_HSDIVIDER_CLKOUT3 = SOC_rcmInsert8 (ptrTopRCMRegs->PLL_CORE_HSDIVIDER_CLKOUT3, 8U, 8U, 0x1U);
            while(CSL_FEXT(ptrTopRCMRegs->PLL_CORE_HSDIVIDER_CLKOUT3,MSS_TOPRCM_PLL_CORE_HSDIVIDER_CLKOUT3_PLL_CORE_HSDIVIDER_CLKOUT3_STATUS) != 1);
            break;
        default:
            DebugP_assert(FALSE);
    }
}

void SOC_rcmCoreDpllDisable(void)
{
    CSL_mss_toprcmRegs *ptrTopRCMRegs;
    uint32_t ptrClkCtrl;

    /* Core PLL settings */
    ptrTopRCMRegs = CSL_TopRCM_getBaseAddress();
    /* 1. Make sure all clocks are switched on in APLL
     * 2. SWitch off ADPLL (LDOEN = 0, IDLE = 1, CLKOUTEN=0)
     */
    /*MSS_TOPRCM_Ptr->PLL_CORE_CLKCTRL_UN.PLL_CORE_CLKCTRL_UL = 0x09831000; */
    ptrClkCtrl    = ptrTopRCMRegs->PLL_CORE_CLKCTRL;
    CSL_FINS(ptrClkCtrl, MSS_TOPRCM_PLL_CORE_CLKCTRL_PLL_CORE_CLKCTRL_CLKOUTLDOEN, 0);
    CSL_FINS(ptrClkCtrl, MSS_TOPRCM_PLL_CORE_CLKCTRL_PLL_CORE_CLKCTRL_IDLE, 1);
    CSL_FINS(ptrClkCtrl, MSS_TOPRCM_PLL_CORE_CLKCTRL_PLL_CORE_CLKCTRL_CLKOUTEN, 0);
    ptrTopRCMRegs->PLL_CORE_CLKCTRL = ptrClkCtrl;

    /*MSS_TOPRCM_Ptr->PLL_CORE_TENABLE_UN.PLL_CORE_TENABLE_UL = 0x1;*/
    CSL_FINS(ptrTopRCMRegs->PLL_CORE_TENABLE,MSS_TOPRCM_PLL_CORE_TENABLE_PLL_CORE_TENABLE_TENABLE, 0x1);

    /*MSS_TOPRCM_Ptr->PLL_CORE_CLKCTRL_UN.PLL_CORE_CLKCTRL_UL = 0x09831001; */
    ptrClkCtrl    = ptrTopRCMRegs->PLL_CORE_CLKCTRL;
    CSL_FINS(ptrClkCtrl, MSS_TOPRCM_PLL_CORE_CLKCTRL_PLL_CORE_CLKCTRL_CLKOUTLDOEN, 0);
    CSL_FINS(ptrClkCtrl, MSS_TOPRCM_PLL_CORE_CLKCTRL_PLL_CORE_CLKCTRL_IDLE, 1);
    CSL_FINS(ptrClkCtrl, MSS_TOPRCM_PLL_CORE_CLKCTRL_PLL_CORE_CLKCTRL_CLKOUTEN, 0);
    CSL_FINS(ptrClkCtrl, MSS_TOPRCM_PLL_CORE_CLKCTRL_PLL_CORE_CLKCTRL_TINTZ, 1);
    ptrTopRCMRegs->PLL_CORE_CLKCTRL = ptrClkCtrl;

    /*MSS_TOPRCM_Ptr->PLL_CORE_TENABLE_UN.PLL_CORE_TENABLE_UL = 0x0;      */
    CSL_FINS(ptrTopRCMRegs->PLL_CORE_TENABLE,MSS_TOPRCM_PLL_CORE_TENABLE_PLL_CORE_TENABLE_TENABLE, 0x0);
}

void SOC_rcmDspDpllDisable(void)
{
    CSL_mss_toprcmRegs *ptrTopRCMRegs;
    uint32_t ptrClkCtrl;

    /* Core PLL settings */
    ptrTopRCMRegs = CSL_TopRCM_getBaseAddress();
    /* 1. Make sure all clocks are switched on in APLL
     * 2. SWitch off ADPLL (LDOEN = 0, IDLE = 1, CLKOUTEN=0)
     */
    /*MSS_TOPRCM_Ptr->PLL_DSP_CLKCTRL_UN.PLL_DSP_CLKCTRL_UL = 0x09831000; */
    ptrClkCtrl    = ptrTopRCMRegs->PLL_DSP_CLKCTRL;
    CSL_FINS(ptrClkCtrl, MSS_TOPRCM_PLL_DSP_CLKCTRL_PLL_DSP_CLKCTRL_CLKOUTLDOEN, 0);
    CSL_FINS(ptrClkCtrl, MSS_TOPRCM_PLL_DSP_CLKCTRL_PLL_DSP_CLKCTRL_IDLE, 1);
    CSL_FINS(ptrClkCtrl, MSS_TOPRCM_PLL_DSP_CLKCTRL_PLL_DSP_CLKCTRL_CLKOUTEN, 0);
    ptrTopRCMRegs->PLL_DSP_CLKCTRL = ptrClkCtrl;

    /*MSS_TOPRCM_Ptr->PLL_DSP_TENABLE_UN.PLL_DSP_TENABLE_UL = 0x1;*/
    CSL_FINS(ptrTopRCMRegs->PLL_DSP_TENABLE,MSS_TOPRCM_PLL_DSP_TENABLE_PLL_DSP_TENABLE_TENABLE, 0x1);

    /*MSS_TOPRCM_Ptr->PLL_DSP_CLKCTRL_UN.PLL_DSP_CLKCTRL_UL = 0x09831001; */
    ptrClkCtrl    = ptrTopRCMRegs->PLL_DSP_CLKCTRL;
    CSL_FINS(ptrClkCtrl, MSS_TOPRCM_PLL_DSP_CLKCTRL_PLL_DSP_CLKCTRL_CLKOUTLDOEN, 0);
    CSL_FINS(ptrClkCtrl, MSS_TOPRCM_PLL_DSP_CLKCTRL_PLL_DSP_CLKCTRL_IDLE, 1);
    CSL_FINS(ptrClkCtrl, MSS_TOPRCM_PLL_DSP_CLKCTRL_PLL_DSP_CLKCTRL_CLKOUTEN, 0);
    CSL_FINS(ptrClkCtrl, MSS_TOPRCM_PLL_DSP_CLKCTRL_PLL_DSP_CLKCTRL_TINTZ, 1);
    ptrTopRCMRegs->PLL_DSP_CLKCTRL = ptrClkCtrl;

    /*MSS_TOPRCM_Ptr->PLL_DSP_TENABLE_UN.PLL_DSP_TENABLE_UL = 0x0;      */
    CSL_FINS(ptrTopRCMRegs->PLL_DSP_TENABLE,MSS_TOPRCM_PLL_DSP_TENABLE_PLL_DSP_TENABLE_TENABLE, 0x0);
}

void SOC_rcmPerDpllDisable(void)
{
    CSL_mss_toprcmRegs *ptrTopRCMRegs;
    uint32_t ptrClkCtrl;

    /* Core PLL settings */
    ptrTopRCMRegs = CSL_TopRCM_getBaseAddress();
    /* 1. Make sure all clocks are switched on in APLL
     * 2. SWitch off ADPLL (LDOEN = 0, IDLE = 1, CLKOUTEN=0)
     */
    /*MSS_TOPRCM_Ptr->PLL_DSP_CLKCTRL_UN.PLL_DSP_CLKCTRL_UL = 0x09831000; */
    ptrClkCtrl    = ptrTopRCMRegs->PLL_PER_CLKCTRL;
    CSL_FINS(ptrClkCtrl, MSS_TOPRCM_PLL_PER_CLKCTRL_PLL_PER_CLKCTRL_CLKOUTLDOEN, 0);
    CSL_FINS(ptrClkCtrl, MSS_TOPRCM_PLL_PER_CLKCTRL_PLL_PER_CLKCTRL_IDLE, 1);
    CSL_FINS(ptrClkCtrl, MSS_TOPRCM_PLL_PER_CLKCTRL_PLL_PER_CLKCTRL_CLKOUTEN, 0);
    ptrTopRCMRegs->PLL_PER_CLKCTRL = ptrClkCtrl;

    /*MSS_TOPRCM_Ptr->PLL_PER_TENABLE_UN.PLL_PER_TENABLE_UL = 0x1;*/
    CSL_FINS(ptrTopRCMRegs->PLL_PER_TENABLE,MSS_TOPRCM_PLL_PER_TENABLE_PLL_PER_TENABLE_TENABLE, 0x1);

    /*MSS_TOPRCM_Ptr->PLL_PER_CLKCTRL_UN.PLL_PER_CLKCTRL_UL = 0x09831001; */
    ptrClkCtrl    = ptrTopRCMRegs->PLL_PER_CLKCTRL;
    CSL_FINS(ptrClkCtrl, MSS_TOPRCM_PLL_PER_CLKCTRL_PLL_PER_CLKCTRL_CLKOUTLDOEN, 0);
    CSL_FINS(ptrClkCtrl, MSS_TOPRCM_PLL_PER_CLKCTRL_PLL_PER_CLKCTRL_IDLE, 1);
    CSL_FINS(ptrClkCtrl, MSS_TOPRCM_PLL_PER_CLKCTRL_PLL_PER_CLKCTRL_CLKOUTEN, 0);
    CSL_FINS(ptrClkCtrl, MSS_TOPRCM_PLL_PER_CLKCTRL_PLL_PER_CLKCTRL_TINTZ, 1);
    ptrTopRCMRegs->PLL_PER_CLKCTRL = ptrClkCtrl;

    /*MSS_TOPRCM_Ptr->PLL_PER_TENABLE_UN.PLL_PER_TENABLE_UL = 0x0;      */
    CSL_FINS(ptrTopRCMRegs->PLL_PER_TENABLE,MSS_TOPRCM_PLL_PER_TENABLE_PLL_PER_TENABLE_TENABLE, 0x0);
}

void SOC_rcmDspPowerOnReset(void)
{
    CSL_dss_rcmRegs *ptrRCMRegs;
    ptrRCMRegs = CSL_DSSRCM_getBaseAddress();
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
    ptrRCMRegs = CSL_RCM_getBaseAddress ();
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
    CSL_mss_ctrlRegs *mssCtrl = CSL_MSS_CTRL_getBaseAddress ();

    regVal = mssCtrl->R5_CONTROL;
    CSL_FINS(regVal, MSS_CTRL_R5_CONTROL_R5_CONTROL_RESET_FSM_TRIGGER, 0x7);
    mssCtrl->R5_CONTROL = regVal;
#if defined(__ARM_ARCH_7R__)
    __wfi();
#endif
}

void SOC_rcmCr5bUnhalt(void)
{
    CSL_mss_ctrlRegs *mssCtrl = CSL_MSS_CTRL_getBaseAddress ();
    //Core B unhalt
    CSL_FINS(mssCtrl->R5_COREB_HALT, MSS_CTRL_R5_COREB_HALT_R5_COREB_HALT_HALT, 0x0);
}

void SOC_rcmC66xStart(void)
{
    CSL_dss_rcmRegs *ptrRCMRegs;
    ptrRCMRegs = CSL_DSSRCM_getBaseAddress();

    CSL_FINS(ptrRCMRegs->DSP_PD_CTRL, DSS_RCM_DSP_PD_CTRL_DSP_PD_CTRL_INTERRUPT_MASK, 0x1);
    CSL_FINS(ptrRCMRegs->DSP_PD_CTRL, DSS_RCM_DSP_PD_CTRL_DSP_PD_CTRL_INTERRUPT_MASK, 0x0);
    CSL_FINS(ptrRCMRegs->DSP_PD_CTRL, DSS_RCM_DSP_PD_CTRL_DSP_PD_CTRL_PROC_HALT, 0x0);
}

void SOC_rcmR5ConfigLockStep(void)
{
    CSL_mss_ctrlRegs *mssCtrl = CSL_MSS_CTRL_getBaseAddress ();
    uint32_t regVal;
    regVal = mssCtrl->R5_CONTROL;
    CSL_FINS(regVal, MSS_CTRL_R5_CONTROL_R5_CONTROL_LOCK_STEP, 0x7);
    CSL_FINS(regVal, MSS_CTRL_R5_CONTROL_R5_CONTROL_LOCK_STEP_SWITCH_WAIT, 0x7);
    mssCtrl->R5_CONTROL = regVal;
}

void SOC_rcmR5ConfigDualCore(void)
{
    uint32_t regVal;
    CSL_mss_ctrlRegs *mssCtrl = CSL_MSS_CTRL_getBaseAddress ();

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

void SOC_generateSwWarmReset(void)
{
    CSL_mss_toprcmRegs *ptrTOPRCMRegs;

    ptrTOPRCMRegs = CSL_TopRCM_getBaseAddress();

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

    ptrTOPRCMRegs = CSL_TopRCM_getBaseAddress();

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

    ptrTOPRCMRegs = CSL_TopRCM_getBaseAddress();

    /* Unlock CONTROLSS_CTRL registers */
    SOC_controlModuleUnlockMMR(SOC_DOMAIN_ID_MSS_TOP_RCM, 0);

    CSL_FINS(ptrTOPRCMRegs->SYS_RST_CAUSE_CLR, MSS_TOPRCM_SYS_RST_CAUSE_CLR_SYS_RST_CAUSE_CLR_CLEAR, 0x1);

    /* Lock CONTROLSS_CTRL registers */
    SOC_controlModuleLockMMR(SOC_DOMAIN_ID_MSS_TOP_RCM, 0);

}

void SOC_rcmDisableTopPbist(void)
{
    CSL_mss_ctrlRegs *mssCtrl = CSL_MSS_CTRL_getBaseAddress ();
    /* Disable TOP PBIST*/
    CSL_FINS(mssCtrl->MSS_PBIST_KEY_RST, MSS_CTRL_MSS_PBIST_KEY_RST_MSS_PBIST_KEY_RST_PBIST_ST_KEY, 0x0);
}

void SOC_rcmSwitchR5Clock(SOC_RcmR5ClockSource clkSrc, uint32_t divVal)
{
    CSL_mss_toprcmRegs *ptrTOPRCMRegs;

    ptrTOPRCMRegs = CSL_TopRCM_getBaseAddress();

    DebugP_assert(clkSrc != SOC_RcmR5ClockSource_MAX_VALUE);

    CSL_FINS(ptrTOPRCMRegs->MSS_CR5_CLK_SRC_SEL,
            MSS_TOPRCM_MSS_CR5_CLK_SRC_SEL_MSS_CR5_CLK_SRC_SEL_CLKSRCSEL,
            gSocRcmR5ClkSrcValMap[clkSrc]);

    CSL_FINS(ptrTOPRCMRegs->MSS_CR5_DIV_VAL, MSS_TOPRCM_MSS_CR5_DIV_VAL_MSS_CR5_DIV_VAL_CLKDIV, divVal);

    return;
}
