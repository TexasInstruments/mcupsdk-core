/*
 *  Copyright (C) 2021-2023 Texas Instruments Incorporated
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
#include <drivers/soc.h>
#if defined(__ARM_ARCH_7R__)
#ifdef __ARM_ACLE
#include <arm_acle.h>
#endif /* __ARM_ACLE */
#endif

/**
 * @brief
 *  Mapping Array for PLL configuration
 *
 * @details
 *  Mapping Array for PLL configuration: core PLL and peripheral PLL
 */

#define DUALCOREBOOTENABLE_START_BIT_R5FSS0        (4U)
#define DUALCOREBOOTENABLE_END_BIT_R5FSS0          (4U)
#define DUALCORESWITCHDISABLE_START_BIT_R5FSS0     (5U)
#define DUALCORESWITCHDISABLE_END_BIT_R5FSS0       (5U)
#define DUALCOREBOOTENABLE_START_BIT_R5FSS1        (6U)
#define DUALCOREBOOTENABLE_END_BIT_R5FSS1          (6U)
#define DUALCORESWITCHDISABLE_START_BIT_R5FSS1     (7U)
#define DUALCORESWITCHDISABLE_END_BIT_R5FSS1       (7U)
#define DUALCOREDISABLE_START_BIT_R5FSS1           (8U)
#define DUALCOREDISABLE_END_BIT_R5FSS1             (8U)

#define SOC_RCM_UTILS_ARRAYSIZE(x)                  (sizeof(x)/sizeof(x[0]))

const SOC_RcmADPLLJConfig_t gADPLLJConfigTbl[] =
{
    /* CORE_2000_25MHz */
    {
        .Finp = 25U,
        .N = 9U,
        .Fout = 2000U,
        .M2 = 1U,
        .M = 800U,
        .FracM = 0U,
    },
    /* PER_1920_25MHz */
    {
        .Finp = 25U,
        .N = 9U,
        .Fout = 1920U,
        .M2 = 1U,
        .M = 768U,
        .FracM = 0U,
    },
    /* PER_800_25MHz */
    {
        .Finp = 25U,
        .N = 11U,
        .Fout = 800U,
        .M2 = 1U,
        .M = 384U,
        .FracM = 0U,
    },
};

const SOC_RcmXTALInfo gXTALInfo[] =
{
    [RCM_XTAL_FREQID_CLK_25MHZ]      = {.Finp = SOC_RCM_FREQ_HZ2MHZ(25000000U),     .div2flag = false},
};

const uint32_t gPLLFreqId2FOutMap[] =
{
    [RCM_PLL_FOUT_FREQID_CLK_2000MHZ]       = 2000U,
    [RCM_PLL_FOUT_FREQID_CLK_1920MHZ]       = 1920U,
    [RCM_PLL_FOUT_FREQID_CLK_800MHZ]        = 800U,
};

const SOC_RcmClkSrcInfo gPeripheralClkSrcInfoMap[] =
{
    [SOC_RcmPeripheralClockSource_XTALCLK] =
    {
        .pllId = RCM_PLLID_XTALCLK,
        .hsDivOut = RCM_PLLHSDIV_OUT_NONE,
    },
    [SOC_RcmPeripheralClockSource_SYS_CLK] =
    {
        .pllId = RCM_PLLID_CORE,
        .hsDivOut = RCM_PLLHSDIV_OUT_0,
    },
    [SOC_RcmPeripheralClockSource_WUCPUCLK] =
    {
        .pllId = RCM_PLLID_WUCPUCLK,
        .hsDivOut = RCM_PLLHSDIV_OUT_NONE,
    },
    [SOC_RcmPeripheralClockSource_EXT_REFCLK] =
    {
        .pllId = RCM_PLLID_EXTREFCLK,
        .hsDivOut = RCM_PLLHSDIV_OUT_NONE,
    },
    [SOC_RcmPeripheralClockSource_RCCLK10M] =
    {
        .pllId = RCM_PLLID_RCCLK10M,
        .hsDivOut = RCM_PLLHSDIV_OUT_NONE,
    },
    [SOC_RcmPeripheralClockSource_RCCLK32K] =
    {
        .pllId = RCM_PLLID_RCCLK32K,
        .hsDivOut = RCM_PLLHSDIV_OUT_NONE,
    },
    [SOC_RcmPeripheralClockSource_CTPS_GENF0] =
    {
        .pllId = RCM_PLLID_PER,
        .hsDivOut = RCM_PLLHSDIV_OUT_1,
    },
    [SOC_RcmPeripheralClockSource_DPLL_CORE_HSDIV0_CLKOUT0] =
    {
        .pllId = RCM_PLLID_CORE,
        .hsDivOut = RCM_PLLHSDIV_OUT_0,
    },
    [SOC_RcmPeripheralClockSource_DPLL_CORE_HSDIV0_CLKOUT1] =
    {
        .pllId = RCM_PLLID_CORE,
        .hsDivOut = RCM_PLLHSDIV_OUT_1,
    },
    [SOC_RcmPeripheralClockSource_DPLL_CORE_HSDIV0_CLKOUT2] =
    {
        .pllId = RCM_PLLID_CORE,
        .hsDivOut = RCM_PLLHSDIV_OUT_2,
    },
    [SOC_RcmPeripheralClockSource_DPLL_PER_HSDIV0_CLKOUT0] =
    {
        .pllId = RCM_PLLID_PER,
        .hsDivOut = RCM_PLLHSDIV_OUT_0,
    },
    [SOC_RcmPeripheralClockSource_DPLL_PER_HSDIV0_CLKOUT1] =
    {
        .pllId = RCM_PLLID_PER,
        .hsDivOut = RCM_PLLHSDIV_OUT_1,
    },
};


const SOC_RcmClkSrcInfo gCR5ClkSrcInfoMap =
{
    .pllId = RCM_PLLID_CORE,
    .hsDivOut = RCM_PLLHSDIV_OUT_2,
};

const SOC_RcmClkSrcInfo gTraceClkSrcInfoMap =
{
    .pllId = RCM_PLLID_CORE,
    .hsDivOut = RCM_PLLHSDIV_OUT_1,
};

const SOC_RcmClkSrcInfo gCLKOUTClkSrcInfoMap =
{
    .pllId = RCM_PLLID_CORE,
    .hsDivOut = RCM_PLLHSDIV_OUT_1,
};

#define UNSUPPORTED_CLOCK_SOURCE                        (0x888U)


/**
 * @brief
 *  Mapping Array for MCAN
 *
 * @details
 *  Mapping Array between Clock mode and Clock Mode Value for MCAN
 */
static const uint16_t gMcanClkSrcValMap[] =
{
    [SOC_RcmPeripheralClockSource_XTALCLK]                     = 0x666U,
    [SOC_RcmPeripheralClockSource_SYS_CLK]                     = 0x222U,
    [SOC_RcmPeripheralClockSource_WUCPUCLK]                    = 0x000U,
    [SOC_RcmPeripheralClockSource_EXT_REFCLK]                  = 0x111U,
    [SOC_RcmPeripheralClockSource_RCCLK10M]                    = 0x555U,   //And 0x777U
    [SOC_RcmPeripheralClockSource_RCCLK32K]                    = UNSUPPORTED_CLOCK_SOURCE,
    [SOC_RcmPeripheralClockSource_CTPS_GENF0]                  = UNSUPPORTED_CLOCK_SOURCE,
    [SOC_RcmPeripheralClockSource_DPLL_CORE_HSDIV0_CLKOUT0]    = 0x444U,
    [SOC_RcmPeripheralClockSource_DPLL_CORE_HSDIV0_CLKOUT1]    = UNSUPPORTED_CLOCK_SOURCE,
    [SOC_RcmPeripheralClockSource_DPLL_CORE_HSDIV0_CLKOUT2]    = UNSUPPORTED_CLOCK_SOURCE,
    [SOC_RcmPeripheralClockSource_DPLL_PER_HSDIV0_CLKOUT0]     = UNSUPPORTED_CLOCK_SOURCE,
    [SOC_RcmPeripheralClockSource_DPLL_PER_HSDIV0_CLKOUT1]     = 0x333U,
};

/**
 * @brief
 *  Mapping Array for OSPI
 *
 * @details
 *  Mapping Array between Clock mode and Clock Mode Value for OSPI
 */
static const uint16_t gOspiClkSrcValMap[] =
{
    [SOC_RcmPeripheralClockSource_XTALCLK]                     = 0x666U,
    [SOC_RcmPeripheralClockSource_SYS_CLK]                     = 0x222U,
    [SOC_RcmPeripheralClockSource_WUCPUCLK]                    = 0x000U,
    [SOC_RcmPeripheralClockSource_EXT_REFCLK]                  = 0x111U,
    [SOC_RcmPeripheralClockSource_RCCLK10M]                    = 0x555U,   //And 0x777U
    [SOC_RcmPeripheralClockSource_RCCLK32K]                    = UNSUPPORTED_CLOCK_SOURCE,
    [SOC_RcmPeripheralClockSource_CTPS_GENF0]                  = UNSUPPORTED_CLOCK_SOURCE,
    [SOC_RcmPeripheralClockSource_DPLL_CORE_HSDIV0_CLKOUT0]    = 0x444U,
    [SOC_RcmPeripheralClockSource_DPLL_CORE_HSDIV0_CLKOUT1]    = UNSUPPORTED_CLOCK_SOURCE,
    [SOC_RcmPeripheralClockSource_DPLL_CORE_HSDIV0_CLKOUT2]    = UNSUPPORTED_CLOCK_SOURCE,
    [SOC_RcmPeripheralClockSource_DPLL_PER_HSDIV0_CLKOUT0]     = UNSUPPORTED_CLOCK_SOURCE,
    [SOC_RcmPeripheralClockSource_DPLL_PER_HSDIV0_CLKOUT1]     = 0x333U,
};

/**
 * @brief
 *  Mapping Array for RTI
 *
 * @details
 *  Mapping Array between Clock mode and Clock Mode Value for RTI
 */
static const uint16_t gRtiClkSrcValMap[] =
{
    [SOC_RcmPeripheralClockSource_XTALCLK]                     = 0x666U,
    [SOC_RcmPeripheralClockSource_SYS_CLK]                     = 0x222U,
    [SOC_RcmPeripheralClockSource_WUCPUCLK]                    = 0x000U,
    [SOC_RcmPeripheralClockSource_EXT_REFCLK]                  = 0x111U,
    [SOC_RcmPeripheralClockSource_RCCLK10M]                    = 0x555U,
    [SOC_RcmPeripheralClockSource_RCCLK32K]                    = UNSUPPORTED_CLOCK_SOURCE,
    [SOC_RcmPeripheralClockSource_CTPS_GENF0]                  = 0x777U,
    [SOC_RcmPeripheralClockSource_DPLL_CORE_HSDIV0_CLKOUT0]    = UNSUPPORTED_CLOCK_SOURCE,
    [SOC_RcmPeripheralClockSource_DPLL_CORE_HSDIV0_CLKOUT1]    = 0x444U,
    [SOC_RcmPeripheralClockSource_DPLL_CORE_HSDIV0_CLKOUT2]    = UNSUPPORTED_CLOCK_SOURCE,
    [SOC_RcmPeripheralClockSource_DPLL_PER_HSDIV0_CLKOUT0]     = UNSUPPORTED_CLOCK_SOURCE,
    [SOC_RcmPeripheralClockSource_DPLL_PER_HSDIV0_CLKOUT1]     = 0x333U,
};

/**
 * @brief
 *  Mapping Array for WDT
 *
 * @details
 *  Mapping Array between Clock mode and Clock Mode Value for WDT
 */
static const uint16_t gWdtClkSrcValMap[] =
{
    [SOC_RcmPeripheralClockSource_XTALCLK]                     = 0x666U,
    [SOC_RcmPeripheralClockSource_SYS_CLK]                     = 0x222U,
    [SOC_RcmPeripheralClockSource_WUCPUCLK]                    = 0x000U,
    [SOC_RcmPeripheralClockSource_EXT_REFCLK]                  = UNSUPPORTED_CLOCK_SOURCE,
    [SOC_RcmPeripheralClockSource_RCCLK10M]                    = 0x111U, //And 0x555U
    [SOC_RcmPeripheralClockSource_RCCLK32K]                    = 0x777U,
    [SOC_RcmPeripheralClockSource_CTPS_GENF0]                  = UNSUPPORTED_CLOCK_SOURCE,
    [SOC_RcmPeripheralClockSource_DPLL_CORE_HSDIV0_CLKOUT0]    = UNSUPPORTED_CLOCK_SOURCE,
    [SOC_RcmPeripheralClockSource_DPLL_CORE_HSDIV0_CLKOUT1]    = 0x444U,
    [SOC_RcmPeripheralClockSource_DPLL_CORE_HSDIV0_CLKOUT2]    = UNSUPPORTED_CLOCK_SOURCE,
    [SOC_RcmPeripheralClockSource_DPLL_PER_HSDIV0_CLKOUT0]     = UNSUPPORTED_CLOCK_SOURCE,
    [SOC_RcmPeripheralClockSource_DPLL_PER_HSDIV0_CLKOUT1]     = 0x333U,
};

/**
 * @brief
 *  Mapping Array for McSPI
 *
 * @details
 *  Mapping Array between Clock mode and Clock Mode Value for McSPI
 */
static const uint16_t gMcSpiClkSrcValMap[] =
{
    [SOC_RcmPeripheralClockSource_XTALCLK]                     = 0x666U,
    [SOC_RcmPeripheralClockSource_SYS_CLK]                     = 0x222U,
    [SOC_RcmPeripheralClockSource_WUCPUCLK]                    = 0x000U,
    [SOC_RcmPeripheralClockSource_EXT_REFCLK]                  = 0x111U,
    [SOC_RcmPeripheralClockSource_RCCLK10M]                    = 0x555U, //And 0x777U
    [SOC_RcmPeripheralClockSource_RCCLK32K]                    = UNSUPPORTED_CLOCK_SOURCE,
    [SOC_RcmPeripheralClockSource_CTPS_GENF0]                  = UNSUPPORTED_CLOCK_SOURCE,
    [SOC_RcmPeripheralClockSource_DPLL_CORE_HSDIV0_CLKOUT0]    = 0x444U,
    [SOC_RcmPeripheralClockSource_DPLL_CORE_HSDIV0_CLKOUT1]    = UNSUPPORTED_CLOCK_SOURCE,
    [SOC_RcmPeripheralClockSource_DPLL_CORE_HSDIV0_CLKOUT2]    = UNSUPPORTED_CLOCK_SOURCE,
    [SOC_RcmPeripheralClockSource_DPLL_PER_HSDIV0_CLKOUT0]     = UNSUPPORTED_CLOCK_SOURCE,
    [SOC_RcmPeripheralClockSource_DPLL_PER_HSDIV0_CLKOUT1]     = 0x333U,
};

/**
 * @brief
 *  Mapping Array for MMC
 *
 * @details
 *  Mapping Array between Clock mode and Clock Mode Value for MMC
 */
static const uint16_t gMmcClkSrcValMap[] =
{
    [SOC_RcmPeripheralClockSource_XTALCLK]                     = 0x666U,
    [SOC_RcmPeripheralClockSource_SYS_CLK]                     = 0x222U,
    [SOC_RcmPeripheralClockSource_WUCPUCLK]                    = 0x000U,
    [SOC_RcmPeripheralClockSource_EXT_REFCLK]                  = 0x111U,
    [SOC_RcmPeripheralClockSource_RCCLK10M]                    = 0x555U, //Also 0x777U
    [SOC_RcmPeripheralClockSource_RCCLK32K]                    = UNSUPPORTED_CLOCK_SOURCE,
    [SOC_RcmPeripheralClockSource_CTPS_GENF0]                  = UNSUPPORTED_CLOCK_SOURCE,
    [SOC_RcmPeripheralClockSource_DPLL_CORE_HSDIV0_CLKOUT0]    = 0x444U,
    [SOC_RcmPeripheralClockSource_DPLL_CORE_HSDIV0_CLKOUT1]    = UNSUPPORTED_CLOCK_SOURCE,
    [SOC_RcmPeripheralClockSource_DPLL_CORE_HSDIV0_CLKOUT2]    = UNSUPPORTED_CLOCK_SOURCE,
    [SOC_RcmPeripheralClockSource_DPLL_PER_HSDIV0_CLKOUT0]     = UNSUPPORTED_CLOCK_SOURCE,
    [SOC_RcmPeripheralClockSource_DPLL_PER_HSDIV0_CLKOUT1]     = 0x333U,
};

/**
 * @brief
 *  Mapping Array for ICSSM UART
 *
 * @details
 *  Mapping Array between Clock mode and Clock Mode Value for ICSSM UART
 */
static const uint16_t gIcssmUartClkSrcValMap[] =
{
    [SOC_RcmPeripheralClockSource_XTALCLK]                     = 0x666U,
    [SOC_RcmPeripheralClockSource_SYS_CLK]                     = 0x222U,
    [SOC_RcmPeripheralClockSource_WUCPUCLK]                    = 0x000U,
    [SOC_RcmPeripheralClockSource_EXT_REFCLK]                  = 0x111U,
    [SOC_RcmPeripheralClockSource_RCCLK10M]                    = 0x555U,
    [SOC_RcmPeripheralClockSource_RCCLK32K]                    = UNSUPPORTED_CLOCK_SOURCE,
    [SOC_RcmPeripheralClockSource_CTPS_GENF0]                  = UNSUPPORTED_CLOCK_SOURCE,
    [SOC_RcmPeripheralClockSource_DPLL_CORE_HSDIV0_CLKOUT0]    = 0x444U,
    [SOC_RcmPeripheralClockSource_DPLL_CORE_HSDIV0_CLKOUT1]    = UNSUPPORTED_CLOCK_SOURCE,
    [SOC_RcmPeripheralClockSource_DPLL_CORE_HSDIV0_CLKOUT2]    = UNSUPPORTED_CLOCK_SOURCE,
    [SOC_RcmPeripheralClockSource_DPLL_PER_HSDIV0_CLKOUT0]     = 0x777U,
    [SOC_RcmPeripheralClockSource_DPLL_PER_HSDIV0_CLKOUT1]     = 0x333U,
};

/**
 * @brief
 *  Mapping Array for CPTS
 *
 * @details
 *  Mapping Array between Clock mode and Clock Mode Value for CPTS
 */
static const uint16_t gCptsClkSrcValMap[] =
{
    [SOC_RcmPeripheralClockSource_XTALCLK]                     = 0x666U,
    [SOC_RcmPeripheralClockSource_SYS_CLK]                     = 0x222U,
    [SOC_RcmPeripheralClockSource_WUCPUCLK]                    = 0x000U,
    [SOC_RcmPeripheralClockSource_EXT_REFCLK]                  = 0x111U,
    [SOC_RcmPeripheralClockSource_RCCLK10M]                    = 0x555U, //Also 0x777U
    [SOC_RcmPeripheralClockSource_RCCLK32K]                    = UNSUPPORTED_CLOCK_SOURCE,
    [SOC_RcmPeripheralClockSource_CTPS_GENF0]                  = UNSUPPORTED_CLOCK_SOURCE,
    [SOC_RcmPeripheralClockSource_DPLL_CORE_HSDIV0_CLKOUT0]    = 0x444U,
    [SOC_RcmPeripheralClockSource_DPLL_CORE_HSDIV0_CLKOUT1]    = 0x333U,
    [SOC_RcmPeripheralClockSource_DPLL_CORE_HSDIV0_CLKOUT2]    = UNSUPPORTED_CLOCK_SOURCE,
    [SOC_RcmPeripheralClockSource_DPLL_PER_HSDIV0_CLKOUT0]     = UNSUPPORTED_CLOCK_SOURCE,
    [SOC_RcmPeripheralClockSource_DPLL_PER_HSDIV0_CLKOUT1]     = UNSUPPORTED_CLOCK_SOURCE,
};

/**
 * @brief
 *  Mapping Array for GPMC
 *
 * @details
 *  Mapping Array between Clock mode and Clock Mode Value for GPMC
 */
// static uint16_t gGpmcClkSrcValMap[] =
// {
//     [SOC_RcmPeripheralClockSource_XTALCLK]                     = 0x666U,
//     [SOC_RcmPeripheralClockSource_SYS_CLK]                     = 0x222U,
//     [SOC_RcmPeripheralClockSource_WUCPUCLK]                    = 0x000U,
//     [SOC_RcmPeripheralClockSource_EXT_REFCLK]                  = 0x111U,
//     [SOC_RcmPeripheralClockSource_RCCLK10M]                    = 0x555U, //Also 0x777U
//     [SOC_RcmPeripheralClockSource_RCCLK32K]                    = UNSUPPORTED_CLOCK_SOURCE,
//     [SOC_RcmPeripheralClockSource_CTPS_GENF0]                  = UNSUPPORTED_CLOCK_SOURCE,
//     [SOC_RcmPeripheralClockSource_DPLL_CORE_HSDIV0_CLKOUT0]    = 0x444U,
//     [SOC_RcmPeripheralClockSource_DPLL_CORE_HSDIV0_CLKOUT1]    = UNSUPPORTED_CLOCK_SOURCE,
//     [SOC_RcmPeripheralClockSource_DPLL_CORE_HSDIV0_CLKOUT2]    = UNSUPPORTED_CLOCK_SOURCE,
//     [SOC_RcmPeripheralClockSource_DPLL_PER_HSDIV0_CLKOUT0]     = UNSUPPORTED_CLOCK_SOURCE,
//     [SOC_RcmPeripheralClockSource_DPLL_PER_HSDIV0_CLKOUT1]     = 0x333U,
// };

/**
 * @brief
 *  Mapping Array for ControlSS PLL
 *
 * @details
 *  Mapping Array between Clock mode and Clock Mode Value for ControlSS PLL
 */
static const uint16_t gControlssPllClkSrcValMap[] =
{
    [SOC_RcmPeripheralClockSource_XTALCLK]                     = 0x666U,
    [SOC_RcmPeripheralClockSource_SYS_CLK]                     = UNSUPPORTED_CLOCK_SOURCE,
    [SOC_RcmPeripheralClockSource_WUCPUCLK]                    = 0x000U,
    [SOC_RcmPeripheralClockSource_EXT_REFCLK]                  = 0x111U,
    [SOC_RcmPeripheralClockSource_RCCLK10M]                    = 0x555U, //Also 0x777U
    [SOC_RcmPeripheralClockSource_RCCLK32K]                    = UNSUPPORTED_CLOCK_SOURCE,
    [SOC_RcmPeripheralClockSource_CTPS_GENF0]                  = UNSUPPORTED_CLOCK_SOURCE,
    [SOC_RcmPeripheralClockSource_DPLL_CORE_HSDIV0_CLKOUT0]    = 0x444U,
    [SOC_RcmPeripheralClockSource_DPLL_CORE_HSDIV0_CLKOUT1]    = UNSUPPORTED_CLOCK_SOURCE,
    [SOC_RcmPeripheralClockSource_DPLL_CORE_HSDIV0_CLKOUT2]    = 0x222U,
    [SOC_RcmPeripheralClockSource_DPLL_PER_HSDIV0_CLKOUT0]     = UNSUPPORTED_CLOCK_SOURCE,
    [SOC_RcmPeripheralClockSource_DPLL_PER_HSDIV0_CLKOUT1]     = 0x333U,
};

/**
 * @brief
 *  Mapping Array for I2C
 *
 * @details
 *  Mapping Array between Clock mode and Clock Mode Value for I2C
 */
static const uint16_t gI2cClkSrcValMap[] =
{
    [SOC_RcmPeripheralClockSource_XTALCLK]                     = 0x666U,
    [SOC_RcmPeripheralClockSource_SYS_CLK]                     = 0x222U,
    [SOC_RcmPeripheralClockSource_WUCPUCLK]                    = 0x000U,
    [SOC_RcmPeripheralClockSource_EXT_REFCLK]                  = 0x111U,
    [SOC_RcmPeripheralClockSource_RCCLK10M]                    = 0x555U, //Also 0x777U
    [SOC_RcmPeripheralClockSource_RCCLK32K]                    = UNSUPPORTED_CLOCK_SOURCE,
    [SOC_RcmPeripheralClockSource_CTPS_GENF0]                  = UNSUPPORTED_CLOCK_SOURCE,
    [SOC_RcmPeripheralClockSource_DPLL_CORE_HSDIV0_CLKOUT0]    = 0x444U,
    [SOC_RcmPeripheralClockSource_DPLL_CORE_HSDIV0_CLKOUT1]    = UNSUPPORTED_CLOCK_SOURCE,
    [SOC_RcmPeripheralClockSource_DPLL_CORE_HSDIV0_CLKOUT2]    = UNSUPPORTED_CLOCK_SOURCE,
    [SOC_RcmPeripheralClockSource_DPLL_PER_HSDIV0_CLKOUT0]     = UNSUPPORTED_CLOCK_SOURCE,
    [SOC_RcmPeripheralClockSource_DPLL_PER_HSDIV0_CLKOUT1]     = 0x333U,
};

/**
 * @brief
 *  Mapping Array for LIN
 *
 * @details
 *  Mapping Array between Clock mode and Clock Mode Value for LIN
 */
static const uint16_t gLinUartClkSrcValMap[] =
{
    [SOC_RcmPeripheralClockSource_XTALCLK]                     = 0x666U,
    [SOC_RcmPeripheralClockSource_SYS_CLK]                     = 0x222U,
    [SOC_RcmPeripheralClockSource_WUCPUCLK]                    = 0x000U,
    [SOC_RcmPeripheralClockSource_EXT_REFCLK]                  = 0x111U,
    [SOC_RcmPeripheralClockSource_RCCLK10M]                    = 0x555U,
    [SOC_RcmPeripheralClockSource_RCCLK32K]                    = UNSUPPORTED_CLOCK_SOURCE,
    [SOC_RcmPeripheralClockSource_CTPS_GENF0]                  = UNSUPPORTED_CLOCK_SOURCE,
    [SOC_RcmPeripheralClockSource_DPLL_CORE_HSDIV0_CLKOUT0]    = 0x444U,
    [SOC_RcmPeripheralClockSource_DPLL_CORE_HSDIV0_CLKOUT1]    = UNSUPPORTED_CLOCK_SOURCE,
    [SOC_RcmPeripheralClockSource_DPLL_CORE_HSDIV0_CLKOUT2]    = UNSUPPORTED_CLOCK_SOURCE,
    [SOC_RcmPeripheralClockSource_DPLL_PER_HSDIV0_CLKOUT0]     = 0x777U,
    [SOC_RcmPeripheralClockSource_DPLL_PER_HSDIV0_CLKOUT1]     = 0x333U,
};

/**
 * @brief
 *  Mapping Array for R5F, SYSCLK
 *
 * @details
 *  Mapping Array between Clock mode and Clock Mode Value for R5F and SYSCLK
 */
static const uint16_t gR5SysClkSrcValMap[] =
{
    [SOC_RcmPeripheralClockSource_XTALCLK]                     = UNSUPPORTED_CLOCK_SOURCE,
    [SOC_RcmPeripheralClockSource_SYS_CLK]                     = UNSUPPORTED_CLOCK_SOURCE,
    [SOC_RcmPeripheralClockSource_WUCPUCLK]                    = 0x000U,
    [SOC_RcmPeripheralClockSource_EXT_REFCLK]                  = 0x111U,
    [SOC_RcmPeripheralClockSource_RCCLK10M]                    = 0x333U, //Also 0x444U, 0x555U, 0x777U
    [SOC_RcmPeripheralClockSource_RCCLK32K]                    = UNSUPPORTED_CLOCK_SOURCE,
    [SOC_RcmPeripheralClockSource_CTPS_GENF0]                  = UNSUPPORTED_CLOCK_SOURCE,
    [SOC_RcmPeripheralClockSource_DPLL_CORE_HSDIV0_CLKOUT0]    = 0x222U,
    [SOC_RcmPeripheralClockSource_DPLL_CORE_HSDIV0_CLKOUT1]    = UNSUPPORTED_CLOCK_SOURCE,
    [SOC_RcmPeripheralClockSource_DPLL_CORE_HSDIV0_CLKOUT2]    = UNSUPPORTED_CLOCK_SOURCE,
    [SOC_RcmPeripheralClockSource_DPLL_PER_HSDIV0_CLKOUT0]     = UNSUPPORTED_CLOCK_SOURCE,
    [SOC_RcmPeripheralClockSource_DPLL_PER_HSDIV0_CLKOUT1]     = UNSUPPORTED_CLOCK_SOURCE,
};

/**
 * @brief
 *  Mapping Array for Trace
 *
 * @details
 *  Mapping Array between Clock mode and Clock Mode Value for Trace
 */
static const uint16_t gTraceClkSrcValMap[] =
{
    [SOC_RcmPeripheralClockSource_XTALCLK]                     = 0x666U,
    [SOC_RcmPeripheralClockSource_SYS_CLK]                     = UNSUPPORTED_CLOCK_SOURCE,
    [SOC_RcmPeripheralClockSource_WUCPUCLK]                    = 0x000U,
    [SOC_RcmPeripheralClockSource_EXT_REFCLK]                  = UNSUPPORTED_CLOCK_SOURCE,
    [SOC_RcmPeripheralClockSource_RCCLK10M]                    = 0x555U, //Also 0x777U
    [SOC_RcmPeripheralClockSource_RCCLK32K]                    = UNSUPPORTED_CLOCK_SOURCE,
    [SOC_RcmPeripheralClockSource_CTPS_GENF0]                  = UNSUPPORTED_CLOCK_SOURCE,
    [SOC_RcmPeripheralClockSource_DPLL_CORE_HSDIV0_CLKOUT0]    = 0x111U,
    [SOC_RcmPeripheralClockSource_DPLL_CORE_HSDIV0_CLKOUT1]    = 0x222U,
    [SOC_RcmPeripheralClockSource_DPLL_CORE_HSDIV0_CLKOUT2]    = UNSUPPORTED_CLOCK_SOURCE,
    [SOC_RcmPeripheralClockSource_DPLL_PER_HSDIV0_CLKOUT0]     = 0x333U,
    [SOC_RcmPeripheralClockSource_DPLL_PER_HSDIV0_CLKOUT1]     = 0x444U,
};

/**
 * @brief
 *  Mapping Array for CLKOUT
 *
 * @details
 *  Mapping Array between Clock mode and Clock Mode Value for CLKOUT
 */
static const uint16_t gClkoutClkSrcValMap[] =
{
    [SOC_RcmPeripheralClockSource_XTALCLK]                     = 0x666U,
    [SOC_RcmPeripheralClockSource_SYS_CLK]                     = UNSUPPORTED_CLOCK_SOURCE,
    [SOC_RcmPeripheralClockSource_WUCPUCLK]                    = 0x000U,
    [SOC_RcmPeripheralClockSource_EXT_REFCLK]                  = UNSUPPORTED_CLOCK_SOURCE,
    [SOC_RcmPeripheralClockSource_RCCLK10M]                    = 0x555U,
    [SOC_RcmPeripheralClockSource_RCCLK32K]                    = UNSUPPORTED_CLOCK_SOURCE,
    [SOC_RcmPeripheralClockSource_CTPS_GENF0]                  = 0x777U,
    [SOC_RcmPeripheralClockSource_DPLL_CORE_HSDIV0_CLKOUT0]    = 0x111U,
    [SOC_RcmPeripheralClockSource_DPLL_CORE_HSDIV0_CLKOUT1]    = 0x222U,
    [SOC_RcmPeripheralClockSource_DPLL_CORE_HSDIV0_CLKOUT2]    = UNSUPPORTED_CLOCK_SOURCE,
    [SOC_RcmPeripheralClockSource_DPLL_PER_HSDIV0_CLKOUT0]     = 0x333U,
    [SOC_RcmPeripheralClockSource_DPLL_PER_HSDIV0_CLKOUT1]     = 0x444U,
};

/**
 * @brief
 *  Mapping Array for Reset Cause
 *
 * @details
 *  Mapping Array between Reset Cause Bit and Reset Cause
 */
static const SOC_RcmResetCause gResetBitToResetCause[12U] =
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
    SOC_RcmResetCause_POR_RST_CTRL0,
    SOC_RcmResetCause_RST_CAUSE_UNKNOWN,
};

/**************************************************************************
 ************************ Local Data Structures ***************************
 **************************************************************************/

/**************************************************************************
 **************************** Local Functions *****************************
 **************************************************************************/

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


/**
 *  @b Description
 *  @n
 *      The function is used to get the base address of the Top-level CTRL
 *      configuration registers.
 *
 *  @retval
 *      Base Address of the Top-level CTRL Register
 */
static CSL_top_ctrlRegs* SOC_rcmGetBaseAddressTOPCTRL (void)
{
    return (CSL_top_ctrlRegs*)CSL_TOP_CTRL_U_BASE;
}

/**
 *  @b Description
 *  @n
 *      The function is used to get the base address of the MSS CTRL
 *      configuration registers.
 *
 *  @retval
 *      Base Address of the MSS CTRL Register
 */
static CSL_mss_ctrlRegs* SOC_rcmGetBaseAddressMSSCTRL (void)
{
    return (CSL_mss_ctrlRegs*) CSL_MSS_CTRL_U_BASE;
}
/**
 *  @b Description
 *  @n
 *      The function is used to get the base address of the ControlSS CTRL
 *      configuration registers.
 *
 *  @retval
 *      Base Address of the ControlSS CTRL Register
 */
CSL_controlss_ctrlRegs* SOC_rcmGetBaseAddressCONTROLSSCTRL (void)
{
    return (CSL_controlss_ctrlRegs*)CSL_CONTROLSS_CTRL_U_BASE;
}

/**
 *  @b Description
 *  @n
 *      The function is used to get the base address of the Top-level RCM
 *      configuration registers.
 *
 *  @retval
 *      Base Address of the Top-level RCM Register
 */
static CSL_top_rcmRegs* SOC_rcmGetBaseAddressTOPRCM (void)
{
    return (CSL_top_rcmRegs*) CSL_TOP_RCM_U_BASE;
}

/**
 *  @b Description
 *  @n
 *      The function is used to get the base address of the MSS RCM
 *      configuration registers.
 *
 *  @retval
 *      Base Address of the MSS RCM Register
 */
static CSL_mss_rcmRegs* SOC_rcmGetBaseAddressMSSRCM (void)
{
    return (CSL_mss_rcmRegs*) CSL_MSS_RCM_U_BASE;
}

static uint8_t CSL_TopCtrl_dualCoreBootEnableEfuse (const CSL_top_ctrlRegs * ptrTopCtrlRegs, uint32_t cpuId)
{
    uint8_t retVal = 0xFFU;
    if ((cpuId == CSL_CORE_ID_R5FSS0_0) || (cpuId == CSL_CORE_ID_R5FSS0_1))
    {
        retVal = SOC_rcmExtract8 (ptrTopCtrlRegs->EFUSE1_ROW_12, \
                              DUALCOREBOOTENABLE_END_BIT_R5FSS0, \
                              DUALCOREBOOTENABLE_START_BIT_R5FSS0);
    }
    else if ((cpuId == CSL_CORE_ID_R5FSS1_0) || (cpuId == CSL_CORE_ID_R5FSS1_1))
    {
        retVal = SOC_rcmExtract8 (ptrTopCtrlRegs->EFUSE1_ROW_12, \
                              DUALCOREBOOTENABLE_END_BIT_R5FSS1, \
                              DUALCOREBOOTENABLE_START_BIT_R5FSS1);
    }
    else
    {
        DebugP_logError("CPU Dual Core Boot Enable failed\r\n");
    }

    return retVal;
}

static uint8_t CSL_TopCtrl_dualCoreSwitchDisableEfuse (const CSL_top_ctrlRegs * ptrTopCtrlRegs, uint32_t cpuId)
{
    uint8_t retVal = 0xFFU;
    if ((cpuId == CSL_CORE_ID_R5FSS0_0) || (cpuId == CSL_CORE_ID_R5FSS0_1))
    {
        retVal = SOC_rcmExtract8 (ptrTopCtrlRegs->EFUSE1_ROW_12, \
                              DUALCORESWITCHDISABLE_END_BIT_R5FSS0, \
                              DUALCORESWITCHDISABLE_START_BIT_R5FSS0);
    }
    else if ((cpuId == CSL_CORE_ID_R5FSS1_0) || (cpuId == CSL_CORE_ID_R5FSS1_1))
    {
        retVal = SOC_rcmExtract8 (ptrTopCtrlRegs->EFUSE1_ROW_12, \
                              DUALCORESWITCHDISABLE_END_BIT_R5FSS1, \
                              DUALCORESWITCHDISABLE_START_BIT_R5FSS1);
        retVal |= SOC_rcmExtract8 (ptrTopCtrlRegs->EFUSE1_ROW_12, \
                              DUALCOREDISABLE_END_BIT_R5FSS1, \
                              DUALCOREDISABLE_START_BIT_R5FSS1);
    }
    else
    {
        DebugP_logError("CPU Dual Core Switch Disable failed\r\n");
    }

    return retVal;
}

static uint32_t SBL_rcmIsDualCoreSwitchSupported(uint32_t cpuId)
{
    CSL_top_ctrlRegs* ptrTopCtrlRegs;
    uint8_t dualCoreBootEnable, dualCoreSwitchDisable;
    uint32_t retVal = FALSE;

    ptrTopCtrlRegs = SOC_rcmGetBaseAddressTOPCTRL ();
    dualCoreBootEnable = CSL_TopCtrl_dualCoreBootEnableEfuse(ptrTopCtrlRegs, cpuId);
    dualCoreSwitchDisable = CSL_TopCtrl_dualCoreSwitchDisableEfuse(ptrTopCtrlRegs, cpuId);
    if ((dualCoreBootEnable == 0) && (dualCoreSwitchDisable == 0))
    {
        retVal = TRUE;
    }
    return retVal;
}

/**************************************************************************
 ************************** RCM Functions *****************************
 **************************************************************************/


/**
 *  @b Description
 *  @n
 *      This API returns Clk Src Selection register, Clk  Divider Register for
 *      specified peripheral. It also returns the Clk Src Selection value for a
 *      specified clock
 *
 *  @param[in]  periphId
 *      Peripheral Id
 *  @param[in]  clkSource
 *      Clock Source
 *  @param[out]  clkSrcVal
 *      Value to be programmed corresponding to the ClkSource
 *  @param[out]  clkSrcReg
 *      Register Address for programming Clock Source Selection
 *  @param[in]  clkdDivReg
 *      PRegister Address for programming Clock Divider Selection
 *
 *  @retval     None
 */
static void SOC_rcmGetClkSrcAndDivReg (SOC_RcmPeripheralId periphId,
                                SOC_RcmPeripheralClockSource clkSource,
                                volatile uint16_t *clkSrcVal,
                                volatile uint32_t **clkSrcReg,
                                volatile uint32_t **clkdDivReg)
{
    CSL_mss_rcmRegs *ptrMSSRCMRegs;

    ptrMSSRCMRegs = SOC_rcmGetBaseAddressMSSRCM ();

    switch (periphId)
    {
        case SOC_RcmPeripheralId_MCAN0:
        {
            *clkSrcReg  = &(ptrMSSRCMRegs->MCAN0_CLK_SRC_SEL);
            *clkdDivReg = &(ptrMSSRCMRegs->MCAN0_CLK_DIV_VAL);
            *clkSrcVal = gMcanClkSrcValMap[clkSource];
            break;
        }
        case SOC_RcmPeripheralId_MCAN1:
        {
            *clkSrcReg  = &(ptrMSSRCMRegs->MCAN1_CLK_SRC_SEL);
            *clkdDivReg = &(ptrMSSRCMRegs->MCAN1_CLK_DIV_VAL);
            *clkSrcVal = gMcanClkSrcValMap[clkSource];
            break;
        }
        case SOC_RcmPeripheralId_MCAN2:
        {
            *clkSrcReg  = &(ptrMSSRCMRegs->MCAN2_CLK_SRC_SEL);
            *clkdDivReg = &(ptrMSSRCMRegs->MCAN2_CLK_DIV_VAL);
            *clkSrcVal = gMcanClkSrcValMap[clkSource];
            break;
        }
        case SOC_RcmPeripheralId_MCAN3:
        {
            *clkSrcReg  = &(ptrMSSRCMRegs->MCAN3_CLK_SRC_SEL);
            *clkdDivReg = &(ptrMSSRCMRegs->MCAN3_CLK_DIV_VAL);
            *clkSrcVal = gMcanClkSrcValMap[clkSource];
            break;
        }
        case SOC_RcmPeripheralId_OSPI0:
        {
            *clkSrcReg  = &(ptrMSSRCMRegs->OSPI0_CLK_SRC_SEL);
            *clkdDivReg = &(ptrMSSRCMRegs->OSPI0_CLK_DIV_VAL);
            *clkSrcVal = gOspiClkSrcValMap[clkSource];
            break;
        }
        case SOC_RcmPeripheralId_RTI0:
        {
            *clkSrcReg  = &(ptrMSSRCMRegs->RTI0_CLK_SRC_SEL);
            *clkdDivReg = &(ptrMSSRCMRegs->RTI0_CLK_DIV_VAL);
            *clkSrcVal = gRtiClkSrcValMap[clkSource];
            break;
        }
        case SOC_RcmPeripheralId_RTI1:
        {
            *clkSrcReg  = &(ptrMSSRCMRegs->RTI1_CLK_SRC_SEL);
            *clkdDivReg = &(ptrMSSRCMRegs->RTI1_CLK_DIV_VAL);
            *clkSrcVal = gRtiClkSrcValMap[clkSource];
            break;
        }
        case SOC_RcmPeripheralId_RTI2:
        {
            *clkSrcReg  = &(ptrMSSRCMRegs->RTI2_CLK_SRC_SEL);
            *clkdDivReg = &(ptrMSSRCMRegs->RTI2_CLK_DIV_VAL);
            *clkSrcVal = gRtiClkSrcValMap[clkSource];
            break;
        }
        case SOC_RcmPeripheralId_RTI3:
        {
            *clkSrcReg  = &(ptrMSSRCMRegs->RTI3_CLK_SRC_SEL);
            *clkdDivReg = &(ptrMSSRCMRegs->RTI3_CLK_DIV_VAL);
            *clkSrcVal = gRtiClkSrcValMap[clkSource];
            break;
        }
        case SOC_RcmPeripheralId_WDT0:
        {
            *clkSrcReg  = &(ptrMSSRCMRegs->WDT0_CLK_SRC_SEL);
            *clkdDivReg = &(ptrMSSRCMRegs->WDT0_CLK_DIV_VAL);
            *clkSrcVal = gWdtClkSrcValMap[clkSource];
            break;
        }
        case SOC_RcmPeripheralId_WDT1:
        {
            *clkSrcReg  = &(ptrMSSRCMRegs->WDT1_CLK_SRC_SEL);
            *clkdDivReg = &(ptrMSSRCMRegs->WDT1_CLK_DIV_VAL);
            *clkSrcVal = gWdtClkSrcValMap[clkSource];
            break;
        }
        case SOC_RcmPeripheralId_WDT2:
        {
            *clkSrcReg  = &(ptrMSSRCMRegs->WDT2_CLK_SRC_SEL);
            *clkdDivReg = &(ptrMSSRCMRegs->WDT2_CLK_DIV_VAL);
            *clkSrcVal = gWdtClkSrcValMap[clkSource];
            break;
        }
        case SOC_RcmPeripheralId_WDT3:
        {
            *clkSrcReg  = &(ptrMSSRCMRegs->WDT3_CLK_SRC_SEL);
            *clkdDivReg = &(ptrMSSRCMRegs->WDT3_CLK_DIV_VAL);
            *clkSrcVal = gWdtClkSrcValMap[clkSource];
            break;
        }
        case SOC_RcmPeripheralId_MCSPI0:
        {
            *clkSrcReg  = &(ptrMSSRCMRegs->MCSPI0_CLK_SRC_SEL);
            *clkdDivReg = &(ptrMSSRCMRegs->MCSPI0_CLK_DIV_VAL);
            *clkSrcVal = gMcSpiClkSrcValMap[clkSource];
            break;
        }
        case SOC_RcmPeripheralId_MCSPI1:
        {
            *clkSrcReg  = &(ptrMSSRCMRegs->MCSPI1_CLK_SRC_SEL);
            *clkdDivReg = &(ptrMSSRCMRegs->MCSPI1_CLK_DIV_VAL);
            *clkSrcVal = gMcSpiClkSrcValMap[clkSource];
            break;
        }
        case SOC_RcmPeripheralId_MCSPI2:
        {
            *clkSrcReg  = &(ptrMSSRCMRegs->MCSPI2_CLK_SRC_SEL);
            *clkdDivReg = &(ptrMSSRCMRegs->MCSPI2_CLK_DIV_VAL);
            *clkSrcVal = gMcSpiClkSrcValMap[clkSource];
            break;
        }
        case SOC_RcmPeripheralId_MCSPI3:
        {
            *clkSrcReg  = &(ptrMSSRCMRegs->MCSPI3_CLK_SRC_SEL);
            *clkdDivReg = &(ptrMSSRCMRegs->MCSPI3_CLK_DIV_VAL);
            *clkSrcVal = gMcSpiClkSrcValMap[clkSource];
            break;
        }
        case SOC_RcmPeripheralId_MCSPI4:
        {
            *clkSrcReg  = &(ptrMSSRCMRegs->MCSPI4_CLK_SRC_SEL);
            *clkdDivReg = &(ptrMSSRCMRegs->MCSPI4_CLK_DIV_VAL);
            *clkSrcVal = gMcSpiClkSrcValMap[clkSource];
            break;
        }
        case SOC_RcmPeripheralId_MCSPI5:
        {
            *clkSrcReg  = &(ptrMSSRCMRegs->MCSPI5_CLK_SRC_SEL);
            *clkdDivReg = &(ptrMSSRCMRegs->MCSPI5_CLK_DIV_VAL);
            *clkSrcVal = gMcSpiClkSrcValMap[clkSource];
            break;
        }
        case SOC_RcmPeripheralId_MCSPI6:
        {
            *clkSrcReg  = &(ptrMSSRCMRegs->MCSPI6_CLK_SRC_SEL);
            *clkdDivReg = &(ptrMSSRCMRegs->MCSPI6_CLK_DIV_VAL);
            *clkSrcVal = gMcSpiClkSrcValMap[clkSource];
            break;
        }
        case SOC_RcmPeripheralId_MCSPI7:
        {
            *clkSrcReg  = &(ptrMSSRCMRegs->MCSPI7_CLK_SRC_SEL);
            *clkdDivReg = &(ptrMSSRCMRegs->MCSPI7_CLK_DIV_VAL);
            *clkSrcVal = gMcSpiClkSrcValMap[clkSource];
            break;
        }
        case SOC_RcmPeripheralId_MMC0:
        {
            *clkSrcReg  = &(ptrMSSRCMRegs->MMC0_CLK_SRC_SEL);
            *clkdDivReg = &(ptrMSSRCMRegs->MMC0_CLK_DIV_VAL);
            *clkSrcVal = gMmcClkSrcValMap[clkSource];
            break;
        }
        case SOC_RcmPeripheralId_ICSSM0_UART0:
        {
            *clkSrcReg  = &(ptrMSSRCMRegs->ICSSM0_UART0_CLK_SRC_SEL);
            *clkdDivReg = &(ptrMSSRCMRegs->ICSSM0_UART_CLK_DIV_VAL);
            *clkSrcVal = gIcssmUartClkSrcValMap[clkSource];
            break;
        }
        case SOC_RcmPeripheralId_CPTS:
        {
            *clkSrcReg  = &(ptrMSSRCMRegs->CPTS_CLK_SRC_SEL);
            *clkdDivReg = &(ptrMSSRCMRegs->CPTS_CLK_DIV_VAL);
            *clkSrcVal = gCptsClkSrcValMap[clkSource];
            break;
        }
        case SOC_RcmPeripheralId_CONTROLSS_PLL:
        {
            *clkSrcReg  = &(ptrMSSRCMRegs->CONTROLSS_PLL_CLK_SRC_SEL);
            *clkdDivReg = &(ptrMSSRCMRegs->CONTROLSS_PLL_CLK_DIV_VAL);
            *clkSrcVal = gControlssPllClkSrcValMap[clkSource];
            break;
        }
        case SOC_RcmPeripheralId_I2C:
        {
            *clkSrcReg  = &(ptrMSSRCMRegs->I2C_CLK_SRC_SEL);
            *clkdDivReg = &(ptrMSSRCMRegs->I2C_CLK_DIV_VAL);
            *clkSrcVal = gI2cClkSrcValMap[clkSource];
            break;
        }
        case SOC_RcmPeripheralId_LIN0_UART0:
        {
            *clkSrcReg  = &(ptrMSSRCMRegs->LIN0_UART0_CLK_SRC_SEL);
            *clkdDivReg = &(ptrMSSRCMRegs->LIN0_UART0_CLK_DIV_VAL);
            *clkSrcVal = gLinUartClkSrcValMap[clkSource];
            break;
        }
        case SOC_RcmPeripheralId_LIN1_UART1:
        {
            *clkSrcReg  = &(ptrMSSRCMRegs->LIN1_UART1_CLK_SRC_SEL);
            *clkdDivReg = &(ptrMSSRCMRegs->LIN1_UART1_CLK_DIV_VAL);
            *clkSrcVal = gLinUartClkSrcValMap[clkSource];
            break;
        }
        case SOC_RcmPeripheralId_LIN2_UART2:
        {
            *clkSrcReg  = &(ptrMSSRCMRegs->LIN2_UART2_CLK_SRC_SEL);
            *clkdDivReg = &(ptrMSSRCMRegs->LIN2_UART2_CLK_DIV_VAL);
            *clkSrcVal = gLinUartClkSrcValMap[clkSource];
            break;
        }
        case SOC_RcmPeripheralId_LIN3_UART3:
        {
            *clkSrcReg  = &(ptrMSSRCMRegs->LIN3_UART3_CLK_SRC_SEL);
            *clkdDivReg = &(ptrMSSRCMRegs->LIN3_UART3_CLK_DIV_VAL);
            *clkSrcVal = gLinUartClkSrcValMap[clkSource];
            break;
        }
        case SOC_RcmPeripheralId_LIN4_UART4:
        {
            *clkSrcReg  = &(ptrMSSRCMRegs->LIN4_UART4_CLK_SRC_SEL);
            *clkdDivReg = &(ptrMSSRCMRegs->LIN4_UART4_CLK_DIV_VAL);
            *clkSrcVal = gLinUartClkSrcValMap[clkSource];
            break;
        }
        case SOC_RcmPeripheralId_LIN5_UART5:
        {
            *clkSrcReg  = &(ptrMSSRCMRegs->LIN5_UART5_CLK_SRC_SEL);
            *clkdDivReg = &(ptrMSSRCMRegs->LIN5_UART5_CLK_DIV_VAL);
            *clkSrcVal = gLinUartClkSrcValMap[clkSource];
            break;
        }
        case SOC_RcmPeripheralId_MCAN4:
        {
            *clkSrcReg  = &(ptrMSSRCMRegs->MCAN4_CLK_SRC_SEL);
            *clkdDivReg = &(ptrMSSRCMRegs->MCAN4_CLK_DIV_VAL);
            *clkSrcVal = gMcanClkSrcValMap[clkSource];
            break;
        }
        case SOC_RcmPeripheralId_MCAN5:
        {
            *clkSrcReg  = &(ptrMSSRCMRegs->MCAN5_CLK_SRC_SEL);
            *clkdDivReg = &(ptrMSSRCMRegs->MCAN5_CLK_DIV_VAL);
            *clkSrcVal = gMcanClkSrcValMap[clkSource];
            break;
        }
        case SOC_RcmPeripheralId_MCAN6:
        {
            *clkSrcReg  = &(ptrMSSRCMRegs->MCAN6_CLK_SRC_SEL);
            *clkdDivReg = &(ptrMSSRCMRegs->MCAN6_CLK_DIV_VAL);
            *clkSrcVal = gMcanClkSrcValMap[clkSource];
            break;
        }
        case SOC_RcmPeripheralId_MCAN7:
        {
            *clkSrcReg  = &(ptrMSSRCMRegs->MCAN7_CLK_SRC_SEL);
            *clkdDivReg = &(ptrMSSRCMRegs->MCAN7_CLK_DIV_VAL);
            *clkSrcVal = gMcanClkSrcValMap[clkSource];
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



/**
 *  @b Description
 *  @n
 *      This API Configure the Dividers and Multipliers for Core PLL
 *
 *  @param[in]  inputClockDiv
 *      Pre-divider value
 *  @param[in]  divider
 *      Divider value
 *  @param[in]  multiplier
 *      Multiplier Value
 *  @param[in]  postDivider
 *      Post Divider Value
 *  @param[in]  fracMultiplier
 *      Fractional Multiplier programmable value
 *
 *  @retval     None
 */
static void SOC_rcmProgPllCoreDivider (uint8_t inputClockDiv , uint8_t divider,
                                uint16_t multiplier, uint8_t postDivider,
                                uint32_t fracMultiplier)
{
    volatile uint32_t *ptrM2NReg, *ptrMN2Reg, *ptrFracMReg;
    CSL_top_rcmRegs *ptrTopRCMRegs;

    ptrTopRCMRegs = SOC_rcmGetBaseAddressTOPRCM ();

    ptrM2NReg   = &(ptrTopRCMRegs->PLL_CORE_M2NDIV);
    ptrMN2Reg   = &(ptrTopRCMRegs->PLL_CORE_MN2DIV);
    ptrFracMReg = &(ptrTopRCMRegs->PLL_CORE_FRACDIV);

    // APPLJ-1 Setting
    // CLOCKOUT = M/(N+1) * CLKINP * (1/M2)

    *ptrM2NReg = SOC_rcmInsert8 (*ptrM2NReg, 22U, 16U, postDivider);    //CSL_TOP_RCM_PLL_CORE_M2NDIV_M2_SHIFT, CSL_TOP_RCM_PLL_CORE_M2NDIV_M2_MASK

    /* program N (input clock divider) */
    *ptrM2NReg = SOC_rcmInsert8 (*ptrM2NReg, 7U, 0U, inputClockDiv);    //CSL_TOP_RCM_PLL_CORE_M2NDIV_N_SHIFT, CSL_TOP_RCM_PLL_CORE_M2NDIV_N_MASK

    /* program M (multiplier) */
    *ptrMN2Reg = SOC_rcmInsert16 (*ptrMN2Reg, 11U, 0U, multiplier);     //CSL_TOP_RCM_PLL_CORE_MN2DIV_M_SHIFT, CSL_TOP_RCM_PLL_CORE_MN2DIV_M_MASK

    /* program N2 (divider) */
    *ptrMN2Reg = SOC_rcmInsert8 (*ptrMN2Reg, 19U, 16U, divider);        //CSL_TOP_RCM_PLL_CORE_MN2DIV_N2_SHIFT, CSL_TOP_RCM_PLL_CORE_MN2DIV_N2_MASK

    /* program Fractional Multiplier */
    *ptrFracMReg = SOC_rcmInsert32 (*ptrFracMReg, 17U, 0U, fracMultiplier); //CSL_TOP_RCM_PLL_CORE_FRACDIV_FRACTIONALM_SHIFT, CSL_TOP_RCM_PLL_CORE_FRACDIV_FRACTIONALM_MASK

    /* Sigma delta divider for optimum jitter bit needs to be updated since ROM code programmed to its need */
    *ptrFracMReg = SOC_rcmInsert8 (*ptrFracMReg, 31U, 24U, 8U); //CSL_TOP_RCM_PLL_CORE_FRACDIV_REGSD_SHIFT, CSL_TOP_RCM_PLL_CORE_FRACDIV_REGSD_MASK
}

/**
 *  @b Description
 *  @n
 *      This API Configure the Dividers and Multipliers for peripheral PLL
 *
 *  @param[in]  inputClockDiv
 *      Pre-divider value
 *  @param[in]  divider
 *      Divider value
 *  @param[in]  multiplier
 *      Multiplier Value
 *  @param[in]  postDivider
 *      Post Divider Value
 *  @param[in]  fracMultiplier
 *      Fractional Multiplier programmable value
 *
 *  @retval     None
 */
static void SOC_rcmProgPllPerDivider (uint8_t inputClockDiv , uint8_t divider,
                               uint16_t multiplier, uint8_t postDivider,
                               uint32_t fracMultiplier)
{
    volatile uint32_t *ptrM2NReg, *ptrMN2Reg, *ptrFracMReg;
    CSL_top_rcmRegs *ptrTopRCMRegs;

    ptrTopRCMRegs = SOC_rcmGetBaseAddressTOPRCM ();

    ptrM2NReg   = &(ptrTopRCMRegs->PLL_PER_M2NDIV);
    ptrMN2Reg   = &(ptrTopRCMRegs->PLL_PER_MN2DIV);
    ptrFracMReg = &(ptrTopRCMRegs->PLL_PER_FRACDIV);

    // APPLJ-1 Setting
    // CLOCKOUT = M/(N+1) * CLKINP * (1/M2)

    *ptrM2NReg = SOC_rcmInsert8 (*ptrM2NReg, 22U, 16U, postDivider);    //CSL_TOP_RCM_PLL_PER_M2NDIV_M2_SHIFT, CSL_TOP_RCM_PLL_PER_M2NDIV_M2_MASK

    /* program N (input clock divider) */
    *ptrM2NReg = SOC_rcmInsert8 (*ptrM2NReg, 7U, 0U, inputClockDiv);    //CSL_TOP_RCM_PLL_PER_M2NDIV_N_SHIFT, CSL_TOP_RCM_PLL_PER_M2NDIV_N_MASK

    /* program M (multiplier) */
    *ptrMN2Reg = SOC_rcmInsert16 (*ptrMN2Reg, 11U, 0U, multiplier);     //CSL_TOP_RCM_PLL_PER_MN2DIV_M_SHIFT, CSL_TOP_RCM_PLL_PER_MN2DIV_M_MASK

    /* program N2 (divider) */
    *ptrMN2Reg = SOC_rcmInsert8 (*ptrMN2Reg, 19U, 16U, divider);        //CSL_TOP_RCM_PLL_PER_MN2DIV_N2_SHIFT, CSL_TOP_RCM_PLL_PER_MN2DIV_N2_MASK

    /* program Fractional Multiplier */
    *ptrFracMReg = SOC_rcmInsert32 (*ptrFracMReg, 17U, 0U, fracMultiplier); //CSL_TOP_RCM_PLL_PER_FRACDIV_FRACTIONALM_SHIFT, CSL_TOP_RCM_PLL_PER_FRACDIV_FRACTIONALM_MASK
}


/**
 *  @b Description
 *  @n
 *      This API Configure the Dividers and Multipliers for Core/DSP PLL
 *
 *  @param[in]  trimVal
 *      Trim value for the PLL
 *  @retval     None
 */
static void SOC_rcmConfigurePllCore (void)
{
    volatile uint32_t *ptrClkCtrl, *ptrTenable, *ptrTenableDiv, *ptrPllStatus;
    CSL_top_rcmRegs *ptrTopRCMRegs;
    uint8_t phaseLockStatus;

    /* Core PLL settings */
    ptrTopRCMRegs = SOC_rcmGetBaseAddressTOPRCM ();

    ptrClkCtrl    = &(ptrTopRCMRegs->PLL_CORE_CLKCTRL);
    ptrTenable    = &(ptrTopRCMRegs->PLL_CORE_TENABLE);
    ptrTenableDiv = &(ptrTopRCMRegs->PLL_CORE_TENABLEDIV);
    ptrPllStatus  = &(ptrTopRCMRegs->PLL_CORE_STATUS);

    /* program CLKDCOLDOEN[29] = 1, IDLE[23] = 0, CLKDCOLDOPWDNZ[17] = 1, SELFREQDCO[12:10] = 4 */
    *ptrClkCtrl = SOC_rcmInsert8 (*ptrClkCtrl, 29U, 29U, 0x1U);
    *ptrClkCtrl = SOC_rcmInsert8 (*ptrClkCtrl, 23U, 23U, 0x0U);
    *ptrClkCtrl = SOC_rcmInsert8 (*ptrClkCtrl, 17U, 17U, 0x1U);
    *ptrClkCtrl = SOC_rcmInsert8 (*ptrClkCtrl, 12U, 10U, 0x4U);


    /* Soft reset Pll. Not required */
    /* *ptrClkCtrl = SOC_rcmInsert8 (*ptrClkCtrl, 0U, 0U, 0x0U); */

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

    /* wait for the Phase lock for Core PLL */
    do
    {
        phaseLockStatus = SOC_rcmExtract8 (*ptrPllStatus, 10U, 10U);
    }while(phaseLockStatus != 1U);
}

/* Pre Requisite Sequence to relock core pll as mentioned in the clock spec */
uint32_t SOC_rcmCoreApllRelockPreRequisite(void)
{

    volatile uint32_t *ptrClkCtrl;
    CSL_top_rcmRegs *ptrTopRCMRegs;

    ptrTopRCMRegs = SOC_rcmGetBaseAddressTOPRCM ();
    ptrClkCtrl    = &(ptrTopRCMRegs->PLL_CORE_CLKCTRL);

    uint32_t r5ClkSrc = ptrTopRCMRegs->R5SS_CLK_SRC_SEL;

	/* Switch the R5F Sorce clock to 25MHz */
    ptrTopRCMRegs->R5SS_CLK_SRC_SEL = SOC_rcmInsert16 (ptrTopRCMRegs->R5SS_CLK_SRC_SEL, 11U, 0U, gR5SysClkSrcValMap[SOC_RcmPeripheralClockSource_WUCPUCLK]);

    /* Assert Soft Reset Pll */
    *ptrClkCtrl = SOC_rcmInsert8 (*ptrClkCtrl, 0U, 0U, 0x0U);

    return r5ClkSrc;
}

/* Set R5 clock source */
void SOC_rcmSetR5ClockSource(uint32_t r5ClkSrc)
{
    CSL_top_rcmRegs *ptrTopRCMRegs;

    ptrTopRCMRegs = SOC_rcmGetBaseAddressTOPRCM ();

	/* Write R5F Source clock */
    ptrTopRCMRegs->R5SS_CLK_SRC_SEL = r5ClkSrc;

}

/**
 *  @b Description
 *  @n
 *      This API Configure the Dividers and Multipliers for Core/DSP PLL
 *
 *  @param[in]  trimVal
 *      Trim value for the PLL
 *  @retval     None
 */
static void SOC_rcmConfigurePllPer (void)
{
    volatile uint32_t *ptrClkCtrl, *ptrTenable, *ptrTenableDiv, *ptrPllStatus;
    CSL_top_rcmRegs *ptrTopRCMRegs;
    uint8_t phaseLockStatus;

    /* Core PLL settings */
    ptrTopRCMRegs = SOC_rcmGetBaseAddressTOPRCM ();

    ptrClkCtrl    = &(ptrTopRCMRegs->PLL_PER_CLKCTRL);
    ptrTenable    = &(ptrTopRCMRegs->PLL_PER_TENABLE);
    ptrTenableDiv = &(ptrTopRCMRegs->PLL_PER_TENABLEDIV);
    ptrPllStatus  = &(ptrTopRCMRegs->PLL_PER_STATUS);

    /* program CLKDCOLDOEN[29] = 1, IDLE[23] = 0, CLKDCOLDOPWDNZ[17] = 1, SELFREQDCO[12:10] = 4 */
    *ptrClkCtrl = SOC_rcmInsert8 (*ptrClkCtrl, 29U, 29U, 0x1U);
    *ptrClkCtrl = SOC_rcmInsert8 (*ptrClkCtrl, 23U, 23U, 0x0U);
    *ptrClkCtrl = SOC_rcmInsert8 (*ptrClkCtrl, 17U, 17U, 0x1U);
    *ptrClkCtrl = SOC_rcmInsert8 (*ptrClkCtrl, 12U, 10U, 0x4U);

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

    /* wait for the Phase lock for peripheral PLL */
    do
    {
        phaseLockStatus = SOC_rcmExtract8 (*ptrPllStatus, 10U, 10U);
    }while(phaseLockStatus != 1U);
}

static uint32_t SOC_rcmGetCoreFout(uint32_t Finp, bool div2flag);

static uint32_t SOC_rcmGetCoreHSDivOut(uint32_t Finp, bool div2flag, SOC_RcmPllHSDIVOutId hsDivOut)
{
    uint32_t FOut;
    uint32_t clkDiv;
    CSL_top_rcmRegs *ptrTopRCMRegs;

    ptrTopRCMRegs = SOC_rcmGetBaseAddressTOPRCM ();
    FOut = SOC_rcmGetCoreFout(Finp, div2flag);
    switch(hsDivOut)
    {
        case RCM_PLLHSDIV_OUT_0:
        {
            clkDiv = CSL_FEXT(ptrTopRCMRegs->PLL_CORE_HSDIVIDER_CLKOUT0, TOP_RCM_PLL_CORE_HSDIVIDER_CLKOUT0_DIV);
            break;
        }
        case RCM_PLLHSDIV_OUT_1:
        {
            clkDiv = CSL_FEXT(ptrTopRCMRegs->PLL_CORE_HSDIVIDER_CLKOUT1, TOP_RCM_PLL_CORE_HSDIVIDER_CLKOUT1_DIV);
            break;
        }
        case RCM_PLLHSDIV_OUT_2:
        {
            clkDiv = CSL_FEXT(ptrTopRCMRegs->PLL_CORE_HSDIVIDER_CLKOUT2, TOP_RCM_PLL_CORE_HSDIVIDER_CLKOUT2_DIV);
            break;
        }
        case RCM_PLLHSDIV_OUT_NONE:
        {
            DebugP_assert(FALSE);
            clkDiv = 0;
            break;
        }
    }
    return (FOut/(clkDiv + 1));
}

static uint32_t SOC_rcmGetCR5SysclkInFrequency(void)
{
    uint32_t clkFreq = 0U;
    SOC_RcmClkSrcInfo const * clkSrcInfo;
    SOC_RcmXtalFreqId clkFreqId;
    uint32_t Finp;


    clkSrcInfo = &gCR5ClkSrcInfoMap;
    clkFreqId = RCM_XTAL_FREQID_CLK_25MHZ;

    Finp = gXTALInfo[clkFreqId].Finp;
    clkFreq = SOC_rcmGetCoreHSDivOut(Finp, gXTALInfo[clkFreqId].div2flag, clkSrcInfo->hsDivOut);

    return (clkFreq);

}

static uint32_t SOC_rcmGetTraceInFrequency(void)
{
    uint32_t clkFreq = 0U;
    SOC_RcmClkSrcInfo const * clkSrcInfo;
    SOC_RcmXtalFreqId clkFreqId;
    uint32_t Finp;


    clkSrcInfo = &gTraceClkSrcInfoMap;
    clkFreqId = RCM_XTAL_FREQID_CLK_25MHZ;

    Finp = gXTALInfo[clkFreqId].Finp;
    clkFreq = SOC_rcmGetCoreHSDivOut(Finp, gXTALInfo[clkFreqId].div2flag, clkSrcInfo->hsDivOut);

    return (clkFreq);

}

static uint32_t SOC_rcmGetCLKOUTInFrequency(void)
{
    uint32_t clkFreq = 0U;
    SOC_RcmClkSrcInfo const * clkSrcInfo;
    SOC_RcmXtalFreqId clkFreqId;
    uint32_t Finp;


    clkSrcInfo = &gCLKOUTClkSrcInfoMap;
    clkFreqId = RCM_XTAL_FREQID_CLK_25MHZ;

    Finp = gXTALInfo[clkFreqId].Finp;
    clkFreq = SOC_rcmGetCoreHSDivOut(Finp, gXTALInfo[clkFreqId].div2flag, clkSrcInfo->hsDivOut);

    return (clkFreq);

}

static uint32_t SOC_rcmGetModuleClkDivVal(uint32_t inFreq, uint32_t outFreq)
{
    uint32_t moduleClkDivVal;
    moduleClkDivVal = inFreq / outFreq;
    uint32_t actOutFreq = inFreq / moduleClkDivVal;
    DebugP_assert(actOutFreq == outFreq);
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

static SOC_RcmADPLLJConfig_t const * SOC_rcmGetADPLLJConfig(uint32_t Finp, SOC_RcmPllFoutFreqId foutFreqId)
{
    uint32_t i;
    SOC_RcmADPLLJConfig_t const *adplljCfg;
    uint32_t Fout;

    Fout = gPLLFreqId2FOutMap[foutFreqId];
    for (i = 0; i < SOC_RCM_UTILS_ARRAYSIZE(gADPLLJConfigTbl); i++)
    {
        if ((gADPLLJConfigTbl[i].Finp == Finp) && (gADPLLJConfigTbl[i].Fout == Fout))
        {
            break;
        }
    }
    if (i < SOC_RCM_UTILS_ARRAYSIZE(gADPLLJConfigTbl))
    {
        adplljCfg = &gADPLLJConfigTbl[i];
    }
    else
    {
        adplljCfg = (SOC_RcmADPLLJConfig_t const *)NULL;
    }
    return adplljCfg;
}


static uint32_t SOC_rcmADPLLJGetFOut(uint32_t Finp, uint32_t N, uint32_t M, uint32_t M2, uint32_t FracM, bool div2flag)
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
    for (i = 0; i < SOC_RCM_UTILS_ARRAYSIZE(gADPLLJConfigTbl); i++)
    {
        if ((gADPLLJConfigTbl[i].Finp == Finp)   &&
            (gADPLLJConfigTbl[i].FracM == FracM) &&
            (gADPLLJConfigTbl[i].M == M)         &&
            (gADPLLJConfigTbl[i].M2 == M2)       &&
            (gADPLLJConfigTbl[i].N  == Nmatch))
        {
            break;
        }
    }
    if (i < SOC_RCM_UTILS_ARRAYSIZE(gADPLLJConfigTbl))
    {
        FOut = SOC_RCM_FREQ_MHZ2HZ(gADPLLJConfigTbl[i].Fout);
    }
    else
    {
        FOut = 0;
    }
    return FOut;
}


static uint32_t SOC_rcmGetCoreFout(uint32_t Finp, bool div2flag)
{
    uint8_t pllSwitchFlag;
    CSL_top_rcmRegs *ptrTopRCMRegs;
    uint32_t FOut;

    ptrTopRCMRegs = SOC_rcmGetBaseAddressTOPRCM ();
    /* read the Core PLL Lock status */
    pllSwitchFlag = SOC_rcmExtract8 (ptrTopRCMRegs->PLL_CORE_STATUS, 10U, 10U);
    if (pllSwitchFlag)
    {
        uint32_t M, N, M2, FracM;

        N  = CSL_FEXT(ptrTopRCMRegs->PLL_CORE_M2NDIV, TOP_RCM_PLL_CORE_M2NDIV_N);
        M2 = CSL_FEXT(ptrTopRCMRegs->PLL_CORE_M2NDIV, TOP_RCM_PLL_CORE_M2NDIV_M2);
        M  = CSL_FEXT(ptrTopRCMRegs->PLL_CORE_MN2DIV, TOP_RCM_PLL_CORE_MN2DIV_M);
        FracM = CSL_FEXT(ptrTopRCMRegs->PLL_CORE_FRACDIV,TOP_RCM_PLL_CORE_FRACDIV_FRACTIONALM);
        FOut = SOC_rcmADPLLJGetFOut(Finp, N, M, M2, FracM,div2flag);
        DebugP_assert(FOut != 0);
    }
    else
    {
        uint32_t ULOWCLKEN = CSL_FEXT(ptrTopRCMRegs->PLL_CORE_CLKCTRL, TOP_RCM_PLL_CORE_CLKCTRL_ULOWCLKEN);
        if (ULOWCLKEN == 0)
        {
            uint32_t N2;

            N2  = CSL_FEXT(ptrTopRCMRegs->PLL_CORE_MN2DIV, TOP_RCM_PLL_CORE_MN2DIV_N2);
            FOut = Finp/(N2 + 1);
        }
        else
        {
            FOut = Finp;
        }
    }
    return FOut;
}


static uint32_t SOC_rcmGetPerFout(uint32_t Finp, bool div2flag)
{
    uint8_t pllSwitchFlag;
    CSL_top_rcmRegs *ptrTopRCMRegs;
    uint32_t FOut;

    ptrTopRCMRegs = SOC_rcmGetBaseAddressTOPRCM ();
    /* read the Core PLL Lock status */
    pllSwitchFlag = SOC_rcmExtract8 (ptrTopRCMRegs->PLL_PER_STATUS, 10U, 10U);
    if (pllSwitchFlag)
    {
        uint32_t M, N, M2, FracM;

        N  = CSL_FEXT(ptrTopRCMRegs->PLL_PER_M2NDIV, TOP_RCM_PLL_PER_M2NDIV_N);
        M2 = CSL_FEXT(ptrTopRCMRegs->PLL_PER_M2NDIV, TOP_RCM_PLL_PER_M2NDIV_M2);
        M  = CSL_FEXT(ptrTopRCMRegs->PLL_PER_MN2DIV, TOP_RCM_PLL_PER_MN2DIV_M);
        FracM = CSL_FEXT(ptrTopRCMRegs->PLL_PER_FRACDIV,TOP_RCM_PLL_PER_FRACDIV_FRACTIONALM);
        FOut = SOC_rcmADPLLJGetFOut(Finp, N, M, M2, FracM, div2flag);
        DebugP_assert(FOut != 0);
    }
    else
    {
        uint32_t ULOWCLKEN = CSL_FEXT(ptrTopRCMRegs->PLL_PER_CLKCTRL, TOP_RCM_PLL_PER_CLKCTRL_ULOWCLKEN);
        if (ULOWCLKEN == 0)
        {
            uint32_t N2;

            N2  = CSL_FEXT(ptrTopRCMRegs->PLL_PER_MN2DIV, TOP_RCM_PLL_PER_MN2DIV_N2);
            FOut = Finp/(N2 + 1);
        }
        else
        {
            FOut = Finp;
        }
    }
    return FOut;
}

static uint32_t SOC_rcmGetPerHSDivOut(uint32_t Finp, bool div2flag, SOC_RcmPllHSDIVOutId hsDivOut)
{
    uint32_t FOut;
    uint32_t clkDiv;
    CSL_top_rcmRegs *ptrTopRCMRegs;

    ptrTopRCMRegs = SOC_rcmGetBaseAddressTOPRCM ();

    FOut = SOC_rcmGetPerFout(Finp, div2flag);
    switch(hsDivOut)
    {
        case RCM_PLLHSDIV_OUT_0:
        {
            clkDiv = CSL_FEXT(ptrTopRCMRegs->PLL_PER_HSDIVIDER_CLKOUT0, TOP_RCM_PLL_PER_HSDIVIDER_CLKOUT0_DIV);
            break;
        }
        case RCM_PLLHSDIV_OUT_1:
        {
            clkDiv = CSL_FEXT(ptrTopRCMRegs->PLL_PER_HSDIVIDER_CLKOUT1, TOP_RCM_PLL_PER_HSDIVIDER_CLKOUT0_DIV);
            break;
        }
        case RCM_PLLHSDIV_OUT_2:
        {
            DebugP_assert(FALSE);
            clkDiv = 0;
            break;
        }
        case RCM_PLLHSDIV_OUT_NONE:
        {
            DebugP_assert(FALSE);
            clkDiv = 0;
            break;
        }
    }

    return (FOut/(clkDiv  + 1));
}

uint32_t SOC_rcmIsR5FInLockStepMode(uint32_t r5fClusterGroupId)
{
    uint32_t retVal = FALSE;
    CSL_mss_ctrlRegs *mssCtrl = SOC_rcmGetBaseAddressMSSCTRL();

    if (r5fClusterGroupId == CSL_ARM_R5_CLUSTER_GROUP_ID_0)
    {
		if (CSL_FEXT(mssCtrl->R5SS0_STATUS_REG, MSS_CTRL_R5SS0_STATUS_REG_LOCK_STEP) == 1U)
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
    if (r5fClusterGroupId == CSL_ARM_R5_CLUSTER_GROUP_ID_1)
    {
		if (CSL_FEXT(mssCtrl->R5SS1_STATUS_REG, MSS_CTRL_R5SS1_STATUS_REG_LOCK_STEP) == 1U)
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

/**
 *  @b Description
 *  @n
 *      This API returns the frequency of the clock which is passed as parameter
 *  @param[in]  clkSource
 *      clock source for which frequency is requested
 *
 *  \ingroup DRIVER_RCM_FUNCTIONS
 *
 *  @retval     Clock Frequency (in Hz)
 *  @note       return value of 0 indicates clock source not available
 */
uint32_t SOC_rcmGetPeripheralClockFrequency(SOC_RcmPeripheralClockSource clkSource)
{
    uint32_t clkFreq = 0U;
    SOC_RcmClkSrcInfo const * clkSrcInfo;
    SOC_RcmXtalFreqId clkFreqId;
    uint32_t Finp;


    clkSrcInfo = &gPeripheralClkSrcInfoMap[clkSource];
    clkFreqId = RCM_XTAL_FREQID_CLK_25MHZ;
    switch (clkSrcInfo->pllId)
    {
        case RCM_PLLID_CORE:
        {
            Finp = gXTALInfo[clkFreqId].Finp;
            clkFreq = SOC_rcmGetCoreHSDivOut(Finp, gXTALInfo[clkFreqId].div2flag, clkSrcInfo->hsDivOut);
            if (clkSource == SOC_RcmPeripheralClockSource_SYS_CLK)
            {
                CSL_top_rcmRegs *ptrTopRCMRegs;
                uint32_t sysClkDiv;

                /* Core PLL settings */
                ptrTopRCMRegs = SOC_rcmGetBaseAddressTOPRCM ();
                sysClkDiv = CSL_FEXT(ptrTopRCMRegs->SYS_CLK_DIV_VAL, TOP_RCM_SYS_CLK_DIV_VAL_CLKDIV);
                sysClkDiv = (sysClkDiv & 0xF) + 1;
                clkFreq = clkFreq / sysClkDiv;
            }
            break;
        }
        case RCM_PLLID_PER:
        {
            Finp = gXTALInfo[clkFreqId].Finp;
            clkFreq = SOC_rcmGetPerHSDivOut(Finp, gXTALInfo[clkFreqId].div2flag, clkSrcInfo->hsDivOut);
            break;
        }
        case RCM_PLLID_XTALCLK:
        {
            Finp = gXTALInfo[clkFreqId].Finp;
            clkFreq = Finp * 1000 * 1000;
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

void SOC_rcmCoreApllConfig(SOC_RcmPllFoutFreqId outFreqId, SOC_RcmPllHsDivOutConfig *hsDivCfg)
{
    SOC_RcmXtalFreqId XTALFreq;
    SOC_RcmADPLLJConfig_t const * adplljCfg;

    /* read the XTAL Frequency */
    XTALFreq = RCM_XTAL_FREQID_CLK_25MHZ;

    /* program PLL dividers and multipliers.  */
    adplljCfg = SOC_rcmGetADPLLJConfig(gXTALInfo[XTALFreq].Finp, outFreqId);
    DebugP_assert(adplljCfg != NULL);

    if (adplljCfg != NULL)
    {
        if (gXTALInfo[XTALFreq].div2flag == false)
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
        SOC_rcmConfigurePllCore ();

        SOC_rcmCoreApllHSDivConfig(outFreqId, hsDivCfg);
    }

    return;
}

void SOC_rcmCoreApllHSDivConfig(SOC_RcmPllFoutFreqId outFreqId, SOC_RcmPllHsDivOutConfig *hsDivCfg)
{
    CSL_top_rcmRegs *ptrTopRCMRegs;
    uint32_t hsDivOutRegVal;
    uint32_t Fout;

    Fout = SOC_RCM_FREQ_MHZ2HZ(gPLLFreqId2FOutMap[outFreqId]);
    /* Core PLL settings */
    ptrTopRCMRegs = SOC_rcmGetBaseAddressTOPRCM ();

    /* Derive Clocks */
    /* Set clock divider values from Core PLL*/
    if (hsDivCfg->hsdivOutEnMask & RCM_PLL_HSDIV_OUTPUT_ENABLE_0)
    {
        DebugP_assert((Fout % hsDivCfg->hsDivOutFreqHz[RCM_PLL_HSDIV_OUTPUT_IDX0]) == 0);
        hsDivOutRegVal = Fout / hsDivCfg->hsDivOutFreqHz[RCM_PLL_HSDIV_OUTPUT_IDX0];
        hsDivOutRegVal--;
        ptrTopRCMRegs->PLL_CORE_HSDIVIDER_CLKOUT0 = SOC_rcmInsert8 (ptrTopRCMRegs->PLL_CORE_HSDIVIDER_CLKOUT0, 4U, 0U, hsDivOutRegVal);
    }
    if (hsDivCfg->hsdivOutEnMask & RCM_PLL_HSDIV_OUTPUT_ENABLE_1)
    {
        DebugP_assert((Fout % hsDivCfg->hsDivOutFreqHz[RCM_PLL_HSDIV_OUTPUT_IDX1]) == 0);
        hsDivOutRegVal = Fout / hsDivCfg->hsDivOutFreqHz[RCM_PLL_HSDIV_OUTPUT_IDX1];
        hsDivOutRegVal--;
        ptrTopRCMRegs->PLL_CORE_HSDIVIDER_CLKOUT1 = SOC_rcmInsert8 (ptrTopRCMRegs->PLL_CORE_HSDIVIDER_CLKOUT1, 4U, 0U, hsDivOutRegVal);
    }
    if (hsDivCfg->hsdivOutEnMask & RCM_PLL_HSDIV_OUTPUT_ENABLE_2)
    {
        DebugP_assert((Fout % hsDivCfg->hsDivOutFreqHz[RCM_PLL_HSDIV_OUTPUT_IDX2]) == 0);
        hsDivOutRegVal = Fout / hsDivCfg->hsDivOutFreqHz[RCM_PLL_HSDIV_OUTPUT_IDX2];
        hsDivOutRegVal--;
        ptrTopRCMRegs->PLL_CORE_HSDIVIDER_CLKOUT2 = SOC_rcmInsert8 (ptrTopRCMRegs->PLL_CORE_HSDIVIDER_CLKOUT2, 4U, 0U, hsDivOutRegVal);
    }
    /* Core PLL output 3 not used.WIll not configure */
    DebugP_assert((hsDivCfg->hsdivOutEnMask & RCM_PLL_HSDIV_OUTPUT_ENABLE_3) == 0);
    /* Generate Trigger to latch these values */
    ptrTopRCMRegs->PLL_CORE_HSDIVIDER         = SOC_rcmInsert8 (ptrTopRCMRegs->PLL_CORE_HSDIVIDER, 2U, 2U, 0x1U);
    ptrTopRCMRegs->PLL_CORE_HSDIVIDER         = SOC_rcmInsert8 (ptrTopRCMRegs->PLL_CORE_HSDIVIDER, 2U, 2U, 0x0U);

    /* Ungate the clocks */
    if (hsDivCfg->hsdivOutEnMask & RCM_PLL_HSDIV_OUTPUT_ENABLE_0)
    {
        ptrTopRCMRegs->PLL_CORE_HSDIVIDER_CLKOUT0 = SOC_rcmInsert8 (ptrTopRCMRegs->PLL_CORE_HSDIVIDER_CLKOUT0, 8U, 8U, 0x1U);
    }
    if (hsDivCfg->hsdivOutEnMask & RCM_PLL_HSDIV_OUTPUT_ENABLE_1)
    {
        ptrTopRCMRegs->PLL_CORE_HSDIVIDER_CLKOUT1 = SOC_rcmInsert8 (ptrTopRCMRegs->PLL_CORE_HSDIVIDER_CLKOUT1, 8U, 8U, 0x1U);
    }
    if (hsDivCfg->hsdivOutEnMask & RCM_PLL_HSDIV_OUTPUT_ENABLE_2)
    {
        ptrTopRCMRegs->PLL_CORE_HSDIVIDER_CLKOUT2 = SOC_rcmInsert8 (ptrTopRCMRegs->PLL_CORE_HSDIVIDER_CLKOUT2, 8U, 8U, 0x1U);
    }

    return;
}

void SOC_rcmPerApllConfig(SOC_RcmPllFoutFreqId outFreqId, SOC_RcmPllHsDivOutConfig *hsDivCfg)
{
    SOC_RcmXtalFreqId XTALFreq;
    CSL_top_rcmRegs *ptrTopRCMRegs;
    SOC_RcmADPLLJConfig_t const * adplljCfg;
    uint32_t hsDivOutRegVal;
    uint32_t Fout;

    Fout = SOC_RCM_FREQ_MHZ2HZ(gPLLFreqId2FOutMap[outFreqId]);
    /* Core PLL settings */
    ptrTopRCMRegs = SOC_rcmGetBaseAddressTOPRCM ();

    /* read the XTAL Frequency */
    XTALFreq = RCM_XTAL_FREQID_CLK_25MHZ;

   /* program PLL dividers and multipliers.  */
    adplljCfg = SOC_rcmGetADPLLJConfig(gXTALInfo[XTALFreq].Finp, outFreqId);
    DebugP_assert(adplljCfg != NULL);

    if (adplljCfg != NULL)
    {
        if (gXTALInfo[XTALFreq].div2flag == false)
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
        SOC_rcmConfigurePllPer ();

        /* Derive Clocks */
        /* Set clock divider values from PER PLL*/
        if (hsDivCfg->hsdivOutEnMask & RCM_PLL_HSDIV_OUTPUT_ENABLE_0)
        {
            DebugP_assert((Fout % hsDivCfg->hsDivOutFreqHz[RCM_PLL_HSDIV_OUTPUT_IDX0]) == 0);
            hsDivOutRegVal = Fout / hsDivCfg->hsDivOutFreqHz[RCM_PLL_HSDIV_OUTPUT_IDX0];
            hsDivOutRegVal--;
            ptrTopRCMRegs->PLL_PER_HSDIVIDER_CLKOUT0 = SOC_rcmInsert8 (ptrTopRCMRegs->PLL_PER_HSDIVIDER_CLKOUT0, 4U, 0U, hsDivOutRegVal);
        }
        if (hsDivCfg->hsdivOutEnMask & RCM_PLL_HSDIV_OUTPUT_ENABLE_1)
        {
            DebugP_assert((Fout % hsDivCfg->hsDivOutFreqHz[RCM_PLL_HSDIV_OUTPUT_IDX1]) == 0);
            hsDivOutRegVal = Fout / hsDivCfg->hsDivOutFreqHz[RCM_PLL_HSDIV_OUTPUT_IDX1];
            hsDivOutRegVal--;
            ptrTopRCMRegs->PLL_PER_HSDIVIDER_CLKOUT1 = SOC_rcmInsert8 (ptrTopRCMRegs->PLL_PER_HSDIVIDER_CLKOUT1, 4U, 0U, hsDivOutRegVal);
        }
        /* Core PLL output 2 not used.WIll not configure */
        /* Core PLL output 3 not used.WIll not configure */

        /* Generate Trigger to latch these values */
        ptrTopRCMRegs->PLL_PER_HSDIVIDER         = SOC_rcmInsert8 (ptrTopRCMRegs->PLL_PER_HSDIVIDER, 2U, 2U, 0x1U);
        ptrTopRCMRegs->PLL_PER_HSDIVIDER         = SOC_rcmInsert8 (ptrTopRCMRegs->PLL_PER_HSDIVIDER, 2U, 2U, 0x0U);

        /* Ungate the clocks */
        if (hsDivCfg->hsdivOutEnMask & RCM_PLL_HSDIV_OUTPUT_ENABLE_0)
        {
            ptrTopRCMRegs->PLL_PER_HSDIVIDER_CLKOUT0 = SOC_rcmInsert8 (ptrTopRCMRegs->PLL_PER_HSDIVIDER_CLKOUT0, 8U, 8U, 0x1U);
        }
        if (hsDivCfg->hsdivOutEnMask & RCM_PLL_HSDIV_OUTPUT_ENABLE_1)
        {
            ptrTopRCMRegs->PLL_PER_HSDIVIDER_CLKOUT1 = SOC_rcmInsert8 (ptrTopRCMRegs->PLL_PER_HSDIVIDER_CLKOUT1, 8U, 8U, 0x1U);
        }

    }

    return;
}

void SOC_rcmsetR5SysClock(uint32_t cr5FreqHz, uint32_t sysClkFreqHz, uint32_t cpuId)
{
    CSL_top_rcmRegs *ptrTopRCMRegs;
    uint32_t Finp, clockStatus = 0U;
    uint32_t moduleClkDivVal;

    ptrTopRCMRegs = SOC_rcmGetBaseAddressTOPRCM ();

    Finp = SOC_rcmGetCR5SysclkInFrequency();

    /*  1.  Program SYS CLK GCD register with the value of 0x111 in-order to switch to a new desired frequency, SYS_CLK_DIV_VAL.CLKDIV = 0x111
        2.  Poll for the CURRDIVR field of corresponding status register to reflect its new frequency change, SYS_CLK_STATUS. CURRDIVIDER = 0x1
    */
    moduleClkDivVal = SOC_rcmGetModuleClkDivVal(Finp, sysClkFreqHz);
    ptrTopRCMRegs->SYS_CLK_DIV_VAL = SOC_rcmInsert16 (ptrTopRCMRegs->SYS_CLK_DIV_VAL, 11U, 0U, SOC_rcmGetModuleClkDivRegVal(moduleClkDivVal));

    /*
        3.	If the R5 clock frequency needs to be same as SYS clock frequency, then program the R5SS0_CLK_DIV_SEL.CLKDIVSEL = 0x7 (or / and) R5SS1_CLK_DIV_SEL.CLKDIVSEL = 0x7 register(s) as required or else leave with default value of 0x0 without any programming
    */
    /* Divide by 1 to get the R5 Core Clock */
    moduleClkDivVal = SOC_rcmGetModuleClkDivVal(Finp, cr5FreqHz);

    if ((cpuId == CSL_CORE_ID_R5FSS0_0) || (cpuId == CSL_CORE_ID_R5FSS0_1))
    {
        ptrTopRCMRegs->R5SS0_CLK_DIV_SEL = SOC_rcmInsert16 (ptrTopRCMRegs->R5SS0_CLK_DIV_SEL, 11U, 0U, SOC_rcmGetModuleClkDivRegVal(moduleClkDivVal));
    }
    else if ((cpuId == CSL_CORE_ID_R5FSS1_0) || (cpuId == CSL_CORE_ID_R5FSS1_1))
    {
        ptrTopRCMRegs->R5SS1_CLK_DIV_SEL = SOC_rcmInsert16 (ptrTopRCMRegs->R5SS1_CLK_DIV_SEL, 11U, 0U, SOC_rcmGetModuleClkDivRegVal(moduleClkDivVal));
    }
    else
    {
        /* Nothing to be done here */
    }

    /*  4.  After the divider configuration, update the R5SS GCM register with the value of 0x222 to select the PLL_CORE_CLOCKOUT0 as its source, R5SS_CLK_SRC_SEL.CLKSRCSEL= 0x222
    */
    /* Select CLKOUT0 as clock for R5 Core */
    ptrTopRCMRegs->R5SS_CLK_SRC_SEL = SOC_rcmInsert16 (ptrTopRCMRegs->R5SS_CLK_SRC_SEL, 11U, 0U, gR5SysClkSrcValMap[SOC_RcmPeripheralClockSource_DPLL_CORE_HSDIV0_CLKOUT0]);

    /* 5.  Poll for the CLKINUSE field of corresponding status register to reflect its new frequency change, R5SS_CLK_STATUS.CLKINUSE = 0x04 */
    do
    {
        clockStatus = SOC_rcmExtract8 (ptrTopRCMRegs->R5SS_CLK_STATUS, 7U, 0U);
    }while(clockStatus != 0x4U);

}

void SOC_rcmsetTraceClock(uint32_t traceFreqHz)
{
    CSL_top_rcmRegs *ptrTopRCMRegs;
    uint32_t Finp;
    uint32_t moduleClkDivVal;

    Finp = SOC_rcmGetTraceInFrequency();
    ptrTopRCMRegs = SOC_rcmGetBaseAddressTOPRCM ();

    /*
        1.  Program TRCCLKOUT GCD register with the value of 0x111 in-order to switch to a new desired frequency, TRCCLKOUT_DIV_VAL.CLKDIV = 0x111
        2.  Poll for the CURRDIVR field of corresponding status register to reflect its new frequency change, TRCCLKOUT_CLK_STATUS.CURRDIVIDER = 0x01
    */
    moduleClkDivVal = SOC_rcmGetModuleClkDivVal(Finp, traceFreqHz);
    ptrTopRCMRegs->TRCCLKOUT_DIV_VAL = SOC_rcmInsert16 (ptrTopRCMRegs->TRCCLKOUT_DIV_VAL, 11U, 0U, SOC_rcmGetModuleClkDivRegVal(moduleClkDivVal));

    /*
        3.  Update the TRCCLKOUT GCM register with the value of 0x222 to select PLL_CORE_CLKOUT1 as its source, TRCCLKOUT_CLK_SRC_SEL.CLKSRCSEL = 0x222
        4.  Poll for the CLKINUSE field of corresponding status register to reflect its new frequency change, TRCCLKOUT_CLK_STATUS.CLKINUSE = 0x04
    */
    ptrTopRCMRegs->TRCCLKOUT_CLK_SRC_SEL = SOC_rcmInsert16 (ptrTopRCMRegs->TRCCLKOUT_CLK_SRC_SEL, 11U, 0U,  gTraceClkSrcValMap[SOC_RcmPeripheralClockSource_DPLL_CORE_HSDIV0_CLKOUT1]);

}

void SOC_rcmsetClkoutClock(uint32_t clkout0FreqHz, uint32_t clkout1FreqHz)
{
    CSL_top_rcmRegs *ptrTopRCMRegs;
    uint32_t Finp;
    uint32_t moduleClkDivVal;

    Finp = SOC_rcmGetCLKOUTInFrequency();
    ptrTopRCMRegs = SOC_rcmGetBaseAddressTOPRCM ();

    /*
        1.	Program CLKOUT0 GCD register with the value of 0x000 in-order to switch to a new desired frequency, CLKOUT0_DIV_VAL.CLKDIV = 0x111
        2.	Poll for the CURRDIVR field of corresponding status register to reflect its new frequency change, CLKOUT0_CLK_STATUS.CURRDIVIDER = 0x01
    */

    moduleClkDivVal = SOC_rcmGetModuleClkDivVal(Finp, clkout0FreqHz);
    ptrTopRCMRegs->CLKOUT0_DIV_VAL = SOC_rcmInsert16 (ptrTopRCMRegs->CLKOUT0_DIV_VAL, 11U, 0U, SOC_rcmGetModuleClkDivRegVal(moduleClkDivVal));

    /*
        3.	Update the CLKOUT0 GCM register with the value of 0x222 to select PLL_CORE_CLKOUT1 as its source, CLKOUT0_CLK_SRC_SEL.CLKSRCSEL = 0x222
        4.	Poll for the CLKINUSE field of corresponding status register to reflect its new frequency change, CLKOUT0_CLK_STATUS.CLKINUSE = 0x04
    */
    ptrTopRCMRegs->CLKOUT0_CLK_SRC_SEL = SOC_rcmInsert16 (ptrTopRCMRegs->CLKOUT0_CLK_SRC_SEL, 11U, 0U, gClkoutClkSrcValMap[SOC_RcmPeripheralClockSource_DPLL_CORE_HSDIV0_CLKOUT1]);


    moduleClkDivVal = SOC_rcmGetModuleClkDivVal(Finp, clkout1FreqHz);
    ptrTopRCMRegs->CLKOUT1_DIV_VAL = SOC_rcmInsert16 (ptrTopRCMRegs->CLKOUT1_DIV_VAL, 11U, 0U, SOC_rcmGetModuleClkDivRegVal(moduleClkDivVal));


    ptrTopRCMRegs->CLKOUT1_CLK_SRC_SEL = SOC_rcmInsert16 (ptrTopRCMRegs->CLKOUT1_CLK_SRC_SEL, 11U, 0U, gClkoutClkSrcValMap[SOC_RcmPeripheralClockSource_DPLL_CORE_HSDIV0_CLKOUT1]);

}

int32_t SOC_rcmSetPeripheralClock (SOC_RcmPeripheralId periphId,
                                      SOC_RcmPeripheralClockSource clkSource,
                                      uint32_t freqHz)
{
    volatile uint32_t   *ptrClkSrcReg, *ptrClkDivReg;
    volatile uint16_t   clkSrcVal;
    uint32_t            clkDivisor;
    int32_t             retVal;
    uint32_t            Finp;

    Finp = SOC_rcmGetPeripheralClockFrequency(clkSource);
    clkDivisor = SOC_rcmGetModuleClkDivVal(Finp, freqHz);
    SOC_rcmGetClkSrcAndDivReg (periphId, clkSource, &clkSrcVal, &ptrClkSrcReg, &ptrClkDivReg);

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

SOC_RcmResetCause SOC_rcmGetResetCause (SOC_Rcmr5fssNum r5fssNum)
{
    CSL_mss_rcmRegs *ptrRCMRegs;
    uint16_t    resetCauseBits = 0x0U;
    uint8_t     resetCause = 0U;

    ptrRCMRegs = SOC_rcmGetBaseAddressMSSRCM ();

    /* Read the Reset Cause Register bits */
    if(r5fssNum==r5fss0)
    {
        resetCauseBits = SOC_rcmExtract16 (ptrRCMRegs->R5SS0_RST_STATUS, 10U, 0U);
    }
    else
    {
        if(r5fssNum==r5fss1)
        {
            resetCauseBits = SOC_rcmExtract16 (ptrRCMRegs->R5SS1_RST_STATUS, 10U, 0U);
        }
    }

    if (resetCauseBits == 0x0U)
    {
        /* Value specifying R5 Reset due to Unknown reason */
        resetCause = 11U;
    }
    else
    {
        while ((resetCauseBits & 0x1U) != 0x1U)
        {
            resetCauseBits = resetCauseBits >> 1U;
            resetCause = resetCause + 1U;
        }

        /* clear the reset cause */

        if(r5fssNum==r5fss0)
        {
            ptrRCMRegs->R5SS0_RST_CAUSE_CLR = SOC_rcmInsert8 (ptrRCMRegs->R5SS0_RST_CAUSE_CLR, 2U, 0U, 0x7U);
        }
        else
        {
            if(r5fssNum==r5fss1)
            {
                ptrRCMRegs->R5SS1_RST_CAUSE_CLR = SOC_rcmInsert8 (ptrRCMRegs->R5SS1_RST_CAUSE_CLR, 2U, 0U, 0x7U);
            }
        }
    }

    return (SOC_RcmResetCause) gResetBitToResetCause[resetCause];
}

int32_t SOC_rcmEnablePeripheralClock(SOC_RcmPeripheralId periphId, uint32_t enable)
{
    int32_t status = SystemP_SUCCESS;

    CSL_mss_rcmRegs *ptrMSSRCMRegs;

    ptrMSSRCMRegs = SOC_rcmGetBaseAddressMSSRCM ();

    switch (periphId)
    {
        case SOC_RcmPeripheralId_MCAN0:
        {
            if(enable==1)
            {
                ptrMSSRCMRegs->MCAN0_CLK_GATE = CSL_MSS_RCM_MCAN0_CLK_GATE_GATED_RESETVAL;
            }
            else
            if(enable==0)
            {
                ptrMSSRCMRegs->MCAN0_CLK_GATE = CSL_MSS_RCM_MCAN0_CLK_GATE_GATED_MASK;
            }
            break;
        }
        case SOC_RcmPeripheralId_MCAN1:
        {
            if(enable==1)
            {
                ptrMSSRCMRegs->MCAN1_CLK_GATE = CSL_MSS_RCM_MCAN1_CLK_GATE_GATED_RESETVAL;
            }
            else
            if(enable==0)
            {
                ptrMSSRCMRegs->MCAN1_CLK_GATE = CSL_MSS_RCM_MCAN1_CLK_GATE_GATED_MASK;
            }
            break;
        }
        case SOC_RcmPeripheralId_MCAN2:
        {
            if(enable==1)
            {
                ptrMSSRCMRegs->MCAN2_CLK_GATE = CSL_MSS_RCM_MCAN2_CLK_GATE_GATED_RESETVAL;
            }
            else
            if(enable==0)
            {
                ptrMSSRCMRegs->MCAN2_CLK_GATE = CSL_MSS_RCM_MCAN2_CLK_GATE_GATED_MASK;
            }
            break;
        }
        case SOC_RcmPeripheralId_MCAN3:
        {
            if(enable==1)
            {
                ptrMSSRCMRegs->MCAN3_CLK_GATE = CSL_MSS_RCM_MCAN3_CLK_GATE_GATED_RESETVAL;
            }
            else
            if(enable==0)
            {
                ptrMSSRCMRegs->MCAN3_CLK_GATE = CSL_MSS_RCM_MCAN3_CLK_GATE_GATED_MASK;
            }
            break;
        }
        //FIXME: Change OSPI TO OSPI. ADD OTHER OSPI RELATED DATA
        case SOC_RcmPeripheralId_OSPI0:
        {
            if(enable==1)
            {
                ptrMSSRCMRegs->OSPI_CLK_GATE = CSL_MSS_RCM_OSPI_CLK_GATE_RESETVAL;
            }
            else
            if(enable==0)
            {
                ptrMSSRCMRegs->OSPI_CLK_GATE = CSL_MSS_RCM_OSPI_CLK_GATE_OSPI_CLK_GATE_GATED_MASK;
            }
            break;
        }
        case SOC_RcmPeripheralId_RTI0:
        {
            if(enable==1)
            {
                ptrMSSRCMRegs->RTI0_CLK_GATE = CSL_MSS_RCM_RTI0_CLK_GATE_GATED_RESETVAL;
            }
            else
            if(enable==0)
            {
                ptrMSSRCMRegs->RTI0_CLK_GATE = CSL_MSS_RCM_RTI0_CLK_GATE_GATED_MASK;
            }
            break;
        }
        case SOC_RcmPeripheralId_RTI1:
        {
            if(enable==1)
            {
                ptrMSSRCMRegs->RTI1_CLK_GATE = CSL_MSS_RCM_RTI1_CLK_GATE_GATED_RESETVAL;
            }
            else
            if(enable==0)
            {
                ptrMSSRCMRegs->RTI1_CLK_GATE = CSL_MSS_RCM_RTI1_CLK_GATE_GATED_MASK;
            }
            break;
        }
        case SOC_RcmPeripheralId_RTI2:
        {
            if(enable==1)
            {
                ptrMSSRCMRegs->RTI2_CLK_GATE = CSL_MSS_RCM_RTI2_CLK_GATE_GATED_RESETVAL;
            }
            else
            if(enable==0)
            {
                ptrMSSRCMRegs->RTI2_CLK_GATE = CSL_MSS_RCM_RTI2_CLK_GATE_GATED_MASK;
            }
            break;
        }
        case SOC_RcmPeripheralId_RTI3:
        {
            if(enable==1)
            {
                ptrMSSRCMRegs->RTI3_CLK_GATE = CSL_MSS_RCM_RTI3_CLK_GATE_GATED_RESETVAL;
            }
            else
            if(enable==0)
            {
                ptrMSSRCMRegs->RTI3_CLK_GATE = CSL_MSS_RCM_RTI3_CLK_GATE_GATED_MASK;
            }
            break;
        }
        case SOC_RcmPeripheralId_WDT0:
        {
            if(enable==1)
            {
                ptrMSSRCMRegs->WDT0_CLK_GATE = CSL_MSS_RCM_WDT0_CLK_GATE_GATED_RESETVAL;
            }
            else
            if(enable==0)
            {
                ptrMSSRCMRegs->WDT0_CLK_GATE = CSL_MSS_RCM_WDT0_CLK_GATE_GATED_MASK;
            }
            break;
        }
        case SOC_RcmPeripheralId_WDT1:
        {
            if(enable==1)
            {
                ptrMSSRCMRegs->WDT1_CLK_GATE = CSL_MSS_RCM_WDT1_CLK_GATE_GATED_RESETVAL;
            }
            else
            if(enable==0)
            {
                ptrMSSRCMRegs->WDT1_CLK_GATE = CSL_MSS_RCM_WDT1_CLK_GATE_GATED_MASK;
            }
            break;
        }
        case SOC_RcmPeripheralId_WDT2:
        {
            if(enable==1)
            {
                ptrMSSRCMRegs->WDT2_CLK_GATE = CSL_MSS_RCM_WDT2_CLK_GATE_GATED_RESETVAL;
            }
            else
            if(enable==0)
            {
                ptrMSSRCMRegs->WDT2_CLK_GATE = CSL_MSS_RCM_WDT2_CLK_GATE_GATED_MASK;
            }
            break;
        }
        case SOC_RcmPeripheralId_WDT3:
        {
            if(enable==1)
            {
                ptrMSSRCMRegs->WDT3_CLK_GATE = CSL_MSS_RCM_WDT3_CLK_GATE_GATED_RESETVAL;
            }
            else
            if(enable==0)
            {
                ptrMSSRCMRegs->WDT3_CLK_GATE = CSL_MSS_RCM_WDT3_CLK_GATE_GATED_MASK;
            }
            break;
        }
        case SOC_RcmPeripheralId_MCSPI0:
        {
            if(enable==1)
            {
                ptrMSSRCMRegs->MCSPI0_CLK_GATE = CSL_MSS_RCM_MCSPI0_CLK_GATE_GATED_RESETVAL;
            }
            else
            if(enable==0)
            {
                ptrMSSRCMRegs->MCSPI0_CLK_GATE = CSL_MSS_RCM_MCSPI0_CLK_GATE_GATED_MASK;
            }
            break;
        }
        case SOC_RcmPeripheralId_MCSPI1:
        {
            if(enable==1)
            {
                ptrMSSRCMRegs->MCSPI1_CLK_GATE = CSL_MSS_RCM_MCSPI1_CLK_GATE_GATED_RESETVAL;
            }
            else
            if(enable==0)
            {
                ptrMSSRCMRegs->MCSPI1_CLK_GATE = CSL_MSS_RCM_MCSPI1_CLK_GATE_GATED_MASK;
            }
            break;
        }
        case SOC_RcmPeripheralId_MCSPI2:
        {
            if(enable==1)
            {
                ptrMSSRCMRegs->MCSPI2_CLK_GATE = CSL_MSS_RCM_MCSPI2_CLK_GATE_GATED_RESETVAL;
            }
            else
            if(enable==0)
            {
                ptrMSSRCMRegs->MCSPI2_CLK_GATE = CSL_MSS_RCM_MCSPI2_CLK_GATE_GATED_MASK;
            }
            break;
        }
        case SOC_RcmPeripheralId_MCSPI3:
        {
            if(enable==1)
            {
                ptrMSSRCMRegs->MCSPI3_CLK_GATE = CSL_MSS_RCM_MCSPI3_CLK_GATE_GATED_RESETVAL;
            }
            else
            if(enable==0)
            {
                ptrMSSRCMRegs->MCSPI3_CLK_GATE = CSL_MSS_RCM_MCSPI3_CLK_GATE_GATED_MASK;
            }
            break;
        }
        case SOC_RcmPeripheralId_MCSPI4:
        {
            if(enable==1)
            {
                ptrMSSRCMRegs->MCSPI4_CLK_GATE = CSL_MSS_RCM_MCSPI4_CLK_GATE_GATED_RESETVAL;
            }
            else
            if(enable==0)
            {
                ptrMSSRCMRegs->MCSPI4_CLK_GATE = CSL_MSS_RCM_MCSPI4_CLK_GATE_GATED_MASK;
            }
            break;
        }
        case SOC_RcmPeripheralId_MCSPI5:
        {
            if(enable==1)
            {
                ptrMSSRCMRegs->MCSPI5_CLK_GATE = CSL_MSS_RCM_MCSPI5_CLK_GATE_GATED_RESETVAL;
            }
            else
            if(enable==0)
            {
                ptrMSSRCMRegs->MCSPI5_CLK_GATE = CSL_MSS_RCM_MCSPI5_CLK_GATE_GATED_MASK;
            }
            break;
        }
        case SOC_RcmPeripheralId_MCSPI6:
        {
            if(enable==1)
            {
                ptrMSSRCMRegs->MCSPI6_CLK_GATE = CSL_MSS_RCM_MCSPI6_CLK_GATE_GATED_RESETVAL;
            }
            else
            if(enable==0)
            {
                ptrMSSRCMRegs->MCSPI6_CLK_GATE = CSL_MSS_RCM_MCSPI6_CLK_GATE_GATED_MASK;
            }
            break;
        }
        case SOC_RcmPeripheralId_MCSPI7:
        {
            if(enable==1)
            {
                ptrMSSRCMRegs->MCSPI7_CLK_GATE = CSL_MSS_RCM_MCSPI7_CLK_GATE_GATED_RESETVAL;
            }
            else
            if(enable==0)
            {
                ptrMSSRCMRegs->MCSPI7_CLK_GATE = CSL_MSS_RCM_MCSPI7_CLK_GATE_GATED_MASK;
            }
            break;
        }
        case SOC_RcmPeripheralId_MMC0:
        {
            if(enable==1)
            {
                ptrMSSRCMRegs->MMC0_CLK_GATE = CSL_MSS_RCM_MMC0_CLK_GATE_GATED_RESETVAL;
            }
            else
            if(enable==0)
            {
                ptrMSSRCMRegs->MMC0_CLK_GATE = CSL_MSS_RCM_MMC0_CLK_GATE_GATED_MASK;
            }
            break;
        }
        case SOC_RcmPeripheralId_ICSSM0_UART0:
        {
            if(enable==1)
            {
                ptrMSSRCMRegs->ICSSM0_UART_CLK_GATE = CSL_MSS_RCM_ICSSM0_UART_CLK_GATE_GATED_RESETVAL;
            }
            else
            if(enable==0)
            {
                ptrMSSRCMRegs->ICSSM0_UART_CLK_GATE = CSL_MSS_RCM_ICSSM0_UART_CLK_GATE_GATED_MASK;
            }
            break;
        }
        case SOC_RcmPeripheralId_CPTS:
        {
            if(enable==1)
            {
                ptrMSSRCMRegs->CPTS_CLK_GATE = CSL_MSS_RCM_CPTS_CLK_GATE_GATED_RESETVAL;
            }
            else
            if(enable==0)
            {
                ptrMSSRCMRegs->CPTS_CLK_GATE = CSL_MSS_RCM_CPTS_CLK_GATE_GATED_MASK;
            }
            break;
        }
        //FIXME REMOVE GPMC FOR AM263PX
        // case SOC_RcmPeripheralId_GPMC:
        // {
        //     if(enable==1)
        //     {
        //         ptrMSSRCMRegs->GPMC_CLK_GATE = CSL_MSS_RCM_GPMC_CLK_GATE_GATED_RESETVAL;
        //     }
        //     else
        //     if(enable==0)
        //     {
        //         ptrMSSRCMRegs->GPMC_CLK_GATE = CSL_MSS_RCM_GPMC_CLK_GATE_GATED_MASK;
        //     }
        //     break;
        // }
        case SOC_RcmPeripheralId_CONTROLSS_PLL:
        {
            if(enable==1)
            {
                ptrMSSRCMRegs->CONTROLSS_PLL_CLK_GATE = CSL_MSS_RCM_CONTROLSS_PLL_CLK_GATE_GATED_RESETVAL;
            }
            else
            if(enable==0)
            {
                ptrMSSRCMRegs->CONTROLSS_PLL_CLK_GATE = CSL_MSS_RCM_CONTROLSS_PLL_CLK_GATE_GATED_MASK;
            }
            break;
        }
        case SOC_RcmPeripheralId_I2C:
        {
            if(enable==1)
            {
                ptrMSSRCMRegs->I2C0_CLK_GATE = CSL_MSS_RCM_I2C0_CLK_GATE_GATED_RESETVAL;
                ptrMSSRCMRegs->I2C1_CLK_GATE = CSL_MSS_RCM_I2C1_CLK_GATE_GATED_RESETVAL;
                ptrMSSRCMRegs->I2C2_CLK_GATE = CSL_MSS_RCM_I2C2_CLK_GATE_GATED_RESETVAL;
                ptrMSSRCMRegs->I2C3_CLK_GATE = CSL_MSS_RCM_I2C3_CLK_GATE_GATED_RESETVAL;
            }
            else
            if(enable==0)
            {
                ptrMSSRCMRegs->I2C0_CLK_GATE = CSL_MSS_RCM_I2C0_CLK_GATE_GATED_MASK;
                ptrMSSRCMRegs->I2C1_CLK_GATE = CSL_MSS_RCM_I2C1_CLK_GATE_GATED_MASK;
                ptrMSSRCMRegs->I2C2_CLK_GATE = CSL_MSS_RCM_I2C2_CLK_GATE_GATED_MASK;
                ptrMSSRCMRegs->I2C3_CLK_GATE = CSL_MSS_RCM_I2C3_CLK_GATE_GATED_MASK;
            }
            break;
        }
        case SOC_RcmPeripheralId_LIN0_UART0:
        {
            if(enable==1)
            {
                ptrMSSRCMRegs->LIN0_CLK_GATE = CSL_MSS_RCM_LIN0_CLK_GATE_GATED_RESETVAL;
                ptrMSSRCMRegs->UART0_CLK_GATE = CSL_MSS_RCM_UART0_CLK_GATE_GATED_RESETVAL;
            }
            else
            if(enable==0)
            {
                ptrMSSRCMRegs->LIN0_CLK_GATE = CSL_MSS_RCM_LIN0_CLK_GATE_GATED_MASK;
                ptrMSSRCMRegs->UART0_CLK_GATE = CSL_MSS_RCM_UART0_CLK_GATE_GATED_MASK;
            }
            break;
        }
        case SOC_RcmPeripheralId_LIN1_UART1:
        {
            if(enable==1)
            {
                ptrMSSRCMRegs->LIN1_CLK_GATE = CSL_MSS_RCM_LIN1_CLK_GATE_GATED_RESETVAL;
                ptrMSSRCMRegs->UART1_CLK_GATE = CSL_MSS_RCM_UART1_CLK_GATE_GATED_RESETVAL;
            }
            else
            if(enable==0)
            {
                ptrMSSRCMRegs->LIN1_CLK_GATE = CSL_MSS_RCM_LIN1_CLK_GATE_GATED_MASK;
                ptrMSSRCMRegs->UART1_CLK_GATE = CSL_MSS_RCM_UART1_CLK_GATE_GATED_MASK;
            }
            break;
        }
        case SOC_RcmPeripheralId_LIN2_UART2:
        {
            if(enable==1)
            {
                ptrMSSRCMRegs->LIN2_CLK_GATE = CSL_MSS_RCM_LIN2_CLK_GATE_GATED_RESETVAL;
                ptrMSSRCMRegs->UART2_CLK_GATE = CSL_MSS_RCM_UART2_CLK_GATE_GATED_RESETVAL;
            }
            else
            if(enable==0)
            {
                ptrMSSRCMRegs->LIN2_CLK_GATE = CSL_MSS_RCM_LIN2_CLK_GATE_GATED_MASK;
                ptrMSSRCMRegs->UART2_CLK_GATE = CSL_MSS_RCM_UART2_CLK_GATE_GATED_MASK;
            }
            break;
        }
        case SOC_RcmPeripheralId_LIN3_UART3:
        {
            if(enable==1)
            {
                ptrMSSRCMRegs->LIN3_CLK_GATE = CSL_MSS_RCM_LIN3_CLK_GATE_GATED_RESETVAL;
                ptrMSSRCMRegs->UART3_CLK_GATE = CSL_MSS_RCM_UART3_CLK_GATE_GATED_RESETVAL;
            }
            else
            if(enable==0)
            {
                ptrMSSRCMRegs->LIN3_CLK_GATE = CSL_MSS_RCM_LIN3_CLK_GATE_GATED_MASK;
                ptrMSSRCMRegs->UART3_CLK_GATE = CSL_MSS_RCM_UART3_CLK_GATE_GATED_MASK;
            }
            break;
        }
        case SOC_RcmPeripheralId_LIN4_UART4:
        {
            if(enable==1)
            {
                ptrMSSRCMRegs->LIN4_CLK_GATE = CSL_MSS_RCM_LIN4_CLK_GATE_GATED_RESETVAL;
                ptrMSSRCMRegs->UART4_CLK_GATE = CSL_MSS_RCM_UART4_CLK_GATE_GATED_RESETVAL;
            }
            else
            if(enable==0)
            {
                ptrMSSRCMRegs->LIN4_CLK_GATE = CSL_MSS_RCM_LIN4_CLK_GATE_GATED_MASK;
                ptrMSSRCMRegs->UART4_CLK_GATE = CSL_MSS_RCM_UART4_CLK_GATE_GATED_MASK;
            }
            break;
        }
        case SOC_RcmPeripheralId_LIN5_UART5:
        {
            if(enable==1)
            {
                ptrMSSRCMRegs->UART5_CLK_GATE = CSL_MSS_RCM_UART5_CLK_GATE_GATED_RESETVAL;
            }
            else
            if(enable==0)
            {
                ptrMSSRCMRegs->UART5_CLK_GATE = CSL_MSS_RCM_UART5_CLK_GATE_GATED_MASK;
            }
            break;
        }
        case SOC_RcmPeripheralId_MCAN4:
        {
            if(enable==1)
            {
                ptrMSSRCMRegs->MCAN4_CLK_GATE = CSL_MSS_RCM_MCAN4_CLK_GATE_GATED_RESETVAL;
            }
            else
            if(enable==0)
            {
                ptrMSSRCMRegs->MCAN4_CLK_GATE = CSL_MSS_RCM_MCAN4_CLK_GATE_GATED_MASK;
            }
            break;
        }
        case SOC_RcmPeripheralId_MCAN5:
        {
            if(enable==1)
            {
                ptrMSSRCMRegs->MCAN5_CLK_GATE = CSL_MSS_RCM_MCAN5_CLK_GATE_GATED_RESETVAL;
            }
            else
            if(enable==0)
            {
                ptrMSSRCMRegs->MCAN5_CLK_GATE = CSL_MSS_RCM_MCAN5_CLK_GATE_GATED_MASK;
            }
            break;
        }
        case SOC_RcmPeripheralId_MCAN6:
        {
            if(enable==1)
            {
                ptrMSSRCMRegs->MCAN6_CLK_GATE = CSL_MSS_RCM_MCAN6_CLK_GATE_GATED_RESETVAL;
            }
            else
            if(enable==0)
            {
                ptrMSSRCMRegs->MCAN6_CLK_GATE = CSL_MSS_RCM_MCAN6_CLK_GATE_GATED_MASK;
            }
            break;
        }
        case SOC_RcmPeripheralId_MCAN7:
        {
            if(enable==1)
            {
                ptrMSSRCMRegs->MCAN7_CLK_GATE = CSL_MSS_RCM_MCAN7_CLK_GATE_GATED_RESETVAL;
            }
            else
            if(enable==0)
            {
                ptrMSSRCMRegs->MCAN7_CLK_GATE = CSL_MSS_RCM_MCAN7_CLK_GATE_GATED_MASK;
            }
            break;
        }
        default:
        {
            status = SystemP_FAILURE;
        }
    }

    return status;
}

int32_t SOC_rcmSetR5Clock(uint32_t r5FreqHz, uint32_t sysClkFreqHz, uint32_t cpuId)
{
    SOC_rcmsetR5SysClock(r5FreqHz, sysClkFreqHz, cpuId);

    return SystemP_SUCCESS;
}

uint32_t SOC_rcmGetR5Clock(uint32_t cpuId)
{
    uint32_t Finp;
    uint32_t moduleClkDivRegVal;
    uint32_t clkDivVal;
    CSL_top_rcmRegs *ptrTopRCMRegs;

    ptrTopRCMRegs = SOC_rcmGetBaseAddressTOPRCM ();
    Finp = SOC_rcmGetCR5SysclkInFrequency();

    moduleClkDivRegVal = ptrTopRCMRegs->R5SS0_CLK_DIV_SEL;
    if ((cpuId == CSL_CORE_ID_R5FSS0_0) || (cpuId == CSL_CORE_ID_R5FSS0_1))
    {
        moduleClkDivRegVal = ptrTopRCMRegs->R5SS0_CLK_DIV_SEL;
    }
    else if ((cpuId == CSL_CORE_ID_R5FSS1_0) || (cpuId == CSL_CORE_ID_R5FSS1_1))
    {
        moduleClkDivRegVal = ptrTopRCMRegs->R5SS1_CLK_DIV_SEL;
    }
    else
    {
        /* Nothing to be done here */
    }
    clkDivVal = SOC_rcmGetModuleClkDivFromRegVal(moduleClkDivRegVal);
    return (Finp / clkDivVal);
}

void SOC_rcmR5ConfigLockStep(uint32_t cpuId)
{
    CSL_mss_ctrlRegs *mssCtrl = SOC_rcmGetBaseAddressMSSCTRL ();
    uint32_t regVal;

    if (cpuId == CSL_CORE_ID_R5FSS0_0)
    {
        regVal = mssCtrl->R5SS0_CONTROL;
        CSL_FINS(regVal, MSS_CTRL_R5SS0_CONTROL_LOCK_STEP, 0x7);
        CSL_FINS(regVal, MSS_CTRL_R5SS0_CONTROL_LOCK_STEP_SWITCH_WAIT, 0x7);
        mssCtrl->R5SS0_CONTROL = regVal;
    }
    else if (cpuId == CSL_CORE_ID_R5FSS1_0)
    {
        regVal = mssCtrl->R5SS1_CONTROL;
        CSL_FINS(regVal, MSS_CTRL_R5SS1_CONTROL_LOCK_STEP, 0x7);
        CSL_FINS(regVal, MSS_CTRL_R5SS1_CONTROL_LOCK_STEP_SWITCH_WAIT, 0x7);
        mssCtrl->R5SS1_CONTROL = regVal;
    }
    else
    {
        DebugP_logError("Config Lockstep failed\r\n");
    }
}

void SOC_rcmR5ConfigDualCore(uint32_t cpuId)
{
    uint32_t regVal;
    CSL_mss_ctrlRegs *mssCtrl = SOC_rcmGetBaseAddressMSSCTRL ();

    if (cpuId == CSL_CORE_ID_R5FSS0_1)
    {
        if (SBL_rcmIsDualCoreSwitchSupported(cpuId) == TRUE)
        {
            regVal = mssCtrl->R5SS0_CONTROL;
            if ((regVal & CSL_MSS_CTRL_R5SS0_CONTROL_LOCK_STEP_MASK) == 0x0)
            {
                /* It is already in dual core mode. Skip programming LOCK STEP Bit. */
            }
            else
            {
                CSL_FINS(regVal, MSS_CTRL_R5SS0_CONTROL_LOCK_STEP, 0x0);
                CSL_FINS(regVal, MSS_CTRL_R5SS0_CONTROL_LOCK_STEP_SWITCH_WAIT, 0x7);
                mssCtrl->R5SS0_CONTROL = regVal;
            }
        }
    }
    else if (cpuId == CSL_CORE_ID_R5FSS1_1)
    {
        if (SBL_rcmIsDualCoreSwitchSupported(cpuId) == TRUE)
        {
            regVal = mssCtrl->R5SS1_CONTROL;
            if ((regVal & CSL_MSS_CTRL_R5SS1_CONTROL_LOCK_STEP_MASK) == 0x0)
            {
                /* It is already in dual core mode. Skip programming LOCK STEP Bit. */
            }
            else
            {
                CSL_FINS(regVal, MSS_CTRL_R5SS1_CONTROL_LOCK_STEP, 0x0);
                CSL_FINS(regVal, MSS_CTRL_R5SS1_CONTROL_LOCK_STEP_SWITCH_WAIT, 0x7);
                mssCtrl->R5SS1_CONTROL = regVal;
            }
        }
    }
    else
    {
        DebugP_logError("Dual Config mode failed\r\n");
    }
}

void SOC_rcmStartMemInitTCMA(uint32_t cpuId)
{
    CSL_mss_ctrlRegs *mssCtrl = SOC_rcmGetBaseAddressMSSCTRL ();

    if ((cpuId == CSL_CORE_ID_R5FSS0_0) || (cpuId == CSL_CORE_ID_R5FSS0_1))
    {
        /* Check MEMINIT STATUS is zero to confirm no inprogress MEM INIT */
        while (CSL_FEXT(mssCtrl->R5SS0_ATCM_MEM_INIT_STATUS, MSS_CTRL_R5SS0_ATCM_MEM_INIT_STATUS_MEM_STATUS) != 0);

        /* Clear MEMINIT DONE before initiating MEMINIT */
        CSL_FINS(mssCtrl->R5SS0_ATCM_MEM_INIT_DONE, MSS_CTRL_R5SS0_ATCM_MEM_INIT_DONE_MEM_INIT_DONE, 1);
        while (CSL_FEXT(mssCtrl->R5SS0_ATCM_MEM_INIT_DONE, MSS_CTRL_R5SS0_ATCM_MEM_INIT_DONE_MEM_INIT_DONE) != 0);

        CSL_FINS(mssCtrl->R5SS0_ATCM_MEM_INIT, MSS_CTRL_R5SS0_ATCM_MEM_INIT_MEM_INIT, 1);
    }
    else if ((cpuId == CSL_CORE_ID_R5FSS1_0) || (cpuId == CSL_CORE_ID_R5FSS1_1))
    {
        /* Check MEMINIT STATUS is zero to confirm no inprogress MEM INIT */
        while (CSL_FEXT(mssCtrl->R5SS1_ATCM_MEM_INIT_STATUS, MSS_CTRL_R5SS1_ATCM_MEM_INIT_STATUS_MEM_STATUS) != 0);

        /* Clear MEMINIT DONE before initiating MEMINIT */
        CSL_FINS(mssCtrl->R5SS1_ATCM_MEM_INIT_DONE, MSS_CTRL_R5SS1_ATCM_MEM_INIT_DONE_MEM_INIT_DONE, 1);
        while (CSL_FEXT(mssCtrl->R5SS1_ATCM_MEM_INIT_DONE, MSS_CTRL_R5SS1_ATCM_MEM_INIT_DONE_MEM_INIT_DONE) != 0);

        CSL_FINS(mssCtrl->R5SS1_ATCM_MEM_INIT, MSS_CTRL_R5SS1_ATCM_MEM_INIT_MEM_INIT, 1);
    }
    else
    {
        DebugP_logError("CPU TCM Init failed\r\n");
    }
}

void SOC_rcmWaitMemInitTCMA(uint32_t cpuId)
{
    CSL_mss_ctrlRegs*        mssCtrl;

    mssCtrl = SOC_rcmGetBaseAddressMSSCTRL ();
    if ((cpuId == CSL_CORE_ID_R5FSS0_0) || (cpuId == CSL_CORE_ID_R5FSS0_1))
    {
        while (CSL_FEXT(mssCtrl->R5SS0_ATCM_MEM_INIT_DONE, MSS_CTRL_R5SS0_ATCM_MEM_INIT_DONE_MEM_INIT_DONE) != 1);
        CSL_FINS(mssCtrl->R5SS0_ATCM_MEM_INIT_DONE, MSS_CTRL_R5SS0_ATCM_MEM_INIT_DONE_MEM_INIT_DONE, 1);
        /* Check MEMINIT STATUS is zero to confirm no inprogress MEM INIT */
        while (CSL_FEXT(mssCtrl->R5SS0_ATCM_MEM_INIT_STATUS, MSS_CTRL_R5SS0_ATCM_MEM_INIT_STATUS_MEM_STATUS) != 0);
        while (CSL_FEXT(mssCtrl->R5SS0_ATCM_MEM_INIT_DONE, MSS_CTRL_R5SS0_ATCM_MEM_INIT_DONE_MEM_INIT_DONE) != 0);
    }
    else if ((cpuId == CSL_CORE_ID_R5FSS1_0) || (cpuId == CSL_CORE_ID_R5FSS1_1))
    {
        while (CSL_FEXT(mssCtrl->R5SS1_ATCM_MEM_INIT_DONE, MSS_CTRL_R5SS1_ATCM_MEM_INIT_DONE_MEM_INIT_DONE) != 1);
        CSL_FINS(mssCtrl->R5SS1_ATCM_MEM_INIT_DONE, MSS_CTRL_R5SS1_ATCM_MEM_INIT_DONE_MEM_INIT_DONE, 1);
        /* Check MEMINIT STATUS is zero to confirm no inprogress MEM INIT */
        while (CSL_FEXT(mssCtrl->R5SS1_ATCM_MEM_INIT_STATUS, MSS_CTRL_R5SS1_ATCM_MEM_INIT_STATUS_MEM_STATUS) != 0);
        while (CSL_FEXT(mssCtrl->R5SS1_ATCM_MEM_INIT_DONE, MSS_CTRL_R5SS1_ATCM_MEM_INIT_DONE_MEM_INIT_DONE) != 0);

    }
    else
    {
        DebugP_logError("CPU TCM Init failed\r\n");
    }
}

void SOC_rcmStartMemInitTCMB(uint32_t cpuId)
{
    CSL_mss_ctrlRegs *mssCtrl = SOC_rcmGetBaseAddressMSSCTRL ();

    if ((cpuId == CSL_CORE_ID_R5FSS0_0) || (cpuId == CSL_CORE_ID_R5FSS0_1))
    {
        while (CSL_FEXT(mssCtrl->R5SS0_BTCM_MEM_INIT_STATUS, MSS_CTRL_R5SS0_BTCM_MEM_INIT_STATUS_MEM_STATUS) != 0);

        /* Check MEMINIT STATUS is zero to confirm no in progress MEM INIT */
        CSL_FINS(mssCtrl->R5SS0_BTCM_MEM_INIT_DONE, MSS_CTRL_R5SS0_BTCM_MEM_INIT_DONE_MEM_INIT_DONE, 1);
        while (CSL_FEXT(mssCtrl->R5SS0_BTCM_MEM_INIT_DONE, MSS_CTRL_R5SS0_BTCM_MEM_INIT_DONE_MEM_INIT_DONE) != 0);

        CSL_FINS(mssCtrl->R5SS0_BTCM_MEM_INIT, MSS_CTRL_R5SS0_BTCM_MEM_INIT_MEM_INIT, 1);
    }
    else if ((cpuId == CSL_CORE_ID_R5FSS1_0) || (cpuId == CSL_CORE_ID_R5FSS1_1))
    {
        while (CSL_FEXT(mssCtrl->R5SS1_BTCM_MEM_INIT_STATUS, MSS_CTRL_R5SS1_BTCM_MEM_INIT_STATUS_MEM_STATUS) != 0);

        /* Check MEMINIT STATUS is zero to confirm no in progress MEM INIT */
        CSL_FINS(mssCtrl->R5SS1_BTCM_MEM_INIT_DONE, MSS_CTRL_R5SS1_BTCM_MEM_INIT_DONE_MEM_INIT_DONE, 1);
        while (CSL_FEXT(mssCtrl->R5SS1_BTCM_MEM_INIT_DONE, MSS_CTRL_R5SS1_BTCM_MEM_INIT_DONE_MEM_INIT_DONE) != 0);

        CSL_FINS(mssCtrl->R5SS1_BTCM_MEM_INIT, MSS_CTRL_R5SS1_BTCM_MEM_INIT_MEM_INIT, 1);
    }
    else
    {
        DebugP_logError("CPU TCM Init failed\r\n");
    }
}

void SOC_rcmWaitMemInitTCMB(uint32_t cpuId)
{
    CSL_mss_ctrlRegs*        mssCtrl;

    mssCtrl = SOC_rcmGetBaseAddressMSSCTRL ();
    if ((cpuId == CSL_CORE_ID_R5FSS0_0) || (cpuId == CSL_CORE_ID_R5FSS0_1))
    {
        while (CSL_FEXT(mssCtrl->R5SS0_BTCM_MEM_INIT_DONE, MSS_CTRL_R5SS0_BTCM_MEM_INIT_DONE_MEM_INIT_DONE) != 1);
        /* Clear MEMINIT DONE before initiating MEMINIT */
        CSL_FINS(mssCtrl->R5SS0_BTCM_MEM_INIT_DONE, MSS_CTRL_R5SS0_BTCM_MEM_INIT_DONE_MEM_INIT_DONE, 1);

        /* Check MEMINIT STATUS is zero to confirm no inprogress MEM INIT */
        while (CSL_FEXT(mssCtrl->R5SS0_BTCM_MEM_INIT_STATUS, MSS_CTRL_R5SS0_BTCM_MEM_INIT_STATUS_MEM_STATUS) != 0);
        while (CSL_FEXT(mssCtrl->R5SS0_BTCM_MEM_INIT_DONE, MSS_CTRL_R5SS0_BTCM_MEM_INIT_DONE_MEM_INIT_DONE) != 0);
    }
    else if ((cpuId == CSL_CORE_ID_R5FSS1_0) || (cpuId == CSL_CORE_ID_R5FSS1_1))
    {
        while (CSL_FEXT(mssCtrl->R5SS1_BTCM_MEM_INIT_DONE, MSS_CTRL_R5SS1_BTCM_MEM_INIT_DONE_MEM_INIT_DONE) != 1);
        /* Clear MEMINIT DONE before initiating MEMINIT */
        CSL_FINS(mssCtrl->R5SS1_BTCM_MEM_INIT_DONE, MSS_CTRL_R5SS1_BTCM_MEM_INIT_DONE_MEM_INIT_DONE, 1);

        /* Check MEMINIT STATUS is zero to confirm no inprogress MEM INIT */
        while (CSL_FEXT(mssCtrl->R5SS1_BTCM_MEM_INIT_STATUS, MSS_CTRL_R5SS1_BTCM_MEM_INIT_STATUS_MEM_STATUS) != 0);
        while (CSL_FEXT(mssCtrl->R5SS1_BTCM_MEM_INIT_DONE, MSS_CTRL_R5SS1_BTCM_MEM_INIT_DONE_MEM_INIT_DONE) != 0);
    }
    else
    {
        DebugP_logError("CPU TCM Init failed\r\n");
    }
}

void SOC_rcmMemInitMailboxMemory(void)
{
    CSL_mss_ctrlRegs *mssCtrl = SOC_rcmGetBaseAddressMSSCTRL();
    CSL_FINS(mssCtrl->MAILBOXRAM_MEM_INIT, MSS_CTRL_MAILBOXRAM_MEM_INIT_MEM0_INIT, 1);
    while (CSL_FEXT(mssCtrl->MAILBOXRAM_MEM_INIT_DONE, MSS_CTRL_MAILBOXRAM_MEM_INIT_DONE_MEM0_DONE) != 1);
}

void SOC_rcmMemInitL2Memory(void)
{
    CSL_mss_ctrlRegs *mssCtrl = SOC_rcmGetBaseAddressMSSCTRL();

    /* MemInit for L2-Bank2 */
    CSL_FINS(mssCtrl->L2IOCRAM_MEM_INIT, MSS_CTRL_L2IOCRAM_MEM_INIT_PARTITION2, 1);
    while (CSL_FEXT(mssCtrl->L2OCRAM_MEM_INIT_DONE, MSS_CTRL_L2OCRAM_MEM_INIT_DONE_PARTITION2) != 1);
    CSL_FINS(mssCtrl->L2OCRAM_MEM_INIT_DONE, MSS_CTRL_L2OCRAM_MEM_INIT_DONE_PARTITION2, 1);

    /* MemInit for L2-Bank3 */
    CSL_FINS(mssCtrl->L2IOCRAM_MEM_INIT, MSS_CTRL_L2IOCRAM_MEM_INIT_PARTITION3, 1);
    while (CSL_FEXT(mssCtrl->L2OCRAM_MEM_INIT_DONE, MSS_CTRL_L2OCRAM_MEM_INIT_DONE_PARTITION3) != 1);
    CSL_FINS(mssCtrl->L2OCRAM_MEM_INIT_DONE, MSS_CTRL_L2OCRAM_MEM_INIT_DONE_PARTITION3, 1);

    /* MemInit for L2-Bank4 */
    CSL_FINS(mssCtrl->L2IOCRAM_MEM_INIT, MSS_CTRL_L2IOCRAM_MEM_INIT_PARTITION4, 1);
    while (CSL_FEXT(mssCtrl->L2OCRAM_MEM_INIT_DONE, MSS_CTRL_L2OCRAM_MEM_INIT_DONE_PARTITION4) != 1);
    CSL_FINS(mssCtrl->L2OCRAM_MEM_INIT_DONE, MSS_CTRL_L2OCRAM_MEM_INIT_DONE_PARTITION4, 1);

    /* MemInit for L2-Bank5 */
    CSL_FINS(mssCtrl->L2IOCRAM_MEM_INIT, MSS_CTRL_L2IOCRAM_MEM_INIT_PARTITION5, 1);
    while (CSL_FEXT(mssCtrl->L2OCRAM_MEM_INIT_DONE, MSS_CTRL_L2OCRAM_MEM_INIT_DONE_PARTITION5) != 1);
    CSL_FINS(mssCtrl->L2OCRAM_MEM_INIT_DONE, MSS_CTRL_L2OCRAM_MEM_INIT_DONE_PARTITION5, 1);

}

void SOC_rcmCoreR5FUnhalt(uint32_t cpuId)
{
    CSL_mss_ctrlRegs *mssCtrl = SOC_rcmGetBaseAddressMSSCTRL ();

    if (cpuId == CSL_CORE_ID_R5FSS0_1)
    {
        /* Core R5SS0_CORE1 unhalt */
        CSL_FINS(mssCtrl->R5SS0_CORE1_HALT, MSS_CTRL_R5SS0_CORE1_HALT_HALT, 0x0);
    }
    else if (cpuId == CSL_CORE_ID_R5FSS1_0)
    {
        /* Core R5SS1_CORE0 unhalt */
        CSL_FINS(mssCtrl->R5SS1_CORE0_HALT, MSS_CTRL_R5SS1_CORE0_HALT_HALT, 0x0);
    }
    else if (cpuId == CSL_CORE_ID_R5FSS1_1)
    {
        /* Core R5SS1_CORE1 unhalt */
        CSL_FINS(mssCtrl->R5SS1_CORE1_HALT, MSS_CTRL_R5SS1_CORE1_HALT_HALT, 0x0);
    }
    else
    {
        DebugP_logError("R5F Core Unhalt failed\r\n");
    }
}

void SOC_rcmR5SS0PowerOnReset(void)
{
    uint32_t regVal;
    CSL_mss_rcmRegs *ptrRCMRegs;
    ptrRCMRegs = SOC_rcmGetBaseAddressMSSRCM ();

    regVal = ptrRCMRegs->R5SS0_RST2ASSERTDLY;
    CSL_FINS(regVal, MSS_RCM_R5SS0_RST2ASSERTDLY_R5SS_CORE0_COUNT, 0x0);
    CSL_FINS(regVal, MSS_RCM_R5SS0_RST2ASSERTDLY_R5SS_CORE1_COUNT, 0x0);
    CSL_FINS(regVal, MSS_RCM_R5SS0_RST2ASSERTDLY_R5_CORE0_COUNT, 0x0);
    CSL_FINS(regVal, MSS_RCM_R5SS0_RST2ASSERTDLY_R5_CORE1_COUNT, 0x0);
    ptrRCMRegs->R5SS0_RST2ASSERTDLY = regVal;

    /* WR_MEM_32(MSS_RCM_U_BASE+RST_WFICHECK, 0x00000707); */ //RSTWFI CHECK
    regVal = ptrRCMRegs->R5SS0_RST_WFICHECK;
    CSL_FINS(regVal, MSS_RCM_R5SS0_RST_WFICHECK_EN_R5SS_CORE0, 0x7);
    CSL_FINS(regVal, MSS_RCM_R5SS0_RST_WFICHECK_EN_R5SS_CORE1, 0x7);
    CSL_FINS(regVal, MSS_RCM_R5SS0_RST_WFICHECK_EN_R5_CORE0, 0x7);
    CSL_FINS(regVal, MSS_RCM_R5SS0_RST_WFICHECK_EN_R5_CORE1, 0x7);
    ptrRCMRegs->R5SS0_RST_WFICHECK = regVal;
}

void SOC_rcmR5SS1PowerOnReset(void)
{
    uint32_t regVal;
    CSL_mss_rcmRegs *ptrRCMRegs;
    ptrRCMRegs = SOC_rcmGetBaseAddressMSSRCM ();

    regVal = ptrRCMRegs->R5SS1_RST2ASSERTDLY;
    CSL_FINS(regVal, MSS_RCM_R5SS1_RST2ASSERTDLY_R5SS_CORE0_COUNT, 0x0);
    CSL_FINS(regVal, MSS_RCM_R5SS1_RST2ASSERTDLY_R5SS_CORE1_COUNT, 0x0);
    CSL_FINS(regVal, MSS_RCM_R5SS1_RST2ASSERTDLY_R5_CORE0_COUNT, 0x0);
    CSL_FINS(regVal, MSS_RCM_R5SS1_RST2ASSERTDLY_R5_CORE1_COUNT, 0x0);
    ptrRCMRegs->R5SS1_RST2ASSERTDLY = regVal;

    /* WR_MEM_32(MSS_RCM_U_BASE+RST_WFICHECK, 0x00000707); */ //RSTWFI CHECK
    regVal = ptrRCMRegs->R5SS1_RST_WFICHECK;
    CSL_FINS(regVal, MSS_RCM_R5SS1_RST_WFICHECK_EN_R5SS_CORE0, 0x7);
    CSL_FINS(regVal, MSS_RCM_R5SS1_RST_WFICHECK_EN_R5SS_CORE1, 0x7);
    CSL_FINS(regVal, MSS_RCM_R5SS1_RST_WFICHECK_EN_R5_CORE0, 0x7);
    CSL_FINS(regVal, MSS_RCM_R5SS1_RST_WFICHECK_EN_R5_CORE1, 0x7);
    ptrRCMRegs->R5SS1_RST_WFICHECK = regVal;
}

void SOC_rcmR5SS1TriggerReset(void)
{
    uint32_t regVal;
    CSL_mss_ctrlRegs *mssCtrl = SOC_rcmGetBaseAddressMSSCTRL ();

    regVal = mssCtrl->R5SS1_CONTROL;
    CSL_FINS(regVal, MSS_CTRL_R5SS1_CONTROL_RESET_FSM_TRIGGER, 0x7);
    mssCtrl->R5SS1_CONTROL = regVal;
}

void SOC_rcmR5SS0TriggerReset(void)
{
    uint32_t regVal;
    CSL_mss_ctrlRegs *mssCtrl = SOC_rcmGetBaseAddressMSSCTRL ();

    regVal = mssCtrl->R5SS0_CONTROL;
    CSL_FINS(regVal, MSS_CTRL_R5SS0_CONTROL_RESET_FSM_TRIGGER, 0x7);
    mssCtrl->R5SS0_CONTROL = regVal;

    /* execute wfi inside a loop to clear any pending interrupts, and reset core0 and core 1 */
#if defined(__ARM_ARCH_7R__)
    while (1)
    {
        __wfi();
    }
#endif
}

void SOC_generateSwWarmReset(void)
{
    CSL_top_rcmRegs *ptrTOPRCMRegs;

    ptrTOPRCMRegs = SOC_rcmGetBaseAddressTOPRCM();

    /* Unlock CONTROLSS_CTRL registers */
    SOC_controlModuleUnlockMMR(SOC_DOMAIN_ID_MAIN, TOP_RCM_PARTITION0);

    CSL_FINS(ptrTOPRCMRegs->WARM_RESET_REQ, TOP_RCM_WARM_RESET_REQ_SW_RST, 0x0);

    /* Lock CONTROLSS_CTRL registers */
    SOC_controlModuleLockMMR(SOC_DOMAIN_ID_MAIN, TOP_RCM_PARTITION0);

    /* execute wfi */
#if defined(__ARM_ARCH_7R__)
    __wfi();
#endif
}

void SOC_configureWarmResetSource(uint32_t source)
{
    CSL_top_rcmRegs *ptrTOPRCMRegs;
    uint32_t regVal;

    ptrTOPRCMRegs = SOC_rcmGetBaseAddressTOPRCM();

    /* Unlock CONTROLSS_CTRL registers */
    SOC_controlModuleUnlockMMR(SOC_DOMAIN_ID_MAIN, TOP_RCM_PARTITION0);

    regVal = ptrTOPRCMRegs->WARM_RESET_CONFIG;
    CSL_REG32_WR(regVal, source);

    /* Lock CONTROLSS_CTRL registers */
    SOC_controlModuleLockMMR(SOC_DOMAIN_ID_MAIN, TOP_RCM_PARTITION0);

}

SOC_WarmResetCause SOC_getWarmResetCause(void)
{
    CSL_top_rcmRegs *ptrTOPRCMRegs;
    uint16_t     resetCause = 0U;

    ptrTOPRCMRegs = SOC_rcmGetBaseAddressTOPRCM();

    /* Unlock CONTROLSS_CTRL registers */
    SOC_controlModuleUnlockMMR(SOC_DOMAIN_ID_MAIN, TOP_RCM_PARTITION0);

    /* Read the Reset Cause Register bits */
    resetCause = SOC_rcmExtract16 (ptrTOPRCMRegs->WARM_RST_CAUSE, 11U, 0U);

    /* clear the reset cause */
    CSL_FINS(ptrTOPRCMRegs->WARM_RST_CAUSE_CLR, TOP_RCM_WARM_RST_CAUSE_CLR_CLEAR, 0x7);

    /* Lock CONTROLSS_CTRL registers */
    SOC_controlModuleLockMMR(SOC_DOMAIN_ID_MAIN, TOP_RCM_PARTITION0);

    return (SOC_WarmResetCause) resetCause;
}

void SOC_clearWarmResetCause(void)
{
    CSL_top_rcmRegs *ptrTOPRCMRegs;

    ptrTOPRCMRegs = SOC_rcmGetBaseAddressTOPRCM();

    /* Unlock CONTROLSS_CTRL registers */
    SOC_controlModuleUnlockMMR(SOC_DOMAIN_ID_MAIN, TOP_RCM_PARTITION0);

    CSL_FINS(ptrTOPRCMRegs->WARM_RST_CAUSE_CLR, TOP_RCM_WARM_RST_CAUSE_CLR_CLEAR, 0x7);

    /* Lock CONTROLSS_CTRL registers */
    SOC_controlModuleLockMMR(SOC_DOMAIN_ID_MAIN, TOP_RCM_PARTITION0);

}

void SOC_configureWarmResetOutputDelay(uint16_t opDelayValue)
{
    CSL_top_rcmRegs   *ptrTOPRCMRegs;
    volatile uint32_t *ptrRstTime1Reg;

    ptrTOPRCMRegs = SOC_rcmGetBaseAddressTOPRCM();
    ptrRstTime1Reg   = &(ptrTOPRCMRegs->WARM_RSTTIME1);

    /* Unlock CONTROLSS_CTRL registers */
    SOC_controlModuleUnlockMMR(SOC_DOMAIN_ID_MAIN, TOP_RCM_PARTITION0);

    *ptrRstTime1Reg = SOC_rcmInsert16 (*ptrRstTime1Reg, 11U, 0U, opDelayValue);

    /* Lock CONTROLSS_CTRL registers */
    SOC_controlModuleLockMMR(SOC_DOMAIN_ID_MAIN, TOP_RCM_PARTITION0);
}

void SOC_configureWarmResetInputRiseDelay(uint16_t inpRiseDelayValue)
{
    CSL_top_rcmRegs   *ptrTOPRCMRegs;
    volatile uint32_t *ptrRstTime2Reg;

    ptrTOPRCMRegs = SOC_rcmGetBaseAddressTOPRCM();
    ptrRstTime2Reg   = &(ptrTOPRCMRegs->WARM_RSTTIME2);

    /* Unlock CONTROLSS_CTRL registers */
    SOC_controlModuleUnlockMMR(SOC_DOMAIN_ID_MAIN, TOP_RCM_PARTITION0);

    *ptrRstTime2Reg = SOC_rcmInsert16 (*ptrRstTime2Reg, 11U, 0U, inpRiseDelayValue);

    /* Lock CONTROLSS_CTRL registers */
    SOC_controlModuleLockMMR(SOC_DOMAIN_ID_MAIN, TOP_RCM_PARTITION0);
}

void SOC_configureWarmResetInputFallDelay(uint16_t inpFallDelayValue)
{
    CSL_top_rcmRegs   *ptrTOPRCMRegs;
    volatile uint32_t *ptrRstTime3Reg;

    ptrTOPRCMRegs = SOC_rcmGetBaseAddressTOPRCM();
    ptrRstTime3Reg   = &(ptrTOPRCMRegs->WARM_RSTTIME3);

    /* Unlock CONTROLSS_CTRL registers */
    SOC_controlModuleUnlockMMR(SOC_DOMAIN_ID_MAIN, TOP_RCM_PARTITION0);

    *ptrRstTime3Reg = SOC_rcmInsert16 (*ptrRstTime3Reg, 11U, 0U, inpFallDelayValue);

    /* Lock CONTROLSS_CTRL registers */
    SOC_controlModuleLockMMR(SOC_DOMAIN_ID_MAIN, TOP_RCM_PARTITION0);
}
