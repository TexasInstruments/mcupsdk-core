/*
 *  Copyright (C) 2021 Texas Instruments Incorporated
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

#ifndef SOC_RCM_AWR294X_H_
#define SOC_RCM_AWR294X_H_

#include <stdint.h>
#include <kernel/dpl/SystemP.h>

#ifdef __cplusplus
extern "C"
{
#endif

/**
 *  \defgroup DRV_SOC_RCM_MODULE APIs for SOC Reset and Clock Functions
 *  \ingroup DRV_SOC_MODULE
 *
 * For more details and example usage, see \ref DRIVERS_SOC_PAGE
 *
 *  @{
 */

#define SOC_RCM_FREQ_HZ2MHZ(hz)     ((hz)/(1000000U))
#define SOC_RCM_FREQ_MHZ2HZ(mhz)    ((mhz)*(1000000U))

/** @brief PG version for ES2.0 devices */
#define SOC_RCM_ES2_PG_VER          ((uint8_t)(0x02U))
/** @brief PG version for ES1.0 devices */
#define SOC_RCM_ES1_PG_VER          ((uint8_t)(0x01U))

/** @brief bit mask to specify DSS L2 MEM BANKs, used with SOC_rcmStartMemInitDSSL2(), SOC_rcmWaitMemInitDSSL2 */
#define SOC_RCM_MEMINIT_DSSL2_MEMBANK_VB00    (1U << 0U)
/** @brief bit mask to specify DSS L2 MEM BANKs, used with SOC_rcmStartMemInitDSSL2(), SOC_rcmWaitMemInitDSSL2 */
#define SOC_RCM_MEMINIT_DSSL2_MEMBANK_VB01    (1U << 1U)
/** @brief bit mask to specify DSS L2 MEM BANKs, used with SOC_rcmStartMemInitDSSL2(), SOC_rcmWaitMemInitDSSL2 */
#define SOC_RCM_MEMINIT_DSSL2_MEMBANK_VB10    (1U << 2U)
/** @brief bit mask to specify DSS L2 MEM BANKs, used with SOC_rcmStartMemInitDSSL2(), SOC_rcmWaitMemInitDSSL2 */
#define SOC_RCM_MEMINIT_DSSL2_MEMBANK_VB11    (1U << 3U)
/** @brief bit mask to specify DSS L2 MEM BANKs, used with SOC_rcmStartMemInitDSSL2(), SOC_rcmWaitMemInitDSSL2 */
#define SOC_RCM_MEMINIT_DSSL2_MEMBANK_VB20    (1U << 4U)
/** @brief bit mask to specify DSS L2 MEM BANKs, used with SOC_rcmStartMemInitDSSL2(), SOC_rcmWaitMemInitDSSL2 */
#define SOC_RCM_MEMINIT_DSSL2_MEMBANK_VB21    (1U << 5U)
/** @brief bit mask to specify DSS L2 MEM BANKs, used with SOC_rcmStartMemInitDSSL2(), SOC_rcmWaitMemInitDSSL2 */
#define SOC_RCM_MEMINIT_DSSL2_MEMBANK_VB30    (1U << 6U)
/** @brief bit mask to specify DSS L2 MEM BANKs, used with SOC_rcmStartMemInitDSSL2(), SOC_rcmWaitMemInitDSSL2 */
#define SOC_RCM_MEMINIT_DSSL2_MEMBANK_VB31    (1U << 7U)
/** @brief bit mask to specify DSS L2 MEM BANKs, used with SOC_rcmStartMemInitDSSL2(), SOC_rcmWaitMemInitDSSL2 */
#define SOC_RCM_MEMINIT_DSSL2_MEMBANK_ALL     (SOC_RCM_MEMINIT_DSSL2_MEMBANK_VB00 | \
                                               SOC_RCM_MEMINIT_DSSL2_MEMBANK_VB01 | \
                                               SOC_RCM_MEMINIT_DSSL2_MEMBANK_VB10 | \
                                               SOC_RCM_MEMINIT_DSSL2_MEMBANK_VB11 | \
                                               SOC_RCM_MEMINIT_DSSL2_MEMBANK_VB20 | \
                                               SOC_RCM_MEMINIT_DSSL2_MEMBANK_VB21 | \
                                               SOC_RCM_MEMINIT_DSSL2_MEMBANK_VB30 | \
                                               SOC_RCM_MEMINIT_DSSL2_MEMBANK_VB31)

/** @brief bit mask to specify DSS L2 MEM BANKs, used with SOC_rcmStartMemInitDSSL3(), SOC_rcmWaitMemInitDSSL2 */
#define SOC_RCM_MEMINIT_DSSL3_MEMBANK_RAM0    (1U << 0U)
/** @brief bit mask to specify DSS L2 MEM BANKs, used with SOC_rcmStartMemInitDSSL3(), SOC_rcmWaitMemInitDSSL2 */
#define SOC_RCM_MEMINIT_DSSL3_MEMBANK_RAM1    (1U << 1U)
/** @brief bit mask to specify DSS L2 MEM BANKs, used with SOC_rcmStartMemInitDSSL3(), SOC_rcmWaitMemInitDSSL2 */
#define SOC_RCM_MEMINIT_DSSL3_MEMBANK_RAM2    (1U << 2U)
/** @brief bit mask to specify DSS L2 MEM BANKs, used with SOC_rcmStartMemInitDSSL3(), SOC_rcmWaitMemInitDSSL2 */
#define SOC_RCM_MEMINIT_DSSL3_MEMBANK_RAM3    (1U << 3U)
/** @brief bit mask to specify DSS L2 MEM BANKs, used with SOC_rcmStartMemInitDSSL3(), SOC_rcmWaitMemInitDSSL2 */
#define SOC_RCM_MEMINIT_DSSL3_MEMBANK_ALL     (SOC_RCM_MEMINIT_DSSL3_MEMBANK_RAM0 | \
                                               SOC_RCM_MEMINIT_DSSL3_MEMBANK_RAM1 | \
                                               SOC_RCM_MEMINIT_DSSL3_MEMBANK_RAM2 | \
                                               SOC_RCM_MEMINIT_DSSL3_MEMBANK_RAM3)

/** @brief HSDIV output enable bitmask, used with \ref SOC_RcmPllHsDivOutConfig */
#define SOC_RCM_PLL_HSDIV_OUTPUT_ENABLE_0      (1U << 0U)
/** @brief HSDIV output enable bitmask, used with \ref SOC_RcmPllHsDivOutConfig */
#define SOC_RCM_PLL_HSDIV_OUTPUT_ENABLE_1      (1U << 1U)
/** @brief HSDIV output enable bitmask, used with \ref SOC_RcmPllHsDivOutConfig */
#define SOC_RCM_PLL_HSDIV_OUTPUT_ENABLE_2      (1U << 2U)
/** @brief HSDIV output enable bitmask, used with \ref SOC_RcmPllHsDivOutConfig */
#define SOC_RCM_PLL_HSDIV_OUTPUT_ENABLE_3      (1U << 3U)
/** @brief HSDIV output enable bitmask, used with \ref SOC_RcmPllHsDivOutConfig */
#define SOC_RCM_PLL_HSDIV_OUTPUT_ENABLE_ALL    (SOC_RCM_PLL_HSDIV_OUTPUT_ENABLE_0   | \
                                                SOC_RCM_PLL_HSDIV_OUTPUT_ENABLE_1   | \
                                                SOC_RCM_PLL_HSDIV_OUTPUT_ENABLE_2   | \
                                                SOC_RCM_PLL_HSDIV_OUTPUT_ENABLE_3)

#define SOC_RCM_PLL_HSDIV_OUTPUT_IDX0                                            (0)
#define SOC_RCM_PLL_HSDIV_OUTPUT_IDX1                                            (1)
#define SOC_RCM_PLL_HSDIV_OUTPUT_IDX2                                            (2)
#define SOC_RCM_PLL_HSDIV_OUTPUT_IDX3                                            (3)
#define SOC_RCM_PLL_HSDIV_OUTPUT_COUNT               (SOC_RCM_PLL_HSDIV_OUTPUT_IDX3 + 1)


#define SOC_RCM_EFUSE_R5_CLK_300_SYS_CLK_150MHz                               (0x0U)
#define SOC_RCM_EFUSE_R5_CLK_200_SYS_CLK_200MHz                               (0x5U)
#define SOC_RCM_EFUSE_R5_CLK_400_SYS_CLK_200MHz                               (0xAU)

/**
 * @brief Reset Causes
 */
typedef enum SOC_RcmResetCause_e
{
    /**
     * @brief   Value specifying Power ON Reset
     */
    SOC_RcmResetCause_POWER_ON_RESET = 0x0U,
    /**
     * @brief   Value specifying Warm Reset
     */
    SOC_RcmResetCause_WARM_RESET = 0x1U,
    /**
     * @brief   Value specifying STC Reset
     */
    SOC_RcmResetCause_STC_RESET = 0x2U,
    /**
     * @brief   Value specifying R5 Core A Subsytem Reset
     */
    SOC_RcmResetCause_MMR_CPU0_VIM0_RESET = 0x3U,
    /**
     * @brief   Value specifying R5 Core B Subsytem Reset
     */
    SOC_RcmResetCause_MMR_CPU1_VIM1_RESET = 0x4U,
    /**
     * @brief   Value specifying R5 Core A (core only) Reset
     */
    SOC_RcmResetCause_MMR_CPU0_RESET = 0x5U,
    /**
     * @brief   Value specifying R5 Core B (core only) Reset
     */
    SOC_RcmResetCause_MMR_CPU1_RESET = 0x6U,
    /**
     * @brief   Value specifying R5 Core A Debug Reset
     */
    SOC_RcmResetCause_DBG_CPU0_RESET = 0x7U,
    /**
     * @brief   Value specifying R5 Core B Debug Reset
     */
    SOC_RcmResetCause_DBG_CPU1_RESET = 0x8U,
    /**
     * @brief   Value specifying R5 Reset due to FSM Trigger
     */
    SOC_RcmResetCause_FSM_TRIGGER_RESET = 0x9U,
    /**
     * @brief   Value specifying R5 Reset due to Unknown reason
     */
    SOC_RcmResetCause_RST_CAUSE_UNKNOWN = 0xAU,
    /**
     * @brief max value
     */
    SOC_RcmResetCause_MAX_VALUE = 0xFFFFFFFFu,

} SOC_RcmResetCause;

/**
 *  \anchor SOC_WarmResetCause_t
 *  \name SOC Warm Reset Causes
 *  @{
 */
typedef enum SOC_WarmResetCause_e
{
    /**
     * \brief Value specifying Power ON Reset
     */
    SOC_WarmResetCause_POWER_ON_RESET = 0x09U,
    /**
     * \brief Value specifying MSS WDT
     */
    SOC_WarmResetCause_MSS_WDT = 0x0AU,
    /**
     * \brief Value specifying Software Warm Reset
     */
    SOC_WarmResetCause_TOP_RCM_WARM_RESET_CONFIG = 0x0CU,
    /**
     * \brief Value specifying External Pad Reset
     */
    SOC_WarmResetCause_EXT_PAD_RESET = 0x08U,
    /**
     * \brief Value specifying HSM WDT
     */
    SOC_WarmResetCause_HSM_WDT = 0x18U,

}SOC_WarmResetCause;
/** @} */

/**
 * @brief Peripheral IDs
 */
typedef enum SOC_RcmPeripheralId_e
{
    SOC_RcmPeripheralId_CSIRX,
    SOC_RcmPeripheralId_MSS_RTIA,
    SOC_RcmPeripheralId_MSS_RTIB,
    SOC_RcmPeripheralId_MSS_RTIC,
    SOC_RcmPeripheralId_MSS_WDT,
    SOC_RcmPeripheralId_MSS_QSPI,
    SOC_RcmPeripheralId_MSS_SPIA,
    SOC_RcmPeripheralId_MSS_SPIB,
    SOC_RcmPeripheralId_MSS_I2C,
    SOC_RcmPeripheralId_MSS_SCIA,
    SOC_RcmPeripheralId_MSS_SCIB,
    SOC_RcmPeripheralId_MSS_MCANA,
    SOC_RcmPeripheralId_MSS_MCANB,
    SOC_RcmPeripheralId_MSS_CPTS,
    SOC_RcmPeripheralId_MSS_CPSW,
    SOC_RcmPeripheralId_DSS_HWA,
    SOC_RcmPeripheralId_DSS_RTIA,
    SOC_RcmPeripheralId_DSS_RTIB,
    SOC_RcmPeripheralId_DSS_WDT,
    SOC_RcmPeripheralId_DSS_SCIA,
} SOC_RcmPeripheralId;

/**
 * @brief Peripheral Clock Sources
 */
typedef enum SOC_RcmPeripheralClockSource_e
{
    SOC_RcmPeripheralClockSource_DPLL_CORE_HSDIV0_CLKOUT1,
    SOC_RcmPeripheralClockSource_DPLL_CORE_HSDIV0_CLKOUT2,
    SOC_RcmPeripheralClockSource_DPLL_DSP_HSDIV0_CLKOUT2,
    SOC_RcmPeripheralClockSource_DPLL_PER_HSDIV0_CLKOUT1,
    SOC_RcmPeripheralClockSource_DPLL_PER_HSDIV0_CLKOUT2,
    SOC_RcmPeripheralClockSource_FE1_REF_CLK,
    SOC_RcmPeripheralClockSource_RC_CLK_10M,
    SOC_RcmPeripheralClockSource_RCCLK32K,
    SOC_RcmPeripheralClockSource_SYS_CLK,
    SOC_RcmPeripheralClockSource_WUCPU_CLK,
    SOC_RcmPeripheralClockSource_XREF_CLK0,
    SOC_RcmPeripheralClockSource_XREF_CLK1,
    SOC_RcmPeripheralClockSource_XTAL_CLK,
} SOC_RcmPeripheralClockSource;

/**
 * @brief DSP Clock Sources
 */
typedef enum SOC_RcmDspClockSource_e
{
    SOC_RcmDspClockSource_XTAL_CLK,
    SOC_RcmDspClockSource_RC_CLK_10M,
    SOC_RcmDspClockSource_DPLL_DSP_HSDIV0_CLKOUT1,
    SOC_RcmDspClockSource_DPLL_DSP_HSDIV0_CLKOUT1_DITH,
    SOC_RcmDspClockSource_DPLL_CORE_HSDIV0_CLKOUT1,
    SOC_RcmDspClockSource_DPLL_PER_HSDIV0_CLKOUT3,
    /**
     * @brief max value
     */
    SOC_RcmDspClockSource_MAX_VALUE = 0xFFFFFFFFu,

} SOC_RcmDspClockSource;

/**
 * @brief R5 Clock Sources
 */
typedef enum SOC_RcmR5ClockSource_e
{
    SOC_RcmR5ClockSource_XTAL_CLK,
    SOC_RcmR5ClockSource_RC_CLK_10M,
    SOC_RcmR5ClockSource_DPLL_CORE_HSDIV0_CLKOUT2,
    SOC_RcmR5ClockSource_SYS_CLK,
    SOC_RcmR5ClockSource_WUCPU_CLK,
    SOC_RcmR5ClockSource_MAX_VALUE = 0xFFFFFFFFu,
} SOC_RcmR5ClockSource;

/**
 * @brief PLL Fout values
 */
typedef enum SOC_RcmPllFoutFreqId_e
{
    SOC_RcmPllFoutFreqId_CLK_800MHZ,
    SOC_RcmPllFoutFreqId_CLK_2000MHZ,
    SOC_RcmPllFoutFreqId_CLK_900MHZ,
    SOC_RcmPllFoutFreqId_CLK_1800MHZ,
    SOC_RcmPllFoutFreqId_CLK_1920MHZ,
    SOC_RcmPllFoutFreqId_CLK_1699p21875MHZ,
    SOC_RcmPllFoutFreqId_CLK_1728MHZ,
    SOC_RcmPllFoutFreqId_CLK_1966p08MHZ,
    SOC_RcmPllFoutFreqId_CLK_1806p336MHZ,
    SOC_RcmPllFoutFreqId_CLK_600MHZ,
    SOC_RcmPllFoutFreqId_CLK_720MHZ,
    SOC_RcmPllFoutFreqId_CLK_1500MHZ,
    /**
     * @brief max value
     */
    SOC_RcmPllFoutFreqId_MAX_VALUE = 0xFFFFFFFFu,
} SOC_RcmPllFoutFreqId;

/**
 * @brief HSI Clock Sources
 */
typedef enum SOC_RcmHSIClockSource_e
{
    SOC_RcmHSIClockSource_PLL_CORE_CLK,
    SOC_RcmHSIClockSource_APLL_CLK_HSI,
    SOC_RcmHSIClockSource_APLL_1p8GHZ,
    SOC_RcmHSIClockSource_PLL_PER_CLK,
    SOC_RcmHSIClockSource_DPLL_CORE_HSDIV0_CLKOUT0,
    SOC_RcmHSIClockSource_RC_CLK_10M,
    SOC_RcmHSIClockSource_DPLL_DSP_HSDIV0_CLKOUT0,
    SOC_RcmHSIClockSource_DPLL_PER_HSDIV0_CLKOUT0,
} SOC_RcmHSIClockSource;

/**
 * @brief APLL IDs
 */
typedef enum SOC_RcmApllId_e
{
    SOC_RcmAPLLID_1P2G,
    SOC_RcmAPLLID_1P8G,
} SOC_RcmApllId;

/**
 * @brief RSS BSS Frc Clock Sources
 */
typedef enum SOC_RcmRssBssFrcClockSource_e
{
    SOC_RcmRssBssFrcClockSource_XTAL_CLK,
    SOC_RcmRssBssFrcClockSource_WUCPU_CLK,
    SOC_RcmRssBssFrcClockSource_SYS_CLK,
    SOC_RcmRssBssFrcClockSource_DPLL_PER_HSDIV0_CLKOUT1,
    SOC_RcmRssBssFrcClockSource_APLL_1p8G_HSDIV0_CLKOUT2,
    SOC_RcmRssBssFrcClockSource_RC_CLK_10M,
    SOC_RcmRssBssFrcClockSource_XREF_CLK0,
    SOC_RcmRssBssFrcClockSource_RCCLK32K,
} SOC_RcmRssBssFrcClockSource;

/**
 * @brief HS Divider Clock Out Mux Clock Sources
 */
typedef enum SOC_RcmHSDIVClkOutMuxClockSource_e
{
    SOC_RcmHSDIVClkOutMuxClockSource_DPLL_CORE_HSDIV0_CLKOUT2_PreMux,
    SOC_RcmHSDIVClkOutMuxClockSource_DPLL_DSP_HSDIV0_CLKOUT1_PreMux,
    SOC_RcmHSDIVClkOutMuxClockSource_DPLL_DSP_HSDIV0_CLKOUT2_PreMux,
    SOC_RcmHSDIVClkOutMuxClockSource_DPLL_PER_HSDIV0_CLKOUT1_PreMux,
    SOC_RcmHSDIVClkOutMuxClockSource_APLL_1p2G_HSDIV0_CLKOUT0,
    SOC_RcmHSDIVClkOutMuxClockSource_APLL_1p2G_HSDIV0_CLKOUT1,
    SOC_RcmHSDIVClkOutMuxClockSource_APLL_1p2G_HSDIV0_CLKOUT2,
    SOC_RcmHSDIVClkOutMuxClockSource_APLL_1p2G_HSDIV0_CLKOUT3,
    SOC_RcmHSDIVClkOutMuxClockSource_APLL_1p8G_HSDIV0_CLKOUT0,
    SOC_RcmHSDIVClkOutMuxClockSource_APLL_1p8G_HSDIV0_CLKOUT1,
    SOC_RcmHSDIVClkOutMuxClockSource_APLL_400MHZ,
} SOC_RcmHSDIVClkOutMuxClockSource;

/**
 * @brief HS Divider Clock Out Mux ID
 */
typedef enum SOC_RcmHSDIVClkOutMuxId_e
{
    SOC_RcmHSDIVClkOutMuxId_DPLL_CORE_OUT2,
    SOC_RcmHSDIVClkOutMuxId_DPLL_DSP_OUT1,
    SOC_RcmHSDIVClkOutMuxId_DPLL_DSP_OUT2,
    SOC_RcmHSDIVClkOutMuxId_DPLL_PER_OUT1,
} SOC_RcmHSDIVClkOutMuxId;

/**
 * @brief RSS Clock Sources
 */
typedef enum SOC_RcmRssClkSrcId_e
{
    SOC_RcmRssClkSrcId_WUCPU_CLK,
    SOC_RcmRssClkSrcId_XTAL_CLK,
    SOC_RcmRssClkSrcId_DPLL_CORE_HSDIV0_CLKOUT2_MUXED,
    SOC_RcmRssClkSrcId_DPLL_PER_HSDIV0_CLKOUT1_MUXED,
    SOC_RcmRssClkSrcId_APLL_1P8_HSDIV0_CLKOUT2,
    SOC_RcmRssClkSrcId_RC_CLK_10M,
    SOC_RcmRssClkSrcId_SYS_CLK,
} SOC_RcmRssClkSrcId;

/**
 * @brief QSPI frequency values
 */
typedef enum SOC_RcmQspiClockFreqId_e {
    /**
     * @brief   Value specifying QSPI clock of 40 Mhz
     */
    SOC_RcmQspiClockFreqId_CLK_40MHZ = 0x0,
    /**
     * @brief   Value specifying QSPI clock of 60 Mhz
     */
    SOC_RcmQspiClockFreqId_CLK_60MHZ = 0x1,
    /**
     * @brief   Value specifying QSPI clock of 80 Mhz
     */
    SOC_RcmQspiClockFreqId_CLK_80MHZ = 0x2,
    /**
     * @brief max value
     */
    SOC_RcmQspiClockFreqId_MAX_VALUE = 0xFFFFFFFFu,

} SOC_RcmQspiClockFreqId;

/**
 * @brief   Efuse value for flash clock mode
 */
typedef enum SOC_RcmEfuseFlashClkModeId_e {

    /**
     * @brief Phase 0 : Polarity 0
     */
    SOC_RcmEfuseFlashClkModeId_0 = 0x0,
    /**
     * @brief Phase 1 : Polarity 1
     */
    SOC_RcmEfuseFlashClkModeId_3 = 0x3,
    /**
     * @brief max value
     */
    SOC_RcmEfuseFlashClkModeId_MAX_VALUE = 0xFFFFFFFFu,
} SOC_RcmEfuseFlashClkModeId;

/**
 * @brief Structure to specific PLL HS divider output frequencies
 */
typedef struct SOC_RcmPllHsDivOutConfig_s
{
    uint32_t hsdivOutEnMask; /**< HS divider output enable bit mark */
    uint32_t hsDivOutFreqHz[SOC_RCM_PLL_HSDIV_OUTPUT_COUNT]; /**< HS divider output frequencies */
} SOC_RcmPllHsDivOutConfig;

/**
 * @brief Efuse value for QSPI config
 */
typedef struct SOC_RcmEfuseQspiConfig_s
{
    /**
     * @brief Efuse value for flash clock mode
     */
    SOC_RcmEfuseFlashClkModeId flashClockModeId;
    /**
     * @brief   Efuse value for QSPI clock frequency
     */
    SOC_RcmQspiClockFreqId  qspiClockFreqId;
} SOC_RcmEfuseQspiConfig;

typedef struct SOC_RcmEfuseBootFreqConfig_s
{
    uint32_t           r5FreqHz;
    uint32_t           sysClkFreqHz;
} SOC_RcmEfuseBootFreqConfig;

typedef struct SOC_RcmDeviceFreqConfig_s
{
    uint32_t           dspFreqHz;
    uint32_t           r5FreqHz;
    uint32_t           sysClkFreqHz;
} SOC_RcmDeviceFreqConfig;

typedef enum SOC_RcmEfusePkgType_e
{
    SOC_RCM_EFUSE_DEVICE_PKG_TYPE_ETS
} SOC_RcmEfusePkgType;

/**
 *  \brief Set CORE PLL Config
 *
 * \param outFreqId [in] Output frequency form the PLL
 * \param hsDivCfg [in] Set HS divider output frequencies
 */
void SOC_rcmCoreDpllConfig(SOC_RcmPllFoutFreqId outFreqId, SOC_RcmPllHsDivOutConfig *hsDivCfg);

/**
 *  \brief Set DSP PLL Config
 *
 * \param outFreqId [in] Output frequency form the PLL
 * \param hsDivCfg [in] Set HS divider output frequencies
 */
void SOC_rcmDspDpllConfig(SOC_RcmPllFoutFreqId outFreqId, SOC_RcmPllHsDivOutConfig *hsDivCfg);

/**
 *  \brief Set Peripheral PLL Config
 *
 * \param outFreqId [in] Output frequency form the PLL
 * \param hsDivCfg [in] Set HS divider output frequencies
 */
void SOC_rcmPerDpllConfig(SOC_RcmPllFoutFreqId outFreqId, SOC_RcmPllHsDivOutConfig *hsDivCfg);

/**
 *  \brief Set PLL HS Div Config
 *
 * \param apllId [in] APLL ID
 * \param hsDivCfg [in] Set HS divider output frequencies
 */
void SOC_rcmApllHSDivConfig(SOC_RcmApllId apllId, SOC_RcmPllHsDivOutConfig *hsDivCfg);

/**
 *  \brief Disable PLL HS Div clock output
 *
 * \param apllId [in] APLL ID
 * \param hsDivIdx [in] Hs Divider clock index
 */
void SOC_rcmApllHSDivDisableOutput(SOC_RcmApllId apllId, uint32_t  hsDivIdx);

/**
 *  \brief Set R5 and SycClk frequency
 *
 * \param r5FreqHz [in] R5 frequency, in Hz
 * \param sysClkFreqHz [in] SysClk frequency, in Hz
 *
 * \return SystemP_SUCCESS on success, else failure
 *
 */
int32_t SOC_rcmSetR5Clock(uint32_t r5FreqHz, uint32_t sysClkFreqHz);

/**
 *  \brief Get R5 frequency
 *
 * \return R5 frequency, in Hz
 */
uint32_t SOC_rcmGetR5Clock(void);

/**
 *  \brief Set DSP frequency
 *
 * \param clkSource [in] DSP clock source to use
 * \param freqHz [in] DSP frequency, in Hz
 *
 * \return SystemP_SUCCESS on success, else failure
 */
int32_t SOC_rcmSetDspClock(SOC_RcmDspClockSource clkSource, uint32_t freqHz);

/**
 *  \brief Get DSP frequency
 *
 * \return DSP frequency, in Hz
 */
uint32_t SOC_rcmGetDspClock(void);

/**
 *  \brief Set peripheral frequency
 *
 * \param periphId [in] Peripheral ID
 * \param clkSource [in] Peripheral clock source to use
 * \param freqHz [in] Peripheral frequency, in Hz
 *
 * \return SystemP_SUCCESS on success, else failure
 */
int32_t SOC_rcmSetPeripheralClock(SOC_RcmPeripheralId periphId, SOC_RcmPeripheralClockSource clkSource, uint32_t freqHz);

/**
 *  \brief Get peripheral frequency
 *
 *  \param periphId [in] Peripheral ID
 *
 *  \return Peripheral frequency, in Hz
 */
uint32_t SOC_rcmGetPeripheralClock(SOC_RcmPeripheralId periphId);

/**
 *  \brief Set HSI frequency
 *
 * \param clkSource [in] HSI clock source to use
 * \param freqHz [in] Peripheral frequency, in Hz
 *
 * \return SystemP_SUCCESS on success, else failure
 */
int32_t SOC_rcmSetHSIClock(SOC_RcmHSIClockSource clkSource, uint32_t freqHz);

/**
 *  \brief Set RSS BSS FRC Clock
 *
 * \param clkSource [in] Clock source to use
 * \param freqHz [in] Peripheral frequency, in Hz
 *
 * \return SystemP_SUCCESS on success, else failure
 */
int32_t SOC_rcmSetRssBssFrcClock(SOC_RcmRssBssFrcClockSource clkSource, uint32_t freqHz);

/**
 *  \brief Get SOC reset cause
 *
 * \return SOC reset cause
 */
SOC_RcmResetCause SOC_rcmGetResetCause(void);

/**
 *  \brief Start memory initialization for R5 TCMA
 */
void SOC_rcmStartMemInitTCMA(void);

/**
 *  \brief Wait memory initialization to complete for R5 TCMA
 */
void SOC_rcmWaitMemInitTCMA(void);

/**
 *  \brief Start memory initialization for R5 TCMB
 */
void SOC_rcmStartMemInitTCMB(void);

/**
 *  \brief Wait memory initialization to complete for R5 TCMB
 */
void SOC_rcmWaitMemInitTCMB(void);

/**
 *  \brief Start memory initialization for TCMBSS
 */
void SOC_rcmStartMeminitTCMBSS(void);

/**
 *  \brief Wait memory initialization to complete for TCMBSS
 */
void SOC_rcmWaitMeminitTCMBSS(void);

/**
 *  \brief Initialize the MSS mailbox memory
 */
void SOC_rcmMemInitMssMailboxMemory(void);

/**
 *  \brief Initialize the DSS mailbox memory
 */
void SOC_rcmMemInitDssMailboxMemory(void);

/**
 *  \brief Start memory initialization for Static BSS
 */
void SOC_rcmStartMeminitStaticBSS(void);

/**
 *  \brief Wait memory initialization to complete for Static BSS
 */
void SOC_rcmWaitMeminitStaticBSS(void);

/**
 *  \brief Start memory initialization for Shared BSS
 */
void SOC_rcmStartMeminitSharedBSS(void);

/**
 *  \brief Wait memory initialization to complete for Shared BSS
 */
void SOC_rcmWaitMeminitSharedBSS(void);

/**
 *  \brief Start memory initialization for L2
 */
void SOC_rcmStartMeminitL2(void);

/**
 *  \brief Wait memory initialization to complete for L2
 */
void SOC_rcmWaitMeminitL2(void);

/**
 *  \brief Start memory initialization for DSS L2
 */
void SOC_rcmStartMemInitDSSL2(uint32_t l2bankMask);

/**
 *  \brief Wait memory initialization to complete for DSS L2
 */
void SOC_rcmWaitMemInitDSSL2(uint32_t l2bankMask);

/**
 *  \brief Start memory initialization for DSS L3
 */
void SOC_rcmStartMemInitDSSL3(uint32_t l3bankMask);

/**
 *  \brief Wait memory initialization to complete for DSS L3
 */
void SOC_rcmWaitMemInitDSSL3(uint32_t l3bankMask);

/**
 *  \brief Get QSPI Efuse configuration
 *
 * \param qspiEfuseCfg [out] QSPI Efuse configuration
 */
void SOC_rcmGetEfuseQspiConfig(SOC_RcmEfuseQspiConfig *qspiEfuseCfg);

/**
 *  \brief Reset Dsp Core
 */
void SOC_rcmDspPowerOnReset(void);

/**
 *  \brief Reset R5 Core
 */
void SOC_rcmR5PowerOnReset(void);

/**
 *  \brief Configure R5 in lock step mode
 */
void SOC_rcmR5ConfigLockStep(void);

/**
 *  \brief Configure R5 in dual core mode
 */
void SOC_rcmR5ConfigDualCore(void);

/**
 *  \brief Trigger R5 core reset
 */
void SOC_rcmR5TriggerReset(void);

/**
 *  \brief Unhalt R5 core 1
 */
void SOC_rcmCr5bUnhalt(void);

/**
 *  \brief Unhalt C66x Core
 */
void SOC_rcmC66xStart(void);

/**
 *  \brief Set CORE PLL Hs Div Config
 *
 * \param hsDivCfg [in] Set HS divider output frequencies
 */
void SOC_rcmCoreApllHSDivConfig(SOC_RcmPllHsDivOutConfig *hsDivCfg);

/**
 *  \brief Set DSP PLL Config
 *
 * \param outFreqId [in] Output frequency form the PLL
 * \param hsDivCfg [in] Set HS divider output frequencies
 */
void SOC_rcmDspPllConfig(SOC_RcmPllFoutFreqId outFreqId, SOC_RcmPllHsDivOutConfig *hsDivCfg);

/**
 *  \brief Set Peripheral PLL Config
 *
 * \param outFreqId [in] Output frequency form the PLL
 * \param hsDivCfg [in] Set HS divider output frequencies
 */
void SOC_rcmPerPllConfig(SOC_RcmPllFoutFreqId outFreqId, SOC_RcmPllHsDivOutConfig *hsDivCfg);

/**
 * \brief Return R5SS status operating in lockstep or dual core mode.
 * \param r5fClusterGroupId [in] R5F Cluster Group Id.
 *        Refer \ref CSL_ArmR5ClusterGroupID for applicable values.
 */
uint32_t SOC_rcmIsR5FInLockStepMode(uint32_t r5fClusterGroupId);

/**
 * \brief Generate SW WARM reset
 */
void SOC_generateSwWarmReset(void);

/**
 * \brief Returns cause of WARM reset
 *
 * \return cause of WARM reset.
 */
SOC_WarmResetCause SOC_getWarmResetCause(void);

/**
 * \brief Clear Reset Cause register
 */
void SOC_clearWarmResetCause(void);

/**
 *  \brief Get PG version from Efuse memory
 *
 * \return SOC_RCM_ES1_PG_VER (0x1) for ES1.0 devices
 *         SOC_RCM_ES2_PG_VER (0x2) for ES2.0 devices
 */
uint8_t SOC_rcmGetEfusePGVer(void);

/**
 * \brief Disable TOP PBIST
 */
void SOC_rcmDisableTopPbist(void);

/**
 * \brief Switches R5 clock to XTAL
 */
void SOC_rcmSwitchR5Clock(SOC_RcmR5ClockSource clkSrc, uint32_t divVal);

/*
 *  Misc functions
 */
void SOC_rcmBSSControl(void);
void SOC_rcmPopulateBSSControl(void);
void SOC_rcmBSSR4Unhalt(void);
void SOC_rcmWaitBSSBootComplete(void);
int32_t SOC_rcmSetHSDivMux(SOC_RcmHSDIVClkOutMuxId clkOutMuxId,
                           SOC_RcmHSDIVClkOutMuxClockSource muxSource);
int32_t SOC_rcmSetRssClkFreq(SOC_RcmRssClkSrcId rssClkSrcId, uint32_t freqHz);
void SOC_rcmGetEfuseBootFrequency(SOC_RcmEfuseBootFreqConfig *bootFreqEfuseCfg);
uint32_t SOC_rcmIsDualCoreSwitchSupported(void);
void SOC_rcmGetDeviceFrequency(SOC_RcmDeviceFreqConfig *deviceFreqEfuseCfg);
void SOC_rcmGetPackageType(SOC_RcmEfusePkgType *deviceTypeEfuse);
void SOC_rcmConfigEthMacIf(void);
void SOC_rcmCoreDpllHSDivOutEnable(uint32_t hsDivOutIdx, uint32_t divVal);
void SOC_rcmCoreDpllDisable(void);
void SOC_rcmDspDpllDisable(void);
void SOC_rcmPerDpllDisable(void);
int32_t SOC_rcmGetRssClkFreq(uint32_t *freqHz);
void SOC_rcmSetFastchargeBias(void);

/** @} */

#ifdef __cplusplus
}
#endif

#endif
