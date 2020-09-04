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

#ifndef SOC_RCM_AM273X_H_
#define SOC_RCM_AM273X_H_

#include <stdint.h>

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

#include <kernel/dpl/SystemP.h>

#define SOC_RCM_FREQ_HZ2MHZ(hz)     ((hz)/(1000000U))
#define SOC_RCM_FREQ_MHZ2HZ(mhz)    ((mhz)*(1000000U))

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

/* @brief HSDIV output enable bitmask, used with \ref SOC_RcmPllHsDivOutConfig */
#define SOC_RCM_PLL_HSDIV_OUTPUT_COUNT         (4U)

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
 * @brief Peripheral IDs
 */
typedef enum SOC_RcmPeripheralId_e
{
    /**
     * @brief   Value specifying CSI RX
     */
    SOC_RcmPeripheralId_CSIRX,
    /**
     * @brief   Value specifying MCANA
     */
    SOC_RcmPeripheralId_MSS_MCANA,
    /**
     * @brief   Value specifying MCANB
     */
    SOC_RcmPeripheralId_MSS_MCANB,
    /**
     * @brief   Value specifying QSPI (Quad SPI)
     */
    SOC_RcmPeripheralId_MSS_QSPI,
    /**
     * @brief   Value specifying MSS RTIA (Timer)
     */
    SOC_RcmPeripheralId_MSS_RTIA,
    /**
     * @brief   Value specifying MSS RTIB (Timer)
     */
    SOC_RcmPeripheralId_MSS_RTIB,
    /**
     * @brief   Value specifying MSS RTIC (Timer)
     */
    SOC_RcmPeripheralId_MSS_RTIC,
    /**
     * @brief   Value specifying MSS WatchDog
     */
    SOC_RcmPeripheralId_MSS_WDT,
    /**
     * @brief   Value specifying MSS SPI-A
     */
    SOC_RcmPeripheralId_MSS_SPIA,
    /**
     * @brief   Value specifying MSS SPI-B
     */
    SOC_RcmPeripheralId_MSS_SPIB,
    /**
     * @brief   Value specifying MSS I2C
     */
    SOC_RcmPeripheralId_MSS_I2C,
    /**
     * @brief   Value specifying MSS SCI-A (UART)
     */
    SOC_RcmPeripheralId_MSS_SCIA,
    /**
     * @brief   Value specifying MSS SCI-B (UART)
     */
    SOC_RcmPeripheralId_MSS_SCIB,
    /**
     * @brief   Value specifying CPTS (Timesync module)
     */
    SOC_RcmPeripheralId_MSS_CPTS,
    /**
     * @brief   Value specifying CPSW (2 port ethernet switch)
     */
    SOC_RcmPeripheralId_MSS_CPSW,
    /**
     * @brief   Value specifying DSS RTIA (Timer)
     */
    SOC_RcmPeripheralId_DSS_RTIA,
    /**
     * @brief   Value specifying DSS RTIB (Timer)
     */
    SOC_RcmPeripheralId_DSS_RTIB,
    /**
     * @brief   Value specifying DSS WatchDog
     */
    SOC_RcmPeripheralId_DSS_WDT,
    /**
     * @brief   Value specifying DSS SCI-A (UART)
     */
    SOC_RcmPeripheralId_DSS_SCIA,
    /**
     * @brief   Value specifying RCSS I2CA
     */
    SOC_RcmPeripheralId_RCSS_I2CA,
    /**
     * @brief   Value specifying RCSS I2CB
     */
    SOC_RcmPeripheralId_RCSS_I2CB,
    /**
     * @brief   Value specifying RCSS SCIA
     */
    SOC_RcmPeripheralId_RCSS_SCIA,
    /**
     * @brief   Value specifying RCSS SPIA
     */
    SOC_RcmPeripheralId_RCSS_SPIA,
    /**
     * @brief   Value specifying RCSS SCIB
     */
    SOC_RcmPeripheralId_RCSS_SPIB,
    /**
     * @brief   Value specifying RCSS ATL
     */
    SOC_RcmPeripheralId_RCSS_ATL,
    /**
     * @brief   Value specifying RCSS MCASPA AUX
     */
    SOC_RcmPeripheralId_RCSS_MCASPA_AUX,
    /**
     * @brief   Value specifying RCSS MCASPB AUX
     */
    SOC_RcmPeripheralId_RCSS_MCASPB_AUX,
    /**
     * @brief   Value specifying RCSS MCASPC AUX
     */
    SOC_RcmPeripheralId_RCSS_MCASPC_AUX,
    /**
     * @brief max value
     */
    SOC_RcmPeripheralId_MAX_VALUE = 0xFFFFFFFFu,

} SOC_RcmPeripheralId;

/**
 * @brief Peripheral Clock Sources
 */
typedef enum SOC_RcmPeripheralClockSource_e
{
    /**
     * @brief   Value specifying Crystal Clock
     */
    SOC_RcmPeripheralClockSource_XTAL_CLK,
    /**
     * @brief   Value specifying System Clock (200Mhz)
     */
    SOC_RcmPeripheralClockSource_SYS_CLK,
    /**
     * @brief   Value specifying PLL Core Clock Out 1 (400 Mhz)
     */
    SOC_RcmPeripheralClockSource_DPLL_CORE_HSDIV0_CLKOUT1,
    /**
     * @brief   Value specifying PLL Core Clock Out 2 (400 Mhz)
     */
    SOC_RcmPeripheralClockSource_DPLL_CORE_HSDIV0_CLKOUT2,
    /**
     * @brief   Value specifying PLL Core Clock Out 1 (192 Mhz)
     */
    SOC_RcmPeripheralClockSource_DPLL_PER_HSDIV0_CLKOUT1,
    /**
     * @brief   Value specifying PLL Core Clock Out 2 (96 Mhz)
     */
    SOC_RcmPeripheralClockSource_DPLL_PER_HSDIV0_CLKOUT2,
    /**
     * @brief   Value specifying PLL Core Clock Out 3 (172.8 Mhz)
     */
    SOC_RcmPeripheralClockSource_DPLL_PER_HSDIV0_CLKOUT3,
    /**
     * @brief   Value specifying Analog Wakeup CPU Clock
     */
    SOC_RcmPeripheralClockSource_WUCPU_CLK,
    /**
     * @brief   Value specifying XREF_CLK0 Clock (From IO)
     */
    SOC_RcmPeripheralClockSource_XREF_CLK0,
    /**
     * @brief   Value specifying XREF_CLK1 Clock (From IO)
     */
    SOC_RcmPeripheralClockSource_XREF_CLK1,
    /**
     * @brief max value
     */
    SOC_RcmPeripheralClockSource_MAX_VALUE = 0xFFFFFFFFu,

} SOC_RcmPeripheralClockSource;

/**
 * @brief DSP Clock Sources
 */
typedef enum SOC_RcmDspClockSource_e
{
    /**
     * @brief   Value specifying Crystal Clock
     */
    SOC_RcmDspClockSource_XTAL_CLK,
    /**
     * @brief   Value specifying PLL DSP Clock Out 2 (450 Mhz)
     */
    SOC_RcmDspClockSource_DPLL_DSP_HSDIV0_CLKOUT1,
    /**
     * @brief   Value specifying PLL Core Clock Out 1
     */
    SOC_RcmDspClockSource_DPLL_CORE_HSDIV0_CLKOUT1,
    /**
     * @brief max value
     */
    SOC_RcmDspClockSource_MAX_VALUE = 0xFFFFFFFFu,

} SOC_RcmDspClockSource;

/**
 * @brief R5 Clock Sources
 */
typedef enum SOC_RcmR5ClockSource
{
    /**
     * @brief   Value specifying PLL Core Clock Out 2
     */
    SOC_RcmR5ClockSource_DPLL_CORE_HSDIV0_CLKOUT2

} SOC_RcmR5ClockSource;

/**
 * @brief PLL Fout values
 */
typedef enum SOC_RcmPllFoutFreqId_e
{
    /**
     * @brief   Value specifying PLL Fout of 800 Mhz
     */
    SOC_RcmPllFoutFreqId_CLK_800MHZ,
    /**
     * @brief   Value specifying PLL Fout of 900 Mhz
     */
    SOC_RcmPllFoutFreqId_CLK_900MHZ,
    /**
     * @brief   Value specifying PLL Fout of 2000 Mhz
     */
    SOC_RcmPllFoutFreqId_CLK_2000MHZ,
    /**
     * @brief   Value specifying PLL Fout of 1800 Mhz
     */
    SOC_RcmPllFoutFreqId_CLK_1800MHZ,
    /**
     * @brief   Value specifying PLL Fout of 1920 Mhz
     */
    SOC_RcmPllFoutFreqId_CLK_1920MHZ,
    /**
     * @brief   Value specifying PLL Fout of 1699.21875 Mhz
     */
    SOC_RcmPllFoutFreqId_CLK_1699p21875MHZ,
    /**
     * @brief   Value specifying PLL Fout of 1728 Mhz
     */
    SOC_RcmPllFoutFreqId_CLK_1728MHZ,
    /**
     * @brief   Value specifying PLL Fout of 1966.08 Mhz
     */
    SOC_RcmPllFoutFreqId_CLK_1966p08MHZ,
    /**
     * @brief   Value specifying PLL Fout of 1806.336 Mhz
     */
    SOC_RcmPllFoutFreqId_CLK_1806p336MHZ,
    /**
     * @brief max value
     */
    SOC_RcmPllFoutFreqId_MAX_VALUE = 0xFFFFFFFFu,

} SOC_RcmPllFoutFreqId;

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

/**
 *  \brief Set CORE PLL Config
 *
 * \param outFreqId [in] Output frequency form the PLL
 * \param hsDivCfg [in] Set HS divider output frequencies
 */
void SOC_rcmCoreApllConfig(SOC_RcmPllFoutFreqId outFreqId, SOC_RcmPllHsDivOutConfig *hsDivCfg);

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
 *  \brief Set R5 and SycClk frequency
 *
 * \param r5FreqHz [in] R5 frequency, in Hz
 * \param sysClkFreqHz [in] SysClk frequency, in Hz
 *
 * \return SystemP_SUCCESS on success, else failure
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
 *  \brief Get QSPI Efuse configuration
 *
 * \param qspiEfuseCfg [out] QSPI Efuse configuration
 */
void SOC_rcmGetEfuseQspiConfig(SOC_RcmEfuseQspiConfig *qspiEfuseCfg);

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
 *  \brief Initialize the MSS mailbox memory
 */
void SOC_rcmMemInitMssMailboxMemory(void);

/**
 *  \brief Initialize the DSS mailbox memory
 */
void SOC_rcmMemInitDssMailboxMemory(void);

/**
 *  \brief Start memory initialization for MSS L2
 */
void SOC_rcmStartMemInitMSSL2(void);

/**
 *  \brief Wait memory initialization to complete for MSS L2
 */
void SOC_rcmWaitMemInitMSSL2(void);

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
 *  \brief RCM configuration for MAC interface
 */
void SOC_rcmConfigEthMacIf(void);

/**
 * \brief Return R5SS status operating in lockstep or dual core mode.
 * \param r5fClusterGroupId [in] R5F Cluster Group Id.
 *        Refer \ref CSL_ArmR5ClusterGroupID for applicable values.
 *
 * \return R5SS operating in lockstep or dual core mode
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

/** @} */

#ifdef __cplusplus
}
#endif

#endif
