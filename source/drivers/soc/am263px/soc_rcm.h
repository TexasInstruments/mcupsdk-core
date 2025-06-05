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

#ifndef SOC_RCM_AM263PX_H_
#define SOC_RCM_AM263PX_H_

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

#define SOC_RCM_FREQ_MHZ2HZ(x)          ((x) * 1000 * 1000)
#define SOC_RCM_FREQ_HZ2MHZ(x)          ((x) / (1000 * 1000))

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
    SOC_WarmResetCause_POWER_ON_RESET = 0x41U,
    /**
     * \brief Value specifying MSS WDT0
     */
    SOC_WarmResetCause_MSS_WDT0 = 0x42U,
    /**
     * \brief Value specifying MSS WDT1
     */
    SOC_WarmResetCause_MSS_WDT1 = 0x44U,
    /**
     * \brief Value specifying MSS WDT2
     */
    SOC_WarmResetCause_MSS_WDT2 = 0x48U,
    /**
     * \brief Value specifying MSS WDT3
     */
    SOC_WarmResetCause_MSS_WDT3 = 0x50U,
    /**
     * \brief Value specifying Software Warm Reset
     */
    SOC_WarmResetCause_TOP_RCM_WARM_RESET_REQ = 0x60U,
    /**
     * \brief Value specifying External Pad Reset
     */
    SOC_WarmResetCause_EXT_PAD_RESET = 0x40U,
    /**
     * \brief Value specifying HSM WDT
     */
    SOC_WarmResetCause_HSM_WDT = 0xC0U,
    /**
     * \brief Value specifying Debugger Reset
     */
    SOC_WarmResetCause_DBG_RESET = 0x140U,
    /**
     * \brief Value specifying Temperature Sensor0 Reset
     */
    SOC_WarmResetCause_TEMP_SENSOR0_RESET = 0x240U,
    /**
     * \brief Value specifying Temperature Sensor1 Reset
     */
    SOC_WarmResetCause_TEMP_SENSOR1_RESET = 0x440U,

}SOC_WarmResetCause;
/** @} */

/**
 *  \anchor SOC_WarmResetSource_t
 *  \name SOC Warm Reset Sources
 *  @{
 */
typedef enum SOC_WarmResetSource_e
{
    /**
     * \brief Value specifying Pad Warm Reset pin
     */
    SOC_WarmResetSource_PAD_BYPASS = CSL_TOP_RCM_WARM_RESET_CONFIG_PAD_BYPASS_MASK,
    /**
     * \brief Value specifying DebugSS
     */
    SOC_WarmResetSource_DEBUGSS = CSL_TOP_RCM_WARM_RESET_CONFIG_DEBUGSS_RST_EN_MASK,
    /**
     * \brief Value specifying Temperature Sensor 0
     */
    SOC_WarmResetSource_TSENSE0 = CSL_TOP_RCM_WARM_RESET_CONFIG_TSENSE0_RST_EN_MASK,
    /**
     * \brief Value specifying Temperature Sensor 1
     */
    SOC_WarmResetSource_TSENSE1 = CSL_TOP_RCM_WARM_RESET_CONFIG_TSENSE1_RST_EN_MASK,
    /**
     * \brief Value specifying Watchdog 0
     */
    SOC_WarmResetSource_WDOG0 = CSL_TOP_RCM_WARM_RESET_CONFIG_WDOG0_RST_EN_MASK,
    /**
     * \brief Value specifying Watchdog 1
     */
    SOC_WarmResetSource_WDOG1 = CSL_TOP_RCM_WARM_RESET_CONFIG_WDOG1_RST_EN_MASK,
    /**
     * \brief Value specifying Watchdog 2
     */
    SOC_WarmResetSource_WDOG2 = CSL_TOP_RCM_WARM_RESET_CONFIG_WDOG2_RST_EN_MASK,
    /**
     * \brief Value specifying Watchdog 3
     */
    SOC_WarmResetSource_WDOG3 = CSL_TOP_RCM_WARM_RESET_CONFIG_WDOG3_RST_EN_MASK,

}SOC_WarmResetSource;
/** @} */

/**
 *  \anchor SOC_RcmWarm_ResetTime123_t
 *  \name SOC programmable values for WARM_RSTTIME1/2/3 registers and
 *        the corresponding delays in ns/us/ms.
 *  @{
 */
typedef enum SOC_RcmWarm_ResetTime123_e
{
    /**
     * \brief   Delay Value specifying in time 500ns
     */
    SOC_WARM_RESET_PAD_TIME_500NS = 0U,
    /**
     * \brief   Delay Value specifying in time 1us
     */
    SOC_WARM_RESET_PAD_TIME_1US,
    /**
     * \brief   Delay Value specifying in time 2us
     */
    SOC_WARM_RESET_PAD_TIME_2US,
    /**
     * \brief   Delay Value specifying in time 4us
     */
    SOC_WARM_RESET_PAD_TIME_4US,
    /**
     * \brief   Delay Value specifying in time 8us
     */
    SOC_WARM_RESET_PAD_TIME_8US,
    /**
     * \brief   Delay Value specifying in time 16us
     */
    SOC_WARM_RESET_PAD_TIME_16US,
    /**
     * \brief   Delay Value specifying in time 32us
     */
    SOC_WARM_RESET_PAD_TIME_32US,
    /**
     * \brief   Delay Value specifying in time 64us
     */
    SOC_WARM_RESET_PAD_TIME_64US,
    /**
     * \brief   Delay Value specifying in time 128us
     */
    SOC_WARM_RESET_PAD_TIME_128US,
    /**
     * \brief   Delay Value specifying in time 256us
     */
    SOC_WARM_RESET_PAD_TIME_256US,
    /**
     * \brief   Delay Value specifying in time 512us
     */
    SOC_WARM_RESET_PAD_TIME_512US,
    /**
     * \brief   Delay Value specifying in time 1024us
     */
    SOC_WARM_RESET_PAD_TIME_1024US,
    /**
     * \brief   Delay Value specifying in time 2048us
     */
    SOC_WARM_RESET_PAD_TIME_2048US,
    /**
     * \brief   Delay Value specifying in time 4096us
     */
    SOC_WARM_RESET_PAD_TIME_4096US,
    /**
     * \brief   Delay Value specifying in time 8192us
     */
    SOC_WARM_RESET_PAD_TIME_8192US,
    /**
     * \brief   Delay Value specifying in time 16384us
     */
    SOC_WARM_RESET_PAD_TIME_16384US,

} SOC_RcmWarm_ResetTime123;
/** @} */

/**
 *  \anchor SOC_RcmResetCause_t
 *  \name SOC RCM Reset Causes
 *  @{
 */
typedef enum SOC_RcmResetCause_e
{
    /**
     * \brief   Value specifying Power ON Reset
     */
    SOC_RcmResetCause_POWER_ON_RESET = 0x0U,
    /**
     * \brief   Value specifying Warm Reset
     */
    SOC_RcmResetCause_WARM_RESET = 0x1U,
    /**
     * \brief   Value specifying STC Reset
     */
    SOC_RcmResetCause_STC_RESET = 0x2U,
    /**
     * \brief   Value specifying R5 Core A Subsytem Reset
     */
    SOC_RcmResetCause_MMR_CPU0_VIM0_RESET = 0x3U,
    /**
     * \brief   Value specifying R5 Core B Subsytem Reset
     */
    SOC_RcmResetCause_MMR_CPU1_VIM1_RESET = 0x4U,
    /**
     * \brief   Value specifying R5 Core A (core only) Reset
     */
    SOC_RcmResetCause_MMR_CPU0_RESET = 0x5U,
    /**
     * \brief   Value specifying R5 Core B (core only) Reset
     */
    SOC_RcmResetCause_MMR_CPU1_RESET = 0x6U,
    /**
     * \brief   Value specifying R5 Core A Debug Reset
     */
    SOC_RcmResetCause_DBG_CPU0_RESET = 0x7U,
    /**
     * \brief   Value specifying R5 Core B Debug Reset
     */
    SOC_RcmResetCause_DBG_CPU1_RESET = 0x8U,
    /**
     * \brief   Value specifying R5 Reset due to FSM Trigger
     */
    SOC_RcmResetCause_FSM_TRIGGER_RESET = 0x9U,
    /**
     * \brief   Value specifying R5 Reset due to write to debug POR RST CTRL Reg
     */
    SOC_RcmResetCause_POR_RST_CTRL0 = 0xAU,
    /**
     * \brief   Value specifying R5 Reset due to Unknown reason
     */
    SOC_RcmResetCause_RST_CAUSE_UNKNOWN = 0xBU,

}SOC_RcmResetCause;
/** @} */

/**
 *  \anchor SOC_RcmR5F_Subsystem_t
 *  \name SOC R5F subsystems
 *  @{
 */
typedef enum SOC_Rcmr5fssNum_e
{
    /**
     * \brief   Value specifying Power ON Reset
     */
    r5fss0 = 0x0U,
    /**
     * \brief   Value specifying Warm Reset
     */
    r5fss1 = 0x1U,
}SOC_Rcmr5fssNum;
/** @} */

/**
 *  \anchor SOC_RcmPeripheral_Id_t
 *  \name SOC Peripheral Ids
 *  @{
 */
typedef enum SOC_RcmPeripheralId_e
{
    /**
     * \brief   Value specifying MCAN0
     */
    SOC_RcmPeripheralId_MCAN0,
    /**
     * \brief   Value specifying MCAN1
     */
    SOC_RcmPeripheralId_MCAN1,
    /**
     * \brief   Value specifying MCAN2
     */
    SOC_RcmPeripheralId_MCAN2,
    /**
     * \brief   Value specifying MCAN3
     */
    SOC_RcmPeripheralId_MCAN3,
    /**
     * \brief   Value specifying OSPI0
     */
    SOC_RcmPeripheralId_OSPI0,
    /**
     * \brief   Value specifying RTI0
     */
    SOC_RcmPeripheralId_RTI0,
    /**
     * \brief   Value specifying RTI1
     */
    SOC_RcmPeripheralId_RTI1,
    /**
     * \brief   Value specifying RTI2
     */
    SOC_RcmPeripheralId_RTI2,
    /**
     * \brief   Value specifying RTI3
     */
    SOC_RcmPeripheralId_RTI3,
    /**
     * \brief   Value specifying WDT0
     */
    SOC_RcmPeripheralId_WDT0,
    /**
     * \brief   Value specifying WDT1
     */
    SOC_RcmPeripheralId_WDT1,
    /**
     * \brief   Value specifying WDT2
     */
    SOC_RcmPeripheralId_WDT2,
    /**
     * \brief   Value specifying WDT3
     */
    SOC_RcmPeripheralId_WDT3,
    /**
     * \brief   Value specifying MCSPI0
     */
    SOC_RcmPeripheralId_MCSPI0,
    /**
     * \brief   Value specifying MCSPI1
     */
    SOC_RcmPeripheralId_MCSPI1,
    /**
     * \brief   Value specifying MCSPI2
     */
    SOC_RcmPeripheralId_MCSPI2,
    /**
     * \brief   Value specifying MCSPI3
     */
    SOC_RcmPeripheralId_MCSPI3,
    /**
     * \brief   Value specifying MCSPI4
     */
    SOC_RcmPeripheralId_MCSPI4,
    /**
     * \brief   Value specifying MCSPI5
     */
    SOC_RcmPeripheralId_MCSPI5,
    /**
     * \brief   Value specifying MCSPI6
     */
    SOC_RcmPeripheralId_MCSPI6,
    /**
     * \brief   Value specifying MCSPI7
     */
    SOC_RcmPeripheralId_MCSPI7,
    /**
     * \brief   Value specifying MMC0
     */
    SOC_RcmPeripheralId_MMC0,
    /**
     * \brief   Value specifying ICSSM0_UART0
     */
    SOC_RcmPeripheralId_ICSSM0_UART0,
    /**
     * \brief   Value specifying CPTS
     */
    SOC_RcmPeripheralId_CPTS,
    /**
     * \brief   Value specifying GPMC
     */
    SOC_RcmPeripheralId_GPMC,
    /**
     * \brief   Value specifying CONTROLSS_PLL
     */
    SOC_RcmPeripheralId_CONTROLSS_PLL,
    /**
     * \brief   Value specifying I2C
     */
    SOC_RcmPeripheralId_I2C,
    /**
     * \brief   Value specifying LIN0_UART0
     */
    SOC_RcmPeripheralId_LIN0_UART0,
    /**
     * \brief   Value specifying LIN1_UART1
     */
    SOC_RcmPeripheralId_LIN1_UART1,
    /**
     * \brief   Value specifying LIN2_UART2
     */
    SOC_RcmPeripheralId_LIN2_UART2,
    /**
     * \brief   Value specifying LIN3_UART3
     */
    SOC_RcmPeripheralId_LIN3_UART3,
    /**
     * \brief   Value specifying LIN4_UART4
     */
    SOC_RcmPeripheralId_LIN4_UART4,
    /**
     * \brief   Value specifying LIN5_UART5
     */
    SOC_RcmPeripheralId_LIN5_UART5,
    /**
     * \brief   Value specifying MCAN4
     */
    SOC_RcmPeripheralId_MCAN4,
    /**
     * \brief   Value specifying MCAN5
     */
    SOC_RcmPeripheralId_MCAN5,
    /**
     * \brief   Value specifying MCAN6
     */
    SOC_RcmPeripheralId_MCAN6,
    /**
     * \brief   Value specifying MCAN7
     */
    SOC_RcmPeripheralId_MCAN7,
}SOC_RcmPeripheralId;
/** @} */

/**
 *  \anchor SOC_RcmPeripheral_Clock_Source_t
 *  \name SOC Peripheral clock sources
 *  @{
 */
typedef enum SOC_RcmPeripheralClockSource_e
{
    /**
     * \brief   Value specifying Crystal Clock
     */
    SOC_RcmPeripheralClockSource_XTALCLK,
    /**
     * \brief   Value specifying System Clock (200Mhz)
     */
    SOC_RcmPeripheralClockSource_SYS_CLK,
    /**
     * \brief   Value specifying wake up clock
     */
    SOC_RcmPeripheralClockSource_WUCPUCLK,
    /**
     * \brief   Value specifying external reference clock
     */
    SOC_RcmPeripheralClockSource_EXT_REFCLK,
    /**
     * \brief   Value specifying RC clock (10MHz)
     */
    SOC_RcmPeripheralClockSource_RCCLK10M,
    /**
     * \brief   Value specifying RC clock (32KHz)
     */
    SOC_RcmPeripheralClockSource_RCCLK32K,
    /**
     * \brief   Value specifying CPTS GENF0 clock
     */
    SOC_RcmPeripheralClockSource_CTPS_GENF0,
    /**
     * \brief   Value specifying PLL Core Clock Out 0 (400 Mhz)
     */
    SOC_RcmPeripheralClockSource_DPLL_CORE_HSDIV0_CLKOUT0,
    /**
     * \brief   Value specifying PLL Core Clock Out 1 (500 Mhz)
     */
    SOC_RcmPeripheralClockSource_DPLL_CORE_HSDIV0_CLKOUT1,
    /**
     * \brief   Value specifying PLL Core Clock Out 2 (400 Mhz)
     */
    SOC_RcmPeripheralClockSource_DPLL_CORE_HSDIV0_CLKOUT2,
    /**
     * \brief   Value specifying PLL Core Clock Out 0 (160 Mhz)
     */
    SOC_RcmPeripheralClockSource_DPLL_PER_HSDIV0_CLKOUT0,
    /**
     * \brief   Value specifying PLL Core Clock Out 1 (192 Mhz)
     */
    SOC_RcmPeripheralClockSource_DPLL_PER_HSDIV0_CLKOUT1,
} SOC_RcmPeripheralClockSource;
/** @} */

/**
 *  \anchor SOC_RcmPll_Freq_Id_t
 *  \name SOC PLL frequency output IDs
 *  @{
 */
typedef enum SOC_RcmPllFoutFreqId_e
{
    /**
     * \brief   Value specifying PLL output frequency 2000MHz
     */
    RCM_PLL_FOUT_FREQID_CLK_2000MHZ,
    /**
     * \brief   Value specifying PLL output frequency 1920MHz
     */
    RCM_PLL_FOUT_FREQID_CLK_1920MHZ,
    /**
     * \brief   Value specifying PLL output frequency 800MHz
     */
    RCM_PLL_FOUT_FREQID_CLK_800MHZ,
} SOC_RcmPllFoutFreqId;
/** @} */

/**
 *  \anchor SOC_RcmXtal_Freq_Id_t
 *  \name SOC XTAL frequency IDs
 *  @{
 */
typedef enum SOC_RcmXtalFreqId_e
{
    /**
     * \brief   Value specifying XTAL frequency 25MHZ
     */
    RCM_XTAL_FREQID_CLK_25MHZ,
} SOC_RcmXtalFreqId;

typedef enum SOC_RcmPllId_e
{
    RCM_PLLID_CORE,
    RCM_PLLID_PER,
    RCM_PLLID_XTALCLK,
    RCM_PLLID_WUCPUCLK,
    RCM_PLLID_RCCLK32K,
    RCM_PLLID_RCCLK10M,
    RCM_PLLID_EXTREFCLK,
} SOC_RcmPllId;
/** @} */

/**
 *  \anchor SOC_RcmHsDivider_Id_t
 *  \name SOC HSDIVIDER IDs
 *  @{
 */
typedef enum SOC_RcmPllHSDIVOutId_e
{
    /**
     * \brief   Value specifying HSDIVIDER 0
     */
    RCM_PLLHSDIV_OUT_0,
    /**
     * \brief   Value specifying HSDIVIDER 1
     */
    RCM_PLLHSDIV_OUT_1,
    /**
     * \brief   Value specifying HSDIVIDER 2
     */
    RCM_PLLHSDIV_OUT_2,
    /**
     * \brief   Value specifying invalid/no HSDIVIDER ID
     */
    RCM_PLLHSDIV_OUT_NONE,
} SOC_RcmPllHSDIVOutId;
/** @} */

typedef struct SOC_RcmClkSrcInfo_s
{
    SOC_RcmPllId pllId;
    SOC_RcmPllHSDIVOutId hsDivOut;
} SOC_RcmClkSrcInfo;

typedef struct SOC_RcmXTALInfo_s
{
    uint32_t Finp;
    Bool     div2flag;
} SOC_RcmXTALInfo;

typedef struct SOC_RcmADPLLJConfig_s
{
    uint32_t N; /* Input Clock divider/Pre-divider (N) */
    uint32_t M2; /* Post divider (M2) */
    uint32_t M;  /* Multiplier integer (M) */
    uint32_t FracM; /* Multiplier fractional (M) */
    uint32_t Fout; /* Output frequency of PLL */
    uint32_t Finp; /* Output frequency of PLL */
} SOC_RcmADPLLJConfig_t;

#define RCM_PLL_HSDIV_OUTPUT_ENABLE_0                                 (1U << 0U)
#define RCM_PLL_HSDIV_OUTPUT_ENABLE_1                                 (1U << 1U)
#define RCM_PLL_HSDIV_OUTPUT_ENABLE_2                                 (1U << 2U)
#define RCM_PLL_HSDIV_OUTPUT_ENABLE_3                                 (1U << 3U)
#define RCM_PLL_HSDIV_OUTPUT_ENABLE_ALL     (RCM_PLL_HSDIV_OUTPUT_ENABLE_0   | \
                                             RCM_PLL_HSDIV_OUTPUT_ENABLE_1   | \
                                             RCM_PLL_HSDIV_OUTPUT_ENABLE_2   | \
                                             RCM_PLL_HSDIV_OUTPUT_ENABLE_3)

#define RCM_PLL_HSDIV_OUTPUT_IDX0                                            (0)
#define RCM_PLL_HSDIV_OUTPUT_IDX1                                            (1)
#define RCM_PLL_HSDIV_OUTPUT_IDX2                                            (2)
#define RCM_PLL_HSDIV_OUTPUT_IDX3                                            (3)
#define RCM_PLL_HSDIV_OUTPUT_COUNT               (RCM_PLL_HSDIV_OUTPUT_IDX3 + 1)

typedef struct SOC_RcmPllHsDivOutConfig_s
{
    uint32_t hsdivOutEnMask;
    uint32_t hsDivOutFreqHz[RCM_PLL_HSDIV_OUTPUT_COUNT];
} SOC_RcmPllHsDivOutConfig;

/**
 * \brief Configure CORE PLL
 *
 * \param outFreqId [in] PLL output frequency ID. Enumberation: SOC_RcmPllFoutFreqId
 * \param *hsDivCfg [in] HSDIVIDER configuration
 *
 */
extern void SOC_rcmCoreApllConfig(SOC_RcmPllFoutFreqId outFreqId, SOC_RcmPllHsDivOutConfig *hsDivCfg);

/**
 * \brief Pre-requisite sequence to Re-configure CORE PLL
 */
extern uint32_t SOC_rcmCoreApllRelockPreRequisite(void);

/**
 * \brief Set R5 clock source
 */
extern void SOC_rcmSetR5ClockSource(uint32_t r5ClkSrc);


/**
 * \brief Configure CORE PLL HSDIVIDERS
 *
 * \param outFreqId [in] PLL output frequency ID. Enumberation: SOC_RcmPllFoutFreqId
 * \param *hsDivCfg [in] HSDIVIDER configuration
 *
 */
extern void SOC_rcmCoreApllHSDivConfig(SOC_RcmPllFoutFreqId outFreqId, SOC_RcmPllHsDivOutConfig *hsDivCfg);

/**
 * \brief Configure PER PLL
 *
 * \param outFreqId [in] PLL output frequency ID. Enumberation: SOC_RcmPllFoutFreqId
 * \param *hsDivCfg [in] HSDIVIDER configuration
 *
 */
extern void SOC_rcmPerApllConfig(SOC_RcmPllFoutFreqId outFreqId, SOC_RcmPllHsDivOutConfig *hsDivCfg);

/**
 * \brief Set R5FSS and Sysclk frequency (Root clock configuration)
 *
 * \param cr5FreqHz [in] R5F frequency
 * \param sysClkFreqHz [in] SYSCLK frequency
 * \param cpuId [in] Cpu Id. Refer \ref CSL_CoreID for applicable values.
 *
 */
extern void SOC_rcmsetR5SysClock(uint32_t cr5FreqHz, uint32_t sysClkFreqHz,
                                 uint32_t cpuId);

/**
 * \brief Set Trace clock frequency
 *
 * \param traceFreqHz [in] Trace frequency
 *
 */
extern void SOC_rcmsetTraceClock(uint32_t traceFreqHz);

/**
 * \brief Set CLKOUT clock frequency
 *
 * \param clkout0FreqHz [in] CLKOUT0 frequency
 * \param clkout1FreqHz [in] CLKOUT1 frequency
 *
 */
extern void SOC_rcmsetClkoutClock(uint32_t clkout0FreqHz, uint32_t clkout1FreqHz);

/**
 * \brief Set module clock (IP clock configuration)
 *
 *      This API programs the Clock Source Selection and Clock divider values
 *      for a specified peripheral Id.
 *
 * \param periphId [in] Peripheral ID
 * \param clkSource [in] Clock source
 * \param freqHz [in] Frequency
 *
 * \return SystemP_SUCCESS Module clock is set
 * \return SystemP_FAILURE Module clock could not be set
 */
extern int32_t SOC_rcmSetPeripheralClock(SOC_RcmPeripheralId periphId, SOC_RcmPeripheralClockSource clkSource, uint32_t freqHz);

/**
 * \brief Get R5FSS reset cause
 *
 *      This API returns the reset cause for R5 core. It also clears the Reset
 *      cause.
 *
 * \param r5fssNum [in] R5FSS0 or R5FSS1
 *
 * \return SOC_RcmResetCause reset cause
 */
extern SOC_RcmResetCause SOC_rcmGetResetCause(SOC_Rcmr5fssNum r5fssNum);

/**
 * \brief Enable/disable module clock (IP clock configuration)
 *
 *      This API programs the IP clock gates
 *      for a specified peripheral Id.
 *
 * \param periphId [in] Peripheral ID
 * \param enable [in] ungate (1)/gate clock (0)
 *
 * \return SystemP_SUCCESS Module clock is set
 * \return SystemP_FAILURE Module clock could not be set
 */
int32_t SOC_rcmEnablePeripheralClock(SOC_RcmPeripheralId periphId, uint32_t enable);

/**
 *  \brief Set R5SS0/R5SS1 and SysClk frequency
 *
 * \param r5FreqHz [in] R5 frequency, in Hz
 * \param sysClkFreqHz [in] SysClk frequency, in Hz
 * \param cpuId [in] Cpu Id. Refer \ref CSL_CoreID for applicable values.
 *
 * \return SystemP_SUCCESS on success, else failure
 */
int32_t SOC_rcmSetR5Clock(uint32_t r5FreqHz, uint32_t sysClkFreqHz, uint32_t cpuId);

/**
 * \brief Get R5SS0/1 frequency
 * \param cpuId [in] Cpu Id. Refer \ref CSL_CoreID for applicable values.
 *
 * \return R5 frequency, in Hz
 */
uint32_t SOC_rcmGetR5Clock(uint32_t cpuId);

/**
 * \brief Configure R5 in lock step mode
 * \param cpuId [in] Cpu Id. Refer \ref CSL_CoreID for applicable values.
 */
void SOC_rcmR5ConfigLockStep(uint32_t cpuId);

/**
 * \brief Configure R5 in dual core mode
 * \param cpuId [in] Cpu Id. Refer \ref CSL_CoreID for applicable values.
 */
void SOC_rcmR5ConfigDualCore(uint32_t cpuId);

/**
 * \brief Set R5 clock source
 */
extern void SOC_rcmSetR5ClockSource(uint32_t r5ClkSrc);

/**
 *  \brief Trigger R5 core reset
 */
void SOC_rcmR5SS0TriggerReset(void);

/**
 * \brief Unhalt R5 cores
 * \param cpuId [in] Cpu Id. Refer \ref CSL_CoreID for applicable values.
 */
void SOC_rcmCoreR5FUnhalt(uint32_t cpuId);

/**
 * \brief Start memory initialization for R5 TCMA
 * \param cpuId [in] Cpu Id. Refer \ref CSL_CoreID for applicable values.
 */
void SOC_rcmStartMemInitTCMA(uint32_t cpuId);

/**
 * \brief Wait memory initialization to complete for R5 TCMA
 * \param cpuId [in] Cpu Id. Refer \ref CSL_CoreID for applicable values.
 */
void SOC_rcmWaitMemInitTCMA(uint32_t cpuId);

/**
 * \brief Start memory initialization for R5 TCMB
 * \param cpuId [in] Cpu Id. Refer \ref CSL_CoreID for applicable values.
 */
void SOC_rcmStartMemInitTCMB(uint32_t cpuId);

/**
 * \brief Wait memory initialization to complete for R5 TCMB
 * \param cpuId [in] Cpu Id. Refer \ref CSL_CoreID for applicable values.
 */
void SOC_rcmWaitMemInitTCMB(uint32_t cpuId);

/**
 *  \brief Wait memory initialization to complete for Mailbox memory
 */
void SOC_rcmMemInitMailboxMemory(void);
/**
 *  \brief Wait memory initialization to complete for L2 Bank2 and Bank3 memory
 */
void SOC_rcmMemInitL2Memory(void);
/**
 *  \brief Reset R5SS0 Core
 */
void SOC_rcmR5SS0PowerOnReset(void);

/**
 *  \brief Trigger R5SS1 core reset
 */
void SOC_rcmR5SS1TriggerReset(void);

/**
 *  \brief Reset R5SS1 Core
 */
void SOC_rcmR5SS1PowerOnReset(void);

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
 * \brief Configure WARM reset source
 *
 * \param source [in] WARM reset source. User needs to set the source as multibit
 *        and program only the required fields.
 */
void SOC_configureWarmResetSource(uint32_t source);

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
 * \brief Program output delay on warm reset Pad 1
 *
 * \param opDelayValue [in] Programmable delay value. Refer \ref SOC_RcmWarm_ResetTime123_t
 */
void SOC_configureWarmResetOutputDelay(uint16_t opDelayValue);

/**
 * \brief Program input rise delay on warm reset Pad 2
 *
 * \param inpRiseDelayValue [in] Programmable delay value. Refer \ref SOC_RcmWarm_ResetTime123_t
 */
void SOC_configureWarmResetInputRiseDelay(uint16_t inpRiseDelayValue);

/**
 * \brief Program output delay on warm reset Pad 3
 *
 * \param inpFallDelayValue [in] Programmable delay value. Refer \ref SOC_RcmWarm_ResetTime123_t
 */
void SOC_configureWarmResetInputFallDelay(uint16_t inpFallDelayValue);

/** @} */

#ifdef __cplusplus
}
#endif

#endif