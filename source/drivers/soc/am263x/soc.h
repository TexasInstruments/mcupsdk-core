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

#ifndef SOC_AM263X_H_
#define SOC_AM263X_H_

#ifdef __cplusplus
extern "C"
{
#endif

/**
 *  \defgroup DRV_SOC_MODULE APIs for SOC Specific Functions
 *  \ingroup DRV_MODULE
 *
 * For more details and example usage, see \ref DRIVERS_SOC_PAGE
 *
 *  @{
 */

#include <kernel/dpl/SystemP.h>
#include <drivers/hw_include/cslr_soc.h>
#include "soc_xbar.h"
#include "soc_rcm.h"
#include <drivers/hw_include/am263x/cslr_soc_baseaddress.h>

/**
 *  \anchor SOC_DomainId_t
 *  \name SOC Domain ID
 *  @{
 */
#define SOC_DOMAIN_ID_MAIN     (0U)
/** @} */

/*Control MMRs partition*/
#define MSS_CTRL_PARTITION0                                (1)
#define TOP_CTRL_PARTITION0                                (2)
#define CONTROLSS_CTRL_PARTITION0                          (3)

/*Clock and reset MMRs partition*/
#define MSS_RCM_PARTITION0                                 (4)
#define TOP_RCM_PARTITION0                                 (5)

/*Pinmux MMR*/
//#define IOMUX_PARTITION0                                   (6)

/* define the unlock and lock values for MSS_CTRL, TOP_CTRL, MSS_RCM, TOP_RCM*/
#define KICK_LOCK_VAL                           (0x00000000U)
#define KICK0_UNLOCK_VAL                        (0x01234567U)
#define KICK1_UNLOCK_VAL                        (0x0FEDCBA8U)

/** \brief API to validate MCSPI base address. */
static inline int32_t MCSPI_lld_isBaseAddrValid(uint32_t baseAddr)
{
    int32_t status = (int32_t)-3;

    if ((baseAddr == CSL_MCSPI0_U_BASE) || \
        (baseAddr == CSL_MCSPI1_U_BASE) || \
        (baseAddr == CSL_MCSPI2_U_BASE) || \
        (baseAddr == CSL_MCSPI3_U_BASE) || \
        (baseAddr == CSL_MCSPI4_U_BASE) )
    {
        status = 0;
    }

    return status;
}
/** \brief Macro to check if the I2C base address is valid */
#define IS_I2C_BASE_ADDR_VALID(baseAddr)    ((baseAddr == CSL_I2C0_U_BASE) || \
                                             (baseAddr == CSL_I2C1_U_BASE) || \
                                             (baseAddr == CSL_I2C2_U_BASE) || \
                                             (baseAddr == CSL_I2C3_U_BASE))


/** \brief Macro to check if the QSPI base address is valid */
#define IS_QSPI_BASE_ADDR_VALID(baseAddr)    (baseAddr == CSL_QSPI0_U_BASE)

/** \brief Macro to check if the QSPI Memory Mapped address is valid */
#define IS_QSPI_MEMORY_MAP_ADDR_VALID(baseAddr)    ((baseAddr == CSL_EXT_FLASH0_U_BASE) || \
                                                    (baseAddr == CSL_EXT_FLASH1_U_BASE))

/** \brief API to validate MMCSD base addresses. */
static inline int32_t MMCSD_lld_isBaseAddrValid(uint32_t baseAddr)
{
    /* Set status to invalid Param */
    int32_t status = (int32_t)(-3);

    if (baseAddr == CSL_MMC0_U_BASE)

    {
        /* Set status to success */
        status = 0;
    }

    return status;
}

/**
 * \brief Enable clock to specified module
 *
 * \param moduleId [in] see \ref SOC_RcmPeripheralId for list of module ID's
 * \param enable [in] 1: enable clock to the module, 0: disable clock to the module
 *
 * \return SystemP_SUCCESS Module clock is enabled
 * \return SystemP_FAILURE Module clock could not be enabled
 */
int32_t SOC_moduleClockEnable(uint32_t moduleId, uint32_t enable);

/**
 * \brief Set module clock to specified frequency
 *
 * \param moduleId [in] see \ref SOC_RcmPeripheralId for list of module ID's
 * \param clkId [in] see \ref SOC_RcmPeripheralClockSource for list of clocks
 * \param clkRate [in] Frequency to set in Hz
 *
 * \return SystemP_SUCCESS Module clock is enabled
 * \return SystemP_FAILURE Module clock could not be enabled
 */
int32_t SOC_moduleSetClockFrequency(uint32_t moduleId, uint32_t clkId, uint64_t clkRate);

/**
 * \brief Convert a core ID to a user readable name
 *
 * \param coreId    [in] see \ref CSL_CoreID
 *
 * \return name as a string
 */
const char *SOC_getCoreName(uint16_t coreId);

/**
 * \brief Get the clock frequency in Hz of the CPU on which the driver is running
 *
 * \return Clock frequency in Hz
 */
uint64_t SOC_getSelfCpuClk(void);

/**
 * \brief Lock control module partition to prevent writes into control MMRs
 *
 * \param domainId    [in] See SOC_DomainId_t
 * \param partition   [in] Partition number to unlock
 */
void SOC_controlModuleLockMMR(uint32_t domainId, uint32_t partition);

/**
 * \brief Unlock control module partition to allow writes into control MMRs
 *
 * \param domainId    [in] See SOC_DomainId_t
 * \param partition   [in] Partition number to unlock
 */
void SOC_controlModuleUnlockMMR(uint32_t domainId, uint32_t partition);

/**
 * \brief Enable or disable ePWM time base clock from Control MMR
 *
 * \param epwmInstance [in] ePWM instance number [0 - (CSL_EPWM_PER_CNT-1)]
 * \param enable       [in] TRUE to enable and FALSE to disable
 */
void SOC_setEpwmTbClk(uint32_t epwmInstance, uint32_t enable);

/**
 * \brief Enable or disable Multiple ePWM time base clock from Control MMR
 *
 * \param epwmMask     [in] ePWM instance Mask [1 - CSL_CONTROLSS_CTRL_EPWM_CLKSYNC_BIT_MAX]
 * \param enable       [in] TRUE to enable and FALSE to disable
 */
void SOC_setMultipleEpwmTbClk(uint32_t epwmMask, uint32_t enable);


/**
 * \brief Enable ADC references by writing to Control MMR
 *
 * \param adcInstance [in] ADC instance number [0 - (CSL_ADC_PER_CNT-1)]
 */
void SOC_enableAdcReference(uint32_t adcInstance);

/**
 * \brief Enables the ADC internal reference
 *
 * \param adcInstance [in] ADC instance number [0 - (CSL_ADC_PER_CNT-1)]
 * \param enable      [in] TRUE to enable internal reference - FALSE to disable.
 */
void SOC_enableAdcInternalReference(uint32_t adcInstance, uint32_t enable);

/**
 * \brief Enable ADC reference Monitors by writing to Control MMR
 *
 * \param adcInstance [in] ADC instance number [0 - (CSL_ADC_PER_CNT-1)]
 * \param enable      [in] TRUE to enable internal reference Monitor - FALSE to disable.
 */
void SOC_enableAdcReferenceMonitor(uint32_t adcInstance, uint32_t enable);

/**
 * \brief Gets the Reference status
 *
 * \param adcInstance  [in] ADC instance number [0 - (CSL_ADC_PER_CNT-1)]
 * \return                  TRUE - reference OK. FALSE - reference NOT OK.
 */
uint32_t SOC_getAdcReferenceStatus(uint32_t adcInstance);

/**
 * \brief Configure the ePWM group
 *
 * \param epwmInstance [in] ePWM instance number [0 - (CSL_EPWM_PER_CNT-1)]
 * \param group        [in] The group for this ePWM instance [0 - 3]
 */
void SOC_setEpwmGroup(uint32_t epwmInstance, uint32_t group);

/**
 * \brief Select the SDFM1 CLK0 source
 *
 * \param source [in] Source of SDFM1 CLK0. 0: source is SDFM1 CK0 from Pinmux. 1: source is SDFM0 CK0 from Pinmux
 */
void SOC_selectSdfm1Clk0Source(uint8_t source);

/**
 * \brief Gate the ePWM clock
 *
 * \param epwmInstance [in] ePWM instance number [0 - (CSL_EPWM_PER_CNT-1)]
 */
void SOC_gateEpwmClock(uint32_t epwmInstance);

/**
 * \brief Ungate the ePWM clock
 *
 * \param epwmInstance [in] ePWM instance number [0 - (CSL_EPWM_PER_CNT-1)]
 */
void SOC_ungateEpwmClock(uint32_t epwmInstance);

/**
 * \brief Gate the FSI-TX clock
 *
 * \param fsitxInstance [in] FSITX instance number [0 - 3]
 */
void SOC_gateFsitxClock(uint32_t fsitxInstance);

/**
 * \brief Gate the FSI-RX clock
 *
 * \param fsirxInstance [in] FSIRX instance number [0 - 3]
 */
void SOC_gateFsirxClock(uint32_t fsirxInstance);

/**
 * \brief Gate the CMPSS-A clock
 *
 * \param cmpssaInstance [in] CMPSS-A instance number [0 - 9]
 */
void SOC_gateCmpssaClock(uint32_t cmpssaInstance);

/**
 * \brief Ungate the CMPSS-A clock
 *
 * \param cmpssaInstance [in] CMPSS-A instance number [0 - 9]
 */
void SOC_ungateCmpssaClock(uint32_t cmpssaInstance);

/**
 * \brief Gate the CMPSS-B clock
 *
 * \param cmpssbInstance [in] CMPSS-B instance number [0 - 9]
 */
void SOC_gateCmpssbClock(uint32_t cmpssbInstance);

/**
 * \brief Ungate the CMPSS-B clock
 *
 * \param cmpssbInstance [in] CMPSS-B instance number [0 - 9]
 */
void SOC_ungateCmpssbClock(uint32_t cmpssbInstance);

/**
 * \brief Gate the ECAP clock
 *
 * \param ecapInstance [in] ECAP instance number [0 - 9]
 */
void SOC_gateEcapClock(uint32_t ecapInstance);

/**
 * \brief Ungate the ECAP clock
 *
 * \param ecapInstance [in] ECAP instance number [0 - 9]
 */
void SOC_ungateEcapClock(uint32_t ecapInstance);

/**
 * \brief Gate the EQEP clock
 *
 * \param eqepInstance [in] EQEP instance number [0 - 2]
 */
void SOC_gateEqepClock(uint32_t eqepInstance);

/**
 * \brief Ungate the EQEP clock
 *
 * \param eqepInstance [in] EQEP instance number [0 - 2]
 */
void SOC_ungateEqepClock(uint32_t eqepInstance);

/**
 * \brief Gate the SDFM clock
 *
 * \param sdfmInstance [in] SDFM instance number [0 - 1]
 */
void SOC_gateSdfmClock(uint32_t sdfmInstance);

/**
 * \brief Ungate the SDFM clock
 *
 * \param sdfmInstance [in] SDFM instance number [0 - 1]
 */
void SOC_ungateSdfmClock(uint32_t sdfmInstance);

/**
 * \brief Gate the DAC clock
 */
void SOC_gateDacClock(void);

/**
 * \brief Ungate the DAC clock
 */
void SOC_ungateDacClock(void);

/**
 * \brief Gate the ADC clock
 *
 * \param adcInstance [in] ADC instance number [0 - 4]
 */
void SOC_gateAdcClock(uint32_t adcInstance);

/**
 * \brief ungate the ADC clock
 *
 * \param adcInstance [in] ADC instance number [0 - 4]
 */
void SOC_ungateAdcClock(uint32_t adcInstance);

/**
 * \brief Gate the OTTO clock
 *
 * \param ottoInstance [in] OTTO instance number [0 - 3]
 */
void SOC_gateOttoClock(uint32_t ottoInstance);

/**
 * \brief Ungate the OTTO clock
 *
 * \param ottoInstance [in] OTTO instance number [0 - 3]
 */
void SOC_ungateOttoClock(uint32_t ottoInstance);

/**
 * \brief Gate the SDFM PLL clock
 *
 * \param sdfmInstance [in] SDFM instance number [0 - 1]
 */
void SOC_gateSdfmPllClock(uint32_t sdfmInstance);

/**
 * \brief Ungate the SDFM PLL clock
 *
 * \param sdfmInstance [in] SDFM instance number [0 - 1]
 */
void SOC_ungateSdfmPllClock(uint32_t sdfmInstance);
/**
 * \brief Gate the FSI-TX PLL clock
 *
 * \param fsiInstance [in] FSI instance number [0 - 3]
 */
void SOC_gateFsiPllClock(uint32_t fsiInstance);

/**
 * \brief Generate ePWM reset
 *
 * \param ePWMInstance [in] ePWM instance number [0 - 31]
 */
void SOC_generateEpwmReset(uint32_t ePWMInstance);

/**
 * \brief Generate FSI-TX reset
 *
 * \param fsitxInstance [in] FSI instance number [0 - 3]
 */
void SOC_generateFsiTxReset(uint32_t fsitxInstance);

/**
 * \brief Generate FSI-RX reset
 *
 * \param fsirxInstance [in] FSI instance number [0 - 3]
 */
void SOC_generateFsiRxReset(uint32_t fsirxInstance);

/**
 * \brief Generate CMPSS-A reset
 *
 * \param cmpssaInstance [in] CMPSS-A instance number [0 - 9]
 */
void SOC_generateCmpssaReset(uint32_t cmpssaInstance);

/**
 * \brief Generate CMPSS-B reset
 *
 * \param cmpssbInstance [in] CMPSS-B instance number [0 - 9]
 */
void SOC_generateCmpssbReset(uint32_t cmpssbInstance);

/**
 * \brief Generate ECAP reset
 *
 * \param ecapInstance [in] ECAP instance number [0 - 9]
 */
void SOC_generateEcapReset(uint32_t ecapInstance);

/**
 * \brief Generate EQEP reset
 *
 * \param eqepInstance [in] EQEP instance number [0 - 2]
 */
void SOC_generateEqepReset(uint32_t eqepInstance);

/**
 * \brief Generate SDFM reset
 *
 * \param sdfmInstance [in] SDFM instance number [0 - 1]
 */
void SOC_generateSdfmReset(uint32_t sdfmInstance);

/**
 * \brief Generate DAC reset
 */
void SOC_generateDacReset(void);

/**
 * \brief Generate ADC reset
 *
 * \param adcInstance [in] ADC instance number [0 - 4]
 */
void SOC_generateAdcReset(uint32_t adcInstance);

/**
 * \brief Halt EPWM with corresponding cPU
 *
 * \param epwmInstance [in] EPWM instance number [0 - 31]
 */

void Soc_enableEPWMHalt (uint32_t epwmInstance);

/**
 * \brief Halt EPWM with corresponding cPU
 *
 * \param epwmInstance [in] EPWM instance number [0 - 31]
 */

void Soc_disableEPWMHalt (uint32_t epwmInstance);

/**
 * \brief Generate OTTO reset
 *
 * \param ottoInstance [in] OTTO instance number [0 - 3]
 */
void SOC_generateOttoReset(uint32_t ottoInstance);

/**
 * \brief Selection of ICSS GPI MUX
 *
 * \param pru_instance [in] PRU instance number [0 - 1]
 * \param mask [in] Bitwise selection of ICSSM GPI source. GPI or PWMXBar select for ICSSM port 0/1. 0-GPI, 1-PWMXBAR
 */
void SOC_selectIcssGpiMux(uint8_t pru_instance, uint32_t mask);

/**
 * \brief Setting CPSW hard reset Bit
 */
void SOC_setResetCPSWBit();

/**
 * \brief Clearing CPSW hard reset bit
 */
void SOC_clearResetCPSWBit();

/**
 *  \brief SOC Virtual (CPU) to Physical address translation function.
 *
 *  \param virtAddr [IN] Virtual/CPU address
 *
 *  \return Corresponding SOC physical address
 */
uint64_t SOC_virtToPhy(void *virtAddr);

/**
 *  \brief Physical to Virtual (CPU) address translation function.
 *
 *  \param phyAddr  [IN] Physical address
 *
 *  \return Corresponding virtual/CPU address
 */
void *SOC_phyToVirt(uint64_t phyAddr);

/**
 *  \brief  Configure GPIO pin Slew rate.
 *
 *  \param baseAddr     Base Address of IOMUX registers (CSL_IOMUX_U_BASE)
 *
 *  \param bankNum      Bank number of GPIO Pin
 *
 *  \param groupNum     Group Number where one group register applies to 8 GPIO's.
 *                      So, one bank will be in two groups. For lower half, pass
 *                      group number as 0 and for upper half pass 1.
 *
 *  \param samplePeriod Sampling period in units of clock cycles.
 *
 *  \note  Qualifier selection should be done for Pad config register using \ref Pinmux_config
 *
 */
void SOC_configSlewRate(uint32_t baseAddr, uint32_t bankNum, uint32_t groupNum, uint32_t samplePeriod);


/** \brief Macro to validate UART base address */

/** \brief API to validate UART base address */
static inline int32_t UART_IsBaseAddrValid(uint32_t baseAddr)
{
    int32_t status = (int32_t)-3;

    if(((baseAddr == CSL_UART0_U_BASE) ||
        (baseAddr == CSL_UART1_U_BASE) ||
        (baseAddr == CSL_UART2_U_BASE) ||
        (baseAddr == CSL_UART3_U_BASE) ||
        (baseAddr == CSL_UART4_U_BASE) ||
        (baseAddr == CSL_UART5_U_BASE)))
    {
        status = 0;
    }

    return status;
}

/**
 *  \brief  This function gets the SOC mapped data base address of the flash
 *
 *  \return Data BaseAddress of the flash
 */
uint32_t SOC_getFlashDataBaseAddr(void);

/** @} */

#ifdef __cplusplus
}
#endif

#endif
