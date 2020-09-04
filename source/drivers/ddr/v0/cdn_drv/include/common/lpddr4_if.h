/**********************************************************************
 * Copyright (C) 2012-2021 Cadence Design Systems, Inc.
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 
 * 1. Redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holder nor the names of its
 * contributors may be used to endorse or promote products derived from
 * this software without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 **********************************************************************
 * WARNING: This file is auto-generated using api-generator utility.
 *          api-generator: 12.02.13bb8d5
 *          Do not edit it manually.
 **********************************************************************
 * Cadence Core Driver for LPDDR4.
 **********************************************************************/

#ifndef LPDDR4_IF_H
#define LPDDR4_IF_H

#ifdef __cplusplus
extern "C"
{
#endif


#include "cdn_stdtypes.h"
#include <drivers/hw_include/soc_config.h>
#ifdef DDR_16BIT
#include "../16bit/lpddr4_16bit_if.h"
#else
#include "../32bit/lpddr4_32bit_if.h"
#endif


/** @defgroup DataStructure Dynamic Data Structures
 *  This section defines the data structures used by the driver to provide
 *  hardware information, modification and dynamic operation of the driver.
 *  These data structures are defined in the header file of the core driver
 *  and utilized by the API.
 *  @{
 */

/**********************************************************************
 * Forward declarations
 **********************************************************************/
typedef struct LPDDR4_Config_s LPDDR4_Config;
typedef struct LPDDR4_PrivateData_s LPDDR4_PrivateData;
typedef struct LPDDR4_DebugInfo_s LPDDR4_DebugInfo;
typedef struct LPDDR4_FspModeRegs_s LPDDR4_FspModeRegs;


/**********************************************************************
 * Enumerations
 **********************************************************************/
/** This is used to indicate whether the Controller, PHY, or PHY Independent module is addressed. */
typedef enum
{
    LPDDR4_CTL_REGS = 0U,
    LPDDR4_PHY_REGS = 1U,
    LPDDR4_PHY_INDEP_REGS = 2U
} LPDDR4_RegBlock;

/** List of informations and warnings from driver. */
typedef enum
{
    LPDDR4_DRV_NONE = 0U,
    LPDDR4_DRV_SOC_PLL_UPDATE = 1U
} LPDDR4_InfoType;

/** Low power interface wake up timing parameters */
typedef enum
{
    LPDDR4_LPI_PD_WAKEUP_FN = 0U,
    LPDDR4_LPI_SR_SHORT_WAKEUP_FN = 1U,
    LPDDR4_LPI_SR_LONG_WAKEUP_FN = 2U,
    LPDDR4_LPI_SR_LONG_MCCLK_GATE_WAKEUP_FN = 3U,
    LPDDR4_LPI_SRPD_SHORT_WAKEUP_FN = 4U,
    LPDDR4_LPI_SRPD_LONG_WAKEUP_FN = 5U,
    LPDDR4_LPI_SRPD_LONG_MCCLK_GATE_WAKEUP_FN = 6U
} LPDDR4_LpiWakeUpParam;

/** Half Datapath mode setting */
typedef enum
{
    LPDDR4_REDUC_ON = 0U,
    LPDDR4_REDUC_OFF = 1U
} LPDDR4_ReducMode;

/** ECC Control parameter setting */
typedef enum
{
    LPDDR4_ECC_DISABLED = 0U,
    LPDDR4_ECC_ENABLED = 1U,
    LPDDR4_ECC_ERR_DETECT = 2U,
    LPDDR4_ECC_ERR_DETECT_CORRECT = 3U
} LPDDR4_EccEnable;

/** Data Byte Inversion mode setting */
typedef enum
{
    LPDDR4_DBI_RD_ON = 0U,
    LPDDR4_DBI_RD_OFF = 1U,
    LPDDR4_DBI_WR_ON = 2U,
    LPDDR4_DBI_WR_OFF = 3U
} LPDDR4_DbiMode;

/** Controller Frequency Set Point number  */
typedef enum
{
    LPDDR4_FSP_0 = 0U,
    LPDDR4_FSP_1 = 1U,
    LPDDR4_FSP_2 = 2U
} LPDDR4_CtlFspNum;

/**********************************************************************
 * Callbacks
 **********************************************************************/
/**
 * Reports informations and warnings that need to be communicated.
 * Params:
 * pD - driver state info specific to this instance.
 * infoType - Type of information.
*/
typedef void (*LPDDR4_InfoCallback)(const LPDDR4_PrivateData* pD, LPDDR4_InfoType infoType);

/**
 * Reports interrupts received by the controller.
 * Params:
 * pD - driver state info specific to this instance.
 * ctlInterrupt - Interrupt raised
 * chipSelect - Chip for which interrupt raised
*/
typedef void (*LPDDR4_CtlCallback)(const LPDDR4_PrivateData* pD, LPDDR4_INTR_CtlInterrupt ctlInterrupt, uint8_t chipSelect);

/**
 * Reports interrupts received by the PHY Independent Module.
 * Params:
 * privateData - driver state info specific to this instance.
 * phyIndepInterrupt - Interrupt raised
 * chipSelect - Chip for which interrupt raised
*/
typedef void (*LPDDR4_PhyIndepCallback)(const LPDDR4_PrivateData* pD, LPDDR4_INTR_PhyIndepInterrupt phyIndepInterrupt, uint8_t chipSelect);

/**
 *  @}
 */

/** @defgroup DriverFunctionAPI Driver Function API
 *  Prototypes for the driver API functions. The user application can link statically to the
 *  necessary API functions and call them directly.
 *  @{
 */

/**********************************************************************
 * API methods
 **********************************************************************/

/**
 * Checks configuration object.
 * @param[in] config Driver/hardware configuration required.
 * @param[out] configSize Size of memory allocations required.
 * @return CDN_EOK on success (requirements structure filled).
 * @return CDN_ENOTSUP if configuration cannot be supported due to driver/hardware constraints.
 */
uint32_t LPDDR4_Probe(const LPDDR4_Config* config, uint16_t* configSize);

/**
 * Init function to be called after LPDDR4_probe() to set up the
 * driver configuration.  Memory should be allocated for drv_data
 * (using the size determined using LPDDR4_probe)  before calling this
 * API.  init_settings should be initialised with base addresses for
 * PHY Indepenent Module, Controller and PHY before calling this
 * function.  If callbacks are required for interrupt handling, these
 * should also be configured in init_settings.
 * @param[in] pD Driver state info specific to this instance.
 * @param[in] cfg Specifies driver/hardware configuration.
 * @return CDN_EOK on success
 * @return CDN_EINVAL if illegal/inconsistent values in cfg.
 * @return CDN_ENOTSUP if hardware has an inconsistent configuration or doesn't support feature(s) required by 'config' parameters.
 */
uint32_t LPDDR4_Init(LPDDR4_PrivateData* pD, const LPDDR4_Config* cfg);

/**
 * Start the driver.
 * @param[in] pD Driver state info specific to this instance.
 */
uint32_t LPDDR4_Start(const LPDDR4_PrivateData* pD);

/**
 * Read a register from the controller, PHY or PHY Independent Module
 * @param[in] pD Driver state info specific to this instance.
 * @param[in] cpp Indicates whether controller, PHY or PHY Independent Module register
 * @param[in] regOffset Register offset
 * @param[out] regValue Register value read
 * @return CDN_EOK on success.
 * @return CDN_EINVAL if regOffset if out of range or regValue is NULL
 */
uint32_t LPDDR4_ReadReg(const LPDDR4_PrivateData* pD, LPDDR4_RegBlock cpp, uint32_t regOffset, uint32_t* regValue);

/**
 * Write a register in the controller, PHY or PHY Independent Module
 * @param[in] pD Driver state info specific to this instance.
 * @param[in] cpp Indicates whether controller, PHY or PHY Independent Module register
 * @param[in] regOffset Register offset
 * @param[in] regValue Register value to be written
 * @return CDN_EOK on success.
 * @return CDN_EINVAL if regOffset is out of range or regValue is NULL
 */
uint32_t LPDDR4_WriteReg(const LPDDR4_PrivateData* pD, LPDDR4_RegBlock cpp, uint32_t regOffset, uint32_t regValue);

/**
 * Read a memory mode register from DRAM
 * @param[in] pD Driver state info specific to this instance.
 * @param[in] readModeRegVal Value to set in 'read_modereg' parameter.
 * @param[out] mmrValue Value which is read from memory mode register(mmr) for all devices.
 * @param[out] mmrStatus Status of mode register read(mrr) instruction.
 * @return CDN_EOK on success.
 * @return CDN_EINVAL if regNumber is out of range or regValue is NULL
 */
uint32_t LPDDR4_GetMmrRegister(const LPDDR4_PrivateData* pD, uint32_t readModeRegVal, uint64_t* mmrValue, uint8_t* mmrStatus);

/**
 * Write a memory mode register in DRAM
 * @param[in] pD Driver state info specific to this instance.
 * @param[in] writeModeRegVal Value to set in 'write_modereg' parameter.
 * @param[out] mrwStatus Status of mode register write(mrw) instruction.
 * @return CDN_EOK on success.
 * @return CDN_EINVAL if regNumber is out of range or regValue is NULL
 */
uint32_t LPDDR4_SetMmrRegister(const LPDDR4_PrivateData* pD, uint32_t writeModeRegVal, uint8_t* mrwStatus);

/**
 * Write a set of initialisation values to the controller registers
 * @param[in] pD Driver state info specific to this instance.
 * @param[in] regValues Register values to be written
 * @param[in] regNum Register Number to be written
 * @param[in] regCount Count of registers to be written.
 * @return CDN_EOK on success.
 * @return CDN_EINVAL if regValues is NULL
 */
uint32_t LPDDR4_WriteCtlConfig(const LPDDR4_PrivateData* pD, uint32_t regValues[], uint16_t regNum[], uint16_t regCount);

/**
 * Write a set of initialisation values to the PHY registers
 * @param[in] pD Driver state info specific to this instance.
 * @param[in] regValues Register values to be written
 * @param[in] regNum Register Number to be written
 * @param[in] regCount Count of registers to be written.
 * @return CDN_EOK on success.
 * @return CDN_EINVAL if regValues is NULL
 */
uint32_t LPDDR4_WritePhyConfig(const LPDDR4_PrivateData* pD, uint32_t regValues[], uint16_t regNum[], uint16_t regCount);

/**
 * Write a set of initialisation values to the PHY Independent Module
 * registers
 * @param[in] pD Driver state info specific to this instance.
 * @param[in] regValues Register values to be written
 * @param[in] regNum Register Number to be written
 * @param[in] regCount Count of registers to be written.
 * @return CDN_EOK on success.
 * @return CDN_EINVAL if regValues is NULL
 */
uint32_t LPDDR4_WritePhyIndepConfig(const LPDDR4_PrivateData* pD, uint32_t regValues[], uint16_t regNum[], uint16_t regCount);

/**
 * Read values of the controller registers in bulk and store in
 * memory.
 * @param[in] pD Driver state info specific to this instance.
 * @param[in] regValues Pointer to feedback the read values.
 * @param[in] regNum Register Number to be read
 * @param[in] regCount Count of registers to be read.
 * @return CDN_EOK on success.
 * @return CDN_EINVAL if regValues is NULL
 */
uint32_t LPDDR4_ReadCtlConfig(const LPDDR4_PrivateData* pD, uint32_t regValues[], uint16_t regNum[], uint16_t regCount);

/**
 * Read the values of the PHY module registers in bulk and store in
 * memory.
 * @param[in] pD Driver state info specific to this instance.
 * @param[in] regValues Pointer to feedback the read values.
 * @param[in] regNum Register Number to be read
 * @param[in] regCount Count of registers to be read.
 * @return CDN_EOK on success.
 * @return CDN_EINVAL if regValues is NULL
 */
uint32_t LPDDR4_ReadPhyConfig(const LPDDR4_PrivateData* pD, uint32_t regValues[], uint16_t regNum[], uint16_t regCount);

/**
 * Read the values of the PHY Independent module registers in bulk and
 * store in memory.
 * @param[in] pD Driver state info specific to this instance.
 * @param[in] regValues Pointer to feedback the read values.
 * @param[in] regNum Register Number to be read
 * @param[in] regCount Count of registers to be read.
 * @return CDN_EOK on success.
 * @return CDN_EINVAL if regValues is NULL
 */
uint32_t LPDDR4_ReadPhyIndepConfig(const LPDDR4_PrivateData* pD, uint32_t regValues[], uint16_t regNum[], uint16_t regCount);

/**
 * Read the current interrupt mask for the controller
 * @param[in] pD Driver state info specific to this instance.
 * @param[out] mask Value of interrupt mask
 * @return CDN_EOK on success.
 * @return CDN_EINVAL if mask pointer is NULL
 */
uint32_t LPDDR4_GetCtlInterruptMask(const LPDDR4_PrivateData* pD, uint64_t* mask);

/**
 * Sets the interrupt mask for the controller
 * @param[in] pD Driver state info specific to this instance.
 * @param[in] mask Value of interrupt mask to be written
 * @return CDN_EOK on success.
 * @return CDN_EINVAL if mask pointer is NULL
 */
uint32_t LPDDR4_SetCtlInterruptMask(const LPDDR4_PrivateData* pD, const uint64_t* mask);

/**
 * Check whether a specific controller interrupt is active
 * @param[in] pD Driver state info specific to this instance.
 * @param[in] intr Interrupt to be checked
 * @param[out] irqStatus Status of the interrupt, TRUE if active
 * @return CDN_EOK on success.
 * @return CDN_EINVAL if intr is not valid
 */
uint32_t LPDDR4_CheckCtlInterrupt(const LPDDR4_PrivateData* pD, LPDDR4_INTR_CtlInterrupt intr, bool* irqStatus);

/**
 * Acknowledge  a specific controller interrupt
 * @param[in] pD Driver state info specific to this instance.
 * @param[in] intr Interrupt to be acknowledged
 * @return CDN_EOK on success.
 * @return CDN_EINVAL if intr is not valid
 */
uint32_t LPDDR4_AckCtlInterrupt(const LPDDR4_PrivateData* pD, LPDDR4_INTR_CtlInterrupt intr);

/**
 * Read the current interrupt mask for the PHY Independent Module
 * @param[in] pD Driver state info specific to this instance.
 * @param[out] mask Value of interrupt mask
 * @return CDN_EOK on success.
 * @return CDN_EINVAL if mask pointer is NULL
 */
uint32_t LPDDR4_GetPhyIndepInterruptMask(const LPDDR4_PrivateData* pD, uint32_t* mask);

/**
 * Sets the interrupt mask for the PHY Independent Module
 * @param[in] pD Driver state info specific to this instance.
 * @param[in] mask Value of interrupt mask to be written
 * @return CDN_EOK on success.
 * @return CDN_EINVAL if mask pointer is NULL
 */
uint32_t LPDDR4_SetPhyIndepInterruptMask(const LPDDR4_PrivateData* pD, const uint32_t* mask);

/**
 * Check whether a specific PHY Independent Module interrupt is active
 * @param[in] pD Driver state info specific to this instance.
 * @param[in] intr Interrupt to be checked
 * @param[out] irqStatus Status of the interrupt, TRUE if active
 * @return CDN_EOK on success.
 * @return CDN_EINVAL if intr is not valid
 */
uint32_t LPDDR4_CheckPhyIndepInterrupt(const LPDDR4_PrivateData* pD, LPDDR4_INTR_PhyIndepInterrupt intr, bool* irqStatus);

/**
 * Acknowledge  a specific PHY Independent Module interrupt
 * @param[in] pD Driver state info specific to this instance.
 * @param[in] intr Interrupt to be acknowledged
 * @return CDN_EOK on success.
 * @return CDN_EINVAL if intr is not valid
 */
uint32_t LPDDR4_AckPhyIndepInterrupt(const LPDDR4_PrivateData* pD, LPDDR4_INTR_PhyIndepInterrupt intr);

/**
 * Retrieve status information after a failed init.  The
 * DebugStructInfo will be filled  in with error codes which can be
 * referenced against the driver documentation for further details.
 * @param[in] pD Driver state info specific to this instance.
 * @param[out] debugInfo status
 * @return CDN_EOK on success.
 * @return CDN_EINVAL if debugInfo is NULL
 */
uint32_t LPDDR4_GetDebugInitInfo(const LPDDR4_PrivateData* pD, LPDDR4_DebugInfo* debugInfo);

/**
 * Get the current value of Low power Interface wake up time.
 * @param[in] pD Driver state info specific to this instance.
 * @param[in] lpiWakeUpParam LPI timing parameter
 * @param[in] fspNum Frequency copy
 * @param[out] cycles Timing value(in cycles)
 * @return CDN_EOK on success.
 * @return CDN_EINVAL if powerMode is NULL
 */
uint32_t LPDDR4_GetLpiWakeUpTime(const LPDDR4_PrivateData* pD, const LPDDR4_LpiWakeUpParam* lpiWakeUpParam, const LPDDR4_CtlFspNum* fspNum, uint32_t* cycles);

/**
 * Set the current value of Low power Interface wake up time.
 * @param[in] pD Driver state info specific to this instance.
 * @param[in] lpiWakeUpParam LPI timing parameter
 * @param[in] fspNum Frequency copy
 * @param[in] cycles Timing value(in cycles)
 * @return CDN_EOK on success.
 * @return CDN_EINVAL if powerMode is NULL
 */
uint32_t LPDDR4_SetLpiWakeUpTime(const LPDDR4_PrivateData* pD, const LPDDR4_LpiWakeUpParam* lpiWakeUpParam, const LPDDR4_CtlFspNum* fspNum, const uint32_t* cycles);

/**
 * Get the current value for ECC auto correction
 * @param[in] pD Driver state info specific to this instance.
 * @param[out] eccParam ECC parameter setting
 * @return CDN_EOK on success.
 * @return CDN_EINVAL if on_off is NULL
 */
uint32_t LPDDR4_GetEccEnable(const LPDDR4_PrivateData* pD, LPDDR4_EccEnable* eccParam);

/**
 * Set the value for ECC auto correction.  This API must be called
 * before startup of memory.
 * @param[in] pD Driver state info specific to this instance.
 * @param[in] eccParam ECC control parameter setting
 * @return CDN_EOK on success.
 * @return CDN_EINVAL if on_off is NULL
 */
uint32_t LPDDR4_SetEccEnable(const LPDDR4_PrivateData* pD, const LPDDR4_EccEnable* eccParam);

/**
 * Get the current value for the Half Datapath option
 * @param[in] pD Driver state info specific to this instance.
 * @param[out] mode Half Datapath setting
 * @return CDN_EOK on success.
 * @return CDN_EINVAL if mode is NULL
 */
uint32_t LPDDR4_GetReducMode(const LPDDR4_PrivateData* pD, LPDDR4_ReducMode* mode);

/**
 * Set the value for the Half Datapath option.  This API must be
 * called before startup of memory.
 * @param[in] pD Driver state info specific to this instance.
 * @param[in] mode Half Datapath setting
 * @return CDN_EOK on success.
 * @return CDN_EINVAL if mode is NULL
 */
uint32_t LPDDR4_SetReducMode(const LPDDR4_PrivateData* pD, const LPDDR4_ReducMode* mode);

/**
 * Get the current value for Data Bus Inversion setting.  This will be
 * compared with the   current DRAM setting using the MR3 register.
 * @param[in] pD Driver state info specific to this instance.
 * @param[out] on_off DBI read value
 * @return CDN_EOK on success.
 * @return CDN_EINVAL if on_off is NULL
 */
uint32_t LPDDR4_GetDbiReadMode(const LPDDR4_PrivateData* pD, bool* on_off);

/**
 * Get the current value for Data Bus Inversion setting.  This will be
 * compared with the   current DRAM setting using the MR3 register.
 * @param[in] pD Driver state info specific to this instance.
 * @param[out] on_off DBI write value
 * @return CDN_EOK on success.
 * @return CDN_EINVAL if on_off is NULL
 */
uint32_t LPDDR4_GetDbiWriteMode(const LPDDR4_PrivateData* pD, bool* on_off);

/**
 * Set the mode for Data Bus Inversion. This will also be set in DRAM
 * using the MR3   controller register. This API must be called before
 * startup of memory.
 * @param[in] pD Driver state info specific to this instance.
 * @param[in] mode status
 * @return CDN_EOK on success.
 * @return CDN_EINVAL if mode is NULL
 */
uint32_t LPDDR4_SetDbiMode(const LPDDR4_PrivateData* pD, const LPDDR4_DbiMode* mode);

/**
 * Get the current value for the refresh rate (reading Refresh per
 * command timing).
 * @param[in] pD Driver state info specific to this instance.
 * @param[in] fspNum Frequency set number
 * @param[out] tref Refresh rate (in cycles)
 * @param[out] tras_max Maximum row active time (in cycles)
 * @return CDN_EOK on success.
 * @return CDN_EINVAL if tras_max is NULL
 */
uint32_t LPDDR4_GetRefreshRate(const LPDDR4_PrivateData* pD, const LPDDR4_CtlFspNum* fspNum, uint32_t* tref, uint32_t* tras_max);

/**
 * Set the refresh rate (writing Refresh per command timing).
 * @param[in] pD Driver state info specific to this instance.
 * @param[in] fspNum Frequency set number
 * @param[in] tref Refresh rate (in cycles)
 * @param[in] tras_max Maximum row active time (in cycles)
 * @return CDN_EOK on success.
 * @return CDN_EINVAL if tras_max is NULL
 */
uint32_t LPDDR4_SetRefreshRate(const LPDDR4_PrivateData* pD, const LPDDR4_CtlFspNum* fspNum, const uint32_t* tref, const uint32_t* tras_max);

/**
 * Handle Refreshing per chip select
 * @param[in] pD Driver state info specific to this instance.
 * @param[in] trefInterval status
 * @return CDN_EOK on success.
 * @return CDN_EINVAL if pD is NULL
 */
uint32_t LPDDR4_RefreshPerChipSelect(const LPDDR4_PrivateData* pD, const uint32_t trefInterval);

/**
 * Verify register write value during deferred duration
 * @param[in] pD Driver state info specific to this instance.
 * @param[in] cpp Register block type
 * @param[in] regValues Pointer to feedback the read values.
 * @param[in] regNum Register Number to be read
 * @param[in] regCount Count of registers to be read.
 * @return CDN_EOK on success.
 * @return CDN_EINVAL if regNum is NULL
 * @return CDN_EIO if regverify is failed
 */
uint32_t LPDDR4_DeferredRegVerify(const LPDDR4_PrivateData* pD, LPDDR4_RegBlock cpp, uint32_t regValues[], uint16_t regNum[], uint16_t regCount);

/**
 *  @}
 */



#ifdef __cplusplus
}
#endif

#endif	/* LPDDR4_IF_H */
