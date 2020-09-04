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
#ifndef LPDDR4_OBJ_IF_H
#define LPDDR4_OBJ_IF_H

#ifdef __cplusplus
extern "C"
{
#endif


#include "lpddr4_if.h"

/** @defgroup DriverObject Driver API Object
 *  API listing for the driver. The API is contained in the object as
 *  function pointers in the object structure. As the actual functions
 *  resides in the Driver Object, the client software must first use the
 *  global GetInstance function to obtain the Driver Object Pointer.
 *  The actual APIs then can be invoked using obj->(api_name)() syntax.
 *  These functions are defined in the header file of the core driver
 *  and utilized by the API.
 *  @{
 */

/**********************************************************************
 * API methods
 **********************************************************************/
typedef struct LPDDR4_OBJ_s
{
    /**
     * Checks configuration object.
     * @param[in] config Driver/hardware configuration required.
     * @param[out] configSize Size of memory allocations required.
     * @return CDN_EOK on success (requirements structure filled).
     * @return CDN_ENOTSUP if configuration cannot be supported due to driver/hardware constraints.
     */
    uint32_t (*probe)(const LPDDR4_Config* config, uint16_t* configSize);

    /**
     * Init function to be called after LPDDR4_probe() to set up the
     * driver configuration.  Memory should be allocated for drv_data
     * (using the size determined using LPDDR4_probe)  before calling
     * this API.  init_settings should be initialised with base addresses
     * for  PHY Indepenent Module, Controller and PHY before calling this
     * function.  If callbacks are required for interrupt handling, these
     * should also be configured in init_settings.
     * @param[in] pD Driver state info specific to this instance.
     * @param[in] cfg Specifies driver/hardware configuration.
     * @return CDN_EOK on success
     * @return CDN_EINVAL if illegal/inconsistent values in cfg.
     * @return CDN_ENOTSUP if hardware has an inconsistent configuration or doesn't support feature(s) required by 'config' parameters.
     */
    uint32_t (*init)(LPDDR4_PrivateData* pD, const LPDDR4_Config* cfg);

    /**
     * Start the driver.
     * @param[in] pD Driver state info specific to this instance.
     */
    uint32_t (*start)(const LPDDR4_PrivateData* pD);

    /**
     * Read a register from the controller, PHY or PHY Independent Module
     * @param[in] pD Driver state info specific to this instance.
     * @param[in] cpp Indicates whether controller, PHY or PHY Independent Module register
     * @param[in] regOffset Register offset
     * @param[out] regValue Register value read
     * @return CDN_EOK on success.
     * @return CDN_EINVAL if regOffset if out of range or regValue is NULL
     */
    uint32_t (*ReadReg)(const LPDDR4_PrivateData* pD, LPDDR4_RegBlock cpp, uint32_t regOffset, uint32_t* regValue);

    /**
     * Write a register in the controller, PHY or PHY Independent Module
     * @param[in] pD Driver state info specific to this instance.
     * @param[in] cpp Indicates whether controller, PHY or PHY Independent Module register
     * @param[in] regOffset Register offset
     * @param[in] regValue Register value to be written
     * @return CDN_EOK on success.
     * @return CDN_EINVAL if regOffset is out of range or regValue is NULL
     */
    uint32_t (*WriteReg)(const LPDDR4_PrivateData* pD, LPDDR4_RegBlock cpp, uint32_t regOffset, uint32_t regValue);

    /**
     * Read a memory mode register from DRAM
     * @param[in] pD Driver state info specific to this instance.
     * @param[in] readModeRegVal Value to set in 'read_modereg' parameter.
     * @param[out] mmrValue Value which is read from memory mode register(mmr) for all devices.
     * @param[out] mmrStatus Status of mode register read(mrr) instruction.
     * @return CDN_EOK on success.
     * @return CDN_EINVAL if regNumber is out of range or regValue is NULL
     */
    uint32_t (*GetMmrRegister)(const LPDDR4_PrivateData* pD, uint32_t readModeRegVal, uint64_t* mmrValue, uint8_t* mmrStatus);

    /**
     * Write a memory mode register in DRAM
     * @param[in] pD Driver state info specific to this instance.
     * @param[in] writeModeRegVal Value to set in 'write_modereg' parameter.
     * @param[out] mrwStatus Status of mode register write(mrw) instruction.
     * @return CDN_EOK on success.
     * @return CDN_EINVAL if regNumber is out of range or regValue is NULL
     */
    uint32_t (*SetMmrRegister)(const LPDDR4_PrivateData* pD, uint32_t writeModeRegVal, uint8_t* mrwStatus);

    /**
     * Write a set of initialisation values to the controller registers
     * @param[in] pD Driver state info specific to this instance.
     * @param[in] regValues Register values to be written
     * @param[in] regNum Register Number to be written
     * @param[in] regCount Count of registers to be written.
     * @return CDN_EOK on success.
     * @return CDN_EINVAL if regValues is NULL
     */
    uint32_t (*WriteCtlConfig)(const LPDDR4_PrivateData* pD, uint32_t regValues[], uint16_t regNum[], uint16_t regCount);

    /**
     * Write a set of initialisation values to the PHY registers
     * @param[in] pD Driver state info specific to this instance.
     * @param[in] regValues Register values to be written
     * @param[in] regNum Register Number to be written
     * @param[in] regCount Count of registers to be written.
     * @return CDN_EOK on success.
     * @return CDN_EINVAL if regValues is NULL
     */
    uint32_t (*WritePhyConfig)(const LPDDR4_PrivateData* pD, uint32_t regValues[], uint16_t regNum[], uint16_t regCount);

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
    uint32_t (*WritePhyIndepConfig)(const LPDDR4_PrivateData* pD, uint32_t regValues[], uint16_t regNum[], uint16_t regCount);

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
    uint32_t (*ReadCtlConfig)(const LPDDR4_PrivateData* pD, uint32_t regValues[], uint16_t regNum[], uint16_t regCount);

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
    uint32_t (*ReadPhyConfig)(const LPDDR4_PrivateData* pD, uint32_t regValues[], uint16_t regNum[], uint16_t regCount);

    /**
     * Read the values of the PHY Independent module registers in bulk
     * and store in memory.
     * @param[in] pD Driver state info specific to this instance.
     * @param[in] regValues Pointer to feedback the read values.
     * @param[in] regNum Register Number to be read
     * @param[in] regCount Count of registers to be read.
     * @return CDN_EOK on success.
     * @return CDN_EINVAL if regValues is NULL
     */
    uint32_t (*ReadPhyIndepConfig)(const LPDDR4_PrivateData* pD, uint32_t regValues[], uint16_t regNum[], uint16_t regCount);

    /**
     * Read the current interrupt mask for the controller
     * @param[in] pD Driver state info specific to this instance.
     * @param[out] mask Value of interrupt mask
     * @return CDN_EOK on success.
     * @return CDN_EINVAL if mask pointer is NULL
     */
    uint32_t (*GetCtlInterruptMask)(const LPDDR4_PrivateData* pD, uint64_t* mask);

    /**
     * Sets the interrupt mask for the controller
     * @param[in] pD Driver state info specific to this instance.
     * @param[in] mask Value of interrupt mask to be written
     * @return CDN_EOK on success.
     * @return CDN_EINVAL if mask pointer is NULL
     */
    uint32_t (*SetCtlInterruptMask)(const LPDDR4_PrivateData* pD, const uint64_t* mask);

    /**
     * Check whether a specific controller interrupt is active
     * @param[in] pD Driver state info specific to this instance.
     * @param[in] intr Interrupt to be checked
     * @param[out] irqStatus Status of the interrupt, TRUE if active
     * @return CDN_EOK on success.
     * @return CDN_EINVAL if intr is not valid
     */
    uint32_t (*CheckCtlInterrupt)(const LPDDR4_PrivateData* pD, LPDDR4_INTR_CtlInterrupt intr, bool* irqStatus);

    /**
     * Acknowledge  a specific controller interrupt
     * @param[in] pD Driver state info specific to this instance.
     * @param[in] intr Interrupt to be acknowledged
     * @return CDN_EOK on success.
     * @return CDN_EINVAL if intr is not valid
     */
    uint32_t (*AckCtlInterrupt)(const LPDDR4_PrivateData* pD, LPDDR4_INTR_CtlInterrupt intr);

    /**
     * Read the current interrupt mask for the PHY Independent Module
     * @param[in] pD Driver state info specific to this instance.
     * @param[out] mask Value of interrupt mask
     * @return CDN_EOK on success.
     * @return CDN_EINVAL if mask pointer is NULL
     */
    uint32_t (*GetPhyIndepInterruptMask)(const LPDDR4_PrivateData* pD, uint32_t* mask);

    /**
     * Sets the interrupt mask for the PHY Independent Module
     * @param[in] pD Driver state info specific to this instance.
     * @param[in] mask Value of interrupt mask to be written
     * @return CDN_EOK on success.
     * @return CDN_EINVAL if mask pointer is NULL
     */
    uint32_t (*SetPhyIndepInterruptMask)(const LPDDR4_PrivateData* pD, const uint32_t* mask);

    /**
     * Check whether a specific PHY Independent Module interrupt is
     * active
     * @param[in] pD Driver state info specific to this instance.
     * @param[in] intr Interrupt to be checked
     * @param[out] irqStatus Status of the interrupt, TRUE if active
     * @return CDN_EOK on success.
     * @return CDN_EINVAL if intr is not valid
     */
    uint32_t (*CheckPhyIndepInterrupt)(const LPDDR4_PrivateData* pD, LPDDR4_INTR_PhyIndepInterrupt intr, bool* irqStatus);

    /**
     * Acknowledge  a specific PHY Independent Module interrupt
     * @param[in] pD Driver state info specific to this instance.
     * @param[in] intr Interrupt to be acknowledged
     * @return CDN_EOK on success.
     * @return CDN_EINVAL if intr is not valid
     */
    uint32_t (*AckPhyIndepInterrupt)(const LPDDR4_PrivateData* pD, LPDDR4_INTR_PhyIndepInterrupt intr);

    /**
     * Retrieve status information after a failed init.  The
     * DebugStructInfo will be filled  in with error codes which can be
     * referenced against the driver documentation for further details.
     * @param[in] pD Driver state info specific to this instance.
     * @param[out] debugInfo status
     * @return CDN_EOK on success.
     * @return CDN_EINVAL if debugInfo is NULL
     */
    uint32_t (*GetDebugInitInfo)(const LPDDR4_PrivateData* pD, LPDDR4_DebugInfo* debugInfo);

    /**
     * Get the current value of Low power Interface wake up time.
     * @param[in] pD Driver state info specific to this instance.
     * @param[in] lpiWakeUpParam LPI timing parameter
     * @param[in] fspNum Frequency copy
     * @param[out] cycles Timing value(in cycles)
     * @return CDN_EOK on success.
     * @return CDN_EINVAL if powerMode is NULL
     */
    uint32_t (*GetLpiWakeUpTime)(const LPDDR4_PrivateData* pD, const LPDDR4_LpiWakeUpParam* lpiWakeUpParam, const LPDDR4_CtlFspNum* fspNum, uint32_t* cycles);

    /**
     * Set the current value of Low power Interface wake up time.
     * @param[in] pD Driver state info specific to this instance.
     * @param[in] lpiWakeUpParam LPI timing parameter
     * @param[in] fspNum Frequency copy
     * @param[in] cycles Timing value(in cycles)
     * @return CDN_EOK on success.
     * @return CDN_EINVAL if powerMode is NULL
     */
    uint32_t (*SetLpiWakeUpTime)(const LPDDR4_PrivateData* pD, const LPDDR4_LpiWakeUpParam* lpiWakeUpParam, const LPDDR4_CtlFspNum* fspNum, const uint32_t* cycles);

    /**
     * Get the current value for ECC auto correction
     * @param[in] pD Driver state info specific to this instance.
     * @param[out] eccParam ECC parameter setting
     * @return CDN_EOK on success.
     * @return CDN_EINVAL if on_off is NULL
     */
    uint32_t (*GetEccEnable)(const LPDDR4_PrivateData* pD, LPDDR4_EccEnable* eccParam);

    /**
     * Set the value for ECC auto correction.  This API must be called
     * before startup of memory.
     * @param[in] pD Driver state info specific to this instance.
     * @param[in] eccParam ECC control parameter setting
     * @return CDN_EOK on success.
     * @return CDN_EINVAL if on_off is NULL
     */
    uint32_t (*SetEccEnable)(const LPDDR4_PrivateData* pD, const LPDDR4_EccEnable* eccParam);

    /**
     * Get the current value for the Half Datapath option
     * @param[in] pD Driver state info specific to this instance.
     * @param[out] mode Half Datapath setting
     * @return CDN_EOK on success.
     * @return CDN_EINVAL if mode is NULL
     */
    uint32_t (*GetReducMode)(const LPDDR4_PrivateData* pD, LPDDR4_ReducMode* mode);

    /**
     * Set the value for the Half Datapath option.  This API must be
     * called before startup of memory.
     * @param[in] pD Driver state info specific to this instance.
     * @param[in] mode Half Datapath setting
     * @return CDN_EOK on success.
     * @return CDN_EINVAL if mode is NULL
     */
    uint32_t (*SetReducMode)(const LPDDR4_PrivateData* pD, const LPDDR4_ReducMode* mode);

    /**
     * Get the current value for Data Bus Inversion setting.  This will
     * be compared with the   current DRAM setting using the MR3
     * register.
     * @param[in] pD Driver state info specific to this instance.
     * @param[out] on_off DBI read value
     * @return CDN_EOK on success.
     * @return CDN_EINVAL if on_off is NULL
     */
    uint32_t (*GetDbiReadMode)(const LPDDR4_PrivateData* pD, bool* on_off);

    /**
     * Get the current value for Data Bus Inversion setting.  This will
     * be compared with the   current DRAM setting using the MR3
     * register.
     * @param[in] pD Driver state info specific to this instance.
     * @param[out] on_off DBI write value
     * @return CDN_EOK on success.
     * @return CDN_EINVAL if on_off is NULL
     */
    uint32_t (*GetDbiWriteMode)(const LPDDR4_PrivateData* pD, bool* on_off);

    /**
     * Set the mode for Data Bus Inversion. This will also be set in DRAM
     * using the MR3   controller register. This API must be called
     * before startup of memory.
     * @param[in] pD Driver state info specific to this instance.
     * @param[in] mode status
     * @return CDN_EOK on success.
     * @return CDN_EINVAL if mode is NULL
     */
    uint32_t (*SetDbiMode)(const LPDDR4_PrivateData* pD, const LPDDR4_DbiMode* mode);

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
    uint32_t (*GetRefreshRate)(const LPDDR4_PrivateData* pD, const LPDDR4_CtlFspNum* fspNum, uint32_t* tref, uint32_t* tras_max);

    /**
     * Set the refresh rate (writing Refresh per command timing).
     * @param[in] pD Driver state info specific to this instance.
     * @param[in] fspNum Frequency set number
     * @param[in] tref Refresh rate (in cycles)
     * @param[in] tras_max Maximum row active time (in cycles)
     * @return CDN_EOK on success.
     * @return CDN_EINVAL if tras_max is NULL
     */
    uint32_t (*SetRefreshRate)(const LPDDR4_PrivateData* pD, const LPDDR4_CtlFspNum* fspNum, const uint32_t* tref, const uint32_t* tras_max);

    /**
     * Handle Refreshing per chip select
     * @param[in] pD Driver state info specific to this instance.
     * @param[in] trefInterval status
     * @return CDN_EOK on success.
     * @return CDN_EINVAL if pD is NULL
     */
    uint32_t (*RefreshPerChipSelect)(const LPDDR4_PrivateData* pD, const uint32_t trefInterval);

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
    uint32_t (*DeferredRegVerify)(const LPDDR4_PrivateData* pD, LPDDR4_RegBlock cpp, uint32_t regValues[], uint16_t regNum[], uint16_t regCount);

} LPDDR4_OBJ;

/**
 * In order to access the LPDDR4 APIs, the upper layer software must call
 * this global function to obtain the pointer to the driver object.
 * @return LPDDR4_OBJ* Driver Object Pointer
 */
extern LPDDR4_OBJ *LPDDR4_GetInstance(void);

/**
 *  @}
 */



#ifdef __cplusplus
}
#endif

#endif	/* LPDDR4_OBJ_IF_H */
