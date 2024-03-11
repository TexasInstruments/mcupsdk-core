/******************************************************************************
 * Copyright (c) 2024 Texas Instruments Incorporated - http://www.ti.com
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

/**
 * @file   pmic_esm.h
 *
 * @brief  PMIC Low Level Driver API/interface file for ESM API
 */

#ifndef PMIC_ESM_H_
#define PMIC_ESM_H_
/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include "pmic.h"
#include "pmic_core.h"
#include "pmic_core_priv.h"
#include "pmic_io_priv.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @defgroup Pmic_ESM PMIC Error State Machine
 * @{
 * @brief Contains definitions related to PMIC ESM functionality.
 */

/* ========================================================================== */
/*                             Macros & Typedefs                              */
/* ========================================================================== */

/**
 * @defgroup Pmic_ESMMacros PMIC Error State Machine Macros
 * @{
 * @ingroup Pmic_ESM
 * @brief Contains macros used in the ESM module of PMIC driver.
 */

/**
 * @brief Boolean value representing the mode of the Error Signaling Module (ESM) as MCU mode.
 * The MCU mode indicates that the ESM is configured for MCU operation.
 *
 * @ingroup Pmic_ESMMacros
 */
#define PMIC_ESM_MODE_MCU   (0x0U)

/**
 * @brief Boolean value representing the mode of the Error Signaling Module (ESM) as SoC mode.
 * The SoC mode indicates that the ESM is configured for SoC operation.
 *
 * @ingroup Pmic_ESMMacros
 */
#define PMIC_ESM_MODE_SOC   (0x1U)

/**
 * @brief Boolean value representing the state of the Error Signaling Module (ESM) as stopped.
 * The stopped state indicates that the ESM is not operational.
 *
 * @ingroup Pmic_ESMMacros
 */
#define PMIC_ESM_STOP       (0x0U)

/**
 * @brief Boolean value representing the state of the Error Signaling Module (ESM) as started.
 * The started state indicates that the ESM is operational.
 *
 * @ingroup Pmic_ESMMacros
 */
#define PMIC_ESM_START      (0x01U)

/**
 * @brief Boolean value representing the state of the Error Signaling Module (ESM) as disabled.
 * The disabled state indicates that the ESM functionality is turned off.
 *
 * @ingroup Pmic_ESMMacros
 */
#define PMIC_ESM_DISABLE    (0x0U)

/**
 * @brief Boolean value representing the state of the Error Signaling Module (ESM) as enabled.
 * The enabled state indicates that the ESM functionality is turned on.
 *
 * @ingroup Pmic_ESMMacros
 */
#define PMIC_ESM_ENABLE     (0x01U)

/**
 * @brief Boolean value representing the mode of the Error Signaling Module (ESM) as level mode.
 * The level mode indicates that the ESM operates in level detection mode.
 *
 * @ingroup Pmic_ESMMacros
 */
#define PMIC_ESM_LEVEL_MODE (0x0U)

/**
 * @brief Boolean value representing the mode of the Error Signaling Module (ESM) as PWM mode.
 * The PWM mode indicates that the ESM operates in pulse-width modulation (PWM) mode.
 *
 * @ingroup Pmic_ESMMacros
 */
#define PMIC_ESM_PWM_MODE   (0x1U)

/**
 * @brief Boolean value representing the configuration of error enable drive clear as disabled.
 * The disabled state indicates that error enable drive clear functionality is turned off.
 *
 * @ingroup Pmic_ESMMacros
 */
#define PMIC_ESM_ERR_EN_DRV_CLEAR_DISABLE   (0x0U)

/**
 * @brief Boolean value representing the configuration of error enable drive clear as enabled.
 * The enabled state indicates that error enable drive clear functionality is turned on.
 *
 * @ingroup Pmic_ESMMacros
 */
#define PMIC_ESM_ERR_EN_DRV_CLEAR_ENABLE    (0x1U)

/**
 * @brief Boolean value representing the configuration of ESM interrupt as disabled.
 * The disabled state indicates that ESM interrupts are turned off.
 *
 * @ingroup Pmic_ESMMacros
 */
#define PMIC_ESM_INTERRUPT_DISABLE          (0x0U)

/**
 * @brief Boolean value representing the configuration of ESM interrupt as enabled.
 * The enabled state indicates that ESM interrupts are turned on.
 *
 * @ingroup Pmic_ESMMacros
 */
#define PMIC_ESM_INTERRUPT_ENABLE           (0x1U)

/**
 * @brief Numeric value representing the value 1 for error state machine.
 *
 * @ingroup Pmic_ESMMacros
 */
#define PMIC_ESM_VAL_1  (1U)

/**
 * @brief Bit mask indicating the validity of delay1 configuration parameter.
 * The bit mask indicates whether delay1 configuration parameter is valid.
 *
 * @ingroup Pmic_ESMMacros
 */
#define PMIC_ESM_CFG_DELAY1_VALID   (0x00U)

/**
 * @brief Bit mask indicating the validity of delay2 configuration parameter.
 * The bit mask indicates whether delay2 configuration parameter is valid.
 *
 * @ingroup Pmic_ESMMacros
 */
#define PMIC_ESM_CFG_DELAY2_VALID   (0x01U)

/**
 * @brief Bit mask indicating the validity of error count threshold configuration parameter.
 * The bit mask indicates whether error count threshold configuration parameter is valid.
 *
 * @ingroup Pmic_ESMMacros
 */
#define PMIC_ESM_CFG_ERR_CNT_THR_VALID  (0x02U)

/**
 * @brief Bit mask indicating the validity of Hmax configuration parameter.
 * The bit mask indicates whether Hmax configuration parameter is valid.
 *
 * @ingroup Pmic_ESMMacros
 */
#define PMIC_ESM_CFG_HMAX_VALID     (0x03U)

/**
 * @brief Bit mask indicating the validity of Hmin configuration parameter.
 * The bit mask indicates whether Hmin configuration parameter is valid.
 *
 * @ingroup Pmic_ESMMacros
 */
#define PMIC_ESM_CFG_HMIN_VALID     (0x04U)

/**
 * @brief Bit mask indicating the validity of Lmax configuration parameter.
 * The bit mask indicates whether Lmax configuration parameter is valid.
 *
 * @ingroup Pmic_ESMMacros
 */
#define PMIC_ESM_CFG_LMAX_VALID     (0x05U)

/**
 * @brief Bit mask indicating the validity of Lmin configuration parameter.
 * The bit mask indicates whether Lmin configuration parameter is valid.
 *
 * @ingroup Pmic_ESMMacros
 */
#define PMIC_ESM_CFG_LMIN_VALID     (0x06U)

/**
 * @brief Bit mask indicating the validity of error enable drive clear configuration parameter.
 * The bit mask indicates whether error enable drive clear configuration parameter is valid.
 *
 * @ingroup Pmic_ESMMacros
 */
#define PMIC_ESM_CFG_EN_DRV_VALID   (0x07U)

/**
 * @brief Bit mask indicating the validity of ESM mode configuration parameter.
 * The bit mask indicates whether ESM mode configuration parameter is valid.
 *
 * @ingroup Pmic_ESMMacros
 */
#define PMIC_ESM_CFG_MODE_VALID     (0x08U)

/**
 * @brief Bit shift representing the validity of delay1 configuration parameter.
 * The bit shift position indicates the validity of delay1 configuration parameter.
 *
 * @ingroup Pmic_ESMMacros
 */
#define PMIC_ESM_CFG_DELAY1_VALID_SHIFT (0x01U << PMIC_ESM_CFG_DELAY1_VALID)

/**
 * @brief Bit shift representing the validity of delay2 configuration parameter.
 * The bit shift position indicates the validity of delay2 configuration parameter.
 *
 * @ingroup Pmic_ESMMacros
 */
#define PMIC_ESM_CFG_DELAY2_VALID_SHIFT (0x01U << PMIC_ESM_CFG_DELAY2_VALID)

/**
 * @brief Bit shift representing the validity of error count threshold configuration parameter.
 * The bit shift position indicates the validity of error count threshold configuration parameter.
 *
 * @ingroup Pmic_ESMMacros
 */
#define PMIC_ESM_CFG_ERR_CNT_THR_VALID_SHIFT\
    (0x01U << PMIC_ESM_CFG_ERR_CNT_THR_VALID)

/**
 * @brief Bit shift representing the validity of Hmax configuration parameter.
 * The bit shift position indicates the validity of Hmax configuration parameter.
 *
 * @ingroup Pmic_ESMMacros
 */
#define PMIC_ESM_CFG_HMAX_VALID_SHIFT (0x01U << PMIC_ESM_CFG_HMAX_VALID)

/**
 * @brief Bit shift representing the validity of Hmin configuration parameter.
 * The bit shift position indicates the validity of Hmin configuration parameter.
 *
 * @ingroup Pmic_ESMMacros
 */
#define PMIC_ESM_CFG_HMIN_VALID_SHIFT (0x01U << PMIC_ESM_CFG_HMIN_VALID)

/**
 * @brief Bit shift representing the validity of Lmax configuration parameter.
 * The bit shift position indicates the validity of Lmax configuration parameter.
 *
 * @ingroup Pmic_ESMMacros
 */
#define PMIC_ESM_CFG_LMAX_VALID_SHIFT (0x01U << PMIC_ESM_CFG_LMAX_VALID)

/**
 * @brief Bit shift representing the validity of Lmin configuration parameter.
 * The bit shift position indicates the validity of Lmin configuration parameter.
 *
 * @ingroup Pmic_ESMMacros
 */
#define PMIC_ESM_CFG_LMIN_VALID_SHIFT (0x01U << PMIC_ESM_CFG_LMIN_VALID)

/**
 * @brief Bit shift representing the validity of error enable drive clear configuration parameter.
 * The bit shift position indicates the validity of error enable drive clear configuration parameter.
 *
 * @ingroup Pmic_ESMMacros
 */
#define PMIC_ESM_CFG_EN_DRV_VALID_SHIFT (0x01U << PMIC_ESM_CFG_EN_DRV_VALID)

/**
 * @brief Bit shift representing the validity of ESM mode configuration parameter.
 * The bit shift position indicates the validity of ESM mode configuration parameter.
 *
 * @ingroup Pmic_ESMMacros
 */
#define PMIC_ESM_CFG_MODE_VALID_SHIFT (0x01U << PMIC_ESM_CFG_MODE_VALID)

/**
 * @}
 */
/* End of Pmic_ESMMacros */

/*==========================================================================*/
/*                         Structures and Enums                             */
/*==========================================================================*/

/**
 * @defgroup Pmic_ESMStructures PMIC ESM Structures
 * @{
 * @ingroup Pmic_ESM
 * @brief Contains structures used in the ESM module of PMIC driver.
 */

/**
 * @brief  PMIC ESM Configuration structure
 *         Note: validParams is input param for all Set and Get APIs. other
 *         params except validParams is input param for Set APIs and output
 *         param for Get APIs
 *
 * @param   validParams          Selection of structure parameters to be set,
 *                               from the combination of \ref Pmic_EsmCflag
 *                               and the corresponding member value must be
 *                               updated.
 * @param   esmDelay1_us         ESM delay-1 time interval in micro seconds.
 *                               To get more effective results, user has to
 *                               program esmDelay1 with multiples of 2048.
 *                               The valid range is (0, 2048, 4096, 6144,
 *                               8192, ......., 522240).
 *                               Valid only when PMIC_ESM_CFG_DELAY1_VALID
 *                               bit is set
 * @param   esmDelay2_us         ESM delay-2 time interval in micro seconds.
 *                               To get more effective results, user has to
 *                               program esmDelay2 with multiples of 2048.
 *                               The valid range is (0, 2048, 4096, 6144,
 *                               8192, ......., 522240).
 *                               Valid only when PMIC_ESM_CFG_DELAY2_VALID
 *                               bit is set
 * @param   esmHmax_us           ESM Maximum high-pulse time-threshold value in
 *                               micro seconds.
 *                               To get more effective results, user has to
 *                               program esmHmax with multiples of 15.
 *                               The valid range is (15, 30, 45, 60, 75
 *                              ....., 3840).
 *                               Valid only when PMIC_ESM_CFG_HMAX_VALID
 *                               bit is set
 * @param   esmHmin_us           ESM Minimum high-pulse time-threshold value in
 *                               micro seconds.
 *                               To get more effective results, user has to
 *                               program esmHmin with multiples of 15.
 *                               The valid range is (15, 30, 45, 60, 75
 *                              ....., 3840).
 *                               Valid only when PMIC_ESM_CFG_HMIN_VALID
 *                               bit is set
 * @param   esmLmax_us           ESM Maximum low-pulse time-threshold value in
 *                               micro seconds.
 *                               To get more effective results, user has to
 *                               program esmLmax with multiples of 15.
 *                               The valid range is (15, 30, 45, 60, 75
 *                              ....., 3840).
 *                               Valid only when PMIC_ESM_CFG_LMAX_VALID
 *                               bit is set
 * @param   esmLmin_us           ESM Minimum low-pulse time-threshold value in
 *                               micro seconds.
 *                               To get more effective results, user has to
 *                               program esmLmin with multiples of 15.
 *                               The valid range is (15, 30, 45, 60, 75
 *                              ....., 3840).
 *                               Valid only when PMIC_ESM_CFG_LMIN_VALID
 *                               bit is set
 * @param   esmErrCntThr         ESM Error count Threshold value.
 *                               Valid only when PMIC_ESM_CFG_ERR_CNT_THR_VALID
 *                               bit is set
 * @param   esmEnDrv             ESM ENABLE_DRV clear configuration.
 *                               Valid values: \ref Pmic_EsmEnDrvSel.
 *                              #endif Valid only when PMIC_ESM_CFG_EN_DRV_VALID
 *                               bit is set
 * @param   esmMode              ESM mode select.
 *                               Valid values: \ref Pmic_EsmMode.
 *                               Valid only when PMIC_ESM_CFG_MODE_VALID
 *                               bit is set
 *
 * @ingroup Pmic_ESMStructures
 */
typedef struct Pmic_EsmCfg_s {
    uint32_t validParams;
    uint32_t esmDelay1_us;
    uint32_t esmDelay2_us;
    uint16_t esmHmax_us;
    uint16_t esmHmin_us;
    uint16_t esmLmax_us;
    uint16_t esmLmin_us;
    uint8_t esmErrCntThr;
    bool esmEnDrv;
    bool esmMode;
}
Pmic_EsmCfg_t;

/**
 * @brief   PMIC ESM Interrupt Configuration Structure.
 *
 * @param   esmPinIntr             ESM Pin Interrupt configuration.
 *                                 Valid values: \ref Pmic_EsmIntr.
 * @param   esmFailIntr            ESM Fail Interrupt configuration.
 *                                 Valid values: \ref Pmic_EsmIntr.
 * @param   esmRstIntr             ESM Reset Interrupt configuration.
 *                                 Valid values: \ref Pmic_EsmIntr.
 *
 * @ingroup Pmic_ESMStructures
 */
typedef struct Pmic_EsmIntrCfg_s {
    bool esmPinIntr;
    bool esmFailIntr;
    bool esmRstIntr;
}
Pmic_EsmIntrCfg_t;

/**
 * @}
 */
/* End of Pmic_ESMStructures */

/*==========================================================================*/
/*                         Function Declarations                            */
/*==========================================================================*/

/**
 * @defgroup Pmic_ESMFunctions PMIC Error State Machine Functions
 * @{
 * @ingroup Pmic_ESM
 * @brief Contains functions used in the ESM module of PMIC driver.
 */

/**
 * @brief Convert a boolean value to its equivalent uint8_t representation.
 * This function converts a boolean value to its equivalent uint8_t representation,
 * where `true` is converted to 1 and `false` is converted to 0. The converted
 * uint8_t value is returned. This utility function is useful in scenarios where
 * boolean values need to be converted to uint8_t values for compatibility or
 * consistency purposes.
 *
 * @param esmVal Boolean value to be converted.
 * @return esmU8Val Equivalent uint8_t representation of the boolean value.
 *
 * @ingroup Pmic_ESMFunctions
 */
static uint8_t Pmic_esmGetU8Val(bool esmVal);

/**
 * @brief Validate the parameters for Error Signaling Module (ESM).
 * This function validates the parameters required for configuring and using
 * the Error Signaling Module (ESM) feature of the PMIC. It checks whether the
 * PMIC core handle pointer is valid and whether the ESM feature is enabled in
 * the PMIC subsystem information structure. If the handle pointer is NULL or
 * if the ESM feature is disabled, it returns an appropriate error code.
 *
 * @param pPmicCoreHandle Pointer to the PMIC core handle.
 * @return pmicStatus Returns PMIC_ST_SUCCESS if the operation is successful;
 * otherwise, returns an error code indicating the validation failure.
 *
 * @ingroup Pmic_ESMFunctions
 */
static int32_t
Pmic_esmValidateParams(const Pmic_CoreHandle_t * pPmicCoreHandle);

/**
 * @brief Get the base register address for Error Signaling Module (ESM).
 * This function retrieves the base register address associated with the specified type of ESM.
 * The base register address is necessary for accessing the configuration and status registers
 * related to the ESM functionality. It takes a boolean parameter to determine the type of ESM,
 * where true indicates one type of ESM and false indicates another type. The base register address
 * is then stored in the memory location pointed to by the provided pointer.
 *
 * @param esmType Type of ESM.
 * @param pEsmBaseAddr Pointer to store the base register address.
 * @return void
 *
 * @ingroup Pmic_ESMFunctions
 */
static void Pmic_esmGetBaseRegAddr(uint8_t * pEsmBaseAddr);

/**
 * @brief Check the state of Error Signaling Module (ESM).
 * This function checks the current state of the specified ESM.
 * It queries the ESM control register to determine if the ESM is currently active or inactive.
 * If the ESM is detected to be in an active state, it returns an error code to indicate that
 * the ESM is already started.
 *
 * @param pPmicCoreHandle Pointer to the PMIC core handle.
 * @return pmicStatus Returns PMIC_ST_SUCCESS if the ESM is inactive;
 * otherwise, returns an error code (PMIC_ST_ERR_ESM_STARTED) indicating that the ESM is already active.
 */
static int32_t Pmic_esmCheckState(Pmic_CoreHandle_t * pPmicCoreHandle);

/**
 * @brief Start or stop the Error Signaling Module (ESM).
 * This function starts or stops the specified ESM based on the provided state.
 * It modifies the ESM control register to initiate or halt the ESM functionality.
 * The function first reads the current ESM state and then updates the control register accordingly.
 *
 * @param pPmicCoreHandle Pointer to the PMIC core handle.
 * @param esmState State of ESM (true for start, false for stop).
 * @return pmicStatus Returns PMIC_ST_SUCCESS if the operation is successful;
 * otherwise, returns an error code indicating the failure to modify the ESM state.
 */
static int32_t Pmic_esmXStart(Pmic_CoreHandle_t * pPmicCoreHandle,
    const bool esmState);

/**
 * @brief Enable or disable Error Signaling Module (ESM).
 * This function enables or disables the specified ESM based on the provided toggle value.
 * It configures the ESM control register to activate or deactivate the ESM functionality.
 * Before modifying the configuration, it checks if the ESM is already started.
 *
 * @param pPmicCoreHandle Pointer to the PMIC core handle.
 * @param esmToggle Toggle value for ESM (true for enable, false for disable).
 * @return pmicStatus Returns PMIC_ST_SUCCESS if the operation is successful;
 * otherwise, returns an error code indicating the failure to modify the ESM configuration.
 *
 * @ingroup Pmic_ESMFunctions
 */
static int32_t Pmic_esmXEnable(Pmic_CoreHandle_t * pPmicCoreHandle,
    const bool esmToggle);

/**
 * @brief Set the delay value for the Error Signaling Module (ESM).
 * This function configures the delay value for the ESM, which determines the time duration
 * before the ESM triggers an error response after detecting a fault condition.
 * It takes the PMIC core handle, ESM configuration, and base register address for ESM as input parameters.
 * The delay value is specified in microseconds and must be within the valid range defined by
 * PMIC_ESM_DELAY_MICROSEC_MIN and PMIC_ESM_DELAY_MICROSEC_MAX.
 * The function calculates the appropriate register value based on the provided delay and sends
 * the data to the corresponding delay register in the PMIC.
 * If the delay value provided is outside the valid range, the function returns an error code.
 *
 * @param pPmicCoreHandle Pointer to the PMIC core handle.
 * @param esmCfg ESM configuration containing the delay value.
 * @return pmicStatus Returns PMIC_ST_SUCCESS if the operation is successful;
 * otherwise, returns an error code (PMIC_ST_ERR_INV_ESM_VAL) if the provided delay value is invalid.
 *
 * @ingroup Pmic_ESMFunctions
 */
static int32_t Pmic_esmSetDelay1Value(Pmic_CoreHandle_t * pPmicCoreHandle,
    const Pmic_EsmCfg_t esmCfg);

/**
 * @brief Set the delay value for the secondary Error Signaling Module (ESM).
 * This function sets the delay value for the secondary ESM (ESM delay 2).
 * It follows a similar procedure to Pmic_esmSetDelay1Value, but configures a different delay register.
 * The delay value specifies the time interval before the secondary ESM triggers an error response
 * after detecting a fault condition.
 *
 * @param pPmicCoreHandle Pointer to the PMIC core handle.
 * @param esmCfg ESM configuration containing the delay value.
 * @return pmicStatus Returns PMIC_ST_SUCCESS if the operation is successful;
 * otherwise, returns an error code (PMIC_ST_ERR_INV_ESM_VAL) if the provided delay value is invalid.
 *
 * @ingroup Pmic_ESMFunctions
 */
static int32_t Pmic_esmSetDelay2Value(Pmic_CoreHandle_t * pPmicCoreHandle,
    const Pmic_EsmCfg_t esmCfg);

/**
 * @brief Set the error count threshold value for the Error Signaling Module (ESM).
 * This function configures the error count threshold for the ESM, which determines the number of
 * consecutive errors required to trigger an error response.
 * It takes the PMIC core handle, ESM configuration, and base register address for ESM as input parameters.
 * The function reads the current ESM error status register, updates the error count threshold field,
 * and writes the modified data back to the register.
 *
 * @param pPmicCoreHandle Pointer to the PMIC core handle.
 * @param esmCfg ESM configuration containing the error count threshold value.
 * @return pmicStatus Returns PMIC_ST_SUCCESS if the operation is successful;
 * otherwise, returns an error code.
 *
 * @ingroup Pmic_ESMFunctions
 */
static int32_t Pmic_esmSetErrCntThrValue(Pmic_CoreHandle_t * pPmicCoreHandle,
    const Pmic_EsmCfg_t esmCfg);

/**
 * @brief Set the maximum high pulse duration value for the Error Signaling Module (ESM).
 * This function configures the maximum high pulse duration for the ESM, which specifies
 * the maximum duration of a high pulse signal that the ESM can generate in response to an error.
 * It takes the PMIC core handle, ESM configuration, and base register address for ESM as input parameters.
 * The function calculates the appropriate register value based on the provided pulse duration,
 * ensuring it falls within the valid range defined by PMIC_ESM_PWM_PULSE_MICROSEC_MIN
 * and PMIC_ESM_PWM_PULSE_MICROSEC_MAX.
 * If the provided pulse duration is outside the valid range, the function returns an error code.
 *
 * @param pPmicCoreHandle Pointer to the PMIC core handle.
 * @param esmCfg ESM configuration containing the maximum high pulse duration value.
 * @return pmicStatus Returns PMIC_ST_SUCCESS if the operation is successful;
 * otherwise, returns an error code (PMIC_ST_ERR_INV_ESM_VAL) if the provided pulse duration is invalid.
 *
 * @ingroup Pmic_ESMFunctions
 */
static int32_t Pmic_esmSetHmaxValue(Pmic_CoreHandle_t * pPmicCoreHandle,
    const Pmic_EsmCfg_t esmCfg);

/**
 * @brief Set the minimum high pulse duration value for the Error Signaling Module (ESM).
 * This function configures the minimum high pulse duration for the ESM, which specifies
 * the minimum duration of a high pulse signal that the ESM can generate in response to an error.
 * It takes the PMIC core handle, ESM configuration, and base register address for ESM as input parameters.
 * Similar to Pmic_esmSetHmaxValue, this function calculates the appropriate register value
 * based on the provided pulse duration and ensures it falls within the valid range.
 * If the provided pulse duration is outside the valid range, the function returns an error code.
 *
 * @param pPmicCoreHandle Pointer to the PMIC core handle.
 * @param esmCfg ESM configuration containing the minimum high pulse duration value.
 * @return pmicStatus Returns PMIC_ST_SUCCESS if the operation is successful;
 * otherwise, returns an error code (PMIC_ST_ERR_INV_ESM_VAL) if the provided pulse duration is invalid.
 *
 * @ingroup Pmic_ESMFunctions
 */
static int32_t Pmic_esmSetHminValue(Pmic_CoreHandle_t * pPmicCoreHandle,
    const Pmic_EsmCfg_t esmCfg);

/**
 * @brief Get the maximum low pulse duration value for the Error Signaling Module (ESM).
 * This function retrieves the maximum duration of a low pulse signal that the ESM can generate
 * in response to an error condition.
 * It takes the PMIC core handle, a pointer to store the ESM configuration, and the base register address for ESM
 * as input parameters.
 * The function reads the corresponding register value, calculates the pulse duration in microseconds,
 * and updates the provided ESM configuration structure.
 *
 * @param pPmicCoreHandle Pointer to the PMIC core handle.
 * @param pEsmCfg Pointer to store the ESM configuration containing the maximum low pulse duration value.
 * @return pmicStatus Returns PMIC_ST_SUCCESS if the operation is successful;
 * otherwise, returns an error code.
 *
 * @ingroup Pmic_ESMFunctions
 */
static int32_t Pmic_esmGetLmaxValue(Pmic_CoreHandle_t * pPmicCoreHandle,
    Pmic_EsmCfg_t * pEsmCfg);

/**
 * @brief Get the minimum low pulse duration value for the Error Signaling Module (ESM).
 * This function retrieves the minimum duration of a low pulse signal that the ESM can generate
 * in response to an error condition.
 * It takes the PMIC core handle, a pointer to store the ESM configuration, and the base register address for ESM
 * as input parameters.
 * The function reads the corresponding register value, calculates the pulse duration in microseconds,
 * and updates the provided ESM configuration structure.
 *
 * @param pPmicCoreHandle Pointer to the PMIC core handle.
 * @param pEsmCfg Pointer to store the ESM configuration containing the minimum low pulse duration value.
 * @return pmicStatus Returns PMIC_ST_SUCCESS if the operation is successful;
 * otherwise, returns an error code.
 *
 * @ingroup Pmic_ESMFunctions
 */
static int32_t Pmic_esmGetLminValue(Pmic_CoreHandle_t * pPmicCoreHandle,
    Pmic_EsmCfg_t * pEsmCfg);

/**
 * @brief Set the mode for the Error Signaling Module (ESM).
 * This function configures the mode of operation for the ESM, enabling or disabling its functionality.
 * It takes the PMIC core handle, ESM configuration, and base register address for ESM as input parameters.
 * The function reads the current configuration from the device, modifies the mode according to the provided
 * ESM configuration, and writes back the updated configuration to the device.
 * The mode parameter in the ESM configuration should be set to true to enable the ESM or false to disable it.
 *
 * @param pPmicCoreHandle Pointer to the PMIC core handle.
 * @param esmCfg ESM configuration containing the mode value to set.
 * @return pmicStatus Returns PMIC_ST_SUCCESS if the operation is successful;
 * otherwise, returns an error code.
 *
 * @ingroup Pmic_ESMFunctions
 */
static int32_t Pmic_esmSetMode(Pmic_CoreHandle_t * pPmicCoreHandle,
    const Pmic_EsmCfg_t esmCfg);

/**
 * @brief Get the delay value for ESM delay 1.
 * This function retrieves the delay value programmed for ESM delay 1.
 * ESM delay 1 represents a configurable time delay used in ESM operations.
 * The function reads the delay value from the device registers, converts it to microseconds,
 * and updates the provided ESM configuration structure with the result.
 *
 * @param pPmicCoreHandle Pointer to the PMIC core handle.
 * @param pEsmCfg Pointer to store the ESM configuration containing the delay value for ESM delay 1.
 * @return pmicStatus Returns PMIC_ST_SUCCESS if the operation is successful;
 * otherwise, returns an error code.
 *
 * @ingroup Pmic_ESMFunctions
 */
static int32_t Pmic_esmGetDelay1Value(Pmic_CoreHandle_t * pPmicCoreHandle,
    Pmic_EsmCfg_t * pEsmCfg);

/**
 * @brief Get the delay value for ESM delay 2.
 * This function retrieves the delay value programmed for ESM delay 2.
 * ESM delay 2 represents a configurable time delay used in ESM operations.
 * The function reads the delay value from the device registers, converts it to microseconds,
 * and updates the provided ESM configuration structure with the result.
 *
 * @param pPmicCoreHandle Pointer to the PMIC core handle.
 * @param pEsmCfg Pointer to store the ESM configuration containing the delay value for ESM delay 2.
 * @return pmicStatus Returns PMIC_ST_SUCCESS if the operation is successful;
 * otherwise, returns an error code.
 *
 * @ingroup Pmic_ESMFunctions
 */
static int32_t Pmic_esmGetDelay2Value(Pmic_CoreHandle_t * pPmicCoreHandle,
    Pmic_EsmCfg_t * pEsmCfg);

/**
 * @brief Get the error count threshold value for the Error Signaling Module (ESM).
 * This function retrieves the error count threshold value configured for the ESM.
 * The error count threshold represents the number of consecutive errors required
 * to trigger an action or interrupt from the ESM.
 * The function reads the threshold value from the device registers and updates
 * the provided ESM configuration structure with the result.
 *
 * @param pPmicCoreHandle Pointer to the PMIC core handle.
 * @param pEsmCfg Pointer to store the ESM configuration containing the error count threshold value.
 * @return pmicStatus Returns PMIC_ST_SUCCESS if the operation is successful;
 * otherwise, returns an error code.
 *
 * @ingroup Pmic_ESMFunctions
 */
static int32_t Pmic_esmGetErrCntThrValue(Pmic_CoreHandle_t * pPmicCoreHandle,
    Pmic_EsmCfg_t * pEsmCfg);

/**
 * @brief Get the mode value for the Error Signaling Module (ESM).
 * This function retrieves the current mode setting of the ESM, indicating whether it is enabled or disabled.
 * The function reads the mode value from the device registers and updates
 * the provided ESM configuration structure accordingly.
 *
 * @param pPmicCoreHandle Pointer to the PMIC core handle.
 * @param pEsmCfg Pointer to store the ESM configuration containing the mode value.
 * @return pmicStatus Returns PMIC_ST_SUCCESS if the operation is successful;
 * otherwise, returns an error code.
 *
 * @ingroup Pmic_ESMFunctions
 */
static int32_t Pmic_esmGetModeValue(Pmic_CoreHandle_t * pPmicCoreHandle,
    Pmic_EsmCfg_t * pEsmCfg);

/**
 * @brief Start or stop the Error Signaling Module (ESM) based on the specified type and state.
 * This function allows the user to start or stop the ESM module, controlling its operational state.
 * It takes the PMIC core handle, type of ESM module, and desired state (start or stop) as input parameters.
 * The function internally determines the base register address for the specified ESM module type,
 * then invokes the appropriate function to initiate the start or stop action.
 * Upon successful execution, it returns PMIC_ST_SUCCESS; otherwise, it returns an error code.
 *
 * @param pPmicCoreHandle Pointer to the PMIC core handle.
 * @param esmType Type of ESM module.
 * @param esmState State of ESM module (start or stop).
 * @return pmicStatus Returns PMIC_ST_SUCCESS if the operation is successful;
 * otherwise, returns an error code.
 *
 * @ingroup Pmic_ESMFunctions
 */
int32_t Pmic_esmStart(Pmic_CoreHandle_t * pPmicCoreHandle,
    const bool esmState);

/**
 * @brief Enable or disable the Error Signaling Module (ESM) based on the specified type and toggle.
 * This function provides the capability to enable or disable the ESM module, controlling its operational mode.
 * It accepts the PMIC core handle, type of ESM module, and toggle value (true for enable, false for disable) as inputs.
 * The function determines the base register address for the specified ESM module type internally,
 * then calls the appropriate function to enable or disable the module as per the provided toggle value.
 * Upon successful execution, it returns PMIC_ST_SUCCESS; otherwise, it returns an error code.
 *
 * @param pPmicCoreHandle Pointer to the PMIC core handle.
 * @param esmType Type of ESM module.
 * @param esmToggle Toggle to enable or disable the ESM module.
 * @return pmicStatus Returns PMIC_ST_SUCCESS if the operation is successful;
 * otherwise, returns an error code.
 *
 * @ingroup Pmic_ESMFunctions
 */
int32_t Pmic_esmEnable(Pmic_CoreHandle_t * pPmicCoreHandle,
    const bool esmToggle);

/**
 * @brief Get the enable state of the Error Signaling Module (ESM) based on the specified type.
 * This function retrieves the current enable state of the ESM module, indicating whether it is enabled or disabled.
 * It takes the PMIC core handle, type of ESM module, and a pointer to store the enable state as input.
 * The function internally determines the base register address for the specified ESM module type,
 * reads the relevant configuration from the device registers, and updates the provided pointer with the enable state.
 * Upon successful execution, it returns PMIC_ST_SUCCESS; otherwise, it returns an error code.
 *
 * @param pPmicCoreHandle Pointer to the PMIC core handle.
 * @param esmType Type of ESM module.
 * @param pEsmState Pointer to store the enable state of the ESM module.
 * @return pmicStatus Returns PMIC_ST_SUCCESS if the operation is successful;
 * otherwise, returns an error code.
 *
 * @ingroup Pmic_ESMFunctions
 */
int32_t Pmic_esmGetEnableState(Pmic_CoreHandle_t * pPmicCoreHandle,
    bool * pEsmState);

/**
 * @brief Set error count threshold and endrv/clr mode configuration for the Error Signaling Module (ESM).
 * This function configures the error count threshold and endrv/clr mode for the ESM module.
 * It accepts the PMIC core handle, ESM configuration containing the required settings,
 * and the base register address for the ESM module as input parameters.
 * The function checks for the validity of the provided parameters, sets the error count threshold if valid,
 * and configures the endrv/clr mode accordingly.
 * Upon successful execution, it returns PMIC_ST_SUCCESS; otherwise, it returns an error code.
 *
 * @param pPmicCoreHandle Pointer to the PMIC core handle.
 * @param esmCfg ESM configuration containing error count threshold and endrv/clr mode settings.
 * @return pmicStatus Returns PMIC_ST_SUCCESS if the operation is successful;
 * otherwise, returns an error code.
 *
 * @ingroup Pmic_ESMFunctions
 */
static int32_t
Pmic_esmSetErrcntthresholdEndrvClrModeCfg(Pmic_CoreHandle_t * pPmicCoreHandle,
    const Pmic_EsmCfg_t esmCfg);

/**
 * @brief Set Hmax, Hmin, Lmax, and Lmin configuration for the Error Signaling Module (ESM).
 * This function configures the Hmax, Hmin, Lmax, and Lmin parameters for the ESM module.
 * It accepts the PMIC core handle, ESM configuration containing the required settings,
 * and the base register address for the ESM module as input parameters.
 * The function individually sets each parameter if it is valid in the provided ESM configuration.
 * Upon successful execution, it returns PMIC_ST_SUCCESS; otherwise, it returns an error code.
 *
 * @param pPmicCoreHandle Pointer to the PMIC core handle.
 * @param esmCfg ESM configuration containing Hmax, Hmin, Lmax, and Lmin settings.
 * @return pmicStatus Returns PMIC_ST_SUCCESS if the operation is successful;
 * otherwise, returns an error code.
 *
 * @ingroup Pmic_ESMFunctions
 */
static int32_t
Pmic_esmSetHmaxHminLmaxLminCfg(Pmic_CoreHandle_t * pPmicCoreHandle,
    const Pmic_EsmCfg_t esmCfg);

/**
 * @brief Set the configuration parameters for the Error Signaling Module (ESM).
 * This function allows the user to configure various parameters of the ESM module, such as delays,
 * threshold values, and operational modes, based on the specified type and configuration.
 * It takes the PMIC core handle, ESM configuration, and base register address for the ESM module as input.
 * The function checks for the validity of the provided configuration parameters and sets them accordingly,
 * invoking internal functions to handle each parameter individually.
 * Upon successful execution, it returns PMIC_ST_SUCCESS; otherwise, it returns an error code.
 *
 * @param pPmicCoreHandle Pointer to the PMIC core handle.
 * @param esmCfg ESM configuration containing the parameters to be set.
 * @return pmicStatus Returns PMIC_ST_SUCCESS if the operation is successful;
 * otherwise, returns an error code.
 *
 * @ingroup Pmic_ESMFunctions
 */
static int32_t Pmic_esmSetConfig(Pmic_CoreHandle_t * pPmicCoreHandle,
    const Pmic_EsmCfg_t esmCfg);

/**
 * @brief Retrieve the Hmax, Hmin, Lmax, and Lmin configuration parameters for the Error Signaling Module (ESM).
 * This function retrieves the configured values of Hmax, Hmin, Lmax, and Lmin for the ESM module
 * and stores them in the provided ESM configuration structure.
 * It takes the PMIC core handle, a pointer to the ESM configuration structure, and the base register address
 * for the ESM module as input parameters.
 * The function individually retrieves each parameter if it is valid in the provided ESM configuration,
 * invoking internal functions to handle each parameter retrieval.
 * Upon successful execution, it returns PMIC_ST_SUCCESS; otherwise, it returns an error code.
 *
 * @param pPmicCoreHandle Pointer to the PMIC core handle.
 * @param pEsmCfg Pointer to store the retrieved ESM configuration parameters.
 * @return pmicStatus Returns PMIC_ST_SUCCESS if the operation is successful;
 * otherwise, returns an error code.
 *
 * @ingroup Pmic_ESMFunctions
 */
static int32_t
Pmic_esmGetHmaxHminLmaxLminCfg(Pmic_CoreHandle_t * pPmicCoreHandle,
    Pmic_EsmCfg_t * pEsmCfg);

/**
 * @brief Retrieve the configuration parameters for the Error Signaling Module (ESM).
 * This function retrieves the current configuration parameters of the ESM module
 * based on the specified type and stores them in the provided ESM configuration structure.
 * It takes the PMIC core handle, type of ESM module, a pointer to the ESM configuration structure,
 * and the base register address for the ESM module as input parameters.
 * The function retrieves each parameter individually if it is valid in the provided ESM configuration,
 * invoking internal functions to handle each parameter retrieval.
 * Upon successful execution, it returns PMIC_ST_SUCCESS; otherwise, it returns an error code.
 *
 * @param pPmicCoreHandle Pointer to the PMIC core handle.
 * @param pEsmCfg Pointer to store the retrieved ESM configuration parameters.
 * @return pmicStatus Returns PMIC_ST_SUCCESS if the operation is successful;
 * otherwise, returns an error code.
 *
 * @ingroup Pmic_ESMFunctions
 */
static int32_t Pmic_esmGetConfig(Pmic_CoreHandle_t * pPmicCoreHandle,
    Pmic_EsmCfg_t * pEsmCfg);

/**
 * @brief Set the configuration for the Error Signaling Module (ESM).
 * This function sets the configuration parameters for the ESM module based on the specified type and configuration.
 * It verifies the validity of the provided parameters, checks the current state of the ESM module,
 * and then proceeds to set the configuration parameters using internal functions.
 * Upon successful execution, it returns PMIC_ST_SUCCESS; otherwise, it returns an error code.
 *
 * @param pPmicCoreHandle Pointer to the PMIC core handle.
 * @param esmCfg ESM configuration containing the parameters to be set.
 * @return pmicStatus Returns PMIC_ST_SUCCESS if the operation is successful;
 * otherwise, returns an error code.
 *
 * @ingroup Pmic_ESMFunctions
 */
int32_t Pmic_esmSetConfiguration(Pmic_CoreHandle_t * pPmicCoreHandle,
    const Pmic_EsmCfg_t esmCfg);

/**
 * @brief Get the configuration for the Error Signaling Module (ESM).
 * This function retrieves the current configuration parameters of the ESM module based on the specified type
 * and stores them in the provided ESM configuration structure.
 * It verifies the validity of the provided parameters and then proceeds to retrieve the configuration parameters
 * using internal functions.
 * Upon successful execution, it returns PMIC_ST_SUCCESS; otherwise, it returns an error code.
 *
 * @param pPmicCoreHandle Pointer to the PMIC core handle.
 * @param esmType Type of ESM module.
 * @param pEsmCfg Pointer to store the retrieved ESM configuration parameters.
 * @return pmicStatus Returns PMIC_ST_SUCCESS if the operation is successful;
 * otherwise, returns an error code.
 *
 * @ingroup Pmic_ESMFunctions
 */
int32_t Pmic_esmGetConfiguration(Pmic_CoreHandle_t * pPmicCoreHandle,
    Pmic_EsmCfg_t * pEsmCfg);

/**
 * @brief Get the error count for the Error Signaling Module (ESM).
 * This function retrieves the current error count of the ESM module based on the specified type
 * and stores it in the provided variable.
 * It verifies the validity of the provided parameters and then proceeds to retrieve the error count
 * using internal functions.
 * Upon successful execution, it returns PMIC_ST_SUCCESS; otherwise, it returns an error code.
 *
 * @param pPmicCoreHandle Pointer to the PMIC core handle.
 * @param esmType Type of ESM module.
 * @param pEsmErrCnt Pointer to store the retrieved error count.
 * @return pmicStatus Returns PMIC_ST_SUCCESS if the operation is successful;
 * otherwise, returns an error code.
 *
 * @ingroup Pmic_ESMFunctions
 */
int32_t Pmic_esmGetErrCnt(Pmic_CoreHandle_t * pPmicCoreHandle,
    uint8_t * pEsmErrCnt);

/**
 * @brief Get the status of the Error Signaling Module (ESM).
 * This function retrieves the current status of the ESM module based on the specified type
 * and stores it in the provided variable.
 * It verifies the validity of the provided parameters and then proceeds to retrieve the status
 * using internal functions.
 * Upon successful execution, it returns PMIC_ST_SUCCESS; otherwise, it returns an error code.
 *
 * @param pPmicCoreHandle Pointer to the PMIC core handle.
 * @param esmType Type of ESM module.
 * @param pEsmState Pointer to store the retrieved status of the ESM module.
 * @return pmicStatus Returns PMIC_ST_SUCCESS if the operation is successful;
 * otherwise, returns an error code.
 *
 * @ingroup Pmic_ESMFunctions
 */
int32_t Pmic_esmGetStatus(Pmic_CoreHandle_t * pPmicCoreHandle,
    uint8_t *pEsmState);

/**
 * @brief Set the maximum low pulse duration value for ESM.
 * This function sets the maximum low pulse duration value for ESM.
 *
 * @param pPmicCoreHandle Pointer to the PMIC core handle.
 * @param esmCfg ESM configuration.
 * @param esmBaseRegAddr Base register address for ESM.
 * @return pmicStatus Returns PMIC_ST_SUCCESS if the operation is successful;
 * otherwise, returns an error code.
 */
static int32_t Pmic_esmSetLmaxValue(Pmic_CoreHandle_t * pPmicCoreHandle,
    const Pmic_EsmCfg_t esmCfg);

/**
 * @brief Set the minimum low pulse duration value for ESM.
 * This function sets the minimum low pulse duration value for ESM.
 *
 * @param pPmicCoreHandle Pointer to the PMIC core handle.
 * @param esmCfg ESM configuration.
 * @param esmBaseRegAddr Base register address for ESM.
 * @return pmicStatus Returns PMIC_ST_SUCCESS if the operation is successful;
 * otherwise, returns an error code.
 */
static int32_t Pmic_esmSetLminValue(Pmic_CoreHandle_t * pPmicCoreHandle,
    const Pmic_EsmCfg_t esmCfg);

/**
 * @brief Get the maximum high pulse duration value for ESM.
 * This function gets the maximum high pulse duration value for ESM.
 *
 * @param pPmicCoreHandle Pointer to the PMIC core handle.
 * @param pEsmCfg Pointer to store the ESM configuration.
 * @param esmBaseRegAddr Base register address for ESM.
 * @return pmicStatus Returns PMIC_ST_SUCCESS if the operation is successful;
 * otherwise, returns an error code.
 */
static int32_t Pmic_esmGetHmaxValue(Pmic_CoreHandle_t * pPmicCoreHandle,
    Pmic_EsmCfg_t * pEsmCfg);

/**
 * @brief Get the minimum high pulse duration value for ESM.
 * This function gets the minimum high pulse duration value for ESM.
 *
 * @param pPmicCoreHandle Pointer to the PMIC core handle.
 * @param pEsmCfg Pointer to store the ESM configuration.
 * @param esmBaseRegAddr Base register address for ESM.
 * @return pmicStatus Returns PMIC_ST_SUCCESS if the operation is successful;
 * otherwise, returns an error code.
 */
static int32_t Pmic_esmGetHminValue(Pmic_CoreHandle_t * pPmicCoreHandle,
    Pmic_EsmCfg_t * pEsmCfg);



/**
 * @}
 */
/* End of Pmic_ESMFunctions */

/**
 * @}
 */
/* End of Pmic_ESM */

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* PMIC_ESM_H_ */
