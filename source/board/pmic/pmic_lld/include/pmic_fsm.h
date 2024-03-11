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
 *   @file    pmic_fsm.h
 *
 *   @brief   This file contains the default MACRO's and function definitions
 * for PMIC FSM state configuration
 *
 */

#ifndef PMIC_FSM_H_
#define PMIC_FSM_H_

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include "pmic_core.h"
#include "pmic_core_priv.h"
#include "pmic_io_priv.h"
#include "pmic_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @defgroup Pmic_FSM PMIC Finite State Machine
 * @{
 * @brief Contains definitions related to PMIC FSM functionality.
 */

/* ========================================================================== */
/*                             Macros & Typedefs                              */
/* ========================================================================== */

/**
 * @defgroup Pmic_FSMMacros PMIC Finite State Machine Macros
 * @{
 * @ingroup Pmic_FSM
 * @brief Contains macros used in the FSM module of PMIC driver.
 */

/**
 * @brief Enable bit value for PMIC BIST.
 * This macro defines the value to enable the Built-In Self-Test (BIST) feature for the Power Management Integrated Circuit (PMIC).
 *
 * @ingroup Pmic_FSMMacros
 */
#define PMIC_BIST_BIT_ENABLE    (0x01U)

/**
 * @brief Disable bit value for PMIC BIST.
 * This macro defines the value to disable the Built-In Self-Test (BIST) feature for the Power Management Integrated Circuit (PMIC).
 *
 * @ingroup Pmic_FSMMacros
 */
#define PMIC_BIST_BIT_DISABLE   (0x00U)

/**
 * @brief Selection mode for PMIC BIST.
 * This macro defines the selection mode for the PMIC Built-In Self-Test (BIST).
 *
 * @ingroup Pmic_FSMMacros
 */
#define PMIC_BIST_MODE_SEL      (0x00U)

/**
 * @brief Selection mode for PMIC LBIST.
 * This macro defines the selection mode for the PMIC Logic Built-In Self-Test (LBIST).
 *
 * @ingroup Pmic_FSMMacros
 */
#define PMIC_LBIST_MODE_SEL     (0x01U)

/**
 * @brief Selection mode for PMIC ABIST.
 * This macro defines the selection mode for the PMIC Analog Built-In Self-Test (ABIST).
 *
 * @ingroup Pmic_FSMMacros
 */
#define PMIC_ABIST_MODE_SEL     (0x02U)

/**
 * @}
 */
/* End of Pmic_FSMMacros */

/*==========================================================================*/
/*                         Structures and Enums                             */
/*==========================================================================*/

/**
 * @defgroup Pmic_FSMStructures PMIC FSM Structures
 * @{
 * @ingroup Pmic_FSM
 * @brief Contains structures used in the FSM module of PMIC driver.
 */

/**
 * @brief Structure representing PMIC Finite State Machine (FSM) configuration.
 * This structure holds the configuration parameters for the Finite State Machine (FSM) of the Power Management Integrated Circuit (PMIC).
 *
 * @param   validParams         Bit field indicating the validity of each parameter.
 * @param   fastBistEn          Enable bit for Fast Built-In Self-Test (BIST) feature.
 * @param   lpStandbySel        Low-Power Standby selection flag.
 * @param   ilimIntfsmCtrlEn    Current Limit Interface Finite State Machine (FSM) control enable.
 * @param   fsmStarupDestSel    FSM startup destination selection.
 *
 * @ingroup Pmic_FSMStructures
 */
typedef struct Pmic_FsmCfg_s {
    uint8_t validParams;
    uint8_t fastBistEn;
    bool lpStandbySel;
    uint8_t ilimIntfsmCtrlEn;
    uint8_t fsmStarupDestSel;
} Pmic_FsmCfg_t;

/**
 * @}
 */
/* End of Pmic_FSMStructures */

/*==========================================================================*/
/*                         Function Declarations                            */
/*==========================================================================*/

/**
 * @defgroup Pmic_FSMFunctions PMIC Finite State Machine Functions
 * @{
 * @ingroup Pmic_FSM
 * @brief Contains functions used in the FSM module of PMIC driver.
 */


/**
 * @brief Retrieves configuration fields for standby state in the PMIC Finite State Machine (FSM).
 *
 * This function retrieves the register address, bit position, bit mask, bit value, and device state
 * corresponding to the given next state in the PMIC FSM.
 *
 * @param   pmicNextState  The next state in the PMIC FSM.
 * @param   pRegAddr       Pointer to store the register address.
 * @param   pBitPos        Pointer to store the bit position.
 * @param   pBitMask       Pointer to store the bit mask.
 * @param   pBitVal        Pointer to store the bit value.
 * @param   pDeviceState   Pointer to store the device state.
 *
 * @ingroup Pmic_FSMFunctions
 */
void Pmic_fsmGetstandByCfgRegFields(uint8_t pmicNextState, uint8_t * pRegAddr,
    uint8_t * pBitPos, uint8_t * pBitMask,
    uint8_t * pBitVal, uint8_t * pDeviceState);

/**
 * @brief Retrieves the bit position and bit mask for configuring nsleep signals in the PMIC FSM.
 *
 * This function determines the bit position and bit mask corresponding to the given nsleep type,
 * enabling configuration of nsleep signals in the Power Management Integrated Circuit (PMIC) Finite State Machine (FSM).
 *
 * @param nsleepType   The type of nsleep signal (NSLEEP1_SIGNAL or NSLEEP2_SIGNAL).
 * @param pBitPos      Pointer to store the bit position.
 * @param pBitMask     Pointer to store the bit mask.
 *
 * @ingroup Pmic_FSMFunctions
 */
void Pmic_fsmGetNsleepMaskBitField(uint8_t nsleepType, uint8_t * pBitPos,
    uint8_t * pBitMask);

/**
 * @brief Sets the Standby to Run (S2R) state in the Power Management Integrated Circuit (PMIC) FSM.
 *
 * This function configures the PMIC to transition from Standby to Run state. It sets the Standby Enable (STBY_EN) bit
 * in the Standby Configuration (STBY_CFG) register and sends the Standby request to the State Control (STATE_CTRL) register.
 * This operation is performed through the communication interface with the PMIC.
 *
 * @param pPmicCoreHandle Pointer to the PMIC core handle structure.
 *
 * @return Status of the operation.
 *         - PMIC_ST_SUCCESS if the operation is successful.
 *         - PMIC_ST_ERR_INV_HANDLE if the PMIC core handle pointer is NULL.
 *
 * @ingroup Pmic_FSMFunctions
 */
int32_t Pmic_setS2RState(Pmic_CoreHandle_t * pPmicCoreHandle);

/**
 * @brief Retrieves the current device state configuration from the Power Management Integrated Circuit (PMIC) FSM.
 *
 * This function reads the device state configuration from the State Status (STATE_STAT) register of the PMIC
 * through the communication interface. It extracts the device state information and updates the provided pointer
 * to the deviceState parameter. The function ensures that the PMIC core handle pointer is valid before proceeding
 * with the communication operation.
 *
 * @param pPmicCoreHandle Pointer to the PMIC core handle structure.
 * @param deviceState     Pointer to store the retrieved device state.
 *
 * @return Status of the operation.
 *         - PMIC_ST_SUCCESS if the operation is successful.
 *         - PMIC_ST_ERR_INV_HANDLE if the PMIC core handle pointer is NULL.
 *         - PMIC_ST_ERR_FAIL if the communication operation fails.
 *
 * @ingroup Pmic_FSMFunctions
 */
int32_t Pmic_fsmGetDeviceStateCfg(Pmic_CoreHandle_t * pPmicCoreHandle,
    uint8_t * deviceState);

/**
 * @brief Configures device request in the Power Management Integrated Circuit (PMIC) FSM.
 *
 * This function configures the device request corresponding to the given FSM state in the PMIC FSM.
 * It retrieves the register address, bit position, bit mask, and bit value for the device request
 * from the standby configuration fields. Then, it sets the appropriate bit in the corresponding
 * register through the communication interface. Additionally, it verifies the device state
 * after configuration to ensure successful transition to the desired FSM state.
 *
 * @param pPmicCoreHandle Pointer to the PMIC core handle structure.
 * @param fsmState        The FSM state for which the device request is to be configured.
 *
 * @return Status of the operation.
 *         - PMIC_ST_SUCCESS if the operation is successful.
 *         - PMIC_ST_ERR_INV_HANDLE if the PMIC core handle pointer is NULL.
 *         - PMIC_ST_ERR_FAIL if the communication operation fails.
 *         - PMIC_ST_ERR_INV_FSM_MODE if the device state does not match the configured FSM state.
 *
 * @ingroup Pmic_FSMFunctions
 */
int32_t Pmic_fsmDeviceRequestCfg(Pmic_CoreHandle_t * pPmicCoreHandle,
    uint8_t fsmState);

/**
 * @brief Sets the state of the Power Management Integrated Circuit (PMIC) Finite State Machine (FSM).
 *
 * This function configures the PMIC to transition to the specified next state in the FSM.
 * It determines the appropriate device request configuration based on the given next state
 * and calls the corresponding configuration function. If the next state is Standby to Run (S2R) state,
 * it directly calls the function to set the S2R state. If the next state is not recognized,
 * it defaults to configuring the Standby state. The function returns the status of the operation.
 *
 * @param pPmicCoreHandle Pointer to the PMIC core handle structure.
 * @param pmicNextState   The next state to which the PMIC FSM should transition.
 *
 * @return Status of the operation.
 *         - PMIC_ST_SUCCESS if the operation is successful.
 *         - PMIC_ST_ERR_INV_HANDLE if the PMIC core handle pointer is NULL.
 *         - PMIC_ST_ERR_FAIL if the device request configuration fails.
 *
 * @ingroup Pmic_FSMFunctions
 */
int32_t Pmic_setState(Pmic_CoreHandle_t * pPmicCoreHandle,
    uint8_t pmicNextState);

/**
 * @brief Requests runtime Built-In Self-Test (BIST) for the Power Management Integrated Circuit (PMIC).
 *
 * This function enables the runtime Built-In Self-Test (BIST) feature for the PMIC by setting the corresponding
 * enable bit in the BIST control register. It performs this operation through the communication interface
 * with the PMIC. The function ensures that the PMIC core handle pointer is valid before proceeding with
 * the communication operation. It returns the status of the operation, indicating success or failure.
 *
 * @param pPmicCoreHandle Pointer to the PMIC core handle structure.
 *
 * @return Status of the operation.
 *         - PMIC_ST_SUCCESS if the operation is successful.
 *         - PMIC_ST_ERR_INV_HANDLE if the PMIC core handle pointer is NULL.
 *         - PMIC_ST_ERR_FAIL if the communication operation fails.
 *
 * @ingroup Pmic_FSMFunctions
 */
int32_t Pmic_fsmRequestRuntimeBist(Pmic_CoreHandle_t * pPmicCoreHandle);

/**
 * @brief Sets the mission state of the Power Management Integrated Circuit (PMIC) Finite State Machine (FSM).
 *
 * This function sets the mission state of the PMIC FSM to the specified state. It first validates the PMIC core handle
 * pointer and ensures that the provided state is within the valid range. Then, it calls the function to set the state
 * of the PMIC FSM using the specified PMIC core handle and state. The function returns the status of the operation,
 * indicating success or failure.
 *
 * @param pPmicCoreHandle Pointer to the PMIC core handle structure.
 * @param pmicState       The mission state to be set for the PMIC FSM.
 *
 * @return Status of the operation.
 *         - PMIC_ST_SUCCESS if the operation is successful.
 *         - PMIC_ST_ERR_INV_HANDLE if the PMIC core handle pointer is NULL.
 *         - PMIC_ST_ERR_INV_PARAM if the provided state is invalid.
 *         - PMIC_ST_ERR_FAIL if setting the state fails.
 *
 * @ingroup Pmic_FSMFunctions
 */
int32_t Pmic_fsmSetMissionState(Pmic_CoreHandle_t * pPmicCoreHandle,
    const uint8_t pmicState);

/**
 * @brief Sets the mask for nsleep signal in the Power Management Integrated Circuit (PMIC).
 *
 * This function sets or clears the mask for the nsleep signal specified by the nsleepType parameter.
 * It retrieves the current configuration of nsleep signal mask from the Wake Configuration (WAKE_CFG) register,
 * modifies the appropriate bit position and bit mask based on the nsleepType, and updates the maskEnable
 * value accordingly. Then, it sends the updated configuration back to the PMIC through the communication interface.
 * The function ensures that the PMIC core handle pointer is valid before proceeding with the communication operation.
 * It returns the status of the operation, indicating success or failure.
 *
 * @param pPmicCoreHandle Pointer to the PMIC core handle structure.
 * @param nsleepType      The type of nsleep signal for which the mask is to be set.
 * @param maskEnable      Boolean value indicating whether to enable (true) or disable (false) the mask.
 *
 * @return Status of the operation.
 *         - PMIC_ST_SUCCESS if the operation is successful.
 *         - PMIC_ST_ERR_INV_HANDLE if the PMIC core handle pointer is NULL.
 *         - PMIC_ST_ERR_FAIL if the communication operation fails.
 *
 * @ingroup Pmic_FSMFunctions
 */
int32_t Pmic_fsmSetNsleepSignalMask(Pmic_CoreHandle_t * pPmicCoreHandle,
    const uint8_t nsleepType, const bool maskEnable);

/**
 * @brief Gets the status of nsleep signal mask for the specified nsleep type in the Power Management Integrated Circuit (PMIC).
 *
 * This function retrieves the current status of the nsleep signal mask for the specified nsleep type
 * from the Wake Configuration (WAKE_CFG) register of the PMIC. It first ensures that the PMIC core handle
 * pointer is valid and that the pointer to store the nsleep status is not NULL. Then, it reads the relevant
 * configuration from the register and updates the status pointer accordingly. The function returns the status
 * of the operation, indicating success or failure.
 *
 * @param pPmicCoreHandle Pointer to the PMIC core handle structure.
 * @param nsleepType      The type of nsleep signal for which the mask status is to be retrieved.
 * @param pNsleepStat     Pointer to store the status of the nsleep signal mask.
 *
 * @return Status of the operation.
 *         - PMIC_ST_SUCCESS if the operation is successful.
 *         - PMIC_ST_ERR_INV_HANDLE if the PMIC core handle pointer is NULL.
 *         - PMIC_ST_ERR_NULL_PARAM if the pointer to store the nsleep status is NULL.
 *         - PMIC_ST_ERR_FAIL if the communication operation fails.
 *
 * @ingroup Pmic_FSMFunctions
 */
int32_t Pmic_fsmGetNsleepSignalMaskStat(Pmic_CoreHandle_t * pPmicCoreHandle,
    const uint8_t nsleepType, bool * pNsleepStat);

/**
 * @brief Enables or disables Fast Built-In Self-Test (BIST) in the Power Management Integrated Circuit (PMIC).
 *
 * This function enables or disables the Fast Built-In Self-Test (BIST) feature in the PMIC based on the configuration
 * provided in the `fsmCfg` parameter. It first determines the appropriate register address, bit mask, and bit position
 * for the BIST control register. Then, it reads the current configuration from the register, modifies the BIST enable bit
 * based on the configuration, and sends the updated configuration back to the PMIC through the communication interface.
 * The function ensures that the PMIC core handle pointer is valid before proceeding with the communication operation.
 * It returns the status of the operation, indicating success or failure.
 *
 * @param pPmicCoreHandle Pointer to the PMIC core handle structure.
 * @param fsmCfg          Configuration for the Fast Built-In Self-Test (BIST) feature.
 *
 * @return Status of the operation.
 *         - PMIC_ST_SUCCESS if the operation is successful.
 *         - PMIC_ST_ERR_INV_HANDLE if the PMIC core handle pointer is NULL.
 *         - PMIC_ST_ERR_FAIL if the communication operation fails.
 *
 * @ingroup Pmic_FSMFunctions
 */
static int32_t Pmic_fsmEnableFastBIST(Pmic_CoreHandle_t * pPmicCoreHandle,
    const Pmic_FsmCfg_t fsmCfg);

/**
 * @brief Retrieves the configuration of Fast Built-In Self-Test (BIST) in the Power Management Integrated Circuit (PMIC).
 *
 * This function retrieves the current configuration of the Fast Built-In Self-Test (BIST) feature from the BIST control
 * register of the PMIC. It reads the register value, extracts the BIST mode information using the appropriate bit mask
 * and bit position, and updates the `fastBistEn` field of the provided `pFsmCfg` structure accordingly. The function
 * ensures that the PMIC core handle pointer is valid before proceeding with the communication operation. It returns the
 * status of the operation, indicating success or failure.
 *
 * @param pPmicCoreHandle Pointer to the PMIC core handle structure.
 * @param pFsmCfg         Pointer to the FSM configuration structure to store the retrieved BIST configuration.
 *
 * @return Status of the operation.
 *         - PMIC_ST_SUCCESS if the operation is successful.
 *         - PMIC_ST_ERR_INV_HANDLE if the PMIC core handle pointer is NULL.
 *         - PMIC_ST_ERR_FAIL if the communication operation fails.
 *
 * @ingroup Pmic_FSMFunctions
 */
static int32_t Pmic_fsmGetFastBISTCfg(Pmic_CoreHandle_t * pPmicCoreHandle,
    Pmic_FsmCfg_t * pFsmCfg);

/**
 * @brief Enables or disables the affect of Internal Limit (ILIM) FSM control on Back LDO in the Power Management Integrated Circuit (PMIC).
 *
 * This function enables or disables the affect of Internal Limit (ILIM) Finite State Machine (FSM) control on Back LDO
 * based on the configuration provided in the `fsmCfg` parameter. It first validates the PMIC core handle pointer and then
 * retrieves the current configuration from the ILIM configuration register. Depending on the `ilimIntfsmCtrlEn` field
 * of the `fsmCfg` parameter, it either enables or disables the ILIM FSM control on Back LDO by setting or clearing
 * the appropriate bit in the register. Finally, it sends the updated configuration back to the PMIC through the
 * communication interface. The function ensures that the PMIC core handle pointer is valid before proceeding with
 * the communication operation. It returns the status of the operation, indicating success or failure.
 *
 * @param pPmicCoreHandle Pointer to the PMIC core handle structure.
 * @param fsmCfg          Configuration for enabling or disabling the affect of ILIM FSM control on Back LDO.
 *
 * @return Status of the operation.
 *         - PMIC_ST_SUCCESS if the operation is successful.
 *         - PMIC_ST_ERR_INV_HANDLE if the PMIC core handle pointer is NULL.
 *         - PMIC_ST_ERR_FAIL if the communication operation fails.
 *
 * @ingroup Pmic_FSMFunctions
 */
static int32_t Pmic_fsmEnabBckLdoIlimIntAffect(Pmic_CoreHandle_t * pPmicCoreHandle,
    const Pmic_FsmCfg_t fsmCfg);

/**
 * @brief Retrieves the configuration of ILIM FSM control affecting Buck LDO in the Power Management Integrated Circuit (PMIC).
 *
 * This function retrieves the current configuration of the Internal Limit (ILIM) Finite State Machine (FSM) control
 * affecting Buck LDO from the ILIM configuration register of the PMIC. It reads the register value, extracts the
 * ILIM FSM control information using the appropriate bit mask and bit position, and updates the `ilimIntfsmCtrlEn`
 * field of the provided `pFsmCfg` structure accordingly. The function ensures that the PMIC core handle pointer is
 * valid before proceeding with the communication operation. It returns the status of the operation, indicating success
 * or failure.
 *
 * @param pPmicCoreHandle Pointer to the PMIC core handle structure.
 * @param pFsmCfg         Pointer to the FSM configuration structure to store the retrieved ILIM FSM control configuration.
 *
 * @return Status of the operation.
 *         - PMIC_ST_SUCCESS if the operation is successful.
 *         - PMIC_ST_ERR_INV_HANDLE if the PMIC core handle pointer is NULL.
 *         - PMIC_ST_ERR_FAIL if the communication operation fails.
 *
 * @ingroup Pmic_FSMFunctions
 */
static int32_t Pmic_fsmGetBuckLdoIlimIntAftCfg(Pmic_CoreHandle_t * pPmicCoreHandle,
    Pmic_FsmCfg_t * pFsmCfg);

/**
 * @brief Sets the configuration for various Finite State Machine (FSM) features in the Power Management Integrated Circuit (PMIC).
 *
 * This function sets the configuration for various Finite State Machine (FSM) features in the PMIC based on the provided
 * `fsmCfg` parameter. It first validates the PMIC core handle pointer. Then, it checks the validity of each parameter
 * in the `fsmCfg` structure and sets the corresponding FSM feature configuration if the parameter is valid. Currently,
 * it supports enabling or disabling Fast Built-In Self-Test (BIST) and enabling or disabling the affect of Internal
 * Limit (ILIM) interrupts on Buck/LDO regulators FSM triggers. The function ensures that the PMIC core handle pointer
 * is valid before proceeding with the configuration setting. It returns the status of the operation, indicating success
 * or failure.
 *
 * @param pPmicCoreHandle Pointer to the PMIC core handle structure.
 * @param fsmCfg          Configuration for various FSM features to be set.
 *
 * @return Status of the operation.
 *         - PMIC_ST_SUCCESS if the operation is successful.
 *         - PMIC_ST_ERR_INV_HANDLE if the PMIC core handle pointer is NULL.
 *         - PMIC_ST_ERR_INVALID_CONFIG if the configuration parameter is invalid.
 *         - PMIC_ST_ERR_FAIL if setting the configuration fails.
 *
 * @ingroup Pmic_FSMFunctions
 */
int32_t Pmic_fsmSetConfiguration(Pmic_CoreHandle_t * pPmicCoreHandle,
    const Pmic_FsmCfg_t fsmCfg);

/**
 * @brief Retrieves the configuration for various Finite State Machine (FSM) features from the Power Management Integrated Circuit (PMIC).
 *
 * This function retrieves the configuration for various FSM features from the PMIC based on the provided `pFsmCfg` parameter.
 * It first validates the PMIC core handle pointer and ensures that the configuration structure pointer is not NULL.
 * Then, it checks each parameter in the `validParams` field of the `pFsmCfg` structure and retrieves the corresponding
 * FSM feature configuration if the parameter is valid. Currently, it retrieves whether Fast Built-In Self-Test (BIST)
 * is enabled and whether the affect of Internal Limit (ILIM) interrupts on Buck/LDO regulators FSM triggers is enabled.
 * The function ensures that both the PMIC core handle pointer and the configuration structure pointer are valid
 * before proceeding with the retrieval operation. It returns the status of the operation, indicating success or failure.
 *
 * @param pPmicCoreHandle Pointer to the PMIC core handle structure.
 * @param pFsmCfg         Pointer to the FSM configuration structure to store the retrieved configuration.
 *
 * @return Status of the operation.
 *         - PMIC_ST_SUCCESS if the operation is successful.
 *         - PMIC_ST_ERR_INV_HANDLE if the PMIC core handle pointer is NULL.
 *         - PMIC_ST_ERR_NULL_PARAM if the configuration structure pointer is NULL.
 *         - PMIC_ST_ERR_INVALID_CONFIG if the configuration is invalid.
 *         - PMIC_ST_ERR_FAIL if retrieving the configuration fails.
 *
 * @ingroup Pmic_FSMFunctions
 */
int32_t Pmic_fsmGetConfiguration(Pmic_CoreHandle_t * pPmicCoreHandle,
    Pmic_FsmCfg_t * pFsmCfg);


/**
 * @}
 */
/* End of Pmic_FSMFunctions */

/**
 * @}
 */
/* End of Pmic_FSM */

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* PMIC_FSM_H_ */
