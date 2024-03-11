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
 *   @file    pmic_ilim.h
 *
 *   @brief   This file contains the default MACRO's and function definitions for
 *            PMIC ILIM configuration
 *
 */

#ifndef PMIC_INC_ILIM_H_
#define PMIC_INC_ILIM_H_

#include "pmic.h"
#include "pmic_core.h"
#include "pmic_types.h"
#include "pmic_io_priv.h"
#include "pmic_core_priv.h"

/**
 * @defgroup Pmic_ILIM PMIC ILIM Module
 * @{
 * @brief Contains definitions related to PMIC ILIM functionality.
 */

/*==========================================================================*/
/*                         Structures and Enums                             */
/*==========================================================================*/

/**
 * @defgroup Pmic_ILIMStructures PMIC ILIM Structures
 * @{
 * @ingroup Pmic_ILIM
 * @brief Contains structures used in the ILIM module of PMIC driver.
 */

/**
 * @brief Structure to hold ILIM configuration register values.
 *
 * @param pldo2ILIMCfg  Bit field for PLDO2 ILIM configuration.
 * @param pldo1ILIMCfg  Bit field for PLDO1 ILIM configuration.
 * @param ldo4ILIMCfg   Bit field for LDO4 ILIM configuration.
 * @param ldo3ILIMCfg   Bit field for LDO3 ILIM configuration.
 * @param ldo2ILIMCfg   Bit field for LDO2 ILIM configuration.
 * @param ldo1ILIMCfg   Bit field for LDO1 ILIM configuration.
 *
 * @ingroup Pmic_ILIMStructures
 */
typedef struct Pmic_ilimCfgReg_s {
    uint8_t pldo2ILIMCfg;
    uint8_t pldo1ILIMCfg;
    uint8_t ldo4ILIMCfg;
    uint8_t ldo3ILIMCfg;
    uint8_t ldo2ILIMCfg;
    uint8_t ldo1ILIMCfg;
} Pmic_ilimCfgReg_t;

/**
 * @brief Structure to hold ILIM deglitch configuration register values.
 *
 * @param pldo2ILIMdglCfg  Bit field for PLDO2 ILIM DGL(de-glitch) configuration.
 * @param pldo1ILIMdglCfg  Bit field for PLDO1 ILIM DGL(de-glitch) configuration.
 * @param ldo4ILIMdglCfg   Bit field for LDO4 ILIM DGL(de-glitch) configuration.
 * @param ldo3ILIMdglCfg   Bit field for LDO3 ILIM DGL(de-glitch) configuration.
 * @param ldo2ILIMdglCfg   Bit field for LDO2 ILIM DGL(de-glitch) configuration.
 * @param ldo1ILIMdglCfg   Bit field for LDO1 ILIM DGL(de-glitch) configuration.
 *
 * @ingroup Pmic_ILIMStructures
 */
typedef struct Pmic_ilimDglCfgReg_s {
    uint8_t pldo2ILIMdglCfg;
    uint8_t pldo1ILIMdglCfg;
    uint8_t ldo4ILIMdglCfg;
    uint8_t ldo3ILIMdglCfg;
    uint8_t ldo2ILIMdglCfg;
    uint8_t ldo1ILIMdglCfg;
} Pmic_ilimDglCfgReg_t;

/**
 * @brief Structure to hold ILIM status register values.
 *
 * @param bbavgILIMErr  Bit field for BB ILIM ERR configuration.
 * @param pldo2ILIMErr  Bit field for PLDO2 ILIM ERR configuration.
 * @param pldo1ILIMErr  Bit field for PLDO1 ILIM ERR configuration.
 * @param ldo4ILIMErr   Bit field for LDO4 ILIM ERR configuration.
 * @param ldo3ILIMErr   Bit field for LDO3 ILIM ERR configuration.
 * @param ldo2ILIMErr   Bit field for LDO2 ILIM ERR configuration.
 * @param ldo1ILIMErr   Bit field for LDO1 ILIM ERR configuration.
 *
 * @ingroup Pmic_ILIMStructures
 */
typedef struct Pmic_ilimStatReg_s {
    uint8_t bbavgILIMErr;
    uint8_t pldo2ILIMErr;
    uint8_t pldo1ILIMErr;
    uint8_t ldo4ILIMErr;
    uint8_t ldo3ILIMErr;
    uint8_t ldo2ILIMErr;
    uint8_t ldo1ILIMErr;
} Pmic_ilimStatReg_t;


/**
 * @}
 */
/* End of Pmic_ILIMStructures */

/* ========================================================================== */
/*                          Function Prototypes                               */
/* ========================================================================== */

/**
 * @defgroup Pmic_ILIMFunctions PMIC ILIM Functions
 * @{
 * @ingroup Pmic_ILIM
 * @brief Contains functions used in the ILIM module of PMIC driver.
 */

/**
 * @brief Initializes the ILIM error register configuration structure with default values.
 * The provided Pmic_ilimStatReg_t structure pointed to by 'config' is populated with default values.
 * This function is typically used during the initialization of ILIM error register settings.
 *
 * @param config Pointer to the ILIM error register configuration structure.
 *
 * @ingroup Pmic_ILIMFunctions
 */
static void initializeILIMeRRReg(Pmic_ilimStatReg_t * config);

/**
 * @brief Initializes the ILIM configuration register structure with default values.
 * This function populates the provided Pmic_ilimCfgReg_t structure pointed to by 'config'
 * with default values for PLDO2, PLDO1, LDO4, LDO3, LDO2, and LDO1 ILIM configurations.
 * It is typically used during the initialization of ILIM configuration settings.
 *
 * @param config Pointer to the ILIM configuration register structure.
 *
 * @ingroup Pmic_ILIMFunctions
 */
static void initializeILIMCfgReg(Pmic_ilimCfgReg_t * config);

/**
 * @brief Initializes the ILIM deglitch configuration register structure with default values.
 * This function populates the provided Pmic_ilimDglCfgReg_t structure pointed to by 'config'
 * with default values for PLDO2, PLDO1, LDO4, LDO3, LDO2, and LDO1 ILIM deglitch configurations.
 * It is typically used during the initialization of ILIM deglitch configuration settings.
 *
 * @param config Pointer to the ILIM deglitch configuration register structure.
 *
 * @ingroup Pmic_ILIMFunctions
 */
static void initializeILIMDglCfgReg(Pmic_ilimDglCfgReg_t * config);

/**
 * @brief Sets the ILIM configuration register with the provided ILIM configuration settings.
 * This function configures the ILIM configuration register with the settings specified
 * in the provided Pmic_ilimCfgReg_t structure pointed to by 'pPmicILIMConfig'.
 * It reads the current register value, modifies the necessary fields according to the
 * provided configuration settings, and writes back the updated value to the register.
 * The function also ensures that the critical section is entered before accessing
 * the PMIC to prevent potential race conditions.
 *
 * @param pPmicCoreHandle Pointer to the PMIC core handle.
 * @param pPmicILIMConfig Pointer to the ILIM configuration structure.
 *
 * @return Returns PMIC_ST_SUCCESS if the operation is successful; otherwise, an error code.
 *
 * @ingroup Pmic_ILIMFunctions
 */
int32_t Pmic_SetILIMConfig(Pmic_CoreHandle_t * pPmicCoreHandle,
                           Pmic_ilimCfgReg_t * pPmicILIMConfig);


/**
 * @brief Sets the ILIM DGL (Dynamic Gain Limiting) configuration register with the provided settings.
 * This function configures the ILIM DGL configuration register with the settings specified
 * in the provided Pmic_ilimDglCfgReg_t structure pointed to by 'pPmicILIMdglConfig'.
 * It reads the current register value, modifies the necessary fields according to the
 * provided configuration settings, and writes back the updated value to the register.
 * The function also ensures that the critical section is entered before accessing
 * the PMIC to prevent potential race conditions.
 *
 * @param pPmicCoreHandle Pointer to the PMIC core handle.
 * @param pPmicILIMdglConfig Pointer to the ILIM DGL configuration structure.
 *
 * @return Returns PMIC_ST_SUCCESS if the operation is successful; otherwise, an error code.
 *
 * @ingroup Pmic_ILIMFunctions
 */
int32_t Pmic_SetILIMDglConfig(Pmic_CoreHandle_t *pPmicCoreHandle,
                               Pmic_ilimDglCfgReg_t *pPmicILIMdglConfig);

/**
 * @brief Clears the ILIM error status bits in the ILIM status register.
 * This function clears the ILIM error status bits in the ILIM status register based on
 * the provided configuration in the Pmic_ilimStatReg_t structure pointed to by 'pPmicILIMStat'.
 * It reads the current register value, modifies the necessary fields according to the
 * provided configuration settings, and writes back the updated value to the register.
 * The function also ensures that the critical section is entered before accessing
 * the PMIC to prevent potential race conditions.
 *
 * @param pPmicCoreHandle Pointer to the PMIC core handle.
 * @param pPmicILIMStat Pointer to the ILIM status register configuration structure.
 *
 * @return Returns PMIC_ST_SUCCESS if the operation is successful; otherwise, an error code.
 *
 * @ingroup Pmic_ILIMFunctions
 */
int32_t Pmic_ClearILIMErrStat(Pmic_CoreHandle_t *pPmicCoreHandle,
                              Pmic_ilimStatReg_t *pPmicILIMStat);

/**
 * @brief Retrieves the ILIM configuration from the ILIM configuration register.
 * This function reads the ILIM configuration register from the PMIC via the communication
 * interface specified in the PMIC core handle 'pPmicCoreHandle'. It then extracts the
 * individual ILIM configuration settings for PLDO2, PLDO1, LDO4, LDO3, LDO2, and LDO1
 * from the register data and stores them in the provided Pmic_ilimCfgReg_t structure
 * pointed to by 'pPmicILIMConfig'. The function ensures the critical section is entered
 * before accessing the PMIC to prevent potential race conditions.
 *
 * @param pPmicCoreHandle Pointer to the PMIC core handle.
 * @param pPmicILIMConfig Pointer to the ILIM configuration structure to store the retrieved values.
 *
 * @return Returns PMIC_ST_SUCCESS if the operation is successful; otherwise, an error code.
 *
 * @ingroup Pmic_ILIMFunctions
 */
int32_t Pmic_GetILIMConfig(Pmic_CoreHandle_t *pPmicCoreHandle,
                            Pmic_ilimCfgReg_t *pPmicILIMConfig);

/**
 * @brief Retrieves the ILIM DGL (Dynamic Gain Limit) configuration from the ILIM DGL configuration register.
 * This function reads the ILIM DGL configuration register from the PMIC via the communication
 * interface specified in the PMIC core handle 'pPmicCoreHandle'. It then extracts the
 * individual ILIM DGL configuration settings for PLDO2, PLDO1, LDO4, LDO3, LDO2, and LDO1
 * from the register data and stores them in the provided Pmic_ilimDglCfgReg_t structure
 * pointed to by 'pPmicILIMdglConfig'. The function ensures the critical section is entered
 * before accessing the PMIC to prevent potential race conditions.
 *
 * @param pPmicCoreHandle Pointer to the PMIC core handle.
 * @param pPmicILIMdglConfig Pointer to the ILIM DGL configuration structure to store the retrieved values.
 *
 * @return Returns PMIC_ST_SUCCESS if the operation is successful; otherwise, an error code.
 *
 * @ingroup Pmic_ILIMFunctions
 */
int32_t Pmic_GetILIMDglConfig(Pmic_CoreHandle_t *pPmicCoreHandle,
                              Pmic_ilimDglCfgReg_t *pPmicILIMdglConfig);

/**
 * @brief Retrieves the ILIM (Current Limit) error status from the ILIM status register.
 * This function reads the ILIM status register from the PMIC via the communication
 * interface specified in the PMIC core handle 'pPmicCoreHandle'. It then extracts the
 * individual ILIM error status for BB Average, PLDO2, PLDO1, LDO4, LDO3, LDO2, and LDO1
 * from the register data and stores them in the provided Pmic_ilimStatReg_t structure
 * pointed to by 'pPmicILIMStat'. The function ensures the critical section is entered
 * before accessing the PMIC to prevent potential race conditions.
 *
 * @param pPmicCoreHandle Pointer to the PMIC core handle.
 * @param pPmicILIMStat Pointer to the ILIM error status structure to store the retrieved values.
 *
 * @return Returns PMIC_ST_SUCCESS if the operation is successful; otherwise, an error code.
 *
 * @ingroup Pmic_ILIMFunctions
 */
int32_t Pmic_GetILIMErrStat(Pmic_CoreHandle_t *pPmicCoreHandle,
                             Pmic_ilimStatReg_t *pPmicILIMStat);


/**
 * @}
 */
/* End of Pmic_ILIMFunctions */

/**
 * @}
 */
/* End of Pmic_ILIM */

#endif /* PMIC_INC_ILIM_H_ */
