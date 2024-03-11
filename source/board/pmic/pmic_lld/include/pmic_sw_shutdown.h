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
 *   @file    pmic_sw_shutdown.h
 *
 *   @brief   This file contains the default MACRO's and function definitions
 * for PMIC SW_SHUTDOWN
 *
 */

#ifndef PMIC_PMIC_SW_SHUTDOWN_H_
#define PMIC_PMIC_SW_SHUTDOWN_H_

#include "pmic_core.h"
#include "pmic_core_priv.h"
#include "pmic_io_priv.h"
#include "pmic_types.h"

/* ============================================================================================
 */
/*                                  Structures and Enums */
/* ============================================================================================
 */
typedef struct Pmic_offstatestat1Reg_s {
    uint8_t initTMO;
    /**< Bit field for INIT_AN_TMO OFF_STATE_STAT1 configuration.
     */
    uint8_t internalOV;
    /**< Bit field for INTERNAL_OV OFF_STATE_STAT1
                            configuration.       */
    uint8_t ClkErr; /**< Bit field for CLK_ERR OFF_STATE_STAT1 configuration. */
    uint8_t firstPWROn;
    /**< Bit field for FIRST_PWR_ON OFF_STATE_STAT1
                            configuration.      */
    uint8_t OffprotEvt;
    /**< Bit field for OFF_PROT_EVT OFF_STATE_STAT1
                            configuration.      */
    uint8_t OffinitEvtErr;
    /**< Bit field for OFF_INT_EVT_ERR OFF_STATE_STAT1
                               configuration.   */
    uint8_t normalOff;
    /**< Bit field for NORMAL_OFF OFF_STATE_STAT1
                           configuration.        */
}
Pmic_offstatestat1Reg_t;

typedef struct Pmic_offstatestat2Reg_s {
    uint8_t BBIlimErr;
    /**< Bit field for BB_PK_ILIM_ERR OFF_STATE_STAT2
                           configuration.    */
    uint8_t
    BBTMO; /**< Bit field for BB_BST_TMO OFF_STATE_STAT2 configuration. */
    uint8_t BBOVPErr;
    /**< Bit field for BB_OVP_ERR OFF_STATE_STAT2 configuration.
     */
    uint8_t VbatOVPErr;
    /**< Bit field for VBAT_OVP_ERR OFF_STATE_STAT2
                            configuration.      */
    uint8_t
    BGXMErr; /**< Bit field for BGXM_ERR OFF_STATE_STAT2 configuration. */
    uint8_t RstMcuTMO;
    /**< Bit field for RST_MCU_TMO OFF_STATE_STAT2
                           configuration.       */
    uint8_t SysClkErr;
    /**< Bit field for SYS_CLK_ERR_PROT OFF_STATE_STAT2
                           configuration.  */
    uint8_t CRCErr; /**< Bit field for CRC_ERR OFF_STATE_STAT2 configuration. */
}
Pmic_offstatestat2Reg_t;

typedef struct Pmic_thermalstat1Reg_s {
    uint8_t ldo4TSDErr;
    /**< Bit field for LDO4_TSD_ERR THERMAL_STAT1
                            configuration.        */
    uint8_t ldo4TpreErr;
    /**< Bit field for LDO4_T_PRE_ERR THERMAL_STAT1
                             configuration.      */
    uint8_t ldo3TSDErr;
    /**< Bit field for LDO3_TSD_ERR THERMAL_STAT1
                            configuration.        */
    uint8_t ldo3TpreErr;
    /**< Bit field for LDO3_T_PRE_ERR THERMAL_STAT1
                             configuration.      */
    uint8_t ldo2TSDErr;
    /**< Bit field for LDO2_TSD_ERR THERMAL_STAT1
                            configuration.        */
    uint8_t ldo2TpreErr;
    /**< Bit field for LDO2_T_PRE_ERR THERMAL_STAT1
                             configuration.      */
    uint8_t ldo1TSDErr;
    /**< Bit field for LDO1_TSD_ERR THERMAL_STAT1
                            configuration.        */
    uint8_t ldo1TpreErr;
    /**< Bit field for LDO1_T_PRE_ERR THERMAL_STAT1
                             configuration.      */
}
Pmic_thermalstat1Reg_t;

typedef struct Pmic_thermalstat2Reg_s {
    uint8_t
    BBTSDErr; /**< Bit field for BB_TSD_ERR THERMAL_STAT2 configuration. */
    uint8_t BBTpreErr;
    /**< Bit field for BB_T_PRE_ERR THERMAL_STAT2
                           configuration.        */
    uint8_t pldo2TSDErr;
    /**< Bit field for PLDO2_TSD_ERR THERMAL_STAT2
                             configuration.       */
    uint8_t pldo2TpreErr;
    /**< Bit field for PLDO2_T_PRE_ERR THERMAL_STAT2
                              configuration.     */
    uint8_t pldo1TSDErr;
    /**< Bit field for PLDO1_TSD_ERR THERMAL_STAT2
                             configuration.       */
    uint8_t pldo1TpreErr;
    /**< Bit field for PLDO1_T_PRE_ERR THERMAL_STAT2
                              configuration.     */
}
Pmic_thermalstat2Reg_t;

/* =========================================================================================================================
 */
/*                                              Function Prototypes */
/* =========================================================================================================================
 */

int32_t
Pmic_GetOffStateStat1Config(Pmic_CoreHandle_t * pPmicCoreHandle,
    Pmic_offstatestat1Reg_t * pPmicOffStateStat1Config);
int32_t
Pmic_GetOffStateStat2Config(Pmic_CoreHandle_t * pPmicCoreHandle,
    Pmic_offstatestat2Reg_t * pPmicOffStateStat2Config);
int32_t Pmic_ClearOffStateStatRegs(Pmic_CoreHandle_t * pPmicCoreHandle);
int32_t
Pmic_GetThermalStat1Config(Pmic_CoreHandle_t * pPmicCoreHandle,
    Pmic_thermalstat1Reg_t * pPmicThermalStat1Config);
int32_t
Pmic_GetThermalStat2Config(Pmic_CoreHandle_t * pPmicCoreHandle,
    Pmic_thermalstat2Reg_t * pPmicThermalStat2Config);
int32_t Pmic_ClearThermalStat1flags(Pmic_CoreHandle_t * pPmicCoreHandle);
int32_t Pmic_ClearThermalStat2flags(Pmic_CoreHandle_t * pPmicCoreHandle);

#endif /* PMIC_PMIC_SW_SHUTDOWN_H_ */
