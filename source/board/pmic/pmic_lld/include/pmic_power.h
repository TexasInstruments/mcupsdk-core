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
 * @file   pmic_power.h
 *
 * @brief  PMIC PMIC Power Resources Driver API/interface file.
 *
 */

#ifndef PMIC_POWER_H_
#define PMIC_POWER_H_

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include "pmic_core_priv.h"
#include "pmic_io_priv.h"
#include "pmic_power_priv.h"
#include "pmic_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================== */
/*                             Macros & Typedefs                              */
/* ========================================================================== */

#define PMIC_BB_PWR_RESOURCE_TYPE_VCCA      (0U)
#define PMIC_BB_PWR_RESOURCE_TYPE_BUCK      (1U)
#define PMIC_BB_PWR_RESOURCE_TYPE_LDO       (2U)
#define PMIC_BB_PWR_RESOURCE_TYPE_PLDO      (3U)

#define PMIC_BB_POWER_SOURCE_VCCA                                              \
  ((((uint16_t)PMIC_BB_PWR_RESOURCE_TYPE_VCCA << 8U) | 0x0U))

#define PMIC_BB_REGULATOR_BUCK                                                 \
  ((((uint16_t)PMIC_BB_PWR_RESOURCE_TYPE_BUCK << 8U) | 0x1U))

#define PMIC_TPS65386X_REGULATOR_LDO1                                          \
  ((((uint16_t)PMIC_BB_PWR_RESOURCE_TYPE_LDO << 8U) | 0x2U))
#define PMIC_TPS65386X_REGULATOR_LDO2                                          \
  ((((uint16_t)PMIC_BB_PWR_RESOURCE_TYPE_LDO << 8U) | 0x3U))
#define PMIC_TPS65386X_REGULATOR_LDO3                                          \
  ((((uint16_t)PMIC_BB_PWR_RESOURCE_TYPE_LDO << 8U) | 0x4U))
#define PMIC_TPS65386X_REGULATOR_LDO4                                          \
  ((((uint16_t)PMIC_BB_PWR_RESOURCE_TYPE_LDO << 8U) | 0x5U))

#define PMIC_TPS65386X_REGULATOR_PLDO1                                         \
  ((((uint16_t)PMIC_BB_PWR_RESOURCE_TYPE_PLDO << 8U) | 0x6U))
#define PMIC_TPS65386X_REGULATOR_PLDO2                                         \
  ((((uint16_t)PMIC_BB_PWR_RESOURCE_TYPE_PLDO << 8U) | 0x7U))

/* PMIC LDO Ramp Time for Soft-Start Configuration Bit */
#define PMIC_LDO_PLDO_SHORT_RAMP_TIME       (0U)
#define PMIC_LDO_PLDO_LONG_RAMP_TIME        (1U)

/* PMIC PLDO Mode Selection Configuration Bit */
#define PMIC_PLDO_NON_TRACKING_MODE         (0U)
#define PMIC_PLDO_TRACKING_MODE             (1U)

/* PMIC LDO Current Limit Level configuration Bit */
#define PMIC_LDO_PLDO_ILIM_LVL_CFG_OPT0     (0U)
#define PMIC_LDO_PLDO_ILIM_LVL_CFG_OPT1     (1U)
#define PMIC_LDO_PLDO_ILIM_LVL_CFG_OPT2     (2U)
#define PMIC_LDO_PLDO_ILIM_LVL_CFG_OPT3     (3U)

/* LDO and PLDO voltage level configuration macros */
#define PMIC_LDO_PLDO_LVL_CFG_VOLT_1_0V     (0U)
#define PMIC_LDO_PLDO_LVL_CFG_VOLT1_05V     (1U)
#define PMIC_LDO_PLDO_LVL_CFG_VOLT_1_1V     (2U)
#define PMIC_LDO_PLDO_LVL_CFG_VOLT1_15V     (3U)
#define PMIC_LDO_PLDO_LVL_CFG_VOLT_1_2V     (4U)
#define PMIC_LDO_PLDO_LVL_CFG_VOLT1_25V     (5U)
#define PMIC_LDO_PLDO_LVL_CFG_VOLT_1_3V     (6U)
#define PMIC_LDO_PLDO_LVL_CFG_VOLT1_35V     (7U)
#define PMIC_LDO_PLDO_LVL_CFG_VOLT_1_4V     (8U)
#define PMIC_LDO_PLDO_LVL_CFG_VOLT1_45V     (9U)
#define PMIC_LDO_PLDO_LVL_CFG_VOLT_1_5V     (10U)
#define PMIC_LDO_PLDO_LVL_CFG_VOLT1_55V     (11U)
#define PMIC_LDO_PLDO_LVL_CFG_VOLT_1_6V     (12U)
#define PMIC_LDO_PLDO_LVL_CFG_VOLT1_65V     (13U)
#define PMIC_LDO_PLDO_LVL_CFG_VOLT_1_7V     (14U)
#define PMIC_LDO_PLDO_LVL_CFG_VOLT1_75V     (15U)
#define PMIC_LDO_PLDO_LVL_CFG_VOLT_1_8V     (16U)
#define PMIC_LDO_PLDO_LVL_CFG_VOLT_2_5V     (17U)
#define PMIC_LDO_PLDO_LVL_CFG_VOLT_3V       (18U)
#define PMIC_LDO_PLDO_LVL_CFG_VOLT_3_3V     (19U)
#define PMIC_LDO_PLDO_LVL_CFG_VOLT_5V       (20U)
#define PMIC_LDO_LVL_CFG_VOLT_BYPASS        (21U)

/* PLDO Voltage monitoring range macros */
#define PMIC_PLDO_VTRACK_RNG_LT_2V          (0U)
#define PMIC_PLDO_VTRACK_RNG_GT_2V          (1U)

/* LDO VMON Threshold ranges */
#define PMIC_LDO_VMON_TH_3_5                (0U)
#define PMIC_LDO_VMON_TH_4                  (1U)
#define PMIC_LDO_VMON_TH_5                  (2U)
#define PMIC_LDO_VMON_TH_6                  (3U)

/* EXT VMON Threshold ranges */
#define PMIC_EXT_VMON_TH_3_5                (0U)
#define PMIC_EXT_VMON_TH_4_5                (1U)
#define PMIC_EXT_VMON_TH_6                  (2U)
#define PMIC_EXT_VMON_TH_DIGITAL            (3U)

/* PLDO VMON Threshold ranges */
#define PMIC_PLDO_VMON_TH_4                 (0U)
#define PMIC_PLDO_VMON_TH_6                 (1U)
#define PMIC_PLDO_VMON_TH_8                 (2U)
#define PMIC_PLDO_VMON_TH_10                (3U)

/* LDO Control ranges */
#define PMIC_LDO_NOT_ENABLED                (0x00)
#define PMIC_LDO_ENABLED_LDO_MODE           (0x01)
#define PMIC_LDO_ENABLED_STBY_MODE          (0x02)
#define PMIC_LDO_ENABLED_VMON_MODE          (0x03)

#define PMIC_ALL_LDO_NOT_ENABLED            (0x00)
#define PMIC_ALL_LDO_ENABLED_LDO_MODE       (0x55)
#define PMIC_ALL_LDO_ENABLED_STBY_MODE      (0xAA)
#define PMIC_ALL_LDO_ENABLED_VMON_MODE      (0xFF)

/* Buck-Boost Timeout configuration ranges */
#define PMIC_BB_TMO_CFG_NO_TIMEOUT          (0U)
#define PMIC_BB_TMO_CFG_8S                  (1U)
#define PMIC_BB_TMO_CFG_12S                 (2U)
#define PMIC_BB_TMO_CFG_16S                 (3U)

/* Buck-Boost VMON Threshold ranges */
#define PMIC_BB_VMON_TH_5                   (0U)
#define PMIC_BB_VMON_TH_6                   (1U)
#define PMIC_BB_VMON_TH_7                   (2U)
#define PMIC_BB_VMON_TH_8                   (3U)

/* Buck-Boost De-Glitch configuration bits */
#define PMIC_VBAT_VMON_DGL_32US             (0U)
#define PMIC_VBAT_VMON_DGL_64US             (1U)

#define PMIC_BB_VMON_DGL_8US                (0U)
#define PMIC_BB_VMON_DGL_16US               (1U)
#define PMIC_BB_VMON_DGL_24US               (2U)
#define PMIC_BB_VMON_DGL_32US               (3U)

/* LDO De-Glitch configuration bits */
#define PMIC_LDO_VMON_DGL_8US               (0U)
#define PMIC_LDO_VMON_DGL_16US              (1U)
#define PMIC_LDO_VMON_DGL_24US              (2U)
#define PMIC_LDO_VMON_DGL_32US              (3U)

/* PLDO De-Glitch configuration bits */
#define PMIC_PLDO_VMON_DGL_8US              (0U)
#define PMIC_PLDO_VMON_DGL_16US             (1U)
#define PMIC_PLDO_VMON_DGL_24US             (2U)
#define PMIC_PLDO_VMON_DGL_32US             (3U)

/* EXT_VMON De-Glitch configuration bits */
#define PMIC_EXT_VMON_DGL_8US               (0U)
#define PMIC_EXT_VMON_DGL_16US              (1U)
#define PMIC_EXT_VMON_DGL_24US              (2U)
#define PMIC_EXT_VMON_DGL_32US              (3U)

/* voltage monitored 192 us (typical) per LP_VMON_PER_CFG period in STANDBY
 * state */
#define LP_VMON_CTRL_DATA1                  (0x00U)
/* continuous voltage monitoring in STANDBY state */
#define LP_VMON_CTRL_DATA2                  (0x01U)

/* 131 ms (typ) */
#define LP_TSD_PER_CFG_DATA1                (0X00U)
/* 524 ms (typ) */
#define LP_TSD_PER_CFG_DATA2                (0X01U)

/* 32 ms (typ) */
#define LP_VMON_PER_CFG_DATA1               (0X00U)
/* 64 ms (typ) */
#define LP_VMON_PER_CFG_DATA2               (0X01U)

/*==========================================================================*/
/*                         Structures and Enums                             */
/*==========================================================================*/

/**
 * @struct Pmic_powerBuckBoostCfgReg_s
 * @brief Structure representing the configuration register for Buck-Boost
 * operation.
 *
 * This structure provides a way to organize and access the individual bit
 * fields of the Buck-Boost Configuration Register.
 */
typedef struct Pmic_powerBuckBoostCfgReg_s {
    uint8_t bbPgoodCfg; /**< Bit field for Buck-Boost PGOOD configuration. */
    uint8_t bbSsEn;
    /**< Bit field for Buck-Boost Dual Random Spread Spectrum
                        (DRSS) modulation. */
    uint8_t bbStbyLvlCfg;
    /**< Bit field for Buck-Boost voltage selection during
                              low-power STANDBY state operation. */
    uint8_t bbLvlCfg;
    /**< Bit field for Buck-Boost voltage selection for
                          operating and sequencing states. */
    uint8_t bbCfgRegAddr; /**< Bit field for Buck-Boost Register Address. */
    uint8_t bbCfgRegShift; /**< Bit field for Buck-Boost Register Shift Value. */
    uint8_t bbCfgRegMask; /**< Bit field for Buck-Boost Register Mask Value. */
}
Pmic_powerBuckBoostCfgReg_t;

typedef struct Pmic_ldoCfgReg_s {
    uint8_t ldoRtCfg;
    uint8_t ldoIlimLvlCfg;
    uint8_t ldoLvlCfg;
}
Pmic_ldoCfgReg_t;

typedef struct Pmic_ldoCtrlReg_s {
    uint8_t ldo1Ctrl;
    uint8_t ldo2Ctrl;
    uint8_t ldo3Ctrl;
    uint8_t ldo4Ctrl;
}
Pmic_ldoCtrlReg_t;

typedef struct Pmic_pldoCfgReg_s {
    uint8_t pldoModeSel;
    uint8_t pldoIlimLvlCfg;
    uint8_t pldoLvlCfg;
    uint8_t pldoRegAddr;
    uint8_t pldoRegShift;
    uint8_t pldoRegMask;
}
Pmic_pldoCfgReg_t;

typedef struct Pmic_pldoVTrackRtReg_s {
    uint8_t pldoVTrackRng;
    uint8_t pldo1RTCfgVal;
    uint8_t pldo2RTCfgVal;
    uint8_t pldoVTrackRTRegAddr;
    uint8_t pldoVTrackRTRegShift;
    uint8_t pldoVTrackRTRegMask;
}
Pmic_pldoVTrackRtReg_t;

typedef struct Pmic_pgoodCfgReg_s {
    uint8_t pldo2PgoodCfg; /**< Bit field for PLDO2 PGOOD configuration. */
    uint8_t pldo1PgoodCfg; /**< Bit field for PLDO1 PGOOD configuration. */
    uint8_t ldo4PgoodCfg; /**< Bit field for LDO4 PGOOD configuration. */
    uint8_t ldo3PgoodCfg; /**< Bit field for LDO3 PGOOD configuration. */
    uint8_t ldo2PgoodCfg; /**< Bit field for LDO2 PGOOD configuration. */
    uint8_t ldo1PgoodCfg; /**< Bit field for LDO1 PGOOD configuration. */
    uint8_t pgoodRegAddr;
    uint8_t pgoodRegShift;
    uint8_t pgoodRegMask;
}
Pmic_pgoodCfgReg_t;

typedef struct Pmic_pldoEnOutCtrlReg_s {
    uint8_t enOut2Enable;
    uint8_t enOut1Enable;
    uint8_t pldo2Ctrl;
    uint8_t pldo1Ctrl;
    uint8_t pldoEnOutRegAddr;
}
Pmic_pldoEnOutCtrlReg_t;

typedef struct Pmic_DscgDisCtrlReg_s {
    uint8_t pldo2DscgDis; /**< Bit 5 (PLDO2_DSCG_DIS). */
    uint8_t pldo1DscgDis; /**< Bit 4 (PLDO1_DSCG_DIS). */
    uint8_t ldo4DscgDis; /**< Bit 3 (LDO4_DSCG_DIS). */
    uint8_t ldo3DscgDis; /**< Bit 2 (LDO3_DSCG_DIS). */
    uint8_t ldo2DscgDis; /**< Bit 1 (LDO2_DSCG_DIS). */
    uint8_t ldo1DscgDis; /**< Bit 0 (LDO1_DSCG_DIS). */
    uint8_t dscgDisCtrlRegAddr;
}
Pmic_DscgDisCtrlReg_t;

typedef struct Pmic_VbatBBVMONDglReg_s {
    uint8_t vbatvmondgl;
    /**< Bit 7   (Configuration bits for VBAT VMON deglitch
                             time). */
    uint8_t bbvmondgl;
    /**< Bit 1-0 (Configuration bits for Buck-Boost VMON
                           deglitch time). */
}
Pmic_VbatBBVMONDglReg_t;

typedef struct Pmic_ldoVMONThresholdReg_s {
    uint8_t ldo4vmonthresh;
    /**< Bit 7-6 (Configuration bits for LDO4 VMON
                             deglitch time). */
    uint8_t ldo3vmonthresh;
    /**< Bit 5-4 (Configuration bits for LDO3 VMON
                             deglitch time). */
    uint8_t ldo2vmonthresh;
    /**< Bit 3-2 (Configuration bits for LDO2 VMON
                             deglitch time). */
    uint8_t ldo1vmonthresh;
    /**< Bit 1-0 (Configuration bits for LDO1 VMON
                             deglitch time). */
}
Pmic_ldoVMONThresholdReg_t;

typedef struct Pmic_ldoVMONDglReg_s {
    uint8_t ldo4vmondgl;
    /**< Bit 7-6 (Configuration bits for LDO4 VMON deglitch
                             time). */
    uint8_t ldo3vmondgl;
    /**< Bit 5-4 (Configuration bits for LDO3 VMON deglitch
                             time). */
    uint8_t ldo2vmondgl;
    /**< Bit 3-2 (Configuration bits for LDO2 VMON deglitch
                             time). */
    uint8_t ldo1vmondgl;
    /**< Bit 1-0 (Configuration bits for LDO1 VMON deglitch
                             time). */
}
Pmic_ldoVMONDglReg_t;

typedef struct Pmic_extpldoVMONDglReg_s {
    uint8_t extvmon1dgl;
    /**< Bit 7-6 (Configuration bits for EXT-VMON1 deglitch
                             time). */
    uint8_t extvmon2dgl;
    /**< Bit 5-4 (Configuration bits for EXT-VMON2 deglitch
                             time). */
    uint8_t pldo2vmondgl;
    /**< Bit 3-2 (Configuration bits for PLDO2 VMON
                              deglitch time). */
    uint8_t pldo1vmondgl;
    /**< Bit 1-0 (Configuration bits for PLDO1 VMON
                              deglitch time). */
}
Pmic_extpldoVMONDglReg_t;

typedef struct Pmic_extpldoVMONThreshReg_s {
    uint8_t extvmon1thresh;
    uint8_t extvmon2thresh;
    uint8_t pldo2vmonthresh;
    uint8_t pldo1vmonthresh;
}
Pmic_extpldoVMONThreshReg_t;

typedef struct Pmic_lpVMonCtrlReg_s {
    uint8_t lpExtVMon2Ctrl; /**< Bit 7 (LP_EXT_VMON2_CTRL)  */
    uint8_t lpExtVMon1Ctrl; /**< Bit 6 (LP_EXT_VMON1_CTRL)  */
    uint8_t lppldo2VMonCtrl; /**< Bit 5 (LP_PLDO2_VMON_CTRL) */
    uint8_t lppldo1VMonCtrl; /**< Bit 4 (LP_PLDO1_VMON_CTRL) */
    uint8_t lpldo4VMonCtrl; /**< Bit 3 (LP_LDO4_VMON_CTRL)  */
    uint8_t lpldo3VMonCtrl; /**< Bit 2 (LP_LDO3_VMON_CTRL)  */
    uint8_t lpldo2VMonCtrl; /**< Bit 1 (LP_LDO2_VMON_CTRL)  */
    uint8_t lpldo1VMonCtrl; /**< Bit 0 (LP_LDO1_VMON_CTRL)  */
    uint8_t lpVmonCtrlRegAddr;
}
Pmic_lpVMonCtrlReg_t;

typedef struct Pmic_lpConfigCtrlReg_s {
    uint8_t lpBBOVPCtrl; /**< Bit 7 (LP_BB_OVP_CTRL)     */
    uint8_t lpBBVMonCtrl; /**< Bit 6 (LP_BB_VMON_CTRL)    */
    uint8_t lpTSDperConfig; /**< Bit 1 (LP_TSD_PER_CFG)     */
    uint8_t lpVMonperConfig; /**< Bit 0 (LP_VMON_PER_CFG)    */
    uint8_t lpCfgRegAddr;
}
Pmic_lpConfigCtrlReg_t;

typedef struct Pmic_BuckBoostVMONConfigReg_s {
    uint8_t buckBoostTmoCfg;
    uint8_t buckBoostVmonTh;
}
Pmic_BuckBoostVMONConfigReg_t;

typedef struct Pmic_powerResourceConfig_s {
    uint8_t pmicConfigShiftVal;
    uint8_t pmicConfigMaskVal;
}
Pmic_powerRsrcCfg_t;

typedef struct Pmic_powerResourceRegCfg_s {
    uint8_t buckConfigRegAddr;
    uint8_t ldo1ConfigRegAddr;
    uint8_t ldo2ConfigRegAddr;
    uint8_t ldo3ConfigRegAddr;
    uint8_t ldo4ConfigRegAddr;
    uint8_t pldo1ConfigRegAddr;
    uint8_t pldo2ConfigRegAddr;
    uint8_t pldoConfigRegAddr;
    uint8_t dscgConfigRegAddr;
    uint8_t pgoodConfigRegAddr;
    uint8_t ldoCtrlRegAddr;
    uint8_t enoutCtrlRegAddr;
    uint8_t vmonTHCfg1RegAddr;
    uint8_t vmonTHCfg2RegAddr;
    uint8_t vmonTHCfg3RegAddr;
    uint8_t lpvmonCtrlRegAddr;
    uint8_t lpConfigRegAddr;
    uint8_t vmonDglCfg1RegAddr;
    uint8_t vmonDglCfg2RegAddr;
    uint8_t vmonDglCfg3RegAddr;
    uint8_t extvmonCfgCtrlRegAddr;
    uint8_t bitPosVal;
    uint8_t bitMaskVal;
}
Pmic_powerRsrcRegCfg_t;

/*==========================================================================*/
/*                         Function Declarations                            */
/*==========================================================================*/

void pmic_get_bb_pwrRsrceRegCfg(Pmic_powerRsrcRegCfg_t ** pPwrRsrcRegCfg);

int32_t
Pmic_powerSetBuckBstPgoodCfg(Pmic_CoreHandle_t * pPmicCoreHandle,
    const Pmic_powerBuckBoostCfgReg_t * buckBstCfg);

int32_t Pmic_powerGetBuckBstPgoodCfg(Pmic_CoreHandle_t * pPmicCoreHandle,
    Pmic_powerBuckBoostCfgReg_t * buckBstCfg);

int32_t Pmic_powerSetBuckBstSsEn(Pmic_CoreHandle_t * pPmicCoreHandle,
    const Pmic_powerBuckBoostCfgReg_t * buckBstCfg);

int32_t Pmic_powerGetBuckBstSsEn(Pmic_CoreHandle_t * pPmicCoreHandle,
    Pmic_powerBuckBoostCfgReg_t * buckBstCfg);

int32_t
Pmic_powerSetBuckBstStbyLvlCfg(Pmic_CoreHandle_t * pPmicCoreHandle,
    const Pmic_powerBuckBoostCfgReg_t * buckBstCfg);

int32_t Pmic_powerGetBuckBstStbyLvlCfg(Pmic_CoreHandle_t * pPmicCoreHandle,
    Pmic_powerBuckBoostCfgReg_t * buckBstCfg);

int32_t
Pmic_powerSetBuckBstLvlCfg(Pmic_CoreHandle_t * pPmicCoreHandle,
    const Pmic_powerBuckBoostCfgReg_t * buckBstCfg);

int32_t Pmic_powerGetBuckBstLvlCfg(Pmic_CoreHandle_t * pPmicCoreHandle,
    Pmic_powerBuckBoostCfgReg_t * buckBstCfg);

int32_t Pmic_powerSetBuckBstCfg(Pmic_CoreHandle_t * pPmicCoreHandle,
    Pmic_powerBuckBoostCfgReg_t * buckBstCfg,
    const Pmic_powerRsrcRegCfg_t * pwrRsrcRegCfg,
        const Pmic_powerRsrcCfg_t * pwrRsrcCfg);

int32_t Pmic_powerGetBuckBstCfg(Pmic_CoreHandle_t * pPmicCoreHandle,
    Pmic_powerBuckBoostCfgReg_t * buckBstCfg,
    const Pmic_powerRsrcRegCfg_t * pwrRsrcRegCfg,
        const Pmic_powerRsrcCfg_t * pwrRsrcCfg);

void Pmic_getLDOCfgFields(uint8_t ldoNumber, Pmic_ldoCfgReg_t * ldoCfg,
    const Pmic_powerRsrcCfg_t * pwrRsrcCfg,
        const Pmic_powerRsrcRegCfg_t * pwrRsrcRegCfg);

int32_t Pmic_powerSetLDORtCfg(Pmic_CoreHandle_t * pPmicCoreHandle,
    const Pmic_ldoCfgReg_t * ldoCfg);

int32_t Pmic_powerGetLDORtCfg(Pmic_CoreHandle_t * pPmicCoreHandle,
    Pmic_ldoCfgReg_t * ldoCfg);

int32_t Pmic_powerSetLDOIlimLvlCfg(Pmic_CoreHandle_t * pPmicCoreHandle,
    const Pmic_ldoCfgReg_t * ldoCfg);

int32_t Pmic_powerGetLDOIlimLvlCfg(Pmic_CoreHandle_t * pPmicCoreHandle,
    Pmic_ldoCfgReg_t * ldoCfg);

int32_t Pmic_powerSetLDOLvlCfg(Pmic_CoreHandle_t * pPmicCoreHandle,
    const Pmic_ldoCfgReg_t * ldoCfg);

int32_t Pmic_powerGetLDOLvlCfg(Pmic_CoreHandle_t * pPmicCoreHandle,
    Pmic_ldoCfgReg_t * ldoCfg);

int32_t Pmic_powerSetBuckBstCfg(Pmic_CoreHandle_t * pPmicCoreHandle,
    Pmic_powerBuckBoostCfgReg_t * buckBstCfg,
    const Pmic_powerRsrcRegCfg_t * pwrRsrcRegCfg,
        const Pmic_powerRsrcCfg_t * pwrRsrcCfg);

int32_t Pmic_powerGetBuckBstCfg(Pmic_CoreHandle_t * pPmicCoreHandle,
    Pmic_powerBuckBoostCfgReg_t * buckBstCfg,
    const Pmic_powerRsrcRegCfg_t * pwrRsrcRegCfg,
        const Pmic_powerRsrcCfg_t * pwrRsrcCfg);

int32_t
Pmic_powerSetLdoConfigRegister(Pmic_CoreHandle_t * pPmicCoreHandle,
                               uint8_t ldoNumber, Pmic_ldoCfgReg_t *ldoConfig);

int32_t
Pmic_powerGetLdoConfigRegister(Pmic_CoreHandle_t * pPmicCoreHandle,
                               uint8_t ldoNumber, Pmic_ldoCfgReg_t *ldoConfig);

void Pmic_getPLDOCfgFields(uint8_t pldoNumber, Pmic_pldoCfgReg_t * pldoCfg,
    const Pmic_powerRsrcCfg_t * pwrRsrcCfg,
        const Pmic_powerRsrcRegCfg_t * pwrRsrcRegCfg);

int32_t Pmic_powerSetPLDOModeSel(Pmic_CoreHandle_t * pPmicCoreHandle,
    const Pmic_pldoCfgReg_t * pldoCfg);

int32_t Pmic_powerGetPLDOModeSel(Pmic_CoreHandle_t * pPmicCoreHandle,
    Pmic_pldoCfgReg_t * pldoCfg);

int32_t Pmic_powerSetPLDOIlimLvlCfg(Pmic_CoreHandle_t * pPmicCoreHandle,
    const Pmic_pldoCfgReg_t * pldoCfg);

int32_t Pmic_powerGetPLDOIlimLvlCfg(Pmic_CoreHandle_t * pPmicCoreHandle,
    Pmic_pldoCfgReg_t * pldoCfg);

int32_t Pmic_powerSetPLDOLvlCfg(Pmic_CoreHandle_t * pPmicCoreHandle,
    const Pmic_pldoCfgReg_t * pldoCfg);

int32_t Pmic_powerGetPLDOLvlCfg(Pmic_CoreHandle_t * pPmicCoreHandle,
    Pmic_pldoCfgReg_t * pldoCfg);

int32_t Pmic_powerSetPLDOConfigRegister(
    Pmic_CoreHandle_t * pPmicCoreHandle, uint8_t pldoNumber,
    Pmic_pldoCfgReg_t * pldoConfig,
    const Pmic_powerRsrcCfg_t * pwrRsrcCfg,
        const Pmic_powerRsrcRegCfg_t * pwrRsrcRegCfg);

int32_t Pmic_powerGetPLDOConfigRegister(
    Pmic_CoreHandle_t * pPmicCoreHandle, uint8_t pldoNumber,
    Pmic_pldoCfgReg_t * pldoConfig,
    const Pmic_powerRsrcCfg_t * pwrRsrcCfg,
        const Pmic_powerRsrcRegCfg_t * pwrRsrcRegCfg);

int32_t Pmic_setPLDOVTrackRng(Pmic_CoreHandle_t * pPmicCoreHandle,
    const Pmic_pldoVTrackRtReg_t * pldoCfg);

int32_t Pmic_getPLDOVTrackRng(Pmic_CoreHandle_t * pPmicCoreHandle,
    Pmic_pldoVTrackRtReg_t * pldoCfg);

int32_t Pmic_setPLDO1RTCfg(Pmic_CoreHandle_t * pPmicCoreHandle,
    const Pmic_pldoVTrackRtReg_t * pldoCfg);

int32_t Pmic_getPLDO1RTCfg(Pmic_CoreHandle_t * pPmicCoreHandle,
    Pmic_pldoVTrackRtReg_t * pldoCfg);

int32_t Pmic_setPLDO2RTCfg(Pmic_CoreHandle_t * pPmicCoreHandle,
    const Pmic_pldoVTrackRtReg_t * pldoCfg);

int32_t Pmic_getPLDO2RTCfg(Pmic_CoreHandle_t * pPmicCoreHandle,
    Pmic_pldoVTrackRtReg_t * pldoCfg);

int32_t Pmic_SetPLDOVTrackRTRegCfg(Pmic_CoreHandle_t * pPmicCoreHandle,
    uint8_t pldoNumber,
    Pmic_pldoVTrackRtReg_t * pldoConfig,
    const Pmic_powerRsrcCfg_t * pwrRsrcCfg,
        const Pmic_powerRsrcRegCfg_t * pwrRsrcRegCfg);

int32_t Pmic_GetPLDOVTrackRTRegCfg(Pmic_CoreHandle_t * pPmicCoreHandle,
    uint8_t pldoNumber,
    Pmic_pldoVTrackRtReg_t * pldoConfig,
    const Pmic_powerRsrcCfg_t * pwrRsrcCfg,
        const Pmic_powerRsrcRegCfg_t * pwrRsrcRegCfg);

int32_t Pmic_setPldoPgoodCfg(Pmic_CoreHandle_t * pPmicCoreHandle,
    uint8_t pldoNumber,
    const Pmic_pgoodCfgReg_t * pldoPgoodCfg);

int32_t Pmic_getPldoPgoodCfg(Pmic_CoreHandle_t * pPmicCoreHandle,
    uint8_t pldoNumber,
    Pmic_pgoodCfgReg_t * pldoPgoodCfg);

int32_t Pmic_setLdoPgoodCfg(Pmic_CoreHandle_t * pPmicCoreHandle,
    uint8_t ldoNumber,
    const Pmic_pgoodCfgReg_t * ldoPgoodCfg);

int32_t Pmic_getLdoPgoodCfg(Pmic_CoreHandle_t * pPmicCoreHandle,
    uint8_t ldoNumber, Pmic_pgoodCfgReg_t * ldoPgoodCfg);

void Pmic_getLDOCtrlFields(uint8_t ldoNumber, uint8_t * pBitPos,
    uint8_t * pBitMask);

int32_t Pmic_setLdoCtrl(Pmic_CoreHandle_t * pPmicCoreHandle,
                        Pmic_ldoCtrlReg_t *ldoControl);

int32_t Pmic_getLdoCtrl(Pmic_CoreHandle_t * pPmicCoreHandle,
                        uint8_t ldoNumber,
                        Pmic_ldoCtrlReg_t *ldoControl);

void Pmic_getEnOutCtrlFields(uint8_t enableNumber, uint8_t * pBitPos,
    uint8_t * pBitMask);

void Pmic_getPLDOCtrlFields(uint8_t pldoNumber, uint8_t * pBitPos,
    uint8_t * pBitMask);

int32_t Pmic_setPowerEnOutCtrlReg(Pmic_CoreHandle_t * pPmicCoreHandle,
    const Pmic_pldoEnOutCtrlReg_t * pldoEnOutCfg);

int32_t Pmic_setPowerPLDOCtrlReg(Pmic_CoreHandle_t * pPmicCoreHandle,
    const Pmic_pldoEnOutCtrlReg_t * pldoEnOutCfg);

int32_t Pmic_setPowerPLDOEnOutControl(Pmic_CoreHandle_t * pPmicCoreHandle,
    Pmic_pldoEnOutCtrlReg_t * pldoEnOutCfg,
    const Pmic_powerRsrcRegCfg_t * pwrRsrcRegCfg);

int32_t Pmic_setPLDODscgDisCtrl(Pmic_CoreHandle_t * pPmicCoreHandle,
    const Pmic_powerRsrcCfg_t * dscgConfig,
        const Pmic_DscgDisCtrlReg_t * dscgDisCtrlCfg);

int32_t Pmic_setLDODscgDisCtrl(Pmic_CoreHandle_t * pPmicCoreHandle,
    const Pmic_powerRsrcCfg_t * dscgConfig,
        const Pmic_DscgDisCtrlReg_t * dscgDisCtrlCfg);

int32_t Pmic_setPowerDscgDisControl(Pmic_CoreHandle_t * pPmicCoreHandle,
    Pmic_DscgDisCtrlReg_t * dscgDisCtrlCfg,
    const Pmic_powerRsrcCfg_t * dscgConfig,
        const Pmic_powerRsrcRegCfg_t * pwrRsrcRegCfg);

int32_t
Pmic_setldoVmonThresholdConfig(Pmic_CoreHandle_t * pPmicCoreHandle,
    uint8_t ldoNumber,
    const Pmic_ldoVMONThresholdReg_t * ldomonThreshCfg,
        const Pmic_powerRsrcRegCfg_t * pwrRsrcRegCfg);
int32_t
Pmic_getldoVmonThresholdConfig(Pmic_CoreHandle_t * pPmicCoreHandle,
    uint8_t ldoNumber,
    Pmic_ldoVMONThresholdReg_t * ldomonThreshCfg,
    const Pmic_powerRsrcRegCfg_t * pwrRsrcRegCfg);

int32_t Pmic_setpldoVMONThreshConfig(Pmic_CoreHandle_t * pPmicCoreHandle,
    uint8_t pldoNumber,
    const Pmic_extpldoVMONThreshReg_t * pldovmonthCfg,
        const Pmic_powerRsrcRegCfg_t * pwrRsrcRegCfg);

int32_t Pmic_getpldoVMONThreshConfig(Pmic_CoreHandle_t * pPmicCoreHandle,
    uint8_t pldoNumber,
    Pmic_extpldoVMONThreshReg_t * pldovmonthCfg,
    const Pmic_powerRsrcRegCfg_t * pwrRsrcRegCfg);

int32_t Pmic_setextVMONThreshConfig(Pmic_CoreHandle_t * pPmicCoreHandle,
    uint8_t extVmonNumber,
    const Pmic_extpldoVMONThreshReg_t * extvmonthCfg,
        const Pmic_powerRsrcRegCfg_t * pwrRsrcRegCfg);

int32_t Pmic_getextVMONThreshConfig(Pmic_CoreHandle_t * pPmicCoreHandle,
    uint8_t extVmonNumber,
    Pmic_extpldoVMONThreshReg_t * extvmonthCfg,
    const Pmic_powerRsrcRegCfg_t * pwrRsrcRegCfg);

int32_t Pmic_setbbTimeoutConfig(Pmic_CoreHandle_t * pPmicCoreHandle,
    const Pmic_BuckBoostVMONConfigReg_t * bbtmoCfg,
        const Pmic_powerRsrcRegCfg_t * pwrRsrcRegCfg);

int32_t Pmic_getbbTimeoutConfig(Pmic_CoreHandle_t * pPmicCoreHandle,
    Pmic_BuckBoostVMONConfigReg_t * bbtmoCfg,
    const Pmic_powerRsrcRegCfg_t * pwrRsrcRegCfg);

int32_t Pmic_setbbVmonThConfig(Pmic_CoreHandle_t * pPmicCoreHandle,
    const Pmic_BuckBoostVMONConfigReg_t * bbtmoCfg,
        const Pmic_powerRsrcRegCfg_t * pwrRsrcRegCfg);

int32_t Pmic_getbbVmonThConfig(Pmic_CoreHandle_t * pPmicCoreHandle,
    Pmic_BuckBoostVMONConfigReg_t * bbtmoCfg,
    const Pmic_powerRsrcRegCfg_t * pwrRsrcRegCfg);

int32_t Pmic_setBBVMONDeGlitchConfig(Pmic_CoreHandle_t * pPmicCoreHandle,
    const Pmic_VbatBBVMONDglReg_t * bbvmondglCfg,
        const Pmic_powerRsrcRegCfg_t * pwrRsrcRegCfg);

int32_t Pmic_getBBVMONDeGlitchConfig(Pmic_CoreHandle_t * pPmicCoreHandle,
    const Pmic_powerRsrcRegCfg_t * pwrRsrcRegCfg,
        uint8_t * bbVmonDglStatus);

int32_t Pmic_setVbatVMONDeGlitchConfig(Pmic_CoreHandle_t * pPmicCoreHandle,
    const Pmic_VbatBBVMONDglReg_t * vbatvmondglCfg,
        const Pmic_powerRsrcRegCfg_t * pwrRsrcRegCfg);

int32_t Pmic_getVbatVMONDeGlitchConfig(Pmic_CoreHandle_t * pPmicCoreHandle,
    const Pmic_powerRsrcRegCfg_t * pwrRsrcRegCfg,
        uint8_t * VbatVmonDglStatus);

int32_t Pmic_setldoVMONDeGlitchConfig(Pmic_CoreHandle_t * pPmicCoreHandle,
    uint8_t ldoNumber,
    const Pmic_ldoVMONDglReg_t * ldovmondglCfg,
        const Pmic_powerRsrcRegCfg_t * pwrRsrcRegCfg);

int32_t Pmic_getldoVMONDeGlitchConfig(Pmic_CoreHandle_t * pPmicCoreHandle,
    uint8_t ldoNumber,
    Pmic_ldoVMONDglReg_t * ldovmondglCfg,
    const Pmic_powerRsrcRegCfg_t * pwrRsrcRegCfg);

int32_t Pmic_setpldoVMONDeGlitchConfig(Pmic_CoreHandle_t * pPmicCoreHandle,
    uint8_t pldoNumber,
    const Pmic_extpldoVMONDglReg_t * pldovmondglCfg,
        const Pmic_powerRsrcRegCfg_t * pwrRsrcRegCfg);

int32_t Pmic_getpldoVMONDeGlitchConfig(Pmic_CoreHandle_t * pPmicCoreHandle,
    uint8_t pldoNumber,
    Pmic_extpldoVMONDglReg_t * pldovmondglCfg,
    const Pmic_powerRsrcRegCfg_t * pwrRsrcRegCfg);

int32_t Pmic_setextVMONDeGlitchConfig(Pmic_CoreHandle_t * pPmicCoreHandle,
    uint8_t extVmonNumber,
    const Pmic_extpldoVMONDglReg_t * extvmondglCfg,
        const Pmic_powerRsrcRegCfg_t * pwrRsrcRegCfg);

int32_t Pmic_getextVMONDeGlitchConfig(Pmic_CoreHandle_t * pPmicCoreHandle,
    uint8_t extVmonNumber,
    Pmic_extpldoVMONDglReg_t * extvmondglCfg,
    const Pmic_powerRsrcRegCfg_t * pwrRsrcRegCfg);

int32_t Pmic_SetLPExtVMonCtrl(Pmic_CoreHandle_t * pPmicCoreHandle,
    const Pmic_lpVMonCtrlReg_t * lpVMonCtrlCfg);

int32_t Pmic_SetLPPLDOVMonCtrl(Pmic_CoreHandle_t * pPmicCoreHandle,
    const Pmic_lpVMonCtrlReg_t * lpVMonCtrlCfg);

int32_t Pmic_SetLPLDOVMonCtrl(Pmic_CoreHandle_t * pPmicCoreHandle,
    const Pmic_lpVMonCtrlReg_t * lpVMonCtrlCfg);

int32_t Pmic_GetLPPLDOVMonCtrl(Pmic_CoreHandle_t * pPmicCoreHandle,
    Pmic_lpVMonCtrlReg_t * lpVMonCtrlCfg);

int32_t Pmic_GetLPLDOVMonCtrl(Pmic_CoreHandle_t * pPmicCoreHandle,
    Pmic_lpVMonCtrlReg_t * lpVMonCtrlCfg);

int32_t Pmic_SetLowPowerVmonCtrl(Pmic_CoreHandle_t * pPmicCoreHandle,
    Pmic_lpVMonCtrlReg_t * lpVMonCtrlCfg);

int32_t Pmic_GetLowPowerVmonCtrl(Pmic_CoreHandle_t * pPmicCoreHandle,
    Pmic_lpVMonCtrlReg_t * lpVMonCtrlCfg);

int32_t Pmic_GetLPExtVMonCtrl(Pmic_CoreHandle_t * pPmicCoreHandle,
    Pmic_lpVMonCtrlReg_t * lpVMonCtrlCfg);

int32_t Pmic_SetlpBBOVPCtrl(Pmic_CoreHandle_t * pPmicCoreHandle,
    const Pmic_lpConfigCtrlReg_t * lpConfig);

int32_t Pmic_GetlpBBOVPCtrl(Pmic_CoreHandle_t * pPmicCoreHandle,
    Pmic_lpConfigCtrlReg_t * lpConfig);

int32_t Pmic_SetlpBBVmonCtrl(Pmic_CoreHandle_t * pPmicCoreHandle,
    const Pmic_lpConfigCtrlReg_t * lpConfig);

int32_t Pmic_GetlpBBVmonCtrl(Pmic_CoreHandle_t * pPmicCoreHandle,
    Pmic_lpConfigCtrlReg_t * lpConfig);

int32_t Pmic_SetlpTSDperCfg(Pmic_CoreHandle_t * pPmicCoreHandle,
    const Pmic_lpConfigCtrlReg_t * lpConfig);

int32_t Pmic_GetlpTSDperCfg(Pmic_CoreHandle_t * pPmicCoreHandle,
    Pmic_lpConfigCtrlReg_t * lpConfig);

int32_t Pmic_SetlpVmonperCfg(Pmic_CoreHandle_t * pPmicCoreHandle,
    const Pmic_lpConfigCtrlReg_t * lpConfig);

int32_t Pmic_GetlpVmonperCfg(Pmic_CoreHandle_t * pPmicCoreHandle,
    Pmic_lpConfigCtrlReg_t * lpConfig);

int32_t Pmic_SetLowPowerConfig(Pmic_CoreHandle_t * pPmicCoreHandle,
    Pmic_lpConfigCtrlReg_t * lpConfig);

int32_t Pmic_GetLowPowerConfig(Pmic_CoreHandle_t * pPmicCoreHandle,
    Pmic_lpConfigCtrlReg_t * lpConfig);
#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* PMIC_POWER_H_ */
