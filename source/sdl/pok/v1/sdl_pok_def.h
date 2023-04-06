/*
 *  Copyright (C) 2023 Texas Instruments Incorporated.
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
 */
/**
 *  \ingroup SDL_IP_MODULE
 *  \defgroup SDL_IP_POK_API POK Low-Level API
 *
 *  @{
 */
/**
 *  \file  sdl_pok_def.h
 *
 *  \brief
 *     Header file containing various enumerations, structure definitions and function
 *  declarations for the Power System modules such as POK/POR IP
 **/

#ifndef SDL_POK_DEF_H
#define SDL_POK_DEF_H

#ifdef __cplusplus
extern "C" {
#endif

#include <sdl/include/sdl_types.h>


#include <sdl/pok/v1/soc/sdl_soc_pok.h>
#include <sdl/esm/v0/v0_0/sdl_ip_esm.h>



/**
*\defgroup SDL_IP_POK_DATASTRUCT  POK Data Structures
*\ingroup SDL_IP_POK_API
*/
/**
*\defgroup SDL_IP_POK_FUNCTION  POK Functions
*\ingroup SDL_IP_POK_API
*/
/**
*\defgroup SDL_IP_POK_ENUMS POK Enum Data defines
*\ingroup SDL_IP_POK_API
*/

typedef struct SDL_pokShiftsAndMasks
{
    volatile    uint32_t    *pokAddr;
    volatile    uint32_t    *pokOVAddr;
    volatile    uint32_t    *pokDetAddr;
    volatile    uint32_t    *pokEnSelAddr;
    volatile    uint32_t    *pokEnPPAddr;
    volatile    uint32_t    *deglitchSelAddr;
    uint32_t    pokEnPPMask;
    uint32_t    pokEnPPShift;
    uint32_t    hystMask;
    uint32_t    hystShift;
    uint32_t    hystOVMask;
    uint32_t    hystOVShift;
    uint32_t    vdDetMask;
    uint32_t    vdDetShift;
    uint32_t    vdDetOVMask;
    uint32_t    vdDetOVShift;
    uint32_t    detEnMask;
    uint32_t    detEnShift;
    uint32_t    ovSelMask;
    uint32_t    ovSelShift;
    uint32_t    pokEnSelMask;
    uint32_t    pokEnSelShift;
    uint32_t    trimMask;
    uint32_t    trimShift;
    uint32_t    trimOVMask;
    uint32_t    trimOVShift;
    uint32_t    deglitchSelMask;
    uint32_t    deglitchSelShift;
} SDL_pokShiftsAndMasks_t;

typedef struct SDL_pokPRGInfo
{
    volatile uint32_t *addr;
    uint32_t pokEnPPMask;
    uint32_t pokEnPPShift;
} SDL_pokPRGInfo_t;

/** @} */
/**
 *  \addtogroup SDL_IP_POK_ENUMS
 *  @{
 */

/**
 * @brief This enumerator defines the possible POK module types
 *
 *  \anchor SDL_pok_types
 *  \name POK types
 *
 *  @{
 *
 */
typedef int8_t  SDL_PRG_Inst;
typedef int8_t  SDL_POK_Inst ;
typedef  uint8_t                           SDL_pok_type;

    /** POK type Power System Module  */
#define SDL_TYPE_POK                          ((SDL_pok_type) 1U)
    /** POK_SA type Power System Module  */
#define SDL_TYPE_POK_SA                       ((SDL_pok_type) 2U)


/** @} */
/**
 * @brief This enumerator defines the possible ping-pong control values for the
 *        Power Subsystem modules
 *
 *  \anchor SDL_PWRSS__types
 *  \name POK/POR Deglitch types
 *
 *  @{
 *
 */

typedef  uint8_t                           SDL_pwrss_deglitch;

/** Deglitch period 5us */
#define SDL_PWRSS_DEGLITCH_5US            ((SDL_pwrss_deglitch) 0U)
/** Deglitch period 10us */
#define SDL_PWRSS_DEGLITCH_10US           ((SDL_pwrss_deglitch) 1U)
/** Deglitch period 15us */
#define SDL_PWRSS_DEGLITCH_15US           ((SDL_pwrss_deglitch) 2U)
/** Deglitch period 20us */
#define SDL_PWRSS_DEGLITCH_20US           ((SDL_pwrss_deglitch) 3U)
/** Get the Deglitch value */
#define SDL_PWRSS_DEGLITCH_GET_VALUE      ((SDL_pwrss_deglitch) 4U)
/** No update on hysteresis for the module  */
#define SDL_PWRSS_DEGLITCH_NO_ACTION      ((SDL_pwrss_deglitch) 5U)

/** @} */

/**
 * @brief This enumerator defines the possible deglitch control values for the
 *        Power Subsystem modules
 *
 *  \anchor SDL_PWRSS_pingpong_types_
 *  \name POK/POR Ping/Pong types
 *
 *  @{
 *
 */

typedef  uint8_t                           SDL_pwrss_pp;

/** Ping Pong Disable */
#define SDL_PWRSS_PP_MODE_DISABLE              ((SDL_pwrss_pp) 0U)
/** Ping Pong Enable */
#define SDL_PWRSS_PP_MODE_ENABLE               ((SDL_pwrss_pp) 1U)
/** No update on ping pong for the module  */
#define SDL_PWRSS_PP_MODE_NO_ACTION            ((SDL_pwrss_pp) 2U)

/** @} */


/**
 * @brief This enumerator defines the possible hysteresis control values for the
 *        Power Subsystem modules
 *
 *  \anchor SDL_PWRSS_hysteris_types
 *  \name POK/POR hysteresIs types
 *
 *  @{
 *
 */

typedef  uint8_t                           SDL_pwrss_hysteresis;

        /** Disable hysteresis for the module  */
#define SDL_PWRSS_SET_HYSTERESIS_DISABLE            ((SDL_pwrss_hysteresis) 0U)
    /** Enable hysteresis for the module  */
#define SDL_PWRSS_SET_HYSTERESIS_ENABLE             ((SDL_pwrss_hysteresis) 1U)
        /** Get hysteresis value for the module  */
#define SDL_PWRSS_GET_HYSTERESIS_VALUE              ((SDL_pwrss_hysteresis) 2U)
    /** No update on hysteresis for the module  */
#define SDL_PWRSS_HYSTERESIS_NO_ACTION              ((SDL_pwrss_hysteresis) 3U)

/** @} */

/**
 * @brief This enumerator defines the possible trim value for
 *        POK/POR modules
 *        Any value between 0 through 127 is valid TRIM value
 *
 *  \verbatim
 *
 *     POK        | Under Voltage   | Over Voltage     |
 *                | Detection       | Detection        | Step Resolution
 *    ----------- | --------------- | ---------------- | ----------------
 *     CORE_POK   | 475mV - 1.35V   |  725mV - 1.65V   | 0.0125V
 *     POK1.8     | 1.432V - 2.168V |  1.432V - 2.168V | 0.02V
 *     POK3.3     | 2.625V - 3.975V |  2.625V - 3.975V | 0.0375V
 *
 *  \endverbatim
 *
 *  \anchor SDL_PWRSS_trim_value
 *  \name POK/POR TRIM values
 *
 *  @{
 *
 */
typedef  uint8_t                          SDL_pwrss_trim;
    /** TRIM is 7 bit value, when the trim value is <= to the
      * MAX value, that value would be written to trim register
      * Any other values, would be treated as the command as described
      * below
      */
    /** TRIM is 7 bit value, and hence the maximum value is 127  */
#define SDL_PWRSS_MAX_TRIM_VALUE               ((SDL_pwrss_trim) 127U)

    /** No update on trim value read/write for the module  */
#define SDL_PWRSS_TRIM_NO_ACTION               ((SDL_pwrss_trim) 128U)

        /** Command to read the TRIM value  */
#define SDL_PWRSS_GET_TRIM_VALUE               ((SDL_pwrss_trim) 129U)

    /** Invalid TRIM value */
#define SDL_PWRSS_INVALID_TRIM_VALUE           ((SDL_pwrss_trim) 255U)


/** @} */

/**
 * @brief This enumerator defines the possible values of Voltage Detection modes
 *
 *  \anchor SDL_PWRSS_vd_modes
 *  \name POK/POR Voltage detection modes
 *
 *  @{
 *
 */

typedef  uint8_t                           SDL_pwrss_vd_mode;

    /** Enable under voltage detection for the module */
#define SDL_PWRSS_SET_UNDER_VOLTAGE_DET_ENABLE           ((SDL_pwrss_vd_mode) 0U)
    /** Enable over voltage detection for the module  */
#define SDL_PWRSS_SET_OVER_VOLTAGE_DET_ENABLE            ((SDL_pwrss_vd_mode) 1U)
    /** Enable ping-pong voltage detection for the module */
#define SDL_PWRSS_SET_PP_VOLTAGE_DET_ENABLE              ((SDL_pwrss_vd_mode) 2U)
    /** Get voltage detection for the module */
#define SDL_PWRSS_GET_VOLTAGE_DET_MODE                   ((SDL_pwrss_vd_mode) 3U)
    /** No update on voltage detection mode update for the module  */
#define SDL_PWRSS_VOLTAGE_DET_NO_ACTION                  ((SDL_pwrss_vd_mode) 4U)

/** @} */

/**
 * @brief This enumerator defines the POK Detection status values
 *
 *  \anchor SDL_POK_detection_status_values
 *  \name POK detection status
 *
 *  @{
 *
 */

typedef uint8_t                             SDL_POK_detection_status;
/** POK Detection disabled */
#define SDL_POK_DETECTION_DISABLED               ((SDL_POK_detection_status) 0U)
/** POK Detection Enabled */
#define SDL_POK_DETECTION_ENABLED                ((SDL_POK_detection_status) 1U)

/** @} */

/**
 * @brief This enumerator defines the POK Detection values
 *
 *  \anchor SDL_POK_detection_values
 *  \name POK detection values
 *
 *  @{
 *
 */

typedef uint8_t                             SDL_POK_detection;
/** POK Detection disabled */
#define SDL_POK_DETECTION_DISABLE               ((SDL_POK_detection) 0U)
/** POK Detection Enabled */
#define SDL_POK_DETECTION_ENABLE                ((SDL_POK_detection) 1U)
/** POK Detection No action */
#define SDL_POK_DETECTION_NO_ACTION             ((SDL_POK_detection) 2U)
/** POK Detection get value */
#define SDL_POK_GET_DETECTION_VALUE             ((SDL_POK_detection) 3U)


/** @} */

/**
 * @brief This enumerator defines the POK Enable selection source
 *
 *  \anchor SDL_POK_Enable Selection source values
 *  \name POK Enable Selection source Values
 *
 *  @{
 *
 */

typedef uint8_t                             SDL_POK_enSelSrc;
/** POK enables come from hardware tie offs */
#define SDL_POK_ENSEL_HWTIEOFFS                  ((SDL_POK_enSelSrc) 0U)
/** POK enables come from CTRLMMR_WKUP_PRG0_CTRL register*/
#define SDL_POK_ENSEL_PRG_CTRL                   ((SDL_POK_enSelSrc) 1U)
/** POK enable selection no action */
#define SDL_POK_ENSEL_NO_ACTION                  ((SDL_POK_enSelSrc) 2U)
/** POK enable selection Get value */
#define SDL_POK_GET_ENSEL_VALUE                  ((SDL_POK_enSelSrc) 3U)

/** @} */

/**
 * @brief This enumerator defines the trim selection values
 *
 *  \anchor SDL_POK_trim_selection
 *  \name POK trim selection values from HHV default or CTRL registers
 *
 *  @{
 *
 */

typedef uint8_t         SDL_por_trim_sel;

/** Trim selections for Bandgap and PORs come from HHV defaults */
#define  SDL_POR_TRIM_SELECTION_FROM_HHV_DEFAULT      ((uint8_t) 0U)

/** Trim selections for Bandgap and POKs come from CTRLMMR_WKUP_POR_BANDGAP_CTRL and
    POR_POKxxx_CTRL registers */
#define  SDL_POR_TRIM_SELECTION_FROM_CTRL_REGS        ((uint8_t) 1U)

/** Trim selections for Bandgap and POKs No Change */
#define  SDL_POR_TRIM_SELECTION_NO_CHANGE             ((uint8_t) 2U)

/** Trim Read selections Bandgap and POKs  */
#define  SDL_POR_TRIM_SELECTION_GET_VALUE             ((uint8_t) 3U)


/** @} */

/**
 * @brief This enumerator defines the POR Module State
 *
 *  \anchor SDL_POR_Module_state
 *  \name POR Module state values
 *
 *  @{
 *
 */

typedef   uint8_t               SDL_por_module_status ;

/** POR in functional mode */
#define   SDL_POR_MODULE_STATUS_FUNCTIONAL_MODE             ((SDL_por_module_status) 0U)
/** POR in Reset mode */
#define   SDL_POR_MODULE_STATUS_RESET_MODE                  ((SDL_por_module_status) 1U)
/** @} */


/**
 * @brief This enumerator defines the Wake up control MMR register
 *
 *  \anchor SDL_POK_
 *  \name POK POK ID values
 *
 *  @{
 *
 */
typedef SDL_mcu_ctrl_mmr_cfg0Regs SDL_mcuCtrlRegsBase_t;

/** @} */

/** @} */
/**
 *  \addtogroup SDL_IP_POK_DATASTRUCT
 *  @{
 */

/**
 * \brief POK Configuration structure
 *
 * This structure contains the POK Control register Configuration
 *
 */

typedef struct SDL_pokCfg
{
    /** hysteresis control */
    SDL_pwrss_hysteresis            hystCtrl;
    /** hysteresis control - OV */
    SDL_pwrss_hysteresis            hystCtrlOV;
    /** Voltage Detection Mode control */
    SDL_pwrss_vd_mode               voltDetMode;
    /** POK Trim bits 7 bits wide */
    SDL_pwrss_trim                  trim;
    /** POK Trim bits 7 bits widt for OV */
    SDL_pwrss_trim                  trimOV;
    /** POK Detection Enable */
    SDL_POK_detection               detectionCtrl;
    /** POK Enable Source control */
    SDL_POK_enSelSrc                pokEnSelSrcCtrl;
    /** Deglitch control */
    SDL_pwrss_deglitch              deglitch;
} SDL_POK_config;

/**
 * \brief POK Configuration structure read value
 *
 * This structure contains the POK Control register Configuration
 *
 */

typedef struct SDL_pokVal
{
    /** hysteresis control */
    SDL_pwrss_hysteresis            hystCtrl;
    /** hysteresis control for OV setting */
    SDL_pwrss_hysteresis            hystCtrlOV;
    /** Voltage Detection Mode control */
    SDL_pwrss_vd_mode               voltDetMode;
    /** POK Trim bits 7 bits wide */
    SDL_pwrss_trim                  trim;
    /** POK Trim bits for OV - 7 bits wide */
    SDL_pwrss_trim                  trimOV;
    /** POK detection Status  */
    SDL_POK_detection_status        detectionStatus;
    /** POK Enable Source control */
    SDL_POK_enSelSrc                pokEnSelSrcCtrl;
    /** POK Deglitch control */
    SDL_pwrss_deglitch              deglitch;
} SDL_pokVal_t;


/**
 * \brief POK functionality of POR Configuration structure
 *
 * This structure contains the configuration for POK functionality of
 * POR Control register
 *
 */

typedef struct SDL_pokPorCfg
{
    /** Mask HHV/SOC_PORz outputs when applying new trim values */
    bool                          maskHHVOutputEnable;
    /** POK Trim selection */
    SDL_por_trim_sel              trim_select;
} SDL_pokPorCfg_t;

/**
 * \brief POK functionality of POR Value structure
 *
 * This structure contains the value for POK functionality of
 * POR Control register
 *
 */

typedef struct SDL_pokPorVal
{
    /** Mask HHV/SOC_PORz outputs when applying new trim values */
    bool                          maskHHVOutputEnable;
    /** POK Trim selection */
    SDL_por_trim_sel              trim_select;
} SDL_pokPorVal_t;

/** @} */

int32_t SDL_pok_getPRGInfo(SDL_mcuCtrlRegsBase_t     *pBaseAddress,
                           SDL_PRG_Inst instance,
                           SDL_pokPRGInfo_t *pPRGInfo);


/* Function to get the error signal wrt given POKID for v1 of IP */
void sdlGetErrSig(uint32_t id, SDL_POK_Inst *instance, uint32_t *esm_err_sig_ov, uint32_t *esm_err_sig_uv, bool *usePorCfgFlag);


#ifdef __cplusplus
}
#endif  /* extern "C" */

#endif  /* end of SDL_POK_DEF_H definition */
