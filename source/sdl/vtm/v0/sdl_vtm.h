/********************************************************************
 * Copyright (C) 2022-23 Texas Instruments Incorporated
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

#ifndef SDL_IP_VTM_TOP_H_
#define SDL_IP_VTM_TOP_H_

#ifdef __cplusplus
extern "C"
{
#endif

#if defined (SOC_AM64X) || defined (SOC_AM243X)
#include <sdl/vtm/v0/soc/sdl_soc_vtm.h>
#include <sdl/vtm/v0/sdl_ip_vtm.h>
#endif

/**
 *
 *   @defgroup SDL_VTM_API APIs for VTM
 *   Provides the APIs for VTM.
 */

/**
@defgroup SDL_VTM_DATASTRUCT  VTM Data Structures
@ingroup SDL_VTM_API
*/
/**
@defgroup SDL_VTM_FUNCTION  VTM Functions
@ingroup SDL_VTM_API
*/

/**
 *  @addtogroup SDL_VTM_DATASTRUCT
    @{
 */
/** \brief VTM Voltage Domain configuration
 *
 *  This structure contains VTM Voltage Domain configuration
 *
 */
typedef struct {
    /** Valid control bit map the configuration */
    SDL_VTM_configVdCtrl    configVdCtrl;
    /** VID OPP  */
    SDL_VTM_vid_opp         vid_opp;
    /** VID OPP value */
    uint8_t                 vid_opp_val;
    /** VD Event selection */
    SDL_VTM_vdEvtSel_set    vd_temp_evts;
    /** Global configuration */
    SDL_VTM_tsGlobal_cfg       tsGlobal_cfg;
} SDL_VTM_configVd;

/** \brief VTM temperature sensor configuration
 *
 *  This structure contains VTM temperature sensor configuration
 *
 */
 typedef struct {
        /** Valid control bit map the configuration */
    SDL_VTM_configTsCtrl    configTsCtrl;
    /**  Temperature sensor control configuration */
    SDL_VTM_Ctrlcfg            tsCtrl_cfg;
    /**  Thermal shutdown range high value*/
    int32_t                 high_temp_in_milli_degree_celsius;
    /**  Thermal shutdown range low value*/
    int32_t                 low_temp_in_milli_degree_celsius;
    /**  Threshold value*/
    SDL_VTM_tsThrVal           thr_val;
} SDL_VTM_configTs;


/** \brief VTM Static Registers for Voltage Domain
 *
 *  This structure contains VTM static register for a given configuration
 *  The register values are not expected to change until a new configuration
 *  is done.
 *
 */

typedef struct {
    /** Voltage domain event selection control values */
    uint32_t                    vtm_vd_evt_sel_ctrl;
    /** VTM OPP voltages */
    uint32_t                    vtm_vd_opp_vid;
    /** VTM global configuration configuration */
    SDL_VTM_tsGlobal_cfg        vtm_global_cfg;
} SDL_VTM_staticRegsVd;

/** \brief VTM Static Registers for Temperature sensor
 *
 *  This structure contains VTM static register for a given configuration
 *  The register values are not expected to change until a new configuration
 *  is done.
 *
 */

typedef struct
{
    /** VTM individual sensor cfg2 control information */
    uint32_t                    vtm_ts_ctrl2;
    /** VTM individual sensor cfg1 control information */
    uint32_t                    vtm_ts_ctrl;
    /** VTM individual sensor threshold information */
    uint32_t                    vtm_ts_th;
    /** VTM individual sensor threshold2 information */
    uint32_t                    vtm_ts_th2;
} SDL_VTM_staticRegsTs;

/** @} */

/**
 *  @addtogroup SDL_VTM_FUNCTION
    @{
 */

/**
 *  \brief VTM Voltage Domain initialization
 *
 *  This function executes a VTM Voltage Domain initialization of the
 *  specified type for a specific VTM VD instance.
 *  VTM is a feature that is used to set OPP VID, event selection, global
 *  configuration
 *
 *  \param instance         [IN]  VTM Voltage Domain instance
 *  \param pConfig          [IN]  Pointer to VTM VD configuration (optional)
 *
 *  \return The SDL error code for the API.
 *                                 If pConfig is NULL: SDL_EBADARGS
 *                                 If instance is invalid: SDL_EBADARGS
 *                                 Success: SDL_PASS
 */
int32_t SDL_VTM_initVd(SDL_VTM_InstVd instance, const SDL_VTM_configVd *pConfig);

/**
 *  \brief VTM Temperature sensor initialization
 *
 *  This function executes a VTM Temperature sensor initialization of the
 *  specified type for a specific VTM TS instance.
 *  VTM is a feature that is used to set control register, thermal shutdown range,
 *    Threshold values and interrupts
 *
 *  \param instance         [IN]  VTM Temperature sensor instance
 *  \param pConfig          [IN]  Pointer to VTM TS configuration (optional)
 *
 *  \return The SDL error code for the API.
 *                                 If pConfig is NULL: SDL_EBADARGS
 *                                 If instance is invalid: SDL_EBADARGS
 *                                 Success: SDL_PASS
 */
int32_t SDL_VTM_initTs(SDL_VTM_InstTs instance, const SDL_VTM_configTs *pConfig);

/**
 *  \brief Read VTM Temperature sensor values in degree celcius
 *
 *  \param instance         [IN]  VTM Temperature sensor instance
 *  \param pTempVal         [OUT]  Pointer to Temperature sensor values
 *
 *  \return The SDL error code for the API.
 *                                 If pTempVal is NULL: SDL_EBADARGS
 *                                 If instance is invalid: SDL_EBADARGS
 *                                 Success: SDL_PASS
 */
int32_t SDL_VTM_getTemp(SDL_VTM_InstTs instance, uint32_t *pTempVal);

/**
 *  \brief Read VTM Temperature sensor status
 *
 *  \param instance         [IN]  VTM Temperature sensor instance
 *  \param pCtrl            [IN]  Pointer to VTM interrupt control
 *  \param pStat_val        [OUT] Pointer to Temperature sensor status
 *
 *  \return The SDL error code for the API.
 *                                 If pCtrl and pStat_val are NULL: SDL_EBADARGS
 *                                 If instance is invalid: SDL_EBADARGS
 *                                 Success: SDL_PASS
 */
int32_t SDL_VTM_getSensorStatus(SDL_VTM_InstTs instance, const SDL_VTM_Stat_read_ctrl *pCtrl, \
                                SDL_VTM_Stat_val *pStat_val);


/**
 *  \brief Set maximum Threshold values of VTM Temperature sensor
 *
 *  \param instance         [IN]  VTM Temperature sensor instance
 *  \param intrCtrl         [IN]  VTM interrupt control
 *
 *  \return The SDL error code for the API.
 *                                 If intrCtrl and instance are invalid: SDL_EBADARGS
 *                                 Success: SDL_PASS
 */

int32_t SDL_VTM_intrCntrl(SDL_VTM_InstVd instance, SDL_VTM_intrCtrl intrCtrl);

/**
 *  \brief VTM Voltage Domain configures verification
 *
 *  \param instance         [IN]  VTM Voltage Domain instance
 *  \param pConfig          [IN]  Pointer to VTM VD configuration (optional)
 *
 *  \return The SDL error code for the API.
 *                                 If pConfig is NULL: SDL_EBADARGS
 *                                 If instance is invalid: SDL_EBADARGS
 *                                 Success: SDL_PASS
 */
int32_t SDL_VTM_verifyConfigVd(SDL_VTM_InstVd instance, const SDL_VTM_configVd *pConfig);

/**
 *  \brief VTM Temperature sensor configures verification
 *
 *  \param instance         [IN]  VTM Temperature sensor instance
 *  \param pConfig          [IN]  Pointer to VTM VD configuration (optional)
 *
 *  \return The SDL error code for the API.
 *                                 If pConfig is NULL: SDL_EBADARGS
 *                                 If instance is invalid: SDL_EBADARGS
 *                                 Success: SDL_PASS
 */
int32_t SDL_VTM_verifyConfigTs(SDL_VTM_InstTs instance, const SDL_VTM_configTs *pConfig);

/**
 *  \brief VTM Voltage Domain readback Static configuration registers
 *
 *  \param instance         [IN]  VTM Voltage Domain instance
 *  \param pStaticRegs      [OUT]  Pointer to VTM VD static register
 *
 *  \return The SDL error code for the API.
 *                                 If pStaticRegs is NULL: SDL_EBADARGS
 *                                 If instance is invalid: SDL_EBADARGS
 *                                 Success: SDL_PASS
 */
int32_t SDL_VTM_getStaticRegistersVd(SDL_VTM_InstVd instance, SDL_VTM_staticRegsVd *pStaticRegs);

/**
 *  \brief VTM Temperature sensor readback Static configuration registers
 *
 *  \param instance         [IN]  VTM Temperature sensor instance
 *  \param pStaticRegs      [OUT]  Pointer to VTM TS static register
 *
 *  \return The SDL error code for the API.
 *                                 If pStaticRegs is NULL: SDL_EBADARGS
 *                                 If instance is invalid: SDL_EBADARGS
 *                                 Success: SDL_PASS
 */
int32_t SDL_VTM_getStaticRegistersTs(SDL_VTM_InstTs instance, SDL_VTM_staticRegsTs *pStaticRegs);

#ifdef __cplusplus
}
#endif  /* extern "C" */

/** @} */


#endif /* SDL_IP_VTM_TOP_H_ */
