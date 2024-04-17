/*
 * Copyright (C) 2024 Texas Instruments Incorporated
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *   Redistributions of source code must retain the above copyright
 *   notice, this list of conditions and the following disclaimer.
 *
 *   Redistributions in binary form must reproduce the above copyright
 *   notice, this list of conditions and the following disclaimer in the
 *   documentation and/or other materials provided with the
 *   distribution.
 *
 *   Neither the name of Texas Instruments Incorporated nor the names of
 *   its contributors may be used to endorse or promote products derived
 *   from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef SDL_IP_VTM_TOP_H_
#define SDL_IP_VTM_TOP_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <sdl/vtm/v1/soc/sdl_soc_vtm.h>
#include <sdl/vtm/v1/sdl_ip_vtm.h>

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
/** \brief VTM temperature sensor configuration
 *
 *  This structure contains VTM temperature sensor configuration
 *
 */
 typedef struct
 {
    /**  Enable Thermal shutdown configuration and interrupts for TS0*/
    uint8_t cfgTs0Tshut;
    /**  Enable Alert configuration and interrupts for TS0*/
    uint8_t cfgTs0Alert;
    /**  TS0 Thermal shutdown hot*/
    int32_t                 ts0_ts_hot_temp_in_milli_degree_celsius;
     /**  TS0 Thermal shutdown cold*/
    int32_t                 ts0_ts_cold_temp_in_milli_degree_celsius;
    /**  TS0 Alert Threshold Hot*/
    int32_t                 ts0_alert_hot_temp_in_milli_degree_celsius;
    /**  TS0 Alert Threshold Cold*/
    int32_t                 ts0_alert_cold_temp_in_milli_degree_celsius;
    /**  Enable Thermal shutdown configuration and interrupts for TS1*/
    uint8_t cfgTs1Tshut;
    /**  Enable Alert configuration and interrupts for TS1*/
    uint8_t cfgTs1Alert;
    /**  TS1 Thermal shutdown hot*/
    int32_t                 ts1_ts_hot_temp_in_milli_degree_celsius;
    /**  TS1 Thermal shutdown cold*/
    int32_t                 ts1_ts_cold_temp_in_milli_degree_celsius;
    /**  TS1 Alert Threshold Hot*/
    int32_t                 ts1_alert_hot_temp_in_milli_degree_celsius;
    /**  TS1 Alert Threshold Cold*/
    int32_t                 ts1_alert_cold_temp_in_milli_degree_celsius;

} SDL_VTM_configTs;

/** \brief VTM Static Registers for Temperature sensor
 *
 *  This structure contains VTM static register for a given configuration
 *  The register values are not expected to change until a new configuration
 *  is done.
 *
 */

typedef struct
{
    uint32_t                    vtm_ts_th2;
    uint32_t                    vtm_ts_cfg;
    uint32_t                    vtm_ts0_tshut;
    uint32_t                    vtm_ts0_alert;
    uint32_t                    vtm_ts0_ctrl;
    uint32_t                    vtm_ts1_tshut;
    uint32_t                    vtm_ts1_alert;
    uint32_t                    vtm_ts1_ctrl;
} SDL_VTM_staticRegsTs;

/** @} */

/**
 *  @addtogroup SDL_VTM_FUNCTION
    @{
 */

/**
 *  \brief VTM Temperature sensor initialization
 *
 *  This function executes a VTM Temperature sensor initialization of the
 *  specified type for a specific VTM TS instance.
 *  VTM is a feature that is used to set control register, thermal shutdown range,
 *    Threshold values and interrupts
 *
 *  \param pConfig          [IN]  Pointer to VTM TS configuration (optional)
 *
 *  \return The SDL error code for the API.
 *                                 If pConfig is NULL: SDL_EBADARGS
 *                                 Success: SDL_PASS
 */
int32_t SDL_VTM_initTs(const SDL_VTM_configTs *pConfig);

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
 *  \brief Set thresholds for temperature alerts
 *
 *  \param instance         [IN]  VTM Temperature sensor instance
 *  \param alert_th_hot     [IN]  Alert hot temperature in milli degree celcius
 *  \param alert_th_cold    [IN]  Alert cold temperature in milli degree celcius
 *
 *  \return The SDL error code for the API.
 *                                 If instance is invalid: SDL_EBADARGS
 *                                 If temperatures are invalid: SDL_EBADARGS
 *                                 Success: SDL_PASS
 */


int32_t SDL_VTM_setAlertTemp(SDL_VTM_InstTs instance, int32_t alert_th_hot,
                             int32_t alert_th_cold);

/**
 *  \brief Set thresholds for temperature Shutdown
 *
 *  \param instance         [IN]  VTM Temperature sensor instance
 *  \param tshut_th_hot     [IN]  Shutdown hot temperature in milli degree celcius
 *  \param tshut_th_cold    [IN]  Shutdown cold temperature in milli degree celcius
 *
 *  \return The SDL error code for the API.
 *                                 If instance is invalid: SDL_EBADARGS
 *                                 If temperatures are invalid: SDL_EBADARGS
 *                                 Success: SDL_PASS
 */
int32_t SDL_VTM_setTShutTemp(SDL_VTM_InstTs instance, int32_t tshut_th_hot, \
                             int32_t tshut_th_cold);

/**
 *  \brief API to set/Clear masks for generating interrupts
 *
 *  \param instance         [IN]  VTM Temperature sensor instance
 *  \param hotintr          [IN]  Enable/disable hot interrupt.
 *  \param coldintr         [IN]  Enable/disable cold interrupt.
*   \param lowthresholdintr [IN]  Enable/disable low treshold interrupt.
 *
 *  \return The SDL error code for the API.
 *                                 If instance is invalid: SDL_EBADARGS
 *                                 Success: SDL_PASS
 */
int32_t SDL_VTM_setClearInterrupts(SDL_VTM_InstTs instance, uint8_t  hotintr, \
                                 uint8_t  coldintr, uint8_t  lowthresholdintr);

/**
 *  \brief Read VTM Temperature sensor status
 *
 *  \param pStat_val        [OUT] Pointer to Temperature sensor status
 *
 *  \return The SDL error code for the API.
 *                                 If pStat_val are NULL: SDL_EBADARGS
 *                                 Success: SDL_PASS
 */
int32_t SDL_VTM_getSensorStatus(SDL_VTM_Stat_val *pStat_val);

/**
 *  \brief Enable Required Temperature sensors
 *
 *  \param sensorSelect     [IN]  Enable configured sensors
 *  \param delay            [IN]  delay between each read of temperature sensor.
 *                                0 is not valid value.
 *
 */
void SDL_VTM_enableTs(uint32_t sensorSelect, uint8_t delay);

/**
 *  \brief Enable Temperature Controller
 *
 */
void SDL_VTM_enableTc(void);

/**
 *  \brief Disable Temperature Controller
 *
 */
void SDL_VTM_disableTc(void);

/**
 *  \brief VTM Temperature sensor readback Static configuration registers
 *
 *  \param pStaticRegs      [OUT]  Pointer to VTM TS static register
 *
 *  \return The SDL error code for the API.
 *                                 If pStaticRegs is NULL: SDL_EBADARGS
 *                                 If instance is invalid: SDL_EBADARGS
 *                                 Success: SDL_PASS
 */
int32_t SDL_VTM_getStaticRegistersTs(SDL_VTM_staticRegsTs *pStaticRegs);

/**
 *  \brief Enable Warm Reset Generation
 *
 *  \param instance         [IN]  VTM Temperature sensor instance
 *
 *  \return The SDL error code for the API.
 *                                 If instance is invalid: SDL_EBADARGS
 *                                 Success: SDL_PASS
 */
int32_t SDL_VTM_enableESMWarmReset(SDL_VTM_InstTs instance);

#ifdef __cplusplus
}
#endif  /* extern "C" */

#endif /* SDL_IP_VTM_TOP_H_ */

/** @} */
