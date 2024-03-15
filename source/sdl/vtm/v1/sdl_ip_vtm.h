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

#ifndef SDL_IP_VTM_H
#define SDL_IP_VTM_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>
#include <sdl/vtm/v1/sdlr_vtm.h>
#include <sdl/vtm/v1/soc/am263px/sdl_soc_vtm.h>

/**
 *  \defgroup SDL_IP_VTM_API VTM Low-Level API
 *  \ingroup SDL_VTM_API
 *
 *  This module contains the Low-Level APIs to program and use the VTM module.
 *
 *  @{
 */

/**
 * \ingroup SDL_VTM_API
 * \defgroup SDL_IP_VTM_Enum VTM IP Enumerated Data Types
 * @{
 *  Provides the APIs for VTM IP.
 */

/**
 * \brief This enumerator define for VTM VD configuration valid map
 */

typedef uint8_t SDL_VTM_configVdCtrl;
#define SDL_VTM_VD_CONFIG_CTRL_VID_OPP                (1U)
#define SDL_VTM_VD_CONFIG_CTRL_EVT_SEL			      (2U)
#define SDL_VTM_VD_CONFIG_CTRL_GLB_CFG			      (4U)


/**
 * \brief This enumerator define for VTM TS configuration valid map
 *
 *  \anchor SDL_VTM_configTsCtrl
 */

typedef uint8_t SDL_VTM_configTsCtrl;
#define SDL_VTM_VD_CONFIG_CTRL_SET_CTL                (1U)
#define SDL_VTM_VD_CONFIG_CTRL_OUTRNG_ALRT		      (2U)
#define SDL_VTM_VD_CONFIG_CTRL_SET_THR			      (4U)


/**
 * \brief This enumerator defines the possible VID Codes to set various
 *        voltage domain supply voltages
 *
 *  \anchor SDL_VTM_vid_opp
 *
 */

#define SDL_VTM_BGR_OFF             0x1U
#define SDL_VTM_BGR_ON              0x0U

#define SDL_VTM_AIP_OFF             0x1U
#define SDL_VTM_AIP_ON              0x0U

#define SDL_VTM_TMPSOFF_OFF         0x1U
#define SDL_VTM_TMPSOFF_ON          0x0U

#define SDL_VTM_TMPSOFF_OFF         0x1U
#define SDL_VTM_TMPSOFF_ON          0x0U

#define SDL_VTM_SENSOR_SEL0         (1U << 0U)
#define SDL_VTM_SENSOR_SEL1         (1U << 1U)
#define SDL_VTM_SENSOR_SEL2         (1U << 2U)
#define SDL_VTM_SENSOR_SEL3         (1U << 3U)

#define SDL_VTM_TSENSE_EN           0x1U
#define SDL_VTM_TSENSE_DIS          0x0U

#define SDL_VTM_TSENSE0_LOW_TH_FLAG       (1U << 0U)
#define SDL_VTM_TSENSE0_HOT_FLAG          (1U << 2U)
#define SDL_VTM_TSENSE0_COLD_FLAG         (1U << 1U)
#define SDL_VTM_TSENSE1_LOW_TH_FLAG       (1U << 4U)
#define SDL_VTM_TSENSE1_HOT_FLAG          (1U << 6U)
#define SDL_VTM_TSENSE1_COLD_FLAG         (1U << 5U)

#define SDL_VTM_OVERRIDE_PATTERN           0x7U

#define SDL_VTM_MASK_LOW_TH     (1U)
#define SDL_VTM_MASK_COLD       (1U)
#define SDL_VTM_MASK_HOT        (1U)


#define SDL_VTM_FREEZE_FIFO     1U
#define SDL_VTM_CLR_FIFO        1U
#define SDL_VTM_CLR_ACCU        1U

#define SDL_VTM_BUF_NUM0        0U
#define SDL_VTM_BUF_NUM1        1U
#define SDL_VTM_BUF_NUM2        2U
#define SDL_VTM_BUF_NUM3        3U

#define SDL_VTM_TSENSE_OFF      0U
#define SDL_VTM_TSENSE_ON       1U

#define SDL_VTM_MAXDELAY         0x2FU

/**
 * \brief This typedef for
 *        VTM temperature sensor ADC code
 *        This is the data_out value of the temperature sensor stat register
 *
 *  \anchor SDL_VTM_adc_code
 */

typedef  int16_t SDL_VTM_adc_code;

/** \brief VTM temperature sensor Stat values
 *
 *  This structure contains VTM temperature sensor Stat values
 *
 */

typedef struct
{
    /** reflects the status of temperature Sensor 1 hot event detect. */
    uint8_t                    s1HotEvent;
    /** reflects the status of the temperature Sensor 1 cold event. */
    uint8_t                    s1ColdEvent;
    /** reflects the status of the temperature Sensor 1 low threshold event. */
    uint8_t                    s1LowThresholdEvent;
    /** reflects the status of temperature Sensor 0 hot event detect. */
    uint8_t                    s0HotEvent;
    /** reflects the status of the temperature Sensor  cold event. */
    uint8_t                    s0ColdEvent;
    /** reflects the status of the temperature Sensor 0 low threshold event. */
    uint8_t                    s0LowThresholdEvent;
} SDL_VTM_Stat_val;

/** @} */

/**
* \defgroup SDL_IP_VTM_FUNCTION  VTM IP Functions
* \ingroup SDL_VTM_API
* @{
*/


/**
 *  \brief read Temperature sensor ADC code
 *
 *  \param instance            [IN] sensor for which ADC code is needed

 *  \return The SDL SDL_VTM_adc_code
 */
SDL_VTM_adc_code SDL_VTM_getAdcCode(SDL_VTM_InstTs  instance);

/**
 *  \brief Read VTM Temperature Sensor Control
 *
 *  \param instance         [IN]  VTM Temperature sensor instance
 *  \param pTsCtrlCfg       [IN]  Pointer to temperature sensor control
 *                                configuration

 *  \return The SDL error code for the API.
 *                                 Success      : SDL_PASS
 *                                 Fail         : SDL_EFAIL
 *                                 Invalid Args : SDL_EBADARGS
 */
int32_t SDL_VTM_tsGetCtrl (SDL_VTM_InstTs   instance,
                          uint32_t         *pTsCtrlCfg);

/**
 *  \brief VTM Temperature ADC code to Temperature conversion
 *
 *  \param adcCode                 [IN]   7 Bit ADC code
 *  \param pMilliDegreeTempVal     [OUT]  Pointer to Temperature in milli
 *                                        degree celcius
 *
 *  \return The SDL error code for the API.
 *                                 Success      : SDL_PASS
 *                                 Invalid Args : SDL_EBADARGS
 */
int32_t SDL_VTM_tsConvADCToTemp (SDL_VTM_adc_code        adcCode,
                                int32_t                 *pMilliDegreeTempVal);

/**
 *  \brief VTM Temperature to ADC code conversion
 *
 *  \param milliDegreeTempVal    [IN]   Temperature in milli degree celcius
 *  \param pAdcCode              [OUT]  Pointer to 7 Bit ADC code
 *
 *  \return The SDL error code for the API.
 *                                 Success      : SDL_PASS
 *                                 Invalid Args : SDL_EBADARGS
 */
int32_t SDL_VTM_tsConvTempToAdc (int32_t milliDegreeTempVal,
                                 SDL_VTM_adc_code *pAdcCode);


/** @} */
/** @} */
#ifdef __cplusplus
}
#endif  /* extern "C" */
#endif  /* end of SDL_IP_VTM_H definition */