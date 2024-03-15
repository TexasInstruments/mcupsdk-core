/*
 *  Copyright (C) 2024 Texas Instruments Incorporated.
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
 *  \file  V1\sdl_vtm_pvt_sensor.h
 *
 *  \brief
 *     Header file containing various enumerations, structure definitions and function
 *  declarations for the Voltage and Thermal Monitor (VTM) PVT Sensor Workaround.
 */


#ifndef SDL_VTM_PVT_SENSOR_H
#define SDL_VTM_PVT_SENSOR_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>
#include <sdl/vtm/v1/sdl_ip_vtm.h>

/* Macros */
#define SDL_VTM_NUM_OF_ADC_CODES                            (129)

/* Uninitialized value */
#define SDL_VTM_VALUES_ARE_UNINITIALIZED                    (-1)
#define SDL_VTM_VALUES_ARE_INITIALIZED                      (1)

/* Minimum and Maximum temperatures */
#define SDL_VTM_TEMPERATURE_MILLI_DEGREE_C_MIN              (-40000)
#define SDL_VTM_TEMPERATURE_MILLI_DEGREE_C_MAX              (150000)

/**
 *  \brief VTM Temperature ADC code to Temperature conversion
 *
 *  \param adc_code                 [IN]   7 Bit ADC code
 *  \param pMilliDegreeTempVal      [OUT]  Pointer to Temperature in milli
 *                                         degree celcius
 *
 *  \return The SDL error code for the API.
 *                                 Success      : SDL_PASS
 *                                 Invalid Args : SDL_EBADARGS
 */
int32_t SDL_VTM_tsConvADCToTemp (SDL_VTM_adc_code       adcCode,
                                int32_t                 *pMilliDegreeTempVal);

/**
 *  \brief VTM Temperature to ADC conversion
 *
 *  \param milliDegreeTempVal          [IN] Temperature in milli degree celcius
 *  \param p_adc_code                  [OUT] pointer to 7 Bit ADC code
 *
 *  \return The SDL error code for the API.
 *                                 Success      : SDL_PASS
 *                                 Invalid Args : SDL_EBADARGS
 */
int32_t SDL_VTM_tsConvTempToAdc (int32_t             milliDegreeTempVal,
                                SDL_VTM_adc_code     *pAdcCode);

#ifdef __cplusplus
}
#endif  /* extern "C" */

#endif  /* end of SDL_VTM_PVT_SENSOR_H definition */
