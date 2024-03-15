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

#include <string.h>
#include <stdbool.h>
#include <sdl/include/soc_config.h>
#include "sdl_vtm_pvt_sensor.h"
#include "sdl_pvt_sensor_lut.h"
#include <sdl/include/sdl_types.h>
#include <stdint.h>
#include <sdl/vtm/v1/sdl_ip_vtm.h>

/* Global variables */

/* Internal functions */

 /**
 * Design: PROC_SDL-1322,PROC_SDL-1323
 */
int32_t SDL_VTM_tsConvADCToTemp (SDL_VTM_adc_code       adcCode,
                                 int32_t                *pMilliDegreeTempVal)
{
    int32_t retVal = SDL_PASS;

    if ((adcCode < (SDL_VTM_adc_code)0) ||
        (adcCode > (SDL_VTM_adc_code)(SDL_VTM_NUM_OF_ADC_CODES-1)))
    {
        retVal = SDL_EBADARGS;
    }

    if ((pMilliDegreeTempVal != NULL_PTR) &&	\
        (retVal                  == SDL_PASS))
    {
        *pMilliDegreeTempVal = gSDL_pvt_poly[adcCode];
    }
    else
    {
        retVal = SDL_EBADARGS;
    }
    return (retVal);
}

 /**
 * Design: PROC_SDL-1320,PROC_SDL-1321
 */
int32_t SDL_VTM_tsConvTempToAdc (int32_t  milliDegreeTempVal,
                                SDL_VTM_adc_code    *pAdcCode)

{
    int32_t             retVal;
    SDL_VTM_adc_code    low  = (SDL_VTM_adc_code)(0);
    SDL_VTM_adc_code    high = (SDL_VTM_adc_code)(SDL_VTM_NUM_OF_ADC_CODES-1);
    SDL_VTM_adc_code    mid;

    if ((milliDegreeTempVal > SDL_VTM_TEMPERATURE_MILLI_DEGREE_C_MAX) ||
        (milliDegreeTempVal < SDL_VTM_TEMPERATURE_MILLI_DEGREE_C_MIN))
    {
        retVal = SDL_EBADARGS;
    }
    else
    {
        retVal = SDL_PASS;
    }

    if ( (pAdcCode     != NULL_PTR) &&	\
         (retVal         == SDL_PASS))
    {
        /* Binary search to find the adc code */
        while (low < (high - (SDL_VTM_adc_code)1))
        {
            mid = (low + high) / 2;
            if (milliDegreeTempVal >= gSDL_pvt_poly[mid])
            {
                high = mid;
                if (milliDegreeTempVal == gSDL_pvt_poly[mid])
                {
                    break;
                }
            }
            else
            {
                low = mid;
            }
        }

        *pAdcCode =  mid;
    }
    else
    {
        retVal = SDL_EBADARGS;
    }

    return (retVal);
}

/* Nothing past this point */

