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

#ifndef SDL_VTM_PVT_SENSOR_LUT_H
#define SDL_VTM_PVT_SENSOR_LUT_H

#ifdef __cplusplus
extern "C" {
#endif

#include <string.h>
#include <stdbool.h>
#include "sdl_vtm_pvt_sensor.h"

/* Global Variables */

/* Below is the temperature in Celcius/Centegrade*/
const int32_t gSDL_pvt_poly0[] =
{
    150, 150, 150, 150, 150, 150, 150, 150, 150, 150,
    150, 150, 150, 150, 150, 150, 150, 150, 150, 150,
    150, 150, 150, 150, 150, 148, 146, 144, 142, 140,
    138, 136, 134, 132, 130, 128, 126, 124, 122, 120,
    118, 116, 114, 112, 110, 108, 106, 104, 102, 100,
    98, 96, 94, 92, 90, 88, 86, 84, 82, 80,
    78, 76, 74, 72, 70, 68, 66, 64, 62, 60,
    58, 56, 54, 52, 50, 48, 46, 44, 42, 40,
    38, 36, 34, 32, 30, 28, 26, 24, 22, 20,
    18, 16, 14, 12, 10, 8, 6, 4, 2, 0,
    -2, -4, -6, -8, -10, -12, -14, -16, -18, -20,
    -22, -24, -26, -28, -30, -32, -34, -36, -38, -40,
    -40, -40, -40, -40,-40, -40, -40, -40, -40
};

/* Below is the temperature in milli Celcius/Centegrade*/
const int32_t gSDL_pvt_poly[] =
{
    150000, 150000, 150000, 150000, 150000, 150000, 150000, 150000, 150000, 150000,
    150000, 150000, 150000, 150000, 150000, 150000, 150000, 150000, 150000, 150000,
    150000, 150000, 150000, 150000, 150000, 148000, 146000, 144000, 142000, 140000,
    138000, 136000, 134000, 132000, 130000, 128000, 126000, 124000, 122000, 120000,
    118000, 116000, 114000, 112000, 110000, 108000, 106000, 104000, 102000, 100000,
    98000, 96000, 94000, 92000, 90000, 88000, 86000, 84000, 82000, 80000,
    78000, 76000, 74000, 72000, 70000, 68000, 66000, 64000, 62000, 60000,
    58000, 56000, 54000, 52000, 50000, 48000, 46000, 44000, 42000, 40000,
    38000, 36000, 34000, 32000, 30000, 28000, 26000, 24000, 22000, 20000,
    18000, 16000, 14000, 12000, 10000, 8000, 6000, 4000, 2000, 0,
    -2000, -4000, -6000, -8000, -10000, -12000, -14000, -16000, -18000, -20000,
    -22000, -24000, -26000, -28000, -30000, -32000, -34000, -36000, -38000, -40000,
    -40000, -40000, -40000, -40000,-40000, -40000, -40000, -40000, -40000
};

extern const int32_t gSDL_pvt_poly[SDL_VTM_NUM_OF_ADC_CODES];

#ifdef __cplusplus
}
#endif  /* extern "C" */

#endif  /* end of SDL_VTM_PVT_SENSOR_LUT_H definition */


/* Nothing past this point */

