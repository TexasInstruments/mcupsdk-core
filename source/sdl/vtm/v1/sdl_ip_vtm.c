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


#include <stdint.h>
#include <sdl/vtm/v1/sdlr_vtm.h>
#include "sdl_ip_vtm.h"
#include <sdl/include/sdl_types.h>
/*=============================================================================
 *   functions
 *===========================================================================*/
/*=============================================================================
 *   macros
 *===========================================================================*/
#define SDL_VTM_VALUES_ARE_UNINITIALIZED    (-1)
#define SDL_VTM_VALUES_ARE_INITIALIZED      (1)
/* Delay for Reg Reads */
#define SDL_VTM_DOUT_REG_READ_DELAY         (100)


/*=============================================================================
 *  global variables
 *===========================================================================*/


/*=============================================================================
 *  Internal functions
 *===========================================================================*/


/*=============================================================================
 *  SDL IP functions
 *===========================================================================*/
 /**
 * Design: PROC_SDL-1319
 */
SDL_VTM_adc_code SDL_VTM_getAdcCode(SDL_VTM_InstTs  instance)
{
    SDL_VTM_adc_code adccode = 0xFF;

    if(instance < SDL_VTM_INSTANCE_TS_MAX_NUM)
    {
        switch(instance)
        {
            case SDL_VTM_INSTANCE_TS_0:
                adccode = (SDL_VTM_adc_code)SDL_REG32_FEXT((SDL_TOP_CTRL_U_BASE+SDL_VTM_TSENSE0_RESULT), \
                                            TOP_CTRL_TSENSE0_RESULT_TSENSE0_RESULT_DTEMP);
            break;
            case SDL_VTM_INSTANCE_TS_1:
                adccode = (SDL_VTM_adc_code)SDL_REG32_FEXT((SDL_TOP_CTRL_U_BASE+SDL_VTM_TSENSE1_RESULT), \
                                            TOP_CTRL_TSENSE1_RESULT_TSENSE1_RESULT_DTEMP);
            break;
            case SDL_VTM_INSTANCE_TS_2:
                adccode = (SDL_VTM_adc_code)SDL_REG32_FEXT((SDL_TOP_CTRL_U_BASE+SDL_VTM_TSENSE2_RESULT), \
                                            TOP_CTRL_TSENSE2_RESULT_TSENSE2_RESULT_DTEMP);
            break;
            case SDL_VTM_INSTANCE_TS_3:
                adccode = (SDL_VTM_adc_code)SDL_REG32_FEXT((SDL_TOP_CTRL_U_BASE+SDL_VTM_TSENSE3_RESULT), \
                                            TOP_CTRL_TSENSE3_RESULT_TSENSE3_RESULT_DTEMP);
            break;
              default:
                adccode = 0xFF;
            break;
        }
    }
   return adccode;
}


 /**
 * Design: PROC_SDL-1330,PROC_SDL-1331
 */
int32_t SDL_VTM_tsGetCtrl (SDL_VTM_InstTs  instance, uint32_t *ptsenseCTRL)
{
    int32_t sdlResult = SDL_PASS;

    /* argument checks */
    if((instance  >=  SDL_VTM_INSTANCE_TS_2) ||
       (ptsenseCTRL == NULL_PTR))
    {
        sdlResult = SDL_EBADARGS;
    }
    else
    {
        switch(instance)
        {
            case SDL_VTM_INSTANCE_TS_0:
                    *ptsenseCTRL = SDL_REG32_RD(SDL_TOP_CTRL_U_BASE+SDL_VTM_TSENSE0_CNTL);
            break;
            case SDL_VTM_INSTANCE_TS_1:
                    *ptsenseCTRL = SDL_REG32_RD(SDL_TOP_CTRL_U_BASE+SDL_VTM_TSENSE1_CNTL);
            break;
            default:
            break;
        }
    }
    return (sdlResult);
}

/* Nothing past this point */
