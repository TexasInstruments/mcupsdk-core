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


#ifndef SDL_SOC_VTM_H_
#define SDL_SOC_VTM_H_

#include <sdl/include/sdlr.h>
#include <sdl/include/soc_config.h>
#ifdef __cplusplus
extern "C"
{
#endif

/** ---------------------------------------------------------------------------
 * @brief This enumerator defines the VTM Temperature sensor
 *
 *  \anchor SDL_VTM_InstTs
 *  \name VTM Temperature sensor
 *
 *  @{
 * ----------------------------------------------------------------------------
 */
typedef enum
{
   /** Temperature sensor  0 */
   SDL_VTM_INSTANCE_TS_0,
   /** Temperature sensor  1 */
   SDL_VTM_INSTANCE_TS_1,
   /** Temperature sensor  2 */
   SDL_VTM_INSTANCE_TS_2,
   /** Temperature sensor  3 */
   SDL_VTM_INSTANCE_TS_3,
   /** No. of Temperature sensor */
   SDL_VTM_INSTANCE_TS_MAX_NUM
}SDL_VTM_InstTs;

/* @} */


bool SDL_VTM_getBaseAddr(uint32_t *vtmBaseAddr);
/* @} */


#ifdef __cplusplus
}
#endif
#endif /* SDL_SOC_VTM_H_ */

