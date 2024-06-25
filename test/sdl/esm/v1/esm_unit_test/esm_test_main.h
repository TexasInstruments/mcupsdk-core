/* Copyright (c) 2021 Texas Instruments Incorporated
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
 *  \file     esm_test_main.h
 *
 *  \brief    This file contains ESM test code defines.
 *
 **/

/*===========================================================================*/
/*                         Include files                                     */
/*===========================================================================*/
/*===========================================================================*/

#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include <sdl/include/sdl_types.h>
#include <sdl/esm/v1/sdl_esm.h>
#include <sdl/include/hw_types.h>
#include <dpl_interface.h>
#include <sdl/esm/v1/v1_0/sdlr_esm.h>
#include "parity_main.h"

#define BITS_PER_WORD (32u)

#if defined (SOC_AM273X)
#include <sdl/include/am273x/sdlr_soc_baseaddress.h>
#include <sdl/esm/soc/am273x/sdl_esm_core.h>

#elif defined (SOC_AWR294X)
#include <sdl/include/awr294x/sdlr_soc_baseaddress.h>
#include <sdl/esm/soc/awr294x/sdl_esm_core.h>
#endif

#define SDL_TEST_ESM_BASE  SDL_MSS_ESM_U_BASE
#define ESM_TEST_BASE SDL_MSS_ESM_U_BASE
#define HI_INTNO (SDL_MSS_INTR_MSS_ESM_HI)
#define LO_INTNO (SDL_MSS_INTR_MSS_ESM_LO)

#if !defined(SDL_ESM_TEST_H)
#define SDL_ESM_TEST_H

/*===========================================================================*/
/*                         Declarations                                      */
/*===========================================================================*/

/* Define the test interface */
typedef struct sdlEsmTest_s
{
    int32_t  (*testFunction)(void);   /* The code that runs the test */
    char      *name;                  /* The test name */
    int32_t    testStatus;            /* Test Status */
} sdlEsmTest_t;

/*===========================================================================*/
/*                         Macros                                            */
/*===========================================================================*/
#define SDL_APP_TEST_NOT_RUN        (-(int32_t) (2))
#define SDL_APP_TEST_FAILED         (-(int32_t) (1))
#define SDL_APP_TEST_PASS           ( (int32_t) (0))
#define SDL_ESM_INSTANCE_INVLD      (246u)
#define ESM_INTR_GRP_NUM                  (32U)
/*===========================================================================*/
/*                         Internal function declarations                    */
/*===========================================================================*/

/*===========================================================================*/
/*                         External function declarations                    */
/*===========================================================================*/
extern int32_t sdl_Esm_posTest(void);
extern int32_t sdl_Esm_negTest(void);
/*===========================================================================*/
/*                   Local Function definitions                              */
/*===========================================================================*/

#endif /* SDL_ESM_TEST_H */
/* Nothing past this point */