/* Copyright (c) 2021-2024 Texas Instruments Incorporated
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
#include <kernel/dpl/ClockP.h>
#include <sdl/include/sdl_types.h>
#if defined (SOC_AM263X)
#include <sdl/esm/v0/v0_0/sdl_ip_esm.h>
#include <sdl/esm/v0/v0_0/sdl_esm_priv.h>
#include <sdl/esm/v0/esm.h>
#include <sdl/esm/v0/sdl_esm.h>
#endif

#if defined (SOC_AM263PX) || defined (SOC_AM261X)
#include <sdl/esm/v2/v2_0/sdl_ip_esm.h>
#include <sdl/esm/v2/v2_0/sdl_esm_priv.h>
#include <sdl/esm/v2/esm.h>
#include <sdl/esm/v2/sdl_esm.h>
#endif

#define BITS_PER_WORD (32u)

#if defined (SOC_AM64X)
#include <sdl/include/am64x_am243x/sdlr_soc_baseaddress.h>
#include <sdl/esm/soc/am64x/sdl_esm_core.h>
#define SDL_TEST_ESM_BASE  SDL_MCU_ESM0_CFG_BASE
#endif

#if defined (SOC_AM263X)
#include <sdl/include/am263x/sdlr_soc_baseaddress.h>
#include <sdl/esm/soc/am263x/sdl_esm_core.h>
#endif

#if defined (SOC_AM263PX)
#include <sdl/include/am263px/sdlr_soc_baseaddress.h>
#include <sdl/esm/soc/am263px/sdl_esm_core.h>
#endif
#if defined (SOC_AM261X)
#include <sdl/include/am261x/sdlr_soc_baseaddress.h>
#include <sdl/esm/soc/am261x/sdl_esm_core.h>
#endif



#if defined (SOC_AM263X) || defined (SOC_AM263PX) || defined (SOC_AM261X)
#define SDL_TEST_ESM_BASE  SDL_TOP_ESM_U_BASE
#endif

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
/*===========================================================================*/
/*                         Internal function declarations                    */
/*===========================================================================*/
void sdlApp_print(char * str);

/*===========================================================================*/
/*                         External function declarations                    */
/*===========================================================================*/
extern int32_t sdl_Esm_posTest(void);
extern int32_t sdl_Esm_negTest(void);
extern int32_t ESM_selfTest(void);
extern int32_t test_sdr_test(void);
/*===========================================================================*/
/*                   Local Function definitions                              */
/*===========================================================================*/

#endif /* SDL_ESM_TEST_H */
/* Nothing past this point */
