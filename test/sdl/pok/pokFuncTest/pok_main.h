/* Copyright (c) 2023 Texas Instruments Incorporated
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
 *  \file     test_main.h
 *
 *  \brief    This file contains POK test code defines.
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
#include <sdl/include/soc_config.h>
#include <sdl/pok/v1/sdl_pok.h>
#include <sdl/pok/v1/sdl_ip_pok.h>
#include <sdl/sdl_esm.h>
#include <sdl/include/am64x_am243x/sdlr_intr_mcu_esm0.h>
#include <drivers/soc/am64x_am243x/soc.h>
#include <sdl/dpl/sdl_dpl.h>
#include <dpl_interface.h>
#include <kernel/dpl/DebugP.h>
#if !defined(TEST_MAIN_H)
#define TEST_MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

#define SDL_INVALID_POK_ID 0xff;

/*===========================================================================*/
/*                         Declarations                                      */
/*===========================================================================*/

/* Define the test interface */
typedef struct sdlPokTest_s
{
    int32_t  (*testFunction)(void);   /* The code that runs the test */
    char      *name;                  /* The test name */
    int32_t    testStatus;            /* Test Status */
} sdlPokTest_t;

/*===========================================================================*/
/*                         Macros                                            */
/*===========================================================================*/
#define SDL_APP_TEST_NOT_RUN        (-(int32_t) (2))
#define SDL_APP_TEST_FAILED         (-(int32_t) (1))
#define SDL_APP_TEST_PASS           ( (int32_t) (0))



/*===========================================================================*/
/*                         Internal function declarations                    */
/*===========================================================================*/
void sdlApp_print(const char * str);

/*===========================================================================*/
/*                         External function declarations                    */
/*===========================================================================*/
extern int32_t sdlPOKInPor_funcTest(void);
extern int32_t sdlPOK_funcTest(void);

#if defined (SOC_AM64X)
#if defined (M4F_CORE)
int32_t SDL_ESM_applicationCallbackFunction(SDL_ESM_Inst esmInst,
                                            SDL_ESM_IntType esmIntrType,
                                            uint32_t grpChannel,
                                            uint32_t index,
                                            uint32_t intSrc,
                                            void *arg);
											
extern int32_t SDL_ESM_applicationCallbackFunction(SDL_ESM_Inst esmInstType,
                                                   SDL_ESM_IntType esmIntType,
                                                   uint32_t grpChannel,
                                                   uint32_t index,
                                                   uint32_t intSrc,
                                                   void *arg);
											
#endif
#endif




/*===========================================================================*/
/*                   Local Function definitions                              */
/*===========================================================================*/

#ifdef __cplusplus
}

#endif /*extern "C" */

#endif /* TEST_MAIN_H */
/* Nothing past this point */