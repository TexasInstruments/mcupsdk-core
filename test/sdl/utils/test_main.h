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
 *  \file     main.h
 *
 *  \brief    This file contains test code defines.
 *
 **/

#ifndef INCLUDE_TEST_UTILS_H_
#define INCLUDE_TEST_UTILS_H_

/*===========================================================================*/
/*                         Include files                                     */
/*===========================================================================*/
#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include <sdl/include/sdl_types.h>
#include <sdl/soc.h>


#include "ti_drivers_config.h"
#include "ti_drivers_open_close.h"
#include "ti_board_open_close.h"
#include <kernel/dpl/DebugP.h>

#include <sdl/r5/v0/sdl_r5_utils.h>

#ifdef UNITY_INCLUDE_CONFIG_H
#include <test/unity/src/unity.h>
#include <test/unity/config/unity_config.h>
#endif





/*===========================================================================*/
/*                         Declarations                                      */
/*===========================================================================*/

/* Define the test interface */
typedef struct sdlasmTest_s
{
    int32_t (*testFunction)(void);      /* The code that runs the test */
    char                *name;          /* The test name */
    int32_t              testStatus;    /* Test Status */
} sdlasmTest_t;

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
extern int32_t sdl_apiTest(void);

/*===========================================================================*/
/*                   Local Function definitions                              */
/*===========================================================================*/

#endif /* INCLUDE_TEST_UTILS_H_*/
/* Nothing past this point */
