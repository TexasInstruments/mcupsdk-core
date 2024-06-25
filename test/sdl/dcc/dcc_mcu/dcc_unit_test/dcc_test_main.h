/*
 *  Copyright (c) 2021 Texas Instruments Incorporated
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
 *  \file     dcc_test_main.h
 *
 *  \brief    This file contains DCC UNIT test code declarations.
 *
 **/

/*===========================================================================*/
/*                         Include files                                     */
/*===========================================================================*/
/*===========================================================================*/


#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include <sdl/sdl_dcc.h>
#include <sdl/include/sdl_types.h>
#include <sdl/include/hw_types.h>
#include <dpl_interface.h>

#include "ti_drivers_config.h"
#include "ti_drivers_open_close.h"
#include "ti_board_open_close.h"
#include "ti_board_config.h"
#include "ti_dpl_config.h"


#if !defined(SDL_DCC_TEST_H)
#define SDL_DCC_TEST_H


/*===========================================================================*/
/*                    Dependant macros in sdl_dcc_negTest.c                  */
/*===========================================================================*/
#define  SDL_DCC_INSTANCE_INVLD1  (46u)
#define INVLD_INTR_TYPE			 (46u)


/*===========================================================================*/
/*                         Declarations                                      */
/*===========================================================================*/

/* Define the test interface */
typedef struct sdlDccTest_s
{
    int32_t  (*testFunction)(void);   /* The code that runs the test */
    char      *name;                  /* The test name */
    int32_t    testStatus;            /* Test Status */
} sdlDccTest_t;



/*===========================================================================*/
/*                         Macros                                            */
/*===========================================================================*/

#define SDL_APP_TEST_NOT_RUN        (-(int32_t) (2))
#define SDL_APP_TEST_FAILED         (-(int32_t) (1))
#define SDL_APP_TEST_PASS           ( (int32_t) (0))



/*===========================================================================*/
/*                         Internal function declarations                    */
/*===========================================================================*/

static int32_t sdlApp_dplInit(void);



/*===========================================================================*/
/*                         External function declarations                    */
/*===========================================================================*/

extern int32_t SDL_DCC_posTest(void);
extern int32_t SDL_DCC_negTest(void);



/*===========================================================================*/
/*                          SOC Specific Macros                     */
/*===========================================================================*/
#if defined (SOC_AM263X) || defined (SOC_AM263PX) || defined (SOC_AM261X)
#define DCC_INST_NUM     (SDL_DCC_INST_MSS_DCCA)
#endif
#if defined (SOC_AWR294X)
#if defined (R5F_INPUTS)
#define DCC_INST_NUM     (SDL_DCC_INST_MSS_DCCA)
#endif
#if defined (C66_INPUTS)
#define DCC_INST_NUM     (SDL_DCC_INST_DSS_DCCA)
#endif
#endif


#endif /* SDL_DCC_TEST_H */
/* Nothing past this point */
