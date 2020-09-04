/* Copyright (c) 2022 Texas Instruments Incorporated
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
 *  \file     mcrc_main.c
 *
 *  \brief    This file contains mcrc function test code.
 *
 *  \details  MCRC tests
 **/

/*===========================================================================*/
/*                         Include files                                     */
/*===========================================================================*/
#include "main.h"
#include <dpl_interface.h>
#include <kernel/dpl/CacheP.h>

/*===========================================================================*/
/*                         Declarations                                      */
/*===========================================================================*/
/* None */

/*===========================================================================*/
/*                         Macros                                            */
/*===========================================================================*/
/* None */

/*===========================================================================*/
/*                         Internal function declarations                    */
/*===========================================================================*/

void sdl_mcrc_semicpu_test_app (void *args);

/*===========================================================================*/
/*                         Global Variables                                  */
/*===========================================================================*/
sdlMCRCApp_t  sdlmcrcAppList[] = {
	{mcrcSemiCPU_main,      "MCRC_semiCPU_mode",     	 SDL_APP_NOT_RUN },
    {NULL,                  "TERMINATING CONDITION",     SDL_APP_NOT_RUN }
};

/*===========================================================================*/
/*                   Local Function definitions                              */
/*===========================================================================*/

static int32_t sdlApp_dplInit(void)
{
    SDL_ErrType_t ret = SDL_PASS;

    ret = SDL_TEST_dplInit();
    if (ret != SDL_PASS)
    {
        DebugP_log("Error: Init Failed\n");
    }

    return ret;
}

/*===========================================================================*/
/*                         Function definitions                              */
/*===========================================================================*/

void sdl_mcrc_semicpu_test_app (void *args)
{
    /* Declarations of variables */
    int32_t    result = SDL_APP_PASS;
    int32_t    i;

    /* Init Dpl */
    sdlApp_dplInit();    
	
    DebugP_log("\n MCRC Test Application\r\n");

    for ( i = 0; sdlmcrcAppList[i].application != NULL; i++)
    {
        result = sdlmcrcAppList[i].application();
        sdlmcrcAppList[i].status = result;
    }
    result = SDL_APP_PASS;
    for ( i = 0; sdlmcrcAppList[i].application != NULL; i++)
    {
        if (sdlmcrcAppList[i].status != SDL_APP_PASS)
        {
            DebugP_log("\nTest Name: %s  FAILED \n", sdlmcrcAppList[i].name);
            result = SDL_APP_FAILED;
            break;
        }
        else
        {
            DebugP_log("\nTest Name: %s  PASSED \n", sdlmcrcAppList[i].name);
        }
    }

    if (result == SDL_APP_PASS)
    {
        DebugP_log("\n All tests have passed. \n");
    }
    else
    {
        DebugP_log("\n Few/all tests Failed \n");
    }
}

/* Nothing past this point */
