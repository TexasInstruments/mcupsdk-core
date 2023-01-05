/*
 *    Copyright (c) 2023 Texas Instruments Incorporated
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

/* This file contains RESET API test code */


#include "reset_test_main.h"
#include <kernel/dpl/DebugP.h>
#include <sdl/sdl_reset.h>
#include <sdl/include/hw_types.h>

int32_t SDL_Reset_Test(void)
{
    int32_t   testStatus = SDL_APP_TEST_PASS;
    uint32_t  sdlRet;

/*******************************************************************************************
*     Call SDL API SDL_r5fGetResetCause
*******************************************************************************************/

    /*This will return either zero or one */
    if (testStatus == SDL_APP_TEST_PASS)
    {
        sdlRet = SDL_r5fGetResetCause();

        if (sdlRet == INVAILD_RETURN)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("SDL_r5fGetResetCause: failure on line no. %d \r\n", __LINE__);
            return (testStatus);
        }
    }

/*******************************************************************************************
*     Call SDL API  SDL_rcmDspLocalReset
*******************************************************************************************/

    /*This will return nothing */
    if (testStatus == SDL_APP_TEST_PASS)
    {
       SDL_rcmDspLocalReset();
    }

/*******************************************************************************************
*     Call SDL API SDL_getWarmResetCause
*******************************************************************************************/

    /*This will return SDL_PASS(0), When all the reset cause is clear, and register value was zero, never return negative value */
    if (testStatus == SDL_APP_TEST_PASS)
    {
        sdlRet = SDL_getWarmResetCause();

        if (sdlRet == INVAILD_RETURN)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("SDL_getWarmResetCause: failure on line no. %d \r\n", __LINE__);
            return (testStatus);
        }

    }


/*******************************************************************************************
*     Call SDL API SDL_generateSwWarmReset
*******************************************************************************************/
 /*This will return nothing */


    if(sdlRet!=SDL_WarmResetCause_TOP_RCM_WARM_RESET_CONFIG)
    {
        if (testStatus == SDL_APP_TEST_PASS)
        {
            /* After running this API, S0C will be reset, for checki*/
            SDL_generateSwWarmReset();

        }
    }

    return (testStatus);

}
