/*
 *    Copyright (c) 2021 Texas Instruments Incorporated
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

/* This file contains DCC API positive test code */


#include "dcc_test_main.h"
#include <kernel/dpl/DebugP.h>
#include <sdl/sdl_dcc.h>


int32_t SDL_DCC_posTest(void)
{
    uint32_t      	    i;
    int32_t     		testStatus = SDL_APP_TEST_PASS, sdlRet;
    SDL_DCC_config      config;
    SDL_DCC_staticRegs  staticRegs;
	SDL_DCC_Status		status;
	SDL_DCC_Inst 		validInstance = DCC_INST_NUM;

    config.mode 			= SDL_DCC_MODE_SINGLE_SHOT;
    config.clk0Src 		    = SDL_DCC_CLK0_SRC_CLOCK0_0;
    config.clk1Src  	 	= SDL_DCC_CLK1_SRC_CLOCKSRC0;
	config.clk0Seed 		= 0xffu;
	config.clk0ValidSeed	= 0xffu;
	config.clk1Seed 		= 0xffu;



/*******************************************************************************************
*     Call SDL API SDL_DCC_configure
*******************************************************************************************/

    if (testStatus == SDL_APP_TEST_PASS)
    {
        for(i= DCC_INST_NUM; i <SDL_DCC_INVALID_INSTANCE; i++)
        {
            sdlRet = SDL_DCC_configure((SDL_DCC_Inst)i, &config);

            if (sdlRet != SDL_PASS)
            {
                testStatus = SDL_APP_TEST_FAILED;
                DebugP_log("SDL_DCC_configure: failure on line no. %d \n", __LINE__);
            }
        }
    }



/*******************************************************************************************
*     Call SDL API SDL_DCC_verifyConfig
*******************************************************************************************/

    if (testStatus == SDL_APP_TEST_PASS)
	{
        for(i= DCC_INST_NUM; i <SDL_DCC_INVALID_INSTANCE; i++)
        {
            sdlRet = SDL_DCC_verifyConfig((SDL_DCC_Inst)i, &config);

            if (sdlRet != SDL_PASS)
            {
                testStatus = SDL_APP_TEST_FAILED;
                DebugP_log("SDL_DCC_verifyConfig: failure on line no. %d \n", __LINE__);
            }
        }
     }

	if (testStatus == SDL_APP_TEST_PASS)
    {
		config.mode 			= SDL_DCC_MODE_SINGLE_SHOT;
		config.clk0Src 		    = SDL_DCC_CLK0_SRC_CLOCK0_0;
		config.clk1Src  	 	= SDL_DCC_CLK1_SRC_CLOCKSRC0;
		config.clk0Seed 		= 0xffu;
		config.clk0ValidSeed	= 3u;
		config.clk1Seed 		= 0xffu;

        sdlRet = SDL_DCC_configure(validInstance, &config);


        if (sdlRet != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("SDL_DCC_configure: failure on line no. %d \n", __LINE__);
        }
    }

	if (testStatus == SDL_APP_TEST_PASS)
    {
		config.mode 			= SDL_DCC_MODE_SINGLE_SHOT;
		config.clk0Src 		    = SDL_DCC_CLK0_SRC_CLOCK0_0;
		config.clk1Src  	 	= SDL_DCC_CLK1_SRC_CLOCKSRC0;
		config.clk0Seed 		= 0xffu;
		config.clk0ValidSeed	= 4u;
		config.clk1Seed 		= 0xffu;

        sdlRet = SDL_DCC_verifyConfig(validInstance, &config);


        if (sdlRet != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("SDL_DCC_verifyConfig: failure on line no. %d \n", __LINE__);
        }
    }



/*******************************************************************************************
*     Call SDL API SDL_DCC_enable
*******************************************************************************************/

     if (testStatus == SDL_APP_TEST_PASS)
     {
        for(i= DCC_INST_NUM; i <SDL_DCC_INVALID_INSTANCE; i++)
        {
            sdlRet = SDL_DCC_enable((SDL_DCC_Inst)i);

            if (sdlRet != SDL_PASS)
            {
                testStatus = SDL_APP_TEST_FAILED;
                DebugP_log("SDL_DCC_enable: failure on line no. %d \n", __LINE__);
            }
        }
     }



/*******************************************************************************************
*     Call SDL API SDL_DCC_disable
*******************************************************************************************/

    if (testStatus == SDL_APP_TEST_PASS)
    {
        for(i= DCC_INST_NUM; i <SDL_DCC_INVALID_INSTANCE; i++)
        {
            sdlRet = SDL_DCC_disable((SDL_DCC_Inst)i);

            if (sdlRet != SDL_PASS)
            {
                testStatus = SDL_APP_TEST_FAILED;
                DebugP_log("\n  SDL_DCC_disable: positive test failed on line no: %d \n", __LINE__);
            }
        }
    }



/*******************************************************************************************
*     Call SDL API SDL_DCC_getStatus
*******************************************************************************************/

    if (testStatus == SDL_APP_TEST_PASS)
    {
        for(i= DCC_INST_NUM; i <SDL_DCC_INVALID_INSTANCE; i++)
        {
            sdlRet = SDL_DCC_getStatus((SDL_DCC_Inst)i, &status);

            if (sdlRet != SDL_PASS)
            {
                testStatus = SDL_APP_TEST_FAILED;
                DebugP_log("\n  SDL_DCC_getStatus: positive test failed on line no: %d \n", __LINE__);
            }
        }
    }



/*******************************************************************************************
*     Call SDL API SDL_DCC_enableIntr
*******************************************************************************************/

    if (testStatus == SDL_APP_TEST_PASS)
    {
        for(i= DCC_INST_NUM; i <SDL_DCC_INVALID_INSTANCE; i++)
        {
            sdlRet = SDL_DCC_enableIntr((SDL_DCC_Inst)i, SDL_DCC_INTERRUPT_DONE);

            if (sdlRet != SDL_PASS)
            {
                testStatus = SDL_APP_TEST_FAILED;
                DebugP_log("\n  SDL_DCC_enableIntr: positive test failed on line no: %d \n", __LINE__);
            }
        }
    }

	if (testStatus == SDL_APP_TEST_PASS)
    {
        for(i= DCC_INST_NUM; i <SDL_DCC_INVALID_INSTANCE; i++)
        {
            sdlRet = SDL_DCC_enableIntr((SDL_DCC_Inst)i, SDL_DCC_INTERRUPT_ERR);

            if (sdlRet != SDL_PASS)
            {
                testStatus = SDL_APP_TEST_FAILED;
                DebugP_log("\n  SDL_DCC_enableIntr: positive test failed on line no: %d \n", __LINE__);
            }
        }
    }



/*******************************************************************************************
*     Call SDL API SDL_DCC_clearIntr
*******************************************************************************************/

    if (testStatus == SDL_APP_TEST_PASS)
    {
        for(i= DCC_INST_NUM; i <SDL_DCC_INVALID_INSTANCE; i++)
        {
            sdlRet = SDL_DCC_clearIntr((SDL_DCC_Inst)i, SDL_DCC_INTERRUPT_DONE);

            if (sdlRet != SDL_PASS)
            {
                testStatus = SDL_APP_TEST_FAILED;
                DebugP_log("\n  SDL_DCC_clearIntr: positive test failed on line no: %d \n", __LINE__);
            }
        }
    }

	if (testStatus == SDL_APP_TEST_PASS)
    {
        for(i= DCC_INST_NUM; i <SDL_DCC_INVALID_INSTANCE; i++)
        {
            sdlRet = SDL_DCC_clearIntr((SDL_DCC_Inst)i, SDL_DCC_INTERRUPT_ERR);

            if (sdlRet != SDL_PASS)
            {
                testStatus = SDL_APP_TEST_FAILED;
                DebugP_log("\n  SDL_DCC_clearIntr: positive test failed on line no: %d \n", __LINE__);
            }
        }
    }



/*******************************************************************************************
*     Call SDL API SDL_DCC_getStaticRegs
*******************************************************************************************/

    if (testStatus == SDL_APP_TEST_PASS)
    {
        for(i= DCC_INST_NUM; i <SDL_DCC_INVALID_INSTANCE; i++)
        {
            sdlRet = SDL_DCC_getStaticRegs((SDL_DCC_Inst)i, &staticRegs);

            if (sdlRet != SDL_PASS)
            {
                testStatus = SDL_APP_TEST_FAILED;
                DebugP_log("\n  SDL_DCC_getStaticRegs: positive test failed on line no: %d \n", __LINE__);
            }
        }
    }


return (testStatus);
}
