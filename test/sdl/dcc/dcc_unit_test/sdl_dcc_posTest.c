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

SDL_DCC_ClkSrc0 gDCCTestClk0Srcs[SDL_DCC_CLK0_SRC_NUM] =
{
    SDL_DCC_CLK0_SRC_CLOCK0_0,
    SDL_DCC_CLK0_SRC_CLOCK0_1,
    SDL_DCC_CLK0_SRC_CLOCK0_2
};

SDL_DCC_ClkSrc1 gDCCTestClk1Srcs[SDL_DCC_CLK1_SRC_NUM] =
{
    SDL_DCC_CLK1_SRC_CLOCK1,
    SDL_DCC_CLK1_SRC_CLOCKSRC0,
    SDL_DCC_CLK1_SRC_CLOCKSRC1,
    SDL_DCC_CLK1_SRC_CLOCKSRC2,
    SDL_DCC_CLK1_SRC_CLOCKSRC3,
    SDL_DCC_CLK1_SRC_CLOCKSRC4,
    SDL_DCC_CLK1_SRC_CLOCKSRC5,
    SDL_DCC_CLK1_SRC_CLOCKSRC6,
    SDL_DCC_CLK1_SRC_CLOCKSRC7,
    SDL_DCC_CLK1_SRC_FICLK
};

SDL_DCC_Mode gDCCTestModes[SDL_DCC_MODES_NUM] =
{
    SDL_DCC_MODE_SINGLE_SHOT_1,
    SDL_DCC_MODE_SINGLE_SHOT_2,
    SDL_DCC_MODE_CONTINUOUS
};

int32_t SDL_DCC_posTest(void)
{
    uint32_t            i, j, k, l;
    int32_t             testStatus = SDL_APP_TEST_PASS, sdlRet;
    SDL_DCC_Config      config;
    SDL_DCC_StaticRegs  staticRegs;
    SDL_DCC_Status      status;
    SDL_DCC_Inst        validInstance = SDL_DCC_INST_DCC0;

    config.mode             = SDL_DCC_MODE_SINGLE_SHOT_1;
    config.clk0Src          = SDL_DCC_CLK0_SRC_CLOCK0_0;
    config.clk1Src          = SDL_DCC_CLK1_SRC_CLOCKSRC0;
    config.clk0Seed         = 0xffu;
    config.clk0ValidSeed    = 0xffu;
    config.clk1Seed         = 0xffu;


/*******************************************************************************
 *     Call SDL API SDL_DCC_configure
 ******************************************************************************/

    if (testStatus == SDL_APP_TEST_PASS)
    {
        for(i = 0x0u; i < SDL_DCC_INVALID_INSTANCE; i++)
        {
            /* cycle through the valid clk0Src */
            for (j = 0; j < SDL_DCC_CLK0_SRC_NUM; j++)
            {

                /* cycle through the valid clk1src */
                for (k = 0; k < SDL_DCC_CLK1_SRC_NUM; k++)
                {
                    for (l = 0; l < SDL_DCC_MODES_NUM; l++)
                    {
                        config.clk0Src = gDCCTestClk0Srcs[j];
                        config.clk1Src = gDCCTestClk1Srcs[k];
                        config.mode = gDCCTestModes[l];
                        sdlRet = SDL_DCC_configure((SDL_DCC_Inst)i, &config);

                        if (sdlRet != SDL_PASS)
                        {
                            testStatus = SDL_APP_TEST_FAILED;
                            DebugP_log("SDL_DCC_configure: failure on line no. %d [i %d j %d k %d l %d ]\n", __LINE__, i, j, k, l);
                            break;
                        }
                    }
                }
            }
        }
    }



/*******************************************************************************
 *     Call SDL API SDL_DCC_verifyConfig
 ******************************************************************************/

    if (testStatus == SDL_APP_TEST_PASS)
    {
        for(i = 0x0u; i < SDL_DCC_INVALID_INSTANCE; i++)
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
        config.mode             = SDL_DCC_MODE_SINGLE_SHOT_1;
        config.clk0Src          = SDL_DCC_CLK0_SRC_CLOCK0_0;
        config.clk1Src          = SDL_DCC_CLK1_SRC_CLOCKSRC0;
        config.clk0Seed         = 0xffu;
        config.clk0ValidSeed    = 3u;
        config.clk1Seed         = 0xffu;

        sdlRet = SDL_DCC_configure(validInstance, &config);

        if (sdlRet != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("SDL_DCC_configure: failure on line no. %d \n", __LINE__);
        }
    }

    if (testStatus == SDL_APP_TEST_PASS)
    {
        config.mode             = SDL_DCC_MODE_SINGLE_SHOT_1;
        config.clk0Src          = SDL_DCC_CLK0_SRC_CLOCK0_0;
        config.clk1Src          = SDL_DCC_CLK1_SRC_CLOCKSRC0;
        config.clk0Seed         = 0xffu;
        config.clk0ValidSeed    = 4u;
        config.clk1Seed         = 0xffu;

        sdlRet = SDL_DCC_verifyConfig(validInstance, &config);

        if (sdlRet != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("SDL_DCC_verifyConfig: failure on line no. %d \n", __LINE__);
        }
    }



/*******************************************************************************
 *     Call SDL API SDL_DCC_enable
 ******************************************************************************/

     if (testStatus == SDL_APP_TEST_PASS)
     {
        for(i = 0x0u; i < SDL_DCC_INVALID_INSTANCE; i++)
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
        for(i = 0x0u; i < SDL_DCC_INVALID_INSTANCE; i++)
        {
            sdlRet = SDL_DCC_disable((SDL_DCC_Inst)i);

            if (sdlRet != SDL_PASS)
            {
                testStatus = SDL_APP_TEST_FAILED;
                DebugP_log("\n  SDL_DCC_disable: positive test failed on line no: %d \n", __LINE__);
            }
        }
    }



/*******************************************************************************
 *     Call SDL API SDL_DCC_getStatus
 ******************************************************************************/

    if (testStatus == SDL_APP_TEST_PASS)
    {
        for(i= 0x0u; i < SDL_DCC_INVALID_INSTANCE; i++)
        {
            sdlRet = SDL_DCC_getStatus((SDL_DCC_Inst)i, &status);

            if (sdlRet != SDL_PASS)
            {
                testStatus = SDL_APP_TEST_FAILED;
                DebugP_log("\n  SDL_DCC_getStatus: positive test failed on line no: %d \n", __LINE__);
            }
        }
    }



/*******************************************************************************
 *     Call SDL API SDL_DCC_enableIntr
 ******************************************************************************/

    if (testStatus == SDL_APP_TEST_PASS)
    {
        for(i = 0x0u; i < SDL_DCC_INVALID_INSTANCE; i++)
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
        for(i = 0x0u; i < SDL_DCC_INVALID_INSTANCE; i++)
        {
            sdlRet = SDL_DCC_enableIntr((SDL_DCC_Inst)i, SDL_DCC_INTERRUPT_ERR);

            if (sdlRet != SDL_PASS)
            {
                testStatus = SDL_APP_TEST_FAILED;
                DebugP_log("\n  SDL_DCC_enableIntr: positive test failed on line no: %d \n", __LINE__);
            }
        }
    }

/*******************************************************************************
 *     Call SDL API SDL_DCC_disableIntr
 ******************************************************************************/	
    if (testStatus == SDL_APP_TEST_PASS)
    {
        for(i = 0x0u; i < SDL_DCC_INVALID_INSTANCE; i++)
        {
            sdlRet = SDL_DCC_disableIntr((SDL_DCC_Inst)i, SDL_DCC_INTERRUPT_DONE);

            if (sdlRet != SDL_PASS)
            {
                testStatus = SDL_APP_TEST_FAILED;
                DebugP_log("\n  SDL_DCC_disableIntr: positive test failed on line no: %d \n", __LINE__);
            }
        }
    }

    if (testStatus == SDL_APP_TEST_PASS)
    {
        for(i = 0x0u; i < SDL_DCC_INVALID_INSTANCE; i++)
        {
            sdlRet = SDL_DCC_disableIntr((SDL_DCC_Inst)i, SDL_DCC_INTERRUPT_ERR);

            if (sdlRet != SDL_PASS)
            {
                testStatus = SDL_APP_TEST_FAILED;
                DebugP_log("\n  SDL_DCC_disableIntr: positive test failed on line no: %d \n", __LINE__);
            }
        }
    }
/*******************************************************************************
 *     Call SDL API SDL_DCC_getBaseaddr
 ******************************************************************************/
	if (testStatus == SDL_APP_TEST_PASS)
    {   
		uint32_t baseAddr; 
        for(i= 0x0u; i < SDL_DCC_INVALID_INSTANCE; i++)
        {
            sdlRet = SDL_DCC_getBaseaddr((SDL_DCC_Inst)i, &baseAddr);

            if (sdlRet != SDL_PASS)
            {
                testStatus = SDL_APP_TEST_FAILED;
                DebugP_log("\n  SDL_DCC_getBaseaddr: positive test failed on line no: %d \n", __LINE__);
            }
        }
    }

/*******************************************************************************
 *     Call SDL API SDL_DCC_clearIntr
 ******************************************************************************/

    if (testStatus == SDL_APP_TEST_PASS)
    {
        for(i = 0x0u; i < SDL_DCC_INVALID_INSTANCE; i++)
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
        for(i = 0x0u; i < SDL_DCC_INVALID_INSTANCE; i++)
        {
            sdlRet = SDL_DCC_clearIntr((SDL_DCC_Inst)i, SDL_DCC_INTERRUPT_ERR);

            if (sdlRet != SDL_PASS)
            {
                testStatus = SDL_APP_TEST_FAILED;
                DebugP_log("\n  SDL_DCC_clearIntr: positive test failed on line no: %d \n", __LINE__);
            }
        }
    }


/*******************************************************************************
 *     Call SDL API SDL_DCC_disableIntr
 ******************************************************************************/

    if (testStatus == SDL_APP_TEST_PASS)
    {
        for(i = 0x0u; i < SDL_DCC_INVALID_INSTANCE; i++)
        {
            sdlRet = SDL_DCC_disableIntr((SDL_DCC_Inst)i, SDL_DCC_INTERRUPT_DONE);

            if (sdlRet != SDL_PASS)
            {
                testStatus = SDL_APP_TEST_FAILED;
                DebugP_log("\n  SDL_DCC_disableIntr: positive test failed on line no: %d \n", __LINE__);
            }
        }
    }

    if (testStatus == SDL_APP_TEST_PASS)
    {
        for(i = 0x0u; i < SDL_DCC_INVALID_INSTANCE; i++)
        {
            sdlRet = SDL_DCC_disableIntr((SDL_DCC_Inst)i, SDL_DCC_INTERRUPT_ERR);

            if (sdlRet != SDL_PASS)
            {
                testStatus = SDL_APP_TEST_FAILED;
                DebugP_log("\n  SDL_DCC_disableIntr: positive test failed on line no: %d \n", __LINE__);
            }
        }
    }


/*******************************************************************************
 *     Call SDL API SDL_DCC_getStaticRegs
 ******************************************************************************/

    if (testStatus == SDL_APP_TEST_PASS)
    {
        for(i = 0x0u; i < SDL_DCC_INVALID_INSTANCE; i++)
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