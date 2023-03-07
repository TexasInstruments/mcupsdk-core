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

/* This file contains DCC API negative test code */

#include "dcc_test_main.h"

int32_t SDL_DCC_negTest()
{
    int32_t                   testStatus = SDL_APP_TEST_PASS, sdlRet;
    SDL_DCC_Config            config;
    SDL_DCC_StaticRegs        staticRegs;
    SDL_DCC_Status            status;
    SDL_DCC_Inst              validInstance,  invalidInstance;

    config.mode             = SDL_DCC_MODE_SINGLE_SHOT_1;
    config.clk0Src          = SDL_DCC_CLK0_SRC_CLOCK0_0;
    config.clk1Src          = SDL_DCC_CLK1_SRC_CLOCKSRC0;
    config.clk0Seed         = 0xffu;
    config.clk0ValidSeed    = 0xffu;
    config.clk1Seed         = 0xffu;

    validInstance    = SDL_DCC_INST_DCC0;
    invalidInstance = (SDL_DCC_Inst)SDL_DCC_TEST_INSTANCE_INVALID;

/*******************************************************************************************
 *     Call SDL API SDL_DCC_configure
 *******************************************************************************************/

    if (testStatus == SDL_APP_TEST_PASS)
    {
        sdlRet = SDL_DCC_configure(invalidInstance, NULL);

        if (sdlRet == SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("SDL_DCC_configure: failure on line no. %d \n", __LINE__);
        }
     }


    if (testStatus == SDL_APP_TEST_PASS)
    {
        sdlRet = SDL_DCC_configure(validInstance, NULL);

        if (sdlRet == SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("SDL_DCC_configure: failure on line no. %d \n", __LINE__);
        }
    }

    if (testStatus == SDL_APP_TEST_PASS)
    {

        sdlRet = SDL_DCC_configure(invalidInstance, &config);

        if (sdlRet == SDL_PASS)
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
        config.clk0ValidSeed    = 0xffu;
        /* Invalid clk1Seed */
        config.clk1Seed         = DCC_SRC1_COUNT_MAX + 1u;

        sdlRet = SDL_DCC_configure(validInstance, &config);

        if (sdlRet == SDL_PASS)
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
        /* Invalid clk0Seed */
        config.clk0Seed         = DCC_SRC0_COUNT_MAX + 1u;
        config.clk0ValidSeed    = 0xffu;
        config.clk1Seed         = 0xffu;

        sdlRet = SDL_DCC_configure(validInstance, &config);

        if (sdlRet == SDL_PASS)
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
        /* Invalid clk0ValidSeed */
        config.clk0ValidSeed    = DCC_SRC0_VALID_MAX + 1u;
        config.clk1Seed         = 0xffu;

        sdlRet = SDL_DCC_configure(validInstance, &config);

        if (sdlRet == SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("SDL_DCC_configure: failure on line no. %d \n", __LINE__);
        }
    }

    if (testStatus == SDL_APP_TEST_PASS)
    {
        /* Invalid mode */
        config.mode             = 0xFFFFFFFFu;
        config.clk0Src          = SDL_DCC_CLK0_SRC_CLOCK0_0;
        config.clk1Src          = SDL_DCC_CLK1_SRC_CLOCKSRC0;
        config.clk0Seed         = 0xffu;
        config.clk0ValidSeed    = 0xffu;
        config.clk1Seed         = 0xffu;

        sdlRet = SDL_DCC_configure(validInstance, &config);

        if (sdlRet == SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("SDL_DCC_configure: failure on line no. %d \n", __LINE__);
        }
    }

    if (testStatus == SDL_APP_TEST_PASS)
    {
        config.mode             = SDL_DCC_MODE_SINGLE_SHOT_1;
        /* Invalid clk0Src */
        config.clk0Src          = 0xFFFFu;
        config.clk1Src          = SDL_DCC_CLK1_SRC_CLOCKSRC0;
        config.clk0Seed         = 0xffu;
        config.clk0ValidSeed    = 0xffu;
        config.clk1Seed         = 0xffu;

        sdlRet = SDL_DCC_configure(validInstance, &config);

        if (sdlRet == SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("SDL_DCC_configure: failure on line no. %d \n", __LINE__);
        }
    }

    if (testStatus == SDL_APP_TEST_PASS)
    {
        config.mode             = SDL_DCC_MODE_SINGLE_SHOT_1;
        config.clk0Src          = SDL_DCC_CLK0_SRC_CLOCK0_0;
        /* Invalid clk1Src */
        config.clk1Src          = 0xFFFFu;
        config.clk0Seed         = 0xffu;
        config.clk0ValidSeed    = 0xffu;
        config.clk1Seed         = 0xffu;

        sdlRet = SDL_DCC_configure(validInstance, &config);

        if (sdlRet == SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("SDL_DCC_configure: failure on line no. %d \n", __LINE__);
        }
    }

/*******************************************************************************
 *     Call SDL API SDL_DCC_verifyConfig
 ******************************************************************************/

    if (testStatus == SDL_APP_TEST_PASS)
    {
        sdlRet = SDL_DCC_verifyConfig(invalidInstance, NULL);

        if (sdlRet == SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("SDL_DCC_verifyConfig: failure on line no. %d \n", __LINE__);
        }
    }

    if (testStatus == SDL_APP_TEST_PASS)
    {
        sdlRet = SDL_DCC_verifyConfig(validInstance, NULL);

        if (sdlRet == SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("SDL_DCC_verifyConfig: failure on line no. %d \n", __LINE__);
        }
    }

    if (testStatus == SDL_APP_TEST_PASS)
    {

        sdlRet = SDL_DCC_verifyConfig(invalidInstance, &config);

        if (sdlRet == SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("SDL_DCC_verifyConfig: failure on line no. %d \n", __LINE__);
        }
    }

    if (testStatus == SDL_APP_TEST_PASS)
    {
        config.mode             = SDL_DCC_MODE_SINGLE_SHOT_1;
        config.clk0Src          = SDL_DCC_CLK0_SRC_CLOCK0_0;
        config.clk1Src          = SDL_DCC_CLK1_SRC_CLOCKSRC0;
        config.clk0Seed         = 0xffu;
        config.clk0ValidSeed    = 0xffu;
        config.clk1Seed         = 0xffu;

        sdlRet = SDL_DCC_configure(validInstance, &config);

        if (sdlRet != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("SDL_DCC_configure: failure on line no %d \n", __LINE__);
        }

        config.mode             = 0x46u;
        config.clk0Src          = 0x46u;
        config.clk1Src          = 0x46u;
        config.clk0Seed         = DCC_SRC0_COUNT_MAX + 1u;
        config.clk0ValidSeed    = DCC_SRC0_VALID_MAX + 1u;
        config.clk1Seed         = DCC_SRC1_COUNT_MAX + 1u;

        sdlRet = SDL_DCC_verifyConfig(validInstance, &config);

        if (sdlRet == SDL_PASS)
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
        config.clk0Seed         = DCC_SRC0_COUNT_MAX + 1u;
        config.clk0ValidSeed    = DCC_SRC0_VALID_MAX + 1u;
        config.clk1Seed         = DCC_SRC1_COUNT_MAX + 1u;

        sdlRet = SDL_DCC_configure(validInstance, &config);
        if (sdlRet == SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("SDL_DCC_configure: failure on line no %d \n", __LINE__);
        }

        config.mode             = SDL_DCC_MODE_SINGLE_SHOT_1;
        config.clk0Src          = SDL_DCC_CLK0_SRC_CLOCK0_0;
        config.clk1Src          = SDL_DCC_CLK1_SRC_CLOCKSRC0;
        config.clk0Seed         = 46u;
        config.clk0ValidSeed    = 46u;
        config.clk1Seed         = 46u;

        sdlRet = SDL_DCC_verifyConfig(validInstance, &config);

        if (sdlRet == SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("SDL_DCC_verifyConfig: failure on line no. %d \n", __LINE__);
        }
    }

    if (testStatus == SDL_APP_TEST_PASS)
    {
        config.mode             = SDL_DCC_MODE_SINGLE_SHOT_1;
        config.clk0Src          = SDL_DCC_CLK0_SRC_CLOCK0_0;
        config.clk1Src          = SDL_DCC_CLK1_SRC_CLOCKSRC0;
        config.clk0Seed         = 0xffu;
        config.clk0ValidSeed    = DCC_SRC0_VALID_MAX + 1u;
        config.clk1Seed         = DCC_SRC1_COUNT_MAX + 1u;

        sdlRet = SDL_DCC_configure(validInstance, &config);
        if (sdlRet == SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("SDL_DCC_configure: failure on line no %d \n", __LINE__);
        }

        config.mode             = SDL_DCC_MODE_SINGLE_SHOT_1;
        config.clk0Src          = SDL_DCC_CLK0_SRC_CLOCK0_0;
        config.clk1Src          = SDL_DCC_CLK1_SRC_CLOCKSRC0;
        config.clk0Seed         = 0xffu;
        config.clk0ValidSeed    = 46u;
        config.clk1Seed         = 46u;

        sdlRet = SDL_DCC_verifyConfig(validInstance, &config);


        if (sdlRet == SDL_PASS)
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
        config.clk0ValidSeed    = 0xffu;
        config.clk1Seed         = DCC_SRC1_COUNT_MAX + 1u;

        sdlRet = SDL_DCC_configure(validInstance, &config);

        if (sdlRet == SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("SDL_DCC_configure: failure on line no %d \n", __LINE__);
        }

        config.mode             = SDL_DCC_MODE_SINGLE_SHOT_1;
        config.clk0Src          = SDL_DCC_CLK0_SRC_CLOCK0_0;
        config.clk1Src          = SDL_DCC_CLK1_SRC_CLOCKSRC0;
        config.clk0Seed         = 0xffu;
        config.clk0ValidSeed    = 0xffu;
        config.clk1Seed         = 46u;

        sdlRet = SDL_DCC_verifyConfig(validInstance, &config);


        if (sdlRet == SDL_PASS)
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
        config.clk0ValidSeed    = 0xffu;
        config.clk1Seed         = DCC_SRC1_COUNT_MAX + 1u;

        sdlRet = SDL_DCC_configure(validInstance, &config);

        if (sdlRet == SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("SDL_DCC_configure: failure on line no %d \n", __LINE__);
        }

        config.mode             = SDL_DCC_MODE_SINGLE_SHOT_1;
        config.clk0Src          = 46u;
        config.clk1Src          = SDL_DCC_CLK1_SRC_CLOCKSRC0;
        config.clk0Seed         = 0xffu;
        config.clk0ValidSeed    = 0xffu;
        config.clk1Seed         = 46u;

        sdlRet = SDL_DCC_verifyConfig(validInstance, &config);


        if (sdlRet == SDL_PASS)
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
        config.clk0ValidSeed    = 0xffu;
        config.clk1Seed         = DCC_SRC1_COUNT_MAX + 1u;

        sdlRet = SDL_DCC_configure(validInstance, &config);

        if (sdlRet == SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("SDL_DCC_configure: failure on line no %d \n", __LINE__);
        }

        config.mode             = SDL_DCC_MODE_SINGLE_SHOT_1;
        config.clk0Src          = SDL_DCC_CLK0_SRC_CLOCK0_0;
        config.clk1Src          = 46u;
        config.clk0Seed         = 0xffu;
        config.clk0ValidSeed    = 0xffu;
        config.clk1Seed         = 46u;

        sdlRet = SDL_DCC_verifyConfig(validInstance, &config);

        if (sdlRet == SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("SDL_DCC_configure: failure on line no. %d \n", __LINE__);
        }
    }

/*******************************************************************************
 *     Call SDL API SDL_DCC_enable
 ******************************************************************************/

     if (testStatus == SDL_APP_TEST_PASS)
     {
        sdlRet = SDL_DCC_enable(invalidInstance);

        if (sdlRet == SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("SDL_DCC_enable: failure on line no. %d \n", __LINE__);
        }
     }

/*******************************************************************************
 *     Call SDL API SDL_DCC_disable
 ******************************************************************************/

    if (testStatus == SDL_APP_TEST_PASS)
    {
        sdlRet = SDL_DCC_disable(invalidInstance);

        if (sdlRet == SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("\n  SDL_DCC_disable: Negative test failed on line no: %d \n", __LINE__);
        }
    }



/*******************************************************************************
 *     Call SDL API SDL_DCC_getStatus
 ******************************************************************************/

    if (testStatus == SDL_APP_TEST_PASS)
    {
        sdlRet = SDL_DCC_getStatus(invalidInstance, NULL);

        if (sdlRet == SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("\n  SDL_DCC_getStatus: Negative test failed on line no: %d \n", __LINE__);
        }
    }

    if (testStatus == SDL_APP_TEST_PASS)
    {
        sdlRet = SDL_DCC_getStatus(validInstance, NULL);

        if (sdlRet == SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("\n  SDL_DCC_getStatus: Negative test failed on line no: %d \n", __LINE__);
        }
    }

    if (testStatus == SDL_APP_TEST_PASS)
    {
        sdlRet = SDL_DCC_getStatus(invalidInstance, &status);

        if (sdlRet == SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("\n  SDL_DCC_getStatus: Negative test failed on line no: %d \n", __LINE__);
        }
    }



/*******************************************************************************
 *     Call SDL API SDL_DCC_enableIntr
 ******************************************************************************/

    if (testStatus == SDL_APP_TEST_PASS)
    {
        sdlRet = SDL_DCC_enableIntr(invalidInstance, SDL_DCC_TEST_INVLD_INTR_TYPE);

        if (sdlRet == SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("\n  SDL_DCC_enableIntr: Negative test failed on line no: %d \n", __LINE__);
        }
    }

    if (testStatus == SDL_APP_TEST_PASS)
    {
        sdlRet = SDL_DCC_enableIntr(validInstance, SDL_DCC_TEST_INVLD_INTR_TYPE);

        if (sdlRet == SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("\n  SDL_DCC_enableIntr: Negative test failed on line no: %d \n", __LINE__);
        }
    }

    if (testStatus == SDL_APP_TEST_PASS)
    {
        sdlRet = SDL_DCC_enableIntr(invalidInstance, SDL_DCC_INTERRUPT_ERR);

        if (sdlRet == SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("\n  SDL_DCC_enableIntr: Negative test failed on line no: %d \n", __LINE__);
        }
    }

    if (testStatus == SDL_APP_TEST_PASS)
    {
        sdlRet = SDL_DCC_enableIntr(invalidInstance, SDL_DCC_INTERRUPT_DONE);

        if (sdlRet == SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("\n  SDL_DCC_enableIntr: Negative test failed on line no: %d \n", __LINE__);
        }
    }



/*******************************************************************************
 *     Call SDL API SDL_DCC_clearIntr
 ******************************************************************************/

    if (testStatus == SDL_APP_TEST_PASS)
    {
        sdlRet = SDL_DCC_clearIntr(invalidInstance, SDL_DCC_TEST_INVLD_INTR_TYPE);

        if (sdlRet == SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("\n  SDL_DCC_clearIntr: Negative test failed on line no: %d \n", __LINE__);
        }
    }

    if (testStatus == SDL_APP_TEST_PASS)
    {
        sdlRet = SDL_DCC_clearIntr(invalidInstance, SDL_DCC_INTERRUPT_ERR);

        if (sdlRet == SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("\n  SDL_DCC_clearIntr: Negative test failed on line no: %d \n", __LINE__);
        }
    }

    if (testStatus == SDL_APP_TEST_PASS)
    {
        sdlRet = SDL_DCC_clearIntr(invalidInstance, SDL_DCC_INTERRUPT_DONE);

        if (sdlRet == SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("\n  SDL_DCC_clearIntr: Negative test failed on line no: %d \n", __LINE__);
        }
    }

/*******************************************************************************
 *     Call SDL API SDL_DCC_disableIntr
 ******************************************************************************/

    if (testStatus == SDL_APP_TEST_PASS)
    {
        sdlRet = SDL_DCC_disableIntr(invalidInstance, SDL_DCC_TEST_INVLD_INTR_TYPE);

        if (sdlRet == SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("\n  SDL_DCC_disableIntr: Negative test failed on line no: %d \n", __LINE__);
        }
    }

    if (testStatus == SDL_APP_TEST_PASS)
    {
        sdlRet = SDL_DCC_disableIntr(validInstance, SDL_DCC_TEST_INVLD_INTR_TYPE);

        if (sdlRet == SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("\n  SDL_DCC_disableIntr: Negative test failed on line no: %d \n", __LINE__);
        }
    }

    if (testStatus == SDL_APP_TEST_PASS)
    {
        sdlRet = SDL_DCC_disableIntr(invalidInstance, SDL_DCC_INTERRUPT_ERR);

        if (sdlRet == SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("\n  SDL_DCC_disableIntr: Negative test failed on line no: %d \n", __LINE__);
        }
    }

    if (testStatus == SDL_APP_TEST_PASS)
    {
        sdlRet = SDL_DCC_disableIntr(invalidInstance, SDL_DCC_INTERRUPT_DONE);

        if (sdlRet == SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("\n  SDL_DCC_disableIntr: Negative test failed on line no: %d \n", __LINE__);
        }
    }

/*******************************************************************************
 *     Call SDL API SDL_DCC_getBaseaddr
 ******************************************************************************/
	 if (testStatus == SDL_APP_TEST_PASS)
    {
        sdlRet = SDL_DCC_getBaseaddr(invalidInstance, NULL);

        if (sdlRet == SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("\n  SDL_DCC_getBaseaddr: Negative test failed on line no: %d \n", __LINE__);
        }
    }

    if (testStatus == SDL_APP_TEST_PASS)
    {
        sdlRet = SDL_DCC_getBaseaddr(validInstance, NULL);

        if (sdlRet == SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("\n  SDL_DCC_getBaseaddr: Negative test failed on line no: %d \n", __LINE__);
        }
    }

/*******************************************************************************
 *     Call SDL API SDL_DCC_getStaticRegs
 ******************************************************************************/

    if (testStatus == SDL_APP_TEST_PASS)
    {
        sdlRet = SDL_DCC_getStaticRegs(invalidInstance, NULL);

        if (sdlRet == SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("\n  SDL_DCC_getStaticRegs: Negative test failed on line no: %d \n", __LINE__);
        }
    }

    if (testStatus == SDL_APP_TEST_PASS)
    {
        sdlRet = SDL_DCC_getStaticRegs(validInstance, NULL);

        if (sdlRet == SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("\n  SDL_DCC_getStaticRegs: Negative test failed on line no: %d \n", __LINE__);
        }
    }

    if (testStatus == SDL_APP_TEST_PASS)
    {
        sdlRet = SDL_DCC_getStaticRegs(invalidInstance, &staticRegs);

        if (sdlRet == SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("\n  SDL_DCC_getStaticRegs: Negative test failed on line no: %d \n", __LINE__);
        }
    }

    /* Returning the test status  */
    return (testStatus);

}