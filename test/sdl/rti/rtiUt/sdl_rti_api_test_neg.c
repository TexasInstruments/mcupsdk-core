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

/* This file contains RTI API negative test code */

#include "rti_main.h"

int32_t SDL_RTI_negTest()
{
    uint32_t          status, baseAddr;
    int32_t         testStatus = SDL_APP_TEST_PASS, sdlRet;

    SDL_RTI_staticRegs        pStaticRegs;
    SDL_RTI_InstanceType   validInstance = SDL_INSTANCE_RTI;
    SDL_RTI_configParms       pConfig;

    pConfig.SDL_RTI_dwwdPreloadVal = RTI_RTIDWDPRLD_INVALID;
    pConfig.SDL_RTI_dwwdWindowSize = RTI_RTIDWWDSIZECTRL_DWWDSIZE_INVALID;
    pConfig.SDL_RTI_dwwdReaction   = RTI_DWWD_REACTION_INVALID;


/******************************************************************************************
* Call SDL_RTI_getBaseaddr
*******************************************************************************************/

	if (testStatus == SDL_APP_TEST_PASS)
	{
		sdlRet = SDL_RTI_getBaseaddr(SDL_INSTANCE_RTI, NULL);
		if (sdlRet != SDL_EBADARGS)
		{
			testStatus = SDL_APP_TEST_FAILED;
			DebugP_log("SDL_RTI_getBaseaddr: failure on line no. %d \r\n", __LINE__);
		}
	}
/*******************************************************************************************
*     Call SDL API SDL_RTI_config
*******************************************************************************************/

    if (testStatus == SDL_APP_TEST_PASS)
     {
        sdlRet = SDL_RTI_config(SDL_INSTANCE_INVALID, NULL);


        if (sdlRet == SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("SDL_RTI_config: failure on line no. %d \r\n", __LINE__);
        }
     }


    if (testStatus == SDL_APP_TEST_PASS)
     {
        sdlRet = SDL_RTI_config(validInstance, NULL);

        if (sdlRet == SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("SDL_RTI_config: failure on line no. %d \r\n", __LINE__);
        }
     }

    if (testStatus == SDL_APP_TEST_PASS)
     {

        sdlRet = SDL_RTI_config(SDL_INSTANCE_INVALID, &pConfig);


        if (sdlRet == SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("SDL_RTI_config: failure on line no. %d \r\n", __LINE__);
        }
     }

     if (testStatus == SDL_APP_TEST_PASS)
     {
        sdlRet = SDL_RTI_config(validInstance, &pConfig);


        if (sdlRet == SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("SDL_RTI_config: failure on line no. %d \r\n", __LINE__);
        }
     }



/*******************************************************************************************
*     Call SDL API SDL_RTI_verifyConfig
*******************************************************************************************/

    if (testStatus == SDL_APP_TEST_PASS)
     {
        sdlRet = SDL_RTI_verifyConfig(SDL_INSTANCE_INVALID, NULL);

        if (sdlRet == SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("SDL_RTI_verifyConfig: failure on line no. %d \r\n", __LINE__);
        }
     }

     if (testStatus == SDL_APP_TEST_PASS)
     {
        sdlRet = SDL_RTI_verifyConfig(validInstance, NULL);

        if (sdlRet == SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("SDL_RTI_verifyConfig: failure on line no. %d \r\n", __LINE__);
        }
     }

     if (testStatus == SDL_APP_TEST_PASS)
     {

        sdlRet = SDL_RTI_verifyConfig(SDL_INSTANCE_INVALID, &pConfig);


        if (sdlRet == SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
            printf("SDL_RTI_verifyConfig: failure on line no. %d \r\n", __LINE__);
        }
     }



/*******************************************************************************************
*     Coverage test case for SDL_RTI_verifyConfig
*******************************************************************************************/

    if (testStatus == SDL_APP_TEST_PASS)
     {
        pConfig.SDL_RTI_dwwdPreloadVal = SDL_RTI_DWWDPRLD_VLD;
        pConfig.SDL_RTI_dwwdWindowSize = RTI_DWWD_WINDOWSIZE_100_PERCENT;
        pConfig.SDL_RTI_dwwdReaction   = RTI_DWWD_REACTION_GENERATE_NMI;

        SDL_RTI_config(validInstance, &pConfig);

        pConfig.SDL_RTI_dwwdReaction   = RTI_DWWD_REACTION_INVALID;

        sdlRet = SDL_RTI_verifyConfig(validInstance, &pConfig);

        if (sdlRet == SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("SDL_RTI_verifyConfig: failure on line no. %d \r\n", __LINE__);
        }
     }

     if (testStatus == SDL_APP_TEST_PASS)
     {
        pConfig.SDL_RTI_dwwdPreloadVal = SDL_RTI_DWWDPRLD_VLD;
        pConfig.SDL_RTI_dwwdWindowSize = RTI_DWWD_WINDOWSIZE_100_PERCENT;
        pConfig.SDL_RTI_dwwdReaction   = RTI_DWWD_REACTION_GENERATE_NMI;

        SDL_RTI_config(validInstance, &pConfig);

        pConfig.SDL_RTI_dwwdPreloadVal = RTI_RTIDWDPRLD_INVALID;

        sdlRet = SDL_RTI_verifyConfig(validInstance, &pConfig);

        if (sdlRet == SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("SDL_RTI_verifyConfig: failure on line no. %d \r\n", __LINE__);
        }
     }

    if (testStatus == SDL_APP_TEST_PASS)
     {
        pConfig.SDL_RTI_dwwdPreloadVal = SDL_RTI_DWWDPRLD_VLD;
        pConfig.SDL_RTI_dwwdWindowSize = RTI_DWWD_WINDOWSIZE_100_PERCENT;
        pConfig.SDL_RTI_dwwdReaction   = RTI_DWWD_REACTION_GENERATE_NMI;

        SDL_RTI_config(validInstance, &pConfig);

        pConfig.SDL_RTI_dwwdWindowSize = RTI_RTIDWWDSIZECTRL_DWWDSIZE_INVALID;

        sdlRet = SDL_RTI_verifyConfig(validInstance, &pConfig);

        if (sdlRet == SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("SDL_RTI_verifyConfig: failure on line no. %d \r\n", __LINE__);
        }
     }


/*******************************************************************************************
*     Call SDL API SDL_RTI_start
*******************************************************************************************/

     if (testStatus == SDL_APP_TEST_PASS)
     {
        sdlRet = SDL_RTI_start(SDL_INSTANCE_INVALID);

        if (sdlRet == SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("SDL_RTI_start: failure on line no. %d \r\n", __LINE__);
        }
     }



/*******************************************************************************************
*     Call SDL API SDL_RTI_service
*******************************************************************************************/

    if (testStatus == SDL_APP_TEST_PASS)
    {
        sdlRet = SDL_RTI_service(SDL_INSTANCE_INVALID);

        if (sdlRet == SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("\n  SDL_RTI_service Negative test failed on line no: %d \r\n", __LINE__);
        }
    }



/*******************************************************************************************
*     Call SDL API SDL_RTI_clearStatus
*******************************************************************************************/

    if (testStatus == SDL_APP_TEST_PASS)
    {
        sdlRet = SDL_RTI_clearStatus(SDL_INSTANCE_INVALID, STATUS_INVLD);

        if (sdlRet == SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("\n  SDL_RTI_clearStatus Negative test failed on line no: %d \r\n", __LINE__);
        }
    }

    if (testStatus == SDL_APP_TEST_PASS)
    {
        sdlRet = SDL_RTI_clearStatus(validInstance, STATUS_INVLD);

        if (sdlRet == SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("\n  SDL_RTI_clearStatus Negative test failed on line no: %d \r\n", __LINE__);
        }
    }

    if (testStatus == SDL_APP_TEST_PASS)
    {
        sdlRet = SDL_RTI_clearStatus(SDL_INSTANCE_INVALID, STATUS_VLD);

        if (sdlRet == SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("\n  SDL_RTI_clearStatus Negative test failed on line no: %d \r\n", __LINE__);
        }
    }



/*******************************************************************************************
*     Call SDL API SDL_RTI_getStatus
*******************************************************************************************/

    if (testStatus == SDL_APP_TEST_PASS)
    {
        sdlRet = SDL_RTI_getStatus(SDL_INSTANCE_INVALID, NULL);

        if (sdlRet == SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("\n  SDL_RTI_getStatus Negative test failed on line no: %d \r\n", __LINE__);
        }
    }

    if (testStatus == SDL_APP_TEST_PASS)
    {
        sdlRet = SDL_RTI_getStatus(validInstance, NULL);

        if (sdlRet == SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("\n  SDL_RTI_getStatus Negative test failed on line no: %d \r\n", __LINE__);
        }
    }

    if (testStatus == SDL_APP_TEST_PASS)
    {
        sdlRet = SDL_RTI_getStatus(SDL_INSTANCE_INVALID, &status);

        if (sdlRet == SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("\n  SDL_RTI_getStatus Negative test failed on line no: %d \r\n", __LINE__);
        }
    }



/*******************************************************************************************
*     Call SDL API SDL_RTI_readStaticRegs
*******************************************************************************************/

    if (testStatus == SDL_APP_TEST_PASS)
    {
        sdlRet = SDL_RTI_readStaticRegs(SDL_INSTANCE_INVALID, NULL);

        if (sdlRet == SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("\n  SDL_RTI_readStaticRegs Negative test failed on line no: %d \r\n", __LINE__);
        }
    }

    if (testStatus == SDL_APP_TEST_PASS)
    {
        sdlRet = SDL_RTI_readStaticRegs(validInstance, NULL);

        if (sdlRet == SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("\n  SDL_RTI_readStaticRegs Negative test failed on line no: %d \r\n", __LINE__);
        }
    }

    if (testStatus == SDL_APP_TEST_PASS)
    {
        sdlRet = SDL_RTI_readStaticRegs(SDL_INSTANCE_INVALID, &pStaticRegs);

        if (sdlRet == SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("\n  SDL_RTI_readStaticRegs Negative test failed on line no: %d \r\n", __LINE__);
        }
    }



/*******************************************************************************************
*     Coverage test case for SDL_RTI_getWindowSize
*******************************************************************************************/

    baseAddr = SDL_RTI_baseAddress[SDL_INSTANCE_RTI];

    if (testStatus == SDL_APP_TEST_PASS)
    {

        sdlRet = SDL_RTI_getWindowSize(0U, NULL);

        if (sdlRet == SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("\n  SDL_RTI_getPreload Negative test failed on line no: %d \r\n", __LINE__);
        }
    }

    if (testStatus == SDL_APP_TEST_PASS)
    {

        sdlRet = SDL_RTI_getWindowSize(baseAddr, NULL);

        if (sdlRet == SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("\n  SDL_RTI_getPreload Negative test failed on line no: %d \r\n", __LINE__);
        }
    }



/*******************************************************************************************
*     Coverage test case for SDL_RTI_getPreload
*******************************************************************************************/

    if (testStatus == SDL_APP_TEST_PASS)
    {

        sdlRet = SDL_RTI_getPreload(baseAddr, NULL);

        if (sdlRet == SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("\n  SDL_RTI_getPreload Negative test failed on line no: %d \r\n", __LINE__);
        }
    }

    if (testStatus == SDL_APP_TEST_PASS)
    {

        sdlRet = SDL_RTI_getPreload(0U, NULL);

        if (sdlRet == SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("\n  SDL_RTI_getPreload Negative test failed on line no: %d \r\n", __LINE__);
        }
    }



/*******************************************************************************************
*     Coverage test case for SDL_RTI_setPreload
*******************************************************************************************/

    if (testStatus == SDL_APP_TEST_PASS)
    {
        sdlRet = SDL_RTI_setPreload(0U, RTI_RTIDWDPRLD_INVALID);

        if (sdlRet == SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("\n  SDL_RTI_setPreload Negative test failed on line no: %d \r\n", __LINE__);
        }
    }



/*******************************************************************************************
*     Coverage test case for SDL_RTI_chkWindowSize
*******************************************************************************************/

    if (testStatus == SDL_APP_TEST_PASS)
    {
        sdlRet = SDL_RTI_chkWindowSize(pConfig.SDL_RTI_dwwdWindowSize);

        if (sdlRet == SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("\n  SDL_RTI_chkWindowSize Negative test failed on line no: %d \r\n", __LINE__);
        }
    }



/*******************************************************************************************
*     Call SDL API SDL_RTI_getWindowSize
*******************************************************************************************/

    if (testStatus == SDL_APP_TEST_PASS)
    {

        sdlRet = SDL_RTI_getWindowSize(0U, NULL);

        if (sdlRet == SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("\n  SDL_RTI_getWindowSize Negative test failed on line no: %d \r\n", __LINE__);
        }

    }




    /* Returning the test status  */
    return (testStatus);

}
