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

/* This file contains RTI API positive test code */


#include "rti_main.h"

int32_t SDL_RTI_posTest(void)
{
    uint32_t      status,i=0, preload_rd, baseAddr, winSize;
    int32_t     testStatus = SDL_APP_TEST_PASS, sdlRet;
    SDL_RTI_configParms       pConfig;
    SDL_RTI_InstanceType   validInstance = SDL_INSTANCE_RTI;

    SDL_RTI_staticRegs     pstaticRegs;


    pConfig.SDL_RTI_dwwdPreloadVal = SDL_RTI_DWWDPRLD_VLD;
    pConfig.SDL_RTI_dwwdWindowSize = RTI_DWWD_WINDOWSIZE_100_PERCENT;
    pConfig.SDL_RTI_dwwdReaction   = RTI_DWWD_REACTION_GENERATE_NMI;

    /* This will assign base address of given Instance type    */
    sdlRet = SDL_RTI_getBaseaddr(validInstance, &baseAddr);

/*******************************************************************************************
*     Call SDL API SDL_RTI_config
*******************************************************************************************/

    if (testStatus == SDL_APP_TEST_PASS)
    {

        sdlRet = SDL_RTI_config(( SDL_INSTANCE_RTI), &pConfig);

        if (sdlRet != SDL_PASS)
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
        sdlRet = SDL_RTI_verifyConfig((SDL_INSTANCE_RTI), &pConfig);

        if (sdlRet != SDL_PASS)
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
        sdlRet = SDL_RTI_start((SDL_INSTANCE_RTI));

        if (sdlRet != SDL_PASS)
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
        sdlRet = SDL_RTI_service((SDL_INSTANCE_RTI));

        if (sdlRet != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("\n  SDL_RTI_service positive test failed on line no: %d \r\n", __LINE__);
        }
    }



/*******************************************************************************************
*     Call SDL API SDL_RTI_clearStatus
*******************************************************************************************/

    if (testStatus == SDL_APP_TEST_PASS)
    {
        sdlRet = SDL_RTI_clearStatus((SDL_INSTANCE_RTI), STATUS_VLD);

        if (sdlRet != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("\n  SDL_RTI_clearStatus positive test failed on line no: %d \r\n", __LINE__);
        }
    }



/*******************************************************************************************
*     Call SDL API SDL_RTI_getStatus
*******************************************************************************************/

    if (testStatus == SDL_APP_TEST_PASS)
    {
        sdlRet = SDL_RTI_getStatus((SDL_INSTANCE_RTI), &status);

        if (sdlRet != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("\n  SDL_RTI_getStatus positive test failed on line no: %d \r\n", __LINE__);
        }
    }



/*******************************************************************************************
*     Call SDL API SDL_RTI_readStaticRegs
*******************************************************************************************/

    if (testStatus == SDL_APP_TEST_PASS)
    {
        sdlRet = SDL_RTI_readStaticRegs((SDL_INSTANCE_RTI), &pstaticRegs);

        if (sdlRet != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("\n  SDL_RTI_readStaticRegs positive test failed on line no: %d \r\n", __LINE__);
        }
    }



/*******************************************************************************************
*     Test case coverage for SDL_RTI_chkWindowSize
*******************************************************************************************/

    if (testStatus == SDL_APP_TEST_PASS)
    {
        for(i= 0; i <= 5; i++)
        {
            switch (i)
            {
                case 0:
                    pConfig.SDL_RTI_dwwdWindowSize = RTI_DWWD_WINDOWSIZE_50_PERCENT;
                    break;
                case 1:
                    pConfig.SDL_RTI_dwwdWindowSize = RTI_DWWD_WINDOWSIZE_25_PERCENT;
                    break;
                case 2:
                    pConfig.SDL_RTI_dwwdWindowSize = RTI_DWWD_WINDOWSIZE_12_5_PERCENT;
                    break;
                case 3:
                    pConfig.SDL_RTI_dwwdWindowSize = RTI_DWWD_WINDOWSIZE_6_25_PERCENT;
                    break;
                case 4:
                    pConfig.SDL_RTI_dwwdWindowSize = RTI_DWWD_WINDOWSIZE_3_125_PERCENT;
                    break;
            }

            sdlRet = SDL_RTI_config(validInstance, &pConfig);

            if (sdlRet != SDL_PASS)
            {
                testStatus = SDL_APP_TEST_FAILED;
                DebugP_log("SDL_RTI_config: failure on line no. %d \r\n", __LINE__);
            }
        }
    }



/*******************************************************************************************
*     Call SDL API SDL_RTI_getPreload
*******************************************************************************************/

    if (testStatus == SDL_APP_TEST_PASS)
    {

        sdlRet = SDL_RTI_getPreload(baseAddr, &preload_rd);

        if (sdlRet != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("\n  SDL_RTI_getPreload positive test failed on line no: %d \r\n", __LINE__);
        }

    }



/*******************************************************************************************
*     Call SDL API SDL_RTI_getPreload
*******************************************************************************************/

    if (testStatus == SDL_APP_TEST_PASS)
    {
        sdlRet = SDL_RTI_setPreload(baseAddr, pConfig.SDL_RTI_dwwdPreloadVal);

        if (sdlRet != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("\n  SDL_RTI_setPreload positive test failed on line no: %d \r\n", __LINE__);
        }
    }



/*******************************************************************************************
*     Call SDL API SDL_RTI_chkReaction
*******************************************************************************************/

    if (testStatus == SDL_APP_TEST_PASS)
    {
        sdlRet = SDL_RTI_chkReaction(pConfig.SDL_RTI_dwwdReaction);

        if (sdlRet != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("\n  SDL_RTI_chkReaction positive test failed on line no: %d \r\n", __LINE__);
        }
    }



/*******************************************************************************************
*     Call SDL API SDL_RTI_writeReaction
*******************************************************************************************/

    if (testStatus == SDL_APP_TEST_PASS)
    {

        SDL_RTI_writeReaction(baseAddr, pConfig.SDL_RTI_dwwdReaction);

        if (sdlRet != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("\n  SDL_RTI_writeReaction positive test failed on line no: %d \r\n", __LINE__);
        }

    }



/*******************************************************************************************
*     Call SDL API SDL_RTI_readReaction
*******************************************************************************************/

    if (testStatus == SDL_APP_TEST_PASS)
    {
        sdlRet = SDL_RTI_readReaction(baseAddr);

        if (sdlRet != pConfig.SDL_RTI_dwwdReaction)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("\n  SDL_RTI_readReaction positive test failed on line no: %d \r\n", __LINE__);
        }
    }





/*******************************************************************************************
*     Call SDL API SDL_RTI_chkWindowSize
*******************************************************************************************/

    if (testStatus == SDL_APP_TEST_PASS)
    {

        sdlRet = SDL_RTI_chkWindowSize(pConfig.SDL_RTI_dwwdWindowSize);

        if (sdlRet != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("\n  SDL_RTI_chkWindowSize positive test failed on line no: %d \r\n", __LINE__);
        }

    }


/*******************************************************************************************
*     Call SDL API SDL_RTI_getWindowSize
*******************************************************************************************/

    if (testStatus == SDL_APP_TEST_PASS)
    {

        sdlRet = SDL_RTI_getWindowSize(baseAddr, &winSize);

        if (sdlRet != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("\n  SDL_RTI_getWindowSize positive test failed on line no: %d \r\n", __LINE__);
        }

    }


/*******************************************************************************************
*     Call SDL API SDL_RTI_writeWinSz
*******************************************************************************************/

    if (testStatus == SDL_APP_TEST_PASS)
    {

        sdlRet = SDL_RTI_writeWinSz(baseAddr, pConfig.SDL_RTI_dwwdWindowSize);

        if (sdlRet != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("\n  SDL_RTI_writeWinSz positive test failed on line no: %d \r\n", __LINE__);
        }

    }



return (testStatus);
}
