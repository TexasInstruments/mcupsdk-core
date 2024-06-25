/* Copyright (c) 2022-23 Texas Instruments Incorporated
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
 *  \file     sdl_hwa_api_test_neg.c
 *
 *  \brief    This file contains hwa API unit test code..
 *
 *  \details  hwa unit tests
 **/

#include "hwa_main.h"

/********************************************************************************************************
* Negative test for HWA API
*********************************************************************************************************/

int32_t sdl_hwa_negTest(void)
{
    int32_t testStatus = SDL_APP_TEST_PASS;
    /********************************************************************************************************
    * Test for SDL_HWA_memParityExecute with invalid input parameters
    *********************************************************************************************************/
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_HWA_memParityExecute(SDL_HWA_INVALID_ID, SDL_HWA_DMEM0) != SDL_EBADARGS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDL_hwa_api_neg_Test: failure on line no. %d \r\n", __LINE__);
        return (testStatus);
    }
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_HWA_memParityExecute(SDL_HWA_DMA0_MEM_ID, SDL_HWA_INVALID) != SDL_EBADARGS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDL_hwa_api_neg_Test: failure on line no. %d \r\n", __LINE__);
        return (testStatus);
    }
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if(SDL_HWA_memParityExecute(SDL_HWA_INVALID_ID, SDL_HWA_INVALID) != SDL_EBADARGS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDL_hwa_api_neg_Test: failure on line no. %d \r\n", __LINE__);
        return (testStatus);
    }
    /********************************************************************************************************
    * Test for SDL_HWA_getMemblockBaseaddr with invalid input parameters
    *********************************************************************************************************/
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_HWA_getMemblockBaseaddr(SDL_HWA_DMA0_MEM_ID,SDL_HWA_DMEM1,NULL_PTR) != SDL_EBADARGS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDL_hwa_api_pos_Test: failure on line no. %d \r\n", __LINE__);
    }
    /********************************************************************************************************
    * Test for SDL_HWA_memParityExecute with invalid input parameters
    *********************************************************************************************************/
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_HWA_memParityExecute(SDL_HWA_DMA1_MEM_ID, SDL_HWA_INVALID) != SDL_EBADARGS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDL_hwa_api_neg_Test: failure on line no. %d \r\n", __LINE__);
        return (testStatus);
    }
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if(SDL_HWA_memParityExecute(SDL_HWA_INVALID_ID, SDL_HWA_INVALID) != SDL_EBADARGS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDL_hwa_api_neg_Test: failure on line no. %d \r\n", __LINE__);
        return (testStatus);
    }
    /********************************************************************************************************
    * Test for SDL_HWA_getMemblockBaseaddr with invalid input parameters
    *********************************************************************************************************/
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_HWA_getMemblockBaseaddr(SDL_HWA_DMA1_MEM_ID,SDL_HWA_DMEM1,NULL_PTR) != SDL_EBADARGS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDL_hwa_api_pos_Test: failure on line no. %d \r\n", __LINE__);
    }
    return (testStatus);
}
