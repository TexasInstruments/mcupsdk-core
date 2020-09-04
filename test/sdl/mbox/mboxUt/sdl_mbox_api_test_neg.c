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
 *  \file     sdl_mbox_api_test_neg.c
 *
 *  \brief    This file contains mbox API unit test code..
 *
 *  \details  mbox unit tests
 **/

#include "mbox_main.h"

int32_t sdl_mbox_negTest(void)
{
    int32_t               testStatus = SDL_APP_TEST_PASS;
#if defined SUBSYS_MSS
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_MSS_MBOX_redExecute(SDL_MBOX_FI_GLOBAL_SAFE, SDL_MBOX_MAIN_CMD_INTERFACE) != SDL_EBADARGS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDL_MBOX_api_neg_Test: failure on line no. %d \n", __LINE__);
        return (testStatus);
    }
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_MSS_MBOX_redExecute(SDL_MBOX_FI_INVALID, SDL_MBOX_FI_TYPE_INVALID) != SDL_EBADARGS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDL_MBOX_api_neg_Test: failure on line no. %d \n", __LINE__);
        return (testStatus);
    }
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_MSS_MBOX_redExecute(SDL_MBOX_FI_INVALID, SDL_MBOX_FI_TYPE_INVALID) != SDL_EBADARGS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDL_MBOX_api_neg_Test: failure on line no. %d \n", __LINE__);
        return (testStatus);
    }
#endif
#if defined SUBSYS_DSS
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_DSS_MBOX_redExecute(SDL_MBOX_FI_GLOBAL_SAFE, SDL_MBOX_FI_TYPE_INVALID) != SDL_EBADARGS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDL_MBOX_api_neg_Test: failure on line no. %d \n", __LINE__);
        return (testStatus);
    }
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_DSS_MBOX_redExecute(SDL_MBOX_FI_INVALID, SDL_MBOX_MAIN_CMD_INTERFACE) != SDL_EBADARGS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDL_MBOX_api_neg_Test: failure on line no. %d \n", __LINE__);
        return (testStatus);
    }
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_DSS_MBOX_redExecute(SDL_MBOX_FI_INVALID, SDL_MBOX_FI_TYPE_INVALID) != SDL_EBADARGS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDL_MBOX_api_neg_Test: failure on line no. %d \n", __LINE__);
        return (testStatus);
    }
#if defined (SOC_AWR294X)
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_RSS_MBOX_redExecute(SDL_MBOX_FI_GLOBAL_SAFE, SDL_MBOX_FI_TYPE_INVALID) != SDL_EBADARGS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDL_MBOX_api_neg_Test: failure on line no. %d \n", __LINE__);
        return (testStatus);
    }
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_RSS_MBOX_redExecute(SDL_MBOX_FI_INVALID, SDL_MBOX_MAIN_CMD_INTERFACE) != SDL_EBADARGS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDL_MBOX_api_neg_Test: failure on line no. %d \n", __LINE__);
        return (testStatus);
    }
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_RSS_MBOX_redExecute(SDL_MBOX_FI_INVALID, SDL_MBOX_FI_TYPE_INVALID) != SDL_EBADARGS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDL_MBOX_api_neg_Test: failure on line no. %d \n", __LINE__);
        return (testStatus);
    }

#endif
#endif
    return (testStatus);
}

