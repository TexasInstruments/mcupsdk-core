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
 *  \file     sdl_mbox_api_test_pos.c
 *
 *  \brief    This file contains mbox API unit test code.
 *
 *  \details  mbox unit tests
 **/

#include "mbox_main.h"

int32_t sdl_mbox_posTest(void)
{
    int32_t testStatus = SDL_APP_TEST_PASS;
#if defined SUBSYS_MSS
    SDL_MSS_MBOX_secExecute();
    SDL_MSS_MBOX_dedExecute();
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_MSS_MBOX_redExecute(SDL_MBOX_FI_MAIN, SDL_MBOX_MAIN_CMD_INTERFACE) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDL_MBOX_api_pos_Test: failure on line no. %d \n", __LINE__);
        return (testStatus);
    }
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_MSS_MBOX_redExecute(SDL_MBOX_FI_SAFE, SDL_MBOX_MAIN_CMD_INTERFACE) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDL_mbox_api_pos_Test: failure on line no. %d \n", __LINE__);
        return (testStatus);
    }
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_MSS_MBOX_redExecute(SDL_MBOX_FI_GLOBAL_MAIN, SDL_MBOX_MAIN_CMD_INTERFACE) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDL_mbox_api_pos_Test: failure on line no. %d \n", __LINE__);
        return (testStatus);
    }
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_MSS_MBOX_redExecute(SDL_MBOX_FI_GLOBAL_SAFE, SDL_MBOX_MAIN_CMD_INTERFACE) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDL_mbox_api_pos_Test: failure on line no. %d \n", __LINE__);
        return (testStatus);
    }
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_MSS_MBOX_redExecute(SDL_MBOX_FI_SAFE, SDL_MBOX_MAIN_CMD_INTERFACE) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDL_mbox_api_pos_Test: failure on line no. %d \n", __LINE__);
        return (testStatus);
    }

    SDL_MSS_MBOX_secErrorClear();
    SDL_MSS_MBOX_dedErrorClear();
#endif
#if defined SUBSYS_DSS
    SDL_DSS_MBOX_secExecute();
    SDL_DSS_MBOX_dedExecute();
#if defined (SOC_AWR294X)
    SDL_RSS_MBOX_secExecute();
    SDL_RSS_MBOX_dedExecute();
#endif
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_DSS_MBOX_redExecute(SDL_MBOX_FI_MAIN, SDL_MBOX_MAIN_CMD_INTERFACE) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDL_MBOX_api_pos_Test: failure on line no. %d \n", __LINE__);
        return (testStatus);
    }
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_DSS_MBOX_redExecute(SDL_MBOX_FI_SAFE, SDL_MBOX_MAIN_CMD_INTERFACE) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDL_mbox_api_pos_Test: failure on line no. %d \n", __LINE__);
        return (testStatus);
    }
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_DSS_MBOX_redExecute(SDL_MBOX_FI_GLOBAL_MAIN, SDL_MBOX_MAIN_CMD_INTERFACE) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDL_mbox_api_pos_Test: failure on line no. %d \n", __LINE__);
        return (testStatus);
    }
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_DSS_MBOX_redExecute(SDL_MBOX_FI_GLOBAL_SAFE, SDL_MBOX_MAIN_CMD_INTERFACE) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDL_mbox_api_pos_Test: failure on line no. %d \n", __LINE__);
        return (testStatus);
    }
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_DSS_MBOX_redExecute(SDL_MBOX_FI_SAFE, SDL_MBOX_MAIN_CMD_INTERFACE) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDL_mbox_api_pos_Test: failure on line no. %d \n", __LINE__);
        return (testStatus);
    }
#if defined (SOC_AWR294X)
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_DSS_MBOX_redExecute(SDL_MBOX_FI_MAIN, SDL_MBOX_MAIN_CMD_INTERFACE) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDL_MBOX_api_pos_Test: failure on line no. %d \n", __LINE__);
        return (testStatus);
    }
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_DSS_MBOX_redExecute(SDL_MBOX_FI_SAFE, SDL_MBOX_MAIN_CMD_INTERFACE) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDL_mbox_api_pos_Test: failure on line no. %d \n", __LINE__);
        return (testStatus);
    }
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_DSS_MBOX_redExecute(SDL_MBOX_FI_GLOBAL_MAIN, SDL_MBOX_MAIN_CMD_INTERFACE) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDL_mbox_api_pos_Test: failure on line no. %d \n", __LINE__);
        return (testStatus);
    }
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_DSS_MBOX_redExecute(SDL_MBOX_FI_GLOBAL_SAFE, SDL_MBOX_MAIN_CMD_INTERFACE) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDL_mbox_api_pos_Test: failure on line no. %d \n", __LINE__);
        return (testStatus);
    }
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_DSS_MBOX_redExecute(SDL_MBOX_FI_SAFE, SDL_MBOX_MAIN_CMD_INTERFACE) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDL_mbox_api_pos_Test: failure on line no. %d \n", __LINE__);
        return (testStatus);
    }

    SDL_RSS_MBOX_secErrorClear();
    SDL_RSS_MBOX_dedErrorClear();
    SDL_RSS_MBOX_redErrorClear();
#endif
    SDL_DSS_MBOX_secErrorStatus();
    SDL_DSS_MBOX_dedErrorStatus();
    SDL_DSS_MBOX_secErrorClear();
    SDL_DSS_MBOX_dedErrorClear();
    SDL_DSS_MBOX_redErrorClear();
#endif
    return (testStatus);
}
