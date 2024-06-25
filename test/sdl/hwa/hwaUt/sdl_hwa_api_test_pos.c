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
 *  \file     sdl_hwa_api_test_pos.c
 *
 *  \brief    This file contains hwa API unit test code.
 *
 *  \details  hwa unit tests
 **/
#include "hwa_main.h"

/********************************************************************************************************
* Macro
*********************************************************************************************************/

#define SDL_INTR_GROUP_NUM 2U

/********************************************************************************************************
* Positive test for HWA API
*********************************************************************************************************/

int32_t sdl_hwa_posTest(void)
{

    int32_t   testStatus = SDL_APP_TEST_PASS;
    /********************************************************************************************************
    * Test for SDL_HWA_memParityExecute with different input parameters
    *********************************************************************************************************/
    /* Window RAM */
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_HWA_memParityExecute(SDL_HWA_WINDOW_RAM_MEM_ID, SDL_HWA_WINDOW_RAM) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDL_hwa_api_pos_Test: failure on line no. %d \r\n", __LINE__);
        return (testStatus);
    }
    /* DMA0 DMEM0 */
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_HWA_memParityExecute(SDL_HWA_DMA0_MEM_ID, SDL_HWA_DMEM0) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDL_hwa_api_pos_Test: failure on line no. %d \r\n", __LINE__);
        return (testStatus);
    }
    /* DMA0 DMEM1 */
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_HWA_memParityExecute(SDL_HWA_DMA0_MEM_ID, SDL_HWA_DMEM1) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDL_hwa_api_pos_Test: failure on line no. %d \r\n", __LINE__);
        return (testStatus);
    }
    /* DMA0 DMEM2 */
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_HWA_memParityExecute(SDL_HWA_DMA0_MEM_ID, SDL_HWA_DMEM2) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDL_hwa_api_pos_Test: failure on line no. %d \r\n", __LINE__);
        return (testStatus);
    }
    /* DMA0 DMEM3 */
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_HWA_memParityExecute(SDL_HWA_DMA0_MEM_ID, SDL_HWA_DMEM3) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDL_hwa_api_pos_Test: failure on line no. %d \r\n", __LINE__);
        return (testStatus);
    }
    /* DMA0 DMEM4 */
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_HWA_memParityExecute(SDL_HWA_DMA0_MEM_ID, SDL_HWA_DMEM4) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDL_hwa_api_pos_Test: failure on line no. %d \r\n", __LINE__);
        return (testStatus);
    }
    /* DMA0 DMEM5 */
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_HWA_memParityExecute(SDL_HWA_DMA0_MEM_ID, SDL_HWA_DMEM5) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDL_hwa_api_pos_Test: failure on line no. %d \r\n", __LINE__);
        return (testStatus);
    }
    /* DMA0 DMEM6 */
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_HWA_memParityExecute(SDL_HWA_DMA0_MEM_ID, SDL_HWA_DMEM6) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDL_hwa_api_pos_Test: failure on line no. %d \r\n", __LINE__);
        return (testStatus);
    }
    /* DMA0 DMEM7 */
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_HWA_memParityExecute(SDL_HWA_DMA0_MEM_ID, SDL_HWA_DMEM7) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDL_hwa_api_pos_Test: failure on line no. %d \r\n", __LINE__);
        return (testStatus);
    }
    /* DMA1 DMEM0 */
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_HWA_memParityExecute(SDL_HWA_DMA1_MEM_ID, SDL_HWA_DMEM0) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDL_hwa_api_pos_Test: failure on line no. %d \r\n", __LINE__);
        return (testStatus);
    }
    /* DMA1 DMEM1 */
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_HWA_memParityExecute(SDL_HWA_DMA1_MEM_ID, SDL_HWA_DMEM1) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDL_hwa_api_pos_Test: failure on line no. %d \r\n", __LINE__);
        return (testStatus);
    }
    /* DMA1 DMEM2 */
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_HWA_memParityExecute(SDL_HWA_DMA1_MEM_ID, SDL_HWA_DMEM2) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDL_hwa_api_pos_Test: failure on line no. %d \r\n", __LINE__);
        return (testStatus);
    }
    /* DMA1 DMEM3 */
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_HWA_memParityExecute(SDL_HWA_DMA1_MEM_ID, SDL_HWA_DMEM3) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDL_hwa_api_pos_Test: failure on line no. %d \r\n", __LINE__);
        return (testStatus);
    }
    /* DMA1 DMEM4 */
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_HWA_memParityExecute(SDL_HWA_DMA1_MEM_ID, SDL_HWA_DMEM4) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDL_hwa_api_pos_Test: failure on line no. %d \r\n", __LINE__);
        return (testStatus);
    }
    /* DMA1 DMEM5 */
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_HWA_memParityExecute(SDL_HWA_DMA1_MEM_ID, SDL_HWA_DMEM5) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDL_hwa_api_pos_Test: failure on line no. %d \r\n", __LINE__);
        return (testStatus);
    }
    /* DMA1 DMEM6 */
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_HWA_memParityExecute(SDL_HWA_DMA1_MEM_ID, SDL_HWA_DMEM6) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDL_hwa_api_pos_Test: failure on line no. %d \r\n", __LINE__);
        return (testStatus);
    }
    /* DMA1 DMEM7 with interrupt configured */
    SDL_HWA_ESM_CallbackFunction(SDL_ESM_INST_DSS_ESM, SDL_INTR_GROUP_NUM,1, NULL);
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_HWA_memParityExecute(SDL_HWA_DMA1_MEM_ID, SDL_HWA_DMEM7) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDL_hwa_api_pos_Test: failure on line no. %d \r\n", __LINE__);
        return (testStatus);
    }
    /********************************************************************************************************
    * Test for SDL_HWA_fsmLockStepExecute
    *********************************************************************************************************/
    /* FSM lock step */
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_HWA_fsmLockStepExecute() != SDL_PASS)
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
