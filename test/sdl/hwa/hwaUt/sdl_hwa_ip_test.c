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
 *  \file     sdl_hwa_ip_test.c
 *
 *  \brief    This file contains hwa ip functionality test code.
 *
 *  \details  hwa unit tests
 **/

#include "hwa_main.h"

int32_t sdl_ip_hwaPosTest(void)
{
    uint32_t baseAddr;
    int32_t    testStatus = SDL_APP_TEST_PASS;
        if (testStatus == SDL_APP_TEST_PASS)
        {
            if (SDL_HWA_getMemblockBaseaddr(SDL_HWA_DMA0_MEM_ID,SDL_HWA_DMEM0,&baseAddr) != SDL_PASS)
            {
                testStatus = SDL_APP_TEST_FAILED;
            }
        }
        if (testStatus != SDL_APP_TEST_PASS)
        {
            DebugP_log("SDL_HWA_IP_Pos_Test: failure on line no. %d \n", __LINE__);
            return (testStatus);
        }

        if (testStatus == SDL_APP_TEST_PASS)
        {
            if (SDL_HWA_getMemblockBaseaddr(SDL_HWA_DMA0_MEM_ID,SDL_HWA_DMEM1,&baseAddr) != SDL_PASS)
            {
                testStatus = SDL_APP_TEST_FAILED;
            }
        }
        if (testStatus != SDL_APP_TEST_PASS)
        {
            DebugP_log("SDL_HWA_IP_Pos_Test: failure on line no. %d \n", __LINE__);
            return (testStatus);
        }
        if (testStatus == SDL_APP_TEST_PASS)
        {
            if (SDL_HWA_getMemblockBaseaddr(SDL_HWA_DMA0_MEM_ID,SDL_HWA_DMEM2,&baseAddr) != SDL_PASS)
            {
                testStatus = SDL_APP_TEST_FAILED;
            }
        }
        if (testStatus != SDL_APP_TEST_PASS)
        {
            DebugP_log("SDL_HWA_IP_Pos_Test: failure on line no. %d \n", __LINE__);
            return (testStatus);
        }
        if (testStatus == SDL_APP_TEST_PASS)
        {
            if (SDL_HWA_getMemblockBaseaddr(SDL_HWA_DMA0_MEM_ID,SDL_HWA_DMEM3,&baseAddr) != SDL_PASS)
            {
                testStatus = SDL_APP_TEST_FAILED;
            }
        }
        if (testStatus != SDL_APP_TEST_PASS)
        {
            DebugP_log("SDL_HWA_IP_Pos_Test: failure on line no. %d \n", __LINE__);
            return (testStatus);
        }
        if (testStatus == SDL_APP_TEST_PASS)
        {
            if (SDL_HWA_getMemblockBaseaddr(SDL_HWA_DMA0_MEM_ID,SDL_HWA_DMEM4,&baseAddr) != SDL_PASS)
            {
                testStatus = SDL_APP_TEST_FAILED;
            }
        }
        if (testStatus != SDL_APP_TEST_PASS)
        {
            DebugP_log("SDL_HWA_IP_Pos_Test: failure on line no. %d \n", __LINE__);
            return (testStatus);
        }
        if (testStatus == SDL_APP_TEST_PASS)
        {
            if (SDL_HWA_getMemblockBaseaddr(SDL_HWA_DMA0_MEM_ID,SDL_HWA_DMEM5,&baseAddr) != SDL_PASS)
            {
                testStatus = SDL_APP_TEST_FAILED;
            }
        }
        if (testStatus != SDL_APP_TEST_PASS)
        {
            DebugP_log("SDL_HWA_IP_Pos_Test: failure on line no. %d \n", __LINE__);
            return (testStatus);
        }
        if (testStatus == SDL_APP_TEST_PASS)
        {
            if (SDL_HWA_getMemblockBaseaddr(SDL_HWA_DMA0_MEM_ID,SDL_HWA_DMEM6,&baseAddr) != SDL_PASS)
            {
                testStatus = SDL_APP_TEST_FAILED;
            }
        }
        if (testStatus != SDL_APP_TEST_PASS)
        {
            DebugP_log("SDL_HWA_IP_Pos_Test: failure on line no. %d \n", __LINE__);
            return (testStatus);
        }
        if (testStatus == SDL_APP_TEST_PASS)
        {
            if (SDL_HWA_getMemblockBaseaddr(SDL_HWA_DMA0_MEM_ID,SDL_HWA_DMEM7,&baseAddr) != SDL_PASS)
            {
                testStatus = SDL_APP_TEST_FAILED;
            }
        }
        if (testStatus != SDL_APP_TEST_PASS)
        {
            DebugP_log("SDL_HWA_IP_Pos_Test: failure on line no. %d \n", __LINE__);
            return (testStatus);
        }

        if (testStatus == SDL_APP_TEST_PASS)
        {
            if (SDL_HWA_getMemblockBaseaddr(SDL_HWA_DMA0_MEM_ID,SDL_HWA_WINDOW_RAM,&baseAddr) != SDL_PASS)
            {
                testStatus = SDL_APP_TEST_FAILED;
            }
        }
        if (testStatus != SDL_APP_TEST_PASS)
        {
            DebugP_log("SDL_HWA_IP_Pos_Test: failure on line no. %d \n", __LINE__);
            return (testStatus);
        }
        if (testStatus == SDL_APP_TEST_PASS)
        {
            if (SDL_HWA_getMemblockBaseaddr(SDL_HWA_DMA1_MEM_ID,SDL_HWA_DMEM0,&baseAddr) != SDL_PASS)
            {
                testStatus = SDL_APP_TEST_FAILED;
            }
        }
        if (testStatus != SDL_APP_TEST_PASS)
        {
            DebugP_log("SDL_HWA_IP_Pos_Test: failure on line no. %d \n", __LINE__);
            return (testStatus);
        }

        if (testStatus == SDL_APP_TEST_PASS)
        {
            if (SDL_HWA_getMemblockBaseaddr(SDL_HWA_DMA1_MEM_ID,SDL_HWA_DMEM1,&baseAddr) != SDL_PASS)
            {
                testStatus = SDL_APP_TEST_FAILED;
            }
        }
        if (testStatus != SDL_APP_TEST_PASS)
        {
            DebugP_log("SDL_HWA_IP_Pos_Test: failure on line no. %d \n", __LINE__);
            return (testStatus);
        }
        if (testStatus == SDL_APP_TEST_PASS)
        {
            if (SDL_HWA_getMemblockBaseaddr(SDL_HWA_DMA1_MEM_ID,SDL_HWA_DMEM2,&baseAddr) != SDL_PASS)
            {
                testStatus = SDL_APP_TEST_FAILED;
            }
        }
        if (testStatus != SDL_APP_TEST_PASS)
        {
            DebugP_log("SDL_HWA_IP_Pos_Test: failure on line no. %d \n", __LINE__);
            return (testStatus);
        }
        if (testStatus == SDL_APP_TEST_PASS)
        {
            if (SDL_HWA_getMemblockBaseaddr(SDL_HWA_DMA1_MEM_ID,SDL_HWA_DMEM3,&baseAddr) != SDL_PASS)
            {
                testStatus = SDL_APP_TEST_FAILED;
            }
        }
        if (testStatus != SDL_APP_TEST_PASS)
        {
            DebugP_log("SDL_HWA_IP_Pos_Test: failure on line no. %d \n", __LINE__);
            return (testStatus);
        }
        if (testStatus == SDL_APP_TEST_PASS)
        {
            if (SDL_HWA_getMemblockBaseaddr(SDL_HWA_DMA1_MEM_ID,SDL_HWA_DMEM4,&baseAddr) != SDL_PASS)
            {
                testStatus = SDL_APP_TEST_FAILED;
            }
        }
        if (testStatus != SDL_APP_TEST_PASS)
        {
            DebugP_log("SDL_HWA_IP_Pos_Test: failure on line no. %d \n", __LINE__);
            return (testStatus);
        }
        if (testStatus == SDL_APP_TEST_PASS)
        {
            if (SDL_HWA_getMemblockBaseaddr(SDL_HWA_DMA1_MEM_ID,SDL_HWA_DMEM5,&baseAddr) != SDL_PASS)
            {
                testStatus = SDL_APP_TEST_FAILED;
            }
        }
        if (testStatus != SDL_APP_TEST_PASS)
        {
            DebugP_log("SDL_HWA_IP_Pos_Test: failure on line no. %d \n", __LINE__);
            return (testStatus);
        }
        if (testStatus == SDL_APP_TEST_PASS)
        {
            if (SDL_HWA_getMemblockBaseaddr(SDL_HWA_DMA1_MEM_ID,SDL_HWA_DMEM6,&baseAddr) != SDL_PASS)
            {
                testStatus = SDL_APP_TEST_FAILED;
            }
        }
        if (testStatus != SDL_APP_TEST_PASS)
        {
            DebugP_log("SDL_HWA_IP_Pos_Test: failure on line no. %d \n", __LINE__);
            return (testStatus);
        }
        if (testStatus == SDL_APP_TEST_PASS)
        {
            if (SDL_HWA_getMemblockBaseaddr(SDL_HWA_DMA1_MEM_ID,SDL_HWA_DMEM7,&baseAddr) != SDL_PASS)
            {
                testStatus = SDL_APP_TEST_FAILED;
            }
        }
        if (testStatus != SDL_APP_TEST_PASS)
        {
            DebugP_log("SDL_HWA_IP_Pos_Test: failure on line no. %d \n", __LINE__);
            return (testStatus);
        }
        if (testStatus == SDL_APP_TEST_PASS)
        {
            if (SDL_HWA_getMemblockBaseaddr(SDL_HWA_DMA1_MEM_ID,SDL_HWA_WINDOW_RAM,&baseAddr) != SDL_PASS)
            {
                testStatus = SDL_APP_TEST_FAILED;
            }
        }
        if (testStatus != SDL_APP_TEST_PASS)
        {
            DebugP_log("SDL_HWA_IP_Pos_Test: failure on line no. %d \n", __LINE__);
            return (testStatus);
        }

        if (testStatus == SDL_APP_TEST_PASS)
        {
            SDL_HWA_memoryInitStart(SDL_HWA_DMEM0);
            while(1U==SDL_HWA_memoryInitStatus(SDL_HWA_DMEM0));
            if (0U!=SDL_HWA_memoryInitDone(SDL_HWA_DMEM0))
            {
                testStatus = SDL_APP_TEST_FAILED;
            }
        }
        if (testStatus != SDL_APP_TEST_PASS)
        {
            DebugP_log("SDL_HWA_IP_Pos_Test: failure on line no. %d \n", __LINE__);
            return (testStatus);
        }

        if (testStatus == SDL_APP_TEST_PASS)
        {
            SDL_HWA_memoryInitStart(SDL_HWA_DMEM1);
            while(1U==SDL_HWA_memoryInitStatus(SDL_HWA_DMEM1));
            if (0U==SDL_HWA_memoryInitDone(SDL_HWA_DMEM1))
            {
                testStatus = SDL_APP_TEST_FAILED;
            }
        }
        if (testStatus != SDL_APP_TEST_PASS)
        {
            DebugP_log("SDL_HWA_IP_Pos_Test: failure on line no. %d \n", __LINE__);
            return (testStatus);
        }

        if (testStatus == SDL_APP_TEST_PASS)
        {
            SDL_HWA_memoryInitStart(SDL_HWA_DMEM2);
            while(1U==SDL_HWA_memoryInitStatus(SDL_HWA_DMEM2));
            if (0U==SDL_HWA_memoryInitDone(SDL_HWA_DMEM2))
            {
                testStatus = SDL_APP_TEST_FAILED;
            }
        }
        if (testStatus != SDL_APP_TEST_PASS)
        {
            DebugP_log("SDL_HWA_IP_Pos_Test: failure on line no. %d \n", __LINE__);
            return (testStatus);
        }

        if (testStatus == SDL_APP_TEST_PASS)
        {
            SDL_HWA_memoryInitStart(SDL_HWA_DMEM3);
            while(1U==SDL_HWA_memoryInitStatus(SDL_HWA_DMEM3));
            if (0U==SDL_HWA_memoryInitDone(SDL_HWA_DMEM3))
            {
                testStatus = SDL_APP_TEST_FAILED;
            }
        }
        if (testStatus != SDL_APP_TEST_PASS)
        {
            DebugP_log("SDL_HWA_IP_Pos_Test: failure on line no. %d \n", __LINE__);
            return (testStatus);
        }

        if (testStatus == SDL_APP_TEST_PASS)
        {
            SDL_HWA_memoryInitStart(SDL_HWA_DMEM4);
            while(1U==SDL_HWA_memoryInitStatus(SDL_HWA_DMEM4));
            if (0U==SDL_HWA_memoryInitDone(SDL_HWA_DMEM4))
            {
                testStatus = SDL_APP_TEST_FAILED;
            }
        }

        if (testStatus == SDL_APP_TEST_PASS)
        {
            SDL_HWA_memoryInitStart(SDL_HWA_DMEM5);
            while(1U==SDL_HWA_memoryInitStatus(SDL_HWA_DMEM5));
            if (0U==SDL_HWA_memoryInitDone(SDL_HWA_DMEM5))
            {
                testStatus = SDL_APP_TEST_FAILED;
            }
        }

        if (testStatus != SDL_APP_TEST_PASS)
        {
            DebugP_log("SDL_HWA_IP_Pos_Test: failure on line no. %d \n", __LINE__);
            return (testStatus);
        }

        if (testStatus == SDL_APP_TEST_PASS)
        {
            SDL_HWA_memoryInitStart(SDL_HWA_DMEM6);
            while(1U==SDL_HWA_memoryInitStatus(SDL_HWA_DMEM6));
            if (0U==SDL_HWA_memoryInitDone(SDL_HWA_DMEM6))
            {
                testStatus = SDL_APP_TEST_FAILED;
            }
        }

        if (testStatus != SDL_APP_TEST_PASS)
        {
            DebugP_log("SDL_HWA_IP_Pos_Test: failure on line no. %d \n", __LINE__);
            return (testStatus);
        }

        if (testStatus == SDL_APP_TEST_PASS)
        {
            SDL_HWA_memoryInitStart(SDL_HWA_DMEM7);
            while(1U==SDL_HWA_memoryInitStatus(SDL_HWA_DMEM7));
            if (0U==SDL_HWA_memoryInitDone(SDL_HWA_DMEM7))
            {
                testStatus = SDL_APP_TEST_FAILED;
            }
        }
        if (testStatus != SDL_APP_TEST_PASS)
        {
            DebugP_log("SDL_HWA_IP_Pos_Test: failure on line no. %d \n", __LINE__);
            return (testStatus);
        }
        if (testStatus == SDL_APP_TEST_PASS)
        {
            SDL_HWA_setParityErrMaskUnmask(SDL_HWA_DMEM0,SDL_HWA_ENABLE_STS);
            if (0U == SDL_HWA_getParityErrMaskUnmask(SDL_HWA_DMEM0))
            {
                testStatus = SDL_APP_TEST_FAILED;
            }
        }
        if (testStatus != SDL_APP_TEST_PASS)
        {
            DebugP_log("SDL_HWA_IP_Pos_Test: failure on line no. %d \n", __LINE__);
            return (testStatus);
        }

        if (testStatus == SDL_APP_TEST_PASS)
        {
            SDL_HWA_setParityErrMaskUnmask(SDL_HWA_DMEM1,SDL_HWA_ENABLE_STS);
            if (0U == SDL_HWA_getParityErrMaskUnmask(SDL_HWA_DMEM1))
            {
                testStatus = SDL_APP_TEST_FAILED;
            }
        }
        if (testStatus != SDL_APP_TEST_PASS)
        {
            DebugP_log("SDL_HWA_IP_Pos_Test: failure on line no. %d \n", __LINE__);
            return (testStatus);
        }

        if (testStatus == SDL_APP_TEST_PASS)
        {
            SDL_HWA_setParityErrMaskUnmask(SDL_HWA_DMEM2,SDL_HWA_ENABLE_STS);
            if (0U==SDL_HWA_getParityErrMaskUnmask(SDL_HWA_DMEM2))
            {
                testStatus = SDL_APP_TEST_FAILED;
            }
        }
        if (testStatus != SDL_APP_TEST_PASS)
        {
            DebugP_log("SDL_HWA_IP_Pos_Test: failure on line no. %d \n", __LINE__);
            return (testStatus);
        }

        if (testStatus == SDL_APP_TEST_PASS)
        {
            SDL_HWA_setParityErrMaskUnmask(SDL_HWA_DMEM3,SDL_HWA_ENABLE_STS);
            if (0U==SDL_HWA_getParityErrMaskUnmask(SDL_HWA_DMEM3))
            {
                testStatus = SDL_APP_TEST_FAILED;
            }
        }
        if (testStatus != SDL_APP_TEST_PASS)
        {
            DebugP_log("SDL_HWA_IP_Pos_Test: failure on line no. %d \n", __LINE__);
            return (testStatus);
        }

        if (testStatus == SDL_APP_TEST_PASS)
        {
            SDL_HWA_setParityErrMaskUnmask(SDL_HWA_DMEM4,SDL_HWA_ENABLE_STS);
            if (0U==SDL_HWA_getParityErrMaskUnmask(SDL_HWA_DMEM4))
            {
                testStatus = SDL_APP_TEST_FAILED;
            }
        }
        if (testStatus != SDL_APP_TEST_PASS)
        {
            DebugP_log("SDL_HWA_IP_Pos_Test: failure on line no. %d \n", __LINE__);
            return (testStatus);
        }

        if (testStatus == SDL_APP_TEST_PASS)
        {
            SDL_HWA_setParityErrMaskUnmask(SDL_HWA_DMEM5,SDL_HWA_ENABLE_STS);
            if (0U==SDL_HWA_getParityErrMaskUnmask(SDL_HWA_DMEM5))
            {
                testStatus = SDL_APP_TEST_FAILED;
            }
        }
        if (testStatus != SDL_APP_TEST_PASS)
        {
            DebugP_log("SDL_HWA_IP_Pos_Test: failure on line no. %d \n", __LINE__);
            return (testStatus);
        }

        if (testStatus == SDL_APP_TEST_PASS)
        {
            SDL_HWA_setParityErrMaskUnmask(SDL_HWA_DMEM6,SDL_HWA_ENABLE_STS);
            if (0U==SDL_HWA_getParityErrMaskUnmask(SDL_HWA_DMEM6))
            {
                testStatus = SDL_APP_TEST_FAILED;
            }
        }
        if (testStatus != SDL_APP_TEST_PASS)
        {
            DebugP_log("SDL_HWA_IP_Pos_Test: failure on line no. %d \n", __LINE__);
            return (testStatus);
        }

        if (testStatus == SDL_APP_TEST_PASS)
        {
            SDL_HWA_setParityErrMaskUnmask(SDL_HWA_DMEM7,SDL_HWA_ENABLE_STS);
            if (0U==SDL_HWA_getParityErrMaskUnmask(SDL_HWA_DMEM7))
            {
                testStatus = SDL_APP_TEST_FAILED;
            }
        }
        if (testStatus != SDL_APP_TEST_PASS)
        {
            DebugP_log("SDL_HWA_IP_Pos_Test: failure on line no. %d \n", __LINE__);
            return (testStatus);
        }

        if (testStatus == SDL_APP_TEST_PASS)
        {
            SDL_HWA_setParityErrMaskUnmask(SDL_HWA_WINDOW_RAM,SDL_HWA_ENABLE_STS);
            if (0U==SDL_HWA_getParityErrMaskUnmask(SDL_HWA_WINDOW_RAM))
            {
                testStatus = SDL_APP_TEST_FAILED;
            }
        }
        if (testStatus != SDL_APP_TEST_PASS)
        {
            DebugP_log("SDL_HWA_IP_Pos_Test: failure on line no. %d \n", __LINE__);
            return (testStatus);
        }

        if (testStatus == SDL_APP_TEST_PASS)
        {
            SDL_HWA_setParityErrMaskUnmask(SDL_HWA_FSM_LOCKSTEP,SDL_HWA_ENABLE_STS);
            if (0U==SDL_HWA_getParityErrMaskUnmask(SDL_HWA_FSM_LOCKSTEP))
            {
                testStatus = SDL_APP_TEST_FAILED;
            }
        }
        if (testStatus != SDL_APP_TEST_PASS)
        {
            DebugP_log("SDL_HWA_IP_Pos_Test: failure on line no. %d \n", __LINE__);
            return (testStatus);
        }

        if (testStatus == SDL_APP_TEST_PASS)
        {
            SDL_HWA_setParityErrMaskUnmask(SDL_HWA_DMEM0,SDL_HWA_DISABLE_STS);
            if (1U == SDL_HWA_getParityErrMaskUnmask(SDL_HWA_DMEM0))
            {
                testStatus = SDL_APP_TEST_FAILED;
            }
        }
        if (testStatus != SDL_APP_TEST_PASS)
        {
            DebugP_log("SDL_HWA_IP_Pos_Test: failure on line no. %d \n", __LINE__);
            return (testStatus);
        }

        if (testStatus == SDL_APP_TEST_PASS)
        {
            SDL_HWA_setParityErrMaskUnmask(SDL_HWA_DMEM1,SDL_HWA_DISABLE_STS);
            if (1U == SDL_HWA_getParityErrMaskUnmask(SDL_HWA_DMEM1))
            {
                testStatus = SDL_APP_TEST_FAILED;
            }
        }
        if (testStatus != SDL_APP_TEST_PASS)
        {
            DebugP_log("SDL_HWA_IP_Pos_Test: failure on line no. %d \n", __LINE__);
            return (testStatus);
        }

        if (testStatus == SDL_APP_TEST_PASS)
        {
            SDL_HWA_setParityErrMaskUnmask(SDL_HWA_DMEM2,SDL_HWA_DISABLE_STS);
            if (1U==SDL_HWA_getParityErrMaskUnmask(SDL_HWA_DMEM2))
            {
                testStatus = SDL_APP_TEST_FAILED;
            }
        }
        if (testStatus != SDL_APP_TEST_PASS)
        {
            DebugP_log("SDL_HWA_IP_Pos_Test: failure on line no. %d \n", __LINE__);
            return (testStatus);
        }

        if (testStatus == SDL_APP_TEST_PASS)
        {
            SDL_HWA_setParityErrMaskUnmask(SDL_HWA_DMEM3,SDL_HWA_DISABLE_STS);
            if (1U==SDL_HWA_getParityErrMaskUnmask(SDL_HWA_DMEM3))
            {
                testStatus = SDL_APP_TEST_FAILED;
            }
        }
        if (testStatus != SDL_APP_TEST_PASS)
        {
            DebugP_log("SDL_HWA_IP_Pos_Test: failure on line no. %d \n", __LINE__);
            return (testStatus);
        }

        if (testStatus == SDL_APP_TEST_PASS)
        {
            SDL_HWA_setParityErrMaskUnmask(SDL_HWA_DMEM4,SDL_HWA_DISABLE_STS);
            if (1U==SDL_HWA_getParityErrMaskUnmask(SDL_HWA_DMEM4))
            {
                testStatus = SDL_APP_TEST_FAILED;
            }
        }
        if (testStatus != SDL_APP_TEST_PASS)
        {
            DebugP_log("SDL_HWA_IP_Pos_Test: failure on line no. %d \n", __LINE__);
            return (testStatus);
        }

        if (testStatus == SDL_APP_TEST_PASS)
        {
            SDL_HWA_setParityErrMaskUnmask(SDL_HWA_DMEM5,SDL_HWA_DISABLE_STS);
            if (1U==SDL_HWA_getParityErrMaskUnmask(SDL_HWA_DMEM5))
            {
                testStatus = SDL_APP_TEST_FAILED;
            }
        }
        if (testStatus != SDL_APP_TEST_PASS)
        {
            DebugP_log("SDL_HWA_IP_Pos_Test: failure on line no. %d \n", __LINE__);
            return (testStatus);
        }

        if (testStatus == SDL_APP_TEST_PASS)
        {
            SDL_HWA_setParityErrMaskUnmask(SDL_HWA_DMEM6,SDL_HWA_DISABLE_STS);
            if (1U==SDL_HWA_getParityErrMaskUnmask(SDL_HWA_DMEM6))
            {
                testStatus = SDL_APP_TEST_FAILED;
            }
        }
        if (testStatus != SDL_APP_TEST_PASS)
        {
            DebugP_log("SDL_HWA_IP_Pos_Test: failure on line no. %d \n", __LINE__);
            return (testStatus);
        }

        if (testStatus == SDL_APP_TEST_PASS)
        {
            SDL_HWA_setParityErrMaskUnmask(SDL_HWA_DMEM7,SDL_HWA_DISABLE_STS);
            if (1U==SDL_HWA_getParityErrMaskUnmask(SDL_HWA_DMEM7))
            {
                testStatus = SDL_APP_TEST_FAILED;
            }
        }
        if (testStatus != SDL_APP_TEST_PASS)
        {
            DebugP_log("SDL_HWA_IP_Pos_Test: failure on line no. %d \n", __LINE__);
            return (testStatus);
        }

        if (testStatus == SDL_APP_TEST_PASS)
        {
            SDL_HWA_setParityErrMaskUnmask(SDL_HWA_WINDOW_RAM,SDL_HWA_DISABLE_STS);
            if (1U==SDL_HWA_getParityErrMaskUnmask(SDL_HWA_WINDOW_RAM))
            {
                testStatus = SDL_APP_TEST_FAILED;
            }
        }
        if (testStatus != SDL_APP_TEST_PASS)
        {
            DebugP_log("SDL_HWA_IP_Pos_Test: failure on line no. %d \n", __LINE__);
            return (testStatus);
        }

        if (testStatus == SDL_APP_TEST_PASS)
        {
            SDL_HWA_setParityErrMaskUnmask(SDL_HWA_FSM_LOCKSTEP,SDL_HWA_DISABLE_STS);
            if (1U==SDL_HWA_getParityErrMaskUnmask(SDL_HWA_FSM_LOCKSTEP))
            {
                testStatus = SDL_APP_TEST_FAILED;
            }
        }
        if (testStatus != SDL_APP_TEST_PASS)
        {
            DebugP_log("SDL_HWA_IP_Pos_Test: failure on line no. %d \n", __LINE__);
            return (testStatus);
        }
        if (testStatus == SDL_APP_TEST_PASS)
        {
            SDL_HWA_setParityEnableDisable(SDL_HWA_DMEM_PARITY,SDL_HWA_ENABLE_STS);
            if (0U==SDL_HWA_getParityStatus(SDL_HWA_DMEM_PARITY))
            {
                testStatus = SDL_APP_TEST_FAILED;
            }
        }
        if (testStatus != SDL_APP_TEST_PASS)
        {
            DebugP_log("SDL_HWA_IP_Pos_Test: failure on line no. %d \n", __LINE__);
            return (testStatus);
        }

        if (testStatus == SDL_APP_TEST_PASS)
        {
            SDL_HWA_setParityEnableDisable(SDL_HWA_DMEM_WINDOW_RAM_PARITY,SDL_HWA_ENABLE_STS);
            if (0U==SDL_HWA_getParityStatus(SDL_HWA_DMEM_WINDOW_RAM_PARITY))
            {
                testStatus = SDL_APP_TEST_FAILED;
            }
        }
        if (testStatus != SDL_APP_TEST_PASS)
        {
            DebugP_log("SDL_HWA_IP_Pos_Test: failure on line no. %d \n", __LINE__);
            return (testStatus);
        }

        if (testStatus == SDL_APP_TEST_PASS)
        {
            SDL_HWA_setParityEnableDisable(SDL_HWA_DMEM_FSM_LOCKSTEP_INV_EN,SDL_HWA_ENABLE_STS);
            if (0U==SDL_HWA_getParityStatus(SDL_HWA_DMEM_FSM_LOCKSTEP_INV_EN))
            {
                testStatus = SDL_APP_TEST_FAILED;
            }
        }
        if (testStatus != SDL_APP_TEST_PASS)
        {
            DebugP_log("SDL_HWA_IP_Pos_Test: failure on line no. %d \n", __LINE__);
            return (testStatus);
        }

        if (testStatus == SDL_APP_TEST_PASS)
        {
            SDL_HWA_setParityEnableDisable(SDL_HWA_DMEM_FSM_LOCKSTEP_EN,SDL_HWA_ENABLE_STS);
            if (0U==SDL_HWA_getParityStatus(SDL_HWA_DMEM_FSM_LOCKSTEP_EN))
            {
                testStatus = SDL_APP_TEST_FAILED;
            }
        }
        if (testStatus != SDL_APP_TEST_PASS)
        {
            DebugP_log("SDL_HWA_IP_Pos_Test: failure on line no. %d \n", __LINE__);
            return (testStatus);
        }


        if (testStatus == SDL_APP_TEST_PASS)
        {
            SDL_HWA_setParityEnableDisable(SDL_HWA_DMEM_PARITY,SDL_HWA_DISABLE_STS);
            if (0U==SDL_HWA_getParityStatus(SDL_HWA_DMEM_PARITY))
            {
                testStatus = SDL_APP_TEST_FAILED;
            }
        }
        if (testStatus != SDL_APP_TEST_PASS)
        {
            DebugP_log("SDL_HWA_IP_Pos_Test: failure on line no. %d \n", __LINE__);
            return (testStatus);
        }

        if (testStatus == SDL_APP_TEST_PASS)
        {
            SDL_HWA_setParityEnableDisable(SDL_HWA_DMEM_WINDOW_RAM_PARITY,SDL_HWA_DISABLE_STS);
            if (0U==SDL_HWA_getParityStatus(SDL_HWA_DMEM_WINDOW_RAM_PARITY))
            {
                testStatus = SDL_APP_TEST_FAILED;
            }
        }
        if (testStatus != SDL_APP_TEST_PASS)
        {
            DebugP_log("SDL_HWA_IP_Pos_Test: failure on line no. %d \n", __LINE__);
            return (testStatus);
        }

        if (testStatus == SDL_APP_TEST_PASS)
        {
            SDL_HWA_setParityEnableDisable(SDL_HWA_DMEM_FSM_LOCKSTEP_INV_EN,SDL_HWA_DISABLE_STS);
            if (0U==SDL_HWA_getParityStatus(SDL_HWA_DMEM_FSM_LOCKSTEP_INV_EN))
            {
                testStatus = SDL_APP_TEST_FAILED;
            }
        }
        if (testStatus != SDL_APP_TEST_PASS)
        {
            DebugP_log("SDL_HWA_IP_Pos_Test: failure on line no. %d \n", __LINE__);
            return (testStatus);
        }

        if (testStatus == SDL_APP_TEST_PASS)
        {
            SDL_HWA_setParityEnableDisable(SDL_HWA_DMEM_FSM_LOCKSTEP_EN,SDL_HWA_DISABLE_STS);
            if (0U==SDL_HWA_getParityStatus(SDL_HWA_DMEM_FSM_LOCKSTEP_EN))
            {
                testStatus = SDL_APP_TEST_FAILED;
            }
        }
        if (testStatus != SDL_APP_TEST_PASS)
        {
            DebugP_log("SDL_HWA_IP_Pos_Test: failure on line no. %d \n", __LINE__);
            return (testStatus);
        }
    return (testStatus);
}

/*****************************************************************************/
int32_t sdl_ip_hwaNegTest(void)
{
    int32_t               testStatus = SDL_APP_TEST_PASS;
    uint32_t              baseAddr;

        if (testStatus == SDL_APP_TEST_PASS)
        {
            if (SDL_HWA_getMemblockBaseaddr(SDL_HWA_DMA0_MEM_ID,SDL_HWA_DMEM0,NULL) != SDL_EBADARGS)
            {
                testStatus = SDL_APP_TEST_FAILED;
            }
        }

        if (testStatus != SDL_APP_TEST_PASS)
        {
            DebugP_log("SDL _HWA_IP_Pos_Test: failure on line no. %d \n", __LINE__);
            return (testStatus);
        }

        if (testStatus == SDL_APP_TEST_PASS)
        {
            if (SDL_HWA_getMemblockBaseaddr(SDL_HWA_INVALID_ID ,SDL_HWA_DMEM0,&baseAddr) != SDL_EBADARGS)
            {
                testStatus = SDL_APP_TEST_FAILED;
            }
        }

        if (testStatus != SDL_APP_TEST_PASS)
        {
            DebugP_log("SDL _HWA_IP_Pos_Test: failure on line no. %d \n", __LINE__);
            return (testStatus);
        }

        if (testStatus == SDL_APP_TEST_PASS)
        {
            if (SDL_HWA_getMemblockBaseaddr(SDL_HWA_WINDOW_RAM_MEM_ID ,SDL_HWA_FSM_LOCKSTEP,&baseAddr) != SDL_EBADARGS)
            {
                testStatus = SDL_APP_TEST_FAILED;
            }
        }

        if (testStatus != SDL_APP_TEST_PASS)
        {
            DebugP_log("SDL _HWA_IP_Pos_Test: failure on line no. %d \n", __LINE__);
            return (testStatus);
        }
    return (testStatus);
}

/*****************************************************************************/
int32_t sdl_ip_hwaCodeCoverageTest(void)
{
    SDL_HWA_Status_s enSts;
    int32_t  testStatus = SDL_APP_TEST_PASS;
        SDL_HWA_memoryInitStart(SDL_HWA_INVALID);
        if (testStatus == SDL_APP_TEST_PASS)
        {
            if (0U!=SDL_HWA_memoryInitDone(SDL_HWA_FSM_LOCKSTEP))
            {
                testStatus = SDL_APP_TEST_FAILED;
            }
        }
        if (testStatus != SDL_APP_TEST_PASS)
        {
            DebugP_log("SDL _HWA_IP_CodeCoverage_Test: failure on line no. %d \n", __LINE__);
            return (testStatus);
        }
        if (testStatus == SDL_APP_TEST_PASS)
        {
            if (0U!=SDL_HWA_memoryInitStatus(SDL_HWA_FSM_LOCKSTEP))
            {
                testStatus = SDL_APP_TEST_FAILED;
            }
        }
        if (testStatus != SDL_APP_TEST_PASS)
        {
            DebugP_log("SDL _HWA_IP_CodeCoverage_Test: failure on line no. %d \n", __LINE__);
            return (testStatus);
        }
        SDL_HWA_setParityErrMaskUnmask(SDL_HWA_INVALID,SDL_HWA_ENABLE_STS );
        SDL_HWA_setParityErrMaskUnmask(SDL_HWA_INVALID,SDL_HWA_DISABLE_STS );
        if (testStatus == SDL_APP_TEST_PASS)
        {
            if (0U!=SDL_HWA_getParityErrMaskUnmask(SDL_HWA_INVALID))
            {
                testStatus = SDL_APP_TEST_FAILED;
            }
        }
        if (testStatus != SDL_APP_TEST_PASS)
        {
            DebugP_log("SDL _HWA_IP_CodeCoverage_Test: failure on line no. %d \n", __LINE__);
            return (testStatus);
        }
        SDL_HWA_setParityEnableDisable(SDL_HWA_INVALID, SDL_HWA_ENABLE_STS);
        SDL_HWA_setParityEnableDisable(SDL_HWA_INVALID, SDL_HWA_DISABLE_STS);
        if (testStatus == SDL_APP_TEST_PASS)
        {
            if (0U!=SDL_HWA_getParityErrMaskUnmask(SDL_HWA_INVALID))
            {
                testStatus = SDL_APP_TEST_FAILED;
            }
        }
        if (testStatus != SDL_APP_TEST_PASS)
        {
            DebugP_log("SDL _HWA_IP_CodeCoverage_Test: failure on line no. %d \n", __LINE__);
            return (testStatus);
        }
        SDL_HWA_setHwaEnableDisable(SDL_HWA_ENABLE_STS) ;
        SDL_HWA_setHwaEnableDisable(SDL_HWA_DISABLE_STS) ;
        SDL_HWA_getHwaEnableDisable(&enSts);
        SDL_HWA_clearErrStatus(SDL_HWA_DMEM0);
        SDL_HWA_clearErrStatus(SDL_HWA_DMEM1);
        SDL_HWA_clearErrStatus(SDL_HWA_DMEM2);
        SDL_HWA_clearErrStatus(SDL_HWA_DMEM3);
        SDL_HWA_clearErrStatus(SDL_HWA_DMEM4);
        SDL_HWA_clearErrStatus(SDL_HWA_DMEM5);
        SDL_HWA_clearErrStatus(SDL_HWA_DMEM6);
        SDL_HWA_clearErrStatus(SDL_HWA_DMEM7);
        SDL_HWA_clearErrStatus(SDL_HWA_WINDOW_RAM);
        SDL_HWA_clearErrStatus(SDL_HWA_FSM_LOCKSTEP);
    return (testStatus);
}
