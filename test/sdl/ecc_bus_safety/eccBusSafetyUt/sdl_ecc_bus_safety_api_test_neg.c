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
 *  \file     sdl_ecc_bus_safety_api_test_neg.c
 *
 *  \brief    This file contains DSS L3 API unit test code..
 *
 *  \details  Ecc Bus Safety unit tests
 **/

#include "ecc_bus_safety_main.h"

int32_t sdl_ecc_bus_safety_negTest(void)
{
    int32_t  testStatus = SDL_APP_TEST_PASS;
#if defined (SOC_AM263X) || defined (SOC_AM263PX) || defined (SOC_AM261X)
    SDL_ECC_BUS_SAFETY_staticRegs regs;
#endif
#if defined (SUBSYS_DSS)
SDL_ECC_BUS_SAFETY_staticRegs regs;
#endif
    uint32_t status =0 ;
    uint32_t writeData = 0x1234567U;

#if defined (SOC_AM273X) ||  (SOC_AWR294X)
#if defined (SUBSYS_MSS)
    SDL_ECC_BUS_SAFETY_staticRegs regs;
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ECC_BUS_SAFETY_MSS_secErrorClear(SDL_ECC_BUS_SAFETY_DAP_R232+20U) != SDL_EBADARGS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDL_ECC_BUS_SAFETY_Neg_Test: failure on line no. %d \r\n", __LINE__);
        return (testStatus);
    }
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ECC_BUS_SAFETY_MSS_getSecErrorStatus(SDL_ECC_BUS_SAFETY_DAP_R232+20U, &status) != SDL_EBADARGS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ECC_BUS_SAFETY_MSS_getSecErrorStatus(SDL_ECC_BUS_SAFETY_MSS_MBOX, NULL_PTR) != SDL_EBADARGS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDL_ECC_BUS_SAFETY_Neg_Test: failure on line no. %d \r\n", __LINE__);
        return (testStatus);
    }
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ECC_BUS_SAFETY_MSS_dedErrorClear(SDL_ECC_BUS_SAFETY_DAP_R232+20U) != SDL_EBADARGS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDL_ECC_BUS_SAFETY_Neg_Test: failure on line no. %d \r\n", __LINE__);
        return (testStatus);
    }
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ECC_BUS_SAFETY_MSS_getDedErrorStatus(SDL_ECC_BUS_SAFETY_DAP_R232+20U, &status) != SDL_EBADARGS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDL_ECC_BUS_SAFETY_Neg_Test: failure on line no. %d \r\n", __LINE__);
        return (testStatus);
    }
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ECC_BUS_SAFETY_MSS_getDedErrorStatus(SDL_ECC_BUS_SAFETY_MSS_MBOX, NULL_PTR) != SDL_EBADARGS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDL_ECC_BUS_SAFETY_Neg_Test: failure on line no. %d \r\n", __LINE__);
        return (testStatus);
    }
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ECC_BUS_SAFETY_MSS_redErrorClear(SDL_ECC_BUS_SAFETY_DAP_R232+20U) != SDL_EBADARGS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDL_ECC_BUS_SAFETY_Neg_Test: failure on line no. %d \r\n", __LINE__);
        return (testStatus);
    }
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ECC_BUS_SAFETY_MSS_getRedErrorStatus(SDL_ECC_BUS_SAFETY_DAP_R232+20U, &status) != SDL_EBADARGS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDL_ECC_BUS_SAFETY_Neg_Test: failure on line no. %d \r\n", __LINE__);
        return (testStatus);
    }
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ECC_BUS_SAFETY_MSS_getRedErrorStatus(SDL_ECC_BUS_SAFETY_MSS_MBOX, NULL_PTR) != SDL_EBADARGS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDL_ECC_BUS_SAFETY_Neg_Test: failure on line no. %d \r\n", __LINE__);
        return (testStatus);
    }
    /*SEC*/
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ECC_BUS_SAFETY_MSS_secExecute(SDL_ECC_BUS_SAFETY_DAP_R232+20U,0U,writeData) != SDL_EBADARGS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDL_ECC_BUS_SAFETY_Neg_Test: failure on line no. %d \r\n", __LINE__);
        return (testStatus);
    }
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ECC_BUS_SAFETY_MSS_secExecute(SDL_ECC_BUS_SAFETY_DAP_R232+20U,0U,0) != SDL_EBADARGS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDL_ECC_BUS_SAFETY_Neg_Test: failure on line no. %d \r\n", __LINE__);
        return (testStatus);
    }
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ECC_BUS_SAFETY_MSS_secExecute(SDL_ECC_BUS_SAFETY_MSS_QSPI,0U, writeData) != SDL_EBADARGS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDL_ECC_BUS_SAFETY_Neg_Test: failure on line no. %d \r\n", __LINE__);
        return (testStatus);
    }
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ECC_BUS_SAFETY_MSS_secExecute(SDL_ECC_BUS_SAFETY_MSS_QSPI,0XCA000000U, writeData) != SDL_EBADARGS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDL_ECC_BUS_SAFETY_Neg_Test: failure on line no. %d \r\n", __LINE__);
        return (testStatus);
    }
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ECC_BUS_SAFETY_MSS_secExecute(SDL_ECC_BUS_SAFETY_MSS_TPTC_A1_RD,0U, 0) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDL_ECC_BUS_SAFETY_Neg_Test: failure on line no. %d \r\n", __LINE__);
        return (testStatus);
    }
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ECC_BUS_SAFETY_MSS_secExecute(SDL_ECC_BUS_SAFETY_MSS_MCRC,SDL_MSS_MCRC_U_BASE, 0) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDL_ECC_BUS_SAFETY_Neg_Test: failure on line no. %d \r\n", __LINE__);
        return (testStatus);
    }
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ECC_BUS_SAFETY_MSS_secExecute(SDL_ECC_BUS_SAFETY_MSS_TPTC_A1_RD,0U, 0) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDL_ECC_BUS_SAFETY_Neg_Test: failure on line no. %d \r\n", __LINE__);
        return (testStatus);
    }
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ECC_BUS_SAFETY_MSS_secExecute(SDL_ECC_BUS_SAFETY_MSS_MBOX,SDL_MSS_MBOX_U_BASE, 0) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDL_ECC_BUS_SAFETY_Neg_Test: failure on line no. %d \r\n", __LINE__);
        return (testStatus);
    }
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ECC_BUS_SAFETY_MSS_secExecute(SDL_ECC_BUS_SAFETY_MSS_CR5A_AHB,0U,0U) != SDL_EBADARGS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDL_ECC_BUS_SAFETY_Neg_Test: failure on line no. %d \r\n", __LINE__);
        return (testStatus);
    }
    /* DED*/
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ECC_BUS_SAFETY_MSS_dedExecute(SDL_ECC_BUS_SAFETY_MSS_MBOX,SDL_MSS_MBOX_U_BASE, 0) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDL_ECC_BUS_SAFETY_Neg_Test: failure on line no. %d \r\n", __LINE__);
        return (testStatus);
    }
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ECC_BUS_SAFETY_MSS_dedExecute(SDL_ECC_BUS_SAFETY_MSS_MCRC,SDL_MSS_MCRC_U_BASE, 0) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDL_ECC_BUS_SAFETY_Neg_Test: failure on line no. %d \r\n", __LINE__);
        return (testStatus);
    }
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ECC_BUS_SAFETY_MSS_dedExecute(SDL_ECC_BUS_SAFETY_MSS_CR5A_AHB,0U,0U) != SDL_EBADARGS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDL_ECC_BUS_SAFETY_Neg_Test: failure on line no. %d \r\n", __LINE__);
        return (testStatus);
    }
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ECC_BUS_SAFETY_MSS_dedExecute(SDL_ECC_BUS_SAFETY_DAP_R232+20U,0U,writeData) != SDL_EBADARGS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDL_ECC_BUS_SAFETY_Neg_Test: failure on line no. %d \r\n", __LINE__);
        return (testStatus);
    }
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ECC_BUS_SAFETY_MSS_dedExecute(SDL_ECC_BUS_SAFETY_DAP_R232+20U,0U,0) != SDL_EBADARGS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDL_ECC_BUS_SAFETY_Neg_Test: failure on line no. %d \r\n", __LINE__);
        return (testStatus);
    }
    if (testStatus == SDL_APP_TEST_PASS)
    {
    if (SDL_ECC_BUS_SAFETY_MSS_dedExecute(SDL_ECC_BUS_SAFETY_MSS_QSPI,0U, writeData) != SDL_EBADARGS)
    {
        testStatus = SDL_APP_TEST_FAILED;
    }
    }
    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDL_ECC_BUS_SAFETY_Neg_Test: failure on line no. %d \r\n", __LINE__);
        return (testStatus);
    }
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ECC_BUS_SAFETY_MSS_dedExecute(SDL_ECC_BUS_SAFETY_MSS_QSPI,0XCA000000U, writeData) != SDL_EBADARGS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDL_ECC_BUS_SAFETY_Neg_Test: failure on line no. %d \r\n", __LINE__);
        return (testStatus);
    }
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ECC_BUS_SAFETY_MSS_dedExecute(SDL_ECC_BUS_SAFETY_MSS_TPTC_A1_RD,0U, 0) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDL_ECC_BUS_SAFETY_Neg_Test: failure on line no. %d \r\n", __LINE__);
        return (testStatus);
    }
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ECC_BUS_SAFETY_MSS_dedExecute(SDL_ECC_BUS_SAFETY_MSS_TPTC_A1_RD,0U, 0) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDL_ECC_BUS_SAFETY_Neg_Test: failure on line no. %d \r\n", __LINE__);
        return (testStatus);
    }
    /* RED */
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ECC_BUS_SAFETY_MSS_redExecute(SDL_ECC_BUS_SAFETY_DAP_R232+20U,SDL_ECC_BUS_SAFETY_FI_GLOBAL_SAFE, SDL_ECC_BUS_SAFETY_MAIN_CMD_INTERFACE ) != SDL_EBADARGS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDL_ECC_BUS_SAFETY_Neg_Test: failure on line no. %d \r\n", __LINE__);
        return (testStatus);
    }
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ECC_BUS_SAFETY_MSS_redExecute(SDL_ECC_BUS_SAFETY_MSS_TPTC_A1_RD,SDL_ECC_BUS_SAFETY_FI_INVALID, SDL_ECC_BUS_SAFETY_MAIN_CMD_INTERFACE ) != SDL_EBADARGS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDL_ECC_BUS_SAFETY_Neg_Test: failure on line no. %d \r\n", __LINE__);
        return (testStatus);
    }
    /* SDL_ECC_BUS_SAFETY_MSS_readStaticRegs test with invalid parameter */
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ECC_BUS_SAFETY_MSS_readStaticRegs(SDL_ECC_BUS_SAFETY_DAP_R232, NULL_PTR) != SDL_EBADARGS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDL_ECC_BUS_SAFETY_Neg_Test: failure on line no. %d \r\n", __LINE__);
        return (testStatus);
    }
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ECC_BUS_SAFETY_MSS_readStaticRegs(SDL_ECC_BUS_SAFETY_DAP_R232+10U,&regs ) != SDL_EBADARGS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDL_ECC_BUS_SAFETY_Neg_Test: failure on line no. %d \r\n", __LINE__);
        return (testStatus);
    }
#endif
#endif

#if defined (SOC_AM263X) || defined (SOC_AM263PX) || defined (SOC_AM261X)

    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ECC_BUS_SAFETY_MSS_secErrorClear(SDL_ECC_BUS_SAFETY_DAP+20U) != SDL_EBADARGS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDL_ECC_BUS_SAFETY_Neg_Test: failure on line no. %d \r\n", __LINE__);
        return (testStatus);
    }
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ECC_BUS_SAFETY_MSS_getSecErrorStatus(SDL_ECC_BUS_SAFETY_DAP+20U, &status) != SDL_EBADARGS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDL_ECC_BUS_SAFETY_Neg_Test: failure on line no. %d \r\n", __LINE__);
        return (testStatus);
    }
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ECC_BUS_SAFETY_MSS_getSecErrorStatus(SDL_ECC_BUS_SAFETY_MSS_MBOX, NULL_PTR) != SDL_EBADARGS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDL_ECC_BUS_SAFETY_Neg_Test: failure on line no. %d \r\n", __LINE__);
        return (testStatus);
    }
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ECC_BUS_SAFETY_MSS_dedErrorClear(SDL_ECC_BUS_SAFETY_DAP+20U) != SDL_EBADARGS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDL_ECC_BUS_SAFETY_Neg_Test: failure on line no. %d \r\n", __LINE__);
        return (testStatus);
    }
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ECC_BUS_SAFETY_MSS_getDedErrorStatus(SDL_ECC_BUS_SAFETY_DAP+20U, &status) != SDL_EBADARGS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDL_ECC_BUS_SAFETY_Neg_Test: failure on line no. %d \r\n", __LINE__);
        return (testStatus);
    }
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ECC_BUS_SAFETY_MSS_getDedErrorStatus(SDL_ECC_BUS_SAFETY_MSS_MBOX, NULL_PTR) != SDL_EBADARGS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDL_ECC_BUS_SAFETY_Neg_Test: failure on line no. %d \r\n", __LINE__);
        return (testStatus);
    }
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ECC_BUS_SAFETY_MSS_redErrorClear(SDL_ECC_BUS_SAFETY_DAP+20U) != SDL_EBADARGS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDL_ECC_BUS_SAFETY_Neg_Test: failure on line no. %d \r\n", __LINE__);
        return (testStatus);
    }
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ECC_BUS_SAFETY_MSS_getRedErrorStatus(SDL_ECC_BUS_SAFETY_DAP+2U, &status) != SDL_EBADARGS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ECC_BUS_SAFETY_MSS_dedExecute(SDL_ECC_BUS_SAFETY_MSS_PERI_VBUSP,0U,0U) != SDL_EBADARGS)
        if (SDL_ECC_BUS_SAFETY_MSS_getRedErrorStatus(SDL_ECC_BUS_SAFETY_DAP+20U, &status) != SDL_EBADARGS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDL_ECC_BUS_SAFETY_Neg_Test: failure on line no. %d \r\n", __LINE__);
        return (testStatus);
    }
    /* SDL_ECC_BUS_SAFETY_MSS_readStaticRegs test with invalid parameter */
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ECC_BUS_SAFETY_MSS_readStaticRegs(SDL_ECC_BUS_SAFETY_MSS_STM_STIM+10U,&regs ) != SDL_EBADARGS)
        if (SDL_ECC_BUS_SAFETY_MSS_getRedErrorStatus(SDL_ECC_BUS_SAFETY_MSS_MBOX, NULL_PTR) != SDL_EBADARGS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDL_ECC_BUS_SAFETY_Neg_Test: failure on line no. %d \r\n", __LINE__);
        return (testStatus);
    }
    /* SDL_ECC_BUS_SAFETY_MSS_readStaticRegs test with invalid parameter */
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ECC_BUS_SAFETY_MSS_readStaticRegs(SDL_ECC_BUS_SAFETY_DAP, NULL_PTR) != SDL_EBADARGS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDL_ECC_BUS_SAFETY_Neg_Test: failure on line no. %d \r\n", __LINE__);
        return (testStatus);
    }

/*SEC*/
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ECC_BUS_SAFETY_MSS_secExecute(SDL_ECC_BUS_SAFETY_DAP+20U,0U,writeData) != SDL_EBADARGS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDL_ECC_BUS_SAFETY_Neg_Test: failure on line no. %d \r\n", __LINE__);
        return (testStatus);
    }
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ECC_BUS_SAFETY_MSS_secExecute(SDL_ECC_BUS_SAFETY_MSS_QSPI,0XCA000000U, writeData) != SDL_EBADARGS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDL_ECC_BUS_SAFETY_Neg_Test: failure on line no. %d \r\n", __LINE__);
        return (testStatus);
    }
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ECC_BUS_SAFETY_MSS_secExecute(SDL_ECC_BUS_SAFETY_MSS_QSPI,0U, writeData) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDL_ECC_BUS_SAFETY_Neg_Test: failure on line no. %d \r\n", __LINE__);
        return (testStatus);
    }
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ECC_BUS_SAFETY_MSS_secExecute(SDL_ECC_BUS_SAFETY_MSS_TPTC_A1_RD,0U, 0) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDL_ECC_BUS_SAFETY_Neg_Test: failure on line no. %d \r\n", __LINE__);
        return (testStatus);
    }
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ECC_BUS_SAFETY_MSS_secExecute(SDL_ECC_BUS_SAFETY_MSS_MBOX,SDL_MBOX_SRAM_U_BASE, 0) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDL_ECC_BUS_SAFETY_Neg_Test: failure on line no. %d \r\n", __LINE__);
        return (testStatus);
    }
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ECC_BUS_SAFETY_MSS_secExecute(SDL_ECC_BUS_SAFETY_DAP+20U,0U,0) != SDL_EBADARGS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDL_ECC_BUS_SAFETY_Neg_Test: failure on line no. %d \r\n", __LINE__);
        return (testStatus);
    }
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ECC_BUS_SAFETY_MSS_secExecute(SDL_ECC_BUS_SAFETY_MSS_MAIN_VBUSP,0U,0U) != SDL_EBADARGS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDL_ECC_BUS_SAFETY_Neg_Test: failure on line no. %d \r\n", __LINE__);
        return (testStatus);
    }

    /* DED*/
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ECC_BUS_SAFETY_MSS_dedExecute(SDL_ECC_BUS_SAFETY_MSS_MAIN_VBUSP,0U,0U) != SDL_EBADARGS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDL_ECC_BUS_SAFETY_Neg_Test: failure on line no. %d \r\n", __LINE__);
        return (testStatus);
    }
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ECC_BUS_SAFETY_MSS_dedExecute(SDL_ECC_BUS_SAFETY_DAP+20U,0U,0) != SDL_EBADARGS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDL_ECC_BUS_SAFETY_Neg_Test: failure on line no. %d \r\n", __LINE__);
        return (testStatus);
    }
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ECC_BUS_SAFETY_MSS_dedExecute(SDL_ECC_BUS_SAFETY_DAP+20U,0U,writeData) != SDL_EBADARGS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDL_ECC_BUS_SAFETY_Neg_Test: failure on line no. %d \r\n", __LINE__);
        return (testStatus);
    }
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ECC_BUS_SAFETY_MSS_dedExecute(SDL_ECC_BUS_SAFETY_MSS_QSPI,0XCA000000U, writeData) != SDL_EBADARGS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDL_ECC_BUS_SAFETY_Neg_Test: failure on line no. %d \r\n", __LINE__);
        return (testStatus);
    }

    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ECC_BUS_SAFETY_MSS_dedExecute(SDL_ECC_BUS_SAFETY_MSS_QSPI,0U, writeData) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDL_ECC_BUS_SAFETY_Neg_Test: failure on line no. %d \r\n", __LINE__);
        return (testStatus);
    }
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ECC_BUS_SAFETY_MSS_dedExecute(SDL_ECC_BUS_SAFETY_MSS_TPTC_A1_RD,0U, 0) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDL_ECC_BUS_SAFETY_Neg_Test: failure on line no. %d \r\n", __LINE__);
        return (testStatus);
    }

    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ECC_BUS_SAFETY_MSS_dedExecute(SDL_ECC_BUS_SAFETY_MSS_MBOX,SDL_MBOX_SRAM_U_BASE, 0) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDL_ECC_BUS_SAFETY_Neg_Test: failure on line no. %d \r\n", __LINE__);
        return (testStatus);
    }

/* RED */
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ECC_BUS_SAFETY_MSS_redExecute(SDL_ECC_BUS_SAFETY_DAP+20U,SDL_ECC_BUS_SAFETY_FI_GLOBAL_SAFE, SDL_ECC_BUS_SAFETY_MAIN_CMD_INTERFACE ) != SDL_EBADARGS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDL_ECC_BUS_SAFETY_Neg_Test: failure on line no. %d \r\n", __LINE__);
        return (testStatus);
    }

    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ECC_BUS_SAFETY_MSS_redExecute(SDL_ECC_BUS_SAFETY_MSS_TPTC_A1_RD,SDL_ECC_BUS_SAFETY_FI_INVALID, SDL_ECC_BUS_SAFETY_MAIN_CMD_INTERFACE ) != SDL_EBADARGS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDL_ECC_BUS_SAFETY_Neg_Test: failure on line no. %d \r\n", __LINE__);
        return (testStatus);
    }

#endif



#if defined (SOC_AM273X) ||  (SOC_AWR294X)
#if defined (SUBSYS_DSS)
    /* Test cases for SDL_ECC_BUS_SAFETY_DSS_secExecute */
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ECC_BUS_SAFETY_DSS_secExecute(SDL_ECC_BUS_SAFETY_DSS_TPTC_A0_WR,0U,writeData) != SDL_EBADARGS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDL_ECC_BUS_SAFETY_Neg_Test: failure on line no. %d \r\n", __LINE__);
        return (testStatus);
    }
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ECC_BUS_SAFETY_DSS_secExecute(SDL_ECC_BUS_SAFETY_DSS_TPTC_C5_WR,0U,writeData) != SDL_EBADARGS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDL_ECC_BUS_SAFETY_Neg_Test: failure on line no. %d \r\n", __LINE__);
        return (testStatus);
    }
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ECC_BUS_SAFETY_DSS_secExecute(SDL_ECC_BUS_SAFETY_DSS_HWA_DMA0,SDL_DSS_HWA_DMA0_U_BASE-100U ,writeData ) != SDL_EBADARGS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDL_ECC_BUS_SAFETY_Neg_Test: failure on line no. %d \r\n", __LINE__);
        return (testStatus);
    }
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ECC_BUS_SAFETY_DSS_secExecute(SDL_ECC_BUS_SAFETY_DSS_HWA_DMA0,SDL_DSS_HWA_DMA0_U_BASE_END+100U ,writeData ) != SDL_EBADARGS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDL_ECC_BUS_SAFETY_Neg_Test: failure on line no. %d \r\n", __LINE__);
        return (testStatus);
    }
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ECC_BUS_SAFETY_DSS_secExecute(SDL_ECC_BUS_SAFETY_DSS_MBOX+10U,SDL_DSS_HWA_DMA0_U_BASE_END+100U ,writeData ) != SDL_EBADARGS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDL_ECC_BUS_SAFETY_Neg_Test: failure on line no. %d \r\n", __LINE__);
        return (testStatus);
    }
    /* Test cases for SDL_ECC_BUS_SAFETY_DSS_dedExecute */
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ECC_BUS_SAFETY_DSS_dedExecute(SDL_ECC_BUS_SAFETY_DSS_TPTC_A0_WR,0U,writeData) != SDL_EBADARGS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDL_ECC_BUS_SAFETY_Neg_Test: failure on line no. %d \r\n", __LINE__);
        return (testStatus);
    }
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ECC_BUS_SAFETY_DSS_dedExecute(SDL_ECC_BUS_SAFETY_DSS_TPTC_C5_WR,0U,writeData) != SDL_EBADARGS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDL_ECC_BUS_SAFETY_Neg_Test: failure on line no. %d \r\n", __LINE__);
        return (testStatus);
    }
    /* DSS_HWA_DMA0 DED test with invalid parameeter */
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ECC_BUS_SAFETY_DSS_dedExecute(SDL_ECC_BUS_SAFETY_DSS_HWA_DMA0,SDL_DSS_HWA_DMA0_U_BASE-100U ,writeData ) != SDL_EBADARGS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDL_ECC_BUS_SAFETY_Neg_Test: failure on line no. %d \r\n", __LINE__);
        return (testStatus);
    }
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ECC_BUS_SAFETY_DSS_dedExecute(SDL_ECC_BUS_SAFETY_DSS_HWA_DMA0,SDL_DSS_HWA_DMA0_U_BASE_END+100U ,writeData ) != SDL_EBADARGS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDL_ECC_BUS_SAFETY_Neg_Test: failure on line no. %d \r\n", __LINE__);
        return (testStatus);
    }
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ECC_BUS_SAFETY_DSS_dedExecute(SDL_ECC_BUS_SAFETY_DSS_MBOX+10U,SDL_DSS_HWA_DMA0_U_BASE_END+100U ,writeData ) != SDL_EBADARGS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDL_ECC_BUS_SAFETY_Neg_Test: failure on line no. %d \r\n", __LINE__);
        return (testStatus);
    }
    /* Test cases for SDL_ECC_BUS_SAFETY_DSS_redExecute */
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ECC_BUS_SAFETY_DSS_redExecute(SDL_ECC_BUS_SAFETY_DSS_TPTC_A1_RD,SDL_ECC_BUS_SAFETY_FI_INVALID, SDL_ECC_BUS_SAFETY_MAIN_CMD_INTERFACE ) != SDL_EBADARGS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDL_ECC_BUS_SAFETY_Neg_Test: failure on line no. %d \r\n", __LINE__);
        return (testStatus);
    }

    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ECC_BUS_SAFETY_DSS_redExecute(SDL_ECC_BUS_SAFETY_DSS_TPTC_A1_RD,SDL_ECC_BUS_SAFETY_FI_GLOBAL_SAFE, SDL_ECC_BUS_SAFETY_FI_TYPE_INVALID ) != SDL_EBADARGS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDL_ECC_BUS_SAFETY_Neg_Test: failure on line no. %d \r\n", __LINE__);
        return (testStatus);
    }
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ECC_BUS_SAFETY_DSS_redExecute(SDL_ECC_BUS_SAFETY_DSS_TPTC_A1_RD,SDL_ECC_BUS_SAFETY_FI_INVALID, SDL_ECC_BUS_SAFETY_FI_TYPE_INVALID ) != SDL_EBADARGS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDL_ECC_BUS_SAFETY_Neg_Test: failure on line no. %d \r\n", __LINE__);
        return (testStatus);
    }
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ECC_BUS_SAFETY_DSS_redExecute(SDL_ECC_BUS_SAFETY_DSS_MBOX+10U,SDL_ECC_BUS_SAFETY_FI_INVALID, SDL_ECC_BUS_SAFETY_FI_TYPE_INVALID ) != SDL_EBADARGS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDL_ECC_BUS_SAFETY_Neg_Test: failure on line no. %d \r\n", __LINE__);
        return (testStatus);
    }
    /* Test cases for SDL_ECC_BUS_SAFETY_DSS_secErrorClear */
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if(SDL_ECC_BUS_SAFETY_DSS_secErrorClear(SDL_ECC_BUS_SAFETY_DSS_MBOX+10U) != SDL_EBADARGS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDL_ECC_BUS_SAFETY_Neg_Test: failure on line no. %d \r\n", __LINE__);
    }
    /* Test cases for SDL_ECC_BUS_SAFETY_DSS_getSecErrorStatus */
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ECC_BUS_SAFETY_DSS_getSecErrorStatus(SDL_ECC_BUS_SAFETY_DSS_L3_BANKA, NULL_PTR) != SDL_EBADARGS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDL_ECC_BUS_SAFETY_Neg_Test: failure on line no. %d \r\n", __LINE__);
        return (testStatus);
    }
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ECC_BUS_SAFETY_DSS_getSecErrorStatus(SDL_ECC_BUS_SAFETY_DSS_MBOX+10U, &status) != SDL_EBADARGS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }

    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ECC_BUS_SAFETY_DSS_getSecErrorStatus(SDL_ECC_BUS_SAFETY_DSS_MBOX+10U, NULL_PTR) != SDL_EBADARGS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDL_ECC_BUS_SAFETY_Neg_Test: failure on line no. %d \r\n", __LINE__);
        return (testStatus);
    }
    /* Test cases for SDL_ECC_BUS_SAFETY_DSS_dedErrorClear*/
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ECC_BUS_SAFETY_DSS_dedErrorClear(SDL_ECC_BUS_SAFETY_DSS_MBOX+10U) != SDL_EBADARGS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDL_ECC_BUS_SAFETY_Neg_Test: failure on line no. %d \r\n", __LINE__);
        return (testStatus);
    }
    /* Test cases for SDL_ECC_BUS_SAFETY_DSS_getDedErrorStatus */
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ECC_BUS_SAFETY_DSS_getDedErrorStatus(SDL_ECC_BUS_SAFETY_DSS_L3_BANKA, NULL_PTR) != SDL_EBADARGS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDL_ECC_BUS_SAFETY_Neg_Test: failure on line no. %d \r\n", __LINE__);
        return (testStatus);
    }
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ECC_BUS_SAFETY_DSS_getDedErrorStatus(SDL_ECC_BUS_SAFETY_DSS_MBOX+10U, &status) != SDL_EBADARGS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDL_ECC_BUS_SAFETY_Neg_Test: failure on line no. %d \r\n", __LINE__);
        return (testStatus);
    }
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ECC_BUS_SAFETY_DSS_getDedErrorStatus(SDL_ECC_BUS_SAFETY_DSS_MBOX+10U, NULL_PTR) != SDL_EBADARGS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDL_ECC_BUS_SAFETY_Neg_Test: failure on line no. %d \r\n", __LINE__);
        return (testStatus);
    }
    /* Test cases for SDL_ECC_BUS_SAFETY_DSS_redErrorClear */
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ECC_BUS_SAFETY_DSS_redErrorClear(SDL_ECC_BUS_SAFETY_DSS_MBOX+10U) != SDL_EBADARGS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDL_ECC_BUS_SAFETY_Neg_Test: failure on line no. %d \r\n", __LINE__);
        return (testStatus);
    }
    /* Test cases for SDL_ECC_BUS_SAFETY_DSS_getRedErrorStatus */
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ECC_BUS_SAFETY_DSS_getRedErrorStatus(SDL_ECC_BUS_SAFETY_DSS_L3_BANKA, NULL_PTR) != SDL_EBADARGS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDL_ECC_BUS_SAFETY_Neg_Test: failure on line no. %d \r\n", __LINE__);
        return (testStatus);
    }
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ECC_BUS_SAFETY_DSS_getRedErrorStatus(SDL_ECC_BUS_SAFETY_DSS_MBOX+10U, NULL_PTR) != SDL_EBADARGS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDL_ECC_BUS_SAFETY_Neg_Test: failure on line no. %d \r\n", __LINE__);
        return (testStatus);
    }
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ECC_BUS_SAFETY_DSS_getRedErrorStatus(SDL_ECC_BUS_SAFETY_DSS_MBOX+10U, &status) != SDL_EBADARGS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDL_ECC_BUS_SAFETY_Neg_Test: failure on line no. %d \r\n", __LINE__);
        return (testStatus);
    }
    /* SDL_ECC_BUS_SAFETY_DSS_readStaticRegs test with invalid parameeter */
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ECC_BUS_SAFETY_DSS_readStaticRegs(SDL_ECC_BUS_SAFETY_DSS_HWA_DMA0,NULL_PTR ) != SDL_EBADARGS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDL_ECC_BUS_SAFETY_Neg_Test: failure on line no. %d \r\n", __LINE__);
        return (testStatus);
    }
    /* SDL_ECC_BUS_SAFETY_DSS_readStaticRegs test with invalid parameeter */
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ECC_BUS_SAFETY_DSS_readStaticRegs(SDL_ECC_BUS_SAFETY_DSS_MBOX+10U,&regs ) != SDL_EBADARGS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDL_ECC_BUS_SAFETY_Neg_Test: failure on line no. %d \r\n", __LINE__);
        return (testStatus);
    }
    /* SDL_ECC_BUS_SAFETY_DSS_readStaticRegs test with invalid parameeter */
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ECC_BUS_SAFETY_DSS_readStaticRegs(SDL_ECC_BUS_SAFETY_DSS_MBOX+10U,NULL_PTR ) != SDL_EBADARGS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDL_ECC_BUS_SAFETY_Neg_Test: failure on line no. %d \r\n", __LINE__);
        return (testStatus);
    }
#endif
#endif
#if defined (SOC_AWR294X) || defined (SOC_AM273X)
#if defined (SUBSYS_DSS)
    writeData = status;
    status = testStatus;
#endif
#endif
    return (testStatus);
}
