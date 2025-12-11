/*
 *   Copyright (C) Texas Instruments Incorporated 2025
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
 *  \file     ecc_ip_test_err.c
 *
 *  \brief    This file contains ECC Error module test code for R5 core.
 *
 *  \details  ECC Error module tests
 **/

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */
#include <stdint.h>
#include <sdl/include/sdl_types.h>
#include <kernel/dpl/DebugP.h>
#include <sdl/sdl_ecc.h>
#include <sdl/ecc/sdl_ip_ecc.h>
#include <sdl/ecc/V0/sdlr_ecc_ram.h>
#include "ecc_test_main.h"
#if defined(SOC_AM64X) || defined(SOC_AM243X)
#include <sdl/include/am64x_am243x/sdlr_soc_baseaddress.h>
#include <sdl/ecc/soc/am64x_am243x/sdl_ecc_soc.h>
#endif

/* ========================================================================== */
/*                                Macros                                      */
/* ========================================================================== */

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */

/* ========================================================================== */
/*                 Internal Function Declarations                             */
/* ========================================================================== */

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

static int32_t ECC_errNegativeTest(void)
{
    #if defined (R5F_CORE)
    SDL_ecc_aggrRegs *pEccAggrRegs = ((SDL_ecc_aggrRegs *)((uintptr_t)SDL_R5FSS0_CORE0_ECC_AGGR_BASE));
    #endif
    #if defined (M4F_CORE)
    SDL_ecc_aggrRegs *pEccAggrRegs = ((SDL_ecc_aggrRegs *)((uintptr_t)SDL_MCU_M4FSS0_ECC_AGGR_BASE));
    #endif
    SDL_Ecc_AggrErrorInfo eccErrorInfo;
    int32_t testStatus = SDL_APP_TEST_PASS;
    uint32_t val;
    bool isPend;

    /* SDL_ecc_aggrReadEccRamWrapRevReg negative test */
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ecc_aggrReadEccRamWrapRevReg(NULL, 0U, &val) != SDL_EBADARGS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ecc_aggrReadEccRamWrapRevReg(pEccAggrRegs, 0xFFFFU, &val) != SDL_EBADARGS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ecc_aggrReadEccRamWrapRevReg(pEccAggrRegs, 0U, NULL) != SDL_EBADARGS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("sdlEccAggr_negTest: failure on line no. %d \r\n", __LINE__);
    }

    /* SDL_ecc_aggrReadEccRamErrCtrlReg negative test */
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ecc_aggrReadEccRamErrCtrlReg(NULL, 0U, 0U, &val) != SDL_EBADARGS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ecc_aggrReadEccRamErrCtrlReg(pEccAggrRegs, 0xFFFFU, 0U, &val) != SDL_EBADARGS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ecc_aggrReadEccRamErrCtrlReg(pEccAggrRegs, 0U, 0xFFFFU, &val) != SDL_EBADARGS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ecc_aggrReadEccRamErrCtrlReg(pEccAggrRegs, 0U, 0U, NULL) != SDL_EBADARGS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("sdlEccAggr_negTest: failure in SDL_ecc_aggrReadEccRamErrCtrlReg on line no. %d \r\n", __LINE__);
    }

    /* SDL_ecc_aggrWriteEccRamErrStatReg negative test */
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ecc_aggrWriteEccRamErrStatReg(NULL, 0U, 0U, 0U) != SDL_EBADARGS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ecc_aggrWriteEccRamErrStatReg(pEccAggrRegs, 0xFFFFU, 0U, 0U) != SDL_EBADARGS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ecc_aggrWriteEccRamErrStatReg(pEccAggrRegs, 0U, 0xFFFFU, 0U) != SDL_EBADARGS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("sdlEccAggr_negTest: failure on line no. %d \r\n", __LINE__);
    }

    /* SDL_ecc_aggrVerifyConfigEccRam negative test */
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ecc_aggrVerifyConfigEccRam(NULL, 0U, 0U, 0U, 0U) != SDL_EBADARGS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ecc_aggrVerifyConfigEccRam(pEccAggrRegs, 0xFFFFU, 0U, 0U, 0U) != SDL_EBADARGS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus == SDL_APP_TEST_PASS)
    {
        /*To get EFAIL, passing wrong arguments*/
        if (SDL_ecc_aggrVerifyConfigEccRam(pEccAggrRegs, 10U, (bool)true, (bool)true, (bool)true) != SDL_EFAIL)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("sdlEccAggr_negTest: failure in SDL_ecc_aggrVerifyConfigEccRam on line no. %d \r\n", __LINE__);
    }

    /* SDL_ecc_aggrForceEccRamError negative test */
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ecc_aggrForceEccRamError(NULL, 0U, &eccErrorInfo) != SDL_EBADARGS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ecc_aggrForceEccRamError(pEccAggrRegs, 0xFFFFU, &eccErrorInfo) != SDL_EBADARGS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ecc_aggrForceEccRamError(pEccAggrRegs, 0U, NULL) != SDL_EBADARGS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }

    /* SDL_ecc_aggrClrEccRamIntrPending negative test */
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ecc_aggrClrEccRamIntrPending(NULL, 0U, SDL_ECC_AGGR_INTR_SRC_SINGLE_BIT) != SDL_EBADARGS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("sdlEccAggr_negTest: failure on line no. %d \r\n", __LINE__);
    }
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ecc_aggrClrEccRamIntrPending(pEccAggrRegs, 0xFFFFU, SDL_ECC_AGGR_INTR_SRC_SINGLE_BIT) != SDL_EBADARGS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("sdlEccAggr_negTest: failure on line no. %d \r\n", __LINE__);
    }
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ecc_aggrClrEccRamIntrPending(pEccAggrRegs, 0U, SDL_ECC_AGGR_INTR_SRC_INVALID) != SDL_EBADARGS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("sdlEccAggr_negTest: failure on line no. %d \r\n", __LINE__);
    }

    /* SDL_ecc_aggrIsIntrPending negative test */
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ecc_aggrIsIntrPending(NULL, 0U, SDL_ECC_AGGR_INTR_SRC_SINGLE_BIT, &isPend) != SDL_EBADARGS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ecc_aggrIsIntrPending(pEccAggrRegs, 0xFFFFU, SDL_ECC_AGGR_INTR_SRC_SINGLE_BIT, &isPend) != SDL_EBADARGS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ecc_aggrIsIntrPending(pEccAggrRegs, 0U, SDL_ECC_AGGR_INTR_SRC_INVALID, &isPend) != SDL_EBADARGS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ecc_aggrIsIntrPending(pEccAggrRegs, 0U, SDL_ECC_AGGR_INTR_SRC_SINGLE_BIT, NULL) != SDL_EBADARGS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("sdlEccAggr_negTest: failure in SDL_ecc_aggrIsIntrPending on line no. %d \r\n", __LINE__);
    }

    /* SDL_ecc_aggrIsAnyIntrPending negative test */
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ecc_aggrIsAnyIntrPending(NULL, 0U, &isPend) != SDL_EBADARGS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ecc_aggrIsAnyIntrPending(pEccAggrRegs, 0xFFFFU, &isPend) != SDL_EBADARGS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ecc_aggrIsAnyIntrPending(pEccAggrRegs, 0U, NULL) != SDL_EBADARGS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("sdlEccAggr_negTest: failure on line no. %d \r\n", __LINE__);
    }

    /* SDL_ecc_aggrEnableAllIntr negative test */
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ecc_aggrEnableAllIntr(NULL, 0U) != SDL_EBADARGS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ecc_aggrEnableAllIntr(pEccAggrRegs, 0xFFFFU) != SDL_EBADARGS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("sdlEccAggr_negTest: failure on line no. %d \r\n", __LINE__);
    }

    /* SDL_ecc_aggrDisableAllIntr negative test */
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ecc_aggrDisableAllIntr(NULL, 0U) != SDL_EBADARGS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ecc_aggrDisableAllIntr(pEccAggrRegs, 0xFFFFU) != SDL_EBADARGS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ecc_aggrDisableAllIntr(pEccAggrRegs, 0xFFFFU) != SDL_EBADARGS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("sdlEccAggr_negTest: failure on line no. %d \r\n", __LINE__);
    }

    /* SDL_ecc_aggrEnableAllIntrs negative test */
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ecc_aggrEnableAllIntrs(NULL) != SDL_EBADARGS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("sdlEccAggr_negTest: failure on line no. %d \r\n", __LINE__);
    }

    /* SDL_ecc_aggrDisableAllIntrs negative test */
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ecc_aggrDisableAllIntrs(NULL) != SDL_EBADARGS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("sdlEccAggr_negTest: failure on line no. %d \r\n", __LINE__);
    }

    /* SDL_ecc_aggrIntrStatusCtrl negative test */
    if (testStatus == SDL_APP_TEST_PASS)
    {
        SDL_ecc_aggrStatusCtrl statusCtrl;
        if (SDL_ecc_aggrIntrStatusCtrl(NULL, &statusCtrl) != SDL_EBADARGS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ecc_aggrIntrStatusCtrl(pEccAggrRegs, NULL) != SDL_EBADARGS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus == SDL_APP_TEST_PASS)
    {
        SDL_ecc_aggrStatusCtrl statusCtrl;
        statusCtrl.timeOutCnt = 5U;
        statusCtrl.parityCnt  = 1U;
        if (SDL_ecc_aggrIntrStatusCtrl(pEccAggrRegs, &statusCtrl) != SDL_EBADARGS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus == SDL_APP_TEST_PASS)
    {
        SDL_ecc_aggrStatusCtrl statusCtrl;
        statusCtrl.timeOutCnt = 1U;
        statusCtrl.parityCnt  = 5U;
        if (SDL_ecc_aggrIntrStatusCtrl(pEccAggrRegs, &statusCtrl) != SDL_EBADARGS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus == SDL_APP_TEST_PASS)
    {
        SDL_ecc_aggrStatusCtrl statusCtrl;
        statusCtrl.timeOutCnt = 1U;
        statusCtrl.parityCnt  = 1U;
        statusCtrl.validCfg   = 0U;
        if (SDL_ecc_aggrIntrStatusCtrl(pEccAggrRegs, &statusCtrl) != SDL_EBADARGS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("sdlEccAggr_negTest: failure on line no. %d \r\n", __LINE__);
    }

    /* SDL_ecc_aggrIntrGetStatus negative test */
    if (testStatus == SDL_APP_TEST_PASS)
    {SDL_ecc_aggrStatusCtrl statusCtrl;
        if (SDL_ecc_aggrIntrGetStatus(NULL, &statusCtrl) != SDL_EBADARGS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ecc_aggrIntrGetStatus(pEccAggrRegs, NULL) != SDL_EBADARGS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus == SDL_APP_TEST_PASS)
    {
        SDL_ecc_aggrStatusCtrl statusCtrl;
        statusCtrl.validCfg   = 0U;
        if (SDL_ecc_aggrIntrGetStatus(pEccAggrRegs, &statusCtrl) != SDL_EBADARGS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("sdlEccAggr_negTest: failure on line no. %d \r\n", __LINE__);
    }

    return (testStatus);
}

/* ECC Error module test */
int32_t ECC_ip_errTest(void)
{
    int32_t testResult;

    testResult = ECC_errNegativeTest();

    return (testResult);
}
/* Nothing past this point */
