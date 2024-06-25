/*
 *   Copyright (c) Texas Instruments Incorporated 2022
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
 *  \file     ecc_test_func.c
 *
 *  \brief    This file contains ECC Functional test code.
 *
 *  \details  ECC Functional tests
 **/

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */
#include <stdint.h>
#include <sdl/include/sdl_types.h>
#include <kernel/dpl/DebugP.h>
#include <sdl/sdl_ecc.h>
#include <sdl/ecc/sdl_ip_ecc.h>
#include <sdl/include/am261x/sdlr_soc_baseaddress.h>
#include <sdl/include/am261x/sdlr_soc_ecc_aggr.h>
#include "ecc_test_main.h"



/* ========================================================================== */
/*                                Macros                                      */
/* ========================================================================== */

/* ========================================================================== */
/*                 Internal Function Declarations                             */
/* ========================================================================== */

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */
/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */


static int32_t ECC_funcAPITest(void)
{
    SDL_ecc_aggrRegs *pEccAggrRegs = ((SDL_ecc_aggrRegs *)((uintptr_t)SDL_ECC_AGG_R5SS0_CORE0_U_BASE));
    SDL_Ecc_AggrEccRamErrorStatusInfo eccRamErrorStatus;
    SDL_Ecc_AggrErrorInfo eccErrorInfo;
    SDL_ECC_staticRegs eccStaticRegs;
    int32_t testStatus = SDL_APP_TEST_PASS;
    uint32_t val;
    bool isPend;
    SDL_ECC_InjectErrorType intsrc;
    uint32_t mainMem, subMemType, errSrc;
    SDL_ECC_InjectErrorConfig_t injectErrorConfig;
    memset(&injectErrorConfig, 0, sizeof(injectErrorConfig));
    injectErrorConfig.pErrMem = (uint32_t *)(0u);
    injectErrorConfig.flipBitMask = 0x3;

    /* SDL_ecc_aggrGetRevision API test */
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ecc_aggrGetRevision(pEccAggrRegs, &val) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("sdlEccAggr_apiTest: failure on line no. %d \n", __LINE__);
        }
    }

    /* SDL_ecc_aggrGetNumRams API test */
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ecc_aggrGetNumRams(pEccAggrRegs, &val) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("sdlEccAggr_apiTest: failure on line no. %d \n", __LINE__);
        }
    }

    /* SDL_ecc_aggrReadEccRamReg API test */
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ecc_aggrReadEccRamReg(pEccAggrRegs, 0U, SDL_ECC_RAM_WRAP_REV, &val) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("sdlEccAggr_apiTest: failure on line no. %d \n", __LINE__);
        }
    }
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ecc_aggrReadEccRamReg(pEccAggrRegs, 0U, SDL_ECC_RAM_ERR_CTRL2, &val) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("sdlEccAggr_apiTest: failure on line no. %d \n", __LINE__);
        }
    }

    /* SDL_ecc_aggrReadEccRamWrapRevReg API test */
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ecc_aggrReadEccRamWrapRevReg(pEccAggrRegs, 0U, &val) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("sdlEccAggr_apiTest: failure on line no. %d \n", __LINE__);
        }
    }

    /* SDL_ecc_aggrReadEccRamCtrlReg API test */
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ecc_aggrReadEccRamCtrlReg(pEccAggrRegs, 0U, &val) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("sdlEccAggr_apiTest: failure on line no. %d \n", __LINE__);
        }
    }

    /* SDL_ecc_aggrReadEccRamErrCtrlReg API test */
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ecc_aggrReadEccRamErrCtrlReg(pEccAggrRegs, 0U, 0U, &val) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("sdlEccAggr_apiTest: failure on line no. %d \n", __LINE__);
        }
    }

    /* SDL_ecc_aggrReadEccRamErrStatReg API test */
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ecc_aggrReadEccRamErrStatReg(pEccAggrRegs, 0U, 0U, &val) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("sdlEccAggr_apiTest: failure on line no. %d \n", __LINE__);
        }
    }

    /* SDL_ecc_aggrWriteEccRamReg API test */
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ecc_aggrWriteEccRamReg(pEccAggrRegs, 0U, SDL_ECC_RAM_WRAP_REV, 0U) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("sdlEccAggr_apiTest: failure on line no. %d \n", __LINE__);
        }
    }

    /* SDL_ecc_aggrWriteEccRamCtrlReg API test */
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ecc_aggrWriteEccRamCtrlReg(pEccAggrRegs, 0U, 0U) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("sdlEccAggr_apiTest: failure on line no. %d \n", __LINE__);
        }
    }

    /* SDL_ecc_aggrWriteEccRamErrCtrlReg API test */
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ecc_aggrWriteEccRamErrCtrlReg(pEccAggrRegs, 0U, 0U, 0U) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("sdlEccAggr_apiTest: failure on line no. %d \n", __LINE__);
        }
    }

    /* SDL_ecc_aggrWriteEccRamErrStatReg API test */
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ecc_aggrWriteEccRamErrStatReg(pEccAggrRegs, 0U, 0U, 0U) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("sdlEccAggr_apiTest: failure on line no. %d \n", __LINE__);
        }
    }

    /* SDL_ecc_aggrConfigEccRam API test */
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ecc_aggrConfigEccRam(pEccAggrRegs, 0U, 0U, 0U, 0U) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("sdlEccAggr_apiTest: failure on line no. %d \n", __LINE__);
        }
    }

    /* SDL_ecc_aggrVerifyConfigEccRam API test */
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ecc_aggrVerifyConfigEccRam(pEccAggrRegs, 0U, 0U, 0U, 0U) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("sdlEccAggr_apiTest: failure on line no. %d \n", __LINE__);
        }
    }
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ecc_aggrConfigEccRam(pEccAggrRegs, 10U, (bool)true, (bool)true, (bool)true) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("sdlEccAggr_apiTest: failure on line no. %d \n", __LINE__);
        }
    }
    if (testStatus == SDL_APP_TEST_PASS)
    {
        /*To get EFAIL, given different values*/
        if (SDL_ecc_aggrVerifyConfigEccRam(pEccAggrRegs, 0U, 0U, 0U, 0U) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("sdlEccAggr_apiTest: failure on line no. %d \n", __LINE__);
        }
    }

    /* SDL_ecc_aggrGetEccRamGetErrorStatus API test */
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ecc_aggrGetEccRamErrorStatus(pEccAggrRegs, 0U, &eccRamErrorStatus) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("sdlEccAggr_apiTest: failure on line no. %d \n", __LINE__);
        }
    }

    /* SDL_ecc_aggrForceEccRamError API test */
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ecc_aggrForceEccRamError(pEccAggrRegs, 0U, &eccErrorInfo) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("sdlEccAggr_apiTest: failure on line no. %d \n", __LINE__);
        }
    }
    if (testStatus == SDL_APP_TEST_PASS)
    {
        eccErrorInfo.intrSrc = SDL_ECC_AGGR_INTR_SRC_SINGLE_BIT;
        eccErrorInfo.bNextRow = false;
        eccErrorInfo.bOneShotMode = false;
        if (SDL_ecc_aggrForceEccRamError(pEccAggrRegs, 0U, &eccErrorInfo) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("sdlEccAggr_apiTest: failure on line no. %d \n", __LINE__);
        }
    }
    if (testStatus == SDL_APP_TEST_PASS)
    {
        eccErrorInfo.intrSrc = SDL_ECC_AGGR_INTR_SRC_DOUBLE_BIT;
        eccErrorInfo.bNextRow = true;
        eccErrorInfo.bOneShotMode = true;
        if (SDL_ecc_aggrForceEccRamError(pEccAggrRegs, 0U, &eccErrorInfo) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("sdlEccAggr_apiTest: failure on line no. %d \n", __LINE__);
        }
    }
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ecc_aggrGetEccRamErrorStatus(pEccAggrRegs, 0U, &eccRamErrorStatus) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("sdlEccAggr_apiTest: failure on line no. %d \n", __LINE__);
        }
    }

    /* SDL_ecc_aggrAckIntr API test */
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ecc_aggrAckIntr(pEccAggrRegs, SDL_ECC_AGGR_INTR_SRC_SINGLE_BIT) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("sdlEccAggr_apiTest: failure on line no. %d \n", __LINE__);
        }
    }
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ecc_aggrAckIntr(pEccAggrRegs, SDL_ECC_AGGR_INTR_SRC_DOUBLE_BIT) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("sdlEccAggr_apiTest: failure on line no. %d \n", __LINE__);
        }
    }

    /* SDL_ecc_aggrSetEccRamIntrPending API test */
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ecc_aggrSetEccRamIntrPending(pEccAggrRegs, 5U, SDL_ECC_AGGR_INTR_SRC_SINGLE_BIT) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("sdlEccAggr_apiTest: failure on line no. %d \n", __LINE__);
        }
    }
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ecc_aggrSetEccRamNIntrPending(pEccAggrRegs, 5U, SDL_ECC_AGGR_INTR_SRC_SINGLE_BIT, 3U) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("sdlEccAggr_apiTest: failure on line no. %d \n", __LINE__);
        }
    }
    /* SDL_ecc_aggrIsEccRamIntrPending API test */
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ecc_aggrIsEccRamIntrPending(pEccAggrRegs, 5U, SDL_ECC_AGGR_INTR_SRC_SINGLE_BIT, &isPend) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("sdlEccAggr_apiTest: failure on line no. %d \n", __LINE__);
        }
    }
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ecc_aggrSetEccRamNIntrPending(pEccAggrRegs, 5U, SDL_ECC_AGGR_INTR_SRC_SINGLE_BIT, 3U) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("sdlEccAggr_apiTest: failure on line no. %d \n", __LINE__);
        }
    }
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ecc_aggrSetEccRamIntrPending(pEccAggrRegs, 5U, SDL_ECC_AGGR_INTR_SRC_DOUBLE_BIT) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("sdlEccAggr_apiTest: failure on line no. %d \n", __LINE__);
        }
    }
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ecc_aggrSetEccRamNIntrPending(pEccAggrRegs, 5U, SDL_ECC_AGGR_INTR_SRC_DOUBLE_BIT, 3U) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("sdlEccAggr_apiTest: failure on line no. %d \n", __LINE__);
        }
    }
    /* SDL_ecc_aggrIsEccRamIntrPending API test */
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ecc_aggrIsEccRamIntrPending(pEccAggrRegs, 5U, SDL_ECC_AGGR_INTR_SRC_DOUBLE_BIT, &isPend) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("sdlEccAggr_apiTest: failure on line no. %d \n", __LINE__);
        }
    }
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ecc_aggrSetEccRamIntrPending(pEccAggrRegs, 5U, SDL_ECC_ADDR_ERROR_TYPE_SUCCESSIVE_SINGLE_BITS) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("sdlEccAggr_apiTest: failure on line no. %d \n", __LINE__);
        }
    }
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ecc_aggrSetEccRamNIntrPending(pEccAggrRegs, 5U, SDL_ECC_ADDR_ERROR_TYPE_SUCCESSIVE_SINGLE_BITS, 3U) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("sdlEccAggr_apiTest: failure on line no. %d \n", __LINE__);
        }
    }
    /* SDL_ecc_aggrIsEccRamIntrPending API test */
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ecc_aggrIsEccRamIntrPending(pEccAggrRegs, 5U, SDL_ECC_ADDR_ERROR_TYPE_SUCCESSIVE_SINGLE_BITS, &isPend) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("sdlEccAggr_apiTest: failure on line no. %d \n", __LINE__);
        }
    }
    /* SDL_ecc_aggrSetEccRamNIntrPending API test */
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ecc_aggrSetEccRamNIntrPending(pEccAggrRegs, 5U, SDL_ECC_AGGR_INTR_SRC_SINGLE_BIT, 3U) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("sdlEccAggr_apiTest: failure on line no. %d \n", __LINE__);
        }
    }
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ecc_aggrSetEccRamNIntrPending(pEccAggrRegs, 0U, SDL_ECC_ADDR_ERROR_TYPE_SUCCESSIVE_SINGLE_BITS, 2U) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("sdlEccAggr_apiTest: failure on line no. %d \n", __LINE__);
        }
    }
    /* SDL_ecc_aggrClrEccRamIntrPending API test */
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ecc_aggrClrEccRamIntrPending(pEccAggrRegs, 5U, SDL_ECC_AGGR_INTR_SRC_SINGLE_BIT) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("sdlEccAggr_apiTest: failure on line no. %d \n", __LINE__);
        }
    }
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ecc_aggrClrEccRamIntrPending(pEccAggrRegs, 5U, SDL_ECC_AGGR_INTR_SRC_DOUBLE_BIT) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("sdlEccAggr_apiTest: failure on line no. %d \n", __LINE__);
        }
    }
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ecc_aggrClrEccRamIntrPending(pEccAggrRegs, 5U, SDL_ECC_ADDR_ERROR_TYPE_SUCCESSIVE_SINGLE_BITS) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("sdlEccAggr_apiTest: failure on line no. %d \n", __LINE__);
        }
    }
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ecc_aggrIsEccRamIntrPending(pEccAggrRegs, 5U, SDL_ECC_AGGR_INTR_SRC_SINGLE_BIT, &isPend) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("sdlEccAggr_apiTest: failure on line no. %d \n", __LINE__);
        }
    }
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ecc_aggrIsEccRamIntrPending(pEccAggrRegs, 5U, SDL_ECC_AGGR_INTR_SRC_DOUBLE_BIT, &isPend) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("sdlEccAggr_apiTest: failure on line no. %d \n", __LINE__);
        }
    }
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ecc_aggrIsEccRamIntrPending(pEccAggrRegs, 5U, SDL_ECC_ADDR_ERROR_TYPE_SUCCESSIVE_SINGLE_BITS, &isPend) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("sdlEccAggr_apiTest: failure on line no. %d \n", __LINE__);
        }
    }
    /* SDL_ecc_aggrClrEccRamNIntrPending API test */
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ecc_aggrClrEccRamNIntrPending(pEccAggrRegs, 0U, SDL_ECC_AGGR_INTR_SRC_SINGLE_BIT, 1U) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("sdlEccAggr_apiTest: failure on line no. %d \n", __LINE__);
        }
    }
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ecc_aggrClrEccRamNIntrPending(pEccAggrRegs, 0U, SDL_ECC_ADDR_ERROR_TYPE_SUCCESSIVE_SINGLE_BITS, 1U) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("sdlEccAggr_apiTest: failure on line no. %d \n", __LINE__);
        }
    }
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ecc_aggrClrEccRamNIntrPending(pEccAggrRegs, 0U, SDL_ECC_ADDR_ERROR_TYPE_SUCCESSIVE_SINGLE_BITS, 1U) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("sdlEccAggr_apiTest: failure on line no. %d \n", __LINE__);
        }
    }

    /* SDL_ecc_aggrEnableIntr API test */
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ecc_aggrEnableIntr(pEccAggrRegs, 0U, SDL_ECC_AGGR_INTR_SRC_SINGLE_BIT) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("sdlEccAggr_apiTest: failure on line no. %d \n", __LINE__);
        }
    }

    /* SDL_ecc_aggrDisableIntr API test */
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ecc_aggrDisableIntr(pEccAggrRegs, 0U, SDL_ECC_AGGR_INTR_SRC_SINGLE_BIT) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("sdlEccAggr_apiTest: failure on line no. %d \n", __LINE__);
        }
    }

    /* SDL_ecc_aggrEnableAllIntr API test */
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ecc_aggrEnableAllIntr(pEccAggrRegs, 0U) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("sdlEccAggr_apiTest: failure on line no. %d \n", __LINE__);
        }
    }

    /* SDL_ecc_aggrDisableAllIntr API test */
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ecc_aggrDisableAllIntr(pEccAggrRegs, 0U) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("sdlEccAggr_apiTest: failure on line no. %d \n", __LINE__);
        }
    }

    /* SDL_ecc_aggrEnableIntrs API test */
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ecc_aggrEnableIntrs(pEccAggrRegs, SDL_ECC_AGGR_INTR_SRC_SINGLE_BIT) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("sdlEccAggr_apiTest: failure on line no. %d \n", __LINE__);
        }
    }

    /* SDL_ecc_aggrDisableIntrs API test */
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ecc_aggrDisableIntrs(pEccAggrRegs, SDL_ECC_AGGR_INTR_SRC_SINGLE_BIT) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("sdlEccAggr_apiTest: failure on line no. %d \n", __LINE__);
        }
    }

    /* SDL_ecc_aggrEnableAllIntrs API test */
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ecc_aggrEnableAllIntrs(pEccAggrRegs) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("sdlEccAggr_apiTest: failure on line no. %d \n", __LINE__);
        }
    }

    /* SDL_ecc_aggrDisableAllIntrs API test */
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ecc_aggrDisableAllIntrs(pEccAggrRegs) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("sdlEccAggr_apiTest: failure on line no. %d \n", __LINE__);
        }
    }

    /* SDL_ecc_aggrReadStaticRegs API test */
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ecc_aggrReadStaticRegs(pEccAggrRegs, &eccStaticRegs) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("sdlEccAggr_apiTest: failure on line no. %d \n", __LINE__);
        }
    }

    /* SDL_ecc_aggrIntrEnableCtrl API test */
    if (testStatus == SDL_APP_TEST_PASS)
    {
        SDL_ecc_aggrEnableCtrl enableCtrl;

        enableCtrl.validCfg = (SDL_ECC_AGGR_VALID_TIMEOUT_ERR | \
                               SDL_ECC_AGGR_VALID_PARITY_ERR) ;
        enableCtrl.intrEnableTimeoutErr = TRUE;
        enableCtrl.intrEnableParityErr  = TRUE;
        if (SDL_ecc_aggrIntrEnableCtrl(pEccAggrRegs, &enableCtrl) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("sdlEccAggr_apiTest: failure on line no. %d \n", __LINE__);
        }
    }

    /* SDL_ecc_aggrIntrEnableCtrl API test */
    if (testStatus == SDL_APP_TEST_PASS)
    {
        SDL_ecc_aggrEnableCtrl enableCtrl;

        enableCtrl.validCfg = (SDL_ECC_AGGR_VALID_TIMEOUT_ERR | \
                               SDL_ECC_AGGR_VALID_PARITY_ERR) ;
        enableCtrl.intrEnableTimeoutErr = FALSE;
        enableCtrl.intrEnableParityErr  = TRUE;
        if (SDL_ecc_aggrIntrEnableCtrl(pEccAggrRegs, &enableCtrl) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("sdlEccAggr_apiTest: failure on line no. %d \n", __LINE__);
        }
    }
    /* SDL_ecc_aggrIntrEnableCtrl API test */
    if (testStatus == SDL_APP_TEST_PASS)
    {
        SDL_ecc_aggrEnableCtrl enableCtrl;

        enableCtrl.validCfg = (SDL_ECC_AGGR_VALID_TIMEOUT_ERR | \
                               SDL_ECC_AGGR_VALID_PARITY_ERR) ;
        enableCtrl.intrEnableTimeoutErr = TRUE;
        enableCtrl.intrEnableParityErr  = FALSE;
        if (SDL_ecc_aggrIntrEnableCtrl(pEccAggrRegs, &enableCtrl) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("sdlEccAggr_apiTest: failure on line no. %d \n", __LINE__);
        }
    }
    /* SDL_ecc_aggrIntrEnableCtrl API test */
    if (testStatus == SDL_APP_TEST_PASS)
    {
        SDL_ecc_aggrEnableCtrl enableCtrl;

        enableCtrl.validCfg = (SDL_ECC_AGGR_VALID_TIMEOUT_ERR | \
                               SDL_ECC_AGGR_VALID_PARITY_ERR) ;
        enableCtrl.intrEnableTimeoutErr = FALSE;
        enableCtrl.intrEnableParityErr  = FALSE;
        if (SDL_ecc_aggrIntrEnableCtrl(pEccAggrRegs, &enableCtrl) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("sdlEccAggr_apiTest: failure on line no. %d \n", __LINE__);
        }
    }
    if (testStatus == SDL_APP_TEST_PASS)
    {
        SDL_ecc_aggrEnableCtrl enableCtrl;
        enableCtrl.validCfg = 0U;
        if (SDL_ecc_aggrIntrEnableCtrl(pEccAggrRegs, &enableCtrl) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("sdlEccAggr_apiTest: failure on line no. %d \n", __LINE__);
        }
    }
    /* SDL_ecc_aggrIntrStatusCtrl API test */
    if (testStatus == SDL_APP_TEST_PASS)
    {
        SDL_ecc_aggrStatusCtrl statusCtrl;

        statusCtrl.validCfg = (SDL_ECC_AGGR_VALID_TIMEOUT_ERR | \
                               SDL_ECC_AGGR_VALID_PARITY_ERR) ;
        statusCtrl.intrStatusSetTimeoutErr  = TRUE;
        statusCtrl.intrStatusSetParityErr   = TRUE;
        statusCtrl.timeOutCnt               = (uint8_t)0U;
        statusCtrl.parityCnt                = (uint8_t)1U;
        if (SDL_ecc_aggrIntrStatusCtrl(pEccAggrRegs, &statusCtrl) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("sdlEccAggr_apiTest: failure on line no. %d \n", __LINE__);
        }
    }
    /* SDL_ecc_aggrIntrStatusCtrl API test */
    if (testStatus == SDL_APP_TEST_PASS)
    {
        SDL_ecc_aggrStatusCtrl statusCtrl;

        statusCtrl.validCfg = (SDL_ECC_AGGR_VALID_TIMEOUT_ERR | \
                               SDL_ECC_AGGR_VALID_PARITY_ERR) ;
        statusCtrl.intrStatusSetTimeoutErr  = TRUE;
        statusCtrl.intrStatusSetParityErr   = FALSE;
        statusCtrl.timeOutCnt               = (uint8_t)0U;
        statusCtrl.parityCnt                = (uint8_t)0U;
        if (SDL_ecc_aggrIntrStatusCtrl(pEccAggrRegs, &statusCtrl) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("sdlEccAggr_apiTest: failure on line no. %d \n", __LINE__);
        }
    }
    /* SDL_ecc_aggrIntrStatusCtrl API test */
    if (testStatus == SDL_APP_TEST_PASS)
    {
        SDL_ecc_aggrStatusCtrl statusCtrl;

        statusCtrl.validCfg = (SDL_ECC_AGGR_VALID_TIMEOUT_ERR | \
                               SDL_ECC_AGGR_VALID_PARITY_ERR) ;
        statusCtrl.intrStatusSetTimeoutErr  = FALSE;
        statusCtrl.intrStatusSetParityErr   = TRUE;
        statusCtrl.timeOutCnt               = (uint8_t)0U;
        statusCtrl.parityCnt                = (uint8_t)1U;
        if (SDL_ecc_aggrIntrStatusCtrl(pEccAggrRegs, &statusCtrl) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("sdlEccAggr_apiTest: failure on line no. %d \n", __LINE__);
        }
    }
    /* SDL_ecc_aggrIntrStatusCtrl API test */
    if (testStatus == SDL_APP_TEST_PASS)
    {
        SDL_ecc_aggrStatusCtrl statusCtrl;

        statusCtrl.validCfg = (SDL_ECC_AGGR_VALID_TIMEOUT_ERR | \
                               SDL_ECC_AGGR_VALID_PARITY_ERR) ;
        statusCtrl.intrStatusSetTimeoutErr  = FALSE;
        statusCtrl.intrStatusSetParityErr   = FALSE;
        statusCtrl.timeOutCnt               = (uint8_t)0U;
        statusCtrl.parityCnt                = (uint8_t)1U;
        if (SDL_ecc_aggrIntrStatusCtrl(pEccAggrRegs, &statusCtrl) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("sdlEccAggr_apiTest: failure on line no. %d \n", __LINE__);
        }
    }

    /* SDL_ecc_aggrIntrGetStatus API test */
    if (testStatus == SDL_APP_TEST_PASS)
    {
        SDL_ecc_aggrStatusCtrl statusCtrl;

        statusCtrl.validCfg = (SDL_ECC_AGGR_VALID_TIMEOUT_ERR | \
                               SDL_ECC_AGGR_VALID_PARITY_ERR) ;
        statusCtrl.intrStatusSetTimeoutErr  = FALSE;
        statusCtrl.intrStatusSetParityErr   = FALSE;
        if (SDL_ecc_aggrIntrGetStatus(pEccAggrRegs, &statusCtrl) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("sdlEccAggr_apiTest: failure on line no. %d \n", __LINE__);
        }
    }

        /* SDL_ecc_aggrIntrGetStatus API test */
    if (testStatus == SDL_APP_TEST_PASS)
    {
        SDL_ecc_aggrStatusCtrl statusCtrl;

        statusCtrl.validCfg = (SDL_ECC_AGGR_VALID_TIMEOUT_ERR | \
                               SDL_ECC_AGGR_VALID_PARITY_ERR) ;
        statusCtrl.intrStatusSetTimeoutErr  = TRUE;
        statusCtrl.intrStatusSetParityErr   = TRUE;
        if (SDL_ecc_aggrIntrGetStatus(pEccAggrRegs, &statusCtrl) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("sdlEccAggr_apiTest: failure on line no. %d \n", __LINE__);
        }
    }

        /* SDL_ecc_aggrIntrGetStatus API test */
    if (testStatus == SDL_APP_TEST_PASS)
    {
        SDL_ecc_aggrStatusCtrl statusCtrl;

        statusCtrl.validCfg = (SDL_ECC_AGGR_VALID_TIMEOUT_ERR | \
                               SDL_ECC_AGGR_VALID_PARITY_ERR) ;
        statusCtrl.intrStatusSetTimeoutErr  = TRUE;
        statusCtrl.intrStatusSetParityErr   = FALSE;
        if (SDL_ecc_aggrIntrGetStatus(pEccAggrRegs, &statusCtrl) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("sdlEccAggr_apiTest: failure on line no. %d \n", __LINE__);
        }
    }

        /* SDL_ecc_aggrIntrGetStatus API test */
    if (testStatus == SDL_APP_TEST_PASS)
    {
        SDL_ecc_aggrStatusCtrl statusCtrl;

        statusCtrl.validCfg = (SDL_ECC_AGGR_VALID_TIMEOUT_ERR | \
                               SDL_ECC_AGGR_VALID_PARITY_ERR) ;
        statusCtrl.intrStatusSetTimeoutErr  = FALSE;
        statusCtrl.intrStatusSetParityErr   = TRUE;
        if (SDL_ecc_aggrIntrGetStatus(pEccAggrRegs, &statusCtrl) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("sdlEccAggr_apiTest: failure on line no. %d \n", __LINE__);
        }
    }

    /* SDL_ecc_aggrIsIntrPending API test */
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ecc_aggrIsIntrPending(pEccAggrRegs, 0U, SDL_ECC_AGGR_INTR_SRC_SINGLE_BIT, &isPend) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("sdlEccAggr_apiTest: failure on line no. %d \n", __LINE__);
        }
    }
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ecc_aggrIsIntrPending(pEccAggrRegs, 0U, SDL_ECC_AGGR_INTR_SRC_DOUBLE_BIT, &isPend) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("sdlEccAggr_apiTest: failure on line no. %d \n", __LINE__);
        }
    }

    /* SDL_ecc_aggrIsAnyIntrPending API test */
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ecc_aggrIsAnyIntrPending(pEccAggrRegs, 0U, &isPend) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("sdlEccAggr_apiTest: failure on line no. %d \n", __LINE__);
        }
    }
    if (testStatus == SDL_APP_TEST_PASS)
    {
        mainMem    = SDL_ECC_MEMTYPE_MAX;
        subMemType = SDL_SOC_ECC_AGGR_MAILBOX_ECC_RAM_ID;
        intsrc     = SDL_INJECT_ECC_ERROR_FORCING_2BIT_REPEAT;
        errSrc     = SDL_ECC_AGGR_INTR_SRC_DOUBLE_BIT;
        SDL_ECC_injectError(mainMem, subMemType, intsrc,&injectErrorConfig);
        SDL_ecc_aggrSetEccRamNIntrPending(pEccAggrRegs, subMemType, errSrc, 3U);
        if (SDL_ecc_aggrIsEccRamIntrPending(pEccAggrRegs, subMemType, 0U, &isPend) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("sdlEccAggr_apiTest: failure on line no. %d \n", __LINE__);
        }
    }
    if (testStatus == SDL_APP_TEST_PASS)
    {
        mainMem    = SDL_ECC_MEMTYPE_MAX;
        subMemType = SDL_SOC_ECC_AGGR_MAILBOX_ECC_RAM_ID;
        intsrc     = SDL_INJECT_ECC_ERROR_FORCING_2BIT_REPEAT;
        errSrc     = SDL_ECC_AGGR_INTR_SRC_DOUBLE_BIT;
        SDL_ECC_injectError(mainMem, subMemType, intsrc,&injectErrorConfig);
        SDL_ecc_aggrSetEccRamIntrPending(pEccAggrRegs, subMemType, errSrc);
        if (SDL_ecc_aggrIsEccRamIntrPending(pEccAggrRegs, subMemType, errSrc, &isPend) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("sdlEccAggr_apiTest: failure on line no. %d \n", __LINE__);
        }
        if (SDL_ecc_aggrIsAnyIntrPending(pEccAggrRegs, subMemType, &isPend) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("sdlEccAggr_apiTest: failure on line no. %d \n", __LINE__);
        }
    }
    if (testStatus == SDL_APP_TEST_PASS)
    {
        mainMem    = SDL_ECC_MEMTYPE_MAX;
        subMemType = SDL_SOC_ECC_AGGR_MAILBOX_ECC_RAM_ID;
        intsrc     = SDL_INJECT_ECC_ERROR_FORCING_1BIT_REPEAT;
        errSrc     = SDL_ECC_AGGR_INTR_SRC_SINGLE_BIT;

        SDL_ECC_injectError(mainMem, subMemType, intsrc,&injectErrorConfig);
        SDL_ecc_aggrSetEccRamIntrPending(pEccAggrRegs, subMemType, errSrc);
        if (SDL_ecc_aggrIsEccRamIntrPending(pEccAggrRegs, subMemType, errSrc, &isPend) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("sdlEccAggr_apiTest: failure on line no. %d \n", __LINE__);
        }
        if (SDL_ecc_aggrIsAnyIntrPending(pEccAggrRegs, subMemType, &isPend) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("sdlEccAggr_apiTest: failure on line no. %d \n", __LINE__);
        }
    }
    if (testStatus == SDL_APP_TEST_PASS)
    {
        mainMem    = SDL_ECC_MEMTYPE_MAX;
        subMemType = SDL_SOC_ECC_AGGR_MAILBOX_ECC_RAM_ID;
        intsrc     = SDL_INJECT_ECC_ERROR_FORCING_1BIT_ONCE;
        errSrc     = SDL_ECC_AGGR_INTR_SRC_SINGLE_BIT;

        SDL_ECC_injectError(mainMem, subMemType, intsrc,&injectErrorConfig);
        SDL_ecc_aggrSetEccRamIntrPending(pEccAggrRegs, subMemType, errSrc);
        if (SDL_ecc_aggrIsEccRamIntrPending(pEccAggrRegs, subMemType, errSrc, &isPend) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("sdlEccAggr_apiTest: failure on line no. %d \n", __LINE__);
        }
        if (SDL_ecc_aggrIsAnyIntrPending(pEccAggrRegs, subMemType, &isPend) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("sdlEccAggr_apiTest: failure on line no. %d \n", __LINE__);
        }
    }
    if (testStatus == SDL_APP_TEST_PASS)
    {
        mainMem    = SDL_ECC_MEMTYPE_MAX;
        subMemType = SDL_R5FSS0_CORE0_ECC_AGGR_CPU0_ITAG_RAM0_RAM_ID;
        intsrc     = SDL_INJECT_ECC_ERROR_FORCING_2BIT_REPEAT;
        errSrc     = SDL_ECC_AGGR_INTR_SRC_DOUBLE_BIT;
        SDL_ECC_injectError(mainMem, subMemType, intsrc,&injectErrorConfig);
        SDL_ecc_aggrSetEccRamNIntrPending(pEccAggrRegs, subMemType, errSrc, 3U);
        if (SDL_ecc_aggrIsEccRamIntrPending(pEccAggrRegs, subMemType, 0U, &isPend) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("sdlEccAggr_apiTest: failure on line no. %d \n", __LINE__);
        }
    }
    if (testStatus == SDL_APP_TEST_PASS)
    {
        mainMem    = SDL_ECC_MEMTYPE_MAX;
        subMemType = SDL_R5FSS0_CORE0_ECC_AGGR_CPU0_ITAG_RAM0_RAM_ID;
        intsrc     = SDL_INJECT_ECC_ERROR_FORCING_2BIT_REPEAT;
        errSrc     = SDL_ECC_AGGR_INTR_SRC_DOUBLE_BIT;
        SDL_ECC_injectError(mainMem, subMemType, intsrc,&injectErrorConfig);
        SDL_ecc_aggrSetEccRamIntrPending(pEccAggrRegs, subMemType, errSrc);
        if (SDL_ecc_aggrIsEccRamIntrPending(pEccAggrRegs, subMemType, errSrc, &isPend) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("sdlEccAggr_apiTest: failure on line no. %d \n", __LINE__);
        }
        if (SDL_ecc_aggrIsAnyIntrPending(pEccAggrRegs, subMemType, &isPend) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("sdlEccAggr_apiTest: failure on line no. %d \n", __LINE__);
        }
    }
    if (testStatus == SDL_APP_TEST_PASS)
    {
        mainMem    = SDL_ECC_MEMTYPE_MAX;
        subMemType = SDL_R5FSS0_CORE0_ECC_AGGR_CPU0_ITAG_RAM0_RAM_ID;
        intsrc     = SDL_INJECT_ECC_ERROR_FORCING_1BIT_REPEAT;
        errSrc     = SDL_ECC_AGGR_INTR_SRC_SINGLE_BIT;

        SDL_ECC_injectError(mainMem, subMemType, intsrc,&injectErrorConfig);
        SDL_ecc_aggrSetEccRamIntrPending(pEccAggrRegs, subMemType, errSrc);
        if (SDL_ecc_aggrIsEccRamIntrPending(pEccAggrRegs, subMemType, errSrc, &isPend) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("sdlEccAggr_apiTest: failure on line no. %d \n", __LINE__);
        }
        if (SDL_ecc_aggrIsAnyIntrPending(pEccAggrRegs, subMemType, &isPend) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("sdlEccAggr_apiTest: failure on line no. %d \n", __LINE__);
        }
    }
    if (testStatus == SDL_APP_TEST_PASS)
    {
        mainMem    = SDL_ECC_MEMTYPE_MAX;
        subMemType = SDL_R5FSS0_CORE0_ECC_AGGR_CPU0_ITAG_RAM0_RAM_ID;
        intsrc     = SDL_INJECT_ECC_ERROR_FORCING_1BIT_ONCE;
        errSrc     = SDL_ECC_AGGR_INTR_SRC_SINGLE_BIT;

        SDL_ECC_injectError(mainMem, subMemType, intsrc,&injectErrorConfig);
        SDL_ecc_aggrSetEccRamIntrPending(pEccAggrRegs, subMemType, errSrc);
        if (SDL_ecc_aggrIsEccRamIntrPending(pEccAggrRegs, subMemType, errSrc, &isPend) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("sdlEccAggr_apiTest: failure on line no. %d \n", __LINE__);
        }
        if (SDL_ecc_aggrIsAnyIntrPending(pEccAggrRegs, subMemType, &isPend) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("sdlEccAggr_apiTest: failure on line no. %d \n", __LINE__);
        }
    }
    if (testStatus == SDL_APP_TEST_PASS)
    {
        mainMem    = SDL_ECC_MEMTYPE_MAX;
        subMemType = SDL_R5FSS0_CORE0_ECC_AGGR_CPU0_ITAG_RAM1_RAM_ID;
        intsrc     = SDL_INJECT_ECC_ERROR_FORCING_2BIT_REPEAT;
        errSrc     = SDL_ECC_AGGR_INTR_SRC_DOUBLE_BIT;
        SDL_ECC_injectError(mainMem, subMemType, intsrc,&injectErrorConfig);
        SDL_ecc_aggrSetEccRamNIntrPending(pEccAggrRegs, subMemType, errSrc, 3U);
        if (SDL_ecc_aggrIsEccRamIntrPending(pEccAggrRegs, subMemType, 0U, &isPend) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("sdlEccAggr_apiTest: failure on line no. %d \n", __LINE__);
        }
    }
    if (testStatus == SDL_APP_TEST_PASS)
    {
        mainMem    = SDL_ECC_MEMTYPE_MAX;
        subMemType = SDL_R5FSS0_CORE0_ECC_AGGR_CPU0_ITAG_RAM1_RAM_ID;
        intsrc     = SDL_INJECT_ECC_ERROR_FORCING_2BIT_REPEAT;
        errSrc     = SDL_ECC_AGGR_INTR_SRC_DOUBLE_BIT;
        SDL_ECC_injectError(mainMem, subMemType, intsrc,&injectErrorConfig);
        SDL_ecc_aggrSetEccRamIntrPending(pEccAggrRegs, subMemType, errSrc);
        if (SDL_ecc_aggrIsEccRamIntrPending(pEccAggrRegs, subMemType, errSrc, &isPend) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("sdlEccAggr_apiTest: failure on line no. %d \n", __LINE__);
        }
        if (SDL_ecc_aggrIsAnyIntrPending(pEccAggrRegs, subMemType, &isPend) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("sdlEccAggr_apiTest: failure on line no. %d \n", __LINE__);
        }
    }
    if (testStatus == SDL_APP_TEST_PASS)
    {
        mainMem    = SDL_ECC_MEMTYPE_MAX;
        subMemType = SDL_R5FSS0_CORE0_ECC_AGGR_CPU0_ITAG_RAM1_RAM_ID;
        intsrc     = SDL_INJECT_ECC_ERROR_FORCING_1BIT_REPEAT;
        errSrc     = SDL_ECC_AGGR_INTR_SRC_SINGLE_BIT;

        SDL_ECC_injectError(mainMem, subMemType, intsrc,&injectErrorConfig);
        SDL_ecc_aggrSetEccRamIntrPending(pEccAggrRegs, subMemType, errSrc);
        if (SDL_ecc_aggrIsEccRamIntrPending(pEccAggrRegs, subMemType, errSrc, &isPend) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("sdlEccAggr_apiTest: failure on line no. %d \n", __LINE__);
        }
        if (SDL_ecc_aggrIsAnyIntrPending(pEccAggrRegs, subMemType, &isPend) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("sdlEccAggr_apiTest: failure on line no. %d \n", __LINE__);
        }
    }
    if (testStatus == SDL_APP_TEST_PASS)
    {
        mainMem    = SDL_ECC_MEMTYPE_MAX;
        subMemType = SDL_R5FSS0_CORE0_ECC_AGGR_CPU0_ITAG_RAM1_RAM_ID;
        intsrc     = SDL_INJECT_ECC_ERROR_FORCING_1BIT_ONCE;
        errSrc     = SDL_ECC_AGGR_INTR_SRC_SINGLE_BIT;

        SDL_ECC_injectError(mainMem, subMemType, intsrc,&injectErrorConfig);
        SDL_ecc_aggrSetEccRamIntrPending(pEccAggrRegs, subMemType, errSrc);
        if (SDL_ecc_aggrIsEccRamIntrPending(pEccAggrRegs, subMemType, errSrc, &isPend) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("sdlEccAggr_apiTest: failure on line no. %d \n", __LINE__);
        }
        if (SDL_ecc_aggrIsAnyIntrPending(pEccAggrRegs, subMemType, &isPend) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("sdlEccAggr_apiTest: failure on line no. %d \n", __LINE__);
        }
    }
    if (testStatus == SDL_APP_TEST_PASS)
    {
        mainMem    = SDL_ECC_MEMTYPE_MAX;
        subMemType = SDL_R5FSS0_CORE0_ECC_AGGR_CPU0_ITAG_RAM2_RAM_ID;
        intsrc     = SDL_INJECT_ECC_ERROR_FORCING_2BIT_REPEAT;
        errSrc     = SDL_ECC_AGGR_INTR_SRC_DOUBLE_BIT;
        SDL_ECC_injectError(mainMem, subMemType, intsrc,&injectErrorConfig);
        SDL_ecc_aggrSetEccRamNIntrPending(pEccAggrRegs, subMemType, errSrc, 3U);
        if (SDL_ecc_aggrIsEccRamIntrPending(pEccAggrRegs, subMemType, 0U, &isPend) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("sdlEccAggr_apiTest: failure on line no. %d \n", __LINE__);
        }
    }
    if (testStatus == SDL_APP_TEST_PASS)
    {
        mainMem    = SDL_ECC_MEMTYPE_MAX;
        subMemType = SDL_R5FSS0_CORE0_ECC_AGGR_CPU0_ITAG_RAM2_RAM_ID;
        intsrc     = SDL_INJECT_ECC_ERROR_FORCING_2BIT_REPEAT;
        errSrc     = SDL_ECC_AGGR_INTR_SRC_DOUBLE_BIT;
        SDL_ECC_injectError(mainMem, subMemType, intsrc,&injectErrorConfig);
        SDL_ecc_aggrSetEccRamIntrPending(pEccAggrRegs, subMemType, errSrc);
        if (SDL_ecc_aggrIsEccRamIntrPending(pEccAggrRegs, subMemType, errSrc, &isPend) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("sdlEccAggr_apiTest: failure on line no. %d \n", __LINE__);
        }
        if (SDL_ecc_aggrIsAnyIntrPending(pEccAggrRegs, subMemType, &isPend) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("sdlEccAggr_apiTest: failure on line no. %d \n", __LINE__);
        }
    }
    if (testStatus == SDL_APP_TEST_PASS)
    {
        mainMem    = SDL_ECC_MEMTYPE_MAX;
        subMemType = SDL_R5FSS0_CORE0_ECC_AGGR_CPU0_ITAG_RAM2_RAM_ID;
        intsrc     = SDL_INJECT_ECC_ERROR_FORCING_1BIT_REPEAT;
        errSrc     = SDL_ECC_AGGR_INTR_SRC_SINGLE_BIT;

        SDL_ECC_injectError(mainMem, subMemType, intsrc,&injectErrorConfig);
        SDL_ecc_aggrSetEccRamIntrPending(pEccAggrRegs, subMemType, errSrc);
        if (SDL_ecc_aggrIsEccRamIntrPending(pEccAggrRegs, subMemType, errSrc, &isPend) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("sdlEccAggr_apiTest: failure on line no. %d \n", __LINE__);
        }
        if (SDL_ecc_aggrIsAnyIntrPending(pEccAggrRegs, subMemType, &isPend) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("sdlEccAggr_apiTest: failure on line no. %d \n", __LINE__);
        }
    }
    if (testStatus == SDL_APP_TEST_PASS)
    {
        mainMem    = SDL_ECC_MEMTYPE_MAX;
        subMemType = SDL_R5FSS0_CORE0_ECC_AGGR_CPU0_ITAG_RAM2_RAM_ID;
        intsrc     = SDL_INJECT_ECC_ERROR_FORCING_1BIT_ONCE;
        errSrc     = SDL_ECC_AGGR_INTR_SRC_SINGLE_BIT;

        SDL_ECC_injectError(mainMem, subMemType, intsrc,&injectErrorConfig);
        SDL_ecc_aggrSetEccRamIntrPending(pEccAggrRegs, subMemType, errSrc);
        if (SDL_ecc_aggrIsEccRamIntrPending(pEccAggrRegs, subMemType, errSrc, &isPend) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("sdlEccAggr_apiTest: failure on line no. %d \n", __LINE__);
        }
        if (SDL_ecc_aggrIsAnyIntrPending(pEccAggrRegs, subMemType, &isPend) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("sdlEccAggr_apiTest: failure on line no. %d \n", __LINE__);
        }
    }
    if (testStatus == SDL_APP_TEST_PASS)
    {
        mainMem    = SDL_ECC_MEMTYPE_MAX;
        subMemType = SDL_R5FSS0_CORE0_ECC_AGGR_CPU0_ITAG_RAM3_RAM_ID;
        intsrc     = SDL_INJECT_ECC_ERROR_FORCING_2BIT_REPEAT;
        errSrc     = SDL_ECC_AGGR_INTR_SRC_DOUBLE_BIT;
        SDL_ECC_injectError(mainMem, subMemType, intsrc,&injectErrorConfig);
        SDL_ecc_aggrSetEccRamNIntrPending(pEccAggrRegs, subMemType, errSrc, 3U);
        if (SDL_ecc_aggrIsEccRamIntrPending(pEccAggrRegs, subMemType, 0U, &isPend) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("sdlEccAggr_apiTest: failure on line no. %d \n", __LINE__);
        }
    }
    if (testStatus == SDL_APP_TEST_PASS)
    {
        mainMem    = SDL_ECC_MEMTYPE_MAX;
        subMemType = SDL_R5FSS0_CORE0_ECC_AGGR_CPU0_ITAG_RAM3_RAM_ID;
        intsrc     = SDL_INJECT_ECC_ERROR_FORCING_2BIT_REPEAT;
        errSrc     = SDL_ECC_AGGR_INTR_SRC_DOUBLE_BIT;
        SDL_ECC_injectError(mainMem, subMemType, intsrc,&injectErrorConfig);
        SDL_ecc_aggrSetEccRamIntrPending(pEccAggrRegs, subMemType, errSrc);
        if (SDL_ecc_aggrIsEccRamIntrPending(pEccAggrRegs, subMemType, errSrc, &isPend) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("sdlEccAggr_apiTest: failure on line no. %d \n", __LINE__);
        }
        if (SDL_ecc_aggrIsAnyIntrPending(pEccAggrRegs, subMemType, &isPend) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("sdlEccAggr_apiTest: failure on line no. %d \n", __LINE__);
        }
    }
    if (testStatus == SDL_APP_TEST_PASS)
    {
        mainMem    = SDL_ECC_MEMTYPE_MAX;
        subMemType = SDL_R5FSS0_CORE0_ECC_AGGR_CPU0_ITAG_RAM3_RAM_ID;
        intsrc     = SDL_INJECT_ECC_ERROR_FORCING_1BIT_REPEAT;
        errSrc     = SDL_ECC_AGGR_INTR_SRC_SINGLE_BIT;

        SDL_ECC_injectError(mainMem, subMemType, intsrc,&injectErrorConfig);
        SDL_ecc_aggrSetEccRamIntrPending(pEccAggrRegs, subMemType, errSrc);
        if (SDL_ecc_aggrIsEccRamIntrPending(pEccAggrRegs, subMemType, errSrc, &isPend) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("sdlEccAggr_apiTest: failure on line no. %d \n", __LINE__);
        }
        if (SDL_ecc_aggrIsAnyIntrPending(pEccAggrRegs, subMemType, &isPend) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("sdlEccAggr_apiTest: failure on line no. %d \n", __LINE__);
        }
    }
    if (testStatus == SDL_APP_TEST_PASS)
    {
        mainMem    = SDL_ECC_MEMTYPE_MAX;
        subMemType = SDL_R5FSS0_CORE0_ECC_AGGR_CPU0_ITAG_RAM3_RAM_ID;
        intsrc     = SDL_INJECT_ECC_ERROR_FORCING_1BIT_ONCE;
        errSrc     = SDL_ECC_AGGR_INTR_SRC_SINGLE_BIT;

        SDL_ECC_injectError(mainMem, subMemType, intsrc,&injectErrorConfig);
        SDL_ecc_aggrSetEccRamIntrPending(pEccAggrRegs, subMemType, errSrc);
        if (SDL_ecc_aggrIsEccRamIntrPending(pEccAggrRegs, subMemType, errSrc, &isPend) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("sdlEccAggr_apiTest: failure on line no. %d \n", __LINE__);
        }
        if (SDL_ecc_aggrIsAnyIntrPending(pEccAggrRegs, subMemType, &isPend) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("sdlEccAggr_apiTest: failure on line no. %d \n", __LINE__);
        }
    }
    if (testStatus == SDL_APP_TEST_PASS)
    {
        mainMem    = SDL_ECC_MEMTYPE_MAX;
        subMemType = SDL_R5FSS0_CORE0_ECC_AGGR_CPU0_DTAG_RAM0_RAM_ID;
        intsrc     = SDL_INJECT_ECC_ERROR_FORCING_2BIT_REPEAT;
        errSrc     = SDL_ECC_AGGR_INTR_SRC_DOUBLE_BIT;
        SDL_ECC_injectError(mainMem, subMemType, intsrc,&injectErrorConfig);
        SDL_ecc_aggrSetEccRamNIntrPending(pEccAggrRegs, subMemType, errSrc, 3U);
        if (SDL_ecc_aggrIsEccRamIntrPending(pEccAggrRegs, subMemType, 0U, &isPend) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("sdlEccAggr_apiTest: failure on line no. %d \n", __LINE__);
        }
    }
    if (testStatus == SDL_APP_TEST_PASS)
    {
        mainMem    = SDL_ECC_MEMTYPE_MAX;
        subMemType = SDL_R5FSS0_CORE0_ECC_AGGR_CPU0_DTAG_RAM0_RAM_ID;
        intsrc     = SDL_INJECT_ECC_ERROR_FORCING_2BIT_REPEAT;
        errSrc     = SDL_ECC_AGGR_INTR_SRC_DOUBLE_BIT;
        SDL_ECC_injectError(mainMem, subMemType, intsrc,&injectErrorConfig);
        SDL_ecc_aggrSetEccRamIntrPending(pEccAggrRegs, subMemType, errSrc);
        if (SDL_ecc_aggrIsEccRamIntrPending(pEccAggrRegs, subMemType, errSrc, &isPend) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("sdlEccAggr_apiTest: failure on line no. %d \n", __LINE__);
        }
        if (SDL_ecc_aggrIsAnyIntrPending(pEccAggrRegs, subMemType, &isPend) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("sdlEccAggr_apiTest: failure on line no. %d \n", __LINE__);
        }
    }
    if (testStatus == SDL_APP_TEST_PASS)
    {
        mainMem    = SDL_ECC_MEMTYPE_MAX;
        subMemType = SDL_R5FSS0_CORE0_ECC_AGGR_CPU0_DTAG_RAM0_RAM_ID;
        intsrc     = SDL_INJECT_ECC_ERROR_FORCING_1BIT_REPEAT;
        errSrc     = SDL_ECC_AGGR_INTR_SRC_SINGLE_BIT;

        SDL_ECC_injectError(mainMem, subMemType, intsrc,&injectErrorConfig);
        SDL_ecc_aggrSetEccRamIntrPending(pEccAggrRegs, subMemType, errSrc);
        if (SDL_ecc_aggrIsEccRamIntrPending(pEccAggrRegs, subMemType, errSrc, &isPend) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("sdlEccAggr_apiTest: failure on line no. %d \n", __LINE__);
        }
        if (SDL_ecc_aggrIsAnyIntrPending(pEccAggrRegs, subMemType, &isPend) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("sdlEccAggr_apiTest: failure on line no. %d \n", __LINE__);
        }
    }
    if (testStatus == SDL_APP_TEST_PASS)
    {
        mainMem    = SDL_ECC_MEMTYPE_MAX;
        subMemType = SDL_R5FSS0_CORE0_ECC_AGGR_CPU0_DTAG_RAM0_RAM_ID;
        intsrc     = SDL_INJECT_ECC_ERROR_FORCING_1BIT_ONCE;
        errSrc     = SDL_ECC_AGGR_INTR_SRC_SINGLE_BIT;

        SDL_ECC_injectError(mainMem, subMemType, intsrc,&injectErrorConfig);
        SDL_ecc_aggrSetEccRamIntrPending(pEccAggrRegs, subMemType, errSrc);
        if (SDL_ecc_aggrIsEccRamIntrPending(pEccAggrRegs, subMemType, errSrc, &isPend) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("sdlEccAggr_apiTest: failure on line no. %d \n", __LINE__);
        }
        if (SDL_ecc_aggrIsAnyIntrPending(pEccAggrRegs, subMemType, &isPend) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("sdlEccAggr_apiTest: failure on line no. %d \n", __LINE__);
        }
    }

    if (testStatus == SDL_APP_TEST_PASS)
    {
        mainMem    = SDL_ECC_MEMTYPE_MAX;
        subMemType = SDL_R5FSS0_CORE0_ECC_AGGR_CPU0_DTAG_RAM1_RAM_ID;
        intsrc     = SDL_INJECT_ECC_ERROR_FORCING_2BIT_REPEAT;
        errSrc     = SDL_ECC_AGGR_INTR_SRC_DOUBLE_BIT;
        SDL_ECC_injectError(mainMem, subMemType, intsrc,&injectErrorConfig);
        SDL_ecc_aggrSetEccRamNIntrPending(pEccAggrRegs, subMemType, errSrc, 3U);
        if (SDL_ecc_aggrIsEccRamIntrPending(pEccAggrRegs, subMemType, 0U, &isPend) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("sdlEccAggr_apiTest: failure on line no. %d \n", __LINE__);
        }
    }
    if (testStatus == SDL_APP_TEST_PASS)
    {
        mainMem    = SDL_ECC_MEMTYPE_MAX;
        subMemType = SDL_R5FSS0_CORE0_ECC_AGGR_CPU0_DTAG_RAM1_RAM_ID;
        intsrc     = SDL_INJECT_ECC_ERROR_FORCING_2BIT_REPEAT;
        errSrc     = SDL_ECC_AGGR_INTR_SRC_DOUBLE_BIT;
        SDL_ECC_injectError(mainMem, subMemType, intsrc,&injectErrorConfig);
        SDL_ecc_aggrSetEccRamIntrPending(pEccAggrRegs, subMemType, errSrc);
        if (SDL_ecc_aggrIsEccRamIntrPending(pEccAggrRegs, subMemType, errSrc, &isPend) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("sdlEccAggr_apiTest: failure on line no. %d \n", __LINE__);
        }
        if (SDL_ecc_aggrIsAnyIntrPending(pEccAggrRegs, subMemType, &isPend) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("sdlEccAggr_apiTest: failure on line no. %d \n", __LINE__);
        }
    }
    if (testStatus == SDL_APP_TEST_PASS)
    {
        mainMem    = SDL_ECC_MEMTYPE_MAX;
        subMemType = SDL_R5FSS0_CORE0_ECC_AGGR_CPU0_DTAG_RAM1_RAM_ID;
        intsrc     = SDL_INJECT_ECC_ERROR_FORCING_1BIT_REPEAT;
        errSrc     = SDL_ECC_AGGR_INTR_SRC_SINGLE_BIT;

        SDL_ECC_injectError(mainMem, subMemType, intsrc,&injectErrorConfig);
        SDL_ecc_aggrSetEccRamIntrPending(pEccAggrRegs, subMemType, errSrc);
        if (SDL_ecc_aggrIsEccRamIntrPending(pEccAggrRegs, subMemType, errSrc, &isPend) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("sdlEccAggr_apiTest: failure on line no. %d \n", __LINE__);
        }
        if (SDL_ecc_aggrIsAnyIntrPending(pEccAggrRegs, subMemType, &isPend) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("sdlEccAggr_apiTest: failure on line no. %d \n", __LINE__);
        }
    }
    if (testStatus == SDL_APP_TEST_PASS)
    {
        mainMem    = SDL_ECC_MEMTYPE_MAX;
        subMemType = SDL_R5FSS0_CORE0_ECC_AGGR_CPU0_DTAG_RAM1_RAM_ID;
        intsrc     = SDL_INJECT_ECC_ERROR_FORCING_1BIT_ONCE;
        errSrc     = SDL_ECC_AGGR_INTR_SRC_SINGLE_BIT;

        SDL_ECC_injectError(mainMem, subMemType, intsrc,&injectErrorConfig);
        SDL_ecc_aggrSetEccRamIntrPending(pEccAggrRegs, subMemType, errSrc);
        if (SDL_ecc_aggrIsEccRamIntrPending(pEccAggrRegs, subMemType, errSrc, &isPend) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("sdlEccAggr_apiTest: failure on line no. %d \n", __LINE__);
        }
        if (SDL_ecc_aggrIsAnyIntrPending(pEccAggrRegs, subMemType, &isPend) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("sdlEccAggr_apiTest: failure on line no. %d \n", __LINE__);
        }
    }

    if (testStatus == SDL_APP_TEST_PASS)
    {
        mainMem    = SDL_ECC_MEMTYPE_MAX;
        subMemType = SDL_R5FSS0_CORE0_ECC_AGGR_CPU0_DTAG_RAM2_RAM_ID;
        intsrc     = SDL_INJECT_ECC_ERROR_FORCING_2BIT_REPEAT;
        errSrc     = SDL_ECC_AGGR_INTR_SRC_DOUBLE_BIT;
        SDL_ECC_injectError(mainMem, subMemType, intsrc,&injectErrorConfig);
        SDL_ecc_aggrSetEccRamNIntrPending(pEccAggrRegs, subMemType, errSrc, 3U);
        if (SDL_ecc_aggrIsEccRamIntrPending(pEccAggrRegs, subMemType, 0U, &isPend) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("sdlEccAggr_apiTest: failure on line no. %d \n", __LINE__);
        }
    }
    if (testStatus == SDL_APP_TEST_PASS)
    {
        mainMem    = SDL_ECC_MEMTYPE_MAX;
        subMemType = SDL_R5FSS0_CORE0_ECC_AGGR_CPU0_DTAG_RAM2_RAM_ID;
        intsrc     = SDL_INJECT_ECC_ERROR_FORCING_2BIT_REPEAT;
        errSrc     = SDL_ECC_AGGR_INTR_SRC_DOUBLE_BIT;
        SDL_ECC_injectError(mainMem, subMemType, intsrc,&injectErrorConfig);
        SDL_ecc_aggrSetEccRamIntrPending(pEccAggrRegs, subMemType, errSrc);
        if (SDL_ecc_aggrIsEccRamIntrPending(pEccAggrRegs, subMemType, errSrc, &isPend) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("sdlEccAggr_apiTest: failure on line no. %d \n", __LINE__);
        }
        if (SDL_ecc_aggrIsAnyIntrPending(pEccAggrRegs, subMemType, &isPend) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("sdlEccAggr_apiTest: failure on line no. %d \n", __LINE__);
        }
    }
    if (testStatus == SDL_APP_TEST_PASS)
    {
        mainMem    = SDL_ECC_MEMTYPE_MAX;
        subMemType = SDL_R5FSS0_CORE0_ECC_AGGR_CPU0_DTAG_RAM2_RAM_ID;
        intsrc     = SDL_INJECT_ECC_ERROR_FORCING_1BIT_REPEAT;
        errSrc     = SDL_ECC_AGGR_INTR_SRC_SINGLE_BIT;

        SDL_ECC_injectError(mainMem, subMemType, intsrc,&injectErrorConfig);
        SDL_ecc_aggrSetEccRamIntrPending(pEccAggrRegs, subMemType, errSrc);
        if (SDL_ecc_aggrIsEccRamIntrPending(pEccAggrRegs, subMemType, errSrc, &isPend) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("sdlEccAggr_apiTest: failure on line no. %d \n", __LINE__);
        }
        if (SDL_ecc_aggrIsAnyIntrPending(pEccAggrRegs, subMemType, &isPend) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("sdlEccAggr_apiTest: failure on line no. %d \n", __LINE__);
        }
    }
    if (testStatus == SDL_APP_TEST_PASS)
    {
        mainMem    = SDL_ECC_MEMTYPE_MAX;
        subMemType = SDL_R5FSS0_CORE0_ECC_AGGR_CPU0_DTAG_RAM2_RAM_ID;
        intsrc     = SDL_INJECT_ECC_ERROR_FORCING_1BIT_ONCE;
        errSrc     = SDL_ECC_AGGR_INTR_SRC_SINGLE_BIT;

        SDL_ECC_injectError(mainMem, subMemType, intsrc,&injectErrorConfig);
        SDL_ecc_aggrSetEccRamIntrPending(pEccAggrRegs, subMemType, errSrc);
        if (SDL_ecc_aggrIsEccRamIntrPending(pEccAggrRegs, subMemType, errSrc, &isPend) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("sdlEccAggr_apiTest: failure on line no. %d \n", __LINE__);
        }
        if (SDL_ecc_aggrIsAnyIntrPending(pEccAggrRegs, subMemType, &isPend) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("sdlEccAggr_apiTest: failure on line no. %d \n", __LINE__);
        }
    }
    if (testStatus == SDL_APP_TEST_PASS)
    {
        mainMem    = SDL_ECC_MEMTYPE_MAX;
        subMemType = SDL_R5FSS0_CORE0_ECC_AGGR_CPU0_DTAG_RAM3_RAM_ID;
        intsrc     = SDL_INJECT_ECC_ERROR_FORCING_2BIT_REPEAT;
        errSrc     = SDL_ECC_AGGR_INTR_SRC_DOUBLE_BIT;
        SDL_ECC_injectError(mainMem, subMemType, intsrc,&injectErrorConfig);
        SDL_ecc_aggrSetEccRamNIntrPending(pEccAggrRegs, subMemType, errSrc, 3U);
        if (SDL_ecc_aggrIsEccRamIntrPending(pEccAggrRegs, subMemType, 0U, &isPend) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("sdlEccAggr_apiTest: failure on line no. %d \n", __LINE__);
        }
    }
    if (testStatus == SDL_APP_TEST_PASS)
    {
        mainMem    = SDL_ECC_MEMTYPE_MAX;
        subMemType = SDL_R5FSS0_CORE0_ECC_AGGR_CPU0_DTAG_RAM3_RAM_ID;
        intsrc     = SDL_INJECT_ECC_ERROR_FORCING_2BIT_REPEAT;
        errSrc     = SDL_ECC_AGGR_INTR_SRC_DOUBLE_BIT;
        SDL_ECC_injectError(mainMem, subMemType, intsrc,&injectErrorConfig);
        SDL_ecc_aggrSetEccRamIntrPending(pEccAggrRegs, subMemType, errSrc);
        if (SDL_ecc_aggrIsEccRamIntrPending(pEccAggrRegs, subMemType, errSrc, &isPend) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("sdlEccAggr_apiTest: failure on line no. %d \n", __LINE__);
        }
        if (SDL_ecc_aggrIsAnyIntrPending(pEccAggrRegs, subMemType, &isPend) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("sdlEccAggr_apiTest: failure on line no. %d \n", __LINE__);
        }
    }
    if (testStatus == SDL_APP_TEST_PASS)
    {
        mainMem    = SDL_ECC_MEMTYPE_MAX;
        subMemType = SDL_R5FSS0_CORE0_ECC_AGGR_CPU0_DTAG_RAM3_RAM_ID;
        intsrc     = SDL_INJECT_ECC_ERROR_FORCING_1BIT_REPEAT;
        errSrc     = SDL_ECC_AGGR_INTR_SRC_SINGLE_BIT;

        SDL_ECC_injectError(mainMem, subMemType, intsrc,&injectErrorConfig);
        SDL_ecc_aggrSetEccRamIntrPending(pEccAggrRegs, subMemType, errSrc);
        if (SDL_ecc_aggrIsEccRamIntrPending(pEccAggrRegs, subMemType, errSrc, &isPend) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("sdlEccAggr_apiTest: failure on line no. %d \n", __LINE__);
        }
        if (SDL_ecc_aggrIsAnyIntrPending(pEccAggrRegs, subMemType, &isPend) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("sdlEccAggr_apiTest: failure on line no. %d \n", __LINE__);
        }
    }
    if (testStatus == SDL_APP_TEST_PASS)
    {
        mainMem    = SDL_ECC_MEMTYPE_MAX;
        subMemType = SDL_R5FSS0_CORE0_ECC_AGGR_CPU0_DTAG_RAM3_RAM_ID;
        intsrc     = SDL_INJECT_ECC_ERROR_FORCING_1BIT_ONCE;
        errSrc     = SDL_ECC_AGGR_INTR_SRC_SINGLE_BIT;

        SDL_ECC_injectError(mainMem, subMemType, intsrc,&injectErrorConfig);
        SDL_ecc_aggrSetEccRamIntrPending(pEccAggrRegs, subMemType, errSrc);
        if (SDL_ecc_aggrIsEccRamIntrPending(pEccAggrRegs, subMemType, errSrc, &isPend) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("sdlEccAggr_apiTest: failure on line no. %d \n", __LINE__);
        }
        if (SDL_ecc_aggrIsAnyIntrPending(pEccAggrRegs, subMemType, &isPend) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("sdlEccAggr_apiTest: failure on line no. %d \n", __LINE__);
        }
    }

    if (testStatus == SDL_APP_TEST_PASS)
    {
        mainMem    = SDL_ECC_MEMTYPE_MAX;
        subMemType = SDL_R5FSS0_CORE0_ECC_AGGR_CPU0_DDIRTY_RAM_RAM_ID;
        intsrc     = SDL_INJECT_ECC_ERROR_FORCING_2BIT_REPEAT;
        errSrc     = SDL_ECC_AGGR_INTR_SRC_DOUBLE_BIT;
        SDL_ECC_injectError(mainMem, subMemType, intsrc,&injectErrorConfig);
        SDL_ecc_aggrSetEccRamNIntrPending(pEccAggrRegs, subMemType, errSrc, 3U);
        if (SDL_ecc_aggrIsEccRamIntrPending(pEccAggrRegs, subMemType, 0U, &isPend) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("sdlEccAggr_apiTest: failure on line no. %d \n", __LINE__);
        }
    }
    if (testStatus == SDL_APP_TEST_PASS)
    {
        mainMem    = SDL_ECC_MEMTYPE_MAX;
        subMemType = SDL_R5FSS0_CORE0_ECC_AGGR_CPU0_DDIRTY_RAM_RAM_ID;
        intsrc     = SDL_INJECT_ECC_ERROR_FORCING_2BIT_REPEAT;
        errSrc     = SDL_ECC_AGGR_INTR_SRC_DOUBLE_BIT;
        SDL_ECC_injectError(mainMem, subMemType, intsrc,&injectErrorConfig);
        SDL_ecc_aggrSetEccRamIntrPending(pEccAggrRegs, subMemType, errSrc);
        if (SDL_ecc_aggrIsEccRamIntrPending(pEccAggrRegs, subMemType, errSrc, &isPend) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("sdlEccAggr_apiTest: failure on line no. %d \n", __LINE__);
        }
        if (SDL_ecc_aggrIsAnyIntrPending(pEccAggrRegs, subMemType, &isPend) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("sdlEccAggr_apiTest: failure on line no. %d \n", __LINE__);
        }
    }
    if (testStatus == SDL_APP_TEST_PASS)
    {
        mainMem    = SDL_ECC_MEMTYPE_MAX;
        subMemType = SDL_R5FSS0_CORE0_ECC_AGGR_CPU0_DDIRTY_RAM_RAM_ID;
        intsrc     = SDL_INJECT_ECC_ERROR_FORCING_1BIT_REPEAT;
        errSrc     = SDL_ECC_AGGR_INTR_SRC_SINGLE_BIT;

        SDL_ECC_injectError(mainMem, subMemType, intsrc,&injectErrorConfig);
        SDL_ecc_aggrSetEccRamIntrPending(pEccAggrRegs, subMemType, errSrc);
        if (SDL_ecc_aggrIsEccRamIntrPending(pEccAggrRegs, subMemType, errSrc, &isPend) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("sdlEccAggr_apiTest: failure on line no. %d \n", __LINE__);
        }
        if (SDL_ecc_aggrIsAnyIntrPending(pEccAggrRegs, subMemType, &isPend) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("sdlEccAggr_apiTest: failure on line no. %d \n", __LINE__);
        }
    }
    if (testStatus == SDL_APP_TEST_PASS)
    {
        mainMem    = SDL_ECC_MEMTYPE_MAX;
        subMemType = SDL_R5FSS0_CORE0_ECC_AGGR_CPU0_DDIRTY_RAM_RAM_ID;
        intsrc     = SDL_INJECT_ECC_ERROR_FORCING_1BIT_ONCE;
        errSrc     = SDL_ECC_AGGR_INTR_SRC_SINGLE_BIT;

        SDL_ECC_injectError(mainMem, subMemType, intsrc,&injectErrorConfig);
        SDL_ecc_aggrSetEccRamIntrPending(pEccAggrRegs, subMemType, errSrc);
        if (SDL_ecc_aggrIsEccRamIntrPending(pEccAggrRegs, subMemType, errSrc, &isPend) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("sdlEccAggr_apiTest: failure on line no. %d \n", __LINE__);
        }
        if (SDL_ecc_aggrIsAnyIntrPending(pEccAggrRegs, subMemType, &isPend) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("sdlEccAggr_apiTest: failure on line no. %d \n", __LINE__);
        }
    }

    if (testStatus == SDL_APP_TEST_PASS)
    {
        mainMem    = SDL_ECC_MEMTYPE_MAX;
        subMemType = SDL_R5FSS0_CORE0_ECC_AGGR_CPU0_DDATA_RAM0_RAM_ID;
        intsrc     = SDL_INJECT_ECC_ERROR_FORCING_2BIT_REPEAT;
        errSrc     = SDL_ECC_AGGR_INTR_SRC_DOUBLE_BIT;
        SDL_ECC_injectError(mainMem, subMemType, intsrc,&injectErrorConfig);
        SDL_ecc_aggrSetEccRamNIntrPending(pEccAggrRegs, subMemType, errSrc, 3U);
        if (SDL_ecc_aggrIsEccRamIntrPending(pEccAggrRegs, subMemType, 0U, &isPend) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("sdlEccAggr_apiTest: failure on line no. %d \n", __LINE__);
        }
    }
    if (testStatus == SDL_APP_TEST_PASS)
    {
        mainMem    = SDL_ECC_MEMTYPE_MAX;
        subMemType = SDL_R5FSS0_CORE0_ECC_AGGR_CPU0_DDATA_RAM0_RAM_ID;
        intsrc     = SDL_INJECT_ECC_ERROR_FORCING_2BIT_REPEAT;
        errSrc     = SDL_ECC_AGGR_INTR_SRC_DOUBLE_BIT;
        SDL_ECC_injectError(mainMem, subMemType, intsrc,&injectErrorConfig);
        SDL_ecc_aggrSetEccRamIntrPending(pEccAggrRegs, subMemType, errSrc);
        if (SDL_ecc_aggrIsEccRamIntrPending(pEccAggrRegs, subMemType, errSrc, &isPend) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("sdlEccAggr_apiTest: failure on line no. %d \n", __LINE__);
        }
        if (SDL_ecc_aggrIsAnyIntrPending(pEccAggrRegs, subMemType, &isPend) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("sdlEccAggr_apiTest: failure on line no. %d \n", __LINE__);
        }
    }
    if (testStatus == SDL_APP_TEST_PASS)
    {
        mainMem    = SDL_ECC_MEMTYPE_MAX;
        subMemType = SDL_R5FSS0_CORE0_ECC_AGGR_CPU0_DDATA_RAM0_RAM_ID;
        intsrc     = SDL_INJECT_ECC_ERROR_FORCING_1BIT_REPEAT;
        errSrc     = SDL_ECC_AGGR_INTR_SRC_SINGLE_BIT;

        SDL_ECC_injectError(mainMem, subMemType, intsrc,&injectErrorConfig);
        SDL_ecc_aggrSetEccRamIntrPending(pEccAggrRegs, subMemType, errSrc);
        if (SDL_ecc_aggrIsEccRamIntrPending(pEccAggrRegs, subMemType, errSrc, &isPend) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("sdlEccAggr_apiTest: failure on line no. %d \n", __LINE__);
        }
        if (SDL_ecc_aggrIsAnyIntrPending(pEccAggrRegs, subMemType, &isPend) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("sdlEccAggr_apiTest: failure on line no. %d \n", __LINE__);
        }
    }
    if (testStatus == SDL_APP_TEST_PASS)
    {
        mainMem    = SDL_ECC_MEMTYPE_MAX;
        subMemType = SDL_R5FSS0_CORE0_ECC_AGGR_CPU0_DDATA_RAM0_RAM_ID;
        intsrc     = SDL_INJECT_ECC_ERROR_FORCING_1BIT_ONCE;
        errSrc     = SDL_ECC_AGGR_INTR_SRC_SINGLE_BIT;

        SDL_ECC_injectError(mainMem, subMemType, intsrc,&injectErrorConfig);
        SDL_ecc_aggrSetEccRamIntrPending(pEccAggrRegs, subMemType, errSrc);
        if (SDL_ecc_aggrIsEccRamIntrPending(pEccAggrRegs, subMemType, errSrc, &isPend) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("sdlEccAggr_apiTest: failure on line no. %d \n", __LINE__);
        }
        if (SDL_ecc_aggrIsAnyIntrPending(pEccAggrRegs, subMemType, &isPend) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("sdlEccAggr_apiTest: failure on line no. %d \n", __LINE__);
        }
    }

    if (testStatus == SDL_APP_TEST_PASS)
    {
        mainMem    = SDL_ECC_MEMTYPE_MAX;
        subMemType = SDL_R5FSS0_CORE0_ECC_AGGR_CPU0_DDATA_RAM1_RAM_ID;
        intsrc     = SDL_INJECT_ECC_ERROR_FORCING_2BIT_REPEAT;
        errSrc     = SDL_ECC_AGGR_INTR_SRC_DOUBLE_BIT;
        SDL_ECC_injectError(mainMem, subMemType, intsrc,&injectErrorConfig);
        SDL_ecc_aggrSetEccRamNIntrPending(pEccAggrRegs, subMemType, errSrc, 3U);
        if (SDL_ecc_aggrIsEccRamIntrPending(pEccAggrRegs, subMemType, 0U, &isPend) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("sdlEccAggr_apiTest: failure on line no. %d \n", __LINE__);
        }
    }
    if (testStatus == SDL_APP_TEST_PASS)
    {
        mainMem    = SDL_ECC_MEMTYPE_MAX;
        subMemType = SDL_R5FSS0_CORE0_ECC_AGGR_CPU0_DDATA_RAM1_RAM_ID;
        intsrc     = SDL_INJECT_ECC_ERROR_FORCING_2BIT_REPEAT;
        errSrc     = SDL_ECC_AGGR_INTR_SRC_DOUBLE_BIT;
        SDL_ECC_injectError(mainMem, subMemType, intsrc,&injectErrorConfig);
        SDL_ecc_aggrSetEccRamIntrPending(pEccAggrRegs, subMemType, errSrc);
        if (SDL_ecc_aggrIsEccRamIntrPending(pEccAggrRegs, subMemType, errSrc, &isPend) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("sdlEccAggr_apiTest: failure on line no. %d \n", __LINE__);
        }
        if (SDL_ecc_aggrIsAnyIntrPending(pEccAggrRegs, subMemType, &isPend) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("sdlEccAggr_apiTest: failure on line no. %d \n", __LINE__);
        }
    }
    if (testStatus == SDL_APP_TEST_PASS)
    {
        mainMem    = SDL_ECC_MEMTYPE_MAX;
        subMemType = SDL_R5FSS0_CORE0_ECC_AGGR_CPU0_DDATA_RAM1_RAM_ID;
        intsrc     = SDL_INJECT_ECC_ERROR_FORCING_1BIT_REPEAT;
        errSrc     = SDL_ECC_AGGR_INTR_SRC_SINGLE_BIT;

        SDL_ECC_injectError(mainMem, subMemType, intsrc,&injectErrorConfig);
        SDL_ecc_aggrSetEccRamIntrPending(pEccAggrRegs, subMemType, errSrc);
        if (SDL_ecc_aggrIsEccRamIntrPending(pEccAggrRegs, subMemType, errSrc, &isPend) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("sdlEccAggr_apiTest: failure on line no. %d \n", __LINE__);
        }
        if (SDL_ecc_aggrIsAnyIntrPending(pEccAggrRegs, subMemType, &isPend) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("sdlEccAggr_apiTest: failure on line no. %d \n", __LINE__);
        }
    }
    if (testStatus == SDL_APP_TEST_PASS)
    {
        mainMem    = SDL_ECC_MEMTYPE_MAX;
        subMemType = SDL_R5FSS0_CORE0_ECC_AGGR_CPU0_DDATA_RAM1_RAM_ID;
        intsrc     = SDL_INJECT_ECC_ERROR_FORCING_1BIT_ONCE;
        errSrc     = SDL_ECC_AGGR_INTR_SRC_SINGLE_BIT;

        SDL_ECC_injectError(mainMem, subMemType, intsrc,&injectErrorConfig);
        SDL_ecc_aggrSetEccRamIntrPending(pEccAggrRegs, subMemType, errSrc);
        if (SDL_ecc_aggrIsEccRamIntrPending(pEccAggrRegs, subMemType, errSrc, &isPend) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("sdlEccAggr_apiTest: failure on line no. %d \n", __LINE__);
        }
        if (SDL_ecc_aggrIsAnyIntrPending(pEccAggrRegs, subMemType, &isPend) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("sdlEccAggr_apiTest: failure on line no. %d \n", __LINE__);
        }
    }

    if (testStatus == SDL_APP_TEST_PASS)
    {
        mainMem    = SDL_ECC_MEMTYPE_MAX;
        subMemType = SDL_R5FSS0_CORE0_ECC_AGGR_CPU0_DDATA_RAM2_RAM_ID;
        intsrc     = SDL_INJECT_ECC_ERROR_FORCING_2BIT_REPEAT;
        errSrc     = SDL_ECC_AGGR_INTR_SRC_DOUBLE_BIT;
        SDL_ECC_injectError(mainMem, subMemType, intsrc,&injectErrorConfig);
        SDL_ecc_aggrSetEccRamNIntrPending(pEccAggrRegs, subMemType, errSrc, 3U);
        if (SDL_ecc_aggrIsEccRamIntrPending(pEccAggrRegs, subMemType, 0U, &isPend) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("sdlEccAggr_apiTest: failure on line no. %d \n", __LINE__);
        }
    }
    if (testStatus == SDL_APP_TEST_PASS)
    {
        mainMem    = SDL_ECC_MEMTYPE_MAX;
        subMemType = SDL_R5FSS0_CORE0_ECC_AGGR_CPU0_DDATA_RAM2_RAM_ID;
        intsrc     = SDL_INJECT_ECC_ERROR_FORCING_2BIT_REPEAT;
        errSrc     = SDL_ECC_AGGR_INTR_SRC_DOUBLE_BIT;
        SDL_ECC_injectError(mainMem, subMemType, intsrc,&injectErrorConfig);
        SDL_ecc_aggrSetEccRamIntrPending(pEccAggrRegs, subMemType, errSrc);
        if (SDL_ecc_aggrIsEccRamIntrPending(pEccAggrRegs, subMemType, errSrc, &isPend) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("sdlEccAggr_apiTest: failure on line no. %d \n", __LINE__);
        }
        if (SDL_ecc_aggrIsAnyIntrPending(pEccAggrRegs, subMemType, &isPend) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("sdlEccAggr_apiTest: failure on line no. %d \n", __LINE__);
        }
    }
    if (testStatus == SDL_APP_TEST_PASS)
    {
        mainMem    = SDL_ECC_MEMTYPE_MAX;
        subMemType = SDL_R5FSS0_CORE0_ECC_AGGR_CPU0_DDATA_RAM2_RAM_ID;
        intsrc     = SDL_INJECT_ECC_ERROR_FORCING_1BIT_REPEAT;
        errSrc     = SDL_ECC_AGGR_INTR_SRC_SINGLE_BIT;

        SDL_ECC_injectError(mainMem, subMemType, intsrc,&injectErrorConfig);
        SDL_ecc_aggrSetEccRamIntrPending(pEccAggrRegs, subMemType, errSrc);
        if (SDL_ecc_aggrIsEccRamIntrPending(pEccAggrRegs, subMemType, errSrc, &isPend) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("sdlEccAggr_apiTest: failure on line no. %d \n", __LINE__);
        }
        if (SDL_ecc_aggrIsAnyIntrPending(pEccAggrRegs, subMemType, &isPend) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("sdlEccAggr_apiTest: failure on line no. %d \n", __LINE__);
        }
    }
    if (testStatus == SDL_APP_TEST_PASS)
    {
        mainMem    = SDL_ECC_MEMTYPE_MAX;
        subMemType = SDL_R5FSS0_CORE0_ECC_AGGR_CPU0_DDATA_RAM2_RAM_ID;
        intsrc     = SDL_INJECT_ECC_ERROR_FORCING_1BIT_ONCE;
        errSrc     = SDL_ECC_AGGR_INTR_SRC_SINGLE_BIT;

        SDL_ECC_injectError(mainMem, subMemType, intsrc,&injectErrorConfig);
        SDL_ecc_aggrSetEccRamIntrPending(pEccAggrRegs, subMemType, errSrc);
        if (SDL_ecc_aggrIsEccRamIntrPending(pEccAggrRegs, subMemType, errSrc, &isPend) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("sdlEccAggr_apiTest: failure on line no. %d \n", __LINE__);
        }
        if (SDL_ecc_aggrIsAnyIntrPending(pEccAggrRegs, subMemType, &isPend) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("sdlEccAggr_apiTest: failure on line no. %d \n", __LINE__);
        }
    }

    if (testStatus == SDL_APP_TEST_PASS)
    {
        mainMem    = SDL_ECC_MEMTYPE_MAX;
        subMemType = SDL_R5FSS0_CORE0_ECC_AGGR_CPU0_DDATA_RAM3_RAM_ID;
        intsrc     = SDL_INJECT_ECC_ERROR_FORCING_2BIT_REPEAT;
        errSrc     = SDL_ECC_AGGR_INTR_SRC_DOUBLE_BIT;
        SDL_ECC_injectError(mainMem, subMemType, intsrc,&injectErrorConfig);
        SDL_ecc_aggrSetEccRamNIntrPending(pEccAggrRegs, subMemType, errSrc, 3U);
        if (SDL_ecc_aggrIsEccRamIntrPending(pEccAggrRegs, subMemType, 0U, &isPend) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("sdlEccAggr_apiTest: failure on line no. %d \n", __LINE__);
        }
    }
    if (testStatus == SDL_APP_TEST_PASS)
    {
        mainMem    = SDL_ECC_MEMTYPE_MAX;
        subMemType = SDL_R5FSS0_CORE0_ECC_AGGR_CPU0_DDATA_RAM3_RAM_ID;
        intsrc     = SDL_INJECT_ECC_ERROR_FORCING_2BIT_REPEAT;
        errSrc     = SDL_ECC_AGGR_INTR_SRC_DOUBLE_BIT;
        SDL_ECC_injectError(mainMem, subMemType, intsrc,&injectErrorConfig);
        SDL_ecc_aggrSetEccRamIntrPending(pEccAggrRegs, subMemType, errSrc);
        if (SDL_ecc_aggrIsEccRamIntrPending(pEccAggrRegs, subMemType, errSrc, &isPend) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("sdlEccAggr_apiTest: failure on line no. %d \n", __LINE__);
        }
        if (SDL_ecc_aggrIsAnyIntrPending(pEccAggrRegs, subMemType, &isPend) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("sdlEccAggr_apiTest: failure on line no. %d \n", __LINE__);
        }
    }
    if (testStatus == SDL_APP_TEST_PASS)
    {
        mainMem    = SDL_ECC_MEMTYPE_MAX;
        subMemType = SDL_R5FSS0_CORE0_ECC_AGGR_CPU0_DDATA_RAM3_RAM_ID;
        intsrc     = SDL_INJECT_ECC_ERROR_FORCING_1BIT_REPEAT;
        errSrc     = SDL_ECC_AGGR_INTR_SRC_SINGLE_BIT;

        SDL_ECC_injectError(mainMem, subMemType, intsrc,&injectErrorConfig);
        SDL_ecc_aggrSetEccRamIntrPending(pEccAggrRegs, subMemType, errSrc);
        if (SDL_ecc_aggrIsEccRamIntrPending(pEccAggrRegs, subMemType, errSrc, &isPend) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("sdlEccAggr_apiTest: failure on line no. %d \n", __LINE__);
        }
        if (SDL_ecc_aggrIsAnyIntrPending(pEccAggrRegs, subMemType, &isPend) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("sdlEccAggr_apiTest: failure on line no. %d \n", __LINE__);
        }
    }
    if (testStatus == SDL_APP_TEST_PASS)
    {
        mainMem    = SDL_ECC_MEMTYPE_MAX;
        subMemType = SDL_R5FSS0_CORE0_ECC_AGGR_CPU0_DDATA_RAM3_RAM_ID;
        intsrc     = SDL_INJECT_ECC_ERROR_FORCING_1BIT_ONCE;
        errSrc     = SDL_ECC_AGGR_INTR_SRC_SINGLE_BIT;

        SDL_ECC_injectError(mainMem, subMemType, intsrc,&injectErrorConfig);
        SDL_ecc_aggrSetEccRamIntrPending(pEccAggrRegs, subMemType, errSrc);
        if (SDL_ecc_aggrIsEccRamIntrPending(pEccAggrRegs, subMemType, errSrc, &isPend) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("sdlEccAggr_apiTest: failure on line no. %d \n", __LINE__);
        }
        if (SDL_ecc_aggrIsAnyIntrPending(pEccAggrRegs, subMemType, &isPend) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("sdlEccAggr_apiTest: failure on line no. %d \n", __LINE__);
        }
    }

    if (testStatus == SDL_APP_TEST_PASS)
    {
        mainMem    = SDL_ECC_MEMTYPE_MAX;
        subMemType = SDL_R5FSS0_CORE0_ECC_AGGR_CPU0_DDATA_RAM4_RAM_ID;
        intsrc     = SDL_INJECT_ECC_ERROR_FORCING_2BIT_REPEAT;
        errSrc     = SDL_ECC_AGGR_INTR_SRC_DOUBLE_BIT;
        SDL_ECC_injectError(mainMem, subMemType, intsrc,&injectErrorConfig);
        SDL_ecc_aggrSetEccRamNIntrPending(pEccAggrRegs, subMemType, errSrc, 3U);
        if (SDL_ecc_aggrIsEccRamIntrPending(pEccAggrRegs, subMemType, 0U, &isPend) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("sdlEccAggr_apiTest: failure on line no. %d \n", __LINE__);
        }
    }
    if (testStatus == SDL_APP_TEST_PASS)
    {
        mainMem    = SDL_ECC_MEMTYPE_MAX;
        subMemType = SDL_R5FSS0_CORE0_ECC_AGGR_CPU0_DDATA_RAM4_RAM_ID;
        intsrc     = SDL_INJECT_ECC_ERROR_FORCING_2BIT_REPEAT;
        errSrc     = SDL_ECC_AGGR_INTR_SRC_DOUBLE_BIT;
        SDL_ECC_injectError(mainMem, subMemType, intsrc,&injectErrorConfig);
        SDL_ecc_aggrSetEccRamIntrPending(pEccAggrRegs, subMemType, errSrc);
        if (SDL_ecc_aggrIsEccRamIntrPending(pEccAggrRegs, subMemType, errSrc, &isPend) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("sdlEccAggr_apiTest: failure on line no. %d \n", __LINE__);
        }
        if (SDL_ecc_aggrIsAnyIntrPending(pEccAggrRegs, subMemType, &isPend) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("sdlEccAggr_apiTest: failure on line no. %d \n", __LINE__);
        }
    }
    if (testStatus == SDL_APP_TEST_PASS)
    {
        mainMem    = SDL_ECC_MEMTYPE_MAX;
        subMemType = SDL_R5FSS0_CORE0_ECC_AGGR_CPU0_DDATA_RAM4_RAM_ID;
        intsrc     = SDL_INJECT_ECC_ERROR_FORCING_1BIT_REPEAT;
        errSrc     = SDL_ECC_AGGR_INTR_SRC_SINGLE_BIT;

        SDL_ECC_injectError(mainMem, subMemType, intsrc,&injectErrorConfig);
        SDL_ecc_aggrSetEccRamIntrPending(pEccAggrRegs, subMemType, errSrc);
        if (SDL_ecc_aggrIsEccRamIntrPending(pEccAggrRegs, subMemType, errSrc, &isPend) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("sdlEccAggr_apiTest: failure on line no. %d \n", __LINE__);
        }
        if (SDL_ecc_aggrIsAnyIntrPending(pEccAggrRegs, subMemType, &isPend) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("sdlEccAggr_apiTest: failure on line no. %d \n", __LINE__);
        }
    }
    if (testStatus == SDL_APP_TEST_PASS)
    {
        mainMem    = SDL_ECC_MEMTYPE_MAX;
        subMemType = SDL_R5FSS0_CORE0_ECC_AGGR_CPU0_DDATA_RAM4_RAM_ID;
        intsrc     = SDL_INJECT_ECC_ERROR_FORCING_1BIT_ONCE;
        errSrc     = SDL_ECC_AGGR_INTR_SRC_SINGLE_BIT;

        SDL_ECC_injectError(mainMem, subMemType, intsrc,&injectErrorConfig);
        SDL_ecc_aggrSetEccRamIntrPending(pEccAggrRegs, subMemType, errSrc);
        if (SDL_ecc_aggrIsEccRamIntrPending(pEccAggrRegs, subMemType, errSrc, &isPend) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("sdlEccAggr_apiTest: failure on line no. %d \n", __LINE__);
        }
        if (SDL_ecc_aggrIsAnyIntrPending(pEccAggrRegs, subMemType, &isPend) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("sdlEccAggr_apiTest: failure on line no. %d \n", __LINE__);
        }
    }

    if (testStatus == SDL_APP_TEST_PASS)
    {
        mainMem    = SDL_ECC_MEMTYPE_MAX;
        subMemType = SDL_R5FSS0_CORE0_ECC_AGGR_CPU0_DDATA_RAM5_RAM_ID;
        intsrc     = SDL_INJECT_ECC_ERROR_FORCING_2BIT_REPEAT;
        errSrc     = SDL_ECC_AGGR_INTR_SRC_DOUBLE_BIT;
        SDL_ECC_injectError(mainMem, subMemType, intsrc,&injectErrorConfig);
        SDL_ecc_aggrSetEccRamNIntrPending(pEccAggrRegs, subMemType, errSrc, 3U);
        if (SDL_ecc_aggrIsEccRamIntrPending(pEccAggrRegs, subMemType, 0U, &isPend) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("sdlEccAggr_apiTest: failure on line no. %d \n", __LINE__);
        }
    }
    if (testStatus == SDL_APP_TEST_PASS)
    {
        mainMem    = SDL_ECC_MEMTYPE_MAX;
        subMemType = SDL_R5FSS0_CORE0_ECC_AGGR_CPU0_DDATA_RAM5_RAM_ID;
        intsrc     = SDL_INJECT_ECC_ERROR_FORCING_2BIT_REPEAT;
        errSrc     = SDL_ECC_AGGR_INTR_SRC_DOUBLE_BIT;
        SDL_ECC_injectError(mainMem, subMemType, intsrc,&injectErrorConfig);
        SDL_ecc_aggrSetEccRamIntrPending(pEccAggrRegs, subMemType, errSrc);
        if (SDL_ecc_aggrIsEccRamIntrPending(pEccAggrRegs, subMemType, errSrc, &isPend) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("sdlEccAggr_apiTest: failure on line no. %d \n", __LINE__);
        }
        if (SDL_ecc_aggrIsAnyIntrPending(pEccAggrRegs, subMemType, &isPend) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("sdlEccAggr_apiTest: failure on line no. %d \n", __LINE__);
        }
    }
    if (testStatus == SDL_APP_TEST_PASS)
    {
        mainMem    = SDL_ECC_MEMTYPE_MAX;
        subMemType = SDL_R5FSS0_CORE0_ECC_AGGR_CPU0_DDATA_RAM5_RAM_ID;
        intsrc     = SDL_INJECT_ECC_ERROR_FORCING_1BIT_REPEAT;
        errSrc     = SDL_ECC_AGGR_INTR_SRC_SINGLE_BIT;

        SDL_ECC_injectError(mainMem, subMemType, intsrc,&injectErrorConfig);
        SDL_ecc_aggrSetEccRamIntrPending(pEccAggrRegs, subMemType, errSrc);
        if (SDL_ecc_aggrIsEccRamIntrPending(pEccAggrRegs, subMemType, errSrc, &isPend) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("sdlEccAggr_apiTest: failure on line no. %d \n", __LINE__);
        }
        if (SDL_ecc_aggrIsAnyIntrPending(pEccAggrRegs, subMemType, &isPend) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("sdlEccAggr_apiTest: failure on line no. %d \n", __LINE__);
        }
    }
    if (testStatus == SDL_APP_TEST_PASS)
    {
        mainMem    = SDL_ECC_MEMTYPE_MAX;
        subMemType = SDL_R5FSS0_CORE0_ECC_AGGR_CPU0_DDATA_RAM5_RAM_ID;
        intsrc     = SDL_INJECT_ECC_ERROR_FORCING_1BIT_ONCE;
        errSrc     = SDL_ECC_AGGR_INTR_SRC_SINGLE_BIT;

        SDL_ECC_injectError(mainMem, subMemType, intsrc,&injectErrorConfig);
        SDL_ecc_aggrSetEccRamIntrPending(pEccAggrRegs, subMemType, errSrc);
        if (SDL_ecc_aggrIsEccRamIntrPending(pEccAggrRegs, subMemType, errSrc, &isPend) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("sdlEccAggr_apiTest: failure on line no. %d \n", __LINE__);
        }
        if (SDL_ecc_aggrIsAnyIntrPending(pEccAggrRegs, subMemType, &isPend) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("sdlEccAggr_apiTest: failure on line no. %d \n", __LINE__);
        }
    }
    if (testStatus == SDL_APP_TEST_PASS)
    {
        mainMem    = SDL_ECC_MEMTYPE_MAX;
        subMemType = SDL_R5FSS0_CORE0_ECC_AGGR_CPU0_DDATA_RAM6_RAM_ID;
        intsrc     = SDL_INJECT_ECC_ERROR_FORCING_2BIT_REPEAT;
        errSrc     = SDL_ECC_AGGR_INTR_SRC_DOUBLE_BIT;
        SDL_ECC_injectError(mainMem, subMemType, intsrc,&injectErrorConfig);
        SDL_ecc_aggrSetEccRamNIntrPending(pEccAggrRegs, subMemType, errSrc, 3U);
        if (SDL_ecc_aggrIsEccRamIntrPending(pEccAggrRegs, subMemType, 0U, &isPend) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("sdlEccAggr_apiTest: failure on line no. %d \n", __LINE__);
        }
    }
    if (testStatus == SDL_APP_TEST_PASS)
    {
        mainMem    = SDL_ECC_MEMTYPE_MAX;
        subMemType = SDL_R5FSS0_CORE0_ECC_AGGR_CPU0_DDATA_RAM6_RAM_ID;
        intsrc     = SDL_INJECT_ECC_ERROR_FORCING_2BIT_REPEAT;
        errSrc     = SDL_ECC_AGGR_INTR_SRC_DOUBLE_BIT;
        SDL_ECC_injectError(mainMem, subMemType, intsrc,&injectErrorConfig);
        SDL_ecc_aggrSetEccRamIntrPending(pEccAggrRegs, subMemType, errSrc);
        if (SDL_ecc_aggrIsEccRamIntrPending(pEccAggrRegs, subMemType, errSrc, &isPend) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("sdlEccAggr_apiTest: failure on line no. %d \n", __LINE__);
        }
        if (SDL_ecc_aggrIsAnyIntrPending(pEccAggrRegs, subMemType, &isPend) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("sdlEccAggr_apiTest: failure on line no. %d \n", __LINE__);
        }
    }
    if (testStatus == SDL_APP_TEST_PASS)
    {
        mainMem    = SDL_ECC_MEMTYPE_MAX;
        subMemType = SDL_R5FSS0_CORE0_ECC_AGGR_CPU0_DDATA_RAM6_RAM_ID;
        intsrc     = SDL_INJECT_ECC_ERROR_FORCING_1BIT_REPEAT;
        errSrc     = SDL_ECC_AGGR_INTR_SRC_SINGLE_BIT;

        SDL_ECC_injectError(mainMem, subMemType, intsrc,&injectErrorConfig);
        SDL_ecc_aggrSetEccRamIntrPending(pEccAggrRegs, subMemType, errSrc);
        if (SDL_ecc_aggrIsEccRamIntrPending(pEccAggrRegs, subMemType, errSrc, &isPend) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("sdlEccAggr_apiTest: failure on line no. %d \n", __LINE__);
        }
        if (SDL_ecc_aggrIsAnyIntrPending(pEccAggrRegs, subMemType, &isPend) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("sdlEccAggr_apiTest: failure on line no. %d \n", __LINE__);
        }
    }
    if (testStatus == SDL_APP_TEST_PASS)
    {
        mainMem    = SDL_ECC_MEMTYPE_MAX;
        subMemType = SDL_R5FSS0_CORE0_ECC_AGGR_CPU0_DDATA_RAM6_RAM_ID;
        intsrc     = SDL_INJECT_ECC_ERROR_FORCING_1BIT_ONCE;
        errSrc     = SDL_ECC_AGGR_INTR_SRC_SINGLE_BIT;

        SDL_ECC_injectError(mainMem, subMemType, intsrc,&injectErrorConfig);
        SDL_ecc_aggrSetEccRamIntrPending(pEccAggrRegs, subMemType, errSrc);
        if (SDL_ecc_aggrIsEccRamIntrPending(pEccAggrRegs, subMemType, errSrc, &isPend) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("sdlEccAggr_apiTest: failure on line no. %d \n", __LINE__);
        }
        if (SDL_ecc_aggrIsAnyIntrPending(pEccAggrRegs, subMemType, &isPend) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("sdlEccAggr_apiTest: failure on line no. %d \n", __LINE__);
        }
    }

    if (testStatus == SDL_APP_TEST_PASS)
    {
        mainMem    = SDL_ECC_MEMTYPE_MAX;
        subMemType = SDL_R5FSS0_CORE0_ECC_AGGR_CPU0_DDATA_RAM7_RAM_ID;
        intsrc     = SDL_INJECT_ECC_ERROR_FORCING_2BIT_REPEAT;
        errSrc     = SDL_ECC_AGGR_INTR_SRC_DOUBLE_BIT;
        SDL_ECC_injectError(mainMem, subMemType, intsrc,&injectErrorConfig);
        SDL_ecc_aggrSetEccRamNIntrPending(pEccAggrRegs, subMemType, errSrc, 3U);
        if (SDL_ecc_aggrIsEccRamIntrPending(pEccAggrRegs, subMemType, 0U, &isPend) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("sdlEccAggr_apiTest: failure on line no. %d \n", __LINE__);
        }
    }
    if (testStatus == SDL_APP_TEST_PASS)
    {
        mainMem    = SDL_ECC_MEMTYPE_MAX;
        subMemType = SDL_R5FSS0_CORE0_ECC_AGGR_CPU0_DDATA_RAM7_RAM_ID;
        intsrc     = SDL_INJECT_ECC_ERROR_FORCING_2BIT_REPEAT;
        errSrc     = SDL_ECC_AGGR_INTR_SRC_DOUBLE_BIT;
        SDL_ECC_injectError(mainMem, subMemType, intsrc,&injectErrorConfig);
        SDL_ecc_aggrSetEccRamIntrPending(pEccAggrRegs, subMemType, errSrc);
        if (SDL_ecc_aggrIsEccRamIntrPending(pEccAggrRegs, subMemType, errSrc, &isPend) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("sdlEccAggr_apiTest: failure on line no. %d \n", __LINE__);
        }
        if (SDL_ecc_aggrIsAnyIntrPending(pEccAggrRegs, subMemType, &isPend) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("sdlEccAggr_apiTest: failure on line no. %d \n", __LINE__);
        }
    }
    if (testStatus == SDL_APP_TEST_PASS)
    {
        mainMem    = SDL_ECC_MEMTYPE_MAX;
        subMemType = SDL_R5FSS0_CORE0_ECC_AGGR_CPU0_DDATA_RAM7_RAM_ID;
        intsrc     = SDL_INJECT_ECC_ERROR_FORCING_1BIT_REPEAT;
        errSrc     = SDL_ECC_AGGR_INTR_SRC_SINGLE_BIT;

        SDL_ECC_injectError(mainMem, subMemType, intsrc,&injectErrorConfig);
        SDL_ecc_aggrSetEccRamIntrPending(pEccAggrRegs, subMemType, errSrc);
        if (SDL_ecc_aggrIsEccRamIntrPending(pEccAggrRegs, subMemType, errSrc, &isPend) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("sdlEccAggr_apiTest: failure on line no. %d \n", __LINE__);
        }
        if (SDL_ecc_aggrIsAnyIntrPending(pEccAggrRegs, subMemType, &isPend) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("sdlEccAggr_apiTest: failure on line no. %d \n", __LINE__);
        }
    }
    if (testStatus == SDL_APP_TEST_PASS)
    {
        mainMem    = SDL_ECC_MEMTYPE_MAX;
        subMemType = SDL_R5FSS0_CORE0_ECC_AGGR_CPU0_DDATA_RAM7_RAM_ID;
        intsrc     = SDL_INJECT_ECC_ERROR_FORCING_1BIT_ONCE;
        errSrc     = SDL_ECC_AGGR_INTR_SRC_SINGLE_BIT;

        SDL_ECC_injectError(mainMem, subMemType, intsrc,&injectErrorConfig);
        SDL_ecc_aggrSetEccRamIntrPending(pEccAggrRegs, subMemType, errSrc);
        if (SDL_ecc_aggrIsEccRamIntrPending(pEccAggrRegs, subMemType, errSrc, &isPend) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("sdlEccAggr_apiTest: failure on line no. %d \n", __LINE__);
        }
        if (SDL_ecc_aggrIsAnyIntrPending(pEccAggrRegs, subMemType, &isPend) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("sdlEccAggr_apiTest: failure on line no. %d \n", __LINE__);
        }
    }

    DebugP_log("ALL sdlEccAggr_apiTest are passed\n");
    return (testStatus);
}

/* ECC Functional test */
int32_t ECC_ip_funcTest(void)
{
    int32_t testResult = 0;

    testResult = ECC_funcAPITest();

    return (testResult);
}
/* Nothing past this point */
