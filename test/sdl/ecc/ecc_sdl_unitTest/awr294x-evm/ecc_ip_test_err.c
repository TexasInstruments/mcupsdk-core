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
 *  \file     ecc_test_err.c
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
#include <sdl/include/awr294x/sdlr_soc_baseaddress.h>
#include "ecc_test_main.h"
#include <sdl/include/awr294x/sdlr_mss_param_regs.h>
#include <sdl/include/awr294x/sdlr_dss_param_regs.h>
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
    SDL_ecc_aggrRegs *pEccAggrRegs = ((SDL_ecc_aggrRegs *)((uintptr_t)SDL_MSS_ECC_AGG_R5A_U_BASE));// R5 core
    SDL_Ecc_AggrEccRamErrorStatusInfo eccRamErrorStatus;
    SDL_Ecc_AggrErrorInfo eccErrorInfo;
    SDL_ECC_staticRegs eccStaticRegs;
    int32_t testStatus = SDL_APP_TEST_PASS;
    uint32_t val;
    bool isPend;

    /* SDL_ecc_aggrGetRevision negative test */
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ecc_aggrGetRevision(NULL, &val) != SDL_EBADARGS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ecc_aggrGetRevision(pEccAggrRegs, NULL) != SDL_EBADARGS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("sdlEccAggr_negTest: failure on line no. %d \n", __LINE__);
    }

    /* SDL_ecc_aggrGetNumRams negative test */
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ecc_aggrGetNumRams(NULL, &val) != SDL_EBADARGS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ecc_aggrGetNumRams(pEccAggrRegs, NULL) != SDL_EBADARGS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("sdlEccAggr_negTest: failure on line no. %d \n", __LINE__);
    }

    /* SDL_ecc_aggrReadEccRamReg negative test */
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ecc_aggrReadEccRamReg(NULL, 0U, SDL_ECC_RAM_WRAP_REV, &val) != SDL_EBADARGS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ecc_aggrReadEccRamReg(pEccAggrRegs, 0xFFFFU, SDL_ECC_RAM_WRAP_REV, &val) != SDL_EBADARGS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ecc_aggrReadEccRamReg(pEccAggrRegs, 0U, 0U, &val) != SDL_EBADARGS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ecc_aggrReadEccRamReg(pEccAggrRegs, 0U, SDL_ECC_RAM_WRAP_REV, NULL) != SDL_EBADARGS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ecc_aggrReadEccRamReg(pEccAggrRegs, 0U, 0xFF, &val) != SDL_EBADARGS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("sdlEccAggr_negTest: failure on line no. %d \n", __LINE__);
    }
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
        DebugP_log("sdlEccAggr_negTest: failure on line no. %d \n", __LINE__);
    }

    /* SDL_ecc_aggrReadEccRamCtrlReg negative test */
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ecc_aggrReadEccRamCtrlReg(NULL, 0U, &val) != SDL_EBADARGS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ecc_aggrReadEccRamCtrlReg(pEccAggrRegs, 0xFFFFU, &val) != SDL_EBADARGS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ecc_aggrReadEccRamCtrlReg(pEccAggrRegs, 0U, NULL) != SDL_EBADARGS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("sdlEccAggr_negTest: failure on line no. %d \n", __LINE__);
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
        DebugP_log("sdlEccAggr_negTest: failure on line no. %d \n", __LINE__);
    }

    /* SDL_ecc_aggrReadEccRamErrStatReg negative test */
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ecc_aggrReadEccRamErrStatReg(NULL, 0U, 0U, &val) != SDL_EBADARGS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ecc_aggrReadEccRamErrStatReg(pEccAggrRegs, 0xFFFFU, 0U, &val) != SDL_EBADARGS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ecc_aggrReadEccRamErrStatReg(pEccAggrRegs, 0U, 0xFFFFU, &val) != SDL_EBADARGS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ecc_aggrReadEccRamErrStatReg(pEccAggrRegs, 0U, 0U, NULL) != SDL_EBADARGS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("sdlEccAggr_negTest: failure on line no. %d \n", __LINE__);
    }

    /* SDL_ecc_aggrWriteEccRamReg negative test */
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ecc_aggrWriteEccRamReg(NULL, 0U, SDL_ECC_RAM_WRAP_REV, 0U) != SDL_EBADARGS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ecc_aggrWriteEccRamReg(pEccAggrRegs, 0xFFFFU, SDL_ECC_RAM_WRAP_REV, 0U) != SDL_EBADARGS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ecc_aggrWriteEccRamReg(pEccAggrRegs, 0U, 0U, 0U) != SDL_EBADARGS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("sdlEccAggr_negTest: failure on line no. %d \n", __LINE__);
    }

    /* SDL_ecc_aggrWriteEccRamCtrlReg negative test */
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ecc_aggrWriteEccRamCtrlReg(NULL, 0U, 0U) != SDL_EBADARGS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ecc_aggrWriteEccRamCtrlReg(pEccAggrRegs, 0xFFFFU, 0U) != SDL_EBADARGS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("sdlEccAggr_negTest: failure on line no. %d \n", __LINE__);
    }

    /* SDL_ecc_aggrWriteEccRamErrCtrlReg negative test */
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ecc_aggrWriteEccRamErrCtrlReg(NULL, 0U, 0U, 0U) != SDL_EBADARGS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ecc_aggrWriteEccRamErrCtrlReg(pEccAggrRegs, 0xFFFFU, 0U, 0U) != SDL_EBADARGS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ecc_aggrWriteEccRamErrCtrlReg(pEccAggrRegs, 0U, 0xFFFFU, 0U) != SDL_EBADARGS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("sdlEccAggr_negTest: failure on line no. %d \n", __LINE__);
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
        DebugP_log("sdlEccAggr_negTest: failure on line no. %d \n", __LINE__);
    }

    /* SDL_ecc_aggrConfigEccRam negative test */
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ecc_aggrConfigEccRam(NULL, 0U,  0U, 0U, 0U) != SDL_EBADARGS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ecc_aggrConfigEccRam(pEccAggrRegs, 0xFFFFU, 0U, 0U, 0U) != SDL_EBADARGS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("sdlEccAggr_negTest: failure on line no. %d \n", __LINE__);
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
        DebugP_log("sdlEccAggr_negTest: failure on line no. %d \n", __LINE__);
    }

    /* SDL_ecc_aggrGetEccRamErrorStatus negative test */
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ecc_aggrGetEccRamErrorStatus(NULL, 0U, &eccRamErrorStatus) != SDL_EBADARGS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ecc_aggrGetEccRamErrorStatus(pEccAggrRegs, 0xFFFFU, &eccRamErrorStatus) != SDL_EBADARGS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ecc_aggrGetEccRamErrorStatus(pEccAggrRegs, 0U, NULL) != SDL_EBADARGS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("sdlEccAggr_negTest: failure on line no. %d \n", __LINE__);
    }

    /* SDL_ecc_aggrGetEccRamErrorStatus negative test */
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ecc_aggrGetEccRamErrorStatus(NULL, 0U, &eccRamErrorStatus) != SDL_EBADARGS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ecc_aggrGetEccRamErrorStatus(pEccAggrRegs, 0xFFFFU, &eccRamErrorStatus) != SDL_EBADARGS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ecc_aggrGetEccRamErrorStatus(pEccAggrRegs, 0U, NULL) != SDL_EBADARGS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
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
    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("sdlEccAggr_negTest: failure on line no. %d \n", __LINE__);
    }
    /* SDL_ecc_aggrAckIntr negative test */
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ecc_aggrAckIntr(NULL, SDL_ECC_AGGR_INTR_SRC_SINGLE_BIT) != SDL_EBADARGS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("sdlEccAggr_negTest: failure on line no. %d \n", __LINE__);
    }
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ecc_aggrAckIntr(pEccAggrRegs, SDL_ECC_AGGR_INTR_SRC_INVALID) != SDL_EBADARGS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("sdlEccAggr_negTest: failure on line no. %d \n", __LINE__);
    }

    /* SDL_ecc_aggrIsEccRamIntrPending negative test */
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ecc_aggrIsEccRamIntrPending(NULL, 0U, SDL_ECC_AGGR_INTR_SRC_SINGLE_BIT, &isPend) != SDL_EBADARGS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("sdlEccAggr_negTest: failure on line no. %d \n", __LINE__);
    }
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ecc_aggrIsEccRamIntrPending(pEccAggrRegs, 0xFFFFU, SDL_ECC_AGGR_INTR_SRC_SINGLE_BIT, &isPend) != SDL_EBADARGS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("sdlEccAggr_negTest: failure on line no. %d \n", __LINE__);
    }
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ecc_aggrIsEccRamIntrPending(pEccAggrRegs, 0U, SDL_ECC_AGGR_INTR_SRC_SINGLE_BIT, NULL) != SDL_EBADARGS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("sdlEccAggr_negTest: failure on line no. %d \n", __LINE__);
    }

    /* SDL_ecc_aggrSetEccRamIntrPending negative test */
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ecc_aggrSetEccRamIntrPending(NULL, 0U, SDL_ECC_AGGR_INTR_SRC_SINGLE_BIT) != SDL_EBADARGS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("sdlEccAggr_negTest: failure on line no. %d \n", __LINE__);
    }
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ecc_aggrSetEccRamIntrPending(pEccAggrRegs, 0xFFFFU, SDL_ECC_AGGR_INTR_SRC_SINGLE_BIT) != SDL_EBADARGS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("sdlEccAggr_negTest: failure on line no. %d \n", __LINE__);
    }
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ecc_aggrSetEccRamIntrPending(pEccAggrRegs, 0U, SDL_ECC_AGGR_INTR_SRC_INVALID) != SDL_EBADARGS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("sdlEccAggr_negTest: failure on line no. %d \n", __LINE__);
    }
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ecc_aggrSetEccRamIntrPending(pEccAggrRegs, 0U, SDL_ECC_AGGR_INTR_SRC_NONE) != SDL_EBADARGS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("sdlEccAggr_negTest: failure on line no. %d \n", __LINE__);
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
        DebugP_log("sdlEccAggr_negTest: failure on line no. %d \n", __LINE__);
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
        DebugP_log("sdlEccAggr_negTest: failure on line no. %d \n", __LINE__);
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
        DebugP_log("sdlEccAggr_negTest: failure on line no. %d \n", __LINE__);
    }

    /* SDL_ecc_aggrSetEccRamNIntrPending negative test */
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ecc_aggrSetEccRamNIntrPending(NULL, 0U, SDL_ECC_AGGR_INTR_SRC_SINGLE_BIT, 1U) != SDL_EBADARGS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("sdlEccAggr_negTest: failure on line no. %d \n", __LINE__);
    }
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ecc_aggrSetEccRamNIntrPending(pEccAggrRegs, 0xFFFFU, SDL_ECC_AGGR_INTR_SRC_SINGLE_BIT, 1U) != SDL_EBADARGS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("sdlEccAggr_negTest: failure on line no. %d \n", __LINE__);
    }
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ecc_aggrSetEccRamNIntrPending(pEccAggrRegs, 0U, SDL_ECC_AGGR_INTR_SRC_INVALID, 1U) != SDL_EBADARGS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("sdlEccAggr_negTest: failure on line no. %d \n", __LINE__);
    }
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ecc_aggrSetEccRamNIntrPending(pEccAggrRegs, 0U, SDL_ECC_AGGR_INTR_SRC_SINGLE_BIT, 0U) != SDL_EBADARGS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("sdlEccAggr_negTest: failure on line no. %d \n", __LINE__);
    }
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ecc_aggrSetEccRamNIntrPending(pEccAggrRegs, 0U, SDL_ECC_ADDR_ERROR_TYPE_SUCCESSIVE_SINGLE_BITS, 0U) != SDL_EBADARGS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("sdlEccAggr_negTest: failure on line no. %d \n", __LINE__);
    }
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ecc_aggrSetEccRamNIntrPending(pEccAggrRegs, 0U, SDL_ECC_AGGR_INTR_SRC_SINGLE_BIT, 4U) != SDL_EBADARGS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("sdlEccAggr_negTest: failure on line no. %d \n", __LINE__);
    }
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ecc_aggrSetEccRamNIntrPending(pEccAggrRegs, 0U, SDL_ECC_ADDR_ERROR_TYPE_SUCCESSIVE_SINGLE_BITS, 4U) != SDL_EBADARGS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("sdlEccAggr_negTest: failure on line no. %d \n", __LINE__);
    }

    /* SDL_ecc_aggrClrEccRamNIntrPending negative test */
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ecc_aggrClrEccRamNIntrPending(NULL, 0U, SDL_ECC_AGGR_INTR_SRC_SINGLE_BIT, 1U) != SDL_EBADARGS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("sdlEccAggr_negTest: failure on line no. %d \n", __LINE__);
    }
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ecc_aggrClrEccRamNIntrPending(pEccAggrRegs, 0xFFFFU, SDL_ECC_AGGR_INTR_SRC_SINGLE_BIT, 1U) != SDL_EBADARGS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("sdlEccAggr_negTest: failure on line no. %d \n", __LINE__);
    }
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ecc_aggrClrEccRamNIntrPending(pEccAggrRegs, 0U, SDL_ECC_AGGR_INTR_SRC_INVALID, 1U) != SDL_EBADARGS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("sdlEccAggr_negTest: failure on line no. %d \n", __LINE__);
    }
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ecc_aggrClrEccRamNIntrPending(pEccAggrRegs, 0U, SDL_ECC_AGGR_INTR_SRC_SINGLE_BIT, 0U) != SDL_EBADARGS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("sdlEccAggr_negTest: failure on line no. %d \n", __LINE__);
    }
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ecc_aggrClrEccRamNIntrPending(pEccAggrRegs, 0U, SDL_ECC_ADDR_ERROR_TYPE_SUCCESSIVE_SINGLE_BITS, 0U) != SDL_EBADARGS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("sdlEccAggr_negTest: failure on line no. %d \n", __LINE__);
    }
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ecc_aggrClrEccRamNIntrPending(pEccAggrRegs, 0U, SDL_ECC_AGGR_INTR_SRC_SINGLE_BIT, 4U) != SDL_EBADARGS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("sdlEccAggr_negTest: failure on line no. %d \n", __LINE__);
    }
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ecc_aggrClrEccRamNIntrPending(pEccAggrRegs, 0U, SDL_ECC_ADDR_ERROR_TYPE_SUCCESSIVE_SINGLE_BITS, 4U) != SDL_EBADARGS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("sdlEccAggr_negTest: failure on line no. %d \n", __LINE__);
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
        DebugP_log("sdlEccAggr_negTest: failure on line no. %d \n", __LINE__);
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
        DebugP_log("sdlEccAggr_negTest: failure on line no. %d \n", __LINE__);
    }

    /* SDL_ecc_aggrEnableIntr negative test */
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ecc_aggrEnableIntr(NULL, 0U, SDL_ECC_AGGR_INTR_SRC_SINGLE_BIT) != SDL_EBADARGS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ecc_aggrEnableIntr(pEccAggrRegs, 0xFFFFU, SDL_ECC_AGGR_INTR_SRC_SINGLE_BIT) != SDL_EBADARGS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ecc_aggrEnableIntr(pEccAggrRegs, 0U, SDL_ECC_AGGR_INTR_SRC_INVALID) != SDL_EBADARGS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("sdlEccAggr_negTest: failure on line no. %d \n", __LINE__);
    }

    /* SDL_ecc_aggrDisableIntr negative test */
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ecc_aggrDisableIntr(NULL, 0U, SDL_ECC_AGGR_INTR_SRC_SINGLE_BIT) != SDL_EBADARGS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ecc_aggrDisableIntr(pEccAggrRegs, 0xFFFFU, SDL_ECC_AGGR_INTR_SRC_SINGLE_BIT) != SDL_EBADARGS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ecc_aggrDisableIntr(pEccAggrRegs, 0U, SDL_ECC_AGGR_INTR_SRC_INVALID) != SDL_EBADARGS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("sdlEccAggr_negTest: failure on line no. %d \n", __LINE__);
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
        DebugP_log("sdlEccAggr_negTest: failure on line no. %d \n", __LINE__);
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
        DebugP_log("sdlEccAggr_negTest: failure on line no. %d \n", __LINE__);
    }

    /* SDL_ecc_aggrEnableIntrs negative test */
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ecc_aggrEnableIntrs(NULL, SDL_ECC_AGGR_INTR_SRC_SINGLE_BIT) != SDL_EBADARGS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ecc_aggrEnableIntrs(pEccAggrRegs, SDL_ECC_AGGR_INTR_SRC_INVALID) != SDL_EBADARGS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("sdlEccAggr_negTest: failure on line no. %d \n", __LINE__);
    }

    /* SDL_ecc_aggrDisableIntrs negative test */
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ecc_aggrDisableIntrs(NULL, SDL_ECC_AGGR_INTR_SRC_SINGLE_BIT) != SDL_EBADARGS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ecc_aggrDisableIntrs(pEccAggrRegs, SDL_ECC_AGGR_INTR_SRC_INVALID) != SDL_EBADARGS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("sdlEccAggr_negTest: failure on line no. %d \n", __LINE__);
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
        DebugP_log("sdlEccAggr_negTest: failure on line no. %d \n", __LINE__);
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
        DebugP_log("sdlEccAggr_negTest: failure on line no. %d \n", __LINE__);
    }

    /* SDL_ecc_aggrReadStaticRegs negative test */
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ecc_aggrReadStaticRegs(NULL, &eccStaticRegs) != SDL_EBADARGS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ecc_aggrReadStaticRegs(pEccAggrRegs, NULL) != SDL_EBADARGS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }

    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("sdlEccAggr_negTest: failure on line no. %d \n", __LINE__);
    }

    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ecc_aggrIntrEnableCtrl(pEccAggrRegs, NULL) != SDL_EBADARGS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus == SDL_APP_TEST_PASS)
    {
        SDL_ecc_aggrEnableCtrl enableCtrl;
        if (SDL_ecc_aggrIntrEnableCtrl(NULL, &enableCtrl) != SDL_EBADARGS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("sdlEccAggr_negTest: failure on line no. %d \n", __LINE__);
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
        DebugP_log("sdlEccAggr_negTest: failure on line no. %d \n", __LINE__);
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
        DebugP_log("sdlEccAggr_negTest: failure on line no. %d \n", __LINE__);
    }
	if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ECC_tcmParity(SDL_TCM_PARITY_ATCM0, 0x7) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("sdlEccAggr_negTest: failure on line no. %d \n", __LINE__);
    }
	if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ECC_tcmParity(SDL_TCM_PARITY_ATCM0, 0x7) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("sdlEccAggr_negTest: failure on line no. %d \n", __LINE__);
    }
	if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ECC_tcmParity(SDL_TCM_PARITY_B0TCM0, 0x700U) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("sdlEccAggr_negTest: failure on line no. %d \n", __LINE__);
    }
	if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ECC_tcmParity(7U, 0x700U) != SDL_EFAIL)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("sdlEccAggr_negTest: failure on line no. %d \n", __LINE__);
    }
	if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ECC_tpccParity(SDL_DSS_TPCCA, 0x3u, (SDL_DSS_PARAM_REG_A_SET0 + 0x20U), 0x7u) == SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("sdlEccAggr_negTest: failure on line no. %d \n", __LINE__);
    }
	if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ECC_tpccParity(SDL_DSS_TPCCB, 0x3u, (SDL_DSS_PARAM_REG_B_SET0 + 0x20U), 0x7u) == SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("sdlEccAggr_negTest: failure on line no. %d \n", __LINE__);
    }
	if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ECC_tpccParity(SDL_TPCC0B, 0x1100u, (SDL_MSS_PARAM_REG_B_SET0 + 0x20U), 0x7u) == SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("sdlEccAggr_negTest: failure on line no. %d \n", __LINE__);
    }
	if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ECC_tpccParity(SDL_TPCC0A, 0x11u, (SDL_MSS_PARAM_REG_A_SET0 + 0x20U), 0x7u) == SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("sdlEccAggr_negTest: failure on line no. %d \n", __LINE__);
    }
	if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ECC_tpccParity(SDL_DSS_TPCCC, 0x3u, (SDL_DSS_PARAM_REG_C_SET0 + 0x20U), 0x7u) == SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("sdlEccAggr_negTest: failure on line no. %d \n", __LINE__);
    }
	if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ECC_tpccParity(7u, 0x11u, (SDL_MSS_PARAM_REG_A_SET0 + 0x20U), 0x7u) != SDL_EFAIL)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("sdlEccAggr_negTest: failure on line no. %d \n", __LINE__);
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
