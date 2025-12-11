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
 *  \file     ecc_ip_test_func.c
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
#include <sdl/ecc/V0/sdlr_ecc_ram.h>
#include <sdl/ecc/V0/sdlr_edc_ctl.h>
#include "ecc_test_main.h"
#if defined(SOC_AM64X) || defined(SOC_AM243X)
#include <sdl/include/am64x_am243x/sdlr_soc_baseaddress.h>
#include <sdl/ecc/soc/am64x_am243x/sdl_ecc_soc.h>
#endif

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
    SDL_ECC_InjectErrorConfig_t injectErrorConfig;
    memset(&injectErrorConfig, 0, sizeof(injectErrorConfig));
    injectErrorConfig.pErrMem = (uint32_t *)(0u);
    injectErrorConfig.flipBitMask = 0x3;

    #if defined (SOC_AM64X) || defined (SOC_AM243X)
    SDL_ECC_InjectErrorType intsrc;
    uint32_t mainMem, subMemType, errSrc;
    #endif

    /* SDL_ecc_aggrReadEccRamWrapRevReg API test */
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ecc_aggrReadEccRamWrapRevReg(pEccAggrRegs, 0U, &val) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("sdlEccAggr_apiTest: failure on line no. %d \r\n", __LINE__);
        }
    }

    /* SDL_ecc_aggrReadEccRamErrCtrlReg API test */
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ecc_aggrReadEccRamErrCtrlReg(pEccAggrRegs, 0U, 0U, &val) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("sdlEccAggr_apiTest: failure in SDL_ecc_aggrReadEccRamErrCtrlReg on line no. %d \r\n", __LINE__);
        }
    }

    /* SDL_ecc_aggrWriteEccRamErrStatReg API test */
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ecc_aggrWriteEccRamErrStatReg(pEccAggrRegs, 0U, 0U, 0U) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("sdlEccAggr_apiTest: failure on line no. %d \r\n", __LINE__);
        }
    }

    /* SDL_ecc_aggrConfigEccRam API test */
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ecc_aggrConfigEccRam(pEccAggrRegs, 0U, 0U, 0U, 0U) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("sdlEccAggr_apiTest: failure on line no. %d \r\n", __LINE__);
        }
    }

    /* SDL_ecc_aggrVerifyConfigEccRam API test */
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ecc_aggrVerifyConfigEccRam(pEccAggrRegs, 0U, 0U, 0U, 0U) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("ECC_ip_funcTest: failure in SDL_ecc_aggrVerifyConfigEccRam on line no. %d \r\n", __LINE__);
        }
    }

    /* SDL_ecc_aggrForceEccRamError API test */
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ecc_aggrForceEccRamError(pEccAggrRegs, 0U, &eccErrorInfo) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("sdlEccAggr_apiTest: failure on line no. %d \r\n", __LINE__);
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
            DebugP_log("sdlEccAggr_apiTest: failure on line no. %d \r\n", __LINE__);
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
            DebugP_log("sdlEccAggr_apiTest: failure on line no. %d \r\n", __LINE__);
        }
    }

    /* SDL_ecc_aggrClrEccRamIntrPending API test */
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ecc_aggrClrEccRamIntrPending(pEccAggrRegs, 5U, SDL_ECC_AGGR_INTR_SRC_SINGLE_BIT) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("sdlEccAggr_apiTest: failure on line no. %d \r\n", __LINE__);
        }
    }
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ecc_aggrClrEccRamIntrPending(pEccAggrRegs, 5U, SDL_ECC_AGGR_INTR_SRC_DOUBLE_BIT) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("sdlEccAggr_apiTest: failure on line no. %d \r\n", __LINE__);
        }
    }
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ecc_aggrClrEccRamIntrPending(pEccAggrRegs, 5U, SDL_ECC_ADDR_ERROR_TYPE_SUCCESSIVE_SINGLE_BITS) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("sdlEccAggr_apiTest: failure on line no. %d \r\n", __LINE__);
        }
    }

    /* SDL_ecc_aggrEnableAllIntr API test */
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ecc_aggrEnableAllIntr(pEccAggrRegs, 0U) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("sdlEccAggr_apiTest: failure on line no. %d \r\n", __LINE__);
        }
    }

    /* SDL_ecc_aggrDisableAllIntr API test */
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ecc_aggrDisableAllIntr(pEccAggrRegs, 0U) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("sdlEccAggr_apiTest: failure on line no. %d \r\n", __LINE__);
        }
    }

    /* SDL_ecc_aggrEnableAllIntrs API test */
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ecc_aggrEnableAllIntrs(pEccAggrRegs) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("sdlEccAggr_apiTest: failure on line no. %d \r\n", __LINE__);
        }
    }

    /* SDL_ecc_aggrDisableAllIntrs API test */
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ecc_aggrDisableAllIntrs(pEccAggrRegs) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("sdlEccAggr_apiTest: failure on line no. %d \r\n", __LINE__);
        }
    }

    #if defined(R5F_CORE)
    /* SDL_ecc_aggrReadEDCInterconnectReg API test */
    if (testStatus == SDL_APP_TEST_PASS)
    {
        SDL_ecc_aggrRegs *pEccEdcAggrRegs = ((SDL_ecc_aggrRegs *)((uintptr_t)SDL_VTM0_ECCAGGR_CFG_BASE));
        uint32_t ramId = SDL_VTM0_K3VTM_N16FFC_ECCAGGR_K3VTM_N16FFC_CFG_CBASS_CFG_SCR_SCR_EDC_CTRL_0_RAM_ID;
        uint32_t regOffset = SDL_EDC_CTL_CONTROL;
        uint32_t pRegVal;

        if (SDL_ecc_aggrReadEDCInterconnectReg(pEccEdcAggrRegs, ramId, regOffset, &pRegVal) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("sdlEccAggr_apiTest: failure on line no. %d \r\n", __LINE__);
        }
    }

    /* SDL_ecc_aggrWriteEDCInterconnectReg API test */
    if (testStatus == SDL_APP_TEST_PASS)
    {
        SDL_ecc_aggrRegs *pEccEdcAggrRegs = ((SDL_ecc_aggrRegs *)((uintptr_t)SDL_VTM0_ECCAGGR_CFG_BASE));
        uint32_t ramId = SDL_VTM0_K3VTM_N16FFC_ECCAGGR_K3VTM_N16FFC_CFG_CBASS_CFG_SCR_SCR_EDC_CTRL_0_RAM_ID;
        uint32_t regOffset = SDL_EDC_CTL_CONTROL;
        uint32_t pRegVal = SDL_EDC_CTL_CONTROL_ECC_PATTERN_VAL_A;

        if (SDL_ecc_aggrWriteEDCInterconnectReg(pEccEdcAggrRegs, ramId, regOffset, pRegVal) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("sdlEccAggr_apiTest: failure on line no. %d \r\n", __LINE__);
        }
    }

    /* SDL_ecc_aggrVerifyConfigEDCInterconnect API test */
    if (testStatus == SDL_APP_TEST_PASS)
    {
        SDL_ecc_aggrRegs *pEccEdcAggrRegs = ((SDL_ecc_aggrRegs *)((uintptr_t)SDL_VTM0_ECCAGGR_CFG_BASE));
        uint32_t ramId = SDL_VTM0_K3VTM_N16FFC_ECCAGGR_K3VTM_N16FFC_CFG_CBASS_CFG_SCR_SCR_EDC_CTRL_0_RAM_ID;

        /* Set expected configuration fields by passing true*/
        bool bEccCheck = true;

        if (SDL_ecc_aggrVerifyConfigEDCInterconnect(pEccEdcAggrRegs, ramId, bEccCheck) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("sdlEccAggr_apiTest: failure on line no. %d \r\n", __LINE__);
        }
    }

    /* SDL_ecc_aggrSetEDCInterconnectNIntrPending API test */
    if (testStatus == SDL_APP_TEST_PASS)
    {
        SDL_ecc_aggrRegs *pEccEdcAggrRegs = ((SDL_ecc_aggrRegs *)((uintptr_t)SDL_VTM0_ECCAGGR_CFG_BASE));
        uint32_t ramId = SDL_VTM0_K3VTM_N16FFC_ECCAGGR_K3VTM_N16FFC_CFG_CBASS_CFG_SCR_SCR_EDC_CTRL_0_RAM_ID;
        SDL_Ecc_AggrIntrSrc intrSrc = SDL_ECC_AGGR_INTR_SRC_SINGLE_BIT;
        SDL_Ecc_AggrEDCErrorSubType subType = SDL_ECC_AGGR_ERROR_SUBTYPE_INJECT;
        uint32_t numEvents = 3U;

        if (SDL_ecc_aggrSetEDCInterconnectNIntrPending(pEccEdcAggrRegs, ramId, intrSrc, subType, numEvents) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("sdlEccAggr_apiTest: failure on line no. %d \r\n", __LINE__);
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
            DebugP_log("sdlEccAggr_apiTest: failure on line no. %d \r\n", __LINE__);
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
            DebugP_log("sdlEccAggr_apiTest: failure on line no. %d \r\n", __LINE__);
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
            DebugP_log("sdlEccAggr_apiTest: failure on line no. %d \r\n", __LINE__);
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
            DebugP_log("sdlEccAggr_apiTest: failure on line no. %d \r\n", __LINE__);
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
            DebugP_log("sdlEccAggr_apiTest: failure on line no. %d \r\n", __LINE__);
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
            DebugP_log("sdlEccAggr_apiTest: failure on line no. %d \r\n", __LINE__);
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
            DebugP_log("sdlEccAggr_apiTest: failure on line no. %d \r\n", __LINE__);
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
            DebugP_log("sdlEccAggr_apiTest: failure on line no. %d \r\n", __LINE__);
        }
    }
    #endif

    /* SDL_ecc_aggrIsIntrPending API test */
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ecc_aggrIsIntrPending(pEccAggrRegs, 0U, SDL_ECC_AGGR_INTR_SRC_SINGLE_BIT, &isPend) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("sdlEccAggr_apiTest: failure on line no. %d \r\n", __LINE__);
        }
    }
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ecc_aggrIsIntrPending(pEccAggrRegs, 0U, SDL_ECC_AGGR_INTR_SRC_DOUBLE_BIT, &isPend) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("sdlEccAggr_apiTest: failure on line no. %d \r\n", __LINE__);
        }
    }

    /* SDL_ecc_aggrIsAnyIntrPending API test */
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ecc_aggrIsAnyIntrPending(pEccAggrRegs, 0U, &isPend) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("sdlEccAggr_apiTest: failure on line no. %d \r\n", __LINE__);
        }
    }

    if (testStatus == SDL_APP_TEST_PASS)
    {
        mainMem    = SDL_ECC_MEMTYPE_MAX;
        subMemType = SDL_PSRAMECC0_PSRAM256X32E_ECC_AGGR_PSRAM256X32E_PSRAM0_ECC_RAM_ID;
        intsrc     = SDL_INJECT_ECC_ERROR_FORCING_2BIT_REPEAT;
        errSrc     = SDL_ECC_AGGR_INTR_SRC_DOUBLE_BIT;
        SDL_ECC_injectError(mainMem, subMemType, intsrc,&injectErrorConfig);
        SDL_ecc_aggrSetEccRamNIntrPending(pEccAggrRegs, subMemType, errSrc, 3U);
        if (SDL_ecc_aggrIsEccRamIntrPending(pEccAggrRegs, subMemType, 0U, &isPend) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("sdlEccAggr_apiTest: failure on line no. %d \r\n", __LINE__);
        }
    }
    if (testStatus == SDL_APP_TEST_PASS)
    {
        mainMem    = SDL_ECC_MEMTYPE_MAX;
        subMemType = SDL_PSRAMECC0_PSRAM256X32E_ECC_AGGR_PSRAM256X32E_PSRAM0_ECC_RAM_ID;
        intsrc     = SDL_INJECT_ECC_ERROR_FORCING_2BIT_REPEAT;
        errSrc     = SDL_ECC_AGGR_INTR_SRC_DOUBLE_BIT;
        SDL_ECC_injectError(mainMem, subMemType, intsrc,&injectErrorConfig);
        SDL_ecc_aggrSetEccRamIntrPending(pEccAggrRegs, subMemType, errSrc);
        if (SDL_ecc_aggrIsEccRamIntrPending(pEccAggrRegs, subMemType, errSrc, &isPend) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("sdlEccAggr_apiTest: failure on line no. %d \r\n", __LINE__);
        }
        if (SDL_ecc_aggrIsAnyIntrPending(pEccAggrRegs, subMemType, &isPend) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("sdlEccAggr_apiTest: failure on line no. %d \r\n", __LINE__);
        }
    }
    if (testStatus == SDL_APP_TEST_PASS)
    {
        mainMem    = SDL_ECC_MEMTYPE_MAX;
        subMemType = SDL_PSRAMECC0_PSRAM256X32E_ECC_AGGR_PSRAM256X32E_PSRAM0_ECC_RAM_ID;
        intsrc     = SDL_INJECT_ECC_ERROR_FORCING_1BIT_REPEAT;
        errSrc     = SDL_ECC_AGGR_INTR_SRC_SINGLE_BIT;

        SDL_ECC_injectError(mainMem, subMemType, intsrc,&injectErrorConfig);
        SDL_ecc_aggrSetEccRamIntrPending(pEccAggrRegs, subMemType, errSrc);
        if (SDL_ecc_aggrIsEccRamIntrPending(pEccAggrRegs, subMemType, errSrc, &isPend) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("sdlEccAggr_apiTest: failure on line no. %d \r\n", __LINE__);
        }
        if (SDL_ecc_aggrIsAnyIntrPending(pEccAggrRegs, subMemType, &isPend) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("sdlEccAggr_apiTest: failure on line no. %d \r\n", __LINE__);
        }
    }
    if (testStatus == SDL_APP_TEST_PASS)
    {
        mainMem    = SDL_ECC_MEMTYPE_MAX;
        subMemType = SDL_PSRAMECC0_PSRAM256X32E_ECC_AGGR_PSRAM256X32E_PSRAM0_ECC_RAM_ID;
        intsrc     = SDL_INJECT_ECC_ERROR_FORCING_1BIT_ONCE;
        errSrc     = SDL_ECC_AGGR_INTR_SRC_SINGLE_BIT;

        SDL_ECC_injectError(mainMem, subMemType, intsrc,&injectErrorConfig);
        SDL_ecc_aggrSetEccRamIntrPending(pEccAggrRegs, subMemType, errSrc);
        if (SDL_ecc_aggrIsEccRamIntrPending(pEccAggrRegs, subMemType, errSrc, &isPend) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("sdlEccAggr_apiTest: failure on line no. %d \r\n", __LINE__);
        }
        if (SDL_ecc_aggrIsAnyIntrPending(pEccAggrRegs, subMemType, &isPend) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("sdlEccAggr_apiTest: failure on line no. %d \r\n", __LINE__);
        }
    }

    DebugP_log("ALL sdlEccAggr_apiTest are passed\r\n");
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
