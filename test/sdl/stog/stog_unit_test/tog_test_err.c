/*
 *   Copyright (c) Texas Instruments Incorporated 2023
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
 *  \file     tog_test_err.c
 *
 *  \brief    This file contains TOG module error tests.
 *
 *  \details  TOG Error tests
 **/

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */
#include <stdint.h>
#include "tog_test_main.h"


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

extern int32_t SDL_TOG_getIntrCountInternal( uint32_t baseAddr, SDL_TOG_IntrSrc intrSrc, uint32_t *pIntrCnt );

static int32_t TOG_errNegativeTest(uint32_t instanceIndex)
{
    int32_t testResult = 0;
    int32_t sdlResult;
	SDL_TOG_config cfg;
    volatile uint32_t readValue;
    SDL_TOG_errInfo errInfo;
	SDL_TOG_Inst instance;
	instance = instanceIndex;
	uint32_t intrCnt;
	sdlResult = SDL_TOG_init((SDL_TOG_MAX_INSTANCE+1), NULL);
	if (sdlResult == SDL_PASS)
	{
		DebugP_log("\n  SDL_TOG_init negative test failed on line no: %d \n", __LINE__);
		testResult = -1;
	}

    if (testResult == 0)
    {
        sdlResult = SDL_TOG_init(instance, NULL);
        if (sdlResult == SDL_PASS)
        {
            DebugP_log("\n  SDL_TOG_init negative test failed on line no: %d \n", __LINE__);
            testResult = -1;
        }
    }
    if (testResult == 0)
    {
        sdlResult = SDL_TOG_init((SDL_TOG_MAX_INSTANCE+1), &cfg);
        if (sdlResult == SDL_PASS)
        {
            DebugP_log("\n  SDL_TOG_init negative test failed on line no: %d \n", __LINE__);
            testResult = -1;
        }
    }
    if (testResult == 0)
    {
		sdlResult = SDL_TOG_verifyConfig((SDL_TOG_MAX_INSTANCE+1), NULL);
	    if (sdlResult == SDL_PASS)
		{
			DebugP_log("\n  SDL_TOG_verifyConfig negative test failed on line no: %d \n", __LINE__);
			testResult = -1;
		}
    }
    if (testResult == 0)
    {
        sdlResult = SDL_TOG_verifyConfig(instance, NULL);
        if (sdlResult == SDL_PASS)
        {
            DebugP_log("\n  SDL_TOG_verifyConfig negative test failed on line no: %d \n", __LINE__);
            testResult = -1;
        }
    }
    if (testResult == 0)
    {
        sdlResult = SDL_TOG_verifyConfig((SDL_TOG_MAX_INSTANCE+1), &cfg);
        if (sdlResult == SDL_PASS)
        {
            DebugP_log("\n  SDL_TOG_verifyConfig negative test failed on line no: %d \n", __LINE__);
            testResult = -1;
        }
    }
    if (testResult == 0)
    {   cfg.cfgCtrl = SDL_TOG_CFG_TIMEOUT;
        cfg.timeoutVal  = (0x40000000U);
        sdlResult = SDL_TOG_init(SDL_TOG_MAX_INSTANCE, &cfg);
        if (sdlResult == SDL_PASS)
        {
            DebugP_log("\n  SDL_TOG_init negative test failed on line no: %d \n", __LINE__);
            testResult = -1;
        }
    }
    if (testResult == 0)
    {   cfg.cfgCtrl = SDL_TOG_CFG_INTR_PENDING;
        cfg.intrSrcs  = 0x0U;
        sdlResult = SDL_TOG_init(SDL_TOG_MAX_INSTANCE, &cfg);
        if (sdlResult == SDL_PASS)
        {
            DebugP_log("\n  SDL_TOG_init negative test failed on line no: %d \n", __LINE__);
            testResult = -1;
        }
    }
    if (testResult == 0)
    {   cfg.cfgCtrl = SDL_TOG_CFG_INTR_PENDING;
        cfg.intrSrcs  = 0x8U;
        sdlResult = SDL_TOG_init(SDL_TOG_MAX_INSTANCE, &cfg);
        if (sdlResult == SDL_PASS)
        {
            DebugP_log("\n  SDL_TOG_init negative test failed on line no: %d \n", __LINE__);
            testResult = -1;
        }
    }
    if (testResult == 0)
    {
        cfg.cfgCtrl = 0x0;
        sdlResult = SDL_TOG_init((SDL_TOG_MAX_INSTANCE+1), &cfg);
        if (sdlResult == SDL_PASS)
        {
            DebugP_log("\n  SDL_TOG_init negative test failed on line no: %d \n", __LINE__);
            testResult = -1;
        }
    }
    if (testResult == 0)
    {
        sdlResult = SDL_TOG_verifyConfig((SDL_TOG_MAX_INSTANCE+1), &cfg);
        if (sdlResult == SDL_PASS)
        {
            DebugP_log("\n  SDL_TOG_verifyConfig negative test failed on line no: %d \n", __LINE__);
            testResult = -1;
        }
    }
    if (testResult == 0)
    {
        sdlResult = SDL_TOG_setIntrEnable(SDL_TOG_MAX_INSTANCE,0U, true);
        if (sdlResult == SDL_PASS)
        {
            DebugP_log("\n  SDL_TOG_setIntrEnable negative test failed on line no: %d \n", __LINE__);
            testResult = -1;
        }
    }
    if (testResult == 0)
    {
        sdlResult = SDL_TOG_setIntrEnable((SDL_TOG_MAX_INSTANCE+1),SDL_TOG_INTRSRC_TRANSACTION_TIMEOUT, true);
        if (sdlResult == SDL_PASS)
        {
            DebugP_log("\n  SDL_TOG_setIntrEnable negative test failed on line no: %d \n", __LINE__);
            testResult = -1;
        }
    }
    if (testResult == 0)
    {
        sdlResult = SDL_TOG_setIntrEnable(instance, 9U, true);
        if (sdlResult == SDL_PASS)
        {
            DebugP_log("\n  SDL_TOG_setIntrEnable negative test failed on line no: %d \n", __LINE__);
            testResult = -1;
        }
    }
    if (testResult == 0)
    {
        sdlResult = SDL_TOG_clrIntrPending((SDL_TOG_MAX_INSTANCE+1),SDL_TOG_INTRSRC_TRANSACTION_TIMEOUT);
        if (sdlResult == SDL_PASS)
        {
            DebugP_log("\n  SDL_TOG_clrIntrPending negative test failed on line no: %d \n", __LINE__);
            testResult = -1;
        }
    }
    if (testResult == 0)
    {
        sdlResult = SDL_TOG_clrIntrPending(instance, 9U);
        if (sdlResult == SDL_PASS)
        {
            DebugP_log("\n  SDL_TOG_clrIntrPending negative test failed on line no: %d \n", __LINE__);
            testResult = -1;
        }
    }
    if (testResult == 0)
    {
        sdlResult = SDL_TOG_clrIntrPending((SDL_TOG_MAX_INSTANCE+1),SDL_TOG_INTRSRC_UNEXPECTED_RESPONSE);
        if (sdlResult == SDL_PASS)
        {
            DebugP_log("\n  SDL_TOG_clrIntrPending negative test failed on line no: %d \n", __LINE__);
            testResult = -1;
        }
    }
    if (testResult == 0)
    {
        sdlResult = SDL_TOG_clrIntrPending(instance, 0U);
        if (sdlResult == SDL_PASS)
        {
            DebugP_log("\n  SDL_TOG_clrIntrPending negative test failed on line no: %d \n", __LINE__);
            testResult = -1;
        }
    }
    if (testResult == 0)
    {
        sdlResult = SDL_TOG_getIntrPending((SDL_TOG_MAX_INSTANCE+1), NULL);
        if (sdlResult == SDL_PASS)
        {
            DebugP_log("\n  SDL_TOG_getIntrPending negative test failed on line no: %d \n", __LINE__);
            testResult = -1;
        }
    }
    if (testResult == 0)
    {
        sdlResult = SDL_TOG_getIntrPending(instance, NULL);
        if (sdlResult == SDL_PASS)
        {
            DebugP_log("\n  SDL_TOG_getIntrPending negative test failed on line no: %d \n", __LINE__);
            testResult = -1;
        }
    }
    if (testResult == 0)
    {
        sdlResult = SDL_TOG_getIntrPending((SDL_TOG_MAX_INSTANCE+1), (uint32_t *)&readValue);
        if (sdlResult == SDL_PASS)
        {
            DebugP_log("\n  SDL_TOG_getIntrPending negative test failed on line no: %d \n", __LINE__);
            testResult = -1;
        }
    }
    if (testResult == 0)
    {
        sdlResult = SDL_TOG_ackIntr((SDL_TOG_MAX_INSTANCE+1), 5U, 0U);
        if (sdlResult == SDL_PASS)
        {
            DebugP_log("\n  SDL_TOG_ackIntr negative test failed on line no: %d \n", __LINE__);
            testResult = -1;
        }
    }
    if (testResult == 0)
    {
        sdlResult = SDL_TOG_ackIntr((SDL_TOG_MAX_INSTANCE+1), SDL_TOG_INTRSRC_TRANSACTION_TIMEOUT, 1U);
        if (sdlResult == SDL_PASS)
        {
            DebugP_log("\n  SDL_TOG_ackIntr negative test failed on line no: %d \n", __LINE__);
            testResult = -1;
        }
    }
    if (testResult == 0)
    {
        /* Try to Ack interrupt when there is not actual interrupt event */
        sdlResult = SDL_TOG_ackIntr(instance, SDL_TOG_INTRSRC_TRANSACTION_TIMEOUT, 1U );
        if (sdlResult == SDL_PASS)
        {
            DebugP_log("\n  SDL_TOG_ackIntr negative test failed on line no: %d \n", __LINE__);
            testResult = -1;
        }
    }
    if (testResult == 0)
    {
        sdlResult = SDL_TOG_ackIntr(instance, 5U, 1U);
        if (sdlResult == SDL_PASS)
        {
            DebugP_log("\n  SDL_TOG_ackIntr negative test failed on line no: %d \n", __LINE__);
            testResult = -1;
        }
    }
    if (testResult == 0)
    {
        sdlResult = SDL_TOG_ackIntr(instance, SDL_TOG_INTRSRC_TRANSACTION_TIMEOUT, 0U);
        if (sdlResult == SDL_PASS)
        {
            DebugP_log("\n  SDL_TOG_ackIntr negative test failed on line no: %d \n", __LINE__);
            testResult = -1;
        }
    }
    if (testResult == 0)
    {
        /* Try to Ack interrupt when there is not actual interrupt event */
        sdlResult = SDL_TOG_ackIntr(instance, SDL_TOG_INTRSRC_UNEXPECTED_RESPONSE, 1U);
        if (sdlResult == SDL_PASS)
        {
            DebugP_log("\n  SDL_TOG_ackIntr negative test failed on line no: %d \n", __LINE__);
            testResult = -1;
        }
    }
    if (testResult == 0)
    {
        sdlResult = SDL_TOG_ackIntr(instance, 10U, 1U);
        if (sdlResult == SDL_PASS)
        {
            DebugP_log("\n  SDL_TOG_ackIntr negative test failed on line no: %d \n", __LINE__);
            testResult = -1;
        }
    }
    if (testResult == 0)
    {
        sdlResult = SDL_TOG_start((SDL_TOG_MAX_INSTANCE+1));
        if (sdlResult == SDL_PASS)
        {
            DebugP_log("\n  SDL_TOG_start negative test failed on line no: %d \n", __LINE__);
            testResult = -1;
        }
    }
    if (testResult == 0)
    {
        sdlResult = SDL_TOG_stop((SDL_TOG_MAX_INSTANCE+1));
        if (sdlResult == SDL_PASS)
        {
            DebugP_log("\n  SDL_TOG_stop negative test failed on line no: %d \n", __LINE__);
            testResult = -1;
        }
    }
    if (testResult == 0)
    {
        sdlResult = SDL_TOG_reset((SDL_TOG_MAX_INSTANCE+1));
        if (sdlResult == SDL_PASS)
        {
            DebugP_log("\n  SDL_TOG_Reset negative test failed on line no: %d \n", __LINE__);
            testResult = -1;
        }
    }
    if (testResult == 0)
    {
		sdlResult = SDL_TOG_getErrInfo((SDL_TOG_MAX_INSTANCE+1), NULL);
	    if (sdlResult == SDL_PASS)
		{
			DebugP_log("\n  SDL_TOG_getErrInfo negative test failed on line no: %d \n", __LINE__);
			testResult = -1;
		}
	}
    if (testResult == 0)
    {
        sdlResult = SDL_TOG_getErrInfo(instance, NULL);
        if (sdlResult == SDL_PASS)
        {
            DebugP_log("\n  SDL_TOG_getErrInfo negative test failed on line no: %d \n", __LINE__);
            testResult = -1;
        }
    }
    if (testResult == 0)
    {
        sdlResult = SDL_TOG_getErrInfo((SDL_TOG_MAX_INSTANCE+1), (SDL_TOG_errInfo *)&readValue);
        if (sdlResult == SDL_PASS)
        {
            DebugP_log("\n  SDL_TOG_getErrInfo negative test failed on line no: %d \n", __LINE__);
            testResult = -1;
        }
    }
    if (testResult == 0)
    {
        /* Getting the error when there is no actual error */
        sdlResult = SDL_TOG_getErrInfo(instance, &errInfo);
        if (sdlResult == SDL_PASS)
        {
            DebugP_log("\n  SDL_TOG_getErrInfo negative test failed on line no: %d \n", __LINE__);
            testResult = -1;
        }
    }
    if (testResult == 0)
    {
        sdlResult = SDL_TOG_getStaticRegisters((SDL_TOG_MAX_INSTANCE+1), NULL);
        if (sdlResult == SDL_PASS)
        {
            DebugP_log("\n  SDL_TOG_getStaticRegisters negative test failed on line no: %d \n", __LINE__);
            testResult = -1;
        }
    }
    if (testResult == 0)
    {
        sdlResult = SDL_TOG_getStaticRegisters(instance, NULL);
        if (sdlResult == SDL_PASS)
        {
            DebugP_log("\n  SDL_TOG_getStaticRegisters negative test failed on line no: %d \n", __LINE__);
            testResult = -1;
        }
    }
    if (testResult == 0)
    {
        sdlResult = SDL_TOG_getStaticRegisters((SDL_TOG_MAX_INSTANCE+1), (SDL_TOG_staticRegs *)(&readValue));
        if (sdlResult == SDL_PASS)
        {
            DebugP_log("\n  SDL_TOG_getStaticRegisters negative test failed on line no: %d \n", __LINE__);
            testResult = -1;
        }
    }
    if (testResult == 0)
    {
        sdlResult = SDL_TOG_setFlushMode((SDL_TOG_MAX_INSTANCE+1), true);
        if (sdlResult == SDL_PASS)
        {
            DebugP_log("\n  SDL_TOG_SetFlushModeEnable negative test failed on line no: %d \n", __LINE__);
            testResult = -1;
        }
    }
    if (testResult == 0)
    {
        sdlResult = SDL_TOG_getIntrCount((SDL_TOG_MAX_INSTANCE+1), 5U, NULL);
        if (sdlResult == SDL_PASS)
        {
            DebugP_log("\n  SDL_TOG_getIntrCount negative test failed on line no: %d \n", __LINE__);
            testResult = -1;
        }
    }
    if (testResult == 0)
    {
        sdlResult = SDL_TOG_getIntrCount((SDL_TOG_MAX_INSTANCE+1), SDL_TOG_INTRSRC_TRANSACTION_TIMEOUT, (uint32_t *)&readValue);
        if (sdlResult == SDL_PASS)
        {
            DebugP_log("\n  SDL_TOG_getIntrCount negative test failed on line no: %d \n", __LINE__);
            testResult = -1;
        }
    }
    if (testResult == 0)
    {
        sdlResult = SDL_TOG_getIntrCount(instance, 5U, (uint32_t *)&readValue);
        if (sdlResult == SDL_PASS)
        {
            DebugP_log("\n  SDL_TOG_getIntrCount negative test failed on line no: %d \n", __LINE__);
            testResult = -1;
        }
    }
    if (testResult == 0)
    {
        sdlResult = SDL_TOG_getIntrCount(instance, SDL_TOG_INTRSRC_TRANSACTION_TIMEOUT, NULL);
        if (sdlResult == SDL_PASS)
        {
            DebugP_log("\n  SDL_TOG_getIntrCount negative test failed on line no: %d \n", __LINE__);
            testResult = -1;
        }
    }
    if (testResult == 0)
    {
        sdlResult = SDL_TOG_getIntrCountInternal(SDL_TOG_baseAddress[instance], SDL_TOG_INTRSRC_COMMAND_TIMEOUT, &intrCnt);
        if (sdlResult == SDL_PASS)
        {
            DebugP_log("\n  SDL_TOG_getIntrCountInternal negative test failed on line no: %d \n", __LINE__);
            testResult = -1;
        }
    }
    if (testResult == 0)
    {
        sdlResult = SDL_TOG_getIntrCountInternal(SDL_TOG_baseAddress[instance], SDL_TOG_INTRSRC_TRANSACTION_TIMEOUT, NULL);
        if (sdlResult == SDL_PASS)
        {
            DebugP_log("\n  SDL_TOG_getIntrCountInternal negative test failed on line no: %d \n", __LINE__);
            testResult = -1;
        }
    }
    if (testResult == 0)
    {
        sdlResult = SDL_TOG_getIntrCountInternal(SDL_TOG_baseAddress[instance], 10U, &intrCnt);
        if (sdlResult == SDL_PASS)
        {
            DebugP_log("\n  SDL_TOG_getIntrCountInternal negative test failed on line no: %d \n", __LINE__);
            testResult = -1;
        }
    }
    if (testResult == 0)
    {
        sdlResult = SDL_TOG_getIntrCountInternal(SDL_TOG_baseAddress[instance], 0U, &intrCnt);
        if (sdlResult == SDL_PASS)
        {
            DebugP_log("\n  SDL_TOG_getIntrCountInternal negative test failed on line no: %d \n", __LINE__);
            testResult = -1;
        }
    }
    return (testResult);
}

/* TOG Error module test */
int32_t TOG_errTest(void)
{
    int32_t testResult;

    testResult = TOG_errNegativeTest(0U);

    return (testResult);
}

/* Nothing past this point */
