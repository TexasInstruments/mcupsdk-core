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
 *  \file     tog_test_api.c
 *
 *  \brief    This file contains TOG functional test code. .
 *
 *  \details  TOG Functional tests
 **/

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */
#include <stdint.h>
#include <string.h>
#include "tog_test_main.h"
#include <dpl_interface.h>


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

/* Run all APIs not exercised by functional test */
static int32_t TOG_apiTestLocal(uint32_t instanceIndex)
{
    int32_t testResult = 0;
    uint32_t readValue;
	SDL_TOG_Inst instance;
	SDL_TOG_config cfg;
    SDL_TOG_staticRegs staticRegs;
	instance = instanceIndex;
	cfg.cfgCtrl = (SDL_TOG_CFG_TIMEOUT | SDL_TOG_CFG_INTR_PENDING);
	cfg.timeoutVal	= 0x10000U;
	cfg.intrSrcs 	= SDL_TOG_INTRSRC_TRANSACTION_TIMEOUT;

    testResult = SDL_TOG_init(SDL_TOG_INSTANCE_TIMEOUT0_CFG, &cfg);
    if (testResult != SDL_PASS)
    {
        DebugP_log("\n  SDL_TOG_init API test failed on line no: %d \n", __LINE__);
        testResult = -1;
    }

    if (testResult == SDL_PASS)
    {
        testResult = SDL_TOG_verifyConfig(SDL_TOG_INSTANCE_TIMEOUT0_CFG, &cfg);
        if (testResult != SDL_PASS)
        {
            DebugP_log("\n  SDL_TOG_verifyConfig API test failed on line no: %d \n", __LINE__);
            testResult = -1;
        }
    }
    cfg.cfgCtrl = 0x0;

	if (testResult == SDL_PASS)
    {
        testResult = SDL_TOG_init(SDL_TOG_INSTANCE_TIMEOUT0_CFG, &cfg);
        if (testResult != SDL_PASS)
        {
            DebugP_log("\n  SDL_TOG_init API test failed on line no: %d \n", __LINE__);
            testResult = -1;
        }
    }
    if (testResult == SDL_PASS)
    {
        testResult = SDL_TOG_verifyConfig(SDL_TOG_INSTANCE_TIMEOUT0_CFG, &cfg);
        if (testResult != SDL_PASS)
        {
            DebugP_log("\n  SDL_TOG_verifyConfig API test failed on line no: %d \n", __LINE__);
            testResult = -1;
        }
    }
    cfg.cfgCtrl = (SDL_TOG_CFG_TIMEOUT | SDL_TOG_CFG_INTR_PENDING);
    if (testResult == SDL_PASS)
    {
        cfg.timeoutVal  = 0x10000U + 10U;
        testResult = SDL_TOG_verifyConfig(SDL_TOG_INSTANCE_TIMEOUT0_CFG, &cfg);
        if (testResult != SDL_EFAIL)
        {
            DebugP_log("\n  SDL_TOG_verifyConfig API test failed on line no: %d \n", __LINE__);
            testResult = -1;
        }
    }
    if (testResult == SDL_EFAIL)
    {
        cfg.intrSrcs    = SDL_TOG_INTRSRC_UNEXPECTED_RESPONSE;
        testResult = SDL_TOG_verifyConfig(SDL_TOG_INSTANCE_TIMEOUT0_CFG, &cfg);
        if (testResult != SDL_EFAIL)
        {
            DebugP_log("\n  SDL_TOG_verifyConfig API test failed on line no: %d \n", __LINE__);
            testResult = -1;
        }
    }

    if (testResult == SDL_EFAIL)
    {
        testResult = SDL_TOG_setIntrEnable(instance, SDL_TOG_INTRSRC_TRANSACTION_TIMEOUT, true );
        if (testResult != SDL_PASS)
        {
            DebugP_log("    SDL_TOG_setIntrEnable API test failed on line no: %d \n", __LINE__);
            testResult = -1;
        }
    }
    if (testResult == SDL_PASS)
    {
        testResult = SDL_TOG_setIntrEnable(instance, SDL_TOG_INTRSRC_TRANSACTION_TIMEOUT, false );
        if (testResult != SDL_PASS)
        {
            DebugP_log("    SDL_TOG_setIntrEnable API test failed on line no: %d \n", __LINE__);
            testResult = -1;
        }
    }
    if (testResult == SDL_PASS)
    {
        testResult = SDL_TOG_clrIntrPending(instance, SDL_TOG_INTRSRC_TRANSACTION_TIMEOUT );
        if (testResult != SDL_PASS)
        {
            DebugP_log("    SDL_TOG_clrIntrPending API test failed on line no: %d \n", __LINE__);
            testResult = -1;
        }
    }
    if (testResult == SDL_PASS)
    {
        testResult = SDL_TOG_getIntrPending(instance, &readValue);
        if (testResult != SDL_PASS)
        {
            DebugP_log("    SDL_TOG_getIntrPending API test failed on line no: %d \n", __LINE__);
            testResult = -1;
        }
    }
    if (testResult == SDL_PASS)
    {
        testResult = SDL_TOG_start(instance);
        if (testResult != SDL_PASS)
        {
            DebugP_log("    SDL_TOG_start API test failed on line no: %d \n", __LINE__);
            testResult = -1;
        }
    }
    if (testResult == SDL_PASS)
    {
        testResult = SDL_TOG_stop(instance);
        if (testResult != SDL_PASS)
        {
            DebugP_log("    SDL_TOG_stop API test failed on line no: %d \n", __LINE__);
            testResult = -1;
        }
    }
    if (testResult == SDL_PASS)
    {
        testResult = SDL_TOG_reset(instance);
        if (testResult != SDL_PASS)
        {
            DebugP_log("    SDL_TOG_Reset API test failed on line no: %d \n", __LINE__);
            testResult = -1;
        }
    }
    if (testResult == SDL_PASS)
    {
        testResult = SDL_TOG_getStaticRegisters(instance, &staticRegs);
        if (testResult != SDL_PASS)
        {
            DebugP_log("    SDL_TOG_ReadBackStaticRegisters API test failed on line no: %d \n", __LINE__);
            testResult = -1;
        }
    }
    if (testResult == SDL_PASS)
    {
        testResult = SDL_TOG_setFlushMode(instance, true);
        if (testResult != SDL_PASS)
        {
            DebugP_log("    SDL_TOG_SetFlushMode API test failed on line no: %d \n", __LINE__);
            testResult = -1;
        }
    }
    if (testResult == SDL_PASS)
    {
        testResult = SDL_TOG_setFlushMode(instance, false);
        if (testResult != SDL_PASS)
        {
            DebugP_log("    SDL_TOG_SetFlushMode API test failed on line no: %d \n", __LINE__);
            testResult = -1;
        }
    }
	if (testResult == SDL_PASS)
    {
        testResult = SDL_TOG_getIntrCount(instance, SDL_TOG_INTRSRC_TRANSACTION_TIMEOUT, &readValue );
        if (testResult != SDL_PASS)
        {
            DebugP_log("    SDL_TOG_getIntrCount API test failed on line no: %d \n", __LINE__);
            testResult = -1;
        }
    }
    return (testResult);
}

/* TOG API module test */
int32_t TOG_apiTest(void)
{
    int32_t testResult;

    testResult = TOG_apiTestLocal(0);

    return (testResult);
}
/* Nothing past this point */
