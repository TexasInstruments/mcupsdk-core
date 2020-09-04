 /* Copyright (c) 2021 Texas Instruments Incorporated
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
 *  \file     sdl_Esm_posTest.h
 *
 *  \brief    This file contains ESM API positive test code.
 *
 **/

#define DISP_APP_ARGB32                   (3U)

#include "esm_test_main.h"
#include <kernel/dpl/DebugP.h>
#include <kernel/dpl/ClockP.h>
#include <kernel/dpl/AddrTranslateP.h>

#define SDTF_NUM_RUNALL_TEST_COMMANDS 3
#define MASK_BIT (1u)
#define STATUS_NUM (1u)
#define SDL_ESM_EN_KEY_ENBALE_VAL (0xFU)

SDL_ESM_NotifyParams params = 
{
	.groupNumber = 1U,
	.errorNumber = 1U,
	.setIntrPriorityLvl = 1U,
	.enableInfluenceOnErrPin = 1U,
	.callBackFunction = NULL,
};

SDL_ESM_OpenParams openPerams = 
{
	.bClearErrors = FALSE
};

int32_t sdl_Esm_posTest(void)
{
 	SDL_ESM_Inst         i = SDL_ESM_INST_MSS_ESM;
	SDL_ESM_Inst Test_instance = i;

    int32_t              testStatus = SDL_APP_TEST_PASS;
    SDL_ESM_staticRegs         staticRegs;
    esmOperationMode_t esmOpMode;
    uint32_t influence;
    uint32_t lowTime;
    uint32_t pinCntrPre;
    uint32_t status;
	SDL_ESM_GroupIntrStatus intrstatus;
    uint32_t New_SDL_TEST_ESM_BASE;
    New_SDL_TEST_ESM_BASE = (uint32_t) AddrTranslateP_getLocalAddr(SDL_TEST_ESM_BASE);

    if (testStatus == SDL_APP_TEST_PASS)
    {

        if (SDL_ESM_setMode(New_SDL_TEST_ESM_BASE, SDL_ESM_OPERATION_MODE_NORMAL) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("sdlEsm_apiTest: failure on line no. %d \n", __LINE__);
        }
    }

    if (testStatus == SDL_APP_TEST_PASS)
    {

        if (SDL_ESM_setMode(New_SDL_TEST_ESM_BASE, SDL_ESM_OPERATION_MODE_ERROR_FORCE) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("sdlEsm_apiTest: failure on line no. %d \n", __LINE__);
        }
    }

    if (testStatus == SDL_APP_TEST_PASS)
    {

        if (SDL_ESM_getPinMode(New_SDL_TEST_ESM_BASE, &esmOpMode) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("sdlEsm_apiTest: failure on line no. %d \n", __LINE__);
        }
    }

    if (testStatus == SDL_APP_TEST_PASS)
    {

        if (SDL_ESM_getStaticRegisters(i, &staticRegs) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("sdlEsm_apiTest: failure on line no. %d \n", __LINE__);
        }
    }
	
	if (testStatus == SDL_APP_TEST_PASS)
    {

        if (SDL_ESM_setInfluenceOnErrPin(New_SDL_TEST_ESM_BASE, 5U, true) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("sdlEsm_apiTest: failure on line no. %d \n", __LINE__);
        }
    }

    if (testStatus == SDL_APP_TEST_PASS)
    {

        if (SDL_ESM_getInfluenceOnErrPin(New_SDL_TEST_ESM_BASE, 5U, &influence) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("sdlEsm_apiTest: failure on line no. %d \n", __LINE__);
        }
    }

    if (testStatus == SDL_APP_TEST_PASS)
    {

        if (SDL_ESM_setInfluenceOnErrPin(New_SDL_TEST_ESM_BASE, 5U, false) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("sdlEsm_apiTest: failure on line no. %d \n", __LINE__);
        }
    }

    if (testStatus == SDL_APP_TEST_PASS)
    {

        if (SDL_ESM_getInfluenceOnErrPin(New_SDL_TEST_ESM_BASE, 5U, &influence) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("sdlEsm_apiTest: failure on line no. %d \n", __LINE__);
        }
    }

    if (testStatus == SDL_APP_TEST_PASS)
    {

        if (SDL_ESM_setErrPinLowTimePreload(New_SDL_TEST_ESM_BASE, 0x0000FFFFU) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("sdlEsm_apiTest: failure on line no. %d \n", __LINE__);
        }
    }

    if (testStatus == SDL_APP_TEST_PASS)
    {

        if (SDL_ESM_getErrPinLowTimePreload(New_SDL_TEST_ESM_BASE, &lowTime) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("sdlEsm_apiTest: failure on line no. %d \n", __LINE__);
        }
    }

    if (testStatus == SDL_APP_TEST_PASS)
    {

        if (SDL_ESM_getCurrErrPinLowTimeCnt(New_SDL_TEST_ESM_BASE, &pinCntrPre) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("sdlEsm_apiTest: failure on line no. %d \n", __LINE__);
        }
    }

    if (testStatus == SDL_APP_TEST_PASS)
    {

        if (SDL_ESM_getErrPinStatus(New_SDL_TEST_ESM_BASE, &status) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("sdlEsm_apiTest: failure on line no. %d \n", __LINE__);
        }
    }

    if (testStatus == SDL_APP_TEST_PASS)
    {

        if (SDL_ESM_resetErrPin(New_SDL_TEST_ESM_BASE) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("sdlEsm_apiTest: failure on line no. %d \n", __LINE__);
        }
    }

    if (testStatus == SDL_APP_TEST_PASS)
    {

        if (SDL_ESM_enableIntr(New_SDL_TEST_ESM_BASE, 5U) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("sdlEsm_apiTest: failure on line no. %d \n", __LINE__);
        }
    }


    if (testStatus == SDL_APP_TEST_PASS)
    {

        if (SDL_ESM_isEnableIntr(New_SDL_TEST_ESM_BASE, 5U, &status ) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("sdlEsm_apiTest: failure on line no. %d \n", __LINE__);
        }
    }

    if (testStatus == SDL_APP_TEST_PASS)
    {

        if (SDL_ESM_enableIntr(New_SDL_TEST_ESM_BASE, 12U) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("sdlEsm_apiTest: failure on line no. %d \n", __LINE__);
        }
    }

        if (testStatus == SDL_APP_TEST_PASS)
    {

        if (SDL_ESM_isEnableIntr(New_SDL_TEST_ESM_BASE, 12U, &status ) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("sdlEsm_apiTest: failure on line no. %d \n", __LINE__);
        }
    }

    if (testStatus == SDL_APP_TEST_PASS)
    {

        if (SDL_ESM_disableIntr(New_SDL_TEST_ESM_BASE, 5U) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("sdlEsm_apiTest: failure on line no. %d \n", __LINE__);
        }
    }

    if (testStatus == SDL_APP_TEST_PASS)
    {

        if (SDL_ESM_setIntrPriorityLvl(New_SDL_TEST_ESM_BASE, 5U, SDL_ESM_INTR_PRIORITY_LEVEL_LOW) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("sdlEsm_apiTest: failure on line no. %d \n", __LINE__);
        }
    }

    if (testStatus == SDL_APP_TEST_PASS)
    {

        if (SDL_ESM_setIntrPriorityLvl(New_SDL_TEST_ESM_BASE, 5U, SDL_ESM_INTR_PRIORITY_LEVEL_HIGH) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("sdlEsm_apiTest: failure on line no. %d \n", __LINE__);
        }
    }

    if (testStatus == SDL_APP_TEST_PASS)
    {

        if (SDL_ESM_getIntrStatus(New_SDL_TEST_ESM_BASE, 5U, &status) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("sdlEsm_apiTest: failure on line no. %d \n", __LINE__);
        }
    }
    if (testStatus == SDL_APP_TEST_PASS)
    {

        if (SDL_ESM_clearIntrStatus(New_SDL_TEST_ESM_BASE, 5U) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("sdlEsm_apiTest: failure on line no. %d \n", __LINE__);
        }
    }
	
    if (testStatus == SDL_APP_TEST_PASS)
    {

        if (SDL_ESM_getGroupIntrStatus(New_SDL_TEST_ESM_BASE, 1u, &intrstatus) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("sdlEsm_apiTest: failure on line no. %d \n", __LINE__);
        }
    }

    if (testStatus == SDL_APP_TEST_PASS)
    {

        if (SDL_ESM_getGroupIntrStatus(New_SDL_TEST_ESM_BASE, 2u, &intrstatus) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("sdlEsm_apiTest: failure on line no. %d \n", __LINE__);
        }
    }
	
    /* SDL_ESM_init API test */
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if ((SDL_ESM_init(Test_instance,&params,NULL, NULL)) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("sdlEsm_pos_apiTest: failure on line no. %d \n", __LINE__);
        }
    }
	
	    /* SDL_ESM_init API test */
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if ((SDL_ESM_init(Test_instance,&params,&openPerams, NULL)) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("sdlEsm_pos_apiTest: failure on line no. %d \n", __LINE__);
        }
    }
	
	    /* Verify config API test */
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if ((SDL_ESM_verifyConfig(Test_instance,&params)) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("sdlEsm_pos_apiTest: failure on line no. %d \n", __LINE__);
        }
    }

return (testStatus);
}

static uint32_t arg;
void  esm_init_appcb(SDL_ESM_Inst esmType)
{
    void *ptr = (void *)&arg;
    SDL_ErrType_t result;
    result = SDL_ESM_init(esmType, &params,NULL,ptr);
        if (result != SDL_PASS) {
            /* print error and quit */
            DebugP_log("ESM_ECC_Example_init: Error initializing MAIN ESM: result = %d\n", result);

        } else {
            DebugP_log("\nESM_ECC_Example_init: Init MAIN ESM complete \n");
        }
}
