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
 *  \file     sdl_Esm_negTest.h
 *
 *  \brief    This file contains ESM API negetive test code.
 *
 **/

#include "esm_test_main.h"

#include <kernel/dpl/DebugP.h>
#include <kernel/dpl/AddrTranslateP.h>

extern int32_t SDL_ESM_applicationCallbackFunction(SDL_ESM_Inst esmInst,
                                            int32_t grpChannel,
                                            int32_t index,
                                            void *arg);


SDL_ESM_NotifyParams esmParams =
{
	.groupNumber = 1U,
	.errorNumber = 1U,
	.setIntrPriorityLvl = 1U,
	.enableInfluenceOnErrPin = 1U,
	.callBackFunction = NULL,
};

SDL_ESM_NotifyParams verifyEsmParams =
{
	.groupNumber = 1U,
	.errorNumber = 1U,
	.setIntrPriorityLvl = 0U,
	.enableInfluenceOnErrPin = 1U,
	.callBackFunction = NULL,
};

SDL_ESM_NotifyParams verifyParams =
{
	.groupNumber = 1U,
	.errorNumber = 1U,
	.setIntrPriorityLvl = 1U,
	.enableInfluenceOnErrPin = 0U,
	.callBackFunction = NULL,
};

  /* This file contains ESM Negtive test code.*/
int32_t sdl_Esm_negTest(void)
{
    SDL_ESM_Inst         instance = SDL_ESM_INSTANCE_MAX;
    int32_t              testStatus = SDL_APP_TEST_PASS;
    int32_t apparg;
    uint32_t             pStatus;
    SDL_ESM_staticRegs         staticRegs;
    SDL_ESM_Inst         i;
    uint32_t esmBaseAddr;
    uint32_t influence;
    esmOperationMode_t esmOpMode;
    uint32_t lowTime;
    uint32_t pinCntrPre;
    uint32_t status;
	SDL_ESM_GroupIntrStatus intrstatus;
    uint32_t New_SDL_TEST_ESM_BASE;

    New_SDL_TEST_ESM_BASE = (uint32_t) AddrTranslateP_getLocalAddr(SDL_TEST_ESM_BASE);

    SDL_ESM_NotifyParams pCofnig;

   /* ESMSetInfluenceOnErrPin negative test */
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ESM_setNError(instance) != SDL_EBADARGS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("sdlEsm_negTest: failure on line no. %d \n", __LINE__);
        return (testStatus);
    }

    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ESM_setNError((SDL_ESM_Inst)SDL_ESM_INSTANCE_INVLD) != SDL_EBADARGS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("sdlEsm_negTest: failure on line no. %d \n", __LINE__);
        return (testStatus);
    }

    /* ESMGetInfluenceOnErrPin negative test */
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ESM_clrNError(instance) != SDL_EBADARGS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("sdlEsm_negTest: failure on line no. %d \n", __LINE__);
        return (testStatus);
    }


   /* ESMGetErrPinStatus negative test */
    if (testStatus == SDL_APP_TEST_PASS)
    {
        pStatus=0x12345678;
        if (SDL_ESM_getNErrorStatus(instance,&pStatus) != SDL_EBADARGS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("sdlEsm_negTest: failure on line no. %d \n", __LINE__);
        return (testStatus);
    }


    if (testStatus == SDL_APP_TEST_PASS)
    {
        instance = SDL_ESM_INST_MSS_ESM;

        if (SDL_ESM_getNErrorStatus(instance, NULL) != SDL_EBADARGS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("sdlEsm_negTest: failure on line no. %d \n", __LINE__);
        return (testStatus);
    }

    if (testStatus == SDL_APP_TEST_PASS)
    {
        instance = SDL_ESM_INST_MSS_ESM;
        if (SDL_ESM_getNErrorStatus(instance, &pStatus) != SDL_PASS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDLEsm_negTest: failure on line no. %d \n", __LINE__);
        return (testStatus);
    }



    if (testStatus == SDL_APP_TEST_PASS)
    {
        instance = SDL_ESM_INSTANCE_MAX;
        if (SDL_ESM_getNErrorStatus(instance, NULL) != SDL_EBADARGS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDLEsm_negTest: failure on line no. %d \n", __LINE__);
        return (testStatus);
    }


    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ESM_getNErrorStatus((SDL_ESM_Inst)SDL_ESM_INSTANCE_INVLD, NULL) != SDL_EBADARGS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDLEsm_negTest: failure on line no. %d \n", __LINE__);
        return (testStatus);
    }


    /* SDL_ESM_getStaticRegisters negative test */

    if (testStatus == SDL_APP_TEST_PASS)
    {
        instance = SDL_ESM_INSTANCE_MAX;
        if (SDL_ESM_getStaticRegisters(instance, &staticRegs) != SDL_EBADARGS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDLEsm_negTest: failure on line no. %d \n", __LINE__);
        return (testStatus);
    }



    if (testStatus == SDL_APP_TEST_PASS)
    {
        instance = SDL_ESM_INST_MSS_ESM;
        if (SDL_ESM_getStaticRegisters(instance, NULL) != SDL_EBADARGS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDLEsm_negTest: failure on line no. %d \n", __LINE__);
        return (testStatus);
    }

    if (testStatus == SDL_APP_TEST_PASS)
    {
        instance = SDL_ESM_INSTANCE_MAX;
        if (SDL_ESM_getStaticRegisters(instance, NULL) != SDL_EBADARGS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDLEsm_negTest: failure on line no. %d \n", __LINE__);
        return (testStatus);
    }

    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ESM_getStaticRegisters((SDL_ESM_Inst)SDL_ESM_INSTANCE_INVLD, NULL) != SDL_EBADARGS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDLEsm_negTest: failure on line no. %d \n", __LINE__);
        return (testStatus);
    }

     /* SDL_ESM_resetErrPin negative test */
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ESM_resetErrPin(0x0u) != SDL_EBADARGS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDLEsm_negTest: failure on line no. %d \n", __LINE__);
        return (testStatus);
    }

    /*  Negative test for API SDL_ESM_init  */
    if (testStatus == SDL_APP_TEST_PASS)
    {
		instance = SDL_ESM_INST_MSS_ESM;
        if (SDL_ESM_init(instance, NULL,NULL, NULL) != SDL_EBADARGS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDLEsm_negTest: failure on line no. %d \n", __LINE__);
        return (testStatus);
    }

    if (testStatus == SDL_APP_TEST_PASS)
    {
        instance = SDL_ESM_INSTANCE_MAX;
        if ((SDL_ESM_init(instance, &pCofnig,NULL, NULL)) != SDL_EBADARGS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDLEsm_negTest: failure on line no. %d \n", __LINE__);
        return (testStatus);
    }

    if (testStatus == SDL_APP_TEST_PASS)
    {
        instance = SDL_ESM_INSTANCE_MAX;
        if ((SDL_ESM_init(instance, &esmParams,NULL, NULL)) != SDL_EBADARGS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDLEsm_negTest: failure on line no. %d \n", __LINE__);
        return (testStatus);
    }

    if (testStatus == SDL_APP_TEST_PASS)
    {
        instance = SDL_ESM_INSTANCE_MAX;
        if ((SDL_ESM_init(instance, NULL,NULL, &apparg)) != SDL_EBADARGS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDLEsm_negTest: failure on line no. %d \n", __LINE__);
        return (testStatus);
    }

    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ESM_init((SDL_ESM_Inst)SDL_ESM_INSTANCE_INVLD, NULL,NULL, &apparg) != SDL_EBADARGS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDLEsm_negTest: failure on line no. %d \n", __LINE__);
        return (testStatus);
    }

    if (testStatus == SDL_APP_TEST_PASS)
    {
        instance = SDL_ESM_INSTANCE_MAX;
        if (SDL_ESM_init((SDL_ESM_Inst)SDL_ESM_INSTANCE_INVLD, &pCofnig,NULL, &apparg) != SDL_EBADARGS)
        {
            testStatus = SDL_APP_TEST_FAILED;
        }
    }
    if (testStatus != SDL_APP_TEST_PASS)
    {
        DebugP_log("SDLEsm_negTest: failure on line no. %d \n", __LINE__);
        return (testStatus);
    }

	/* Verify config API test */
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if ((SDL_ESM_verifyConfig((SDL_ESM_Inst)SDL_ESM_INSTANCE_INVLD,&esmParams)) != SDL_EBADARGS)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("sdlEsm_pos_apiTest: failure on line no. %d \n", __LINE__);
        }
    }

		/* Verify config API test */
    if (testStatus == SDL_APP_TEST_PASS)
    {
		SDL_ESM_init(SDL_ESM_INST_MSS_ESM, &esmParams,NULL, &apparg);
        if ((SDL_ESM_verifyConfig(SDL_ESM_INST_MSS_ESM,&verifyEsmParams)) != SDL_EFAIL)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("sdlEsm_pos_apiTest: failure on line no. %d \n", __LINE__);
        }
    }

	if (testStatus == SDL_APP_TEST_PASS)
    {
		SDL_ESM_init(SDL_ESM_INST_MSS_ESM, &esmParams,NULL, &apparg);
        if ((SDL_ESM_verifyConfig(SDL_ESM_INST_MSS_ESM,&verifyParams)) != SDL_EFAIL)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("sdlEsm_pos_apiTest: failure on line no. %d \n", __LINE__);
        }
    }

/* sdl_esm_core.c APIs start     */
    if (testStatus == SDL_APP_TEST_PASS)
    {
        for( i=1;i<=SDL_ESM_INST_DSS_ESM; i++)
        {
            if (SDL_ESM_getBaseAddr((SDL_ESM_Inst)i, NULL) != false)
            {
                testStatus = SDL_APP_TEST_FAILED;
                DebugP_log("sdlEsm_negTest: failure on line no. %d \n", __LINE__);
            }
        }
    }

    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ESM_getBaseAddr(SDL_ESM_INSTANCE_MAX, &esmBaseAddr) != false)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("sdlEsm_negTest: failure on line no. %d \n", __LINE__);
        }
    }

/* sdl_ip_esm.c APIs start     */

    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ESM_setMode(0U, SDL_ESM_OPERATION_MODE_ERROR_FORCE) != SDL_EBADARGS)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("sdlEsm_apiTest: failure on line no. %d \n", __LINE__);
        }
    }

    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ESM_getPinMode(0U, &esmOpMode) != SDL_EBADARGS)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("SDLEsm_negTest: failure on line no. %d \n", __LINE__);
        }
    }

    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ESM_getPinMode(New_SDL_TEST_ESM_BASE, NULL) != SDL_EBADARGS)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("SDLEsm_negTest: failure on line no. %d \n", __LINE__);
        }
    }

    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ESM_setInfluenceOnErrPin(0U, 5U, true) != SDL_EBADARGS)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("SDLEsm_negTest: failure on line no. %d \n", __LINE__);
        }
    }

    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ESM_setInfluenceOnErrPin(New_SDL_TEST_ESM_BASE, 1025U, false) != SDL_EBADARGS)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("SDLEsm_negTest: failure on line no. %d \n", __LINE__);
        }
    }

    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ESM_getInfluenceOnErrPin(0U, 5U, &influence) != SDL_EBADARGS)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("SDLEsm_negTest: failure on line no. %d \n", __LINE__);
        }
    }

    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ESM_getInfluenceOnErrPin(New_SDL_TEST_ESM_BASE, 1025U, &influence) != SDL_EBADARGS)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("SDLEsm_negTest: failure on line no. %d \n", __LINE__);
        }
    }

    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ESM_getInfluenceOnErrPin(New_SDL_TEST_ESM_BASE, 5U, NULL) != SDL_EBADARGS)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("SDLEsm_negTest: failure on line no. %d \n", __LINE__);
        }
    }

    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ESM_setErrPinLowTimePreload(0U, 0x0000FFFFU) != SDL_EBADARGS)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("SDLEsm_negTest: failure on line no. %d \n", __LINE__);
        }
    }

    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ESM_setErrPinLowTimePreload(New_SDL_TEST_ESM_BASE, 0xFFFFFFFFU) != SDL_EBADARGS)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("SDLEsm_negTest: failure on line no. %d \n", __LINE__);
        }
    }

    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ESM_getErrPinLowTimePreload(0U, &lowTime) != SDL_EBADARGS)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("SDLEsm_negTest: failure on line no. %d \n", __LINE__);
        }
    }

    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ESM_getErrPinLowTimePreload(New_SDL_TEST_ESM_BASE, NULL) != SDL_EBADARGS)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("SDLEsm_negTest: failure on line no. %d \n", __LINE__);
        }
    }

    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ESM_getCurrErrPinLowTimeCnt(0U, &pinCntrPre) != SDL_EBADARGS)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("SDLEsm_negTest: failure on line no. %d \n", __LINE__);
        }
    }

    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ESM_getCurrErrPinLowTimeCnt(New_SDL_TEST_ESM_BASE, NULL) != SDL_EBADARGS)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("SDLEsm_negTest: failure on line no. %d \n", __LINE__);
        }
    }

    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ESM_getErrPinStatus(0U, &status) != SDL_EBADARGS)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("SDLEsm_negTest: failure on line no. %d \n", __LINE__);
        }
    }

    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ESM_getErrPinStatus(New_SDL_TEST_ESM_BASE, NULL) != SDL_EBADARGS)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("SDLEsm_negTest: failure on line no. %d \n", __LINE__);
        }
    }

    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ESM_resetErrPin(0U) != SDL_EBADARGS)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("SDLEsm_negTest: failure on line no. %d \n", __LINE__);
        }
    }

    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ESM_isEnableIntr(0U, 5U, &status ) != SDL_EBADARGS)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("SDLEsm_negTest: failure on line no. %d \n", __LINE__);
        }
    }

    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ESM_isEnableIntr(New_SDL_TEST_ESM_BASE, 0xFFFFFFFFU, &status ) != SDL_EBADARGS)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("SDLEsm_negTest: failure on line no. %d \n", __LINE__);
        }
    }

    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ESM_isEnableIntr(New_SDL_TEST_ESM_BASE, 5U, NULL ) != SDL_EBADARGS)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("SDLEsm_negTest: failure on line no. %d \n", __LINE__);
        }
    }

    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ESM_enableIntr(0U, 5U) != SDL_EBADARGS)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("SDLEsm_negTest: failure on line no. %d \n", __LINE__);
        }
    }

    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ESM_enableIntr(New_SDL_TEST_ESM_BASE, 0xFFFFFFFFU) != SDL_EBADARGS)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("SDLEsm_negTest: failure on line no. %d \n", __LINE__);
        }
    }

    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ESM_disableIntr(0U, 5U) != SDL_EBADARGS)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("SDLEsm_negTest: failure on line no. %d \n", __LINE__);
        }
    }

    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ESM_disableIntr(New_SDL_TEST_ESM_BASE, 0xFFFFFFFFU) != SDL_EBADARGS)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("SDLEsm_negTest: failure on line no. %d \n", __LINE__);
        }
    }

    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ESM_setIntrPriorityLvl(0u, 5U, SDL_ESM_INTR_PRIORITY_LEVEL_LOW) != SDL_EBADARGS)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("SDLEsm_negTest: failure on line no. %d \n", __LINE__);
        }
    }

    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ESM_setIntrPriorityLvl(New_SDL_TEST_ESM_BASE, 0xFFFFFFFFU, SDL_ESM_INTR_PRIORITY_LEVEL_LOW) != SDL_EBADARGS)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("SDLEsm_negTest: failure on line no. %d \n", __LINE__);
        }
    }

    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ESM_getIntrStatus(0U, 5U, &status) != SDL_EBADARGS)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("SDLEsm_negTest: failure on line no. %d \n", __LINE__);
        }
    }

    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ESM_getIntrStatus(New_SDL_TEST_ESM_BASE, 0xFFFFFFFFU, &status) != SDL_EBADARGS)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("SDLEsm_negTest: failure on line no. %d \n", __LINE__);
        }
    }
    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ESM_getIntrStatus(New_SDL_TEST_ESM_BASE, 5U, NULL) != SDL_EBADARGS)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("SDLEsm_negTest: failure on line no. %d \n", __LINE__);
        }
    }

    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ESM_getGroupIntrStatus(0U, SDL_ESM_INTR_PRIORITY_LEVEL_LOW, &intrstatus) != SDL_EBADARGS)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("SDLEsm_negTest: failure on line no. %d \n", __LINE__);
        }
    }

    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ESM_getGroupIntrStatus(New_SDL_TEST_ESM_BASE, SDL_ESM_INTR_PRIORITY_LEVEL_LOW, NULL) != SDL_EBADARGS)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("SDLEsm_negTest: failure on line no. %d \n", __LINE__);
        }
    }

    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ESM_getGroupIntrStatus(New_SDL_TEST_ESM_BASE, 3u, NULL) != SDL_EFAIL)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("SDLEsm_negTest: failure on line no. %d \n", __LINE__);
        }
    }

    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ESM_getGroupIntrStatus(New_SDL_TEST_ESM_BASE, 4u, &intrstatus) != SDL_EFAIL)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("SDLEsm_negTest: failure on line no. %d \n", __LINE__);
        }
    }

     if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ESM_clearGroupIntrStatus(New_SDL_TEST_ESM_BASE, 4u) != SDL_EBADARGS)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("SDLEsm_negTest: failure on line no. %d \n", __LINE__);
        }
    }

    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ESM_clearIntrStatus(0U, 5U) != SDL_EBADARGS)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("SDLEsm_negTest: failure on line no. %d \n", __LINE__);
        }
    }

    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ESM_clearIntrStatus(New_SDL_TEST_ESM_BASE, 0xFFFFFFFFU) != SDL_EBADARGS)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("SDLEsm_negTest: failure on line no. %d \n", __LINE__);
        }
    }

    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ESM_getLowPriorityLvlIntrStatus((uint32_t)(0u), NULL) != SDL_EBADARGS)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("SDLEsm_negTest: failure on line no. %d \n", __LINE__);
        }
    }

    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ESM_getHighPriorityLvlIntrStatus((uint32_t)(0u), NULL) != SDL_EBADARGS)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("SDLEsm_negTest: failure on line no. %d \n", __LINE__);
        }
    }

    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ESM_getIntrPriorityLvl((uint32_t)(0u), 1025u,((void *) 0u)) != SDL_EBADARGS)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("SDLEsm_negTest: failure on line no. %d \n", __LINE__);
        }
    }

    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ESM_getIntrPriorityLvl(New_SDL_TEST_ESM_BASE, 1025u,((void *) 0u)) != SDL_EBADARGS)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("SDLEsm_negTest: failure on line no. %d \n", __LINE__);
        }
    }

    if (testStatus == SDL_APP_TEST_PASS)
    {
        if (SDL_ESM_getIntrPriorityLvl(New_SDL_TEST_ESM_BASE, 0u,((void *) 0u)) != SDL_EBADARGS)
        {
            testStatus = SDL_APP_TEST_FAILED;
            DebugP_log("SDLEsm_negTest: failure on line no. %d \n", __LINE__);
        }
    }

/* sdl_ip_esm.c APIs end     */


    return (testStatus);
}
