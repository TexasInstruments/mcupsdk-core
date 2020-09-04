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
 *  \file     esm_test_main.c
 *
 *  \brief    This file contains ESM test code.
 *
 *  \details  ESM tests
 **/

/*===========================================================================*/
/*                         Include files                                     */
/*===========================================================================*/
#include "esm_test_main.h"
#include <dpl_interface.h>
#include <sdl/sdl_esm.h>
#include <unity.h>
#include <kernel/dpl/DebugP.h>


/*===========================================================================*/
/*                         Declarations                                      */
/*===========================================================================*/
/* None */

/*===========================================================================*/
/*                         Macros                                            */
/*===========================================================================*/
/* None */

/*===========================================================================*/
/*                         Internal function declarations                    */
/*===========================================================================*/
/* Unity functions */
void test_sdl_esm_baremetal_test_app_runner(void);
void test_sdl_esm_baremetal_test_app (void *args);

void sdlApp_print(char * str)
{
    DebugP_log(str);
}

/*===========================================================================*/
/*                         Global Variables                                  */
/*===========================================================================*/
sdlEsmTest_t  sdlEsmTestList[] = {
    {sdl_Esm_posTest, "ESM POSITIVE TEST" ,    SDL_APP_TEST_NOT_RUN },
    {sdl_Esm_negTest, "ESM NEGATIVE TEST",         SDL_APP_TEST_NOT_RUN },
    {NULL,           "TERMINATING CONDITION",     SDL_APP_TEST_NOT_RUN }
};

int32_t SDL_ESM_applicationCallbackFunction(SDL_ESM_Inst esmInst,
                                            SDL_ESM_IntType esmIntrType,
                                            uint32_t grpChannel,
                                            uint32_t index,
                                            uint32_t intSrc,
                                            void *arg)
{
    int32_t retVal = SDL_PASS;
    DebugP_log("\n  ESM Call back function called : instType 0x%x, intType 0x%x, " \
                "grpChannel 0x%x, index 0x%x, intSrc 0x%x \n",
                esmInst, esmIntrType, grpChannel, index, intSrc);
    DebugP_log("  Take action \n");

    /* Any additional customer specific actions can be added here */

    return retVal;
}
int32_t SDL_ESM_errorInsert (const SDL_ESM_Inst esmInstType,
                                SDL_ESM_ErrorConfig_t *esmErrorConfig)
{
    uint32_t   esmInstBaseAddr;
    int32_t result = SDL_EFAIL;

    if (SDL_ESM_getBaseAddr(esmInstType, &esmInstBaseAddr) == ((bool)true)) {
        if (esmErrorConfig != ((void *)0u)) {
            if ((esmErrorConfig->groupNumber < SDL_ESM_MAX_EVENT_MAP_NUM_WORDS)
             && (esmErrorConfig->bitNumber < BITS_PER_WORD)) {
                /* Insert error */
                (void)SDL_ESM_setIntrStatusRAW(esmInstBaseAddr,
                                    (esmErrorConfig->groupNumber*BITS_PER_WORD)
                                    + esmErrorConfig->bitNumber);
                result = SDL_PASS;
            }
        }
    }

    return result;
}

int32_t SDR_ESM_selfTest (const SDL_ESM_Inst esmInstType,
                             uint32_t loopCount)
{
    uint32_t            esmInstBaseAddr;
    SDL_ESM_Instance_t *SDR_ESM_instance;
    uint32_t            timeCount = 0u;
    int32_t          result    = SDL_PASS;

    /* Check for valid esmInstType, and initialize appropriate esmInstBaseAddr for
     * register base and SDM_ESM_instance for SW instance structure. */
    if ((SDL_ESM_getBaseAddr(esmInstType, &esmInstBaseAddr) == ((bool)true)) &&
        (SDL_ESM_selectEsmInst(esmInstType, &SDR_ESM_instance) == ((bool)true))) {
        /* reset error and timout flags */
        SDR_ESM_instance->selfTestFlag = (bool)false;
        /* Insert error for configured self test error number */
        result = SDL_ESM_errorInsert(esmInstType,
                                     &(SDR_ESM_instance->esmInitConfig.esmErrorConfig));
        if (result == SDL_PASS) {
            do
            {
                timeCount++;
                if ((loopCount != 0u)
                    && (timeCount >= loopCount)) {
                    break;
                }
            } while(!SDR_ESM_instance->selfTestFlag);

            /* Check expected error occurred or timeout */
            if (!SDR_ESM_instance->selfTestFlag ) {
                /* Timed out */
                result = SDL_EFAIL;
            }
        }
        /* reset error and timout flags */
        SDR_ESM_instance->selfTestFlag = (bool)false;
    }

    return result;
}
/*===========================================================================*/
/*                   Local Function definitions                              */
/*===========================================================================*/
#ifdef UNITY_INCLUDE_CONFIG_H
/*
 *  ======== Unity set up and tear down ========
 */
void setUp(void)
{
    /* Do nothing */
}

void tearDown(void)
{
    /* Do nothing */
}
#endif

static int32_t sdlApp_dplInit(void)
{
    SDL_ErrType_t ret = SDL_PASS;

    ret = SDL_TEST_dplInit();
    if (ret != SDL_PASS)
    {
        DebugP_log("Error: Init Failed\n");
    }

    return ret;
}

/*===========================================================================*/
/*                         Function definitions                              */
/*===========================================================================*/

void test_sdl_esm_baremetal_test_app (void *args)
{
    /* Declarations of variables */
    int32_t    testResult = SDL_APP_TEST_PASS;
    int32_t    i;

    /* Init dpl */
    sdlApp_dplInit();

    sdlApp_print("\n ESM Test Application\r\n");

    for ( i = 0; sdlEsmTestList[i].testFunction != NULL; i++)
    {
        testResult = sdlEsmTestList[i].testFunction();
        sdlEsmTestList[i].testStatus = testResult;
    }

    testResult = SDL_APP_TEST_PASS;
    for ( i = 0; sdlEsmTestList[i].testFunction != NULL; i++)
    {
        if (sdlEsmTestList[i].testStatus != SDL_APP_TEST_PASS)
        {
            DebugP_log("Test Name: %s  FAILED \n", sdlEsmTestList[i].name);
            testResult = SDL_APP_TEST_FAILED;
            break;
        }
        else
        {
            DebugP_log("Test Name: %d  PASSED \n", i);
        }
    }


    if (testResult == SDL_APP_TEST_PASS)
    {

        DebugP_log("\n All tests have passed. \n");
    }
    else
    {

        DebugP_log("\n Few/all tests Failed \n");
    }

}

void test_sdl_esm_baremetal_test_app_runner(void)
{
    UNITY_BEGIN();
    RUN_TEST (test_sdl_esm_baremetal_test_app, 0, NULL);
    UNITY_END();
    return;
}

int32_t test_main(void)
{
    test_sdl_esm_baremetal_test_app_runner();
    return 0;
}

/* Nothing past this point */
