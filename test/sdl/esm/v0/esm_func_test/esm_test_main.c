/* Copyright (c) 2021-2024 Texas Instruments Incorporated
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
 *  \file     esm_test.c
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
#include <unity.h>
#include <kernel/dpl/AddrTranslateP.h>
#include <kernel/dpl/DebugP.h>

#include "ti_drivers_config.h"
#include "ti_drivers_open_close.h"
#include "ti_board_open_close.h"
#include "ti_board_config.h"
#include "ti_dpl_config.h"
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
static int32_t  sdlApp_initBoard(void);
/* Unity functions */
void test_sdl_esm_baremetal_test_app_runner(void);
void test_sdl_esm_baremetal_test_app (void *args);
extern int32_t sdl_config_test(void);
extern int32_t sdl_ecc_cb_test(void);
extern int32_t sdl_config_pwm_test(void);

void sdlApp_print(char * str)
{
    DebugP_log(str);
}

/*===========================================================================*/
/*                         Global Variables                                  */
/*===========================================================================*/
extern volatile uint8_t cfg_triggered;

sdlEsmTest_t  sdlEsmTestList[] = {
    {sdl_config_pwm_test,   "sdl config pwm test" , SDL_APP_TEST_NOT_RUN},
    {sdl_ecc_cb_test,   "ecc cb test" , SDL_APP_TEST_NOT_RUN},
    {test_sdr_test,   "callback test" , SDL_APP_TEST_NOT_RUN},
    {sdl_config_test,   "sdl config test" , SDL_APP_TEST_NOT_RUN},
    {NULL,           "TERMINATING CONDITION",     SDL_APP_TEST_NOT_RUN }
};

int32_t SDL_ESM_applicationCallbackFunction(SDL_ESM_Inst esmInst,
                                            SDL_ESM_IntType esmIntrType,
                                            uint32_t grpChannel,
                                            uint32_t index,
                                            uint32_t intSrc,
                                            void *arg)
{
    int32_t retVal = 0;
    DebugP_log("\n  ESM Call back function called : instType 0x%x, intType 0x%x, " \
                "grpChannel 0x%x, index 0x%x, intSrc 0x%x \n",
                esmInst, esmIntrType, grpChannel, index, intSrc);
    DebugP_log("  Take action \n");

    cfg_triggered = 0x1;

    /* Any additional customer specific actions can be added here */
    SDL_ESM_clrNError(esmInst);

    return retVal;
}

int32_t SDL_ESM_ECCapplicationCallbackFunction(SDL_ESM_Inst esmInst,
                                            SDL_ESM_IntType esmIntrType,
                                            uint32_t grpChannel,
                                            uint32_t index,
                                            uint32_t intSrc,
                                            void *arg)
{
    int32_t retVal = 0;
    uint32_t cb_arg = (uint32_t)arg;
    uint32_t esm_base_addr=0x0;
    SDL_ESM_getBaseAddr(esmInst, &esm_base_addr);
    retVal = SDL_ESM_disableGlobalIntr(esm_base_addr);
    DebugP_log("\n  ESM ECC Call back function called : instType 0x%x, intType 0x%x, " \
                "grpChannel 0x%x, index 0x%x, intSrc 0x%x \n",
                esmInst, esmIntrType, grpChannel, index, intSrc);
    if (cb_arg == 0x1)
    {
        DebugP_log("  This is application-registered callback\n");
    }
    else if (cb_arg == 0x2)
    {
        DebugP_log("  This is ECC-registered callback");
    }

    DebugP_log("  Take action \n");

    /* Any additional customer specific actions can be added here */
    SDL_ESM_clrNError(esmInst);
    retVal = SDL_ESM_disableGlobalIntr(esm_base_addr);
    return retVal;
}

int32_t SDR_ESM_errorInsert (const SDL_ESM_Inst esmInstType,
                                const SDL_ESM_ErrorConfig_t *esmErrorConfig)
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
DebugP_log("inside test_sdl_esm_baremetal_test_app \r\n");

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
            DebugP_log("Test Name: %s  FAILED \r\n", sdlEsmTestList[i].name);
            testResult = SDL_APP_TEST_FAILED;
            break;
        }
        else
        {
            DebugP_log("Test Name: %s  PASSED \r\n", sdlEsmTestList[i].name);
        }
    }

    if (testResult == SDL_APP_TEST_PASS)
    {
        DebugP_log("\r\n All tests have passed. \r\n");
    }
    else
    {
        DebugP_log("\r\n Few/all tests Failed \r\n");
    }
#if defined (UNITY_INCLUDE_CONFIG_H)
    TEST_ASSERT_EQUAL_INT32(SDL_APP_TEST_PASS, testResult);
#endif
}

void test_sdl_esm_baremetal_test_app_runner(void)
{
    DebugP_log("inside test_sdl_esm_baremetal_test_app_runner \r\n");
#if defined(UNITY_INCLUDE_CONFIG_H)
    UNITY_BEGIN();
    RUN_TEST (test_sdl_esm_baremetal_test_app,0, NULL);
    UNITY_END();
#else
    test_sdl_esm_baremetal_test_app();
#endif
    return;
}

int32_t test_main(void)
{
    Drivers_open();
    Board_driversOpen();
    test_sdl_esm_baremetal_test_app_runner();
    Board_driversClose();
    Drivers_close();
    return 0;
}

/* Nothing past this point */
