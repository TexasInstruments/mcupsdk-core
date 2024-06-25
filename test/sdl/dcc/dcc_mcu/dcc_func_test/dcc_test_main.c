/*
 *  Copyright (c) 2022 Texas Instruments Incorporated
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
 *  \file     main.c
 *
 *  \brief    This file contains DCC Function test code.
 *
 *  \details  DCC tests
 **/

/*===========================================================================*/
/*                         Include files                                     */
/*===========================================================================*/
#include "dcc_test_main.h"


#ifdef UNITY_INCLUDE_CONFIG_H
#include <unity/unity.h>
#include <unity/unity_config.h>
#endif


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

static void sdlApp_print( char * str);

/* Unity functions */
void test_sdl_dcc_baremetal_test_app_runner(void);
void test_sdl_dcc_baremetal_test_app (void);


/*===========================================================================*/
/*                         Global Variables                                  */
/*===========================================================================*/
sdlDccTest_t  sdlDccTestList[] = {
    {SDL_DCC_funcTest, "DCC FUNCTION TEST" ,     SDL_APP_TEST_NOT_RUN },
    {NULL,             "TERMINATING CONDITION",  SDL_APP_TEST_NOT_RUN }
};

#if defined (SOC_AM263X) || defined (SOC_AM263PX) || defined (SOC_AM261X)
SDL_ESM_config DCC_Test_esmInitConfig_MAIN =
{
      .esmErrorConfig = {1u, 8u}, /* Self test error config */
    .enableBitmap = {0x01E00000u, 0x00000000u, 0x00000000, 0x00000000u,
                },
     /**< Only DCC events enable:**/
	  /* CCM_1_SELFTEST_ERR and _R5FSS1_COMPARE_ERR_PULSE_0 */
    .priorityBitmap = {0x01E00000u, 0x00000000u, 0x00000000, 0x00000000u,
                        },
    /**< DCC events high priority:**/
    .errorpinBitmap = {0x01E00000u, 0x00000000u, 0x00000000, 0x00000000u,
                      },
    /**< All events high priority:**/
};

#endif

#if defined (SOC_AWR294X)
SDL_ESM_NotifyParams gESM_Params=

{
    ESM_ERROR_GROUP_1,
    DCCA_MSS_ESM_ERROR,
    0U,
    TRUE,
    NULL,
    &SDL_ESM_applicationCallbackFunction

};

SDL_ESM_OpenParams esmOpenParams=
{
    TRUE
};


#endif

static uint32_t arg;

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

static void sdlApp_print( char * str)
{
    DebugP_log(str);
}


static int32_t sdlApp_dplInit(void)
{
    SDL_ErrType_t ret = SDL_PASS;

    ret = SDL_TEST_dplInit();
    if (ret != SDL_PASS)
    {
        DebugP_log("Error: dpl Init Failed\r\n");
    }

    return ret;
}

#if defined (SOC_AWR294X)
void SDL_DCCA_clockInit( void )
{
    HW_WR_FIELD32(SDL_MSS_RCM_U_BASE+SDL_MSS_RCM_MSS_MCANA_CLK_SRC_SEL,\
        SDL_MSS_RCM_MSS_MCANA_CLK_SRC_SEL_MSS_MCANA_CLK_SRC_SEL_CLKSRCSEL, 0X111U);
    HW_WR_FIELD32(SDL_MSS_RCM_U_BASE+SDL_MSS_RCM_MSS_MCANA_CLK_DIV_VAL,\
        SDL_MSS_RCM_MSS_MCANA_CLK_DIV_VAL_MSS_MCANA_CLK_DIV_VAL_CLKDIVR, 0X777U);
    HW_WR_FIELD32(SDL_MSS_RCM_U_BASE+SDL_MSS_RCM_MSS_RTIA_CLK_SRC_SEL,\
        SDL_MSS_RCM_MSS_RTIA_CLK_SRC_SEL_MSS_RTIA_CLK_SRC_SEL_CLKSRCSEL, 0X333U);
    HW_WR_FIELD32(SDL_MSS_RCM_U_BASE+SDL_MSS_RCM_MSS_RTIA_CLK_DIV_VAL,\
        SDL_MSS_RCM_MSS_RTIA_CLK_DIV_VAL_MSS_RTIA_CLK_DIV_VAL_CLKDIVR, 0X000U);

    HW_WR_FIELD32(SDL_DSS_RCM_U_BASE + SDL_DSS_RCM_DSS_RTIA_CLK_SRC_SEL,\
        SDL_DSS_RCM_DSS_RTIA_CLK_SRC_SEL_DSS_RTIA_CLK_SRC_SEL_CLKSRCSEL, 0X222U);
    HW_WR_FIELD32(SDL_DSS_RCM_U_BASE+SDL_DSS_RCM_DSS_RTIA_CLK_DIV_VAL,\
        SDL_DSS_RCM_DSS_RTIA_CLK_DIV_VAL_DSS_RTIA_CLK_DIV_VAL_CLKDIV, 0X000U);
     HW_WR_FIELD32((SDL_DSS_RCM_U_BASE + SDL_DSS_RCM_DSS_WDT_CLK_SRC_SEL),\
        SDL_DSS_RCM_DSS_WDT_CLK_SRC_SEL_DSS_WDT_CLK_SRC_SEL_CLKSRCSEL, 0X333);
    HW_WR_FIELD32((SDL_DSS_RCM_U_BASE + SDL_DSS_RCM_DSS_WDT_CLK_DIV_VAL),\
        SDL_DSS_RCM_DSS_WDT_CLK_DIV_VAL_DSS_WDT_CLK_DIV_VAL_CLKDIV, 0X111);
}
#endif

#if defined (SOC_AWR294X)
extern int32_t SDL_ESM_applicationCallbackFunction(SDL_ESM_Inst esmInst,
                                            int grpChannel, int intSrc, void *arg);

#endif
/*===========================================================================*/
/*                         Function definitions                              */
/*===========================================================================*/

void test_sdl_dcc_baremetal_test_app (void)
{
    /* Declarations of variables */
    int32_t    testResult = SDL_APP_TEST_PASS;
    int32_t    i,result;
    void *ptr = (void *)&arg;


    Drivers_open();
    Board_driversOpen();
    sdlApp_dplInit();
#if defined (SOC_AWR294X)
    SDL_DCCA_clockInit();
#endif
    sdlApp_print("\n DCC Function Test Application\r\n");


#if defined (SOC_AM263X) || defined (SOC_AM263PX) || defined (SOC_AM261X)
    /* Initialize MAIN DCC module */
    result = SDL_ESM_init(SDL_ESM_INST_MAIN_ESM0, &DCC_Test_esmInitConfig_MAIN, SDL_ESM_applicationCallbackFunction, ptr);

    if (result != SDL_PASS)
    {
        /* print error and quit */
        DebugP_log("DCC_Test_init: Error initializing MCU ESM: result = %d\r\n", result);
    }
    else
    {
        sdlApp_print("\nDCC_Test_init: Init MCU ESM complete \r\n\n");
    }
#elif  defined (SOC_AWR294X)
    #if defined (R5F_INPUTS)
    gESM_Params.groupNumber=ESM_ERROR_GROUP_1;
    gESM_Params.errorNumber=DCCA_MSS_ESM_ERROR;

    /* Initialize MCU DCC module */
    result = SDL_ESM_init(SDL_ESM_INST_MSS_ESM, &gESM_Params, &esmOpenParams, ptr);
    #endif

    #if defined (C66_INPUTS)
    gESM_Params.groupNumber=ESM_ERROR_GROUP_2;
    gESM_Params.errorNumber=DCCA_DSS_ESM_ERROR;


    result = SDL_ESM_init(SDL_ESM_INST_DSS_ESM, &gESM_Params, &esmOpenParams, ptr);
    #endif

    if (result == SDL_PASS)
    {

        DebugP_log("\r\nDCC_Test_init: Init ESM complete \r\n\n");
    }
    else
    {

        /* print error and quit */
        DebugP_log("DCC_Test_init: Error initializing ESM: result = %d\r\n", result);

    }

#endif


    for ( i = 0; sdlDccTestList[i].testFunction != NULL; i++)
    {
        testResult = sdlDccTestList[i].testFunction();
        sdlDccTestList[i].testStatus = testResult;
    }

    testResult = SDL_APP_TEST_PASS;
    for ( i = 0; sdlDccTestList[i].testFunction != NULL; i++)
    {
        if (sdlDccTestList[i].testStatus != SDL_APP_TEST_PASS)
        {
            DebugP_log("\r\nTest Name: %s  FAILED \r\n", sdlDccTestList[i].name);
            testResult = SDL_APP_TEST_FAILED;
            break;
        }
        else
        {
            DebugP_log("\r\nTest Name: %s  PASSED \r\n", sdlDccTestList[i].name);
        }
    }

    if (testResult == SDL_APP_TEST_PASS)
    {
        sdlApp_print("\r\n All tests have passed. \r\n");
    }
    else
    {
        sdlApp_print("\r\n Few/all tests Failed \r\n");
    }
#if defined (UNITY_INCLUDE_CONFIG_H)
    TEST_ASSERT_EQUAL_INT32(SDL_APP_TEST_PASS, testResult);
#endif

    Board_driversClose();
    Drivers_close();

}

void test_sdl_dcc_baremetal_test_app_runner(void)
{
    /* @description:Test runner for DCC tests

       @cores: mcu1_0 */

#if defined(UNITY_INCLUDE_CONFIG_H)
    UNITY_BEGIN();
    RUN_TEST (test_sdl_dcc_baremetal_test_app);
    UNITY_END();
    /* Function to print results defined in our unity_config.h file */
    print_unityOutputBuffer_usingUARTstdio();
#else
    test_sdl_dcc_baremetal_test_app();
#endif
    return;
}

void test_main(void *args)
{
    test_sdl_dcc_baremetal_test_app_runner();

}

/* Nothing past this point */
