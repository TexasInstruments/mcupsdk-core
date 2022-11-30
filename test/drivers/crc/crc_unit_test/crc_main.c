/* Copyright (C) 2023 Texas Instruments Incorporated
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
 *  \file     crc_main.c
 *
 *  \brief    This file contains crc test code.
 *
 *  \details  crc tests
 **/

/*===========================================================================*/
/*                         Include files                                     */
/*===========================================================================*/
#include "crc_main.h"


#ifdef UNITY_INCLUDE_CONFIG_H
#include <ti/build/unit-test/Unity/src/unity.h>
#include <ti/build/unit-test/config/unity_config.h>
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
/* Unity functions */
void test_crc_test_app_runner(void);
void test_crc_test_app (void);

/*===========================================================================*/
/*                         Global Variables                                  */
/*===========================================================================*/
crcTest_t  crcTestList[] = {
    {crc_posTest,      "crc API POSITIVE TEST",    CSL_APP_TEST_NOT_RUN },
	{crc_negTest,      "crc API NEGATIVE TEST",    CSL_APP_TEST_NOT_RUN },
    {NULL,             "TERMINATING CONDITION",    CSL_APP_TEST_NOT_RUN }
};

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


/*===========================================================================*/
/*                         Function definitions                              */
/*===========================================================================*/

void test_crc_test_app (void)
{
    /* Declarations of variables */
    int32_t    testResult = CSL_APP_TEST_PASS;
    int32_t    i;


    DebugP_log("\n crc Test Application\r\n");

    for ( i = 0; crcTestList[i].testFunction != NULL; i++)
    {
        testResult = crcTestList[i].testFunction();
        crcTestList[i].testStatus = testResult;
    }

    testResult = CSL_APP_TEST_PASS;
    for ( i = 0; crcTestList[i].testFunction != NULL; i++)
    {
        if (crcTestList[i].testStatus != CSL_APP_TEST_PASS)
        {
            DebugP_log("Test Name: %s  FAILED \r\n", crcTestList[i].name);
            testResult = CSL_APP_TEST_FAILED;
            break;
        }
        else
        {
            DebugP_log("Test Name: %s  PASSED \r\n", crcTestList[i].name);
        }
    }

    if (testResult == CSL_APP_TEST_PASS)
    {
        DebugP_log("\n All tests have passed. \r\n");
    }
    else
    {
        DebugP_log("\n Few/all tests Failed \r\n");
    }
#if defined (UNITY_INCLUDE_CONFIG_H)
    TEST_ASSERT_EQUAL_INT32(CSL_APP_TEST_PASS, testResult);
#endif
}

void test_crc_test_app_runner(void)
{
#if defined(UNITY_INCLUDE_CONFIG_H)
    UNITY_BEGIN();
    RUN_TEST (test_crc_test_app);
    UNITY_END();
    /* Function to print results defined in our unity_config.h file */
    print_unityOutputBuffer_usingUARTstdio();
#else
    test_crc_test_app();
#endif
    return;
}

void testcase_main(void)
{
    test_crc_test_app_runner();

}

/* Nothing past this point */
