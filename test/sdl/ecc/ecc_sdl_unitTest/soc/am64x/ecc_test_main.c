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
 *  \file     ecc_test_main.c
 *
 *  \brief    This file contains ECC test code.
 *
 *  \details  ECC unit tests
 **/

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */
#include "ecc_test_main.h"
#include <drivers/sciclient.h>
#include <sdl/dpl/sdl_dpl.h>
#include <sdl/sdl_ecc.h>
#include <dpl_interface.h>
#include <kernel/dpl/DebugP.h>
#include <unity.h>

/* ========================================================================== */
/*                                Macros                                      */
/* ========================================================================== */



/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */
   volatile bool   gMsmcMemParityInterrupt;
/* ========================================================================== */
/*                 Internal Function Declarations                             */
/* ========================================================================== */

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

/* ========================================================================== */
/*                 Internal Function Definitions                              */
/* ========================================================================== */

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

#if defined (SOC_AM64X)
#define AUX_NUM_DEVICES 40
int32_t status = SDL_PASS;
uint32_t aux_devices[AUX_NUM_DEVICES] =
{
  TISCI_DEV_MMCSD1,
  TISCI_DEV_COMPUTE_CLUSTER0,
  TISCI_DEV_ADC0,
  TISCI_DEV_SA2_UL0,
  TISCI_DEV_MCAN0,
  TISCI_DEV_DMASS0,
  TISCI_DEV_MCAN1,
  TISCI_DEV_PRU_ICSSG1,
  TISCI_DEV_PRU_ICSSG0,
  TISCI_DEV_FSS0_OSPI_0,
  TISCI_DEV_CPSW0,
  TISCI_DEV_GICSS0,
  TISCI_DEV_PCIE0,
  TISCI_DEV_USB0,
  TISCI_DEV_DMSC0,
  TISCI_DEV_MCU_M4FSS0,
  TISCI_DEV_MCU_M4FSS0_CORE0,
  TISCI_DEV_MMCSD0,
  TISCI_DEV_VTM0,
  TISCI_DEV_R5FSS0_CORE0,
  TISCI_DEV_R5FSS0_CORE1,
  TISCI_DEV_R5FSS1_CORE0,
  TISCI_DEV_R5FSS1_CORE1,
  TISCI_DEV_COMPUTE_CLUSTER0,
  TISCI_DEV_A53SS0,
  TISCI_DEV_A53SS0_CORE_0,
  TISCI_DEV_A53SS0_CORE_1
};
#endif

#if defined (SOC_AM243X)
#define AUX_NUM_DEVICES 40

int32_t status = SDL_PASS;
uint32_t aux_devices[AUX_NUM_DEVICES] =
{
  TISCI_DEV_MMCSD1,
  TISCI_DEV_COMPUTE_CLUSTER0,
  TISCI_DEV_ADC0,
  TISCI_DEV_SA2_UL0,
  TISCI_DEV_MCAN0,
  TISCI_DEV_DMASS0,
  TISCI_DEV_MCAN1,
  TISCI_DEV_PRU_ICSSG1,
  TISCI_DEV_PRU_ICSSG0,
  TISCI_DEV_FSS0_OSPI_0,
  TISCI_DEV_CPSW0,
  TISCI_DEV_GICSS0,
  TISCI_DEV_PCIE0,
  TISCI_DEV_USB0,
  TISCI_DEV_DMSC0,
  TISCI_DEV_MCU_M4FSS0,
  TISCI_DEV_MCU_M4FSS0_CORE0,
  TISCI_DEV_MMCSD0,
  TISCI_DEV_VTM0,
  TISCI_DEV_R5FSS0_CORE0,
  TISCI_DEV_R5FSS0_CORE1,
  TISCI_DEV_R5FSS1_CORE0,
  TISCI_DEV_R5FSS1_CORE1,
  TISCI_DEV_COMPUTE_CLUSTER0,
  TISCI_DEV_A53SS0,
  TISCI_DEV_A53SS0_CORE_0,
  TISCI_DEV_A53SS0_CORE_1
};
#endif
int32_t SDL_ESM_applicationCallbackFunction(SDL_ESM_Inst esmInst,
                                            SDL_ESM_IntType esmIntrType,
                                            uint32_t grpChannel,
                                            uint32_t index,
                                            uint32_t intSrc,
                                            uintptr_t *arg)
{

    SDL_ECC_MemType eccmemtype;
    SDL_Ecc_AggrIntrSrc eccIntrSrc;
    SDL_ECC_ErrorInfo_t eccErrorInfo;
    int32_t retVal;

    DebugP_log("\r\n  ESM Call back function called : instType 0x%x, intType 0x%x, " \
                "grpChannel 0x%x, index 0x%x, intSrc 0x%x \n",
                esmInst, esmIntrType, grpChannel, index, intSrc);
    DebugP_log("\r\n  Take action \n");
	gMsmcMemParityInterrupt = TRUE;
    retVal = SDL_ECC_getESMErrorInfo(esmInst, intSrc, &eccmemtype, &eccIntrSrc);

    if (retVal != SDL_PASS)
    {
        DebugP_log("\rSDL_ECC_getESMErrorInfo failed\n");
    }
    else
    {
        /* Any additional customer specific actions can be added here */
        retVal = SDL_ECC_getErrorInfo(eccmemtype, eccIntrSrc, &eccErrorInfo);
    }

    if (retVal == SDL_PASS)
    {
        DebugP_log("\r\n  ECC Error Call back function called : eccMemType %d, errorSrc 0x%x, " \
                   "ramId %d, bitErrorOffset 0x%04x%04x, bitErrorGroup %d\n",
                   eccmemtype, eccIntrSrc, eccErrorInfo.memSubType, (uint32_t)(eccErrorInfo.bitErrorOffset >> 32),
                   (uint32_t)(eccErrorInfo.bitErrorOffset & 0x00000000FFFFFFFF), eccErrorInfo.bitErrorGroup);

        if (eccErrorInfo.bitErrCnt != 0)
        {
            SDL_ECC_clearNIntrPending(eccmemtype, eccErrorInfo.memSubType, eccIntrSrc, SDL_ECC_AGGR_ERROR_SUBTYPE_NORMAL, eccErrorInfo.bitErrCnt);
        }
        SDL_ECC_clearNIntrPending(eccmemtype, eccErrorInfo.memSubType, eccIntrSrc, SDL_ECC_AGGR_ERROR_SUBTYPE_NORMAL, 0x1);

        retVal = SDL_ECC_ackIntr(eccmemtype, eccIntrSrc);
    }

    return retVal;
}

void SDL_ECC_applicationCallbackFunction(SDL_ECC_MemType eccMemType,
                                         uint32_t errorSrc,
                                         uint32_t address,
                                         uint32_t ramId,
                                         uint64_t bitErrorOffset,
                                         uint32_t bitErrorGroup){

    DebugP_log("\r\n  ECC Error Call back function called : eccMemType %d, errorSrc 0x%x, " \
                "address 0x%x, ramId %d, bitErrorOffset 0x%04x%04x, bitErrorGroup %d\n",
                eccMemType, errorSrc, address, ramId, (uint32_t)(bitErrorOffset >> 32),
                (uint32_t)(bitErrorOffset & 0x00000000FFFFFFFF), bitErrorGroup);
    DebugP_log("\r\n  Take action \n");

    /* Any additional customer specific actions can be added here */

}

static int32_t sdlApp_dplInit(void)
{
    SDL_ErrType_t ret = SDL_PASS;

    ret = SDL_TEST_dplInit();
    if (ret != SDL_PASS)
    {
        DebugP_log("\r\nError: Init Failed\n");
    }

    for (int i = 0; i < AUX_NUM_DEVICES; i++)
    {
        /* Power up RTI */
        status = Sciclient_pmSetModuleState(aux_devices[i],
                                            TISCI_MSG_VALUE_DEVICE_SW_STATE_ON,
                                            TISCI_MSG_FLAG_AOP,
                                            SystemP_WAIT_FOREVER);

        if (status != SDL_PASS)
        {
            DebugP_log("\r\n   Sciclient_pmSetModuleState 0x%x ...FAILED: retValue %d\n",
                        aux_devices[i], status);
        }
    }

    return ret;
}

static
int32_t ECC_appTest(uint32_t testId)
{
    int32_t    testResult;

    switch (testId)
    {
        case ECC_FUNC_TEST_ID:
            testResult = ECC_funcTest();
            DebugP_log("\r\n ECC Functionality Test");
            if (testResult == SDL_PASS)
            {
                DebugP_log("\r\n Passed.\r\n");
            }
            else
            {
                DebugP_log("\r\n Failed.\r\n");
            }
            break;

        case ECC_ERROR_TEST_ID:
            testResult = ECC_errTest();
            DebugP_log("\r\n ECC Error Module Test");
            if (testResult == SDL_PASS)
            {
                DebugP_log("\r\n Passed.\r\n");
            }
            else
            {
                DebugP_log("\r\n Failed.\r\n");
            }
            break;

        default:
            DebugP_log("\r\n [Error] Invalid ECC test ID.\r\n");
            testResult = SDL_EFAIL;
            break;
    }

    return (testResult);
}

void test_sdl_ecc_test_app(void *args)
{
    uint32_t testId;
    int32_t  testResult;

    for (testId = ((uint32_t)(0U)); testId < ECC_TOTAL_NUM_TESTS; testId++)
    {
        testResult = ECC_appTest(testId);
        if (testResult != SDL_PASS)
        {
            break;
        }
    }

    if (testResult == SDL_PASS)
    {
        DebugP_log("\r\nAll tests have passed. \r\n");
#ifdef UNITY_INCLUDE_CONFIG_H
        TEST_PASS();
#endif
    }
    else
    {
        DebugP_log("\r\nSome tests have failed. \r\n");
#ifdef UNITY_INCLUDE_CONFIG_H
        TEST_FAIL();
#endif
    }
}

void test_sdl_ecc_test_app_runner(void)
{
#ifdef UNITY_INCLUDE_CONFIG_H
    UNITY_BEGIN();
    RUN_TEST(test_sdl_ecc_test_app,0,NULL);
    UNITY_END();
#else
    test_sdl_ecc_test_app();
#endif
}

int32_t test_main(void)
{
        sdlApp_dplInit();

        test_sdl_ecc_test_app_runner();

    return (0);
}

/* Nothing past this point */
