/*
 * Copyright (C) 2024 Texas Instruments Incorporated
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *   Redistributions of source code must retain the above copyright
 *   notice, this list of conditions and the following disclaimer.
 *
 *   Redistributions in binary form must reproduce the above copyright
 *   notice, this list of conditions and the following disclaimer in the
 *   documentation and/or other materials provided with the
 *   distribution.
 *
 *   Neither the name of Texas Instruments Incorporated nor the names of
 *   its contributors may be used to endorse or promote products derived
 *   from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <stdio.h>
#include <string.h>
#include <inttypes.h>
#include <unity.h>
#include <drivers/gp_timer.h>
#include <kernel/dpl/DebugP.h>
#include <kernel/dpl/ClockP.h>
#include <kernel/dpl/AddrTranslateP.h>
#include "ti_drivers_config.h"
#include "ti_drivers_open_close.h"

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

extern GPTIMER_Config gGpTimerConfig[];

/* ========================================================================== */
/*                         Structures and Enums                               */
/* ========================================================================== */

/** \brief GPIO test params */
typedef struct Test_GptimerTestPrms_s
{
    /* Index of the global GPTIMER Config */
    uint32_t configIdx;

    /* Open Parameters */
    GPTIMER_Params  gptimerParams;

    /* Timer Config Mode Parameters */
    uint8_t                     timerConfigMode;
    GPTIMER_Compare_Config      compareConfig;
    GPTIMER_Capture_Config      captureConfig;
    GPTIMER_PWM_Config          pwmConfig;

    /* Timer interrupt related Parameters */
    bool                        intrEnable;

    GPTIMER_OverflowCallbackFxn         overflowCallbackFunction;
    GPTIMER_CompareMatchCallbackFxn     compareMatchCallbackFunction;
    GPTIMER_CaptureCallbackFxn          captureCallbackFunction;

} Test_GptimerTestPrms;

/* ========================================================================== */
/*                 Internal Function Declarations                             */
/* ========================================================================== */

/* Testcases */
static void test_gp_timer_free_run_no_interrupt(void *args);
static void test_gp_timer_free_run_callback_no_overflow_mask(void *args);
static void test_gp_timer_free_run_callback_overflow_mask(void *args);
static void test_gp_timer_compare_match_callback(void *args);
static void test_gp_timer_change_timer_config(void *args);

/* Helper functions */
static void test_gptimer_set_params(Test_GptimerTestPrms *testParams,
                                    int8_t setting_id);
static void test_gptimer_update_module_params(Test_GptimerTestPrms *testParams,
                                              GPTIMER_HwAttrs *hwAttrs,
                                              GPTIMER_Object *object);

/* Callback Functions */
void overflowCallback(GPTIMER_Handle handle);
void compareMatchCallback(GPTIMER_Handle handle);
/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */

Test_GptimerTestPrms testParams;
SemaphoreP_Object overflowSemObj;
SemaphoreP_Object compareMatchSemObj;
uint32_t counterVal;

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

void test_main(void *args)
{
    /* Open drivers to open the UART driver for console */
    Drivers_open();
    UNITY_BEGIN();

    RUN_TEST(test_gp_timer_free_run_no_interrupt, 13451, (void*)&testParams);
    RUN_TEST(test_gp_timer_free_run_callback_no_overflow_mask, 13452, (void*)&testParams);
    RUN_TEST(test_gp_timer_free_run_callback_overflow_mask, 13453, (void*)&testParams);
    RUN_TEST(test_gp_timer_compare_match_callback, 13454, (void*)&testParams);
    RUN_TEST(test_gp_timer_change_timer_config, 13455, (void*)&testParams);

    UNITY_END();
    Drivers_close();

    return;
}

/*
 * Unity framework required functions
 */
void setUp(void)
{
}

void tearDown(void)
{
}

/*
 * Testcases
 */
static void test_gp_timer_free_run_no_interrupt(void* args)
{
    Test_GptimerTestPrms    *testParams = (Test_GptimerTestPrms*)args;
    GPTIMER_Params          *gptimerParams = &(testParams->gptimerParams);
    GPTIMER_Handle          gpTimerHandle = NULL;
    GPTIMER_HwAttrs         *hwAttrs = NULL;
    GPTIMER_Object          *object = NULL;
    uint32_t                configIdx = testParams->configIdx;

    /* Store handle for Local use */
    gpTimerHandle = gGpTimerHandle[configIdx];
    /* Get the Attributes */
    hwAttrs = (GPTIMER_HwAttrs *) (gGpTimerConfig[configIdx].hwAttrs);
    /* Get the Object */
    object = (GPTIMER_Object *) (gGpTimerConfig[configIdx].object);

    /* Counter Free Run No Reload Disabled Prescaler ________________________ */
    /* Close the Instance */
    GPTIMER_close(gpTimerHandle);
    /* Change Handle Back to NULL*/
    gpTimerHandle = NULL;
    /* Update Test Params */
    test_gptimer_set_params(testParams, 0);
    /* Update the driver instance Parameters */
    test_gptimer_update_module_params(testParams, hwAttrs, object);

    /* Open the GPTIMER instance with updated Params */
    gpTimerHandle = GPTIMER_open(configIdx, gptimerParams);
    TEST_ASSERT_NOT_NULL(gpTimerHandle);

    /* Do The test */
    /* Set the Count  */
    GPTIMER_setCount(gpTimerHandle, (0xFFFFFFFFU - 0x61A8U + 1U));
    /* Start the TImer */
    GPTIMER_start(gpTimerHandle);
    /* Wait for overflow */
    while((GPTIMER_getIRQStatus(gpTimerHandle) &
                                            TIMER_IRQ_OVF_IT_FLAG_MASK) == 0)
    {
        ClockP_usleep(100U);
    }
    /* Clear Status */
    GPTIMER_clearIRQStatus(gpTimerHandle, TIMER_IRQ_OVF_IT_FLAG_MASK);
    ClockP_usleep(100U);
    /* Get Current Count */
    counterVal = GPTIMER_getCount(gpTimerHandle);
    TEST_ASSERT_EQUAL_UINT32(0U, counterVal);

    /* Counter Free Run Auto Reload Disabled Prescaler ______________________ */
    /* Close the Instance */
    GPTIMER_close(gpTimerHandle);
    /* Change Handle Back to NULL*/
    gpTimerHandle = NULL;
    /* Update Test Params */
    test_gptimer_set_params(testParams, 1);
    /* Update the driver instance Parameters */
    test_gptimer_update_module_params(testParams, hwAttrs, object);

    /* Open the GPTIMER instance with updated Params */
    gpTimerHandle = GPTIMER_open(configIdx, gptimerParams);
    TEST_ASSERT_NOT_NULL(gpTimerHandle);

    /* Do The test */
    /* Set the Count  */
    GPTIMER_setCount(gpTimerHandle, (0xFFFFFFFFU - 0x61A8U + 1U));
    /* Start the TImer */
    GPTIMER_start(gpTimerHandle);
    /* Wait for overflow */
    while((GPTIMER_getIRQStatus(gpTimerHandle) &
                                            TIMER_IRQ_OVF_IT_FLAG_MASK) == 0)
    {
        ClockP_usleep(100U);
    }
    /* Clear Status */
    GPTIMER_clearIRQStatus(gpTimerHandle, TIMER_IRQ_OVF_IT_FLAG_MASK);
    ClockP_usleep(100U);
    /* Get Current Count */
    counterVal = GPTIMER_getCount(gpTimerHandle);
    TEST_ASSERT_GREATER_OR_EQUAL_UINT32(1U, counterVal);

    /* Counter Free Run No Reload Enabled Prescaler _________________________ */
    /* Close the Instance */
    GPTIMER_close(gpTimerHandle);
    /* Change Handle Back to NULL*/
    gpTimerHandle = NULL;
    /* Update Test Params */
    test_gptimer_set_params(testParams, 2);
    /* Update the driver instance Parameters */
    test_gptimer_update_module_params(testParams, hwAttrs, object);

    /* Open the GPTIMER instance with updated Params */
    gpTimerHandle = GPTIMER_open(configIdx, gptimerParams);
    TEST_ASSERT_NOT_NULL(gpTimerHandle);

    /* Do The test */
    /* Set the Count  */
    GPTIMER_setCount(gpTimerHandle, (0xFFFFFFFFU - 0x61A8U + 1U));
    /* Start the TImer */
    GPTIMER_start(gpTimerHandle);
    /* Wait for overflow */
    while((GPTIMER_getIRQStatus(gpTimerHandle) &
                                            TIMER_IRQ_OVF_IT_FLAG_MASK) == 0)
    {
        ClockP_usleep(100U);
    }
    /* Clear Status */
    GPTIMER_clearIRQStatus(gpTimerHandle, TIMER_IRQ_OVF_IT_FLAG_MASK);
    ClockP_usleep(100U);
    /* Get Current Count */
    counterVal = GPTIMER_getCount(gpTimerHandle);
    TEST_ASSERT_EQUAL_UINT32(0U, counterVal);

    /* Counter Free Run Auto Reload Enabled Prescaler _______________________ */
    /* Close the Instance */
    GPTIMER_close(gpTimerHandle);
    /* Change Handle Back to NULL*/
    gpTimerHandle = NULL;
    /* Update Test Params */
    test_gptimer_set_params(testParams, 3);
    /* Update the driver instance Parameters */
    test_gptimer_update_module_params(testParams, hwAttrs, object);

    /* Open the GPTIMER instance with updated Params */
    gpTimerHandle = GPTIMER_open(configIdx, gptimerParams);
    TEST_ASSERT_NOT_NULL(gpTimerHandle);

    /* Do The test */
    /* Set the Count  */
    GPTIMER_setCount(gpTimerHandle, (0xFFFFFFFFU - 0x61A8U + 1U));
    /* Start the TImer */
    GPTIMER_start(gpTimerHandle);
    /* Wait for overflow */
    while((GPTIMER_getIRQStatus(gpTimerHandle) &
                                            TIMER_IRQ_OVF_IT_FLAG_MASK) == 0)
    {
        ClockP_usleep(100U);
    }
    /* Clear Status */
    GPTIMER_clearIRQStatus(gpTimerHandle, TIMER_IRQ_OVF_IT_FLAG_MASK);
    ClockP_usleep(100U);
    /* Get Current Count */
    counterVal = GPTIMER_getCount(gpTimerHandle);
    TEST_ASSERT_GREATER_OR_EQUAL_UINT32(1U, counterVal);
}

static void test_gp_timer_free_run_callback_no_overflow_mask(void* args)
{
    Test_GptimerTestPrms    *testParams = (Test_GptimerTestPrms*)args;
    GPTIMER_Params          *gptimerParams = &(testParams->gptimerParams);
    GPTIMER_Handle          gpTimerHandle = NULL;
    GPTIMER_HwAttrs         *hwAttrs = NULL;
    GPTIMER_Object          *object = NULL;
    uint32_t                configIdx = testParams->configIdx;

    /* Store handle for Local use */
    gpTimerHandle = gGpTimerHandle[configIdx];
    /* Get the Attributes */
    hwAttrs = (GPTIMER_HwAttrs *) (gGpTimerConfig[configIdx].hwAttrs);
    /* Get the Object */
    object = (GPTIMER_Object *) (gGpTimerConfig[configIdx].object);

    SemaphoreP_constructBinary(&overflowSemObj, 0);

    /* Counter Free Run No Reload Disabled Prescaler No Overflow Mask________ */
    /* Close the Instance */
    GPTIMER_close(gpTimerHandle);
    /* Change Handle Back to NULL*/
    gpTimerHandle = NULL;
    /* Update Test Params */
    test_gptimer_set_params(testParams, 4);
    /* Update the driver instance Parameters */
    test_gptimer_update_module_params(testParams, hwAttrs, object);

    /* Open the GPTIMER instance with updated Params */
    gpTimerHandle = GPTIMER_open(configIdx, gptimerParams);
    TEST_ASSERT_NOT_NULL(gpTimerHandle);

    /* Do The test */
    /* Set the Count  */
    GPTIMER_setCount(gpTimerHandle, (0xFFFFFFFFU - 0x61A8U  + 1U));
    /* Start the TImer */
    GPTIMER_start(gpTimerHandle);
    /* Pend Semaphore, Wait for Post in Callback */
    SemaphoreP_pend(&overflowSemObj, SystemP_WAIT_FOREVER);

    ClockP_usleep(100U);
    /* Get Current Count */
    counterVal = GPTIMER_getCount(gpTimerHandle);
    TEST_ASSERT_EQUAL_UINT32(0U, counterVal);

    /* Counter Free Run Auto Reload Disabled Prescaler No Overflow Mask _____ */
    /* Close the Instance */
    GPTIMER_close(gpTimerHandle);
    /* Change Handle Back to NULL*/
    gpTimerHandle = NULL;
    /* Update Test Params */
    test_gptimer_set_params(testParams, 5);
    /* Update the driver instance Parameters */
    test_gptimer_update_module_params(testParams, hwAttrs, object);

    /* Open the GPTIMER instance with updated Params */
    gpTimerHandle = GPTIMER_open(configIdx, gptimerParams);
    TEST_ASSERT_NOT_NULL(gpTimerHandle);

    /* Do The test */
    /* Set the Count  */
    GPTIMER_setCount(gpTimerHandle, (0xFFFFFFFFU - 0x61A8U + 1U));
    /* Start the TImer */
    GPTIMER_start(gpTimerHandle);
    /* Pend Semaphore, Wait for Post in Callback */
    SemaphoreP_pend(&overflowSemObj, SystemP_WAIT_FOREVER);

    ClockP_usleep(100U);
    /* Get Current Count */
    counterVal = GPTIMER_getCount(gpTimerHandle);
    TEST_ASSERT_GREATER_OR_EQUAL_UINT32(2500U, counterVal);

    /* Counter Free Run No Reload Enabled Prescaler No Overflow Mask ________ */
    /* Close the Instance */
    GPTIMER_close(gpTimerHandle);
    /* Change Handle Back to NULL*/
    gpTimerHandle = NULL;
    /* Update Test Params */
    test_gptimer_set_params(testParams, 6);
    /* Update the driver instance Parameters */
    test_gptimer_update_module_params(testParams, hwAttrs, object);

    /* Open the GPTIMER instance with updated Params */
    gpTimerHandle = GPTIMER_open(configIdx, gptimerParams);
    TEST_ASSERT_NOT_NULL(gpTimerHandle);

    /* Do The test */
    /* Set the Count  */
    GPTIMER_setCount(gpTimerHandle, (0xFFFFFFFFU - 0x61A8U + 1U));
    /* Start the TImer */
    GPTIMER_start(gpTimerHandle);
    /* Pend Semaphore, Wait for Post in Callback */
    SemaphoreP_pend(&overflowSemObj, SystemP_WAIT_FOREVER);

    ClockP_usleep(200U);
    /* Get Current Count */
    counterVal = GPTIMER_getCount(gpTimerHandle);
    TEST_ASSERT_EQUAL_UINT32(0U, counterVal);

    /* Counter Free Run Auto Reload Enabled Prescaler No Overflow Mask ______ */
    /* Close the Instance */
    GPTIMER_close(gpTimerHandle);
    /* Change Handle Back to NULL*/
    gpTimerHandle = NULL;
    /* Update Test Params */
    test_gptimer_set_params(testParams, 7);
    /* Update the driver instance Parameters */
    test_gptimer_update_module_params(testParams, hwAttrs, object);

    /* Open the GPTIMER instance with updated Params */
    gpTimerHandle = GPTIMER_open(configIdx, gptimerParams);
    TEST_ASSERT_NOT_NULL(gpTimerHandle);

    /* Do The test */
    /* Set the Count  */
    GPTIMER_setCount(gpTimerHandle, (0xFFFFFFFFU - 0x61A8U + 1U));
    /* Start the TImer */
    GPTIMER_start(gpTimerHandle);
    /* Pend Semaphore, Wait for Post in Callback */
    SemaphoreP_pend(&overflowSemObj, SystemP_WAIT_FOREVER);

    ClockP_usleep(400U);
    /* Get Current Count */
    counterVal = GPTIMER_getCount(gpTimerHandle);
    TEST_ASSERT_GREATER_OR_EQUAL_UINT32(2500U, counterVal);

    SemaphoreP_destruct(&overflowSemObj);
}

static void test_gp_timer_free_run_callback_overflow_mask(void* args)
{
    Test_GptimerTestPrms    *testParams = (Test_GptimerTestPrms*)args;
    GPTIMER_Params          *gptimerParams = &(testParams->gptimerParams);
    GPTIMER_Handle          gpTimerHandle = NULL;
    GPTIMER_HwAttrs         *hwAttrs = NULL;
    GPTIMER_Object          *object = NULL;
    uint32_t                configIdx = testParams->configIdx;
    uint32_t                ticks = 0U;

    /* Store handle for Local use */
    gpTimerHandle = gGpTimerHandle[configIdx];
    /* Get the Attributes */
    hwAttrs = (GPTIMER_HwAttrs *) (gGpTimerConfig[configIdx].hwAttrs);
    /* Get the Object */
    object = (GPTIMER_Object *) (gGpTimerConfig[configIdx].object);

    SemaphoreP_constructBinary(&overflowSemObj, 0);

    /* Counter Free Run Auto Reload Enabled Prescaler Interrupt Callback
       Overflow Mask ________________________________________________________ */
    /* Close the Instance */
    GPTIMER_close(gpTimerHandle);
    /* Change Handle Back to NULL*/
    gpTimerHandle = NULL;
    /* Update Test Params */
    test_gptimer_set_params(testParams, 8);
    /* Update the driver instance Parameters */
    test_gptimer_update_module_params(testParams, hwAttrs, object);

    /* Open the GPTIMER instance with updated Params */
    gpTimerHandle = GPTIMER_open(configIdx, gptimerParams);
    TEST_ASSERT_NOT_NULL(gpTimerHandle);

    /* Do The test */
    /* Set the Count  */
    GPTIMER_setCount(gpTimerHandle, (0xFFFFFFFFU -  0x17D7840 + 1U));
    /* Get Current Ticks */
    ticks = ClockP_getTicks();
    /* Start the TImer */
    GPTIMER_start(gpTimerHandle);
    /* Pend Semaphore, Wait for Post in Callback */
    SemaphoreP_pend(&overflowSemObj, SystemP_WAIT_FOREVER);
    /* Get ticks passed */
    ticks = ClockP_getTicks() - ticks;
    /* 2 Overflow interrupts are masked */
    TEST_ASSERT_GREATER_OR_EQUAL_UINT32(3000000U, ClockP_ticksToUsec(ticks));

    SemaphoreP_destruct(&overflowSemObj);
}

static void test_gp_timer_compare_match_callback(void *args)
{
    Test_GptimerTestPrms    *testParams = (Test_GptimerTestPrms*)args;
    GPTIMER_Params          *gptimerParams = &(testParams->gptimerParams);
    GPTIMER_Handle          gpTimerHandle = NULL;
    GPTIMER_HwAttrs         *hwAttrs = NULL;
    GPTIMER_Object          *object = NULL;
    uint32_t                configIdx = testParams->configIdx;

    /* Store handle for Local use */
    gpTimerHandle = gGpTimerHandle[configIdx];
    /* Get the Attributes */
    hwAttrs = (GPTIMER_HwAttrs *) (gGpTimerConfig[configIdx].hwAttrs);
    /* Get the Object */
    object = (GPTIMER_Object *) (gGpTimerConfig[configIdx].object);

    SemaphoreP_constructBinary(&compareMatchSemObj, 0);

    /* Close the Instance */
    GPTIMER_close(gpTimerHandle);
    /* Change Handle Back to NULL*/
    gpTimerHandle = NULL;
    /* Update Test Params */
    test_gptimer_set_params(testParams, 9);
    /* Update the driver instance Parameters */
    test_gptimer_update_module_params(testParams, hwAttrs, object);

    /* Open the GPTIMER instance with updated Params */
    gpTimerHandle = GPTIMER_open(configIdx, gptimerParams);
    TEST_ASSERT_NOT_NULL(gpTimerHandle);

    /* Do The test */
    /* Start the TImer */
    GPTIMER_start(gpTimerHandle);
    /* Pend Semaphore, Wait for Post in Callback */
    SemaphoreP_pend(&compareMatchSemObj, SystemP_WAIT_FOREVER);
    /* Get Current Count */
    counterVal = GPTIMER_getCount(gpTimerHandle);
    TEST_ASSERT_GREATER_OR_EQUAL_UINT32(25000000U, counterVal);

    SemaphoreP_destruct(&compareMatchSemObj);
}

static void test_gp_timer_change_timer_config(void *args)
{
    Test_GptimerTestPrms    *testParams = (Test_GptimerTestPrms*)args;
    GPTIMER_Params          *gptimerParams = &(testParams->gptimerParams);
    GPTIMER_Handle          gpTimerHandle = NULL;
    GPTIMER_HwAttrs         *hwAttrs = NULL;
    GPTIMER_Object          *object = NULL;
    uint32_t                configIdx = testParams->configIdx;

    uint32_t                conifgMode;
    GPTIMER_Compare_Config  compareConfig;


    /* Store handle for Local use */
    gpTimerHandle = gGpTimerHandle[configIdx];
    /* Get the Attributes */
    hwAttrs = (GPTIMER_HwAttrs *) (gGpTimerConfig[configIdx].hwAttrs);
    /* Get the Object */
    object = (GPTIMER_Object *) (gGpTimerConfig[configIdx].object);

    SemaphoreP_constructBinary(&overflowSemObj, 0);
    SemaphoreP_constructBinary(&compareMatchSemObj, 0);

    /* Close the Instance */
    GPTIMER_close(gpTimerHandle);
    /* Change Handle Back to NULL*/
    gpTimerHandle = NULL;
    /* Update Test Params */
    test_gptimer_set_params(testParams, 10);
    /* Update the driver instance Parameters */
    test_gptimer_update_module_params(testParams, hwAttrs, object);

    /* Open the GPTIMER instance with updated Params */
    gpTimerHandle = GPTIMER_open(configIdx, gptimerParams);
    TEST_ASSERT_NOT_NULL(gpTimerHandle);

    /* Do The test */
    /* Set the Count  */
    GPTIMER_setCount(gpTimerHandle, (0xFFFFFFFFU - 0x61A8U + 1U));
    /* Start the TImer */
    GPTIMER_start(gpTimerHandle);
    /* Pend Semaphore, Wait for Post in Callback */
    SemaphoreP_pend(&overflowSemObj, SystemP_WAIT_FOREVER);
    /* Get Current Count */
    counterVal = GPTIMER_getCount(gpTimerHandle);
    TEST_ASSERT_GREATER_OR_EQUAL_UINT32(0xFFFF9E57U, counterVal);

    /* Update to Compare Config */
    conifgMode = GPTIMER_MODE_CONFIG_OUTPUT_COMPARE;
    compareConfig.cntCompareValComp = (0x017D7840U);

    GPTIMER_setTimerConfigMode(gpTimerHandle, conifgMode,
                               (void *)(&compareConfig));
    /* Add Callback */
    GPTIMER_setCallbackFxn(gpTimerHandle, NULL, compareMatchCallback, NULL);

    /* Set the Count  */
    /* Start the TImer */
    GPTIMER_start(gpTimerHandle);
    /* Pend Semaphore, Wait for Post in Callback */
    SemaphoreP_pend(&compareMatchSemObj, SystemP_WAIT_FOREVER);
    /* Get Current Count */
    counterVal = GPTIMER_getCount(gpTimerHandle);
    TEST_ASSERT_GREATER_OR_EQUAL_UINT32(25000000U, counterVal);

    SemaphoreP_destruct(&overflowSemObj);
    SemaphoreP_destruct(&compareMatchSemObj);
}

/*
 * Helper functions
 */
static void test_gptimer_set_params(Test_GptimerTestPrms *testParams,
                                    int8_t setting_id)
{

    switch (setting_id)
    {
        case 0:
        {
            testParams->configIdx = 0;

            testParams->gptimerParams.enablePrescaler = false;
            testParams->gptimerParams.cntPrescaler = 0U;
            testParams->gptimerParams.oneShotMode = true;
            testParams->gptimerParams.cntReloadVal = 0U;
            testParams->gptimerParams.overflowMaskCount = 0U;

            testParams->timerConfigMode = GPTIMER_MODE_CONFIG_FREE_RUN;
            testParams->compareConfig.cntCompareValComp = 0U;
            testParams->intrEnable = false;
            testParams->overflowCallbackFunction = NULL;
            testParams->compareMatchCallbackFunction = NULL;
            testParams->captureCallbackFunction = NULL;
        }
        break;

        case 1:
        {
            testParams->configIdx = 0;

            testParams->gptimerParams.enablePrescaler = false;
            testParams->gptimerParams.cntPrescaler = 0U;
            testParams->gptimerParams.oneShotMode = false;
            testParams->gptimerParams.cntReloadVal = 0U;
            testParams->gptimerParams.overflowMaskCount = 0U;

            testParams->timerConfigMode = GPTIMER_MODE_CONFIG_FREE_RUN;
            testParams->compareConfig.cntCompareValComp = 0U;
            testParams->intrEnable = false;
            testParams->overflowCallbackFunction = NULL;
            testParams->compareMatchCallbackFunction = NULL;
            testParams->captureCallbackFunction = NULL;
        }
        break;

        case 2:
        {
            testParams->configIdx = 0;

            testParams->gptimerParams.enablePrescaler = true;
            testParams->gptimerParams.cntPrescaler = 0U;
            testParams->gptimerParams.oneShotMode = true;
            testParams->gptimerParams.cntReloadVal = 0U;
            testParams->gptimerParams.overflowMaskCount = 0U;

            testParams->timerConfigMode = GPTIMER_MODE_CONFIG_FREE_RUN;
            testParams->compareConfig.cntCompareValComp = 0U;
            testParams->intrEnable = false;
            testParams->overflowCallbackFunction = NULL;
            testParams->compareMatchCallbackFunction = NULL;
            testParams->captureCallbackFunction = NULL;
        }
        break;

        case 3:
        {
            testParams->configIdx = 0;

            testParams->gptimerParams.enablePrescaler = true;
            testParams->gptimerParams.cntPrescaler = 1U;
            testParams->gptimerParams.oneShotMode = false;
            testParams->gptimerParams.cntReloadVal = 0U;
            testParams->gptimerParams.overflowMaskCount = 0U;

            testParams->timerConfigMode = GPTIMER_MODE_CONFIG_FREE_RUN;
            testParams->compareConfig.cntCompareValComp = 0U;
            testParams->intrEnable = false;
            testParams->overflowCallbackFunction = NULL;
            testParams->compareMatchCallbackFunction = NULL;
            testParams->captureCallbackFunction = NULL;
        }
        break;

        case 4:
        {
            testParams->configIdx = 0;

            testParams->gptimerParams.enablePrescaler = false;
            testParams->gptimerParams.cntPrescaler = 0U;
            testParams->gptimerParams.oneShotMode = true;
            testParams->gptimerParams.cntReloadVal = 0U;
            testParams->gptimerParams.overflowMaskCount = 0U;

            testParams->timerConfigMode = GPTIMER_MODE_CONFIG_FREE_RUN;
            testParams->compareConfig.cntCompareValComp = 0U;
            testParams->intrEnable = true;
            testParams->overflowCallbackFunction = overflowCallback;
            testParams->compareMatchCallbackFunction = NULL;
            testParams->captureCallbackFunction = NULL;
        }
        break;

        case 5:
        {
            testParams->configIdx = 0;

            testParams->gptimerParams.enablePrescaler = false;
            testParams->gptimerParams.cntPrescaler = 0U;
            testParams->gptimerParams.oneShotMode = false;
            testParams->gptimerParams.cntReloadVal = 0U;
            testParams->gptimerParams.overflowMaskCount = 0U;

            testParams->timerConfigMode = GPTIMER_MODE_CONFIG_FREE_RUN;
            testParams->compareConfig.cntCompareValComp = 0U;
            testParams->intrEnable = true;
            testParams->overflowCallbackFunction = overflowCallback;
            testParams->compareMatchCallbackFunction = NULL;
            testParams->captureCallbackFunction = NULL;
        }
        break;

        case 6:
        {
            testParams->configIdx = 0;

            testParams->gptimerParams.enablePrescaler = true;
            testParams->gptimerParams.cntPrescaler = 0U;
            testParams->gptimerParams.oneShotMode = true;
            testParams->gptimerParams.cntReloadVal = 0U;
            testParams->gptimerParams.overflowMaskCount = 0U;

            testParams->timerConfigMode = GPTIMER_MODE_CONFIG_FREE_RUN;
            testParams->compareConfig.cntCompareValComp = 0U;
            testParams->intrEnable = true;
            testParams->overflowCallbackFunction = overflowCallback;
            testParams->compareMatchCallbackFunction = NULL;
            testParams->captureCallbackFunction = NULL;
        }
        break;

        case 7:
        {
            testParams->configIdx = 0;

            testParams->gptimerParams.enablePrescaler = true;
            testParams->gptimerParams.cntPrescaler = 1U;
            testParams->gptimerParams.oneShotMode = false;
            testParams->gptimerParams.cntReloadVal = 0U;
            testParams->gptimerParams.overflowMaskCount = 0U;

            testParams->timerConfigMode = GPTIMER_MODE_CONFIG_FREE_RUN;
            testParams->compareConfig.cntCompareValComp = 0U;
            testParams->intrEnable = true;
            testParams->overflowCallbackFunction = overflowCallback;
            testParams->compareMatchCallbackFunction = NULL;
            testParams->captureCallbackFunction = NULL;
        }
        break;

        case 8:
        {
            testParams->configIdx = 0;

            testParams->gptimerParams.enablePrescaler = false;
            testParams->gptimerParams.cntPrescaler = 0U;
            testParams->gptimerParams.oneShotMode = false;
            testParams->gptimerParams.cntReloadVal = (0xFFFFFFFFU -  0x17D7840 + 1U);
            testParams->gptimerParams.overflowMaskCount = 2U;

            testParams->timerConfigMode = GPTIMER_MODE_CONFIG_FREE_RUN;
            testParams->compareConfig.cntCompareValComp = 0U;
            testParams->intrEnable = true;
            testParams->overflowCallbackFunction = overflowCallback;
            testParams->compareMatchCallbackFunction = NULL;
            testParams->captureCallbackFunction = NULL;
        }
        break;

        case 9:
        {
            testParams->configIdx = 0;

            testParams->gptimerParams.enablePrescaler = false;
            testParams->gptimerParams.cntPrescaler = 0U;
            testParams->gptimerParams.oneShotMode = true;
            testParams->gptimerParams.cntReloadVal = 0U;
            testParams->gptimerParams.overflowMaskCount = 0U;

            testParams->timerConfigMode = GPTIMER_MODE_CONFIG_OUTPUT_COMPARE;
            testParams->compareConfig.cntCompareValComp = 0x017D7840U;
            testParams->intrEnable = true;
            testParams->overflowCallbackFunction = NULL;
            testParams->compareMatchCallbackFunction = compareMatchCallback;
            testParams->captureCallbackFunction = NULL;
        }
        break;

        case 10:
        {
            testParams->configIdx = 0;

            testParams->gptimerParams.enablePrescaler = false;
            testParams->gptimerParams.cntPrescaler = 0U;
            testParams->gptimerParams.oneShotMode = false;
            testParams->gptimerParams.cntReloadVal = (0xFFFFFFFFU - 0x61A8U + 1U);
            testParams->gptimerParams.overflowMaskCount = 0U;

            testParams->timerConfigMode = GPTIMER_MODE_CONFIG_FREE_RUN;
            testParams->compareConfig.cntCompareValComp = 0U;
            testParams->intrEnable = true;
            testParams->overflowCallbackFunction = overflowCallback;
            testParams->compareMatchCallbackFunction = NULL;
            testParams->captureCallbackFunction = NULL;
        }
        break;

        default:
        break;
    }
}

static void test_gptimer_update_module_params(Test_GptimerTestPrms *testParams,
                                              GPTIMER_HwAttrs *hwAttrs,
                                              GPTIMER_Object *object)
{
    /* Update interrupt setting */
    hwAttrs->enableIntr = testParams->intrEnable;
    /* Update Object Paramters */
    object->timerConfigMode = testParams->timerConfigMode;
    object->compareConfig = testParams->compareConfig;
    object->captureConfig = testParams->captureConfig;
    object->pwmConfig = testParams->pwmConfig;

    object->overflowCallbackFunction = testParams->overflowCallbackFunction;
    object->compareMatchCallbackFunction =
                                    testParams->compareMatchCallbackFunction;
    object->captureCallbackFunction = testParams->captureCallbackFunction;
}

void overflowCallback(GPTIMER_Handle handle)
{
    /* Post Semaphore */
    SemaphoreP_post(&overflowSemObj);
}

void compareMatchCallback(GPTIMER_Handle handle)
{
    /* Post Semaphore */
    SemaphoreP_post(&compareMatchSemObj);
}
