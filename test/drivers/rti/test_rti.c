/*
 *  Copyright (C) 2024 Texas Instruments Incorporated
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
 */

#include <drivers/gpio.h>
#include <string.h>
#include <kernel/dpl/DebugP.h>
#include <unity.h>
#include "ti_drivers_config.h"
#include "ti_drivers_open_close.h"
#include "ti_board_open_close.h"
#include <drivers/hw_include/soc_config.h>

#define LED_ON           (0x01U)
#define LED_OFF          (0x00U)
#define LED_BLINK_COUNT  (5U)

typedef struct RTI_TestParams_s {

    uint32_t counterBlkOutputClkFreq;
    uint8_t compareEvent;
    uint8_t interruptNum;
    uint8_t interruptFlag;
    uint16_t dmaInterruptFlag;
    uint8_t counterBlock;
    bool compIntrEnable;
    bool enableHalt;
    uint8_t enableContinueOnSuspend;

} RTI_TestParams;

HwiP_Object gRtiEvent0HwiObj[RTI_NUM_INSTANCES];
void rtiEvent0(void);

void RTI0_event0Isr(void *args)
{
    RTI_TestParams *testParams = (RTI_TestParams*)args;
    rtiEvent0();
    RTI_intStatusClear(CONFIG_RTI0_BASE_ADDR, testParams->interruptFlag);
    HwiP_clearInt(testParams->interruptNum);
}

/*
 * This example configures a GPIO pin connected to an LED on the EVM in
 * output mode.
 * The application toggles the LED on/off using RTI timer.
 */

volatile uint32_t gLedState, gBlinkCount;
uint32_t gpioBaseAddr, pinNum;

static void test_rti_set_params(RTI_TestParams *testParams, uint32_t testCaseId);
static void test_rti_params_init(RTI_TestParams *prms);

static void rti_enable_counter_block(void *args);
static void rti_enable_compare_event(void *args);
static void rti_test_cos_bit(void *args);

/**
 *  \brief  Function to initialize the #RTI_Params struct to its defaults
 *
 *  \param  trans       Pointer to #RTI_Params structure for
 *                      initialization
 */
static void test_rti_params_init(RTI_TestParams *prms)
{
    if(prms != NULL)
    {
        prms->counterBlkOutputClkFreq   = 1000000u;
        prms->compareEvent              = RTI_TMR_CMP_BLK_INDEX_0;
        prms->counterBlock              = RTI_TMR_CNT_BLK_INDEX_0;
        /* RTI Interrupt configurations */
        prms->compIntrEnable                = TRUE;
        prms->interruptNum              = CONFIG_RTI0_INT_NUM_EVENT0;
        prms->interruptFlag             = RTI_TMR_INT_INT0_FLAG;
        prms->dmaInterruptFlag          = RTI_TMR_INT_DMA0_FLAG;
        /* RTI COS bit test */
        prms->enableHalt                = FALSE;
        prms->enableContinueOnSuspend   = RTI_GC_STALL_MODE_OFF;
    }
}

void test_main(void *args)
{
    RTI_TestParams      testParams;

    Drivers_open();
    Board_driversOpen();

    UNITY_BEGIN();

    test_rti_set_params(&testParams, 13416);
    RUN_TEST(rti_enable_counter_block, 13416, (void*)&testParams);

    test_rti_set_params(&testParams, 13417);
    RUN_TEST(rti_enable_counter_block, 13417, (void*)&testParams);

    test_rti_set_params(&testParams, 13418);
    RUN_TEST(rti_enable_compare_event, 13418, (void*)&testParams);

    test_rti_set_params(&testParams, 13419);
    RUN_TEST(rti_enable_compare_event, 13419, (void*)&testParams);

    test_rti_set_params(&testParams, 13420);
    RUN_TEST(rti_enable_compare_event, 13420, (void*)&testParams);

    test_rti_set_params(&testParams, 13421);
    RUN_TEST(rti_enable_compare_event, 13421, (void*)&testParams);

    test_rti_set_params(&testParams, 13415);
    RUN_TEST(rti_test_cos_bit, 13415, (void*)&testParams);

    test_rti_set_params(&testParams, 13423);
    RUN_TEST(rti_test_cos_bit, 13423, (void*)&testParams);

    UNITY_END();

    Board_driversClose();
    Drivers_close();
}

static void rti_enable_counter_block(void *args)
{
    RTI_TestParams *testParams = (RTI_TestParams*)args;
    HwiP_Params rtiHwiParams;
    uint32_t cntrPrescaler;
    uint64_t timeInUsec;
    uint32_t compPrescaler;
    uint32_t status;

    /* Get address after translation translate */
    gpioBaseAddr = (uint32_t) AddrTranslateP_getLocalAddr(GPIO_LED_BASE_ADDR);
    pinNum       = GPIO_LED_PIN;
    gLedState = LED_ON;
    gBlinkCount = 0;

    /* Set LED GPIO pin in output mode */
    GPIO_setDirMode(gpioBaseAddr, pinNum, GPIO_LED_DIR);
    /* Set LED GPIO pin HIGH */
    GPIO_pinWriteHigh(gpioBaseAddr, pinNum);

    cntrPrescaler = (CONFIG_RTI0_INPUT_CLK_HZ/testParams->counterBlkOutputClkFreq) - 1;
    RTI_counterConfigure(CONFIG_RTI0_BASE_ADDR, testParams->counterBlock, RTI_TMR_CLK_SRC_COUNTER, RTI_TMR_NTU_0, cntrPrescaler);
    RTI_captureConfig(CONFIG_RTI0_BASE_ADDR, testParams->counterBlock, RTI_TMR_CAPTURE_EVT_0);

     /* Configure Compare event 0 */
    timeInUsec = 500000u;

    compPrescaler = (timeInUsec*testParams->counterBlkOutputClkFreq)/1000000;

    RTI_compareEventConfig(CONFIG_RTI0_BASE_ADDR, RTI_TMR_CMP_BLK_INDEX_0, testParams->counterBlock, compPrescaler, compPrescaler);
    RTI_intStatusClear(CONFIG_RTI0_BASE_ADDR, RTI_TMR_INT_INT0_FLAG);
    HwiP_Params_init(&rtiHwiParams);
    rtiHwiParams.intNum = CONFIG_RTI0_INT_NUM_EVENT0;
    rtiHwiParams.callback = RTI0_event0Isr;
    rtiHwiParams.isPulse = 0;
    rtiHwiParams.priority = 4;
    rtiHwiParams.args = testParams;
    status = HwiP_construct(&gRtiEvent0HwiObj[CONFIG_RTI0], &rtiHwiParams);
    DebugP_assertNoLog(status==SystemP_SUCCESS);

    RTI_intEnable(CONFIG_RTI0_BASE_ADDR, RTI_TMR_INT_INT0_FLAG);

    RTI_intDisable(CONFIG_RTI0_BASE_ADDR, RTI_TMR_INT_DMA0_FLAG);

    /* Start the RTI counter */
    (void)RTI_counterEnable(CONFIG_RTI0_BASE_ADDR, testParams->counterBlock);

    DebugP_log("Timer Started, Observe the LED blink for every 0.5 sec...!\r\n");

    /* Wait until the LED is blinked specified number of times */
    while(gBlinkCount < LED_BLINK_COUNT);

    /* Stop the RTI counter */
    (void)RTI_counterDisable(CONFIG_RTI0_BASE_ADDR, testParams->counterBlock);

    DebugP_log("Timer Stopped...\r\n");

    return;
}

static void rti_enable_compare_event(void *args)
{
    RTI_TestParams  *testParams = (RTI_TestParams*)args;
    HwiP_Params rtiHwiParams;
    uint32_t cntrPrescaler;
    uint64_t timeInUsec;
    uint32_t compPrescaler;
    uint32_t status;

    /* Get address after translation translate */
    gpioBaseAddr = (uint32_t) AddrTranslateP_getLocalAddr(GPIO_LED_BASE_ADDR);
    pinNum       = GPIO_LED_PIN;
    gLedState = LED_ON;
    gBlinkCount = 0;

    /* Set LED GPIO pin in output mode */
    GPIO_setDirMode(gpioBaseAddr, pinNum, GPIO_LED_DIR);
    /* Set LED GPIO pin HIGH */
    GPIO_pinWriteHigh(gpioBaseAddr, pinNum);

    cntrPrescaler = (CONFIG_RTI0_INPUT_CLK_HZ/testParams->counterBlkOutputClkFreq) - 1;
    RTI_counterConfigure(CONFIG_RTI0_BASE_ADDR, RTI_TMR_CNT_BLK_INDEX_0, RTI_TMR_CLK_SRC_COUNTER, RTI_TMR_NTU_0, cntrPrescaler);
    RTI_captureConfig(CONFIG_RTI0_BASE_ADDR, RTI_TMR_CNT_BLK_INDEX_0, RTI_TMR_CAPTURE_EVT_0);

     /* Configure Compare event 0 */
    timeInUsec = 500000u;

    compPrescaler = (timeInUsec*testParams->counterBlkOutputClkFreq)/1000000;

    RTI_compareEventConfig(CONFIG_RTI0_BASE_ADDR, testParams->compareEvent, RTI_TMR_CNT_BLK_INDEX_0, compPrescaler, compPrescaler);
    RTI_intStatusClear(CONFIG_RTI0_BASE_ADDR, testParams->interruptFlag);
    HwiP_Params_init(&rtiHwiParams);
    rtiHwiParams.intNum = testParams->interruptNum;
    rtiHwiParams.callback = RTI0_event0Isr;
    rtiHwiParams.isPulse = 0;
    rtiHwiParams.priority = 4;
    rtiHwiParams.args = testParams;
    status = HwiP_construct(&gRtiEvent0HwiObj[CONFIG_RTI0], &rtiHwiParams);
    DebugP_assertNoLog(status==SystemP_SUCCESS);

    RTI_intEnable(CONFIG_RTI0_BASE_ADDR, testParams->interruptFlag);

    RTI_intDisable(CONFIG_RTI0_BASE_ADDR, testParams->dmaInterruptFlag);

    /* Start the RTI counter */
    (void)RTI_counterEnable(CONFIG_RTI0_BASE_ADDR, RTI_TMR_CNT_BLK_INDEX_0);

    DebugP_log("Timer Started, Observe the LED blink for every 0.5 sec...!\r\n");

    /* Wait until the LED is blinked specified number of times */
    while(gBlinkCount < LED_BLINK_COUNT);

    /* Stop the RTI counter */
    (void)RTI_counterDisable(CONFIG_RTI0_BASE_ADDR, RTI_TMR_CNT_BLK_INDEX_0);

    DebugP_log("Timer Stopped...\r\n");

    return;
}

static void rti_test_cos_bit(void *args)
{
    RTI_TestParams *testParams = (RTI_TestParams*)args;
    HwiP_Params rtiHwiParams;
    uint32_t cntrPrescaler;
    uint64_t timeInUsec;
    uint32_t compPrescaler;
    uint32_t status;
    char testPass = 'n';

    /* Get address after translation translate */
    gpioBaseAddr = (uint32_t) AddrTranslateP_getLocalAddr(GPIO_LED_BASE_ADDR);
    pinNum       = GPIO_LED_PIN;
    gLedState = LED_ON;
    gBlinkCount = 0;

    /* Set LED GPIO pin in output mode */
    GPIO_setDirMode(gpioBaseAddr, pinNum, GPIO_LED_DIR);
    /* Set LED GPIO pin HIGH */
    GPIO_pinWriteHigh(gpioBaseAddr, pinNum);

    cntrPrescaler = (CONFIG_RTI0_INPUT_CLK_HZ/testParams->counterBlkOutputClkFreq) - 1;
    RTI_counterConfigure(CONFIG_RTI0_BASE_ADDR, testParams->counterBlock, RTI_TMR_CLK_SRC_COUNTER, RTI_TMR_NTU_0, cntrPrescaler);
    RTI_captureConfig(CONFIG_RTI0_BASE_ADDR, testParams->counterBlock, RTI_TMR_CAPTURE_EVT_0);

     /* Configure Compare event 0 */
    timeInUsec = 500000u;

    compPrescaler = (timeInUsec*testParams->counterBlkOutputClkFreq)/1000000;

    RTI_compareEventConfig(CONFIG_RTI0_BASE_ADDR, RTI_TMR_CMP_BLK_INDEX_0, testParams->counterBlock, compPrescaler, compPrescaler);
    RTI_intStatusClear(CONFIG_RTI0_BASE_ADDR, RTI_TMR_INT_INT0_FLAG);
    HwiP_Params_init(&rtiHwiParams);
    rtiHwiParams.intNum = CONFIG_RTI0_INT_NUM_EVENT0;
    rtiHwiParams.callback = RTI0_event0Isr;
    rtiHwiParams.isPulse = 0;
    rtiHwiParams.priority = 4;
    rtiHwiParams.args = testParams;
    status = HwiP_construct(&gRtiEvent0HwiObj[CONFIG_RTI0], &rtiHwiParams);
    DebugP_assertNoLog(status==SystemP_SUCCESS);

    RTI_intEnable(CONFIG_RTI0_BASE_ADDR, RTI_TMR_INT_INT0_FLAG);

    RTI_intDisable(CONFIG_RTI0_BASE_ADDR, RTI_TMR_INT_DMA0_FLAG);

    HW_WR_FIELD32(CSL_MSS_CTRL_U_BASE + CSL_MSS_CTRL_RTI1_HALTEN, CSL_MSS_CTRL_RTI1_HALTEN_CR5A0_HALTEN, testParams->enableHalt);

    /* Enable/Disable Continue on Suspend */
    RTIG_setStallMode(CONFIG_RTI0_BASE_ADDR, testParams->enableContinueOnSuspend);

    if(testParams->enableHalt == TRUE)
    {
        DebugP_log("Observe that RTIFRC/RTIUC regsiters of RTI-1 should be stopped while in debug mode...!\r\n");
    }
    else
    {
        DebugP_log("Observe that RTIFRC/RTIUC registers of RTI-1 counter should be running even while in debug mode...!\r\n");
    }
    /* Start the RTI counter */
    (void)RTI_counterEnable(CONFIG_RTI0_BASE_ADDR, testParams->counterBlock);

    DebugP_log("Timer Started, Observe the LED blink for every 0.5 sec...!\r\n");

    /* Wait until the LED is blinked specified number of times */
    while(gBlinkCount < LED_BLINK_COUNT);

    /* Stop the RTI counter */
    (void)RTI_counterDisable(CONFIG_RTI0_BASE_ADDR, testParams->counterBlock);

    DebugP_log("Timer Stopped...\r\n");

    DebugP_log("Press 'y' if test case passed otherwise press anykey\r\n");
    DebugP_scanf("%c", &testPass);

    DebugP_assert(testPass == 'y');

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

static void test_rti_set_params(RTI_TestParams *params, uint32_t tcId)
{
    test_rti_params_init(params);

    switch (tcId)
    {
        case 13416:
            params->counterBlock = RTI_TMR_CNT_BLK_INDEX_0;
            break;
        case 13417:
            params->counterBlock = RTI_TMR_CNT_BLK_INDEX_1;
            break;
        case 13418:
            params->compareEvent = RTI_TMR_CMP_BLK_INDEX_0;
            params->interruptNum = CONFIG_RTI0_INT_NUM_EVENT0;
            params->interruptFlag = RTI_TMR_INT_INT0_FLAG;
            params->dmaInterruptFlag = RTI_TMR_INT_DMA0_FLAG;
            break;
        case 13419:
            params->compareEvent = RTI_TMR_CMP_BLK_INDEX_1;
            params->interruptNum = CONFIG_RTI0_INT_NUM_EVENT1;
            params->interruptFlag = RTI_TMR_INT_INT1_FLAG;
            params->dmaInterruptFlag = RTI_TMR_INT_DMA1_FLAG;
            break;
        case 13420:
            params->compareEvent = RTI_TMR_CMP_BLK_INDEX_2;
            params->interruptNum = CONFIG_RTI0_INT_NUM_EVENT2;
            params->interruptFlag = RTI_TMR_INT_INT2_FLAG;
            params->dmaInterruptFlag = RTI_TMR_INT_DMA2_FLAG;
            break;
        case 13421:
            params->compareEvent = RTI_TMR_CMP_BLK_INDEX_3;
            params->interruptNum = CONFIG_RTI0_INT_NUM_EVENT3;
            params->interruptFlag = RTI_TMR_INT_INT3_FLAG;
            params->dmaInterruptFlag = RTI_TMR_INT_DMA3_FLAG;
            break;
        case 13415:
            params->enableHalt  = TRUE;
            params->enableContinueOnSuspend = RTI_GC_STALL_MODE_ON;
            break;
        case 13423:
            params->enableHalt = FALSE;
            params->enableContinueOnSuspend = RTI_GC_STALL_MODE_OFF;
            break;

    }

    return;
}

void rtiEvent0(void)
{
    if(gLedState == LED_ON)
    {
        GPIO_pinWriteLow(gpioBaseAddr, pinNum);
        gLedState = LED_OFF;
    }
    else{
        GPIO_pinWriteHigh(gpioBaseAddr, pinNum);
        gLedState = LED_ON;
    }
    gBlinkCount++;
}