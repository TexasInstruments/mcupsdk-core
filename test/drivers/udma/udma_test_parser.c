/*
 *  Copyright (c) Texas Instruments Incorporated 2018
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

/**
 *  \file udma_test_parser.c
 *
 *  \brief UDMA test application parser file.
 *
 */

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include "udma_test.h"
#include "udma_testcases.h"
#include "udma_testconfig.h"

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

/* Test application stack size */
#define APP_TSK_STACK_MAIN              (16U * 1024U)

/* ========================================================================== */
/*                         Structure Declarations                             */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

static int32_t udmaTestInit(UdmaTestObj *testObj);
static int32_t udmaTestDeinit(UdmaTestObj *testObj);

static int32_t udmaTestRunTc(UdmaTestObj *testObj, UdmaTestParams *testPrms);
static int32_t udmaTestCreateTestTasks(UdmaTestObj *testObj, UdmaTestParams *testPrms);
static int32_t udmaTestDeleteTestTasks(UdmaTestObj *testObj);
static void udmaTestTask(void *arg0, void *arg1);
static void udmaTestInitTestObj(UdmaTestObj *testObj, UdmaTestParams *testPrms);

static const UdmaTestChPrm *udmaTestGetChPrms(UdmaTestChPrmId chPrmId);
static const Udma_ChTxPrms *udmaTestGetTxChPrms(UdmaTestTxChPrmId txChPrmId);
static const Udma_ChRxPrms *udmaTestGetRxChPrms(UdmaTestRxChPrmId rxChPrmId);
static const Udma_ChPdmaPrms *udmaTestGetPdmaChPrms(UdmaTestPdmaChPrmId pdmaChPrmId);
static const UdmaTestRingPrm *udmaTestGetRingPrms(UdmaTestRingPrmId ringPrmId);

static int32_t udmaTestGetTcIdx(uint32_t tcId);
static bool udmaTestCheckIfTestToBeSkipped(UdmaTestObj    *testObj,
                                           UdmaTestParams *testPrms,
                                           uint32_t        tcType);

static int32_t udmaTestDisplayTestInfo(UdmaTestObj *testObj);
static int32_t udmaTestGenerateTestReports(UdmaTestObj *testObj);

static void udmaTestMenuSettings(UdmaTestObj *testObj);
static void udmaTestMenuMainShow(UdmaTestObj *testObj);
static void udmaTestMenuSettingsShow(UdmaTestObj *testObj);
static void udmaTestMenuCurrentSettingsShow(UdmaTestObj *testObj);
static void udmaTestSetDefaultCfg(UdmaTestObj *testObj);
static uint32_t udmaTestGetTestId(UdmaTestObj *testObj, uint32_t tcType);

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */

#if !defined (UDMA_UT_BAREMETAL)
/* Task stack */
static uint8_t  gUdmaParserTskStack[UDMA_TEST_MAX_TASKS][APP_TSK_STACK_MAIN] __attribute__((aligned(32)));;
#endif

/* UDMA UT object. */
UdmaTestObj gUdmaTestUtObj;

/* Main menu 0 string */
static const char gUdmaTestMenuMain0[] =
{
    "\r\n "
    "\r\n ==============="
    "\r\n UDMA Test Select"
    "\r\n ==============="
    "\r\n "
};

/* Main menu string */
static const char gUdmaTestMenuMain1[] =
{
    "\r\n 1: Manual testing (select specific test case to run)"
    "\r\n 2: Sanity (BFT) testing"
    "\r\n 3: Regression testing"
    "\r\n 4: Full testing"
    "\r\n 5: Performance testing"
    "\r\n d: Display test cases"
    "\r\n g: Generate test report"
    "\r\n "
    "\r\n s: System Settings"
    "\r\n "
    "\r\n q: Quit "
    "\r\n "
    "\r\n Enter Choice: "
    "\r\n"
};

static const char gUdmaTestMenuSettings0[] = {
    "\r\n ==============="
    "\r\n System Settings"
    "\r\n ==============="
    "\r\n "
};

static const char gUdmaTestMenuSettings1[] = {
    "\r\n "
    "\r\n l: Loop Count"
    "\r\n d: Queue Depth (How many outstanding transfer per channel)"
    "\r\n r: Runtime Print Enable"
    "\r\n "
    "\r\n q: Quit "
    "\r\n"
};

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

/**
 *  udmaTestParser
 */
int32_t udmaTestParser(void)
{
    char            option;
    bool            done;
    int32_t         retVal = UDMA_SOK, getCharRetVal = UDMA_SOK;
    int32_t         tcIdx, startIdx;
    uint32_t        testCnt, tcType, tcId;
    uint32_t        startTime, elapsedTime, startTime1, elapsedTime1;
    uint32_t        hrs, mins, secs, durationInSecs;
    UdmaTestObj    *testObj;
    UdmaTestParams *testPrms;
    uint32_t        isValidInput = FALSE;

    testObj = &gUdmaTestUtObj;
    udmaTestSetDefaultCfg(testObj);
    udmaTestResetTestResult();

    retVal = udmaTestInit(testObj);
    if(UDMA_SOK != retVal)
    {
        return (retVal);
    }

    startTime             = AppUtils_getCurTimeInMsec();
    done                  = FALSE;
    testObj->skipCount    = 0U;
    testObj->disableCount = 0U;
    while(!done)
    {
        udmaTestMenuMainShow(testObj);
        if(isValidInput != TRUE)
        {
#if defined (UDMA_UT_ENABLE_MANUAL_ENTRY)
            getCharRetVal = AppUtils_getCharTimeout(&option, UDMA_TEST_UART_TIMEOUT_MSEC);
#else
            /* Disable user input - do full automation */
            getCharRetVal = UDMA_EFAIL;
#endif
            if(UDMA_SOK != getCharRetVal)
            {
                /* Timeout */
                GT_0trace(
                    testObj->traceMask, GT_INFO,
                    " UART read timeout (10 sec)!!\r\n");
                /* Default to full test */
                GT_0trace(
                    testObj->traceMask, GT_INFO, " Automating FULL test!!\r\n");
                option = '4';
                /* Set done flag to end test */
                done   = TRUE;
            }
            else
            {
                isValidInput = TRUE;
            }
        }
        else
        {
            option = AppUtils_getChar();
        }
        GT_0trace(testObj->traceMask, GT_INFO, " \r\n");

        tcType = UDMA_TCT_SANITY;
        switch(option)
        {
            case 's':
            case 'S':
                udmaTestMenuSettings(testObj);
                break;

            case 'q':
            case 'Q':
                done = TRUE;
                break;

            case '1':
                GT_0trace(testObj->traceMask, GT_INFO, " Manual testing \r\n");
                tcId = udmaTestGetTestId(testObj, UDMA_TCT_ALL);
                /* Find the test case to run */
                tcIdx = udmaTestGetTcIdx(tcId);
                if(tcIdx < 0)
                {
                    GT_0trace(testObj->traceMask, GT_INFO,
                              " Test case ID not found\r\n");
                }
                else
                {
                    testPrms = &gUdmaTestCases[tcIdx];
                    udmaTestRunTc(testObj, testPrms);
                }
                break;

            case '4':
                tcType |= UDMA_TCT_FULL;
            case '3':
                tcType |= UDMA_TCT_REGRESSION;
            case '2':
                if(TRUE == done)
                {
                    /* Auto run, start from 1st test case */
                    tcId = 0U;
                }
                else
                {
                    GT_0trace(
                        testObj->traceMask, GT_INFO,
                        " Enter Start Test Case Id "
                        "(Enter 0 to start from first): \r\n");
                    tcId = udmaTestGetTestId(testObj, tcType);
                }
                if(0U == tcId)
                {
                    startIdx = 0U;
                }
                else
                {
                    startIdx = udmaTestGetTcIdx(tcId);
                    if(startIdx < 0)
                    {
                        GT_0trace(testObj->traceMask, GT_INFO,
                                  " Test case ID not found\r\n");
                        continue;
                    }
                }

                /* Run all test cases one after the other depending on selected
                 * flag */
                startTime1         = AppUtils_getCurTimeInMsec();
                testObj->skipCount = testObj->disableCount = 0U;

                for(testCnt = startIdx;
                     testCnt < UDMA_TEST_NUM_TESTCASES;
                     testCnt++)
                {
                    testPrms = &gUdmaTestCases[testCnt];

                    /* Check whether to execute test or not */
                    if(udmaTestCheckIfTestToBeSkipped(testObj, testPrms, tcType))
                    {
                        continue;       /* Skip test */
                    }

                    udmaTestRunTc(testObj, testPrms);
                }

                /* Print test results */
                udmaTestPrintTestResult(
                    testObj,
                    testObj->skipCount,
                    testObj->disableCount);

                elapsedTime1   = AppUtils_getElapsedTimeInMsec(startTime1);
                durationInSecs = ((elapsedTime1) / 1000U);
                hrs  = durationInSecs / (60U * 60U);
                mins = (durationInSecs / 60U) - (hrs * 60U);
                secs = durationInSecs - (hrs * 60U * 60U) - (mins * 60U);
                GT_3trace(testObj->traceMask, GT_INFO,
                          " |TOTAL TEST DURATION|: %d hrs %d mins %d secs\r\n",
                          hrs, mins, secs);
                break;

            case '5':
                tcType = UDMA_TCT_PERFORMANCE;
                startIdx = 0U;

                /* Run all test cases one after the other depending on selected
                 * flag */
                startTime1         = AppUtils_getCurTimeInMsec();
                testObj->skipCount = testObj->disableCount = 0U;
                for(testCnt = startIdx;
                    testCnt < UDMA_TEST_NUM_TESTCASES;
                    testCnt++)
                {
                    testPrms = &gUdmaTestCases[testCnt];

                    /* Check whether to execute test or not */
                    if(udmaTestCheckIfTestToBeSkipped(testObj, testPrms, tcType))
                    {
                        continue;       /* Skip test */
                    }

                    udmaTestRunTc(testObj, testPrms);
                }

                /* Print test results */
                udmaTestPrintTestResult(
                    testObj, testObj->skipCount, testObj->disableCount);

                elapsedTime1   = AppUtils_getElapsedTimeInMsec(startTime1);
                durationInSecs = ((elapsedTime1) / 1000U);
                hrs  = durationInSecs / (60U * 60U);
                mins = (durationInSecs / 60U) - (hrs * 60U);
                secs = durationInSecs - (hrs * 60U * 60U) - (mins * 60U);
                GT_3trace(testObj->traceMask, GT_INFO,
                          " |TOTAL TEST DURATION|: %d hrs %d mins %d secs\r\n",
                          hrs, mins, secs);
                break;

            case 'd':
            case 'D':
                /* Display test info */
                udmaTestDisplayTestInfo(testObj);
                break;

            case 'g':
            case 'G':
                /* Generate test report */
                udmaTestGenerateTestReports(testObj);
                break;

            default:
                GT_0trace(testObj->traceMask, GT_INFO,
                          " Invalid option try again!!\r\n");
                break;
        }
    }

    udmaTestDisplayTestInfo(testObj);
    udmaTestGenerateTestReports(testObj);

    elapsedTime    = AppUtils_getElapsedTimeInMsec(startTime);
    durationInSecs = ((elapsedTime) / 1000U);
    hrs  = durationInSecs / (60U * 60U);
    mins = (durationInSecs / 60U) - (hrs * 60U);
    secs = durationInSecs - (hrs * 60U * 60U) - (mins * 60U);
    GT_3trace(testObj->traceMask, GT_INFO,
              " |TOTAL UT DURATION|: %d hrs %d mins %d secs\r\n",
              hrs, mins, secs);

    /* Check if all testcases passed and accordingly return pass/fail */
    for(testCnt = 0U; testCnt < UDMA_TEST_NUM_TESTCASES; testCnt++)
    {
        testPrms = &gUdmaTestCases[testCnt];
        if(TRUE == testPrms->isRun)
        {
            if(UDMA_SOK != testPrms->testResult)
            {
                retVal = testPrms->testResult;
            }
        }
    }

    retVal = udmaTestDeinit(testObj);
    if(UDMA_SOK != retVal)
    {
        GT_0trace(testObj->traceMask, GT_ERR, " UDMA deinit failed!!\n");
    }

    /* Print test results */
    udmaTestPrintTestResult(testObj, testObj->skipCount, testObj->disableCount);

    return (retVal);
}

/**
 *  udmaTestRunTc
 */
static int32_t udmaTestRunTc(UdmaTestObj *testObj, UdmaTestParams *testPrms)
{
    static char     tempString[UDMA_TEST_PRINT_BUFSIZE];
    int32_t         retVal = UDMA_SOK;
    uint32_t        startTime, hrs, mins, secs, msecs;
    uint32_t        durationInMsecs, durationInSecs;
    static char    *enableDisableName[] = {"Disabled", "Enabled"};

    /* NULL pointer check */
    GT_assert(testObj->traceMask, (NULL != testPrms));
    GT_assert(testObj->traceMask, (NULL != testPrms->tcName));

    GT_0trace(testObj->traceMask, GT_INFO, " \r\n");
    GT_1trace(testObj->traceMask, GT_INFO,
              " |TEST START|:: %d ::\r\n", testPrms->tcId);
    GT_1trace(testObj->traceMask, GT_INFO,
              " |TEST NAME|:: %s ::\r\n", testPrms->tcName);
    GT_1trace(testObj->traceMask, GT_INFO,
              " |TEST INFO|:: Num Tasks         : %d ::\r\n", testPrms->numTasks);
    GT_1trace(testObj->traceMask, GT_INFO,
              " |TEST INFO|:: Data Check        : %s ::\r\n", enableDisableName[testPrms->dcEnable]);
    GT_1trace(testObj->traceMask, GT_INFO,
              " |TEST INFO|:: Performance       : %s ::\r\n", enableDisableName[testPrms->prfEnable]);
    GT_1trace(testObj->traceMask, GT_INFO,
              " |TEST INFO|:: Print             : %s ::\r\n", enableDisableName[testPrms->printEnable]);

    udmaTestInitTestObj(testObj, testPrms);
    /* Start the load calculation */
    Utils_prfLoadCalcStart();

    testPrms->isRun = TRUE;
    startTime = AppUtils_getCurTimeInMsec();
    retVal = udmaTestCreateTestTasks(testObj, testPrms);
    durationInMsecs = AppUtils_getElapsedTimeInMsec(startTime);

    Utils_prfLoadCalcStop();
    Utils_prfLoadPrintAll(TRUE, testObj->traceMask);
    Utils_prfLoadCalcReset();
    retVal += udmaTestDeleteTestTasks(testObj);

    durationInSecs  = (durationInMsecs / 1000U);
    hrs   = durationInSecs / (60U * 60U);
    mins  = (durationInSecs / 60U) - (hrs * 60U);
    secs  = durationInSecs - (hrs * 60U * 60U) - (mins * 60U);
    msecs = durationInMsecs - ((hrs * 60U * 60U) + (mins * 60U) + secs) * 1000U;
    GT_4trace(testObj->traceMask, GT_INFO,
              " |TEST DURATION|:: %d:%0.2d:%0.2d:%0.3d ::\r\n",
              hrs, mins, secs, msecs);

    if((TRUE == testPrms->prfEnable) && (testPrms->numTasks > 1U))
    {
        /* Total performance applicable only for multi thread.
         * Incase of multi thread, individual performance doesn't makes sense
         * Incase of single thread, the thread itself will print the accurate
         * performance. Total is not very accurate. Hence disable from here */
        udmaTestCalcTotalPerformance(testObj, durationInMsecs);
    }

    GT_assert(testObj->traceMask, (NULL != testPrms->tcName));
    snprintf(tempString, sizeof (tempString), "%s", testPrms->tcName);
    udmaTestLogTestResult(testObj, retVal, testPrms->tcId, tempString);

    if(UDMA_SOK == retVal)
    {
        GT_1trace(testObj->traceMask, GT_INFO,
            " |TEST RESULT|:: UDMA Test Case %d Successful!! ::\r\n", testObj->testPrms->tcId);
    }
    else
    {
        GT_1trace(testObj->traceMask, GT_INFO,
            " |TEST RESULT|:: UDMA Test Case %d Failed!! ::\r\n", testObj->testPrms->tcId);
    }

    /* Store the test result */
    testPrms->testResult = retVal;

    return (retVal);
}
#if defined (UDMA_UT_BAREMETAL)
int32_t udmaTestCreateTestTasks(UdmaTestObj *testObj, UdmaTestParams *testPrms)
{
    int32_t             retVal = UDMA_SOK;
    uint32_t            taskCnt;
    UdmaTestTaskObj    *taskObj;

    for(taskCnt = 0U; taskCnt < testPrms->numTasks; taskCnt++)
    {
        taskObj = &testObj->taskObj[taskCnt];
        udmaTestTask(taskObj, NULL);
    }

    for(taskCnt = 0u; taskCnt < testPrms->numTasks; taskCnt++)
    {
        retVal += testObj->taskObj[taskCnt].testResult;
    }

    return (retVal);
}

static int32_t udmaTestDeleteTestTasks(UdmaTestObj *testObj)
{
    return UDMA_SOK;
}

#else
int32_t udmaTestCreateTestTasks(UdmaTestObj *testObj, UdmaTestParams *testPrms)
{
    int32_t             retVal = UDMA_SOK;
    uint32_t            taskCnt, createdTsk = 0U;
    TaskP_Params        taskPrms;
    UdmaTestTaskObj    *taskObj;

    retVal = SemaphoreP_constructCounting(&testObj->taskCompleteSem, 0U, 100U);
    if(UDMA_SOK == retVal)
    {
        for(taskCnt = 0U; taskCnt < testPrms->numTasks; taskCnt++)
        {
            taskObj = &testObj->taskObj[taskCnt];

            TaskP_Params_init(&taskPrms);
            taskPrms.args = taskObj;
            taskPrms.stack = &gUdmaParserTskStack[taskCnt][0U];
            taskPrms.stackSize = APP_TSK_STACK_MAIN;
            taskPrms.taskMain = (void *) &udmaTestTask;
            retVal = TaskP_construct(&taskObj->taskHandle, &taskPrms);
            if(retVal != UDMA_SOK)
            {
                break;
            }
            createdTsk++;

            /* Register the task to the load module for calculating the load */
            snprintf(taskObj->prfTsName, sizeof (taskObj->prfTsName), "Task%d", taskCnt);
            Utils_prfLoadRegister(&taskObj->taskHandle, taskObj->prfTsName);
        }
    }

    if(UDMA_SOK == retVal)
    {
        for(taskCnt = 0U; taskCnt < createdTsk; taskCnt++)
        {
            /* Wait for tasks to complete */
            SemaphoreP_pend(&testObj->taskCompleteSem, SystemP_WAIT_FOREVER);
        }
    }

    if(UDMA_SOK == retVal)
    {
        for(taskCnt = 0u; taskCnt < createdTsk; taskCnt++)
        {
            retVal += testObj->taskObj[taskCnt].testResult;
        }
    }
    SemaphoreP_destruct(&testObj->taskCompleteSem);

    return (retVal);
}

static int32_t udmaTestDeleteTestTasks(UdmaTestObj *testObj)
{
    int32_t             retVal = UDMA_SOK;
    uint32_t            taskCnt;
    UdmaTestTaskObj    *taskObj;

    /* Delete all created tasks and semephores */
    for(taskCnt = 0U; taskCnt < UDMA_TEST_MAX_TASKS; taskCnt++)
    {
        taskObj = &testObj->taskObj[taskCnt];
        Utils_prfLoadUnRegister(&taskObj->taskHandle);
        TaskP_destruct(&taskObj->taskHandle);
    }

    return (retVal);
}
#endif

static void udmaTestTask(void *arg0, void *arg1)
{
    int32_t             retVal;
    UdmaTestObj        *testObj;
    UdmaTestTaskObj    *taskObj;

    /* Run the test */
    taskObj = (UdmaTestTaskObj *) arg0;
    testObj = taskObj->testObj;

    /* Run the test */
    GT_assert(testObj->traceMask, (taskObj->testFxnPtr != NULL));
    retVal = taskObj->testFxnPtr(taskObj);

    taskObj->testResult = retVal;

#if !defined (UDMA_UT_BAREMETAL)
    /* Test complete. Signal it */
    SemaphoreP_post(&testObj->taskCompleteSem);
#endif

    return;
}

static int32_t udmaTestInit(UdmaTestObj *testObj)
{
    int32_t             retVal = UDMA_SOK;

    retVal += Utils_memInit();
    retVal += udmaTestInitDriver(testObj);

    return (retVal);
}

static int32_t udmaTestDeinit(UdmaTestObj *testObj)
{
    int32_t     retVal = UDMA_SOK;

    /* Deinit the drivers */
    retVal += udmaTestDeinitDriver(testObj);
    retVal += Utils_memDeInit();

    return (retVal);
}

static void udmaTestInitTestObj(UdmaTestObj *testObj, UdmaTestParams *testPrms)
{
    uint32_t                taskCnt, chCnt, qCnt, chIdx, totalCh = 0U, cnt, dim;
    volatile UdmaTestTaskObj    *taskObj;
    const UdmaTestChPrm    *chPrms;
    const Udma_ChTxPrms    *txPrms = NULL;
    const Udma_ChRxPrms    *rxPrms = NULL;
    const Udma_ChPdmaPrms  *pdmaPrms = NULL;
    UdmaTestChObj          *chObj;

    testObj->testPrms  = testPrms;
    testObj->traceMask = (GT_INFO1 | GT_TraceState_Enable);
    if(FALSE == testPrms->printEnable)
    {
        /* Restrict the number of prints when print is disabled */
        testObj->traceMask = (GT_INFO | GT_TraceState_Enable);
    }

    GT_assert(testObj->traceMask, (testPrms->numTasks <= UDMA_TEST_MAX_TASKS));
    for(taskCnt = 0U; taskCnt < testPrms->numTasks; taskCnt++)
    {
        taskObj = &testObj->taskObj[taskCnt];
        taskObj->testPrms   = testPrms;
        taskObj->testType   = testPrms->testType[taskCnt];
        taskObj->testFxnPtr = testPrms->testFxnPtr[taskCnt];
        GT_assert(testObj->traceMask, (taskObj->testFxnPtr != NULL));
        GT_assert(testObj->traceMask, (testPrms->numCh[taskCnt] <= UDMA_TEST_MAX_CH));
        GT_assert(testObj->traceMask, (testPrms->numCh[taskCnt] != 0U));
        taskObj->pacingTime = testPrms->pacingTime[taskCnt];
        taskObj->numCh      = testPrms->numCh[taskCnt];
        taskObj->traceMask  = testObj->traceMask;
        taskObj->testResult = UDMA_SOK;
        taskObj->totalTransfer = 0U;
        taskObj->durationMs = 1U;
        taskObj->mps        = 0U;
        taskObj->testObj    = testObj;
        taskObj->taskId     = taskCnt;
        taskObj->prfTsHandle = NULL;
        taskObj->loopCnt    = testPrms->loopCnt;
        if(USE_DEF_LP_CNT == taskObj->loopCnt)
        {
            taskObj->loopCnt = testObj->sysCtrl.loopCnt;
        }

        taskObj->ringPrms = udmaTestGetRingPrms(testPrms->ringPrmId);
        for(chCnt = totalCh; chCnt < (taskObj->numCh + totalCh); chCnt++)
        {
            chIdx = chCnt - totalCh;
            chObj = &testObj->chObjs[chCnt];
            taskObj->chObj[chIdx] = chObj;

            chObj->chIdx            = chIdx;
            chObj->chHandle         = NULL;
            chObj->cqEventHandle    = NULL;
            chObj->tdCqEventHandle  = NULL;
            chObj->drvHandle        = &testObj->drvObj[testPrms->instId[chCnt]];
            chObj->instId           = testPrms->instId[chCnt];
            chObj->queueCnt         = 0U;
            chObj->dequeueCnt       = 0U;
            chObj->fqRingMem        = NULL;
            chObj->cqRingMem        = NULL;
            chObj->tdCqRingMem      = NULL;
            chObj->srcBuf           = NULL;
            for(qCnt = 0U; qCnt < UDMA_TEST_MAX_QDEPTH; qCnt++)
            {
                chObj->trpdMem[qCnt]    = NULL;
                chObj->destBuf[qCnt]    = NULL;
            }

            chPrms = udmaTestGetChPrms(testPrms->chPrmId[chCnt]);
            GT_assert(testObj->traceMask, (chPrms != NULL));
            chObj->chPrms = chPrms;
            if(NULL != chPrms)
            {
                txPrms = udmaTestGetTxChPrms(chPrms->txPrmId);
                rxPrms = udmaTestGetRxChPrms(chPrms->rxPrmId);
                pdmaPrms = udmaTestGetPdmaChPrms(chPrms->pdmaPrmId);
            }
            chObj->txPrms = txPrms;
            chObj->rxPrms = rxPrms;
            chObj->pdmaPrms = pdmaPrms;

            chObj->qdepth = testPrms->qdepth[chCnt];
            if(USE_DEF_QDEPTH == chObj->qdepth)
            {
                chObj->qdepth = testObj->sysCtrl.qdepth;
            }
            GT_assert(testObj->traceMask, (chObj->qdepth <= UDMA_TEST_MAX_QDEPTH));
            for(cnt = 0U; cnt < UDMA_TEST_MAX_ICNT; cnt++)
            {
                chObj->icnt[cnt]    = testPrms->icnt[chCnt][cnt];
                chObj->dicnt[cnt]   = testPrms->dicnt[chCnt][cnt];
            }
            for(dim = 0U; dim < UDMA_TEST_MAX_DIM; dim++)
            {
                chObj->dim[dim]    = testPrms->dim[chCnt][dim];
                chObj->ddim[dim]   = testPrms->ddim[chCnt][dim];
            }
            chObj->heapIdSrc    = testPrms->heapIdSrc[chCnt];
            chObj->heapIdDest   = testPrms->heapIdDest[chCnt];
            chObj->srcBufSize   = testPrms->srcBufSize[chCnt];
            chObj->destBufSize  = testPrms->destBufSize[chCnt];
            chObj->ringMemSize  = 0U;
            chObj->trpdSize     = 0U;
        }
        totalCh += taskObj->numCh;
        GT_assert(testObj->traceMask, (totalCh <= UDMA_TEST_MAX_CH));
    }

    return;
}

static const UdmaTestChPrm *udmaTestGetChPrms(UdmaTestChPrmId chPrmId)
{
    uint32_t                index;
    const UdmaTestChPrm    *chPrms = NULL;

    /* Search in channel table */
    for(index = 0U; index < UDMA_TEST_NUM_CH_PRM; index++)
    {
        if(gUdmaTestChPrm[index].chPrmId == chPrmId)
        {
            chPrms = &gUdmaTestChPrm[index];
            break;
        }
    }

    return (chPrms);
}

static const Udma_ChTxPrms *udmaTestGetTxChPrms(UdmaTestTxChPrmId txChPrmId)
{
    uint32_t index;
    const Udma_ChTxPrms *txPrms = NULL;

    /* Search in channel table */
    for(index = 0U; index < UDMA_TEST_NUM_TX_CH_PRM; index++)
    {
        if(gUdmaTestTxChPrm[index].txChPrmId == txChPrmId)
        {
            txPrms = &gUdmaTestTxChPrm[index].txPrms;
            break;
        }
    }

    return (txPrms);
}

static const Udma_ChRxPrms *udmaTestGetRxChPrms(UdmaTestRxChPrmId rxChPrmId)
{
    uint32_t index;
    const Udma_ChRxPrms *rxPrms = NULL;

    /* Search in channel table */
    for(index = 0U; index < UDMA_TEST_NUM_RX_CH_PRM; index++)
    {
        if(gUdmaTestRxChPrm[index].rxChPrmId == rxChPrmId)
        {
            rxPrms = &gUdmaTestRxChPrm[index].rxPrms;
            break;
        }
    }

    return (rxPrms);
}

static const Udma_ChPdmaPrms *udmaTestGetPdmaChPrms(UdmaTestPdmaChPrmId pdmaChPrmId)
{
    uint32_t                index;
    const Udma_ChPdmaPrms  *pdmaPrms = NULL;

    /* Search in channel table */
    for(index = 0U; index < UDMA_TEST_NUM_PDMA_CH_PRM; index++)
    {
        if(gUdmaTestPdmaChPrm[index].pdmaChPrmId == pdmaChPrmId)
        {
            pdmaPrms = &gUdmaTestPdmaChPrm[index].pdmaPrms;
            break;
        }
    }

    return (pdmaPrms);
}

static const UdmaTestRingPrm *udmaTestGetRingPrms(UdmaTestRingPrmId ringPrmId)
{
    uint32_t                index;
    const UdmaTestRingPrm  *ringPrms = NULL;

    /* Search in ring table */
    for(index = 0U; index < UDMA_TEST_NUM_RING_PRM; index++)
    {
        if(gUdmaTestRingPrm[index].ringPrmId == ringPrmId)
        {
            ringPrms = &gUdmaTestRingPrm[index];
            break;
        }
    }

    return (ringPrms);
}

/**
 *  udmaTestGetTcIdx
 */
static int32_t udmaTestGetTcIdx(uint32_t tcId)
{
    int32_t         tcIdx = -1;
    uint32_t        testCnt;
    UdmaTestParams *testPrms;

    testPrms = &gUdmaTestCases[0U];
    for(testCnt = 0U; testCnt < UDMA_TEST_NUM_TESTCASES; testCnt++)
    {
        if(testPrms->tcId == tcId)
        {
            tcIdx = testCnt;
            break;
        }
        testPrms++;
    }

    return (tcIdx);
}

/**
 *  udmaTestCheckIfTestToBeSkipped
 */
static bool udmaTestCheckIfTestToBeSkipped(UdmaTestObj    *testObj,
                                           UdmaTestParams *testPrms,
                                           uint32_t        tcType)
{
    bool skipTest = FALSE;

    /* Check whether test case is disabled */
    if(FALSE == testPrms->enableTest)
    {
        GT_assert(testObj->traceMask, (NULL != testPrms->tcName));
        GT_0trace(testObj->traceMask, GT_INFO, " \r\n");
        GT_1trace(testObj->traceMask, GT_INFO,
                  " |TEST DISABLED|:: %d ::\r\n", testPrms->tcId);
        GT_1trace(testObj->traceMask, GT_INFO,
                  " |TEST PARAM|:: %s ::\r\n", testPrms->tcName);
        if(NULL != testPrms->disableInfo)
        {
            GT_1trace(testObj->traceMask, GT_INFO,
                      " |TEST DISABLE REASON|:: %s ::\r\n",
                      testPrms->disableInfo);
        }
        else
        {
            GT_0trace(testObj->traceMask, GT_INFO,
                      " |TEST DISABLE REASON|:: Not Provided!! ::\r\n");
        }

        testObj->disableCount++;
        skipTest = TRUE;        /* Skip test */
    }

    /* Ignore test case depending on test flag and selected option */
    if((FALSE == skipTest) && (UDMA_TCT_MISC != testPrms->tcType))
    {
        if(!(tcType & testPrms->tcType))
        {
            testObj->skipCount++;
            skipTest = TRUE;    /* Skip test */
        }
    }

    /* Ignore test case depending on run flag */
    if(FALSE == skipTest)
    {
        if((testObj->runFlag & testPrms->runFlag) != testObj->runFlag)
        {
            testObj->skipCount++;
            skipTest = TRUE;    /* Skip test */
        }
    }

    return (skipTest);
}

/**
 *  udmaTestDisplayTestInfo
 */
static int32_t udmaTestDisplayTestInfo(UdmaTestObj *testObj)
{
    uint32_t        sCnt, testCnt;
    char           *runStatus;
    UdmaTestParams *testPrms;
    static char    *enableDisableName[] = {"Disabled", "Enabled"};
    static char     printBuf[UDMA_TEST_PRINT_BUFSIZE];

    /* Display test info */
    sCnt = 1;
    GT_0trace(testObj->traceMask, GT_INFO, " \r\n");
    GT_0trace(
        testObj->traceMask, GT_INFO,
        " S.No        ID         Description                                   "
        "                              Status    Auto Run\r\n");
    GT_0trace(
        testObj->traceMask, GT_INFO,
        " ---------------------------------------------------------------------"
        "------------------------------------------------\r\n");
    for(testCnt = 0U; testCnt < UDMA_TEST_NUM_TESTCASES; testCnt++)
    {
        testPrms = &gUdmaTestCases[testCnt];

        runStatus = "NRY";
        if(FALSE == testPrms->isRun)
        {
            if(FALSE == testPrms->enableTest)
            {
                runStatus = "NRQ";
            }
            if((testObj->runFlag & testPrms->runFlag) != testObj->runFlag)
            {
                runStatus = "NA";
            }
        }
        else
        {
            if(UDMA_SOK == testPrms->testResult)
            {
                runStatus = "PASS";
            }
            else
            {
                runStatus = "FAIL";
            }
        }

        GT_assert(testObj->traceMask, (NULL != testPrms->tcName));
        snprintf(printBuf, sizeof (printBuf),
                 "  %3d  PDK-%-4d  %-75.75s  %-8.8s  %-8.8s",
                 sCnt,
                 testPrms->tcId,
                 testPrms->tcName,
                 runStatus,
                 enableDisableName[testPrms->enableTest]);
        GT_1trace(testObj->traceMask, GT_INFO, "%s\r\n", printBuf);

        sCnt++;
    }
    GT_0trace(testObj->traceMask, GT_INFO, " \r\n");

    return (UDMA_SOK);
}

/**
 *  udmaTestGenerateTestReports
 */
static int32_t udmaTestGenerateTestReports(UdmaTestObj *testObj)
{
    uint32_t        sCnt, testCnt;
    char           *runStatus, *category, *adequacy;
    UdmaTestParams *testPrms;
    static char     printBuf[UDMA_TEST_PRINT_BUFSIZE];

    sCnt = 1;
    GT_0trace(testObj->traceMask, GT_INFO, " \r\n");
    GT_0trace(
        testObj->traceMask, GT_INFO,
        "S.No;ID;Requirement Mapping;Description;Pass Fail Criteria;"
        "IR;Category;Test Adequacy;Test Result;\r\n");
    for(testCnt = 0U; testCnt < UDMA_TEST_NUM_TESTCASES; testCnt++)
    {
        testPrms = &gUdmaTestCases[testCnt];

        runStatus = "NRY";
        if(FALSE == testPrms->isRun)
        {
            if(FALSE == testPrms->enableTest)
            {
                runStatus = "NRQ";
            }
            if((testObj->runFlag & testPrms->runFlag) != testObj->runFlag)
            {
                runStatus = "NA";
            }
        }
        else
        {
            if(UDMA_SOK == testPrms->testResult)
            {
                runStatus = "PASS";
            }
            else
            {
                runStatus = "FAIL";
            }
        }

        if(testPrms->tcType & UDMA_TCT_FULL)
        {
            category = "Full";
        }
        else if(testPrms->tcType & UDMA_TCT_REGRESSION)
        {
            category = "Regression";
        }
        else
        {
            category = "Sanity";
        }

        if(testPrms->tcType & UDMA_TCT_STRESS)
        {
            adequacy = "Stress";
        }
        else if(testPrms->tcType & UDMA_TCT_NEGATIVE)
        {
            adequacy = "Negative";
        }
        else if(testPrms->tcType & UDMA_TCT_PERFORMANCE)
        {
            adequacy = "Performance";
        }
        else if(testPrms->tcType & UDMA_TCT_MISC)
        {
            adequacy = "Misc";
        }
        else if(testPrms->tcType & UDMA_TCT_API)
        {
            adequacy = "Api";
        }
        else
        {
            adequacy = "Functional";
        }

        GT_assert(testObj->traceMask, (NULL != testPrms->tcName));
        snprintf(printBuf, sizeof (printBuf),
                 "%d;PDK-%d;%s;;%s;%s;%s",
                 sCnt,
                 testPrms->tcId,
                 testPrms->tcName,
                 category,
                 adequacy,
                 runStatus);
        GT_1trace(testObj->traceMask, GT_INFO, "%s\r\n", printBuf);
        sCnt++;
    }
    GT_0trace(testObj->traceMask, GT_INFO, " \r\n");

    return (UDMA_SOK);
}

/**
 *  udmaTestMenuSettings
 */
static void udmaTestMenuSettings(UdmaTestObj *testObj)
{
    char                option;
    bool                done = FALSE;
    int32_t             value;
    UdmaTestSystemCtrl *sysCtrl = &testObj->sysCtrl;

    udmaTestMenuSettingsShow(testObj);

    while(!done)
    {
        GT_0trace(testObj->traceMask, GT_INFO, " Enter Choice: \r\n");
        option = AppUtils_getChar();

        switch(option)
        {
            case 'd':
            case 'D':
                GT_0trace(testObj->traceMask, GT_INFO,
                          " Queue count: \r\n");
                value = AppUtils_getNum();

                if(value != USE_DEF_QDEPTH)
                {
                    sysCtrl->qdepth = value;
                }
                else
                {
                    GT_0trace(
                        testObj->traceMask, GT_INFO,
                        " This matches with default flag, give another value\r\n");
                }
                break;

            case 'l':
            case 'L':
                GT_0trace(testObj->traceMask, GT_INFO,
                          " Loop count: \r\n");
                value = AppUtils_getNum();

                if(value != USE_DEF_LP_CNT)
                {
                    sysCtrl->loopCnt = value;
                }
                else
                {
                    GT_0trace(
                        testObj->traceMask, GT_INFO,
                        " This matches with default flag, give another value\r\n");
                }
                break;

            case 'r':
            case 'R':
                GT_0trace(testObj->traceMask, GT_INFO,
                          " Runtime Print Enable [0: Disable, 1: Enable]: \r\n");
                value = AppUtils_getNum();

                sysCtrl->rtPrintEnable = FALSE;
                if(1 == value)
                {
                    sysCtrl->rtPrintEnable = TRUE;
                }
                break;

            case 'q':
            case 'Q':
                done = TRUE;
                break;
        }
        fflush(stdin);
    }

    return;
}

/**
 *  udmaTestMenuMainShow
 */
static void udmaTestMenuMainShow(UdmaTestObj *testObj)
{
    GT_0trace(testObj->traceMask, GT_INFO, gUdmaTestMenuMain0);
    udmaTestMenuCurrentSettingsShow(testObj);
    GT_0trace(testObj->traceMask, GT_INFO, gUdmaTestMenuMain1);

    return;
}

/**
 *  udmaTestMenuSettingsShow
 */
static void udmaTestMenuSettingsShow(UdmaTestObj *testObj)
{
    GT_0trace(testObj->traceMask, GT_INFO, gUdmaTestMenuSettings0);
    udmaTestMenuCurrentSettingsShow(testObj);
    GT_0trace(testObj->traceMask, GT_INFO, gUdmaTestMenuSettings1);

    return;
}

/**
 *  udmaTestMenuCurrentSettingsShow
 */
static void udmaTestMenuCurrentSettingsShow(UdmaTestObj *testObj)
{
    static char       *enableDisableName[] = {"OFF", "ON"};
    UdmaTestSystemCtrl *sysCtrl = &testObj->sysCtrl;

    GT_0trace(testObj->traceMask, GT_INFO, "\r\n Current System Settings:");
    GT_0trace(testObj->traceMask, GT_INFO, "\r\n ------------------------");

    GT_1trace(testObj->traceMask, GT_INFO,
              "\r\n Loop Count             : %d", sysCtrl->loopCnt);
    GT_1trace(testObj->traceMask, GT_INFO,
              "\r\n Queue Depth            : %d", sysCtrl->qdepth);
    GT_1trace(testObj->traceMask, GT_INFO,
              "\r\n Runtime Print          : %s",
              enableDisableName[sysCtrl->rtPrintEnable]);
    GT_0trace(testObj->traceMask, GT_INFO, "\r\n ");

    return;
}

/**
 *  udmaTestSetDefaultCfg
 */
static void udmaTestSetDefaultCfg(UdmaTestObj *testObj)
{
    uint32_t        testCnt;
    UdmaTestParams *testPrms;

    memset(testObj, 0, sizeof(UdmaTestObj));
    testObj->traceMask             = (GT_INFO1 | GT_TraceState_Enable);
    testObj->sysCtrl.loopCnt       = UDMA_TEST_DEF_LOOP_CNT;
    testObj->sysCtrl.qdepth        = UDMA_TEST_DEF_QDEPTH;
#if defined (UDMA_UT_DYNAMIC_ANALYSIS)
    testObj->sysCtrl.loopCnt       = 1U;
    testObj->sysCtrl.qdepth        = 1U;
#endif
    testObj->sysCtrl.rtPrintEnable = FALSE;

    /* Set run flag */
    testObj->runFlag = 0U;
    testObj->runFlag |= UDMA_TEST_RF_SOC;
    testObj->runFlag |= UDMA_TEST_RF_CORE;
    testObj->runFlag |= UDMA_TEST_RF_CFG_DEF;
#if defined (UDMA_UT_DYNAMIC_ANALYSIS)
    testObj->runFlag |= UDMA_TEST_RF_CFG_DYN;
#endif

    /* Mark all test cases as not run and set result to PASS */
    for(testCnt = 0U; testCnt < UDMA_TEST_NUM_TESTCASES; testCnt++)
    {
        testPrms             = &gUdmaTestCases[testCnt];
        testPrms->isRun      = FALSE;
        testPrms->testResult = UDMA_SOK;
    }

    return;
}

/**
 *  \brief Return the test ID to run.
 */
static uint32_t udmaTestGetTestId(UdmaTestObj *testObj, uint32_t tcType)
{
    uint32_t        testCnt;
    static int32_t  testId = 0U;
    UdmaTestParams *testPrms;

    GT_0trace(testObj->traceMask, GT_INFO, "\r\n");
    GT_0trace(testObj->traceMask, GT_INFO,
              " --------------------------------------\n");
    GT_0trace(testObj->traceMask, GT_INFO,
              " Select test to run as per below table:\n");
    GT_0trace(testObj->traceMask, GT_INFO,
              " --------------------------------------\n");
    GT_0trace(testObj->traceMask, GT_INFO, " \n");
    for(testCnt = 0U; testCnt < UDMA_TEST_NUM_TESTCASES; testCnt++)
    {
        testPrms = &gUdmaTestCases[testCnt];
        if(FALSE != testPrms->enableTest && (tcType & testPrms->tcType))
        {
            GT_assert(testObj->traceMask, (NULL != testPrms->tcName));
            GT_2trace(testObj->traceMask, GT_INFO,
                      "%5d: %s\n", gUdmaTestCases[testCnt].tcId,
                      gUdmaTestCases[testCnt].tcName);
        }
    }
    GT_0trace(testObj->traceMask, GT_INFO, " \n");
    GT_0trace(testObj->traceMask, GT_INFO,
              " Enter Test to Run (Use UART1 console for all cores or MCU_UART1 console for MCU) \n");

    while(1U)
    {
        testId = AppUtils_getNum();
        GT_1trace(testObj->traceMask, GT_INFO, "%d\n", testId);
        for(testCnt = 0U; testCnt < UDMA_TEST_NUM_TESTCASES; testCnt++)
        {
            if((tcType & UDMA_TCT_FULL) && (tcType != UDMA_TCT_ALL))
            {
                if(0U == testId)
                {
                    break;
                }
            }
            if(testId == gUdmaTestCases[testCnt].tcId)
            {
                break;
            }
        }

        if(testCnt == UDMA_TEST_NUM_TESTCASES)
        {
            GT_0trace(testObj->traceMask, GT_INFO,
                      "Invalid Test ID. Enter Again!!\n");
        }
        else
        {
            break;
        }
    }

    return (testId);
}

void udmaDrvPrint(const char *str)
{
    DebugP_log(str);

    return;
}
