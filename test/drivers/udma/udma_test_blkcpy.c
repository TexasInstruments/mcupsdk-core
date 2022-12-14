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
 *  \file udma_test_blkcpy.c
 *
 *  \brief UDMA block copy test case file.
 *
 */

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <udma_test.h>

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                         Structure Declarations                             */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

static int32_t udmaTestBlkcpyTestLoop(UdmaTestTaskObj *taskObj, uint32_t pauseTest, uint32_t chainTest);
static int32_t udmaTestBlkcpyTest(UdmaTestTaskObj *taskObj, uint32_t pauseTest, uint32_t chainTest);

static int32_t udmaTestBlkCpyRingPrimeLcdmaTestLoop(UdmaTestTaskObj *taskObj);

static int32_t udmaTestBlkcpyCreate(UdmaTestTaskObj *taskObj, uint32_t chainTest);
static int32_t udmaTestBlkcpyDelete(UdmaTestTaskObj *taskObj, uint32_t chainTest);
static int32_t udmaTestBlkcpyAlloc(UdmaTestTaskObj *taskObj);
static int32_t udmaTestBlkcpyFree(UdmaTestTaskObj *taskObj);
static int32_t udmaTestBlkcpyCompareData(UdmaTestTaskObj *taskObj,
                                         UdmaTestChObj *chObj);

static void udmaTestBlkcpyEventDmaCb(Udma_EventHandle eventHandle,
                                     uint32_t eventType,
                                     void *appData);

static void udmaTestBlkcpyTrpdInit(UdmaTestChObj *chObj, uint32_t qCnt);

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */

/* Global test pass/fail flag */
static volatile int32_t gUdmaTestBlkcpyResult = UDMA_SOK;

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

int32_t udmaTestBlkcpyTc(UdmaTestTaskObj *taskObj)
{
    int32_t         retVal = UDMA_SOK;

    GT_1trace(taskObj->traceMask, GT_INFO1,
              " |TEST INFO|:: Task:%d: Block Copy Testcase ::\r\n", taskObj->taskId);
    GT_2trace(taskObj->traceMask, GT_INFO1,
              " |TEST INFO|:: Task:%d: Num channels         : %d ::\r\n", taskObj->taskId, taskObj->numCh);
    GT_2trace(taskObj->traceMask, GT_INFO1,
              " |TEST INFO|:: Task:%d: Loop count           : %d ::\r\n", taskObj->taskId, taskObj->loopCnt);
    GT_2trace(taskObj->traceMask, GT_INFO1,
              " |TEST INFO|:: Task:%d: Pacing time          : %d ms ::\r\n", taskObj->taskId, taskObj->pacingTime);

    retVal = udmaTestBlkcpyAlloc(taskObj);
    retVal += udmaTestBlkcpyCreate(taskObj, FALSE);

    Utils_prfTsBegin(taskObj->prfTsHandle);
    retVal += udmaTestBlkcpyTestLoop(taskObj, FALSE, FALSE);
    Utils_prfTsEnd(taskObj->prfTsHandle, (taskObj->loopCnt * taskObj->numCh));

    retVal += udmaTestBlkcpyDelete(taskObj, FALSE);
    retVal += udmaTestBlkcpyFree(taskObj);

    retVal += gUdmaTestBlkcpyResult;

    return (retVal);
}

int32_t udmaTestBlkcpyPauseResumeTc(UdmaTestTaskObj *taskObj)
{
    int32_t         retVal = UDMA_SOK;

    GT_1trace(taskObj->traceMask, GT_INFO1,
              " |TEST INFO|:: Task:%d: Block Copy Pause and Resume Testcase ::\r\n", taskObj->taskId);
    GT_2trace(taskObj->traceMask, GT_INFO1,
              " |TEST INFO|:: Task:%d: Num channels         : %d ::\r\n", taskObj->taskId, taskObj->numCh);
    GT_2trace(taskObj->traceMask, GT_INFO1,
              " |TEST INFO|:: Task:%d: Loop count           : %d ::\r\n", taskObj->taskId, taskObj->loopCnt);
    GT_2trace(taskObj->traceMask, GT_INFO1,
              " |TEST INFO|:: Task:%d: Pacing time          : %d ms ::\r\n", taskObj->taskId, taskObj->pacingTime);

    retVal = udmaTestBlkcpyAlloc(taskObj);
    retVal += udmaTestBlkcpyCreate(taskObj, FALSE);

    Utils_prfTsBegin(taskObj->prfTsHandle);
    retVal += udmaTestBlkcpyTestLoop(taskObj, TRUE, FALSE);
    Utils_prfTsEnd(taskObj->prfTsHandle, (taskObj->loopCnt * taskObj->numCh));

    retVal += udmaTestBlkcpyDelete(taskObj, FALSE);
    retVal += udmaTestBlkcpyFree(taskObj);

    retVal += gUdmaTestBlkcpyResult;

    return (retVal);
}

int32_t udmaTestBlkcpyChainingTc(UdmaTestTaskObj *taskObj)
{
    int32_t         retVal = UDMA_SOK;

    GT_1trace(taskObj->traceMask, GT_INFO1,
              " |TEST INFO|:: Task:%d: Block Copy Testcase ::\r\n", taskObj->taskId);
    GT_2trace(taskObj->traceMask, GT_INFO1,
              " |TEST INFO|:: Task:%d: Num channels         : %d ::\r\n", taskObj->taskId, taskObj->numCh);
    GT_2trace(taskObj->traceMask, GT_INFO1,
              " |TEST INFO|:: Task:%d: Loop count           : %d ::\r\n", taskObj->taskId, taskObj->loopCnt);
    GT_2trace(taskObj->traceMask, GT_INFO1,
              " |TEST INFO|:: Task:%d: Pacing time          : %d ms ::\r\n", taskObj->taskId, taskObj->pacingTime);

    retVal = udmaTestBlkcpyAlloc(taskObj);
    retVal += udmaTestBlkcpyCreate(taskObj, TRUE);

    Utils_prfTsBegin(taskObj->prfTsHandle);
    retVal += udmaTestBlkcpyTestLoop(taskObj, FALSE, TRUE);
    Utils_prfTsEnd(taskObj->prfTsHandle, (taskObj->loopCnt * taskObj->numCh));

    retVal += udmaTestBlkcpyDelete(taskObj, TRUE);
    retVal += udmaTestBlkcpyFree(taskObj);

    retVal += gUdmaTestBlkcpyResult;

    return (retVal);
}

int32_t udmaTestBlkCpyRingPrimeLcdmaTest(UdmaTestTaskObj *taskObj)
{
    int32_t     retVal = UDMA_SOK;
    uint32_t    loopCnt = 0U;

    retVal = udmaTestBlkcpyAlloc(taskObj);
    retVal += udmaTestBlkcpyCreate(taskObj, FALSE);

    while(loopCnt < taskObj->loopCnt)
    {
        /* Perform block copy with ring prime */
        retVal = udmaTestBlkCpyRingPrimeLcdmaTestLoop(taskObj);
        if(UDMA_SOK != retVal)
        {
            break;
        }

        loopCnt++;
    }

    retVal += udmaTestBlkcpyDelete(taskObj, FALSE);
    retVal += udmaTestBlkcpyFree(taskObj);

    retVal += gUdmaTestBlkcpyResult;

    return (retVal);
}

static int32_t udmaTestBlkcpyTestLoop(UdmaTestTaskObj *taskObj, uint32_t pauseTest, uint32_t chainTest)
{
    int32_t         retVal = UDMA_SOK;
    uint32_t        loopCnt = 0U;
    uint32_t        chCnt, qCnt;
    UdmaTestChObj  *chObj;
    uint32_t        hrs, mins, secs, durationInSecs, msecs;
    uint32_t        startTime, elapsedTime, expectedTime;

    startTime = AppUtils_getCurTimeInMsec();
    while(loopCnt < taskObj->loopCnt)
    {
        if(TRUE == taskObj->testPrms->dcEnable)
        {
            /* Reset dest buffers */
            for(chCnt = 0U ; chCnt < taskObj->numCh; chCnt++)
            {
                chObj = taskObj->chObj[chCnt];
                GT_assert(taskObj->traceMask, chObj != NULL);

                for(qCnt = 0U; qCnt < chObj->qdepth; qCnt++)
                {
                    memset(chObj->destBuf[qCnt], 0, chObj->destBufSize);
                    /* Writeback destination buffer */
                    CacheP_wb(chObj->destBuf[qCnt], chObj->destBufSize, CacheP_TYPE_ALLD);
                }
            }
        }

        /* Perform UDMA memcpy */
        retVal = udmaTestBlkcpyTest(taskObj, pauseTest, chainTest);
        if(UDMA_SOK != retVal)
        {
            break;
        }

        loopCnt++;

        if(taskObj->pacingTime != PACING_NONE)
        {
            while(1U)
            {
                elapsedTime = AppUtils_getElapsedTimeInMsec(startTime);
                expectedTime = loopCnt * taskObj->pacingTime;
                if(elapsedTime >= expectedTime)
                {
                    break;
                }
                ClockP_usleep((expectedTime - elapsedTime) * 1000U);
            }
        }
    }

    elapsedTime = AppUtils_getElapsedTimeInMsec(startTime);
    durationInSecs = ((elapsedTime) / 1000U);
    hrs  = durationInSecs / (60U * 60U);
    mins = (durationInSecs / 60U) - (hrs * 60U);
    secs = durationInSecs - (hrs * 60U * 60U) - (mins * 60U);
    msecs = elapsedTime - (((hrs * 60U * 60U) + (mins * 60U) + secs) * 1000U);
    GT_5trace(taskObj->traceMask, GT_INFO1,
              " |Block Copy TEST DURATION|: Task:%d: %d:%0.2d:%0.2d:%0.3d \r\n",
              taskObj->taskId, hrs, mins, secs, msecs);
    if(TRUE == taskObj->testPrms->prfEnable)
    {
        udmaTestCalcPerformance(taskObj, elapsedTime);
    }

    return (retVal);
}

static int32_t udmaTestBlkcpyTest(UdmaTestTaskObj *taskObj, uint32_t pauseTest, uint32_t chainTest)
{
    int32_t         retVal = UDMA_SOK;
    uint32_t       *pTrResp, trRespStatus;
    void           *trpdMem;
    uint64_t        pDesc = 0;
    uint32_t        chCnt, qCnt;
    uint32_t        triggerCnt, tCnt;
    UdmaTestChObj  *chObj;
    volatile uint64_t   intrStatusReg;

    /* Update TR packet descriptor */
    for(chCnt = 0U ; chCnt < taskObj->numCh; chCnt++)
    {
        chObj = taskObj->chObj[chCnt];
        GT_assert(taskObj->traceMask, chObj != NULL);
        for(qCnt = 0U; qCnt < chObj->qdepth; qCnt++)
        {
            udmaTestBlkcpyTrpdInit(chObj, qCnt);
        }
    }

    /* Submit TRPD to channel */
    for(chCnt = 0U ; chCnt < taskObj->numCh; chCnt++)
    {
        chObj = taskObj->chObj[chCnt];
        GT_assert(taskObj->traceMask, chObj != NULL);

        /* Pause the channel before transfer */
        if(TRUE == pauseTest)
        {
            retVal = Udma_chPause(chObj->chHandle);
            if(UDMA_SOK != retVal)
            {
                GT_0trace(taskObj->traceMask, GT_ERR, " Channel pause failed!!\n");
                break;
            }
        }

        for(qCnt = 0U; qCnt < chObj->qdepth; qCnt++)
        {
            retVal = Udma_ringQueueRaw(
                         Udma_chGetFqRingHandle(chObj->chHandle),
                         Udma_defaultVirtToPhyFxn(chObj->trpdMem[qCnt], 0U, NULL));
            if(UDMA_SOK != retVal)
            {
                GT_0trace(taskObj->traceMask, GT_ERR, " Channel queue failed!!\n");
                break;
            }
            else
            {
                chObj->queueCnt++;
            }

            if(CSL_UDMAP_TR_FLAGS_TRIGGER_NONE != chObj->chPrms->trigger)
            {
                /* Set number of times to trigger based on event size */
                triggerCnt = 1U;
                if(CSL_UDMAP_TR_FLAGS_EVENT_SIZE_ICNT1_DEC == chObj->chPrms->eventSize)
                {
                    triggerCnt = chObj->icnt[1];
                }
                if(CSL_UDMAP_TR_FLAGS_EVENT_SIZE_ICNT2_DEC == chObj->chPrms->eventSize)
                {
                    triggerCnt = chObj->icnt[2];
                }
                if(CSL_UDMAP_TR_FLAGS_EVENT_SIZE_ICNT3_DEC == chObj->chPrms->eventSize)
                {
                    triggerCnt = chObj->icnt[3];
                }

                for(tCnt = 0U; tCnt < triggerCnt; tCnt++)
                {
                    /* Set channel trigger - otherwise transfer won't happen */
                    retVal = Udma_chSetSwTrigger(chObj->chHandle, chObj->chPrms->trigger);
                    if(UDMA_SOK != retVal)
                    {
                        GT_0trace(taskObj->traceMask, GT_ERR, " Channel trigger failed!!\n");
                        break;
                    }

                    /* Incase TR event is set, wait for TR event */
                    if(NULL != chObj->trEventHandle)
                    {
                        while(1U)
                        {
                            intrStatusReg = CSL_REG64_RD(chObj->trEventPrms.intrStatusReg);
                            if(intrStatusReg & chObj->trEventPrms.intrMask)
                            {
                                /* Clear interrupt */
                                CSL_REG64_WR(chObj->trEventPrms.intrClearReg, chObj->trEventPrms.intrMask);
                                break;
                            }
                        #if !defined (UDMA_UT_BAREMETAL)
                            TaskP_yield();
                        #endif
                        }
                    }
                }
                if(UDMA_SOK != retVal)
                {
                    break;
                }
            }
        }
        if(UDMA_SOK != retVal)
        {
            break;
        }
    }

    if(UDMA_SOK == retVal)
    {
        if(TRUE == pauseTest)
        {
            /* Sleep for sometime and check if the transfer has completed;
             * if so then it is an error */
            ClockP_sleep(1); /* 1 sec sleep */

            for(chCnt = 0U ; chCnt < taskObj->numCh; chCnt++)
            {
                chObj = taskObj->chObj[chCnt];
                GT_assert(taskObj->traceMask, chObj != NULL);

                /* Check ring occupancy */
                retVal =
                        Udma_ringDequeueRaw(Udma_chGetCqRingHandle(chObj->chHandle), &pDesc);
                if(UDMA_SOK == retVal)
                {
                    GT_0trace(taskObj->traceMask, GT_ERR,
                        " Channel completed when it should not!! Pause didnot work!!\n");
                    retVal = UDMA_EFAIL;
                    break;
                }
                else
                {
                    /* Not an error; continue */
                    retVal = UDMA_SOK;
                }

                /* Resume channel */
                retVal = Udma_chResume(chObj->chHandle);
                if(UDMA_SOK != retVal)
                {
                    GT_0trace(taskObj->traceMask, GT_ERR, " Channel resume failed!!\n");
                    break;
                }
            }
        }
    }

    if(UDMA_SOK == retVal)
    {
        for(chCnt = 0U ; chCnt < taskObj->numCh; chCnt++)
        {
            chObj = taskObj->chObj[chCnt];
            GT_assert(taskObj->traceMask, chObj != NULL);

            if(UDMA_TEST_EVENT_INTR == chObj->chPrms->eventMode)
            {
                if ((chainTest == FALSE) ||
                    ((chainTest == TRUE) && (chCnt == (taskObj->numCh - 1))))
                {
                    for(qCnt = 0U; qCnt < chObj->qdepth; qCnt++)
                    {
                        /* Wait for return descriptor in completion ring */
                        SemaphoreP_pend(&chObj->transferDoneSem, SystemP_WAIT_FOREVER);
                    }
                }
            }
        }
    }

    if(UDMA_SOK == retVal)
    {
        for(chCnt = 0U ; chCnt < taskObj->numCh; chCnt++)
        {
            chObj = taskObj->chObj[chCnt];
            GT_assert(taskObj->traceMask, chObj != NULL);

            for(qCnt = 0U; qCnt < chObj->qdepth; qCnt++)
            {
                trpdMem  = (void *) chObj->trpdMem[qCnt];

                if(UDMA_TEST_EVENT_INTR == chObj->chPrms->eventMode)
                {
                    /* Response received in completion queue */
                    retVal =
                        Udma_ringDequeueRaw(Udma_chGetCqRingHandle(chObj->chHandle), &pDesc);
                    if(UDMA_SOK != retVal)
                    {
                        GT_0trace(taskObj->traceMask, GT_ERR,
                            " No descriptor after callback!!\n");
                        retVal = UDMA_EFAIL;
                    }
                }
                else
                {
                    while(1U)
                    {
                        /* Wait for Response received in completion queue */
                        retVal =
                            Udma_ringDequeueRaw(Udma_chGetCqRingHandle(chObj->chHandle), &pDesc);
                        if(UDMA_SOK == retVal)
                        {
                            break;
                        }

                    #if !defined (UDMA_UT_BAREMETAL)
                        TaskP_yield();
                    #endif
                    }
                }

                if(UDMA_SOK == retVal)
                {
                    chObj->dequeueCnt++;
                    /*
                     * Sanity check
                     */
                    /* Check returned descriptor pointer */
                    if(Udma_defaultPhyToVirtFxn(pDesc, 0U, NULL) != trpdMem)
                    {
                        GT_0trace(taskObj->traceMask, GT_ERR,
                            " TR descriptor pointer returned doesn't match the submitted address!!\n");
                        retVal = UDMA_EFAIL;
                    }
                }

                if((UDMA_SOK == retVal) && (TRUE == taskObj->testPrms->dcEnable))
                {
                    /* Invalidate cache */
                    CacheP_inv(trpdMem, chObj->trpdSize, CacheP_TYPE_ALLD);

                    /* check TR response status */
                    pTrResp = (uint32_t *) (((uint8_t *) trpdMem) + (sizeof(CSL_UdmapTR15) * 2U));
                    trRespStatus = CSL_FEXT(*pTrResp, UDMAP_TR_RESPONSE_STATUS_TYPE);
                    if(trRespStatus != CSL_UDMAP_TR_RESPONSE_STATUS_COMPLETE)
                    {
                        GT_0trace(taskObj->traceMask, GT_ERR,
                            " TR Response not completed!!\n");
                        retVal = UDMA_EFAIL;
                    }
                }

                if(UDMA_SOK != retVal)
                {
                    break;
                }
            }

            if((UDMA_SOK == retVal) && (TRUE == taskObj->testPrms->dcEnable))
            {
                /* Compare data */
                retVal = udmaTestBlkcpyCompareData(taskObj, chObj);
            }

            if(UDMA_SOK != retVal)
            {
                break;
            }
        }
    }

    return (retVal);
}

static int32_t udmaTestBlkCpyRingPrimeLcdmaTestLoop(UdmaTestTaskObj *taskObj)
{
    int32_t             retVal = UDMA_SOK;
    uint32_t            chCnt, qCnt;
    UdmaTestChObj      *chObj;
    Udma_RingHandle     ringHandle;
    void               *trpdMem;
    uint64_t            pDesc = 0;
    uint32_t            elemCnt, ringMemSize, rOcc;
    void               *ringMem = NULL;

    /* Update TR packet descriptor */
    for(chCnt = 0U ; chCnt < taskObj->numCh; chCnt++)
    {
        chObj = taskObj->chObj[chCnt];
        GT_assert(taskObj->traceMask, chObj != NULL);
        for(qCnt = 0U; qCnt < chObj->qdepth; qCnt++)
        {
            udmaTestBlkcpyTrpdInit(chObj, qCnt);
        }
    }

    for(chCnt = 0U ; chCnt < taskObj->numCh; chCnt++)
    {
        chObj       = taskObj->chObj[chCnt];
        ringHandle  = Udma_chGetFqRingHandle(chObj->chHandle);
        elemCnt     = chObj->qdepth;
        ringMem     = chObj->fqRingMem;
        ringMemSize = chObj->ringMemSize;

        /* Check ring mode after allocation */
        if (Udma_ringGetMode(ringHandle) != TISCI_MSG_VALUE_RM_RING_MODE_RING)
        {
            GT_0trace   (taskObj->traceMask, GT_ERR, " Ring mode mismatch!!\n");
            retVal = UDMA_EFAIL;
            break;
        }

        /* Check ring elements count after allocation */
        if (Udma_ringGetElementCnt(ringHandle) != elemCnt)
        {
            GT_0trace(taskObj->traceMask, GT_ERR, " Ring elements count mismatch!!\n");
            retVal = UDMA_EFAIL;
            break;
        }

        /* Check wrIdx of ring after allocation */
        if (Udma_ringGetWrIdx(ringHandle) != 0U)
        {
            GT_0trace(taskObj->traceMask, GT_ERR, " Ring rwIdx value mismatch!!\n");
            retVal = UDMA_EFAIL;
            break;
        }

        /* Queue through prime API */
        for(qCnt = 0U; qCnt < elemCnt; qCnt++)
        {
            Udma_ringPrime(ringHandle,
                           Udma_defaultVirtToPhyFxn(chObj->trpdMem[qCnt], 0U, NULL));
        }

        /* Check if the HW occupancy is zero as the queue is not committed */
        if(udmaTestCompareRingHwOccDriver(ringHandle, 0U, UDMA_TEST_RING_ACC_DIRECTION_FORWARD) != UDMA_SOK)
        {
            GT_0trace(taskObj->traceMask, GT_ERR, " Ring element count mismatch!!\n");
            retVal = UDMA_EFAIL;
            break;
        }

        /* Pause the channel before setting the doorbell, to verify the Forward Occupancy.
         * Else, as soon as doorbell is set, transfer happens and FOCC can't be verified. */
        retVal = Udma_chPause(chObj->chHandle);
        if(UDMA_SOK != retVal)
        {
            GT_0trace(taskObj->traceMask, GT_ERR, " Channel pause failed!!\n");
            break;
        }

        /* Do Cache flush and commit to ring */
        CacheP_wb(ringMem, ringMemSize, CacheP_TYPE_ALLD);
        Udma_ringSetDoorBell(ringHandle, elemCnt);

        /* Check if the HW occupancy is same as what is queued */
        if(udmaTestCompareRingHwOccDriver(ringHandle, elemCnt, UDMA_TEST_RING_ACC_DIRECTION_FORWARD) != UDMA_SOK)
        {
            GT_0trace(taskObj->traceMask, GT_ERR, " Ring element count mismatch!!\n");
            retVal = UDMA_EFAIL;
            break;
        }

        /* Check wrIdx of ring after queuing, wrIdx should be back to zero as
         * ring is full */
        if (Udma_ringGetWrIdx(ringHandle) != 0U)
        {
            GT_0trace(taskObj->traceMask, GT_ERR, " Ring wrIdx value mismatch!!\n");
            retVal = UDMA_EFAIL;
            break;
        }

        /* After verifying FOCC and wrIdx, Resume channel to start the transfer */
        retVal = Udma_chResume(chObj->chHandle);
        if(UDMA_SOK != retVal)
        {
            GT_0trace(taskObj->traceMask, GT_ERR, " Channel resume failed!!\n");
            break;
        }

        /* Wait for the reverse occupancy of the Dual ring to become elemCnt
         * (waiting for the transfer to complete),
         * before starting ring prime read.  */
        while(1)
        {
            /* Update cfg->occ count */
            rOcc = Udma_ringGetReverseRingOcc(ringHandle);

            if(elemCnt == rOcc)
            {
                break;
            }
        }

        /* Do Cache invalidate before reading ring elements */
        CacheP_inv(ringMem, ringMemSize, CacheP_TYPE_ALLD);

        /* Dequeue using prime read API */
        for(qCnt = 0U; qCnt < elemCnt; qCnt++)
        {
            trpdMem  = (void *) chObj->trpdMem[qCnt];

            Udma_ringPrimeRead(ringHandle, &pDesc);
            /* Check returned descriptor pointer */
            if(Udma_defaultPhyToVirtFxn(pDesc, 0U, NULL) != trpdMem)
            {
                GT_0trace(taskObj->traceMask, GT_ERR,
                    " TR descriptor pointer returned doesn't match the submitted address!!\n");
                retVal = UDMA_EFAIL;
                break;
            }
        }
        if(UDMA_SOK != retVal)
        {
            break;
        }

        /* Check if the HW occupancy is same as elemCnt as the queue is not committed */
        if(udmaTestCompareRingHwOccDriver(ringHandle, elemCnt, UDMA_TEST_RING_ACC_DIRECTION_REVERSE) != UDMA_SOK)
        {
            GT_0trace(taskObj->traceMask, GT_ERR, " Ring element count mismatch!!\n");
            retVal = UDMA_EFAIL;
            break;
        }

        /* Set door bell value as -1 * elemCnt to reduce ring occupancy after reading */
        Udma_ringSetDoorBell(ringHandle, (-1 * (int32_t)elemCnt));

        /* Check if the HW occupancy is zero */
        if(udmaTestCompareRingHwOccDriver(ringHandle, 0U, UDMA_TEST_RING_ACC_DIRECTION_REVERSE) != UDMA_SOK)
        {
            GT_0trace(taskObj->traceMask, GT_ERR, " Ring not empty!!\n");
            retVal = UDMA_EFAIL;
            break;
        }
    }

    return (retVal);
}

static int32_t udmaTestBlkcpyCreate(UdmaTestTaskObj *taskObj, uint32_t chainTest)
{
    int32_t             retVal = UDMA_SOK;
    uint32_t            chCnt, qCnt;
    UdmaTestChObj      *chObj;
    uint32_t            chType;
    Udma_ChPrms         chPrms;
    Udma_ChTxPrms       txPrms;
    Udma_ChRxPrms       rxPrms;
    Udma_EventHandle    eventHandle, controllerEventHandle;
    Udma_EventPrms      eventPrms;
    uint32_t           *buf;
    uint32_t            i;

    controllerEventHandle = NULL;
    for(chCnt = 0U ; chCnt < taskObj->numCh; chCnt++)
    {
        chObj = taskObj->chObj[chCnt];
        GT_assert(taskObj->traceMask, chObj != NULL);

        /* Init buffers */
        buf = (uint32_t *) chObj->srcBuf;
        for(i = 0U; i < ((chObj->srcBufSize) / sizeof(uint32_t)); i++)
        {
            buf[i] = i + chObj->chIdx;
        }

        /* Writeback source buffer */
        CacheP_wb(chObj->srcBuf, chObj->srcBufSize, CacheP_TYPE_ALLD);

        for(qCnt = 0U; qCnt < chObj->qdepth; qCnt++)
        {
            memset(chObj->destBuf[qCnt], 0, chObj->destBufSize);
            /* Writeback destination buffer */
            CacheP_wb(chObj->destBuf[qCnt], chObj->destBufSize, CacheP_TYPE_ALLD);
        }

        retVal = SemaphoreP_constructBinary(&chObj->transferDoneSem, 0);
        if(SystemP_SUCCESS != retVal)
        {
            GT_0trace(taskObj->traceMask, GT_ERR, " Sem create failed!!\n");
        }

        if(UDMA_SOK == retVal)
        {
            /* Init channel parameters */
            chType = chObj->chPrms->chType;
            UdmaChPrms_init(&chPrms, chType);
            chPrms.fqRingPrms.ringMem   = chObj->fqRingMem;
            chPrms.cqRingPrms.ringMem   = chObj->cqRingMem;
            chPrms.tdCqRingPrms.ringMem = chObj->tdCqRingMem;
            chPrms.fqRingPrms.ringMemSize   = chObj->ringMemSize;
            chPrms.cqRingPrms.ringMemSize   = chObj->ringMemSize;
            chPrms.tdCqRingPrms.ringMemSize = chObj->ringMemSize;
            chPrms.fqRingPrms.elemCnt   = chObj->qdepth;
            chPrms.cqRingPrms.elemCnt   = chObj->qdepth;
            chPrms.tdCqRingPrms.elemCnt = chObj->qdepth;

            /* Open channel for block copy */
            retVal = Udma_chOpen(chObj->drvHandle, &chObj->drvChObj, chType, &chPrms);
            if(UDMA_SOK != retVal)
            {
                GT_0trace(taskObj->traceMask, GT_ERR,
                    " UDMA channel open failed!!\n");
            }
            else
            {
                chObj->chHandle = &chObj->drvChObj;
                GT_3trace(taskObj->traceMask, GT_INFO1,
                          " |TEST INFO|:: Task:%d: CH:%d: Allocated Ch   : %d ::\r\n",
                          taskObj->taskId, chObj->chIdx, Udma_chGetNum(chObj->chHandle));
            }
        }

        if(UDMA_SOK == retVal)
        {
            /* Config TX channel */
            if(NULL == chObj->txPrms)
            {
                UdmaChTxPrms_init(&txPrms, chType);
                chObj->txPrms = &txPrms;
            }
            retVal = Udma_chConfigTx(chObj->chHandle, chObj->txPrms);
            if(UDMA_SOK != retVal)
            {
                GT_0trace(taskObj->traceMask, GT_ERR,
                    " UDMA TX channel config failed!!\n");
            }
        }

        if(UDMA_SOK == retVal)
        {
            /* Config RX channel - which is implicitly paired to TX channel in
             * block copy mode */
            if(NULL == chObj->rxPrms)
            {
                UdmaChRxPrms_init(&rxPrms, chType);
                chObj->rxPrms = &rxPrms;
            }
            retVal = Udma_chConfigRx(chObj->chHandle, chObj->rxPrms);
            if(UDMA_SOK != retVal)
            {
                GT_0trace(taskObj->traceMask, GT_ERR,
                    " UDMA RX channel config failed!!\n");
            }
        }

        if((UDMA_SOK == retVal) && (UDMA_TEST_EVENT_NONE != chObj->chPrms->eventMode))
        {
            if((chainTest == FALSE) ||
               ((chainTest == TRUE) && (chCnt == taskObj->numCh - 1)))
            {
                /* Register ring completion callback */
                /* In case of chaining test, register ring completion only for last channel. */
                eventHandle = &chObj->cqEventObj;
                UdmaEventPrms_init(&eventPrms);
                eventPrms.eventType         = UDMA_EVENT_TYPE_DMA_COMPLETION;
                eventPrms.eventMode         = UDMA_EVENT_MODE_SHARED;
                eventPrms.chHandle          = chObj->chHandle;
                eventPrms.controllerEventHandle = controllerEventHandle;
                if(UDMA_TEST_EVENT_INTR == chObj->chPrms->eventMode)
                {
                    eventPrms.controllerEventHandle = Udma_eventGetGlobalHandle(chObj->drvHandle);
                    eventPrms.eventCb           = &udmaTestBlkcpyEventDmaCb;
                    eventPrms.appData           = chObj;
                }
                retVal = Udma_eventRegister(chObj->drvHandle, eventHandle, &eventPrms);
                if(UDMA_SOK != retVal)
                {
                    GT_0trace(taskObj->traceMask, GT_ERR,
                        " UDMA CQ event register failed!!\n");
                }
                else
                {
                    chObj->cqEventHandle = eventHandle;
                }
                if(NULL == controllerEventHandle)
                {
                    /* Which ever event gets registered first is the master event */
                    controllerEventHandle = eventHandle;
                }
            }
        }

        if((UDMA_SOK == retVal) && (CSL_UDMAP_TR_FLAGS_EVENT_SIZE_COMPLETION != chObj->chPrms->eventSize))
        {
            /* Register TR event */
            eventHandle = &chObj->trEventObj;
            UdmaEventPrms_init(&chObj->trEventPrms);
            chObj->trEventPrms.eventType         = UDMA_EVENT_TYPE_TR;
            chObj->trEventPrms.eventMode         = UDMA_EVENT_MODE_SHARED;
            chObj->trEventPrms.chHandle          = chObj->chHandle;
            chObj->trEventPrms.controllerEventHandle = controllerEventHandle;
            chObj->trEventPrms.eventCb           = NULL;
            chObj->trEventPrms.appData           = NULL;
            retVal = Udma_eventRegister(chObj->drvHandle, eventHandle, &chObj->trEventPrms);
            if(UDMA_SOK != retVal)
            {
                GT_0trace(taskObj->traceMask, GT_ERR,
                    " UDMA TR event register failed!!\n");
            }
            else
            {
                chObj->trEventHandle = eventHandle;
            }
        }

        if(UDMA_SOK == retVal)
        {
            /* Channel enable */
            retVal = Udma_chEnable(chObj->chHandle);
            if(UDMA_SOK != retVal)
            {
                GT_0trace(taskObj->traceMask, GT_ERR,
                    " UDMA channel enable failed!!\n");
            }
        }

        if(UDMA_SOK != retVal)
        {
            break;
        }
    }

    if((UDMA_SOK == retVal) && (chainTest == TRUE))
    {
        for(chCnt = 0U ; chCnt < taskObj->numCh; chCnt++)
        {
            chObj = taskObj->chObj[chCnt];
            if(chCnt < (taskObj->numCh - 1))
            {
                Udma_ChHandle chainedChHandle;

                chainedChHandle = taskObj->chObj[chCnt + 1]->chHandle;
                retVal = Udma_chSetChaining(
                             chObj->chHandle,
                             chainedChHandle,
                             CSL_UDMAP_TR_FLAGS_TRIGGER_GLOBAL0);
                if(UDMA_SOK != retVal)
                {
                    GT_0trace(taskObj->traceMask, GT_ERR,
                        "[Error] UDMA channel chaining failed!!\n");
                }
            }
            if(UDMA_SOK != retVal)
            {
                break;
            }
        }
    }

    return (retVal);
}

static int32_t udmaTestBlkcpyDelete(UdmaTestTaskObj *taskObj, uint32_t chainTest)
{
    int32_t         retVal = UDMA_SOK, tempRetVal;
    uint64_t        pDesc;
    uint32_t        chCnt;
    UdmaTestChObj  *chObj;

    for(chCnt = 0U ; chCnt < taskObj->numCh; chCnt++)
    {
        chObj = taskObj->chObj[chCnt];
        GT_assert(taskObj->traceMask, chObj != NULL);

        retVal = Udma_chDisable(chObj->chHandle, UDMA_DEFAULT_CH_DISABLE_TIMEOUT);
        if(UDMA_SOK != retVal)
        {
            GT_0trace(taskObj->traceMask, GT_ERR,
                " UDMA channel disable failed!!\n");
        }

        /* Flush any pending request from the free queue */
        while(1)
        {
            tempRetVal = Udma_ringFlushRaw(
                             Udma_chGetFqRingHandle(chObj->chHandle), &pDesc);
            if(UDMA_ETIMEOUT == tempRetVal)
            {
                break;
            }

        #if !defined (UDMA_UT_BAREMETAL)
            TaskP_yield();
        #endif
        }

        if(chainTest == TRUE)
        {
            if(chCnt < (taskObj->numCh - 1))
            {
                Udma_ChHandle chainedChHandle;

                chainedChHandle = taskObj->chObj[chCnt + 1]->chHandle;
                retVal = Udma_chBreakChaining(
                             chObj->chHandle,
                             chainedChHandle);
                if(UDMA_SOK != retVal)
                {
                    GT_0trace(taskObj->traceMask, GT_ERR,
                        "[Error] UDMA channel break chaining failed!!\n");
                }
            }
        }
    }

    for(chCnt = 0U ; chCnt < taskObj->numCh; chCnt++)
    {
        chObj = taskObj->chObj[chCnt];
        /* Unregister master event at the end - CQ is the master event */
        if(NULL != chObj->trEventHandle)
        {
            retVal += Udma_eventUnRegister(chObj->trEventHandle);
            if(UDMA_SOK != retVal)
            {
                GT_0trace(taskObj->traceMask, GT_ERR,
                    " UDMA TR event unregister failed!!\n");
            }
            chObj->trEventHandle = NULL;
        }
        if(NULL != chObj->tdCqEventHandle)
        {
            retVal += Udma_eventUnRegister(chObj->tdCqEventHandle);
            if(UDMA_SOK != retVal)
            {
                GT_0trace(taskObj->traceMask, GT_ERR,
                    " UDMA TDCQ event unregister failed!!\n");
            }
            chObj->tdCqEventHandle = NULL;
        }
        if(NULL != chObj->cqEventHandle)
        {
            retVal += Udma_eventUnRegister(chObj->cqEventHandle);
            if(UDMA_SOK != retVal)
            {
                GT_0trace(taskObj->traceMask, GT_ERR,
                    " UDMA CQ event unregister failed!!\n");
            }
            chObj->cqEventHandle = NULL;
        }

        retVal += Udma_chClose(chObj->chHandle);
        if(UDMA_SOK != retVal)
        {
            GT_0trace(taskObj->traceMask, GT_ERR,
                " UDMA channel close failed!!\n");
        }

        SemaphoreP_destruct(&chObj->transferDoneSem);
    }

    return (retVal);
}

static int32_t udmaTestBlkcpyAlloc(UdmaTestTaskObj *taskObj)
{
    int32_t         retVal = UDMA_SOK;
    uint32_t        chCnt, qCnt;
    UdmaTestChObj  *chObj;
    static char    *heapStr[] = {"MSMC", "DDR", "Internal"};
    static char    *instanceIdStr[] = {"MAIN", "MCU", "BCDMA", "PKTDMA"};
    static char    *triggerStr[] = {"None", "Global0", "Global1", "Local"};
    static char    *eventModeStr[] = {"None(Ring Dequeue Polling)", "Interrupt(IA+IR)", "Polling(IA)"};

    /* Create TimeStamp object */
    taskObj->prfTsHandle = Utils_prfTsCreate(taskObj->prfTsName);
    GT_assert(taskObj->traceMask, (taskObj->prfTsHandle != NULL));

    for(chCnt = 0U ; chCnt < taskObj->numCh; chCnt++)
    {
        chObj = taskObj->chObj[chCnt];
        GT_assert(taskObj->traceMask, chObj != NULL);
        GT_assert(taskObj->traceMask, chObj->chPrms != NULL);

        GT_3trace(taskObj->traceMask, GT_INFO1,
                " |TEST INFO|:: Task:%d: CH:%d: Instance ID    : %s ::\r\n",
                taskObj->taskId, chObj->chIdx, instanceIdStr[chObj->instId]);
        GT_2trace(taskObj->traceMask, GT_INFO1,
                " |TEST INFO|:: Task:%d: CH:%d: Channel Type   : UDMAP BLKCOPY ::\r\n",
                taskObj->taskId, chObj->chIdx);
        GT_3trace(taskObj->traceMask, GT_INFO1,
                  " |TEST INFO|:: Task:%d: CH:%d: Event Mode     : %s ::\r\n",
                  taskObj->taskId, chObj->chIdx, eventModeStr[chObj->chPrms->eventMode]);
        GT_3trace(taskObj->traceMask, GT_INFO1,
                  " |TEST INFO|:: Task:%d: CH:%d: Qdepth         : %d ::\r\n",
                  taskObj->taskId, chObj->chIdx, chObj->qdepth);
        GT_6trace(taskObj->traceMask, GT_INFO1,
                  " |TEST INFO|:: Task:%d: CH:%d: icnt           : %d x %d x %d x %d ::\r\n",
                  taskObj->taskId, chObj->chIdx,
                  chObj->icnt[0], chObj->icnt[1], chObj->icnt[2], chObj->icnt[3]);
        GT_6trace(taskObj->traceMask, GT_INFO1,
                  " |TEST INFO|:: Task:%d: CH:%d: dicnt          : %d x %d x %d x %d ::\r\n",
                  taskObj->taskId, chObj->chIdx,
                  chObj->dicnt[0], chObj->dicnt[1], chObj->dicnt[2], chObj->dicnt[3]);
        GT_5trace(taskObj->traceMask, GT_INFO1,
                  " |TEST INFO|:: Task:%d: CH:%d: dim            : %d, %d, %d ::\r\n",
                  taskObj->taskId, chObj->chIdx,
                  chObj->dim[0], chObj->dim[1], chObj->dim[2]);
        GT_5trace(taskObj->traceMask, GT_INFO1,
                  " |TEST INFO|:: Task:%d: CH:%d: ddim           : %d, %d, %d ::\r\n",
                  taskObj->taskId, chObj->chIdx,
                  chObj->ddim[0], chObj->ddim[1], chObj->ddim[2]);
        GT_3trace(taskObj->traceMask, GT_INFO1,
                  " |TEST INFO|:: Task:%d: CH:%d: Src Heap ID    : %s ::\r\n",
                  taskObj->taskId, chObj->chIdx, heapStr[chObj->heapIdSrc]);
        GT_3trace(taskObj->traceMask, GT_INFO1,
                  " |TEST INFO|:: Task:%d: CH:%d: Dest Heap ID   : %s ::\r\n",
                  taskObj->taskId, chObj->chIdx, heapStr[chObj->heapIdDest]);
        GT_3trace(taskObj->traceMask, GT_INFO1,
                  " |TEST INFO|:: Task:%d: CH:%d: Trigger        : %s ::\r\n",
                  taskObj->taskId, chObj->chIdx, triggerStr[chObj->chPrms->trigger]);

        chObj->ringMemSize = chObj->qdepth * sizeof(uint64_t);
        chObj->trpdSize = (sizeof(CSL_UdmapTR15) * 2U) + 4U;

        chObj->fqRingMem = Utils_memAlloc(chObj->heapIdSrc, chObj->ringMemSize, UDMA_CACHELINE_ALIGNMENT);
        if(NULL == chObj->fqRingMem)
        {
            retVal = UDMA_EALLOC;
            GT_0trace(taskObj->traceMask, GT_ERR,
                " FQ ring memory allocation failure\r\n");
        }
        chObj->cqRingMem = Utils_memAlloc(chObj->heapIdSrc, chObj->ringMemSize, UDMA_CACHELINE_ALIGNMENT);
        if(NULL == chObj->cqRingMem)
        {
            retVal = UDMA_EALLOC;
            GT_0trace(taskObj->traceMask, GT_ERR,
                " CQ ring memory allocation failure\r\n");
        }
        chObj->tdCqRingMem = Utils_memAlloc(chObj->heapIdSrc, chObj->ringMemSize, UDMA_CACHELINE_ALIGNMENT);
        if(NULL == chObj->tdCqRingMem)
        {
            retVal = UDMA_EALLOC;
            GT_0trace(taskObj->traceMask, GT_ERR,
                " TD CQ ring memory allocation failure\r\n");
        }
        chObj->srcBuf = Utils_memAlloc(chObj->heapIdSrc, chObj->srcBufSize, UDMA_CACHELINE_ALIGNMENT);
        if(NULL == chObj->srcBuf)
        {
            retVal = UDMA_EALLOC;
            GT_0trace(taskObj->traceMask, GT_ERR,
                " Source buffer memory allocation failure\r\n");
        }
        for(qCnt = 0U; qCnt < chObj->qdepth; qCnt++)
        {
            chObj->trpdMem[qCnt] = Utils_memAlloc(chObj->heapIdSrc, chObj->trpdSize, UDMA_CACHELINE_ALIGNMENT);
            if(NULL == chObj->trpdMem[qCnt])
            {
                retVal = UDMA_EALLOC;
                GT_0trace(taskObj->traceMask, GT_ERR,
                    " TRPD memory allocation failure\r\n");
            }
            chObj->destBuf[qCnt] = Utils_memAlloc(chObj->heapIdDest, chObj->destBufSize, UDMA_CACHELINE_ALIGNMENT);
            if(NULL == chObj->destBuf[qCnt])
            {
                retVal = UDMA_EALLOC;
                GT_0trace(taskObj->traceMask, GT_ERR,
                    " Dest memory allocation failure\r\n");
            }
        }
    }

    return (retVal);
}

static int32_t udmaTestBlkcpyFree(UdmaTestTaskObj *taskObj)
{
    int32_t         retVal = UDMA_SOK;
    uint32_t        chCnt, qCnt;
    UdmaTestChObj  *chObj;

    if(NULL != taskObj->prfTsHandle)
    {
        Utils_prfTsDelete(taskObj->prfTsHandle);
        taskObj->prfTsHandle = NULL;
    }

    for(chCnt = 0U ; chCnt < taskObj->numCh; chCnt++)
    {
        chObj = taskObj->chObj[chCnt];
        GT_assert(taskObj->traceMask, chObj != NULL);

        if(NULL != chObj->fqRingMem)
        {
            retVal += Utils_memFree(chObj->heapIdSrc, chObj->fqRingMem, chObj->ringMemSize);
            chObj->fqRingMem = NULL;
        }
        if(NULL != chObj->cqRingMem)
        {
            retVal += Utils_memFree(chObj->heapIdSrc, chObj->cqRingMem, chObj->ringMemSize);
            chObj->cqRingMem = NULL;
        }
        if(NULL != chObj->tdCqRingMem)
        {
            retVal += Utils_memFree(chObj->heapIdSrc, chObj->tdCqRingMem, chObj->ringMemSize);
            chObj->tdCqRingMem = NULL;
        }
        if(NULL != chObj->srcBuf)
        {
            retVal += Utils_memFree(chObj->heapIdSrc, chObj->srcBuf, chObj->srcBufSize);
            chObj->srcBuf = NULL;
        }
        for(qCnt = 0U; qCnt < UDMA_TEST_MAX_QDEPTH; qCnt++)
        {
            if(NULL != chObj->trpdMem[qCnt])
            {
                retVal += Utils_memFree(chObj->heapIdSrc, chObj->trpdMem[qCnt], chObj->trpdSize);
                chObj->trpdMem[qCnt] = NULL;
            }
            if(NULL != chObj->destBuf[qCnt])
            {
                retVal += Utils_memFree(chObj->heapIdDest, chObj->destBuf[qCnt], chObj->destBufSize);
                chObj->destBuf[qCnt] = NULL;
            }
        }
    }

    return (retVal);
}

static int32_t udmaTestBlkcpyCompareData(UdmaTestTaskObj *taskObj,
                                         UdmaTestChObj *chObj)
{
    int32_t         retVal = UDMA_SOK;
    uint32_t        qCnt, i, srcIdx, destIdx;

    for(qCnt = 0U; qCnt < chObj->qdepth; qCnt++)
    {
        srcIdx = 0U;
        /* When dest buffer is circular */
        if(0U == chObj->ddim[1])
        {
            /* Go to the last block of source */
            srcIdx = chObj->dim[1] * (chObj->icnt[2] - 1U);
        }
        destIdx = 0U;
        /* Invalidate destination buffer */
        CacheP_inv(chObj->destBuf[qCnt], chObj->destBufSize, CacheP_TYPE_ALLD);
        for(i = 0U; i < chObj->destBufSize; i++)
        {
            if(chObj->srcBuf[srcIdx] != chObj->destBuf[qCnt][destIdx])
            {
                GT_2trace(taskObj->traceMask, GT_ERR,
                    " Data mismatch for Ch:%d, qCnt:%d!!\n", chObj->chIdx, qCnt);
                retVal = UDMA_EFAIL;
                break;
            }

            srcIdx++;
            if(srcIdx >= chObj->srcBufSize)
            {
                srcIdx = 0U;
            }
            destIdx++;
        }
    }

    return (retVal);
}

static void udmaTestBlkcpyEventDmaCb(Udma_EventHandle eventHandle,
                                     uint32_t eventType,
                                     void *appData)
{
    UdmaTestChObj *chObj = (UdmaTestChObj *) appData;

    if(chObj != NULL)
    {
        if(UDMA_EVENT_TYPE_DMA_COMPLETION == eventType)
        {
            SemaphoreP_post(&chObj->transferDoneSem);
        }
        else
        {
            gUdmaTestBlkcpyResult = UDMA_EFAIL;
        }
    }
    else
    {
        gUdmaTestBlkcpyResult = UDMA_EFAIL;
    }

    return;
}

static void udmaTestBlkcpyTrpdInit(UdmaTestChObj *chObj, uint32_t qCnt)
{
    CSL_UdmapTR15 *pTr = (CSL_UdmapTR15 *)(chObj->trpdMem[qCnt] + sizeof(CSL_UdmapTR15));
    uint32_t *pTrResp = (uint32_t *) (chObj->trpdMem[qCnt] + (sizeof(CSL_UdmapTR15) * 2U));
    uint32_t cqRingNum = Udma_chGetCqRingNum(chObj->chHandle);

    /* Make TRPD */
    UdmaUtils_makeTrpdTr15(chObj->trpdMem[qCnt], 1U, cqRingNum);

    /* Setup TR */
    pTr->flags  = CSL_FMK(UDMAP_TR_FLAGS_TYPE, CSL_UDMAP_TR_FLAGS_TYPE_4D_BLOCK_MOVE_REPACKING_INDIRECTION);
    pTr->flags |= CSL_FMK(UDMAP_TR_FLAGS_STATIC, 0U);
    pTr->flags |= CSL_FMK(UDMAP_TR_FLAGS_EOL, 0U);
    pTr->flags |= CSL_FMK(UDMAP_TR_FLAGS_EVENT_SIZE, chObj->chPrms->eventSize);
    pTr->flags |= CSL_FMK(UDMAP_TR_FLAGS_TRIGGER0, chObj->chPrms->trigger);
    pTr->flags |= CSL_FMK(UDMAP_TR_FLAGS_TRIGGER1, CSL_UDMAP_TR_FLAGS_TRIGGER_NONE);
    pTr->flags |= CSL_FMK(UDMAP_TR_FLAGS_TRIGGER0_TYPE, chObj->chPrms->triggerType);
    pTr->flags |= CSL_FMK(UDMAP_TR_FLAGS_TRIGGER1_TYPE, CSL_UDMAP_TR_FLAGS_TRIGGER_TYPE_ALL);
    pTr->flags |= CSL_FMK(UDMAP_TR_FLAGS_CMD_ID, 0x25U);
    pTr->flags |= CSL_FMK(UDMAP_TR_FLAGS_SA_INDIRECT, 0U);
    pTr->flags |= CSL_FMK(UDMAP_TR_FLAGS_DA_INDIRECT, 0U);
    pTr->flags |= CSL_FMK(UDMAP_TR_FLAGS_EOP, 1U);

    pTr->icnt0    = chObj->icnt[0];
    pTr->icnt1    = chObj->icnt[1];
    pTr->icnt2    = chObj->icnt[2];
    pTr->icnt3    = chObj->icnt[3];
    pTr->dim1     = chObj->dim[0];
    pTr->dim2     = chObj->dim[1];
    pTr->dim3     = chObj->dim[2];
    pTr->addr     = Udma_defaultVirtToPhyFxn(chObj->srcBuf, 0U, NULL);
    pTr->fmtflags = 0x00000000U;
    pTr->dicnt0   = chObj->dicnt[0];
    pTr->dicnt1   = chObj->dicnt[1];
    pTr->dicnt2   = chObj->dicnt[2];
    pTr->dicnt3   = chObj->dicnt[3];
    pTr->ddim1    = chObj->ddim[0];
    pTr->ddim2    = chObj->ddim[1];
    pTr->ddim3    = chObj->ddim[2];
    pTr->daddr    = Udma_defaultVirtToPhyFxn(chObj->destBuf[qCnt], 0U, NULL);

    /* Clear TR response memory */
    *pTrResp = 0xFFFFFFFFU;

    /* Writeback cache */
    CacheP_wb(chObj->trpdMem[qCnt], chObj->trpdSize, CacheP_TYPE_ALLD);

    return;
}
