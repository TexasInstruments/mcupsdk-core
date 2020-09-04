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
 *  \file udma_test_ring.c
 *
 *  \brief UDMA ring related test case file.
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

static int32_t udmaTestRingProxyTestLoop(UdmaTestTaskObj *taskObj);
static int32_t udmaTestRingFlushTestLoop(UdmaTestTaskObj *taskObj);
static int32_t udmaTestRingEventTestLoop(UdmaTestTaskObj *taskObj);
static int32_t udmaTestRingParamCheckTestLoop(UdmaTestTaskObj *taskObj);
static int32_t udmaTestRingAttachTestLoop(UdmaTestTaskObj *taskObj);
static int32_t udmaTestRingResetTestLoop(UdmaTestTaskObj *taskObj);
static int32_t udmaTestRingPrimeTestLoop(UdmaTestTaskObj *taskObj);
static void udmaTestRingEventCb(Udma_EventHandle eventHandle,
                                uint32_t eventType,
                                void *appData);

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */

/* Global test pass/fail flag */
static volatile int32_t gUdmaTestRingResult = UDMA_SOK;

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

int32_t udmaTestRingProxyTc(UdmaTestTaskObj *taskObj)
{
    int32_t     retVal = UDMA_SOK;
    uint32_t    loopCnt = 0U;

    GT_1trace(taskObj->traceMask, GT_INFO1,
              " |TEST INFO|:: Task:%d: Proxy Testcase ::\r\n", taskObj->taskId);
    GT_2trace(taskObj->traceMask, GT_INFO1,
              " |TEST INFO|:: Task:%d: Loop count           : %d ::\r\n", taskObj->taskId, taskObj->loopCnt);

    gUdmaTestRingResult = UDMA_SOK;
    while(loopCnt < taskObj->loopCnt)
    {
        /* Perform proxy test */
        retVal = udmaTestRingProxyTestLoop(taskObj);
        if(UDMA_SOK != retVal)
        {
            break;
        }

        loopCnt++;
    }

    retVal += gUdmaTestRingResult;

    return (retVal);
}

int32_t udmaTestRingFlushTc(UdmaTestTaskObj *taskObj)
{
    int32_t     retVal = UDMA_SOK;
    uint32_t    loopCnt = 0U;

    GT_1trace(taskObj->traceMask, GT_INFO1,
              " |TEST INFO|:: Task:%d: Ring Flush API Testcase ::\r\n", taskObj->taskId);
    GT_2trace(taskObj->traceMask, GT_INFO1,
              " |TEST INFO|:: Task:%d: Loop count           : %d ::\r\n", taskObj->taskId, taskObj->loopCnt);

    gUdmaTestRingResult = UDMA_SOK;
    while(loopCnt < taskObj->loopCnt)
    {
        /* Perform proxy test */
        retVal = udmaTestRingFlushTestLoop(taskObj);
        if(UDMA_SOK != retVal)
        {
            break;
        }

        loopCnt++;
    }

    retVal += gUdmaTestRingResult;

    return (retVal);
}

int32_t udmaTestRingEventTc(UdmaTestTaskObj *taskObj)
{
    int32_t     retVal = UDMA_SOK;
    uint32_t    loopCnt = 0U;

    GT_1trace(taskObj->traceMask, GT_INFO1,
              " |TEST INFO|:: Task:%d: Ring Event API Testcase ::\r\n", taskObj->taskId);
    GT_2trace(taskObj->traceMask, GT_INFO1,
              " |TEST INFO|:: Task:%d: Loop count           : %d ::\r\n", taskObj->taskId, taskObj->loopCnt);

    gUdmaTestRingResult = UDMA_SOK;
    while(loopCnt < taskObj->loopCnt)
    {
        /* Perform ring event test */
        retVal = udmaTestRingEventTestLoop(taskObj);
        if(UDMA_SOK != retVal)
        {
            break;
        }

        loopCnt++;
    }

    retVal += gUdmaTestRingResult;

    return (retVal);
}

int32_t udmaTestRingParamCheckTc(UdmaTestTaskObj *taskObj)
{
    int32_t     retVal = UDMA_SOK;
    uint32_t    loopCnt = 0U;

    GT_1trace(taskObj->traceMask, GT_INFO1,
              " |TEST INFO|:: Task:%d: Ring Paramter Check Testcase ::\r\n", taskObj->taskId);
    GT_2trace(taskObj->traceMask, GT_INFO1,
              " |TEST INFO|:: Task:%d: Loop count           : %d ::\r\n", taskObj->taskId, taskObj->loopCnt);

    gUdmaTestRingResult = UDMA_SOK;
    while(loopCnt < taskObj->loopCnt)
    {
        retVal = udmaTestRingParamCheckTestLoop(taskObj);
        if(UDMA_SOK != retVal)
        {
            break;
        }

        loopCnt++;
    }

    retVal += gUdmaTestRingResult;

    return (retVal);
}

int32_t udmaTestRingUtilsMemSizeTc(UdmaTestTaskObj *taskObj)
{
    int32_t     retVal = UDMA_SOK;
    uint32_t    ringMode, ringSize;
    uint32_t    elemCnt = 1U;
    uint32_t    ringMemSize, ringMemSizeExpected;
    char *ringModeString[] = {"RING", "MESSAGE", "CREDENTIAL", "QM"};
    uint32_t ringSizeArray[] = {4, 8, 16, 32, 64, 128, 256};

    GT_1trace(taskObj->traceMask, GT_INFO1,
              " |TEST INFO|:: Task:%d: Ring Utils Mem Size Testcase ::\r\n", taskObj->taskId);

    for(ringMode = UDMA_TEST_RING_MODE_MIN;
        ringMode <= UDMA_TEST_RING_MODE_MAX;
        ringMode++)
    {
        for(ringSize = UDMA_RING_ES_4BYTES;
            ringSize <= UDMA_RING_ES_256BYTES;
            ringSize++)
        {
            ringMemSize = UdmaUtils_getRingMemSize(ringMode, elemCnt, ringSize);
            GT_4trace(taskObj->traceMask, GT_INFO1,
                      " Ring mem size for %d elements:%4d bytes (Size:%3d, Mode:%12s)\r\n",
                      elemCnt, ringMemSize, ringSizeArray[ringSize], ringModeString[ringMode]);
            ringMemSizeExpected = elemCnt * ringSizeArray[ringSize];
            if(ringMode >= TISCI_MSG_VALUE_RM_RING_MODE_CREDENTIALS)
            {
                ringMemSizeExpected *= 2U;
            }
            if(ringMemSize != ringMemSizeExpected)
            {
                retVal = UDMA_EFAIL;
                GT_0trace(taskObj->traceMask, GT_ERR,
                    " Ring memory size compare mismatched!!\r\n");
            }
        }
    }

    return (retVal);
}

int32_t udmaTestRingMemPtrTc(UdmaTestTaskObj *taskObj)
{
    int32_t             retVal = UDMA_SOK;
    uint16_t            ringNum = UDMA_RING_ANY;
    uint32_t            instId, ringMode;
    uint32_t            elemCnt = 100U, ringMemSize;
    uint32_t            heapId = UTILS_MEM_HEAP_ID_MSMC;
    Udma_DrvHandle      drvHandle;
    Udma_RingPrms       ringPrms;
    Udma_RingObject     ringObj;
    Udma_RingHandle     ringHandle = &ringObj;
    void               *ringMem = NULL;
    Udma_RmInitPrms    *rmInitPrms;
    char *instanceIdStr[] = {"MAIN", "MCU", "BCDMA", "PKTDMA"};
    char *ringModeString[] = {"RING", "MESSAGE"};

    GT_1trace(taskObj->traceMask, GT_INFO1,
              " |TEST INFO|:: Task:%d: Ring Mem Pointer Testcase ::\r\n", taskObj->taskId);

    ringMemSize = elemCnt * sizeof (uint64_t);
    ringMem = Utils_memAlloc(heapId, ringMemSize, UDMA_CACHELINE_ALIGNMENT);
    if(NULL == ringMem)
    {
        retVal = UDMA_EALLOC;
        GT_0trace(taskObj->traceMask, GT_ERR, " Ring memory allocation failure\r\n");
    }

    if(UDMA_SOK == retVal)
    {
        for(instId = UDMA_INST_ID_START; instId <= UDMA_INST_ID_MAX; instId++)
        {
            for(ringMode = UDMA_TEST_RING_MODE_DEFAULT_START;
                ringMode <= UDMA_TEST_RING_MODE_DEFAULT_STOP;
                ringMode++)
            {
                GT_2trace(taskObj->traceMask, GT_INFO1,
                          " Testing for Inst: %s, Ring Mode: %s...\r\n",
                          instanceIdStr[instId], ringModeString[ringMode]);
                drvHandle = &taskObj->testObj->drvObj[instId];
                Udma_DrvObjectInt  *drvObj = (Udma_DrvObjectInt *) drvHandle;

                UdmaRingPrms_init(&ringPrms);
                ringPrms.ringMem = ringMem;
                ringPrms.ringMemSize = ringMemSize;
                ringPrms.mode = ringMode;
                ringPrms.elemCnt = elemCnt;

                /* Since no free/extra rings are available in BCDMA/PKTDMA(Unmapped Channels)
                   #Udma_ringAlloc with #UDMA_RING_ANY will fail.
                   So get the default ringNum for a channel */
                rmInitPrms = &drvObj->rmInitPrms;
                ringNum =  (uint16_t) (rmInitPrms->startTxCh + drvObj->txChOffset);

                /* Allocate ring */
                retVal = Udma_ringAlloc(drvHandle, ringHandle, ringNum, &ringPrms);
                if(UDMA_SOK != retVal)
                {
                    GT_0trace(taskObj->traceMask, GT_ERR, " Ring alloc failed!!\n");
                    break;
                }

                /* Check Ring Mem Pointer */
                if(Udma_ringGetMemPtr(ringHandle) != ringMem)
                {
                    GT_0trace(taskObj->traceMask, GT_ERR,
                        " Ring memory pointer mismatch!!\n");
                    retVal = UDMA_EFAIL;
                    break;
                }

                retVal = Udma_ringFree(ringHandle);
                if(UDMA_SOK != retVal)
                {
                    GT_0trace(taskObj->traceMask, GT_ERR, " Ring free failed!!\n");
                    break;
                }

                GT_2trace(taskObj->traceMask, GT_INFO1,
                          " Testing for Inst: %s, Ring Mode: %s passed!!\r\n",
                          instanceIdStr[instId], ringModeString[ringMode]);
            }

            if(UDMA_SOK != retVal)
            {
                break;
            }
        }
    }

    if(NULL != ringMem)
    {
        retVal += Utils_memFree(heapId, ringMem, ringMemSize);
        if(UDMA_SOK != retVal)
        {
            GT_0trace(taskObj->traceMask, GT_ERR, " Ring free failed!!\n");
        }
    }

    return (retVal);
}

int32_t udmaTestRingAttachTc(UdmaTestTaskObj *taskObj)
{
    int32_t     retVal = UDMA_SOK;
    uint32_t    loopCnt = 0U;

    GT_1trace(taskObj->traceMask, GT_INFO1,
              " |TEST INFO|:: Task:%d: Ring Attach/Detach Testcase ::\r\n", taskObj->taskId);
    GT_2trace(taskObj->traceMask, GT_INFO1,
              " |TEST INFO|:: Task:%d: Loop count           : %d ::\r\n", taskObj->taskId, taskObj->loopCnt);

    gUdmaTestRingResult = UDMA_SOK;
    while(loopCnt < taskObj->loopCnt)
    {
        /* Perform ring attach test */
        retVal = udmaTestRingAttachTestLoop(taskObj);
        if(UDMA_SOK != retVal)
        {
            break;
        }

        loopCnt++;
    }

    retVal += gUdmaTestRingResult;

    return (retVal);
}

int32_t udmaTestRingResetTc(UdmaTestTaskObj *taskObj)
{
    int32_t     retVal = UDMA_SOK;
    uint32_t    loopCnt = 0U;

    GT_1trace(taskObj->traceMask, GT_INFO1,
              " |TEST INFO|:: Task:%d: Ring Reset Testcase ::\r\n", taskObj->taskId);
    GT_2trace(taskObj->traceMask, GT_INFO1,
              " |TEST INFO|:: Task:%d: Loop count           : %d ::\r\n", taskObj->taskId, taskObj->loopCnt);

    gUdmaTestRingResult = UDMA_SOK;
    while(loopCnt < taskObj->loopCnt)
    {
        /* Perform ring reset test */
        retVal = udmaTestRingResetTestLoop(taskObj);
        if(UDMA_SOK != retVal)
        {
            break;
        }

        loopCnt++;
    }

    retVal += gUdmaTestRingResult;

    return (retVal);
}

int32_t udmaTestRingPrimeTc(UdmaTestTaskObj *taskObj)
{
    int32_t     retVal = UDMA_SOK;
    uint32_t    loopCnt = 0U;

    GT_1trace(taskObj->traceMask, GT_INFO1,
              " |TEST INFO|:: Task:%d: Ring Prime Testcase ::\r\n", taskObj->taskId);
    GT_2trace(taskObj->traceMask, GT_INFO1,
              " |TEST INFO|:: Task:%d: Loop count           : %d ::\r\n", taskObj->taskId, taskObj->loopCnt);

    gUdmaTestRingResult = UDMA_SOK;
    while(loopCnt < taskObj->loopCnt)
    {
        /* Perform ring prime test */
        retVal = udmaTestRingPrimeTestLoop(taskObj);
        if(UDMA_SOK != retVal)
        {
            break;
        }

        loopCnt++;
    }

    retVal += gUdmaTestRingResult;

    return (retVal);
}

int32_t udmaTestRingPrimeLcdmaTc(UdmaTestTaskObj *taskObj)
{
    int32_t     retVal = UDMA_SOK;

    GT_1trace(taskObj->traceMask, GT_INFO1,
              " |TEST INFO|:: Task:%d: LCDMA Ring Prime Testcase ::\r\n", taskObj->taskId);
    GT_2trace(taskObj->traceMask, GT_INFO1,
              " |TEST INFO|:: Task:%d: Loop count                 : %d ::\r\n", taskObj->taskId, taskObj->loopCnt);

    /* Perform ring prime lcdma test using block copy */
    retVal = udmaTestBlkCpyRingPrimeLcdmaTest(taskObj);

    return (retVal);
}

static int32_t udmaTestRingProxyTestLoop(UdmaTestTaskObj *taskObj)
{
    int32_t             retVal = UDMA_SOK;
    uint32_t            instId, qCnt, ringMode;
    uint32_t            elemCnt = 500U, ringMemSize;
    uint32_t            heapId = UTILS_MEM_HEAP_ID_MSMC;
    Udma_DrvHandle      drvHandle;
    Udma_RingPrms       ringPrms;
    Udma_RingObject     ringObj;
    Udma_RingHandle     ringHandle = &ringObj;
    void               *ringMem = NULL;
    uint64_t            ringData;
    char *instanceIdStr[] = {"MAIN", "MCU", "BCDMA", "PKTDMA"};
    char *ringModeString[] = {"RING", "MESSAGE"};

    ringMemSize = elemCnt * sizeof (uint64_t);
    ringMem = Utils_memAlloc(heapId, ringMemSize, UDMA_CACHELINE_ALIGNMENT);
    if(NULL == ringMem)
    {
        retVal = UDMA_EALLOC;
        GT_0trace(taskObj->traceMask, GT_ERR, " Ring memory allocation failure\r\n");
    }

    if(UDMA_SOK == retVal)
    {
        for(instId = UDMA_INST_ID_START; instId <= UDMA_INST_ID_MAX; instId++)
        {
            for(ringMode = UDMA_TEST_RING_MODE_DEFAULT_START;
                ringMode <= UDMA_TEST_RING_MODE_DEFAULT_STOP;
                ringMode++)
            {
                GT_2trace(taskObj->traceMask, GT_INFO1,
                          " Testing for Inst: %s, Ring Mode: %s...\r\n",
                          instanceIdStr[instId], ringModeString[ringMode]);
                drvHandle = &taskObj->testObj->drvObj[instId];

                UdmaRingPrms_init(&ringPrms);
                ringPrms.ringMem = ringMem;
                ringPrms.ringMemSize = ringMemSize;
                ringPrms.mode = ringMode;
                ringPrms.elemCnt = elemCnt;

                /* Allocate a free ring */
                retVal = Udma_ringAlloc(drvHandle, ringHandle, UDMA_RING_ANY, &ringPrms);
                if(UDMA_SOK != retVal)
                {
                    GT_0trace(taskObj->traceMask, GT_ERR, " Ring alloc failed!!\n");
                    break;
                }

                /* Queue through proxy */
                for(qCnt = 0U; qCnt < elemCnt; qCnt++)
                {
                    ringData = ((uint64_t) qCnt | (uint64_t) 0xDEADBEEF00000000UL);
                    retVal = Udma_ringQueueRaw(ringHandle, ringData);
                    if(UDMA_SOK != retVal)
                    {
                        GT_0trace(taskObj->traceMask, GT_ERR, " Proxy Ring queue failed!!\n");
                        break;
                    }
                }
                if(UDMA_SOK != retVal)
                {
                    break;
                }

                /* Check if the HW occupancy is same as what is queued */
                if(udmaTestCompareRingHwOccDriver(ringHandle, elemCnt, UDMA_TEST_RING_ACC_DIRECTION_FORWARD) != UDMA_SOK)
                {
                    GT_0trace(taskObj->traceMask, GT_ERR, " Ring element count mismatch!!\n");
                    retVal = UDMA_EFAIL;
                    break;
                }

                /* Dequeue through proxy */
                for(qCnt = 0U; qCnt < elemCnt; qCnt++)
                {
                    ringData = 0UL;
                    retVal = Udma_ringDequeueRaw(ringHandle, &ringData);
                    if(UDMA_SOK != retVal)
                    {
                        GT_0trace(taskObj->traceMask, GT_ERR, " Proxy Ring dequeue failed!!\n");
                        break;
                    }

                    if(ringData != ((uint64_t) qCnt | (uint64_t) 0xDEADBEEF00000000UL))
                    {
                        GT_0trace(taskObj->traceMask, GT_ERR, " Ring data mismatch!!\n");
                        break;
                    }
                }
                if(UDMA_SOK != retVal)
                {
                    break;
                }

                /* Check if the HW occupancy is zero */
                if(udmaTestCompareRingHwOccDriver(ringHandle, 0U, UDMA_TEST_RING_ACC_DIRECTION_FORWARD) != UDMA_SOK)
                {
                    GT_0trace(taskObj->traceMask, GT_ERR, " Ring not empty!!\n");
                    retVal = UDMA_EFAIL;
                    break;
                }

                retVal = Udma_ringFree(ringHandle);
                if(UDMA_SOK != retVal)
                {
                    GT_0trace(taskObj->traceMask, GT_ERR, " Ring free failed!!\n");
                    break;
                }

                GT_2trace(taskObj->traceMask, GT_INFO1,
                          " Testing for Inst: %s, Ring Mode: %s passed!!\r\n",
                          instanceIdStr[instId], ringModeString[ringMode]);
            }

            if(UDMA_SOK != retVal)
            {
                break;
            }
        }
    }

    if(NULL != ringMem)
    {
        retVal += Utils_memFree(heapId, ringMem, ringMemSize);
        if(UDMA_SOK != retVal)
        {
            GT_0trace(taskObj->traceMask, GT_ERR, " Ring free failed!!\n");
        }
    }

    return (retVal);
}

static int32_t udmaTestRingFlushTestLoop(UdmaTestTaskObj *taskObj)
{
    int32_t             retVal = UDMA_SOK;
    uint16_t            ringNum = UDMA_RING_ANY;
    uint32_t            instId, qCnt, ringMode, testLoopCnt, testLoopMax;
    uint32_t            elemCnt = 50U, ringMemSize;
    uint32_t            heapId = UTILS_MEM_HEAP_ID_MSMC;
    Udma_DrvHandle      drvHandle;
    Udma_RingPrms       ringPrms;
    Udma_RingObject     ringObj;
    Udma_RingHandle     ringHandle = &ringObj;
    void               *ringMem = NULL;
    uint64_t            ringData;
    Udma_RmInitPrms    *rmInitPrms;
    char *instanceIdStr[] = {"MAIN", "MCU", "BCDMA", "PKTDMA"};
    char *ringModeString[] = {"RING", "MESSAGE"};

    ringMemSize = elemCnt * sizeof (uint64_t);
    ringMem = Utils_memAlloc(heapId, ringMemSize, UDMA_CACHELINE_ALIGNMENT);
    if(NULL == ringMem)
    {
        retVal = UDMA_EALLOC;
        GT_0trace(taskObj->traceMask, GT_ERR, " Ring memory allocation failure\r\n");
    }

    if(UDMA_SOK == retVal)
    {
        for(instId = UDMA_INST_ID_START; instId <= UDMA_INST_ID_MAX; instId++)
        {
            for(ringMode = UDMA_TEST_RING_MODE_DEFAULT_START;
                ringMode <= UDMA_TEST_RING_MODE_DEFAULT_STOP;
                ringMode++)
            {
                GT_2trace(taskObj->traceMask, GT_INFO1,
                          " Testing for Inst: %s, Ring Mode: %s...\r\n",
                          instanceIdStr[instId], ringModeString[ringMode]);
                drvHandle = &taskObj->testObj->drvObj[instId];
                Udma_DrvObjectInt  *drvObj = (Udma_DrvObjectInt *) drvHandle;

                UdmaRingPrms_init(&ringPrms);
                ringPrms.ringMem = ringMem;
                ringPrms.ringMemSize = ringMemSize;
                ringPrms.mode = ringMode;
                ringPrms.elemCnt = elemCnt;

                /* Since no free/extra rings are available in BCDMA/PKTDMA(Unmapped Channels)
                   #Udma_ringAlloc with #UDMA_RING_ANY will fail.
                   So get the default ringNum for a channel */
                rmInitPrms = &drvObj->rmInitPrms;
                ringNum =  (uint16_t) (rmInitPrms->startTxCh + drvObj->txChOffset);
                testLoopMax = 3U; /* After flush, Test ring operation, Also test after ring reset */

                /* Allocate ring */
                retVal = Udma_ringAlloc(drvHandle, ringHandle, ringNum, &ringPrms);
                if(UDMA_SOK != retVal)
                {
                    GT_0trace(taskObj->traceMask, GT_ERR, " Ring alloc failed!!\n");
                    break;
                }

                for(testLoopCnt = 0U; testLoopCnt < testLoopMax; testLoopCnt++)
                {
                    /* Ring queue */
                    for(qCnt = 0U; qCnt < elemCnt; qCnt++)
                    {
                        ringData = ((uint64_t) qCnt | (uint64_t) 0xDEADBEEF00000000UL);
                        retVal = Udma_ringQueueRaw(ringHandle, ringData);
                        if(UDMA_SOK != retVal)
                        {
                            GT_0trace(taskObj->traceMask, GT_ERR, " Ring queue failed!!\n");
                            break;
                        }
                    }
                    if(UDMA_SOK != retVal)
                    {
                        break;
                    }

                    /* Check if the HW occupancy is same as what is queued */
                    if(udmaTestCompareRingHwOccDriver(ringHandle, elemCnt, UDMA_TEST_RING_ACC_DIRECTION_FORWARD) != UDMA_SOK)
                    {
                        GT_0trace(taskObj->traceMask, GT_ERR, " Ring element count mismatch!!\n");
                        retVal = UDMA_EFAIL;
                        break;
                    }

                    /* Dequeue using Flush API */
                    for(qCnt = 0U; qCnt < elemCnt; qCnt++)
                    {
                        ringData = 0UL;
                        retVal = Udma_ringFlushRaw(ringHandle, &ringData);
                        if(UDMA_SOK != retVal)
                        {
                            GT_0trace(taskObj->traceMask, GT_ERR, " Ring flush failed!!\n");
                            break;
                        }

                        if(ringData != ((uint64_t) qCnt | (uint64_t) 0xDEADBEEF00000000UL))
                        {
                            GT_0trace(taskObj->traceMask, GT_ERR, " Ring data mismatch!!\n");
                            break;
                        }
                    }
                    if(UDMA_SOK != retVal)
                    {
                        break;
                    }

                    /* Check if the HW occupancy is zero */
                    if(udmaTestCompareRingHwOccDriver(ringHandle, 0U, UDMA_TEST_RING_ACC_DIRECTION_FORWARD) != UDMA_SOK)
                    {
                        GT_0trace(taskObj->traceMask, GT_ERR, " Ring not empty!!\n");
                        retVal = UDMA_EFAIL;
                        break;
                    }

                    /* First, test Ring operation without reset after ring flush.
                     * Second, try reset */
                    if(1U == testLoopCnt)
                    {
                        /* For LCDMA Rings, Reset the Ring - By calling the #Udma_ringFlushRaw
                         * when there are no more unprocessed descriptors */
                        Udma_ringFlushRaw(ringHandle, &ringData);
                    }
                }
                retVal = Udma_ringFree(ringHandle);
                if(UDMA_SOK != retVal)
                {
                    GT_0trace(taskObj->traceMask, GT_ERR, " Ring free failed!!\n");
                    break;
                }

                GT_2trace(taskObj->traceMask, GT_INFO1,
                          " Testing for Inst: %s, Ring Mode: %s passed!!\r\n",
                          instanceIdStr[instId], ringModeString[ringMode]);
            }

            if(UDMA_SOK != retVal)
            {
                break;
            }
        }
    }

    if(NULL != ringMem)
    {
        retVal += Utils_memFree(heapId, ringMem, ringMemSize);
        if(UDMA_SOK != retVal)
        {
            GT_0trace(taskObj->traceMask, GT_ERR, " Ring free failed!!\n");
        }
    }

    return (retVal);
}

static int32_t udmaTestRingEventTestLoop(UdmaTestTaskObj *taskObj)
{
    int32_t             retVal = UDMA_SOK;
    uint32_t            instId, qCnt, ringMode;
    uint32_t            elemCnt = 50U, ringMemSize;
    uint32_t            heapId = UTILS_MEM_HEAP_ID_MSMC;
    Udma_DrvHandle      drvHandle;
    Udma_RingPrms       ringPrms;
    Udma_RingObject     ringObj;
    Udma_RingHandle     ringHandle = &ringObj;
    void               *ringMem = NULL;
    uint64_t            ringData;
    char *instanceIdStr[] = {"MAIN", "MCU", "BCDMA", "PKTDMA"};
    char *ringModeString[] = {"RING", "MESSAGE"};
    Udma_EventPrms      eventPrms;
    SemaphoreP_Object   transferDoneSem;
    Udma_EventObject    eventObj;
    Udma_EventHandle    eventHandle = NULL;
    volatile uint64_t   intrStatusReg;

    ringMemSize = elemCnt * sizeof (uint64_t);
    ringMem = Utils_memAlloc(heapId, ringMemSize, UDMA_CACHELINE_ALIGNMENT);
    if(NULL == ringMem)
    {
        retVal = UDMA_EALLOC;
        GT_0trace(taskObj->traceMask, GT_ERR, " Ring memory allocation failure\r\n");
    }

    if(UDMA_SOK == retVal)
    {
        retVal = SemaphoreP_constructCounting(&transferDoneSem, 0, elemCnt);
        if(UDMA_SOK != retVal)
        {
            GT_0trace(taskObj->traceMask, GT_ERR, " Sem create failed!!\n");
        }
    }

    if(UDMA_SOK == retVal)
    {
        for(instId = UDMA_INST_ID_START; instId <= UDMA_INST_ID_MAX; instId++)
        {
            for(ringMode = UDMA_TEST_RING_MODE_DEFAULT_START;
                ringMode <= UDMA_TEST_RING_MODE_DEFAULT_STOP;
                ringMode++)
            {
                GT_2trace(taskObj->traceMask, GT_INFO1,
                          " Testing Ring Event for Inst: %s, Ring Mode: %s...\r\n",
                          instanceIdStr[instId], ringModeString[ringMode]);
                drvHandle = &taskObj->testObj->drvObj[instId];

                UdmaRingPrms_init(&ringPrms);
                ringPrms.ringMem = ringMem;
                ringPrms.ringMemSize = ringMemSize;
                ringPrms.mode = ringMode;
                ringPrms.elemCnt = elemCnt;

                /* Allocate a free ring */
                retVal = Udma_ringAlloc(drvHandle, ringHandle, UDMA_RING_ANY, &ringPrms);
                if(UDMA_SOK != retVal)
                {
                    GT_0trace(taskObj->traceMask, GT_ERR, " Ring alloc failed!!\n");
                    break;
                }

                GT_assert(taskObj->traceMask, (taskObj->ringPrms != NULL));
                if(UDMA_TEST_EVENT_NONE != taskObj->ringPrms->eventMode)
                {
                    /* Register ring completion */
                    eventHandle = &eventObj;
                    UdmaEventPrms_init(&eventPrms);
                    eventPrms.eventType         = UDMA_EVENT_TYPE_RING;
                    eventPrms.eventMode         = UDMA_EVENT_MODE_SHARED;
                    eventPrms.ringHandle        = ringHandle;
                    eventPrms.masterEventHandle = NULL;
                    if(UDMA_TEST_EVENT_INTR == taskObj->ringPrms->eventMode)
                    {
                        eventPrms.masterEventHandle = Udma_eventGetGlobalHandle(drvHandle);
                        eventPrms.eventCb           = &udmaTestRingEventCb;
                        eventPrms.appData           = &transferDoneSem;
                    }
                    retVal = Udma_eventRegister(drvHandle, eventHandle, &eventPrms);
                    if(UDMA_SOK != retVal)
                    {
                        GT_0trace(taskObj->traceMask, GT_ERR,
                            " UDMA Ring event register failed!!\n");
                        break;
                    }
                }

                /* Ring queue */
                for(qCnt = 0U; qCnt < elemCnt; qCnt++)
                {
                    ringData = ((uint64_t) qCnt | (uint64_t) 0xDEADBEEF00000000UL);
                    retVal = Udma_ringQueueRaw(ringHandle, ringData);
                    if(UDMA_SOK != retVal)
                    {
                        GT_0trace(taskObj->traceMask, GT_ERR, " Ring queue failed!!\n");
                        break;
                    }
                }
                if(UDMA_SOK != retVal)
                {
                    break;
                }

                if(UDMA_TEST_EVENT_NONE == taskObj->ringPrms->eventMode)
                {
                    /* Do nothing as ring is full the moment ring queue is done */
                }
                else
                {
                    if(UDMA_TEST_EVENT_INTR == taskObj->ringPrms->eventMode)
                    {
                        SemaphoreP_pend(&transferDoneSem, SystemP_WAIT_FOREVER);
                    }
                    else    /* Event polled mode */
                    {
                        /* Wait for event in polled mode */
                        while(1U)
                        {
                            intrStatusReg = CSL_REG64_RD(eventPrms.intrStatusReg);
                            if(intrStatusReg & eventPrms.intrMask)
                            {
                                /* Clear interrupt */
                                CSL_REG64_WR(eventPrms.intrClearReg, eventPrms.intrMask);
                                break;
                            }
                        #if !defined (UDMA_UT_BAREMETAL)
                            TaskP_yield();
                        #endif
                        }
                    }
                }

                /* Ring Event Unregister with descriptors present in ring - Error Check*/
                retVal = Udma_eventUnRegister(eventHandle);
                if(UDMA_SOK == retVal)
                {
                    GT_0trace(taskObj->traceMask, GT_ERR,
                        " Ring Event Unregister did not fail when descriptors present in ring!!\n");
                    retVal = UDMA_EFAIL;
                    break;
                }

                /* Dequeue */
                for(qCnt = 0U; qCnt < elemCnt; qCnt++)
                {
                    ringData = 0UL;
                    retVal = Udma_ringFlushRaw(ringHandle, &ringData);
                    if(UDMA_SOK != retVal)
                    {
                        GT_0trace(taskObj->traceMask, GT_ERR, " Ring flush failed!!\n");
                        break;
                    }

                    if(ringData != ((uint64_t) qCnt | (uint64_t) 0xDEADBEEF00000000UL))
                    {
                        GT_0trace(taskObj->traceMask, GT_ERR, " Ring data mismatch!!\n");
                        break;
                    }
                }
                if(UDMA_SOK != retVal)
                {
                    break;
                }

                /* Check if the HW occupancy is zero */
                if(udmaTestCompareRingHwOccDriver(ringHandle, 0U, UDMA_TEST_RING_ACC_DIRECTION_FORWARD) != UDMA_SOK)
                {
                    GT_0trace(taskObj->traceMask, GT_ERR, " Ring not empty!!\n");
                    retVal = UDMA_EFAIL;
                    break;
                }

                if(UDMA_TEST_EVENT_NONE != taskObj->ringPrms->eventMode)
                {
                    retVal = Udma_eventUnRegister(eventHandle);
                    if(UDMA_SOK != retVal)
                    {
                        GT_0trace(taskObj->traceMask, GT_ERR, " Event unregister failed!!\n");
                        break;
                    }
                }

                retVal = Udma_ringFree(ringHandle);
                if(UDMA_SOK != retVal)
                {
                    GT_0trace(taskObj->traceMask, GT_ERR, " Ring free failed!!\n");
                    break;
                }

                GT_2trace(taskObj->traceMask, GT_INFO1,
                          " Testing Ring Event for Inst: %s, Ring Mode: %s passed!!\r\n",
                          instanceIdStr[instId], ringModeString[ringMode]);
            }

            if(UDMA_SOK != retVal)
            {
                break;
            }
        }
    }

    if(NULL != ringMem)
    {
        retVal += Utils_memFree(heapId, ringMem, ringMemSize);
        if(UDMA_SOK != retVal)
        {
            GT_0trace(taskObj->traceMask, GT_ERR, " Ring free failed!!\n");
        }
    }

    SemaphoreP_destruct(&transferDoneSem);

    return (retVal);
}

static int32_t udmaTestRingParamCheckTestLoop(UdmaTestTaskObj *taskObj)
{
    int32_t             retVal = UDMA_SOK;
    uint16_t            ringNum = UDMA_RING_ANY;
    uint32_t            instId, ringMode, checkRingMode;
    uint32_t            elemCnt = 50U, ringMemSize;
    uint32_t            heapId = UTILS_MEM_HEAP_ID_MSMC;
    Udma_DrvHandle      drvHandle;
    Udma_RingPrms       ringPrms;
    Udma_RingObject     ringObj;
    Udma_RingHandle     ringHandle = &ringObj;
    void               *ringMem = NULL;
    Udma_RmInitPrms    *rmInitPrms;
    char *instanceIdStr[] = {"MAIN", "MCU", "BCDMA", "PKTDMA"};
    char *ringModeString[] = {"RING", "MESSAGE"};

    ringMemSize = elemCnt * sizeof (uint64_t);
    ringMem = Utils_memAlloc(heapId, ringMemSize, UDMA_CACHELINE_ALIGNMENT);
    if(NULL == ringMem)
    {
        retVal = UDMA_EALLOC;
        GT_0trace(taskObj->traceMask, GT_ERR, " Ring memory allocation failure\r\n");
    }

    if(UDMA_SOK == retVal)
    {
        for(instId = UDMA_INST_ID_START; instId <= UDMA_INST_ID_MAX; instId++)
        {
            for(ringMode = UDMA_TEST_RING_MODE_DEFAULT_START;
                ringMode <= UDMA_TEST_RING_MODE_DEFAULT_STOP;
                ringMode++)
            {
                GT_2trace(taskObj->traceMask, GT_INFO1,
                          " Testing ring params check for Inst: %s, Ring Mode: %s...\r\n",
                          instanceIdStr[instId], ringModeString[ringMode]);
                drvHandle = &taskObj->testObj->drvObj[instId];
                Udma_DrvObjectInt  *drvObj = (Udma_DrvObjectInt *) drvHandle;

                /* Ring memory NULL check */
                UdmaRingPrms_init(&ringPrms);
                ringPrms.ringMem = NULL;
                ringPrms.ringMemSize = ringMemSize;
                ringPrms.mode = ringMode;
                ringPrms.elemCnt = elemCnt;
                rmInitPrms = &drvObj->rmInitPrms;
                ringNum =  (uint16_t) (rmInitPrms->startTxCh + drvObj->txChOffset);

                retVal = Udma_ringAlloc(drvHandle, ringHandle, ringNum, &ringPrms);
                if(UDMA_SOK == retVal)
                {
                    GT_0trace(taskObj->traceMask, GT_ERR,
                        " Ring alloc didnot fail for mem NULL check!!\n");
                    break;
                }

                /* Ring memory alignment check */
                UdmaRingPrms_init(&ringPrms);
                ringPrms.ringMem = (void *) ((uintptr_t)ringMem + 4U);
                ringPrms.ringMemSize = ringMemSize;
                ringPrms.mode = ringMode;
                ringPrms.elemCnt = elemCnt;
                rmInitPrms = &drvObj->rmInitPrms;
                ringNum =  (uint16_t) (rmInitPrms->startTxCh + drvObj->txChOffset);

                retVal = Udma_ringAlloc(drvHandle, ringHandle, ringNum, &ringPrms);
                if(UDMA_SOK == retVal)
                {
                    GT_0trace(taskObj->traceMask, GT_ERR,
                        " Ring alloc didnot fail for mem align check!!\n");
                    break;
                }

                /* Ring zero element count check */
                UdmaRingPrms_init(&ringPrms);
                ringPrms.ringMem = ringMem;
                ringPrms.ringMemSize = ringMemSize;
                ringPrms.mode = ringMode;
                ringPrms.elemCnt = 0U;
                rmInitPrms = &drvObj->rmInitPrms;
                ringNum =  (uint16_t) (rmInitPrms->startTxCh + drvObj->txChOffset);

                retVal = Udma_ringAlloc(drvHandle, ringHandle, ringNum, &ringPrms);
                if(UDMA_SOK == retVal)
                {
                    GT_0trace(taskObj->traceMask, GT_ERR,
                        " Ring alloc didnot fail for zero element count check!!\n");
                    break;
                }

                /* Invalid Ring Mode check */
                for(checkRingMode = TISCI_MSG_VALUE_RM_RING_MODE_MESSAGE;
                    checkRingMode <= TISCI_MSG_VALUE_RM_RING_MODE_QM;
                    checkRingMode++)
                {
                    UdmaRingPrms_init(&ringPrms);
                    ringPrms.ringMem = ringMem;
                    ringPrms.ringMemSize = ringMemSize;
                    ringPrms.mode = checkRingMode;
                    ringPrms.elemCnt = elemCnt;
                    rmInitPrms = &drvObj->rmInitPrms;
                    ringNum =  (uint16_t) (rmInitPrms->startTxCh + drvObj->txChOffset);

                    retVal = Udma_ringAlloc(drvHandle, ringHandle, ringNum, &ringPrms);
                    if(UDMA_SOK == retVal)
                    {
                        GT_0trace(taskObj->traceMask, GT_ERR,
                            " Ring alloc didnot fail for invalid Ring Mode!!\n");
                        break;
                    }
                }

                /* Ring mem size check */
                UdmaRingPrms_init(&ringPrms);
                ringPrms.ringMem = ringMem;
                ringPrms.ringMemSize = ringMemSize;
                ringPrms.mode = ringMode;
                ringPrms.elemCnt = elemCnt + 1U;
                rmInitPrms = &drvObj->rmInitPrms;
                ringNum =  (uint16_t) (rmInitPrms->startTxCh + drvObj->txChOffset);

                retVal = Udma_ringAlloc(drvHandle, ringHandle, ringNum, &ringPrms);
                if(UDMA_SOK == retVal)
                {
                    GT_0trace(taskObj->traceMask, GT_ERR,
                        " Ring alloc didnot fail for memsize check!!\n");
                    break;
                }

#if((UDMA_NUM_MAPPED_TX_GROUP + UDMA_NUM_MAPPED_RX_GROUP) > 0)
                /* Ring incorrect mappedRingGrp check */
                UdmaRingPrms_init(&ringPrms);
                ringPrms.ringMem = ringMem;
                ringPrms.mode = ringMode;
                ringPrms.elemCnt = elemCnt;
                ringPrms.mappedRingGrp = UDMA_NUM_MAPPED_TX_GROUP + UDMA_NUM_MAPPED_RX_GROUP;
                rmInitPrms = &drvObj->rmInitPrms;
                ringPrms.mappedChNum = rmInitPrms->startMappedTxCh[0U];
                retVal = Udma_ringAlloc(drvHandle, ringHandle, ringNum, &ringPrms);
                if(UDMA_SOK == retVal)
                {
                    GT_0trace(taskObj->traceMask, GT_ERR,
                        " Ring alloc did not fail for incorrect mappedRingGrp check!!\n");
                    break;
                }
#endif

                /* Ring mem size check skip with more memory - should not return error */
                UdmaRingPrms_init(&ringPrms);
                ringPrms.ringMem = ringMem;
                ringPrms.mode = ringMode;
                ringPrms.elemCnt = elemCnt + 1U;
                rmInitPrms = &drvObj->rmInitPrms;
                ringNum =  (uint16_t) (rmInitPrms->startTxCh + drvObj->txChOffset);

                retVal = Udma_ringAlloc(drvHandle, ringHandle, ringNum, &ringPrms);
                if(UDMA_SOK != retVal)
                {
                    GT_0trace(taskObj->traceMask, GT_ERR,
                        " Ring alloc memsize check failed when it shoudl not!!\n");
                    break;
                }
                else
                {
                    /* Free-up the allocated ring */
                    retVal = Udma_ringFree(ringHandle);
                    if(UDMA_SOK != retVal)
                    {
                        GT_0trace(taskObj->traceMask, GT_ERR, " Ring free failed!!\n");
                        break;
                    }
                }
            }

            if(UDMA_SOK != retVal)
            {
                break;
            }
        }
    }

    if(NULL != ringMem)
    {
        retVal += Utils_memFree(heapId, ringMem, ringMemSize);
        if(UDMA_SOK != retVal)
        {
            GT_0trace(taskObj->traceMask, GT_ERR, " Ring free failed!!\n");
        }
    }

    return (retVal);
}

static int32_t udmaTestRingAttachTestLoop(UdmaTestTaskObj *taskObj)
{
    int32_t             retVal = UDMA_SOK;
    uint16_t            ringNum = UDMA_RING_ANY;
    uint32_t            instId, qCnt, ringMode;
    uint32_t            elemCnt = 100U, ringMemSize;
    uint32_t            heapId = UTILS_MEM_HEAP_ID_MSMC;
    uint32_t            numMappedRingGrp, ringGrp, mappedRingGrp = 0U;
    Udma_DrvHandle      drvHandle;
    Udma_RingPrms       ringPrms;
    Udma_RingObject     ringObj, attachRingObj;
    Udma_RingHandle     ringHandle = &ringObj, attachRingHandle = &attachRingObj;
    void               *ringMem = NULL;
    uint64_t            ringData;
    Udma_RmInitPrms    *rmInitPrms;
    char *instanceIdStr[]    = {"MAIN", "MCU", "BCDMA", "PKTDMA"};
    char *mappedRingGrpStr[] = { "CPSW TX", "SAUL TX", "ICSSG0 TX", "ICSSG1_TX",
                                 "CPSW RX", "SAUL RX", "ICSSG0 RX", "ICSSG1_RX"};
    char *ringModeString[]   = {"RING", "MESSAGE"};
#if ((UDMA_NUM_MAPPED_TX_GROUP + UDMA_NUM_MAPPED_RX_GROUP) > 0)
    Udma_MappedChRingAttributes  chAttr;
#endif

    ringMemSize = elemCnt * sizeof (uint64_t);
    ringMem = Utils_memAlloc(heapId, ringMemSize, UDMA_CACHELINE_ALIGNMENT);
    if(NULL == ringMem)
    {
        retVal = UDMA_EALLOC;
        GT_0trace(taskObj->traceMask, GT_ERR, " Ring memory allocation failure\r\n");
    }

    if(UDMA_SOK == retVal)
    {
        for(instId = UDMA_INST_ID_START; instId <= UDMA_INST_ID_MAX; instId++)
        {
            for(ringMode = UDMA_TEST_RING_MODE_DEFAULT_START;
                ringMode <= UDMA_TEST_RING_MODE_DEFAULT_STOP;
                ringMode++)
            {
                GT_2trace(taskObj->traceMask, GT_INFO1,
                          " Testing for Inst: %s, Ring Mode: %s...\r\n",
                          instanceIdStr[instId], ringModeString[ringMode]);
                drvHandle = &taskObj->testObj->drvObj[instId];
                Udma_DrvObjectInt  *drvObj = (Udma_DrvObjectInt *) drvHandle;

#if ((UDMA_NUM_MAPPED_TX_GROUP + UDMA_NUM_MAPPED_RX_GROUP) > 0)
                if(UDMA_INST_ID_PKTDMA_0 == instId)
                {
                    numMappedRingGrp = UDMA_NUM_MAPPED_TX_GROUP + UDMA_NUM_MAPPED_RX_GROUP;
                }
                else
#endif
                {
                    numMappedRingGrp = 0U;
                }

                for(ringGrp = 0U; ringGrp <= numMappedRingGrp; ringGrp++)
                {
                    ringNum = UDMA_RING_ANY;

                    rmInitPrms = &drvObj->rmInitPrms;
                    UdmaRingPrms_init(&ringPrms);
                    ringPrms.ringMem = ringMem;
                    ringPrms.ringMemSize = ringMemSize;
                    ringPrms.mode = ringMode;
                    ringPrms.elemCnt = elemCnt;

#if ((UDMA_NUM_MAPPED_TX_GROUP + UDMA_NUM_MAPPED_RX_GROUP) > 0)
                    if(ringGrp != 0U)
                    {
                        mappedRingGrp = ringGrp - 1U;

                        GT_1trace(taskObj->traceMask, GT_INFO1,
                                  " Testing for PKTDMA Mapped Ring Group : %s ...\r\n",
                                  mappedRingGrpStr[mappedRingGrp]);


                        if(0U == rmInitPrms->numMappedRing[mappedRingGrp])
                        {
                            GT_1trace(taskObj->traceMask, GT_INFO1,
                                      " Skipping the Test for PKTDMA Mapped Ring Group : %s , since no rings are reserved!!\r\n",
                                      mappedRingGrpStr[mappedRingGrp]);
                            continue;
                        }

                        ringPrms.mappedRingGrp = mappedRingGrp;
                        ringPrms.mappedChNum   = udmaTestGetMappedRingChNum(drvHandle, mappedRingGrp, rmInitPrms->startMappedRing[mappedRingGrp]);
                        Udma_getMappedChRingAttributes(drvHandle, mappedRingGrp, ringPrms.mappedChNum, &chAttr);
                        if(0U == chAttr.numFreeRing)
                        {
                            /* When no free mapped rings are avilable for PKTDMA Mapped Channels,
                             * #Udma_ringAlloc with #UDMA_RING_ANY will fail.
                             * So use the default ring for that mapped channel */
                            ringNum = chAttr.defaultRing;
                        }
                    }
#endif
                    /* Since no free/extra rings are available in BCDMA/PKTDMA-Unmapped Channels,
                     * #Udma_ringAlloc with #UDMA_RING_ANY will fail.
                     * So get the default ringNum for a channel */
                    ringNum =  (uint16_t) (rmInitPrms->startTxCh + drvObj->txChOffset);

                    /* Allocate ring */
                    retVal = Udma_ringAlloc(drvHandle, ringHandle, ringNum, &ringPrms);
                    if(UDMA_SOK != retVal)
                    {
                        GT_0trace(taskObj->traceMask, GT_ERR, " Ring alloc failed!!\n");
                        break;
                    }

                    /* Attach to the allocated ring */
                    ringNum = Udma_ringGetNum(ringHandle);
                    retVal = Udma_ringAttach(drvHandle, attachRingHandle, ringNum);
                    if(UDMA_SOK != retVal)
                    {
                        GT_0trace(taskObj->traceMask, GT_ERR, " Ring attach failed!!\n");
                        break;
                    }

                    /* Queue and check ring operation through attach handle */
                    for(qCnt = 0U; qCnt < elemCnt; qCnt++)
                    {
                        ringData = ((uint64_t) qCnt | (uint64_t) 0xDEADBEEF00000000UL);
                        retVal = Udma_ringQueueRaw(attachRingHandle, ringData);
                        if(UDMA_SOK != retVal)
                        {
                            GT_0trace(taskObj->traceMask, GT_ERR, " Ring queue failed!!\n");
                            break;
                        }
                    }
                    if(UDMA_SOK != retVal)
                    {
                        break;
                    }

                    /* Check if the HW occupancy is same as what is queued */
                    if(udmaTestCompareRingHwOccDriver(attachRingHandle, elemCnt, UDMA_TEST_RING_ACC_DIRECTION_FORWARD) != UDMA_SOK)
                    {
                        GT_0trace(taskObj->traceMask, GT_ERR, " Ring element count mismatch!!\n");
                        retVal = UDMA_EFAIL;
                        break;
                    }

                    /* Dequeue and flush the ring */
                    for(qCnt = 0U; qCnt < elemCnt; qCnt++)
                    {
                        ringData = 0UL;
                        /* In case of LCDMA, #Udma_ringDequeueRaw will dequeue from the
                        * reverse direction in Dual ring. So it can't be used to dequeue the
                        * descriptors in the forward direction. In this casse use #Udma_ringFlushRaw */
                        retVal = Udma_ringFlushRaw(attachRingHandle, &ringData);
                        if(UDMA_SOK != retVal)
                        {
                            GT_0trace(taskObj->traceMask, GT_ERR, " Ring dequeue failed!!\n");
                            break;
                        }

                        if(ringData != ((uint64_t) qCnt | (uint64_t) 0xDEADBEEF00000000UL))
                        {
                            GT_0trace(taskObj->traceMask, GT_ERR, " Ring data mismatch!!\n");
                            break;
                        }
                    }
                    if(UDMA_SOK != retVal)
                    {
                        break;
                    }

                    /* Check if the HW occupancy is zero */
                    if(udmaTestCompareRingHwOccDriver(attachRingHandle, 0U, UDMA_TEST_RING_ACC_DIRECTION_FORWARD) != UDMA_SOK)
                    {
                        GT_0trace(taskObj->traceMask, GT_ERR, " Ring not empty!!\n");
                        retVal = UDMA_EFAIL;
                        break;
                    }

                    retVal = Udma_ringDetach(attachRingHandle);
                    if(UDMA_SOK != retVal)
                    {
                        GT_0trace(taskObj->traceMask, GT_ERR, " Ring detach failed!!\n");
                        break;
                    }

                    retVal = Udma_ringFree(ringHandle);
                    if(UDMA_SOK != retVal)
                    {
                        GT_0trace(taskObj->traceMask, GT_ERR, " Ring free failed!!\n");
                        break;
                    }

                    if(ringGrp != 0U)
                    {
                        GT_1trace(taskObj->traceMask, GT_INFO1,
                                  " Testing for PKTDMA Mapped Ring Group : %s passed!!\r\n",
                                  mappedRingGrpStr[mappedRingGrp]);
                    }

                }

                if(UDMA_SOK == retVal)
                {
                    GT_2trace(taskObj->traceMask, GT_INFO1,
                              " Testing for Inst: %s, Ring Mode: %s passed!!\r\n",
                              instanceIdStr[instId], ringModeString[ringMode]);
                }
                else
                {
                    break;
                }
            }

            if(UDMA_SOK != retVal)
            {
                break;
            }
        }
    }

    if(NULL != ringMem)
    {
        retVal += Utils_memFree(heapId, ringMem, ringMemSize);
        if(UDMA_SOK != retVal)
        {
            GT_0trace(taskObj->traceMask, GT_ERR, " Ring free failed!!\n");
        }
    }

    return (retVal);
}

static int32_t udmaTestRingResetTestLoop(UdmaTestTaskObj *taskObj)
{
    int32_t             retVal = UDMA_SOK;
    uint16_t            ringNum = UDMA_RING_ANY;
    uint32_t            instId, qCnt, ringMode;
    uint32_t            elemCnt = 500U, ringMemSize;
    uint32_t            heapId = UTILS_MEM_HEAP_ID_MSMC;
    Udma_DrvHandle      drvHandle;
    Udma_RingPrms       ringPrms;
    Udma_RingObject     ringObj;
    Udma_RingHandle     ringHandle = &ringObj;
    void               *ringMem = NULL;
    uint64_t            ringData;
    Udma_RmInitPrms    *rmInitPrms;
    char *instanceIdStr[] = {"MAIN", "MCU", "BCDMA", "PKTDMA"};
    char *ringModeString[] = {"RING", "MESSAGE"};

    ringMemSize = elemCnt * sizeof (uint64_t);
    ringMem = Utils_memAlloc(heapId, ringMemSize, UDMA_CACHELINE_ALIGNMENT);
    if(NULL == ringMem)
    {
        retVal = UDMA_EALLOC;
        GT_0trace(taskObj->traceMask, GT_ERR, " Ring memory allocation failure\r\n");
    }

    if(UDMA_SOK == retVal)
    {
        for(instId = UDMA_INST_ID_START; instId <= UDMA_INST_ID_MAX; instId++)
        {
            for(ringMode = UDMA_TEST_RING_MODE_DEFAULT_START;
                ringMode <= UDMA_TEST_RING_MODE_DEFAULT_STOP;
                ringMode++)
            {
                GT_2trace(taskObj->traceMask, GT_INFO1,
                          " Testing for Inst: %s, Ring Mode: %s...\r\n",
                          instanceIdStr[instId], ringModeString[ringMode]);
                drvHandle = &taskObj->testObj->drvObj[instId];
                Udma_DrvObjectInt  *drvObj = (Udma_DrvObjectInt *) drvHandle;

                UdmaRingPrms_init(&ringPrms);
                ringPrms.ringMem = ringMem;
                ringPrms.ringMemSize = ringMemSize;
                ringPrms.mode = ringMode;
                ringPrms.elemCnt = elemCnt;
                /* Allocate a FQ ring */
                rmInitPrms = &drvObj->rmInitPrms;
                ringNum =  (uint16_t) (rmInitPrms->startTxCh + drvObj->txChOffset);
                retVal = Udma_ringAlloc(drvHandle, ringHandle, ringNum, &ringPrms);
                if(UDMA_SOK != retVal)
                {
                    GT_0trace(taskObj->traceMask, GT_ERR, " Ring alloc failed!!\n");
                    break;
                }

                /* Queue through proxy */
                for(qCnt = 0U; qCnt < elemCnt; qCnt++)
                {
                    ringData = ((uint64_t) qCnt | (uint64_t) 0xDEADBEEF00000000UL);
                    retVal = Udma_ringQueueRaw(ringHandle, ringData);
                    if(UDMA_SOK != retVal)
                    {
                        GT_0trace(taskObj->traceMask, GT_ERR, " Proxy Ring queue failed!!\n");
                        break;
                    }
                }
                if(UDMA_SOK != retVal)
                {
                    break;
                }

                /* Check if the HW occupancy is same as what is queued */
                if(udmaTestCompareRingHwOccDriver(ringHandle, elemCnt, UDMA_TEST_RING_ACC_DIRECTION_FORWARD) != UDMA_SOK)
                {
                    GT_0trace(taskObj->traceMask, GT_ERR, " Ring element count mismatch!!\n");
                    retVal = UDMA_EFAIL;
                    break;
                }

                /* Free the ring and alloc again - this will do the ring reset automatically */
                retVal = Udma_ringFree(ringHandle);
                if(UDMA_SOK != retVal)
                {
                    GT_0trace(taskObj->traceMask, GT_ERR, " Ring free failed!!\n");
                    break;
                }
                retVal = Udma_ringAlloc(drvHandle, ringHandle, ringNum, &ringPrms);
                if(UDMA_SOK != retVal)
                {
                    GT_0trace(taskObj->traceMask, GT_ERR, " Ring alloc failed!!\n");
                    break;
                }

                /* Check if the HW occupancy is zero */
                if(udmaTestCompareRingHwOccDriver(ringHandle, 0U, UDMA_TEST_RING_ACC_DIRECTION_FORWARD) != UDMA_SOK)
                {
                    GT_0trace(taskObj->traceMask, GT_ERR, " Ring reset failed!!\n");
                    retVal = UDMA_EFAIL;
                    break;
                }

                /* Queue the descriptors */
                for(qCnt = 0U; qCnt < elemCnt; qCnt++)
                {
                    ringData = ((uint64_t) qCnt | (uint64_t) 0xDEADBEEF00000000UL);
                    retVal = Udma_ringQueueRaw(ringHandle, ringData);
                    if(UDMA_SOK != retVal)
                    {
                        GT_0trace(taskObj->traceMask, GT_ERR, " Proxy Ring queue failed!!\n");
                        break;
                    }
                }
                if(UDMA_SOK != retVal)
                {
                    break;
                }

                /* Check if the HW occupancy is same as what is queued */
                if(udmaTestCompareRingHwOccDriver(ringHandle, elemCnt, UDMA_TEST_RING_ACC_DIRECTION_FORWARD) != UDMA_SOK)
                {
                    GT_0trace(taskObj->traceMask, GT_ERR, " Ring element count mismatch!!\n");
                    retVal = UDMA_EFAIL;
                    break;
                }

                /* Dequeue the descriptors */
                for(qCnt = 0U; qCnt < elemCnt; qCnt++)
                {
                    ringData = 0UL;
                    /* In case of LCDMA, #Udma_ringDequeueRaw will dequeue from the
                     * reverse direction in Dual ring. So it can't be used to dequeue the
                     * descriptors in the forward direction. In this casse use #Udma_ringFlushRaw */
                    retVal = Udma_ringFlushRaw(ringHandle, &ringData);
                    if(UDMA_SOK != retVal)
                    {
                        GT_0trace(taskObj->traceMask, GT_ERR, " Proxy Ring dequeue failed!!\n");
                        break;
                    }

                    if(ringData != ((uint64_t) qCnt | (uint64_t) 0xDEADBEEF00000000UL))
                    {
                        GT_0trace(taskObj->traceMask, GT_ERR, " Ring data mismatch!!\n");
                        break;
                    }
                }
                if(UDMA_SOK != retVal)
                {
                    break;
                }

                /* Check if the HW occupancy is zero */
                if(udmaTestCompareRingHwOccDriver(ringHandle, 0U, UDMA_TEST_RING_ACC_DIRECTION_FORWARD) != UDMA_SOK)
                {
                    GT_0trace(taskObj->traceMask, GT_ERR, " Ring not empty!!\n");
                    retVal = UDMA_EFAIL;
                    break;
                }

                retVal = Udma_ringFree(ringHandle);
                if(UDMA_SOK != retVal)
                {
                    GT_0trace(taskObj->traceMask, GT_ERR, " Ring free failed!!\n");
                    break;
                }

                GT_2trace(taskObj->traceMask, GT_INFO1,
                          " Testing for Inst: %s, Ring Mode: %s passed!!\r\n",
                          instanceIdStr[instId], ringModeString[ringMode]);
            }

            if(UDMA_SOK != retVal)
            {
                break;
            }
        }
    }

    if(NULL != ringMem)
    {
        retVal += Utils_memFree(heapId, ringMem, ringMemSize);
        if(UDMA_SOK != retVal)
        {
            GT_0trace(taskObj->traceMask, GT_ERR, " Ring free failed!!\n");
        }
    }

    return (retVal);
}

static int32_t udmaTestRingPrimeTestLoop(UdmaTestTaskObj *taskObj)
{
    int32_t             retVal = UDMA_SOK;
    uint32_t            instId, qCnt, ringMode;
    uint32_t            elemCnt = 500U, ringMemSize;
    uint32_t            heapId = UTILS_MEM_HEAP_ID_MSMC;
    Udma_DrvHandle      drvHandle;
    Udma_RingPrms       ringPrms;
    Udma_RingObject     ringObj;
    Udma_RingHandle     ringHandle = &ringObj;
    void               *ringMem = NULL;
    uint64_t            ringData;
    char *instanceIdStr[] = {"MAIN", "MCU", "BCDMA", "PKTDMA"};
    char *ringModeString[] = {"RING", "MESSAGE"};

    ringMemSize = elemCnt * sizeof (uint64_t);
    ringMem = Utils_memAlloc(heapId, ringMemSize, UDMA_CACHELINE_ALIGNMENT);
    if(NULL == ringMem)
    {
        retVal = UDMA_EALLOC;
        GT_0trace(taskObj->traceMask, GT_ERR, " Ring memory allocation failure\r\n");
    }

    if(UDMA_SOK == retVal)
    {
        for(instId = UDMA_INST_ID_START; instId <= UDMA_INST_ID_MAX; instId++)
        {
            ringMode = TISCI_MSG_VALUE_RM_RING_MODE_RING;
            GT_2trace(taskObj->traceMask, GT_INFO1,
                      " Testing for Inst: %s, Ring Mode: %s...\r\n",
                      instanceIdStr[instId], ringModeString[ringMode]);
            drvHandle = &taskObj->testObj->drvObj[instId];

            UdmaRingPrms_init(&ringPrms);
            ringPrms.ringMem = ringMem;
            ringPrms.ringMemSize = ringMemSize;
            ringPrms.mode = ringMode;
            ringPrms.elemCnt = elemCnt;

            /* Allocate a free ring */
            retVal = Udma_ringAlloc(drvHandle, ringHandle, UDMA_RING_ANY, &ringPrms);
            if(UDMA_SOK != retVal)
            {
                GT_0trace(taskObj->traceMask, GT_ERR, " Ring alloc failed!!\n");
                break;
            }

            /* Check ring mode after allocation */
            if (Udma_ringGetMode(ringHandle) != TISCI_MSG_VALUE_RM_RING_MODE_RING)
            {
                GT_0trace(taskObj->traceMask, GT_ERR, " Ring mode mismatch!!\n");
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
                ringData = ((uint64_t) qCnt | (uint64_t) 0xDEADBEEF00000000UL);
                Udma_ringPrime(ringHandle, ringData);
            }

            /* Check if the HW occupancy is zero as the queue is not committed */
            if(udmaTestCompareRingHwOccDriver(ringHandle, 0U, UDMA_TEST_RING_ACC_DIRECTION_FORWARD) != UDMA_SOK)
            {
                GT_0trace(taskObj->traceMask, GT_ERR, " Ring element count mismatch!!\n");
                retVal = UDMA_EFAIL;
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
                GT_0trace(taskObj->traceMask, GT_ERR, " Ring rwIdx value mismatch!!\n");
                retVal = UDMA_EFAIL;
                break;
            }

            /* Do Cache invalidate before reading ring elements */
            CacheP_inv(ringMem, ringMemSize, CacheP_TYPE_ALLD);

            /* Update cfg->occ count */
            Udma_ringGetReverseRingOcc(ringHandle);

            /* Dequeue using prime read API */
            for(qCnt = 0U; qCnt < elemCnt; qCnt++)
            {
                ringData = 0UL;
                Udma_ringPrimeRead(ringHandle, &ringData);
                if(ringData != ((uint64_t) qCnt | (uint64_t) 0xDEADBEEF00000000UL))
                {
                    GT_0trace(taskObj->traceMask, GT_ERR, " Ring data mismatch!!\n");
                    break;
                }
            }
            if(UDMA_SOK != retVal)
            {
                break;
            }

            /* Check if the HW occupancy is same as elemCnt as the queue is not committed */
            if(udmaTestCompareRingHwOccDriver(ringHandle, elemCnt, UDMA_TEST_RING_ACC_DIRECTION_FORWARD) != UDMA_SOK)
            {
                GT_0trace(taskObj->traceMask, GT_ERR, " Ring element count mismatch!!\n");
                retVal = UDMA_EFAIL;
                break;
            }

            /* Set door bell value as -1 * elemCnt to reduce ring occupancy after reading */
            Udma_ringSetDoorBell(ringHandle, (-1 * (int32_t)elemCnt));

            /* Check if the HW occupancy is zero */
            if(udmaTestCompareRingHwOccDriver(ringHandle, 0U, UDMA_TEST_RING_ACC_DIRECTION_FORWARD) != UDMA_SOK)
            {
                GT_0trace(taskObj->traceMask, GT_ERR, " Ring not empty!!\n");
                retVal = UDMA_EFAIL;
                break;
            }

            retVal = Udma_ringFree(ringHandle);
            if(UDMA_SOK != retVal)
            {
                GT_0trace(taskObj->traceMask, GT_ERR, " Ring free failed!!\n");
                break;
            }

            GT_2trace(taskObj->traceMask, GT_INFO1,
                      " Testing for Inst: %s, Ring Mode: %s passed!!\r\n",
                      instanceIdStr[instId], ringModeString[ringMode]);
        }
    }

    if(NULL != ringMem)
    {
        retVal += Utils_memFree(heapId, ringMem, ringMemSize);
        if(UDMA_SOK != retVal)
        {
            GT_0trace(taskObj->traceMask, GT_ERR, " Ring free failed!!\n");
        }
    }

    return (retVal);
}

static void udmaTestRingEventCb(Udma_EventHandle eventHandle,
                                uint32_t eventType,
                                void *appData)
{
    SemaphoreP_Object  *transferDoneSem = (SemaphoreP_Object *) appData;

    if(transferDoneSem != NULL)
    {
        if(UDMA_EVENT_TYPE_RING == eventType)
        {
            SemaphoreP_post(transferDoneSem);
        }
        else
        {
            gUdmaTestRingResult = UDMA_EFAIL;
        }
    }
    else
    {
        gUdmaTestRingResult = UDMA_EFAIL;
    }

    return;
}
