/*
 *  Copyright (C) 2023 Texas Instruments Incorporated
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
 *  \file    HeapP_internal_pos_test.c
 *
 *  \brief    This file contains HeapP_internal positive unit test code.
 *
 *  \details  nortos unit tests
 **/

#include <stdio.h>
#include <string.h>
#include <inttypes.h>
#include <kernel/dpl/SystemP.h>
#include <kernel/dpl/HwiP.h>
#include <kernel/dpl/CacheP.h>
#include <kernel/dpl/SemaphoreP.h>
#include <kernel/dpl/TaskP.h>
#include <kernel/dpl/ClockP.h>
#include <kernel/dpl/HeapP.h>
#include <kernel/dpl/CycleCounterP.h>
#include <kernel/dpl/EventP.h>
#include <kernel/dpl/QueueP.h>
#include <drivers/soc.h>
#include <unity.h>
#include "ti_drivers_open_close.h"
#include <kernel/nortos/dpl/common/HeapP_internal.h>

#define MY_HEAP_MEM_SIZE            (2 * 1024U)
#define MY_HEAP_MEM_SIZE_ONE        (0U)

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */

    int32_t   testStatus = SystemP_SUCCESS;
    StaticHeap_t *heap;
    static HeapP_Object gMyHeap;
	void *pv=NULL;
    HeapP_MemStats *pxHeapStats;
    uint8_t gMyHeapMem[MY_HEAP_MEM_SIZE];


/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

/*
Test to verify vHeapCreateStatic API with valid parameters
*/
void test_vHeapCreateStatic(void * args)
{

    if (testStatus == SystemP_SUCCESS)
    {
      vHeapCreateStatic(heap,gMyHeapMem,MY_HEAP_MEM_SIZE);

    }

	if (testStatus != SystemP_SUCCESS)
    {
        testStatus = SystemP_FAILURE;
        DebugP_log("pos_Test: failure on line no %d \n", __LINE__);
    }
    TEST_ASSERT_EQUAL_INT32(testStatus,SystemP_SUCCESS);
}

/*
Test to verify vHeapDelete API with valid parameters
*/ 
void test_vHeapDelete(void * args)
{
    if (testStatus == SystemP_SUCCESS)
    {
        vHeapDelete(heap);
    }

	if (testStatus != SystemP_SUCCESS)
    {
        testStatus = SystemP_FAILURE;
        DebugP_log("pos_Test: failure on line no %d \n", __LINE__);

    }
    TEST_ASSERT_EQUAL_INT32(testStatus,SystemP_SUCCESS);
}

/*
Test to verify pvHeapMalloc API with xWantedSize as 64U value.
*/
void test_pvHeapMalloc(void * args)
{
    if (testStatus == SystemP_SUCCESS)
    {
        size_t xWantedSize=64U;
        heap->xBlockAllocatedBit = ( ( size_t ) 1 ) << ( ( sizeof( size_t ) * (( size_t ) 8)  ) - 1 );
        heap -> xStart.pxNextFreeBlock = heap->pvHeap;
        if (pvHeapMalloc(heap,xWantedSize) != NULL)
        {
            testStatus = SystemP_FAILURE;
        }
    }

	if (testStatus != SystemP_SUCCESS)
    {
        testStatus = SystemP_FAILURE;
        DebugP_log("pos_Test: failure on line no %d \n", __LINE__);

    }
    TEST_ASSERT_EQUAL_INT32(testStatus,SystemP_SUCCESS);
 }

/*
Test to verify pvHeapMalloc API by calling it multiple times to achieve Branch and MC/DC.
*/
void test_pvHeapMallocThree(void * args)
{
    if (testStatus == SystemP_SUCCESS)
    {
        uint8_t *ptr =  NULL;
        size_t xWantedSize=200U;
        vHeapGetHeapStats(heap,pxHeapStats);
        pvHeapMalloc(heap,400);
        pvHeapMalloc(heap,xWantedSize);
        pvHeapMalloc(heap,xWantedSize);
        vHeapGetHeapStats(heap,pxHeapStats);
        pvHeapMalloc(heap,415);
        pvHeapMalloc(heap,xWantedSize);
        pvHeapMalloc(heap,xWantedSize);
        ptr = pvHeapMalloc(heap,0);
        vHeapFree(heap, ptr);
        pvHeapMalloc(heap,xWantedSize);
        vHeapGetHeapStats(heap,pxHeapStats);
        pvHeapMalloc(heap,250);
        vHeapGetHeapStats(heap,pxHeapStats);
    }

	if (testStatus != SystemP_SUCCESS)
    {
        testStatus = SystemP_FAILURE;
        DebugP_log("pos_Test: failure on line no %d \n", __LINE__);

    }
    TEST_ASSERT_EQUAL_INT32(testStatus,SystemP_SUCCESS);
}

/*
Test to verify pvHeapMalloc API by creating and free those pointers with different xWantedSize for Branch and MC/DC coverage.
*/
void test_pvHeapMallocFive(void * args)
{
    if (testStatus == SystemP_SUCCESS)
    {
        uint8_t *ptr = NULL;
        uint8_t *ptr1 = NULL;
        uint8_t *ptr2 = NULL;

        pvHeapMalloc(heap,400);
        pvHeapMalloc(heap,600);
        pvHeapMalloc(heap,450);
        pvHeapMalloc(heap,200);
        pvHeapMalloc(heap,100);
        pvHeapMalloc(heap,0);
        pvHeapMalloc(heap,50);
        ptr=(uint8_t *)pvHeapMalloc(heap,50);
        ptr1=(uint8_t *)pvHeapMalloc(heap,100);
        ptr2=(uint8_t *)pvHeapMalloc(heap,200);
        vHeapFree(heap, ptr);
        vHeapFree(heap, ptr1);
        vHeapFree(heap, ptr2);
        pvHeapMalloc(heap,400);

    }

	if (testStatus != SystemP_SUCCESS)
    {
        testStatus = SystemP_FAILURE;
        DebugP_log("pos_Test: failure on line no %d \n", __LINE__);

    }
    TEST_ASSERT_EQUAL_INT32(testStatus,SystemP_SUCCESS);
}

/*
Test to verify pvHeapMalloc API with xWantedSize as 0U value.
*/
void test_pvHeapMallocFour(void * args)
{
    if (testStatus == SystemP_SUCCESS)
    {
        size_t xWantedSize=0U;
        pvHeapMalloc(heap,xWantedSize);
    }

	if (testStatus != SystemP_SUCCESS)
    {
        testStatus = SystemP_FAILURE;
        DebugP_log("pos_Test: failure on line no %d \n", __LINE__);

    }
    TEST_ASSERT_EQUAL_INT32(testStatus,SystemP_SUCCESS);
}

/*
Test to verify xHeapGetFreeHeapSize API with valid parameters.
*/
void test_xHeapGetFreeHeapSize(void * args)
{

    if (testStatus == SystemP_SUCCESS)
    {
        if (xHeapGetFreeHeapSize(heap) == FALSE)
        {
            testStatus = SystemP_FAILURE;
        }
    }

	if (testStatus != SystemP_SUCCESS)
    {
        testStatus = SystemP_FAILURE;
        DebugP_log("pos_Test: failure on line no %d \n", __LINE__);

    }
    TEST_ASSERT_EQUAL_INT32(testStatus,SystemP_SUCCESS);
}

/*
Test to verify xHeapGetMinimumEverFreeHeapSize API with valid parameters.
*/
void test_xHeapGetMinimumEverFreeHeapSize(void * args)
{

    if (testStatus == SystemP_SUCCESS)
    {
        if (xHeapGetMinimumEverFreeHeapSize(heap) == FALSE)
        {
            testStatus = SystemP_FAILURE;
        }
    }

	if (testStatus != SystemP_SUCCESS)
    {
        testStatus = SystemP_FAILURE;
        DebugP_log("pos_Test: failure on line no %d \n", __LINE__);

    }
    TEST_ASSERT_EQUAL_INT32(testStatus,SystemP_SUCCESS);
}

/*
Test to verify vHeapGetHeapStats API with valid parameters.
*/
void test_vHeapGetHeapStats(void * args)
{

    if (testStatus == SystemP_SUCCESS)
    {
        heap -> xStart.pxNextFreeBlock = heap->pvHeap;
        vHeapGetHeapStats(heap,pxHeapStats);

    }

	if (testStatus != SystemP_SUCCESS)
    {
        testStatus = SystemP_FAILURE;
        DebugP_log("pos_Test: failure on line no %d \n", __LINE__);

    }
    TEST_ASSERT_EQUAL_INT32(testStatus,SystemP_SUCCESS);
}

/*
Test to verify vHeapGetHeapStats API with multiple pvHeapMalloc calls for achieving Branch and MC/DC.
*/
void test_vHeapGetHeapStatsOne(void * args)
{
    if (testStatus == SystemP_SUCCESS)
    {
        size_t xWantedSize=64U;
        vHeapGetHeapStats(heap,pxHeapStats);
        pvHeapMalloc(heap,xWantedSize);
        pvHeapMalloc(heap,xWantedSize);
        pvHeapMalloc(heap,xWantedSize);
    }

	if (testStatus != SystemP_SUCCESS)
    {
        testStatus = SystemP_FAILURE;
        DebugP_log("pos_Test: failure on line no %d \n", __LINE__);

    }
    TEST_ASSERT_EQUAL_INT32(testStatus,SystemP_SUCCESS);
}

/*
Test to verify pvHeapMalloc API for achieving Branch and MC/DC.
*/
void test_pvHeapMallocOne(void * args)
{
    if (testStatus == SystemP_SUCCESS)
    {
        size_t xWantedSize = HeapP_getFreeHeapSize(&gMyHeap);
        heap->xStart.pxNextFreeBlock = NULL;
        if (pvHeapMalloc(heap,xWantedSize) != NULL)
        {
            testStatus = SystemP_FAILURE;
        }
    }

	if (testStatus != SystemP_SUCCESS)
    {
        testStatus = SystemP_FAILURE;
        DebugP_log("pos_Test: failure on line no %d \n", __LINE__);

    }
    TEST_ASSERT_EQUAL_INT32(testStatus,SystemP_SUCCESS);
}

/*
* Test to verify vHeapCreateStatic with xTotalHeapSize as MY_HEAP_MEM_SIZE_ONE to achieve Branch and MC/DC.
*/
void test_vHeapCreateStaticOne(void * args)
{

    if (testStatus == SystemP_SUCCESS)
    {
        StaticHeap_t *heap = NULL;
        size_t uxAddress = 65U;
        heap->pvHeap =(void *)uxAddress;
      vHeapCreateStatic(heap,gMyHeapMem,MY_HEAP_MEM_SIZE_ONE);

    }

	if (testStatus != SystemP_SUCCESS)
    {
        testStatus = SystemP_FAILURE;
        DebugP_log("pos_Test: failure on line no %d \n", __LINE__);
    }
    TEST_ASSERT_EQUAL_INT32(testStatus,SystemP_SUCCESS);
}

/*
* Test to verify vHeapCreateStatic with xTotalHeapSize as MY_HEAP_MEM_SIZE and *pvHeap as &gMyHeapMem[1] to achieve Branch and MC/DC.
*/
void test_vHeapCreateStaticTwo(void * args)
{

    if (testStatus == SystemP_SUCCESS)
    {
        StaticHeap_t *heap = NULL;
        size_t uxAddress = 65U;
        heap->pvHeap =(void *)uxAddress;
      vHeapCreateStatic(heap,&gMyHeapMem[1],MY_HEAP_MEM_SIZE);

    }

	if (testStatus != SystemP_SUCCESS)
    {
        testStatus = SystemP_FAILURE;
        DebugP_log("pos_Test: failure on line no %d \n", __LINE__);
    }
    TEST_ASSERT_EQUAL_INT32(testStatus,SystemP_SUCCESS);
}

/*
* Test to verify vHeapFree with valid parameters.
*/
void test_vHeapFree(void * args)
{
    if (testStatus == SystemP_SUCCESS)
    {
        StaticHeap_t *heap = NULL;
        void * vp = NULL;
        vHeapFree(heap,vp);
    }

	if (testStatus != SystemP_SUCCESS)
    {
        testStatus = SystemP_FAILURE;
        DebugP_log("pos_Test: failure on line no %d \n", __LINE__);
    }
    TEST_ASSERT_EQUAL_INT32(testStatus,SystemP_SUCCESS);
}

void test_pos_main(void *args)
{
     RUN_TEST(test_vHeapCreateStatic,6006,NULL);
     RUN_TEST(test_vHeapCreateStaticOne,9339,NULL);
     RUN_TEST(test_xHeapGetFreeHeapSize,6010,NULL);
     RUN_TEST(test_xHeapGetMinimumEverFreeHeapSize,6011,NULL);
     RUN_TEST(test_vHeapGetHeapStats,6012,NULL);
     RUN_TEST(test_pvHeapMallocFive, 10802, NULL);
     RUN_TEST(test_pvHeapMallocFour,10801,NULL);
     RUN_TEST(test_pvHeapMalloc,9340,NULL);
     RUN_TEST(test_pvHeapMallocOne,6008,NULL);
     RUN_TEST(test_pvHeapMallocThree,10800,NULL);
     RUN_TEST(test_vHeapCreateStaticTwo,9339,NULL);
     RUN_TEST(test_vHeapGetHeapStatsOne,10803,NULL);
     RUN_TEST(test_vHeapFree, 6009, NULL);
     RUN_TEST(test_vHeapDelete,6007,NULL);
}
