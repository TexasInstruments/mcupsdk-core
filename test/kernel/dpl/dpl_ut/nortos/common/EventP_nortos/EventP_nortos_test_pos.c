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
 *  \file     EventP_nortos_test_pos.c
 *
 *  \brief    This file contains EventP_nortos API unit test code.
 *
 *  \details  EventP_nortos unit tests
 **/

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

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

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */
#define EVENT_BIT_FROM_PING             (0x000001u)
#define EVENT_BIT_FROM_PONG             (0x000002u)
#define EVENT_BIT_FROM_ISR              (0x100000u)
#define EVENT_BIT2_FROM_ISR             (0x200000u)

static EventP_Object gMyEvent;

/*
* Test to verify EventP_construct API with valid parameters.
*/
void posTest_EventP_construct(void * args)
{
    EventP_Object obj;
    int32_t testStatus = SystemP_SUCCESS;
    if(testStatus == SystemP_SUCCESS)
    {
        if(EventP_construct(&obj) != SystemP_SUCCESS)
        {
            testStatus = SystemP_FAILURE;
        }
    }
    if(testStatus != SystemP_SUCCESS)
    {
        testStatus = SystemP_FAILURE;
        DebugP_log("EventP_nortos_pos_Test: failure on line no. %d \n", __LINE__);
    }
    TEST_ASSERT_EQUAL_INT32(testStatus,SystemP_SUCCESS);
}

/*
* Test to verify EventP_destruct API with valid parameters.
*/
void posTest_EventP_destruct(void * args)
{
    EventP_Object obj;
    int32_t testStatus = SystemP_SUCCESS;
    if(testStatus == SystemP_SUCCESS)
    {
        EventP_destruct(&obj);
    }
    if(testStatus != SystemP_SUCCESS)
    {
        testStatus = SystemP_FAILURE;
        DebugP_log("EventP_nortos_pos_Test: failure on line no. %d \n", __LINE__);
    }
    TEST_ASSERT_EQUAL_INT32(testStatus,SystemP_SUCCESS);
}

/*
* Test to verify EventP_waitBits API with valid parameters.
*/
void posTest_EventP_waitBits(void * args)
{
    int32_t testStatus;
    uint32_t eventBits;

    testStatus = EventP_construct(&gMyEvent);
    TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, testStatus);
    testStatus = EventP_getBits(&gMyEvent, &eventBits);
    TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, testStatus);
    TEST_ASSERT_EQUAL_INT32(0x0, eventBits);

    testStatus = EventP_setBits(&gMyEvent, EVENT_BIT_FROM_PING);
    TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, testStatus);

    testStatus = EventP_getBits(&gMyEvent, &eventBits);
    TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, testStatus);
    TEST_ASSERT_EQUAL_INT32(EVENT_BIT_FROM_PING, eventBits);

    testStatus = EventP_setBits(&gMyEvent, EVENT_BIT_FROM_PONG);
    TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, testStatus);

    testStatus = EventP_getBits(&gMyEvent, &eventBits);
    TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, testStatus);
    TEST_ASSERT_EQUAL_INT32(EVENT_BIT_FROM_PING | EVENT_BIT_FROM_PONG, eventBits);

    testStatus = EventP_waitBits(&gMyEvent, EVENT_BIT_FROM_PING | EVENT_BIT_FROM_PONG, 0, 1, 0 , &eventBits);
    TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, testStatus);
    TEST_ASSERT_EQUAL_INT32(EVENT_BIT_FROM_PING | EVENT_BIT_FROM_PONG, eventBits);

    testStatus = EventP_waitBits(&gMyEvent, EVENT_BIT_FROM_PING | EVENT_BIT_FROM_PONG, 0, 0, 1000 , &eventBits);
    TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, testStatus);
    TEST_ASSERT_EQUAL_INT32(EVENT_BIT_FROM_PING | EVENT_BIT_FROM_PONG, eventBits);

    testStatus = EventP_waitBits(&gMyEvent, EVENT_BIT_FROM_PING | EVENT_BIT_FROM_PONG | EVENT_BIT_FROM_ISR, 1, 1, 10 , &eventBits);
    TEST_ASSERT_EQUAL_INT32(SystemP_TIMEOUT, testStatus);
    TEST_ASSERT_EQUAL_INT32(EVENT_BIT_FROM_PING | EVENT_BIT_FROM_PONG, eventBits);

    testStatus = EventP_waitBits(&gMyEvent, EVENT_BIT_FROM_PING | EVENT_BIT_FROM_PONG | EVENT_BIT_FROM_ISR, 1, 1, 0 , &eventBits);
    TEST_ASSERT_EQUAL_INT32(SystemP_TIMEOUT, testStatus);
    
    testStatus = EventP_clearBits(&gMyEvent, EVENT_BIT_FROM_PING | EVENT_BIT_FROM_PONG);
    TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, testStatus);

    testStatus = EventP_getBits(&gMyEvent, &eventBits);
    TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, testStatus);
    TEST_ASSERT_EQUAL_INT32(0x0, eventBits);
}

/*
* Test to verify EventP_waitBits API by calling it multiple times with different parameters for Branch coverage.
*/
void posTest_EventP_waitBitsOne(void * args)
{
    int32_t testStatus;
    uint32_t eventBits;

     testStatus = EventP_construct(&gMyEvent);
    TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, testStatus);
    testStatus = EventP_getBits(&gMyEvent, &eventBits);
    TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, testStatus);
    TEST_ASSERT_EQUAL_INT32(0x0, eventBits);

    testStatus = EventP_setBits(&gMyEvent, EVENT_BIT_FROM_PING);
    TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, testStatus);

    testStatus = EventP_getBits(&gMyEvent, &eventBits);
    TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, testStatus);
    TEST_ASSERT_EQUAL_INT32(EVENT_BIT_FROM_PING, eventBits);

    testStatus = EventP_waitBits(&gMyEvent, EVENT_BIT_FROM_PING, 0, 0, 0 , &eventBits);
    TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, testStatus);
    TEST_ASSERT_EQUAL_INT32(EVENT_BIT_FROM_PING, eventBits);

    testStatus = EventP_waitBits(&gMyEvent, EVENT_BIT_FROM_PING, 0, 0, SystemP_WAIT_FOREVER , &eventBits);
    TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, testStatus);
    TEST_ASSERT_EQUAL_INT32(EVENT_BIT_FROM_PING, eventBits);

    testStatus = EventP_clearBits(&gMyEvent, EVENT_BIT_FROM_PING);
    TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, testStatus);

    testStatus = EventP_getBits(&gMyEvent, &eventBits);
    TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, testStatus);
    TEST_ASSERT_EQUAL_INT32(0x0, eventBits);
}

/*
* Test to verify EventP_waitBits API by calling it multiple times with EVENT_BIT_FROM_PING and EVENT_BIT_FROM_PONG bits for Branch and MC/DC coverage .
*/
void posTest_EventP_waitBitsTwo(void * args)
{
    int32_t testStatus;
    uint32_t eventBits;

     testStatus = EventP_construct(&gMyEvent);
    TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, testStatus);
    testStatus = EventP_getBits(&gMyEvent, &eventBits);
    TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, testStatus);
    TEST_ASSERT_EQUAL_INT32(0x0, eventBits);

    testStatus = EventP_setBits(&gMyEvent, EVENT_BIT_FROM_PING | EVENT_BIT_FROM_PONG);
    TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, testStatus);

    testStatus = EventP_getBits(&gMyEvent, &eventBits);
    TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, testStatus);
    TEST_ASSERT_EQUAL_INT32(EVENT_BIT_FROM_PING| EVENT_BIT_FROM_PONG, eventBits);

    testStatus = EventP_waitBits(&gMyEvent, EVENT_BIT_FROM_PING | EVENT_BIT_FROM_PONG, 0, 0, 0 , &eventBits);
    TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, testStatus);
    TEST_ASSERT_EQUAL_INT32(EVENT_BIT_FROM_PING | EVENT_BIT_FROM_PONG, eventBits);

    testStatus = EventP_waitBits(&gMyEvent, EVENT_BIT_FROM_PING | EVENT_BIT_FROM_PONG, 0, 0, SystemP_WAIT_FOREVER , &eventBits);
    TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, testStatus);
    TEST_ASSERT_EQUAL_INT32(EVENT_BIT_FROM_PING | EVENT_BIT_FROM_PONG, eventBits);

    testStatus = EventP_clearBits(&gMyEvent, EVENT_BIT_FROM_PING | EVENT_BIT_FROM_PONG);
    TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, testStatus);

    testStatus = EventP_getBits(&gMyEvent, &eventBits);
    TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, testStatus);
    TEST_ASSERT_EQUAL_INT32(0x0, eventBits);
}

/*
* Test to verify EventP_setBits API with valid parameters.
*/
void posTest_EventP_setBits(void * args)
{
    EventP_Object obj;
    uint32_t bitsToSet= 1;
    int32_t testStatus = SystemP_SUCCESS;
    if(testStatus == SystemP_SUCCESS)
    {
        if(EventP_setBits(&obj,bitsToSet) != SystemP_SUCCESS)
        {
            testStatus = SystemP_FAILURE;
        }
    }
    if(testStatus != SystemP_SUCCESS)
    {
        testStatus = SystemP_FAILURE;
        DebugP_log("EventP_nortos_pos_Test: failure on line no. %d \n", __LINE__);
    }
    TEST_ASSERT_EQUAL_INT32(testStatus,SystemP_SUCCESS);
}

/*
* Test to verify EventP_clearBits API with valid parameters.
*/
void posTest_EventP_clearBits(void * args)
{
    EventP_Object obj;
    uint32_t bitsToClear = 1;
    int32_t testStatus = SystemP_SUCCESS;
    if(testStatus == SystemP_SUCCESS)
    {
        if(EventP_clearBits(&obj,bitsToClear) != SystemP_SUCCESS)
        {
            testStatus = SystemP_FAILURE;
        }
    }
    if(testStatus != SystemP_SUCCESS)
    {
        testStatus = SystemP_FAILURE;
        DebugP_log("EventP_nortos_pos_Test: failure on line no. %d \n", __LINE__);
    }
    TEST_ASSERT_EQUAL_INT32(testStatus,SystemP_SUCCESS);
}

/*
* Test to verify EventP_getBits API with different valid parameters.
*/
void posTest_EventP_getBits(void * args)
{
    EventP_Object obj;
    uint32_t eventBits;
    int32_t testStatus = SystemP_SUCCESS;
    int32_t testStatus1 = SystemP_SUCCESS;
    int32_t testStatus2 = SystemP_SUCCESS;
    int32_t testStatus3 = SystemP_SUCCESS;
    if(testStatus1 == SystemP_SUCCESS)
    {
        if(EventP_getBits(&obj,&eventBits) != SystemP_SUCCESS)
        {
            testStatus1 = SystemP_FAILURE;
        }
    }
    if(testStatus1 != SystemP_SUCCESS)
    {
        testStatus1 = SystemP_FAILURE;
        DebugP_log("EventP_nortos_pos_Test: failure on line no. %d \n", __LINE__);
    }
    if(testStatus2 == SystemP_SUCCESS)
    {
        if(EventP_getBits(NULL,&eventBits) != SystemP_SUCCESS)
        {
            testStatus2 = SystemP_FAILURE;
        }
    }
    if(testStatus2 != SystemP_SUCCESS)
    {
        testStatus2 = SystemP_FAILURE;
        DebugP_log("EventP_nortos_pos_Test: failure on line no. %d \n", __LINE__);
    }
    if(testStatus3 == SystemP_SUCCESS)
    {
        if(EventP_getBits(&obj,NULL) != SystemP_SUCCESS)
        {
            testStatus3 = SystemP_FAILURE;
        }
    }
    if(testStatus3 != SystemP_SUCCESS)
    {
        testStatus3 = SystemP_FAILURE;
        DebugP_log("EventP_nortos_pos_Test: failure on line no. %d \n", __LINE__);
    }
    if(testStatus1 != SystemP_FAILURE && testStatus2 != SystemP_FAILURE && testStatus3 != SystemP_FAILURE)
    {
        testStatus = SystemP_SUCCESS;
    }
    else
    {
        testStatus = SystemP_FAILURE;
    }
    TEST_ASSERT_EQUAL_INT32(testStatus,SystemP_SUCCESS);
}

void test_pos_main(void * args)
{

    RUN_TEST(posTest_EventP_construct,5994,NULL);
    RUN_TEST(posTest_EventP_destruct,5995,NULL);
    RUN_TEST(posTest_EventP_waitBits,5996,NULL);
    RUN_TEST(posTest_EventP_waitBitsOne,10804,NULL);
    RUN_TEST(posTest_EventP_waitBitsTwo,10805,NULL);
    RUN_TEST(posTest_EventP_setBits,5997,NULL);
    RUN_TEST(posTest_EventP_clearBits,5998,NULL);
    RUN_TEST(posTest_EventP_getBits, 5999 ,NULL);

}
