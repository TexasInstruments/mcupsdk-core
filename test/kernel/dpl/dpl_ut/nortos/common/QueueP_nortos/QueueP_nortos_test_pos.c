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
 *  \file    QueueP_nortos_test_pos.c
 *
 *  \brief    This file contains QueueP No RTOS positive unit test code.
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


/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */

int32_t  testStatus = SystemP_SUCCESS;
QueueP_Object obj;
QueueP_Handle handle=NULL;
QueueP_Elem lnk;

typedef struct Test_Queue_Elem_s
{
QueueP_Elem lnk;
uint32_t    index;
} Test_Queue_Elem;

Test_Queue_Elem  elem1;
void test_create   (void *args);
void test_delete(void *args);
void test_get(void *args);
void test_put(void *args);
void test_isEmpty(void *args);

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

/*
* Positive Test of QueueP_create API
*/
void test_create(void * args)
{
if (testStatus == SystemP_SUCCESS)
{
    if (QueueP_create(&obj) == NULL)
    {
        testStatus = SystemP_FAILURE;
    }
}

if (testStatus != SystemP_SUCCESS)
{
    DebugP_log("pos_Test: failure on line no %d \n", __LINE__);
    testStatus = SystemP_FAILURE;
}
TEST_ASSERT_EQUAL_INT32(testStatus,SystemP_SUCCESS);
}

/*
* Positive Test of QueueP_delete API
*/
void test_delete(void * args)
{
if (testStatus == SystemP_SUCCESS)
{
    handle= QueueP_create(&obj);
    if ((QueueP_delete(handle) != SystemP_SUCCESS))
    {
        testStatus = SystemP_FAILURE;
    }
}

if (testStatus != SystemP_SUCCESS)
{
    DebugP_log("pos_Test: failure on line no %d \n", __LINE__);
    testStatus = SystemP_FAILURE;
}
TEST_ASSERT_EQUAL_INT32(testStatus,SystemP_SUCCESS);
}

/*
* Positive Test of QueueP_get API
*/
void test_get(void * args)
{
if (testStatus == SystemP_SUCCESS)
{
    handle= QueueP_create(&obj);
    (QueueP_get(handle));
}

if (testStatus != SystemP_SUCCESS)
{
    DebugP_log("pos_Test: failure on line no %d \n", __LINE__);
    testStatus = SystemP_FAILURE;
}
TEST_ASSERT_EQUAL_INT32(testStatus,SystemP_SUCCESS);
}

/*
* Positive Test of QueueP_put API
*/
void test_put(void * args)
{
int32_t testStatus = SystemP_SUCCESS;

if (testStatus == SystemP_SUCCESS)
{
handle= QueueP_create(&obj);
Test_Queue_Elem  elem1;
if (QueueP_put(handle, (QueueP_Elem *)&elem1) != SystemP_SUCCESS)
{
    testStatus = SystemP_FAILURE;
}
}

if (testStatus != SystemP_SUCCESS)
{
DebugP_log("pos_Test: failure on line no %d \n", __LINE__);
testStatus = SystemP_FAILURE;
}

TEST_ASSERT_EQUAL_INT32(testStatus,SystemP_SUCCESS);
}

/*
* Positive Test of QueueP_isEmpty API
*/
void test_isEmpty(void * args)
{
if (testStatus == SystemP_SUCCESS)
{
handle= QueueP_create(&obj);
if (QueueP_isEmpty(handle) != QueueP_EMPTY)
{
    testStatus = SystemP_FAILURE;
}
}

if (testStatus != SystemP_SUCCESS)
{
DebugP_log("pos_Test: failure on line no %d \n", __LINE__);
testStatus = SystemP_FAILURE;
}
TEST_ASSERT_EQUAL_INT32(testStatus,SystemP_SUCCESS);
}


void test_pos_main(void *args)
{
RUN_TEST(test_create, 9341, NULL);
RUN_TEST(test_delete, 9342, NULL);
RUN_TEST(test_get, 9343, NULL);
RUN_TEST(test_put, 9344, NULL);
RUN_TEST(test_isEmpty, 9345, NULL);

}
