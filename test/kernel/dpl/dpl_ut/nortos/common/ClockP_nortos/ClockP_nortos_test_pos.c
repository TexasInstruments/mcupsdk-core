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
#include <kernel/nortos/dpl/common/ClockP_nortos_priv.h>

 int32_t  testStatus = SystemP_SUCCESS;
 int32_t  testStatus1 = SystemP_SUCCESS;
 int32_t  testStatus2 = SystemP_SUCCESS;
 int32_t  testStatus3 = SystemP_SUCCESS;
	void *args = NULL;
	ClockP_Object handle,handle1;
	ClockP_Params params;
	uint64_t usecs = 10000U;
	uint32_t ticks = 1U;
	uint32_t timeout = 0;
	uint32_t sec = 0;
	uint32_t usec = 0;
    static ClockP_Object gMyClock,gMyClock1,gMyClock2;
    static SemaphoreP_Object gMyDoneSem,gMyDoneSem1,gMyDoneSem2;
    #define TEST_INT_NUM     (20U)


/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

static void myClockCallback(ClockP_Object *obj, void *args)
{
    SemaphoreP_Object *pSemObj = (SemaphoreP_Object *)args;

    SemaphoreP_post(pSemObj);
}

static void myClockCallback1(ClockP_Object *obj, void *args)
{
    SemaphoreP_Object *pSemObj = (SemaphoreP_Object *)args;

    SemaphoreP_post(pSemObj);
}

static void myClockCallback2(ClockP_Object *obj, void *args)
{
    SemaphoreP_Object *pSemObj = (SemaphoreP_Object *)args;

    SemaphoreP_post(pSemObj);
}
static void test_clockWithHwiIsr(void *args)
{
    ClockP_start((ClockP_Object *)args);
}

static void test_clockWithHwiIsr1(void *args)
{
    ClockP_start((ClockP_Object *)args);
}

static void test_clockWithHwiIsr2(void *args)
{
    ClockP_start((ClockP_Object *)args);
}

/*
 * Test to verify ClockP_timerTickIsr API with valid parameters.
 */
 void test_timerTickIsrOne(void * args)
 {

    if (testStatus == SystemP_SUCCESS)
    {

        ClockP_timerTickIsr(args);
	}
	if (testStatus != SystemP_SUCCESS)
    {
		testStatus = SystemP_FAILURE;
        DebugP_log("pos_Test: failure on line no %d \n", __LINE__);
    }
    TEST_ASSERT_EQUAL_INT32(testStatus,SystemP_SUCCESS);
 }

/*
 * Test to verify ClockP_timerTickIsr API with parameter as NULL.
 */
 void test_timerTickIsrTwo(void * args)
 {

    if (testStatus == SystemP_SUCCESS)
    {
        int *ptr=NULL;
		args=ptr;

        ClockP_timerTickIsr(args);

    }

	if (testStatus != SystemP_SUCCESS)
    {
		testStatus = SystemP_FAILURE;
        DebugP_log("pos_Test: failure on line no %d \n", __LINE__);

    }
    TEST_ASSERT_EQUAL_INT32(testStatus,SystemP_SUCCESS);
 }

/*
 * Test to verify ClockP_construct API with valid parameters.
 */
void test_construct(void * args)
{
    if (testStatus == SystemP_SUCCESS)
    {
        params.start=1;
        ClockP_construct(&handle, &params);
    }

	if (testStatus != SystemP_SUCCESS)
    {
		testStatus = SystemP_FAILURE;
        DebugP_log("pos_Test: failure on line no %d \n", __LINE__);

    }
    TEST_ASSERT_EQUAL_INT32(testStatus,SystemP_SUCCESS);
}

/*
 * Test to verify ClockP_destruct API with parameter as NULL.
 */
void test_destruct(void *args)
{
    if (testStatus == SystemP_SUCCESS)
    {

        ClockP_destruct(&handle);
    }

	if (testStatus != SystemP_SUCCESS)
    {
		testStatus = SystemP_FAILURE;
        DebugP_log("pos_Test: failure on line no %d \n", __LINE__);

    }
    TEST_ASSERT_EQUAL_INT32(testStatus,SystemP_SUCCESS);
}

/*
 * Test to verify by creating two or more Clock Objects and then destruct them in descending order to cover statement/branch coverage.
 */
void test_destructOne(void * args)
{
    ClockP_Params clockParams,clockParams1;

    ClockP_Params_init(&clockParams);
    ClockP_construct(&gMyClock, &clockParams);
    ClockP_start(&gMyClock);
    ClockP_stop(&gMyClock);

    ClockP_Params_init(&clockParams1);
    ClockP_construct(&gMyClock1, &clockParams1);
    ClockP_start(&gMyClock1);
    ClockP_stop(&gMyClock1);

    ClockP_destruct(&gMyClock1);
    ClockP_destruct(&gMyClock);

     TEST_ASSERT_EQUAL_INT32(testStatus,SystemP_SUCCESS);

}

/*
 * Test to verify ClockP_usecToTicks API with valid parameter.
 */
void test_usecToTicks(void * args)
{
    if (testStatus == SystemP_SUCCESS)
    {
        if(ClockP_usecToTicks(usecs)==0U)
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
 * Test to verify ClockP_ticksToUsec API with valid parameters.
 */
void test_ticksToUsec(void * args)
{
    if (testStatus == SystemP_SUCCESS)
    {
        if(ClockP_ticksToUsec(ticks) == 0U)
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
 * Test to verify ClockP_getTicks API with valid parameters.
 */
void test_getTicks(void * args)
{
    if (testStatus == SystemP_SUCCESS)
    {
        if(ClockP_getTicks() == 0U)
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
 * Test to verify ClockP_getTimeout API with valid parameters.
 */
void test_getTimeout(void *args)
{
    if (testStatus1 == SystemP_SUCCESS)
    {
        ClockP_getTimeout(&handle);
    }
	if (testStatus1 != SystemP_SUCCESS)
    {
        DebugP_log("pos_Test: failure on line no %d \n", __LINE__);
    }

    if (testStatus2 == SystemP_SUCCESS)
    {
        ClockP_setTimeout(&handle, 6);
        ClockP_getTimeout(&handle);
    }
	if (testStatus2 != SystemP_SUCCESS)
    {
        DebugP_log("pos_Test: failure on line no %d \n", __LINE__);

    }

    if ((testStatus1 != SystemP_FAILURE) && (testStatus2 != SystemP_FAILURE))
    {
        testStatus = SystemP_SUCCESS;
    }
    else
    {
        testStatus = SystemP_FAILURE;
    }
    TEST_ASSERT_EQUAL_INT32(testStatus,SystemP_SUCCESS);
}

/*
 * Test to verify ClockP_getTimeout API for branch condition coverage.
 */
void test_getTimeoutOne(void *args)
{
    if (testStatus1 == SystemP_SUCCESS)
    {
        ClockP_setTimeout(&handle, (uint32_t)SystemP_TIMEOUT);
        ClockP_getTimeout(&handle);
    }
	if (testStatus1 != SystemP_SUCCESS)
    {
        DebugP_log("pos_Test: failure on line no %d \n", __LINE__);
    }
}

/*
 * Test to verify ClockP_isActive API with valid parameters.
 */
 void test_isActive(void *args)
 {
    if (testStatus == SystemP_SUCCESS)
    {
        if (ClockP_isActive(&handle) == FALSE)
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
 * Test to verify ClockP_Params_init API with valid parameters.
 */
void test_Params_init(void *args)
{
    if (testStatus == SystemP_SUCCESS)
    {
        ClockP_Params_init(&params);
    }
	if (testStatus != SystemP_SUCCESS)
    {
        DebugP_log("pos_Test: failure on line no %d \n", __LINE__);
        testStatus = SystemP_FAILURE;
    }
    TEST_ASSERT_EQUAL_INT32(testStatus,SystemP_SUCCESS);
}

/*
 * Test to verify ClockP_setTimeout API with valid parameters.
 */
 void test_setTimeout(void *args)
{
    if (testStatus == SystemP_SUCCESS)
    {
       ClockP_setTimeout(&handle, timeout);
    }
	if (testStatus != SystemP_SUCCESS)
    {
        DebugP_log("pos_Test: failure on line no %d \n", __LINE__);
        testStatus = SystemP_FAILURE;
    }
    TEST_ASSERT_EQUAL_INT32(testStatus,SystemP_SUCCESS);
}

/*
 * Test to verify ClockP_start API with valid parameters.
 */
void test_start(void *args)
{
    if (testStatus == SystemP_SUCCESS)
    {
        ClockP_setTimeout(&handle, 10);
        ClockP_start(&handle);
    }
	if (testStatus != SystemP_SUCCESS)
    {
        DebugP_log("pos_Test: failure on line no %d \n", __LINE__);
        testStatus = SystemP_FAILURE;
    }
    TEST_ASSERT_EQUAL_INT32(testStatus,SystemP_SUCCESS);
}

/*
* Test to verify ClockP_start API by calling multiple times to acheive MCDC Coverage
*/
void test_startOne(void *args)
{
    if (testStatus == SystemP_SUCCESS)
    {
        ClockP_start(&handle);
        ClockP_start(&handle);
    }
	if (testStatus != SystemP_SUCCESS)
    {
        DebugP_log("pos_Test: failure on line no %d \n", __LINE__);
        testStatus = SystemP_FAILURE;
    }
    TEST_ASSERT_EQUAL_INT32(testStatus,SystemP_SUCCESS);
}

/*
 * Test to verify ClockP_stop API with valid parameters.
 */
void test_stop(void *args)
{
    if (testStatus == SystemP_SUCCESS)
    {
        ClockP_stop(&handle);
    }
	if (testStatus != SystemP_SUCCESS)
    {
        DebugP_log("pos_Test: failure on line no %d \n", __LINE__);
       testStatus = SystemP_FAILURE;
    }
    TEST_ASSERT_EQUAL_INT32(testStatus,SystemP_SUCCESS);
}

/*
* Initializing 3 ClockParams and giving different timeouts, args, ClockCallback values and construct with diff gMyClock and ClockParams.
*/
void test_stopOne(void * args)
{
    ClockP_Params clockParams,clockParams1,clockParams2;
    int32_t status;
    HwiP_Params hwiParams,hwiParams1,hwiParams2;
    HwiP_Object hwiObj,hwiObj1,hwiObj2;
    ClockP_Object gMyClock4;

    status = SemaphoreP_constructBinary(&gMyDoneSem, 0);
    TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, status);

    ClockP_Params_init(&clockParams);
    clockParams.timeout = 10;
    clockParams.period = clockParams.timeout;
    clockParams.callback = myClockCallback;
    clockParams.args = &gMyDoneSem;
    status = ClockP_construct(&gMyClock, &clockParams);
    TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, status);
    status = ClockP_isActive(&gMyClock);
    TEST_ASSERT_EQUAL_INT32(status, SystemP_SUCCESS);

    HwiP_Params_init(&hwiParams);
    hwiParams.intNum = TEST_INT_NUM;
    hwiParams.callback = test_clockWithHwiIsr;
    hwiParams.args = &gMyClock;
    status = HwiP_construct(&hwiObj, &hwiParams);
    TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, status);

    HwiP_post(hwiParams.intNum);

    status = SemaphoreP_constructBinary(&gMyDoneSem1, 0);
    TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, status);

    ClockP_Params_init(&clockParams1);
    clockParams1.timeout = 2;
    clockParams1.period = clockParams1.timeout;
    clockParams1.callback = myClockCallback1;
    clockParams1.args = &gMyDoneSem1;
    status = ClockP_construct(&gMyClock1, &clockParams1);
    TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, status);
    status = ClockP_isActive(&gMyClock1);
    TEST_ASSERT_EQUAL_INT32(status, SystemP_SUCCESS);

    HwiP_Params_init(&hwiParams1);
    hwiParams1.intNum = 15U;
    hwiParams1.callback = test_clockWithHwiIsr1;
    hwiParams1.args = &gMyClock1;
    status = HwiP_construct(&hwiObj1, &hwiParams1);
    TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, status);

    HwiP_post(hwiParams1.intNum);

    status = SemaphoreP_constructBinary(&gMyDoneSem2, 0);
    TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, status);

    ClockP_Params_init(&clockParams2);
    clockParams2.timeout = 10;
    clockParams2.period = clockParams2.timeout;
    clockParams2.callback = myClockCallback2;
    clockParams2.args = &gMyDoneSem2;
    status = ClockP_construct(&gMyClock2, &clockParams2);
    TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, status);
    status = ClockP_isActive(&gMyClock2);
    TEST_ASSERT_EQUAL_INT32(status, SystemP_SUCCESS);

    HwiP_Params_init(&hwiParams2);
    hwiParams2.intNum = TEST_INT_NUM;
    hwiParams2.callback = test_clockWithHwiIsr2;
    hwiParams2.args = &gMyClock2;
    status = HwiP_construct(&hwiObj2, &hwiParams2);
    TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, status);

    HwiP_post(hwiParams2.intNum);

    status = SemaphoreP_pend(&gMyDoneSem2, clockParams2.timeout * 2);
    TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, status);

    ClockP_stop(&gMyClock4);
    ClockP_stop(&gMyClock2);

    /* check if no more clock callbacks */
    status = SemaphoreP_pend(&gMyDoneSem2, clockParams2.timeout * 2);
    TEST_ASSERT_EQUAL_INT32(SystemP_TIMEOUT, status);

    HwiP_destruct(&hwiObj2);
    ClockP_destruct(&gMyClock2);
    SemaphoreP_destruct(&gMyDoneSem2);

    status = SemaphoreP_pend(&gMyDoneSem1, clockParams1.timeout * 2);
    TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, status);

    ClockP_stop(&gMyClock1);

    /* check if no more clock callbacks */
    status = SemaphoreP_pend(&gMyDoneSem1, clockParams1.timeout * 2);
    TEST_ASSERT_EQUAL_INT32(SystemP_TIMEOUT, status);

    HwiP_destruct(&hwiObj1);
    ClockP_destruct(&gMyClock1);
    SemaphoreP_destruct(&gMyDoneSem1);

    status = SemaphoreP_pend(&gMyDoneSem, clockParams.timeout * 2);
    TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, status);

    ClockP_stop(&gMyClock);

    /* check if no more clock callbacks */
    status = SemaphoreP_pend(&gMyDoneSem, clockParams.timeout * 2);
    TEST_ASSERT_EQUAL_INT32(SystemP_TIMEOUT, status);

    HwiP_destruct(&hwiObj);
    ClockP_destruct(&gMyClock);
    SemaphoreP_destruct(&gMyDoneSem);
}

/*
 * Initializing 2 ClockParams and giving different timeouts, args, ClockCallback values and construct with diff gMyClock and ClockParams.
 */
void test_stopTwo(void * args)
{
    ClockP_Params clockParams,clockParams1;
    int32_t status;
    HwiP_Params hwiParams,hwiParams1;
    HwiP_Object hwiObj,hwiObj1;

    status = SemaphoreP_constructBinary(&gMyDoneSem, 0);
    TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, status);

    ClockP_Params_init(&clockParams);
    clockParams.timeout = 10;
    clockParams.period = clockParams.timeout;
    clockParams.callback = myClockCallback;
    clockParams.args = &gMyDoneSem;
    status = ClockP_construct(&gMyClock, &clockParams);
    TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, status);
    status = ClockP_isActive(&gMyClock);
    TEST_ASSERT_EQUAL_INT32(status, SystemP_SUCCESS);

    HwiP_Params_init(&hwiParams);
    hwiParams.intNum = TEST_INT_NUM;
    hwiParams.callback = test_clockWithHwiIsr;
    hwiParams.args = &gMyClock;
    status = HwiP_construct(&hwiObj, &hwiParams);
    TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, status);

    HwiP_post(hwiParams.intNum);

    status = SemaphoreP_constructBinary(&gMyDoneSem1, 0);
    TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, status);

    ClockP_Params_init(&clockParams1);
    clockParams1.timeout = 10;
    clockParams1.period = clockParams1.timeout;
    clockParams1.callback = myClockCallback1;
    clockParams1.args = &gMyDoneSem1;
    status = ClockP_construct(&gMyClock1, &clockParams1);
    TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, status);
    status = ClockP_isActive(&gMyClock1);
    TEST_ASSERT_EQUAL_INT32(status, SystemP_SUCCESS);

    HwiP_Params_init(&hwiParams1);
    hwiParams1.intNum = TEST_INT_NUM;
    hwiParams1.callback = test_clockWithHwiIsr1;
    hwiParams1.args = &gMyClock1;
    status = HwiP_construct(&hwiObj1, &hwiParams1);
    TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, status);

    HwiP_post(hwiParams1.intNum);

    status = SemaphoreP_pend(&gMyDoneSem1, clockParams1.timeout * 2);
    TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, status);

    ClockP_stop(&gMyClock1);

    /* check if no more clock callbacks */
    status = SemaphoreP_pend(&gMyDoneSem1, clockParams1.timeout * 2);
    TEST_ASSERT_EQUAL_INT32(SystemP_TIMEOUT, status);

    HwiP_destruct(&hwiObj1);
    ClockP_destruct(&gMyClock1);
    SemaphoreP_destruct(&gMyDoneSem1);

    status = SemaphoreP_pend(&gMyDoneSem, clockParams.timeout * 2);
    TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, status);

    ClockP_stop(&gMyClock);

    /* check if no more clock callbacks */
    status = SemaphoreP_pend(&gMyDoneSem, clockParams.timeout * 2);
    TEST_ASSERT_EQUAL_INT32(SystemP_TIMEOUT, status);

    HwiP_destruct(&hwiObj);
    ClockP_destruct(&gMyClock);
    SemaphoreP_destruct(&gMyDoneSem);

}

/*
 * Test to verify ClockP_sleep API with valid parameters.
 */
void test_sleep(void *args)
{
    if (testStatus == SystemP_SUCCESS)
    {
       ClockP_sleep(sec);
    }
	if (testStatus != SystemP_SUCCESS)
    {
        DebugP_log("pos_Test: failure on line no %d \n", __LINE__);
        testStatus = SystemP_FAILURE;
    }
    TEST_ASSERT_EQUAL_INT32(testStatus,SystemP_SUCCESS);
}

/*
 * Test to verify ClockP_usleep API with valid parameters.
 */
 void test_usleep(void *args)
{
    if (testStatus == SystemP_SUCCESS)
    {
        ClockP_usleep(usec);
    }
	if (testStatus != SystemP_SUCCESS)
    {
        DebugP_log("pos_Test: failure on line no %d \n", __LINE__);
       testStatus = SystemP_FAILURE;
    }
    TEST_ASSERT_EQUAL_INT32(testStatus,SystemP_SUCCESS);
}

/*
 * Test to verify ClockP_usleep API with valid parameters for MC/DC coverage.
 */
void test_usleepTwo(void *args)
    {
    if (testStatus == SystemP_SUCCESS)
    {
        uint32_t usec = 100*1000U;
        ClockP_usleep(usec);
    }
	if (testStatus != SystemP_SUCCESS)
    {
        DebugP_log("pos_Test: failure on line no %d \n", __LINE__);
        testStatus = SystemP_FAILURE;
    }
    TEST_ASSERT_EQUAL_INT32(testStatus,SystemP_SUCCESS);
}

/*
 * Test to verify ClockP_getTimeUsec API with valid parameters.
 */
 void test_getTimeUsec(void *args)
    {
    if (testStatus == SystemP_SUCCESS)
    {
        if (ClockP_getTimeUsec() == 0)
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
    Drivers_open();
    UNITY_BEGIN();

    RUN_TEST(test_destructOne,10728,NULL);
    RUN_TEST(test_timerTickIsrOne, 5727,NULL);
    RUN_TEST(test_timerTickIsrTwo, 5728,NULL);
    RUN_TEST(test_construct,5657,NULL);
    RUN_TEST(test_destruct,5731,NULL);
    RUN_TEST(test_usecToTicks,5733,NULL);
    RUN_TEST(test_ticksToUsec,5735,NULL);
    RUN_TEST(test_stopTwo,10733,NULL);
    RUN_TEST(test_getTicks,5737,NULL);
    RUN_TEST(test_getTimeout,5739,NULL);
    RUN_TEST(test_isActive,5743,NULL);
    RUN_TEST(test_Params_init,5748,NULL);
    RUN_TEST(test_setTimeout,5754,NULL);
    RUN_TEST(test_start,5765,NULL);
    RUN_TEST(test_startOne,10734,NULL);
    RUN_TEST(test_stop,5771,NULL);
    RUN_TEST(test_stopOne,10732,NULL);
    RUN_TEST(test_sleep,5773,NULL);
    RUN_TEST(test_usleep,5779,NULL);
    RUN_TEST(test_usleepTwo,5777,NULL);
    RUN_TEST(test_getTimeUsec,5780,NULL);
    RUN_TEST(test_getTimeoutOne,11737,NULL);

    UNITY_END();
    Drivers_close();
}
