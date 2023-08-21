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
 *  \file     DebugP_shmLogWriter_test_pos.c
 *
 *  \brief    This file contains DebugP_shmLogWriter API unit test code.
 *
 *  \details  DebugP_shmLogWriter unit tests
 **/

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <drivers/soc.h>
#include <kernel/dpl/SystemP.h>
#include <kernel/dpl/DebugP.h>
#include <kernel/dpl/ClockP.h>
#include <stdio.h>
#include <string.h>
#include <inttypes.h>
#include <kernel/dpl/HwiP.h>
#include <kernel/dpl/CacheP.h>
#include <kernel/dpl/SemaphoreP.h>
#include <kernel/dpl/TaskP.h>
#include <kernel/dpl/HeapP.h>
#include <kernel/dpl/CycleCounterP.h>
#include <kernel/dpl/EventP.h>
#include <kernel/dpl/QueueP.h>
#include <drivers/soc.h>
#include <unity.h>
#include "ti_drivers_open_close.h"
#include <kernel/dpl/SystemP.h>
#include <kernel/dpl/DebugP.h>
#include <kernel/dpl/ClockP.h>
#include <kernel/nortos/dpl/common/printf.h>
#include <drivers/hw_include/cslr_soc.h>
void DebugP_shmLogReaderTaskMain(void *args);
uint32_t DebugP_shmLogReaderGetString(DebugP_ShmLog *shmLog,char *buf, uint32_t buf_size);

#define DebugP_SHM_LOG_SIZE_SEVEN  ((2*1024U) - 20U)
#define DebugP_SHM_LOG_SIZE_TWO  ((2*1024U) + 2U)
#define DebugP_SHM_LOG_SIZE_EIGHT  ((2*1024U) - 17U)
#define DEBUG_SHM_LOG_READER_LINE_BUF_SIZE  (130u)

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */

	int32_t         testStatus=SystemP_SUCCESS;
    int32_t         testStatus1 = SystemP_SUCCESS;
    int32_t         testStatus2 = SystemP_SUCCESS;
    int32_t         testStatus3 = SystemP_SUCCESS;
    int32_t         testStatus4 = SystemP_SUCCESS;
    DebugP_ShmLog shmLog;
    typedef struct {

    uint8_t isCoreShmLogInialized[CSL_CORE_ID_MAX];
    DebugP_ShmLog *shmLog;
    uint8_t numCores;
    char lineBuf[DEBUG_SHM_LOG_READER_LINE_BUF_SIZE+UNSIGNED_INTEGERVAL_THREE]; /* +3 to add \r\n and null char at end of string in worst case */

} DebugP_ShmLogReaderCtrl;
    DebugP_ShmLogReaderCtrl gDebugShmLogReaderCtrl;
    uint16_t selfCoreId=2;
    char *buf=(char*)0xFF;
    uint16_t num_bytes=1;
    uint16_t numCores = 0;
    uint32_t buf_size = 0;
	void* args= NULL;
    char character= '\0';

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */
#define DebugP_SHM_LOG_SIZE_ONE  ((2*1024U) - 15U)

/*
 * Test to verify DebugP_shmLogReaderInit API with valid parameters.
 */
void posTest_DebugP_shmLogReaderInit(void * args)
{
    if(testStatus==SystemP_SUCCESS)
	{
        DebugP_shmLogReaderInit(&shmLog,numCores);
    }
	if (testStatus != SystemP_SUCCESS)
    {
        testStatus = SystemP_FAILURE;
        DebugP_log("DebugP_shmLogReader_nortos_posTest: failure on line no. %d \n", __LINE__);
    }
    TEST_ASSERT_EQUAL_INT32(testStatus,SystemP_SUCCESS);
}

/*
 * Test to verify DebugP_shmLogRead API with valid parameters.
 */
void posTest_DebugP_shmLogRead(void * args)
{
    if(testStatus1==SystemP_SUCCESS)
	{
        DebugP_shmLogRead();
    }
	if (testStatus1 != SystemP_SUCCESS)
    {
        testStatus1 = SystemP_FAILURE;
        DebugP_log("DebugP_shmLogWriter_nortos_posTest: failure on line no. %d \n", __LINE__);
    }

    if(testStatus2 == SystemP_SUCCESS)
    {
        gDebugShmLogReaderCtrl.numCores =10;
        shmLog.isValid = DebugP_SHM_LOG_IS_VALID;
        DebugP_shmLogRead();
    }

    if (testStatus2 != SystemP_SUCCESS)
    {
        testStatus2 = SystemP_FAILURE;
        DebugP_log("DebugP_shmLogReader_nortos_posTest: failure on line no. %d \n", __LINE__);
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
 * Test to verify DebugP_shmLogReaderGetString API with valid parameters.
 */
void posTest_DebugP_shmLogReaderGetString(void * args)
{
    if(testStatus1==SystemP_SUCCESS)
	{
        DebugP_shmLogReaderGetString(&shmLog, buf, num_bytes);
    }
	if (testStatus1 != SystemP_SUCCESS)
    {
        testStatus1 = SystemP_FAILURE;
        DebugP_log("DebugP_shmLogWriter_nortos_posTest: failure on line no. %d \n", __LINE__);
    }

    if(testStatus2 == SystemP_SUCCESS)
    {
        shmLog.wrIndex = DebugP_SHM_LOG_SIZE_SEVEN;
        shmLog.rdIndex = DebugP_SHM_LOG_SIZE_EIGHT;
    }

    if (testStatus2 != SystemP_SUCCESS)
    {
        testStatus2 = SystemP_FAILURE;
        DebugP_log("DebugP_shmLogReader_nortos_posTest: failure on line no. %d \n", __LINE__);
    }

    if(testStatus3 == SystemP_SUCCESS)
    {
        char abc[140] = "Hydrophilic signal molecules, such as growth factors, adrenaline, and insulin,it cannot pass the cell membrane because as the cell.\n";
        int32_t len = strlen(abc);
        shmLog.wrIndex = DebugP_SHM_LOG_SIZE_SEVEN;
        shmLog.rdIndex = DebugP_SHM_LOG_SIZE_EIGHT;
        int32_t i = 0;
        for(i=0; i<len; i++)
        {
            DebugP_shmLogReaderGetString(&shmLog, buf, num_bytes);
        }
    }

    if (testStatus2 != SystemP_SUCCESS)
    {
        testStatus2 = SystemP_FAILURE;
        DebugP_log("DebugP_shmLogReader_nortos_posTest: failure on line no. %d \n", __LINE__);
    }

    if(testStatus4 == SystemP_SUCCESS)
    {
        shmLog.wrIndex = DebugP_SHM_LOG_SIZE;
        shmLog.rdIndex = DebugP_SHM_LOG_SIZE;
        DebugP_shmLogReaderGetString(&shmLog, buf, num_bytes);
    }

    if (testStatus4 != SystemP_SUCCESS)
    {
        testStatus4 = SystemP_FAILURE;
        DebugP_log("DebugP_shmLogReader_nortos_posTest: failure on line no. %d \n", __LINE__);
    }

    if ((testStatus1 != SystemP_FAILURE) && (testStatus2 != SystemP_FAILURE) && (testStatus3 != SystemP_FAILURE) && (testStatus4 != SystemP_FAILURE))
    {
        testStatus = SystemP_SUCCESS;
    }
    else
    {
        testStatus = SystemP_FAILURE;
    }
	TEST_ASSERT_EQUAL_INT32(testStatus,SystemP_SUCCESS);
}

void test_pos_main(void *args)
{
    RUN_TEST(posTest_DebugP_shmLogReaderGetString, 11546, NULL);
    RUN_TEST(posTest_DebugP_shmLogReaderInit, 11547, NULL);
    RUN_TEST(posTest_DebugP_shmLogRead, 11548,NULL);
    RUN_TEST(posTest_DebugP_shmLogReaderGetString, 11549, NULL);
}
