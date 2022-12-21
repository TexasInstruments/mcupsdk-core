/* Copyright (c) 2023 Texas Instruments Incorporated
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
 *
 */

 /**
 *  \file     edma_test_pos.c
 *
 *  \brief    This file contains edma API unit test code.
 *
 *  \details  edma unit tests
 **/

#include <stdio.h>
#include <string.h>
#include <inttypes.h>
#include <unity.h>
#include <kernel/dpl/SemaphoreP.h>
#include <kernel/dpl/DebugP.h>
#include <kernel/dpl/ClockP.h>
#include <kernel/dpl/TimerP.h>
#include <drivers/edma.h>
#include "ti_drivers_open_close.h"
#include "ti_board_open_close.h"

int32_t  testStatus = SystemP_SUCCESS;
EDMA_InitParams initParam;
EDMACCPaRAMEntry paramEntry;
uint32_t baseAddr;
uint32_t chType = EDMA_CHANNEL_TYPE_DMA;
uint32_t chNum = 0U;

extern EDMA_Config gEdmaConfig[];

/*  Positive test of EDMA_initParamsInit  API*/
void test_edmaInitParamsInitPos(void *args)
{
	int32_t  testStatus = SystemP_SUCCESS;
    if (testStatus == SystemP_SUCCESS)
    {
        EDMA_initParamsInit (&initParam);
    }

    if (testStatus != SystemP_SUCCESS)
    {
		testStatus = SystemP_FAILURE;
        DebugP_log("failure on line no. %d \r\n", __LINE__);

    }
	TEST_ASSERT_EQUAL_INT32(testStatus,SystemP_SUCCESS);
}


/*  Positive test of EDMA_ccPaRAMEntry_init API*/
void test_edmaCcPaRAMEntryInitPos(void *args)
{
	int32_t  testStatus = SystemP_SUCCESS;

    if (testStatus == SystemP_SUCCESS)
    {
        EDMA_ccPaRAMEntry_init(&paramEntry);
    }

    if (testStatus != SystemP_SUCCESS)
    {
		testStatus = SystemP_FAILURE;
        DebugP_log("failure on line no. %d \r\n", __LINE__);

    }
	TEST_ASSERT_EQUAL_INT32(testStatus,SystemP_SUCCESS);
}


/*  Positive test of EDMA_getCCErrStatus API*/
void test_edmaGetCCErrStatusPos(void *args)
{
	int32_t  testStatus = SystemP_SUCCESS;

	if (testStatus == SystemP_SUCCESS)
	{
		baseAddr = EDMA_getBaseAddr(gEdmaHandle[0]);
		EDMA_getCCErrStatus(baseAddr);
	}

	if (testStatus != SystemP_SUCCESS)
	{
		testStatus = SystemP_FAILURE;
		DebugP_log(" failure on line no. %d \r\n", __LINE__);
	}
	TEST_ASSERT_EQUAL_INT32(testStatus,SystemP_SUCCESS);
}


/*  Positive test of EDMA_freeChannelRegion API*/
void test_edmaFreeChannelRegionPos(void *args)
{
	int32_t  testStausOne = SystemP_SUCCESS;
	int32_t  testStatusTwo = SystemP_SUCCESS;
	int32_t  testStatusThree = SystemP_SUCCESS;
	int32_t  testStatus = SystemP_SUCCESS;

	if(testStausOne == SystemP_SUCCESS)
    {
		uint32_t regionId=EDMA_getRegionId(gEdmaHandle[0]);
		baseAddr = EDMA_getBaseAddr(gEdmaHandle[0]);
        uint32_t tccNum = 70U;
		uint32_t param;
        uint32_t paramId = EDMA_allocParam(gEdmaHandle[0], &param);
        uint32_t evtQNum = 0U;
        chType = EDMA_CHANNEL_TYPE_DMA;
        chNum = 70U;
        if(EDMA_freeChannelRegion(baseAddr,regionId,chType,chNum,tccNum,paramId,evtQNum) != FALSE)
        {
            testStausOne = SystemP_FAILURE;
			DebugP_log("failure on line no. %d \r\n", __LINE__);
        }
	}

	if(testStatusTwo == SystemP_SUCCESS)
    {
        uint32_t regionId=EDMA_getRegionId(gEdmaHandle[0]);
		baseAddr = EDMA_getBaseAddr(gEdmaHandle[0]);
        uint32_t tccNum = 70U;
		uint32_t param;
        uint32_t paramId = EDMA_allocParam(gEdmaHandle[0], &param);
        uint32_t evtQNum = 0U;
        chType = 20U;
        chNum = 10U;

        if(EDMA_freeChannelRegion(baseAddr,regionId,chType,chNum,tccNum,paramId,evtQNum) != FALSE)
        {
            testStatusTwo = SystemP_FAILURE;
			DebugP_log("failure on line no. %d \r\n", __LINE__);
        }
	}

	if(testStatusThree == SystemP_SUCCESS)
	{
		uint32_t regionId=EDMA_getRegionId(gEdmaHandle[0]);
		baseAddr = EDMA_getBaseAddr(gEdmaHandle[0]);
        uint32_t tccNum = 30U;
		uint32_t chNum = 30U;
		uint32_t chType = EDMA_CHANNEL_TYPE_DMA;
		uint32_t trigMode = EDMA_TRIG_MODE_MANUAL;
		uint32_t evtQNum = 0U;
		if(EDMA_freeChannelRegion(baseAddr,regionId,chType,chNum,trigMode,tccNum,evtQNum) != TRUE)
		{
			testStatusThree = SystemP_FAILURE;
			DebugP_log("failure on line no. %d \r\n", __LINE__);
		}
	}

	if((testStausOne == SystemP_SUCCESS) && (testStatusTwo == SystemP_SUCCESS) &&
	 (testStatusThree == SystemP_SUCCESS))
	{
		testStatus = SystemP_SUCCESS;
	}
	else
	{
		testStatus = SystemP_FAILURE;
		DebugP_log("failure on line no. %d \r\n", __LINE__);
	}

    TEST_ASSERT_EQUAL_INT32(testStatus,SystemP_SUCCESS);
}

/*  Positive test of EDMA_enableTransferRegion API*/
void test_edmaEnableTransferRegionPos(void *args)
{
	int32_t  testStatusOne = SystemP_SUCCESS;
	int32_t  testStatusTwo = SystemP_SUCCESS;
	int32_t  testStatusThree = SystemP_SUCCESS;
	int32_t  testStatusFour = SystemP_SUCCESS;
	int32_t  testStatusFive = SystemP_SUCCESS;
	int32_t  testStatusSix = SystemP_SUCCESS;
	int32_t  testStatus = SystemP_SUCCESS;

	if (testStatusOne == SystemP_SUCCESS)
	{
		uint32_t regionId=EDMA_getRegionId(gEdmaHandle[0]);
		baseAddr = EDMA_getBaseAddr(gEdmaHandle[0]);
        chNum = 50U;
		if(EDMA_enableTransferRegion(baseAddr,regionId,chNum,EDMA_TRIG_MODE_MANUAL) != TRUE)
		{
			testStatusOne = SystemP_FAILURE;
			DebugP_log("failure on line no. %d \r\n", __LINE__);
		}
	}

	if (testStatusTwo == SystemP_SUCCESS)
	{
		uint32_t regionId=EDMA_getRegionId(gEdmaHandle[0]);
		baseAddr = EDMA_getBaseAddr(gEdmaHandle[0]);
		if(EDMA_enableTransferRegion(baseAddr,regionId,7U,EDMA_TRIG_MODE_QDMA) != TRUE)
		{
			testStatusTwo = SystemP_FAILURE;
			DebugP_log("failure on line no. %d \r\n", __LINE__);
		}
	}

	if (testStatusThree == SystemP_SUCCESS)
	{
		uint32_t regionId=EDMA_getRegionId(gEdmaHandle[0]);
		baseAddr = EDMA_getBaseAddr(gEdmaHandle[0]);
		if(EDMA_enableTransferRegion(baseAddr,regionId,10U,EDMA_TRIG_MODE_QDMA) != FALSE)
		{
			testStatusThree = SystemP_FAILURE;
			DebugP_log("failure on line no. %d \r\n", __LINE__);
		}
	}

	if (testStatusFour == SystemP_SUCCESS)
	{

		uint32_t regionId=EDMA_getRegionId(gEdmaHandle[0]);
		baseAddr = EDMA_getBaseAddr(gEdmaHandle[0]);
		if(EDMA_enableTransferRegion(baseAddr,regionId,7U,EDMA_TRIG_MODE_EVENT) != TRUE)
		{
			testStatusFour = SystemP_FAILURE;
			DebugP_log("failure on line no. %d \r\n", __LINE__);
		}
	}

	if (testStatusFive == SystemP_SUCCESS)
	{

		uint32_t regionId=EDMA_getRegionId(gEdmaHandle[0]);
		baseAddr = EDMA_getBaseAddr(gEdmaHandle[0]);
		if(EDMA_enableTransferRegion(baseAddr,regionId,80U,EDMA_TRIG_MODE_EVENT) != FALSE)
		{
			testStatusFive = SystemP_FAILURE;
			DebugP_log("failure on line no. %d \r\n", __LINE__);
		}
	}

	if (testStatusSix == SystemP_SUCCESS)
	{
		uint32_t regionId=EDMA_getRegionId(gEdmaHandle[0]);
		baseAddr = EDMA_getBaseAddr(gEdmaHandle[0]);
		if(EDMA_enableTransferRegion(baseAddr,regionId,80U,EDMA_TRIG_MODE_MANUAL) != FALSE)
		{
			testStatusSix = SystemP_FAILURE;
			DebugP_log("failure on line no. %d \r\n", __LINE__);
		}
	}

	if ((testStatusOne == SystemP_SUCCESS) && (testStatusTwo == SystemP_SUCCESS) && (testStatusThree == SystemP_SUCCESS)
	&& (testStatusFour == SystemP_SUCCESS) && (testStatusFive == SystemP_SUCCESS) && (testStatusSix == SystemP_SUCCESS))
	{
		testStatus = SystemP_SUCCESS;
	}
	else
	{
		testStatus = SystemP_FAILURE;
		DebugP_log("failure on line no. %d \r\n", __LINE__);
	}

	TEST_ASSERT_EQUAL_INT32(testStatus,SystemP_SUCCESS);
}

/* Positive test case for EdmaRegionID setting isOpen to False State and Handle to Valid value*/
void test_edmaRegionIdThreePos(void *args)
{
	int32_t  testStatus = SystemP_SUCCESS;
	if (testStatus == SystemP_SUCCESS)
	{
		EDMA_Handle handle = (EDMA_Handle)&gEdmaConfig[0];
		gEdmaConfig[0].object->isOpen = FALSE;
		if(EDMA_getRegionId(handle)==0U)
		{
			testStatus = SystemP_FAILURE;
			DebugP_log("EDMA test case: failure on line no. %d \r\n", __LINE__);
		}

	}
	TEST_ASSERT_EQUAL_INT32(testStatus,SystemP_SUCCESS);
}

/* Positive test case for EDMA_disableTransferRegion API */
void test_edmaDisableTransferRegionPos(void *args)
{
	int32_t  testStatusOne = SystemP_SUCCESS;
	int32_t  testStatusTwo = SystemP_SUCCESS;
	int32_t  testStatusThree = SystemP_SUCCESS;
	int32_t  testStatusFour = SystemP_SUCCESS;
	int32_t  testStatusFive = SystemP_SUCCESS;
	int32_t  testStatus = SystemP_SUCCESS;

	if (testStatusOne == SystemP_SUCCESS)
	{
		uint32_t regionId=EDMA_getRegionId(gEdmaHandle[0]);
		baseAddr = EDMA_getBaseAddr(gEdmaHandle[0]);
		if(EDMA_disableTransferRegion(baseAddr,regionId,80U,EDMA_TRIG_MODE_MANUAL) != FALSE)
		{
			testStatusOne = SystemP_FAILURE;
		}
	}

	if (testStatusTwo == SystemP_SUCCESS)
	{
		uint32_t regionId=EDMA_getRegionId(gEdmaHandle[0]);
		baseAddr = EDMA_getBaseAddr(gEdmaHandle[0]);
		if(EDMA_disableTransferRegion(baseAddr,regionId,7U,EDMA_TRIG_MODE_QDMA) != TRUE)
		{
			testStatusTwo = SystemP_FAILURE;
		}
	}

	if (testStatusThree == SystemP_SUCCESS)
	{
		uint32_t regionId=EDMA_getRegionId(gEdmaHandle[0]);
		baseAddr = EDMA_getBaseAddr(gEdmaHandle[0]);
		if(EDMA_disableTransferRegion(baseAddr,regionId,10U,EDMA_TRIG_MODE_QDMA) != FALSE)
		{
			testStatusThree = SystemP_FAILURE;
		}
	}

	if (testStatusFour == SystemP_SUCCESS)
	{
		uint32_t regionId=EDMA_getRegionId(gEdmaHandle[0]);
		baseAddr = EDMA_getBaseAddr(gEdmaHandle[0]);
		if(EDMA_disableTransferRegion(baseAddr,regionId,7U,EDMA_TRIG_MODE_EVENT) != TRUE)
		{
			testStatusFour = SystemP_FAILURE;
		}
	}

	if (testStatusFive == SystemP_SUCCESS)
	{
		uint32_t regionId=EDMA_getRegionId(gEdmaHandle[0]);
		baseAddr = EDMA_getBaseAddr(gEdmaHandle[0]);
		if(EDMA_disableTransferRegion(baseAddr,regionId,80U,EDMA_TRIG_MODE_EVENT) != FALSE)
		{
			testStatusFive = SystemP_FAILURE;
		}
	}
	if ((testStatusThree == SystemP_SUCCESS) && (testStatusThree == SystemP_SUCCESS) && (testStatusThree == SystemP_SUCCESS)
	&& (testStatusFour == SystemP_SUCCESS) && (testStatusFive == SystemP_SUCCESS))
	{
		testStatus = SystemP_SUCCESS;
	}
	else
	{
		testStatus = SystemP_FAILURE;
		DebugP_log("failure on line no. %d \r\n", __LINE__);
	}
	TEST_ASSERT_EQUAL_INT32(testStatus,SystemP_SUCCESS);
}

/*  Positive test case for EDMA_open API*/
void test_edmaOpenPos(void *args)
{
	int32_t  testStatus = SystemP_SUCCESS;

	if (testStatus == SystemP_SUCCESS)
    {
		EDMA_Params prms;
		prms.intrEnable = FALSE;
		if (EDMA_open(4U,&prms) != NULL)
		{
		    testStatus = SystemP_FAILURE;
		}
    }
	else
	{
		testStatus = SystemP_FAILURE;
		DebugP_log("failure on line no. %d \r\n", __LINE__);
	}
	TEST_ASSERT_EQUAL_INT32(testStatus,SystemP_SUCCESS);
}

/* Test case for EDMA_registerIntr API */
void test_edmaRegisterIntrPos(void *args)
{

	int32_t  testStatus= SystemP_SUCCESS;

	if (testStatus == SystemP_SUCCESS)
	{
		EDMA_Handle  gEdmaHandle;
		Edma_IntrObject intrObj;
		EDMA_Params  params;
		gEdmaHandle = EDMA_open(CONFIG_EDMA0, &params);
		if(EDMA_registerIntr(gEdmaHandle, &intrObj ) == 0U)
		{
			testStatus = SystemP_FAILURE;
		}
	}

	if (testStatus != SystemP_SUCCESS)
	{
		testStatus= SystemP_FAILURE;
		DebugP_log("edma_pos_Test: failure on line no. %d \r\n", __LINE__);

	}
	TEST_ASSERT_EQUAL_INT32(testStatus,SystemP_SUCCESS);
}

/* Positive test case for EDMA_unregisterIntr API */
void test_edmaUnRegisterIntrPos(void *args)
{
	int32_t  testStatus= SystemP_SUCCESS;
	if (testStatus == SystemP_SUCCESS)
	{
		EDMA_Handle  gEdmaHandle;
		Edma_IntrObject intrObj;
		EDMA_Params  params;
		gEdmaHandle = EDMA_open(CONFIG_EDMA0, &params);
		if(EDMA_unregisterIntr(gEdmaHandle, &intrObj ) == 0U)
		{
			testStatus = SystemP_FAILURE;
		}
	}

	if (testStatus != SystemP_SUCCESS)
	{
		testStatus= SystemP_FAILURE;
		DebugP_log("edma_pos_Test: failure on line no. %d \r\n", __LINE__);
	}

	TEST_ASSERT_EQUAL_INT32(testStatus,SystemP_SUCCESS);
}

/* Positive test case for EDMA_init API */
void EDMA_initializePos(void *args)
{
	int32_t  testStatus= SystemP_SUCCESS;
	if (testStatus == SystemP_SUCCESS)
	{
 	gEdmaConfig[0].attrs->initPrms.initParamSet = TRUE;
	EDMA_init();
	}
	TEST_ASSERT_EQUAL_INT32(testStatus,SystemP_SUCCESS);
}

/*Test to verify EDMA_getBaseAddr by setting Handle to NULL value */
void test_edmaBaseAddrTwoPos(void *args)
{
	int32_t  testStatus = SystemP_SUCCESS;
	if (testStatus == SystemP_SUCCESS)
	{
		EDMA_Handle handle = NULL;
		gEdmaConfig[0].object->isOpen = FALSE;
		EDMA_getBaseAddr(handle);

	}
	else
	{
		testStatus = SystemP_FAILURE;
		DebugP_log("edma_pos_Test: failure on line no. %d \r\n", __LINE__);
	}
	TEST_ASSERT_EQUAL_INT32(testStatus,SystemP_SUCCESS);
}

/*Test to verify EDMA_getRegionId by setting Handle to NULL value */
void test_edmaRegionIdTwoPos(void *args)
{
	int32_t  testStatus = SystemP_SUCCESS;
	if (testStatus == SystemP_SUCCESS)
	{
		EDMA_Handle gEdmaHandle = NULL;
		gEdmaConfig[0].object->isOpen = FALSE;
		EDMA_getRegionId(gEdmaHandle);
	}

	else
	{
		 testStatus = SystemP_FAILURE;
		DebugP_log("edma_pos_Test: failure on line no. %d \r\n", __LINE__);
	}
	TEST_ASSERT_EQUAL_INT32(testStatus,SystemP_SUCCESS);
}


void edma_posTest(void *args)
{

    uint32_t  baseAddr;

    Drivers_open();
    Board_driversOpen();

    baseAddr = EDMA_getBaseAddr(gEdmaHandle[0]);
    DebugP_assert(baseAddr != 0);

    UNITY_BEGIN();

	RUN_TEST(test_edmaInitParamsInitPos, 5487, NULL);
	RUN_TEST(test_edmaCcPaRAMEntryInitPos, 5488, NULL);
	RUN_TEST(test_edmaGetCCErrStatusPos, 5525, NULL);
	RUN_TEST(test_edmaFreeChannelRegionPos, 5521, NULL);
	RUN_TEST(test_edmaEnableTransferRegionPos, 5522, NULL);
	RUN_TEST(test_edmaDisableTransferRegionPos, 5523, NULL);
	RUN_TEST(test_edmaUnRegisterIntrPos, 5544, NULL);
	RUN_TEST(test_edmaRegisterIntrPos, 5543, NULL);
	RUN_TEST(test_edmaOpenPos, 5539, NULL);
	RUN_TEST(EDMA_initializePos, 6840, NULL);
	RUN_TEST(test_edmaBaseAddrTwoPos, 9001, NULL);
	RUN_TEST(test_edmaRegionIdTwoPos, 8997, NULL);
	RUN_TEST(test_edmaRegionIdThreePos, 5546, NULL);

	UNITY_END();

    Board_driversClose();
    Drivers_close();

	return 0;
}






