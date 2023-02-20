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
 *  \file     edma_test_neg.c
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

extern uint32_t    gEdmaConfigNum;
extern EDMA_Config gEdmaConfig[];
extern EDMA_InitParams gEdmaInitParams[];


/*  Error/Fault test of EDMA_initParamsInit API*/
void test_edmaInitParamsInitNeg(void *args)
{
	int32_t  testStatus = SystemP_SUCCESS;
    if (testStatus == SystemP_SUCCESS)
    {
        EDMA_initParamsInit (NULL);
    }

    if (testStatus != SystemP_SUCCESS)
    {
		testStatus = SystemP_FAILURE;
        DebugP_log(" EDMA test case: failure on line no. %d \r\n", __LINE__);
    }
	TEST_ASSERT_EQUAL_INT32(testStatus,SystemP_SUCCESS);
}

/*  Error/Fault test of EDMA_ccPaRAMEntry_init API*/
void test_edmaCcPaRAMEntry_initNeg(void *args)
{
	int32_t  testStatus = SystemP_SUCCESS;

    if (testStatus == SystemP_SUCCESS)
    {
        EDMA_ccPaRAMEntry_init(NULL);
    }

    if (testStatus != SystemP_SUCCESS)
    {
		testStatus = SystemP_FAILURE;
        DebugP_log("EDMA test case: failure on line no. %d \r\n", __LINE__);
    }
	TEST_ASSERT_EQUAL_INT32(testStatus,SystemP_SUCCESS);
}

/*  Error/Fault test of EDMA_registerIntr API*/
void test_edmaRegisterIntrNeg(void *args)
{

	int32_t  testStatusOne = SystemP_SUCCESS;
	int32_t  testStatusTwo = SystemP_SUCCESS;
	int32_t  testStatus = SystemP_SUCCESS;
	if (testStatusOne == SystemP_SUCCESS)
	{
		Edma_IntrObject *intrObj = NULL;
		EDMA_Handle handle = NULL;
		if(EDMA_registerIntr(handle,intrObj) == SystemP_SUCCESS)
		{
			testStatusOne = SystemP_FAILURE;
		}
	}

	if (testStatusTwo == SystemP_SUCCESS)
	{
		Edma_IntrObject *intrObj = NULL;
		EDMA_Handle handle = (EDMA_Handle)&gEdmaConfig[1];
		if(EDMA_registerIntr(handle,intrObj) == SystemP_SUCCESS)
		{
			testStatusTwo = SystemP_FAILURE;
		}
	}
	if((testStatusOne == SystemP_SUCCESS) && (testStatusTwo == SystemP_SUCCESS))
	{
		testStatus = SystemP_SUCCESS;
	}
	else
	{
		testStatus = SystemP_FAILURE;
		DebugP_log("EDMA test case : failure on line no. %d \r\n", __LINE__);
	}
	TEST_ASSERT_EQUAL_INT32(testStatus,SystemP_SUCCESS);
}

/*  Negative test of EDMA_unregisterIntr API*/
void test_edmaUnRegisterIntrNeg(void *args)
{
	int32_t  testStatusOne = SystemP_SUCCESS;
	int32_t  testStatusTwo = SystemP_SUCCESS;
	int32_t  testStatus = SystemP_SUCCESS;

	if (testStatusOne == SystemP_SUCCESS)
	{
		Edma_IntrObject *intrObj = NULL;
		EDMA_Handle handle = (EDMA_Handle)&gEdmaConfig[1];
		if(EDMA_unregisterIntr(handle,intrObj) == SystemP_SUCCESS)
		{
			testStatus = SystemP_FAILURE;
		}
	}
	if (testStatus == SystemP_SUCCESS)
	{
		EDMA_Handle handle=NULL;
		Edma_IntrObject 	*intrObj=NULL;
		EDMA_Object         *object=NULL;
		object->firstIntr = NULL;
		EDMA_Config        *config=NULL;
		config->object = NULL;
		config->object->isOpen = FALSE;
		config->object->openPrms.intrEnable = FALSE;
		if(EDMA_unregisterIntr(handle,intrObj) == SystemP_SUCCESS)
		{
			testStatus = SystemP_FAILURE;
		}
	}
    if((testStatusOne == SystemP_SUCCESS)&&(testStatusTwo == SystemP_SUCCESS))
	{
		testStatus = SystemP_SUCCESS;
	}
	else
	{
		testStatus = SystemP_FAILURE;
		DebugP_log("Edma test case :failure on line no. %d \r\n", __LINE__);
	}
	TEST_ASSERT_EQUAL_INT32(testStatus,SystemP_SUCCESS);
}

/*  Negative test case  for EDMA_getRegionId API*/
void test_edmaGetRegionIdNeg(void *args)
{
	int32_t  testStatusOne = SystemP_SUCCESS;
	int32_t  testStatusFour = SystemP_SUCCESS;
	int32_t  testStatusFive = SystemP_SUCCESS;
	int32_t  testStatus = SystemP_SUCCESS;

	if (testStatusOne == SystemP_SUCCESS)
	{
		EDMA_Handle handle = NULL;
		if(EDMA_getRegionId (&handle ) != SOC_EDMA_NUM_REGIONS)
		{
			testStatusOne = SystemP_FAILURE;
			DebugP_log("Edma Test case: failure on line no. %d \r\n", __LINE__);
		}
	}

	if(testStatusFour == SystemP_SUCCESS)
	{
		EDMA_Handle gEdmaHandle;
		EDMA_Config  config;
		config.object = NULL;
		config.object->isOpen = TRUE;

		if(EDMA_getRegionId(&gEdmaHandle) != SOC_EDMA_NUM_REGIONS)
		{
			testStatusFour = SystemP_FAILURE;
			DebugP_log("Edma Test case: failure on line no. %d \r\n", __LINE__);
		}
	}

	if(testStatusFive == SystemP_SUCCESS)
	{
		EDMA_Handle gEdmaHandle = NULL;
		EDMA_Config  *config = NULL;
		config->object = NULL;
		config->object->isOpen = FALSE;

		if(EDMA_getRegionId(gEdmaHandle) != SOC_EDMA_NUM_REGIONS)
		{
			testStatusFive = SystemP_FAILURE;
			DebugP_log("Edma Test case: failure on line no. %d \r\n", __LINE__);
		}
	}

	if((testStatusOne == SystemP_SUCCESS) && (testStatusFour == SystemP_SUCCESS) && (testStatusFive == SystemP_SUCCESS))
	{
		testStatus = SystemP_SUCCESS;
	}
	else
	{
		testStatus = SystemP_FAILURE;
		DebugP_log("Edma Test case: failure on line no. %d \r\n", __LINE__);
	}
	TEST_ASSERT_EQUAL_INT32(testStatus,SystemP_SUCCESS);
}

/*Test to verify EDMA_getBaseAddr and EDMA_getRegionId API*/
void test_edmaBaseAddrRegionIdNeg(void *args)
{

	int32_t testStatus = SystemP_SUCCESS;
	if (testStatus == SystemP_SUCCESS)
	{
		EDMA_Handle gEdmaHandle = NULL;
		gEdmaConfig[0].object->isOpen = TRUE;
		EDMA_getBaseAddr(&gEdmaHandle);
		EDMA_getRegionId(&gEdmaHandle);

	}
	else
	{
		 testStatus = SystemP_FAILURE;
		DebugP_log("edma_pos_Test: failure on line no. %d \r\n", __LINE__);
	}
	TEST_ASSERT_EQUAL_INT32(testStatus,SystemP_SUCCESS);
}

/*Test to verify EDMA_getBaseAddr and EDMA_getRegionId API*/
void test_edmaBaseAddrRegionIdTwoNeg(void *args)
{
	int32_t  testStatusOne = SystemP_SUCCESS;
	int32_t  testStatusTwo = SystemP_SUCCESS;
	int32_t  testStatus = SystemP_SUCCESS;
	if (testStatusOne == SystemP_SUCCESS)
	{
		EDMA_Handle handle = NULL;
		gEdmaConfig[0].object->isOpen = FALSE;
		EDMA_getBaseAddr(&handle);
		EDMA_getRegionId(&handle);
	}

	if (testStatusTwo == SystemP_SUCCESS)
	{
		EDMA_Handle handle;
		gEdmaConfig[0].object->isOpen = FALSE;
		EDMA_getBaseAddr(&handle);
		EDMA_getRegionId(&handle);
	}

	if((testStatusTwo == SystemP_SUCCESS) && (testStatusOne == SystemP_SUCCESS))
	{
		testStatus = SystemP_SUCCESS;
	}
	else
	{
		 testStatus = SystemP_FAILURE;
		DebugP_log("edma_pos_Test: failure on line no. %d \r\n", __LINE__);
	}
	TEST_ASSERT_EQUAL_INT32(testStatus,SystemP_SUCCESS);
}

/*Test to verify EDMA_getRegionId API by setting handle = NULL */
void test_edmaBaseAddrRegionIdThreeNeg(void *args)
{
	int32_t  testStatus = SystemP_SUCCESS;
	if (testStatus == SystemP_SUCCESS)
	{
		EDMA_Handle handle = NULL;
		if((EDMA_getBaseAddr(&handle)!=0U) && (EDMA_getRegionId(&handle)!=0U))
		{
			testStatus = SystemP_FAILURE;
			DebugP_log("edma_pos_Test: failure on line no. %d \r\n", __LINE__);
		}

	}
	TEST_ASSERT_EQUAL_INT32(testStatus,SystemP_SUCCESS);
}

/*Test case for Edma Alloc DMA channel Region*/
void test_edmaAllocDmaChannelNeg(void *args)
{

	int32_t  testStatusTwo = SystemP_SUCCESS;
	int32_t  testStatusFour = SystemP_SUCCESS;
	int32_t  testStatusFive = SystemP_SUCCESS;
	int32_t  testStatus = SystemP_SUCCESS;

	if (testStatusTwo == SystemP_SUCCESS)
	{
		uint32_t *dmaCh = NULL;
		EDMA_Handle gEdmaHandle = NULL;
		gEdmaConfig[0].object->isOpen = FALSE;

		if(EDMA_allocDmaChannel(gEdmaHandle,dmaCh) == SystemP_SUCCESS)
		{
			testStatusTwo = SystemP_FAILURE;
			DebugP_log("EDMA test case: failure on line no. %d \r\n", __LINE__);
		}
	}

	if (testStatusFour == SystemP_SUCCESS)
	{
		uint32_t *dmaCh = NULL;
		EDMA_Handle gEdmaHandle = NULL;
		gEdmaConfig[0].object->isOpen =TRUE;
		if(EDMA_allocDmaChannel(gEdmaHandle,dmaCh) != SystemP_FAILURE)
		{
			testStatusFour = SystemP_FAILURE;
			DebugP_log("EDMA test case: failure on line no. %d \r\n", __LINE__);
		}
	}

	if (testStatusFive == SystemP_SUCCESS)
	{
		uint32_t dmaCh = 34U;
		EDMA_Handle gEdmaHandle = NULL;
		gEdmaConfig[0].object->isOpen = FALSE;
		if(EDMA_allocDmaChannel(gEdmaHandle,&dmaCh) != SystemP_FAILURE)
		{
			testStatusFive = SystemP_FAILURE;
			DebugP_log("EDMA test case: failure on line no. %d \r\n", __LINE__);
		}
	}

	if((testStatusTwo == SystemP_SUCCESS)&& (testStatusFour == SystemP_SUCCESS)&& (testStatusFive == SystemP_SUCCESS))
	{
		testStatus = SystemP_SUCCESS;
	}
	else
	{
		testStatus = SystemP_FAILURE;
		DebugP_log("EDMA test case: failure on line no. %d \r\n", __LINE__);
	}

	TEST_ASSERT_EQUAL_INT32(testStatus,SystemP_SUCCESS);
}

/* Negative test Case for EDMA_open API */
void test_edmaOpenNeg(void *args)
{
	int32_t  testStatusOne = SystemP_SUCCESS;
	int32_t  testStatusTwo= SystemP_SUCCESS;
	int32_t  testStatusThree= SystemP_SUCCESS;
	int32_t testStatus = SystemP_SUCCESS;

	if (testStatusOne == SystemP_SUCCESS)
	{
		EDMA_Object  *object  = NULL;
		EDMA_Params prms;
    	prms.intrEnable = FALSE;
		uint32_t index = 0U;
		object->isOpen = FALSE;
		if(EDMA_open(index,&prms) == NULL)
		{
			testStatus = SystemP_FAILURE;
			DebugP_log("EDMA test case: failure on line no. %d \r\n", __LINE__);
		}
	}

	if (testStatusTwo == SystemP_SUCCESS)
	{
		EDMA_Params   *prms = (EDMA_Params *)args;
		prms->intrEnable = FALSE;
		uint32_t index = 0U;
		if(EDMA_open(index,prms) != NULL)
		{
			testStatus = SystemP_FAILURE;
			DebugP_log("EDMA test case: failure on line no. %d \r\n", __LINE__);
		}
	}

	if (testStatusThree == SystemP_SUCCESS)
	{

		EDMA_Params   *prms = (EDMA_Params *)args;
		uint32_t index = 0U;
		gEdmaConfig[0].object->openPrms.intrEnable = FALSE;
		if(EDMA_open(index,prms) != NULL)
		{
			testStatus = SystemP_FAILURE;
			DebugP_log("EDMA test case: failure on line no. %d \r\n", __LINE__);
		}
	}
	if ((testStatusOne == SystemP_SUCCESS) && (testStatusTwo == SystemP_SUCCESS)&&(testStatusThree == SystemP_SUCCESS))
	{
		testStatus = SystemP_SUCCESS;
	}
	else
	{
		testStatus = SystemP_FAILURE;
		DebugP_log("EDMA test case: failure on line no. %d \r\n", __LINE__);
	}
	TEST_ASSERT_EQUAL_INT32(testStatus,SystemP_SUCCESS);
}

/* test case for EDMA_getHandle API */
void test_edmaGetHandleNeg(void *args)
{
	int32_t  testStatusOne = SystemP_SUCCESS;
	if (testStatusOne == SystemP_SUCCESS)
	{
		uint32_t index = 0U;
		if(EDMA_getHandle(index) == NULL)
		{
			testStatusOne = SystemP_FAILURE;
		}

		TEST_ASSERT_EQUAL_INT32(testStatusOne,SystemP_SUCCESS);
	}
}


/* test case for EDMA_getHandle API  by assigning index to 1U*/
void test_edmaGetHandleNegOne(void *args)
{
	int32_t  testStatusTwo = SystemP_SUCCESS;
	if (testStatusTwo == SystemP_SUCCESS)
	{
    	uint32_t index = 1U;
		if(EDMA_getHandle(index) != NULL)
		{
			testStatusTwo = SystemP_FAILURE;
		}
	}
	TEST_ASSERT_EQUAL_INT32(testStatusTwo,SystemP_SUCCESS);
}

/* Test case for EDMA_getHandle API  by assigning isOpen to False state */
void test_edmaGetHandleNegTwo(void *args)
{
	int32_t  testStatusThree = SystemP_SUCCESS;
	if (testStatusThree == SystemP_SUCCESS)
	{
    	uint32_t index = 0U;
		gEdmaConfig[0].object->isOpen = FALSE;
		if(EDMA_getHandle(index) != NULL)
		{
			testStatusThree = SystemP_FAILURE;
		}
	}
	TEST_ASSERT_EQUAL_INT32(testStatusThree,SystemP_SUCCESS);
}


/* Test case for EDMA_close API */
void  test_edmaCloseNeg(void *args)
{
	int32_t testStatusTwo = SystemP_SUCCESS;
	int32_t testStatusThree = SystemP_SUCCESS;
	int32_t  testStatus = SystemP_SUCCESS;

	if(testStatusTwo == SystemP_SUCCESS)
	{
		gEdmaConfig[0].object->hwiHandle = NULL;
		EDMA_Handle gEdmaHandle;
	 	EDMA_close(&gEdmaHandle);
	}
	if(testStatusThree == SystemP_SUCCESS)
	{
		EDMA_Handle gEdmaHandle;
	 	EDMA_close(&gEdmaHandle);
	}

 	if((testStatusTwo == SystemP_SUCCESS) && (testStatusThree == SystemP_SUCCESS))
	{
		testStatus = SystemP_SUCCESS;
	}
	else
	{
		testStatus = SystemP_FAILURE;
	    DebugP_log("EDMA test case: failure on line no. %d \r\n", __LINE__);
	}

	TEST_ASSERT_EQUAL_INT32(testStatus,SystemP_SUCCESS);
}

/* Test case for EDMA_getBaseAddr by setting handle to NULL */
void test_edmaBaseAddrNeg(void *args)
{
	int32_t  testStatus = SystemP_SUCCESS;
	if (testStatus == SystemP_SUCCESS)
	{
		if(EDMA_getBaseAddr(NULL)!=0U)
		{
			testStatus = SystemP_FAILURE;
			DebugP_log("edma_pos_Test: failure on line no. %d \r\n", __LINE__);
		}

	}
	TEST_ASSERT_EQUAL_INT32(testStatus,SystemP_SUCCESS);
}

/* Test case for EDMA_getRegionId by setting handle to NULL */
void test_edmaRegionIdNeg(void *args)
{
	int32_t  testStatus = SystemP_SUCCESS;
	if (testStatus == SystemP_SUCCESS)
	{
		if(EDMA_getRegionId(NULL) != SOC_EDMA_NUM_REGIONS)
		{
			testStatus = SystemP_FAILURE;

		}

	}
	TEST_ASSERT_EQUAL_INT32(testStatus,SystemP_SUCCESS);
}

/* Test case for EDMA_getRegionId by setting gEdmaConfig[0].object->isOpen to FALSE  */
void test_edmaRegionIdOneNeg(void *args)
{
	int32_t  testStatus = SystemP_SUCCESS;
	if (testStatus == SystemP_SUCCESS)
	{
		EDMA_Handle handle = (EDMA_Handle)&gEdmaConfig[0];
		gEdmaConfig[0].object->isOpen = FALSE;
		if(EDMA_getBaseAddr(handle)!=0U)
		{
			testStatus = SystemP_FAILURE;
			DebugP_log("EDMA test case: failure on line no. %d \r\n", __LINE__);
		}

	}
	TEST_ASSERT_EQUAL_INT32(testStatus,SystemP_SUCCESS);
}

/* Test case for EDMA_getBaseAddr by setting gEdmaConfig[0].object->isOpen to FALSE  */
void test_edmaBaseAddrOneNeg(void *args)
{
	int32_t  testStatus = SystemP_SUCCESS;
	if (testStatus == SystemP_SUCCESS)
	{
		EDMA_Handle handle = (EDMA_Handle)&gEdmaConfig[0];
		gEdmaConfig[0].object->isOpen = FALSE;
		if(EDMA_getBaseAddr(handle)!=0U)
		{
			testStatus = SystemP_FAILURE;
			DebugP_log("EDMA test case: failure on line no. %d \r\n", __LINE__);
		}

	}
	TEST_ASSERT_EQUAL_INT32(testStatus,SystemP_SUCCESS);
}

/*Test to verify EDMA_edmaFreeDmaChannel API*/
void test_edmaFreeDmaChannelNeg(void *args)
{

	int32_t  testStatusOne = SystemP_SUCCESS;
	int32_t  testStatusTwo = SystemP_SUCCESS;
	int32_t  testStatus = SystemP_SUCCESS;

	if (testStatusOne == SystemP_SUCCESS)
	{
		EDMA_Handle gEdmaHandle = NULL;
		EDMA_Config *config = NULL;
   		config->object = NULL;
		gEdmaConfig[0].object->isOpen = FALSE;
		if(EDMA_freeDmaChannel(&gEdmaHandle[0], NULL) == SystemP_SUCCESS)
		{
			testStatusOne = SystemP_FAILURE;
			DebugP_log("EDMA test case: failure on line no. %d \r\n", __LINE__);
		}
	}

	if (testStatusTwo == SystemP_SUCCESS)
	{
		uint32_t dmaCh = 36U;
		EDMA_Handle gEdmaHandle = NULL;
		EDMA_Config *config = &gEdmaConfig[CONFIG_EDMA_NUM_INSTANCES];
		EDMA_Object *objectParams = (EDMA_Object *)config->object;
		config->object = objectParams;
		objectParams->isOpen = FALSE;

		if(EDMA_freeDmaChannel(&gEdmaHandle[0], &dmaCh) == SystemP_SUCCESS)
		{
			testStatusTwo = SystemP_FAILURE;
			DebugP_log("EDMA test case: failure on line no. %d \r\n", __LINE__);
		}
	}

	if ((testStatusOne == SystemP_SUCCESS) && (testStatusTwo == SystemP_SUCCESS))
	{
		testStatus = SystemP_SUCCESS;
	}
	else
	{
		testStatus = SystemP_FAILURE;
	}

	TEST_ASSERT_EQUAL_INT32(testStatus,SystemP_SUCCESS);
}

void edma_negTest(void *args)
{
    uint32_t  baseAddr;

    Drivers_open();
    Board_driversOpen();

    baseAddr = EDMA_getBaseAddr(gEdmaHandle[0]);
    DebugP_assert(baseAddr != 0);

    UNITY_BEGIN();

	RUN_TEST(test_edmaInitParamsInitNeg, 8726, NULL);
	RUN_TEST(test_edmaCcPaRAMEntry_initNeg, 5597, NULL);
	RUN_TEST(test_edmaRegisterIntrNeg, 5713, NULL);
	RUN_TEST(test_edmaUnRegisterIntrNeg, 5714, NULL);
	RUN_TEST(test_edmaBaseAddrRegionIdNeg, 5715, NULL);
	RUN_TEST(test_edmaBaseAddrRegionIdTwoNeg, 9002, NULL);
	RUN_TEST(test_edmaAllocDmaChannelNeg, 5717, NULL);
	RUN_TEST(test_edmaOpenNeg, 5709, NULL);
	RUN_TEST(test_edmaGetHandleNeg, 5540, NULL);
	RUN_TEST(test_edmaFreeDmaChannelNeg, 5721, NULL);
	RUN_TEST(test_edmaGetHandleNegOne,5710, NULL);
	RUN_TEST(test_edmaGetHandleNegTwo,9058, NULL);
	RUN_TEST(test_edmaBaseAddrNeg, 9059, NULL);
	RUN_TEST(test_edmaRegionIdNeg, 5716, NULL);
	RUN_TEST(test_edmaBaseAddrOneNeg, 9060, NULL);
	RUN_TEST(test_edmaCloseNeg, 5711, NULL);

	UNITY_END();

    Board_driversClose();
    Drivers_close();

}
