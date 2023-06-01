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
	Edma_IntrObject *intrObj = NULL;
	EDMA_Handle handle = NULL;

	if (testStatusOne == SystemP_SUCCESS)
	{
		if(EDMA_registerIntr(handle,intrObj) == SystemP_SUCCESS)
		{
			testStatusOne = SystemP_FAILURE;
		}
	}

	if (testStatusTwo == SystemP_SUCCESS)
	{
		handle = (EDMA_Handle)&gEdmaConfig[1];
	
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
	Edma_IntrObject *intrObj = NULL;
	EDMA_Handle  handle = (EDMA_Handle)&gEdmaConfig[1];
	EDMA_Object  *object = NULL;
	EDMA_Config  *config=NULL;

	if (testStatusOne == SystemP_SUCCESS)
	{
		if(EDMA_unregisterIntr(handle,intrObj) == SystemP_SUCCESS)
		{
			testStatus = SystemP_FAILURE;
		}
	}

	if (testStatus == SystemP_SUCCESS)
	{
		handle = NULL;
		object->firstIntr = NULL;
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
	EDMA_Handle gEdmaHandle;
	EDMA_Config  *config = NULL;

	if (testStatusOne == SystemP_SUCCESS)
	{
		if(EDMA_getRegionId (NULL) != SOC_EDMA_NUM_REGIONS)
		{
			testStatusOne = SystemP_FAILURE;
			DebugP_log("Edma Test case: failure on line no. %d \r\n", __LINE__);
		}
	}

	if(testStatusFour == SystemP_SUCCESS)
	{
		config->object = NULL;
		config->object->isOpen = TRUE;

		if(EDMA_getRegionId(&gEdmaHandle) != SOC_EDMA_NUM_REGIONS)
		{
			testStatusFour = SystemP_FAILURE;
			DebugP_log("Edma Test case: failure on line no. %d \r\n", __LINE__);
		}
	}

	if(testStatusFive == SystemP_SUCCESS)
	{
		
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
	EDMA_Handle gEdmaHandle = NULL;

	if (testStatus == SystemP_SUCCESS)
	{
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
	EDMA_Handle handle = NULL;

	if (testStatusOne == SystemP_SUCCESS)
	{
		gEdmaConfig[0].object->isOpen = FALSE;
		EDMA_getBaseAddr(&handle);
		EDMA_getRegionId(&handle);
	}

	if (testStatusTwo == SystemP_SUCCESS)
	{
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
	EDMA_Handle handle = NULL;

	if (testStatus == SystemP_SUCCESS)
	{
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
	uint32_t *dmaCh = NULL;
	EDMA_Handle gEdmaHandle = NULL;

	if (testStatusTwo == SystemP_SUCCESS)
	{
		gEdmaConfig[0].object->isOpen = FALSE;

		if(EDMA_allocDmaChannel(gEdmaHandle,dmaCh) == SystemP_SUCCESS)
		{
			testStatusTwo = SystemP_FAILURE;
			DebugP_log("EDMA test case: failure on line no. %d \r\n", __LINE__);
		}
	}

	if (testStatusFour == SystemP_SUCCESS)
	{
		gEdmaConfig[0].object->isOpen =TRUE;
		if(EDMA_allocDmaChannel(gEdmaHandle,dmaCh) != SystemP_FAILURE)
		{
			testStatusFour = SystemP_FAILURE;
			DebugP_log("EDMA test case: failure on line no. %d \r\n", __LINE__);
		}
	}

	if (testStatusFive == SystemP_SUCCESS)
	{
		dmaCh = (uint32_t *)34U;
		gEdmaConfig[0].object->isOpen = FALSE;

		if(EDMA_allocDmaChannel(gEdmaHandle,dmaCh) != SystemP_FAILURE)
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
	EDMA_Params   *prms = (EDMA_Params *)args;
	EDMA_Object  *object  = NULL;
	uint32_t index = 0U;

	if (testStatusOne == SystemP_SUCCESS)
	{
    	prms->intrEnable = FALSE;
		object->isOpen = FALSE;
		if(EDMA_open(index,prms) != NULL)
		{
			testStatus = SystemP_FAILURE;
			DebugP_log("EDMA test case: failure on line no. %d \r\n", __LINE__);
		}
	}

	if (testStatusTwo == SystemP_SUCCESS)
	{
		prms->intrEnable = FALSE;
		if(EDMA_open(index,prms) != NULL)
		{
			testStatus = SystemP_FAILURE;
			DebugP_log("EDMA test case: failure on line no. %d \r\n", __LINE__);
		}
	}

	if (testStatusThree == SystemP_SUCCESS)
	{
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
	uint32_t index = 0U;

	if (testStatusOne == SystemP_SUCCESS)
	{
		if(EDMA_getHandle(index) == NULL)
		{
			testStatusOne = SystemP_FAILURE;
		}
		TEST_ASSERT_EQUAL_INT32(testStatusOne,SystemP_FAILURE);
	}
}

/* test case for EDMA_getHandle API  by assigning index to 1U*/
void test_edmaGetHandleNegOne(void *args)
{
	int32_t  testStatusTwo = SystemP_SUCCESS;
	uint32_t index = 1U;

	if (testStatusTwo == SystemP_SUCCESS)
	{
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
	uint32_t index = 0U;

	if (testStatusThree == SystemP_SUCCESS)
	{	
		gEdmaConfig[0].object->isOpen = FALSE;
		if(EDMA_getHandle(index) != NULL)
		{
			testStatusThree = SystemP_FAILURE;
		}
	}
	TEST_ASSERT_EQUAL_INT32(testStatusThree,SystemP_SUCCESS);
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
	EDMA_Handle handle = (EDMA_Handle)&gEdmaConfig[0];

	if (testStatus == SystemP_SUCCESS)
	{
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
	EDMA_Handle handle = (EDMA_Handle)&gEdmaConfig[0];

	if (testStatus == SystemP_SUCCESS)
	{
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
	uint32_t dmaCh = 36U;
	EDMA_Handle gEdmaHandle = NULL;
	EDMA_Config *config = &gEdmaConfig[CONFIG_EDMA_NUM_INSTANCES];
	EDMA_Object *objectParams = (EDMA_Object *)config->object;

	if (testStatusOne == SystemP_SUCCESS)
	{
		gEdmaConfig[0].object->isOpen = FALSE;
		if(EDMA_freeDmaChannel(&gEdmaHandle[0], NULL) == SystemP_SUCCESS)
		{
			testStatusOne = SystemP_FAILURE;
			DebugP_log("EDMA test case: failure on line no. %d \r\n", __LINE__);
		}
	}

	if (testStatusTwo == SystemP_SUCCESS)
	{
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

/*Test to verify EDMA_allocDmaChannel API*/
void test_edmaAllocDmaChannelNegOne(void *args)
{
	int32_t  testStatus = SystemP_SUCCESS;
	uint32_t *dmaCh = (uint32_t *)32;

	if (testStatus == SystemP_SUCCESS)
	{
		gEdmaConfig[0].object->isOpen = FALSE;
		if(EDMA_allocDmaChannel(&gEdmaHandle[0],dmaCh) != SystemP_SUCCESS)
		{
			testStatus = SystemP_FAILURE;
			DebugP_log("EDMA test case: failure on line no. %d \r\n", __LINE__);
		}
	}
	TEST_ASSERT_EQUAL_INT32(testStatus,SystemP_SUCCESS);
}

/*Test to verify EDMA_allocDmaChannel API by assigning gEdmaConfig[0].object->isOpen = FALSE state*/
void test_edmaAllocQdmaChannelNegOne(void *args)
{
	int32_t  testStatus = SystemP_SUCCESS;
	uint32_t *qdmaCh = (uint32_t *)32;

	if (testStatus == SystemP_SUCCESS)
	{
		gEdmaConfig[0].object->isOpen = FALSE;
		if(EDMA_allocQdmaChannel(&gEdmaHandle[0],qdmaCh) != SystemP_SUCCESS)
		{
			testStatus = SystemP_FAILURE;
			DebugP_log("EDMA test case: failure on line no. %d \r\n", __LINE__);
		}
	}
	TEST_ASSERT_EQUAL_INT32(testStatus,SystemP_SUCCESS);
}

/*Test to verify EDMA_allocQdmaChannel by allocating qdmaCh to NULL state */
void test_edmaAllocQdmaChannelNegTwo(void *args)
{
	int32_t  testStatus = SystemP_SUCCESS;

	if (testStatus == SystemP_SUCCESS)
	{	
		if(EDMA_allocQdmaChannel(&gEdmaHandle[0],NULL) == SystemP_SUCCESS)
		{
			testStatus = SystemP_FAILURE;
			DebugP_log("EDMA test case: failure on line no. %d \r\n", __LINE__);
		}
	}
	TEST_ASSERT_EQUAL_INT32(testStatus,SystemP_SUCCESS);
}

/*Test to verify EDMA_allocTcc by gEdmaConfig[0].object->isOpen = FALSE state */
void test_edmaAllocTccChannelNegOne(void *args)
{

	int32_t  testStatus = SystemP_SUCCESS;
	uint32_t *tcc = (uint32_t *)32;

	if (testStatus == SystemP_SUCCESS)
	{
		gEdmaConfig[0].object->isOpen = FALSE;
		if(EDMA_allocTcc(&gEdmaHandle[0],tcc) != SystemP_SUCCESS)
		{
			testStatus = SystemP_FAILURE;
			DebugP_log("EDMA test case: failure on line no. %d \r\n", __LINE__);
		}
	}
	TEST_ASSERT_EQUAL_INT32(testStatus,SystemP_SUCCESS);
}

/* Test to verify EDMA_allocTcc by tcc to NULL */
void test_edmaAllocTccChannelNegTwo(void *args)
{
	int32_t  testStatus = SystemP_SUCCESS;

	if (testStatus == SystemP_SUCCESS)
	{
		if(EDMA_allocTcc(&gEdmaHandle[0],NULL) == SystemP_SUCCESS)
		{
			testStatus = SystemP_FAILURE;
			DebugP_log("EDMA test case: failure on line no. %d \r\n", __LINE__);
		}
	}
	TEST_ASSERT_EQUAL_INT32(testStatus,SystemP_SUCCESS);
}

/* Test to verify EDMA_allocParam by gEdmaConfig[0].object->isOpen = FALSE state */
void test_edmaAllocParamNegOne(void *args)
{
	int32_t  testStatus = SystemP_SUCCESS;
	uint32_t *param = (uint32_t *)32;

	if (testStatus == SystemP_SUCCESS)
	{
		gEdmaConfig[0].object->isOpen = FALSE;
		if(EDMA_allocParam(&gEdmaHandle[0],param) != SystemP_SUCCESS)
		{
			testStatus = SystemP_FAILURE;
			DebugP_log("EDMA test case: failure on line no. %d \r\n", __LINE__);
		}
	}

	TEST_ASSERT_EQUAL_INT32(testStatus,SystemP_SUCCESS);
}

/* Test to verify EDMA_allocParam by param as NULL state */
void test_edmaAllocParamNegTwo(void *args)
{
	int32_t  testStatus = SystemP_SUCCESS;
	
	if (testStatus == SystemP_SUCCESS)
	{
		gEdmaConfig[0].object->isOpen = FALSE;
		if(EDMA_allocParam(&gEdmaHandle[0],NULL) == SystemP_SUCCESS)
		{
			testStatus = SystemP_FAILURE;
			DebugP_log("EDMA test case: failure on line no. %d \r\n", __LINE__);
		}
	}
	TEST_ASSERT_EQUAL_INT32(testStatus,SystemP_SUCCESS);
}

/* Test to verify EDMA_allocParam by NULL to parameters */
void test_edmaAllocParamNegThree(void *args)
{
	int32_t  testStatus = SystemP_SUCCESS;

	if (testStatus == SystemP_SUCCESS)
	{
		if(EDMA_allocParam(NULL,NULL) == SystemP_SUCCESS)
		{
			testStatus = SystemP_FAILURE;
			DebugP_log("EDMA test case: failure on line no. %d \r\n", __LINE__);
		}
	}
	TEST_ASSERT_EQUAL_INT32(testStatus,SystemP_SUCCESS);
}

/* Test to verify EDMA_freeDmaChannel bygEdmaConfig[0].object->isOpen = FALSE */
void test_edmaFreeDmaChannelNegOne(void *args)
{
	int32_t  testStatus = SystemP_SUCCESS;
	uint32_t dmach = 30U;

	if (testStatus == SystemP_SUCCESS)
	{
		gEdmaConfig[0].object->isOpen = FALSE;
		if(EDMA_freeDmaChannel(&gEdmaHandle[0], &dmach) != SystemP_SUCCESS)
		{
			testStatus = SystemP_FAILURE;
			DebugP_log("EDMA test case: failure on line no. %d \r\n", __LINE__);
		}
	}
	TEST_ASSERT_EQUAL_INT32(testStatus,SystemP_SUCCESS);
}

/* Test to verify EDMA_freeQdmaChannel bygEdmaConfig[0].object->isOpen = FALSE */
void test_edmaFreeQdmaChannelNegOne(void *args)
{
	int32_t  testStatus = SystemP_SUCCESS;
	uint32_t qdmach = 7U;

	if (testStatus == SystemP_SUCCESS)
	{
		gEdmaConfig[0].object->isOpen = FALSE;
		if(EDMA_freeQdmaChannel(&gEdmaHandle[0], &qdmach) == SystemP_FAILURE)
		{
			testStatus = SystemP_FAILURE;
			DebugP_log("EDMA test case: failure on line no. %d \r\n", __LINE__);
		}
	}
	TEST_ASSERT_EQUAL_INT32(testStatus,SystemP_SUCCESS);
}

/* Test to verify EDMA_freeQdmaChannel by qdmaCh to NULL state */
void test_edmaFreeQdmaChannelNegTwo(void *args)
{
	int32_t  testStatus = SystemP_SUCCESS;

	if (testStatus == SystemP_SUCCESS)
	{
		if(EDMA_freeQdmaChannel(&gEdmaHandle[0], NULL) == SystemP_SUCCESS)
		{
			testStatus = SystemP_FAILURE;
			DebugP_log("EDMA test case: failure on line no. %d \r\n", __LINE__);
		}
	}
	TEST_ASSERT_EQUAL_INT32(testStatus,SystemP_SUCCESS);
}

/* Test to verify EDMA_freeTcc by tcc to NULL state */
void test_edmaFreeTccNeg(void *args)
{
	int32_t  testStatus = SystemP_SUCCESS;

	if (testStatus == SystemP_SUCCESS)
	{
		if(EDMA_freeTcc(&gEdmaHandle[0], NULL) == SystemP_SUCCESS)
		{
			testStatus = SystemP_FAILURE;
			DebugP_log("EDMA test case: failure on line no. %d \r\n", __LINE__);
		}
	}
	TEST_ASSERT_EQUAL_INT32(testStatus,SystemP_SUCCESS);
}

/* Test to verify EDMA_freeParam API*/
void test_edmaFreeParamNeg(void *args)
{
	int32_t  testStatus = SystemP_SUCCESS;
	uint32_t param = 260U;

	if (testStatus == SystemP_SUCCESS)
	{	
		if(EDMA_freeParam(&gEdmaHandle[0], &param) != SystemP_FAILURE)
		{
			testStatus = SystemP_FAILURE;
			DebugP_log("EDMA test case: failure on line no. %d \r\n", __LINE__);
		}
	}
	TEST_ASSERT_EQUAL_INT32(testStatus,SystemP_SUCCESS);
}

/* Test to verify EDMA_freeParam API with 260 value*/
void test_edmaFreeParamNegOne(void *args)
{
	int32_t  testStatus = SystemP_SUCCESS;
	uint32_t *param = (uint32_t *)200U;

	if (testStatus == SystemP_SUCCESS)
	{
		gEdmaConfig[0].object->isOpen = FALSE;
		if(EDMA_freeParam(&gEdmaHandle[0], param) != SystemP_SUCCESS)
		{
			testStatus = SystemP_FAILURE;
			DebugP_log("EDMA test case: failure on line no. %d \r\n", __LINE__);
		}
	}
	TEST_ASSERT_EQUAL_INT32(testStatus,SystemP_SUCCESS);
}

/* Test to verify EDMA_freeParam API with NULL value as the parameters */
void test_edmaFreeParamNegTwo(void *args)
{
	int32_t  testStatus = SystemP_SUCCESS;

	if (testStatus == SystemP_SUCCESS)
	{
		if(EDMA_freeParam(&gEdmaHandle[0], NULL) == SystemP_SUCCESS)
		{
			testStatus = SystemP_FAILURE;
			DebugP_log("EDMA test case: failure on line no. %d \r\n", __LINE__);
		}
	}
	TEST_ASSERT_EQUAL_INT32(testStatus,SystemP_SUCCESS);
}

/* Test to verify EDMA_registerIntr API withgEdmaConfig[0].object = NULL */
void test_edmaRegisterIntrNegThree(void *args)
{
	int32_t  testStatus = SystemP_SUCCESS;
	Edma_IntrObject intrObj;

	if (testStatus == SystemP_SUCCESS)
	{
		gEdmaConfig[0].object = NULL;
		if(EDMA_registerIntr(gEdmaHandle[0], &intrObj) == SystemP_SUCCESS)
		{
			testStatus = SystemP_FAILURE;
			DebugP_log("EDMA test case: failure on line no. %d \r\n", __LINE__);
		}
	}
	TEST_ASSERT_EQUAL_INT32(testStatus,SystemP_SUCCESS);
}

/* Test to verify EDMA_unregisterIntr API withgEdmaConfig[0].object = NULL */
void test_edmaUnRegisterIntrNegThree(void *args)
{
	int32_t  testStatus = SystemP_SUCCESS;
	Edma_IntrObject intrObj;

	if (testStatus == SystemP_SUCCESS)
	{
		gEdmaConfig[0].object = NULL;
		if(EDMA_unregisterIntr(gEdmaHandle[0], &intrObj) != SystemP_SUCCESS)
		{
			testStatus = SystemP_FAILURE;
			DebugP_log("EDMA test case: failure on line no. %d \r\n", __LINE__);
		}
	}
	TEST_ASSERT_EQUAL_INT32(testStatus,SystemP_SUCCESS);
}


void edma_negTest(void *args)
{
    Drivers_open();
    Board_driversOpen();

    UNITY_BEGIN();

	RUN_TEST(test_edmaInitParamsInitNeg, 8726, NULL);
	RUN_TEST(test_edmaCcPaRAMEntry_initNeg, 5597, NULL);
	RUN_TEST(test_edmaRegisterIntrNeg, 5713, NULL);
	RUN_TEST(test_edmaUnRegisterIntrNeg, 5714, NULL);
	RUN_TEST(test_edmaBaseAddrRegionIdNeg, 5715, NULL);
	RUN_TEST(test_edmaBaseAddrRegionIdTwoNeg, 9002, NULL);
	RUN_TEST(test_edmaAllocDmaChannelNeg, 5717, NULL);
	RUN_TEST(test_edmaAllocDmaChannelNegOne, 10014, NULL);
	RUN_TEST(test_edmaAllocQdmaChannelNegOne, 10015, NULL);
	RUN_TEST(test_edmaAllocQdmaChannelNegTwo, 10016, NULL);
	RUN_TEST(test_edmaAllocTccChannelNegOne, 10017, NULL);
	RUN_TEST(test_edmaAllocTccChannelNegTwo, 10018, NULL);
	RUN_TEST(test_edmaAllocParamNegOne, 10019, NULL);
	RUN_TEST(test_edmaAllocParamNegTwo, 10020, NULL);
	RUN_TEST(test_edmaAllocParamNegThree, 10021, NULL);
	RUN_TEST(test_edmaOpenNeg, 5709, NULL);
	RUN_TEST(test_edmaGetHandleNeg, 5540, NULL);
	RUN_TEST(test_edmaFreeDmaChannelNeg, 5721, NULL);
	RUN_TEST(test_edmaFreeQdmaChannelNegTwo, 10022, NULL);
	RUN_TEST(test_edmaFreeParamNeg, 10023, NULL);
	RUN_TEST(test_edmaFreeParamNegOne, 10024, NULL);
	RUN_TEST(test_edmaFreeParamNegTwo, 10025, NULL);
	RUN_TEST(test_edmaFreeTccNeg, 10026, NULL);
	RUN_TEST(test_edmaGetHandleNegOne,5710, NULL);
	RUN_TEST(test_edmaFreeDmaChannelNegOne, 10027, NULL);
	RUN_TEST(test_edmaBaseAddrNeg, 9059, NULL);
	RUN_TEST(test_edmaRegionIdNeg, 5716, NULL);
	RUN_TEST(test_edmaGetHandleNegTwo,9058, NULL);
	RUN_TEST(test_edmaFreeQdmaChannelNegOne, 10028, NULL);
	RUN_TEST(test_edmaBaseAddrOneNeg, 9060, NULL);
	RUN_TEST(test_edmaRegisterIntrNegThree, 10029, NULL);
	RUN_TEST(test_edmaUnRegisterIntrNegThree, 10030, NULL);
	
	UNITY_END();

    Board_driversClose();
    Drivers_close();

}
