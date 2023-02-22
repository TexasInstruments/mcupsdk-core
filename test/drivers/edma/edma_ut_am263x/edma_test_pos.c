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


/* Positive test of EDMA_ccPaRAMEntry_init API*/
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
	int32_t  testStatusFour = SystemP_SUCCESS;
	uint32_t regionId=EDMA_getRegionId(gEdmaHandle[0]);
	uint32_t baseAddr = EDMA_getBaseAddr(gEdmaHandle[0]);
    uint32_t tccNum;
	uint32_t param;
    uint32_t paramId= EDMA_allocParam(gEdmaHandle[0], &param);
    uint32_t evtQNum;
    uint32_t chType;
    uint32_t chNum;
	uint32_t trigMode = EDMA_TRIG_MODE_MANUAL;

	if(testStausOne == SystemP_SUCCESS)
    {
        tccNum = 70U;
		evtQNum = 0U;
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
        tccNum = 70U;
        evtQNum = 0U;
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
        tccNum = 30U;
		evtQNum = 0U;

		if(EDMA_freeChannelRegion(baseAddr,regionId,chType,chNum,tccNum,paramId,evtQNum) != FALSE)
		{
			testStatusThree = SystemP_FAILURE;
			DebugP_log("failure on line no. %d \r\n", __LINE__);
		}
	}

	if(testStatusFour == SystemP_SUCCESS)
	{
        tccNum = 30U;
		chNum = 30U;
		chType = EDMA_CHANNEL_TYPE_DMA;
		evtQNum = 0U;
		if(EDMA_freeChannelRegion(0U,regionId,chType,chNum,trigMode,tccNum,evtQNum) != TRUE)
		{
			testStatusThree = SystemP_FAILURE;
			DebugP_log("failure on line no. %d \r\n", __LINE__);
		}
	}

	if((testStausOne == SystemP_SUCCESS) 
	&& (testStatusTwo == SystemP_SUCCESS)
	&&(testStatusThree == SystemP_SUCCESS)
	&& (testStatusThree == SystemP_SUCCESS))
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
	uint32_t regionId=EDMA_getRegionId(gEdmaHandle[0]);
	uint32_t baseAddr = EDMA_getBaseAddr(gEdmaHandle[0]);

	if (testStatusOne == SystemP_SUCCESS)
	{
        chNum = 50U;
		if(EDMA_enableTransferRegion(baseAddr,regionId,chNum,EDMA_TRIG_MODE_MANUAL) != TRUE)
		{
			testStatusOne = SystemP_FAILURE;
			DebugP_log("failure on line no. %d \r\n", __LINE__);
		}
	}

	if (testStatusTwo == SystemP_SUCCESS)
	{
		if(EDMA_enableTransferRegion(baseAddr,regionId,7U,EDMA_TRIG_MODE_QDMA) != TRUE)
		{
			testStatusTwo = SystemP_FAILURE;
			DebugP_log("failure on line no. %d \r\n", __LINE__);
		}
	}

	if (testStatusThree == SystemP_SUCCESS)
	{
		if(EDMA_enableTransferRegion(baseAddr,regionId,10U,EDMA_TRIG_MODE_QDMA) != FALSE)
		{
			testStatusThree = SystemP_FAILURE;
			DebugP_log("failure on line no. %d \r\n", __LINE__);
		}
	}

	if (testStatusFour == SystemP_SUCCESS)
	{
		if(EDMA_enableTransferRegion(baseAddr,regionId,7U,EDMA_TRIG_MODE_EVENT) != TRUE)
		{
			testStatusFour = SystemP_FAILURE;
			DebugP_log("failure on line no. %d \r\n", __LINE__);
		}
	}

	if (testStatusFive == SystemP_SUCCESS)
	{
		if(EDMA_enableTransferRegion(baseAddr,regionId,80U,EDMA_TRIG_MODE_EVENT) != FALSE)
		{
			testStatusFive = SystemP_FAILURE;
			DebugP_log("failure on line no. %d \r\n", __LINE__);
		}
	}

	if (testStatusSix == SystemP_SUCCESS)
	{
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
	EDMA_Handle handle = (EDMA_Handle)&gEdmaConfig[0];

	if (testStatus == SystemP_SUCCESS)
	{
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
	uint32_t regionId=EDMA_getRegionId(gEdmaHandle[0]);
	uint32_t baseAddr = EDMA_getBaseAddr(gEdmaHandle[0]);

	if (testStatusOne == SystemP_SUCCESS)
	{
		if(EDMA_disableTransferRegion(baseAddr,regionId,80U,EDMA_TRIG_MODE_MANUAL) != FALSE)
		{
			testStatusOne = SystemP_FAILURE;
		}
	}

	if (testStatusTwo == SystemP_SUCCESS)
	{
		if(EDMA_disableTransferRegion(baseAddr,regionId,7U,EDMA_TRIG_MODE_QDMA) != TRUE)
		{
			testStatusTwo = SystemP_FAILURE;
		}
	}

	if (testStatusThree == SystemP_SUCCESS)
	{
		if(EDMA_disableTransferRegion(baseAddr,regionId,10U,EDMA_TRIG_MODE_QDMA) != FALSE)
		{
			testStatusThree = SystemP_FAILURE;
		}
	}

	if (testStatusFour == SystemP_SUCCESS)
	{
		if(EDMA_disableTransferRegion(baseAddr,regionId,7U,EDMA_TRIG_MODE_EVENT) != TRUE)
		{
			testStatusFour = SystemP_FAILURE;
		}
	}

	if (testStatusFive == SystemP_SUCCESS)
	{
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
	EDMA_Params   *prms = (EDMA_Params *)args;
	uint32_t index = 0U;

	if (testStatus == SystemP_SUCCESS)
	{
		gEdmaConfig[0].object->openPrms.intrEnable = FALSE;
		if(EDMA_open(index,prms) != NULL)
		{
			testStatus = SystemP_FAILURE;
			DebugP_log("EDMA test case: failure on line no. %d \r\n", __LINE__);
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
	EDMA_Handle  gEdmaHandle;
	Edma_IntrObject intrObj;
	EDMA_Params  params;

	if (testStatus == SystemP_SUCCESS)
	{
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
	EDMA_Handle  gEdmaHandle;
	Edma_IntrObject intrObj;
	EDMA_Params  params;

	if (testStatus == SystemP_SUCCESS)
	{	
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
		gEdmaConfig[0].object->isOpen = FALSE;
		EDMA_getBaseAddr(gEdmaHandle);
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

/* Test case for EDMA_close API */
void  test_edmaClosePos(void *args)
{
	int32_t testStatusTwo = SystemP_SUCCESS;
	int32_t testStatusThree = SystemP_SUCCESS;
	int32_t  testStatus = SystemP_SUCCESS;
	EDMA_Handle gEdmaHandle;

	if(testStatusTwo == SystemP_SUCCESS)
	{
		gEdmaConfig[0].object->hwiHandle = NULL;
		
	 	EDMA_close(&gEdmaHandle);
	}
	if(testStatusThree == SystemP_SUCCESS)
	{
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

/* Test case for EDMA_readEventStatusRegion API by chNum TO 20U */
void test_EDMA_readEventStatusRegion(void *args)
{
	uint32_t baseAddr = EDMA_getBaseAddr(gEdmaHandle[0]);
	uint32_t chNum = 20U;
	int32_t testStatus = SystemP_SUCCESS;

	if(EDMA_readEventStatusRegion(baseAddr,chNum)!= SystemP_SUCCESS)
	{
		testStatus = SystemP_FAILURE;
	}
	TEST_ASSERT_EQUAL_INT32(testStatus,SystemP_SUCCESS);
}

/* Test case for EDMA_readEventStatusRegion API by chNum TO 38U */
void test_EDMA_readEventStatusRegionOne(void *args)
{
	uint32_t baseAddr = EDMA_getBaseAddr(gEdmaHandle[0]);
	uint32_t chNum = 38U;
	int32_t testStatus = SystemP_SUCCESS;

	if(EDMA_readEventStatusRegion(baseAddr,chNum)!= SystemP_SUCCESS)
	{
		testStatus = SystemP_FAILURE;
	}
	TEST_ASSERT_EQUAL_INT32(testStatus,SystemP_SUCCESS);
}

/* Test case for EDMA_getHandle API by gEdmaConfig[0].object->isOpen = TRUE state */
void test_edmaGetHandleNegFour(void *args)
{
	int32_t  testStatusThree = SystemP_SUCCESS;
	uint32_t index = 0U;

	if (testStatusThree == SystemP_SUCCESS)
	{
		gEdmaConfig[0].object->isOpen = TRUE;
		if(EDMA_getHandle(index) != NULL)
		{
			testStatusThree = SystemP_FAILURE;
		}
	}
	TEST_ASSERT_EQUAL_INT32(testStatusThree,SystemP_FAILURE);
}

/* Test case for EDMA_readIntrStatusRegion API by tccNum TO 30U */
void test_EDMA_readIntrStatusRegionNeg(void *args)
{
	uint32_t baseAddr = EDMA_getBaseAddr(gEdmaHandle[0]);
	uint32_t regionId = EDMA_getRegionId(gEdmaHandle[0]);
	uint32_t tccNum = 30U;
	int32_t testStatus = SystemP_SUCCESS;

	if(EDMA_readIntrStatusRegion(baseAddr,regionId,tccNum)!= SystemP_SUCCESS)
	{
		testStatus = SystemP_FAILURE;
	}
	TEST_ASSERT_EQUAL_INT32(testStatus,SystemP_SUCCESS);
}

/* Test case for EDMA_readIntrStatusRegion API by tccNum TO 20U */
void test_EDMA_readIntrStatusRegionNegOne(void *args)
{
	uint32_t baseAddr = EDMA_getBaseAddr(gEdmaHandle[0]);
	uint32_t regionId = EDMA_getRegionId(gEdmaHandle[0]);
	uint32_t tccNum = 20U;
	int32_t testStatus = SystemP_SUCCESS;
	if(EDMA_readIntrStatusRegion(baseAddr,regionId,tccNum)!= SystemP_SUCCESS)
	{
		testStatus = SystemP_FAILURE;
	}
	TEST_ASSERT_EQUAL_INT32(testStatus,SystemP_SUCCESS);
}

/* Test case for EDMA_readEventStatusRegion API by tccNum TO 30U */
void test_EDMA_readEventStatusRegionNeg(void *args)
{
	uint32_t baseAddr = EDMA_getBaseAddr(gEdmaHandle[0]);
	uint32_t tccNum = 30U;
	int32_t testStatus = SystemP_SUCCESS;

	if(EDMA_readEventStatusRegion(baseAddr,tccNum)!= SystemP_SUCCESS)
	{
		testStatus = SystemP_FAILURE;
	}
	TEST_ASSERT_EQUAL_INT32(testStatus,SystemP_SUCCESS);
}

/* Test case for EDMA_readEventStatusRegion API by tccNum TO 40U */
void test_EDMA_readEventStatusRegionNegOne(void *args)
{
	uint32_t baseAddr = EDMA_getBaseAddr(gEdmaHandle[0]);
	uint32_t tccNum = 40U;
	int32_t testStatus = SystemP_SUCCESS;

	if(EDMA_readEventStatusRegion(baseAddr,tccNum)!= SystemP_SUCCESS)
	{
		testStatus = SystemP_FAILURE;
	}
	TEST_ASSERT_EQUAL_INT32(testStatus,SystemP_SUCCESS);
}

/* Test case for EDMA_registerIntr API by gEdmaConfig[0].object->isOpen = FALSE */
void test_edmaRegisterIntrNegOne(void *args)
{
	int32_t testStatus = SystemP_SUCCESS;
	Edma_IntrObject     intrObj;

	if(testStatus == SystemP_SUCCESS)
	{
		gEdmaConfig[0].object->isOpen = FALSE;
		if(EDMA_registerIntr(gEdmaHandle[0],&intrObj) != SystemP_FAILURE)
		{
			testStatus = SystemP_FAILURE;
		}
	}
}
	
/* Test case for EDMA_registerIntr API by gEdmaConfig[0].object->openPrms.intrEnable = FALSE */	
void test_edmaRegisterIntrNegTwo(void *args)
{
	int32_t testStatus = SystemP_SUCCESS;
	Edma_IntrObject     intrObj;

	if(testStatus == SystemP_SUCCESS)
	{
		gEdmaConfig[0].object->isOpen = FALSE;
		gEdmaConfig[0].object->openPrms.intrEnable = FALSE;

		if(EDMA_registerIntr(NULL,&intrObj) != SystemP_FAILURE)
		{
			testStatus = SystemP_FAILURE;
		}
	}
}

/* Test case for EDMA_unregisterIntr API by gEdmaConfig[0].object->isOpen = FALSE */	
void test_edmaUnRegisterIntrNegOne(void *args)
{
	int32_t testStatus = SystemP_SUCCESS;
	Edma_IntrObject     intrObj;

	if(testStatus == SystemP_SUCCESS)
	{
		gEdmaConfig[0].object->isOpen = FALSE;
		if(EDMA_unregisterIntr(gEdmaHandle[0],&intrObj) != SystemP_FAILURE)
		{
			testStatus = SystemP_FAILURE;
		}
	}
}

/* Test case for EDMA_open API by index to 1U */
void test_edmaOpenNegOne(void *args)
{
	int32_t testStatus = SystemP_SUCCESS;
	EDMA_Params   *prms = (EDMA_Params *)args;
	uint32_t index = 1U;
		
	if (testStatus == SystemP_SUCCESS)
	{
		if(EDMA_open(index,prms) != NULL)
		{
			testStatus = SystemP_FAILURE;
			DebugP_log("EDMA test case: failure on line no. %d \r\n", __LINE__);
		}
	}
	TEST_ASSERT_EQUAL_INT32(testStatus,SystemP_SUCCESS);
}

/* Test case for EDMA_freeDmaChannel API by passing NULL to handle parameter */
void test_EDMA_freeResource(void *args)
{
	int32_t  testStatus = SystemP_SUCCESS;

	if (testStatus == SystemP_SUCCESS)
	{	
		if(EDMA_freeDmaChannel(NULL,(uint32_t *)10U) == SystemP_SUCCESS)
		{
			testStatus = SystemP_FAILURE;
			DebugP_log("EDMA test case: failure on line no. %d \r\n", __LINE__);
		}
	}
}

/* Test case for EDMA_close API */
void  test_edmaCloseNegTwo(void *args)
{
	int32_t testStatusTwo = SystemP_SUCCESS;
	int32_t  testStatus = SystemP_SUCCESS;

	if(testStatusTwo == SystemP_SUCCESS)
	{
	 	EDMA_close(NULL);
	}

	else
	{
		testStatus = SystemP_FAILURE;
	    DebugP_log("EDMA test case: failure on line no. %d \r\n", __LINE__);
	}

	TEST_ASSERT_EQUAL_INT32(testStatus,SystemP_SUCCESS);
}

/* Test case for EDMA_getHandle API  by assigning isOpen to False state */
void test_edmaGetHandlePosTwo(void *args)
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

/* Test case for EDMA_close API by gEdmaConfig[0].object = NULL */
void  test_edmaGetHandlePos(void *args)
{
	int32_t  testStatusTwo = SystemP_SUCCESS;
	uint32_t index = 0U;

	if (testStatusTwo == SystemP_SUCCESS)
	{
		gEdmaConfig[0].object = NULL;
		if(EDMA_getHandle(index) != NULL)
		{
			testStatusTwo = SystemP_FAILURE;
		}
	}
	TEST_ASSERT_EQUAL_INT32(testStatusTwo,SystemP_SUCCESS);
}

/* Test case for EDMA_close API */
void  test_edmaClosePosFive(void *args)
{
	int32_t testStatusTwo = SystemP_SUCCESS;
	int32_t testStatusThree = SystemP_SUCCESS;
	int32_t  testStatus = SystemP_SUCCESS;
	EDMA_Handle handle = (EDMA_Handle)&gEdmaConfig[0];

	if(testStatusTwo == SystemP_SUCCESS)
	{
	 	EDMA_close(&handle);
	}
	
	if(testStatusThree == SystemP_SUCCESS)
	{
	 	EDMA_close(&handle);
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

/*  Positive test of EDMA_freeChannelRegion API by assigning chNum to 70U*/
void test_edmaFreeChannelRegionPosThree(void *args)
{
	int32_t  testStatus = SystemP_SUCCESS;
	uint32_t regionId = EDMA_getRegionId(&gEdmaHandle[0]);
	uint32_t baseAddr = EDMA_getBaseAddr(&gEdmaHandle[0]);
	uint32_t chType = EDMA_CHANNEL_TYPE_DMA;
    uint32_t tccNum = 10U;
    uint32_t evtQNum = 1U;
    uint32_t trigMode = EDMA_TRIG_MODE_MANUAL;
    uint32_t chNum = 70U;
		
	if(testStatus == SystemP_SUCCESS)
    {
        if(EDMA_freeChannelRegion(baseAddr,regionId,chType,chNum,trigMode,tccNum,evtQNum) != FALSE)
        {
            testStatus = SystemP_FAILURE;
			DebugP_log("failure on line no. %d \r\n", __LINE__);
        }
	}
}

/*  Positive test of EDMA_freeChannelRegion API by assigning chNum to 80U*/
void test_edmaFreeChannelRegionPosFour(void *args)
{
	int32_t  testStatus = SystemP_SUCCESS;
	uint32_t regionId = EDMA_getRegionId(&gEdmaHandle[0]);
	uint32_t baseAddr = EDMA_getBaseAddr(&gEdmaHandle[0]);
	uint32_t chType = EDMA_CHANNEL_TYPE_DMA;
    uint32_t tccNum = 0U;
    uint32_t evtQNum = 1U;
    uint32_t trigMode = EDMA_TRIG_MODE_MANUAL;
    uint32_t chNum = 80U;

	if(testStatus == SystemP_SUCCESS)
    {
        if(EDMA_freeChannelRegion(baseAddr,regionId,chType,chNum,trigMode,tccNum,evtQNum) != FALSE)
        {
            testStatus = SystemP_FAILURE;
			DebugP_log("failure on line no. %d \r\n", __LINE__);
        }
	}
}

/*  Positive test of EDMA_freeChannelRegion API by assigning chNum to 100U*/
void test_edmaFreeChannelRegionPosFive(void *args)
{
	int32_t  testStatus = SystemP_SUCCESS;
	uint32_t regionId = EDMA_getRegionId(&gEdmaHandle[0]);
	uint32_t baseAddr = EDMA_getBaseAddr(&gEdmaHandle[0]);
	uint32_t chType = EDMA_CHANNEL_TYPE_DMA;
    uint32_t tccNum = 0U;
    uint32_t evtQNum = 1U;
    uint32_t trigMode = EDMA_TRIG_MODE_MANUAL;
    uint32_t chNum = 100U;

	if(testStatus == SystemP_SUCCESS)
    {
        if(EDMA_freeChannelRegion(baseAddr,regionId,chType,chNum,trigMode,tccNum,evtQNum) != FALSE)
        {
            testStatus = SystemP_FAILURE;
			DebugP_log("failure on line no. %d \r\n", __LINE__);
        }
	}
}

/*  Positive test of EDMA_allocDmaChannel API by assigning dmaCh to 0U*/
void test_edmaAllocDmaChannelPosOne(void *args)
{
	int32_t  testStatus = SystemP_SUCCESS;
	uint32_t dmaCh = 0U;

	if (testStatus == SystemP_SUCCESS)
	{
		if(EDMA_allocDmaChannel(&gEdmaHandle[0],&dmaCh) != SystemP_SUCCESS)
		{
			testStatus = SystemP_FAILURE;
			DebugP_log("EDMA test case: failure on line no. %d \r\n", __LINE__);
		}
	}
	TEST_ASSERT_EQUAL_INT32(testStatus,SystemP_SUCCESS);
}

/*  Positive test of EDMA_allocDmaChannel API by assigning dmaCh to 65U*/
void test_edmaAllocDmaChannelPosTwo(void *args)
{
	int32_t  testStatus = SystemP_SUCCESS;
	uint32_t dmaCh = 65U;

	if (testStatus == SystemP_SUCCESS)
	{
		if(EDMA_allocDmaChannel(&gEdmaHandle[0],&dmaCh) != SystemP_SUCCESS)
		{
			testStatus = SystemP_FAILURE;
			DebugP_log("EDMA test case: failure on line no. %d \r\n", __LINE__);
		}
	}
	TEST_ASSERT_EQUAL_INT32(testStatus,SystemP_SUCCESS);
}

/*  Positive test of EDMA_allocDmaChannel API by calling 30 times */
void test_allocResource(void *args)
{
    int32_t             testStatus = SystemP_SUCCESS;
    uint32_t            dmaCh;

     for(int i=0;i<30;i++)
	{
		dmaCh = EDMA_RESOURCE_ALLOC_ANY;
    	testStatus = EDMA_allocDmaChannel(gEdmaHandle[0], &dmaCh);

	}
	TEST_ASSERT_EQUAL_INT32(SystemP_FAILURE, testStatus);
}

/*  Positive test of EDMA_allocDmaChannel API by calling 30 times */
void test_allocResourceTwo(void *args)
{
    int32_t             testStatus = SystemP_SUCCESS;
    uint32_t            dmaCh;

	for(int i=0;i<40;i++)
	{
		dmaCh = EDMA_RESOURCE_ALLOC_ANY;
    	testStatus = EDMA_allocTcc(gEdmaHandle[0], &dmaCh);
	}

	TEST_ASSERT_EQUAL_INT32(SystemP_FAILURE, testStatus);
}

/*  Positive test of EDMA_allocQdmaChannel API by allocating dmaCh to 0  */
void test_allocResourceOne(void *args)
{
    int32_t             testStatus = SystemP_SUCCESS;
    uint32_t           dmaCh = 0;

    testStatus = EDMA_allocQdmaChannel(&gEdmaHandle[0], &dmaCh);
    TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, testStatus);
}

/*  Positive test of EDMA_deinit */
void test_edmaInitPos(void *args)
{
    int32_t      testStatus = SystemP_SUCCESS;
    EDMA_init();
	EDMA_deinit();
    TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, testStatus);
}

/*  Positive test of EDMA_clrCCErr API*/
void test_EDMA_clrCCErr(void *args)
{
    uint32_t            baseAddr;
    int32_t             testStatus = SystemP_SUCCESS;
   
    baseAddr = EDMA_getBaseAddr(gEdmaHandle[0]);
    EDMA_clrCCErr(baseAddr, 1);
    TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, testStatus);

}

void edma_posTest(void *args)
{

    uint32_t  baseAddr;

    Drivers_open();
    Board_driversOpen();

	baseAddr = EDMA_getBaseAddr(gEdmaHandle[0]);
    DebugP_assert(baseAddr != 0);

    UNITY_BEGIN();

	RUN_TEST(test_edmaInitPos, 10031, NULL);
	RUN_TEST(test_edmaInitParamsInitPos, 5487, NULL);
	RUN_TEST(test_edmaCcPaRAMEntryInitPos, 5488, NULL);
	RUN_TEST(test_edmaGetCCErrStatusPos, 5525, NULL);
	RUN_TEST(test_edmaEnableTransferRegionPos, 5522, NULL);
	RUN_TEST(test_edmaDisableTransferRegionPos, 5523, NULL);
	RUN_TEST(test_edmaUnRegisterIntrPos, 5544, NULL);
	RUN_TEST(test_edmaRegisterIntrPos, 5543, NULL);
	RUN_TEST(test_edmaOpenPos, 5539, NULL);
	RUN_TEST(EDMA_initializePos, 6840, NULL);
	RUN_TEST(test_edmaBaseAddrTwoPos, 9001, NULL);
	RUN_TEST(test_edmaRegionIdTwoPos, 8997, NULL);
	RUN_TEST(test_edmaRegionIdThreePos, 5546, NULL);
	RUN_TEST(test_edmaFreeChannelRegionPos, 5521, NULL);
	RUN_TEST(test_EDMA_readEventStatusRegion, 10032, NULL);
	RUN_TEST(test_EDMA_readEventStatusRegionOne, 10033, NULL);
	RUN_TEST(test_edmaGetHandleNegFour, 10034,NULL);
	RUN_TEST(test_edmaOpenNegOne, 5709, NULL);
	RUN_TEST(test_EDMA_readIntrStatusRegionNeg, 10035, NULL);
	RUN_TEST(test_edmaFreeChannelRegionPosFour, 10036, NULL);
	RUN_TEST(test_EDMA_readIntrStatusRegionNegOne, 10037, NULL);
	RUN_TEST(test_edmaRegisterIntrNegOne, 10038, NULL);
	RUN_TEST(test_edmaRegisterIntrNegTwo, 10039, NULL);
	RUN_TEST(test_edmaUnRegisterIntrNegOne, 10040, NULL);
	RUN_TEST(test_edmaFreeChannelRegionPosFive, 10041, NULL);
	RUN_TEST(test_EDMA_freeResource, 10042, NULL);
	RUN_TEST(test_edmaClosePos, 10043, NULL);
	RUN_TEST(test_edmaCloseNegTwo,10044, NULL);
	RUN_TEST(test_edmaGetHandlePosTwo, 10045, NULL);
	RUN_TEST(test_edmaGetHandlePos, 10046, NULL);
	RUN_TEST(test_edmaFreeChannelRegionPosThree, 10047, NULL);
	RUN_TEST(test_edmaAllocDmaChannelPosOne, 10049, NULL);
	RUN_TEST(test_edmaAllocDmaChannelPosTwo, 10048, NULL);
	RUN_TEST(test_allocResource, 10050, NULL);
	RUN_TEST(test_EDMA_clrCCErr, 10051, NULL);
	RUN_TEST(test_allocResourceOne, 10052, NULL);
	RUN_TEST(test_allocResourceTwo, 10053, NULL);
	RUN_TEST(test_edmaClosePosFive, 5711, NULL);
	
	UNITY_END();

    Board_driversClose();
    Drivers_close();

}
