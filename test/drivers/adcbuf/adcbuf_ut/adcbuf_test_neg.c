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

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <stdio.h>
#include <string.h>
#include <inttypes.h>
#include <drivers/soc.h>
#include <kernel/dpl/ClockP.h>
#include <kernel/dpl/SemaphoreP.h>
#include <drivers/adcbuf.h>
#include <unity.h>
#include <drivers/adcbuf/v0/adcbuf.h>
#include "ti_drivers_open_close.h"


/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */
#define CONFIG_ADCBUF10   						10
#define ADCBufMMWave_CMD_DEFAULT_MODE        	100U   

/* ========================================================================== */
/*                 Internal Function Declarations                             */
/* ========================================================================== */

/* Negative test for ADCBuf_open API */
void negTest_adcbuf_open(void *args)
{
    int32_t    testStatus = SystemP_SUCCESS;
	ADCBuf_Params        params;
    ADCBuf_Params_init(&params);
	
	if (testStatus == SystemP_SUCCESS)
    {
        if(ADCBuf_open(CONFIG_ADCBUF10, &params) != (ADCBuf_Handle)NULL)
        {
            DebugP_log("adcbuf_neg_Test: failure on line no. %d \n", __LINE__);
            testStatus = SystemP_FAILURE;
        }
    }
	
    TEST_ASSERT_EQUAL_INT32(testStatus,SystemP_SUCCESS);
}

/* Negative test for ADCBuf_control API */
void negTest_adcbuf_control(void *args)
{
    int32_t    			testStatus = SystemP_SUCCESS;
	uint32_t            arg;
	ADCBuf_Handle       handle;
    ADCBufMMWave_CMD    command;
	ADCBuf_Params       params;
	ADCBuf_CQConf      cqConf;
	
	command        = ADCBufMMWave_CMD_START_CONTINUOUS_MODE;
	arg            = 0x5U;
	
	ADCBuf_Params_init(&params);
    handle = ADCBuf_open(CONFIG_ADCBUF0, &params);

	if (testStatus == SystemP_SUCCESS)
    {
        if(ADCBuf_control(handle, command, (void *)&arg) == ADCBUF_STATUS_INVALID_PARAMS)
        {
            DebugP_log("adcbuf_neg_Test: failure on line no. %d \n", __LINE__);
            testStatus = SystemP_FAILURE;
        }
    }
	
	command        = ADCBufMMWave_CMD_SET_SRC;
	arg            = (uint32_t)((0x1U << 1U) -1);
	
	if (testStatus == SystemP_SUCCESS)
    {
        if(ADCBuf_control(handle, command, (void *)&arg) != (int32_t)SystemP_SUCCESS)
        {
            DebugP_log("adcbuf_neg_Test: failure on line no. %d \n", __LINE__);
            testStatus = SystemP_FAILURE;
        }

    }
	
	command        = ADCBufMMWave_CMD_SET_PING_CHIRP_THRESHHOLD;	
	arg            = 0x5U;
	if (testStatus == SystemP_SUCCESS)
    {
        if(ADCBuf_control(handle, command, (void *)&arg) != (int32_t)SystemP_SUCCESS)
        {
            DebugP_log("adcbuf_neg_Test: failure on line no. %d \n", __LINE__);
            testStatus = SystemP_FAILURE;
        }
    }
	
	command        = ADCBufMMWave_CMD_SET_PONG_CHIRP_THRESHHOLD;
	arg            = 0x5U;	
	if (testStatus == SystemP_SUCCESS)
    {
        if(ADCBuf_control(handle, command, (void *)&arg) != (int32_t)SystemP_SUCCESS)
        {
            DebugP_log("adcbuf_neg_Test: failure on line no. %d \n", __LINE__);
            testStatus = SystemP_FAILURE;
        }
    }
	
	command        = ADCBufMMWave_CMD_STOP_CONTINUOUS_MODE;	
	arg            = 0x5U;
	if (testStatus == SystemP_SUCCESS)
    {
        if(ADCBuf_control(handle, command, (void *)&arg) != (int32_t)SystemP_SUCCESS)
        {
            DebugP_log("adcbuf_neg_Test: failure on line no. %d \n", __LINE__);
            testStatus = SystemP_FAILURE;
        }
    }
	
	command        = ADCBufMMWave_CMD_CONF_CQ;	
	cqConf.cqDataWidth 			= (uint8_t)((0x1U<<2U) -1U);
    cqConf.cq96BitPackEn 			= (uint8_t)((0x1U<<1U) -1U);
	if (testStatus == SystemP_SUCCESS)
    {
        if(ADCBuf_control(handle, command, (void *)&cqConf) != (int32_t)SystemP_SUCCESS)
        {
            DebugP_log("adcbuf_pos_Test: failure on line no. %d \n", __LINE__);
            testStatus = SystemP_FAILURE;
        }
    }
	
	if (testStatus == SystemP_SUCCESS)
    {
        if(ADCBuf_control(handle, ADCBufMMWave_CMD_DEFAULT_MODE, (void *)&arg) != (int32_t)ADCBUF_STATUS_UNDEFINEDCMD)
        {
            DebugP_log("adcbuf_neg_Test: failure on line no. %d \n", __LINE__);
            testStatus = SystemP_FAILURE;
        }
    }
    TEST_ASSERT_EQUAL_INT32(testStatus,SystemP_SUCCESS);
	ADCBuf_close(handle);
}

/* Negative test for ADCBUF_MMWave_getCQBufAddr API */
void negTest_adcbufmmwave_get_cqbufferaddr(void *args)
{
    int32_t  testStatus = SystemP_SUCCESS;
	int32_t  retVal 	= 	0;
	ADCBuf_Params       params;
	ADCBuf_Handle       handle;
	handle 				= (ADCBuf_Handle)NULL;
		
	ADCBuf_Params_init(&params);
    handle = ADCBuf_open(CONFIG_ADCBUF0, &params);

	if (testStatus == SystemP_SUCCESS)
    {
		ADCBUF_MMWave_getCQBufAddr(handle, ADCBufMMWave_CQType_CQ0, &retVal);
        if(retVal != SystemP_SUCCESS)
        {
            DebugP_log("adcbuf_neg_Test: failure on line no. %d \n", __LINE__);
            testStatus = SystemP_FAILURE;
        }
    }
	
	if (testStatus == SystemP_SUCCESS)
    {
		ADCBUF_MMWave_getCQBufAddr(handle, ADCBufMMWave_CQType_CQ0, &retVal);
        if(retVal != SystemP_SUCCESS)
        {
            DebugP_log("adcbuf_neg_Test: failure on line no. %d \n", __LINE__);
            testStatus = SystemP_FAILURE;
        }
    }
	
	if (testStatus == SystemP_SUCCESS)
    {
		ADCBUF_MMWave_getCQBufAddr(handle, ADCBufMMWave_CQType_CQ1, &retVal);
        if(retVal != SystemP_SUCCESS)
        {
            DebugP_log("adcbuf_neg_Test: failure on line no. %d \n", __LINE__);
            testStatus = SystemP_FAILURE;
        }
    }
	
		if (testStatus == SystemP_SUCCESS)
    {
		ADCBUF_MMWave_getCQBufAddr(handle, ADCBufMMWave_CQType_CQ2, &retVal);
        if(retVal != SystemP_SUCCESS)
        {
            DebugP_log("adcbuf_neg_Test: failure on line no. %d \n", __LINE__);
            testStatus = SystemP_FAILURE;
        }
    }
	
	if (testStatus == SystemP_SUCCESS)
    {
		ADCBUF_MMWave_getCQBufAddr(handle, ADCBufMMWave_CQType_MAX_CQ, &retVal);
        if(retVal != ADCBUF_STATUS_INVALID_PARAMS)
        {
            DebugP_log("adcbuf_neg_Test: failure on line no. %d \n", __LINE__);
            testStatus = SystemP_FAILURE;
        }
    }
    TEST_ASSERT_EQUAL_INT32(testStatus,SystemP_SUCCESS);
	ADCBuf_close(handle);
}

/* Negative test for ADCBUF_verifySrcSelCfg API */
void negTest_adcbufverifySrcSelCfg(void *args)
{
	int32_t    				testStatus = SystemP_SUCCESS;
	uint32_t            	source;
	ADCBuf_Params       	params;
	ADCBuf_Handle        	handle;
	
	ADCBuf_Params_init(&params);
    handle = ADCBuf_open(CONFIG_ADCBUF0, &params);
	
	source = 0U;
	if (testStatus == SystemP_SUCCESS)
    {
        if(ADCBUF_verifySrcSelCfg(handle,source) != SystemP_SUCCESS)
        {
            DebugP_log("adcbuf_neg_Test: failure on line no. %d \n", __LINE__);
            testStatus = SystemP_FAILURE;
        }
    }
	
	source = ADCBUF_SOURCE_SELECT_MAX;
	if (testStatus == SystemP_SUCCESS)
    {
        if(ADCBUF_verifySrcSelCfg(handle,source) != SystemP_FAILURE)
        {
            DebugP_log("adcbuf_neg_Test: failure on line no. %d \n", __LINE__);
            testStatus = SystemP_FAILURE;
        }
    }
	ADCBuf_close(handle);
	TEST_ASSERT_EQUAL_INT32(testStatus,SystemP_SUCCESS);
}

/* Negative test for ADCBUF_verifyChirpThreshold API */
void negTest_adcbufVerifyChirpThreshold(void *args)
{
	int32_t    				testStatus = SystemP_SUCCESS;
	uint32_t            	pingThreshCfg, pongThreshCfg;
	ADCBuf_Handle        	handle;
	ADCBuf_Params       	params;
	
	handle         = (ADCBuf_Handle) NULL;
	pongThreshCfg         = ADCBUF_PING_THRESHOLD_MAX;
	pingThreshCfg		  = ADCBUF_PONG_THRESHOLD_MAX;
	if (testStatus == SystemP_SUCCESS)
    {
        if(ADCBUF_verifyChirpThreshold(handle,pingThreshCfg,pongThreshCfg) != ADCBUF_STATUS_INVALID_PARAMS)
        {
            DebugP_log("adcbuf_neg_Test: failure on line no. %d \n", __LINE__);
            testStatus = SystemP_FAILURE;
        }
    }
	
	ADCBuf_Params_init(&params);
    handle = ADCBuf_open(CONFIG_ADCBUF0, &params);
	
	pongThreshCfg         = ADCBUF_PING_THRESHOLD_MAX +1U;
	pingThreshCfg		  = ADCBUF_PONG_THRESHOLD_MAX;
	if (testStatus == SystemP_SUCCESS)
    {
        if(ADCBUF_verifyChirpThreshold(handle,pingThreshCfg,pongThreshCfg) != ADCBUF_STATUS_INVALID_PARAMS)
        {
            DebugP_log("adcbuf_neg_Test: failure on line no. %d \n", __LINE__);
            testStatus = SystemP_FAILURE;
        }
    }
	
	pongThreshCfg         = ADCBUF_PING_THRESHOLD_MAX;
	pingThreshCfg		  = ADCBUF_PONG_THRESHOLD_MAX +1U;
	if (testStatus == SystemP_SUCCESS)
    {
        if(ADCBUF_verifyChirpThreshold(handle,pingThreshCfg,pongThreshCfg) != ADCBUF_STATUS_INVALID_PARAMS)
        {
            DebugP_log("adcbuf_neg_Test: failure on line no. %d \n", __LINE__);
            testStatus = SystemP_FAILURE;
        }
    }
		
	pingThreshCfg		  = 1U;
	pongThreshCfg         = ADCBUF_PING_THRESHOLD_MAX;
	if (testStatus == SystemP_SUCCESS)
    {
        if(ADCBUF_verifyChirpThreshold(handle,pingThreshCfg,pongThreshCfg) != SystemP_FAILURE)
        {
            DebugP_log("adcbuf_neg_Test: failure on line no. %d \n", __LINE__);
            testStatus = SystemP_FAILURE;
        }
    }
   
   pongThreshCfg         = ADCBUF_PING_THRESHOLD_MAX;
	pingThreshCfg		  = ADCBUF_PING_THRESHOLD_MAX;
	if (testStatus == SystemP_SUCCESS)
    {
        if(ADCBUF_verifyChirpThreshold(handle,pingThreshCfg,pongThreshCfg) != SystemP_FAILURE)
        {
            DebugP_log("adcbuf_neg_Test: failure on line no. %d \n", __LINE__);
            testStatus = SystemP_FAILURE;
        }
    }
	ADCBuf_close(handle);
   TEST_ASSERT_EQUAL_INT32(testStatus,SystemP_SUCCESS);
	
}

/* Negative test for ADCBUF_verifyContinuousModeCfg API */
void negTest_adcbufVerifyContinuousModeCfg(void *args)
{
	int32_t    				testStatus = SystemP_SUCCESS;
	uint32_t            	continuousModeCfg;
	ADCBuf_Handle        	handle;
	ADCBuf_Params       	params;
	
	handle         			  = (ADCBuf_Handle) NULL;
	continuousModeCfg         = ADCBUF_CONTINUOUS_MODE_MAX;
	if (testStatus == SystemP_SUCCESS)
    {
        if(ADCBUF_verifyContinuousModeCfg(handle,continuousModeCfg) != ADCBUF_STATUS_INVALID_PARAMS)
        {
            DebugP_log("adcbuf_neg_Test: failure on line no. %d \n", __LINE__);
            testStatus = SystemP_FAILURE;
        }
    }
	
	ADCBuf_Params_init(&params);
    handle = ADCBuf_open(CONFIG_ADCBUF0, &params);
	
	continuousModeCfg         = ADCBUF_CONTINUOUS_MODE_MAX +1U;
	if (testStatus == SystemP_SUCCESS)
    {
        if(ADCBUF_verifyContinuousModeCfg(handle,continuousModeCfg) != ADCBUF_STATUS_INVALID_PARAMS)
        {
            DebugP_log("adcbuf_neg_Test: failure on line no. %d \n", __LINE__);
            testStatus = SystemP_FAILURE;
        }
    }
	
	continuousModeCfg         = ADCBUF_CONTINUOUS_MODE_MAX;
	if (testStatus == SystemP_SUCCESS)
    {
        if(ADCBUF_verifyContinuousModeCfg(handle,continuousModeCfg) != SystemP_FAILURE)
        {
            DebugP_log("adcbuf_neg_Test: failure on line no. %d \n", __LINE__);
            testStatus = SystemP_FAILURE;
        }
    }
	
	ADCBuf_close(handle);
   TEST_ASSERT_EQUAL_INT32(testStatus,SystemP_SUCCESS);
	
}

/* Negative test for ADCBUF_readStaticRegs API */
void negTest_adcbufrRadStaticRegs(void *args)
{
	int32_t    				testStatus = SystemP_SUCCESS;
	ADCBuf_Handle        	handle;
	ADCBuf_Params       	params;
	ADCBUF_StaticRegs   dummyStaticRegs;
	ADCBUF_StaticRegs   *pStaticRegs = &dummyStaticRegs;
		
	handle         			  = (ADCBuf_Handle) NULL;
	if (testStatus == SystemP_SUCCESS)
    {
        if(ADCBUF_readStaticRegs(handle,pStaticRegs) != ADCBUF_STATUS_INVALID_PARAMS)
        {
            DebugP_log("adcbuf_neg_Test: failure on line no. %d \n", __LINE__);
            testStatus = SystemP_FAILURE;
        }
    }
	
	handle         			  = (ADCBuf_Handle) NULL;
	pStaticRegs                = NULL;
	if (testStatus == SystemP_SUCCESS)
    {
        if(ADCBUF_readStaticRegs(handle,pStaticRegs) != ADCBUF_STATUS_INVALID_PARAMS)
        {
            DebugP_log("adcbuf_neg_Test: failure on line no. %d \n", __LINE__);
            testStatus = SystemP_FAILURE;
        }
    }
	
	ADCBuf_Params_init(&params);
    handle = ADCBuf_open(CONFIG_ADCBUF0, &params);	
	pStaticRegs                = NULL;
	if (testStatus == SystemP_SUCCESS)
    {
        if(ADCBUF_readStaticRegs(handle,pStaticRegs) != ADCBUF_STATUS_INVALID_PARAMS)
        {
            DebugP_log("adcbuf_neg_Test: failure on line no. %d \n", __LINE__);
            testStatus = SystemP_FAILURE;
        }
    }
	ADCBuf_close(handle);
   TEST_ASSERT_EQUAL_INT32(testStatus,SystemP_SUCCESS);
	
}

/* Negavtive test for ADCBUF_verifyDataFormatCfg API */
void negTest_adcbufVerifyDataFormatCfg(void *args)
{
	int32_t    				testStatus = SystemP_SUCCESS;
	ADCBuf_Handle        	handle;
	ADCBuf_Params       	params;
	uint32_t 				dataFormatcfg;
	uint32_t 				interleavecfg;
	uint32_t 				iqConfig;
	
	handle         			  = (ADCBuf_Handle) NULL;
	dataFormatcfg = ADCBUF_DATA_FMT_MAX;
	interleavecfg = ADCBUF_WRITEMODE_MAX;
	iqConfig	  = ADCBUF_IQSWAP_CFG_MAX;
	
	if (testStatus == SystemP_SUCCESS)
    {
        if(ADCBUF_verifyDataFormatCfg(handle,dataFormatcfg,interleavecfg,iqConfig) != ADCBUF_STATUS_INVALID_PARAMS)
        {
            DebugP_log("adcbuf_neg_Test: failure on line no. %d \n", __LINE__);
            testStatus = SystemP_FAILURE;
        }
    }
	
	ADCBuf_Params_init(&params);
    handle = ADCBuf_open(CONFIG_ADCBUF0, &params);	
	
	dataFormatcfg = ADCBUF_DATA_FMT_MAX +1U;
	interleavecfg = ADCBUF_WRITEMODE_MAX;
	iqConfig	  = ADCBUF_IQSWAP_CFG_MAX;
	
	if (testStatus == SystemP_SUCCESS)
    {
        if(ADCBUF_verifyDataFormatCfg(handle,dataFormatcfg,interleavecfg,iqConfig) != ADCBUF_STATUS_INVALID_PARAMS)
        {
            DebugP_log("adcbuf_neg_Test: failure on line no. %d \n", __LINE__);
            testStatus = SystemP_FAILURE;
        }
    }
	
	dataFormatcfg = ADCBUF_DATA_FMT_MAX;
	interleavecfg = ADCBUF_WRITEMODE_MAX +1U;
	iqConfig	  = ADCBUF_IQSWAP_CFG_MAX;
	
	if (testStatus == SystemP_SUCCESS)
    {
        if(ADCBUF_verifyDataFormatCfg(handle,dataFormatcfg,interleavecfg,iqConfig) != ADCBUF_STATUS_INVALID_PARAMS)
        {
            DebugP_log("adcbuf_neg_Test: failure on line no. %d \n", __LINE__);
            testStatus = SystemP_FAILURE;
        }
    }
	
	dataFormatcfg = ADCBUF_DATA_FMT_MAX;
	interleavecfg = ADCBUF_WRITEMODE_MAX;
	iqConfig	  = ADCBUF_IQSWAP_CFG_MAX +1U;
	
	if (testStatus == SystemP_SUCCESS)
    {
        if(ADCBUF_verifyDataFormatCfg(handle,dataFormatcfg,interleavecfg,iqConfig) != ADCBUF_STATUS_INVALID_PARAMS)
        {
            DebugP_log("adcbuf_neg_Test: failure on line no. %d \n", __LINE__);
            testStatus = SystemP_FAILURE;
        }
    }
	
	dataFormatcfg = ADCBUF_DATA_FMT_MAX - 1U;
	interleavecfg = 0U;
	iqConfig	  = 0U;
	
	if (testStatus == SystemP_SUCCESS)
    {
        if(ADCBUF_verifyDataFormatCfg(handle,dataFormatcfg,interleavecfg,iqConfig) != SystemP_FAILURE )
        {
            DebugP_log("adcbuf_pos_Test: failure on line no. %d \n", __LINE__);
            testStatus = SystemP_FAILURE;
        }
    }
	
	dataFormatcfg = 1U;
	interleavecfg = ADCBUF_WRITEMODE_MAX;
	iqConfig	  = 0U;
	
	if (testStatus == SystemP_SUCCESS)
    {
        if(ADCBUF_verifyDataFormatCfg(handle,dataFormatcfg,interleavecfg,iqConfig) != SystemP_FAILURE )
        {
            DebugP_log("adcbuf_pos_Test: failure on line no. %d \n", __LINE__);
            testStatus = SystemP_FAILURE;
        }
    }
	dataFormatcfg = 1U;
	interleavecfg = 0U;
	iqConfig	  = ADCBUF_IQSWAP_CFG_MAX;
	
	if (testStatus == SystemP_SUCCESS)
    {
        if(ADCBUF_verifyDataFormatCfg(handle,dataFormatcfg,interleavecfg,iqConfig) != SystemP_FAILURE )
        {
            DebugP_log("adcbuf_pos_Test: failure on line no. %d \n", __LINE__);
            testStatus = SystemP_FAILURE;
        }
    }
	ADCBuf_close(handle);
   TEST_ASSERT_EQUAL_INT32(testStatus,SystemP_SUCCESS);
	
}

void test_neg_main(void *args)
{
	RUN_TEST(negTest_adcbuf_open, 10891, NULL);
	RUN_TEST(negTest_adcbuf_control, 10892, NULL);
	RUN_TEST(negTest_adcbufmmwave_get_cqbufferaddr, 10890, NULL);	
	RUN_TEST(negTest_adcbufverifySrcSelCfg, 11196, NULL);
	RUN_TEST(negTest_adcbufVerifyChirpThreshold, 11199, NULL);
	RUN_TEST(negTest_adcbufVerifyContinuousModeCfg, 11200, NULL);	
	RUN_TEST(negTest_adcbufrRadStaticRegs, 11203, NULL);
	RUN_TEST(negTest_adcbufVerifyDataFormatCfg, 11204, NULL);
}
