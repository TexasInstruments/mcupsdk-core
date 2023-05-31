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

/* Test definition specific to the device */
#define TEST_ADCBUF_TOTAL_TEST_CASES        (4U)
#define TEST_ADCBUF_NUM_SAMPLES             (1024U)
#define TEST_ADCBUF_DMADATA_BASE_ADDRESS    (CSL_DSS_L3_U_BASE + 0x8000U)

#if defined(_TMS320C6X)
#define RSS_ADC_CAPTURE_COMPLETE_IRQ        (57U)
#else
#define RSS_ADC_CAPTURE_COMPLETE_IRQ        (142U)
#endif
/* ========================================================================== */
/*                         Structures and Enums                               */
/* ========================================================================== */

typedef struct ADCBufTestParams_t
{
    /* Number of ADC samples */
    uint32_t    numSamples;
    /* Device specific ADCBUF memory address */
    uint32_t    memAddr;
    /* Memory address to store ADC samples */
    uint32_t    dstMemAddr;
    /* Chirp Interrupt Number */
    uint32_t    chirpIntNumber;
} ADCBufTestParams;


/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */

/* Device specific test parameters */
static ADCBufTestParams gADCBufTestParam =
{
    TEST_ADCBUF_NUM_SAMPLES,
    CSL_RSS_ADCBUF_READ_U_BASE,
    TEST_ADCBUF_DMADATA_BASE_ADDRESS,
    RSS_ADC_CAPTURE_COMPLETE_IRQ
};

/* Test cases with different data format */
static ADCBuf_dataFormat gADCBufDataFmtTestCase[TEST_ADCBUF_TOTAL_TEST_CASES] =
{
    {0, 0, 0},      /* Complex, Q+I, non-interleaved */
    {0, 1, 0},      /* Complex, I+Q, non-interleaved */
    {1, 0, 0},      /* Real, I, non-interleaved */
    {1, 1, 0},      /* Real, Q, non-interleaved */
};


/* ========================================================================== */
/*                 Internal Function Declarations                             */
/* ========================================================================== */


/* Positive test for ADCBuf_open API */
void posTest_adcbuf_open(void *args)
{
    int32_t    		testStatus  = SystemP_SUCCESS;
	ADCBuf_Params   *params		= NULL;
	ADCBuf_Handle   handle;

	if (testStatus == SystemP_SUCCESS)
    {
		handle = ADCBuf_open(CONFIG_ADCBUF0, params);
        if( handle == NULL)
        {
            DebugP_log("adcbuf_pos_Test: failure on line no. %d \n", __LINE__);
            testStatus = SystemP_FAILURE;
        }
		ADCBuf_close(handle);
    }
    TEST_ASSERT_EQUAL_INT32(testStatus,SystemP_SUCCESS);
}

/* Positive test for ADCBuf_Params_init API */
void posTest_adcbuf_Params_init(void *args)
{
    int32_t        testStatus 		= SystemP_SUCCESS;
	ADCBuf_Params  *paramsPointer 	= NULL;
	ADCBuf_Params  params;
	

	if (testStatus == SystemP_SUCCESS)
    {
		ADCBuf_Params_init(paramsPointer);
        if(paramsPointer != NULL)
        {
            DebugP_log("adcbuf_pos_Test: failure on line no. %d \n", __LINE__);
            testStatus = SystemP_FAILURE;
        }
    }
	
	if (testStatus == SystemP_SUCCESS)
    {
		ADCBuf_Params_init(&params);
        if(params.continousMode != 0)
        {
            DebugP_log("adcbuf_pos_Test: failure on line no. %d \n", __LINE__);
            testStatus = SystemP_FAILURE;
        }
    }

    TEST_ASSERT_EQUAL_INT32(testStatus,SystemP_SUCCESS);
}

/* Positive test for ADCBuf_control API */
void posTest_adcbuf_control(void *args)
{
    int32_t    				testStatus     	= SystemP_SUCCESS;
	uint32_t            	arg;
	ADCBuf_Handle       	handle;
    ADCBufMMWave_CMD    	command;
	ADCBuf_Params       	params;
    int32_t             	retVal;
    ADCBuf_dataFormat   	dataFormat;
    ADCBuf_RxChanConf   	rxChanConf;
    ADCBuf_CQConf       	cqConf;
    ADCBuf_dataFormat   	*ptrDataFormat 	= &dataFormat;
	uint32_t            	numSamples;
	ADCBuf_TestPatternConf  testPatternConf;
	uint32_t                numOfClks 		= 0x32;
	uint32_t                rxChanMask 		= 0xF;
	
	
	handle         = (ADCBuf_Handle) NULL;
	command        = ADCBufMMWave_CMD_START_CONTINUOUS_MODE;
	arg            = 0x5U;
	if (testStatus == SystemP_SUCCESS)
    {
        if(ADCBuf_control(handle, command, (void *)&arg) != ADCBUF_STATUS_INVALID_PARAMS)
        {
            DebugP_log("adcbuf_pos_Test: failure on line no. %d \n", __LINE__);
            testStatus = SystemP_FAILURE;
        }
    }
	
	ADCBuf_Params_init(&params);
    handle = ADCBuf_open(CONFIG_ADCBUF0, &params);
	if (testStatus == SystemP_SUCCESS)
    {
		if(handle == NULL)
		{
			DebugP_log("adcbuf_pos_Test: failure on line no. %d \n", __LINE__);
			testStatus = SystemP_FAILURE;
		}
	}

    /* Params check Test */
    command = ADCBufMMWave_CMD_SET_SRC;
    arg = 0x5;
	if (testStatus == SystemP_SUCCESS)
    {
		if((retVal = ADCBuf_control(handle, command, (void *)&arg)) >= 0)
		{
			DebugP_log("adcbuf_pos_Test: failure on line no. %d \n", __LINE__);
			testStatus = SystemP_FAILURE;
		}
	}

    command = ADCBufMMWave_CMD_SET_CONTINUOUS_MODE;
    arg = 0x8;
	if (testStatus == SystemP_SUCCESS)
    {
		if((retVal = ADCBuf_control(handle, command, (void *)&arg)) >= 0)
		{
			DebugP_log("adcbuf_pos_Test: failure on line no. %d \n", __LINE__);
			testStatus = SystemP_FAILURE;
		}
	}

    command = ADCBufMMWave_CMD_CONF_DATA_FORMAT;
    memset((void *)&dataFormat, 0, sizeof(dataFormat));
    dataFormat.adcOutFormat = 2;
	if (testStatus == SystemP_SUCCESS)
    {
		if((retVal = ADCBuf_control(handle, command, (void *)&dataFormat)) >= 0)
		{
			DebugP_log("adcbuf_pos_Test: failure on line no. %d \n", __LINE__);
			testStatus = SystemP_FAILURE;
		}
	}
	
    memset((void *)&dataFormat, 0, sizeof(dataFormat));
    dataFormat.sampleInterleave = 2;
	if (testStatus == SystemP_SUCCESS)
    {
		if((retVal = ADCBuf_control(handle, command, (void *)&dataFormat)) >= 0)
		{
			DebugP_log("adcbuf_pos_Test: failure on line no. %d \n", __LINE__);
			testStatus = SystemP_FAILURE;
		}
	}


    memset((void *)&dataFormat, 0, sizeof(dataFormat));
    dataFormat.channelInterleave = 2;
	if (testStatus == SystemP_SUCCESS)
    {
		if((retVal = ADCBuf_control(handle, command, (void *)&dataFormat)) >= 0)
		{
			DebugP_log("adcbuf_pos_Test: failure on line no. %d \n", __LINE__);
			testStatus = SystemP_FAILURE;
		}
	}


    command = ADCBufMMWave_CMD_CONF_CQ;
    memset((void *)&cqConf, 0, sizeof(cqConf));
    cqConf.cqDataWidth= 5;
	if (testStatus == SystemP_SUCCESS)
    {
		if((retVal = ADCBuf_control(handle, command, (void *)&cqConf)) >= 0)
		{
			DebugP_log("adcbuf_pos_Test: failure on line no. %d \n", __LINE__);
			testStatus = SystemP_FAILURE;
		}
	}


    command = ADCBufMMWave_CMD_CONF_CQ;
    memset((void *)&cqConf, 0, sizeof(cqConf));
    cqConf.cq96BitPackEn = 4;
	if (testStatus == SystemP_SUCCESS)
    {
		if((retVal = ADCBuf_control(handle, command, (void *)&cqConf)) >= 0)
		{
			DebugP_log("adcbuf_pos_Test: failure on line no. %d \n", __LINE__);
			testStatus = SystemP_FAILURE;
		}
	}
	

    command = ADCBufMMWave_CMD_START_CONTINUOUS_MODE;
    arg = 0x1U<<16;
	if (testStatus == SystemP_SUCCESS)
    {
		if((retVal = ADCBuf_control(handle, command, (void *)&arg)) >= 0)
		{
			DebugP_log("adcbuf_pos_Test: failure on line no. %d \n", __LINE__);
			testStatus = SystemP_FAILURE;
		}
	}


    command = ADCBufMMWave_CMD_CHANNEL_ENABLE;
    memset((void *)&rxChanConf, 0, sizeof(rxChanConf));
    rxChanConf.channel = 6;
	if (testStatus == SystemP_SUCCESS)
    {
		if((retVal = ADCBuf_control(handle, command, (void *)&rxChanConf)) >= 0)
		{
			DebugP_log("adcbuf_pos_Test: failure on line no. %d \n", __LINE__);
			testStatus = SystemP_FAILURE;
		}
	}


    memset((void *)&rxChanConf, 0, sizeof(rxChanConf));
    rxChanConf.channel = 2;
    rxChanConf.offset = 0x1U << 15U;
	if (testStatus == SystemP_SUCCESS)
    {
		if((retVal = ADCBuf_control(handle, command, (void *)&rxChanConf)) >= 0)
		{
			DebugP_log("adcbuf_pos_Test: failure on line no. %d \n", __LINE__);
			testStatus = SystemP_FAILURE;
		}
	}


    command = ADCBufMMWave_CMD_SET_CHIRP_THRESHHOLD;
    arg = 35;
	if (testStatus == SystemP_SUCCESS)
    {
		if((retVal = ADCBuf_control(handle, command, (void *)&arg)) >= 0)
		{
			DebugP_log("adcbuf_pos_Test: failure on line no. %d \n", __LINE__);
			testStatus = SystemP_FAILURE;
		}
	}


    command = ADCBufMMWave_CMD_SET_PING_CHIRP_THRESHHOLD;
    arg = 35;
	if (testStatus == SystemP_SUCCESS)
    {
		if((retVal = ADCBuf_control(handle, command, (void *)&arg)) >= 0)
		{
			DebugP_log("adcbuf_pos_Test: failure on line no. %d \n", __LINE__);
			testStatus = SystemP_FAILURE;
		}
	}


    command = ADCBufMMWave_CMD_SET_PONG_CHIRP_THRESHHOLD;
    arg = 35;
	if (testStatus == SystemP_SUCCESS)
    {
		if((retVal = ADCBuf_control(handle, command, (void *)&arg)) >= 0)
		{
			DebugP_log("adcbuf_pos_Test: failure on line no. %d \n", __LINE__);
			testStatus = SystemP_FAILURE;
		}
	}

    command = ADCBufMMWave_CMD_CHANNEL_DISABLE;
    arg = 0xc;
	if (testStatus == SystemP_SUCCESS)
    {
		if((retVal = ADCBuf_control(handle, command, (void *)&arg)) < 0)
		{
			DebugP_log("adcbuf_pos_Test: failure on line no. %d \n", __LINE__);
			testStatus = SystemP_FAILURE;
		}
	}

    command = ADCBufMMWave_CMD_CHANNEL_DISABLE;
    arg = 0x1c;
	if (testStatus == SystemP_SUCCESS)
    {
		if((retVal = ADCBuf_control(handle, command, (void *)&arg)) >= 0)
		{
			DebugP_log("adcbuf_pos_Test: failure on line no. %d \n", __LINE__);
			testStatus = SystemP_FAILURE;
		}
	}

    command = ADCBufMMWave_CMD_CHANNEL_DISABLE;
    arg = 0x0;
	if (testStatus == SystemP_SUCCESS)
    {
		if((retVal = ADCBuf_control(handle, command, (void *)&arg)) >= 0)
		{
			DebugP_log("adcbuf_pos_Test: failure on line no. %d \n", __LINE__);
			testStatus = SystemP_FAILURE;
		}
	}
	
	/* Configure ADC buffer in continuous mode */
    arg = 1;
	if (testStatus == SystemP_SUCCESS)
    {
		if((retVal = ADCBuf_control(handle, ADCBufMMWave_CMD_SET_CONTINUOUS_MODE, (void *)&arg)) < 0)
		{
			DebugP_log("adcbuf_pos_Test: failure on line no. %d \n", __LINE__);
			testStatus = SystemP_FAILURE;
		}
	}

    /* Start the continuous streaming mode in ADCBUFF */
    numSamples = gADCBufTestParam.numSamples;
	if (testStatus == SystemP_SUCCESS)
    {
		if((retVal = ADCBuf_control(handle, ADCBufMMWave_CMD_START_CONTINUOUS_MODE, (void *)&numSamples)) < 0)
		{
			DebugP_log("adcbuf_pos_Test: failure on line no. %d \n", __LINE__);
			testStatus = SystemP_FAILURE;
		}
	}

	/* Configure ADC buffer data format */
	dataFormat = gADCBufDataFmtTestCase[0];	
	if (testStatus == SystemP_SUCCESS)
    {
		if((retVal = ADCBuf_control(handle, ADCBufMMWave_CMD_CONF_DATA_FORMAT, (void *)ptrDataFormat)) < 0)
		{
			DebugP_log("adcbuf_pos_Test: failure on line no. %d \n", __LINE__);
			testStatus = SystemP_FAILURE;
		}
	}
	
	/* Configure Test Pattern generation */
    testPatternConf.period = 255;
    testPatternConf.numSamples = gADCBufTestParam.numSamples;
	testPatternConf.rxConfig[0].rxIOffset   = 0x0000;
	testPatternConf.rxConfig[0].rxIInc      = 2;
	testPatternConf.rxConfig[0].rxQOffset   = 0x1000 ;
	testPatternConf.rxConfig[0].rxQInc      = 2;
    

    /* Send control command to driver */
	if (testStatus == SystemP_SUCCESS)
    {
		if((retVal = ADCBuf_control(handle, ADCBufMMWave_CMD_CONF_TEST_PATTERN, (void *)&testPatternConf)) < 0)
		{
			DebugP_log("Error: ADCBufMMWave_CMD_CONF_TEST_PATTERN failed with [Error=%d]\r\n", retVal);
			TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, retVal);
		}
	}
	
    /* Start Test Pattern generation */
	if (testStatus == SystemP_SUCCESS)
    {
		if((retVal = ADCBuf_control(handle, ADCBufMMWave_CMD_START_TEST_PATTERN, (void *)&numOfClks)) < 0)
		{
			DebugP_log("Error: ADCBufMMWave_CMD_START_TEST_PATTERN failed with [Error=%d]\r\n", retVal);
			TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, retVal);
		}
	}
	
	command = ADCBufMMWave_CMD_CONF_DATA_FORMAT;
    memset((void *)&dataFormat, 0, sizeof(dataFormat));
    dataFormat.adcOutFormat = 1;
	if (testStatus == SystemP_SUCCESS)
    {
		if((retVal = ADCBuf_control(handle, command, (void *)&dataFormat)) < 0)
		{
			DebugP_log("adcbuf_pos_Test: failure on line no. %d \n", __LINE__);
			testStatus = SystemP_FAILURE;
		}
	}
	
	if (testStatus == SystemP_SUCCESS)
    {
		if((retVal = ADCBuf_control(handle, ADCBufMMWave_CMD_CHANNEL_DISABLE, (void *)&rxChanMask)) < 0)
		{
			DebugP_log("adcbuf_pos_Test: failure on line no. %d \n", __LINE__);
			testStatus = SystemP_FAILURE;
		}
	}
	
	/* StopTest Pattern generation */
	if (testStatus == SystemP_SUCCESS)
    {
		if((retVal = ADCBuf_control(handle, ADCBufMMWave_CMD_STOP_TEST_PATTERN, NULL)) < 0)
		{
			DebugP_log("adcbuf_pos_Test: failure on line no. %d \n", __LINE__);
			testStatus = SystemP_FAILURE;
		}
	}
		
    /* Close ADCbuf driver */
    ADCBuf_close(handle);
    TEST_ASSERT_EQUAL_INT32(testStatus,SystemP_SUCCESS);
}
	
/* Positive test for ADCBuf_getChanBufAddr API */
void posTest_adcbuf_getChanBufAddr(void *args)
{
    int32_t    			testStatus = SystemP_SUCCESS;
	ADCBuf_Handle       handle;
	ADCBuf_Params       params;
	uint8_t            	channel= 0;
    int32_t             retVal = 0;
	ADCBuf_RxChanConf   rxChanConf;
	uint32_t channelAddr 			= 0;
	uint16_t offset 				= 0;
	uint32_t     rxChanMask         = 0x01;
	
	
	ADCBuf_Params_init(&params);
    handle = ADCBuf_open(CONFIG_ADCBUF0, &params);
	
	if (testStatus == SystemP_SUCCESS)
    {
		rxChanConf.channel   = 0;
		rxChanConf.offset    = 0;
		ADCBuf_control(handle, ADCBufMMWave_CMD_CHANNEL_DISABLE, (void *)&rxChanMask);      
		ADCBuf_getChanBufAddr(handle, channel, &retVal);
		if(retVal != ADCBUF_STATUS_INVALID_PARAMS)
		{
			DebugP_log("adcbuf_pos_Test: failure on line no. %d \n", __LINE__);
			testStatus = SystemP_FAILURE;
		}
	}
	
			
	ADCBuf_close(handle);
	
	ADCBuf_Params_init(&params);
    handle = ADCBuf_open(CONFIG_ADCBUF0, &params);
	
	
	if (testStatus == SystemP_SUCCESS)
    {
		memset((void *)&rxChanConf, 0, sizeof(rxChanConf));
		rxChanConf.channel = 0;
		rxChanConf.offset  = 0;
		ADCBuf_control(handle, ADCBufMMWave_CMD_CHANNEL_ENABLE, (void *)&rxChanConf);		
		ADCBuf_getChanBufAddr(handle, channel, &retVal);

        if( retVal != SystemP_SUCCESS)
        {
            DebugP_log("adcbuf_pos_Test: failure on line no. %d \n", __LINE__);
            testStatus = SystemP_FAILURE;
        }
    }
	
	if (testStatus == SystemP_SUCCESS)
    {
	
		for(channel=0; channel < SOC_ADCBUF_NUM_RX_CHANNEL; channel++)
		{
			rxChanConf.channel   = channel;
			rxChanConf.offset    = offset;
			if((retVal = ADCBuf_control(handle, ADCBufMMWave_CMD_CHANNEL_ENABLE, (void *)&rxChanConf)) < 0)
			{
				DebugP_log("adcbuf_pos_Test: failure on line no. %d \n", __LINE__);
				testStatus = SystemP_FAILURE;
			}
			/* Verify channel address */
			{            

				if((channelAddr = ADCBuf_getChanBufAddr(handle, channel, &retVal)) != 0)
				{
					channelAddr -= gADCBufTestParam.memAddr;
					if(channelAddr != offset)
					{
						DebugP_log("Error: ADCBuf_getChanBufAddr() return mismatched channel(%d) buffer address [%x: %x]\r\n",
									 channel, offset, channelAddr);

						retVal = -1;
						DebugP_log("adcbuf_pos_Test: failure on line no. %d \n", __LINE__);
						testStatus = SystemP_FAILURE;
					}
				}
				else
				{
					DebugP_log("Error: ADCBuf_getChanBufAddr failed for channel %d with [Error = %d]\r\n",
											 channel, retVal);
					DebugP_log("adcbuf_pos_Test: failure on line no. %d \n", __LINE__);
					testStatus = SystemP_FAILURE;
				}
			}

			offset += gADCBufTestParam.numSamples * 4;
		}
	}
	ADCBuf_close(handle);
	
	
	channel = 4U; /*invalid*/
	retVal = 0;
	
	if (testStatus == SystemP_SUCCESS)
    {
		ADCBuf_getChanBufAddr(handle, channel, &retVal);
        if( retVal != ADCBUF_STATUS_INVALID_PARAMS)
        {
            DebugP_log("adcbuf_pos_Test: failure on line no. %d \n", __LINE__);
            testStatus = SystemP_FAILURE;
        }			
    }
		
	channel = SOC_ADCBUF_NUM_RX_CHANNEL;
	retVal = 0;
	
	if (testStatus == SystemP_SUCCESS)
    {
		ADCBuf_getChanBufAddr(handle, channel, &retVal);
        if( retVal != ADCBUF_STATUS_INVALID_PARAMS)
        {
            DebugP_log("adcbuf_pos_Test: failure on line no. %d \n", __LINE__);
            testStatus = SystemP_FAILURE;
        }
    }
	
	handle = (ADCBuf_Handle)NULL;
	channel = 0;
	retVal = 0;
	
	if (testStatus == SystemP_SUCCESS)
    {
		ADCBuf_getChanBufAddr(handle, channel, &retVal);
        if( retVal != ADCBUF_STATUS_INVALID_PARAMS)
        {
            DebugP_log("adcbuf_pos_Test: failure on line no. %d \n", __LINE__);
            testStatus = SystemP_FAILURE;
        }
    }
    TEST_ASSERT_EQUAL_INT32(testStatus,SystemP_SUCCESS);
}

/* Positive test for ADCBUF_MMWave_open API */
void posTest_adcbuf_mmwave_open(void *args)
{
    int32_t    				testStatus = SystemP_SUCCESS;
	ADCBuf_Params        	params;
	
	
    ADCBuf_Params_init(&params);
	
	params.continousMode =(uint32_t)0x1U << 1U;
    params.chirpThresholdPing =0U;
    params.chirpThresholdPong =0U;	
	if (testStatus == SystemP_SUCCESS)
    {
        if(ADCBuf_open(CONFIG_ADCBUF0, &params) != (ADCBuf_Handle)NULL)
        {
            DebugP_log("adcbuf_pos_Test: failure on line no. %d \n", __LINE__);
            testStatus = SystemP_FAILURE;
        }
    }
    TEST_ASSERT_EQUAL_INT32(testStatus,SystemP_SUCCESS);
}


/* Positive test for ADCBUF_MMWave_getCQBufAddr API */
void posTest_adcbufmmwave_get_cqbufferaddr(void *args)
{
    int32_t  			testStatus 	= SystemP_SUCCESS;
	ADCBuf_Handle       handle 		= (ADCBuf_Handle)NULL;
	int32_t  			retVal 		= 	0;

	if (testStatus == SystemP_SUCCESS)
    {
		ADCBUF_MMWave_getCQBufAddr(handle, ADCBufMMWave_CQType_CQ0, &retVal);
        if(retVal != ADCBUF_STATUS_INVALID_PARAMS)
        {
            DebugP_log("adcbuf_pos_Test: failure on line no. %d \n", __LINE__);
            testStatus = SystemP_FAILURE;
        }
    }
    TEST_ASSERT_EQUAL_INT32(testStatus,SystemP_SUCCESS);
}

/* Positive test for ADCBUFCmdParamCheck API */
void posTest_adcbufCmdParamCheck(void *args)
{
    int32_t    				testStatus = SystemP_SUCCESS;
    uint32_t            	arg;
	ADCBuf_Params       	params;
	ADCBuf_Handle        	handle;
	ADCBufMMWave_CMD    	command;
	ADCBuf_dataFormat  		dataFormat;
	ADCBuf_CQConf      		cqConf;
	ADCBuf_RxChanConf  		rxChanConf;
	
	ADCBuf_Params_init(&params);
    handle = ADCBuf_open(CONFIG_ADCBUF0, &params);
	
	command        = ADCBufMMWave_CMD_START_TEST_PATTERN;
	arg            = 0xFFFU;

	if (testStatus == SystemP_SUCCESS)
    {
        if(ADCBuf_control(handle, command, (void *)&arg) != ADCBUF_STATUS_INVALID_PARAMS)
        {
            DebugP_log("adcbuf_pos_Test: failure on line no. %d \n", __LINE__);
            testStatus = SystemP_FAILURE;
        }
    }
	ADCBuf_close(handle);
    TEST_ASSERT_EQUAL_INT32(testStatus,SystemP_SUCCESS);

	
	command        					= ADCBufMMWave_CMD_CONF_DATA_FORMAT;
    dataFormat.adcOutFormat 		= (uint8_t)(0x1U << 1U) ;
	
	if (testStatus == SystemP_SUCCESS)
    {
        if(ADCBuf_control(handle,command,(void *)&dataFormat) != ADCBUF_STATUS_INVALID_PARAMS)
        {
            DebugP_log("adcbuf_pos_Test: failure on line no. %d \n", __LINE__);
            testStatus = SystemP_FAILURE;
        }
    }

	dataFormat.channelInterleave 	= (uint8_t)(0x1U << 1U);
		if (testStatus == SystemP_SUCCESS)
    {
        if(ADCBuf_control(handle,command,(void *)&dataFormat) != ADCBUF_STATUS_INVALID_PARAMS)
        {
            DebugP_log("adcbuf_pos_Test: failure on line no. %d \n", __LINE__);
            testStatus = SystemP_FAILURE;
        }
    }
	
    dataFormat.sampleInterleave 	= (uint8_t)(0x1U << 1U);
		if (testStatus == SystemP_SUCCESS)
    {
        if(ADCBuf_control(handle,command,(void *)&dataFormat) != ADCBUF_STATUS_INVALID_PARAMS)
        {
            DebugP_log("adcbuf_pos_Test: failure on line no. %d \n", __LINE__);
            testStatus = SystemP_FAILURE;
        }
    }
	
	command   = ADCBufMMWave_CMD_CONF_CQ;
	cqConf.cqDataWidth 			= (uint8_t)((0x1U<<2U) +1U);
    cqConf.cq96BitPackEn 			= (uint8_t)((0x1U<<1U) +1U);
	if (testStatus == SystemP_SUCCESS)
    {
        if(ADCBuf_control(handle,command,(void *)&cqConf) != ADCBUF_STATUS_INVALID_PARAMS)
        {
            DebugP_log("adcbuf_pos_Test: failure on line no. %d \n", __LINE__);
            testStatus = SystemP_FAILURE;
        }
    }
	
	cqConf.cqDataWidth 			= (uint8_t)((0x1U<<2U));
    cqConf.cq96BitPackEn 			= (uint8_t)((0x1U<<1U) +1U);
	if (testStatus == SystemP_SUCCESS)
    {
        if(ADCBuf_control(handle,command,(void *)&cqConf) != ADCBUF_STATUS_INVALID_PARAMS)
        {
            DebugP_log("adcbuf_pos_Test: failure on line no. %d \n", __LINE__);
            testStatus = SystemP_FAILURE;
        }
    }
	
	cqConf.cqDataWidth 			= (uint8_t)((0x1U<<2U) +1);
    cqConf.cq96BitPackEn 			= (uint8_t)((0x1U<<1U));
	if (testStatus == SystemP_SUCCESS)
    {
        if(ADCBuf_control(handle,command,(void *)&cqConf) != ADCBUF_STATUS_INVALID_PARAMS)
        {
            DebugP_log("adcbuf_pos_Test: failure on line no. %d \n", __LINE__);
            testStatus = SystemP_FAILURE;
        }
    }
	
	cqConf.cqDataWidth 			= (uint8_t)((0x1U<<2U) -1U);
    cqConf.cq96BitPackEn 			= (uint8_t)((0x1U<<1U) -1U);
	if (testStatus == SystemP_SUCCESS)
    {
        if(ADCBuf_control(handle,command,(void *)&cqConf) == ADCBUF_STATUS_INVALID_PARAMS)
        {
            DebugP_log("adcbuf_pos_Test: failure on line no. %d \n", __LINE__);
            testStatus = SystemP_FAILURE;
        }
    }
	
	command = ADCBufMMWave_CMD_CHANNEL_ENABLE;
	rxChanConf.channel = SOC_ADCBUF_NUM_RX_CHANNEL;
	rxChanConf.offset  = 0U;	
	
	if (testStatus == SystemP_SUCCESS)
    {
        if(ADCBuf_control(handle,command,(void *)&rxChanConf) != ADCBUF_STATUS_INVALID_PARAMS)
        {
            DebugP_log("adcbuf_pos_Test: failure on line no. %d \n", __LINE__);
            testStatus = SystemP_FAILURE;
        }
    }
	
	uint32_t        *paramArg = NULL;
	if (testStatus == SystemP_SUCCESS)
    {
        if(ADCBuf_control(handle, command, (void *) paramArg) != ADCBUF_STATUS_INVALID_PARAMS)
        {
            DebugP_log("adcbuf_pos_Test: failure on line no. %d \n", __LINE__);
            testStatus = SystemP_FAILURE;
        }
    }
	
    TEST_ASSERT_EQUAL_INT32(testStatus,SystemP_SUCCESS);
}

/* Positive test for ADCBUF_verifySrcSelCfg API */
void posTest_adcbufVerifySrcSelCfg(void *args)
{
	int32_t    				testStatus = SystemP_SUCCESS;
	uint32_t            	source;
	ADCBuf_Handle        	handle;
	
	handle         = (ADCBuf_Handle) NULL;
	source         = 3U;
	if (testStatus == SystemP_SUCCESS)
    {
        if(ADCBUF_verifySrcSelCfg(handle,source) != ADCBUF_STATUS_INVALID_PARAMS)
        {
            DebugP_log("adcbuf_pos_Test: failure on line no. %d \n", __LINE__);
            testStatus = SystemP_FAILURE;
        }
    }
   TEST_ASSERT_EQUAL_INT32(testStatus,SystemP_SUCCESS);
	
}

/* Positive test for ADCBUF_verifyChirpThreshold API */
void posTest_adcbufVerifyChirpThreshold(void *args)
{
	int32_t    				testStatus = SystemP_SUCCESS;
	uint32_t            	pingThreshCfg, pongThreshCfg;
	ADCBuf_Handle        	handle;
	ADCBuf_Params       	params;
	
	ADCBuf_Params_init(&params);
    handle = ADCBuf_open(CONFIG_ADCBUF0, &params);
	
	pongThreshCfg         = 1U;
	pingThreshCfg		  = 1U;
	if (testStatus == SystemP_SUCCESS)
    {
        if(ADCBUF_verifyChirpThreshold(handle,pingThreshCfg,pongThreshCfg) != SystemP_SUCCESS)
        {
            DebugP_log("adcbuf_pos_Test: failure on line no. %d \n", __LINE__);
            testStatus = SystemP_FAILURE;
        }
    }
	ADCBuf_close(handle);
   TEST_ASSERT_EQUAL_INT32(testStatus,SystemP_SUCCESS);
	
}

/* Positive test for ADCBUF_verifyContinuousModeCfg API */
void posTest_adcbufVerifyContinuousModeCfg(void *args)
{
	int32_t    				testStatus = SystemP_SUCCESS;
	uint32_t            	continuousModeCfg;
	ADCBuf_Handle        	handle;
	ADCBuf_Params       	params;
	
	ADCBuf_Params_init(&params);
    handle = ADCBuf_open(CONFIG_ADCBUF0, &params);
	
	continuousModeCfg         = 0U;
	if (testStatus == SystemP_SUCCESS)
    {
        if(ADCBUF_verifyContinuousModeCfg(handle,continuousModeCfg) != SystemP_SUCCESS)
        {
            DebugP_log("adcbuf_pos_Test: failure on line no. %d \n", __LINE__);
            testStatus = SystemP_FAILURE;
        }
    }
	ADCBuf_close(handle);
   TEST_ASSERT_EQUAL_INT32(testStatus,SystemP_SUCCESS);
	
}

/* Positive test for ADCBUF_readStaticRegs API */
void posTest_adcbufrRadStaticRegs(void *args)
{
	int32_t    				testStatus = SystemP_SUCCESS;
	ADCBuf_Handle        	handle;
	ADCBuf_Params       	params;
	ADCBUF_StaticRegs   dummyStaticRegs;
	ADCBUF_StaticRegs   *pStaticRegs = &dummyStaticRegs;
	
	ADCBuf_Params_init(&params);
    handle = ADCBuf_open(CONFIG_ADCBUF0, &params);
	
	if (testStatus == SystemP_SUCCESS)
    {
        if(ADCBUF_readStaticRegs(handle,pStaticRegs) != SystemP_SUCCESS)
        {
            DebugP_log("adcbuf_pos_Test: failure on line no. %d \n", __LINE__);
            testStatus = SystemP_FAILURE;
        }
    }
	ADCBuf_close(handle);
   TEST_ASSERT_EQUAL_INT32(testStatus,SystemP_SUCCESS);
	
}
								
/* Positive test for ADCBUF_verifyDataFormatCfg API */
void posTest_adcbufVerifyDataFormatCfg(void *args)
{
	int32_t    				testStatus = SystemP_SUCCESS;
	ADCBuf_Handle        	handle;
	ADCBuf_Params       	params;
	uint32_t 				dataFormatcfg;
	uint32_t 				interleavecfg;
	uint32_t 				iqConfig;
	
	ADCBuf_Params_init(&params);
    handle = ADCBuf_open(CONFIG_ADCBUF0, &params);
	
	dataFormatcfg = 1U;
	interleavecfg = 0U;
	iqConfig	  = 0U;
	
	if (testStatus == SystemP_SUCCESS)
    {
        if(ADCBUF_verifyDataFormatCfg(handle,dataFormatcfg,interleavecfg,iqConfig) != SystemP_SUCCESS)
        {
            DebugP_log("adcbuf_pos_Test: failure on line no. %d \n", __LINE__);
            testStatus = SystemP_FAILURE;
        }
    }
	ADCBuf_close(handle);
   TEST_ASSERT_EQUAL_INT32(testStatus,SystemP_SUCCESS);
	
}
void test_pos_main(void *args)
{
	RUN_TEST(posTest_adcbuf_open, 10883, NULL); 
	RUN_TEST(posTest_adcbuf_Params_init, 10884, NULL); 
	RUN_TEST(posTest_adcbuf_control, 10885, NULL); 
	RUN_TEST(posTest_adcbuf_getChanBufAddr, 10886, NULL); 
	RUN_TEST(posTest_adcbuf_mmwave_open, 10887, NULL); 
	RUN_TEST(posTest_adcbufmmwave_get_cqbufferaddr, 10889, NULL);
	RUN_TEST(posTest_adcbufCmdParamCheck, 10888, NULL); 
	RUN_TEST(posTest_adcbufVerifySrcSelCfg, 11197, NULL); 
	RUN_TEST(posTest_adcbufVerifyChirpThreshold, 11198, NULL);
	RUN_TEST(posTest_adcbufVerifyContinuousModeCfg, 11201, NULL);
	RUN_TEST(posTest_adcbufrRadStaticRegs, 11202, NULL);
	RUN_TEST(posTest_adcbufVerifyDataFormatCfg, 11206, NULL);	

}
