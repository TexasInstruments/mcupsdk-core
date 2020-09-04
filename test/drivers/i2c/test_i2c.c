/*
 * Copyright (C) 2021 Texas Instruments Incorporated
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *   Redistributions of source code must retain the above copyright
 *   notice, this list of conditions and the following disclaimer.
 *
 *   Redistributions in binary form must reproduce the above copyright
 *   notice, this list of conditions and the following disclaimer in the
 *   documentation and/or other materials provided with the
 *   distribution.
 *
 *   Neither the name of Texas Instruments Incorporated nor the names of
 *   its contributors may be used to endorse or promote products derived
 *   from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <string.h>
#include <stdlib.h>
#include <inttypes.h>
#include <unity.h>
#include <drivers/i2c.h>
#include <kernel/dpl/DebugP.h>
#include <kernel/dpl/SemaphoreP.h>
#include "ti_drivers_config.h"
#include "ti_drivers_open_close.h"

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

#define NON_EXISTENT_DEVICE_ADDRESS (0x01U) 
#define APP_I2C_BUFSIZE (300U)

extern I2C_Config gI2cConfig[];

/* ========================================================================== */
/*                         Structures and Enums                               */
/* ========================================================================== */

typedef struct I2C_ProbeSettings_s {

    uint8_t instance;
    uint8_t numActualAddresses;

} I2C_ProbeSettings;

typedef struct I2C_TestParams_s {

    I2C_Params  i2cParams;
    uint16_t memAddress;
    uint8_t deviceAddress;
    uint8_t numBytes;
    uint8_t numWritesReads;
    bool testSetFrequency;
    bool intrEnable;

} I2C_TestParams;

/* ========================================================================== */
/*                 Internal Function Declarations                             */
/* ========================================================================== */

/* Testcases */
static void test_i2c_write_read(void* args);
static void test_i2c_probe(void* args);
static void test_i2c_callback_mode(void* args);
static void test_i2c_timeout(void* args);
static void test_i2c_open_close(void* args);

/* Helpers */
static void test_i2c_set_test_params(I2C_TestParams *testParams, int8_t setting_id);
static void test_i2c_callback(I2C_Handle i2cHnd, I2C_Transaction * msg, int32_t transferStatus);

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */

SemaphoreP_Object gTestI2cCallbackDoneSemObj;

uint8_t gI2cTxBuffer[APP_I2C_BUFSIZE];
uint8_t gI2cRxBuffer[APP_I2C_BUFSIZE];

/* ========================================================================== */
/*                            Global Functions                                */
/* ========================================================================== */

uint8_t Board_i2cGetEepromDeviceAddr();
uint16_t Board_i2cGetEepromMemAddr();
uint8_t Board_i2cGetEepromAddrSize();


/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

void test_main(void *args)
{
    I2C_TestParams      testParams;
    I2C_ProbeSettings   probeSettings;
    uint8_t             i;

    Drivers_open();

    UNITY_BEGIN();

    test_i2c_set_test_params(&testParams, 0);
    RUN_TEST(test_i2c_write_read, 261, (void*)&testParams);

    test_i2c_set_test_params(&testParams, 1);
    RUN_TEST(test_i2c_write_read, 262, (void*)&testParams);

    test_i2c_set_test_params(&testParams, 2);
    RUN_TEST(test_i2c_write_read, 263, (void*)&testParams);

    test_i2c_set_test_params(&testParams, 3);
    RUN_TEST(test_i2c_write_read, 264, (void*)&testParams);

    test_i2c_set_test_params(&testParams, 4);
    RUN_TEST(test_i2c_write_read, 265, (void*)&testParams);

    for (i=0; i<CONFIG_I2C_NUM_INSTANCES; i++)
    {
        probeSettings.instance = i;
        RUN_TEST(test_i2c_probe, 266 + i, (void*)&probeSettings);

    }
    RUN_TEST(test_i2c_callback_mode, 269, NULL);
    RUN_TEST(test_i2c_open_close, 270, NULL);
    RUN_TEST(test_i2c_timeout, 271, NULL);

    /* Polling mode test */
    test_i2c_set_test_params(&testParams, 5);
    RUN_TEST(test_i2c_write_read, 345, (void*)&testParams);

    UNITY_END();

     Drivers_close();
    return;
}

/*
 * Unity framework required functions
 */
void setUp(void)
{
}

void tearDown(void)
{
}

/*
 * Testcases
 */

/*
 * Test to write bytes to the EEPROM and read it back
 */
static void test_i2c_write_read(void* args)
{
    I2C_TestParams *testParams = (I2C_TestParams*)args;
    I2C_Params     *i2cParams = &(testParams->i2cParams);
    uint32_t        i;
    uint32_t        loopCount;
    int32_t         status;
    I2C_Handle      i2cHandle;
    I2C_Transaction i2cTransaction;
    I2C_HwAttrs  *hwAttrs = NULL;

    I2C_close(gI2cHandle[CONFIG_I2C0]);

    /* Disable interrupt registration in case of polling */
    if (testParams->intrEnable == false)
    {
        hwAttrs = (I2C_HwAttrs *) (gI2cConfig[CONFIG_I2C0]).hwAttrs;
        hwAttrs->enableIntr = FALSE;
    }
   
    
    i2cHandle = I2C_open(CONFIG_I2C0, i2cParams);
    TEST_ASSERT_NOT_NULL(i2cHandle);

    if (testParams->testSetFrequency)
    {
        status = I2C_setBusFrequency(i2cHandle, I2C_400KHZ);
        TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, status);
    }


    for (loopCount=0; loopCount<testParams->numWritesReads; loopCount++)
    {
        for (i=0; i<testParams->numBytes; i++)    /* Data Bytes */
        {
            gI2cTxBuffer[i+Board_i2cGetEepromAddrSize()] = i;
            gI2cRxBuffer[i] = 0;
        }

        /* Writing to EEPROM */
        I2C_Transaction_init(&i2cTransaction);
        i2cTransaction.writeBuf   = gI2cTxBuffer;
        i2cTransaction.writeCount = Board_i2cGetEepromAddrSize() + testParams->numBytes;
        i2cTransaction.slaveAddress = testParams->deviceAddress;
		
        if(Board_i2cGetEepromAddrSize()==0x01U)
        {
			gI2cTxBuffer[0] = (uint8_t)(testParams->memAddress);	/* Address Byte */
        }
        else
        {
			gI2cTxBuffer[0] = (uint8_t)(testParams->memAddress >> 8);     /* Address Byte 1 */
            gI2cTxBuffer[1] = (uint8_t)(testParams->memAddress & 0x00FF); /* Address Byte 2 */
        }
		
		/* To ensure that eeprom is ready, added delay of 4ms */
		ClockP_usleep(4000);
        status = I2C_transfer(i2cHandle, &i2cTransaction);
        TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, status);

        /* After write operation flash will not respond for write cycle time.
         * This is approximately 4ms (min). */
        ClockP_usleep(4000);

        /* wait for write to finish */
        /* Dummy write to set the address to be read from */
        I2C_Transaction_init(&i2cTransaction);
        i2cTransaction.writeBuf   = gI2cTxBuffer;
        i2cTransaction.writeCount = Board_i2cGetEepromAddrSize();
        i2cTransaction.readBuf    = gI2cRxBuffer;
        i2cTransaction.readCount  = 1;
        i2cTransaction.slaveAddress = testParams->deviceAddress;

        /* wait for previous write to complete */
        do
        {
            status = I2C_transfer(i2cHandle, &i2cTransaction);
            if(status==I2C_STS_ERR_NO_ACK)
            {
                /* previous write is not yet complete, try again */
            }
        } while(status == I2C_STS_ERR_NO_ACK);
        TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, status);
        ClockP_usleep(4000);
		
        /* Read from EEPROM */
        /* Actual read from the address */
        I2C_Transaction_init(&i2cTransaction);
        i2cTransaction.writeBuf   = gI2cTxBuffer;
        i2cTransaction.writeCount = Board_i2cGetEepromAddrSize();
        i2cTransaction.readBuf   = gI2cRxBuffer;
        i2cTransaction.readCount = testParams->numBytes;
        i2cTransaction.slaveAddress = testParams->deviceAddress;

        status = I2C_transfer(i2cHandle, &i2cTransaction);
        TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, status);
        TEST_ASSERT_EQUAL_UINT8_ARRAY(gI2cTxBuffer+Board_i2cGetEepromAddrSize(),
            gI2cRxBuffer, testParams->numBytes);
    }

    I2C_close(i2cHandle);

    return;
}

/*
 * Test to see if probe returns same addresses as known from schematic
 */
static void test_i2c_probe(void* args)
{
    I2C_ProbeSettings   *probeSettings = (I2C_ProbeSettings*)args;
    I2C_Params          i2cParams;
    I2C_Handle          i2cHandle;
    uint8_t             probeDeviceAddr;
    int32_t             status;

    I2C_close(gI2cHandle[probeSettings->instance]);
    
    I2C_Params_init(&i2cParams);
    i2cHandle = I2C_open(probeSettings->instance, &i2cParams);
    TEST_ASSERT_NOT_NULL(i2cHandle);

    for(probeDeviceAddr = 0; probeDeviceAddr <= 0x7F; probeDeviceAddr++)
    {
        ClockP_usleep(5);
        status = I2C_probe(i2cHandle, probeDeviceAddr);
        if(status == SystemP_SUCCESS)
        {
            DebugP_log("Found device at address 0x%08x\r\n", probeDeviceAddr);
        }
    }
    I2C_close(i2cHandle);
    return;
}

static void test_i2c_callback_mode(void* args)
{
    I2C_Params      i2cParams;
    uint8_t         txBuffer1[3];
    uint8_t         txBuffer2[3];
    int32_t         status;
    I2C_Handle      i2cHandle;
    I2C_Transaction i2cTransaction;
    
    I2C_close(gI2cHandle[CONFIG_I2C0]);
    SemaphoreP_constructBinary(&gTestI2cCallbackDoneSemObj, 0);

    I2C_Params_init(&i2cParams);
    i2cParams.transferMode = I2C_MODE_CALLBACK;
    i2cParams.transferCallbackFxn = &test_i2c_callback;
    i2cHandle = I2C_open(CONFIG_I2C0, &i2cParams);
    TEST_ASSERT_NOT_NULL(i2cHandle);

    /* Writing to EEPROM */
    I2C_Transaction_init(&i2cTransaction);
    i2cTransaction.writeBuf   = txBuffer1;
    i2cTransaction.writeCount = 3;
    i2cTransaction.slaveAddress = Board_i2cGetEepromDeviceAddr();
    i2cTransaction.arg = NULL;
	
    if (Board_i2cGetEepromAddrSize()==0x01U)
    {
		txBuffer1[0] = (uint8_t)(Board_i2cGetEepromMemAddr());    /* Address Byte */
		txBuffer1[1] = 0x12;
    }
    else
    {
		txBuffer1[0] = (uint8_t)(Board_i2cGetEepromMemAddr() >> 8);     /* Address Byte 1 */
        txBuffer1[1] = (uint8_t)(Board_i2cGetEepromMemAddr() & 0x00FF); /* Address Byte 2 */
        txBuffer1[2] = 0x12;
    }

	status = I2C_transfer(i2cHandle, &i2cTransaction);
    TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, status);

    /* Read from EEPROM */
    /* Dummy write to set the address to be read from */
    I2C_Transaction_init(&i2cTransaction);
    i2cTransaction.writeBuf   = txBuffer2;
    i2cTransaction.writeCount = 2;
    i2cTransaction.slaveAddress = Board_i2cGetEepromDeviceAddr();
    i2cTransaction.arg = &gTestI2cCallbackDoneSemObj;
	
    if(Board_i2cGetEepromAddrSize()==0x01)
    {
		txBuffer2[0] = (uint8_t)((Board_i2cGetEepromMemAddr()+1));     /* Address Byte */
        txBuffer2[1] = 0x34;
    }
    else
    {
		txBuffer2[0] = (uint8_t)((Board_i2cGetEepromMemAddr()+1) >> 8);     /* Address Byte 1 */
        txBuffer2[1] = (uint8_t)((Board_i2cGetEepromMemAddr()+2) & 0x00FF); /* Address Byte 2 */
		txBuffer2[2] = 0x34;
    }
	
    status = I2C_transfer(i2cHandle, &i2cTransaction);
    TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, status);

    SemaphoreP_pend(&gTestI2cCallbackDoneSemObj, SystemP_WAIT_FOREVER);

    I2C_close(i2cHandle);

    SemaphoreP_destruct(&gTestI2cCallbackDoneSemObj);
    return;
}

static void test_i2c_timeout(void* args)
{
    I2C_Handle          i2cHandle;
    I2C_Params          params;
    I2C_Transaction     transaction;
    uint8_t             txBuffer;
    int32_t             status;

    I2C_close(gI2cHandle[CONFIG_I2C0]);
    
    I2C_Params_init(&params);
    params.bitRate = I2C_400KHZ;
    i2cHandle = I2C_open(CONFIG_I2C0, &params);

    I2C_Transaction_init(&transaction);
    transaction.timeout = 50;
    transaction.writeBuf   = &txBuffer;
    transaction.writeCount = 1;
    transaction.slaveAddress = NON_EXISTENT_DEVICE_ADDRESS;
    txBuffer = 0xFE;

    status = I2C_transfer(i2cHandle, &transaction);
    TEST_ASSERT_EQUAL_INT32(I2C_STS_ERR_NO_ACK, status);

    I2C_close(i2cHandle);
}

static void test_i2c_open_close(void* args)
{
    I2C_Handle          i2cHandle;
    I2C_Params          params;
    I2C_Transaction     transaction;
    uint8_t             rxBuffer;
    int32_t             status;
    uint8_t             i;

    I2C_close(gI2cHandle[CONFIG_I2C0]);
    
    I2C_Params_init(&params);
    params.bitRate = I2C_400KHZ;

    I2C_Transaction_init(&transaction);
    transaction.readBuf   = &rxBuffer;
    transaction.readCount = 1;
    transaction.slaveAddress = Board_i2cGetEepromDeviceAddr();

    for (i=0; i<100; i++)
    {
        i2cHandle = I2C_open(CONFIG_I2C0, &params);
        status = I2C_transfer(i2cHandle, &transaction);
        TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, status);
        I2C_close(i2cHandle);
    }
}

/*
 * Helper Functions
 */
static void test_i2c_set_test_params(I2C_TestParams *testParams, int8_t setting_id)
{
    I2C_Params *params = &(testParams->i2cParams);
    I2C_Params_init(params);
    testParams->deviceAddress = Board_i2cGetEepromDeviceAddr();
    testParams->memAddress = Board_i2cGetEepromMemAddr();
    testParams->numBytes = 3;
    testParams->numWritesReads = 2;
    testParams->testSetFrequency = false;
    testParams->intrEnable = true;

    switch (setting_id)
    {
        /* Blocking, 400KHZ */
        case 0:
            params->bitRate = I2C_400KHZ;
            break;

        /* Blocking, 100KHZ */
        case 1:
            break;

        /* Blocking, 400KHZ, 20 Bytes */
        case 2:
            params->bitRate = I2C_400KHZ;
			
            if (Board_i2cGetEepromAddrSize()==0x01U)
            {
				testParams->numBytes = 16;
            }
            else
            {
				testParams->numBytes = 20;
            }
            break;

        case 3:
            params->bitRate = I2C_400KHZ;
            testParams->numWritesReads = 10;
            break;

        case 4:
            testParams->testSetFrequency = true;
            break;
        /* Polling mode test */
        case 5:
            params->bitRate = I2C_400KHZ;
            testParams->intrEnable = false;
            break;
    }

    return;
}

static void test_i2c_callback(I2C_Handle i2cHnd, I2C_Transaction * msg, int32_t transferStatus)
{
    if(msg && msg->arg!=NULL)
    {
        SemaphoreP_post((SemaphoreP_Object*)msg->arg);
    }
}
