/*
 * Copyright (C) 2023 Texas Instruments Incorporated
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

/* This test demonstrates the SA2UL RNG implementation */

#include <string.h>
#include <stdlib.h>
#include <unity.h>
#include <drivers/soc.h>
#include <kernel/dpl/DebugP.h>
#include <kernel/dpl/CycleCounterP.h>
#include "ti_drivers_config.h"
#include "ti_drivers_open_close.h"
#include "ti_board_open_close.h"

/* Sa2ul Rng Number of words to be read */
#define TEST_SA2ULRNG_OUTPUT_WORDS_LENGTH    (4U)
#define TEST_SA2ULRNG_NUM_OF_TEST_CASES      (5U)
#define TEST_SA2ULRNG_TOTAL_OUT_LENGTH       (20U)

uint32_t gSa2ulRngOutBuf[TEST_SA2ULRNG_TOTAL_OUT_LENGTH];

static void test_sa2ulRng(void *args);
static int32_t test_sa2ulRngOutPutCheck(uint32_t *outArray, uint32_t arrayLength);

void test_main(void *args)
{
    RUN_TEST(test_sa2ulRng, 2133, NULL);
}

void test_sa2ulRng(void *args)
{
    Drivers_open();
    Board_driversOpen();
    RNG_Handle    handle   = NULL;
    uint32_t i = 0; 
    RNG_Return_t status = RNG_RETURN_SUCCESS;

    DebugP_log("[SA2UL] Sa2ul Rng test started ...\r\n");

    handle = RNG_open(0);
    DebugP_assert(handle != NULL);

    status = RNG_setup(handle);
    TEST_ASSERT_EQUAL_UINT32(RNG_RETURN_SUCCESS, status);

    for(i = 0; i< TEST_SA2ULRNG_NUM_OF_TEST_CASES; i++)
    {
        status = RNG_read(handle, &gSa2ulRngOutBuf[i*TEST_SA2ULRNG_OUTPUT_WORDS_LENGTH]);
        TEST_ASSERT_EQUAL_UINT32(RNG_RETURN_SUCCESS, status);
    }
    test_sa2ulRngOutPutCheck((uint32_t *)&gSa2ulRngOutBuf, TEST_SA2ULRNG_TOTAL_OUT_LENGTH);
    
    /* Close RNG instance */
    status = RNG_close(handle);

    DebugP_log("[SA2UL] Sa2ul Rng example completed!!\r\n");
    DebugP_log("All tests have passed!!\r\n");

    Board_driversClose();
    Drivers_close();
}

/** Unity framework required functions */
void setUp(void)
{
}

void tearDown(void)
{
}

int32_t test_sa2ulRngOutPutCheck(uint32_t *outArray, uint32_t arrayLength)
{
    uint32_t i = 0, j = 0, status = RNG_RETURN_SUCCESS;
    for(i=0; i<arrayLength; i++)
    {
        for(j=i+1; j<arrayLength; j++)
        {
            if(outArray[i] == outArray[j])
            {
                status+=1;
            }
        }
    }
    TEST_ASSERT_EQUAL_UINT32(RNG_RETURN_SUCCESS, status);
    return (status);
}