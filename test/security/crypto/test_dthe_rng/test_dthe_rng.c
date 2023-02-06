/*
 * TIFS-MCU Source File
 *
 * This test demonstrates the DTHE rng (random number generator)
 *
 * Copyright (C) 2023 Texas Instruments Incorporated - http://www.ti.com/
 * ALL RIGHTS RESERVED
 */
#include <string.h>
#include <stdlib.h>
#include <unity.h>
#include <drivers/soc.h>
#include <kernel/dpl/DebugP.h>
#include <kernel/dpl/CycleCounterP.h>
#include "ti_drivers_config.h"
#include "ti_drivers_open_close.h"
#include "ti_board_open_close.h"
#include <security/crypto/rng/rng.h>
#include <security/crypto/dthe/dthe.h>

/* DTHE Public address */
#define CSL_DTHE_PUBLIC_U_BASE                        (0xCE000810U)
/* DTHE RNG base addres */
#define CSL_DTHE_RNG_U_BASE                           (0xCE00A000U)

/* Rng Number of words to be read */
#define TEST_RNG_OUTPUT_WORDS_LENGTH        (4U)
#define TEST_RNG_NUM_OF_TEST_CASES          (5U)
#define TEST_RNG_TOTAL_OUT_LENGTH           (20U)

uint32_t gRngOutBuf[TEST_RNG_TOTAL_OUT_LENGTH];

static void test_Rng(void *args);
static int32_t test_RngOutPutCheck(uint32_t *outArray, uint32_t arrayLength);

void test_main(void *args)
{
    RUN_TEST(test_Rng, 216, NULL);
}

void test_Rng(void *args)
{
    Drivers_open();
    Board_driversOpen();
    RNG_Handle    handle   = NULL;
    uint32_t i = 0;
    RNG_Return_t status = RNG_RETURN_SUCCESS;

    DebugP_log("[DTHE] Rng test started ...\r\n");

    handle = RNG_open(0);
    DebugP_assert(handle != NULL);

    status = RNG_setup(handle);
    TEST_ASSERT_EQUAL_UINT32(RNG_RETURN_SUCCESS, status);

    for(i = 0; i< TEST_RNG_NUM_OF_TEST_CASES; i++)
    {
        status = RNG_read(handle, &gRngOutBuf[i*TEST_RNG_OUTPUT_WORDS_LENGTH]);
        TEST_ASSERT_EQUAL_UINT32(RNG_RETURN_SUCCESS, status);
    }
    test_RngOutPutCheck((uint32_t *)&gRngOutBuf, TEST_RNG_TOTAL_OUT_LENGTH);

    /* Close RNG instance */
    status = RNG_close(handle);

    DebugP_log("[DTHE] Rng test completed!!\r\n");
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

int32_t test_RngOutPutCheck(uint32_t *outArray, uint32_t arrayLength)
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

RNG_Attrs gRNG_Attrs[1] =
{
    {
        .caBaseAddr         = CSL_DTHE_PUBLIC_U_BASE,
        .rngBaseAddr        = CSL_DTHE_RNG_U_BASE,
        .isOpen             = FALSE,
    },
};

RNG_Config gRngConfig[1] =
{
    {
        &gRNG_Attrs[0],
    },
};

uint32_t gRngConfigNum = 1;