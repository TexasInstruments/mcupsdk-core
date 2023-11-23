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

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <inttypes.h>
#include <unity.h>
#include <kernel/dpl/DebugP.h>
#include <kernel/dpl/ClockP.h>
#include "ti_drivers_open_close.h"
#include "ti_board_open_close.h"
#include "test_mb.h"

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

#define APP_BUFFER_SIZE (0x128000) /* Size is slightly greater than 1 MB */

/* ========================================================================== */
/*                 Internal Function Declarations                             */
/* ========================================================================== */

/* Test cases */
static void test_mb(void *args);

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */

/* Prefilled array of multiples of 2 */
uint32_t gBigBuffer[APP_BUFFER_SIZE/4] = TEST_ARRAY;

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

void test_main(void *args)
{
    /* Open drivers to open the UART driver for console */
    Drivers_open();
    UNITY_BEGIN();

    RUN_TEST(test_mb, 0, NULL);

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
 * Test cases
 */

static void test_mb(void *args)
{
    uint32_t i;
    int32_t retVal = SystemP_SUCCESS;
    /* Verify the big buffer, to check if it was loaded correctly */
    for(i = 0; i < APP_BUFFER_SIZE/4; i++)
    {
        if(gBigBuffer[i] != 2*i)
        {
            retVal = SystemP_FAILURE;
            break;
        }
    }
    TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, retVal);
}
