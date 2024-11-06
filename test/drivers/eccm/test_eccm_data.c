/*
 *  Copyright (C) 2024 Texas Instruments Incorporated
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

#include <drivers/gpio.h>
#include <string.h>
#include <kernel/dpl/DebugP.h>
#include <unity.h>
#include "ti_drivers_config.h"
#include "ti_drivers_open_close.h"
#include "ti_board_open_close.h"
#include <drivers/hw_include/soc_config.h>
#include "testdata.h"


static void test_eccm_id_31(void*args);

static void test_eccm_id_32(void*args);

static void test_eccm_id_33(void*args);

void test_main(void *args)
{
    Drivers_open();
    Board_driversOpen();

    UNITY_BEGIN();

    RUN_TEST(test_eccm_id_31, 13849, (void*)NULL);

    RUN_TEST(test_eccm_id_32, 13850, (void*)NULL);

    RUN_TEST(test_eccm_id_33, 13852, (void*)NULL);

    UNITY_END();

    Board_driversClose();
    Drivers_close();
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

static void test_eccm_id_31(void*args)
{
    int32_t status = SystemP_SUCCESS;
    for(uint32_t i = 0; i < TEST_DATA_LENGTH; i++)
    {
        if(byte_array_dec_1[i] != i)
        {
            status = SystemP_FAILURE;
            break;
        }
    }
    TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, status);
}

static void test_eccm_id_32(void*args)
{
    int32_t status = SystemP_SUCCESS;
    for(uint32_t i = 0; i < TEST_DATA_LENGTH; i++)
    {
        if(byte_array_dec_2[i] != i)
        {
            status = SystemP_FAILURE;
            break;
        }
    }
    TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, status);
}

static void test_eccm_id_33(void*args)
{
    int32_t status = SystemP_SUCCESS;
    for(uint32_t i = 0; i < TEST_DATA_LENGTH; i++)
    {
        if(byte_array_dec_3[i] != i)
        {
            status = SystemP_FAILURE;
            break;
        }
    }
    TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, status);
}
