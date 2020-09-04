/*
 *  Copyright (C) 2021 Texas Instruments Incorporated
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

/** \file test_ddr.c
 *
 *  \brief This test is a simple DDR memory test.
 */

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <unity.h>
#include <kernel/dpl/DebugP.h>
#include "ti_drivers_open_close.h"

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

#define DDR_BASE_ADDR       (0x80000000U)
#define DDR_ADDR_END   		(0xC0000000U)
#define ONE_MEGABYTE        (0x100000)
#define BIT_COUNT			(32)
#define ONE			    	(0x00000001)
#define MSG_FREQ        	(0xFFFFFFC)
#define BIT_COUNT_EXT_MEM	(64)
#define BOARD_DIAG_MEM_LWORD_SPLIT(lw) ((lw >> 32) & 0xFFFFFFFF), (lw & 0xFFFFFFFF)


/* ========================================================================== */
/*                 Internal Function Declarations                             */
/* ========================================================================== */

void write_read_test(void *arg);
void board_walking1s_test (void *arg);
void board_walking0s_test (void *arg);

char rdBuf = 'y';

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */
/*
 * \brief   Memory test that writes and reads 1MB data, skips 1MB in a loop until 2GB DDR Space is reached.
 *
 * \return  None.
 */
void write_read_test(void *arg)
{
    int32_t status = SystemP_SUCCESS;
    uint32_t *memPtr = (uint32_t *)DDR_BASE_ADDR;
    uint32_t loop, loop1;

    /* Loop through 2GB of DDR. Write/Read 1MB, Skip 1MB. */
    for(loop1=0;loop1<1024;loop1++)
    {
        if(status == SystemP_SUCCESS)
        {
            /* Write 1MB of data */
            for(loop=0; loop<ONE_MEGABYTE/4; loop++)
            {
                *memPtr = loop;
                 memPtr++;
            }

            /*Reset to start of 1MB.*/
            memPtr -= (ONE_MEGABYTE/4);

            /* Read 1MB of data and compare. */
            for(loop=0; loop<ONE_MEGABYTE/4; loop++)
            {
                if(*memPtr != loop)
                {
                    DebugP_log("Write and read test failed!!\r\n");
                    status = SystemP_FAILURE;
                    break;
                }
                memPtr++;
            }

            /* Skip 1MB of data */
            memPtr += (ONE_MEGABYTE/4) ;
        }
        else
        {
            break;
        }
    }

    TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, status);
}


 /*
 * \brief   Memory test that does walking 1's test.
 *
 * \return  None.
 */
void board_walking1s_test (void *arg)
{
    uint64_t shift;
    uint64_t index;
    uint64_t value;
    uint64_t one =1;
	int32_t status = SystemP_SUCCESS;

    DebugP_log("\nRunning Walking 1s Test.\r\n");

    for (index = DDR_BASE_ADDR; (index >= DDR_BASE_ADDR) &&
         (index < DDR_ADDR_END); index += 8)
    {
        for (shift = 0; shift < BIT_COUNT_EXT_MEM; shift++)
        {
            value = *(volatile uint64_t *) index = (one << shift);
            if(value != *(volatile uint64_t *) index)
            {
                DebugP_log("DDR Walking 1's Test Failed at Location 0x%x\r\n",
                             (index));
                status = SystemP_FAILURE;		
            }
        }
		
		if(status == SystemP_SUCCESS)
		{
			if (!(index & MSG_FREQ))
			{
				DebugP_log("Write up to 0x%x done\r\n",
							index);
			}
		}
		else		
		{
			break;
		}
    }
	
	TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, status);
}
 /*
 * \brief   Memory test that does walking 0's test.
 *
 * \return  None.
 */
void board_walking0s_test (void *arg)
{
    uint64_t shift;
    uint64_t index;
    uint64_t value;
    uint64_t one =1;
	int32_t status = SystemP_SUCCESS;

    DebugP_log("\nRunning Walking 0s Test.\r\n");

    for (index = DDR_BASE_ADDR; (index >= DDR_BASE_ADDR) &&
         (index < DDR_ADDR_END); index += 8)
    {
        for (shift = 0; shift < BIT_COUNT_EXT_MEM; shift++)
        {
            value = *(volatile uint64_t *) index = ~(one << shift);
            if( value != *(volatile uint64_t *)index)
            {
                DebugP_log("DDR Walking 0's Test Failed at Location 0x%x\r\n",
                            (index));
                status = SystemP_FAILURE;
            }
        }
		
		if(status == SystemP_SUCCESS)
		{
			if (!(index & MSG_FREQ))
			{
				DebugP_log("Write up to 0x%x done\r\n", (index));
			}
		}
		else
		{
			break;
		}	
    }

	TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, status);
}


void *test_ddr_main(void *args)
{
    /* Open drivers to open the UART driver for console */
    Drivers_open();
    UNITY_BEGIN();

    RUN_TEST(write_read_test, 649, NULL);
	RUN_TEST(board_walking1s_test, 875, NULL);
	RUN_TEST(board_walking0s_test, 876, NULL);	

    UNITY_END();
    Drivers_close();

    return NULL;
}

void setUp(void)
{
}

void tearDown(void)
{
}
