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

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <string.h>
#include <stdlib.h>
#include <unity.h>
#include <drivers/resolver.h>
#include <drivers/soc.h>
#include "ti_drivers_open_close.h"
#include "ti_board_open_close.h"
#include <kernel/dpl/ClockP.h>

#include "menu.h"


/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */



/* ========================================================================== */
/*                         Structure Declarations                             */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

/* Testcases */


/* Non-JIRA Test case*/
static void RDC_apiCheck(void *args);

/*Utility/helper functions*/
int32_t test_rdc_cases(uint8_t in);

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */


#define TOTAL_TEST_CASES (1)

void test_main(void *args)
{
    /* Open drivers */
    Drivers_open();
    Board_driversOpen();

    tester_init();
    UNITY_BEGIN();
    char test_title[] = "----------------------RDC-TEST-CASES----------------------";



    menu_input test_list[TOTAL_TEST_CASES] =
    {
        {0, 1,     RDC_apiCheck,                                           "RDC_apiCheck"},
    };

    menu(TOTAL_TEST_CASES, test_list, test_title);

    if(enableLog)
    {
        DebugP_logZoneEnable(DebugP_LOG_ZONE_ERROR);
        DebugP_logZoneEnable(DebugP_LOG_ZONE_INFO);
    }
    UNITY_END();

    /* Close drivers */
    Board_driversClose();
    Drivers_close();

    return;
}

/* Unity framework required information */
void setUp(void)
{
}

void tearDown(void)
{
}

/* Test cases are numbered from 1 */
static void RDC_apiCheck(void *args)
{
    TEST_ASSERT_EQUAL_INT32(test_rdc_cases(1), 0);
}

uint32_t util_RDC_getInstanceFromBase(uint32_t base)
{
    switch(base)
    {
    case CSL_CONTROLSS_HW_RESOLVER_U_BASE:
            return 0;
    default:
            return 0xFFFFFFFF;
    }
}


#define NUM_RDC_INSTANCES (1)
uint32_t rdc_base_addr[NUM_RDC_INSTANCES] =
{
    CSL_CONTROLSS_HW_RESOLVER_U_BASE,
};

int32_t rdc_inst_tc_RDC_apiCheck(uint32_t base)
{
    int32_t error=0;

    uint32_t val = 2;

    /* Call the RDC_setAdcSequencerOperationalMode API */
    RDC_setAdcSequencerOperationalMode(base, val);

    /* Check if the value was written correctly */
    if(RDC_getAdcSequencerOperationalMode(base) != val)
    {
        error++;
    }

    return error;
}

uint8_t isTestSupported[1][1] =
{
    {1},  //Supported tests for RDC0
};

int32_t test_rdc_cases(uint8_t in)
{
    int32_t failcount=0;

    uint32_t base;
    uint32_t rdc_instance = 0;
    for(rdc_instance = 0; rdc_instance < NUM_RDC_INSTANCES; rdc_instance++)
    {
        base = rdc_base_addr[rdc_instance];
        if(enableLog)
        {
            DebugP_log("\r\n\nbase=0x%08x, instance=%d", base, util_RDC_getInstanceFromBase(base));
        }

        if(isTestSupported[util_RDC_getInstanceFromBase(base)][in-1])
        {
            switch(in)
            {
                case 1:
                    failcount += rdc_inst_tc_RDC_apiCheck(base);
                    break;
            }
            if(enableLog)
            {
                DebugP_log("\r\nfailcount=%d", failcount);
            }

        }
        else
        {
            if(enableLog)
            {
                DebugP_log("\r\nRDC %u, Test %u not supported", util_RDC_getInstanceFromBase(base), in-1);
            }

        }

    }


    if(failcount!=0)
    {
        if(enableLog)
        {
            DebugP_log("\r\nFAIL %d", failcount);
        }

        return 1;
    }
    else
    {
        if(enableLog)
        {
            DebugP_log("\r\nPass");
        }

        return 0;
    }

}
