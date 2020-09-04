/*
 * Copyright (C) 2022 Texas Instruments Incorporated
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

#include <stdio.h>
#include <string.h>
#include <inttypes.h>
#include <unity.h>
#include <kernel/dpl/DebugP.h>
#include <kernel/dpl/ClockP.h>
#include "ti_drivers_open_close.h"
#include "ti_board_open_close.h"
#include <drivers/firewall.h>

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */


/* ========================================================================== */
/*                 Internal Function Declarations                             */
/* ========================================================================== */

/* Test cases */
static void test_fwl_read_write_region(void *args);


/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */


/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */
void loop_forever()
{
    volatile uint32_t loop = 1;
    while(loop)
        ;
}

void test_main(void *args)
{
    /* Open drivers to open the UART driver for console */
    Drivers_open();
    UNITY_BEGIN();

    RUN_TEST(test_fwl_read_write_region, 0, NULL);

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

static void test_fwl_read_write_region(void *args)
{
    int32_t retVal = SystemP_SUCCESS;
    int32_t data = 0x22222222;
    int32_t readData;
    Firewall_Handle handle = NULL;

    /* Firewall region settings */
    Firewall_RegionCfg regionConfigFirewall0[1] =
    {
        {
                .regionIndex    = 0,
                .control        = FWL_CONTROL_ENABLE | FWL_CONTROL_BG | FWL_CONTROL_CACHE_MODE,
                .permissions[0] = 0xD4FFFF,
                .permissions[1] = 0x0,
                .permissions[2] = 0x0,
                .startAddr      = 0x70000000,
                .endAddr        = 0x7003FFFF,
        },
    };

    /* Firewall Driver Attributes */
    Firewall_Attrs gFirewallAttrs[1] =
    {
        {
            .firewallId = 14,
            .totalRegions = 4,
            .regionInfo = regionConfigFirewall0,
            .initRegions = 1,
        },
    };

    retVal = Firewall_configureRegion(handle, gFirewallAttrs);

    CSL_REG32_WR(CSL_MSRAM_256K0_RAM_BASE, data);
    readData = CSL_REG32_RD(CSL_MSRAM_256K0_RAM_BASE);
    if(data != readData)
    {
        retVal = SystemP_FAILURE;
    }
    TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, retVal);

}



