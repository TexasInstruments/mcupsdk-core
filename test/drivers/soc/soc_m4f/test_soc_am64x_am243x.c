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

#include <stdio.h>
#include <string.h>
#include <inttypes.h>
#include <unity.h>
#include <kernel/dpl/SemaphoreP.h>
#include <kernel/dpl/DebugP.h>
#include <kernel/dpl/ClockP.h>
#include "ti_drivers_open_close.h"
#include "ti_board_open_close.h"
#include <drivers/soc.h>
#include <drivers/hw_include/cslr_soc.h>
#include <kernel/dpl/CpuIdP.h>

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */
#define MHZ                        (1000000)


/* ========================================================================== */
/*                 Internal Function Declarations                             */
/* ========================================================================== */
void test_socSwWarmResetFromMcu(void *args);
/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */



/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */
void test_socSwWarmResetFromMcu(void *args)
{
    uint32_t resetCause;
    int32_t status = SystemP_FAILURE;

    resetCause = SOC_getWarmResetCauseMcuDomain();
    if (resetCause == 0U)
    {
        status = SystemP_SUCCESS;
        DebugP_log("MCU POR Reset Test Passed\r\n");
        /* SW Warm Reset */
        SOC_generateSwWarmResetMcuDomain();
    }
    else if(((resetCause & CSL_MCU_CTRL_MMR_CFG0_RST_SRC_SW_MCU_WARMRST_MASK) >> 
              CSL_MCU_CTRL_MMR_CFG0_RST_SRC_SW_MCU_WARMRST_SHIFT) == 1U)
    {
        DebugP_log("MCU Software Warm Reset Test Passed\r\n");
        status = SystemP_SUCCESS;
        DebugP_log("Reset value before clearing MCU Software Warm Reset %x\r\n", resetCause);
        SOC_clearResetCauseMainMcuDomain(resetCause);
        DebugP_log("Reset value after clearing MCU Software Warm Reset %x\r\n", SOC_getWarmResetCauseMcuDomain());
        /* MCU SW POR Reset */
        SOC_generateSwPORResetMainDomainFromMcuDomain();
    }
    else if(((resetCause & CSL_MCU_CTRL_MMR_CFG0_RST_SRC_SW_MAIN_POR_FROM_MCU_MASK) >> 
              CSL_MCU_CTRL_MMR_CFG0_RST_SRC_SW_MAIN_POR_FROM_MCU_SHIFT) == 1U)
    {
        DebugP_log("MCU Software POR Reset Test Passed\r\n");
        status = SystemP_SUCCESS;
        DebugP_log("Reset value before clearing MCU Software POR Reset %x\r\n", resetCause);
        SOC_clearResetCauseMainMcuDomain(resetCause);
        DebugP_log("Reset value after clearing MCU Software POR Reset %x\r\n", SOC_getWarmResetCauseMcuDomain());
        /* MCU SW Warm Reset */
        SOC_generateSwWarmResetMainDomainFromMcuDomain();
    }
    else if(((resetCause & CSL_MCU_CTRL_MMR_CFG0_RST_SRC_SW_MAIN_WARMRST_FROM_MCU_MASK) >> 
              CSL_MCU_CTRL_MMR_CFG0_RST_SRC_SW_MAIN_WARMRST_FROM_MCU_SHIFT) == 1U)
    {
        DebugP_log("MCU Software MAIN Reset Test Passed\r\n");
        status = SystemP_SUCCESS;
        DebugP_log("Reset value before clearing MCU Software MAIN Reset %x\r\n", resetCause);
        SOC_clearResetCauseMainMcuDomain(resetCause);
        DebugP_log("Reset value after clearing MCU Software MAIN Reset %x\r\n", SOC_getWarmResetCauseMcuDomain());
    }
    else
    {
        status = SystemP_FAILURE;
    }

    TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, status);
    return;
}

void test_socdriver(void *args)
{
#if defined(__ARM_ARCH_7EM__)
    RUN_TEST(test_socSwWarmResetFromMcu, 4109, NULL);
#endif
}
