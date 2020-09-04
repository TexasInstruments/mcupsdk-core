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


/* ========================================================================== */
/*                 Internal Function Declarations                             */
/* ========================================================================== */
void test_socPhyToVirtAndVirtToPhy(void *args);
void test_socSwWarmReset(void *args);
/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */



/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */
/* Passing physical address with random offset and expecting the return value
 * with the actual virtual address */
void test_socPhyToVirtAndVirtToPhy(void *args)
{

#if (__ARM_ARCH == 7) && (__ARM_ARCH_PROFILE == 'R')
    CSL_ArmR5CPUInfo cpuInfo;
    uint64_t    phyAddr;
    void       *virtAddr;

    /* Get Core ID Info */
    CSL_armR5GetCpuID(&cpuInfo);
    if (cpuInfo.grpId == (uint32_t)CSL_ARM_R5_CLUSTER_GROUP_ID_0) /* R5SS0-0 */
    {
        if(cpuInfo.cpuID == CSL_ARM_R5_CPU_ID_0)
        {
            phyAddr = SOC_virtToPhy((void *)(CSL_MSS_TCMA_RAM_BASE + 0x10));
            TEST_ASSERT_EQUAL_UINT32(phyAddr, CSL_MSS_TCMA_CR5A_U_BASE + 0x10);
            phyAddr = SOC_virtToPhy((void *)(CSL_MSS_TCMB_RAM_BASE + 0x10));
            TEST_ASSERT_EQUAL_UINT32(phyAddr, CSL_MSS_TCMB_CR5A_U_BASE + 0x10);

            virtAddr = SOC_phyToVirt(CSL_MSS_TCMA_CR5A_U_BASE + (CSL_MSS_TCMA_RAM_SIZE / 2));
            TEST_ASSERT_EQUAL_UINT32((uint32_t)virtAddr, CSL_MSS_TCMA_RAM_BASE + (CSL_MSS_TCMA_RAM_SIZE / 2));
            virtAddr = SOC_phyToVirt(CSL_MSS_TCMB_CR5A_U_BASE + (CSL_MSS_TCMB_RAM_SIZE / 2));
            TEST_ASSERT_EQUAL_UINT32((uint32_t)virtAddr, CSL_MSS_TCMB_RAM_BASE + (CSL_MSS_TCMB_RAM_SIZE / 2));

            DebugP_log("R5FSS0 Core0 PhyToVirtAndVirtToPhy Test Passed\r\n");
        }
        if(cpuInfo.cpuID == CSL_ARM_R5_CPU_ID_1)
        {
            phyAddr = SOC_virtToPhy((void *)(CSL_MSS_TCMA_RAM_BASE + 0x20));
            TEST_ASSERT_EQUAL_UINT32(phyAddr, CSL_MSS_TCMA_CR5B_U_BASE + 0x20);
            phyAddr = SOC_virtToPhy((void *)(CSL_MSS_TCMB_RAM_BASE + 0x20));
            TEST_ASSERT_EQUAL_UINT32(phyAddr, CSL_MSS_TCMB_CR5B_U_BASE + 0x20);

            virtAddr = SOC_phyToVirt(CSL_MSS_TCMA_CR5B_U_BASE + (CSL_MSS_TCMA_RAM_SIZE / 2));
            TEST_ASSERT_EQUAL_UINT32((uint32_t)virtAddr, CSL_MSS_TCMA_RAM_BASE + (CSL_MSS_TCMA_RAM_SIZE / 2));
            virtAddr = SOC_phyToVirt(CSL_MSS_TCMB_CR5B_U_BASE + (CSL_MSS_TCMB_RAM_SIZE / 2));
            TEST_ASSERT_EQUAL_UINT32((uint32_t)virtAddr, CSL_MSS_TCMB_RAM_BASE + (CSL_MSS_TCMB_RAM_SIZE / 2));

            DebugP_log("R5FSS0 Core1 PhyToVirtAndVirtToPhy Test Passed\r\n");
        }
    }
#endif

}

void test_socSwWarmReset(void *args)
{
    uint16_t resetCause;
    int32_t status = SystemP_FAILURE;

    resetCause = SOC_getWarmResetCause();
    if (resetCause == SOC_WarmResetCause_POWER_ON_RESET)
    {
        status = SystemP_SUCCESS;
        DebugP_log("POR Reset Test Passed\r\n");
        /* Clear and Trigger SW Warm Reset */
        SOC_clearWarmResetCause();
        SOC_generateSwWarmReset();
    }
    else if(resetCause == SOC_WarmResetCause_TOP_RCM_WARM_RESET_CONFIG)
    {
        DebugP_log("Software Warm Reset Test Passed\r\n");
        status = SystemP_SUCCESS;
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
    /* Run below tests only on R5SS0-0 as other cores not required */
#if (__ARM_ARCH == 7) && (__ARM_ARCH_PROFILE == 'R')
    CSL_ArmR5CPUInfo cpuInfo;

    /* Get Core ID Info */
    CSL_armR5GetCpuID(&cpuInfo);
    if (cpuInfo.grpId == (uint32_t)CSL_ARM_R5_CLUSTER_GROUP_ID_0)
    {
        if(cpuInfo.cpuID == CSL_ARM_R5_CPU_ID_0)
        {
            RUN_TEST(test_socSwWarmReset, 3890, NULL);
        }
    }
#endif
    RUN_TEST(test_socPhyToVirtAndVirtToPhy, 3881, NULL);
}
