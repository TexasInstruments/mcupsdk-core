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
            phyAddr = SOC_virtToPhy((void *)(CSL_R5FSS0_ATCM_BASE + 0x10));
            TEST_ASSERT_EQUAL_UINT32(phyAddr, CSL_R5FSS0_CORE0_ATCM_BASE + 0x10);
            phyAddr = SOC_virtToPhy((void *)(CSL_R5FSS0_BTCM_BASE + 0x10));
            TEST_ASSERT_EQUAL_UINT32(phyAddr, CSL_R5FSS0_CORE0_BTCM_BASE + 0x10);

            virtAddr = SOC_phyToVirt(CSL_R5FSS0_CORE0_ATCM_BASE + (CSL_R5FSS0_ATCM_SIZE / 2));
            TEST_ASSERT_EQUAL_UINT32((uint32_t)virtAddr, CSL_R5FSS0_ATCM_BASE + (CSL_R5FSS0_ATCM_SIZE / 2));
            virtAddr = SOC_phyToVirt(CSL_R5FSS0_CORE0_BTCM_BASE + (CSL_R5FSS0_BTCM_SIZE / 2));
            TEST_ASSERT_EQUAL_UINT32((uint32_t)virtAddr, CSL_R5FSS0_BTCM_BASE + (CSL_R5FSS0_BTCM_SIZE / 2));

            DebugP_log("R5FSS0 Core0 PhyToVirtAndVirtToPhy Test Passed\r\n");
        }
        if(cpuInfo.cpuID == CSL_ARM_R5_CPU_ID_1)
        {
            phyAddr = SOC_virtToPhy((void *)(CSL_R5FSS0_ATCM_BASE + 0x20));
            TEST_ASSERT_EQUAL_UINT32(phyAddr, CSL_R5FSS0_CORE1_ATCM_BASE + 0x20);
            phyAddr = SOC_virtToPhy((void *)(CSL_R5FSS0_BTCM_BASE + 0x20));
            TEST_ASSERT_EQUAL_UINT32(phyAddr, CSL_R5FSS0_CORE1_BTCM_BASE + 0x20);

            virtAddr = SOC_phyToVirt(CSL_R5FSS0_CORE1_ATCM_BASE + (CSL_R5FSS0_ATCM_SIZE / 2));
            TEST_ASSERT_EQUAL_UINT32((uint32_t)virtAddr, CSL_R5FSS0_ATCM_BASE + (CSL_R5FSS0_ATCM_SIZE / 2));
            virtAddr = SOC_phyToVirt(CSL_R5FSS0_CORE1_BTCM_BASE + (CSL_R5FSS0_BTCM_SIZE / 2));
            TEST_ASSERT_EQUAL_UINT32((uint32_t)virtAddr, CSL_R5FSS0_BTCM_BASE + (CSL_R5FSS0_BTCM_SIZE / 2));

            DebugP_log("R5FSS0 Core1 PhyToVirtAndVirtToPhy Test Passed\r\n");
        }
    }
    if (cpuInfo.grpId == (uint32_t)CSL_ARM_R5_CLUSTER_GROUP_ID_1) /* R5SS1-0 */
    {
        if(cpuInfo.cpuID == CSL_ARM_R5_CPU_ID_0)
        {
            phyAddr = SOC_virtToPhy((void *)(CSL_R5FSS1_ATCM_BASE));
            TEST_ASSERT_EQUAL_UINT32(phyAddr, CSL_R5FSS1_CORE0_ATCM_BASE);
            phyAddr = SOC_virtToPhy((void *)(CSL_R5FSS1_BTCM_BASE + (CSL_R5FSS1_BTCM_SIZE - 1U)));
            TEST_ASSERT_EQUAL_UINT32(phyAddr, CSL_R5FSS1_CORE0_BTCM_BASE + (CSL_R5FSS1_BTCM_SIZE - 1U));

            virtAddr = SOC_phyToVirt(CSL_R5FSS1_CORE0_ATCM_BASE + (CSL_R5FSS1_ATCM_SIZE / 2));
            TEST_ASSERT_EQUAL_UINT32((uint32_t)virtAddr, CSL_R5FSS1_ATCM_BASE + (CSL_R5FSS1_ATCM_SIZE / 2));
            virtAddr = SOC_phyToVirt(CSL_R5FSS1_CORE0_BTCM_BASE + (CSL_R5FSS0_BTCM_SIZE / 2));
            TEST_ASSERT_EQUAL_UINT32((uint32_t)virtAddr, CSL_R5FSS1_BTCM_BASE + (CSL_R5FSS1_BTCM_SIZE / 2));

            DebugP_log("R5FSS1 Core0 PhyToVirtAndVirtToPhy Test Passed\r\n");
        }
        if(cpuInfo.cpuID == CSL_ARM_R5_CPU_ID_1)
        {
            phyAddr = SOC_virtToPhy((void *)(CSL_R5FSS1_ATCM_BASE + (CSL_R5FSS1_ATCM_SIZE / 2)));
            TEST_ASSERT_EQUAL_UINT32(phyAddr, CSL_R5FSS1_CORE1_ATCM_BASE + (CSL_R5FSS1_ATCM_SIZE / 2));
            phyAddr = SOC_virtToPhy((void *)(CSL_R5FSS1_BTCM_BASE + 0xFF));
            TEST_ASSERT_EQUAL_UINT32(phyAddr, CSL_R5FSS1_CORE1_BTCM_BASE + 0xFF);

            virtAddr = SOC_phyToVirt(CSL_R5FSS1_CORE1_ATCM_BASE + (CSL_R5FSS1_ATCM_SIZE / 2));
            TEST_ASSERT_EQUAL_UINT32((uint32_t)virtAddr, CSL_R5FSS1_ATCM_BASE + (CSL_R5FSS1_ATCM_SIZE / 2));
            virtAddr = SOC_phyToVirt(CSL_R5FSS1_CORE1_BTCM_BASE + (CSL_R5FSS1_BTCM_SIZE / 2));
            TEST_ASSERT_EQUAL_UINT32((uint32_t)virtAddr, CSL_R5FSS1_BTCM_BASE + (CSL_R5FSS1_BTCM_SIZE / 2));

            DebugP_log("R5FSS1 Core1 PhyToVirtAndVirtToPhy Test Passed\r\n");
        }
    }

#endif

}

void test_socSwWarmReset(void *args)
{
    uint32_t resetCause;
    int32_t status = SystemP_FAILURE;

    resetCause = SOC_getWarmResetCauseMainDomain();
    if (resetCause == 0U)
    {
        status = SystemP_SUCCESS;
        DebugP_log("POR Reset Test Passed\r\n");
        /* SW Warm Reset */
        SOC_generateSwWarmResetMainDomain();
    }
    else if(((resetCause & CSL_MAIN_CTRL_MMR_CFG0_RST_SRC_SW_MAIN_WARMRST_FROM_MAIN_MASK) >>
              CSL_MAIN_CTRL_MMR_CFG0_RST_SRC_SW_MAIN_WARMRST_FROM_MAIN_SHIFT) == 1U)
    {
        DebugP_log("Software Warm Reset Test Passed\r\n");
        status = SystemP_SUCCESS;
        DebugP_log("Reset value before clearing Software Warm Reset %x\r\n", resetCause);
        SOC_clearResetCauseMainMcuDomain(resetCause);
        DebugP_log("Reset value after clearing Software Warm Reset %x\r\n", SOC_getWarmResetCauseMainDomain());
        /* SW POR Reset */
        SOC_generateSwPORResetMainDomain();
    }
    else if(((resetCause & CSL_MAIN_CTRL_MMR_CFG0_RST_SRC_SW_MAIN_POR_FROM_MAIN_MASK) >>
              CSL_MAIN_CTRL_MMR_CFG0_RST_SRC_SW_MAIN_POR_FROM_MAIN_SHIFT) == 1U)
    {
        DebugP_log("Software POR Reset Test Passed\r\n");
        status = SystemP_SUCCESS;
        DebugP_log("Reset value before clearing Software POR Reset %x\r\n", resetCause);
        SOC_clearResetCauseMainMcuDomain(resetCause);
        DebugP_log("Reset value after clearing Software POR Reset %x\r\n", SOC_getWarmResetCauseMainDomain());
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
#if defined(__ARM_ARCH_7R__)

    /* Disabling the test as SBL does a MCU warm reset */
#if 0
    /* Run below tests only on R5SS0-0 as other cores not required */
#if (__ARM_ARCH == 7) && (__ARM_ARCH_PROFILE == 'R')
    CSL_ArmR5CPUInfo cpuInfo;

    /* Get Core ID Info */
    CSL_armR5GetCpuID(&cpuInfo);
    if (cpuInfo.grpId == (uint32_t)CSL_ARM_R5_CLUSTER_GROUP_ID_0) /* R5SS0-0 */
    {
        if(cpuInfo.cpuID == CSL_ARM_R5_CPU_ID_0)
        {
            RUN_TEST(test_socSwWarmReset, 3890, NULL);
        }
    }
#endif
#endif
    RUN_TEST(test_socPhyToVirtAndVirtToPhy, 3881, NULL);
#endif
}
