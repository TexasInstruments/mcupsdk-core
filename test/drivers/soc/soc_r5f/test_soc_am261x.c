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
void test_socSetFrequencyR5FSS(void *args);
void test_socPhyToVirtAndVirtToPhy(void *args);
void test_socSwWarmReset(void *args);
void i2c_flash_reset();

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */



/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */
void test_socSetFrequencyR5FSS(void *args)
{
    int32_t          retVal;

    DebugP_log("SOC Test Started...\r\n");

    SOC_controlModuleUnlockMMR(0, MSS_RCM_PARTITION0);
    SOC_controlModuleUnlockMMR(0, TOP_RCM_PARTITION0);

    DebugP_log("Set MCAN clock\r\n");
    retVal = SOC_rcmSetPeripheralClock(SOC_RcmPeripheralId_MCAN0, SOC_RcmPeripheralClockSource_DPLL_CORE_HSDIV0_CLKOUT0, 80 * MHZ);
    TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, retVal);

    retVal = SOC_rcmSetPeripheralClock(SOC_RcmPeripheralId_MCAN1, SOC_RcmPeripheralClockSource_DPLL_CORE_HSDIV0_CLKOUT0, 80 * MHZ);
    TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, retVal);

    retVal = SOC_rcmSetPeripheralClock(SOC_RcmPeripheralId_MCAN2, SOC_RcmPeripheralClockSource_DPLL_CORE_HSDIV0_CLKOUT0, 80 * MHZ);
    TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, retVal);

    retVal = SOC_rcmSetPeripheralClock(SOC_RcmPeripheralId_MCAN3, SOC_RcmPeripheralClockSource_DPLL_CORE_HSDIV0_CLKOUT0, 80 * MHZ);
    TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, retVal);

    DebugP_log("Set OSPI clock\r\n");
    retVal = SOC_rcmSetPeripheralClock(SOC_RcmPeripheralId_OSPI0, SOC_RcmPeripheralClockSource_DPLL_CORE_HSDIV0_CLKOUT0, 80 * MHZ);
    TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, retVal);

    DebugP_log("Set RTI clock\r\n");
    retVal = SOC_rcmSetPeripheralClock(SOC_RcmPeripheralId_RTI0, SOC_RcmPeripheralClockSource_SYS_CLK, 200 * MHZ);
    TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, retVal);

    retVal = SOC_rcmSetPeripheralClock(SOC_RcmPeripheralId_RTI1, SOC_RcmPeripheralClockSource_SYS_CLK, 200 * MHZ);
    TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, retVal);

    retVal = SOC_rcmSetPeripheralClock(SOC_RcmPeripheralId_RTI2, SOC_RcmPeripheralClockSource_SYS_CLK, 200 * MHZ);
    TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, retVal);

    retVal = SOC_rcmSetPeripheralClock(SOC_RcmPeripheralId_RTI3, SOC_RcmPeripheralClockSource_SYS_CLK, 200 * MHZ);
    TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, retVal);

    DebugP_log("Set WDT clock\r\n");
    retVal = SOC_rcmSetPeripheralClock(SOC_RcmPeripheralId_WDT0, SOC_RcmPeripheralClockSource_SYS_CLK, 200 * MHZ);
    TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, retVal);

    retVal = SOC_rcmSetPeripheralClock(SOC_RcmPeripheralId_WDT1, SOC_RcmPeripheralClockSource_SYS_CLK, 200 * MHZ);
    TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, retVal);

    retVal = SOC_rcmSetPeripheralClock(SOC_RcmPeripheralId_WDT2, SOC_RcmPeripheralClockSource_SYS_CLK, 200 * MHZ);
    TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, retVal);

    retVal = SOC_rcmSetPeripheralClock(SOC_RcmPeripheralId_WDT3, SOC_RcmPeripheralClockSource_SYS_CLK, 200 * MHZ);
    TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, retVal);

    DebugP_log("Set MCSPI clock\r\n");
    retVal = SOC_rcmSetPeripheralClock(SOC_RcmPeripheralId_MCSPI0, SOC_RcmPeripheralClockSource_DPLL_CORE_HSDIV0_CLKOUT0, 50 * MHZ);
    TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, retVal);

    retVal = SOC_rcmSetPeripheralClock(SOC_RcmPeripheralId_MCSPI1, SOC_RcmPeripheralClockSource_DPLL_CORE_HSDIV0_CLKOUT0, 50 * MHZ);
    TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, retVal);

    retVal = SOC_rcmSetPeripheralClock(SOC_RcmPeripheralId_MCSPI2, SOC_RcmPeripheralClockSource_DPLL_CORE_HSDIV0_CLKOUT0, 50 * MHZ);
    TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, retVal);

    retVal = SOC_rcmSetPeripheralClock(SOC_RcmPeripheralId_MCSPI3, SOC_RcmPeripheralClockSource_DPLL_CORE_HSDIV0_CLKOUT0, 50 * MHZ);
    TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, retVal);

    retVal = SOC_rcmSetPeripheralClock(SOC_RcmPeripheralId_MCSPI4, SOC_RcmPeripheralClockSource_DPLL_CORE_HSDIV0_CLKOUT0, 50 * MHZ);
    TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, retVal);

    DebugP_log("Set MMC clock\r\n");
    retVal = SOC_rcmSetPeripheralClock(SOC_RcmPeripheralId_MMC0, SOC_RcmPeripheralClockSource_DPLL_CORE_HSDIV0_CLKOUT0, 50 * MHZ);
    TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, retVal);

    retVal = SOC_rcmSetPeripheralClock(SOC_RcmPeripheralId_MMC0, SOC_RcmPeripheralClockSource_DPLL_PER_HSDIV0_CLKOUT1, 48 * MHZ);
    TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, retVal);

    DebugP_log("Set ICSSM UART clock\r\n");
    retVal = SOC_rcmSetPeripheralClock(SOC_RcmPeripheralId_ICSSM0_UART0, SOC_RcmPeripheralClockSource_DPLL_PER_HSDIV0_CLKOUT1, 192 * MHZ);
    TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, retVal);

    DebugP_log("Set CPTS clock\r\n");
    retVal = SOC_rcmSetPeripheralClock(SOC_RcmPeripheralId_CPTS, SOC_RcmPeripheralClockSource_DPLL_CORE_HSDIV0_CLKOUT1, 250 * MHZ);
    TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, retVal);

    DebugP_log("Set ControlSS PLL clock\r\n");
    retVal = SOC_rcmSetPeripheralClock(SOC_RcmPeripheralId_CONTROLSS_PLL, SOC_RcmPeripheralClockSource_DPLL_CORE_HSDIV0_CLKOUT2, 400 * MHZ);
    TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, retVal);

    DebugP_log("Set I2C clock\r\n");
    retVal = SOC_rcmSetPeripheralClock(SOC_RcmPeripheralId_I2C, SOC_RcmPeripheralClockSource_DPLL_PER_HSDIV0_CLKOUT1, 48 * MHZ);
    TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, retVal);

    DebugP_log("Set LIN UART clock\r\n");
    /* Disabling UART0 Clock as UART0 is used in this testcase for logging */
    /* retVal = SOC_rcmSetPeripheralClock(SOC_RcmPeripheralId_LIN0_UART0, SOC_RcmPeripheralClockSource_DPLL_PER_HSDIV0_CLKOUT1, 192 * MHZ);
    TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, retVal); */

    retVal = SOC_rcmSetPeripheralClock(SOC_RcmPeripheralId_LIN1_UART1, SOC_RcmPeripheralClockSource_DPLL_PER_HSDIV0_CLKOUT1, 192 * MHZ);
    TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, retVal);

    retVal = SOC_rcmSetPeripheralClock(SOC_RcmPeripheralId_LIN2_UART2, SOC_RcmPeripheralClockSource_DPLL_PER_HSDIV0_CLKOUT1, 192 * MHZ);
    TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, retVal);

    retVal = SOC_rcmSetPeripheralClock(SOC_RcmPeripheralId_LIN3_UART3, SOC_RcmPeripheralClockSource_DPLL_PER_HSDIV0_CLKOUT1, 192 * MHZ);
    TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, retVal);

    retVal = SOC_rcmSetPeripheralClock(SOC_RcmPeripheralId_LIN4_UART4, SOC_RcmPeripheralClockSource_DPLL_PER_HSDIV0_CLKOUT1, 192 * MHZ);
    TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, retVal);

    retVal = SOC_rcmSetPeripheralClock(SOC_RcmPeripheralId_LIN5_UART5, SOC_RcmPeripheralClockSource_DPLL_PER_HSDIV0_CLKOUT1, 192 * MHZ);
    TEST_ASSERT_EQUAL_INT32(SystemP_SUCCESS, retVal);

    //FIXME: Verify
    uint32_t cpuFreq = 400 * MHZ, sysClkFreq = 200 * MHZ;
    uint32_t testCpuFreq;

    SOC_rcmsetR5SysClock(cpuFreq, sysClkFreq, CSL_CORE_ID_R5FSS0_0);
    testCpuFreq = SOC_rcmGetR5Clock(CSL_CORE_ID_R5FSS0_0);
    TEST_ASSERT_EQUAL_INT32(testCpuFreq, cpuFreq);
    DebugP_log("R5FSS0 Core Frequency %u MHZ\r\n", testCpuFreq / MHZ);

    SOC_rcmsetR5SysClock(cpuFreq, sysClkFreq, CSL_CORE_ID_R5FSS1_0);
    testCpuFreq = SOC_rcmGetR5Clock(CSL_CORE_ID_R5FSS1_0);
    TEST_ASSERT_EQUAL_INT32(testCpuFreq, cpuFreq);
    DebugP_log("R5FSS1 Core Frequency %u MHZ\r\n", testCpuFreq / MHZ);

    SOC_rcmsetTraceClock(250 * MHZ);

    SOC_rcmsetClkoutClock(250 * MHZ, 250 * MHZ);

}

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
            TEST_ASSERT_EQUAL_UINT32(phyAddr, CSL_R5SS0_CORE0_TCMA_U_BASE + 0x10);
            phyAddr = SOC_virtToPhy((void *)(CSL_MSS_TCMB_RAM_BASE + 0x10));
            TEST_ASSERT_EQUAL_UINT32(phyAddr, CSL_R5SS0_CORE0_TCMB_U_BASE + 0x10);

            virtAddr = SOC_phyToVirt(CSL_R5SS0_CORE0_TCMA_U_BASE + (CSL_MSS_TCMA_RAM_SIZE / 2));
            TEST_ASSERT_EQUAL_UINT32((uint32_t)virtAddr, CSL_MSS_TCMA_RAM_BASE + (CSL_MSS_TCMA_RAM_SIZE / 2));
            virtAddr = SOC_phyToVirt(CSL_R5SS0_CORE0_TCMB_U_BASE + (CSL_MSS_TCMB_RAM_SIZE / 2));
            TEST_ASSERT_EQUAL_UINT32((uint32_t)virtAddr, CSL_MSS_TCMB_RAM_BASE + (CSL_MSS_TCMB_RAM_SIZE / 2));

            DebugP_log("R5FSS0 Core0 PhyToVirtAndVirtToPhy Test Passed\r\n");
        }
        if(cpuInfo.cpuID == CSL_ARM_R5_CPU_ID_1)
        {
            phyAddr = SOC_virtToPhy((void *)(CSL_MSS_TCMA_RAM_BASE + 0x20));
            TEST_ASSERT_EQUAL_UINT32(phyAddr, CSL_R5SS0_CORE1_TCMA_U_BASE + 0x20);
            phyAddr = SOC_virtToPhy((void *)(CSL_MSS_TCMB_RAM_BASE + 0x20));
            TEST_ASSERT_EQUAL_UINT32(phyAddr, CSL_R5SS0_CORE1_TCMB_U_BASE + 0x20);

            virtAddr = SOC_phyToVirt(CSL_R5SS0_CORE1_TCMA_U_BASE + (CSL_MSS_TCMA_RAM_SIZE / 2));
            TEST_ASSERT_EQUAL_UINT32((uint32_t)virtAddr, CSL_MSS_TCMA_RAM_BASE + (CSL_MSS_TCMA_RAM_SIZE / 2));
            virtAddr = SOC_phyToVirt(CSL_R5SS0_CORE1_TCMB_U_BASE + (CSL_MSS_TCMB_RAM_SIZE / 2));
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
        /* Do a flash reset before SW warm reset to reset the flash configuration to 1s
         * and make it available for the ROM to load the image from the flash
         */
        #if (CPU_R5_0_0)
        i2c_flash_reset();
        #endif
        SOC_generateSwWarmReset();
    }
    else if(resetCause == SOC_WarmResetCause_TOP_RCM_WARM_RESET_REQ)
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
            RUN_TEST(test_socSetFrequencyR5FSS, 3880, NULL);
        }
    }
#endif
    RUN_TEST(test_socPhyToVirtAndVirtToPhy, 3881, NULL);
}
