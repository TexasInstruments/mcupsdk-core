/* Copyright (c) 2022-2025 Texas Instruments Incorporated
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
 *
 */

 /**
 *  \file     sdl_pbist_test_func.c
 *
 *  \brief    This file contains PBIST Functional test code.
 *
 *  \details  PBIST Functional tests
 **/

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */
#include <stdint.h>
#include <string.h>
#include <sdl/include/sdl_types.h>
#include <sdl/sdl_pbist.h>
#ifndef SDL_SOC_MCU_R5F
#include <drivers/sciclient.h>
#include "power_seq.h"
#endif

/* DPL API header files */
#include <kernel/dpl/ClockP.h>
#include <kernel/dpl/DebugP.h>


#include <pbist_test_cfg.h>
#include <sdl/include/hw_types.h>

/* ========================================================================== */
/*                                Macros                                      */
/* ========================================================================== */

/* This is to power up the cores before test and power down afterwards */
#define PBIST_POWERUP_CORES_BEFORE_TEST
#define PBIST_APP_TIMEOUT               (0x400000U)

#if defined (SOC_AM64X)
#if defined (R5F_CORE)
#define PBIST_PSC_BASE_ADDR             ((uint32_t)0x00400000)
#else
#define PBIST_PSC_BASE_ADDR             ((uint32_t)0x80400000)
#endif

/* The following macros will be used in modifying PSC register values */
#define PBIST_PSC_NEXT_MASK             (0x0000003F)
#define PBIST_PSC_FORCE_OFF             (0x80000001)
#define PBIST_PSC_MDCTL_OFFSET          (0xA00)
#define PBIST_PSC_MDCTL_ADDR            (PBIST_PSC_BASE_ADDR + PBIST_PSC_MDCTL_OFFSET)
#define PBIST_PSC_PTCMD_OFFSET          (0x120)
#define PBIST_PSC_PTCMD_ADDR            (PBIST_PSC_BASE_ADDR + PBIST_PSC_PTCMD_OFFSET)
#define PBIST_PSC_PTCMD_TIMEOUT         (1000000U)
#define PBIST_PSC_PTSTAT_OFFSET         (0x128)
#define PBIST_PSC_PTSTAT_ADDR           (PBIST_PSC_BASE_ADDR + PBIST_PSC_PTSTAT_OFFSET)
#endif

/* The following macro gives the PSC register address of a device */
#define PBIST_A53_0_ADDR                (PBIST_PSC_MDCTL_ADDR + 4*CSL_MAIN_LPSC_A53_0)

/* ========================================================================== */
/*                 Internal Function Declarations                             */
/* ========================================================================== */

volatile int32_t PBIST_PSCForceOff(uint32_t pscAddr);

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */

#if defined (SOC_AM64X) || defined (SOC_AM243X)
static uint64_t PBIST_profilingTime = 0;
#endif

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

/*
 * Certain IP(s) are left in a transition state after performing PBIST
 * tests in AM64x. These cannot be turned off using Sciclient because
 * they need a force bit to be set for turning off. The following API
 * is used to perform this task for the affected IPs after PBIST test.
 */
#if defined(SOC_AM64X)
volatile int32_t PBIST_PSCForceOff(uint32_t pscAddr)
{
    int32_t pscTimeout = PBIST_PSC_PTCMD_TIMEOUT;
    uint32_t pscRdValue;
    int32_t result = SDL_PASS;

    /* Read current value of the register */
    pscRdValue = HW_RD_REG32(pscAddr);
    /* Delete bits corresponding to state */
    pscRdValue &= ~PBIST_PSC_NEXT_MASK;
    /* Bits to be set for SyncRst state and Force bit */
    pscRdValue |= PBIST_PSC_FORCE_OFF;
    /* Write back to the PSC register */
    HW_WR_REG32(pscAddr,pscRdValue);
    /* Write 1 to PSC_PTCMD to cause state change */
    HW_WR_REG32(PBIST_PSC_PTCMD_ADDR,0x1);

    /* Wait until state transition is completed */
    while(((HW_RD_REG32(PBIST_PSC_PTSTAT_ADDR) & 0x1) != 0) && (pscTimeout>0))
    {
        pscTimeout--;
    }

    /* If the transition wait timed out */
    if((HW_RD_REG32(PBIST_PSC_PTSTAT_ADDR) & 0x1) != 0)
    {
        result = SDL_EFAIL;
    }
    return result;
}
#endif

#if defined (SOC_AM273X) || defined (SOC_AWR294X)
extern uint32_t gInst;
#endif
int32_t PBIST_runTest(uint32_t instanceId, bool runNegTest)
{
    int32_t testResult = 0;
    SDL_ErrType_t status;
    bool PBISTResult = false;
    SDL_PBIST_testType testType;

    uint64_t startTime , testStartTime,  testEndTime, endTime;
    uint64_t prepTime, diffTime, restoreTime;
#if defined (SOC_AM64X) || defined (SOC_AM243X)
    uint64_t instanceTime;
#endif
#ifdef DEBUG
    char inputChar;
#endif
#if defined(SOC_AM64X)
    uint32_t pscAddr;
    bool a53Skip = false;
#endif

#ifndef SDL_SOC_MCU_R5F
    int i;
    uint32_t moduleState = TISCI_MSG_VALUE_DEVICE_HW_STATE_OFF;
    uint32_t resetState = 0U;
    uint32_t contextLossState = 0U;

    if (runNegTest == true)
    {
        DebugP_log("\r\n Starting PBIST failure insertion test on %s, index %d...\r\n",
                    PBIST_TestHandleArray[instanceId].testName,
                    instanceId);
        testType = SDL_PBIST_NEG_TEST;
    }
    else
    {
        DebugP_log("\r\n Starting PBIST test on %s, index %d...\r\n",
                    PBIST_TestHandleArray[instanceId].testName,
                    instanceId);
        testType = SDL_PBIST_TEST;
    }

#ifdef DEBUG
    DebugP_log("\r\n Press any key to continue...");
    inputChar = UART_getChar();

    if (inputChar == 'n')
    {
        DebugP_log("\r\n Skipping this test. on request \r\n");
        return 0;
    }
#endif

    /* Get start time of test */
    startTime = ClockP_getTimeUsec();

    if ((testResult == 0) && (PBIST_TestHandleArray[instanceId].procRstNeeded))
    {
        if (PBIST_TestHandleArray[instanceId].tisciProcId != 0u)
        {
#ifdef DEBUG
            DebugP_log("  Primary core: %s: Requesting processor \r\n",
                        PBIST_TestHandleArray[instanceId].coreName);
#endif
            /* Request Primary core */
            status = Sciclient_procBootRequestProcessor(PBIST_TestHandleArray[instanceId].tisciProcId,
                                                        SystemP_WAIT_FOREVER);
            if (status != SDL_PASS)
            {
                DebugP_log("   Primary core: Sciclient_procBootRequestProcessor, ProcId 0x%x...FAILED \r\n",
                            PBIST_TestHandleArray[instanceId].tisciProcId);
                testResult = -1;
            }
        }
    }

    if ((testResult == 0) && (PBIST_TestHandleArray[instanceId].procRstNeeded))
    {
        if ((PBIST_TestHandleArray[instanceId].secondaryCoreNeeded)
            && (PBIST_TestHandleArray[instanceId].tisciSecProcId != 0u))
        {

#ifdef DEBUG
            DebugP_log("  Secondary core: %s: Requesting processor \r\n",
                    PBIST_TestHandleArray[instanceId].secCoreName);
#endif
            /* Request secondary core */
            status = Sciclient_procBootRequestProcessor(PBIST_TestHandleArray[instanceId].tisciSecProcId,
                                                        SystemP_WAIT_FOREVER);
            if (status != SDL_PASS)
            {
                DebugP_log("   Secondary core: Sciclient_procBootRequestProcessor, ProcId 0x%x...FAILED \r\n",
                            PBIST_TestHandleArray[instanceId].tisciSecProcId);
                testResult = -1;
            }
        }
    }

    if ((testResult == 0) && (PBIST_TestHandleArray[instanceId].procRstNeeded))
    {
        if (PBIST_TestHandleArray[instanceId].tisciDeviceId != 0u)
        {
            /* Set Local reset for Primary core */
#ifdef DEBUG
            DebugP_log("  %s: Primary core: Set module reset \r\n",
                        PBIST_TestHandleArray[instanceId].coreName);
#endif
            status =  Sciclient_pmSetModuleRst(PBIST_TestHandleArray[instanceId].tisciDeviceId,
                                               0x1, /* Local Reset asserted */
                                               SystemP_WAIT_FOREVER);
            if (status != SDL_PASS)
            {
                DebugP_log("   Primary core: Sciclient_pmSetModuleRst...FAILED \r\n");
                testResult = -1;
            }
        }
    }

    if ((testResult == 0) && (PBIST_TestHandleArray[instanceId].procRstNeeded))
    {
        if ((PBIST_TestHandleArray[instanceId].secondaryCoreNeeded)
            && (PBIST_TestHandleArray[instanceId].tisciSecDeviceId != 0u))
        {
            /* Set Local reset for Secondary core */
#ifdef DEBUG
            DebugP_log("  %s: Secondary core: Set Module reset \r\n",
                        PBIST_TestHandleArray[instanceId].secCoreName);
#endif
            status =  Sciclient_pmSetModuleRst(PBIST_TestHandleArray[instanceId].tisciSecDeviceId,
                                               0x1, /* Local Reset asserted */
                                               SystemP_WAIT_FOREVER);
            if (status != SDL_PASS)
            {
                DebugP_log("   Secondary core: Sciclient_pmSetModuleRst...FAILED \r\n");
                testResult = -1;
            }
        }
    }

#ifdef PBIST_POWERUP_CORES_BEFORE_TEST
    /* Custom core power restore sequence - needed to allow core to be powered
     * up later by Secondary Bootloader (SBL) */
    if ((testResult == 0) &&
        (PBIST_TestHandleArray[instanceId].coreCustPwrSeqNeeded) &&
        (PBIST_TestHandleArray[instanceId].tisciProcId != 0u))
    {
        status = customPrepareForPowerUpSequence(PBIST_TestHandleArray[instanceId].tisciProcId);
        if (status != SDL_PASS)
        {
            DebugP_log("  Custom core power restore sequence, ProcId 0x%x ...FAILED \r\n",
                        PBIST_TestHandleArray[instanceId].tisciProcId);
            testResult = -1;
        }
    }

    /* Power up of Auxilliary modules needed to run test */
    if (testResult == 0)
    {
        /* Power all modules required for test */
        for ( i = 0; i < PBIST_TestHandleArray[instanceId].numAuxDevices; i++)
        {
#ifdef DEBUG
            DebugP_log("  Powering on Device number %d Device Id %x\r\n",
                        i, PBIST_TestHandleArray[instanceId].auxDeviceIdsP[i]);
#endif

            status = Sciclient_pmSetModuleState(PBIST_TestHandleArray[instanceId].auxDeviceIdsP[i],
                                                TISCI_MSG_VALUE_DEVICE_SW_STATE_ON,
                                                TISCI_MSG_FLAG_AOP,
                                                SystemP_WAIT_FOREVER);
            if (status != SDL_PASS)
            {
                DebugP_log("  Sciclient_pmSetModuleState 0x%x ...FAILED \r\n",
                            PBIST_TestHandleArray[instanceId].auxDeviceIdsP[i]);
                testResult = -1;
                break;
            }
        }
    }

    if ((testResult == 0) && (PBIST_TestHandleArray[instanceId].procRstNeeded)
                          && (PBIST_TestHandleArray[instanceId].tisciDeviceId != 0U))
    {
        /* power on Primary core*/
#ifdef DEBUG
        DebugP_log("  Primary core: Powering on %s \r\n",
                    PBIST_TestHandleArray[instanceId].coreName);
#endif
        status = Sciclient_pmSetModuleState(PBIST_TestHandleArray[instanceId].tisciDeviceId,
                                            TISCI_MSG_VALUE_DEVICE_SW_STATE_ON,
                                            TISCI_MSG_FLAG_AOP,
                                            SystemP_WAIT_FOREVER);

        if (status != SDL_PASS)
        {
            DebugP_log("   Primary core: Sciclient_pmSetModuleState 0x%x ...FAILED \r\n",
                        PBIST_TestHandleArray[instanceId].tisciDeviceId);
            testResult = -1;
        }
    }

    if ((testResult == 0) && (PBIST_TestHandleArray[instanceId].procRstNeeded)
                          && (PBIST_TestHandleArray[instanceId].tisciSecDeviceId != 0U))
    {
        if (PBIST_TestHandleArray[instanceId].secondaryCoreNeeded)
        {
            /* power on Secondary core*/
#ifdef DEBUG
            DebugP_log("  Secondary core: Powering on %s \r\n",
                        PBIST_TestHandleArray[instanceId].secCoreName);
#endif
            status = Sciclient_pmSetModuleState(PBIST_TestHandleArray[instanceId].tisciSecDeviceId,
                                                TISCI_MSG_VALUE_DEVICE_SW_STATE_ON,
                                                TISCI_MSG_FLAG_AOP,
                                                SystemP_WAIT_FOREVER);
            if (status != SDL_PASS)
            {
                DebugP_log("   Secondary core: Sciclient_pmSetModuleState 0x%x ...FAILED \r\n",
                            PBIST_TestHandleArray[instanceId].tisciSecDeviceId);
                testResult = -1;
                return testResult;
            }
        }
    }
#endif /* #ifdef PBIST_POWERUP_CORES_BEFORE_TEST */
    /* Double check the Power up of Auxilliary modules needed to run test and wait until they
     * are powered up */
    if (testResult == 0)
    {
        /* Wait for all modules required for test to be powered up */
        for ( i = 0; i < PBIST_TestHandleArray[instanceId].numAuxDevices; i++)
        {
#ifdef DEBUG
        DebugP_log("  Double checking Powering on Device number %d Device Id %x\r\n",
                        i, PBIST_TestHandleArray[instanceId].auxDeviceIdsP[i]);
#endif
            do
            {
                status = Sciclient_pmGetModuleState(PBIST_TestHandleArray[instanceId].auxDeviceIdsP[i],
                                                    &moduleState,
                                                    &resetState,
                                                    &contextLossState,
                                                    SystemP_WAIT_FOREVER);
                if (status != SDL_PASS)
                {
                    DebugP_log("  Sciclient_pmGetModuleState 0x%x ...FAILED \r\n",
                                PBIST_TestHandleArray[instanceId].auxDeviceIdsP[i]);
                    testResult = -1;
                    break;
                }
            } while (moduleState != TISCI_MSG_VALUE_DEVICE_HW_STATE_ON);
        }
    }

    if ((testResult == 0) && (PBIST_TestHandleArray[instanceId].procRstNeeded)
                          && (PBIST_TestHandleArray[instanceId].tisciDeviceId != 0U))
    {
        /* Double check power on Primary core*/
#ifdef DEBUG
        DebugP_log("  Primary core: Double checking Powering on %s \r\n",
                        PBIST_TestHandleArray[instanceId].coreName);
#endif
        do
        {
            status = Sciclient_pmGetModuleState(PBIST_TestHandleArray[instanceId].tisciDeviceId,
                                                &moduleState,
                                                &resetState,
                                                &contextLossState,
                                                SystemP_WAIT_FOREVER);
            if (status != SDL_PASS)
            {
                DebugP_log("   Primary core: Sciclient_pmGetModuleState 0x%x ...FAILED \r\n",
                            PBIST_TestHandleArray[instanceId].tisciDeviceId);
                testResult = -1;
                break;
            }
        } while (moduleState != TISCI_MSG_VALUE_DEVICE_HW_STATE_ON);
    }

    if ((testResult == 0) && (PBIST_TestHandleArray[instanceId].procRstNeeded)
                          && (PBIST_TestHandleArray[instanceId].tisciSecDeviceId != 0U))
    {
        if (PBIST_TestHandleArray[instanceId].secondaryCoreNeeded)
        {
            /* Double check power on Secondary core*/
#ifdef DEBUG
            DebugP_log("  Secondary core: Double checking Powering on %s \r\n",
                            PBIST_TestHandleArray[instanceId].coreName);
#endif
            do
            {
                status = Sciclient_pmGetModuleState(PBIST_TestHandleArray[instanceId].tisciSecDeviceId,
                                                    &moduleState,
                                                    &resetState,
                                                    &contextLossState,
                                                    SystemP_WAIT_FOREVER);
                if (status != SDL_PASS)
                {
                    DebugP_log("   Secondary core: Sciclient_pmGetModuleState 0x%x ...FAILED \r\n",
                                PBIST_TestHandleArray[instanceId].tisciSecDeviceId);
                    testResult = -1;
                    break;
                }
            } while (moduleState != TISCI_MSG_VALUE_DEVICE_HW_STATE_ON);
        }
    }

    /* Power up PBIST */
    if ((testResult == 0) && (PBIST_TestHandleArray[instanceId].tisciPBISTDeviceId != 0u))
    {
#ifdef DEBUG
        DebugP_log("  Powering on PBIST %d \r\n",
                    PBIST_TestHandleArray[instanceId].tisciPBISTDeviceId);
#endif
        status = Sciclient_pmSetModuleState(PBIST_TestHandleArray[instanceId].tisciPBISTDeviceId,
                                            TISCI_MSG_VALUE_DEVICE_SW_STATE_ON,
                                            TISCI_MSG_FLAG_AOP,
                                            SystemP_WAIT_FOREVER);

        if (status != SDL_PASS)
        {
            DebugP_log("   PBIST Sciclient_pmSetModuleState 0x%x ...FAILED: retValue %d\r\n",
                        PBIST_TestHandleArray[instanceId].tisciPBISTDeviceId, status);
            testResult = -1;
        }
    }

    /* Execute Auxilliary init function */
    if (testResult == 0)
    {

        if (PBIST_TestHandleArray[instanceId].auxInitRestoreFunction != 0)
        {
            status = PBIST_TestHandleArray[instanceId].auxInitRestoreFunction(TRUE);
            if (status != SDL_PASS)
            {
                testResult = -1;
            }
        }

    }

    if (testResult == 0)
    {
        /* Get start time for PBIST test */
        testStartTime = ClockP_getTimeUsec();

        status = SDL_PBIST_selfTest((SDL_PBIST_inst)PBIST_TestHandleArray[instanceId].pbistInst, testType, PBIST_APP_TIMEOUT, &PBISTResult);
        if ((status != SDL_PASS) || (PBISTResult == false))
        {
            DebugP_log("  SDL_PBIST_selfTest failed status [%d] result [%d] \n", status, PBISTResult);
            testResult = -1;
        }
    }

    /* Record test end time */
    testEndTime = ClockP_getTimeUsec();

    /* Execute Auxilliary restore function */
    if (testResult == 0)
    {

        if (PBIST_TestHandleArray[instanceId].auxInitRestoreFunction != 0)
        {
            status = PBIST_TestHandleArray[instanceId].auxInitRestoreFunction(FALSE);
            if (status != SDL_PASS)
            {
                testResult = -1;
            }
        }

    }

    /* The following sequence is needed to restore core to normal operation */
    /* Power off PBIST */
    if ((testResult == 0) && (PBIST_TestHandleArray[instanceId].tisciPBISTDeviceId != 0u))
    {
#ifdef DEBUG
        DebugP_log("  Powering off PBIST %d \r\n",
                    PBIST_TestHandleArray[instanceId].tisciPBISTDeviceId);
#endif
        status = Sciclient_pmSetModuleState(PBIST_TestHandleArray[instanceId].tisciPBISTDeviceId,
                                            TISCI_MSG_VALUE_DEVICE_SW_STATE_AUTO_OFF,
                                            TISCI_MSG_FLAG_AOP,
                                            SystemP_WAIT_FOREVER);

        if (status != SDL_PASS)
        {
            DebugP_log("   PBIST Sciclient_pmSetModuleState 0x%x ...FAILED \r\n",
                            PBIST_TestHandleArray[instanceId].tisciPBISTDeviceId);
            testResult = -1;
        }
    }

#ifdef PBIST_POWERUP_CORES_BEFORE_TEST
    if ((testResult == 0) && (PBIST_TestHandleArray[instanceId].procRstNeeded)
                          && (PBIST_TestHandleArray[instanceId].tisciSecDeviceId != 0U))
    {
        if (PBIST_TestHandleArray[instanceId].secondaryCoreNeeded)
        {
            /* power off Secondary core*/
#ifdef DEBUG
            DebugP_log("  Secondary core: Powering off %s \r\n",
                        PBIST_TestHandleArray[instanceId].secCoreName);
#endif
            status = Sciclient_pmSetModuleState(PBIST_TestHandleArray[instanceId].tisciSecDeviceId,
                                                TISCI_MSG_VALUE_DEVICE_SW_STATE_AUTO_OFF,
                                                TISCI_MSG_FLAG_AOP,
                                                SystemP_WAIT_FOREVER);
            if (status != SDL_PASS)
            {
                DebugP_log("   Secondary core: Sciclient_pmSetModuleState Power off 0x%x ...FAILED \r\n",
                            PBIST_TestHandleArray[instanceId].tisciSecDeviceId);
                testResult = -1;
                return testResult;
            }
        }
    }

    /* Custom core power down sequence */
    if ((testResult == 0) &&
        (PBIST_TestHandleArray[instanceId].coreCustPwrSeqNeeded) &&
        (PBIST_TestHandleArray[instanceId].tisciProcId != 0u))
    {
        status = customPowerDownSequence(PBIST_TestHandleArray[instanceId].tisciProcId);
        if (status != SDL_PASS)
        {
            DebugP_log("  Custom core power down sequence, ProcId 0x%x ...FAILED \r\n",
                        PBIST_TestHandleArray[instanceId].tisciProcId);
            testResult = -1;
        }
    }

    if ((testResult == 0) && (PBIST_TestHandleArray[instanceId].tisciProcId != 0u)
                    && (PBIST_TestHandleArray[instanceId].procRstNeeded))
    {
        /* power off Primary core*/
#ifdef DEBUG
        DebugP_log("  Primary core: Powering off %s \r\n",
                    PBIST_TestHandleArray[instanceId].coreName);
#endif
        status = Sciclient_pmSetModuleState(PBIST_TestHandleArray[instanceId].tisciDeviceId,
                                            TISCI_MSG_VALUE_DEVICE_SW_STATE_AUTO_OFF,
                                            TISCI_MSG_FLAG_AOP,
                                            SystemP_WAIT_FOREVER);

        if (status != SDL_PASS)
        {
            DebugP_log("   Primary core: Sciclient_pmSetModuleState Power off 0x%x ...FAILED \r\n",
                        PBIST_TestHandleArray[instanceId].tisciDeviceId);
            testResult = -1;
        }
    }

    /* Power off of Auxilliary modules needed to run test */
    if (testResult == 0)
    {
        /* Power all modules required for test */
        for ( i = 0; i < PBIST_TestHandleArray[instanceId].numAuxDevices; i++)
        {
#ifdef DEBUG
            DebugP_log("  Powering off Device number %d Device Id %x\r\n",
                        i, PBIST_TestHandleArray[instanceId].auxDeviceIdsP[i]);
#endif
            /* 
             * In AM64x, the device TISCI_DEV_A53SS0 is left in a transition state after PBIST tests,
             * and cannot be powered off by Sciclient. Hence, we check for that and skip it here. *
             */
#if defined (SOC_AM64X)
            if (PBIST_TestHandleArray[instanceId].auxDeviceIdsP[i] == TISCI_DEV_A53SS0)
            {
                /* Flag to denote that A53 IP's power off has been skipped in this test */
                a53Skip = true;
                continue;
            }
#endif
            status = Sciclient_pmSetModuleState(PBIST_TestHandleArray[instanceId].auxDeviceIdsP[i],
                                                TISCI_MSG_VALUE_DEVICE_SW_STATE_AUTO_OFF,
                                                TISCI_MSG_FLAG_AOP,
                                                SystemP_WAIT_FOREVER);
            if (status != SDL_PASS)
            {
                DebugP_log("  Sciclient_pmSetModuleState 0x%x ...FAILED \r\n",
                            PBIST_TestHandleArray[instanceId].auxDeviceIdsP[i]);
                testResult = -1;
                break;
            }
        }
    }

    /*
     * Separately power off TISCI_DEV_A53SS0 (SyncRst state) by writing to PSC register
     * with force bit set. This is required because the A53 IP is in a transition state
     */
#if defined (SOC_AM64X)
    if ((testResult == 0) && (a53Skip))
    {
        /* This is the address of the PSC register corresponding to the A53 core */
        pscAddr = PBIST_A53_0_ADDR;

        status = PBIST_PSCForceOff(pscAddr);
        if (status != SDL_PASS)
        {
            DebugP_log("   A53: Force bit power-off failed\r\n");
            testResult = -1;
        }
    }
#endif

    /* Custom core power restore sequence - needed to allow core to be powered
     * up properly later */
    if ((testResult == 0) &&
        (PBIST_TestHandleArray[instanceId].coreCustPwrSeqNeeded) &&
        (PBIST_TestHandleArray[instanceId].tisciProcId != 0u))
    {
        status = customPrepareForPowerUpSequence(PBIST_TestHandleArray[instanceId].tisciProcId);
        if (status != SDL_PASS)
        {
            DebugP_log("  Custom core power restore sequence, ProcId 0x%x ...FAILED \r\n",
                        PBIST_TestHandleArray[instanceId].tisciProcId);
            testResult = -1;
        }
    }

    /* Take Primary core out of local reset */
    if ((testResult == 0) && (PBIST_TestHandleArray[instanceId].procRstNeeded)
                          && (PBIST_TestHandleArray[instanceId].tisciDeviceId != 0U))
    {
#ifdef DEBUG
        DebugP_log("  Primary core: Taking out of local reset the core %s \r\n",
                    PBIST_TestHandleArray[instanceId].coreName);
#endif
        status = Sciclient_pmSetModuleRst(PBIST_TestHandleArray[instanceId].tisciDeviceId,
                                          0x0, /* Local Reset de-asserted */
                                          SystemP_WAIT_FOREVER);
        if (status != SDL_PASS)
        {
             DebugP_log("  Sciclient_pmSetModuleRst 0x%x ...FAILED \r\n",
                         PBIST_TestHandleArray[instanceId].tisciDeviceId);
             testResult = -1;
        }
    }

    /* Take Secondary core out of local reset */
    if ((testResult == 0) && (PBIST_TestHandleArray[instanceId].procRstNeeded)
                          && (PBIST_TestHandleArray[instanceId].tisciSecDeviceId != 0U))
    {
#ifdef DEBUG
        DebugP_log("  Secondary core: Taking out of local reset the core %s \r\n",
                    PBIST_TestHandleArray[instanceId].secCoreName);
#endif
        status = Sciclient_pmSetModuleRst(PBIST_TestHandleArray[instanceId].tisciSecProcId,
                                          0x0, /* Local Reset de-asserted */
                                          SystemP_WAIT_FOREVER);
        if (status != SDL_PASS)
        {
             DebugP_log("  Sciclient_pmSetModuleRst 0x%x ...FAILED \r\n",
                         PBIST_TestHandleArray[instanceId].tisciSecDeviceId);
             testResult = -1;
        }
    }
#endif /* #ifdef PBIST_POWERUP_CORES_BEFORE_TEST */

    /* Ensure that cores have been turned off */
    if ((testResult == 0) && (PBIST_TestHandleArray[instanceId].procRstNeeded))
    {
        if (PBIST_TestHandleArray[instanceId].tisciDeviceId != 0u)
        {
            /* Set Software Reset Disable State for Primary core */
#ifdef DEBUG
            DebugP_log("  %s: Primary core: Put in Software Reset Disable \r\n",
                        PBIST_TestHandleArray[instanceId].coreName);
#endif
            status =  Sciclient_pmSetModuleState(PBIST_TestHandleArray[instanceId].tisciDeviceId,
                                                 TISCI_MSG_VALUE_DEVICE_SW_STATE_AUTO_OFF,
                                                 TISCI_MSG_FLAG_AOP,
                                                 SystemP_WAIT_FOREVER);
            if (status != SDL_PASS)
            {
                DebugP_log("   Primary core: Sciclient_pmSetModuleState...FAILED \r\n");
                testResult = -1;
            }
        }
    }

    if ((testResult == 0) && (PBIST_TestHandleArray[instanceId].procRstNeeded))
    {
        if ((PBIST_TestHandleArray[instanceId].secondaryCoreNeeded)
            && (PBIST_TestHandleArray[instanceId].tisciSecDeviceId != 0u))
        {
            /* Set Software Reset Disable State for Secondary core */
#ifdef DEBUG
            DebugP_log("  %s: Secondary Core Put in Software Reset Disable \r\n",
                        PBIST_TestHandleArray[instanceId].secCoreName);
#endif
            status =  Sciclient_pmSetModuleState(PBIST_TestHandleArray[instanceId].tisciSecDeviceId,
                                                 TISCI_MSG_VALUE_DEVICE_SW_STATE_AUTO_OFF,
                                                 TISCI_MSG_FLAG_AOP,
                                                 SystemP_WAIT_FOREVER);
            if (status != SDL_PASS)
            {
                DebugP_log("   Secondary core Sciclient_pmSetModuleState...FAILED \r\n");
                testResult = -1;
            }
        }
    }

    if ((testResult == 0) && (PBIST_TestHandleArray[instanceId].tisciProcId != 0u)
            && (PBIST_TestHandleArray[instanceId].procRstNeeded))
    {
        /* release processor Primary core */
#ifdef DEBUG
        DebugP_log("  Primary core: Releasing %s \r\n",
                    PBIST_TestHandleArray[instanceId].coreName);
#endif

        status = Sciclient_procBootReleaseProcessor(PBIST_TestHandleArray[instanceId].tisciProcId,
                                                    TISCI_MSG_FLAG_AOP,
                                                    SystemP_WAIT_FOREVER);
        if (status != SDL_PASS)
        {
            DebugP_log("   Primary core: Sciclient_procBootReleaseProcessor, ProcId 0x%x...FAILED \r\n",
                        PBIST_TestHandleArray[instanceId].tisciProcId);
            testResult = -1;
        }
    }

    if (testResult == 0)
    {
        if ((PBIST_TestHandleArray[instanceId].secondaryCoreNeeded)
            && (PBIST_TestHandleArray[instanceId].tisciSecProcId != 0u)
            && (PBIST_TestHandleArray[instanceId].procRstNeeded))
        {
            /* release processor Secondary core */
#ifdef DEBUG
            DebugP_log("  Secondary core: Releasing %s \r\n",
                        PBIST_TestHandleArray[instanceId].secCoreName);
#endif
            status = Sciclient_procBootReleaseProcessor(PBIST_TestHandleArray[instanceId].tisciSecProcId,
                                                        TISCI_MSG_FLAG_AOP,
                                                        SystemP_WAIT_FOREVER);
            if (status != SDL_PASS)
            {
                DebugP_log("   Secondary core: Sciclient_procBootReleaseProcessor, ProcId 0x%x...FAILED \r\n",
                            PBIST_TestHandleArray[instanceId].tisciSecProcId);
                testResult = -1;
            }
        }
    }
#endif

#ifdef SDL_SOC_MCU_R5F
    if (runNegTest == true)
    {
    #if defined (SOC_AM273X) || defined (SOC_AWR294X)
    #if defined (R5F0_INPUTS) || defined (R5F1_INPUTS)
        if (gInst == SDL_PBIST_INST_TOP)
        {
            DebugP_log("\r\n Starting PBIST failure insertion test on TOP PBIST\r\n",
                        PBIST_TestHandleArray[instanceId].testName,
                        instanceId);
        }
        else if (gInst == SDL_PBIST_INST_DSS)
        {
            DebugP_log("\r\n Starting PBIST failure insertion test on DSS PBIST\r\n",
                    PBIST_TestHandleArray[instanceId].testName,
                    instanceId);
        }

    #endif
    #elif defined (SOC_AM263X)

        {
            DebugP_log("\r\n Starting PBIST failure insertion test on TOP PBIST\r\n",
                        PBIST_TestHandleArray[instanceId].testName,
                        instanceId);
        }

    #endif
        testType = SDL_PBIST_NEG_TEST;
    }
    else
    {
    #if defined (SOC_AM273X) || defined (SOC_AWR294X)
        if (gInst == SDL_PBIST_INST_TOP)
        {
            DebugP_log("\r\n Starting PBIST test on TOP PBIST\r\n",
                        PBIST_TestHandleArray[instanceId].testName,
                        instanceId);
        }
        else if (gInst == SDL_PBIST_INST_DSS)
        {
            DebugP_log("\r\n Starting PBIST test on DSS PBIST\r\n",
                        PBIST_TestHandleArray[instanceId].testName,
                        instanceId);
        }
    #elif defined (SOC_AM263X)
        {
            DebugP_log("\r\n Starting PBIST test on TOP PBIST\r\n",
                        PBIST_TestHandleArray[instanceId].testName,
                        instanceId);
        }
    #endif
        testType = SDL_PBIST_TEST;
    }

#ifdef DEBUG
    DebugP_log("\r\n Press any key to continue...");
    inputChar = UART_getChar();

    if (inputChar == 'n')
    {
        DebugP_log("\r\n Skipping this test. on request \r\n");
        return 0;
    }
#endif

    /* Get start time of test */
    startTime = ClockP_getTimeUsec();
#ifndef SDL_SOC_MCU_R5F
    if (testResult == 0)
    {
#endif
        /* Get start time for PBIST test */
        testStartTime = ClockP_getTimeUsec();

        status = SDL_PBIST_selfTest((SDL_PBIST_inst)PBIST_TestHandleArray[instanceId].pbistInst, testType, PBIST_APP_TIMEOUT, &PBISTResult);
        if ((status != SDL_PASS) || (PBISTResult == false))
        {
            testResult = -1;
        }
#ifndef SDL_SOC_MCU_R5F
    }
#endif

    /* Record test end time */
    testEndTime = ClockP_getTimeUsec();


#endif

    /* Record end time */
    endTime = ClockP_getTimeUsec();

    prepTime = testStartTime - startTime;
    diffTime = testEndTime - testStartTime;
    restoreTime = endTime - testEndTime;
    DebugP_log("  Delta Cores prep time in micro secs %d \r\n", (uint32_t)prepTime );
    DebugP_log("  Delta PBIST execution time in micro secs %d \r\n", (uint32_t)diffTime );
    DebugP_log("  Delta Cores restore time in micro secs %d \r\n", (uint32_t)restoreTime );
#if defined (SOC_AM64X) || defined (SOC_AM243X)
    if (testResult == 0)
    {
        instanceTime = prepTime + diffTime + restoreTime;
        PBIST_profilingTime += instanceTime;
        DebugP_log("  Total PBIST time for %s in micro secs %u \r\n", PBIST_TestHandleArray[instanceId].testName, (uint32_t)instanceTime );
    }
#endif
    DebugP_log(" PBIST complete %s, test index %d\r\n",
                PBIST_TestHandleArray[instanceId].testName,
                instanceId);
#if defined (SOC_AWR294X)
    if((status == SDL_PASS) && (testType == SDL_PBIST_TEST))
    {
      #if defined (R5F0_INPUTS)
        if (gInst == SDL_PBIST_INST_TOP)
        {

            DebugP_log(" PBIST complete for ADCBUF\r\n");
            DebugP_log(" PBIST complete for TPCC\r\n");
            DebugP_log(" PBIST complete for MAILBOX\r\n");
            DebugP_log(" PBIST complete for COREB VIM\r\n");
            DebugP_log(" PBIST complete for MCAN\r\n");
            DebugP_log(" PBIST complete for SPIA\r\n");
            DebugP_log(" PBIST complete for SPIB\r\n");
            DebugP_log(" PBIST complete for CORE B R5FSS RAM\r\n");
            DebugP_log(" PBIST complete for MSS_L2_1\r\n");
            DebugP_log(" PBIST complete for CPSW\r\n");
            DebugP_log(" PBIST complete for GPADC\r\n");
            DebugP_log(" PBIST complete for RETRAM\r\n");
            DebugP_log(" PBIST complete for STCROM\r\n");
            DebugP_log(" PBIST complete for CORE B ATCM\r\n");
            DebugP_log(" PBIST complete for CORE B BTCM\r\n");

        }
        else if (gInst == SDL_PBIST_INST_DSS)
        {
            DebugP_log(" PBIST complete for DSS C66 STCROM\r\n");
            DebugP_log(" PBIST complete for HWA STCROM\r\n");
            DebugP_log(" PBIST complete for DSS PBISTROM\r\n");
            DebugP_log(" PBIST complete for C66 L1D\r\n");
            DebugP_log(" PBIST complete for C66 L1P\r\n");
            DebugP_log(" PBIST complete for PBIST C66 L2 TAG\r\n");
            DebugP_log(" PBIST complete for DSS HWA\r\n");
            DebugP_log(" PBIST complete for DSS HWA MBOX\r\n");
            DebugP_log(" PBIST complete for PBIST DSS L3 BANKA SUB0\r\n");
            DebugP_log(" PBIST complete for PBIST DSS L3 BANKB SUB0\r\n");
            DebugP_log(" PBIST complete for PBIST DSS L3 BANKC SUB0\r\n");
            DebugP_log(" PBIST complete for DSS MBOX RAM\r\n");
            DebugP_log(" PBIST complete for DSS TPCC RAM\r\n");
            DebugP_log(" PBIST complete for DSS L2 BANK0\r\n");
            DebugP_log(" PBIST complete for DSS L2 BANK1\r\n");
            DebugP_log(" PBIST complete for DSS L2 PARITY\r\n");
            DebugP_log(" PBIST complete for HWA RAM\r\n");
            DebugP_log(" PBIST complete for DSS CBUF\r\n");
            DebugP_log(" PBIST complete for PBIST DSS L3 BANKA SUB1\r\n");
            DebugP_log(" PBIST complete for PBIST DSS L3 BANKB SUB1\r\n");
            DebugP_log(" PBIST complete for PBIST DSS L3 BANKB SUB2\r\n");
            DebugP_log(" PBIST complete for PBIST DSS L3 BANKC SUB1\r\n");
        }
        if (testResult == SDL_PASS)
        {
            DebugP_log("\r\r\nAll tests have passed. \r\r\n");
        }
        else
        {
            DebugP_log("\r\r\nSome tests have failed. \r\r\n");
        }

    #elif defined (R5F1_INPUTS)
    {
        DebugP_log(" PBIST complete for MSS_TCMAROM_0\r\n");
        DebugP_log(" PBIST complete for MSS_TCMAROM_1\r\n");
        DebugP_log(" PBIST complete for PBISTROM\r\n");
        DebugP_log(" PBIST complete for CORE A VIM\r\n");
        DebugP_log(" PBIST complete for MSS_L2_0\r\n");
        DebugP_log(" PBIST complete for CORE A ATCM\r\n");
        DebugP_log(" PBIST complete for CORE A BTCM\r\n");
        DebugP_log(" PBIST complete for CORE A R5SS RAM\r\n");
        DebugP_log(" PBIST complete for MEM_TOP_AURORA\r\n");
        DebugP_log(" PBIST complete for MEM_TOP_MDO\r\n");
        DebugP_log(" PBIST complete for DBGSS_TRACE\r\n");
    }
    #endif
  }
#endif
#if defined (SOC_AM273X)
    if((status == SDL_PASS) && (testType == SDL_PBIST_TEST))
    {
        #if defined (R5F0_INPUTS)
        if (gInst == SDL_PBIST_INST_TOP)
        {
            DebugP_log(" PBIST complete for TPCC\r\n");
            DebugP_log(" PBIST complete for MAILBOX\r\n");
            DebugP_log(" PBIST complete for COREB VIM\r\n");
            DebugP_log(" PBIST complete for MCAN\r\n");
            DebugP_log(" PBIST complete for SPIA\r\n");
            DebugP_log(" PBIST complete for SPIB\r\n");
            DebugP_log(" PBIST complete for CORE B R5FSS RAM\r\n");
            DebugP_log(" PBIST complete for MSS_L2_1\r\n");
            DebugP_log(" PBIST complete for CPSW\r\n");
            DebugP_log(" PBIST complete for GPADC\r\n");
            DebugP_log(" PBIST complete for RETRAM\r\n");
            DebugP_log(" PBIST complete for STCROM\r\n");
            DebugP_log(" PBIST complete for CORE B ATCM\r\n");
            DebugP_log(" PBIST complete for CORE B BTCM\r\n");

        }
        else if (gInst == SDL_PBIST_INST_DSS)
        {
            DebugP_log(" PBIST complete for DSS C66 STCROM\r\n");
            DebugP_log(" PBIST complete for HWA STCROM\r\n");
            DebugP_log(" PBIST complete for DSS PBISTROM\r\n");
            DebugP_log(" PBIST complete for C66 L1D\r\n");
            DebugP_log(" PBIST complete for C66 L1P\r\n");
            DebugP_log(" PBIST complete for PBIST C66 L2 TAG\r\n");
            DebugP_log(" PBIST complete for DSS HWA\r\n");
            DebugP_log(" PBIST complete for DSS HWA MBOX\r\n");
            DebugP_log(" PBIST complete for PBIST DSS L3 BANKA SUB0\r\n");
            DebugP_log(" PBIST complete for PBIST DSS L3 BANKB SUB0\r\n");
            DebugP_log(" PBIST complete for PBIST DSS L3 BANKC SUB0\r\n");
            DebugP_log(" PBIST complete for DSS MBOX RAM\r\n");
            DebugP_log(" PBIST complete for DSS TPCC RAM\r\n");
            DebugP_log(" PBIST complete for DSS L2 BANK0\r\n");
            DebugP_log(" PBIST complete for DSS L2 BANK1\r\n");
            DebugP_log(" PBIST complete for DSS L2 PARITY\r\n");
            DebugP_log(" PBIST complete for HWA RAM\r\n");
            DebugP_log(" PBIST complete for DSS CBUF\r\n");
            DebugP_log(" PBIST complete for PBIST DSS L3 BANKA SUB1\r\n");
            DebugP_log(" PBIST complete for PBIST DSS L3 BANKB SUB1\r\n");
            DebugP_log(" PBIST complete for PBIST DSS L3 BANKB SUB2\r\n");
            DebugP_log(" PBIST complete for PBIST DSS L3 BANKC SUB1\r\n");
        }
        if (testResult == SDL_PASS)
        {
            DebugP_log("\r\r\nAll tests have passed. \r\r\n");
        }
        else
        {
            DebugP_log("\r\r\nSome tests have failed. \r\r\n");
        }

    #elif defined (R5F1_INPUTS)
    {
        DebugP_log(" PBIST complete for MSS_TCMAROM_0\r\n");
        DebugP_log(" PBIST complete for MSS_TCMAROM_1\r\n");
        DebugP_log(" PBIST complete for PBISTROM\r\n");
        DebugP_log(" PBIST complete for CORE A VIM\r\n");
        DebugP_log(" PBIST complete for MSS_L2_0\r\n");
        DebugP_log(" PBIST complete for CORE A ATCM\r\n");
        DebugP_log(" PBIST complete for CORE A BTCM\r\n");
        DebugP_log(" PBIST complete for CORE A R5SS RAM\r\n");
        DebugP_log(" PBIST complete for MEM_TOP_AURORA\r\n");
        DebugP_log(" PBIST complete for MEM_TOP_MDO\r\n");
        DebugP_log(" PBIST complete for DBGSS_TRACE\r\n");
    }
    #endif
  }
#endif
#if defined (SOC_AM263X)
#if defined (R5F0_INPUTS)
    if((status == SDL_PASS) && (testType == SDL_PBIST_TEST))
    {
        DebugP_log(" PBIST complete for R5 STC\r\n");
        DebugP_log(" PBIST complete for R51 STC\r\n");
        DebugP_log(" PBIST complete for PBISTROM\r\n");
        DebugP_log(" PBIST complete for CPSW\r\n");
        DebugP_log(" PBIST complete for ICSSM\r\n");
        DebugP_log(" PBIST complete for MBOX\r\n");
        DebugP_log(" PBIST complete for MCAN\r\n");
        DebugP_log(" PBIST complete for TPCC\r\n");
        DebugP_log(" PBIST complete for MSS_L2_1\r\n");
        DebugP_log(" PBIST complete for MSS_L2_2\r\n");
        DebugP_log(" PBIST complete for MSS_L2_3\r\n");
        DebugP_log(" PBIST complete for VIM1 R5SS0\r\n");
        DebugP_log(" PBIST complete for VIM0 R5SS1\r\n");
        DebugP_log(" PBIST complete for VIM1 R5SS1\r\n");
        DebugP_log(" PBIST complete for R5SS1 RAM\r\n");
        DebugP_log(" PBIST complete for MSS CR5B ATCM0\r\n");
        DebugP_log(" PBIST complete for MSS CR5B ATCM1\r\n");
        DebugP_log(" PBIST complete for MSS CR5B BTCM0\r\n");
        DebugP_log(" PBIST complete for MSS CR5B BTCM1\r\n");
    }
#elif defined (R5F1_INPUTS)
    if((status == SDL_PASS) && (testType == SDL_PBIST_TEST))
    {
        DebugP_log(" PBIST complete for  VIM0 R5SS0\r\n");
        DebugP_log(" PBIST complete for MSS_L2_0\r\n");
        DebugP_log(" PBIST complete for R5SS0 RAM\r\n");
        DebugP_log(" PBIST complete for CR5A ROM0\r\n");
        DebugP_log(" PBIST complete for TRACE\r\n");
        DebugP_log(" PBIST complete for MMCH0\r\n");
        DebugP_log(" PBIST complete for MSS CR5A ATCM0\r\n");
        DebugP_log(" PBIST complete for MSS CR5A ATCM1\r\n");
        DebugP_log(" PBIST complete for MSS CR5A BTCM0\r\n");
        DebugP_log(" PBIST complete for MSS CR5A BTCM1\r\n");

    }
#endif
#endif

#if defined (SOC_AM263PX)
    if((status == SDL_PASS) && (testType == SDL_PBIST_TEST))
    {
        {
            DebugP_log(" PBIST complete for R5 STC\r\n");
            DebugP_log(" PBIST complete for R51 STC\r\n");
            DebugP_log(" PBIST complete for R50 TMU1\r\n");
            DebugP_log(" PBIST complete for R50 TMU2\r\n");
            DebugP_log(" PBIST complete for R50 TMU3\r\n");
            DebugP_log(" PBIST complete for R50 TMU4\r\n");
            DebugP_log(" PBIST complete for R50 TMU5\r\n");
            DebugP_log(" PBIST complete for R50 TMU6\r\n");
            DebugP_log(" PBIST complete for R51 TMU1\r\n");
            DebugP_log(" PBIST complete for R51 TMU2\r\n");
            DebugP_log(" PBIST complete for R51 TMU3\r\n");
            DebugP_log(" PBIST complete for R51 TMU4\r\n");
            DebugP_log(" PBIST complete for R51 TMU5\r\n");
            DebugP_log(" PBIST complete for R51 TMU6\r\n");
            DebugP_log(" PBIST complete for ROM0\r\n");
            DebugP_log(" PBIST complete for ROM1\r\n");
            DebugP_log(" PBIST complete for CPSW\r\n");
            DebugP_log(" PBIST complete for ECU_PERIPH\r\n");
            DebugP_log(" PBIST complete for FOTA\r\n");
            DebugP_log(" PBIST complete for ICSSM\r\n");
            DebugP_log(" PBIST complete for MBOX\r\n");
            DebugP_log(" PBIST complete for MSS_L2_2\r\n");
            DebugP_log(" PBIST complete for MSS_L2_3\r\n");
            DebugP_log(" PBIST complete for MSS_L2_4\r\n");
            DebugP_log(" PBIST complete for MSS_L2_5\r\n");
            DebugP_log(" PBIST complete for TPCC\r\n");
            DebugP_log(" PBIST complete for OSPI\r\n");
            DebugP_log(" PBIST complete for MSS R5SS0 C0\r\n");
            DebugP_log(" PBIST complete for MSS R5SS0 C1\r\n");
            DebugP_log(" PBIST complete for MSS R5SS1 C0\r\n");
            DebugP_log(" PBIST complete for MSS R5SS1 C1\r\n");
        }
        if (testResult == SDL_PASS)
        {
            DebugP_log("\r\nAll tests have passed. \r\n");
        }
        else
        {
            DebugP_log("\r\nSome tests have failed. \r\n");
        }
    }
#endif

#if defined (SOC_AM261X)
    if((status == SDL_PASS) && (testType == SDL_PBIST_TEST))
    {
        {
            DebugP_log(" PBIST complete for R5 STC\r\n");
            DebugP_log(" PBIST complete for R50 TMU1\r\n");
            DebugP_log(" PBIST complete for R50 TMU2\r\n");
            DebugP_log(" PBIST complete for R50 TMU3\r\n");
            DebugP_log(" PBIST complete for R50 TMU4\r\n");
            DebugP_log(" PBIST complete for R50 TMU5\r\n");
            DebugP_log(" PBIST complete for R50 TMU6\r\n");
            DebugP_log(" PBIST complete for PBISTROM\r\n");
            DebugP_log(" PBIST complete for ROM0\r\n");
            DebugP_log(" PBIST complete for ROM1\r\n");
            DebugP_log(" PBIST complete for ICSSM0\r\n");
            DebugP_log(" PBIST complete for ICSSM1\r\n");
            DebugP_log(" PBIST complete for CPSW\r\n");
            DebugP_log(" PBIST complete for ECU_PERIPH\r\n");
            DebugP_log(" PBIST complete for FOTA\r\n");
            DebugP_log(" PBIST complete for MBOX\r\n");
            DebugP_log(" PBIST complete for TPCC\r\n");
            DebugP_log(" PBIST complete for OSPI\r\n");
            DebugP_log(" PBIST complete for MSS R5SS0 RL2 CPU0\r\n");
            DebugP_log(" PBIST complete for MSS R5SS0 RL2 CPU1\r\n");
            DebugP_log(" PBIST complete for MSS TRACE\r\n");
            DebugP_log(" PBIST complete for MSS USB\r\n");
            DebugP_log(" PBIST complete for MSS R5SS0 C0\r\n");
            DebugP_log(" PBIST complete for MSS R5SS0 C1\r\n");
            DebugP_log(" PBIST complete for MSS_L2_1\r\n");
            DebugP_log(" PBIST complete for MSS_L2_2\r\n");
            DebugP_log(" PBIST complete for MSS MMCH0\r\n");
            DebugP_log(" PBIST complete for MSS MMCH1\r\n");
        }
        if (testResult == SDL_PASS)
        {
            DebugP_log("\r\nAll tests have passed. \r\n");
        }
        else
        {
            DebugP_log("\r\nSome tests have failed. \r\n");
        }
    }
#endif

#ifdef SDL_SOC_MCU_R5F
    if((status == SDL_PASS) && (testType == SDL_PBIST_NEG_TEST))
    {
        DebugP_log(" PBIST failure Insertion test complete for TOP BIST\r\n");
    }
#endif

    return (testResult);
}

/* PBIST Functional test */
int32_t PBIST_funcTest(void)
{
    int32_t    testResult = 0;
#if defined (SOC_AM64X) || defined (SOC_AM243X)
    uint64_t totalProfTime = 0;
#endif

    testResult = PBIST_commonInit();

    if (testResult != 0)
    {
        DebugP_log("  PBIST_commonInit ...FAILED \r\n");
    }
    else
    {
#ifndef SDL_SOC_MCU_R5F
        /* Run the test for diagnostics first */
        for (uint32_t i = 0; i < PBIST_NUM_INSTANCE; i++)
        {
            /* Run test on selected instance */
            testResult = PBIST_runTest(i, true);
            if ( testResult != 0)
            {
                break;
            }
        }

        if (testResult == 0)
        {
        #if defined (SOC_AM64X) || defined (SOC_AM243X)
            DebugP_log("\r\n Total time for PBIST negative tests in microseconds: %llu\r\n", PBIST_profilingTime);
            totalProfTime += PBIST_profilingTime;
            PBIST_profilingTime = 0;
        #endif
            /* Then run the pbist test */
            for (uint32_t i = 0; i < PBIST_NUM_INSTANCE; i++)
            {
                /* Run test on selected instance */
                testResult = PBIST_runTest(i, false);
                if ( testResult != 0)
                {
                    break;
                }
            }
        #if defined (SOC_AM64X) || defined (SOC_AM243X)
            if (testResult == 0)
            {
                totalProfTime += PBIST_profilingTime;
                DebugP_log("\r\n Total time for PBIST positive tests in microseconds: %llu\r\n", PBIST_profilingTime);
                DebugP_log("\r\n Total time for PBIST test runs in microseconds: %llu\r\n", totalProfTime);
            }
        #endif
        }
#else
        /* Run the test for diagnostics first */
        /* Run test on selected instance */
        testResult = PBIST_runTest(SDL_PBIST_INST_TOP, true);

        if (testResult == 0)
        {
          /* Run test on selected instance */

          testResult = PBIST_runTest(SDL_PBIST_INST_TOP, false);
        }
        #if defined (SOC_AM273X) || (SOC_AWR294X)
        #if defined R5F0_INPUTS
        gInst = SDL_PBIST_INST_DSS;
        /* Run the test for diagnostics first */
        /* Run test on selected instance */
            testResult = PBIST_runTest(SDL_PBIST_INST_DSS, true);

            if (testResult == 0)
            {
              /* Run test on selected instance */
              testResult = PBIST_runTest(SDL_PBIST_INST_DSS, false);
            }
          #endif
          #endif
#endif
      }

    return (testResult);
}
/* Nothing past this point */
