/*
 *  Copyright (C) 2023 Texas Instruments Incorporated
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
 *  \file     pbist_test_func.c
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
#include <sdl/pbist/sdl_pbist_priv.h>
#ifndef SDL_SOC_MCU_R5F
#include <drivers/sciclient.h>
#include "power_seq.h"
#endif

/* DPL API header files */
#include <kernel/dpl/HwiP.h>
#include <kernel/dpl/ClockP.h>
#include <kernel/dpl/AddrTranslateP.h>
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
#define PBIST_PSC_BASE_ADDR             ((uint32_t)0x400A00)

/* The following macro gives the PSC register address of a device */
#define PBIST_A53_0_ADDR                (PBIST_PSC_BASE_ADDR + 4*CSL_MAIN_LPSC_A53_0)

/* The following macros will be used in modifying PSC register values */
#define PBIST_PSC_NEXT_MASK             (0x0000003F)
#define PBIST_PSC_FORCE_OFF             (0x80000001)
#define PBIST_PSC_PTCMD_ADDR            (0x400120)
#define PBIST_PSC_PTCMD_TIMEOUT         (1000000U)
#define PBIST_PSC_PTSTAT_ADDR           (0x400128)
#endif


/* ========================================================================== */
/*                 Internal Function Declarations                             */
/* ========================================================================== */
void PBIST_eventHandler( uint32_t instanceId );
volatile int32_t PBIST_PSCForceOff(uint32_t pscAddr);

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */
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

int32_t PBIST_runTest(uint32_t instanceId, bool runNegTest)
{

    uint32_t resetState = 0U;
    uint32_t contextLossState = 0U;
    uint32_t moduleState = TISCI_MSG_VALUE_DEVICE_HW_STATE_OFF;
    int32_t testResult = 0;
    SDL_ErrType_t status;
    HwiP_Params hwiParams;
    HwiP_Object PBIST_hwiPObj;
    SDL_pbistRegs *pPBISTRegs;
    bool PBISTResult;
	SDL_PBIST_testType testType;
    uint64_t startTime , testStartTime,  testEndTime, endTime;
    uint64_t prepTime, diffTime, restoreTime;
    int i;
#if defined(SOC_AM64X)
    uint32_t pscAddr;
    bool a53Skip = false;
#endif
#ifdef DEBUG
    char inputChar;
#endif

    uint32_t numRuns = 0;

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

    if (PBIST_TestHandleArray[instanceId].interruptNumber != SDL_PBIST_INTERRUPT_INVALID)
    {
        /* Disable interrupt */
        HwiP_disableInt(PBIST_TestHandleArray[instanceId].interruptNumber);

        /* Default parameter initialization */
        HwiP_Params_init(&hwiParams);

        /* Pass core Index as argument to handler*/
        hwiParams.args = (void *)instanceId;
        hwiParams.intNum = PBIST_TestHandleArray[instanceId].interruptNumber;
        hwiParams.callback = (HwiP_FxnCallback)PBIST_eventHandler;
#ifdef DEBUG
        DebugP_log("\r\n HwiP_Params_init complete \r\n");
#endif
        /* Register call back function for PBIST Interrupt */
        HwiP_construct(&PBIST_hwiPObj, &hwiParams);
    }

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
        DebugP_log(
                        "  Double checking Powering on Device number %d Device Id %x\r\n",
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
        DebugP_log(
                        "  Primary core: Double checking Powering on %s \r\n",
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
            DebugP_log(
                            "  Secondary core: Double checking Powering on %s \r\n",
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

    /* Get PBIST register space Pointer */
    pPBISTRegs = PBIST_TestHandleArray[instanceId].pPBISTRegs;

    if (testResult == 0)
    {
        /* Translate to get the local address */
        pPBISTRegs = (SDL_pbistRegs *)AddrTranslateP_getLocalAddr((uint64_t)pPBISTRegs);
    }
    PBIST_TestHandleArray[instanceId].doneFlag = false;

    /* Get start time for PBIST test */
    testStartTime = ClockP_getTimeUsec();

    if (runNegTest == true)
    {
        numRuns = 1;
    }
    else
    {
        numRuns = PBIST_TestHandleArray[instanceId].numPBISTRuns;
    }

    /* Start the PBIST test */
    for (i = 0; i < numRuns; i++)
    {
#ifdef DEBUG
        DebugP_log("\r\n Starting PBIST Run %d for Instance ID #%d\r\n",
                    i, instanceId);
#endif
        if (testResult == 0)
        {
#ifdef DEBUG
            DebugP_log("\r\n Starting %s in PBIST Run %d\r\n",
                        runNegTest ? "SDL_PBIST_selfTest for Negative test" : "SDL_PBIST_selfTest for Positive test", i);
#endif
            if (runNegTest == true)
            {
                status = SDL_PBIST_selfTest((SDL_PBIST_inst)PBIST_TestHandleArray[instanceId].pbistInst, testType, PBIST_APP_TIMEOUT, &PBISTResult);
            }
            else
            {
                status = SDL_PBIST_selfTest((SDL_PBIST_inst)PBIST_TestHandleArray[instanceId].pbistInst, testType, PBIST_APP_TIMEOUT, &PBISTResult);
            }
            if (status != SDL_PASS)
            {
                DebugP_log(" %s failed in PBIST Run %d\r\n",
                            runNegTest ? "SDL_PBIST_selfTest for Negative test" : "SDL_PBIST_selfTest for Positive test", i);
                testResult = -1;
            }
        }

        /* Do a Soft Reset */
        if (testResult == 0)
        {
#ifdef DEBUG
            DebugP_log("\r\n Starting SDL_PBIST_softReset \r\n");
#endif

            /* Run PBIST test */
            status = SDL_PBIST_softReset(pPBISTRegs);
            if (status != SDL_PASS)
            {
                DebugP_log(" SDL_PBIST_softReset failed \r\n");
                testResult = -1;
            }
        }

        /* Execute exit sequence */
        if (testResult == 0)
        {
#ifdef DEBUG
            DebugP_log("\r\n Starting SDL_PBIST_releaseTestMode \r\n");
#endif
            /* Exit PBIST test */
            status = SDL_PBIST_releaseTestMode(pPBISTRegs);
            if (status != SDL_PASS)
            {
                DebugP_log(" SDL_PBIST_releaseTestMode failed \r\n");
                testResult = -1;
            }
        }
    } /* for (i = 0; i < PBIST_TestHandleArray[instanceId].numPBISTRuns; i++) */

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
    /* Record end time */
    endTime = ClockP_getTimeUsec();

    prepTime = testStartTime - startTime;
    diffTime = testEndTime - testStartTime;
    restoreTime = endTime - testEndTime;
    DebugP_log("  Delta Cores prep time in micro secs %d \r\n", (uint32_t)prepTime );
    DebugP_log("  Delta PBIST execution time in micro secs %d \r\n", (uint32_t)diffTime );
    DebugP_log("  Delta Cores restore time in micro secs %d \r\n", (uint32_t)restoreTime );
    DebugP_log(" PBIST complete %s, test index %d\r\n",
                PBIST_TestHandleArray[instanceId].testName,
                instanceId);
    return (testResult);
}


/* PBIST Functional test */
int32_t PBIST_funcTest(void)
{
    int32_t    testResult = 0;

    testResult = PBIST_commonInit();

    if (testResult != 0)
    {
        DebugP_log("  PBIST_commonInit ...FAILED \r\n");
    }
    else
    {

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
        }
    }

    return (testResult);
}
/* Nothing past this point */
