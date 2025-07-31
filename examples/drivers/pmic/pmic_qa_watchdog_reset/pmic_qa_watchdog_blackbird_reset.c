/*
 *  Copyright (C) 2025 Texas Instruments Incorporated
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

/**
 *  \file pmic_qa_watchdog_blackbird_reset.c
 *
 *  \brief This is a PMIC watchdog example for Q&A mode which will generate
 *         a reset when MCU fails to send a signal to the watchdog.
 */

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <stdio.h>
#include <kernel/dpl/DebugP.h>
#include "ti_drivers_config.h"
#include "ti_drivers_open_close.h"
#include "ti_board_open_close.h"
#include <drivers/hw_include/cslr_soc.h>

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

#define PMIC_WDG_WINDOW_DELAY       (71U)
#define PMIC_WDG_TIMEOUT_DELAY      (3000U)
#define PMIC_REG_STATE_LOCK         (1U)

/* ========================================================================== */
/*                         Structure Declarations                             */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

static int32_t PMICApp_isWdogResetDone(Pmic_CoreHandle_t *coreHandle);
static int32_t PMICApp_wdogQAModeReset(Pmic_CoreHandle_t *coreHandle);
static int32_t PMICApp_checkForWdgErrors(Pmic_CoreHandle_t *handle, const char *str);

static void PMICApp_wait_ms(uint16_t milliseconds);
static void PMICApp_checkIfWdgConfigurable(Pmic_CoreHandle_t *pmicHandle);

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */


void pmic_qa_watchdog_reset_main(void *args)
{
    Pmic_CoreHandle_t* handle;
    int32_t status = PMIC_ST_SUCCESS;
    int32_t pmicStatus = PMIC_ST_SUCCESS;

    /* Open drivers to open the UART driver for console */
    Drivers_open();
    Board_driversOpen();

    /* PMIC interface handle initialized by PMIC_open */
    handle = PMIC_getCoreHandle(CONFIG_PMIC0);
    DebugP_assert(NULL != handle);

    /* Check if the example has already run and core is reset due to watchdog
     * error warm reset. If yes, print the success message and exit the program.
     * Otherwise, continue executing the program to trigger a watchdog reset.
     * This check is for testing purpose, actual application may not require it.
     */
    if(PMICApp_isWdogResetDone(handle) == PMIC_ST_SUCCESS)
    {
        DebugP_log("Warm reset has occurred due to watchdog failure!! \r\n");
        pmicStatus = Pmic_wdgSetReturnToLongWindow(handle, (bool)TRUE);
        pmicStatus += Pmic_wdgSetPowerHold(handle, (bool)TRUE);
        if (pmicStatus ==  PMIC_ST_SUCCESS)
        {
            pmicStatus += Pmic_wdgClrErrStatusAll(handle);
            if (pmicStatus != PMIC_ST_SUCCESS)
            {
                status = SystemP_FAILURE;
            }
        }
    }
    else
    {
        DebugP_log("\r\n");
        DebugP_log("Starting Q&A watchdog reset example !!\r\n");
        DebugP_log("Checking if prerequisites for configuring the WDG has been met...\r\n");
        PMICApp_checkIfWdgConfigurable(handle);
        status = PMICApp_wdogQAModeReset(handle);
    }
    
    if(PMIC_ST_SUCCESS == status)
    {
        DebugP_log("All tests have passed!!\r\n");
    }
    else
    {
        DebugP_logError ("Some tests have failed!!\r\n");
    }

    Board_driversClose();
    Drivers_close();
    return;
}

static void PMICApp_checkIfWdgConfigurable(Pmic_CoreHandle_t *pmicHandle)
{
    bool val = (bool)FALSE;
    uint8_t val1 = 0;
    int32_t status = 0;

    // Unlock PMIC registers if Locked
    status = Pmic_getRegLockState(pmicHandle, &val1);
    if(val1 == PMIC_REG_STATE_LOCK)
    {
        status = Pmic_setRegLockState(pmicHandle,PMIC_LOCK_DISABLE);
        DebugP_assert(status == PMIC_ST_SUCCESS);
    }

    // Verify PMIC configuration register lock status
    status = Pmic_getRegLockState(pmicHandle, &val1);
    DebugP_assert((status == PMIC_ST_SUCCESS) && (val1 == (uint8_t)0));

    // Enable WDG
    status = Pmic_wdgEnable(pmicHandle);
    DebugP_assert(status == PMIC_ST_SUCCESS);

    // Verify WDG enable status
    status = Pmic_wdgGetEnableState(pmicHandle, &val);
    DebugP_assert((status == PMIC_ST_SUCCESS) && (val == (bool)TRUE));

    // Enable WD_RETURN_LONGWIN so that the PMIC can return to Long Window after
    // completion of the current WDG sequence
    status = Pmic_wdgSetReturnToLongWindow(pmicHandle, (bool)TRUE);
    DebugP_assert(status == PMIC_ST_SUCCESS);

    // Verify WD_RETURN_LONGWIN is set to 1
    status = Pmic_wdgGetReturnToLongWindow(pmicHandle, &val);
    DebugP_assert((status == PMIC_ST_SUCCESS) && (val == (bool)TRUE));

    // Enable WD_PWRHOLD so that the PMIC can stay in Long Window once entered
    status = Pmic_wdgSetPowerHold(pmicHandle, (bool)TRUE);
    DebugP_assert(status == PMIC_ST_SUCCESS);

    // Verify WD_PWRHOLD is set to 1
    status = Pmic_wdgGetPowerHold(pmicHandle, &val);
    DebugP_assert((status == PMIC_ST_SUCCESS) && (val == (bool)TRUE));
}

static int32_t PMICApp_wdogQAModeReset(Pmic_CoreHandle_t* pmicHandle)
{
    int32_t        status = PMIC_ST_SUCCESS;
    int32_t        retVal = SystemP_FAILURE;
    uint16_t      numSequences = 100U;
    uint8_t       answCnt = 0;

    Pmic_WdgCfg_t wdgCfg = {
        .validParams =
            (PMIC_CFG_WDG_LONGWINDURATION_VALID_SHIFT | PMIC_CFG_WDG_WIN1DURATION_VALID_SHIFT |
             PMIC_CFG_WDG_WIN2DURATION_VALID_SHIFT | PMIC_CFG_WDG_THRESHOLD_1_VALID_SHIFT |
             PMIC_CFG_WDG_THRESHOLD_2_VALID_SHIFT | PMIC_CFG_WDG_MODE_VALID_SHIFT |
             PMIC_CFG_WDG_TIME_BASE_VALID_SHIFT | PMIC_CFG_WDG_QA_FDBK_VALID_SHIFT |
             PMIC_CFG_WDG_QA_LFSR_VALID_SHIFT | PMIC_CFG_WDG_QA_QUES_SEED_VALID_SHIFT),
        .longWinCode = 0xFFU,
        .mode = PMIC_WDG_QA_MODE,
        .timeBase = PMIC_WDG_TIME_BASE_550_US,
        .win1Code = 0x7FU,
        .win2Code = 0x7FU,
        .threshold1 = PMIC_WDG_THRESHOLD_COUNT_MAX,
        .threshold2 = PMIC_WDG_THRESHOLD_COUNT_MAX,
        .qaFdbk = PMIC_WDG_QA_FEEDBACK_VALUE_0,
        .qaLfsr = PMIC_WDG_QA_LFSR_VALUE_0,
        .qaQuesSeed = PMIC_WDG_QA_QUES_SEED_VALUE_0};

    /* Configure watchdog for Q&A mode */
    status = Pmic_wdgSetCfg(pmicHandle, &wdgCfg);
    DebugP_assert(status == PMIC_ST_SUCCESS);

    DebugP_log("Configure PMIC watchdog in Q&A mode... DONE \r\n");

    // Clear all WDG error statuses
    status = Pmic_wdgClrErrStatusAll(pmicHandle);
    DebugP_assert(status == PMIC_ST_SUCCESS);

    // Disable WD_PWRHOLD to enable PMIC to exit Long Window
    status = Pmic_wdgSetPowerHold(pmicHandle, (bool)FALSE);
    DebugP_assert(status == PMIC_ST_SUCCESS);

    // Disable WD_RETURN_LONGWIN so that PMIC does not return to Long Window at
    // the end of the next Q&A sequence
    status = Pmic_wdgSetReturnToLongWindow(pmicHandle, (bool)FALSE);
    DebugP_assert(status == PMIC_ST_SUCCESS);

    DebugP_log("Started watchdog sequence... Sending answers to the PMIC... \r\n");

    // Exit Long Window by sending all 4 answer bytes
    for (answCnt = 4U; answCnt != 0U; answCnt--)
    {
        status = Pmic_wdgQaSequenceWriteAnswer(pmicHandle);
        DebugP_assert(status == PMIC_ST_SUCCESS);
        status = PMICApp_checkForWdgErrors(pmicHandle, "LW");
        DebugP_assert(status == PMIC_ST_SUCCESS);
    }

    // Undergo Q&A sequences
    while (numSequences != 0U)
    {
        // Enter Window-1; calculate and send answer byte for
        // Answer-3, Answer-2, and Answer-1
        for (answCnt = 3U; answCnt >= 1U; answCnt--)
        {
            status = Pmic_wdgQaSequenceWriteAnswer(pmicHandle);
            DebugP_assert(status == PMIC_ST_SUCCESS);
            status = PMICApp_checkForWdgErrors(pmicHandle, "Win-1");
            DebugP_assert(status == PMIC_ST_SUCCESS);
        }

        // Wait until Window-1 time elapses
        PMICApp_wait_ms(PMIC_WDG_WINDOW_DELAY);

        // Enter Window-2; calculate and send last answer byte
        status = Pmic_wdgQaSequenceWriteAnswer(pmicHandle);
        DebugP_assert(status == PMIC_ST_SUCCESS);
        status = PMICApp_checkForWdgErrors(pmicHandle, "Win-2");
        DebugP_assert(status == PMIC_ST_SUCCESS);

        // End of Q&A sequence; next question will be generated
        // and the next sequence will begin
        numSequences--;
    }

    if(status == PMIC_ST_SUCCESS)
    {
        DebugP_log("Stopped sending watchdog answers... Waiting for the warm reset to occur... \r\n");
        PMICApp_wait_ms(PMIC_WDG_TIMEOUT_DELAY);
    }

    /* The program should not reach here */
    DebugP_logError("PMIC Q&A watchdog reset mode test... Failed !!\r\n");
    retVal = SystemP_FAILURE;

    return retVal;
}

static int32_t PMICApp_checkForWdgErrors(Pmic_CoreHandle_t *handle, const char *str)
{
    Pmic_WdgError_t wdgErrStat = {.validParams = PMIC_CFG_WD_ERRSTAT_ALL_VALID_SHIFT};

    // Get WDG error statuses
    int32_t status = Pmic_wdgGetErrorStatus(handle, &wdgErrStat);

    // Check for WDG errors
    if ((status == PMIC_ST_SUCCESS) &&
        ((wdgErrStat.longWindowTimeout == (bool)TRUE) ||
        (wdgErrStat.timeout == (bool)TRUE) ||
        (wdgErrStat.triggerEarlyError == (bool)TRUE) ||
        (wdgErrStat.answerEarlyError == (bool)TRUE) ||
        (wdgErrStat.sequenceError == (bool)TRUE) ||
        (wdgErrStat.answerError == (bool)TRUE) ||
        (wdgErrStat.threshold1Error == (bool)TRUE)||
        (wdgErrStat.threshold2Error == (bool)TRUE)))
    {
        DebugP_log("%s error\r\n", str);
        status = PMIC_ST_ERR_FAIL;
    }
    return status;
}

static int32_t PMICApp_isWdogResetDone(Pmic_CoreHandle_t *coreHandle)
{
    int32_t        pmicStatus = PMIC_ST_SUCCESS;
    int32_t        retVal = SystemP_FAILURE;
    Pmic_WdgError_t wdgErrStat =
    {
        .validParams = PMIC_CFG_WD_ERRSTAT_ALL_VALID_SHIFT
    };

    /* Check whether WD_RST_INT error is raised */
    pmicStatus = Pmic_wdgGetErrorStatus(coreHandle, &wdgErrStat);
    if((pmicStatus == PMIC_ST_SUCCESS) && (wdgErrStat.timeout == TRUE))
    {
        retVal = PMIC_ST_SUCCESS;
    }

    return retVal;
}

static void PMICApp_wait_ms(uint16_t milliseconds)
{
    ClockP_usleep(milliseconds*1000U + ClockP_ticksToUsec(1U));
}

