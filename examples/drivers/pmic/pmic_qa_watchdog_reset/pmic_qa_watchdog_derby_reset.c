/*
 *  Copyright (C) 2024 Texas Instruments Incorporated
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
 *  \file pmic_qa_watchdog_reset.c
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

/* ========================================================================== */
/*                         Structure Declarations                             */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

static int32_t PMICApp_isWdogResetDone(const Pmic_CoreHandle_t *coreHandle);
static int32_t PMICApp_wdogQAModeReset(const Pmic_CoreHandle_t *coreHandle);
static int32_t PMICApp_checkForWdgErrors(const Pmic_CoreHandle_t *handle, const char *str);

static void PMICApp_wait_ms(uint16_t milliseconds);
static void PMICApp_checkIfWdgConfigurable(const Pmic_CoreHandle_t *pmicHandle);

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */


void pmic_qa_watchdog_reset_main(void *args)
{
    const Pmic_CoreHandle_t* handle;
    int32_t status = SystemP_SUCCESS;
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
    if(PMICApp_isWdogResetDone(handle) == SystemP_SUCCESS)
    {
        DebugP_log("Warm reset has occurred due to watchdog failure!! \r\n");
        pmicStatus = Pmic_wdgSetRetLongWin(handle, PMIC_ENABLE);
        pmicStatus += Pmic_wdgSetPwrHold(handle, PMIC_ENABLE);
        if (pmicStatus ==  PMIC_ST_SUCCESS)
        {
            pmicStatus += Pmic_wdgClrErrStatAll(handle);
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
    
    if(SystemP_SUCCESS == status)
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

static void PMICApp_checkIfWdgConfigurable(const Pmic_CoreHandle_t *pmicHandle)
{
    bool val = (bool)FALSE;

    // Unlock PMIC registers
    int32_t status = Pmic_unlockRegs(pmicHandle);
    DebugP_assert(status == PMIC_ST_SUCCESS);

    // Verify PMIC configuration register lock status
    status = Pmic_getRegLock(pmicHandle, &val);
    DebugP_assert((status == PMIC_ST_SUCCESS) && (val == (bool)FALSE));

    // Enable WDG
    status = Pmic_wdgEnable(pmicHandle);
    DebugP_assert(status == PMIC_ST_SUCCESS);

    // Verify WDG enable status
    status = Pmic_wdgGetEnable(pmicHandle, &val);
    DebugP_assert((status == PMIC_ST_SUCCESS) && (val == (bool)TRUE));

    // Enable WD_RETURN_LONGWIN so that the PMIC can return to Long Window after
    // completion of the current WDG sequence
    status = Pmic_wdgSetRetLongWin(pmicHandle, (bool)TRUE);
    DebugP_assert(status == PMIC_ST_SUCCESS);

    // Verify WD_RETURN_LONGWIN is set to 1
    status = Pmic_wdgGetRetLongWin(pmicHandle, &val);
    DebugP_assert((status == PMIC_ST_SUCCESS) && (val == (bool)TRUE));

    // Enable WD_PWRHOLD so that the PMIC can stay in Long Window once entered
    status = Pmic_wdgSetPwrHold(pmicHandle, (bool)TRUE);
    DebugP_assert(status == PMIC_ST_SUCCESS);

    // Verify WD_PWRHOLD is set to 1
    status = Pmic_wdgGetPwrHold(pmicHandle, &val);
    DebugP_assert((status == PMIC_ST_SUCCESS) && (val == (bool)TRUE));
}

static int32_t PMICApp_wdogQAModeReset(const Pmic_CoreHandle_t* pmicHandle)
{
    int32_t        status = PMIC_ST_SUCCESS;
    int32_t        retVal = SystemP_FAILURE;
    uint16_t      numSequences = 100U;
    uint8_t       answCnt = 0;

    Pmic_WdgCfg_t wdgCfg = {
        .validParams = (PMIC_WD_RST_EN_VALID | PMIC_WD_MODE_VALID |
                        PMIC_WD_FAIL_THR_VALID | PMIC_WD_RST_THR_VALID |
                        PMIC_WD_LONG_WIN_DURATION_VALID | PMIC_WD_WIN1_DURATION_VALID |
                        PMIC_WD_WIN2_DURATION_VALID | PMIC_WD_QA_FDBK_VALID |
                        PMIC_WD_QA_LFSR_VALID | PMIC_WD_QA_SEED_VALID),
        .rstEn = (bool)TRUE,
        .mode = PMIC_QA_MODE,
        .failThr = PMIC_WD_FAIL_THR_MAX,
        .rstThr = PMIC_WD_RST_THR_MAX,
        .longWinDuration = PMIC_WD_LONG_WIN_DURATION_MAX,
        .win1Duration = PMIC_WD_WIN1_DURATION_MAX,
        .win2Duration = PMIC_WD_WIN2_DURATION_MAX,
        .qaFdbk = 0U,
        .qaLfsr = 1U,
        .qaSeed = 2U,
    };

    /* Configure watchdog for Q&A mode */
    status = Pmic_wdgSetCfg(pmicHandle, &wdgCfg);
    DebugP_assert(status == PMIC_ST_SUCCESS);

    DebugP_log("Configure PMIC watchdog in Q&A mode... DONE \r\n");

    // Clear all WDG error statuses
    status = Pmic_wdgClrErrStatAll(pmicHandle);
    DebugP_assert(status == PMIC_ST_SUCCESS);

    // Disable WD_PWRHOLD to enable PMIC to exit Long Window
    status = Pmic_wdgSetPwrHold(pmicHandle, (bool)FALSE);
    DebugP_assert(status == PMIC_ST_SUCCESS);

    // Disable WD_RETURN_LONGWIN so that PMIC does not return to Long Window at
    // the end of the next Q&A sequence
    status = Pmic_wdgSetRetLongWin(pmicHandle, (bool)FALSE);
    DebugP_assert(status == PMIC_ST_SUCCESS);

    DebugP_log("Started watchdog sequence... Sending answers to the PMIC... \r\n");

    // Exit Long Window by sending all 4 answer bytes
    for (answCnt = 4U; answCnt != 0U; answCnt--)
    {
        status = Pmic_wdgWriteAnswer(pmicHandle);
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
            status = Pmic_wdgWriteAnswer(pmicHandle);
            DebugP_assert(status == PMIC_ST_SUCCESS);
            status = PMICApp_checkForWdgErrors(pmicHandle, "Win-1");
            DebugP_assert(status == PMIC_ST_SUCCESS);
        }

        // Wait until Window-1 time elapses
        PMICApp_wait_ms(PMIC_WDG_WINDOW_DELAY);

        // Enter Window-2; calculate and send last answer byte
        status = Pmic_wdgWriteAnswer(pmicHandle);
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

static int32_t PMICApp_checkForWdgErrors(const Pmic_CoreHandle_t *handle, const char *str)
{
    Pmic_WdgErrStat_t wdgErrStat = {.validParams = PMIC_WDG_ERR_STAT_ALL_VALID};

    // Get WDG error statuses
    int32_t status = Pmic_wdgGetErrStat(handle, &wdgErrStat);

    // Check for WDG errors
    if ((status == PMIC_ST_SUCCESS) &&
        ((wdgErrStat.rstInt == (bool)TRUE) ||
        (wdgErrStat.failInt == (bool)TRUE) ||
        (wdgErrStat.answErr == (bool)TRUE) ||
        (wdgErrStat.seqErr == (bool)TRUE) ||
        (wdgErrStat.answEarlyErr == (bool)TRUE) ||
        (wdgErrStat.timeoutErr == (bool)TRUE) ||
        (wdgErrStat.longWinTimeoutInt == (bool)TRUE)))
    {
        DebugP_log("%s error\r\n", str);
        status = PMIC_ST_ERR_FAIL;
    }
    return status;
}

static int32_t PMICApp_isWdogResetDone(const Pmic_CoreHandle_t *coreHandle)
{
    int32_t        pmicStatus = PMIC_ST_SUCCESS;
    int32_t        retVal = SystemP_FAILURE;
    Pmic_WdgErrStat_t wdgErrStat =
    {
        .validParams = PMIC_WDG_ERR_STAT_ALL_VALID
    };

    /* Check whether WD_RST_INT error is raised */
    pmicStatus = Pmic_wdgGetErrStat(coreHandle, &wdgErrStat);
    if((pmicStatus == PMIC_ST_SUCCESS) && (wdgErrStat.timeoutErr == TRUE))
    {
        retVal = SystemP_SUCCESS;
    }

    return retVal;
}

static void PMICApp_wait_ms(uint16_t milliseconds)
{
    ClockP_usleep(milliseconds*1000U + ClockP_ticksToUsec(1U));
}

