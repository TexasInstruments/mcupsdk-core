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
 *  \file pmic_qa_watchdog_derby_interrupt.c
 *
 *  \brief This is a PMIC watchdog example for Q&A mode which will generate
 *         a interrupt when MCU fails to send a signal to the watchdog.
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
#define PMIC_REG_STATE_LOCK         (1U)

/* ========================================================================== */
/*                         Structure Declarations                             */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                        External Function Declarations                      */
/* ========================================================================== */

extern void Board_gpioInit(void);
extern void Board_gpioDeinit(void);
extern uint32_t Board_getGpioIntrNum(void);

/* ========================================================================== */
/*                         Internal Function Declarations                     */
/* ========================================================================== */

static int32_t PMICApp_gpioIntConfigure();
static int32_t PMICApp_wdogQAModeInterrupt(Pmic_CoreHandle_t *coreHandle);
static int32_t PMICApp_checkForWdgErrors(Pmic_CoreHandle_t *handle, const char *str);
static int32_t PMICApp_setGpoNint(Pmic_CoreHandle_t *handle);
static void PMICApp_wait_ms(uint16_t milliseconds);
static void PMICApp_checkIfWdgConfigurable(Pmic_CoreHandle_t *pmicHandle);
static void PMICApp_gpioISR(void *args);

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */

/* The address of GPIO instance used for receiving the interrupt from PMIC */
uint32_t            gGpioBaseAddr = GPIO_PMIC_INT_BASE_ADDR;

/* HwiP object for GPIO interrupt */
HwiP_Object         gGpioHwiObject;

/* Semaphore object to notify PMIC GPIO interrupt has occurred */
static SemaphoreP_Object gGpioISRDoneSem;

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */


void pmic_qa_watchdog_interrupt_main(void *args)
{
    Pmic_CoreHandle_t* handle;
    int32_t status = PMIC_ST_SUCCESS;
    bool val = (uint8_t)TRUE;

    /* Open drivers to open the UART driver for console */
    Drivers_open();
    Board_driversOpen();

    /* PMIC interface handle initialized by PMIC_open */
    handle = PMIC_getCoreHandle(CONFIG_PMIC0);
    DebugP_assert(NULL != handle);

    DebugP_log("Starting Q&A watchdog Interrupt example !!\r\n");

    /* Unlock PMIC registers */
    status = Pmic_unlockRegs(handle);
    DebugP_assert(status == PMIC_ST_SUCCESS);

    // Verify PMIC configuration register lock status
    status = Pmic_getRegLock(handle, &val);
    DebugP_assert((status == PMIC_ST_SUCCESS) && (val == (bool)FALSE));

    if(PMIC_ST_SUCCESS == status)
    {
        /* Configure the PMIC NINT pin to generate interrupt */
        status = PMICApp_setGpoNint(handle);
    }
       
    if(PMIC_ST_SUCCESS == status)
    {
        /* Configure the GPIO pin to receive interrupt from PMIC */
        status = PMICApp_gpioIntConfigure();
    }

    /* The PMIC automatically runs built-in self tests called ABIST and LBIST upon bootup. 
    After the tests are done, the PMIC sets the ABIST_DONE and LBIST_DONE flags to 1, causing nINT to assert 
    clearing all interrupt flags */
    status = Pmic_irqClrAllFlags(handle);
    DebugP_assert(status == PMIC_ST_SUCCESS);
   

    DebugP_log("Checking if prerequisites for configuring the WDG has been met...\r\n");

    if(PMIC_ST_SUCCESS == status)
    {
        PMICApp_checkIfWdgConfigurable(handle);
        status = PMICApp_wdogQAModeInterrupt(handle);
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

static int32_t PMICApp_setGpoNint(Pmic_CoreHandle_t *pmicHandle)
{
    uint32_t status = PMIC_ST_SUCCESS;

    Pmic_GpioCfg_t gpiocfg = {
        .validParams = PMIC_FUNCTIONALITY_VALID,
        .functionality = PMIC_NINT_GPI_NINT,
    };
    status = Pmic_gpioSetCfg(pmicHandle, PMIC_NINT_GPI, &gpiocfg);
    return status;
}

static int32_t PMICApp_gpioIntConfigure()
{
    int32_t status = PMIC_ST_SUCCESS;
    uint32_t pinNum, intrNum;
    HwiP_Params hwiPrms;

    Board_gpioInit();

    pinNum          = GPIO_PMIC_INT_PIN;
    intrNum         = Board_getGpioIntrNum();

    /* Address translate */
    gGpioBaseAddr = (uint32_t) AddrTranslateP_getLocalAddr(gGpioBaseAddr);

    /* Register pin interrupt */
    HwiP_Params_init(&hwiPrms);
    hwiPrms.intNum   = intrNum;
    hwiPrms.callback = &PMICApp_gpioISR;
    hwiPrms.args     = (void *) pinNum;
    /* GPIO interrupt is a pulse type interrupt */
    hwiPrms.isPulse  = TRUE;
    status = HwiP_construct(&gGpioHwiObject, &hwiPrms);
    DebugP_assert(status == PMIC_ST_SUCCESS );
    return status;
}

static void PMICApp_checkIfWdgConfigurable(Pmic_CoreHandle_t *pmicHandle)
{
    bool val = (bool)TRUE;
    int32_t status = PMIC_ST_SUCCESS;

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

static int32_t PMICApp_wdogQAModeInterrupt(Pmic_CoreHandle_t* pmicHandle)
{
    int32_t status = PMIC_ST_SUCCESS;
    uint16_t numSequences = 40U;
    uint8_t answCnt = 0;
    bool isWdogInt = FALSE;
    Pmic_WdgErrStat_t wdgErrStat = {.validParams = PMIC_WDG_ERR_STAT_ALL_VALID};

    Pmic_WdgCfg_t wdgCfg = {
        .validParams = (PMIC_WD_RST_EN_VALID | PMIC_WD_MODE_VALID |
                        PMIC_WD_FAIL_THR_VALID | PMIC_WD_RST_THR_VALID |
                        PMIC_WD_LONG_WIN_DURATION_VALID | PMIC_WD_WIN1_DURATION_VALID |
                        PMIC_WD_WIN2_DURATION_VALID | PMIC_WD_QA_FDBK_VALID |
                        PMIC_WD_QA_LFSR_VALID | PMIC_WD_QA_SEED_VALID),
        .rstEn = (bool)FALSE,
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

    status = SemaphoreP_constructBinary(&gGpioISRDoneSem, 0);

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

        bool irqFirstNOK = FALSE;
        status = Pmic_irqGetFlag(pmicHandle, PMIC_WD_FIRST_NOK_NMI, &irqFirstNOK);
        DebugP_assert(status == PMIC_ST_SUCCESS);

        if(irqFirstNOK == TRUE)
        {
            status = Pmic_irqClrFlag(pmicHandle, PMIC_WD_FIRST_NOK_NMI);
            DebugP_assert(status == PMIC_ST_SUCCESS);
        }
        // End of Q&A sequence; next question will be generated
        // and the next sequence will begin
        numSequences--;
    }

    if(status == PMIC_ST_SUCCESS)
    {
        DebugP_log("Stopped sending watchdog answers... Waiting for the PMIC interrupt... \r\n");
        while(!isWdogInt)
        {
            /* Stop sending the Q&A signal and wait for PMIC GPIO interrupt */
            status = SemaphoreP_pend(&gGpioISRDoneSem, SystemP_WAIT_FOREVER);
            if(status == PMIC_ST_SUCCESS)
            {
                /* Check whether the PMIC interrupt is due to watchdog error */
                status = Pmic_wdgGetErrStat(pmicHandle, &wdgErrStat);
                if(status == PMIC_ST_SUCCESS)
                {
                    isWdogInt = TRUE;
                    /* Clear all interrupts */
                    status = Pmic_wdgClrErrStatAll(pmicHandle);
                }
            }
        }
    }

    if(status == PMIC_ST_SUCCESS)
    {
        DebugP_log("Received GPIO interrupt for PMIC watchdog error !! \r\n");
        /* Set return to long window */
        status = Pmic_wdgSetRetLongWin(pmicHandle, PMIC_ENABLE);
        status += Pmic_wdgSetPwrHold(pmicHandle, (bool)TRUE);
    }

    if(status == PMIC_ST_SUCCESS)
    {
        DebugP_log("PMIC Q&A watchdog interrupt mode test... DONE\r\n");
    }
    else
    {
        DebugP_logError("PMIC Q&A watchdog interrupt mode test... Failed !!!\r\n");
    }

    return status;
}

static int32_t PMICApp_checkForWdgErrors(Pmic_CoreHandle_t *handle, const char *str)
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

static void PMICApp_wait_ms(uint16_t milliseconds)
{
    ClockP_usleep(milliseconds*1000U + ClockP_ticksToUsec(1U));
}

static void PMICApp_gpioISR(void *args)
{
    uint32_t pinNum = (uint32_t) args;
    uint32_t bankNum =  GPIO_GET_BANK_INDEX(pinNum);
    uint32_t intrStatus, pinMask = GPIO_GET_BANK_BIT_MASK(pinNum);

    /* Get and clear bank interrupt status */
    intrStatus = GPIO_getBankIntrStatus(gGpioBaseAddr, bankNum);
    GPIO_clearBankIntrStatus(gGpioBaseAddr, bankNum, intrStatus);
    /* Per pin interrupt handling */
    if(intrStatus & pinMask)
    {
        SemaphoreP_post(&gGpioISRDoneSem);
    }
    return;
}