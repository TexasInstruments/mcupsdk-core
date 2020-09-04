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

/**
 *  \file   watchdog_rti.c
 *
 *  \brief   File containing WDT Driver APIs implementation for version V0.
 *
 */

/**************************************************************************
 *************************** Include Files ********************************
 **************************************************************************/

#include <kernel/dpl/HwiP.h>
#include <drivers/watchdog.h>
#include <drivers/hw_include/hw_types.h>

/* ========================================================================== */
/*                                 Macros                                     */
/* ========================================================================== */

/**
 * \brief  RTI Multiplier shift.
 */
#define RTI_DWWDPRLD_MULTIPLIER_SHIFT       (13U)
/**
 * \brief  Minimum possible preload value for RTI DWD counter (2^13).
 */
#define RTI_DWD_MIN_PRELOAD_VAL             (0x1FFFU)

/* ========================================================================== */
/*                         Structure Definitions                              */
/* ========================================================================== */

/* Default Watchdog parameters structure */
const Watchdog_Params Watchdog_defaultParams = {
    NULL,                           /* callbackFxn */
    NULL,                           /* callbackFxnArgs */
    Watchdog_RESET_ON,              /* resetMode */
    Watchdog_DEBUG_STALL_ON,        /* stall mode*/
    Watchdog_WINDOW_100_PERCENT,    /* Windowed watchdog window size. */
    1000,                           /* Expiration time of 1000 ms (1 second) */
};

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

/* Externs */
extern Watchdog_Config gWatchdogConfig[];
extern uint32_t gWatchdogConfigNum;

uint32_t Watchdog_getWindowSize(Watchdog_Handle handle)
{
    uint32_t   windowSize;
    Watchdog_Config*        ptrWatchdogConfig;
    Watchdog_HwAttrs*       ptrHwCfg;

    ptrWatchdogConfig = (Watchdog_Config*)handle;
    ptrHwCfg = (Watchdog_HwAttrs*)ptrWatchdogConfig->hwAttrs;
    /* Get configured Window Size */
    windowSize = HW_RD_REG32(ptrHwCfg->baseAddr + CSL_RTI_RTIWWDSIZECTRL);
    return (windowSize);
}

void Watchdog_setWindowSize(Watchdog_Handle handle, uint32_t dwwdWindowSize)
{
    Watchdog_Config*        ptrWatchdogConfig;
    Watchdog_HwAttrs*       ptrHwCfg;

    ptrWatchdogConfig = (Watchdog_Config*)handle;
    ptrHwCfg = (Watchdog_HwAttrs*)ptrWatchdogConfig->hwAttrs;

    HW_WR_FIELD32(ptrHwCfg->baseAddr + CSL_RTI_RTIWWDSIZECTRL,
                    CSL_RTI_RTIWWDSIZECTRL_WWDSIZE,
                    dwwdWindowSize);
}

bool Watchdog_isClosedWindow(Watchdog_Handle handle)
{
    Watchdog_Config*        ptrWatchdogConfig;
    Watchdog_HwAttrs*       ptrHwCfg;

    ptrWatchdogConfig = (Watchdog_Config*)handle;
    ptrHwCfg = (Watchdog_HwAttrs*)ptrWatchdogConfig->hwAttrs;

    uint32_t currentDownCounter, windowSizeShift;
    uint32_t windowStartTime, timeOutValue, windowSize;

    windowSizeShift    = (uint32_t)
                         CSL_RTI_RTIWWDSIZECTRL_WWDSIZE_100_PERCENT_SHIFT;
    /* Get configured Window Size */
    windowSize = Watchdog_getWindowSize(handle);

    switch (windowSize)
    {
        case Watchdog_WINDOW_100_PERCENT:
            windowSizeShift = (uint32_t)
                              CSL_RTI_RTIWWDSIZECTRL_WWDSIZE_100_PERCENT_SHIFT;
            break;
        case Watchdog_WINDOW_50_PERCENT:
            windowSizeShift = (uint32_t)
                              CSL_RTI_RTIWWDSIZECTRL_WWDSIZE_50_PERCENT_SHIFT;
            break;
        case Watchdog_WINDOW_25_PERCENT:
            windowSizeShift = (uint32_t)
                              CSL_RTI_RTIWWDSIZECTRL_WWDSIZE_25_PERCENT_SHIFT;
            break;
        case Watchdog_WINDOW_12_5_PERCENT:
            windowSizeShift = (uint32_t)
                              CSL_RTI_RTIWWDSIZECTRL_WWDSIZE_12_5_PERCENT_SHIFT;
            break;
        case Watchdog_WINDOW_6_25_PERCENT:
            windowSizeShift = (uint32_t)
                              CSL_RTI_RTIWWDSIZECTRL_WWDSIZE_6_25_PERCENT_SHIFT;
            break;
        case Watchdog_WINDOW_3_125_PERCENT:
            windowSizeShift = (uint32_t)
                              CSL_RTI_RTIWWDSIZECTRL_WWDSIZE_3_125_PERCENT_SHIFT;
            break;
        default:
            break;
    }
    timeOutValue    = ((HW_RD_REG32(ptrHwCfg->baseAddr + CSL_RTI_RTIDWDPRLD)
                        << RTI_DWWDPRLD_MULTIPLIER_SHIFT) |
                        RTI_DWD_MIN_PRELOAD_VAL);

    windowStartTime = (timeOutValue /
                       (uint32_t) ((uint32_t) 0x1U << windowSizeShift));

    /* Get current down counter */
    currentDownCounter = HW_RD_REG32(ptrHwCfg->baseAddr + CSL_RTI_RTIDWDCNTR);
    if (currentDownCounter > windowStartTime)
    {
        return true;
    }
    else
    {
        return false;
    }

}

void Watchdog_setReaction(Watchdog_Handle handle, uint32_t dwwdReaction)
{
    Watchdog_Config*        ptrWatchdogConfig;
    Watchdog_HwAttrs*       ptrHwCfg;

    ptrWatchdogConfig = (Watchdog_Config*)handle;
    ptrHwCfg = (Watchdog_HwAttrs*)ptrWatchdogConfig->hwAttrs;

    HW_WR_FIELD32(ptrHwCfg->baseAddr + CSL_RTI_RTIWWDRXNCTRL,
                CSL_RTI_RTIWWDRXNCTRL_WWDRXN,
                dwwdReaction);
}

void Watchdog_paramsInit(Watchdog_Params *params)
{
    *params = Watchdog_defaultParams;
}

void Watchdog_clear(Watchdog_Handle handle)
{
    Watchdog_Config*        ptrWatchdogConfig;
    Watchdog_HwAttrs*       ptrHwCfg;
    Watchdog_MCB*           ptrWatchdogMCB;

    /* Sanity Check: We need to ensure that a valid argument was passed */
    if (handle != NULL)
    {
        /* Get the Watchdog Configuration: */
        ptrWatchdogConfig = (Watchdog_Config*)handle;

        /* Get the Watchdog Driver Object */
        ptrWatchdogMCB = (Watchdog_MCB*)ptrWatchdogConfig->object;

        ptrWatchdogMCB->watchdogCleared++;

        /* Get the hardware configuration: */
        ptrHwCfg = (Watchdog_HwAttrs*)ptrWatchdogConfig->hwAttrs;

        /* Service the WDT */
        HW_WR_FIELD32( ptrHwCfg->baseAddr + CSL_RTI_RTIWDKEY,
                      CSL_RTI_RTIWDKEY_WDKEY,
                      CSL_RTI_RTIWDKEY_WDKEY_FIRST_WRITE);
        HW_WR_FIELD32( ptrHwCfg->baseAddr + CSL_RTI_RTIWDKEY,
                      CSL_RTI_RTIWDKEY_WDKEY,
                      CSL_RTI_RTIWDKEY_WDKEY_SECOND_WRITE);
    }
    return;
}

void Watchdog_close(Watchdog_Handle handle)
{
    return;
}

void Watchdog_init(void)
{
    return;
}

void Watchdog_deinit(void)
{
    return;
}

Watchdog_Handle Watchdog_open(uint8_t index, Watchdog_Params* params)
{
    Watchdog_Config*        ptrWatchdogConfig;
    Watchdog_MCB*           ptrWatchdogMCB;
    Watchdog_HwAttrs*       ptrHwCfg;
    Watchdog_Handle         handle = NULL;
    int32_t                 retVal = 0;
    uint32_t                preloadMaxValue;
    uint64_t                preloadValueTmp;
    int32_t                 status = 0;

    /* Check index */
    if(index >= gWatchdogConfigNum)
    {
        status = SystemP_FAILURE;
    }
    else
    {
        handle = (Watchdog_Handle)&(gWatchdogConfig[index]);
    }

    if(SystemP_SUCCESS == status)
    {
        /* Get the Watchdog Configuration: */
        ptrWatchdogConfig = (Watchdog_Config*)handle;

        /* Get the hardware configuration: */
        ptrHwCfg = (Watchdog_HwAttrs*)ptrWatchdogConfig->hwAttrs;

        /* Get the Watchdog Driver Object */
        ptrWatchdogMCB = (Watchdog_MCB*)ptrWatchdogConfig->object;

        /* Initialize the memory: */
        memset ((void *)ptrWatchdogMCB, 0, sizeof(Watchdog_MCB));

        /* Store the WDT parameters */
        if (params == NULL) {
            /* No params passed in, so use the defaults */
            Watchdog_paramsInit(&(ptrWatchdogMCB->params));
        }
        else {
            /* Copy the params contents */
            ptrWatchdogMCB->params = *params;
        }

        /* Calculate preload value from the expiration time */
        /* Expiration time (in millisecond)/1000 = (preload value + 1)*(2^13)/RTICLK */
        preloadValueTmp =((uint64_t)(ptrHwCfg->wdtClkFrequency)) * ((uint64_t)ptrWatchdogMCB->params.expirationTime) / 1000;
        /* Pre load is 12 bit value in register and actual value should be multiplied by 2^13) */
        preloadMaxValue = (1 << (12 + 13)) - 1;
        if (preloadValueTmp > preloadMaxValue)
        {
            retVal = -1;
        }

        if (retVal >= 0)
        {
            /* Configure the Watchdog driver. */

            /* Bring watchdog out of reset */
            Watchdog_reset(handle);

            /* if NMI interrupt mode is configured */
            if (ptrWatchdogMCB->params.resetMode == Watchdog_RESET_OFF)
            {
                 /* Clear the status flags */
 //               HW_WR_REG32(ptrHwCfg->baseAddr + CSL_RTI_RTIWDSTATUS, WATCHDOG_CLEAR_STATUS);
            }
            else
            {
                /* Configure the SOC moule to trigger a warm reset upon watchdog reset */
                Watchdog_configureWarmReset(handle);
            }
        }

        if (retVal >= 0 )
        {
            uint32_t preloadValue;
            uint32_t dwwdPreloadVal_l;
            /* CSL API needs shifted value but doesnot do -1 while programming.
             * So updating the preload value.
             */
            preloadValue = ((uint32_t)(preloadValueTmp) >> RTI_DWWDPRLD_MULTIPLIER_SHIFT) - 1;
            preloadValue = preloadValue << RTI_DWWDPRLD_MULTIPLIER_SHIFT;

            /* Clear the status flags */
            HW_WR_REG32(ptrHwCfg->baseAddr + CSL_RTI_RTIWDSTATUS, WATCHDOG_CLEAR_STATUS);

            /* Configure window in which watch-dog should be serviced */
            Watchdog_setWindowSize(handle, ptrWatchdogMCB->params.windowSize);

            /* Set the preload value */
            dwwdPreloadVal_l = (preloadValue >>
                                ((uint32_t) RTI_DWWDPRLD_MULTIPLIER_SHIFT));
            if ((uint32_t) CSL_RTI_RTIDWDPRLD_DWDPRLD_MAX > dwwdPreloadVal_l)
            {
                /* Initialize DWD Expiration Period */
                HW_WR_FIELD32(ptrHwCfg->baseAddr + CSL_RTI_RTIDWDPRLD,
                              CSL_RTI_RTIDWDPRLD_DWDPRLD,
                              dwwdPreloadVal_l);
            }

             /* Configure the reset mode    */
            Watchdog_setReaction(handle, ptrWatchdogMCB->params.resetMode);

            /* Configure the stall mode */
            HW_WR_FIELD32(ptrHwCfg->baseAddr + CSL_RTI_RTIGCTRL,
                          CSL_RTI_RTIGCTRL_COS,
                          ptrWatchdogMCB->params.debugStallMode);

            /* Enable DWWD by writing pre-defined value '0xA98559DA' to RTIDWDCTRL */
            HW_WR_REG32(ptrHwCfg->baseAddr + CSL_RTI_RTIDWDCTRL,
                        CSL_RTI_RTIDWDCTRL_DWDCTRL_ENABLE);

            /* Mark the driver to be operational */
            ptrWatchdogMCB->state = Watchdog_DriverState_OPERATIONAL;

        }

    }

    return handle;
}
