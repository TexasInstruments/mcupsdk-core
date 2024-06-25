/*
 *  Copyright (c) 2021-2023 Texas Instruments Incorporated
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
 *  \file     sdl_rti_example.c
 *
 *  \brief    This file contains RTI DWWD Function test code.
 *
 **/


#include "rti_app_main.h"
/*===========================================================================*/
/*                         Macros                                            */
/*===========================================================================*/
/* None */
#if defined (SOC_AM64X) || defined (SOC_AM243X)
#if defined (M4F_CORE)
#define SDL_INSTANCE_RTI SDL_INSTANCE_MCU_RTI0_CFG
#define SDL_RTI_BASE SDL_MCU_RTI0_CFG_BASE
#endif
#endif
#if defined (SOC_AM64X) || defined (SOC_AM243X)
#if defined (R5F_CORE)
#define SDL_INSTANCE_RTI SDL_INSTANCE_RTI8_CFG
#define SDL_RTI_BASE SDL_RTI8_CFG_BASE
#endif
#endif
#if defined (SOC_AM263X) || defined (SOC_AM263PX) || defined (SOC_AM261X)
#define SDL_INSTANCE_RTI SDL_INSTANCE_WDT0
#define SDL_RTI_BASE SDL_WDT0_U_BASE
#endif
/* MSS Instance for AM273x and AWR294X*/
#if defined (R5F_INPUTS)
#define SDL_INSTANCE_RTI SDL_INSTANCE_MSS_WDT
#define SDL_RTI_BASE  SDL_MSS_WDT_U_BASE
#define SDL_ESM_U_BASE SDL_MSS_ESM_U_BASE
#endif
/* DSS Instance for AM273x and AWR294X*/
#if defined (C66_INPUTS)
#define SDL_INSTANCE_RTI SDL_INSTANCE_DSS_WDT
#define SDL_RTI_BASE  SDL_DSS_WDT_U_BASE
#define SDL_ESM_U_BASE SDL_DSS_ESM_U_BASE
#endif
/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */

volatile uint32_t isrFlag = RTI_NO_INTERRUPT;
/**< Flag used to indicate interrupt is generated */
SDL_RTI_configParms     pConfig;
uint32_t rtiModule = SDL_RTI_BASE;
/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */
#if defined (R5F_INPUTS)
static void RTISetClockSource(uint32_t rtiModuleSelect,
                              uint32_t rtiClockSourceSelect);
#endif
#if defined (SOC_AM64X) || defined (SOC_AM243X)
static void RTISetClockSource(uint32_t rtiModuleSelect,
                              uint32_t rtiClockSourceSelect);
#endif
static uint32_t RTIGetPreloadValue(uint32_t rtiClkSource, uint32_t timeoutVal);
int32_t RTIDwwdIsClosedWindow(uint32_t baseAddr, uint32_t *pWinStatus);

/* internal function to read the window size */
static inline uint32_t RTIDwwdReadWinSz(uint32_t baseAddr)
{
    uint32_t   windowSize;
    /* Get configured Window Size */
    windowSize = HW_RD_REG32(baseAddr + RTI_RTIDWWDSIZECTRL);
    return (windowSize);
}

static inline uint32_t RTIDwwdReadTimeOut(uint32_t baseAddr)
{
    uint32_t timeOutValue;

    timeOutValue = ((HW_RD_REG32(baseAddr + RTI_RTIDWDPRLD)
                     << RTI_DWWDPRLD_MULTIPLIER_SHIFT) |
                    RTI_DWD_MIN_PRELOAD_VAL);

    return timeOutValue;
}

int32_t RTIDwwdIsClosedWindow(uint32_t baseAddr, uint32_t *pIsClosedWindow)
{
    uint32_t closedWindowstatus, currentDownCounter, windowSizeShift;
    uint32_t windowStartTime, timeOutValue, windowSize;
#if defined (SOC_AM64X) || defined (SOC_AM243X)
#if defined (M4F_CORE)
	uint32_t getBaseAddr;
	SDL_RTI_getBaseaddr(baseAddr,&getBaseAddr);
	baseAddr=getBaseAddr;
#endif
#endif
    int32_t retVal = SDL_EFAIL;
    if ((baseAddr        != ((uint32_t) NULL)) &&
        (pIsClosedWindow != (NULL_PTR)))
    {
        windowSizeShift    = (uint32_t)
                             RTI_RTIDWWDSIZECTRL_DWWDSIZE_100_PERCENT_SHIFT;
        /* Get configured Window Size */
        windowSize = RTIDwwdReadWinSz(baseAddr);
        switch (windowSize)
        {
            case RTI_DWWD_WINDOWSIZE_100_PERCENT:
                windowSizeShift = (uint32_t)
                                  RTI_RTIDWWDSIZECTRL_DWWDSIZE_100_PERCENT_SHIFT;
                break;
            case RTI_DWWD_WINDOWSIZE_50_PERCENT:
                windowSizeShift = (uint32_t)
                                  RTI_RTIDWWDSIZECTRL_DWWDSIZE_50_PERCENT_SHIFT;
                break;
            case RTI_DWWD_WINDOWSIZE_25_PERCENT:
                windowSizeShift = (uint32_t)
                                  RTI_RTIDWWDSIZECTRL_DWWDSIZE_25_PERCENT_SHIFT;
                break;
            case RTI_DWWD_WINDOWSIZE_12_5_PERCENT:
                windowSizeShift = (uint32_t)
                                  RTI_RTIDWWDSIZECTRL_DWWDSIZE_12_5_PERCENT_SHIFT;
                break;
            case RTI_DWWD_WINDOWSIZE_6_25_PERCENT:
                windowSizeShift = (uint32_t)
                                  RTI_RTIDWWDSIZECTRL_DWWDSIZE_6_25_PERCENT_SHIFT;
                break;
            case RTI_DWWD_WINDOWSIZE_3_125_PERCENT:
                windowSizeShift = (uint32_t)
                                  RTI_RTIDWWDSIZECTRL_DWWDSIZE_3_125_PERCENT_SHIFT;
                break;
            default:
                break;
        }
        timeOutValue    = RTIDwwdReadTimeOut(baseAddr);
        windowStartTime = (timeOutValue /
                           (uint32_t) ((uint32_t) 0x1U << windowSizeShift));
        /* Get current down counter */
        currentDownCounter = HW_RD_REG32(baseAddr + RTI_RTIDWDCNTR);
        if (currentDownCounter > windowStartTime)
        {
            closedWindowstatus = TRUE;
        }
        else
        {
            closedWindowstatus = FALSE;
        }
        /* Update the status */
        *pIsClosedWindow = closedWindowstatus;
        retVal = SDL_PASS;
    }
    return (retVal);
}

int32_t SDL_RTI_exampleTest(void)
{
    uint32_t rtiModule;
    uint32_t closedWinStatus;
    int32_t  retVal = SDL_PASS;
    SDL_RTI_configParms     pConfig;
    SDL_RTI_staticRegs         pStaticRegs;

    rtiModule = SDL_RTI_BASE;
    DebugP_log("RTI Example code UC-3 started\r\n");

    /* Register Interrupt */
    isrFlag = RTI_NO_INTERRUPT;

    /* Configure RTI parameters */
    pConfig.SDL_RTI_dwwdPreloadVal = RTIGetPreloadValue(RTI_CLOCK_SOURCE_200MHZ_FREQ_KHZ, RTI_WDT_TIMEOUT);
    pConfig.SDL_RTI_dwwdWindowSize = RTI_DWWD_WINDOWSIZE_50_PERCENT;
    pConfig.SDL_RTI_dwwdReaction   = RTI_DWWD_REACTION_GENERATE_NMI;

    retVal = SDL_RTI_config(SDL_INSTANCE_RTI, &pConfig);

    if (retVal == SDL_EFAIL)
    {
        DebugP_log("Error during Window configuration.\r\n");
    }

    /* Verify the config */
    retVal = SDL_RTI_verifyConfig(SDL_INSTANCE_RTI, &pConfig);

    if (retVal == SDL_EFAIL)
    {
        DebugP_log("Error during Window Verify configuration.\r\n");
    }

    if (retVal == SDL_PASS)
    {
        SDL_RTI_readStaticRegs(SDL_INSTANCE_RTI, &pStaticRegs);

        switch(pStaticRegs.RTI_WWDSIZECTRL)
        {
            case RTI_RTIDWWDSIZECTRL_DWWDSIZE_100_PERCENT:
                 DebugP_log("   DWWD configured to 100 percent window size\r\n");
                 break;
            case RTI_RTIDWWDSIZECTRL_DWWDSIZE_50_PERCENT:
                 DebugP_log("   DWWD configured to 50 percent window size\r\n");
                 break;
            case RTI_RTIDWWDSIZECTRL_DWWDSIZE_25_PERCENT:
                 DebugP_log("   DWWD configured to 25 percent window size\r\n");
                 break;
            case RTI_RTIDWWDSIZECTRL_DWWDSIZE_12_5_PERCENT:
                 DebugP_log("   DWWD configured to 6.25 percent window size\r\n");
                 break;
            case RTI_RTIDWWDSIZECTRL_DWWDSIZE_6_25_PERCENT:
                 DebugP_log("   DWWD configured to 3.125 percent window size\r\n");
                 break;
        }
		/* DWWD is configured to 10 sec timeout */
        DebugP_log("    DWWD is configured for %u ms time-out \r\n", RTI_WDT_TIMEOUT);
	}


    if ((retVal == SDL_PASS) &&
        (pConfig.SDL_RTI_dwwdWindowSize != RTI_DWWD_WINDOWSIZE_100_PERCENT))
    {
        /* RTI is serviced in closed window. Generates DWWD violation.
         * Closed window violation cant be generated for 100% window size.
         */
        volatile uint32_t loopBreak = FALSE;
        isrFlag = RTI_NO_INTERRUPT;

		SDL_RTI_start(SDL_INSTANCE_RTI);

        while (loopBreak == FALSE)
        {
			/* UC-3 EARLY TRIGGER: Servicing DWWD before window open generates violation
			   and generates interrupt to ESM module */
            DebugP_log("\r\nRTI DWWD Early Trigger violation test running.\r\n");

            RTIDwwdIsClosedWindow(rtiModule, &closedWinStatus);

            if (closedWinStatus == TRUE)
            {
                loopBreak = TRUE;

				DebugP_log("\r\nServicing before the window open.\r\n");

				/* Servicing the DWWD in closed window */
                SDL_RTI_service(SDL_INSTANCE_RTI);

                while (RTI_NO_INTERRUPT == isrFlag)
                {
                    /* Wait for interrupt */
                }
            }
            else
            {
                /* DWWD is in open window, just service it. */
                SDL_RTI_service(SDL_INSTANCE_RTI);
            }
        }
        if (retVal == SDL_PASS)
        {
            DebugP_log("\r\nRTI DWWD Early Trigger violation test successful. \r\n");
        }
        else
        {
            DebugP_log("RTI DWWD Early Trigger violation test failed. \r\n");
        }
    }

    return retVal;
}

/* ========================================================================== */
/*                 Internal Function Definitions                              */
/* ========================================================================== */

#if defined (R5F_INPUTS)
static void RTISetClockSource(uint32_t rtiModuleSelect,
                              uint32_t rtiClockSourceSelect)
{
#if !defined (SOC_AWR294X)
    switch (rtiModuleSelect) {
#if defined (SOC_AM263X) || defined (SOC_AM263PX) || defined (SOC_AM261X)
        case SDL_WDT0_U_BASE:
            HW_WR_FIELD32(SDL_MCU_CTRL_MMR0_CFG0_BASE +
                          SDL_MCU_CTRL_MMR_CFG0_MCU_RTI0_CLKSEL,
                          SDL_MCU_CTRL_MMR_CFG0_MCU_RTI0_CLKSEL_CLK_SEL,
                          rtiClockSourceSelect);
            break;
#endif
#if defined (SOC_AM273X)
        case SDL_MSS_WDT_U_BASE:
            HW_WR_FIELD32(SDL_MCU_CTRL_MMR0_CFG0_BASE +
                          SDL_MCU_CTRL_MMR_CFG0_MCU_RTI0_CLKSEL,
                          SDL_MCU_CTRL_MMR_CFG0_MCU_RTI0_CLKSEL_CLK_SEL,
                          rtiClockSourceSelect);
            break;
#endif
        default:
            DebugP_log("Error : RTI Instance not supported "
                "!!!\r\n");
            break;
    }
#endif
}
#endif

#if defined (SOC_AM64X) || defined (SOC_AM243X)
static void RTISetClockSource(uint32_t rtiModuleSelect,
                              uint32_t rtiClockSourceSelect)
{
    uint32_t baseAddr;

	switch (rtiModuleSelect) {
#if defined (R5F_CORE)
        case SDL_RTI8_CFG_BASE:
#elif defined (M4F_CORE)
        case SDL_MCU_RTI0_CFG_BASE:
#endif
			baseAddr = (uint32_t)SDL_DPL_addrTranslate(SDL_MCU_CTRL_MMR_CFG0_MCU_RTI0_CLKSEL, SDL_MCU_CTRL_MMR0_CFG0_SIZE);
            HW_WR_FIELD32(baseAddr,
                          SDL_MCU_CTRL_MMR_CFG0_MCU_RTI0_CLKSEL_CLK_SEL,
                          rtiClockSourceSelect);
            break;
	}
}
#endif

static uint32_t RTIGetPreloadValue(uint32_t rtiClkSource, uint32_t timeoutVal)
{
    uint32_t clkFreqKHz       = (uint32_t) RTI_CLOCK_SOURCE_32KHZ_FREQ_KHZ,
            timeoutNumCycles = 0;

    switch (rtiClkSource)
    {
        case RTI_CLOCK_SOURCE_32KHZ:
            clkFreqKHz = (uint32_t) RTI_CLOCK_SOURCE_32KHZ_FREQ_KHZ;
            break;
        case RTI_CLOCK_SOURCE_200MHZ:
            clkFreqKHz = (uint32_t) RTI_CLOCK_SOURCE_200MHZ_FREQ_KHZ;
            break;
        default:
            break;
    }
    /* Get the clock ticks for given time-out value */
    timeoutNumCycles = timeoutVal * clkFreqKHz;
    return timeoutNumCycles;
}

static void IntrDisable(uint32_t intsrc)
{
    uint32_t intrStatus;

    SDL_RTI_getStatus(SDL_INSTANCE_RTI, &intrStatus);
    SDL_RTI_clearStatus(SDL_INSTANCE_RTI, intrStatus);
    /* Clear ESM registers. */
#if defined(SOC_AM263X) || defined (SOC_AM263PX) || defined (SOC_AM261X)
    SDL_ESM_disableIntr(SDL_TOP_ESM_U_BASE, intsrc);
    SDL_ESM_clrNError(SDL_ESM_INST_MAIN_ESM0);
#endif
#if defined (SOC_AM273X) || defined (SOC_AWR294X)
    SDL_ESM_clrNError(SDL_INSTANCE_ESM0);
    SDL_ESM_disableIntr(SDL_INSTANCE_ESM0, intsrc);
#endif
}

#if defined (SOC_AM263X) || defined (SOC_AM64X) || defined (SOC_AM243X) || defined (SOC_AM263PX) || defined (SOC_AM261X)
int32_t SDL_ESM_applicationCallbackFunction(SDL_ESM_Inst esmInst, SDL_ESM_IntType esmIntrType,
                                            uint32_t grpChannel,  uint32_t index, uint32_t intSrc, void *arg)
{
    int32_t retVal = SDL_PASS;

    DebugP_log("\n   Interrupt is generated to ESM\r\n");
    DebugP_log("   ESM Call back function called : instType 0x%x, intType 0x%x, " \
                "grpChannel 0x%x, index 0x%x, intSrc 0x%x \r\n",
                esmInst, esmIntrType, grpChannel, index, intSrc);
    DebugP_log("  Take action \n");
	isrFlag = RTI_ESM_INTRPT;
    /* For clearing the interrupt */
    IntrDisable(intSrc);

    return retVal;
}
#elif defined (SOC_AM273X) || defined (SOC_AWR294X)
int32_t SDL_ESM_applicationCallbackFunction (SDL_ESM_Inst instance, int32_t grpChannel, int32_t vecNum, void *arg)
{
  int32_t retVal = SDL_PASS;
  IntrDisable(vecNum);

  isrFlag |= RTI_ESM_INTRPT;
  /* For clearing the interrupt */
  return retVal;
}
#endif

/********************************* End of file ******************************/
