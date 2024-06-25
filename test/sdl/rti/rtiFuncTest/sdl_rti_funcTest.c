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
 *  \file     sdl_rti_funcTest.c
 *
 *  \brief    This file contains RTI DWWD Function test code.
 *
 **/


#include "rti_main.h"



/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */

volatile uint32_t isrFlag = RTI_NO_INTERRUPT;
/**< Flag used to indicate interrupt is generated */

	SDL_RTI_configParms     pConfig;
    uint32_t rtiModule = SDL_WDT_BASE;

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */
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

#if defined (R5F_INPUTS)
static void RTISetClockSource(uint32_t rtiModuleSelect,
                              uint32_t rtiClockSourceSelect)
{
#if !defined(SOC_TPR12) && !defined (SOC_AWR294X) /* No need to set clock for TPR12 */
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
                          SDL_MCU_CTRL_MMR_CFG0_MCU_RTI1_CLKSEL,
                          SDL_MCU_CTRL_MMR_CFG0_MCU_RTI1_CLKSEL_CLK_SEL,
                          rtiClockSourceSelect);
            break;
#endif
        default:
            DebugP_log ("Error : RTI Instance not supported "
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

static void RTIAppExpiredDwwdService(uint32_t rtiModule, uint32_t rtiWindow_size)
{
	#if defined (SOC_AM64X) || defined (SOC_AM243X)
    #if defined (M4F_CORE)
	uint32_t getBaseAddr;
	SDL_RTI_getBaseaddr(rtiModule,&getBaseAddr);
	rtiModule=getBaseAddr;
    #endif
	#endif
    /* Set dwwd window size to 100 percent. */
    SDL_RTI_writeWinSz(rtiModule, RTI_DWWD_WINDOWSIZE_100_PERCENT);
    /* Servicing watchdog will generate error. */
    SDL_RTI_service(SDL_INSTANCE_RTI);
    SDL_RTI_writeWinSz(rtiModule, rtiWindow_size);
    /* Service watchdog again. */
    SDL_RTI_service(SDL_INSTANCE_RTI);
}

int32_t SDL_RTI_funcTest(void)
{
    uint32_t closedWinStatus;
    int32_t  retVal = SDL_PASS;
    SDL_RTI_staticRegs         pStaticRegs;
    DebugP_log("RTI Function test started\r\n");

    /* Register Interrupt */
    isrFlag = RTI_NO_INTERRUPT;

    /* Configure RTI parameters */
    pConfig.SDL_RTI_dwwdPreloadVal = RTIGetPreloadValue(RTI_CLOCK_SOURCE_32KHZ, RTI_WDT_TIMEOUT);
    pConfig.SDL_RTI_dwwdWindowSize = RTI_DWWD_WINDOWSIZE_100_PERCENT;
    pConfig.SDL_RTI_dwwdReaction   = RTI_DWWD_REACTION_GENERATE_NMI;

	#if defined (SOC_AM64X) || defined (SOC_AM243X)
	/* Select RTI module clock source */
    RTISetClockSource(rtiModule, RTI_CLOCK_SOURCE_32KHZ);
	#endif
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
                DebugP_log("    DWWD configured to 100 percent window size\r\n");
                break;
            case RTI_RTIDWWDSIZECTRL_DWWDSIZE_50_PERCENT:
                DebugP_log("    DWWD configured to 50 percent window size\r\n");
                break;
            case RTI_RTIDWWDSIZECTRL_DWWDSIZE_25_PERCENT:
                DebugP_log("    DWWD configured to 25 percent window size\r\n");
                break;
            case RTI_RTIDWWDSIZECTRL_DWWDSIZE_12_5_PERCENT:
                DebugP_log("    DWWD configured to 6.25 percent window size\r\n");
                break;
            case RTI_RTIDWWDSIZECTRL_DWWDSIZE_6_25_PERCENT:
                DebugP_log("    DWWD configured to 3.125 percent window size\r\n");
                break;
        }

        /* Configure RTI and do not service. Generates End time violation. */
        DebugP_log("    DWWD is configured for %u ms time-out \r\n", RTI_WDT_TIMEOUT);

        DebugP_log("    DWWD will generate interrupt after "
            "above time-out period.\r\n");
        SDL_RTI_start(SDL_INSTANCE_RTI);
        /* Let DWWD expire here */
        DebugP_log("\nWait for %u ms for interrupt "
            "to be generated by DWWD.\r\n", RTI_WDT_TIMEOUT);

        while (RTI_NO_INTERRUPT == isrFlag)
        {
            /* Wait for interrupt */
        }
        isrFlag = RTI_NO_INTERRUPT;

        if (retVal == SDL_PASS)
        {
            DebugP_log("\nRTI End time violation test successful. \r\n\n");
        }
        else
        {
            DebugP_log("RTI End time violation test failed. \r\n");
        }
    }


    if ((retVal == SDL_PASS) &&
        (pConfig.SDL_RTI_dwwdWindowSize != RTI_DWWD_WINDOWSIZE_100_PERCENT))
    {
        /* RTI is serviced in closed window. Generates DWWD violation.
         * Closed window violation cant be generated for 100% window size.
         */
        volatile uint32_t loopBreak = FALSE;
        isrFlag = RTI_NO_INTERRUPT;

        while (loopBreak == FALSE)
        {
            DebugP_log("RTI DWWD closed window violation test running. \r\n");
            RTIDwwdIsClosedWindow(rtiModule, &closedWinStatus);
            if (closedWinStatus == TRUE)
            {
                loopBreak = TRUE;
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
          DebugP_log("RTI DWWD closed window violation test successful. \r\n\n");
        }
        else
        {
            DebugP_log("RTI DWWD closed window violation test failed. \r\n");
        }
    }


    if (retVal == SDL_PASS)
    {
        int32_t numIteration = 3;
        DebugP_log("    RTI DWWD proper servicing test running. \r\n");
        DebugP_log("    Please wait for max %d ms. \r\n\n", 3 * RTI_WDT_TIMEOUT);

        isrFlag = RTI_NO_INTERRUPT;

        /* Service DWWD in open window multiple times. Should not generate interrupt.
         * servicing 3 times, will run RTI for more than configured timeout for all window sizes. */
        while (numIteration-- >= 0)
        {
            RTIDwwdIsClosedWindow(rtiModule, &closedWinStatus);
            while (closedWinStatus == TRUE)
            {
                RTIDwwdIsClosedWindow(rtiModule, &closedWinStatus);
                /* Keep checking till window is open. */
            }
            SDL_RTI_service(SDL_INSTANCE_RTI);
        }
        if (isrFlag != RTI_NO_INTERRUPT)
        {
            /* DWWD interrupt is generated when it is not expected to. */
            retVal = SDL_EFAIL;
        }
        if (retVal == SDL_PASS)
        {
           DebugP_log("RTI DWWD proper servicing test successful. \r\n\n");
        }
        else
        {
            DebugP_log("RTI DWWD proper servicing test failed. \r\n");
        }
    }
    return retVal;
}

/* ========================================================================== */
/*                 Internal Function Definitions                              */
/* ========================================================================== */
#if defined(SOC_TPR12) || defined (SOC_AWR294X)
#define SDL_TEST_ESM_INT_HI_LVL SDL_MSS_INTR_MSS_ESM_HI
#endif

static void IntrDisable(uint32_t intsrc)
{
    uint32_t intrStatus;
#if defined (SOC_AM263X) || defined (SOC_AM263PX) || defined (SOC_AM261X)
    SDL_RTI_getStatus(SDL_INSTANCE_WDT0, &intrStatus);
    SDL_RTI_clearStatus(SDL_INSTANCE_WDT0, intrStatus);
#elif defined (R5F_INPUTS)
    SDL_RTI_getStatus(SDL_INSTANCE_MSS_WDT, &intrStatus);
    SDL_RTI_clearStatus(SDL_INSTANCE_MSS_WDT, intrStatus);
	RTIAppExpiredDwwdService(rtiModule, pConfig.SDL_RTI_dwwdWindowSize);
#elif defined (C66_INPUTS)
    SDL_RTI_getStatus(SDL_INSTANCE_DSS_WDT, &intrStatus);
    SDL_RTI_clearStatus(SDL_INSTANCE_DSS_WDT, intrStatus);
    RTIAppExpiredDwwdService(rtiModule, pConfig.SDL_RTI_dwwdWindowSize);
#endif
#if defined (SOC_AM64X) || defined (SOC_AM243X)
#if defined (M4F_CORE)
	SDL_RTI_getStatus(SDL_INSTANCE_MCU_RTI0_CFG, &intrStatus);
    SDL_RTI_clearStatus(SDL_INSTANCE_MCU_RTI0_CFG, intrStatus);
    SDL_ESM_clrNError(SDL_ESM_INST_MCU_ESM0);
#endif
#endif
#if defined (SOC_AM64X) || defined (SOC_AM243X)
#if defined (R5F_CORE)
	SDL_RTI_getStatus(SDL_INSTANCE_RTI8_CFG, &intrStatus);
    SDL_RTI_clearStatus(SDL_INSTANCE_RTI8_CFG, intrStatus);
    SDL_ESM_clrNError(SDL_ESM_INST_MAIN_ESM0);
#endif
#endif
   /* Clear ESM registers. */
#if defined (SOC_AM263X) || defined (SOC_AM263PX) || defined (SOC_AM261X)
    SDL_ESM_disableIntr(SDL_ESM_U_BASE, intsrc);
    SDL_ESM_clrNError(SDL_INSTANCE_ESM0);
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

    DebugP_log("\nInterrupt is generated to ESM\n");
    DebugP_log("    ESM Call back function called : instType 0x%x, intType 0x%x, " \
                "grpChannel 0x%x, index 0x%x, intSrc 0x%x \r\n",
                esmInst, esmIntrType, grpChannel, index, intSrc);
    DebugP_log("  Take action \r\n");

    /* For clearing the interrupt */
    IntrDisable(intSrc);

    isrFlag  |= RTI_ESM_INTRPT;

    return retVal;
}
#elif defined (SOC_AM273X) || defined (SOC_AWR294X)
int32_t SDL_ESM_applicationCallbackFunction (SDL_ESM_Inst instance, int32_t grpChannel, int32_t vecNum, void *arg)
{
    int32_t retVal = SDL_PASS;
    IntrDisable(vecNum);

    isrFlag  |= RTI_ESM_INTRPT;

    /* For clearing the interrupt */
    return retVal;
}
#endif

/********************************* End of file ******************************/
