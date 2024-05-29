/*
 *  Copyright (c) 2021-2024 Texas Instruments Incorporated
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
 *  \brief    This file contains RTI DWWD Example test code.
 *
 **/

 #include <stdint.h>
 #include <string.h>
 #include <stdio.h>
 #include <drivers/soc.h> /* needed for clocks */
 #include "sdlexample.h"
 #include <sdl/sdl_rti.h>

 /* ========================================================================== */
 /*                     Dependant macros in sdl_rti_funcTest.c                 */
 /* ========================================================================== */

 #define RTI_WDT_TIMEOUT     (10000U)
 #define SOC_MODULES_END     (0xFFFFFFFFu)

 /** RTI Clock Source Selection */
 #define RTI_CLOCK_SOURCE_32KHZ_FREQ_KHZ                                     (32U)
 #define RTI_CLOCK_SOURCE_200MHZ_FREQ_KHZ                                    (200000U)
 #define MCU_ESM_RTI0_INTR                                                   (104U)
 #define MCU_ESM_RTI1_INTR                                                   (105U)
 #define SDL_MCU_CTRL_MMR_CFG0_MCU_RTI1_CLKSEL_CLK_SEL_MASK                  (0x00000007U)
 #define SDL_MCU_CTRL_MMR0_CFG0_BASE                                         (0x4500000UL)
 #define SDL_MCU_CTRL_MMR_CFG0_MCU_RTI0_CLKSEL                               (0x00008180U)
 #define SDL_MCU_CTRL_MMR_CFG0_MCU_RTI0_CLKSEL_CLK_SEL_MASK                  (0x00000007U)
 #define SDL_MCU_CTRL_MMR_CFG0_MCU_RTI0_CLKSEL_CLK_SEL_SHIFT                 (0x00000000U)
 #define SDL_MCU_CTRL_MMR_CFG0_MCU_RTI1_CLKSEL                               (0x00008184U)
 #define SDL_MCU_CTRL_MMR_CFG0_MCU_RTI1_CLKSEL_CLK_SEL_SHIFT                 (0x00000000U)
 #define SDL_MCU_CTRL_MMR_CFG0_MCU_RTI1_CLKSEL_CLK_SEL_MAX                   (0x00000007U)
 #define SDLR_MCU_R5FSS0_CORE0_INTR_MCU_RTI0_INTR_WWD_0                      (42U)
 #define SDLR_MCU_R5FSS0_CORE0_INTR_MCU_ESM0_ESM_INT_HI_LVL_0                (49U)
 #define SDLR_MCU_R5FSS0_CORE0_INTR_MCU_RTI1_INTR_WWD_0                      (43U)
 #define RTI_DWD_MIN_PRELOAD_VAL                                             (0x1FFFU)

 /** Flag used to indicate interrupt is generated */
 #define RTI_NO_INTERRUPT                       (0U)
 #define RTI_CPU_INTERRUPT                      (1U)
 #define RTI_ESM_INTRPT                         (2U)


 /*===========================================================================*/
 /*                         Declarations                                      */
 /*===========================================================================*/

 /* Define the test interface */
 typedef struct sdlRtiTest_s
 {
     int32_t  (*testFunction)(void);   /* The code that runs the test */
     char      *name;                  /* The test name */
     int32_t    testStatus;            /* Test Status */
 } sdlRtiTest_t;

 typedef enum rtiClockSource
 {
     RTI_CLOCK_SOURCE_HFOSC0_CLKOUT = 0U,
     /**< to select clock frequency of hfosc0 */
     RTI_CLOCK_SOURCE_LFOSC_CLKOUT = 1U,
     /**< to select clock frequency of lfosc */
     RTI_CLOCK_SOURCE_12MHZ = 2U,
     /**< to select clock frequency of 12 MHz */
     RTI_CLOCK_SOURCE_32KHZ = 3U,
     /**< to select clock frequency of 32KHz */
     RTI_CLOCK_SOURCE_200MHZ = 4U,
     /**< to select clock frequency of 200 MHz */
 }rtiClockSource_t;

 /*===========================================================================*/
 /*                         Macros                                            */
 /*===========================================================================*/

/*===========================================================================*/
/*                         Macros                                            */
/*===========================================================================*/
/* None */

#define SDL_INSTANCE_RTI SDL_INSTANCE_WDT0
#define SDL_WDT_BASE SDL_WDT0_U_BASE

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */
typedef struct {

    uint32_t moduleId;
    uint32_t clkId;
    uint32_t clkRate;

} SOC_SDL_ModuleClockFrequency;

 SOC_SDL_ModuleClockFrequency sdl_gSocModulesClockFrequency[] = {
    { SOC_RcmPeripheralId_WDT0, SOC_RcmPeripheralClockSource_SYS_CLK, 32000 },

    { SOC_MODULES_END, SOC_MODULES_END, SOC_MODULES_END },
};

static int32_t Sdl_Module_clockEnable()
{
  int32_t status;

  status =  SOC_moduleClockEnable(SOC_RcmPeripheralId_WDT0, 1);

  return status;
}
static int32_t Sdl_Module_clockSetFrequency()
{
    int32_t status;
    status = SOC_moduleSetClockFrequency(
                    sdl_gSocModulesClockFrequency[0].moduleId,
                    sdl_gSocModulesClockFrequency[0].clkId,
                    sdl_gSocModulesClockFrequency[0].clkRate
                    );
  return status;
}

  SDL_RTI_configParms     pConfig;
  uint32_t rtiModule = SDL_WDT_BASE;

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

static uint32_t RTIGetPreloadValue(uint32_t rtiClkSource, uint32_t timeoutVal);
int32_t RTIDwwdIsClosedWindow(uint32_t baseAddr, uint32_t *pWinStatus);

static inline uint32_t RTIDwwdReadTimeOut(uint32_t baseAddr)
{
    uint32_t timeOutValue;

    timeOutValue = ((HW_RD_REG32(baseAddr + RTI_RTIDWDPRLD)
                     << RTI_DWWDPRLD_MULTIPLIER_SHIFT) |
                    RTI_DWD_MIN_PRELOAD_VAL);

    return timeOutValue;
}
/* internal function to read the window size */
static inline uint32_t RTIDwwdReadWinSz(uint32_t baseAddr)
{
    uint32_t   windowSize;
    /* Get configured Window Size */
    windowSize = HW_RD_REG32(baseAddr + RTI_RTIDWWDSIZECTRL);
    return (windowSize);
}

/* Clear the esm for RTI events */
void rti_clearESM(uint32_t intSrc)
{
  uint32_t   intrStatus;
  SDL_RTI_getStatus(SDL_INSTANCE_WDT0, &intrStatus);
  SDL_RTI_clearStatus(SDL_INSTANCE_WDT0, intrStatus);
  /* Clear ESM registers. */
  SDL_ESM_disableIntr(SDL_TOP_ESM_U_BASE, intSrc);
  SDL_ESM_clrNError(SDL_ESM_INST_MAIN_ESM0);
  sdlstats.rti.isrFlag  |= RTI_ESM_INTRPT;
  return;
}

int32_t RTIDwwdIsClosedWindow(uint32_t baseAddr, uint32_t *pIsClosedWindow)
{
    uint32_t closedWindowstatus, currentDownCounter, windowSizeShift;
    uint32_t windowStartTime, timeOutValue, windowSize;

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

int32_t RTI_uc1(void)
{
    uint32_t                rtiModule, closedWinStatus;
    int32_t                 retVal = SDL_PASS;
    SDL_RTI_configParms     pConfig;
    SDL_RTI_staticRegs      pStaticRegs;

   sdlstats.rti.isrFlag = RTI_NO_INTERRUPT;
   rtiModule = SDL_WDT_BASE;

   Sdl_Module_clockEnable();
   Sdl_Module_clockSetFrequency();

    /* Register Interrupt */
    sdlstats.rti.isrFlag = RTI_NO_INTERRUPT;

    /* Configure RTI parameters */
    pConfig.SDL_RTI_dwwdPreloadVal = RTIGetPreloadValue(RTI_CLOCK_SOURCE_200MHZ_FREQ_KHZ, RTI_WDT_TIMEOUT);
    pConfig.SDL_RTI_dwwdWindowSize = RTI_DWWD_WINDOWSIZE_100_PERCENT;
    pConfig.SDL_RTI_dwwdReaction   = RTI_DWWD_REACTION_GENERATE_NMI;

    retVal = SDL_RTI_config(SDL_INSTANCE_RTI, &pConfig);

    if (retVal == SDL_EFAIL)
    {
        return retVal;
    }

    /* Verify the config */
    retVal = SDL_RTI_verifyConfig(SDL_INSTANCE_RTI, &pConfig);

    if (retVal != SDL_PASS)
    {
        return retVal;
    }

    SDL_RTI_readStaticRegs(SDL_INSTANCE_RTI, &pStaticRegs);
    sdlstats.rti.isrFlag = RTI_NO_INTERRUPT;

		 retVal = SDL_RTI_start(SDL_INSTANCE_RTI);
     if (retVal != SDL_PASS)
     {
        return retVal;
      }

      /* Servicing DWWD before testing for window end */
      RTIDwwdIsClosedWindow(rtiModule, &closedWinStatus);

      while (closedWinStatus == TRUE)
      {
          RTIDwwdIsClosedWindow(rtiModule, &closedWinStatus);
          /* Keep checking till window is open. */
      }

      retVal = SDL_RTI_service(SDL_INSTANCE_RTI);

      if (sdlstats.rti.isrFlag != RTI_NO_INTERRUPT)
      {
          /* DWWD interrupt is generated when it is not expected to. */
          retVal = SDL_EFAIL;
      }

      /* clear esm error for next test - interrupt may be not be generated */
      sdlstats.esmcb = ESMCB_NONE;

    return retVal;
}

/* ========================================================================== */
/*                 Internal Function Definitions                              */
/* ========================================================================== */


#if defined (R5F_INPUTS)
static void RTISetClockSource(uint32_t rtiModuleSelect,
                              uint32_t rtiClockSourceSelect)
{
    switch (rtiModuleSelect) {
        case SDL_WDT0_U_BASE:
            HW_WR_FIELD32(SDL_MCU_CTRL_MMR0_CFG0_BASE +
                          SDL_MCU_CTRL_MMR_CFG0_MCU_RTI0_CLKSEL,
                          SDL_MCU_CTRL_MMR_CFG0_MCU_RTI0_CLKSEL_CLK_SEL,
                          rtiClockSourceSelect);
            break;

        default:
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


/********************************* End of file ******************************/
