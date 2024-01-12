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
 *  \file     dcc_uc1.c
 *
 *  \brief    This file contains DCC Example test code.
 *
 *  \details  DCC tests
 **/

/*===========================================================================*/
/*                         Include files                                     */
/*===========================================================================*/
#include "dcc_uc1.h"
#include <sdl/dpl/sdl_dpl.h>
#include <dpl_interface.h>
#include <kernel/dpl/DebugP.h>
#include "ti_drivers_open_close.h"
#include "ti_board_open_close.h"


/*===========================================================================*/
/*                         Declarations                                      */
/*===========================================================================*/
volatile uint32_t isrFlag = 0U;
/**< Flag used to indicate occurrence of the error interrupt */
volatile uint32_t doneIsrFlag = 0U;
/**< Flag used to indecate occurrence of the completion interrupt */
volatile SDL_DCC_Inst gCurDccInst;
#if defined (SOC_AM64X) || defined(SOC_AM243X)
#if defined (M4F_CORE)
#include <sdl/include/am64x_am243x/sdlr_intr_mcu_m4fss0_core0.h>
#define NUM_USE_CASES          (0x9U)

static DCC_TEST_UseCase DCC_Test_UseCaseArray[NUM_USE_CASES] =
{
    /* Continuous - error generated */
    {
        "HFOSC0",
        "SYSCLK0",
        SDL_DCC_INST_MCU_DCC0,
        SDL_DCC_CLK0_SRC_CLOCK0_0,
        25000, /* 25 MHz for HFOSC0 */
        SDL_DCC_CLK1_SRC_CLOCK1,
        200000, /* 200 MHz for SYSCLK0 */
        SDL_DCC_MODE_CONTINUOUS,
        0x0,
        APP_DCC_TEST_CLOCK_SRC_1_HIGHER,
        0x1
    },
    /* Single Shot - No error */
    {
        "HFOSC0",
        "SYSCLK0",
        SDL_DCC_INST_MCU_DCC0,
        SDL_DCC_CLK0_SRC_CLOCK0_0,
        25000, /* 25 MHz for HFOSC0 */
        SDL_DCC_CLK1_SRC_CLOCK1,
        200000, /* 200 MHz for SYSCLK0 */
        SDL_DCC_MODE_SINGLE_SHOT_2,
		SDLR_MCU_M4FSS0_CORE0_NVIC_MCU_DCC0_INTR_DONE_LEVEL_0,
        0xFFFF,
        0x0
    },
    /* Continuous - no error */
    {
        "HFOSC0",
        "SYSCLK0",
        SDL_DCC_INST_MCU_DCC0,
        SDL_DCC_CLK0_SRC_CLOCK0_0,
        25000, /* 25MHz for HFOSC0 */
        SDL_DCC_CLK1_SRC_CLOCK1,
        200000, /* 200 MHz for SYSCLK0 */
        SDL_DCC_MODE_CONTINUOUS,
        0x0,
        0xFFFF,
        0x0
    },
    /* Single Shot - No error */
    {
        "RC OSC",
        "SYSCLK0",
        SDL_DCC_INST_MCU_DCC0,
        SDL_DCC_CLK0_SRC_CLOCK0_2,
        12000, /* 12.5 MHz for RC OSC */
        SDL_DCC_CLK1_SRC_CLOCK1,
        200000, /* 200 MHz for SYSCLK0 */
        SDL_DCC_MODE_SINGLE_SHOT_2,
        SDLR_MCU_M4FSS0_CORE0_NVIC_MCU_DCC0_INTR_DONE_LEVEL_0,
        0xFFFF,
        0x0
    },
    /* Continuous - error generated */
    {
        "RC OSC",
        "SYSCLK0",
        SDL_DCC_INST_MCU_DCC0,
        SDL_DCC_CLK0_SRC_CLOCK0_2,
        12000, /* 12.5 MHz for RC OSC */
        SDL_DCC_CLK1_SRC_CLOCK1,
        200000, /* 200 MHz for SYSCLK0 */
        SDL_DCC_MODE_CONTINUOUS,
        0x0,
        APP_DCC_TEST_CLOCK_SRC_1_HIGHER,
        0x1
    },
    /* Continuous - no error */
    {
        "RC OSC",
        "SYSCLK0",
        SDL_DCC_INST_MCU_DCC0,
        SDL_DCC_CLK0_SRC_CLOCK0_2,
        12000, /* 12.5 MHz for RC OSC */
        SDL_DCC_CLK1_SRC_CLOCK1,
        200000, /* 200 MHz for SYSCLK0 */
        SDL_DCC_MODE_CONTINUOUS,
        0x0,
        0xFFFF,
        0x0
    },
    /* Continuous - no error */
    {
        "HFOSC0",
        "MAIN_SYSCLK0",
        SDL_DCC_INST_DCC0,
        SDL_DCC_CLK0_SRC_CLOCK0_0,
        25000, /* 25 MHz for HSOSC0 */
        SDL_DCC_CLK1_SRC_CLOCKSRC6,
        500000, /* 500 MHz for MAIN_SYSCLK0 */
        SDL_DCC_MODE_CONTINUOUS,
        0x0,
        0xFFFF,
        0x0
    },
    /* Continuous - error generated */
    {
        "HFOSC0",
        "MAIN_SYSCLK0",
        SDL_DCC_INST_DCC0,
        SDL_DCC_CLK0_SRC_CLOCK0_0,
        25000, /* 25 MHz for HSOSC0 */
        SDL_DCC_CLK1_SRC_CLOCKSRC6,
        500000, /* 500 MHz for MAIN_SYSCLK0 */
        SDL_DCC_MODE_CONTINUOUS,
        0x0,
        SDL_DCC2_DCCCLKSRC1_CLKSRC_OTHER,
        0x1
    },
    /* Continuous - no error */
    {
        "RC OSC",
        "MAIN_SYSCLK0",
        SDL_DCC_INST_DCC0,
        SDL_DCC_CLK0_SRC_CLOCK0_2,
        12000, /* 12.5 MHz for RC OSC */
        SDL_DCC_CLK1_SRC_CLOCKSRC6,
        500000, /* 500 MHz for MAIN_SYSCLK0 */
        SDL_DCC_MODE_CONTINUOUS,
        0x0,
        0xFFFF,
        0x0
    },
};
#endif
#if defined (R5F_CORE)
#include <sdl/include/am64x_am243x/sdlr_intr_r5fss0_core0.h>
#define NUM_USE_CASES          (0x9U)

static DCC_TEST_UseCase DCC_Test_UseCaseArray[NUM_USE_CASES] =
{
    /* Continuous - error generated */
    {
        "HFOSC0",
        "SYSCLK0",
        SDL_DCC_INST_MCU_DCC0,
        SDL_DCC_CLK0_SRC_CLOCK0_0,
        25000, /* 25 MHz for HFOSC0 */
        SDL_DCC_CLK1_SRC_CLOCK1,
        200000, /* 200 MHz for SYSCLK0 */
        SDL_DCC_MODE_CONTINUOUS,
        0x0,
        APP_DCC_TEST_CLOCK_SRC_1_HIGHER,
        0x1
    },
    /* Single Shot - No error */
    {
        "HFOSC0",
        "SYSCLK0",
        SDL_DCC_INST_MCU_DCC0,
        SDL_DCC_CLK0_SRC_CLOCK0_0,
        25000, /* 25 MHz for HFOSC0 */
        SDL_DCC_CLK1_SRC_CLOCK1,
        200000, /* 200 MHz for SYSCLK0 */
        SDL_DCC_MODE_SINGLE_SHOT_2,
        SDLR_R5FSS0_CORE0_INTR_MCU_DCC0_INTR_DONE_LEVEL_0,
        0xFFFF,
        0x0
    },
    /* Continuous - no error */
    {
        "HFOSC0",
        "SYSCLK0",
        SDL_DCC_INST_MCU_DCC0,
        SDL_DCC_CLK0_SRC_CLOCK0_0,
        25000, /* 25MHz for HFOSC0 */
        SDL_DCC_CLK1_SRC_CLOCK1,
        200000, /* 200 MHz for SYSCLK0 */
        SDL_DCC_MODE_CONTINUOUS,
        0x0,
        0xFFFF,
        0x0
    },
    /* Single Shot - No error */
    {
        "RC OSC",
        "SYSCLK0",
        SDL_DCC_INST_MCU_DCC0,
        SDL_DCC_CLK0_SRC_CLOCK0_2,
        12000, /* 12.5 MHz for RC OSC */
        SDL_DCC_CLK1_SRC_CLOCK1,
        200000, /* 200 MHz for SYSCLK0 */
        SDL_DCC_MODE_SINGLE_SHOT_2,
		SDLR_R5FSS0_CORE0_INTR_MCU_DCC0_INTR_DONE_LEVEL_0,
        0xFFFF,
        0x0
    },
    /* Continuous - error generated */
    {
        "RC OSC",
        "SYSCLK0",
        SDL_DCC_INST_MCU_DCC0,
        SDL_DCC_CLK0_SRC_CLOCK0_2,
        12000, /* 12.5 MHz for RC OSC */
        SDL_DCC_CLK1_SRC_CLOCK1,
        200000, /* 200 MHz for SYSCLK0 */
        SDL_DCC_MODE_CONTINUOUS,
        0x0,
        APP_DCC_TEST_CLOCK_SRC_1_HIGHER,
        0x1
    },
    /* Continuous - no error */
    {
        "RC OSC",
        "SYSCLK0",
        SDL_DCC_INST_MCU_DCC0,
        SDL_DCC_CLK0_SRC_CLOCK0_2,
        12000, /* 12.5 MHz for RC OSC */
        SDL_DCC_CLK1_SRC_CLOCK1,
        200000, /* 200 MHz for SYSCLK0 */
        SDL_DCC_MODE_CONTINUOUS,
        0x0,
        0xFFFF,
        0x0
    },
    /* Continuous - no error */
    {
        "HFOSC0",
        "MAIN_SYSCLK0",
        SDL_DCC_INST_DCC0,
        SDL_DCC_CLK0_SRC_CLOCK0_0,
        25000, /* 25 MHz for HSOSC0 */
        SDL_DCC_CLK1_SRC_CLOCKSRC6,
        500000, /* 500 MHz for MAIN_SYSCLK0 */
        SDL_DCC_MODE_CONTINUOUS,
        0x0,
        0xFFFF,
        0x0
    },
    /* Continuous - error generated */
    {
        "HFOSC0",
        "MAIN_SYSCLK0",
        SDL_DCC_INST_DCC0,
        SDL_DCC_CLK0_SRC_CLOCK0_0,
        25000, /* 25 MHz for HSOSC0 */
        SDL_DCC_CLK1_SRC_CLOCKSRC6,
        500000, /* 500 MHz for MAIN_SYSCLK0 */
        SDL_DCC_MODE_CONTINUOUS,
        0x0,
        SDL_DCC2_DCCCLKSRC1_CLKSRC_OTHER,
        0x1
    },
    /* Continuous - no error */
    {
        "RC OSC",
        "MAIN_SYSCLK0",
        SDL_DCC_INST_DCC0,
        SDL_DCC_CLK0_SRC_CLOCK0_2,
        12000, /* 12.5 MHz for RC OSC */
        SDL_DCC_CLK1_SRC_CLOCKSRC6,
        500000, /* 500 MHz for MAIN_SYSCLK0 */
        SDL_DCC_MODE_CONTINUOUS,
        0x0,
        0xFFFF,
        0x0
    },
};
#endif
#endif

/*===========================================================================*/
/*                         Macros                                            */
/*===========================================================================*/
/* None */

/*===========================================================================*/
/*                         Internal function declarations                    */
/*===========================================================================*/

void test_sdl_dcc_test_app (void);

static void SDL_DCCAppPrint(char * str);

/**
 * \brief   This function waits infinitely for DCC done interrupt
 *
 * \retval  SDL_PASS on occurrence DCC completion and no error.
 *          SDL_EFAIL otherwise.
 */
static int32_t SDL_DCCAppWaitForCompletion();

/*===========================================================================*/
/*                         Global Variables                                  */
/*===========================================================================*/
#if defined (SOC_AM64X) || defined(SOC_AM243X)
#if defined (M4F_CORE)
SDL_ESM_config DCC_Test_esmInitConfig_MCU =
{
    .esmErrorConfig = {0u, 3u}, /* Self test error config - not used in this test*/
    .enableBitmap = {0x00000007u, 0x00000020u, 0x00000000u,
                },
     /**< Enabling Main domain ESM output and MCU Domain DCC events */
    .priorityBitmap = {0x0000003u, 0x00000020u, 0x00000000u,
                        },
    /**< All events high priority: except low-priority Main ESM output */
    .errorpinBitmap = {0x00000003u, 0x00000020u, 0x00000000,
                      },
    /**< All high priority events to error pin */
};

SDL_ESM_config DCC_Test_esmInitConfig_Main =
{
    .esmErrorConfig = {0u, 3u}, /* Self test error config - not used in this test*/
    .enableBitmap = {0x00000000u, 0x00000000u, 0x00000000u, 0x0003f000u,
                     0x00000000u, 0x00000000u,
                },
     /**< Enabling Main domain ESM output and MCU Domain DCC events */
    .priorityBitmap = {0x0000000u, 0x00000000u, 0x00000000u, 0x0003f000u,
                       0x0000000u, 0x00000000u,
                        },
    /**< All events high priority: except low-priority Main ESM output */
    .errorpinBitmap = {0x00000000u, 0x00000000u, 0x00000000, 0x0003f000u,
                       0x00000000u, 0x00000000u,
                      },
    /**< All high priority events to error pin */
};
#endif
#if defined (R5F_CORE)

SDL_ESM_config DCC_Test_esmInitConfig_MCU =
{

     /**< All high priority events to error pin */
        .esmErrorConfig = {1u, 8u}, /* Self test error config */
    .enableBitmap = {0x00000000u, 0x00000006bu, 0x00000000u, 0x00000000u,
                 0x00000200u, 0x00400380u,

                },
     /**< All events enable: except clkstop events for unused clocks
      *   and PCIE events */
    .priorityBitmap = {0x00000000u, 0x00000006bu, 0x00000000u, 0x00000000u,
                 0x00000200u, 0x00400380u,

                        },
    /**< All events high priority: except clkstop events for unused clocks
     *   and PCIE events */
    .errorpinBitmap = {0x00000000u, 0x00000006bu, 0x00000000u, 0x00000000u,
                 0x00000200u, 0x00400380u,

                      },
    /**< All events high priority: except clkstop for unused clocks
     *   and PCIE events */


};

SDL_ESM_config DCC_Test_esmInitConfig_Main =
{


  /**< All high priority events to error pin */
        .esmErrorConfig = {1u, 8u}, /* Self test error config */
    .enableBitmap = {0x00000000u, 0x000000078u, 0x00000000u,0x003f0000u,
                 0x00000200u, 0x00040380u,

                },
     /**< All events enable: except clkstop events for unused clocks
      *   and PCIE events */
    .priorityBitmap = {0x00000000u, 0x000000078u, 0x00000000u,0x003f0000u,
                 0x00000200u, 0x00040380u,
                        },
    /**< All events high priority: except clkstop events for unused clocks
     *   and PCIE events */
    .errorpinBitmap = {0x00000000u, 0x000000078u, 0x00000000u,0x003f0000u,
                 0x00000200u, 0x00040380u,

                      },
    /**< All events high priority: except clkstop for unused clocks
     *   and PCIE events */


 };
#endif
#endif

/*===========================================================================*/
/*                   Local Function definitions                              */
/*===========================================================================*/

static void SDL_DCCAppPrint(char * str)
{
    DebugP_log(str);
}

static int32_t sdlApp_dplInit(void)
{
    SDL_ErrType_t ret = SDL_PASS;

    ret = SDL_TEST_dplInit();
    if (ret != SDL_PASS)
    {
        DebugP_log("Error: Init Failed\r\n");
    }

    return ret;
}

static void SDL_DCCAppGetClkRatio(uint32_t  refClkFreq,
                              uint32_t  testClkFreq,
                              uint32_t *refClkRatioNum,
                              uint32_t *testClkRatioNum)
{
    uint32_t loopCnt, hcf = 1U;

    for (loopCnt = 1;
         (loopCnt <= refClkFreq) && (loopCnt <= testClkFreq);
         loopCnt++)
    {
        if ((refClkFreq % loopCnt == 0) && (testClkFreq % loopCnt == 0))
        {
            hcf = loopCnt;
        }
    }
    *refClkRatioNum  = (refClkFreq / hcf);
    *testClkRatioNum = (testClkFreq / hcf);
}

static void SDL_DCCAppSetSeedVals(uint32_t       refClkFreq,
                                  uint32_t       testClkFreq,
                                  uint32_t       refClkRatioNum,
                                  uint32_t       testClkRatioNum,
                                  uint32_t       drfitPer,
                                  SDL_DCC_Config *configParams)
{
    uint32_t maxFreqKHz, maxCntLimit;
    uint32_t maxRefCnt, minRefCnt;
    uint64_t mulVar;

    /* Find maximum frequency between test and reference clock */
    if (refClkFreq > testClkFreq)
    {
        maxFreqKHz  = refClkFreq;
        maxCntLimit = APP_DCC_SRC0_MAX_VAL;
    }
    else
    {
        maxFreqKHz  = testClkFreq;
        maxCntLimit = APP_DCC_SRC1_MAX_VAL;
    }
    /* Calculate seed values for 0% drift */
    if (maxFreqKHz == refClkFreq)
    {
        configParams->clk0Seed = maxCntLimit / refClkRatioNum;
        configParams->clk0Seed = configParams->clk0Seed * refClkRatioNum;
        mulVar = ((uint64_t) (configParams->clk0Seed) *
                  (uint32_t) (testClkRatioNum));
        configParams->clk1Seed   = (uint32_t) (mulVar / refClkRatioNum);
        configParams->clk0ValidSeed = refClkRatioNum;
    }
    else
    {
        configParams->clk1Seed = maxCntLimit / testClkRatioNum;
        configParams->clk1Seed = configParams->clk1Seed * testClkRatioNum;
        mulVar = ((uint64_t) (configParams->clk1Seed) *
                  (uint32_t) (refClkRatioNum));
        configParams->clk0Seed   = (uint32_t) (mulVar / testClkRatioNum);
        configParams->clk0ValidSeed = 1U;
    }
    /* Applying allowed drift */
    if (((APP_DCC_SRC0_MAX_VAL + APP_DCC_SRC0_VALID_MAX_VAL) <
         (configParams->clk0Seed * (100U + drfitPer) / 100U)))
    {
        /* Seed values with drift exceeds maximum range */
        SDL_DCCAppPrint(APP_DCC_STR ": Seed values with drift exceeds"
                        " allowed range\r\n");
        SDL_DCCAppPrint(APP_DCC_STR ": Application will run with 0% "
                        " allowed drift\r\n");
    }
    else if (100U < drfitPer)
    {
        /* Error percentage is greater than 100 */
        SDL_DCCAppPrint(APP_DCC_STR ": Warning Wrong drift %,Not applying drift\r\n");
        SDL_DCCAppPrint(APP_DCC_STR ": Application will run with 0% drift\r\n");
    }
    else
    {
        maxRefCnt = (configParams->clk0Seed * (100U + drfitPer) / 100U);
        minRefCnt = (configParams->clk0Seed * (100U - drfitPer) / 100U);
        if (APP_DCC_SRC0_VALID_MAX_VAL < (maxRefCnt - minRefCnt))
        {
            SDL_DCCAppPrint(APP_DCC_STR ": Warning Seed value for valid count "
                        "exceeds allowed range.\r\n");
            SDL_DCCAppPrint(APP_DCC_STR ": Application will run with 0 allowed"
                        " drift.\r\n");
        }
        else
        {
            if (maxRefCnt == minRefCnt)
            {
                configParams->clk0ValidSeed = 1U;
            }
            else
            {
                configParams->clk0Seed   = minRefCnt;
                configParams->clk0ValidSeed = (maxRefCnt - minRefCnt);
            }
        }
    }
    SDL_DCCAppPrint(APP_DCC_STR ": Seed values calculation done.\r\n");
}

int32_t SDL_ESM_applicationCallbackFunction(SDL_ESM_Inst esmInst, SDL_ESM_IntType esmIntrType,
                                            uint32_t grpChannel,  uint32_t index, uint32_t intSrc, void *arg)
{
    int32_t retVal = SDL_PASS;

   DebugP_log("\r\nInterrupt is generated to ESM\r\n");
   DebugP_log("    ESM Call back function called : instType 0x%x, intType 0x%x, " \
               "grpChannel 0x%x, index 0x%x, intSrc 0x%x \r\n",
               esmInst, esmIntrType, grpChannel, index, intSrc);
   DebugP_log("    Take action \r\n");

    isrFlag = DCC_INTERRUPT;

    /* Clear DCC event */
    SDL_DCC_clearIntr(gCurDccInst, SDL_DCC_INTERRUPT_ERR);


    return retVal;
}

static void SDL_DCCAppDoneIntrISR(void *arg)
{
    SDL_DCC_Status dccStatus;

    SDL_DCC_getStatus(gCurDccInst, &dccStatus);

    if (dccStatus.doneIntr == TRUE)
    {
        SDL_DCC_clearIntr(gCurDccInst, SDL_DCC_INTERRUPT_DONE);
        doneIsrFlag  = 1U;
    }
}

static int32_t SDL_DCCAppRegisterIsr(uint32_t uc, pSDL_DPL_HwipHandle *handle)
{
    int32_t retVal = SDL_EFAIL;
    SDL_DPL_HwipParams intrParams;

    intrParams.intNum      = DCC_Test_UseCaseArray[uc].intNum;
    intrParams.callback    = &SDL_DCCAppDoneIntrISR;
    intrParams.callbackArg = 0x0;

    /* Register call back function for DCC Done interrupt */
    retVal = SDL_DPL_registerInterrupt(&intrParams, handle);

    return (retVal);
}

static void SDL_DCCAppDeRegisterIsr(pSDL_DPL_HwipHandle handle)
{
    SDL_DPL_deregisterInterrupt(handle);
}

static int32_t SDL_DCCAppWaitForCompletion(void)
{
    int32_t retVal;

    /* Wait for completion interrupt / or error flag*/
    while ((0U == doneIsrFlag) && (0U == isrFlag));

    /* Ensure no error */
    if (isrFlag == DCC_INTERRUPT)
    {
        retVal = SDL_EFAIL;
    }
    else
    {
        retVal = SDL_PASS;
    }
    return (retVal);
}

/*===========================================================================*/
/*                         Function definitions                              */
/*===========================================================================*/

void test_sdl_dcc_test_app (void)
{
    /* Declarations of variables */
    int32_t  retVal;
    uint32_t clk0Freq, clk1Freq, refClkRatioNum, testClkRatioNum;
    SDL_DCC_Config configParams;
    uint32_t i;

    DebugP_log("\r\n DCC Example Test Application\r\n");

    /* Init Dpl */
    sdlApp_dplInit();

    /* Initialize MCU DCC module */
    retVal = SDL_ESM_init(SDL_ESM_INST_MCU_ESM0, &DCC_Test_esmInitConfig_MCU, SDL_ESM_applicationCallbackFunction, NULL);

    if (retVal != SDL_PASS)
    {
        /* print error and quit */
        DebugP_log("DCC_Test_init: Error initializing MCU ESM: result = %d\r\n", retVal);
    }
    else
    {
        DebugP_log("\r\nDCC_Test_init: Init MCU ESM complete \r\n");
        retVal = SDL_ESM_init(SDL_ESM_INST_MAIN_ESM0, &DCC_Test_esmInitConfig_Main, SDL_ESM_applicationCallbackFunction, NULL);

        if (retVal != SDL_PASS)
        {
            /* print error and quit */
            DebugP_log("DCC_Test_init: Error initializing ESM: result = %d\r\n", retVal);
        }
    }

    for (i = 0; i < NUM_USE_CASES; i++)
    {

        DebugP_log("\r\nUSECASE: %d\r\n", i);

        DebugP_log("Source clock: %s \r\n", DCC_Test_UseCaseArray[i].srcStr);
        DebugP_log("Test clock: %s\r\n", DCC_Test_UseCaseArray[i].testStr);

        gCurDccInst = DCC_Test_UseCaseArray[i].dccInst;
        clk0Freq = DCC_Test_UseCaseArray[i].clk0Freq;
        clk1Freq = DCC_Test_UseCaseArray[i].clk1Freq;

        if (DCC_Test_UseCaseArray[i].errorTest == 0x1)
        {
            /* Deliberately change the Reference Clock to 2 times to
             * introduce the error in the clock ratio
             */
            clk1Freq *= 2;
        }

        if (SDL_PASS == retVal)
        {
            /* Get clock ratio */
            SDL_DCCAppGetClkRatio(clk0Freq,
                                  clk1Freq,
                                  &refClkRatioNum,
                                  &testClkRatioNum);


            configParams.mode    = DCC_Test_UseCaseArray[i].mode;
            configParams.clk0Src = DCC_Test_UseCaseArray[i].clk0;
            configParams.clk1Src = DCC_Test_UseCaseArray[i].clk1;

            /* Get the seed values for given clock selections and allowed drift */
            SDL_DCCAppSetSeedVals(clk0Freq,
                                  clk1Freq,
                                  refClkRatioNum,
                                  testClkRatioNum,
                                  APP_DCC_TEST_CLOCK_SRC_1_DRIFT,
                                  &configParams);


            retVal = SDL_DCC_configure(DCC_Test_UseCaseArray[i].dccInst, &configParams);


            if (SDL_PASS == retVal)
            {
                retVal = SDL_DCC_verifyConfig(DCC_Test_UseCaseArray[i].dccInst, &configParams);

            }
            else
            {
                retVal = SDL_EFAIL;
            }

            if (retVal == SDL_PASS)
            {


                /* Enable ERROR interrupt */
                SDL_DCC_enableIntr(DCC_Test_UseCaseArray[i].dccInst, SDL_DCC_INTERRUPT_ERR);

                /*
		 * Check for single-shot mode and enable interrupt for Done notification
                 * then wait for completion.
                 */
                if (DCC_Test_UseCaseArray[i].mode != SDL_DCC_MODE_CONTINUOUS)
                {

                    pSDL_DPL_HwipHandle handle;

                    SDL_DCCAppRegisterIsr(i, &handle);

                    /* Enable DONE interrupt(only for single shot mode) */
                    SDL_DCC_enableIntr(DCC_Test_UseCaseArray[i].dccInst, SDL_DCC_INTERRUPT_DONE);

                    SDL_DCC_enable(DCC_Test_UseCaseArray[i].dccInst);


                    if (SDL_PASS == SDL_DCCAppWaitForCompletion())
                    {
                        SDL_DCCAppPrint(APP_DCC_STR ": DCC Generated completion interrupt \r\n");
                        SDL_DCCAppPrint(APP_DCC_STR ": No Clock Drift was observed \r\n");
                    }
                    else
                    {
                        SDL_DCCAppPrint(APP_DCC_STR ": Error : DCC Generated error interrupt\r\n");
                        SDL_DCCAppPrint(APP_DCC_STR ": Error interrupt is not expected \r\n");
                        retVal = SDL_EFAIL;
                    }

                    SDL_DCCAppDeRegisterIsr(handle);
                    isrFlag = DCC_NO_INTERRUPT;
                    doneIsrFlag = 0x0;
                }
                else
                {
                    if (DCC_Test_UseCaseArray[i].errorTest == 0x1)
                    {
                        SDL_DCCAppPrint(APP_DCC_STR ": Enabling DCC and waiting for "
                                        "Error interrupt \r\n");
                    }
                    else
                    {
                        SDL_DCCAppPrint(APP_DCC_STR ": Enabling DCC and running for some time \r\n");
                    }

                    SDL_DCC_enable(DCC_Test_UseCaseArray[i].dccInst);

                    /* Wait for error notification */

                    volatile int32_t j = 0;
                    /* Wait for the ESM interrupt to report the error */
                    do {
                        j++;
                        if (j > 0x0FFFFFF)
                        {
                            /* Timeout for the wait */

							//DebugP_log("\r\nDebug dcc_uc1 line no %d (TIMEOUT) \r\n",__LINE__);
                            break;
                        }

                    } while (isrFlag == DCC_NO_INTERRUPT);


                    if (isrFlag == DCC_INTERRUPT)
                    {
                        DebugP_log(APP_DCC_STR ": DCC Generated Error interrupt \r\n");
                        DebugP_log(APP_DCC_STR ": Indicating clock drift/change \r\n");
                        if (DCC_Test_UseCaseArray[i].errorTest == 0x0)
                        {
                            DebugP_log(APP_DCC_STR ":    Error Event was not expected \r\n");
                            retVal = SDL_EFAIL;
                        }
                    }
                    else
                    {
                        if (DCC_Test_UseCaseArray[i].errorTest == 0x1)
                        {
                            DebugP_log(APP_DCC_STR ": Could not generate Error interrupt \r\n");
                            retVal = SDL_EFAIL;
                        }
                    }
                    isrFlag = DCC_NO_INTERRUPT;
                }
                SDL_DCC_disable(DCC_Test_UseCaseArray[i].dccInst);
            }
        }
        else
        {
             DebugP_log(APP_DCC_STR ": Error : Could not derive clock "
                        "frequency!!!\r\n");
        }

        if (retVal != SDL_PASS)
        {
            DebugP_log("UC-%d Failed\r\n", i);
            break;
        }
        else
        {
            DebugP_log("UC-%d Completed Successfully\r\n", i);
        }
    }

    if (retVal == SDL_PASS)
    {
        DebugP_log("\r\n All tests have passed. \r\n");
    }
    else
    {
        DebugP_log("\r\n Few/all tests Failed \r\n");
    }
}


int32_t dcc_test_main(void)
{
	Drivers_open();
	Board_driversOpen();
    test_sdl_dcc_test_app();
	Board_driversClose();
	Drivers_close();
    /* Stop the test and wait here */
    while (1);
}

/* Nothing past this point */
