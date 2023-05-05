/*
 *   Copyright (c) Texas Instruments Incorporated 2022-2023
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
 *  \file     mtog_func.c
 *
 *  \brief    This file contains MTOG functional code. .
 *
 *  \details  MTOG Safety Example
 **/

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */
#include <string.h>
#include <stdint.h>
#include <kernel/dpl/DebugP.h>
#include <kernel/dpl/HwiP.h>
#include <kernel/dpl/ClockP.h>
#include "mtog_main.h"
#include <sdl/include/sdl_types.h>
#include <sdl/sdl_mtog.h>
#include <sdl/esm/v0/sdl_esm.h>
#include <sdl/mtog/soc/sdl_soc_mtog.h>
#include <sdl/esm/v0/v0_0/sdl_esm_priv.h>
#include <sdl/include/am64x_am243x/sdlr_intr_mcu_esm0.h>
#include <sdl/esm/v0/esm.h>
#include <sdl/esm/soc/sdl_esm_soc.h>

/* ========================================================================== */
/*                                Macros                                      */
/* ========================================================================== */
#define MAIN_ESM_BASE 				   (SDL_ESM0_CFG_BASE)
#define MTOG_MAX_TIMEOUT_VALUE         (1000000000u)
/* ========================================================================== */
/*                 Internal Function Declarations                             */
/* ========================================================================== */
void MTOG_datAbortExceptionHandler(void *param);

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */
int32_t SDL_ESM_applicationCallbackFunction(SDL_ESM_Inst esmInst,
                                                   SDL_ESM_IntType esmIntrType,
                                                   uint32_t grpChannel,
                                                   uint32_t index,
                                                   uint32_t intSrc,
                                                   void *arg);
void MTOG_eventHandler( uint32_t instanceIndex );
int32_t MTOG_runTest(uint32_t instanceIndex);

typedef void (*MTOG_handlerPtr)(uint32_t instanceIndex);

int32_t apparg;
/** Instance name */
char instanceName[16] = "MCU MTOG0";
/** ESM error event number */
uint32_t ESMEventNumber;
/** Flag to indicate test done, will be set when interrupt event comes */
volatile bool doneFlag;

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */
/* Initialization structure for MCU ESM instance */
static SDL_ESM_config MTOG_Example_esmInitConfig_MCU =
{
    .esmErrorConfig = {0u, 3u}, /* Self test error config */
    .enableBitmap = {0x04000000u, 0x00000000u, 0x00000000u, 0x00000000u,
                },
     /**< All events enable: except timer and self test  events, */
    /*    and Main ESM output.Configured based off esmErrorConfig to test high or low priorty events.*/
    .priorityBitmap = {0x04000000u, 0x00000000u, 0x00000000u, 0x00000000u,
                },
    /**< Configured based off esmErrorConfig to test high or low priorty events. */
    .errorpinBitmap = {0x04000000u, 0x00000000u, 0x00000000u, 0x00000000u,
                },
};

static void IntrDisable(uint32_t intrSrc)
{
    /* Clear ESM registers. */
    uint32_t baseAddr = 0U;
    SDL_ESM_getBaseAddr(SDL_ESM_INST_MCU_ESM0, &baseAddr);
    SDL_ESM_disableIntr(baseAddr, intrSrc);
    SDL_ESM_clrNError(SDL_ESM_INST_MCU_ESM0);

}

void MTOG_eventHandler( uint32_t instanceIndex )
{
    int32_t status = SDL_PASS;

    /* Reset the Timeout gasket */
    status = SDL_MTOG_reset( instanceIndex );

    if (status == SDL_PASS)
    {
        DebugP_log("\r\n MTOG Reset done\r\n");
    }
    else{
        DebugP_log("\r\n MTOG Reset failed");
    }
    doneFlag = true;
    return;
}

int32_t MTOG_runTest(uint32_t instanceIndex)
{
    int32_t result = 0;
    int32_t status=0;
    uint64_t startTime,testStartTime,testEndTime, endTime;
    uint64_t prepTime, diffTime, restoreTime;
    volatile uint32_t timeoutCount = 0;
    uint32_t mtog_base_addr=0x0;
    SDL_MTOG_getBaseaddr(SDL_INSTANCE_MCU_MTOG0, &mtog_base_addr);
    SDL_MTOG_config config;
    config.timeOut = SDL_MTOG_VAL_1K;
    SDL_MTOG_staticRegs staticRegs;

	ESMEventNumber         = SDLR_MCU_ESM0_ESM_LVL_EVENT_MCU_MASTER_SAFETY_GASKET0_TIMED_OUT_0 ;
    doneFlag               = false;

    DebugP_log("\r\n Starting MTOG test on %s, index %d...",
                instanceName,
                instanceIndex);

#ifdef DEBUG
    char inputChar;

    DebugP_log("\r\n Press 'n' to skip..Press any key to continue...");
    inputChar = UART_getc();

    if (inputChar == 'n')
    {
        DebugP_log("   Skipping this test. on request \r\n");
        return 0;
    }
    DebugP_log("\r\n  HwiP_Params_init complete \r\n");
#endif

    /* Initialize done flag */
    doneFlag = false;

    /* Get start time of test */
    startTime = ClockP_getTimeUsec();
    if (status != SDL_PASS)
    {
        DebugP_log("   sdlAppEsmConfig Failed \r\n");
        result = -1;
    }

    /** Step 2: Configure and start Master Timeout Gasket */
    if (result == 0)
    {
        SDL_MTOG_reset(instanceIndex);
        result = SDL_MTOG_init(instanceIndex, &config);
        if (status != SDL_PASS)
        {
            DebugP_log("   SDL_MTOG_init Failed \r\n");
            result = -1;
        }
    }
    if (result == 0)
    {
        result = SDL_MTOG_verifyConfig(instanceIndex, &config);
        if (status != SDL_PASS)
        {
            DebugP_log("   SDL_MTOG_verifyConfig Failed \r\n");
            result = -1;
        }
    }
	if (result == 0)
    {
        result = SDL_MTOG_getStaticRegisters(instanceIndex, &staticRegs);
        if (status != SDL_PASS)
        {
            DebugP_log("   SDL_MTOG_getStaticRegisters Failed \r\n");
            result = -1;
        }
    }
    if (result == 0)
    {
        /* Call SDL API to enable Timeout Gasket */
        status = SDL_MTOG_start(instanceIndex);
        if (status != SDL_PASS)
        {
            DebugP_log("   SDL_MTOG_start Failed \r\n");
            result = -1;
        }
    }
    /* Get start time of test */
    testStartTime = ClockP_getTimeUsec();
    /* Step 3: Inject master timeout error */
	if (result == 0)
  {
      status = SDL_MTOG_forceTimeout(instanceIndex);
      if (status != SDL_PASS)
      {
          DebugP_log("\r\n SDL_MTOG_forceTimeout Failed \r\n");
          result = -1;
      }
  }
    /**--- Step 3: Wait for MTOG Interrupt ---*/
    if (result == 0)
    {
        /* Timeout if exceeds time */
        while ((!doneFlag)
               && (timeoutCount++ < MTOG_MAX_TIMEOUT_VALUE))
        {
                /* Use Polling */
                MTOG_eventHandler(instanceIndex);
        }
        if(timeoutCount >= MTOG_MAX_TIMEOUT_VALUE)
        {
            DebugP_log("\r\n MTOG Timed out  \r\n");
            result = -1;
        }
    }
    /* Get end time of test */
     testEndTime = ClockP_getTimeUsec();

    /**--- Step 4: Disable ESM ---*/
    if (result == 0)
    {
	    status=SDL_ESM_disableIntr(SDL_ESM_INST_MCU_ESM0, ESMEventNumber);
        if (status != SDL_PASS)
        {
            DebugP_log("   sdlAppEsmDisable Failed \r\n");
            result = -1;
        }
    }

    /* Here MTOG test is complete , get end time of test */
    endTime = ClockP_getTimeUsec();

    prepTime = testStartTime - startTime;
    diffTime = testEndTime - testStartTime;
    restoreTime = endTime - testEndTime;
    DebugP_log("\r\n Delta MTOG prep time in micro secs %d \r\n", (uint32_t)prepTime );
    DebugP_log(" \r\n Delta MTOG execution time in micro secs %d \r\n", (uint32_t)diffTime );
    DebugP_log(" \r\n  Delta MTOG restore time in micro secs %d \r\n", (uint32_t)restoreTime );

    DebugP_log("  MTOG complete for %s \r\n",instanceName);

    return (result);
}

void MTOG_datAbortExceptionHandler(void *param)
{
    /* This is a fake exception so return */
}

int32_t SDL_ESM_applicationCallbackFunction(SDL_ESM_Inst esmInst, SDL_ESM_IntType esmIntrType,
                                            uint32_t grpChannel,  uint32_t index, uint32_t intSrc, void *arg)
{
    int32_t retVal = SDL_PASS;
    MTOG_eventHandler(1);
    DebugP_log("\r\nInterrupt is generated to ESM\r\n");
    DebugP_log("    ESM Call back function called : instType 0x%x, intType 0x%x, " \
                "grpChannel 0x%x, index 0x%x, intSrc 0x%x \r\n",
                esmInst, esmIntrType, grpChannel, index, intSrc);
    DebugP_log("  Take action \r\n");

    /* For clearing the interrupt */
    IntrDisable(intSrc);

    return retVal;
}

/* MTOG prepare for test */
int32_t MTOG_PrepareForTest(void)
{
    int32_t sdlResult = SDL_PASS;
	int32_t retValue=0;
	sdlResult = SDL_ESM_init(SDL_ESM_INST_MCU_ESM0, &MTOG_Example_esmInitConfig_MCU, SDL_ESM_applicationCallbackFunction, &apparg);
    if (sdlResult != SDL_PASS) {
        /* print error and quit */
        DebugP_log("\r\n TIMER_ESM_init: Error initializing MCU ESM: result = %d\r\n", sdlResult);

        retValue = -1;
    } else {
        DebugP_log("\r\nTIMER_ESM_init: Init MCU ESM complete \r\n");
    }
    return retValue;
}

/* MTOG Functional test */
int32_t MTOG_func(void)
{
    int32_t    result = 0;
	uint32_t instanceIndex;
    result = MTOG_PrepareForTest();
    if (result != 0)
    {
        DebugP_log("   MTOG_PrepareForTest failed \r\n");
    }
    if (result == 0)
    {
      instanceIndex = SDL_INSTANCE_MCU_MTOG0;
	  result = MTOG_runTest(instanceIndex);
    }
    return (result);
}
/* Nothing past this point */
