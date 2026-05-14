/*
 *   Copyright (c) Texas Instruments Incorporated 2023-2025
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
 *  \file     ecc_main.c
 *
 * \brief This file demonstrates using the Error Correcting Code Module (ECC),
 *        utilizing the ECC and ESM Software Diagnostic Reference (SDL) functions.
 *
 *  \details  ECC TCM Safety Example
 **/

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */
#include "ecc_main.h"
#include <drivers/sciclient.h>
#include <sdl/dpl/sdl_dpl.h>
#include <kernel/dpl/TimerP.h>
#include <kernel/dpl/DebugP.h>
#include <sdl/sdl_ecc.h>
#include <dpl_interface.h>
#include "ti_drivers_open_close.h"
#include "ti_board_open_close.h"
#include <sdl/r5/v0/sdl_r5_utils.h>
#include <sdl/ecc/sdl_ecc_utils.h>
#include <sdl/sdl_exception.h>
#include <sdl/r5/v0/sdl_interrupt.h>

/* ========================================================================== */
/*                                Macros                                      */
/* ========================================================================== */

#define SDL_PULSAR_EVNT_BUS_ESM_CLR  (0x3C01801C) // should we also clear in r5fss1?

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */
volatile bool esmError = false;
int32_t status = SDL_PASS;

/* ========================================================================== */
/*                 Internal Function Declarations                             */
/* ========================================================================== */

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

/* ========================================================================== */
/*                 Internal Function Definitions                              */
/* ========================================================================== */


int32_t SDL_ESM_applicationCallbackFunction(SDL_ESM_Inst esmInst,
                                            SDL_ESM_IntType esmIntrType,
                                            uint32_t grpChannel,
                                            uint32_t index,
                                            uint32_t intSrc,
                                            uintptr_t *arg)
{
    uint32_t evntbus_esm_clr;

    /* To clear the ESM ECC events generated from R5FSS Core Memory,
     * read and write back the register SDL_PULSAR_EVNT_BUS_ESM_CLR */
    /* Read PULSAR event register. */
    evntbus_esm_clr = SDL_REG32_RD(SDL_PULSAR_EVNT_BUS_ESM_CLR);
    /* Write back the same data to clear the events */
    SDL_REG32_WR(SDL_PULSAR_EVNT_BUS_ESM_CLR, evntbus_esm_clr);

    esmError = true;

    return 0;
}

static int32_t sdlApp_dplInit(void)
{
    SDL_ErrType_t ret = SDL_PASS;

    ret = SDL_TEST_dplInit();
    if (ret != SDL_PASS)
    {
        DebugP_log("\r\nError: Init Failed\r\n");
    }
    return ret;
}

void ECC_Example_app(void *args)
{
    int32_t    testResult;

  	testResult = ECC_funcTest();

  	DebugP_log("\r\nECC UC-1 and UC-2 Test\r\n");
  	if (testResult == SDL_PASS)
    {
  	    DebugP_log("\r\nAll tests have passed. \r\n");
    }
    else
    {
        DebugP_log("\r\nSome tests have failed. \r\n");
    }
}

void ecc_app_runner(void)
{
    ECC_Example_app(ECC_Example_app);
}

int32_t ecc_main(void)
{
    sdlApp_dplInit();

    DebugP_log("\r\nECC TCM Example Application\r\n");

    /*Enabling the ECC for TCM*/
    SDL_ECC_UTILS_enableECCATCM();
    SDL_ECC_UTILS_enableECCB0TCM();
    SDL_ECC_UTILS_enableECCB1TCM();
    /*Enabling the Event bus*/
    SDL_UTILS_enable_event_bus();

    (void)ecc_app_runner();

    return 0;
}

/* Nothing past this point */
