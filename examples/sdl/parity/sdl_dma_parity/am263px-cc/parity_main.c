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
 *  \file       parity_main.c
 *
 * \brief       This file demonstrates the parity error injection,
 *              utilizing the ECC and ESM Software Diagnostic Reference (SDL) functions.
 *
 *  \details    TCM Parity Safety Example module tests
 **/

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */
#include <stdio.h>
#include "ti_drivers_config.h"
#include "ti_drivers_open_close.h"
#include "ti_board_open_close.h"
#include "parity_main.h"
#include <sdl/include/sdl_types.h>
#include <sdl/dpl/sdl_dpl.h>
#include <kernel/dpl/TimerP.h>
#include <kernel/dpl/DebugP.h>
#include <sdl/sdl_ecc.h>
#include <dpl_interface.h>
#include <sdl/ecc/soc/am263px/sdl_ecc_soc.h>
/* ========================================================================== */
/*                                Macros                                      */
/* ========================================================================== */

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */
volatile bool esmError = false;
uint32_t  errormask =  0xFFFFFFFF;
uint32_t  errorunmask =  0xFFFFFF0F;
uint32_t  clearstatusbit = 0x10000;
uint32_t  clearerraggbit =  0x10;
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

    int32_t retVal = 0;

    printf("\r\nESM Call back function called : instType 0x%x, " \
                "grpChannel 0x%x, intSrc 0x%x \r\n",
                esmInst, grpChannel, intSrc);
    printf("\r\nTake action \r\n");
	/* setting control register to clear TPCC status */
	SDL_REG32_WR(SDL_R5FSS0_CORE0_TPCC0_PARITY_CTRL, clearstatusbit);
	/* clearing error aggregator status bit  */
	SDL_REG32_WR(SDL_TPCC0_ERRAGG_STATUS, clearerraggbit);

	esmError = true;

    return retVal;
}

void parity_main(void *args)
{
	int32_t    testResult;
	uint32_t mask=0u;

    /* Open drivers to open the UART driver for console */
    Drivers_open();

	/* masking TPCC0 error aggregator  */
	SDL_REG32_WR(SDL_TPCC0_ERRAGG_MASK,errormask);
    DebugP_log("\r\nParity Example Application\r\n");
	/* unmasking TPCC0_ERRAGG_MASK register  */
	mask = errorunmask & SDL_REG32_RD(SDL_TPCC0_ERRAGG_MASK);
	SDL_REG32_WR(SDL_TPCC0_ERRAGG_MASK,mask);
    testResult = Parity_funcTest();

    if (testResult == SDL_PASS)
    {
        DebugP_log("\r\nAll tests have passed. \r\n");
    }
    else
    {
        DebugP_log("\r\nSome tests have failed. \r\n");
    }

    /* Close drivers to close the UART driver for console */
    Drivers_close();

}

/* Nothing past this point */
