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
 *  \file     vim_main.c
 *
 *  \brief    This file contains VIM Safety Example.
 *
 *  \details
 **/

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */
#include <stdio.h>
#include "vim_main.h"
#include <kernel/dpl/DebugP.h>
#include <sdl/sdl_ecc.h>
#include "ti_drivers_config.h"
#include "ti_drivers_open_close.h"
#include "ti_board_open_close.h"
/* ========================================================================== */
/*                                Macros                                      */
/* ========================================================================== */

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */

/* ========================================================================== */
/*                 Internal Function Declarations                             */
/* ========================================================================== */

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

/* ========================================================================== */
/*                 Internal Function Definitions                              */
/* ========================================================================== */

void SDL_ECC_applicationCallbackFunction(SDL_ECC_MemType eccMemType,
                                         uint32_t errorSrc,
                                         uint32_t address,
                                         uint32_t ramId,
                                         uint64_t bitErrorOffset,
                                         uint32_t bitErrorGroup){

    printf("\n  ECC Error Call back function called : eccMemType %d, errorSrc 0x%x, " \
                "address 0x%x, ramId %d, bitErrorOffset 0x%04x%04x, bitErrorGroup %d\n",
                eccMemType, errorSrc, address, ramId, (uint32_t)(bitErrorOffset >> 32),
                (uint32_t)(bitErrorOffset & 0x00000000FFFFFFFF), bitErrorGroup);
    printf("  Take action \n");

    /* Any additional customer specific actions can be added here */

}

static int32_t sdlApp_dplInit(void)
{
    SDL_ErrType_t ret = SDL_PASS;

    ret = SDL_TEST_dplInit();
    if (ret != SDL_PASS)
    {
        DebugP_log("Error: Init Failed\n");
    }

    return ret;
}

void VIM_main(void *args)
{
    /* Declaration of variables */
    int32_t  testResult;

	Drivers_open();
    Board_driversOpen();
    /* DPL Init */
    testResult = sdlApp_dplInit();

	if (testResult == SDL_PASS)
    {
		DebugP_log("R5 VIM Application\r\n");
        testResult = VIM_funcTest();
		if (testResult == SDL_PASS)
		{
			DebugP_log("All tests have passed. \r\n");
		}
		else
		{
			DebugP_log("Some tests have failed.\r\n");
		}
    }
    else
    {
        DebugP_log("DPL Init failed. Exiting the app.\r\n");
    }
	Board_driversClose();
    Drivers_close();

}

/* Nothing past this point */
