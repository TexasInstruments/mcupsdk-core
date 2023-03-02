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
 *  \file     mtog_main.c
 *
 *  \brief    This file contains MTOG Example code for R5 core.
 *
 *  \details  MTOG Safety Example
 **/

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */
#include "mtog_main.h"
#include <kernel/dpl/DebugP.h>
#include <kernel/dpl/HwiP.h>
#include <kernel/dpl/ClockP.h>
#include <kernel/dpl/AddrTranslateP.h>
#include <dpl_interface.h>
#include <drivers/soc/am64x_am243x/soc.h>
#include <sdl/include/am64x_am243x/sdlr_soc_baseaddress.h>
#include <sdl/include/am64x_am243x/sdlr_mcu_ctrl_mmr.h>
#include "ti_drivers_config.h"
#include "ti_drivers_open_close.h"
#include "ti_board_open_close.h"
/* ========================================================================== */
/*                 Internal Function Declarations                             */
/* ========================================================================== */

void test_sdl_mtog_test_app(void)
{
    /* @description: Run mtog tests
    @cores: mcu1_0 */
    int32_t  testResult;
    testResult = MTOG_func();

    if (testResult == SDL_PASS)
    {
        DebugP_log("\r\n MTOG Saftey Example passed. \r\n");
    }
    else
    {
        DebugP_log("\r\nMTOG Saftey Example failed. \r\n");
    }
	Board_driversClose();
    Drivers_close();
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

/* define the unlock and lock values */

int32_t mtog_main(void)
{
	Drivers_open();
    Board_driversOpen();
	
	sdlApp_dplInit();
	
	SOC_controlModuleUnlockMMR(SOC_DOMAIN_ID_MCU, 1);
	
	test_sdl_mtog_test_app(); 

    return (0);
}

/* Nothing past this point */

