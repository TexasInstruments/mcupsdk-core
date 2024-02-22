/*
 *
 * ROM Checksum Example
 *
 * ROm Checksum Example Application
 *
 *  Copyright (c) 2024 Texas Instruments Incorporated
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
 *  \file tmu_rom_checksum_main.c
 *
 *  \brief This file demonstrates using  TMU ROM Checksum SDL APIs to check the
 *         Data integrity of TMU ROM regions.
 */

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */
#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include <sdl/include/sdl_types.h>
#include <kernel/dpl/DebugP.h>
#include <sdl/dpl/sdl_dpl.h>
#include <dpl_interface.h>
#include <sdl/sdl_tmu_rom_checksum.h>
#include <sdl/sdl_mcrc.h>
#include "ti_drivers_open_close.h"
#include "ti_board_open_close.h"


/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                         Structure Declarations                             */
/* ========================================================================== */

/* None */

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */

/* ========================================================================== */
/*                            External Variables                              */
/* ========================================================================== */

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

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

void test_main(void)
{

    /* Declaration of variables */
    int32_t  testResult = SDL_EFAIL;

    Drivers_open();
	Board_driversOpen();
    /* Init Dpl */
    sdlApp_dplInit();

    DebugP_log("\nTMU ROM Checksum Example Application\r\n");
    DebugP_log("\nCalculating TMU ROM Checksum\r\n");

    SDL_MCRC_Signature_t  sectSignVal;

    testResult = SDL_TMU_ROM_Checksum_compute(SDL_MCRC_CHANNEL_1, &sectSignVal);

    if(testResult == SDL_PASS)
    {
        DebugP_log("\nTMU ROM-Checksum Data integrity passed\r\n");
        DebugP_log("\nCalcuated TMU CRC High : 0x%x\r\n", sectSignVal.regH);
        DebugP_log("\nCalcuated TMU CRC Low : 0x%x\r\n", sectSignVal.regL);
        DebugP_log("\nAll tests Passed\r\n");
    }
    else if(testResult == SDL_EBADARGS)
    {
        DebugP_log("\nCompute ROM-Checksum fails Because of invalid arguments\r\n");
    }
    else
    {
        DebugP_log("\nTMU ROM-Checksum Data integrity failed\r\n");
    }

    Board_driversClose();
	Drivers_close();
}
