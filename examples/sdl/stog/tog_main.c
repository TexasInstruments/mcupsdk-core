/*
 * TOG Example
 *
 * Timeout Gasket (TOG) Example Application
 *
 *  Copyright (c) 2023 Texas Instruments Incorporated
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
 *  \file tog_main.c
 *
 *  \brief This file demonstrates using the Timeout Gasket (TOG)
 */

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <dpl_interface.h>
#include <kernel/dpl/DebugP.h>
#include <sdl/dpl/sdl_dpl.h>
#include "tog_main.h"
#include "ti_drivers_config.h"
#include "ti_drivers_open_close.h"
#include "ti_board_open_close.h"

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */


/* ========================================================================== */
/*                         Structure Declarations                             */
/* ========================================================================== */


/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

/* Unity functions */
void test_sdl_tog_example_app_runner(void);
void tog_example_app(void);
extern int32_t tog_minTimeout(uint32_t instanceIndex);

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */

/* ========================================================================== */
/*                            External Variables                              */
/* ========================================================================== */

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */


/*****************************************************************************/

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

void tog_example_app(void)
{
    int32_t    result = 0;
    uint32_t indexId = (uint32_t)SDL_TOG_INSTANCE_TIMEOUT0_CFG;
    /* For other Instance test, there have to be passed instance index */
    result = tog_minTimeout(indexId);
    if (result != 0)
    {
        DebugP_log(" \n  TOG example failed \n");
    }

    return;
}

void test_sdl_tog_example_app_runner(void)
{
    tog_example_app();
    return;
}

int32_t tog_test_main(void)
{
    Drivers_open();
    Board_driversOpen();

    /* Init dpl */
    sdlApp_dplInit();

    DebugP_log("\nTOG Sample Example \r\n");
    test_sdl_tog_example_app_runner();

    Board_driversClose();
    Drivers_close();

    return (0);
}
