/*
 *   Copyright (c) Texas Instruments Incorporated 2023
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
 *  \file     	edc_main.c
 *
 * \brief 		This file demonstrates using the Error Detection and Correction Module (EDC),
 *         		utilizing the EDC and ESM Software Diagnostic Reference (SDL) functions.
 *
 *  \details  	EDC Safety Example module tests
 **/

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */
#include <stdio.h>
#include "edc_main.h"
#include <sdl/dpl/sdl_dpl.h>
#include <kernel/dpl/TimerP.h>
#include <kernel/dpl/DebugP.h>
#include <sdl/sdl_ecc.h>
#include <dpl_interface.h>
#include <sdl/include/am273x/sdlr_dss_ecc_agg.h>

/* ========================================================================== */
/*                                Macros                                      */
/* ========================================================================== */
#define SDL_DSS_ECC_AGG_ECC_VECTOR_ADDR         (0x060A0008)
#define SDL_DSS_ECC_AGG_ERROR_STATUS1_ADDR      (0x060A0020)

#define SDL_DSS_ECC_AGG_RAM_IDS_TOTAL_ENTRIES   22

#define SDL_DSP_ICFG_DISABLE                    (0)
/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */
volatile bool esmError = false;
/* ========================================================================== */
/*                 Internal Function Declarations                             */
/* ========================================================================== */

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

/* ========================================================================== */
/*                 Internal Function Definitions                              */
/* ========================================================================== */

int32_t SDL_ESM_applicationCallbackFunction(SDL_ESM_Inst esmInstType,
										   int32_t grpChannel,
										   int32_t intSrc,
										   void *arg)
{
    
	DebugP_log("\r\nESM Call back function called : instType 0x%x, " \
                "grpChannel 0x%x, intSrc 0x%x \r\n",
                esmInstType, grpChannel, intSrc);
    DebugP_log("\r\nTake action \r\n");

	/* Write to DSP_ICFG__EDCINTMASK REGISTER and DSP_ICFG__EDCINTFLG REGISTER 
	 * TO disable PROPOGATION OF EXCEPTION
	 */

    SDL_REG32_WR(SDL_DSP_ICFG_EDCINTMASK, SDL_DSP_ICFG_DISABLE);
    SDL_REG32_WR(SDL_DSP_ICFG_EDCINTFLG, SDL_DSP_ICFG_DISABLE);
	
    esmError = true;

    return 0;
}/* End of SDL_ESM_applicationCallbackFunction() */

int32_t SDL_ECC_DED_applicationCallbackFunction(SDL_ESM_Inst esmInstType,
                                           int32_t grpChannel,
                                           int32_t intSrc,
                                           void *arg)
{

    DebugP_log("\r\nECC DED Call back function called : instType 0x%x, " \
                "grpChannel 0x%x, intSrc 0x%x \r\n",
                esmInstType, grpChannel, intSrc);
    DebugP_log("\r\nTake action \r\n");

    /* Write to DSP_ICFG__EDCINTMASK REGISTER and DSP_ICFG__EDCINTFLG REGISTER
     * TO disable PROPOGATION OF EXCEPTION
     */

    SDL_REG32_WR(SDL_DSP_ICFG_EDCINTMASK, SDL_DSP_ICFG_DISABLE);
    SDL_REG32_WR(SDL_DSP_ICFG_EDCINTFLG, SDL_DSP_ICFG_DISABLE);

    esmError = true;

    return 0;
}/* End of SDL_ECC_DED_applicationCallbackFunction() */

int32_t SDL_ECC_SEC_applicationCallbackFunction(SDL_ESM_Inst esmInstType,
                                           int32_t grpChannel,
                                           int32_t intSrc,
                                           void *arg)
{

    DebugP_log("\r\nECC SEC Call back function called : instType 0x%x, " \
                "grpChannel 0x%x, intSrc 0x%x \r\n",
                esmInstType, grpChannel, intSrc);
    DebugP_log("\r\nTake action \r\n");

    /* Write to DSP_ICFG__EDCINTMASK REGISTER and DSP_ICFG__EDCINTFLG REGISTER
     * TO disable PROPOGATION OF EXCEPTION
     */

    SDL_REG32_WR(SDL_DSP_ICFG_EDCINTMASK, SDL_DSP_ICFG_DISABLE);
    SDL_REG32_WR(SDL_DSP_ICFG_EDCINTFLG, SDL_DSP_ICFG_DISABLE);

    esmError = true;

    return 0;
}/* End of SDL_ECC_SEC_applicationCallbackFunction() */

void EDC_Example_app(void)
{
    int32_t    testResult;

    uint8_t i;
    /* Clear all status registers.*/
    for(i=0;i<=SDL_DSS_ECC_AGG_RAM_IDS_TOTAL_ENTRIES;i++)
    {
        /* Write the RAM ID in to Vector register*/
        SDL_REG32_FINS(SDL_DSS_ECC_AGG_ECC_VECTOR_ADDR, DSS_ECC_AGG_ECC_VECTOR_ECC_VECTOR, i);
        /* Clear pending interrupts.*/
        SDL_REG32_WR(SDL_DSS_ECC_AGG_ERROR_STATUS1_ADDR, 0xF0F);
        ClockP_usleep(100);
    }

	DebugP_log("\r\nEDC UC-1 Example\r\n");
	testResult = EDC_funcTest();

	if (testResult == SDL_PASS)
    {
        DebugP_log("\r\nAll Use_Cases have passed. \r\n");
    }
    else
    {
        DebugP_log("\r\nSome Use_Cases have failed. \r\n");
    }
	/* Close drivers to close the UART driver for console */
    Board_driversClose();
    Drivers_close();
}/* End of EDC_Example_app() */

void edc_main(void *args)
{
	/* Open drivers to open the UART driver for console */
	Drivers_open();
	Board_driversOpen();
    DebugP_log("\r\nEDC Example Application\r\n");
    (void)EDC_Example_app();
}/* end of edc_main() */

/* Nothing past this point */
