/*
 *   Copyright (c) Texas Instruments Incorporated 2024
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
 *  \file     parity_trigger.c
 *
 *  \brief    This file contains functions that provide input event triggers
 *            for the TCM Parity application.
 *
 *  \details  TCM Parity Safety Example module tests
 **/

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */
#include <stdint.h>
#include <stdio.h>
#include <sdl/include/sdl_types.h>
#include <sdl/sdl_ecc.h>
#include <dpl_interface.h>
#include <kernel/dpl/DebugP.h>
#include <sdl/dpl/sdl_dpl.h>
#include <sdl/include/am263x/sdlr_soc_ecc_aggr.h>
#include <kernel/dpl/TimerP.h>
#include <sdl/sdl_ecc.h>
#include <sdl/ecc/soc/am263x/sdl_ecc_soc.h>

#include "sdlexample.h"

/* ========================================================================== */
/*                                Macros                                      */
/* ========================================================================== */
/* value to write into param register */
#define PARAM_REG_VALUE  0x7u
/* Enable testmode and parity in TPCC0_PARITY_CTRL register */
#define PARITY_ENABLE    0x11u

/**
*	  Developer can enable or disable below macros for the testing off
*   TCM different banks
**/
#define SDL_R5F0ATCM0_MASK							(0x7U)
#define SDL_R5F0B0TCM0_MASK							(0x700U)
#define	SDL_R5F0B1TCM0_MASK							(0x70000U)

#define SDL_R5F0ATCM1_MASK							(0x70U)
#define SDL_R5F0B0TCM1_MASK							(0x7000U)
#define	SDL_R5F0B1TCM1_MASK							(0x700000U)

#define SDL_R5F1ATCM0_MASK							(0x7U)
#define SDL_R5F1B0TCM0_MASK							(0x700U)
#define	SDL_R5F1B1TCM0_MASK							(0x70000U)

#define SDL_R5F1ATCM1_MASK							(0x70U)
#define SDL_R5F1B0TCM1_MASK							(0x7000U)
#define	SDL_R5F1B1TCM1_MASK							(0x700000U)

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */
volatile static bool paritytcm_esmError = false;
static int32_t  setValue=0x7u; /* for clearing TCM registers */



/* ========================================================================== */
/*                 Internal Function Declarations                             */
/* ========================================================================== */


/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

/* handles ESM callback */
int32_t ParityTCM_clear(void)
{
    int32_t retVal = SDL_PASS;

  sdlstats.tcm.statusRegValue0 = SDL_REG32_RD(SDL_R5FSS0_CORE0_TCM_ERR_STATUS);
  sdlstats.tcm.statusRegValue1 = SDL_REG32_RD(SDL_R5FSS0_CORE1_TCM_ERR_STATUS);
  sdlstats.tcm.statusRegValue2 = SDL_REG32_RD(SDL_R5FSS1_CORE0_TCM_ERR_STATUS);
  sdlstats.tcm.statusRegValue3 = SDL_REG32_RD(SDL_R5FSS1_CORE1_TCM_ERR_STATUS);

	SDL_cleartcmStatusRegs(setValue);

	paritytcm_esmError = true;

  return retVal;
}


/*********************************************************************
 *     ParityTCM_test
 *
 *    TCM parity test
 *
 *    None
 *
 * return  0 : Success; < 0 for failures
 **********************************************************************/
int32_t ParityTCM_test(void)
{
  int32_t retVal = SDL_PASS;
	int32_t result = SDL_PASS;

	/* R5FSS0_0 */
	if (retVal == SDL_PASS)
    {
        result = SDL_ECC_tcmParity(SDL_R5SS0_CPU0_TCM,\
								 SDL_R5FSS0_CORE0_ATCM0,\
								 SDL_R5F0ATCM0_MASK);
		/* Wait until ESM interrupt happens */
		while(paritytcm_esmError !=true)
    ;
		paritytcm_esmError = false;
		if (result != SDL_PASS)
		{
			retVal = -1;
		}
		else{
			retVal = SDL_PASS;
		}
	}
	if (retVal == SDL_PASS)
    {
		result = SDL_ECC_tcmParity(SDL_R5SS0_CPU0_TCM,\
								 SDL_R5FSS0_CORE0_B0TCM0,\
								 SDL_R5F0B0TCM0_MASK);
		/* Wait until ESM interrupt happens */
		while(paritytcm_esmError !=true);
		paritytcm_esmError = false;
		if (result != SDL_PASS)
		{
			retVal = -1;
		}
		else{
			retVal = SDL_PASS;
		}
	}

	if (retVal == SDL_PASS)
    {
        result = SDL_ECC_tcmParity(SDL_R5SS0_CPU0_TCM,\
								 SDL_R5FSS0_CORE0_B1TCM0,\
								 SDL_R5F0B1TCM0_MASK);
		/* Wait until ESM interrupt happens */
		while(paritytcm_esmError !=true);
		paritytcm_esmError = false;
		if (result != SDL_PASS)
		{
			retVal = -1;
		}
		else{
			retVal = SDL_PASS;
		}
	}

/* R5FSS0_1 */

	if (retVal == SDL_PASS)
    {
        result = SDL_ECC_tcmParity(SDL_R5SS0_CPU0_TCM,\
								 SDL_R5FSS0_CORE1_ATCM1,\
								 SDL_R5F0ATCM1_MASK);
		/* Wait until ESM interrupt happens */
		while(paritytcm_esmError !=true);
		paritytcm_esmError = false;
		if (result != SDL_PASS)
		{
			retVal = -1;
		}
		else{
			retVal = SDL_PASS;
		}
	}

	if (retVal == SDL_PASS)
    {
        result = SDL_ECC_tcmParity(SDL_R5SS0_CPU0_TCM,\
								 SDL_R5FSS0_CORE1_B0TCM1,\
								 SDL_R5F0B0TCM1_MASK);
		/* Wait until ESM interrupt happens */
		while(paritytcm_esmError !=true);
		paritytcm_esmError = false;
		if (result != SDL_PASS)
		{
			retVal = -1;
		}
		else{
			retVal = SDL_PASS;
		}
	}

	if (retVal == SDL_PASS)
    {
        result = SDL_ECC_tcmParity(SDL_R5SS0_CPU0_TCM,\
								 SDL_R5FSS0_CORE1_B1TCM1,\
								 SDL_R5F0B1TCM1_MASK);

		/* Wait until ESM interrupt happens */
    while(paritytcm_esmError !=true);
		paritytcm_esmError = false;
		if (result != SDL_PASS)
		{
			retVal = -1;
		}
		else{
			retVal = SDL_PASS;
		}
	}

#ifdef TCM_ON_R5F1
/* R5FSS1_0 */

	if (retVal == SDL_PASS)
    {
        result = SDL_ECC_tcmParity(SDL_R5SS1_CPU0_TCM,\
								 SDL_R5FSS1_CORE0_ATCM0,\
								 SDL_R5F1ATCM0_MASK);
		/* Wait until ESM interrupt happens */
		while(paritytcm_esmError !=true);
		paritytcm_esmError = false;
		if (result != SDL_PASS)
		{
			retVal = -1;
		}
		else{
			retVal = SDL_PASS;
		}
	}

	if (retVal == SDL_PASS)
    {
        result = SDL_ECC_tcmParity(SDL_R5SS1_CPU0_TCM,\
								 SDL_R5FSS1_CORE0_B0TCM0,\
								 SDL_R5F1B0TCM0_MASK);
		/* Wait until ESM interrupt happens */
		while(paritytcm_esmError !=true);
		paritytcm_esmError = false;
		if (result != SDL_PASS)
		{
			retVal = -1;
		}
		else{
			retVal = SDL_PASS;
		}
	}

	if (retVal == SDL_PASS)
    {
        result = SDL_ECC_tcmParity(SDL_R5SS1_CPU0_TCM,\
								 SDL_R5FSS1_CORE0_B1TCM0,\
								 SDL_R5F1B1TCM0_MASK);
		/* Wait until ESM interrupt happens */
		while(paritytcm_esmError !=true);
		paritytcm_esmError = false;
		if (result != SDL_PASS)
		{
			retVal = -1;
		}
		else{
			retVal = SDL_PASS;
		}
	}

/* R5FSS1_1 */

	if (retVal == SDL_PASS)
    {
        result = SDL_ECC_tcmParity(SDL_R5SS1_CPU0_TCM,\
								 SDL_R5FSS1_CORE1_ATCM1,\
								 SDL_R5F1ATCM1_MASK);
		/* Wait until ESM interrupt happens */
		while(paritytcm_esmError !=true);
		paritytcm_esmError = false;
		if (result != SDL_PASS)
		{
			retVal = -1;
		}
		else{
			retVal = SDL_PASS;
		}
	}

	if (retVal == SDL_PASS)
    {
        result = SDL_ECC_tcmParity(SDL_R5SS1_CPU0_TCM,\
								 SDL_R5FSS1_CORE1_B0TCM1,\
								 SDL_R5F1B0TCM1_MASK);
		/* Wait until ESM interrupt happens */
		while(paritytcm_esmError !=true);
		paritytcm_esmError = false;
		if (result != SDL_PASS)
		{
			retVal = -1;
		}
		else{
			retVal = SDL_PASS;
		}
	}

	if (retVal == SDL_PASS)
    {
        result = SDL_ECC_tcmParity(SDL_R5SS1_CPU0_TCM,\
								 SDL_R5FSS1_CORE1_B1TCM1,\
								 SDL_R5F1B1TCM1_MASK);
		/* Wait until ESM interrupt happens */
		while(paritytcm_esmError !=true);
		paritytcm_esmError = false;
		if (result != SDL_PASS)
		{
			retVal = -1;
		}
		else{
			retVal = SDL_PASS;
		}
	}
#endif /* TCM on R5F1 */

    return retVal;
}



/* Nothing past this point */
