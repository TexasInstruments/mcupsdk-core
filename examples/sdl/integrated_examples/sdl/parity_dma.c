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
 *  \file     parity_dma.c
 *
 *  \brief    This file contains functions that provide input event triggers
 *            for the DMA Parity application.
 *
 *  \details  DMA Parity Safety Example module tests
 **/

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */
#include <stdint.h>
#include <stdio.h>
#include <sdl/include/sdl_types.h>
#include <sdl/sdl_ecc.h>
#include <kernel/dpl/DebugP.h>
#include <sdl/include/am263x/sdlr_soc_ecc_aggr.h>
#include <sdl/ecc/soc/am263x/sdl_ecc_soc.h>

/* ========================================================================== */
/*                                Macros                                      */
/* ========================================================================== */
/* value to write into param register */
#define PARAM_REG_VALUE  0x7u
/* Enable testmode and parity in TPCC0_PARITY_CTRL register */
#define PARITY_ENABLE    0x11u
/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */
volatile static bool paritydma_esmError = false;
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

/* handles ESM callback */
int32_t ParityDMA_clear(void)
{
    int32_t retVal = SDL_PASS;

	/* setting control register to clear TPCC status */
	SDL_REG32_WR(SDL_R5FSS0_CORE0_TPCC0_PARITY_CTRL, clearstatusbit);
	/* clearing error aggregator status bit  */
	SDL_REG32_WR(SDL_TPCC0_ERRAGG_STATUS, clearerraggbit);

	paritydma_esmError = true;

  return retVal;
}

/*********************************************************************
 * @fn      ParityDMA_test
 *
 * @brief   Execute ParityDMA_test function test
 *
 * @param   None
 *
 * @return  0 : Success; < 0 for failures
 **********************************************************************/
int32_t ParityDMA_test(void)
{
    int32_t retVal = 0;
	int32_t result= SDL_PASS;
    uint32_t mask=0u;

    /* masking TPCC0 error aggregator  */
	SDL_REG32_WR(SDL_TPCC0_ERRAGG_MASK,errormask);
	/* unmasking TPCC0_ERRAGG_MASK register  */
	mask = errorunmask & SDL_REG32_RD(SDL_TPCC0_ERRAGG_MASK);
	SDL_REG32_WR(SDL_TPCC0_ERRAGG_MASK,mask);

	if (retVal == 0)
    {
        retVal = SDL_ECC_tpccParity(SDL_TPCC0, \
								 PARITY_ENABLE, SDL_PARAM_REG_1, \
								 PARAM_REG_VALUE);

		/* Wait until ESM interrupt happens */
		while(paritydma_esmError !=true);

		if(paritydma_esmError == true)
		{
			paritydma_esmError = false;
		}
		else{
			result = SDL_EFAIL;
		}
	}

    return result;
}

/* Nothing past this point */
