/*
 * SDL CPU STATIC REGISTER
 *
 * Software Diagnostics Library module for handling exceptions
 *
 *  Copyright (c) Texas Instruments Incorporated 2023-2024
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

#include <stddef.h>
#include <stdbool.h>
#include "sdl_r5_utils.h"

/********************************************************************************************************
 *   API for reading the R5F CPU static registers values
 *********************************************************************************************************/

/**
 *  Design: PROC_SDL-6330
 */

int32_t SDL_CPU_staticRegisterRead(SDL_R5FCPU_StaticRegs *pCPUStaticRegs)
{

    int32_t sdlResult;

    if (pCPUStaticRegs != NULL)
    {
        pCPUStaticRegs->SCTLR = SDL_UTILS_getSCTLR();
        /* Get The SCTLR register value  */

        pCPUStaticRegs->ACTLR = SDL_UTILS_getACTLR();
        /* Get The ACTLR register value  */

        pCPUStaticRegs->SecondaryACTLR = SDL_UTILS_getSecondaryACTLR();
        /* Get The SecondaryACTLR register value  */

        pCPUStaticRegs->CPACR = SDL_UTILS_getCPACR();
        /* Get The CPACR register value  */

        pCPUStaticRegs->BTCMRegionR = SDL_UTILS_getBTCMRegionR();
        /* Get The BTCMRegionR register value  */

        pCPUStaticRegs->ATCMRegionR = SDL_UTILS_getATCMRegionR();
        /* Get The ATCMRegionR register value  */

        pCPUStaticRegs->SlavePortControlR = SDL_UTILS_getSlavePortControlR();
        /* Get The SlavePortControlR register value  */

        pCPUStaticRegs->CONTEXTIDR = SDL_UTILS_getCONTEXTIDR();
        /* Get The CONTEXTIDR register value  */

        pCPUStaticRegs->nVALIRQSET = SDL_UTILS_getnVALIRQSET();
        /* Get The nVALIRQSET register value  */

        pCPUStaticRegs->nVALFIQSET = SDL_UTILS_getnVALFIQSET();
        /* Get The nVALFIQSET register value  */

        pCPUStaticRegs->nVALRESETSET = SDL_UTILS_getnVALRESETSET();
        /* Get The nVALRESETSET register value  */

        pCPUStaticRegs->nVALDEBUGSET = SDL_UTILS_getnVALDEBUGSET();
        /* Get The nVALDEBUGSET register value  */

        pCPUStaticRegs->nVALIRQCLEAR = SDL_UTILS_getnVALIRQCLEAR();
        /* Get The nVALIRQCLEAR register value  */

        pCPUStaticRegs->nVALFIQCLEAR = SDL_UTILS_getnVALFIQCLEAR();
        /* Get The nVALFIQCLEAR register value  */

        pCPUStaticRegs->nVALRESETCLEAR = SDL_UTILS_getnVALRESETCLEAR();
        /* Get The nVALRESETCLEAR register value  */

        pCPUStaticRegs->nVALDEBUGCLEAR = SDL_UTILS_getnVALDEBUGCLEAR();
        /* Get The nVALDEBUGCLEAR register value  */

        pCPUStaticRegs->BuildOption1R = SDL_UTILS_getBuildOption1R();
        /* Get The BuildOption1R register value  */

        pCPUStaticRegs->BuildOption2R = SDL_UTILS_getBuildOption2R();
        /* Get The BuildOption2R register value  */

        pCPUStaticRegs->PinOptionR = SDL_UTILS_getPinOptionR();
        /* Get The PinOptionR register value  */

        pCPUStaticRegs->LLPPnormalAXIRR = SDL_UTILS_getLLPPnormalAXIRR();
        /* Get The LLPPnormalAXIRR register value  */

        pCPUStaticRegs->LLPPvirtualAXIRR = SDL_UTILS_getLLPPvirtualAXIRR();
        /* Get The LLPPvirtualAXIRR register value  */

        pCPUStaticRegs->AHBRR = SDL_UTILS_getAHBRR();
        /* Get The AHBRR register value  */

        pCPUStaticRegs->PMCR = SDL_UTILS_getPMCR();
        /* Get The PMCR register value  */

        pCPUStaticRegs->PMCNTENSET = SDL_UTILS_getPMCNTENSET();
        /* Get The PMCNTENSET register value */

        pCPUStaticRegs->PMUSERENR = SDL_UTILS_getPMUSERENR();
        /* Get The PMUSERENR register value */

        pCPUStaticRegs->PMINTENSET = SDL_UTILS_getPMINTENSET();
        /* Get The PMINTENSET register value */

        pCPUStaticRegs->PMINTENCLR = SDL_UTILS_getPMINTENCLR();
        /* Get The PMINTENCLR register value */

        sdlResult = SDL_PASS;
    }
    else
    {
        sdlResult = SDL_EBADARGS;
    }

    return sdlResult;
}

