/*
 * SDL CPU STATIC REGISTER
 *
 * Software Diagnostics Library module for handling exceptions
 *
 *  Copyright (c) Texas Instruments Incorporated 2023
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

        pCPUStaticRegs->MIDR = SDL_UTILS_getMIDR();
        /* Get The MIDR register value  */

        pCPUStaticRegs->CTR = SDL_UTILS_getCTR();
        /* Get The CTR register value  */

        pCPUStaticRegs->TCMTR = SDL_UTILS_getTCMTR();
        /* Get The TCMTR register value  */

        pCPUStaticRegs->MPUIR = SDL_UTILS_getMPUIR();
        /* Get The MPUIR register value  */

        pCPUStaticRegs->MPIDR = SDL_UTILS_getMPIDR();
        /* Get The MPIDR register value  */

        pCPUStaticRegs->PFR0 = SDL_UTILS_getPFR0();
        /* Get The PFR0 register value  */

        pCPUStaticRegs->PFR1 = SDL_UTILS_getPFR1();
        /* Get The PFR1 register value  */

        pCPUStaticRegs->ID_DFR0 = SDL_UTILS_getID_DFR0();
        /* Get The ID_DFR0 register value  */

        pCPUStaticRegs->ID_AFR0 = SDL_UTILS_getID_AFR0();
        /* Get The ID_AFR0 register value  */

        pCPUStaticRegs->ID_MMFR0 = SDL_UTILS_getID_MMFR0();
        /* Get The ID_MMFR0 register value  */

        pCPUStaticRegs->ID_MMFR1 = SDL_UTILS_getID_MMFR1();
        /* Get The ID_MMFR1 register value  */

        pCPUStaticRegs->ID_MMFR2 = SDL_UTILS_getID_MMFR2();
        /* Get The ID_MMFR2 register value  */

        pCPUStaticRegs->ID_MMFR3 = SDL_UTILS_getID_MMFR3();
        /* Get The ID_MMFR3 register value  */

        pCPUStaticRegs->ID_ISAR0 = SDL_UTILS_getID_ISAR0();
        /* Get The ID_ISAR0 register value  */

        pCPUStaticRegs->ID_ISAR1 = SDL_UTILS_getID_ISAR1();
        /* Get The ID_ISAR1 register value  */

        pCPUStaticRegs->ID_ISAR2 = SDL_UTILS_getID_ISAR2();
        /* Get The ID_ISAR2 register value  */

        pCPUStaticRegs->ID_ISAR3 = SDL_UTILS_getID_ISAR3();
        /* Get The ID_ISAR3 register value  */

        pCPUStaticRegs->ID_ISAR4 = SDL_UTILS_getID_ISAR4();
        /* Get The ID_ISAR4 register value  */

        pCPUStaticRegs->ID_ISAR5 = SDL_UTILS_getID_ISAR5();
        /* Get The ID_ISAR5 register value  */

        pCPUStaticRegs->CCSIDR = SDL_UTILS_getCCSIDR();
        /* Get The CCSIDR register value  */

        pCPUStaticRegs->CLIDR = SDL_UTILS_getCLIDR();
        /* Get The CLIDR register value  */

        pCPUStaticRegs->AIDR = SDL_UTILS_getAIDR();
        /* Get The AIDR register value  */

        pCPUStaticRegs->CSSELR = SDL_UTILS_getCSSELR();
        /* Get The CSSELR register value  */

        pCPUStaticRegs->SCTLR = SDL_UTILS_getSCTLR();
        /* Get The SCTLR register value  */

        pCPUStaticRegs->ACTLR = SDL_UTILS_getACTLR();
        /* Get The ACTLR register value  */

        pCPUStaticRegs->SecondaryACTLR = SDL_UTILS_getSecondaryACTLR();
        /* Get The SecondaryACTLR register value  */

        pCPUStaticRegs->CPACR = SDL_UTILS_getCPACR();
        /* Get The CPACR register value  */

        pCPUStaticRegs->MPURegionBaseADDR = SDL_UTILS_getMPURegionBaseADDR();
        /* Get The MPURegionBaseADDR register value  */

        pCPUStaticRegs->MPURegionEnableR = SDL_UTILS_getMPURegionEnableR();
        /* Get The MPURegionEnableR register value  */

        pCPUStaticRegs->MPURegionAccessControlR = SDL_UTILS_getMPURegionAccessControlR();
        /* Get The MPURegionAccessControlR register value  */

        pCPUStaticRegs->RGNR = SDL_UTILS_getRGNR();
        /* Get The RGNR register value  */

        pCPUStaticRegs->BTCMRegionR = SDL_UTILS_getBTCMRegionR();
        /* Get The BTCMRegionR register value  */

        pCPUStaticRegs->ATCMRegionR = SDL_UTILS_getATCMRegionR();
        /* Get The ATCMRegionR register value  */

        pCPUStaticRegs->SlavePortControlR = SDL_UTILS_getSlavePortControlR();
        /* Get The SlavePortControlR register value  */

        pCPUStaticRegs->CONTEXTIDR = SDL_UTILS_getCONTEXTIDR();
        /* Get The CONTEXTIDR register value  */

        pCPUStaticRegs->ThreadProcessIDR1 = SDL_UTILS_getThreadProcessIDR1();
        /* Get The ThreadProcessIDR1 register value  */

        pCPUStaticRegs->ThreadProcessIDR2 = SDL_UTILS_getThreadProcessIDR2();
        /* Get The ThreadProcessIDR2 register value  */

        pCPUStaticRegs->ThreadProcessIDR3 = SDL_UTILS_getThreadProcessIDR3();
        /* Get The ThreadProcessIDR3 register value  */

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

        pCPUStaticRegs->CFLR = SDL_UTILS_getCFLR();
        /* Get The CFLR register value  */

        pCPUStaticRegs->PMOVSR = SDL_UTILS_getPMOVSR();
        /* Get The PMOVSR register value  */

        pCPUStaticRegs->DFSR = SDL_UTILS_getDFSR();
        /* Get The DFSR register value  */

        pCPUStaticRegs->ADFSR = SDL_UTILS_getADFSR();
        /* Get The ADFSR register value  */

        pCPUStaticRegs->DFAR = SDL_UTILS_getDFAR();
        /* Get The DFAR register value  */

        pCPUStaticRegs->IFSR = SDL_UTILS_getIFSR();
        /* Get The IFSR register value  */

        pCPUStaticRegs->IFAR = SDL_UTILS_getIFAR();
        /* Get The IFAR register value  */

        pCPUStaticRegs->AIFSR = SDL_UTILS_getAIFSR();
        /* Get The AIFSR register value  */

        sdlResult = SDL_PASS;
    }
    else
    {
        sdlResult = SDL_EBADARGS;
    }

    return sdlResult;
}
