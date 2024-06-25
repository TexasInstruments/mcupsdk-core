/*
 * SDL UTILS
 *
 * Software Diagnostics Library utilities
 *
 *  Copyright (c) Texas Instruments Incorporated 2020-2023
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
 *
 *  \defgroup SDL_R5FCPU_API SDL R5F CPU UTILS
 *  \ingroup SDL_MODULE
 *
 *   Provides the APIs for R5F CPU STATIC REGISTER READ.
 *  @{
 */

/**
 *  \file     sdl_r5_utils.h
 *
 *  \brief    This file contains the prototypes of the APIs present in the
 *            device abstraction layer file of R5F CPU static register.
 *            This also contains some related local functions.
 */

#ifndef INCLUDE_SDL_UTILS_H_
#define INCLUDE_SDL_UTILS_H_

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <stdint.h>
#include <sdl/include/sdl_types.h>


/**
@defgroup SDL_R5FCPU_StaticRegisterRead_FUNCTIONS R5F CPU Static Register Read Functions
@ingroup SDL_R5FCPU_API
*/

/**
@defgroup SDL_R5FCPU_DATASTRUCT R5F CPU STATIC REGISTER Data Structures
@ingroup SDL_R5FCPU_API
*/

/* ========================================================================== */
/*                         Structures                                         */
/* ========================================================================== */

/**

@addtogroup SDL_R5FCPU_DATASTRUCT
@{
*/

/**
 * \brief Structure containing R5F CPU Static Registers
 *
 * Design: PROC_SDL-6331
 *
 */
typedef struct {
    uint32_t MIDR;
    /* MIDR register */
    uint32_t CTR;
    /* CTR register */
    uint32_t TCMTR;
    /* TCMTR register */
    uint32_t MPUIR;
    /* MPUIR register */
    uint32_t MPIDR;
    /* MPIDR register */
    uint32_t PFR0;
    /* PFR0 register */
    uint32_t PFR1;
    /* PFR1 register */
    uint32_t ID_DFR0;
    /* ID_DFR0 register */
    uint32_t ID_AFR0;
    /* ID_AFR0 register */
    uint32_t ID_MMFR0;
    /* ID_MMFR0 register */
    uint32_t ID_MMFR1;
    /* ID_MMFR1 register */
    uint32_t ID_MMFR2;
    /* ID_MMFR2 register */
    uint32_t ID_MMFR3;
    /* ID_MMFR3 register */
    uint32_t ID_ISAR0;
    /* ID_ISAR0 register */
    uint32_t ID_ISAR1;
    /* ID_ISAR1 register */
    uint32_t ID_ISAR2;
    /* ID_ISAR2 register */
    uint32_t ID_ISAR3;
    /* ID_ISAR3 register */
    uint32_t ID_ISAR4;
    /* ID_ISAR4 register */
    uint32_t ID_ISAR5;
    /* ID_ISAR5 register */
    uint32_t CCSIDR;
    /* CCSIDR register */
    uint32_t CLIDR;
    /* CLIDR register */
    uint32_t AIDR;
    /* AIDR register */
    uint32_t CSSELR;
    /* CSSELR register */
    uint32_t SCTLR;
    /* SCTLR register */
    uint32_t ACTLR;
    /* ACTLR register */
    uint32_t SecondaryACTLR;
    /* SecondaryACTLR register */
    uint32_t CPACR;
    /* CPACR register */
    uint32_t MPURegionBaseADDR;
    /* MPURegionBaseADDR register */
    uint32_t MPURegionEnableR;
    /* MPURegionEnableR register */
    uint32_t MPURegionAccessControlR;
    /* MPURegionAccessControlR register */
    uint32_t RGNR;
    /* RGNR register */
    uint32_t BTCMRegionR;
    /* BTCMRegionR register */
    uint32_t ATCMRegionR;
    /* ATCMRegionR register */
    uint32_t SlavePortControlR;
    /* SlavePortControlR register */
    uint32_t CONTEXTIDR;
    /* CONTEXTIDR register */
    uint32_t ThreadProcessIDR1;
    /* ThreadProcessIDR1 register */
    uint32_t ThreadProcessIDR2;
    /* ThreadProcessIDR2 register */
    uint32_t ThreadProcessIDR3;
    /* ThreadProcessIDR3 register */
    uint32_t nVALIRQSET;
    /* nVALIRQSET register */
    uint32_t nVALFIQSET;
    /* nVALFIQSET register */
    uint32_t nVALRESETSET;
    /* nVALRESETSET register */
    uint32_t nVALDEBUGSET;
    /* nVALDEBUGSET register */
    uint32_t nVALIRQCLEAR;
    /* nVALIRQCLEAR register */
    uint32_t nVALFIQCLEAR;
    /* nVALFIQCLEAR register */
    uint32_t nVALRESETCLEAR;
    /* nVALRESETCLEAR register */
    uint32_t nVALDEBUGCLEAR;
    /* nVALDEBUGCLEAR register */
    uint32_t BuildOption1R;
    /* BuildOption1R register */
    uint32_t BuildOption2R;
    /* BuildOption2R register */
    uint32_t PinOptionR;
    /* PinOptionR register */
    uint32_t LLPPnormalAXIRR;
    /* LLPPnormalAXIRR register */
    uint32_t LLPPvirtualAXIRR;
    /* LLPPvirtualAXIRR register */
    uint32_t AHBRR;
    /* AHBRR register */
    uint32_t CFLR;
    /* CFLR register */
    uint32_t PMOVSR;
    /* PMOVSR register */
    uint32_t DFSR;
    /* DFSR register */
    uint32_t ADFSR;
    /* ADFSR register */
    uint32_t DFAR;
    /* DFAR register */
    uint32_t IFSR;
    /* IFSR register */
    uint32_t IFAR;
    /* IFAR register */
    uint32_t AIFSR;
    /* AIFSR register */

}SDL_R5FCPU_StaticRegs;

/** @} */

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

/**

@addtogroup SDL_R5FCPU_StaticRegisterRead_FUNCTIONS
@{
*/

/**
 * \brief   This API is used to get the value of static registers for R5F CPU
 *
 * \param   pCPUStaticRegs      Pointer to the SDL_R5FCPU_StaticRegs structure.
 */
int32_t SDL_CPU_staticRegisterRead(SDL_R5FCPU_StaticRegs *pCPUStaticRegs);

/** @} */
/** @} */

/* ========================================================================== */
/*                         Local Function Declarations                             */
/* ========================================================================== */


uint32_t SDL_UTILS_getMIDR(void);
uint32_t SDL_UTILS_getCTR(void);
uint32_t SDL_UTILS_getTCMTR(void);
uint32_t SDL_UTILS_getMPUIR(void);
uint32_t SDL_UTILS_getMPIDR(void);
uint32_t SDL_UTILS_getPFR0(void);
uint32_t SDL_UTILS_getPFR1(void);
uint32_t SDL_UTILS_getID_DFR0(void);
uint32_t SDL_UTILS_getID_AFR0(void);
uint32_t SDL_UTILS_getID_MMFR0(void);
uint32_t SDL_UTILS_getID_MMFR1(void);
uint32_t SDL_UTILS_getID_MMFR2(void);
uint32_t SDL_UTILS_getID_MMFR3(void);
uint32_t SDL_UTILS_getID_ISAR0(void);
uint32_t SDL_UTILS_getID_ISAR1(void);
uint32_t SDL_UTILS_getID_ISAR2(void);
uint32_t SDL_UTILS_getID_ISAR3(void);
uint32_t SDL_UTILS_getID_ISAR4(void);
uint32_t SDL_UTILS_getID_ISAR5(void);
uint32_t SDL_UTILS_getCCSIDR(void);
uint32_t SDL_UTILS_getCLIDR(void);
uint32_t SDL_UTILS_getAIDR(void);
uint32_t SDL_UTILS_getCSSELR(void);
uint32_t SDL_UTILS_getSCTLR(void);
uint32_t SDL_UTILS_getACTLR(void);
uint32_t SDL_UTILS_getSecondaryACTLR(void);
uint32_t SDL_UTILS_getCPACR(void);
uint32_t SDL_UTILS_getMPURegionBaseADDR(void);
uint32_t SDL_UTILS_getMPURegionEnableR(void);
uint32_t SDL_UTILS_getMPURegionAccessControlR(void);
uint32_t SDL_UTILS_getRGNR(void);
uint32_t SDL_UTILS_getBTCMRegionR(void);
uint32_t SDL_UTILS_getATCMRegionR(void);
uint32_t SDL_UTILS_getSlavePortControlR(void);
uint32_t SDL_UTILS_getCONTEXTIDR(void);
uint32_t SDL_UTILS_getThreadProcessIDR1(void);
uint32_t SDL_UTILS_getThreadProcessIDR2(void);
uint32_t SDL_UTILS_getThreadProcessIDR3(void);
uint32_t SDL_UTILS_getnVALIRQSET(void);
uint32_t SDL_UTILS_getnVALFIQSET(void);
uint32_t SDL_UTILS_getnVALRESETSET(void);
uint32_t SDL_UTILS_getnVALDEBUGSET(void);
uint32_t SDL_UTILS_getnVALIRQCLEAR(void);
uint32_t SDL_UTILS_getnVALFIQCLEAR(void);
uint32_t SDL_UTILS_getnVALRESETCLEAR(void);
uint32_t SDL_UTILS_getnVALDEBUGCLEAR(void);
uint32_t SDL_UTILS_getBuildOption1R(void);
uint32_t SDL_UTILS_getBuildOption2R(void);
uint32_t SDL_UTILS_getPinOptionR(void);
uint32_t SDL_UTILS_getLLPPnormalAXIRR(void);
uint32_t SDL_UTILS_getLLPPvirtualAXIRR(void);
uint32_t SDL_UTILS_getAHBRR(void);
uint32_t SDL_UTILS_getCFLR(void);
uint32_t SDL_UTILS_getPMOVSR(void);
uint32_t SDL_UTILS_getDFSR(void);
uint32_t SDL_UTILS_getADFSR(void);
uint32_t SDL_UTILS_getDFAR(void);
uint32_t SDL_UTILS_getIFSR(void);
uint32_t SDL_UTILS_getIFAR(void);
uint32_t SDL_UTILS_getAIFSR(void);

/* Some other function not related to CPU Static register*/
void SDL_UTILS_enable_event_bus(void);
void SDL_UTILS_enable_cache_event_bus(void);

#endif /* INCLUDE_SDL_UTILS_H_ */
