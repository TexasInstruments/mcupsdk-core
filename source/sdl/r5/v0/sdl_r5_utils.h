/*
 * SDL UTILS
 *
 * Software Diagnostics Library utilities
 *
 *  Copyright (c) Texas Instruments Incorporated 2020-2024
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
    uint32_t SCTLR;
    /* SCTLR register */
    uint32_t ACTLR;
    /* ACTLR register */
    uint32_t SecondaryACTLR;
    /* SecondaryACTLR register */
    uint32_t CPACR;
    /* CPACR register */
    uint32_t BTCMRegionR;
    /* BTCMRegionR register */
    uint32_t ATCMRegionR;
    /* ATCMRegionR register */
    uint32_t SlavePortControlR;
    /* SlavePortControlR register */
    uint32_t CONTEXTIDR;
    /* CONTEXTIDR register */
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
    uint32_t PMCR;
    /* PMCR Register */
    uint32_t PMCNTENSET;
    /* PMCNTENSET Register */
    uint32_t PMUSERENR;
    /* PMUSERENR Register */
    uint32_t PMINTENSET;
    /* PMINTENSET Register */
    uint32_t PMINTENCLR;
    /* PMINTENCLR Register */

}SDL_R5FCPU_StaticRegs;

/** ---------------------------------------------------------------------------
 * \brief MPU Static Registers structure
 *
 * This structure defines the MPU static configuration registers
 * ----------------------------------------------------------------------------
 */
typedef struct SDL_MPU_staticReg_read
{
    uint32_t sysControlReg;
    /**< SCTLR (System Control Register)*/
    uint32_t mpuTypeReg;
    /**< MPUIR (MPU Type Register) */
	 uint32_t regionId;
    /**< Region number to configure.
     *   Range: 0 to (SDL_ARM_R5F_MPU_REGIONS_MAX     1U) */
    uint32_t baseAddr;
    /**< Region base address: 32 bytes aligned. */
    uint32_t size;
    /**< Region size */
	uint32_t accessPermission;
    /**< Access permissions */
	
}SDL_R5MPU_staticRegs;

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
extern int32_t SDL_CPU_staticRegisterRead(SDL_R5FCPU_StaticRegs *pCPUStaticRegs);

/**
 *
 * \brief   MPU API to Read the Static Registers.
 *          This function reads the values of the static registers such as
 *          System Control Register, MPU Type Register and MPU Region Number Register.
 *
 * \param   pMPUStaticRegs  [IN]    Pointer to the static registers structure
 * \param   regionNum    [IN]    Region number
 *
 */

extern void SDL_R5MPU_readStaticRegisters(SDL_R5MPU_staticRegs *pMPUStaticRegs,uint32_t regionNum);	

/** @} */
/** @} */

/* ========================================================================== */
/*                         Local Function Declarations                             */
/* ========================================================================== */

uint32_t SDL_UTILS_getSCTLR(void);
uint32_t SDL_UTILS_getACTLR(void);
uint32_t SDL_UTILS_getSecondaryACTLR(void);
uint32_t SDL_UTILS_getCPACR(void);
uint32_t SDL_UTILS_getBTCMRegionR(void);
uint32_t SDL_UTILS_getATCMRegionR(void);
uint32_t SDL_UTILS_getSlavePortControlR(void);
uint32_t SDL_UTILS_getCONTEXTIDR(void);
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
uint32_t SDL_UTILS_getPMCR(void);
uint32_t SDL_UTILS_getPMCNTENSET(void);
uint32_t SDL_UTILS_getPMUSERENR (void);
uint32_t SDL_UTILS_getPMINTENSET(void);
uint32_t SDL_UTILS_getPMINTENCLR(void);

/* Some other function not related to CPU Static register*/
void SDL_UTILS_enable_event_bus(void);
void SDL_UTILS_enable_cache_event_bus(void);

#endif /* INCLUDE_SDL_UTILS_H_ */
