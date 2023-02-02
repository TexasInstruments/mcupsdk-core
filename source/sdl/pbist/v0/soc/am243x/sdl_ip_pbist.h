/********************************************************************
 * Copyright (C) 2023 Texas Instruments Incorporated.
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
 *   @ingroup  SDL_PBIST_MODULE
 *   @defgroup SDL_IP_PBIST_API PBIST Low-Level API
 *
 *   @{
 */
/*
 *   @file  sdl/pbist/v0/sdl_ip_pbist.h
 *
 *   @brief This file contains the SDL-FL API's for PBIST
 *
 *   This is the SDL-FL API documentation for the Programmable Built-In Self Test
 *   (PBIST) module.
 *
 */

#ifndef SDL_IP_PBIST_H_
#define SDL_IP_PBIST_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>
#include <sdl/pbist/v0/sdlr_pbist.h>
#include <sdl/pbist/v0/soc/sdl_soc_pbist.h>
#include <sdl/sdl_esm.h>

#define PBIST_MAX_NUM_RUNS    2

/** ---------------------------------------------------------------------------
 * @brief   This structure contains the different configuration used for PBIST
 *
 * Design: PROC_SDL-967
 *
 * ----------------------------------------------------------------------------
 */

typedef struct
{
    /** Override value for memory configuration.
        Check SOC documentation for supported override mode for each PBIST instance. */
    uint32_t override;

    /** Bitmap to select algorithms to use for test.

        NOTE: For a given PBIST instance, there is a selected
        set of memory groups a particular Algorithm can be run.
        Check SOC documentation for supported algorithms for each PBIST instance. */
    uint32_t algorithmsBitMap;

    /** Bitmap to select memory groups to run test on.
        Check SOC documentation for supported memory groups for each PBIST instance. */
    uint64_t memoryGroupsBitMap;

    /** Scramble value to be used for test.
        Check SOC documentation for scramble value to be used. */
    uint64_t scrambleValue;
} SDL_PBIST_config;

/** ---------------------------------------------------------------------------
 * @brief   This structure contains the different configuration used for PBIST
 *          for the failure insertion test to generate negative result.
 *
 * Design: PROC_SDL-977
 *
 * ----------------------------------------------------------------------------
 */

typedef struct
{
    /** Failure insertion value for CA0
        Value for the Failure Insertion Test Vector CA0 */
    uint32_t CA0;

    /** Failure insertion value for CA1
        Value for the Failure Insertion Test Vector CA1 */
    uint32_t CA1;

    /** Failure insertion value for CA2
        Value for the Failure Insertion Test Vector CA2 */
    uint32_t CA2;

    /** Failure insertion value for CA3
        Value for the Failure Insertion Test Vector CA3 */
    uint32_t CA3;

    /** Failure insertion value for CL0
        Value for the Failure Insertion Test Vector CL0 */
    uint32_t CL0;

    /** Failure insertion value for CL1
        Value for the Failure Insertion Test Vector CL1 */
    uint32_t CL1;

    /** Failure insertion value for CL2
        Value for the Failure Insertion Test Vector CL2 */
    uint32_t CL2;

    /** Failure insertion value for CL3
        Value for the Failure Insertion Test Vector CL3 */
    uint32_t CL3;

    /** Failure insertion value for CMS
        Value for the Failure Insertion Test Vector CMS */
    uint32_t CMS;

    /** Failure insertion value for CSR
        Value for the Failure Insertion Test Vector CSR */
    uint32_t CSR;

    /** Failure insertion value for I0
        Value for the Failure Insertion Test Vector I0 */
    uint32_t I0;

    /** Failure insertion value for I1
        Value for the Failure Insertion Test Vector I1 */
    uint32_t I1;

    /** Failure insertion value for I2
        Value for the Failure Insertion Test Vector I2 */
    uint32_t I2;

    /** Failure insertion value for I3
        Value for the Failure Insertion Test Vector I3 */
    uint32_t I3;

    /** Failure insertion value for RAMT
        Value for the Failure Insertion Test Vector RAMT */
    uint32_t RAMT;
} SDL_PBIST_configNeg;

/**
 *  \brief PBIST Soft reset
 *
 *  This function resets PBIST Module. In general called before running a new
 *  test.
 *
 *
 *  \param pPBISTRegs       [IN]  Pointer to PBIST registers base
 *
 *  \return The SDL error code for the API.
 *                                 If pBISTRegs is NULL: SDL_EBADARGS
 *                                 Success: SDL_PASS
 */
int32_t SDL_PBIST_softReset(SDL_pbistRegs *pPBISTRegs);

/**
 *  \brief PBIST Start
 *
 *  This function configures the paramters for PBIST and starts execution.
 *
 *
 *  \param pPBISTRegs       [IN]  Pointer to PBIST registers base
 *  \param pConfig          [IN]  Pointer to PBIST configuration
 *
 *  \return The SDL error code for the API.
 *                                 If pBISTRegs or pConfig is NULL   : SDL_EBADARGS
 *                                 Success: SDL_PASS
 */
int32_t SDL_PBIST_start(SDL_pbistRegs *pPBISTRegs,
                        const SDL_PBIST_config * pConfig);

/**
 *  \brief PBIST Failure Insertion Test Start
 *
 *  This function configures the parameters for PBIST Test for Diagnostics to insert
 *  a failure to generate the negative result and starts execution.
 *
 *  \param pPBISTRegs       [IN]  Pointer to PBIST registers base
 *  \param pConfig          [IN]  Pointer to PBIST configuration
 *
 *  \return The SDL error code for the API.
 *                                 If pBISTRegs or pConfig in NULL   : SDL_EBADARGS
 *                                 Success: SDL_PASS
 */
int32_t SDL_PBIST_startNeg(SDL_pbistRegs *pPBISTRegs,
                           const SDL_PBIST_configNeg *pConfig);

/**
 *  \brief PBIST check result
 *
 *  This function checks if PBIST test has passed.
 *
 *
 *  \param pPBISTRegs       [IN]  Pointer to PBIST registers base
 *  \param pResult          [OUT] Pointer to variable to indicate result
 *
 *  \return The SDL error code for the API.
 *                                 If pBISTRegs or pResult is NULL: SDL_EBADARGS
 *                                 Success: SDL_PASS
 */
int32_t SDL_PBIST_checkResult (const SDL_pbistRegs *pPBISTRegs, bool *pResult);

/**
 *  \brief PBIST Release Test mode
 *
 *  This function releases test mode. In general called after test is complete
 *  to switch back to normal operation.
 *
 *
 *  \param pPBISTRegs       [IN]  Pointer to PBIST registers base
 *
 *  \return The SDL error code for the API.
 *                                 If pBISTRegs is NULL: SDL_EBADARGS
 *                                 Success: SDL_PASS
 */
int32_t SDL_PBIST_releaseTestMode(SDL_pbistRegs *pPBISTRegs);

/** @} */

typedef struct
{
    uint64_t PBISTRegsHiAddress;
    SDL_pbistRegs *pPBISTRegs;
    uint32_t numPBISTRuns;
    SDL_PBIST_config PBISTConfigRun[PBIST_MAX_NUM_RUNS];
    uint32_t interruptNumber;
    SDL_ESM_Inst esmInst;
    uint32_t esmEventNumber;
    volatile uint32_t doneFlag;
    SDL_PBIST_configNeg PBISTNegConfigRun;
} SDL_pbistInstInfo;

#ifdef __cplusplus
}
#endif

#endif /* SDL_IP_PBIST_H_ */
