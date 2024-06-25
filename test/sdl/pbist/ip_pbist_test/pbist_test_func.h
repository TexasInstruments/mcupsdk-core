/*
 *   Copyright (c) Texas Instruments Incorporated 2021-2023
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
 *  \file     sdl_pbist_test_func.h
 *
 *  \brief    This file contains PBIST test function structures
 *
 *  \details  PBIST Test function structures
 **/
#ifndef PBIST_TEST_FUNC_H
#define PBIST_TEST_FUNC_H

#ifdef __cplusplus
extern "C"
{
#endif

/* ========================================================================== */
/*                                Macros                                      */
/* ========================================================================== */

#define PBIST_INSTANCE_NAME_MAX_LENGTH    20
#if defined (SOC_AM64X) || defined (SOC_AM243X)
#define PBIST_MAX_NUM_RUNS                2
#endif
/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */
#include <stdint.h>
#include <string.h>
#include <sdl/sdl_pbist.h>

/* ========================================================================== */
/*                                Data Structures                             */
/* ========================================================================== */

typedef void (*PBIST_handlerPtr)(uint32_t instanceId);

/*
    InitRestore function : Initialize or Restore based on init flag
    init : TRUE  --> Initialize
    init : FALSE --> Restore
*/
typedef int32_t (*PBIST_auxInitRestoreFunctionPtr)(bool init);

typedef struct PBIST_TestHandle_s
{
#ifndef SDL_SOC_MCU_R5F
    char     testName[PBIST_INSTANCE_NAME_MAX_LENGTH];
    SDL_PBIST_inst pbistInst;
    uint64_t PBISTRegsHiAddress;
    SDL_pbistRegs *pPBISTRegs;
    SDL_PBIST_config PBISTConfigRun[PBIST_MAX_NUM_RUNS];
    uint32_t numPBISTRuns;
    SDL_PBIST_configNeg PBISTNegConfigRun;
    uint32_t tisciPBISTDeviceId;
    bool     pollMode;
    uint32_t interruptNumber;
    SDL_ESM_Inst esmInst;
    uint32_t esmEventNumber;
    bool procRstNeeded;
    bool secondaryCoreNeeded;
    char coreName[16];
    char secCoreName[16];
    uint32_t tisciProcId;
    uint32_t tisciSecProcId;
    uint32_t tisciDeviceId;
    uint32_t tisciSecDeviceId;
    volatile bool doneFlag;
    bool     coreCustPwrSeqNeeded;
    uint8_t  numPostPbistToCheck;
    uint32_t numAuxDevices;
    uint32_t *auxDeviceIdsP;
    PBIST_auxInitRestoreFunctionPtr auxInitRestoreFunction;
#else
    char testName[PBIST_INSTANCE_NAME_MAX_LENGTH];
    SDL_PBIST_inst pbistInst;
    uint64_t PBISTRegsHiAddress;
    SDL_pbistRegs *pPBISTRegs;
    uint32_t numPBISTRuns;
    SDL_PBIST_configNeg PBISTNegConfigRun;
    uint32_t interruptNumber;
    volatile bool doneFlag;
#endif

} PBIST_TestHandle_t;

int32_t PBIST_commonInit(void);

int32_t PBIST_clecConfig(uint32_t instance);

#ifdef __cplusplus
}
#endif

#endif /* PBIST_TEST_FUNC_H */

/* Nothing past this point */
