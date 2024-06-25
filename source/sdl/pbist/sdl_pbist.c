/**
 * @file  sdl_pbist.c
 *
 * @brief
 *  SDL implementation file for the pbist module.
 *
 *  \par
 *  ============================================================================
 *  @n   (C) Copyright 2022, Texas Instruments, Inc.
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

#include <sdl/sdl_pbist.h>
#include <sdl/include/sdl_types.h>
#include <sdl/pbist/sdl_pbist_priv.h>

#include <sdl/dpl/sdl_dpl.h>

#define PBIST_REG_REGION_SIZE             (0x400u)

#define PBIST_MAX_TIMEOUT_VALUE           (100000000u)
#if defined (SOC_AM273X) || (SOC_AWR294X)
extern uint32_t gInst;
#endif

static int32_t SDL_PBIST_prepareTest(SDL_PBIST_inst instance, const SDL_pbistInstInfo *pInfo,
                                     SDL_pbistRegs **pRegs,
                                     pSDL_DPL_HwipHandle *PBIST_intrHandle)
{
    int32_t ret = SDL_PASS;
    SDL_DPL_HwipParams intrParams;
    void *localAddr = NULL;
    #if defined (SOC_AM273X) || defined (SOC_AWR294X)
    SDL_PBIST_Instance(instance);
    #endif
    /* Disable interrupt */
    if (pInfo->interruptNumber != SDL_PBIST_INTERRUPT_INVALID)
    {
        ret = SDL_DPL_disableInterrupt((int32_t)pInfo->interruptNumber);

        if (ret == SDL_PASS)
        {
            intrParams.intNum = (int32_t)pInfo->interruptNumber;
            intrParams.callback = (pSDL_DPL_InterruptCallbackFunction)SDL_PBIST_eventHandler;
            intrParams.callbackArg = (uintptr_t)instance;

            /* Register call back function for PBIST Interrupt */
            ret = SDL_DPL_registerInterrupt(&intrParams,
                                            PBIST_intrHandle);
        }
        if (*PBIST_intrHandle == NULL)
        {
            ret = SDL_EFAIL;
        }
    }

    /* Get PBIST register space Pointer */
    if (ret == SDL_PASS)
    {
        localAddr = SDL_DPL_addrTranslate((uint64_t)pInfo->pPBISTRegs, PBIST_REG_REGION_SIZE);
        if (localAddr == (void *)(-1))
        {
            ret = SDL_EFAIL;
        }
        *pRegs = (SDL_pbistRegs *)(localAddr);
    }

    return ret;
}

static int32_t SDL_PBIST_getResult(SDL_PBIST_testType testType, const SDL_pbistRegs *pRegs, bool *pResult)
{
    int32_t ret = SDL_PASS;
    bool PBISTResult;

    ret = SDL_PBIST_checkResult(pRegs, &PBISTResult);

    if (ret == SDL_PASS)
    {
        /* Check the PBIST result */
        if ((testType == SDL_PBIST_TEST) && (PBISTResult == (bool) true))
        {
            *pResult = (bool) true;
        }
        else if ((testType == SDL_PBIST_NEG_TEST) && (PBISTResult == (bool) false))
        {
            *pResult = (bool) true;
        }
        else
        {
            *pResult = (bool) false;
            ret = SDL_EFAIL;
        }
    }

    return ret;
}

static int32_t SDL_PBIST_runTest(SDL_PBIST_testType testType, SDL_pbistRegs *pRegs,
                                 SDL_pbistInstInfo *pInfo, uint32_t timeout,
                                 bool *pResult)
{
    int32_t ret = SDL_PASS;
    uint32_t numRuns = 1;
    uint32_t timeoutCount = 0;
    uint32_t i = 0;

    if (testType == SDL_PBIST_TEST)
    {
        numRuns = pInfo->numPBISTRuns;
    }
    pInfo->doneFlag = PBIST_NOT_DONE;

    for (i = 0; (i < numRuns) && (ret == SDL_PASS); i++)
    {
        if (testType == SDL_PBIST_TEST)
        {
#if defined (SOC_AM273X) || defined (SOC_AWR294X)
            if (gInst == (uint32_t)SDL_PBIST_INST_TOP)
            {
                ret = SDL_PBIST_start(pRegs, &pInfo->PBISTConfigRun1[i]);
            }
            else if (gInst == (uint32_t)SDL_PBIST_INST_DSS)
            {
                    ret = SDL_PBIST_start(pRegs, &pInfo->PBISTConfigRun2[i]);
            }
            else
            {
                ret = SDL_EBADARGS ;
            }
#else
            ret = SDL_PBIST_start(pRegs, &pInfo->PBISTConfigRun[i]);
#endif
        }
        else /* (testType == SDL_PBIST_NEG_TEST) */
        {
          ret = SDL_PBIST_startNeg(pRegs, &pInfo->PBISTNegConfigRun);
        }

        if (ret == SDL_PASS)
        {
            timeoutCount = timeout;
            /* Timeout if exceeds time */
            while ((pInfo->doneFlag == PBIST_NOT_DONE)
                   && (timeoutCount > (uint32_t)0))
            {
                #ifndef SDL_SOC_MCU_R5F
                SDL_PBIST_checkDone(pInfo);
                #endif
                timeoutCount--;
            }

            if (pInfo->doneFlag == PBIST_NOT_DONE)
            {
                ret = SDL_EFAIL;
            }
            else
            {
                ret = SDL_PBIST_getResult(testType, pRegs, pResult);

                /* Do a Soft Reset */
                if (ret == SDL_PASS)
                {
                    ret = SDL_PBIST_softReset(pRegs);

                    /* Execute exit sequence */
                    if (ret == SDL_PASS)
                    {
                        ret = SDL_PBIST_releaseTestMode(pRegs);
                    }
                }
            }

            /* reset Done flag so we can run again */
            pInfo->doneFlag = PBIST_NOT_DONE;
        }
    }

    return ret;
}

static int32_t SDL_PBIST_cleanupTest(pSDL_DPL_HwipHandle PBIST_intrHandle)
{
    int32_t ret = SDL_PASS;

    /* Destroy the interrupt handler */
    if (PBIST_intrHandle != NULL)
    {
        (void)SDL_DPL_deregisterInterrupt(PBIST_intrHandle);
    }

    return ret;
}

/**
 * Design: PROC_SDL-947,PROC_SDL-948,PROC_SDL-974,PROC_SDL-949,PROC_SDL-950,PROC_SDL-951,PROC_SDL-952,PROC_SDL-957,PROC_SDL-958,PROC_SDL-1178,PROC_SDL-1179,PROC_SDL-1180,PROC_SDL-1181,PROC_SDL-1182
 */
int32_t SDL_PBIST_selfTest(SDL_PBIST_inst instance, SDL_PBIST_testType testType,
                           uint32_t timeout, bool *pResult)
{
    int32_t ret = SDL_PASS;
    SDL_pbistRegs *pRegs;
    SDL_pbistInstInfo *pInfo;

    pSDL_DPL_HwipHandle PBIST_intrHandle = NULL;

    /* Get the PBIST Instance Info */
    pInfo = SDL_PBIST_getInstInfo(instance);

    if ((pResult == NULL) ||
        ((testType != SDL_PBIST_TEST) && (testType != SDL_PBIST_NEG_TEST)) ||
        (pInfo == NULL))
    {
        ret = SDL_EBADARGS;
    }

    if (ret == SDL_PASS)
    {
        ret = SDL_PBIST_prepareTest(instance, pInfo, &pRegs, &PBIST_intrHandle);

        if (ret == SDL_PASS)
        {
            ret = SDL_PBIST_runTest(testType, pRegs, pInfo, timeout, pResult);
        }
    }

    (void)SDL_PBIST_cleanupTest(PBIST_intrHandle);

    return ret;
}