/*
 * Copyright (C) 2023 Texas Instruments Incorporated
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *   Redistributions of source code must retain the above copyright
 *   notice, this list of conditions and the following disclaimer.
 *
 *   Redistributions in binary form must reproduce the above copyright
 *   notice, this list of conditions and the following disclaimer in the
 *   documentation and/or other materials provided with the
 *   distribution.
 *
 *   Neither the name of Texas Instruments Incorporated nor the names of
 *   its contributors may be used to endorse or promote products derived
 *   from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/**
 * @file  sdl_ip_lbist.c
 *
 * @brief
 *  SDL-FL implementation file for the LBIST module.
 *
 **/

#include "sdl/lbist/V0/sdl_ip_lbist.h"
#include <sdl/include/sdl_types.h>
#include "sdl/lbist/V0/sdlr_lbist.h"

/* MACRO DEFINES */
#define SDL_LBIST_STAT_MISR_MUX_CTL_COMPACT_MISR            (0x0U)

#define SDL_LBIST_STAT_OUT_MUX_CTL_CTRLMMR_PID              (0x0U)
#define SDL_LBIST_STAT_OUT_MUX_CTL_CTRL_ID                  (0x1U)
#define SDL_LBIST_STAT_OUT_MUX_CTL_MISR_VALUE_1             (0x2U)
#define SDL_LBIST_STAT_OUT_MUX_CTL_MISR_VALUE_2             (0x3U)

/* Local Function prototypes */
static void SDL_LBIST_setLoadDiv(SDL_lbistRegs *pLBISTRegs);

static void SDL_LBIST_clearLoadDiv(SDL_lbistRegs *pLBISTRegs);

static void SDL_LBIST_setDivideRatio(SDL_lbistRegs *pLBISTRegs, uint32_t divideRatio);

static void SDL_LBIST_setNumStuckAtPatterns(SDL_lbistRegs *pLBISTRegs, uint32_t stuckAtPatterns);

static void SDL_LBIST_setNumSetPatterns(SDL_lbistRegs *pLBISTRegs, uint32_t setPatterns);

static void SDL_LBIST_setNumResetPatterns(SDL_lbistRegs *pLBISTRegs, uint32_t resetPatterns);

static void SDL_LBIST_setNumChainTestPatterns(SDL_lbistRegs *pLBISTRegs, uint32_t chainTestPatterns);

static void SDL_LBIST_setSeed(SDL_lbistRegs *pLBISTRegs, uint64_t seed);

static void SDL_LBIST_setClockDelay(SDL_lbistRegs *pLBISTRegs, uint32_t clockDelay);

/**
 * Design: PROC_SDL-1037,PROC_SDL-1038
 */
int32_t SDL_LBIST_getMISR(SDL_lbistRegs *pLBISTRegs, uint32_t *pMISRValue)
{
    uint32_t regVal;
    uint32_t muxVal;
    int32_t status = SDL_PASS;

    if (pLBISTRegs == NULL)
    {
        status = SDL_EBADARGS;
    }
    else
    {
        /* Setting to 0 also selects compacted 32-bit version of full MISR */
        regVal  = SDL_LBIST_STAT_MISR_MUX_CTL_COMPACT_MISR;
        /* The MISR value is available at two locations :
         *  Choosing location SDL_LBIST_STAT_OUT_MUX_CTL_MISR_VALUE_1 */
        muxVal  = SDL_LBIST_STAT_OUT_MUX_CTL_MISR_VALUE_1;
        regVal |= (muxVal << SDL_LBIST_STAT_OUT_MUX_CTL_SHIFT);
        pLBISTRegs->LBIST_STAT = regVal;
        *pMISRValue  = pLBISTRegs->LBIST_MISR;
    }
    return status;
}

/**
 * Design: PROC_SDL-1039,PROC_SDL-1040
 */
int32_t SDL_LBIST_getExpectedMISR(const uint32_t *pLBISTSig, uint32_t *pEpectedMISRValue)
{
    int32_t status = SDL_PASS;
    if (pLBISTSig == NULL)
    {
        status = SDL_EBADARGS;
    }
    else
    {
        *pEpectedMISRValue = *pLBISTSig;
    }
    return status;
}

/**
 * Design: PROC_SDL-1021,PROC_SDL-1022
 */
int32_t SDL_LBIST_programConfig(SDL_lbistRegs *pLBISTRegs, const SDL_LBIST_config_t * const pConfig )
{
    int32_t status = SDL_PASS;

    if ((pLBISTRegs == NULL) || (pConfig == NULL))
    {
        status = SDL_EBADARGS;
    }
    else
    {
        SDL_LBIST_setClockDelay( pLBISTRegs, pConfig->dc_def );

        SDL_LBIST_setDivideRatio( pLBISTRegs, pConfig->divide_ratio );

        SDL_LBIST_clearLoadDiv( pLBISTRegs );
        SDL_LBIST_setLoadDiv( pLBISTRegs );

        SDL_LBIST_setNumStuckAtPatterns  ( pLBISTRegs, pConfig->static_pc_def );
        SDL_LBIST_setNumSetPatterns      ( pLBISTRegs, pConfig->set_pc_def    );
        SDL_LBIST_setNumResetPatterns    ( pLBISTRegs, pConfig->reset_pc_def  );
        SDL_LBIST_setNumChainTestPatterns( pLBISTRegs, pConfig->scan_pc_def   );

        SDL_LBIST_setSeed( pLBISTRegs, pConfig->prpg_def );
    }
    return status;
}

/**
 * Design: PROC_SDL-1023,PROC_SDL-1024
 */
int32_t SDL_LBIST_enableIsolation(SDL_lbistRegs *pLBISTRegs)
{
    int32_t status = SDL_PASS;

    if (pLBISTRegs == NULL)
    {
        status = SDL_EBADARGS;
    }
    else
    {
        pLBISTRegs->LBIST_SPARE0 |= SDL_LBIST_SPARE0_LBIST_SELFTEST_EN_MASK;
    }
    return status;
}

/**
 * Design: PROC_SDL-1041,PROC_SDL-1042
 */
int32_t SDL_LBIST_disableIsolation(SDL_lbistRegs *pLBISTRegs)
{
    int32_t status = SDL_PASS;

    if (pLBISTRegs == NULL)
    {
        status = SDL_EBADARGS;
    }
    else
    {
        pLBISTRegs->LBIST_SPARE0 &= (~((uint32_t)SDL_LBIST_SPARE0_LBIST_SELFTEST_EN_MASK));
    }
    return status;
}

/**
 * Design: PROC_SDL-1025,PROC_SDL-1026
 */
int32_t SDL_LBIST_reset(SDL_lbistRegs *pLBISTRegs)
{
    int32_t status = SDL_PASS;

    if (pLBISTRegs == NULL)
    {
        status = SDL_EBADARGS;
    }
    else
    {
        pLBISTRegs->LBIST_CTRL &= (~((uint32_t)SDL_LBIST_CTRL_BIST_RESET_MAX << SDL_LBIST_CTRL_BIST_RESET_SHIFT));
    }
    return status;
}

/**
 * Design: PROC_SDL-1027,PROC_SDL-1028
 */
int32_t SDL_LBIST_enableRunBISTMode(SDL_lbistRegs *pLBISTRegs)
{
    int32_t status = SDL_PASS;

    if (pLBISTRegs == NULL)
    {
        status = SDL_EBADARGS;
    }
    else
    {
        pLBISTRegs->LBIST_CTRL |= ((uint32_t)SDL_LBIST_CTRL_RUNBIST_MODE_MAX << SDL_LBIST_CTRL_RUNBIST_MODE_SHIFT);
    }
    return status;
}

/**
 * Design: PROC_SDL-1043,PROC_SDL-1044
 */
int32_t SDL_LBIST_clearRunBISTMode(SDL_lbistRegs *pLBISTRegs)
{
    int32_t status = SDL_PASS;

    if (pLBISTRegs == NULL)
    {
        status = SDL_EBADARGS;
    }
    else
    {
        pLBISTRegs->LBIST_CTRL &= (~((uint32_t)SDL_LBIST_CTRL_RUNBIST_MODE_MAX << SDL_LBIST_CTRL_RUNBIST_MODE_SHIFT));
    }
    return status;
}

/**
 * Design: PROC_SDL-1029,PROC_SDL-1030
 */
int32_t SDL_LBIST_start(SDL_lbistRegs *pLBISTRegs)
{
    int32_t status = SDL_PASS;

    if (pLBISTRegs == NULL)
    {
        status = SDL_EBADARGS;
    }
    else
    {
        pLBISTRegs->LBIST_CTRL |= ((uint32_t)SDL_LBIST_CTRL_BIST_RESET_MAX << SDL_LBIST_CTRL_BIST_RESET_SHIFT);  /* Deassert reset */
        pLBISTRegs->LBIST_CTRL |= ((uint32_t)SDL_LBIST_CTRL_BIST_RUN_MAX << SDL_LBIST_CTRL_BIST_RUN_SHIFT);
    }
    return status;
}

/**
 * Design: PROC_SDL-1031,PROC_SDL-1032
 */
int32_t SDL_LBIST_stop(SDL_lbistRegs *pLBISTRegs)
{
    int32_t status = SDL_PASS;

    if (pLBISTRegs == NULL)
    {
        status = SDL_EBADARGS;
    }
    else
    {
        pLBISTRegs->LBIST_CTRL &= (~((uint32_t)SDL_LBIST_CTRL_BIST_RUN_MAX << SDL_LBIST_CTRL_BIST_RUN_SHIFT));
    }
    return status;
}

/**
 * Design: PROC_SDL-1033,PROC_SDL-1034
 */
int32_t SDL_LBIST_isRunning (const SDL_lbistRegs *pLBISTRegs, bool *pIsRunning)
{
    int32_t status = SDL_PASS;

    if (pLBISTRegs == NULL)
    {
        status = SDL_EBADARGS;
    }
    else
    {
        *pIsRunning = ((pLBISTRegs->LBIST_STAT & SDL_LBIST_STAT_BIST_RUNNING_MASK) != ((uint32_t)0u))
                          ? (bool)true : (bool)false;
    }
    return status;
}

/**
 * Design: PROC_SDL-1035,PROC_SDL-1036
 */
int32_t SDL_LBIST_isDone (const SDL_lbistRegs *pLBISTRegs, bool *pIsDone)
{
    int32_t status = SDL_PASS;

    if (pLBISTRegs == NULL)
    {
        status = SDL_EBADARGS;
    }
    else
    {
        if ((pLBISTRegs->LBIST_STAT & SDL_LBIST_STAT_BIST_DONE_MASK) != ((uint32_t)0u))
        {
          *pIsDone = TRUE;

        }
        else
        {
          *pIsDone = FALSE;
        }
    }
    return status;
}

static void SDL_LBIST_setLoadDiv(SDL_lbistRegs *pLBISTRegs)
{
    pLBISTRegs->LBIST_CTRL |= SDL_LBIST_CTRL_LOAD_DIV_MASK;
}

static void SDL_LBIST_clearLoadDiv(SDL_lbistRegs *pLBISTRegs)
{
    pLBISTRegs->LBIST_CTRL &= (~((uint32_t)SDL_LBIST_CTRL_LOAD_DIV_MASK));
}

static void SDL_LBIST_setDivideRatio(SDL_lbistRegs *pLBISTRegs,
                                     uint32_t divideRatio)
{
    pLBISTRegs->LBIST_CTRL &= (~(uint32_t)SDL_LBIST_CTRL_DIVIDE_RATIO_MASK);
    pLBISTRegs->LBIST_CTRL |= (uint32_t)(divideRatio & SDL_LBIST_CTRL_DIVIDE_RATIO_MASK);
}

static void SDL_LBIST_setNumStuckAtPatterns(SDL_lbistRegs *pLBISTRegs,
                                            uint32_t stuckAtPatterns)
{
    pLBISTRegs->LBIST_PATCOUNT &= (~((uint32_t)SDL_LBIST_PATCOUNT_STATIC_PC_DEF_MASK));
    pLBISTRegs->LBIST_PATCOUNT |= ((stuckAtPatterns & SDL_LBIST_PATCOUNT_STATIC_PC_DEF_MAX)
                                << SDL_LBIST_PATCOUNT_STATIC_PC_DEF_SHIFT);
}

static void SDL_LBIST_setNumSetPatterns(SDL_lbistRegs *pLBISTRegs,
                                        uint32_t setPatterns)
{
    pLBISTRegs->LBIST_PATCOUNT &= (~((uint32_t)SDL_LBIST_PATCOUNT_SET_PC_DEF_MASK));
    pLBISTRegs->LBIST_PATCOUNT |= ((setPatterns & SDL_LBIST_PATCOUNT_RESET_PC_DEF_MAX)
                                << SDL_LBIST_PATCOUNT_SET_PC_DEF_SHIFT);
}

static void SDL_LBIST_setNumResetPatterns(SDL_lbistRegs *pLBISTRegs,
                                          uint32_t resetPatterns)
{
    pLBISTRegs->LBIST_PATCOUNT &= (~((uint32_t)SDL_LBIST_PATCOUNT_RESET_PC_DEF_MASK));
    pLBISTRegs->LBIST_PATCOUNT |= ((resetPatterns & SDL_LBIST_PATCOUNT_RESET_PC_DEF_MAX)
                                << SDL_LBIST_PATCOUNT_RESET_PC_DEF_SHIFT);
}

static void SDL_LBIST_setNumChainTestPatterns(SDL_lbistRegs *pLBISTRegs,
                                              uint32_t chainTestPatterns)
{
    pLBISTRegs->LBIST_PATCOUNT &= (~((uint32_t)SDL_LBIST_PATCOUNT_SCAN_PC_DEF_MASK));
    pLBISTRegs->LBIST_PATCOUNT |= ((chainTestPatterns & SDL_LBIST_PATCOUNT_SCAN_PC_DEF_MAX)
                                << SDL_LBIST_PATCOUNT_SCAN_PC_DEF_SHIFT);
}

static void SDL_LBIST_setSeed(SDL_lbistRegs *pLBISTRegs, uint64_t seed)
{
    pLBISTRegs->LBIST_SEED0 = ((uint32_t)seed & SDL_LBIST_SEED0_PRPG_DEF_MASK);
    pLBISTRegs->LBIST_SEED1 = ((seed >> 32) & SDL_LBIST_SEED1_PRPG_DEF_MASK);
}

static void SDL_LBIST_setClockDelay(SDL_lbistRegs *pLBISTRegs,
                                    uint32_t clockDelay)
{
    pLBISTRegs->LBIST_CTRL &= (SDL_LBIST_CTRL_DC_DEF_MASK);
    pLBISTRegs->LBIST_CTRL |= ((clockDelay & SDL_LBIST_CTRL_DC_DEF_MAX)
                            << SDL_LBIST_CTRL_DC_DEF_SHIFT);
}

/* Nothing past this point */
