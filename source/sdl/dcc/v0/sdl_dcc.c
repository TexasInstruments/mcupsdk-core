/*
 *  Copyright (c) 2022-2024 Texas Instruments Incorporated
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
 *  \file     sdl_dcc.c
 *
 *  \brief    This file contains the implementation of the APIs present in the
 *            device abstraction layer file of DCC.
 *            This also contains some related macros.
 */

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include "sdlr_dcc2.h"
#include "sdl_dcc.h"

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */


/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

/* Internal functions */

static int32_t SDL_DCC_checkMode(SDL_DCC_Mode mode)
{
    int32_t sdlResult = SDL_PASS;

    if ((mode != SDL_DCC_MODE_SINGLE_SHOT_1) &&
        (mode != SDL_DCC_MODE_SINGLE_SHOT_2) &&
        (mode != SDL_DCC_MODE_CONTINUOUS))
    {
        sdlResult = SDL_EFAIL;
    }

    return sdlResult;
}

static int32_t SDL_DCC_checkClkSrc0(SDL_DCC_ClkSrc0 clkSrc0)
{
    int32_t sdlResult = SDL_PASS;

    if ((clkSrc0!= SDL_DCC_CLK0_SRC_CLOCK0_0) &&
        (clkSrc0 != SDL_DCC_CLK0_SRC_CLOCK0_1) &&
        (clkSrc0 != SDL_DCC_CLK0_SRC_CLOCK0_2))
    {
        sdlResult = SDL_EFAIL;
    }

    return sdlResult;
}

static int32_t SDL_DCC_checkClkSrc1(SDL_DCC_ClkSrc1 clkSrc1)
{
    int32_t sdlResult = SDL_PASS;

    if ((clkSrc1 != SDL_DCC_CLK1_SRC_CLOCK1) &&
        (clkSrc1 != SDL_DCC_CLK1_SRC_CLOCKSRC0) &&
        (clkSrc1 != SDL_DCC_CLK1_SRC_CLOCKSRC1) &&
        (clkSrc1 != SDL_DCC_CLK1_SRC_CLOCKSRC2) &&
        (clkSrc1 != SDL_DCC_CLK1_SRC_CLOCKSRC3) &&
        (clkSrc1 != SDL_DCC_CLK1_SRC_CLOCKSRC4) &&
        (clkSrc1 != SDL_DCC_CLK1_SRC_CLOCKSRC5) &&
        (clkSrc1 != SDL_DCC_CLK1_SRC_CLOCKSRC6) &&
        (clkSrc1 != SDL_DCC_CLK1_SRC_CLOCKSRC7) &&
        (clkSrc1 != SDL_DCC_CLK1_SRC_FICLK))
    {
        sdlResult = SDL_EFAIL;
    }
    return sdlResult;
}

/*******************************************************************************
 *   API for configuring the DCC module
 ******************************************************************************/

/**
 *  Design: PROC_SDL-2071
 */

int32_t SDL_DCC_configure(SDL_DCC_Inst instance, const SDL_DCC_Config *pConfig)
{
    int32_t sdlResult = SDL_EFAIL;
    uint32_t clksrcVal, baseAddr;

    if((instance < SDL_DCC_INVALID_INSTANCE) && (pConfig != NULL))
    {
        if ((pConfig->clk0Seed      <= DCC_SRC0_COUNT_MAX) &&
            (pConfig->clk0ValidSeed <= DCC_SRC0_VALID_MAX) &&
            (pConfig->clk1Seed      <= DCC_SRC1_COUNT_MAX) &&
            (SDL_DCC_checkMode(pConfig->mode) != SDL_EFAIL) &&
            (SDL_DCC_checkClkSrc0(pConfig->clk0Src) != SDL_EFAIL) &&
            (SDL_DCC_checkClkSrc1(pConfig->clk1Src) != SDL_EFAIL))
        {
            /* Getting base address */
            (void)SDL_DCC_getBaseaddr(instance, &baseAddr);

            /* Configure DCC mode of operation */
            HW_WR_FIELD32(baseAddr + SDL_DCC2_DCCGCTRL, SDL_DCC2_DCCGCTRL_SINGLESHOT,
                          pConfig->mode);

            /* Select clock source for COUNT0 */
            clksrcVal = ((pConfig->clk0Src & SDL_DCC2_DCCCLKSRC0_CLKSRC0_MASK) |
                        ((uint32_t)SDL_DCC2_DCCCLKSRC0_KEY_ENABLE << SDL_DCC2_DCCCLKSRC0_KEY_SHIFT));

            HW_WR_REG32(baseAddr + SDL_DCC2_DCCCLKSRC0, clksrcVal);

            /* Select clock source for COUNT1*/
            /* Enable clock source selection for COUNT1 */
            clksrcVal = (((pConfig->clk1Src % (uint32_t)16U) << SDL_DCC2_DCCCLKSRC1_CLKSRC1_SHIFT) |
                        ((uint32_t)SDL_DCC2_DCCCLKSRC1_KEY_ENABLE << SDL_DCC2_DCCCLKSRC1_KEY_SHIFT));

            HW_WR_REG32(baseAddr + SDL_DCC2_DCCCLKSRC1, clksrcVal);

            /* Configure COUNT0 preload/seed value */
            HW_WR_FIELD32(baseAddr + SDL_DCC2_DCCCNTSEED0, SDL_DCC2_DCCCNT0_COUNT0,
                          pConfig->clk0Seed);

            if(pConfig->clk0ValidSeed < DCC_MIN_CLK0_VLD_SEED)
            {
                /* Configure VALID0 preload/seed value */
                HW_WR_FIELD32(baseAddr + SDL_DCC2_DCCVALIDSEED0, SDL_DCC2_DCCVALID0_VALID0,
                              DCC_MIN_CLK0_VLD_SEED);
            }
            else
            {
                /* Configure VALID0 preload/seed value */
                HW_WR_FIELD32(baseAddr + SDL_DCC2_DCCVALIDSEED0, SDL_DCC2_DCCVALID0_VALID0,
                              pConfig->clk0ValidSeed);
            }
            /* Configure COUNT1 preload/seed value */
            HW_WR_FIELD32(baseAddr + SDL_DCC2_DCCCNTSEED1, SDL_DCC2_DCCCNT1_COUNT1,
                          pConfig->clk1Seed);
            sdlResult = SDL_PASS;
        }
        else
        {
            sdlResult = SDL_EBADARGS;
        }
    }
    else
    {
        sdlResult = SDL_EBADARGS;
    }
    return sdlResult;
}


/********************************************************************************************************
*   API for verify the configuration of DCC module
*********************************************************************************************************/

/**
 *  Design: PROC_SDL-2074
 */
int32_t SDL_DCC_verifyConfig(SDL_DCC_Inst instance, const SDL_DCC_Config *pConfig)
{
    uint32_t mode_chk;
    uint32_t clk0Src_chk, clk1Src_chk, baseAddr;
    uint32_t clk0Seed_chk, clk0ValidSeed_chk, clk1Seed_chk;
    int32_t  sdlResult;

    if((instance < SDL_DCC_INVALID_INSTANCE) && (pConfig != NULL) )
    {
        /* Getting base address */
        (void)SDL_DCC_getBaseaddr(instance, &baseAddr);

        /* Get the configured mode of operation for DCC module */
        mode_chk = HW_RD_FIELD32(baseAddr + SDL_DCC2_DCCGCTRL,
                                 SDL_DCC2_DCCGCTRL_SINGLESHOT);

        /* Get the clock source for COUNT0 */
        clk0Src_chk = HW_RD_FIELD32(baseAddr + SDL_DCC2_DCCCLKSRC0,
                                    SDL_DCC2_DCCCLKSRC0_CLKSRC0);

        /* Get the clock source for COUNT1 */
        clk1Src_chk = HW_RD_FIELD32(baseAddr + SDL_DCC2_DCCCLKSRC1,
                                    SDL_DCC2_DCCCLKSRC1_CLKSRC1);

        /* Get the current value of COUNT0 */
        clk0Seed_chk = HW_RD_FIELD32(baseAddr + SDL_DCC2_DCCCNTSEED0,
                                     SDL_DCC2_DCCCNT0_COUNT0);

        /* Get the current value of VALID0 */
        clk0ValidSeed_chk = HW_RD_FIELD32(baseAddr + SDL_DCC2_DCCVALIDSEED0,
                                          SDL_DCC2_DCCVALID0_VALID0);

        /* Get the current value of COUNT1 */
        clk1Seed_chk = HW_RD_FIELD32(baseAddr + SDL_DCC2_DCCCNTSEED1,
                                     SDL_DCC2_DCCCNT1_COUNT1);

        if((mode_chk == pConfig->mode) && \
           (clk0Src_chk == pConfig->clk0Src) && \
           (clk1Src_chk == pConfig->clk1Src) && \
           (clk0Seed_chk == pConfig->clk0Seed) && \
           (clk0ValidSeed_chk == pConfig->clk0ValidSeed) && \
           (clk1Seed_chk == pConfig->clk1Seed))
        {
            sdlResult = SDL_PASS;
        }
        else
        {
            sdlResult = SDL_EFAIL;
        }
    }
    else
    {
        sdlResult = SDL_EBADARGS;
    }

    return sdlResult;
}


/********************************************************************************************************
*   API for Enabling the DCC module
*********************************************************************************************************/

/**
 *  Design: PROC_SDL-2072
 */
int32_t SDL_DCC_enable(SDL_DCC_Inst instance)
{
    uint32_t baseAddr;
    int32_t sdlResult;

    if(instance < SDL_DCC_INVALID_INSTANCE)
    {
        /* Getting base address */
        (void)SDL_DCC_getBaseaddr(instance, &baseAddr);
        /* Enable DCC */
        HW_WR_FIELD32(baseAddr + SDL_DCC2_DCCGCTRL, SDL_DCC2_DCCGCTRL_DCCENA,
                      SDL_DCC2_DCCGCTRL_DCCENA_ENABLE);
        sdlResult = SDL_PASS;
    }
    else
    {
        sdlResult = SDL_EBADARGS;
    }

    return sdlResult;
}


/********************************************************************************************************
*   API for Disabling the DCC module
*********************************************************************************************************/

/**
 *  Design: PROC_SDL-2073
 */
int32_t SDL_DCC_disable(SDL_DCC_Inst instance)
{
    uint32_t baseAddr;
    int32_t sdlResult;

    if(instance < SDL_DCC_INVALID_INSTANCE)
    {
        /* Getting base address */
        (void)SDL_DCC_getBaseaddr(instance, &baseAddr);
        /* Disable DCC */
        HW_WR_FIELD32(baseAddr + SDL_DCC2_DCCGCTRL, SDL_DCC2_DCCGCTRL_DCCENA,
                      SDL_DCC2_DCCGCTRL_DCCENA_DISABLE);

        sdlResult = SDL_PASS;
    }
    else
    {
        sdlResult = SDL_EBADARGS;
    }

    return sdlResult;
}


/********************************************************************************************************
*   API for getting the status of specified DCC instance
*********************************************************************************************************/

/**
 *  Design: PROC_SDL-2075
 */
int32_t SDL_DCC_getStatus(SDL_DCC_Inst instance, SDL_DCC_Status *pStatus)
{
    uint32_t baseAddr;
    int32_t sdlResult, intrStatus;

    if((instance < SDL_DCC_INVALID_INSTANCE) && (pStatus != NULL))
    {
        /* Getting base address */
        (void)SDL_DCC_getBaseaddr(instance, &baseAddr);

        /* Checking if DONE Interrupt occured or not */
        /* TRUE = Interrupt pending, FALSE = INTERRUPT not pending */
        intrStatus = (int32_t)HW_RD_FIELD32(baseAddr + SDL_DCC2_DCCSTATUS, SDL_DCC2_DCCSTATUS_DONE);
        if(intrStatus !=  (int32_t)0U)
        {
            pStatus->doneIntr = TRUE;
        }
        else
        {
            pStatus->doneIntr = FALSE;
        }

        /* Checking if ERROR Interrupt occured or not */
        /* TRUE = Interrupt pending, FALSE = INTERRUPT not pending */
        intrStatus = (int32_t)HW_RD_FIELD32(baseAddr + SDL_DCC2_DCCSTATUS, SDL_DCC2_DCCSTATUS_ERR);
        if(intrStatus != (int32_t)0U)
        {
            pStatus->errIntr = TRUE;
        }
        else
        {
            pStatus->errIntr = FALSE;
        }


        /* Getting Current configuration of DCC */
        /* Get the configured mode of operation for DCC module */
        pStatus->config.mode          = HW_RD_FIELD32(baseAddr + SDL_DCC2_DCCGCTRL,
                                        SDL_DCC2_DCCGCTRL_SINGLESHOT);
        /* Get the clock source for COUNT0 */
        pStatus->config.clk0Src       = HW_RD_FIELD32(baseAddr + SDL_DCC2_DCCCLKSRC0,
                                                SDL_DCC2_DCCCLKSRC0_CLKSRC0);
        /* Get the clock source for COUNT1 */
        pStatus->config.clk1Src          = HW_RD_FIELD32(baseAddr + SDL_DCC2_DCCCLKSRC1,
                                                    SDL_DCC2_DCCCLKSRC1_CLKSRC1);
        /* Get the current value of COUNT0 */
        pStatus->config.clk0Seed      = HW_RD_FIELD32(baseAddr + SDL_DCC2_DCCCNTSEED0,
                                                        SDL_DCC2_DCCCNT0_COUNT0);
        /* Get the current value of VALID0 */
        pStatus->config.clk0ValidSeed = HW_RD_FIELD32(baseAddr + SDL_DCC2_DCCVALIDSEED0,
                                                                SDL_DCC2_DCCVALID0_VALID0);
        /* Get the current value of COUNT1 */
        pStatus->config.clk1Seed      = HW_RD_FIELD32(baseAddr + SDL_DCC2_DCCCNTSEED1,
                                                    SDL_DCC2_DCCCNT1_COUNT1);

        /* Current values of counters  */
        /* Get the current value of COUNT0 */
        pStatus->clk0Cnt = HW_RD_FIELD32(baseAddr + SDL_DCC2_DCCCNTSEED0,
                                                        SDL_DCC2_DCCCNT0_COUNT0);

        /* Get the current value of VALID0 */
        pStatus->clk0Valid = HW_RD_FIELD32(baseAddr + SDL_DCC2_DCCVALIDSEED0,
                                                                SDL_DCC2_DCCVALID0_VALID0);

        /* Get the current value of COUNT1 */
        pStatus->clk1Cnt = HW_RD_FIELD32(baseAddr + SDL_DCC2_DCCCNTSEED1,
                                                    SDL_DCC2_DCCCNT1_COUNT1);

        sdlResult = SDL_PASS;
    }
    else
    {
        sdlResult = SDL_EBADARGS;
    }

    return sdlResult;
}



/********************************************************************************************************
*   API for Enabling the Error and Done Interrupts
*********************************************************************************************************/

/**
 *  Design: PROC_SDL-2140
 */
int32_t SDL_DCC_enableIntr(SDL_DCC_Inst instance, SDL_DCC_IntrType intr)
{
    int32_t sdlResult = SDL_EFAIL;
    uint32_t baseAddr;

    if(instance < SDL_DCC_INVALID_INSTANCE)
    {
        /* Getting base address */
        (void)SDL_DCC_getBaseaddr(instance, &baseAddr);

        switch (intr)
        {
            case SDL_DCC_INTERRUPT_ERR:
                /* Enable ERROR interrupt */
                HW_WR_FIELD32(baseAddr + SDL_DCC2_DCCGCTRL, SDL_DCC2_DCCGCTRL_ERRENA,
                            SDL_DCC2_DCCGCTRL_ERRENA_ENABLE);
                sdlResult = SDL_PASS;
                break;
            case SDL_DCC_INTERRUPT_DONE:
                /* Enable DONE interrupt(only for single shot mode) */
                HW_WR_FIELD32(baseAddr + SDL_DCC2_DCCGCTRL, SDL_DCC2_DCCGCTRL_DONEENA,
                            SDL_DCC2_DCCGCTRL_DONEENA_ENABLE);
                sdlResult = SDL_PASS;
                break;
            default:
                sdlResult = SDL_EFAIL;
                break;
        }
    }
    else
    {
        sdlResult = SDL_EBADARGS;
    }

    return sdlResult;

}


/********************************************************************************************************
*   API for disabling the Interrupt
*********************************************************************************************************/

/**
 *  Design: PROC_SDL-2076
 */
int32_t SDL_DCC_disableIntr(SDL_DCC_Inst instance, SDL_DCC_IntrType intr)
{
    int32_t sdlResult = SDL_EFAIL;
    uint32_t baseAddr;

    if(instance < SDL_DCC_INVALID_INSTANCE)
    {
        /* Getting base address */
        (void)SDL_DCC_getBaseaddr(instance, &baseAddr);

        switch (intr)
        {
            case SDL_DCC_INTERRUPT_ERR:
                /* Disable ERROR interrupt */
                HW_WR_FIELD32(baseAddr + SDL_DCC2_DCCGCTRL, SDL_DCC2_DCCGCTRL_ERRENA,
                            SDL_DCC2_DCCGCTRL_ERRENA_DISABLE);
                sdlResult = SDL_PASS;
                break;
            case SDL_DCC_INTERRUPT_DONE:
                /* Disable DONE interrupt(only for single shot mode) */
                HW_WR_FIELD32(baseAddr + SDL_DCC2_DCCGCTRL, SDL_DCC2_DCCGCTRL_DONEENA,
                            SDL_DCC2_DCCGCTRL_DONEENA_DISABLE);
                sdlResult = SDL_PASS;
                break;
            default:
                sdlResult = SDL_EBADARGS;
                break;
        }
    }
    else
    {
        sdlResult = SDL_EBADARGS;
    }
    return sdlResult;
}

/********************************************************************************************************
*   API for clearing the Interrupt
*********************************************************************************************************/

/**
 *  Design: PROC_SDL-??
 */
int32_t SDL_DCC_clearIntr(SDL_DCC_Inst instance, SDL_DCC_IntrType intr)
{
    int32_t sdlResult = SDL_EFAIL;
    uint32_t baseAddr;

    if(instance < SDL_DCC_INVALID_INSTANCE)
    {
        /* Getting base address */
        (void)SDL_DCC_getBaseaddr(instance, &baseAddr);

        /* Clear status of given interrupt */
        HW_WR_REG32(baseAddr + SDL_DCC2_DCCSTATUS, ((uint32_t) 0x1U << intr));
        sdlResult = SDL_PASS;
    }
    else
    {
        sdlResult = SDL_EBADARGS;
    }
    return sdlResult;
}


/********************************************************************************************************
*   API for reading the static registers values
*********************************************************************************************************/

/**
 *  Design: PROC_SDL-2077
 */
int32_t SDL_DCC_getStaticRegs(SDL_DCC_Inst instance, SDL_DCC_StaticRegs *pStaticRegs)
{
    uint32_t baseAddr;
    int32_t  sdlResult;

    if((instance < SDL_DCC_INVALID_INSTANCE) && (pStaticRegs != NULL))
    {
        /* Getting base address */
        (void)SDL_DCC_getBaseaddr(instance, &baseAddr);

        /* Get the complete DCC_REV register value */
        pStaticRegs->DCC_REV = HW_RD_REG32_RAW(baseAddr + SDL_DCC2_DCCREV);

        /* Get the clock source for COUNT0 */
        pStaticRegs->DCC_CLKSRC0 = HW_RD_FIELD32(baseAddr + SDL_DCC2_DCCCLKSRC0,
                                                 SDL_DCC2_DCCCLKSRC0_CLKSRC0);

        /* Get the clock source for COUNT1 */
        pStaticRegs->DCC_CLKSRC1 = HW_RD_FIELD32(baseAddr + SDL_DCC2_DCCCLKSRC1,
                                                 SDL_DCC2_DCCCLKSRC1_CLKSRC1);

        /* Get the current value of COUNT0 */
        pStaticRegs->DCC_CNTSEED0 = HW_RD_FIELD32(baseAddr + SDL_DCC2_DCCCNTSEED0,
                                                  SDL_DCC2_DCCCNT0_COUNT0);

        /* Get the current value of VALID0 */
        pStaticRegs->DCC_VALIDSEED0 = HW_RD_FIELD32(baseAddr + SDL_DCC2_DCCVALIDSEED0,
                                                    SDL_DCC2_DCCVALID0_VALID0);

        /* Get the current value of COUNT1 */
        pStaticRegs->DCC_CNTSEED1 = HW_RD_FIELD32(baseAddr + SDL_DCC2_DCCCNTSEED1,
                                                  SDL_DCC2_DCCCNT1_COUNT1);

        sdlResult = SDL_PASS;
    }
    else
    {
        sdlResult = SDL_EBADARGS;
    }

    return sdlResult;

}




