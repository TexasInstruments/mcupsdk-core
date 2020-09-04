/*
 *  Copyright (C) 2021 Texas Instruments Incorporated
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
 *  \file sdl_hwa.c
 *
 *  \brief Common across test-cases using HWA
 */

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */
#include "hwa_main.h"

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */
#define SDL_HWA_ECC_TIMEOUT    (0x10000U)
/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */
/* ========================================================================== */
/*                         Structure Declarations                             */
/* ========================================================================== */


/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */


/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */
/** \brief Defines the various HWA test cases. */
int32_t SDL_HWA_DMA0_SEC_test(void)
{
    int32_t ret_val = SDL_EFAIL;
    uint32_t timeout = SDL_HWA_ECC_TIMEOUT;
    SDL_HWA_DMA0_secExecute();
    /* wait for error notification from ESM, or for timeout */
    /* fault injection gets deasserted in ISR */
    while((SDL_HWA_DMA0_secErrorStatus()!=0U) && (timeout!=0U))
    {
        timeout--;
    }
    if(SDL_HWA_DMA0_secErrorStatus()==1U)
    {
        SDL_HWA_DMA0_secErrorClear();
        ret_val = SDL_PASS;
    }
    else
    {
        ret_val = SDL_EFAIL;
    }
    return ret_val;
}

int32_t SDL_HWA_DMA1_SEC_test(void)
{
    int32_t ret_val = SDL_EFAIL;
    uint32_t timeout = SDL_HWA_ECC_TIMEOUT;
    SDL_HWA_DMA1_secExecute();
    /* wait for error notification from ESM, or for timeout */
    /* fault injection gets deasserted in ISR */
    while((SDL_HWA_DMA1_secErrorStatus()!=0U) && (timeout!=0U))
    {
        timeout--;
    }
    if(SDL_HWA_DMA1_secErrorStatus()==1U)
    {
        SDL_HWA_DMA1_secErrorClear();
        ret_val = SDL_PASS;
    }
    else
    {
        ret_val = SDL_EFAIL;
    }
    return ret_val;
}

int32_t SDL_HWA_DMA0_DED_test(void)
{
    int32_t ret_val = SDL_EFAIL;
    uint32_t timeout = SDL_HWA_ECC_TIMEOUT;
    SDL_HWA_DMA0_dedExecute();
    /* wait for error notification from ESM, or for timeout */
    /* fault injection gets deasserted in ISR */
    while((SDL_HWA_DMA0_dedErrorStatus()!=0U) && (timeout!=0U))
    {
        timeout--;
    }
    if(SDL_HWA_DMA0_dedErrorStatus()==1U)
    {
        SDL_HWA_DMA0_dedErrorClear();
        ret_val = SDL_PASS;
    }
    else
    {
        ret_val = SDL_EFAIL;
    }
    return ret_val;
}

int32_t SDL_HWA_DMA1_DED_test(void)
{
    int32_t ret_val = SDL_EFAIL;
    uint32_t timeout = SDL_HWA_ECC_TIMEOUT;
    SDL_HWA_DMA1_dedExecute();
    /* wait for error notification from ESM, or for timeout */
    /* fault injection gets deasserted in ISR */
    while((SDL_HWA_DMA1_dedErrorStatus()!=0U) && (timeout!=0U))
    {
        timeout--;
    }
    if(SDL_HWA_DMA1_dedErrorStatus()==1U)
    {
        SDL_HWA_DMA1_dedErrorClear();
        ret_val = SDL_PASS;
    }
    else
    {
        ret_val = SDL_EFAIL;
    }
    return ret_val;
}

int32_t SDL_HWA_DMA0_RED_test(void)
{
    int32_t ret_val = SDL_EFAIL;
    uint32_t timeout = SDL_HWA_ECC_TIMEOUT;
    ret_val= SDL_HWA_DMA0_redExecute(SDL_HWA_FI_GLOBAL_SAFE, SDL_HWA_MAIN_CMD_INTERFACE);
    if(ret_val == SDL_PASS )
    {
        /* Wait for test to complete/timeout. */
        while((SDL_HWA_DMA0_redErrorStatus()!=0U) && (timeout!=0U))
        {
            timeout--;
        }
        /* Check for the failure. */
        if(SDL_HWA_DMA0_redErrorStatus()==1U)
        {
            SDL_HWA_DMA0_redErrorClear();
            ret_val = SDL_PASS;
        }
        else
        {
            ret_val = SDL_EFAIL;
        }
    }
    else
    {
        ret_val = SDL_EFAIL;
    }
    return ret_val;
}

int32_t SDL_HWA_DMA1_RED_test(void)
{
    int32_t ret_val = SDL_EFAIL;
    uint32_t timeout = SDL_HWA_ECC_TIMEOUT;
    ret_val= SDL_HWA_DMA1_redExecute(SDL_HWA_FI_GLOBAL_SAFE, SDL_HWA_MAIN_CMD_INTERFACE);
    if(ret_val == SDL_PASS )
    {
        /* Wait for test to complete/timeout. */
        while((SDL_HWA_DMA1_redErrorStatus()!=0U) && (timeout!=0U))
        {
            timeout--;
        }
        /* Check for the failure. */
        if(SDL_HWA_DMA0_redErrorStatus()==1U)
        {
            SDL_HWA_DMA1_redErrorClear();
            ret_val = SDL_PASS;
        }
        else
        {
            ret_val = SDL_EFAIL;
        }
    }
    else
    {
        ret_val = SDL_EFAIL;
    }
    return ret_val;
}

int32_t hwaParityDMA0DMEM0_testExecute(void)
{
    return (SDL_HWA_memParityExecute(SDL_HWA_DMA0_MEM_ID , SDL_HWA_DMEM0));
}
int32_t hwaParityDMA0DMEM1_testExecute(void)
{
    return (SDL_HWA_memParityExecute(SDL_HWA_DMA0_MEM_ID , SDL_HWA_DMEM1));
}
int32_t hwaParityDMA0DMEM2_testExecute(void)
{
    return(SDL_HWA_memParityExecute(SDL_HWA_DMA0_MEM_ID , SDL_HWA_DMEM2));
}
int32_t hwaParityDMA0DMEM3_testExecute(void)
{
    return(SDL_HWA_memParityExecute(SDL_HWA_DMA0_MEM_ID , SDL_HWA_DMEM3));
}
int32_t hwaParityDMA0DMEM4_testExecute(void)
{
    return(SDL_HWA_memParityExecute(SDL_HWA_DMA0_MEM_ID , SDL_HWA_DMEM4));
}
int32_t hwaParityDMA0DMEM5_testExecute(void)
{
    return(SDL_HWA_memParityExecute(SDL_HWA_DMA0_MEM_ID , SDL_HWA_DMEM5));
}
int32_t hwaParityDMA0DMEM6_testExecute(void)
{
    return(SDL_HWA_memParityExecute(SDL_HWA_DMA0_MEM_ID , SDL_HWA_DMEM6));
}

int32_t hwaParityDMA0DMEM7_testExecute(void)
{
    return(SDL_HWA_memParityExecute(SDL_HWA_DMA0_MEM_ID , SDL_HWA_DMEM7));
}

int32_t hwaParityDMA0WindowRam_testExecute(void)
{
    return(SDL_HWA_memParityExecute(SDL_HWA_WINDOW_RAM_MEM_ID , SDL_HWA_WINDOW_RAM));
}

int32_t hwaParityDMA1DMEM0_testExecute(void)
{
    return(SDL_HWA_memParityExecute(SDL_HWA_DMA1_MEM_ID , SDL_HWA_DMEM0));
}
int32_t hwaParityDMA1DMEM1_testExecute(void)
{
    return(SDL_HWA_memParityExecute(SDL_HWA_DMA1_MEM_ID , SDL_HWA_DMEM1));
}
int32_t hwaParityDMA1DMEM2_testExecute(void)
{
    return(SDL_HWA_memParityExecute(SDL_HWA_DMA1_MEM_ID , SDL_HWA_DMEM2));
}
int32_t hwaParityDMA1DMEM3_testExecute(void)
{
    return(SDL_HWA_memParityExecute(SDL_HWA_DMA1_MEM_ID , SDL_HWA_DMEM3));
}
int32_t hwaParityDMA1DMEM4_testExecute(void)
{
    return(SDL_HWA_memParityExecute(SDL_HWA_DMA1_MEM_ID , SDL_HWA_DMEM4));
}
int32_t hwaParityDMA1DMEM5_testExecute(void)
{
    return(SDL_HWA_memParityExecute(SDL_HWA_DMA1_MEM_ID , SDL_HWA_DMEM5));
}
int32_t hwaParityDMA1DMEM6_testExecute(void)
{
    return(SDL_HWA_memParityExecute(SDL_HWA_DMA1_MEM_ID , SDL_HWA_DMEM6));
}
int32_t hwaParityDMA1DMEM7_testExecute(void)
{
    return(SDL_HWA_memParityExecute(SDL_HWA_DMA1_MEM_ID , SDL_HWA_DMEM7));
}
int32_t hwaParityDMA1WindowRam_testExecute(void)
{
    return(SDL_HWA_memParityExecute(SDL_HWA_WINDOW_RAM_MEM_ID , SDL_HWA_WINDOW_RAM));
}

