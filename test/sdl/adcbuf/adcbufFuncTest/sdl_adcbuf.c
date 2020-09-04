/*
 *  Copyright (C) 2022 Texas Instruments Incorporated
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
 *  \file sdl_adcbuf.c
 *
 *  \brief Common across test-cases using ADCBUF.
 *
 */

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */
#include "adcbuf_main.h"

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */
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
/** \brief Defines the various adcbuf test cases. */


int32_t SDL_ADCBUF_WR_SEC_test(void)
{
    int32_t ret_val = SDL_EFAIL;
    uint32_t timeout = SDL_ADCBUF_ECC_TIMEOUT;
    SDL_ADCBUF_WR_secExecute();
    /* wait for error notification from ESM, or for timeout */
    /* fault injection gets deasserted in ISR */
        while((SDL_ADCBUF_wrSecErrorStatus()!=1U) && (timeout--));
    {
        timeout--;
    }
    if(SDL_ADCBUF_wrSecErrorStatus()==1U)
    {
        SDL_ADCBUF_wrSecErrorClear();
        ret_val = SDL_PASS;
    }

    return ret_val;
}

int32_t SDL_ADCBUF_WR_DED_test(void)
{
    int32_t ret_val = SDL_EFAIL;
    uint32_t timeout = SDL_ADCBUF_ECC_TIMEOUT;
    SDL_ADCBUF_WR_dedExecute();
    /* wait for error notification from ESM, or for timeout */
    /* fault injection gets deasserted in ISR */
    /* wait for error notification from ESM, or for timeout */
        while((SDL_ADCBUF_wrDedErrorStatus()!=1U) && (timeout--));
    {
         timeout--;
    }

    if(SDL_ADCBUF_wrDedErrorStatus()==1U)
    {
        SDL_ADCBUF_wrDedErrorClear();
        ret_val = SDL_PASS;
    }

    return ret_val;
}


int32_t SDL_ADCBUF_WR_RED_test(void)
{
    int32_t ret_val = SDL_EFAIL;
    uint32_t timeout = SDL_ADCBUF_ECC_TIMEOUT;
    ret_val= SDL_ADCBUF_WR_redExecute(SDL_ADCBUF_FI_GLOBAL_SAFE, SDL_ADCBUF_MAIN_CMD_INTERFACE);
    if(ret_val == SDL_PASS )
    {
        /* Wait for test to complete/timeout. */
        while((SDL_ADCBUF_wrRedErrorStatus()!=1U) && (timeout--));
        /* Check for the failure. */
        if(SDL_ADCBUF_wrRedErrorStatus()==1U)
        {
            ret_val = SDL_PASS;
            SDL_ADCBUF_wrRedErrorClear();
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

int32_t SDL_ADCBUF_RD_RED_test(void)
{
    int32_t ret_val = SDL_EFAIL;
    uint32_t timeout = SDL_ADCBUF_ECC_TIMEOUT;
    ret_val= SDL_ADCBUF_RD_redExecute(SDL_ADCBUF_FI_GLOBAL_SAFE, SDL_ADCBUF_MAIN_CMD_INTERFACE);
    if(ret_val == SDL_PASS )
    {
        /* Wait for test to complete/timeout. */
        while((SDL_ADCBUF_rdRedErrorStatus()!=1U) && (timeout--));
        /* Check for the failure. */
        if(SDL_ADCBUF_rdRedErrorStatus()==1U)
        {
            ret_val = SDL_PASS;
            SDL_ADCBUF_rdRedErrorClear();
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



