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
 *  \file sdl_dss_l3.c
 *
 *  \brief Common across test-cases using DSS L3
 */

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */
#include "dss_l3_main.h"

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */
#define SDL_DSS_L3_ECC_TIMEOUT    (0x10000U)
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
/** \brief Defines the various DSS_L3 test cases. */

int32_t SDL_DSS_L3_BANKA_SEC_test(void)
{
    int32_t ret_val = SDL_EFAIL;
    uint32_t timeout = SDL_DSS_L3_ECC_TIMEOUT;
    SDL_DSS_L3_BANKA_secExecute();
    /* wait for error notification from ESM, or for timeout */
    /* fault injection gets deasserted in ISR */
    while((SDL_DSS_L3_BankA_secErrorStatus()!=1U) && (timeout!=0U));

    if(SDL_DSS_L3_BankA_secErrorStatus()==1U)
    {
        ret_val = SDL_PASS;
        SDL_DSS_L3_BankA_secErrorClear();
    }
    else
    {
        ret_val = SDL_EFAIL;
    }

    return ret_val;
}
int32_t SDL_DSS_L3_BANKB_SEC_test(void)
{
    int32_t ret_val = SDL_EFAIL;
    uint32_t timeout = SDL_DSS_L3_ECC_TIMEOUT;
    SDL_DSS_L3_BANKB_secExecute();
    /* wait for error notification from ESM, or for timeout */
    /* fault injection gets deasserted in ISR */
    while((SDL_DSS_L3_BankB_secErrorStatus()!=1U) && (timeout!=0U));

    if(SDL_DSS_L3_BankB_secErrorStatus()==1U)
    {
        ret_val = SDL_PASS;
        SDL_DSS_L3_BankB_secErrorClear();
    }
    else
    {
        ret_val = SDL_EFAIL;
    }

    return ret_val;
}
int32_t SDL_DSS_L3_BANKC_SEC_test(void)
{
    int32_t ret_val = SDL_EFAIL;
    uint32_t timeout = SDL_DSS_L3_ECC_TIMEOUT;
    SDL_DSS_L3_BANKC_secExecute();
    /* wait for error notification from ESM, or for timeout */
    /* fault injection gets deasserted in ISR */
    while((SDL_DSS_L3_BankC_secErrorStatus()!=1U) && (timeout!=0U));

    if(SDL_DSS_L3_BankC_secErrorStatus()==1U)
    {
        ret_val = SDL_PASS;
        SDL_DSS_L3_BankC_secErrorClear();
    }
    else
    {
        ret_val = SDL_EFAIL;
    }

    return ret_val;
}
int32_t SDL_DSS_L3_BANKD_SEC_test(void)
{
    int32_t ret_val = SDL_EFAIL;
    uint32_t timeout = SDL_DSS_L3_ECC_TIMEOUT;
    SDL_DSS_L3_BANKD_secExecute();
    /* wait for error notification from ESM, or for timeout */
    /* fault injection gets deasserted in ISR */
    while((SDL_DSS_L3_BankD_secErrorStatus()!=1U) && (timeout!=0U));


    if(SDL_DSS_L3_BankD_secErrorStatus()==1U)
    {
        ret_val = SDL_PASS;
        SDL_DSS_L3_BankD_secErrorClear();
    }
    else
    {
        ret_val = SDL_EFAIL;
    }

    return ret_val;
}

int32_t SDL_DSS_L3_BANKA_DED_test(void)
{
    int32_t ret_val = SDL_EFAIL;
    uint32_t timeout = SDL_DSS_L3_ECC_TIMEOUT;
    SDL_DSS_L3_BANKA_dedExecute();
    /* wait for error notification from ESM, or for timeout */
    /* fault injection gets deasserted in ISR */
    while((SDL_DSS_L3_BankA_dedErrorStatus()!=1U) && (timeout!=0U));

    if(SDL_DSS_L3_BankA_dedErrorStatus()==1U)
    {
        ret_val = SDL_PASS;
        SDL_DSS_L3_BankA_dedErrorClear();
    }
    else
    {
        ret_val = SDL_EFAIL;
    }
    return ret_val;
}

int32_t SDL_DSS_L3_BANKB_DED_test(void)
{
    int32_t ret_val = SDL_EFAIL;
    uint32_t timeout = SDL_DSS_L3_ECC_TIMEOUT;
    SDL_DSS_L3_BANKB_dedExecute();
    /* wait for error notification from ESM, or for timeout */
    /* fault injection gets deasserted in ISR */
    while((SDL_DSS_L3_BankB_dedErrorStatus()!=1U) && (timeout!=0U));

    if(SDL_DSS_L3_BankB_dedErrorStatus()==1U)
    {
        ret_val = SDL_PASS;
        SDL_DSS_L3_BankB_dedErrorClear();
    }
    else
    {
        ret_val = SDL_EFAIL;
    }
    return ret_val;
}

int32_t SDL_DSS_L3_BANKC_DED_test(void)
{
    int32_t ret_val = SDL_EFAIL;
    uint32_t timeout = SDL_DSS_L3_ECC_TIMEOUT;
    SDL_DSS_L3_BANKC_dedExecute();
    /* wait for error notification from ESM, or for timeout */
    /* fault injection gets deasserted in ISR */
    while((SDL_DSS_L3_BankC_dedErrorStatus()!=1U) && (timeout!=0U));

    if(SDL_DSS_L3_BankC_dedErrorStatus()==1U)
    {
        ret_val = SDL_PASS;
        SDL_DSS_L3_BankC_dedErrorClear();
    }
    else
    {
        ret_val = SDL_EFAIL;
    }
    return ret_val;
}

int32_t SDL_DSS_L3_BANKD_DED_test(void)
{
    int32_t ret_val = SDL_EFAIL;
    uint32_t timeout = SDL_DSS_L3_ECC_TIMEOUT;
    SDL_DSS_L3_BANKD_dedExecute();
    /* wait for error notification from ESM, or for timeout */
    /* fault injection gets deasserted in ISR */
    while((SDL_DSS_L3_BankD_dedErrorStatus()!=1U) && (timeout!=0U));

    if(SDL_DSS_L3_BankD_dedErrorStatus()==1U)
    {
        ret_val = SDL_PASS;
        SDL_DSS_L3_BankD_dedErrorClear();
    }
    else
    {
        ret_val = SDL_EFAIL;
    }
    return ret_val;
}


int32_t SDL_DSS_L3_BANKA_RED_test(void)
{
    int32_t ret_val = SDL_EFAIL;
    uint32_t timeout = SDL_DSS_L3_ECC_TIMEOUT;
    ret_val= SDL_DSS_L3_BANKA_redExecute(SDL_DSS_L3_FI_GLOBAL_SAFE, SDL_DSS_L3_MAIN_CMD_INTERFACE);
    if(ret_val == SDL_PASS )
    {
        /* Wait for test to complete/timeout. */
         while((SDL_DSS_L3_BankA_redErrorStatus()!=1U) && (timeout!=0U));
        /* Check for the failure. */
        if(SDL_DSS_L3_BankA_redErrorStatus()==1U)
        {
            ret_val = SDL_PASS;
            SDL_DSS_L3_BankA_redErrorClear();
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

int32_t SDL_DSS_L3_BANKB_RED_test(void)
{
    int32_t ret_val = SDL_EFAIL;
    uint32_t timeout = SDL_DSS_L3_ECC_TIMEOUT;
    ret_val= SDL_DSS_L3_BANKB_redExecute(SDL_DSS_L3_FI_GLOBAL_SAFE, SDL_DSS_L3_MAIN_CMD_INTERFACE);
    if(ret_val == SDL_PASS )
    {
        /* Wait for test to complete/timeout. */
        while((SDL_DSS_L3_BankB_redErrorStatus()!=1U) && (timeout!=0U));
        /* Check for the failure. */
        if(SDL_DSS_L3_BankB_redErrorStatus()==1U)
        {
            ret_val = SDL_PASS;
            SDL_DSS_L3_BankB_redErrorClear();
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

int32_t SDL_DSS_L3_BANKC_RED_test(void)
{
    int32_t ret_val = SDL_EFAIL;
    uint32_t timeout = SDL_DSS_L3_ECC_TIMEOUT;
    ret_val= SDL_DSS_L3_BANKC_redExecute(SDL_DSS_L3_FI_GLOBAL_SAFE, SDL_DSS_L3_MAIN_CMD_INTERFACE);
    if(ret_val == SDL_PASS )
    {
        /* Wait for test to complete/timeout. */
        while((SDL_DSS_L3_BankC_redErrorStatus()!=1U) && (timeout!=0U));
        /* Check for the failure. */
        if(SDL_DSS_L3_BankC_redErrorStatus()==1U)
        {
            ret_val = SDL_PASS;
            SDL_DSS_L3_BankC_redErrorClear();
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

int32_t SDL_DSS_L3_BANKD_RED_test(void)
{
    int32_t ret_val = SDL_EFAIL;
    uint32_t timeout = SDL_DSS_L3_ECC_TIMEOUT;
    ret_val= SDL_DSS_L3_BANKD_redExecute(SDL_DSS_L3_FI_GLOBAL_SAFE, SDL_DSS_L3_MAIN_CMD_INTERFACE);
    if(ret_val == SDL_PASS )
    {
        /* Wait for test to complete/timeout. */
        while((SDL_DSS_L3_BankD_redErrorStatus()!=1U) && (timeout!=0U));
        /* Check for the failure. */
        if(SDL_DSS_L3_BankD_redErrorStatus()==1U)
        {
            ret_val = SDL_PASS;
            SDL_DSS_L3_BankD_redErrorClear();
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



