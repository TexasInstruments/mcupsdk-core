# DSS_L3 {#SDL_DSS_L3_PAGE}

[TOC]

# Features Supported

DSS L3 module consists ECC Bus Safety Diagnostics.

* BUS Safety Errors : This includes SEC , DED and RED error injection on DSS L3   BANK A, BANK B, BANK C and BANK D

  Note : SEC - Single Error Correction, DED - Double Error Correction, RED - Redundancy Error Correction

The module supports below API's for the application

* API to support and induce error on  DSS L3 BUS.
* API to support interrupt configuration

## SysConfig Features

- None

## Features NOT Supported

- None

## Important Usage Guidelines

- None

## Example Usage

The following shows an example of SDL DSS L3 API usage by the application for Error Injection Tests.

Include the below file to access the APIs

\code{.c}
#include <sdl/sdl_adcbuf.h>
\endcode

Induce the error SEC in DSS L3 Bank A
\code{.c}
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

\endcode

Induce the error DED in DSS L3 Bank A
\code{.c}
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
\endcode

Induce the error RED in DSS L3 Bank A
\code{.c}
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
\endcode

Induce the error SEC in DSS L3 Bank B
\code{.c}
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
\endcode

Induce the error DED in DSS L3 Bank B
\code{.c}
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
\endcode

Induce the error RED in DSS L3 Bank B
\code{.c}
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
\endcode

Induce the error SEC in DSS L3 Bank C
\code{.c}
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
\endcode

Induce the error DED in DSS L3 Bank C
\code{.c}
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
\endcode

Induce the error RED in DSS L3 Bank C
\code{.c}
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
\endcode

Induce the error SEC in DSS L3 Bank D
\code{.c}
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
\endcode

Induce the error DED in DSS L3 Bank D
\code{.c}
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
\endcode

Induce the error RED in DSS L3 Bank D
\code{.c}
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
\endcode


## API

\ref SDL_DSS_L3_MODULE
