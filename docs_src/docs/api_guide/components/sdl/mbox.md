# MBOX {#SDL_MBOX_PAGE}

[TOC]

MBOX module consists ECC Bus Safety Diagnostics.

## Features Supported

MBOX supports Diagnostic check for ECC BUS Safety Errors

* ECC BUS Safety Errors : This includes SEC , DED and RED error injection on MSS MBOX , DSS MBOX and RSS MBOX

  Note : SEC - Single Error Correction, DED - Double Error Correction, RED - Redundancy Error Correction

The module supports below API's for the application

* API to support and induce error on  MBOX BUS.
* API to support interrupt configuration

## SysConfig Features

- None

## Features NOT Supported

- None

## Important Usage Guidelines

- None

## Example Usage

The following shows an example of SDL MBOX API usage by the application for Error Injection Tests.

Include the below file to access the APIs

\code{.c}
#include <sdl/sdl_mbox.h>
\endcode

Induce the error SEC in MSS MBOX
\code{.c}
    SDL_MSS_MBOX_secExecute();
    /* wait for error notification from ESM, or for timeout */
    /* fault injection gets deasserted in ISR */
    while((mboxSecFlag!=TRUE) && (timeout!=0U))
    {
        timeout--;
    }
    if(mboxSecFlag==TRUE)
    {
        mboxSecFlag=FALSE;
        ret_val = SDL_PASS;
    }
    else
    {
        ret_val = SDL_EFAIL;
    }
\endcode

Induce the error SEC in DSS MBOX
\code{.c}
    SDL_DSS_MBOX_secExecute();
    /* wait for error notification from ESM, or for timeout */
    /* fault injection gets deasserted in ISR */
    while((SDL_DSS_MBOX_secErrorStatus()!=1U) && (timeout!=0U))
    {
        timeout--;
    }
    if(SDL_DSS_MBOX_secErrorStatus()==1U)
    {
        SDL_DSS_MBOX_secErrorClear();
        ret_val = SDL_PASS;
    }
    else
    {
        ret_val = SDL_EFAIL;
    }
\endcode

Induce the error SEC in RSS MBOX
\code{.c}
    SDL_RSS_MBOX_secExecute();
    /* wait for error notification from ESM, or for timeout */
    /* fault injection gets deasserted in ISR */
    while((SDL_RSS_MBOX_secErrorStatus()!=1U) && (timeout!=0U))
    {
        timeout--;
    }
    if(SDL_RSS_MBOX_secErrorStatus()==1U)
    {
        SDL_RSS_MBOX_secErrorClear();
        ret_val = SDL_PASS;
    }
    else
    {
        ret_val = SDL_EFAIL;
    }
\endcode

Induce the error DED in MSS MBOX
\code{.c}
    SDL_MSS_MBOX_dedExecute();
    /* wait for error notification from ESM, or for timeout */
    /* fault injection gets deasserted in ISR */
    while((mssMboxDedRedFlag!=TRUE) && (timeout!=0U))
    {
        timeout--;
    }
    if(mssMboxDedRedFlag==TRUE)
    {
        mssMboxDedRedFlag=FALSE;
        ret_val = SDL_PASS;
    }
    else
    {
        ret_val = SDL_EFAIL;
    }
\endcode

Induce the error DED in DSS MBOX
\code{.c}
    SDL_DSS_MBOX_dedExecute();
    /* wait for error notification from ESM, or for timeout */
    /* fault injection gets deasserted in ISR */
    while((SDL_DSS_MBOX_dedErrorStatus()!=1U) && (timeout!=0U))
    {
        timeout--;
    }
    if(SDL_DSS_MBOX_dedErrorStatus()==1U)
    {
        SDL_DSS_MBOX_dedErrorClear();
        ret_val = SDL_PASS;
    }
    else
    {
        ret_val = SDL_EFAIL;
    }
\endcode

Induce the error DED in RSS MBOX
\code{.c}
    SDL_RSS_MBOX_dedExecute();
    /* wait for error notification from ESM, or for timeout */
    /* fault injection gets deasserted in ISR */
    while((SDL_RSS_MBOX_dedErrorStatus()!=1U) && (timeout!=0U))
    {
        timeout--;
    }
    if(SDL_RSS_MBOX_dedErrorStatus()==1U)
    {
        SDL_RSS_MBOX_dedErrorClear();
        ret_val = SDL_PASS;
    }
    else
    {
        ret_val = SDL_EFAIL;
    }
\endcode

Induce the error RED in MSS MBOX
\code{.c}
    ret_val= SDL_MSS_MBOX_redExecute(SDL_MBOX_FI_GLOBAL_SAFE, SDL_MBOX_MAIN_CMD_INTERFACE);
    if(ret_val == SDL_PASS )
    {
        /* Wait for test to complete/timeout. */
        while((mssMboxDedRedFlag!=TRUE) && (timeout!=0U))
        {
            timeout--;
        }
        /* Check for the failure. */
        if(mssMboxDedRedFlag==TRUE)
        {
            ret_val = SDL_PASS;
            mssMboxDedRedFlag =FALSE;
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

Induce the error RED in DSS MBOX
\code{.c}
    ret_val= SDL_DSS_MBOX_redExecute(SDL_MBOX_FI_GLOBAL_SAFE, SDL_MBOX_MAIN_CMD_INTERFACE);
    if(ret_val == SDL_PASS )
    {
        /* Wait for test to complete/timeout. */
        while((SDL_DSS_MBOX_redErrorStatus()!=1U) && (timeout!=0U))
        {
            timeout--;
        }
        /* Check for the failure. */
        if(SDL_DSS_MBOX_redErrorStatus()==1U)
        {
            ret_val = SDL_PASS;
            SDL_DSS_MBOX_redErrorClear();
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

Induce the error RED in RSS MBOX
\code{.c}
    ret_val= SDL_RSS_MBOX_redExecute(SDL_MBOX_FI_GLOBAL_SAFE, SDL_MBOX_MAIN_CMD_INTERFACE);
    if(ret_val == SDL_PASS )
    {
        /* Wait for test to complete/timeout. */
        while((SDL_RSS_MBOX_redErrorStatus()!=1U) && (timeout!=0U))
        {
            timeout--;
        }
        /* Check for the failure. */
        if(SDL_RSS_MBOX_redErrorStatus()==1U)
        {
            ret_val = SDL_PASS;
            SDL_RSS_MBOX_redErrorClear();
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

\ref SDL_MBOX_MODULE
