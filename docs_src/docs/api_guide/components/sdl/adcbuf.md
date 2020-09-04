# ADCBUF {#SDL_ADCBUF_PAGE}

[TOC]

ADCBUF module consists ECC Bus Safety Diagnostics.

## Features Supported

ADCBUF supports Diagnostic check for ECC BUS Safety Errors

* BUS Safety Errors : This incluedes SEC , DED and RED error injection on ADCBUF WR  DMA0 and RED error injection on ADCBUF RD

The module supports below API's for the application

* API to support and induce error on  ADCBUF BUS.
* API to support interrupt configuration

## SysConfig Features

- None

## Features NOT Supported

- None

## Important Usage Guidelines

- None

## Example Usage

The following shows an example of SDL ADCBUF API usage by the application for Error Injection Tests.

Include the below file to access the APIs
\code{.c}
#include <sdl/sdl_adcbuf.h>
\endcode

Induce the error SEC in ADCBUF WR
\code{.c}
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

    return ret_val;;
\endcode

Induce the error DED in ADCBUF WR
\code{.c}
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
\endcode

Induce the error RED in ADCBUF WR
\code{.c}
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
\endcode

Induce the error RED in ADCBUF RD
\code{.c}
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
\endcode

## API

\ref SDL_ADCBUF_MODULE
