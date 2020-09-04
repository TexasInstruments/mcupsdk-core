# HWA {#SDL_HWA_PAGE}

[TOC]

HWA module consists of ECC Bus Safety Diagnostics, Parity and FSM Lockstep Diagnostics

## Features Supported

1.HWA supports Diagnostic check for ECC BUS Safety Errors on HWA DMA0 and HWA DMA1.

* ECC BUS Safety Errors : This includes SEC , DED and RED error injection on both HWA DMA0 and DMA1

  Note : SEC - Single Error Correction, DED - Double Error Correction, RED - Redundancy Error Correction

2.HWA supports Diagnostic check for Parity Errors on HWA Data memoriers and window ram on both HWA DMA0 and HWA DMA1.

* Parity Errors : This incluedes parity error injection on both HWA DMA0 and DMA1.
* FSM LOCKstep Errors : This incluedes fsm lockstep error injection HWA.

The module supports below API's for the application

* API to support and induce error on  HWA DMA BUS.
* API to support interrupt configuration

## SysConfig Features

- None

## Features NOT Supported

- None

## Important Usage Guidelines

- None

## Example Usage

The following shows an example of SDL HWA API usage by the application for Error Injection Tests.

Include the below file to access the APIs

\code{.c}
#include <sdl/sdl_hwa.h>
\endcode
Induce the error DMA0 DMEM0
\code{.c}
    return (SDL_HWA_memParityExecute(SDL_HWA_DMA0_MEM_ID , SDL_HWA_DMEM0));
\endcode

Induce the error DMA0 DMEM1
\code{.c}
    return (SDL_HWA_memParityExecute(SDL_HWA_DMA0_MEM_ID , SDL_HWA_DMEM1));
\endcode

Induce the error DMA0 DMEM2
\code{.c}
    return (SDL_HWA_memParityExecute(SDL_HWA_DMA0_MEM_ID , SDL_HWA_DMEM2));
\endcode

Induce the error DMA0 DMEM3
\code{.c}
    return (SDL_HWA_memParityExecute(SDL_HWA_DMA0_MEM_ID , SDL_HWA_DMEM3));
\endcode

Induce the error DMA0 DMEM4
\code{.c}
    return (SDL_HWA_memParityExecute(SDL_HWA_DMA0_MEM_ID , SDL_HWA_DMEM4));
\endcode

Induce the error DMA0 DMEM5
\code{.c}
    return (SDL_HWA_memParityExecute(SDL_HWA_DMA0_MEM_ID , SDL_HWA_DMEM5));
\endcode

Induce the error DMA0 DMEM6
\code{.c}
    return (SDL_HWA_memParityExecute(SDL_HWA_DMA0_MEM_ID , SDL_HWA_DMEM6));
\endcode

Induce the error DMA0 DMEM7
\code{.c}
    return (SDL_HWA_memParityExecute(SDL_HWA_DMA0_MEM_ID , SDL_HWA_DMEM7));
\endcode

Induce the error DMA0 WINDOW_RAM
\code{.c}
    return (SDL_HWA_memParityExecute(SDL_HWA_WINDOW_RAM_MEM_ID , SDL_HWA_WINDOW_RAM));
\endcode

Induce the error DMA1 DMEM0
\code{.c}
    return (SDL_HWA_memParityExecute(SDL_HWA_DMA1_MEM_ID , SDL_HWA_DMEM0));
\endcode

Induce the error DMA1 DMEM1
\code{.c}
    return (SDL_HWA_memParityExecute(SDL_HWA_DMA1_MEM_ID , SDL_HWA_DMEM1));
\endcode

Induce the error DMA1 DMEM2
\code{.c}
    return (SDL_HWA_memParityExecute(SDL_HWA_DMA1_MEM_ID , SDL_HWA_DMEM2));
\endcode

Induce the error DMA1 DMEM3
\code{.c}
    return (SDL_HWA_memParityExecute(SDL_HWA_DMA1_MEM_ID , SDL_HWA_DMEM3));
\endcode

Induce the error DMA1 DMEM4
\code{.c}
    return (SDL_HWA_memParityExecute(SDL_HWA_DMA1_MEM_ID , SDL_HWA_DMEM4));
\endcode

Induce the error DMA1 DMEM5
\code{.c}
    return (SDL_HWA_memParityExecute(SDL_HWA_DMA1_MEM_ID , SDL_HWA_DMEM5));
\endcode

Induce the error DMA1 DMEM6
\code{.c}
    return (SDL_HWA_memParityExecute(SDL_HWA_DMA1_MEM_ID , SDL_HWA_DMEM6));
\endcode

Induce the error DMA1 DMEM7
\code{.c}
    return (SDL_HWA_memParityExecute(SDL_HWA_DMA1_MEM_ID , SDL_HWA_DMEM7));
\endcode

Induce the error DMA1 WINDOW_RAM
\code{.c}
    return (SDL_HWA_memParityExecute(SDL_HWA_WINDOW_RAM_MEM_ID , SDL_HWA_WINDOW_RAM));
\endcode

Induce the error FSM Lockstep
\code{.c}
    return (SDL_HWA_fsmLockStepExecute());
\endcode

Induce the error SEC in HWA DMA0
\code{.c}
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
\endcode

Induce the error DED in HWA DMA0
\code{.c}
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
\endcode

Induce the error RED in HWA DMA0
\code{.c}
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
\endcode


Induce the error SEC in HWA DMA1
\code{.c}
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
\endcode

Induce the error DED in HWA DMA1
\code{.c}
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
\endcode

Induce the error RED in HWA DMA1
\code{.c}
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
\endcode

## API

\ref SDL_HWA_MODULE
