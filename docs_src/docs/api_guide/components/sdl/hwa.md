# HWA {#SDL_HWA_PAGE}

[TOC]

HWA module consists of Parity and FSM Lockstep Diagnostics

## Features Supported

HWA supports Diagnostic check for Parity Errors on HWA Data memoriers and window ram on both HWA DMA0 and HWA DMA1.

* Parity Errors : This incluedes parity error injection on both HWA DMA0 and DMA1.
* FSM LOCKstep Errors : This incluedes fsm lockstep error injection HWA.


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


## API

\ref SDL_HWA_MODULE
