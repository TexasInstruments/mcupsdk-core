# R5F_CPU_UTILS {#SDL_R5FCPU_PAGE}

[TOC]

The R5F CPU utils is having some Cortex R5F CORE functionality like reading the CPU Static Registers of ARM CORTEX R5F CORE.



## Features Supported

The system control coprocessor, CP15, controls and provides status information for the
functions implemented in the processor. The main functions of the system control coprocessor
are:
* overall system control and configuration
* cache configuration and management
* Memory Protection Unit (MPU) configuration and management
* system performance monitoring.

In this section, all the resisters are being read of the System control coprocessor.
* NOTE-> For detail description of these register, one can refer [ARM CORTEX R5F TRM](https://developer.arm.com/documentation/ddi0460/c/System-Control/Register-descriptions?lang=en).

## SysConfig Features

- None

## Features NOT Supported

- None

## Important Usage Guidelines

- None
## Example Usage

The following shows an example of SDL R5F CPU UTILS  API usage by the application.

Include the below file to access the APIs
\code{.c}
#include <sdl/r5/v0/sdl_r5_utils.h>
\endcode

For reading all the R5F CPU Static Register, the below API will be used.
\code{.c}

int32_t  sdlResult;
/* Declaration of Global structure to contain all register values*/
SDL_R5FCPU_StaticRegs  pCPUStaticRegs;

    /*Read all R5F cpu static registers*/
    sdlResult = SDL_CPU_staticRegisterRead(&pCPUStaticRegs);

After calling the above API, All register values can be found into the structure.


\endcode

## API

\ref SDL_R5FCPU_API