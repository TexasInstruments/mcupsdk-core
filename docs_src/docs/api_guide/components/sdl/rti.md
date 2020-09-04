# RTI {#SDL_RTI_PAGE}

[TOC]
The Windowed Watchdog Timer (WWDT) generates reset after a programmable period, if not serviced within that period. This time-out boundary is configurable, and the windowed feature allows the start time boundary to be configurable. The WWDT can generate an Interrupt, if not serviced within window (Open Window) defined by start time and time-out boundary. Also the WWDT can generate an Interrupt if serviced outside Open Window (within Closed Window). Generation of Interrupt depends on the WWDT Reaction configuration. SDL supports configuration of the watchdog timers. It also supports notification of the error via ESM interrupt. Additionally, APIs for checking the status of the watchdog timer is provided.

## Features Supported
The RTI modules include the following main features:
* Ability to to initialize the RTI -DWWD module.
* Ability to configure RTI -DWWD module.
* Ability to service an RTI instance.
* Ability to read Status of the configuration.
* Ability to read back static register.

\cond SOC_AM263X
There are Four RTI Timer instances, implemented by the Real-time Interrupt function of the RTI/WWDT module.
Four Windowed Watchdog Timer (WWDT) instances, implemented by the Digital Windowed Watchdog
(DWWD) function of the RTI/WWDT module

The WWDT instances are intended to function as a digital windowed watchdog for the CPU core that they are
associated with:
* WWDT0 is dedicated to the first R5F CPU core (R5FSS0_CORE0)
* WWDT1 is dedicated to the second R5F CPU core (R5FSS0_CORE1)
* WWDT2 is dedicated to the third R5F CPU core (R5FSS1_CORE0)
* WWDT3 is dedicated to the fourth R5F CPU core (R5FSS1_CORE1)
\endcond

\cond SOC_AM273X || SOC_AWR294X
There are Five Real Time Interrupts (RTI) modules, three in MSS and two in DSS.
Two Watchdog modules, one in DSS and one in MSS (Same RTI IP but in Watchdog configuration)

The WWDT instances are intended to function as a digital windowed watchdog for the CPU core that they are
associated with:

* MSS_WDT is dedicated to the first R5F CPU core (R5FSS0_CORE0)
* DSS WDT is dedicated to C66 DSP core (C66SS0)
\endcond

All WWDT instances that are provisioned for a particular CPU core should not be used by any other CPU cores.

## SysConfig Features
- None

## Features NOT Supported
- None

## Important Usage Guidelines
- None

## Example Usage

The following shows an example of SDL RTI API.

Include the below file to access the APIs

\code{.c}
#include <sdl/sdl_rti.h>
\endcode

\cond SOC_AM263X
Config an RTI Instance
\code{.c}
SDL_RTI_configParms pConfig;

/* Configure RTI parameters for preload, window and reaction*/
pConfig.SDL_RTI_dwwdPreloadVal = RTIGetPreloadValue(RTI_CLOCK_SOURCE_200KHZ, RTI_WDT_TIMEOUT);
pConfig.SDL_RTI_dwwdWindowSize = RTI_DWWD_WINDOWSIZE_100_PERCENT;
pConfig.SDL_RTI_dwwdReaction   = RTI_DWWD_REACTION_GENERATE_NMI;

retVal = SDL_RTI_config(SDL_WDT0_U_BASE, &pConfig);

if (retVal == SDL_EFAIL)
{
    UART_printf("Error during Window configuration.\n");
}
\endcode

Verify the config
\code{.c}
/* Verify the config */
retVal = SDL_RTI_verifyConfig(SDL_WDT0_U_BASE, &pConfig);

if (retVal == SDL_EFAIL)
{
    UART_printf("Error during Window Verify configuration.\n");
}
\endcode

Read the static registers
\code{.c}
SDL_RTI_staticRegs pStaticRegs;

retVal = SDL_RTI_readStaticRegs(SDL_WDT0_U_BASE, &pStaticRegs);
\endcode

Start an RTI Instance
\code{.c}
SDL_RTI_start(SDL_WDT0_U_BASE);
\endcode

Start an RTI Instance
\code{.c}
/* Servicing of the watchdog is done by the core that is being monitored with the watchdg */
SDL_RTI_service(SDL_WDT0_U_BASE);
\endcode
\endcond

\cond SOC_AM273X || SOC_AWR294X
Config an RTI Instance
\code{.c}
SDL_RTI_configParms pConfig;

/* Configure RTI parameters for preload, window and reaction*/
pConfig.SDL_RTI_dwwdPreloadVal = RTIGetPreloadValue(RTI_CLOCK_SOURCE_200KHZ, RTI_WDT_TIMEOUT);
pConfig.SDL_RTI_dwwdWindowSize = RTI_DWWD_WINDOWSIZE_100_PERCENT;
pConfig.SDL_RTI_dwwdReaction   = RTI_DWWD_REACTION_GENERATE_NMI;

retVal = SDL_RTI_config(SDL_INSTANCE_MSS_WDT, &pConfig);

if (retVal == SDL_EFAIL)
{
    UART_printf("Error during Window configuration.\n");
}
\endcode

Verify the config
\code{.c}
/* Verify the config */
retVal = SDL_RTI_verifyConfig(SDL_INSTANCE_MSS_WDT, &pConfig);

if (retVal == SDL_EFAIL)
{
    UART_printf("Error during Window Verify configuration.\n");
}
\endcode

Read the static registers
\code{.c}
SDL_RTI_staticRegs pStaticRegs;

retVal = SDL_RTI_readStaticRegs(SDL_INSTANCE_MSS_WDT, &pStaticRegs);
\endcode

Start an RTI Instance
\code{.c}
SDL_RTI_start(SDL_INSTANCE_MSS_WDT);
\endcode

Start an RTI Instance
\code{.c}
/* Servicing of the watchdog is done by the core that is being monitored with the watchdg */
SDL_RTI_service(SDL_INSTANCE_MSS_WDT);
\endcode
\endcond
## API

\ref SDL_RTI_API