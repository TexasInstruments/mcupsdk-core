# STOG {#SDL_STOG_PAGE}

[TOC]

The Interconnect Isolation Gasket is a TI proprietary IP that is used to prevent a hang caused by target, which can hold up the interconnect and terminate such a transaction safely. It tracks transactions, and times out if outstanding too long, and reports the error via interrupt that can be monitored.

The TOGs help to achieve Freedom From Interference by monitoring the various VBUS transactions and providing a way to detect any errors, and helping to avoid the bus to be locked up due to a transaction error.


## Features Supported

These TOGs have the following functions, to avoid these concerns:

1. Monitors various VBUS transaction and provide a way to detect errors
2. Helps avoid the bus to be locked up due to a transaction in error.
3. Tracks outstanding transactions and allows for timeout/recovery.

Each of the Timeout gaskets can be programmed to a specific timeout and any transaction exceeding the timeout will result in abort of the transaction. This will also result in an error event triggered through ESM, which will in turn can be programmed to interrupt the CPU.

 In addition the timeout gasket can be stopped, started and reset at anytime.

 Safety diagnostics are provided for TOG module through API’s

1. Configure the TOG
2. Read the static registers
3. Verify the written configuration


## SysConfig Features

- None

## Features NOT Supported

- None

## Important Usage Guidelines

- None

## Example Usage

The following shows an example of SDL TOG API usage by the application to set up the TOG for monitoring for events. Events can be monitored by enabling the events in the associated ESM instance.

\code{.c}
#include <sdl/sdlr_tog.h>
\endcode

Configure a TOG instance with a timeout value

\code{.c}

SDL_TOG_Inst instance;
SDL_TOG_config cfg;
int32_t status;

cfg.timeoutVal = TOG_TEST_TIMEOUTVAL;

status = SDL_TOG_init(instance, &cfg);
if (status != SDL_PASS)
{
    // init failed
}
\endcode

Enable the TOG interrupts

\code{.c}
SDL_TOG_Inst instance;
int32_t status;

/* Enable interrupts */
status = SDL_TOG_setIntrEnable(instance, SDL_TOG_INTRSRC_ALL, true);
if (status != SDL_PASS)
{
    // interrupt enable failed
}
\endcode

Start the TOG

\code{.c}
/* Call SDL API to enable Timeout Gasket */
status = SDL_TOG_start(instance);
if (status != SDL_PASS)
{
    // start failed
}
\endcode

Stop the TOG

\code{.c}
SDL_TOG_Inst instance;

SDL_TOG_stop(instance);
\endcode

Reset the TOG

\code{.c}
/* Reset the Timeout gasket */
SDL_TOG_reset( instance );
\endcode

Get the Error Information (can be called when an error occurs)

\code{.c}
SDL_TOG_Inst instance;
SDL_TOG_errInfo errInfo;

/* Read error info */
status = SDL_TOG_getErrInfo(instance, &errInfo)
\endcode

Acknowledge the interrupts for an interrupt source

\code{.c}
SDL_TOG_Inst instance;
uint32_t intCount;

/* Get Transaction timeout interrupt count */
status = SDL_TOG_getIntrCount(instance, SDL_TOG_INTRSRC_TRANSACTION_TIMEOUT, &intCount);

/* Clear Transaction timeout interrupt events */
if ((status == SDL_PASS) && (intCount != 0))
{
    status = SDL_TOG_ackIntr(instance, SDL_TOG_INTRSRC_TRANSACTION_TIMEOUT, intCount);
    TEST_ASSERT_EQUAL_INT32(SDL_PASS, status);
}
\endcode

Clear the pending interrupts for an interrupt source

\code{.c}
SDL_TOG_Inst instance;

status = SDL_TOG_clrIntrPending(instance, SDL_TOG_INTRSRC_TRANSACTION_TIMEOUT);
\endcode

## API

\ref SDL_STOG_API
