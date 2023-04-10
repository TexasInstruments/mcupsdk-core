# MTOG {#SDL_MTOG_PAGE}

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

Include the below file to access the APIs
\code{.c}
#include <sdl/sdlr_mtog.h>
\endcode

\cond SOC_AM64X || SOC_AM243X
\code{.c}
Initialization structure for MCU ESM instance
static SDL_ESM_config MTOG_Example_esmInitConfig_MCU =
{
    .esmErrorConfig = {0u, 3u}, /* Self test error config */
    .enableBitmap = {0x04000000u, 0x00000000u, 0x00000000u, 0x00000000u,
                },
     /**< All events enable: except timer and self test  events, */
    /*    and Main ESM output.Configured based off esmErrorConfig to test high or low priorty events.*/
    .priorityBitmap = {0x04000000u, 0x00000000u, 0x00000000u, 0x00000000u,
                },
    /**< Configured based off esmErrorConfig to test high or low priorty events. */
    .errorpinBitmap = {0x04000000u, 0x00000000u, 0x00000000u, 0x00000000u,
                },
};
\endcode

Event handler API
\code{.c}
void MTOG_eventHandler( uint32_t instanceIndex )
{
    int32_t status = SDL_PASS;

    /* Reset the Timeout gasket */
    status = SDL_MTOG_reset( instanceIndex );

    if (status == SDL_PASS)
    {
        DebugP_log("\n MTOG Reset done\n");
    }
    else{
        DebugP_log("\n MTOG Reset failed");
    }
    doneFlag = true;
    return;
}
\endcode

ESM callback function.
\code{.c}
int32_t SDL_ESM_applicationCallbackFunction(SDL_ESM_Inst esmInst, SDL_ESM_IntType esmIntrType,
                                            uint32_t grpChannel,  uint32_t index, uint32_t intSrc, void *arg)
{
    int32_t retVal = SDL_PASS;
    MTOG_eventHandler(instanceIndex);
    DebugP_log("\nInterrupt is generated to ESM\n");
    DebugP_log("    ESM Call back function called : instType 0x%x, intType 0x%x, " \
                "grpChannel 0x%x, index 0x%x, intSrc 0x%x \n",
                esmInst, esmIntrType, grpChannel, index, intSrc);
    DebugP_log("  Take action \n");

    /* For clearing the interrupt */
    IntrDisable(intSrc);

    return retVal;
}
\endcode

Initialize ESM
\code{.c}
SDL_ESM_init(SDL_ESM_INST_MCU_ESM0, &MTOG_Example_esmInitConfig_MCU, SDL_ESM_applicationCallbackFunction, &apparg);
\endcode

Configure a TOG instance with a timeout value
\code{.c}
uint32_t instance;
SDL_MTOG_config config;
int32_t status=0;
uint32_t ESMEventNumber;

ESMEventNumber	   = SDLR_MCU_ESM0_ESM_LVL_EVENT_MCU_MASTER_SAFETY_GASKET0_TIMED_OUT_0;
config.timeOut 	   = SDL_MTOG_VAL_1K;
instanceIndex 	   = SDL_INSTANCE_MCU_MTOG0;

result = SDL_MTOG_init(instanceIndex, &config);
if (status != SDL_PASS)
{
    DebugP_log("   SDL_MTOG_init Failed \n");
    result = -1;
}
\endcode

Call SDL API to enable Timeout Gasket
\code{.c}
status = SDL_MTOG_start(instanceIndex);
if (status != SDL_PASS)
{
    DebugP_log("   SDL_MTOG_start Failed \n");
    result = -1;
}
\endcode

Inject master timeout error
\code{.c}
status = SDL_MTOG_forceTimeout(instanceIndex);
if (status != SDL_PASS)
{
    DebugP_log("\n SDL_MTOG_forceTimeout Failed \n");
    result = -1;
}
\endcode

Wait for MTOG Interrupt
\code{.c}
/* Timeout if exceeds time */
while ((!doneFlag)
        && (timeoutCount++ < MTOG_MAX_TIMEOUT_VALUE))
{
	/* Use Polling */
    MTOG_eventHandler(instanceIndex);
}
if(timeoutCount >= MTOG_MAX_TIMEOUT_VALUE)
{
    DebugP_log("\n MTOG Timed out  \n");
    result = -1;
}
\endcode

Disable ESM Interrupts
\code{.c}
status=SDL_ESM_disableIntr(SDL_ESM_INST_MCU_ESM0, ESMEventNumber);
if (status != SDL_PASS)
{
    DebugP_log("   sdlAppEsmDisable Failed \n");
    result = -1;
}
\endcode
\endcond

## API

\ref SDL_MTOG_MODULE