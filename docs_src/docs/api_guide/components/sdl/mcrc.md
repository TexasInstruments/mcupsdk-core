# MCRC {#SDL_MCRC_PAGE}

[TOC]

MCRC module is used to perform CRC (Cyclic Redundancy check) to verify integrity of a memory system. The MCRC controller is used to calculate the signature for a set of data and then compare the calculated signature value against a predetermined good signature value. MCRC controller provides four channels to perform CRC calculation on multiple memories.

## Features Supported

Each MCRC instance supports 4 channels. Each channel can be configured to operate in one of 3 modes

* Auto Mode: In this mode, the MCRC controller, in conjunction with DMA controller, can perform CRC totally without CPU intervention.
* Full Mode: In this mode, the CPU does the data patterns transfer and signature verification all by itself. CPU performs data patterns transfer by reading data from the memory system to the PSA signature register.
* Semi Mode: In this mode, DMA controller is used to perform data patterns transfer and signature verification.

Timeout is configured to generate timeout interrupt to ensure that the memory system is examined with predefined time frame and no loss of incoming data. A 24-bit counter per CRC channel is configured.

The module supports below API's for the application

* API to support and configure Full, Semi, and Auto MCRC mode
* API to support readback of static registers
* API to support readback of written configuration
* API to support interrupt configuration

## SysConfig Features

- None

## Features NOT Supported

- No DMA mode examples for M4

## Important Usage Guidelines

- None

## Example Usage

The following shows an example of SDL MCRC API usage by the application for Full CPU mode.

Include the below file to access the APIs

\code{.c}
#include <sdl/sdl_mcrc.h>
\endcode

Initialize an MCRC instance and channel

\cond SOC_AM64X || SOC_AM243X
\code{.c}
SDL_MCRC_InstType instance = MCU_MCRC64_0;
\endcode
\endcond

\cond SOC_AM263X || SOC_AM263PX || SOC_AM261X
\code{.c}
SDL_MCRC_InstType instance = MCRC0;
\endcode
\endcond

\cond SOC_AWR294X || SOC_AM273X
\code{.c}
SDL_MCRC_InstType instance = DSS_MCRC;
\endcode
\endcond

\code{.c}
result = SDL_MCRC_init(instance, SDL_MCRC_CHANNEL_1, MCRC_DEF_WATCHDOG_PRELOAD, MCRC_DEF_BLOCK_PRELOAD);
\endcode

Reset and configure the MCRC channel
\code{.c}
result = SDL_MCRC_channelReset(instance, SDL_MCRC_CHANNEL_1);
SDL_MCRC_config(instance, SDL_MCRC_CHANNEL_1, MCRC_DEF_PATTERN_COUNT, MCRC_DEF_SECTOR_COUNT, SDL_MCRC_OPERATION_MODE_FULLCPU);
\endcode

Verify the configuration
\code{.c}
result = SDL_MCRC_verifyConfig(instance, SDL_MCRC_CHANNEL_1, MCRC_DEF_PATTERN_COUNT, MCRC_DEF_SECTOR_COUNT, SDL_MCRC_OPERATION_MODE_FULLCPU);
if (result == SDL_PASS)
{
    DebugP_log("\n Configurationn verified");
}
\endcode

Compute the signature
\code{.c}
SDL_MCRC_dataConfig_t mcrcData;
        SDL_MCRC_Signature_t  sectSignVal;

        mcrcData.pMCRCData      = (uint32_t *)testParams[testCase].sourceMemory;
        mcrcData.size           = testParams[testCase].dataSize;
        mcrcData.dataBitSize    = SDL_MCRC_DATA_32_BIT;

        /* Pass the data to MCRC and get the signature */
        result = SDL_MCRC_computeSignCPUmode(instance, SDL_MCRC_CHANNEL_1, &mcrcData, &sectSignVal);
\endcode

Check the calculated signature
\code{.c}
/*
 * Check if the generated MCRC signature value
 * matches with the reference signaure vaule
 */
if ((sectSignVal.regH == testParams[testCase].mcrcSignHigh) &&
    (sectSignVal.regL == testParams[testCase].mcrcSignLow))
{
    result = SDL_PASS;
}
else {
    result = SDL_EFAIL;
}
\endcode

## API

\ref SDL_MCRC_MODULE
