# TMU_ROM_CHECKSUM {#SDL_TMU_ROM_CHECKSUM_PAGE}

[TOC]

TMU ROM Checksum is a feature that is used to check the integrity of the TMU ROM data. Its work is to take a set of data associated with the memory regions of TMU ROM and perform checksum on that data and then compare that resultant data value against a pre-determined golden vector value.



## Features Supported

This module provides the following functionality:

* Ability to select the MCRC channel to calculate checksum
* Ability to compare resultant checksum data with golden value

## SysConfig Features

- None

## Features NOT Supported

- None

## Important Usage Guidelines

- None
## Example Usage

The following shows an example of SDL TMU ROM CHECKSUM API usage by the application to check the integrity of data at ROM region.

Include the below file to access the APIs

\code{.c}
#include <sdl/sdl_tmu_rom_checksum.h>
\endcode

Use below API For calculating the TMU_ROM_CHECKSUM.
\code{.c}

int32_t  test_Result;

SDL_MCRC_Signature_t  sectSignVal;

testResult = SDL_TMU_ROM_Checksum_Compute(SDL_MCRC_CHANNEL_1, &sectSignVal);

The above API will return SDL_PASS, if the calculated CRC matches with the golden CRC.

\endcode

## API

\ref SDL_TMU_ROM_CHECKSUM_API