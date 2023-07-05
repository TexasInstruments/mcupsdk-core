# ROM_CHECKSUM {#SDL_ROM_CHECKSUM_PAGE}

[TOC]

ROM Checksum is a feature that is used to check the integrity of the data. Its work is to take a set of data associated with the memory regions of ROM and perform checksum on that data and then compare that resultant data value against a pre-determined golden vector value (golden vector has the expected value which should come as a result of 512-bit of hash message, golden vector is already defined and it has fixed address in ROM region).



## Features Supported

This module provides the following functionality:

* Ability initialize the ROM Checksum
* Ability to process the data of ROM region
* Ability to compress the data of ROM region and store the result in buffer
* Ability to compare resultant data with golden value (golden value has the expected value which should come as a result of 512-bit of hash message, golden value is already defined and it has fixed address in ROM region)

## SysConfig Features

- None

## Features NOT Supported

- None

## Important Usage Guidelines

- None
## Example Usage

The following shows an example of SDL ROM CHECKSUM API usage by the application to check the integrity of data at ROM region.

Include the below file to access the APIs

\code{.c}
#include <sdl/sdl_rom_checksum.h>
\endcode

Use below API For calculating the ROM_CHECKSUM.
\code{.c}

int32_t  test_Result;

testResult = SDL_ROM_Checksum_compute();

The above API will return SDL_PASS, if the API will calculate rom checksum successfully.

\endcode

## API

\ref SDL_ROM_CHECKSUM_API