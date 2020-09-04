# STC {#SDL_STC_PAGE}

[TOC]

The Self-Test Controller (STC) is used to test logic cores based on the On-Product Multiple Input Signature Register (OPMISR) scan compression architecture.The self test controller facilitates complete isolation of the logical segment under test from rest of the system during the self-test run.

The STC have some internal configuration.Gernally The configuration need not to be changed for running STC. For running STC The core should be in low power mode for which STC is about to run.

## Features Supported

The STC module provides the following functionality:

* Ability to configure a STC instance
* Implement OPMISR controller along with On-chip Self-Test controller
for the synthesizable module logic which enables to achieve high test
coverage.
* Ability to divide the complete test run into independent test sets
(intervals).
* Capable of running the complete test as well as running few intervals at
a time.
* Ability to continue from the last executed interval(test set) as well as
ability to restart from the beginning(1st interval in the ROM) or start
from the 1st interval of each segment.

Errors detected by the STC module during operation are reported via ESM error (application will receive via ESM application callback).

## SysConfig Features

- None

## Features NOT Supported

- None

## Important Usage Guidelines

- Once STC run is Done, Core will be reset and STC Reset cause will also be set. For again running STC, SOC will need power cycle.

## Example Usage

The following shows an example of SDL STC API usage by the application.

Include the below file to access the APIs

\code{.c}
#include <sdl/sdl_stc.h>
\endcode


\code{.c}
First of all, Need to check that STC has been Done or not, for this the below API will be used-

SDL_STC_testResult test_Result;

test_Result= SDL_STC_getStatus(SDL_STC_Inst instance);

According to return result of this API, Return value type is ENUM as below
typedef enum
{
    /** The STC completed and the test passed  */
    SDL_STC_COMPLETED_SUCCESS,
    /** The STC completed and the test failed  */
    SDL_STC_COMPLETED_FAILURE,
    /**The STC was active but could not be completed.  */
     SDL_STC_NOT_COMPLETED,
    /** The STC was not performed on this device  */
     SDL_STC_NOT_RUN,
    /** The STC was not performed on this device  */
    INVALID_RESULT

}SDL_STC_testResult;

if return value is (SDL_STC_NOT_RUN), Then STC need to run, we should call below API.


\endcode

\code
For running STC, Need to Provide two inputs.
1. Instance for which, STC need to be run.
2. testType(Positive or negative)

typedef enum
{
    /** The STC test should be  completed and the test passed for this testType  */
     SDL_STC_TEST,
    /** The STC test should be completed and the test failed for this testType */
     SDL_STC_NEG_TEST

}SDL_STC_testType;

int32_t sdlResult=SDL_PAAS;

sdlResult=   SDL_STC_selfTest(SDL_STC_Inst instance, SDL_STC_testType testType);

if STC will be Done Successfully, it will return nothing, beacuse core reset would have been done till now.
if something will be going wrong, it will return something (!=SDL_PASS) value.

\endcode


## API

\ref SDL_STC_API
