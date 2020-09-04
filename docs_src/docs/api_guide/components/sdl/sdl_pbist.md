# PBIST {#SDL_PBIST_PAGE}

[TOC]

Built-in Self-test (BIST) is a feature that allows self testing of the memory areas and logic circuitry in an Integrated Circuit (IC) without any external test equipment. In an embedded system, these tests are typically used during boot time or shutdown of the system to check the health of an SoC.

PBIST is used to test the memory regions in the SoC and provides detection for permanent faults. The primary use case for PBIST is when it is invoked at start-up providing valuable information on any stuck-at bits in the memory.

There can be multiple instances of PBIST in the SoC, and each has a number of memory regions associated with it. SDL provides support for PBIST features such as execution of PBIST test-for-diagnostic to test the PBIST logic and execution of PBIST. The same API is used with different configuration parameters to execute each instance. Checking of the status of HW POST PBIST execution is also supported.

Some things to note:

* PBIST is expected to be run at boot-time or once per drive cycle.
* PBIST must be run from a different core than is being tested. This is because the test is destructive in nature. For this reason also, after BIST test it is necessary to reset the module.
* If SW-initiated PBIST is executed after the IP under test is already in use in the system, it is the application's responsibility to perform any necessary context save/restore necessary for the core/IP.

## Features Supported

The PBIST Module of the SDL supports execution of the software-initiated PBIST for the various supported instances. It provides the following services:

* Execution of the PBIST test for a specified instance
* Checking of the PBIST results
* Restore core to system control (PBIST reset and release test mode)
* Return the status of the test

## SysConfig Features

- None

## Features NOT Supported

- None

## Important Usage Guidelines

- Execution of the PBIST tests requires preparation of the IPs under test by bringing them to a certain power and reset state before executing the test. It will be required that the application bring the cores/IPs to the proper state before executing the PBIST. Additionally, there is an "exit sequence" that is required to bring the cores/IPs back to the system control after the PBIST test is executed. This will also be the responsibility of the application. The PBIST examples provided with SDL will give the necessary sequences, which can be used by the application for implementing the sequence.
\cond SOC_AM64X
- The PBIST module uses ESM events to detect completion of the test. The application should not enable these events through SDL_ESM_init in order to prevent interference with the test execution.
\endcond

## Example Usage

The following shows an example of SDL PBIST API usage by the application to execute the PBIST test and test-for-diagnostic.

Include the below file to access the APIs

\code{.c}
#include <sdl/sdl_pbist.h>
\endcode

\cond SOC_AM64X
Note: Do not initialize the ESM to detect PBIST completion events
\endcond

Before executing the following PBIST tests, the IP under test must be brought to a certain power and reset state. The included PBIST example shows the sequence needed for each of the PBIST instances.

Once the core(s) are brought to the required state, the following APIs can be run.

Run the PBIST test-for-diagnostic:

\code{.c}

bool PBISTResult;
        
status = SDL_PBIST_selfTest(SDL_PBIST_INST_MPU, SDL_PBIST_NEG_TEST, timeoutVal, &PBISTResult);
if ((status != SDL_PASS) || (PBISTResult == false))
{
    // test failed
}

\endcode

Perform the PBIST test

\code{.c}

bool PBISTResult;

status = SDL_PBIST_selfTest(SDL_PBIST_INST_MPU, SDL_PBIST_TEST, timeoutVal, &PBISTResult);
if ((status != SDL_PASS) || (PBISTResult == false))
{
    // test failed
}

\endcode

## API

\ref SDL_PBIST_MODULE