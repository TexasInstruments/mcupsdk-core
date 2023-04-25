# LBIST {#SDL_LBIST_PAGE}

[TOC]

Built-in Self-test (BIST) is a feature that allows self testing of the memory areas and logic circuitry in an Integrated Circuit (IC) without any external test equipment. In an embedded system, these tests are typically used during boot time or shutdown of the system to check the health of an SoC.

LBIST is used to test the logic circuitry in an SoC associated with the CPU cores. There are multiple LBIST instances in the SoC, and each has a different processor core associated with it. There are LBIST tests that can be software-initiated. Those are supported by SDL.

Some things to note:

* LBIST is expected to be run at startup or once per drive cycle.
* LBIST must be initiated from a different core than is being tested, similar to PBIST

Device supports a software-initiated LBIST that can run be initiated at runtime. Note, however, that LBIST is destructive to the core/IP on which it is run. If it is executed after the IP under test is already in use in the system, it is the application's responsibility to perform any necessary context save/restore necessary for the core/IP. Also, it is not supported to run the LBIST test for a core on itself, as it will be self destructive.

The LBIST Module of the SDL supports execution of the software-initiated LBIST for the various supported instances. It provides the following services:

* Execution of the LBIST test for a specified instance
* Configuration of the LBIST test for an instance
* Start of the LBIST test
* Checking of the LBIST results
* Restore core to system control (LBIST reset and release test mode)
* Return the status of the test

## Note
    Execution of the LBIST tests requires preparation of the IPs under test by bringing them to a certain power and reset state before executing the test. It will be required that the application bring the cores/IPs to the proper state before executing the LBIST. Additionally, there is an "exit sequence" that is required to bring the cores/IPs back to the system control after the LBIST test is executed. This will also be the responsibility of the application. The LBIST examples provided with SDL will give the necessary sequences, which can be used by the application for implementing the sequence.


## SysConfig Features

- None

## Features NOT Supported

- None

## Important Usage Guidelines

- Call SDL_LBIST_selfTest(LBIST_MCU_M4F, SDL_LBIST_TEST ) to start LBIST test.
- Call SDL_LBIST_checkDone( ) to know the if the LBIST test is completed.
- Call SDL_LBIST_checkResult( ) to know the if the MISR signatures are matching. This API should be called only after SDL_LBIST_checkDone( ) API returns LBIST_DONE.
- Call SDL_LBIST_selfTest(LBIST_MCU_M4F, SDL_LBIST_TEST_RELEASE ) to release the LBIST test mode.

## Example Usage

- None

## API

\ref SDL_LBIST_API
