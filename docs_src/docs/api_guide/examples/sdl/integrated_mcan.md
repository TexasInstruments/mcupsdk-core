# SDL INTEGRATED MCAN EXAMPLE {#EXAMPLES_SDL_INTEGRATED_MCAN}

[TOC]

# Introduction {#EXAMPLES_SDL_MCAN_INTRO}

This is a simple example showing the Software Development Kit (SDK) example MCAN driver in Polled Mode running with some of the example Software Diagnostic Library (SDL) diagnostics in FreeRTOS.  This example currently only supports the am263x-cc.

It will:
* Run startup diagnostics using SDL.
* Send and receive data over the MCAN interface in loopback mode.
* Periodically perform runtime diagnostics with SDL.

Assuming no unexpected ESM events or diagnostic failures, the example will send and receive a fixed amount of data and perform periodic runtime diagnostics showing such on the UART console. After sending and receiving a fixed amount of data it will Shutdown.

# Overview {#EXAMPLES_SDL_MCAN_OVERVIEW}
As the example runs, it will output what is happening over the UART interface showing:
* how much data has been sent an received over the MCAN interface (with error stats). See MCAN statistics.
* the results of the runtime diagnostics: PASS or FAIL.

The SDK Users Guide talks about how to configure and use the UART.

# SDL Diagnostics
The table below describes the diagnostics used from SDL in this example. In general, they will not be used to perform negative or destructive tests.

| Diagnostic            |    Use         |  Comments                                                                       |
| ----------------------|----------------|---------------------------------------------------------------------------------|
| Self-Test Check (STC) | Startup        | Checks R5F0-0 and R5F0-1 Cores.                                                 |
| PBIST                 | Startup        | Checks R5F0.                                                                    |
| CCM        (Lockstep) | Startup        | Checks R5F0-0 and R5F0-1 Cores.                                                 |
| MCRC                  | Runtime        | Autocpu mode.                                                                   |
| MCAN0 ECC             | Runtime        | Performs 1 bit and 2 bit injection tests for MCAN0.                             |
| ICSSM ECC             | Runtime        | Performs 1 bit and 2 bit injection tests for ICSSM.                             |
| MSSL2 ECC             | Runtime        | Performs 1 bit and 2 bit injection tests for MSSL2.                             |
| RTI                   | Runtime        | R5F0-0 UC1 RTI diagnostic.                                                      |
| RTI                   | Runtime        | R5F0-0 UC2 RTI diagnostic.                                                      |
| DCC                   | Runtime        | DCC diagnostic UC1.                                                             |
| TCM Parity            | Runtime        | Parity TCM diagnostic.                                                          |
| DMA Parity            | Runtime        | Parity DMA diagnostic.                                                          |
| ECC Bus Safety        | Runtime        | SEC, DED and RED tests.                                                         |
| CCM        (Lockstep) | Runtime        | Checks R5F0-0 and R5F0-1 Cores.                                                 |

## Common ESM {#EXAMPLES_SDL_MCAN_COMMONESM}
This example uses a common ESM and callback. In configuring the ESM we do not make any assertions on the error pin. Instead, if a diagnostic fails, or there is an unexpected ESM event, we will assert the error PIN (LD16).

NOTE that our common call back will make additional calls into diagnostic specific functions if we are running diagnostics. This is necessary so that the diagnostic code can verify operation.

# FreeRTOS Tasks {#EXAMPLES_SDL_MCAN_FREERTOSTASKS}
Two FreeRTOS tasks are created:
* MCAN - will send and receive data in loopback mode on MCAN0.
* Main - controls the system and will perform periodic runtime SDL diagnostics along with handling ESM events.

The Main task will run with the highest priority of the two.

# Operation {#EXAMPLES_SDL_MCAN_FLOW}

The basic flow of the example is divided into three major pieces:
* Startup
* Runtime
* Shutdown

## Start up {#EXAMPLES_SDL_MCAN_FLOW_STARTUP}
When the example starts up, several diagnostics are run before the example will continue.  Should they fail, the ESM safety LED (LD16) is lit up and the example goes into an infinite loop. Nothing is printed to the UART as we do not have access to the UART until FreeRTOS is running.

## Runtime {#EXAMPLES_SDL_MCAN_FLOW_RUNTIME}
The example will send and receive MCAN0 using a loopback mode. Periodic diagnostics are performed. The SDL diagnostics are triggered by using a FreeRTOS timer which is checked for by the Main task. The timer is configured to run about every 2 minutes. This is an arbitrary value, just to show the diagnostics running periodically as the example runs.

## Shutdown {#EXAMPLES_SDL_MCAN_FLOW_SHUTDOWN}
The example will stop and shutdown whenever the following occurs first:
•	an unexpected ESM event occurs.
•	a runtime diagnostic fails, or
•	the example has sent and received roughly 20Mb of data.

## Example States {#EXAMPLES_SDL_MCAN_FLOW_STATES}
The example uses several states to track what is happening in the system. These states are used by the FreeRTOS tasks. The states are described in the table below.

State           |    Description                   |  Comments
----------------|----------------------------------|----------------------------------------------------------------------------------------------------
Startup         | System is in startup             | Startup diagnostics are run.
Startup Done    | Startup is complete              | If start-up diagnostics were successful the Main task will transition the example to Freerun after finalizing initialization.
Freerun         | System is normal.                | Example can send and receive MCAN data.
Block           | An unexpected ESM event occurred | Only the Main task will run in a blocked state and will transition the example to Shutdown.
Run Diagnostics | Performing runtime diagnostics   | The Main task will transition to Freerun if successful or Block if it is not.
Shutdown        | Shuts the system down            | See 1 below.
1 - This example will enter the Shutdown state if:
* A runtime Diagnostic fails.
* An unexpected ESM event occurs.
* If the example sends and receives a certain amount of data with the MCAN driver. This is considered a normal shutdown.

## MCAN Statistics {#EXAMPLES_SDL_MCAN_STATS}

The MCAN task will display statistics about every 1K polls. During a poll cycle it will send and receive a small amount of fixed data.

An example of the statistics output is below:
Polled: 2000 TxBytes: 800000  RxBytes:800000  TxErr: 0  RxErr: 0  BadID: 0  MismatchData: 0

* Polled:  The number of times the polling routine has been called.
* TxBytes: The number of bytes that have been transmitted (cumulative).
* RxBytes: The number of bytes that have been received (cumulative).
* TxErr:   The number of transmit errors (cumulative).
* RxErr:   The number of receive errors (cumulative).
* BadID:   The number of times the transmitted and received header IDs did not match (cumulative).
* MisMatchData:  The number of times the transmitted and received data packets did not match. (cumulative).

# Supported Combinations {#EXAMPLES_SDL_MCAN_COMBOS}

 Parameter      | Value
 ---------------|-------------------------------------------------
 CPU + OS       | r5fss0-0 FreeRTOS
 Toolchain      | ti-arm-clang
 Board          | am263x-cc
 Example folder | examples/sdl/integrated_examples

# Steps to Run the Example {#EXAMPLES_SDL_MCAN_STEPSTORUN}

* **When using CCS projects to build**, import the CCS project for the required combination
  and build it using the CCS project menu (see \ref CCS_PROJECTS_PAGE).
* **When using makefiles to build**, note the required combination and build using
  make command (see \ref MAKEFILE_BUILD_PAGE)
* Flash the image and access the UART.

# Steps to Test the Example

- Integrated example should only be tested in QSPI boot mode and should not be tested in no boot mode with debugger connected.
- Follow the below procedure to test the Integrated example.
  1. A quick recap of steps done so far that are needed for the flashing to work
     - Make sure the UART port used for terminal is identified as mentioned in \ref CCS_UART_TERMINAL
     - Make sure python3 is installed as mentioned in \ref INSTALL_PYTHON3
     - Make sure you have the EVM power cable and UART cable connected as shown in \ref EVM_CABLES
  2. Build the integrated application as mentioned in Build a Hello World example, \ref GETTING_STARTED_BUILD
  3. As part of the build process in the final step a file with extension .appimage is generated. This is the file we need to flash.
  4. When building with makefiles and single-core projects, this file can be found here

			${SDK_INSTALL_PATH}/examples/sdl/integrtaed_examples/{board}/r5fss0-0_freertos/ti-arm-clang/mcan_sdl.release.appimage

  5. A default configuration file can be found at below path. You can edit this file directly or take a copy and edit this file.

			${SDK_INSTALL_PATH}/tools/boot/sbl_prebuilt/{board}/default_sbl_qspi.cfg

  6. Edit below line in the config file to point to your application .appimage file. Give the absolute path to the .appimage file
    or path relative to ${SDK_INSTALL_PATH}/tools/boot. **Make sure to use forward slash / in the filename path**.

			--file=../../examples/sdl/integrtaed_examples/{board}/r5fss0-0_freertos/ti-arm-clang/mcan_sdl.release.appimage --operation=flash --flash-offset=0x80000
  7. Save and close the config file.
  8. Flash the application by follow the steps mentioned in the page, \ref GETTING_STARTED_FLASH

# See Also {#EXAMPLES_SDL_MCAN_SEEALSO}
* AM263x Sitara™ Microcontrollers Texas Instruments Families of Products Technical Reference Manual.
* Individual Examples for the diagnostics in this Users Guide.

# Sample Output {#EXAMPLES_SDL_MCAN_SAMPLEOUT}

Shown below is a sample output when the application is run:
On a failure
\code
On startup:

[MAIN] Start up diagnostics (STC,PBIST,CCM) passed.
[MAIN] MCAN set to Loopback Mode.
[MAIN] Diagnostic check timer set to 120000 ticks.
[MAIN] System set to FREERUN.
[MAIN] MCAN task priority = 29
[MAIN] MAIN task priority = 30
[MAIN] Polled: 0 TxBytes: 0  RxBytes:  0  TxErr: 0  RxErr: 0  BadID: 0 MismatchData: 0

As the example runs MCAN stats printed every 1K polls:

[MCAN] Polled: 1000 TxBytes: 400000  RxBytes:400000  TxErr: 0  RxErr: 0  BadID: 0  MismatchData: 0
[MCAN] Polled: 2000 TxBytes: 800000  RxBytes:800000  TxErr: 0  RxErr: 0  BadID: 0  MismatchData: 0

When runtime diagnostics are performed:

[MCAN] Polled: 16000 TxBytes: 6400000  RxBytes:6400000  TxErr: 0  RxErr: 0  BadID: 0  MismatchData: 0
[MAIN] Running diagnostics...
    RTI UC1...... PASSED. 
    RTI UC2...... PASSED. 
    MCRC AUTO mode test... PASSED. 
    ECC MCAN0.... PASSED. 
    ECC ICSSM.... PASSED. 
    ECC MSSL2.... PASSED. 
    DCC UC1...... PASSED. 
    TCM Parity... PASSED. 
    DMA Parity... PASSED. 
    Bus Safety... PASSED. 
    Lockstep mode test... PASSED. 
[MAIN] Diagnostics Complete.
[MCAN] Polled: 17000 TxBytes: 6800000  RxBytes:6800000  TxErr: 0  RxErr: 0  BadID: 0  MismatchData: 0

On a diagnostic failure you will see something like this:

[MCAN] Polled: 31000 TxBytes: 12400000  RxBytes:12400000  TxErr: 0  RxErr: 0  BadID: 0  MismatchData: 0
[MCAN] Polled: 32000 TxBytes: 12800000  RxBytes:12800000  TxErr: 0  RxErr: 0  BadID: 0  MismatchData: 0
[MAIN] Running diagnostics...
    RTI UC1...... PASSED.
    ECC MCAN0.... PASSED.
    DCC UC1...... PASSED.
    TCM Parity... FAILED.
[MAIN] Example entered BLOCKED state based on a safety check.
[MAIN] Shutting down.
[MAIN] Example is done.
Some tests have failed

Normal example completion will look like this:

[MCAN] Polled: 50000 TxBytes: 20000000  RxBytes:20000000  TxErr: 0  RxErr: 0  BadID: 0  MismatchData: 0
[MAIN] Shutting down.
[MAIN] Example is done.
All tests have passed
\endcode
