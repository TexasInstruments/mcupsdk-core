# SDL ECC {#EXAMPLES_SDL_ECC}

[TOC]

# Introduction

This example shows setup and usage of some ECC Aggregators for a few events in the Main domain.  It shows the following:

* Setup of an ESM application callback to receive Single Error Correction (SEC) and Double Error Detection (DED) events and setup of ECC Aggregators in general
* Triggering of ECC events for a small number of RAM IDs, including Interconnect type and Wrapper type
* Printing out error information within the ECC callback upon reception of ECC events

The following use cases are implemented in this example:

Use Cases
---------
Use Case | Description
---------|------------
UC-0     | Single DED error on Main ESM for wrapper RAM ID type
UC-1     | Single SEC error on Main ESM for wrapper RAM ID type
UC-2     | Parity error injection on Main ESM for interconnect RAM ID type


# Supported Combinations {#EXAMPLES_SDL_ECC_COMBOS}

\cond SOC_AM64X

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | m4fss0-0 nortos
 Toolchain      | ti-arm-clang
 Board          | @VAR_BOARD_NAME_LOWER
 Example folder | examples/sdl/ecc/

\endcond


\cond SOC_AM62X

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0 nortos
 ^				| m4fss0-0 nortos
 Toolchain      | ti-arm-clang
 Board          | @VAR_BOARD_NAME_LOWER
 Example folder | examples/sdl/ecc/

\endcond
# Steps to Run the Example

- **When using CCS projects to build**, import the CCS project for the required combination
  and build it using the CCS project menu (see \ref CCS_PROJECTS_PAGE).
- **When using makefiles to build**, note the required combination and build using
  make command (see \ref MAKEFILE_BUILD_PAGE)
- Launch a CCS debug session and run the executable, see \ref CCS_LAUNCH_PAGE
# Sample Output

Shown below is a sample output when the application is run,

\code
[BLAZAR_Cortex_M4F_1]
ECC Example Application

Unit tests started ...

ECC_Example_init: Init MAIN ESM complete


ECC_Example_init: Init WKUP ESM complete



SDTF_init: AGGR0 ECC Init complete



 ESM Safety Example tests: starting

 AGGR0 Double bit error inject Example test UC-1: starting

 AGGR0 Double bit error inject test: Subtype 0x00000001 test complete

 Waiting for ESM Interrupt



UC-1: Got Low priority ESM Interrupt



 AGGR0 Double bit error inject Example test UC-2: starting

 AGGR0 Double bit error inject test: Subtype 0x00000002 test complete

 Waiting for ESM Interrupt



 UC-2: Got High priority ESM Interrupt





**** AGGR0 Memory Parity TEST ****



 AGGR0 Single bit error self test: Subtype 0x00000001 test complete

 Waiting for ESM Interrupt



 Memory Parity Error Test Complete



 ECC Safety Example tests: success
 ECC UC-1 and UC-2 Test
All Use_Cases have passed.
../../../ecc_main.c:259:ECC_Example_app:MCUSDK-0:PASS

-----------------------
1 Tests 0 Failures 0 Ignored
All tests have passed
\endcode
