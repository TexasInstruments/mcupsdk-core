# SDL DSS ECC MAILBOX {#EXAMPLES_SDL_DSS_MAILBOX}

[TOC]

# Introduction

This example demonstrates the usage of the SDL DSS MAILBOX module. The example shows how to setup and use the ECC BUS Safety Diagnostic operation.
Shows the generation of SEC and DED error on DSS MAILBOX ECC Aggregator.

Use Cases
---------

 Use Case | Description
 ---------|------------
 UC-1     | Single bit error injection on DSS_MAILBOX.
 UC-2     | Double bit error injection on DSS_MAILBOX.

# Supported Combinations {#EXAMPLES_SDL_DSS_MAILBOX_COMBOS}

\cond SOC_AM273X || SOC_AWR294X

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | c66ss0 nortos
 Toolchain      | ti-c6000
 Board          | @VAR_BOARD_NAME_LOWER
 Example folder | examples/sdl/ecc/sdl_dss_mailbox/

\endcond

# Steps to Run the Example

- **When using CCS projects to build**, import the CCS project for the required combination
  and build it using the CCS project menu (see \ref CCS_PROJECTS_PAGE).
- **When using makefiles to build**, note the required combination and build using
  make command (see \ref MAKEFILE_BUILD_PAGE)
- Launch a CCS debug session and run the executable, see \ref CCS_LAUNCH_PAGE

# See Also

\ref SDL_ECC_PAGE

# Sample Output

Shown below is a sample output when the application is run,

\cond (SOC_AM273X) || (SOC_AWR294X)
\code

ECC Example Application

ECC UC-1 and UC-2 Test
ECC_Test_init: Initialize of DSS ECC AGGR Memory is complete

ESM_Test_init: Init DSS ESM complete

ECC_Test_init: DSS ECC AGGR initialization is completed

ECC Safety Example tests: starting
DSS MAILBOX Single bit error inject: test starting

DSS MAILBOX Single bit error inject at pErrMem 0x83100000

Waiting for ESM Interrupt

UC-1: Injected 1-bit error and got ESM Interrupt

DSS MAILBOX Double bit error inject: starting

DSS MAILBOX Double bit error inject at pErrMem 0x83100000

Waiting for ESM Interrupt

UC-2: Injected 2-bit error and got ESM Interrupt

All Use_Cases have passed.

\endcode
\endcond
