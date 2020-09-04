# Runtime Software Revision Update Example {#EXAMPLES_RUNTIME_SWREV}

[TOC]

# Introduction

This example demonstrates the runtime software revision update. For now only the software revision update for secure boardcfg is supported. In a fresh sample the revision number would be 0U. Example reads the SWREV revision first. If the revision number is 0, then it tries to write to the eFUSE. This example is currently supported only in HS-SE devices. This is a special example, and is booted by ROM. Because of this it is to be treated like a bootloader application. It makes use of Sciclient API calls to do this, there are wrapper functions provided in the example for this.

# Supported Combinations {#EXAMPLES_RUNTIME_SWREV_COMBOS}

\cond SOC_AM64X

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0 nortos
 Toolchain      | ti-arm-clang
 Board          | @VAR_BOARD_NAME_LOWER
 Example folder | examples/otp/runtime_swrev/

\endcond

# Steps to Run the Example

- **When using CCS projects to build**, import the CCS project for the required combination
  and build it using the CCS project menu (see \ref CCS_PROJECTS_PAGE).
- **When using makefiles to build**, note the required combination and build using
  make command (see \ref MAKEFILE_BUILD_PAGE)
- Launch a CCS debug session and run the executable, see \ref CCS_LAUNCH_PAGE

# Sample Output

Shown below is a sample output when the application is run:

\code
Starting Runtime SWREV writer
SWREV read : 0x1
SWREV already written to eFUSE, value : 0x1
All tests have passed!!
\endcode
