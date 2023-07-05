# SDL ROM_CHECKSUM Example {#EXAMPLES_SDL_ROM_CHECKSUM}

[TOC]

# Introduction

This example demonstrates usage of the SDL ROM Checksum. This example is used to check the integrity of the data. Its work is to take a set of data associated with the memory regions of ROM and perform checksum on that data and then compare that resultant data value against a pre-determined golden vector value (golden vector has the expected value which should come as a result of 512-bit of hash message, golden vector is already defined and it has fixed address in ROM region).


\cond SOC_AM243X
Use Cases
---------
 Use Case | Description
 ---------|------------
 UC-1     | Calculate ROM CHECKSUM from R5F Core.
\endcond


# Supported Combinations {#EXAMPLES_SDL_ROM_CHECKSUM_COMBOS}


\cond SOC_AM243X

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0 nortos
 Toolchain      | ti-arm-clang
 Board          | @VAR_BOARD_NAME_LOWER
 Example folder | examples/sdl/rom_checksum/

\endcond


# Steps to Run the Example

- **When using CCS projects to build**, import the CCS project for the required combination
  and build it using the CCS project menu (see \ref CCS_PROJECTS_PAGE).
- **When using makefiles to build**, note the required combination and build using
  make command (see \ref MAKEFILE_BUILD_PAGE)
- Launch a CCS debug session and run the executable, see \ref CCS_LAUNCH_PAGE

# See Also

\ref SDL_ROM_CHECKSUM_PAGE

# Sample Output

\cond  SOC_AM243X
\code
[MAIN_Cortex_R5_0_0]
ROM Checksum Example Application
Compute ROM-Checksum Data integrity passed
\endcode
\endcond

