# SDL TMU ROM_CHECKSUM Example {#EXAMPLES_SDL_TMU_ROM_CHECKSUM}

[TOC]

# Introduction

This example demonstrates usage of the SDL TMU ROM Checksum. This example is used to check the integrity of the TMU ROM data. Its work is to take a set of data associated with the memory regions of TMU ROM and perform checksum on that data and then compare that resultant data value against a pre-determined golden vector value.


Use Cases
---------
 Use Case | Description
 ---------|------------
 UC-1     | Calculate TMU ROM CHECKSUM from R5F Core.



# Supported Combinations {#EXAMPLES_SDL_ROM_CHECKSUM_COMBOS}



 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0 nortos
 Toolchain      | ti-arm-clang
 Board          | @VAR_BOARD_NAME_LOWER
 Example folder | examples/sdl/tmu_rom_checksum/



# Steps to Run the Example

- **When using CCS projects to build**, import the CCS project for the required combination
  and build it using the CCS project menu (see \ref CCS_PROJECTS_PAGE).
- **When using makefiles to build**, note the required combination and build using
  make command (see \ref MAKEFILE_BUILD_PAGE)
- Launch a CCS debug session and run the executable, see \ref CCS_LAUNCH_PAGE

# See Also

\ref SDL_TMU_ROM_CHECKSUM_PAGE

# Sample Output


\code
[MAIN_Cortex_R5_0_0]
ROM Checksum Example Application
Compute ROM-Checksum Data integrity passed
\endcode


