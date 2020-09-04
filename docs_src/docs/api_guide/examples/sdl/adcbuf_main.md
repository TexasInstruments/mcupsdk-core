# SDL ADCBUF {#EXAMPLES_SDL_ADCBUF_MAIN}

[TOC]

# Introduction

This example demonstrates the usage of the SDL ADCBUF module. The example shows how to setup and use the ECC BUS Safety Diagnostic operation.
Shows the generation of SEC, DED and RED error on ADCBUF WR and  RED error on ADCBUF RD.

Use Cases
---------

 Use Case | Description
 ---------|------------
 UC-1     | SEC Error insertion on ADCBUF WR.
 UC-2     | DED Error insertion on ADCBUF WR.
 UC-3     | RED Error insertion on ADCBUF WR.
 UC-4     | RED Error insertion on ADCBUF RD.


# Supported Combinations {#EXAMPLES_SDL_ADCBUF_MAIN_COMBOS}

\cond SOC_AWR294X

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | c66  nortos
 Toolchain      | ti-c6000
 Board          | @VAR_BOARD_NAME_LOWER
 Example folder | examples/sdl/adcbuf/adcbuf_main/

\endcond

# Steps to Run the Example

- **When using CCS projects to build**, import the CCS project for the required combination
  and build it using the CCS project menu (see \ref CCS_PROJECTS_PAGE).
- **When using makefiles to build**, note the required combination and build using
  make command (see \ref MAKEFILE_BUILD_PAGE)
- Launch a CCS debug session and run the executable, see \ref CCS_LAUNCH_PAGE

# See Also

\ref SDL_ADCBUF_PAGE

# Sample Output

Shown below is a sample output when the application is run,

\code
 ADCBUF Application

 ADCBUF TEST START : starting
 Applications Name: ADCBUF_WR_SecTest  PASSED

 Applications Name: ADCBUF_WR_DedTest  PASSED

 Applications Name: ADCBUF_WR_RedTest  PASSED

 Applications Name: ADCBUF_RD_RedTest  PASSED

 All tests have passed
\endcode


