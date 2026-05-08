# SDL ECC EDMA SRAM Scrub {#EXAMPLES_SDL_ECC_EDMA_SRAM_SCRUB}

[TOC]

# Introduction

This example demonstrates how to use EDMA (Enhanced Direct Memory Access) and RTI (Real-Time Interrupt)
to implement a periodic SRAM scrubbing mechanism for detecting and mitigating Single-Bit Errors (SBE)
in AM263Px/AM263x microcontrollers.

The scrubber reads the first 3 SRAM banks (1.5 MB total) using EDMA transfers triggered by RTI every
50 μs. Each EDMA transfer processes 4 × 64-bit beats (32 bytes). When a SEC (Single Error Correction)
event occurs, the example determines whether:

- The EDMA read buffer matches SRAM data → EDMA read triggered and corrected the error (scrub successful)
- The EDMA read buffer does not match SRAM data → CPU may have read corrupted data; a device reset is required

Memory layout and EDMA/RTI configuration are managed through SysConfig.

Use Cases
---------

 Use Case | Description
 ---------|------------
 UC-1     | Periodic SRAM scrub completes without errors.
 UC-2     | SEC error detected during EDMA scrub; error classified as EDMA-corrected or CPU-impacting.


# Supported Combinations {#EXAMPLES_SDL_ECC_EDMA_SRAM_SCRUB_COMBOS}

\cond SOC_AM263X || SOC_AM263PX

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0 nortos
 Toolchain      | ti-arm-clang
 Board          | @VAR_BOARD_NAME_LOWER
 Example folder | examples/sdl/ecc/sdl_ecc_edma_sram_scrub/

\endcond

# Steps to Run the Example

- **When using CCS projects to build**, import the CCS project for the required combination
  and build it using the CCS project menu (see \ref CCS_PROJECTS_PAGE).
- **When using makefiles to build**, note the required combination and build using
  make command (see \ref MAKEFILE_BUILD_PAGE).
- Launch a CCS debug session and run the executable, see \ref CCS_LAUNCH_PAGE.

# See Also

\ref SDL_ECC_PAGE

# Sample Output

Shown below is a sample output when the application is run:

\cond (SOC_AM263X || SOC_AM263PX)
\code
EDMA RTI triggered SRAM Scrub Test ...
Starting RTI for periodic trigger for EDMA ...

ECC_Test_init: MSS L2 ECC initialization is completed 

MSS L2 RAMB Single bit error inject: starting 
Data mismatch - EDMA read the wrong data - Scrub successful. ...

\endcode
\endcond

The application continuously scrubs SRAM in the background. Any detected SEC errors are
classified and reported via UART.
