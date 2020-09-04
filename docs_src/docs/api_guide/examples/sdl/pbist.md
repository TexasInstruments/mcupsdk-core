# SDL PBIST {#EXAMPLES_SDL_PBIST}

[TOC]

# Introduction

This example demonstrates the usage of the PBIST module. The example shows how to setup and use the PBIST controller.
The example configures the algorithm and memory group. Example also prints the time taken for test execution.

Use Cases
---------

 Use Case | Description
 ---------|------------
 UC-1     | Configure wrong combination of algorithm and memory group.
 UC-2     | Configure correct combination of algorithm and memory group.

# Supported Combinations {#EXAMPLES_SDL_PBIST_COMBOS}

\cond SOC_AM273X || SOC_AWR294X || SOC_AM263X

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0 nortos
 Toolchain      | ti-arm-clang
 Board          | @VAR_BOARD_NAME_LOWER
 Example folder | examples/sdl/pbist/pbist/

\endcond

# Steps to Run the Example

- **When using CCS projects to build**, import the CCS project for the required combination
  and build it using the CCS project menu (see \ref CCS_PROJECTS_PAGE).
- **When using makefiles to build**, note the required combination and build using
  make command (see \ref MAKEFILE_BUILD_PAGE)
- Launch a CCS debug session and run the executable, see \ref CCS_LAUNCH_PAGE

# See Also

\ref SDL_PBIST_PAGE

# Sample Output

Shown below is a sample output when the application is run,

\cond SOC_AM263X
\code
[Cortex_R5_0]
PBIST Application

 Starting PBIST failure insertion test on TOP PBIST
 PBIST failure Insertion test complete for TOP BIST
PBIST Failure Insertion Test completed in 47 micro secs

 Starting PBIST test on TOP PBIST
 PBIST complete for R5 STC
 PBIST complete for R51 STC
 PBIST complete for PBISTROM
 PBIST complete for CPSW
 PBIST complete for ICSSM
 PBIST complete for MBOX
 PBIST complete for MCAN
 PBIST complete for TPCC
 PBIST complete for MSS_L2_1
 PBIST complete for MSS_L2_2
 PBIST complete for MSS_L2_3
 PBIST complete for VIM1 R5SS0
 PBIST complete for VIM0 R5SS1
 PBIST complete for VIM1 R5SS1
 PBIST complete for R5SS1 RAM
 PBIST complete for MSS CR5B ATCM0
 PBIST complete for MSS CR5B ATCM1
 PBIST complete for MSS CR5B BTCM0
 PBIST complete for MSS CR5B BTCM1

 All tests have passed.

 [Cortex_R5_2]
  PBIST Application

  Starting PBIST failure insertion test on TOP PBIST
  PBIST failure Insertion test complete for TOP BIST
 PBIST Failure Insertion Test completed in 0 micro secs

  Starting PBIST test on TOP PBIST
  PBIST complete for  VIM0 R5SS0
  PBIST complete for MSS_L2_0
  PBIST complete for R5SS0 RAM
  PBIST complete for CR5A ROM0
  PBIST complete for TRACE
  PBIST complete for MMCH0
  PBIST complete for MSS CR5A ATCM0
  PBIST complete for MSS CR5A ATCM1
  PBIST complete for MSS CR5A BTCM0
  PBIST complete for MSS CR5A BTCM1

  All tests have passed.
\endcode
\endcond

\cond SOC_AM273X || SOC_AWR294X
\code
[Cortex_R5_0]
PBIST Application

Starting PBIST failure insertion test on TOP PBIST
PBIST failure Insertion test complete for TOP BIST
PBIST Failure Insertion Test completed in 19 micro secs

Starting PBIST test on TOP PBIST
PBIST complete for ADCBUF
PBIST complete for TPCC
PBIST complete for MAILBOX
PBIST complete for COREB VIM
PBIST complete for MCAN
PBIST complete for SPIA
PBIST complete for SPIB
PBIST complete for CORE B R5FSS RAM
PBIST complete for MSS_L2_1
PBIST complete for CPSW
PBIST complete for GPADC
PBIST complete for RETRAM
PBIST complete for STCROM
PBIST complete for CORE B ATCM
PBIST complete for CORE B BTCM

All tests have passed.

Starting PBIST failure insertion test on TOP PBIST
PBIST failure Insertion test complete for TOP BIST
PBIST Failure Insertion Test completed in 14 micro secs

Starting PBIST test on DSP PBIST
PBIST complete for DSS C66 STCROM
PBIST complete for HWA STCROM
PBIST complete for DSS PBISTROM
PBIST complete for C66 L1D
PBIST complete for C66 L1P
PBIST complete for PBIST C66 L2 TAG
PBIST complete for DSS HWA
PBIST complete for DSS HWA MBOX
PBIST complete for PBIST DSS L3 BANKA SUB0
PBIST complete for PBIST DSS L3 BANKB SUB0
PBIST complete for PBIST DSS L3 BANKC SUB0
PBIST complete for DSS MBOX RAM
PBIST complete for DSS TPCC RAM
PBIST complete for DSS L2 BANK0
PBIST complete for DSS L2 BANK1
PBIST complete for DSS L2 PARITY
PBIST complete for HWA RAM
PBIST complete for DSS CBUF
PBIST complete for PBIST DSS L3 BANKA SUB1
PBIST complete for PBIST DSS L3 BANKB SUB1
PBIST complete for PBIST DSS L3 BANKB SUB2
PBIST complete for PBIST DSS L3 BANKC SUB1

All tests have passed.

[Cortex_R5_1]
 PBIST Application

 Starting PBIST failure insertion test on TOP PBIST
 PBIST failure Insertion test complete for TOP BIST
PBIST Failure Insertion Test completed in 19 micro secs

 Starting PBIST test on TOP PBIST
 PBIST complete for MSS_TCMAROM_0
 PBIST complete for MSS_TCMAROM_1
 PBIST complete for PBISTROM
 PBIST complete for CORE A VIM
 PBIST complete for MSS_L2_0
 PBIST complete for CORE A ATCM
 PBIST complete for CORE A BTCM
 PBIST complete for CORE A R5SS RAM
 PBIST complete for MEM_TOP_AURORA
 PBIST complete for MEM_TOP_MDO
 PBIST complete for DBGSS_TRACE

All tests have passed.  
\endcode
\endcond
