# SDL DSS L3 {#EXAMPLES_SDL_DSS_L3_MAIN}

[TOC]

# Introduction

This example demonstrates the usage of the SDL DSS_L3 module. The example shows how to setup and use the ECC BUS Safety Diagnostic operation.
Shows the generation of SEC, DED and RED error on DSS L3 BANK A, BANK B, BANK C and BANK D.

Note : SEC - Single Error Correction, DED - Double Error Correction, RED - Redundancy Error Correction
Use Cases
---------

 Use Case | Description
 ---------|------------
 UC-1     | SEC Error insertion on DSS L3 BANK A.
 UC-2     | DED Error insertion on DSS L3 BANK A.
 UC-3     | RED Error insertion on DSS L3 BANK A.
 UC-4     | SEC Error insertion on DSS L3 BANK B.
 UC-5     | DED Error insertion on DSS L3 BANK B.
 UC-6     | RED Error insertion on DSS L3 BANK B.
 UC-7     | SEC Error insertion on DSS L3 BANK C.
 UC-8     | DED Error insertion on DSS L3 BANK C.
 UC-9     | RED Error insertion on DSS L3 BANK C.
 UC-10    | SEC Error insertion on DSS L3 BANK D.
 UC-11    | DED Error insertion on DSS L3 BANK D.
 UC-12    | RED Error insertion on DSS L3 BANK D.


# Supported Combinations {#EXAMPLES_SDL_DSS_L3_MAIN_COMBOS}

\cond SOC_AWR294X

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | c66  nortos
 Toolchain      | ti-c6000
 Board          | @VAR_BOARD_NAME_LOWER
 Example folder | examples/sdl/dss_l3/dss_l3_main/

\endcond

# Steps to Run the Example

- **When using CCS projects to build**, import the CCS project for the required combination
  and build it using the CCS project menu (see \ref CCS_PROJECTS_PAGE).
- **When using makefiles to build**, note the required combination and build using
  make command (see \ref MAKEFILE_BUILD_PAGE)
- Launch a CCS debug session and run the executable, see \ref CCS_LAUNCH_PAGE

# See Also

\ref SDL_DSS_L3_PAGE

# Sample Output

Shown below is a sample output when the application is run,

\code
 DSS_L3 Application

 DSS_L3 TEST START : starting

 Applications Name: DSS_L3_BANKA_SecTest  PASSED

 Applications Name: DSS_L3_BANKA_DedTest  PASSED

 Applications Name: DSS_L3_BANKA_RedTest  PASSED

 Applications Name: DSS_L3_BANKB_SecTest  PASSED

 Applications Name: DSS_L3_BANKB_DedTest  PASSED

 Applications Name: DSS_L3_BANKB_RedTest  PASSED

 Applications Name: DSS_L3_BANKC_SecTest  PASSED

 Applications Name: DSS_L3_BANKC_DedTest  PASSED

 Applications Name: DSS_L3_BANKC_RedTest  PASSED

 Applications Name: DSS_L3_BANKD_SecTest  PASSED

 Applications Name: DSS_L3_BANKD_DedTest  PASSED

 Applications Name: DSS_L3_BANKD_RedTest  PASSED

 All tests have passed
\endcode


