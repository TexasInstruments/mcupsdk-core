# SA2UL Random Number Generator Test {#EXAMPLES_DRIVERS_SA2UL_RNG}

[TOC]

# Introduction

This example demonstrates the Random number generation using SA2UL module.

# Supported Combinations {#EXAMPLES_DRIVERS_SA2UL_RNG_COMBOS}

\cond SOC_AM64X

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0 nortos
 Toolchain      | ti-arm-clang
 Board          | @VAR_BOARD_NAME_LOWER
 Example folder | examples/security/crypto/sa2ul_rng/sa2ul_rng.c

\endcond

\cond SOC_AM243X

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0 nortos
 Toolchain      | ti-arm-clang
 Board          | @VAR_BOARD_NAME_LOWER, @VAR_LP_BOARD_NAME_LOWER
 Example folder | examples/security/crypto/sa2ul_rng/sa2ul_rng.c

\endcond

# Steps to Run the Example

- **When using CCS projects to build**, import the CCS project for the required combination
  and build it using the CCS project menu (see \ref CCS_PROJECTS_PAGE).
- **When using makefiles to build**, note the required combination and build using
  make command (see \ref MAKEFILE_BUILD_PAGE)
- Launch a CCS debug session and run the executable, see \ref CCS_LAUNCH_PAGE

# See Also

\ref SECURITY_SA2UL_MODULE_PAGE

# Sample Output

Shown below is a sample output when the application is run.


CCS Console:
\code
All tests have passed!!
[SA2UL] Sa2ul Rng example started ...
[SA2UL] Sa2ul Rng output word 1 -- 0x3704DD0A
[SA2UL] Sa2ul Rng output word 2 -- 0x68719DEA
[SA2UL] Sa2ul Rng output word 3 -- 0x3A2733EE
[SA2UL] Sa2ul Rng output word 4 -- 0x943F007E
[SA2UL] Sa2ul Rng output word 1 -- 0xCE83C20D
[SA2UL] Sa2ul Rng output word 2 -- 0xAEB445B0
[SA2UL] Sa2ul Rng output word 3 -- 0xE052FEC9
[SA2UL] Sa2ul Rng output word 4 -- 0x713231C3
[SA2UL] Sa2ul Rng output word 1 -- 0xD5CF553B
[SA2UL] Sa2ul Rng output word 2 -- 0x1AB11E5F
[SA2UL] Sa2ul Rng output word 3 -- 0x25163494
[SA2UL] Sa2ul Rng output word 4 -- 0x5CD0432F
[SA2UL] Sa2ul Rng output word 1 -- 0x8372B743
[SA2UL] Sa2ul Rng output word 2 -- 0x75AA97AC
[SA2UL] Sa2ul Rng output word 3 -- 0x59F277C7
[SA2UL] Sa2ul Rng output word 4 -- 0x8DFC768C
[SA2UL] Sa2ul Rng output word 1 -- 0xAF3B154C
[SA2UL] Sa2ul Rng output word 2 -- 0x3A93F540
[SA2UL] Sa2ul Rng output word 3 -- 0xC3174A84
[SA2UL] Sa2ul Rng output word 4 -- 0xA361D316
[SA2UL] Sa2ul Rng example completed!!
All tests have passed!!

\endcode


