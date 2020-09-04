# SA2UL HMAC SHA-1 Test {#EXAMPLES_DRIVERS_SA2UL_HMAC_SHA1}

[TOC]

# Introduction

This example demonstrates the how to generate HMAC SHA-1 hash using SA2UL module.

# Supported Combinations {#EXAMPLES_DRIVERS_SA2UL_HMAC_SHA1_COMBOS}

\cond SOC_AM64X || SOC_AM243X

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0 nortos
 Toolchain      | ti-arm-clang
 Board          | @VAR_BOARD_NAME_LOWER
 Example folder | examples/security/crypto/sa2ul_hmac_sha/crypto_hmac_sha1/crypto_hmac_sha1.c

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
[CRYPTO] HMAC SHA-1 example started ... 
[CRYPTO] HMAC SHA-1 example completed!!
All tests have passed!!

\endcode


