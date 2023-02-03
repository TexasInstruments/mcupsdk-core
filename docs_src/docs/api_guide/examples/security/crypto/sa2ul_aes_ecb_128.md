# SA2UL AES ECB-128 Test {#EXAMPLES_DRIVERS_SA2UL_AES_ECB_128}

[TOC]

# Introduction

This example demonstrates the AES ECB-128 Encryption and Decryptions using SA2UL module.

# Supported Combinations {#EXAMPLES_DRIVERS_SA2UL_AES_ECB_128_COMBOS}

\cond SOC_AM64X || SOC_AM243X

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0 nortos
 Toolchain      | ti-arm-clang
 Board          | @VAR_BOARD_NAME_LOWER, @VAR_LP_BOARD_NAME_LOWER
 Example folder | examples/security/crypto/sa2ul_aes/crypto_aes_ecb_128/crypto_aes_ecb_128.c

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
[CRYPTO] AES ECB-128 example started ...
[CRYPTO] AES ECB-128 example completed!!
All tests have passed!!

\endcode


