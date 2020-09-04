# Extended OTP Test {#EXAMPLES_EXT_OTP}

[TOC]

# Introduction

This example demonstrates how can one write to the extended OTP eFuses of the device. This is currently only supported in HS-SE devices. This is a special example, and is booted by ROM. Because of this it is to be treated like a bootloader application.

The example tries to write the USB and PCIE VID/PID onto the OTP rows. It also dumps the OTP MMR rows. It makes use of Sciclient API calls to do this, there are wrapper functions provided in the examples for these.

# Supported Combinations {#EXAMPLES_EXT_OTP_COMBOS}

\cond SOC_AM64X

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0 nortos
 Toolchain      | ti-arm-clang
 Board          | @VAR_BOARD_NAME_LOWER
 Example folder | examples/otp/ext_otp/

\endcond

# Steps to Run the Example

- **When using CCS projects to build**, import the CCS project for the required combination
  and build it using the CCS project menu (see \ref CCS_PROJECTS_PAGE).
- **When using makefiles to build**, note the required combination and build using
  make command (see \ref MAKEFILE_BUILD_PAGE)
- Launch a CCS debug session and run the executable, see \ref CCS_LAUNCH_PAGE

# Sample Output

Shown below is a sample output when the application is run.

UART Console:
\code
Enabled VPP
Success programming VID/PID
OTP MMR 0: 0x1000090
OTP MMR 1: 0x0
OTP MMR 2: 0x0
OTP MMR 3: 0x0
OTP MMR 4: 0x0
OTP MMR 5: 0x0
OTP MMR 6: 0x0
OTP MMR 7: 0x0
OTP MMR 8: 0x0
OTP MMR 9: 0x12344321
OTP MMR 10: 0x45a0e047
OTP MMR 11: 0xebc84321
USB VID: 0x45a0
USB PID: 0xe047
PCIE VID: 0x1234
PCIE PID: 0x4321
All tests have passed!!

\endcode
