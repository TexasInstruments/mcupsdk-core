# Real Time Debug {#EXAMPLES_REAL_TIME_DEBUG}

[TOC]

# Introduction

This examples demonstrates the real time debug functionality. It has a global variables which
is continuously updated in the application. This variable can be monitored in runtime from the
expression window in CCS.

# Supported Combinations

\cond SOC_AM263X

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0 freertos
 ^              | r5fss0-0 nortos
 Toolchain      | ti-arm-clang
 Board          | @VAR_BOARD_NAME_LOWER, @VAR_LP_BOARD_NAME_LOWER
 Example folder | examples/kernel/dpl/interrupt_prioritization/

\endcond

# Steps to Run the Example

- **When using CCS projects to build**, import the CCS project for the required combination
  and build it using the CCS project menu (see \ref CCS_PROJECTS_PAGE).
- **When using makefiles to build**, note the required combination and build using
  make command (see \ref MAKEFILE_BUILD_PAGE)
- Launch a CCS debug session and run the executable, see \ref REAL_TIME_DEBUG_SUPPORT_GUIDE
- The value of global variable can be monitored runtime in the expression window.
