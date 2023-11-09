# Coremark Benchmark {#EXAMPLES_COREMARK}

[TOC]

\cond SOC_AM243X || SOC_AM263X || SOC_AM263PX
# Supported Combinations {#EXAMPLES_COREMARK_COMBOS}

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0 nortos
 Toolchain      | ti-arm-clang
 Boards         | @VAR_BOARD_NAME_LOWER, @VAR_LP_BOARD_NAME_LOWER
 Example folder | examples/benchmarks/coremark_benchmark
\endcond

\cond SOC_AM273X || SOC_AM64X
# Supported Combinations {#EXAMPLES_COREMARK_COMBOS}

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0 nortos
 Toolchain      | ti-arm-clang
 Boards         | @VAR_BOARD_NAME_LOWER,
 Example folder | examples/benchmarks/coremark_benchmark
\endcond
# Introduction

- Coremark is an industry standard benchmark used for testing processor's core features
- This demo provides a means of measuring the performance of MCUs (Microcontrollers) and CPUs (Central Processing Units) used in embedded systems
  The demo initializes the data block and runs the benchmark algorithm. Here, 2000 performance run parameters are used for coremark benchmarking.
  The input seeds are initialized from a source that cannot be determined at compile time, memory blocks are initialized and then the application is run and timed.
- The output validity is tests if the seeds are known, and the output is a single-number score which shows the performance for quick comparison.
- The out of box demo contains the .code in STACK and .data in TCMA memory section.
- The memory placement of these sections can be changed from the syscfg memory configuration section.

# Steps to Run the Example

## Building Coremark application

- When using CCS projects to build, import the CCS project for the required combination
  and build it using the CCS project menu (see \ref CCS_PROJECTS_PAGE).
- When using makefiles to build, note the required combination and build using
  make command (see \ref MAKEFILE_BUILD_PAGE)

# Performance statistics

Device              | Value (CoreMark/MHz)
--------------------|-------------------------
\cond SOC_AM243X
AM243x-EVM          | 7.539067
AM243x-LP           | 7.539031
\endcond
\cond SOC_AM263X
AM263x-CC           | 3.768698
AM263x-LP           | 7.539031
\endcond
\cond SOC_AM263PX
AM263Px-CC          | 3.769163
\endcond
\cond SOC_AM273X
AM263x-EVM          | 3.769906
\endcond
\cond SOC_AM64X
AM64x-EVM          | 7.539166
\endcond

## Sample output for Coremark example

\code

\cond SOC_AM243X

[MAIN_Cortex_R5_0_0] 2K performance run parameters for coremark.
CoreMark Size    : 666
begin tick      : 981
end tick      : 4975071
Total ticks      : 4974090
Total time (secs): 4.974090
Iterations/Sec   : 3015.626979
Iterations       : 15000
Compiler version : GCCTI Clang 15.0.7 (ssh://git@bitbucket.itg.ti.com/code/llvm-project.git 915543d86f9d0c566b00bff612fa4c438b6e7803)
Compiler flags   : (flags unknown)
Memory location  : STACK
seedcrc          : 0xe9f5
[0]crclist       : 0xe714
[0]crcmatrix     : 0x1fd7
[0]crcstate      : 0x8e3a
[0]crcfinal      : 0x65c5
Correct operation validated. See README.md for run and reporting rules.
CoreMark 1.0 : 3015.626979 / GCCTI Clang 15.0.7 (ssh://git@bitbucket.itg.ti.com/code/llvm-project.git 915543d86f9d0c566b00bff612fa4c438b6e7803) (flags unknown)
CoreMark/MHz :7.539067 / STACK

\endcond

\cond SOC_AM64X

2K performance run parameters for coremark.
CoreMark Size    : 666
begin tick      : 960
end tick      : 4974985
Total ticks      : 4974025
Total time (secs): 4.974025
Iterations/Sec   : 3015.666387
Iterations       : 15000
Compiler version : GCCTI Clang 15.0.7 (ssh://git@bitbucket.itg.ti.com/code/llvm-project.git 915543d86f9d0c566b00bff612fa4c438b6e7803)
Compiler flags   : (flags unknown)
Memory location  : STACK
seedcrc          : 0xe9f5
[0]crclist       : 0xe714
[0]crcmatrix     : 0x1fd7
[0]crcstate      : 0x8e3a
[0]crcfinal      : 0x65c5
Correct operation validated. See README.md for run and reporting rules.
CoreMark 1.0 : 3015.666387 / GCCTI Clang 15.0.7 (ssh://git@bitbucket.itg.ti.com/code/llvm-project.git 915543d86f9d0c566b00bff612fa4c438b6e7803) (flags unknown)
CoreMark/MHz :7.539166 / STACK

\endcond

\cond SOC_AM263X

[MAIN_Cortex_R5_0_0] 2K performance run parameters for coremark.
CoreMark Size    : 666
begin tick      : 981
end tick      : 4975071
Total ticks      : 4974090
Total time (secs): 4.974090
Iterations/Sec   : 3015.626979
Iterations       : 15000
Compiler version : GCCTI Clang 15.0.7 (ssh://git@bitbucket.itg.ti.com/code/llvm-project.git 915543d86f9d0c566b00bff612fa4c438b6e7803)
Compiler flags   : (flags unknown)
Memory location  : STACK
seedcrc          : 0xe9f5
[0]crclist       : 0xe714
[0]crcmatrix     : 0x1fd7
[0]crcstate      : 0x8e3a
[0]crcfinal      : 0x65c5
Correct operation validated. See README.md for run and reporting rules.
CoreMark 1.0 : 3015.626979 / GCCTI Clang 15.0.7 (ssh://git@bitbucket.itg.ti.com/code/llvm-project.git 915543d86f9d0c566b00bff612fa4c438b6e7803) (flags unknown)
CoreMark/MHz :7.539067 / STACK

\endcond

\cond SOC_AM263PX

[Cortex_R5_0] 2K performance run parameters for coremark.
CoreMark Size    : 666
begin tick      : 71
end tick      : 9949230
Total ticks      : 9949159
Total time (secs): 9.949159
Iterations/Sec   : 1507.665120
Iterations       : 15000
Compiler version : GCCTI Clang 15.0.7 (ssh://git@bitbucket.itg.ti.com/code/llvm-project.git 915543d86f9d0c566b00bff612fa4c438b6e7803)
Compiler flags   : (flags unknown)
Memory location  : STACK
seedcrc          : 0xe9f5
[0]crclist       : 0xe714
[0]crcmatrix     : 0x1fd7
[0]crcstate      : 0x8e3a
[0]crcfinal      : 0x65c5
Correct operation validated. See README.md for run and reporting rules.
CoreMark 1.0 : 1507.665120 / GCCTI Clang 15.0.7 (ssh://git@bitbucket.itg.ti.com/code/llvm-project.git 915543d86f9d0c566b00bff612fa4c438b6e7803) (flags unknown)
CoreMark/MHz :3.769163 / STACK

\endcond

\cond SOC_AM273X

[Cortex_R5_0] 2K performance run parameters for coremark.
CoreMark Size    : 666
begin tick      : 79
end tick      : 9947276
Total ticks      : 9947197
Total time (secs): 9.947197
Iterations/Sec   : 1507.962494
Iterations       : 15000
Compiler version : GCCTI Clang 15.0.7 (ssh://git@bitbucket.itg.ti.com/code/llvm-project.git 915543d86f9d0c566b00bff612fa4c438b6e7803)
Compiler flags   : (flags unknown)
Memory location  : STACK
seedcrc          : 0xe9f5
[0]crclist       : 0xe714
[0]crcmatrix     : 0x1fd7
[0]crcstate      : 0x8e3a
[0]crcfinal      : 0x65c5
Correct operation validated. See README.md for run and reporting rules.
CoreMark 1.0 : 1507.962494 / GCCTI Clang 15.0.7 (ssh://git@bitbucket.itg.ti.com/code/llvm-project.git 915543d86f9d0c566b00bff612fa4c438b6e7803) (flags unknown)
CoreMark/MHz :3.769906 / STACK

\endcond

\endcode

## Troubleshooting issues

