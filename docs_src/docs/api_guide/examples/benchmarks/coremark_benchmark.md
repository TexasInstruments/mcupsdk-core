# Coremark Benchmark {#EXAMPLES_COREMARK}

[TOC]

Coremark is an industry standard benchmark used for testing processor's core features.

Read more about Coremark here: https://www.eembc.org/coremark/

# Supported Combinations {#EXAMPLES_COREMARK_COMBOS}
\cond SOC_AM243X || SOC_AM263X || SOC_AM263PX
 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0 nortos
 Toolchain      | ti-arm-clang
 Boards         | @VAR_BOARD_NAME_LOWER, @VAR_LP_BOARD_NAME_LOWER
 Example folder | examples/benchmarks/coremark_benchmark
\endcond

\cond SOC_AM64X || SOC_AM273X
 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0 nortos
 Toolchain      | ti-arm-clang
 Boards         | @VAR_BOARD_NAME_LOWER
 Example folder | examples/benchmarks/coremark_benchmark
\endcond
# Introduction

- This demo provides a means of measuring the performance of MCUs (Microcontrollers) and CPUs (Central Processing Units) used in embedded systems
- The demo uses linked lists pointers to exercise the memory units of the processor to find and alter data. All operations are inplace operations without using extra memory.
- The example does the following:
  1. Initializes the drivers and board
  2. Initializes the data block and runs the benchmark algorithm. Here, 2000 performance run parameters are used for coremark benchmarking. The input seeds are initialized from a source that cannot be determined at compile time, memory blocks are initialized and then the application is run and timed.
  3. The output validity is tests if the seeds are known, and the output is a single-number score which shows the performance for quick comparison.

\note
The out of box demo contains the .code in STACK and .data in TCMA memory section.
The memory placement of these sections can be changed from the syscfg memory configuration section.

# Performance statistics

\cond SOC_AM243X
Device          | Coremark/MHz
----------------|-----------
AM243x-EVM       | 7.539067
AM243x-LP        | 7.539031
\endcond

\cond SOC_AM263X
Device          | Coremark/MHz
----------------|-----------
AM263x-CC        | 3.768698
AM263x-LP        | 3.769956
\endcond

\cond SOC_AM263PX
Device          | Coremark/MHz
----------------|-----------
AM263Px-CC       | 3.769163
AM263Px-LP       | 3.769089
\endcond

\cond SOC_AM273X
Device          | Coremark/MHz
----------------|-----------
AM273x-EVM       | 3.769906
\endcond

\cond SOC_AM64X
Device          | Coremark/MHz
----------------|-----------
AM64x-EVM        | 7.539166
\endcond

# Steps to Run the Example

## Building Coremark application

- When using CCS projects to build, import the CCS project for the required combination
  and build it using the CCS project menu (see \ref CCS_PROJECTS_PAGE).
- When using makefiles to build, note the required combination and build using
  make command (see \ref MAKEFILE_BUILD_PAGE)

## Running the Coremark application
Once you have the application binary built following the aforementioned steps, Load and run the application using CCS.
 (see \ref CCS_LAUNCH_PAGE).

If you want to flash the application binary to the device, follow the steps mentioned here
 (see \ref GETTING_STARTED_FLASH).
## Sample output for Coremark example

\cond SOC_AM243X
\code
BENCHMARK START - ARM R5F - COREMARK
2K performance run parameters for coremark.
- CoreMark Size    : 666
- begin tick      : 981
- end tick      : 4975071
- Total ticks      : 4974090
- Total time (secs): 4.974090
- Iterations/Sec   : 3015.626979
- Iterations       : 15000
- Memory location  : STACK
- seedcrc          : 0xe9f5
- [0]crclist       : 0xe714
- [0]crcmatrix     : 0x1fd7
- [0]crcstate      : 0x8e3a
- [0]crcfinal      : 0x65c5
CoreMark 1.0 : 3015.626979
CoreMark/MHz :7.539067 / STACK
BENCHMARK END
\endcode
\endcond

\cond SOC_AM64X
\code
BENCHMARK START - ARM R5F - COREMARK
2K performance run parameters for coremark.
- CoreMark Size    : 666
- begin tick       : 960
- end tick         : 4974985
- Total ticks      : 4974025
- Total time (secs): 4.974025
- Iterations/Sec   : 3015.666387
- Iterations       : 15000
- Memory location  : STACK
- seedcrc          : 0xe9f5
- [0]crclist       : 0xe714
- [0]crcmatrix     : 0x1fd7
- [0]crcstate      : 0x8e3a
- [0]crcfinal      : 0x65c5
CoreMark 1.0 : 3015.666387
CoreMark/MHz :7.539166 / STACK
BENCHMARK END
\endcode
\endcond

\cond SOC_AM263X
\code
BENCHMARK START - ARM R5F - COREMARK
2K performance run parameters for coremark.
- CoreMark Size    : 666
- Begin tick       : 71
- End tick         : 9947138
- Total ticks      : 9947067
- Total time (secs): 9.947067
- Iterations/Sec   : 1507.982202
- Iterations       : 15000
- Memory location  : STACK
- seedcrc          : 0xe9f5
- [0]crclist       : 0xe714
- [0]crcmatrix     : 0x1fd7
- [0]crcstate      : 0x8e3a
- [0]crcfinal      : 0x65c5
CoreMark 1.0 : 1507.982202
CoreMark/MHz :3.769956 / STACK
BENCHMARK END
\endcode
\endcond

\cond SOC_AM263PX
\code
BENCHMARK START - ARM R5F - COREMARK
2K performance run parameters for coremark.
CoreMark Size    : 666
begin tick      : 71
end tick      : 9949230
Total ticks      : 9949159
Total time (secs): 9.949159
Iterations/Sec   : 1507.665120
Iterations       : 15000
Memory location  : STACK
seedcrc          : 0xe9f5
[0]crclist       : 0xe714
[0]crcmatrix     : 0x1fd7
[0]crcstate      : 0x8e3a
[0]crcfinal      : 0x65c5
CoreMark 1.0 : 1507.665120
CoreMark/MHz :3.769163 / STACK
BENCHMARK END
\endcode
\endcond

\cond SOC_AM273X
\code
BENCHMARK START - ARM R5F - COREMARK
- 2K performance run parameters for coremark.
- CoreMark Size    : 666
- begin tick       : 79
- end tick         : 9947276
- Total ticks      : 9947197
- Total time (secs): 9.947197
- Iterations/Sec   : 1507.962494
- Iterations       : 15000
- Memory location  : STACK
- seedcrc          : 0xe9f5
- [0]crclist       : 0xe714
- [0]crcmatrix     : 0x1fd7
- [0]crcstate      : 0x8e3a
- [0]crcfinal      : 0x65c5
CoreMark 1.0 : 1507.962494
CoreMark/MHz :3.769906 / STACK
BENCHMARK END
\endcode
\endcond

\cond SOC_AM263X
## Official Coremark score for AM263x
The official Coremark benchmark number for AM263x is 3.7550 CoreMark/MHz.

More about the official AM263x coremark score: https://www.eembc.org/viewer/?benchmark_seq=13583
\endcond