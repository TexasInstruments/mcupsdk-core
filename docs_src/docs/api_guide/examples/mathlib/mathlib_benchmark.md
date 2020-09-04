# MATHLIB Benchmark {#EXAMPLES_MATHLIB_BENCHMARK}

[TOC]

# Introduction

This example calls the mathlib trignometric functions for different angles between 0 to 2Pi

- Calls trignometric functions with SDK mathlib APIs and CLANG mathlib APIs
- Compares the performance between the two versions and prints the time taken and the error

# Supported Combinations {#EXAMPLES_MATHLIB_BENCHMARK_COMBOS}


 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0 nortos
 Toolchain      | ti-arm-clang
 Board          | @VAR_BOARD_NAME_LOWER
 Example folder | examples/mathlib/benchmark/mathlib_benchmark.c

# Steps to Run the Example

- **When using CCS projects to build**, import the CCS project for the required combination
  and build it using the CCS project menu (see \ref CCS_PROJECTS_PAGE).
- **When using makefiles to build**, note the required combination and build using
  make command (see \ref MAKEFILE_BUILD_PAGE)
- Launch a CCS debug session and run the executable, see \ref CCS_LAUNCH_PAGE

# See Also

\ref MATHLIB_DRIVER

# Sample Output

Shown below is a sample output when the application is run,

\code
Trig Benchmark Test

Function        | Err           | Max Cycles Mathlib (mcusdk)   | avg cycles Mathlib (mcusdk)   | max cycles mathlib (clang)    | avg cycles mathlib (clang)    |
----------------|---------------|-----------------------|-----------------------|-----------------------|-----------------------|
sin             |0.0000007150   | 37                    | 37.782001             | 740                   | 297.473999            |
cos             |0.0000002870   | 48                    | 48.554001             | 717                   | 290.843994            |
sincos sin      |0.0000001790   | 70                    | 70.778000             | 551                   | 289.044006            |
sincos cos      |0.0000001900   |                       |                       |                       |                       |
asin            |0.0000003430   | 68                    | 68.755997             | 1064                  | 441.868011            |
acos            |0.0000004770   | 71                    | 70.776001             | 613                   | 394.201996            |
atan            |0.0000005360   | 79                    | 79.736000             | 649                   | 388.132019            |
atan2           |0.0000007150   | 110                   | 101.139999            | 626                   | 490.440002            |

Trig Benchmark Test Completed!!
All tests have passed!!
\endcode

