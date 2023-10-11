# Dhrystone Benchmark {#EXAMPLES_DHRYSTONE}

[TOC]

Dhrystone is a synthetic benchmark that measures and compares processor performance.

Read more about Dhrystone benchmark here: 

# Supported Combinations {#EXAMPLES_DHRYSTONE_COMBOS}
\cond SOC_AM243X || SOC_AM263X || SOC_AM263PX
 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0 nortos
 Toolchain      | ti-arm-clang
 Boards         | @VAR_BOARD_NAME_LOWER, @VAR_LP_BOARD_NAME_LOWER
 Example folder | examples/benchmarks/dhrystone_benchmark
\endcond

\cond SOC_AM64X || SOC_AM273X
 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0 nortos
 Toolchain      | ti-arm-clang
 Boards         | @VAR_BOARD_NAME_LOWER
 Example folder | examples/benchmarks/dhrystone_benchmark
\endcond

# Introduction

The Dhrystone benchmark program contains 52 assignments, 33 control statements and 17 procedures/ function calls. Unlike Whetstone benchmarking, the program doesn't contain floating point operation and concentrates on string handling and standard code, heavily influenced by hardware and software design, compiler and linker options, code optimizing, cache memory, wait states and integer data types.
103 statements are dynamically executed. The output of the Dhrystone program is 

# Performance statistics

\cond SOC_AM243X
Device          | Normalised MIPS/MHz
----------------|-----------
AM243x-EVM      | 1.9675
AM243x-LP       | 1.9675
\endcond

\cond SOC_AM263X
Device          | Normalised MIPS/MHz
----------------|-----------
AM263x-CC       | 1.9678
AM263x-LP       | 1.9678
\endcond

\cond SOC_AM263PX
Device          | Normalised MIPS/MHz
----------------|-----------
AM263Px-CC      | 1.9610
AM263Px-LP      | 1.9610
\endcond

\cond SOC_AM273X
Device          | Normalised MIPS/MHz
----------------|-----------
AM273x-EVM      | 1.9677
\endcond

\cond SOC_AM64X
Device          | Normalised MIPS/MHz
----------------|-----------
AM64x-EVM       | 1.9607
\endcond

# Steps to Run the Example

## Building Dhrystone benchmark application

- When using CCS projects to build, import the CCS project for the required combination
  and build it using the CCS project menu (see \ref CCS_PROJECTS_PAGE).
- When using makefiles to build, note the required combination and build using
  make command (see \ref MAKEFILE_BUILD_PAGE)

## Running the Dhrystone benchmark application
Once you have the application binary built following the aforementioned steps, Load and run the application using CCS.
 (see \ref CCS_LAUNCH_PAGE).

If you want to flash the application binary to the device, follow the steps mentioned here
 (see \ref GETTING_STARTED_FLASH).
## Sample output for Dhrystone benchmark example

\cond SOC_AM263X
\code
BENCHMARK START - ARM R5F - DHRYSTONE
status 0

Dhrystone Benchmark, Version 2.1 (Language: C)

- BEGIN cycle count:                         34
- END Cycle count:                           144614397
- USER cycle count:                          144614363
- Usertime in sec:                           0.723072
- Microseconds for one run through Dhrystone:   1.4
- Dhrystones per Second:                     691494.2

Normalized MIPS/MHz:                         1.9678
BENCHMARK END
\endcode
\endcond

\cond SOC_AM64X
\code
BENCHMARK START - ARM R5F - DHRYSTONE

Dhrystone Benchmark, Version 2.1 (Language: C)

- BEGIN cycle count:                         58
- END Cycle count:                           145142816
- USER cycle count:                          145142758
- Usertime in sec:                           0.725714
- Microseconds for one run through Dhrystone:   1.5
- Dhrystones per Second:                     688976.9

Normalized MIPS/MHz:                         1.9607
BENCHMARK END
\endcode
\endcond

\cond SOC_AM243X
\code
BENCHMARK START - ARM R5F - DHRYSTONE

Dhrystone Benchmark, Version 2.1 (Language: C)

- BEGIN cycle count:                         61
- END Cycle count:                           144641715
- USER cycle count:                          144641654
- Usertime in sec:                           0.723208
- Microseconds for one run through Dhrystone:   1.4
- Dhrystones per Second:                     691363.8

Normalized MIPS/MHz:                         1.9675
BENCHMARK END
\endcode
\endcond

\cond SOC_AM263PX
\code
BENCHMARK START - ARM R5F - DHRYSTONE
status 0

Dhrystone Benchmark, Version 2.1 (Language: C)

- BEGIN cycle count:                         7
- END Cycle count:                           145117762
- USER cycle count:                          145117755
- Usertime in sec:                           0.725589
- Microseconds for one run through Dhrystone:   1.5
- Dhrystones per Second:                     689095.5

Normalized MIPS/MHz:                         1.9610
BENCHMARK END
\endcode
\endcond

\cond SOC_AM273X
\code
BENCHMARK START - ARM R5F - DHRYSTONE
status 0

Dhrystone Benchmark, Version 2.1 (Language: C)

- BEGIN cycle count:                         29
- END Cycle count:                           144623211
- USER cycle count:                          144623182
- Usertime in sec:                           0.723116
- Microseconds for one run through Dhrystone:   1.4
- Dhrystones per Second:                     691452.1

Normalized MIPS/MHz:                         1.9677
BENCHMARK END
\endcode
\endcond