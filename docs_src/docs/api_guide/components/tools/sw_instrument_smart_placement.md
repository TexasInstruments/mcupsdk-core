# Software Instrument Smart Placement {#SW_INSTRUMENT_SMART_PLACEMENT}

[TOC]

## Introduction

This page goes over how to use software instrument smart placement.

Software instrumentation implementation make use of ti-arm-clang compiler's instrumentation feature.

\note
Make sure that ti-arm-clang compiler version is more than 3.1.0 STS.

## Short Coming of This Method

1. Will not work for C++ code.
2. All static functions are never considered.
3. Memory constraint applications.

## Steps

### 1. Recompilation of Libraries

It needs to made sure that all the source code are built using `-fprofile-instr-generate -fcoverage-mapping` flags. Therefore, it is required to recompile entire SDK with these flags. To do this, please execute the following command on SDK top level.

\code
gmake -j libs-scrub
\endcode

This will remove all compiled libraries. This will make sure to have a clean start. Then,

\code
gmake -j libs DEVICE=am263px INSTRUMENTATION_MODE=yes
\endcode

The above command will compile all the SDK libraries in `instrumentation mode` and in `release` profile. If it required to build libraries in `debug` mode then `PROFILE=debug` can be added at the end of the command.

Also, in Linux, `gmake` is to be replaced by `make`.

### 2. Recompilation of Application

Application is also required to be compiled with those flags. Use the following command to build the application:

\code
gmake scrub && gmake INSTRUMENTATION_MODE=yes
\endcode

Again, profile flag can be added as well.

\note
The binary that is generated is an instrumented binary and its size is going to be more than the size of original application size. Therefore, it might be the case that linker.cmd file needs to be changed or if memory configurator is used then it is it memory size needs to be changed.

### 3. Test Run / Profiling Data Generation

The instrumented binary needs to be run on the target. Make sure to run the application under correct scenario that would lead to good and complete profiling data generation. This is a very important step and quality and accuracy of the generated profiling data will determine quality of rest of the process.

This can be done connecting via CCS and can be read at \ref CCS_LAUNCH_PAGE

### 4. Profiling Data Extraction

1. Connect to target via CCS. Steps to do this can be found at \ref CCS_LAUNCH_PAGE
2. Open CCS scripting console `CCS Tool Bar > View > Scripting Console` and do below,

        js:> loadJSFile "C:/ti/mcu_plus_sdk/tools/smart_placement/coverage_dump.js"

   - **NOTE** replace `C:/ti/mcu_plus_sdk` with the absolute path to where the SDK is installed.

What this will do is, make a new `.cnt` file in same place as that of location of application’s `.out` file.

### 5. Extracted Data Processing

All the profiling data that is extracted, is stored in `.cnt` file. This `.cnt` file has to be processed and needs to be converted into a format that compiler will understand. To do this, type the following command:

\code
gmake coverage
\endcode

Again, if application is being built in debug mode, then make sure to provide `PROFILE` flag to the above command.

The output of this step would an ASM file with `.S` extension.

## 6. Rebuilding the Application

All that is required to be done is recompile the application with new generated ASM file. To do this, add the file in makefile's `ASMFILES_common` variable.


## Benchmark Application

\ref BENCHMARK_SMART_PLACEMENT

This demo provides a means of measuring the performance of a realistic application where the text of the application is sitting in various memory locations and the data is sitting in On-Chip-Memory RAM (referred to as OCM, OCMC or OCMRAM).
