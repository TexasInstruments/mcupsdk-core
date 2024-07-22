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
3. Internal memory constraint applications may not be able to use this.

## Steps

### 1. Recompilation of Libraries

It needs to made sure that all the source code are built using `-fprofile-instr-generate -fcoverage-mapping` flags. Therefore, it is required to recompile entire SDK with these flags. To do this, please execute the following command on SDK top level.

> gmake -j libs-scrub DEVICE=@VAR_SOC_NAME_LOWER

This will remove all compiled libraries. This will make sure to have a clean start. Then,

> gmake -j libs DEVICE=@VAR_SOC_NAME_LOWER INSTRUMENTATION_MODE=yes

The above command will compile all the SDK libraries in `instrumentation mode` and in `release` profile. If it required to build libraries in `debug` mode then `PROFILE=debug` can be added at the end of the command.

Also, in Linux, `gmake` is to be replaced by `make`. The above commands assumes that current working directory is defined in `MCU_PLUS_SDK_PATH` environment variable.

### 2. Recompilation of Application

Application is also required to be compiled with those flags.
\cond SOC_AM263PX || SOC_AM261X
\note
The binary that is generated is an instrumented binary and its size is going to be more than the size of original application size. Therefore, it might be the case that linker.cmd file needs to be changed or if memory configurator is used then memory size needs to be changed.

For the changes that are required to be done for in memory configurator/linker script for instrumentation, please go through the following steps documented at \ref INSTRUMENTED_APPLICATION_SPECIAL_SECTION_ADD.

\endcond
\cond  SOC_AM263X || SOC_AM243X

The binary that is generated is an instrumented binary and its size is going to be more than the size of original application size. Therefore, it might be the case that linker.cmd file needs to be changed.

For the changes that are required to be done for in linker script for instrumentation, please go through the following steps documented at \ref INSTRUMENTED_APPLICATION_SPECIAL_SECTION_ADD.


\endcond

\note
Although these changes are already at-least in empty SDK examples. So, changes can be copied and pasted directly from it and any more changes can be done on top of that.

Once those changes are done, execute following commands:

\code
gmake -C path/to/folder/with/application/makefile scrub
gmake -C path/to/folder/with/application/makefile INSTRUMENTATION_MODE=yes
\endcode

The above two commands assume that current working directory is defined in `MCU_PLUS_SDK_PATH` environment variable. Again, profile flag can be added as well to above commands.

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
gmake -C path/to/folder/with/application/makefile coverage
\endcode

Again, if application is being built in debug mode, then make sure to provide `PROFILE` flag to the above command.

The output of this step would an ASM file with `.S` extension.

## 6. Linker update related to Smart Placement

Update linker as described at \ref SMART_PLACEMENT_LINKER_CHANGE.

\note
Although these changes are already at-least in \ref BENCHMARK_SMART_PLACEMENT SDK examples. So, changes can be copied and pasted directly from it and any more changes can be done on top of that.

## 7. Rebuilding the Application

Recompile the application with new generated ASM file. To do this, add the file in makefile's `ASMFILES_common` variable. Note that, now INSTRUMENTATION_MODE flag is not required. So rebuild libs and application without these flags.

## Benchmark Application

This demo provides a means of measuring the performance of a realistic application where the text of the application is sitting in various memory locations and the data is sitting in On-Chip-Memory RAM (referred to as OCM, OCMC or OCMRAM).

\note
All above steps are already done and generated ASM file has been renamed to `annotations.S`.

In case if above steps are required to run, then all linker related changes can be skipped as those are already taken into account.

\ref BENCHMARK_SMART_PLACEMENT