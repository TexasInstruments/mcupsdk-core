# Memory Access Latency Benchmark {#EXAMPLES_MEMORY_ACCESS_LATENCY}

[TOC]

# Supported Combinations {#EXAMPLES_MEMORY_ACCESS_LATENCY_COMBOS}
\cond SOC_AM263X || SOC_AM263PX
 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0 nortos
 Toolchain      | ti-arm-clang
 Boards         | @VAR_BOARD_NAME_LOWER, @VAR_LP_BOARD_NAME_LOWER
 Example folder | examples/benchmarks/memory_access_latency
\endcond

# Introduction

- This demo provides a rough measurement of the time taken for a Read or Write instruction to be executed at different memory regions (L2OCRAM, TCM, Flash memory).
- The demo uses simple assembly load and store commands to read and write the data at a specific memory location in different regions.
- The example does the following:
  1. Initializes the drivers and board
  2. Calls the Read and Write functions at TCM, L2OCRAM, Flash, Non self TCM (peripheral address) for 32-bit and 64-bit access.
  3. The Read and Write 32-bit functions consist of 32 commands for reading from and writing to 32-bit (4-byte) registers.
  4. The Read and Write 64-bit functions consist of 32 commands for reading from and writing to two 32-bit (4-byte) registers.
  5. Average cycles per byte is calculated in example.

# Performance statistics

\cond SOC_AM263X

## AM263x
Operation | Cycles/byte |
----------------|-----------|
Read (32-bit access) - TCM          |  0.359375 |
Read (32-bit access) - L2OCRAM      | 5.609375 |
Read (32-bit access) - Non self TCM | 7.625000 |
Read (32-bit access) - FLASH        | 87.750000 |
Write (32-bit access) - TCM          | 0.351562 |
Write (32-bit access) - L2OCRAM      | 0.367188 |
Write (32-bit access) - Non self TCM | 0.367188 |
Read (64-bit access) - TCM          | 0.171875 |
Read (64-bit access) - L2OCRAM      | 2.804688 |
Read (64-bit access) - Non self TCM | 3.812500 |
Read (64-bit access) - FLASH        | 49.406250 |
Write (64-bit access) - TCM          | 0.171875 |
Write (64-bit access) - L2OCRAM      | 0.171875 |
Write (64-bit access) - Non self TCM | 0.171875 |

\endcond

\cond SOC_AM263PX

## AM263Px-CC
Operation | Cycles/byte |
----------------|-----------|
Read (32-bit access) - TCM          | 0.359375 |
Read (32-bit access) - L2OCRAM      | 5.109375 |
Read (32-bit access) - Non self TCM | 9.625000 |
Read (32-bit access) - FLASH        | 55.390625 |
Write (32-bit access) - TCM          | 0.351562 |
Write (32-bit access) - L2OCRAM      | 0.367188 |
Write (32-bit access) - Non self TCM | 0.367188 |
Read (64-bit access) - TCM          | 0.171875 |
Read (64-bit access) - L2OCRAM      | 2.554688 |
Read (64-bit access) - Non self TCM |  4.812500 |
Read (64-bit access) - FLASH        | 28.437500 |
Write (64-bit access) - TCM          | 0.171875 |
Write (64-bit access) - L2OCRAM      | 0.171875 |
Write (64-bit access) - Non self TCM | 0.171875 |

\endcond

# Steps to Run the Example

## Building Memory access latency application

- When using CCS projects to build, import the CCS project for the required combination
  and build it using the CCS project menu (see \ref CCS_PROJECTS_PAGE).
- When using makefiles to build, note the required combination and build using
  make command (see \ref MAKEFILE_BUILD_PAGE)

## Running the Memory access latency application

Flash the application binary to the device, follow the steps mentioned here
 (see \ref GETTING_STARTED_FLASH).

## Sample output for Memory access latency example

\cond SOC_AM263X
\code
BENCHMARK START - ARM R5F - Memory Access latency

[32-bit WRITE] Self TCM Access Latency: 0.351562 cycles/byte
[32-bit WRITE] L2OCRAM Access Latency: 0.367188 cycles/byte
[32-bit WRITE] Non Self TCM Access Latency: 0.367188 cycles/byte

[32-BIT READ] Self TCM Access Latency: 0.359375 cycles/byte
[32-BIT READ] L2OCRAM Access Latency: 5.609375 cycles/byte
[32-BIT READ] Non-Self TCM Access Latency: 7.625000 cycles/byte
[32-BIT READ] Flash (Memory Map Mode) Access Latency: 87.750000 cycles/byte

[64-bit WRITE] Self TCM Access Latency: 0.171875 cycles/byte
[64-bit WRITE] L2OCRAM Access Latency: 0.171875 cycles/byte
[64-bit WRITE] Non-Self TCM Access Latency: 0.171875 cycles/byte

[64-BIT READ] Self TCM Access Latency: 0.171875 cycles/byte
[64-BIT READ] L2OCRAM Access Read Latency: 2.804688 cycles/byte
[64-BIT READ] Non-Self TCM Access Read Latency: 3.812500 cycles/byte
[64-BIT READ] Flash (Memory Map Mode) Access Read Latency: 49.406250 cycles/byte

BENCHMARK END
\endcode
\endcond

\cond SOC_AM263PX
\code
BENCHMARK START - ARM R5F - Memory Access latency

[32-bit WRITE] Self TCM Access Latency: 0.351562 cycles/byte
[32-bit WRITE] L2OCRAM Access Latency: 0.367188 cycles/byte
[32-bit WRITE] Non Self TCM Access Latency: 0.367188 cycles/byte

[32-BIT READ] Self TCM Access Latency: 0.359375 cycles/byte
[32-BIT READ] L2OCRAM Access Latency: 5.109375 cycles/byte
[32-BIT READ] Non-Self TCM Access Latency: 9.625000 cycles/byte
[32-BIT READ] Flash (Memory Map Mode) Access Latency: 55.390625 cycles/byte

[64-bit WRITE] Self TCM Access Latency: 0.171875 cycles/byte
[64-bit WRITE] L2OCRAM Access Latency: 0.171875 cycles/byte
[64-bit WRITE] Non-Self TCM Access Latency: 0.171875 cycles/byte

[64-BIT READ] Self TCM Access Latency: 0.171875 cycles/byte
[64-BIT READ] L2OCRAM Access Read Latency: 2.554688 cycles/byte
[64-BIT READ] Non-Self TCM Access Read Latency: 4.812500 cycles/byte
[64-BIT READ] Flash (Memory Map Mode) Access Read Latency: 28.437500 cycles/byte

BENCHMARK END
\endcode
\endcond

