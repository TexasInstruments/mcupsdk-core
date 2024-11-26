# Sram Overlay Benchmark {#EXAMPLES_SRAM_OVERLAY}

[TOC]

# Supported Combinations {#EXAMPLES_SRAM_OVERLAY_COMBOS}
\cond SOC_AM263PX

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0 nortos
 Toolchain      | ti-arm-clang
 Boards         | @VAR_BOARD_NAME_LOWER, @VAR_LP_BOARD_NAME_LOWER
 Example folder | examples/benchmarks/memory_access_latency

\endcond

# Introduction

- This demo provides a rough measurement of the CPU cycles taken for a given code execution which is located in different memory regions (L2OCRAM and Flash memory) and also shows the performance when utilizing the overlay feature.
- The example does the following:
  1. Initializes the drivers and board
  2. Initializes the cycle counters for counting the CPU cycles.
  3. The triggers the FLC(fast Local Copy) HW to copy the code from flash to RAM.
  4. Three iterations are done one with code executing from ram next from flash(XIP) and third with FLC overlay.
  5. Average cycles for code execution is calculated in example.

# Steps to Run the Example

## Building SRAM Overlay application

- When using CCS projects to build, import the CCS project for the required combination
  and build it using the CCS project menu (see \ref CCS_PROJECTS_PAGE).
- When using makefiles to build, note the required combination and build using
  make command (see \ref MAKEFILE_BUILD_PAGE)

## Running the SRAM Overlay application

Flash the application binary to the device, follow the steps mentioned here
 (see \ref GETTING_STARTED_FLASH).

## Sample output for SRAM Overlay example


\cond SOC_AM263PX
\code
Starting OSPI Bootloader ... 
[BOOTLOADER_PROFILE] Boot Media       : NOR SPI FLASH 
KPI_DATA: [BOOTLOADER_PROFILE] Boot Media Clock : 133.333 MHz 
KPI_DATA: [BOOTLOADER_PROFILE] Boot Image Size  : 98 KB 
[BOOTLOADER_PROFILE] Cores present    : 
r5f0-0
KPI_DATA: [BOOTLOADER PROFILE] System_init                      :        647us 
KPI_DATA: [BOOTLOADER PROFILE] Drivers_open                     :        118us 
KPI_DATA: [BOOTLOADER PROFILE] LoadHsmRtFw                      :       8500us 
KPI_DATA: [BOOTLOADER PROFILE] Board_driversOpen                :      33858us 
KPI_DATA: [BOOTLOADER PROFILE] CPU load                         :         81us 
KPI_DATA: [BOOTLOADER_PROFILE] SBL Total Time Taken             :      45369us 

Image loading done, switching to application ...
T1 SRAM: 74458, 9962, 801, 5700
T1 XIP : 141912, 10123, 826, 5837
T1 OVL : 1256926, 204701, 756, 132094
T2 SRAM: 40798, 6634, 410, 4150
T2 XIP : 64829, 6634, 411, 4157
T2 XIP : 1044332, 201263, 321, 130260
T3 SRAM: 25889, 5354, 263, 3574
T3 XIP : 25877, 5354, 260, 3583
T3 XIP : 979075, 199876, 241, 129730
T4 SRAM: 17505, 5608, 315, 3769
T4 XIP : 20543, 5608, 314, 3767
T4 XIP : 985577, 200399, 210, 130078
All tests have passed!!

BENCHMARK END
\endcode
\endcond

