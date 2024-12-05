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

- This demo provides a rough measurement of the CPU cycles taken for a given code execution (Measured in CPI) which is located in different memory regions (L2OCRAM and Flash memory) and also shows the performance when utilizing the overlay feature.
- The example does the following:
  1. Initializes the drivers and board
  2. Initializes the cycle counters for counting the CPU cycles.
  3. The triggers the FLC(fast Local Copy) HW to copy the code from flash to RAM.
  4. Three iterations are done one with code executing from ram next from flash(XIP) and third with FLC overlay.
  5. Average cycles for code execution is calculated in example.

## Performance optimization for overlay

\note Here, code does not wait for function to be copied entirely in the internal RAM. 

Looking at the sample output shows that, in case, when CPI of a function is less than 5, then, code should wait for function to get copied else it is really not required to wait for copy to complete and execution of the function can be done right away.

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
KPI_DATA: [BOOTLOADER_PROFILE] Boot Media       : NOR SPI FLASH 
KPI_DATA: [BOOTLOADER_PROFILE] Boot Media Clock : 133.333 MHz 
KPI_DATA: [BOOTLOADER_PROFILE] Boot Image Size  : 98 KB 
KPI_DATA: [BOOTLOADER_PROFILE] Cores present    : 
r5f0-0
KPI_DATA: [BOOTLOADER PROFILE] System_init                      :        541us 
KPI_DATA: [BOOTLOADER PROFILE] Drivers_open                     :        126us 
KPI_DATA: [BOOTLOADER PROFILE] LoadHsmRtFw                      :       9373us 
KPI_DATA: [BOOTLOADER PROFILE] Board_driversOpen                :       2751us 
KPI_DATA: [BOOTLOADER PROFILE] CPU load                         :       5308us 
KPI_DATA: [BOOTLOADER PROFILE] SBL End                          :         20us 
KPI_DATA: [BOOTLOADER_PROFILE] SBL Total Time Taken             :      18123us 

Image loading done, switching to application ...

Test started
Label          :CPU Cycles     CPU Instr.     CPI
T1 SRAM        :74444          9962           7.5 
T1 XIP         :142542         10097          14.1 
T1 OVL         :75501          10031          7.5 

T2 SRAM        :40882          6634           6.2 
T2 XIP         :65101          6634           9.8 
T2 OVL         :42501          6703           6.3 

T3 SRAM        :26013          5354           4.9 
T3 XIP         :25910          5354           4.8 
T3 OVL         :36757          5423           6.8 

T4 SRAM        :17539          5608           3.1 
T4 XIP         :21282          5608           3.8 
T4 OVL         :50720          5677           8.9 
All tests have passed!!

BENCHMARK END
\endcode
\endcond

