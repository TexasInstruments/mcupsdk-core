# Release Notes 09.03.00 {#RELEASE_NOTES_09_03_00_PAGE}

[TOC]

\attention Also refer to individual module pages for more details on each feature, unsupported features, important usage guidelines.

\note The examples will show usage of SW modules and APIs on a specific CPU instance and OS combination. \n
      Unless explicitly noted otherwise, the SW modules would work in both FreeRTOS and no-RTOS environment. \n

## New in this Release

Feature                                                          | Module
-----------------------------------------------------------------|--------------------------
SBL with NULL, UART, MMCSD, OSPI and PCIe boot                   | Bootloader
UART Uniflash support                                            | Bootloader
OSPI driver support                                              | OSPI
MCSPI driver support                                             | McSPI
MMCSD driver support                                             | MMCSD
PCIE driver support                                              | PCIE


## Device and Validation Information

SOC   | Supported CPUs  | EVM                                             | Host PC
------|-----------------|-------------------------------------------------|-----------------------------------
AM65x | R5F             | AM65x IDK (referred to as am65x-idk in code)    | Windows 10 64b or Ubuntu 18.04 64b



## Dependent Tools and Compiler Information

Tools                   | Supported CPUs | Version
------------------------|----------------|-----------------------
Code Composer Studio    | R5F            | @VAR_CCS_VERSION
SysConfig               | R5F            | @VAR_SYSCFG_VERSION, build @VAR_SYSCFG_BUILD
TI ARM CLANG            | R5F            | @VAR_TI_ARM_CLANG_VERSION
FreeRTOS Kernel         | R5F            | @VAR_FREERTOS_KERNEL_VERSION

## Key Features

### OS Kernel

OS              | Supported CPUs  | SysConfig Support | Key features tested                                             | Key features not tested / NOT supported
----------------|-----------------|-------------------|-----------------------------------------------------------------|----------------------------------------
FreeRTOS Kernel | R5F             | NA                | Task, Task notification, interrupts, semaphores, mutexs, timers | Task load measurement using FreeRTOS run time statistics APIs. Limited support for ROV features.
NO RTOS         | R5F             | NA                | See **Driver Porting Layer (DPL)** below                        | -

### Driver Porting Layer (DPL)

Module            | Supported CPUs  | SysConfig Support | OS support       | Key features tested                                           | Key features not tested / NOT supported
------------------|-----------------|-------------------|------------------|---------------------------------------------------------------|----------------------------------------
Address Translate | R5F             | YES               | FreeRTOS, NORTOS | Use RAT to allow M3F access to peripheral address space       | -
Clock             | R5F             | YES               | FreeRTOS, NORTOS | Tick timer at user specified resolution, timeouts and delays  | -
CycleCounter      | R5F             | NA                | FreeRTOS, NORTOS | Measure CPU cycles using CPU specific internal counters       | -
Debug             | R5F             | YES               | FreeRTOS, NORTOS | Logging and assert to any combo of: UART, CCS, shared memory  | -
Heap              | R5F             | NA                | FreeRTOS, NORTOS | Create arbitrary heaps in user defined memory segments        | -
Hwi               | R5F             | YES               | FreeRTOS, NORTOS | Interrupt register, enable/disable/restore                    | -
MPU               | R5F             | YES               | FreeRTOS, NORTOS | Setup MPU and control access to address space                 | -
Semaphore         | R5F             | NA                | FreeRTOS, NORTOS | Binary, Counting Semaphore, recursive mutexs with timeout     | -
Task              | R5F             | NA                | FreeRTOS         | Create, delete tasks                                          | -
Timer             | R5F             | YES               | FreeRTOS, NORTOS | Configure arbitrary timers                                    | -

### Secondary Bootloader (SBL)

Module     | Supported CPUs  | SysConfig Support | OS support       | Key features tested                                                                             | Key features not tested / NOT supported
-----------|-----------------|-------------------|------------------|-------------------------------------------------------------------------------------------------|----------------------------------------
Bootloader | R5FSS0-0        | YES               | NORTOS           | Boot modes: UART, OSPI, SD, PCIe. All R5F's NORTOS/FreeRTOS, A53 Linux boot. RPRC, multi-core image format, DDR init  | -

### SOC Device Drivers

Peripheral        | Supported CPUs  | SysConfig Support |DMA Supported | Key features tested                                           | Key features not tested / NOT supported
------------------|-----------------|-------------------|--------------|--------------------------------------------------------------------------------------------|-----------------------------------------
GPIO              | R5F             | YES               | No           | Basic input/output, GPIO as interrupt                         | -
IPC Notify        | R5F             | YES               | No           | Low latency IPC between RTOS/NORTOS CPUs                      | -
IPC Rpmsg         | R5F             | YES               | No           | RPMessage protocol based IPC, R5F and Linux A53 cores         | -
I2C               | R5F             | YES               | No           | Controller mode, basic read/write, polling and interrupt mode | -
MCSPI             | R5F             | YES               | YES          | Controller mode, basic read/write, polling, interrupt and DMA mode | -
MMCSD             | R5F             | YES               | YES          | Raw read/write and file I/O on MMCSD0 eMMC, and MMCSD1 SD. eMMC tested till HS200 mode (8-bit data), SD tested till SD HS mode (4-bit, 25 MHz)  | Interrupt mode not tested
OSPI              | R5F             | YES               | YES          | Read direct, Write indirect, Read/Write commands, DMA for read, PHY Mode | Interrupt mode not tested
PCIe              | R5F             | YES               | No           | Buffer Transfer between EP and RC modes                       | Legacy interrupt, MSI and MSIx capability
Pinmux            | R5F             | YES               | No           | Tested with multiple peripheral pinmuxes                      | -
Sciclient         | R5F             | YES               | No           | Tested with clock setup, module on/off                        | -
UART              | R5F             | YES               | No           | Basic read/write, polling, interrupt mode                     | -
UDMA              | R5F             | YES               | No           | Basic memory copy, SW trigger, Chaining                       | -

## Fixed Issues

NA

## Known Issues

NA

## Limitations

NA
