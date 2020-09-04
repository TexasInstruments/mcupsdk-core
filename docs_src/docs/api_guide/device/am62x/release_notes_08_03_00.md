# Release Notes 08.03.00 {#RELEASE_NOTES_08_03_00_PAGE}

[TOC]

\attention Also refer to individual module pages for more details on each feature, unsupported features, important usage guidelines.

\note The examples will show usage of SW modules and APIs on a specific CPU instance and OS combination. \n
      Unless explicitly noted otherwise, the SW modules would work in both FreeRTOS and no-RTOS environment. \n

## New in this Release

Feature                                                                                         | Module
------------------------------------------------------------------------------------------------|-----------------------------------
32 Task Priority Levels for FreeRTOS Tasks                                                      | DPL, FreeRTOS

## Device and Validation Information

\cond SOC_AM64X
SOC   | Supported CPUs  | EVM                                             | Host PC
------|-----------------|-------------------------------------------------|-----------------------------------
AM62x | M4F             | AM62x SK EVM (referred to as am62x-sk in code) | Windows 10 64b or Ubuntu 18.04 64b
\endcond


## Dependent Tools and Compiler Information

Tools                   | Supported CPUs | Version
------------------------|----------------|-----------------------
Code Composer Studio    | M4F            | @VAR_CCS_VERSION
SysConfig               | M4F            | @VAR_SYSCFG_VERSION, build @VAR_SYSCFG_BUILD
TI ARM CLANG            | M4F            | @VAR_TI_ARM_CLANG_VERSION
FreeRTOS Kernel         | M4F            | @VAR_FREERTOS_KERNEL_VERSION

## Key Features

### OS Kernel

OS              | Supported CPUs  | SysConfig Support | Key features tested                                             | Key features not tested / NOT supported
----------------|-----------------|-------------------|-----------------------------------------------------------------|----------------------------------------
FreeRTOS Kernel | M4F             | NA                | Task, Task notification, interrupts, semaphores, mutexs, timers | Task load measurement using FreeRTOS run time statistics APIs. Limited support for ROV features.
NO RTOS         | M4F             | NA                | See **Driver Porting Layer (DPL)** below                        | -

### Driver Porting Layer (DPL)

Module            | Supported CPUs  | SysConfig Support | OS support       | Key features tested                                           | Key features not tested / NOT supported
------------------|-----------------|-------------------|------------------|---------------------------------------------------------------|----------------------------------------
Address Translate | M4F             | YES               | FreeRTOS, NORTOS | Use RAT to allow M4F access to peripheral address space       | -
Clock             | M4F             | YES               | FreeRTOS, NORTOS | Tick timer at user specified resolution, timeouts and delays  | -
CycleCounter      | M4F             | NA                | FreeRTOS, NORTOS | Measure CPU cycles using CPU specific internal counters       | -
Debug             | M4F             | YES               | FreeRTOS, NORTOS | Logging and assert to any combo of: UART, CCS, shared memory  | -
Heap              | M4F             | NA                | FreeRTOS, NORTOS | Create arbitrary heaps in user defined memory segments        | -
Hwi               | M4F             | YES               | FreeRTOS, NORTOS | Interrupt register, enable/disable/restore                    | -
MPU               | M4F             | YES               | FreeRTOS, NORTOS | Setup MPU and control access to address space                 | -
Semaphore         | M4F             | NA                | FreeRTOS, NORTOS | Binary, Counting Semaphore, recursive mutexs with timeout     | -
Task              | M4F             | NA                | FreeRTOS         | Create, delete tasks                                          | -
Timer             | M4F             | YES               | FreeRTOS, NORTOS | Configure arbitrary timers                                    | -

### SOC Device Drivers

Peripheral        | Supported CPUs  | SysConfig Support | Key features tested                                           | Key features not tested / NOT supported
------------------|-----------------|-------------------|---------------------------------------------------------------|----------------------------------------
IPC Rpmsg         | M4F             | YES               | RPMessage protocol based IPC, M4F and Linux A53 cores         | -
GPIO              | M4F             | YES               | Basic input/output, GPIO as interrupt                         | -
I2C               | M4F             | YES               | Controller mode, basic read/write, polling and interrupt mode     | -
UART              | M4F             | YES               | Basic read/write, polling, interrupt mode,                    | -
MCAN              | M4F             | YES               | RX, TX, interrupt and polling mode                            | -

## Fixed Issues

NA

## Known Issues

NA

## Limitations

<table>
<tr>
    <th> ID
    <th> Head Line
    <th> Module
    <th> Reported in release
    <th> Workaround
</tr>
<tr>
    <td> MCUSDK-208
    <td> gmake with -j can sometimes lock up Windows command prompt
    <td> Build
    <td> 7.3.0
    <td> Use bash for windows as part of git for windows or don't use -j option
</tr>
<tr>
    <td> SITSW-1382
    <td> MCU UART prints a single character of junk data on the first print after the power cycle of the EVM
    <td> MCU UART
    <td> 8.3.0
    <td> None
</tr>
</table>
