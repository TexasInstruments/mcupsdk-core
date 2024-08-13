# Release Notes 09.01.00 {#RELEASE_NOTES_09_01_00_PAGE}

[TOC]

\attention Also refer to individual module pages for more details on each feature, unsupported features, important usage guidelines.

\note The examples will show usage of SW modules and APIs on a specific CPU instance and OS combination. \n
      Unless explicitly noted otherwise, the SW modules would work in both FreeRTOS and no-RTOS environment. \n

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

### SOC Device Drivers

Peripheral        | Supported CPUs  | SysConfig Support | Key features tested                                           | Key features not tested / NOT supported
------------------|-----------------|-------------------|---------------------------------------------------------------|----------------------------------------
GPIO              | R5F             | YES               | Basic input/output, GPIO as interrupt                         | -
IPC Notify        | R5F             | YES               | Low latency IPC between RTOS/NORTOS CPUs                      | -
IPC Rpmsg         | R5F             | YES               | RPMessage protocol based IPC, R5F and Linux A53 cores         | -
I2C               | R5F             | YES               | Controller mode, basic read/write, polling and interrupt mode | -
Pinmux            | R5F             | YES               | Tested with multiple peripheral pinmuxes                      | -
Sciclient         | R5F             | YES               | Tested with clock setup, module on/off                        | -
UART              | R5F             | YES               | Basic read/write, polling, interrupt mode,                    | -
UDMA              | R5F             | YES               | Basic memory copy, SW trigger, Chaining                       | -

## Fixed Issues

NA

## Known Issues

NA

## Limitations

NA
