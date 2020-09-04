# Release Notes 07.03.03 {#RELEASE_NOTES_07_03_03_PAGE}

[TOC]

\attention Also refer to individual module pages for more details on each feature, unsupported features, important usage guidelines.

\note The examples will show usage of SW modules and APIs on a specific CPU instance and OS combination. \n
      Unless explicitly noted otherwise, the SW modules would work in both FreeRTOS and no-RTOS environment. \n
      Unless explicitly noted otherwise, the SW modules would work on any of the R5F's present on the SOC. \n

## Device and Validation Information

\cond SOC_AM263X
SOC   | Supported CPUs  | EVM                                             | Host PC
------|-----------------|-------------------------------------------------|-----------------------------------
AM263x| R5F             | -                                               | Windows 10 64b or Ubuntu 18.04 64b
\endcond


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
FreeRTOS POSIX  | R5F             | NA                | pthread, mqueue, semaphore, clock                               | -
NO RTOS         | R5F             | NA                | See **Driver Porting Layer (DPL)** below                        | -

### Driver Porting Layer (DPL)

Module            | Supported CPUs  | SysConfig Support | OS support       | Key features tested                                           | Key features not tested / NOT supported
------------------|-----------------|-------------------|------------------|---------------------------------------------------------------|----------------------------------------
Cache             | R5F             | YES               | FreeRTOS, NORTOS | Cache write back, invalidate, enable/disable                  | -
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

Module     | Supported CPUs  | SysConfig Support | OS support       | Key features tested                                                         | Key features not tested / NOT supported
-----------|-----------------|-------------------|------------------|-----------------------------------------------------------------------------|----------------------------------------
-          | -               | -                 | -                | -                                                                           | -

### SOC Device Drivers

Peripheral   | Supported CPUs | SysConfig Support | Key features tested                                                                             | Key features not tested / NOT supported
-------------|----------------|-------------------|-------------------------------------------------------------------------------------------------|----------------------------------------
ADC          | R5F            | YES               | Software triggered conversion                                                                   | -
CMPSS        | R5F            | YES               | Only compiled                                                                                   | -
DAC          | R5F            | YES               | Constant voltage and square wave generation                                                     | -
ECAP         | R5F            | YES               | ECAP APWM generation mode                                                                       | -
EDMA         | R5F            | YES               | DMA transfer using interrupt and polling mode, QDMA Transfer, Channel Chaining, PaRAM Linking   | -
EPWM         | R5F            | YES               | Different Frequency, Duty cycle, interrupt mode                                                 | -
EQEP         | R5F            | YES               | Frequency measurement                                                                           | -
I2C          | R5F            | YES               | Controller mode, basic read/write                                                                   | -
IPC Notify   | R5F            | YES               | Low latency IPC between RTOS/NORTOS CPUs                                                        | -
IPC Rpmsg    | R5F            | YES               | RPMessage protocol based IPC, All R5F cores                                                     | M4F core
MCAN         | R5F            | YES               | Only compiled                                                                                   | -
MPU Firewall | R5F            | YES               | Only compiled                                                                                   | -
QSPI         | R5F            | YES               | Read direct, Write indirect, Read/Write commands, DMA for read                                  | -
SDFM         | R5F            | YES               | Only compiled                                                                                   | -
SOC          | R5F            | YES               | lock/unlock MMRs, get CPU name, clock enable, set Hz                                            | -
SPINLOCK     | R5F            | NA                | Lock, unlock HW spinlocks                                                                       | -
UART         | R5F            | YES               | Basic read/write, polling, interrupt mode                                                       | HW flow control not tested, DMA mode not supported
WATCHDOG     | R5F            | YES               | Only compiled                                                                                   | -

### Board Device Drivers

Peripheral | Supported CPUs | SysConfig Support | Key features tested                                         | Key features not tested
-----------|----------------|-------------------|-------------------------------------------------------------|------------------------
-          | -              | -                 | -                                                           | -


### CMSIS

Module                      | Supported CPUs | SysConfig Support | OS Support        | Key features tested                                                                         | Key features not tested
----------------------------|----------------|-------------------|-------------------|---------------------------------------------------------------------------------------------|------------------------
-                           | -              | -                 | -                 | -                                                                                           |  -

### Motor Control

Module                      | Supported CPUs | SysConfig Support | OS Support        | Key features tested                             | Key features not tested
----------------------------|----------------|-------------------|-------------------|-------------------------------------------------|------------------------
-                           | -              | -                 | -                 | -                                               |  -

### Networking

Module                      | Supported CPUs | SysConfig Support | OS Support  | Key features tested                                 | Key features not tested
----------------------------|----------------|-------------------|-------------|-----------------------------------------------------|------------------------
-                           | -              | -                 | -           | -                                                   |  -


### Demos

Module                      | Supported CPUs | SysConfig Support | OS Support        | Key features tested                             | Key features not tested
----------------------------|----------------|-------------------|-------------------|-------------------------------------------------|------------------------
-                           | -              | -                 | -                 | -                                               |  -

## Fixed Issues

NA

## Known Issues

<table>
<tr>
    <th> ID
    <th> Head Line
    <th> Module
    <th> Reported in release
    <th> Workaround
</tr>
<tr>
    <td> -
    <td> -
    <td> -
    <td> -
    <td> -
</tr>

</table>

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
    <td> -
    <td> -
    <td> -
    <td> -
    <td> -
</tr>
</table>

## Upgrade and Compatibility Information

Not Applicable

### SOC Device Drivers

<table>
<tr>
    <th> Module
    <th> Affected API
    <th> Change
    <th> Additional Remarks
</tr>
<tr>
    <td> -
    <td> -
    <td> -
    <td> -
</tr>
</table>


