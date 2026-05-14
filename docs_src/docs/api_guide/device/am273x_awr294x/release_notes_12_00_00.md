# Release Notes 12.00.00 {#RELEASE_NOTES_12_00_00_PAGE}

[TOC]

\attention Also refer to individual module pages for more details on each feature, unsupported features, important usage guidelines.

\note The examples will show usage of SW modules and APIs on a specific CPU instance and OS combination. \n
      Unless noted otherwise, the SW modules would work in both FreeRTOS and NO-RTOS environment. \n
      Unless noted otherwise, the SW modules would work on any of the CPU present on the SOC. \n
      Unless noted otherwise, the SW modules would work on all supported EVMs \n

## New in this Release

Feature                                                                                         | Module
------------------------------------------------------------------------------------------------|--------------------------
**AM273x MSS_L2 Memory Split** - Optimized memory layout with 32-byte boundary alignment        | Memory, SysConfig
FreeRTOS kernel updated to 11.1.0 PATCH 1                                                      | FreeRTOS
Enhanced MQTT networking support                                                                | Networking
Improved ECC and SDL modules                                                                   | Safety

## Device and Validation Information

SOC     | Supported CPUs  | EVM                                                | Host PC
--------|-----------------|----------------------------------------------------|-----------------------------------
AM273x  | R5F, C66x       | AM273x GP EVM (referred to as am273x-evm in code)  | Windows 10 64b or Ubuntu 18.04 64b

## Tools, Compiler and Other Open Source SW Module Information

Tools / SW module       | Supported CPUs | Version
------------------------|----------------|-----------------------
Code Composer Studio    | R5F, C66x      | 20.5.0
SysConfig               | R5F, C66x      | 1.27.0
TI ARM CLANG            | R5F            | 4.0.4.LTS
TI C6000 Compiler       | C66x           | 8.5.0
FreeRTOS Kernel         | R5F, C66x      | 11.1.0 PATCH 1
DSP LIB                 | C66x           | 3.4.0.0
Mbed-TLS                | R5F            | mbedtls-2.13.1

## Key Features

### Important Memory Architecture Update

\htmlonly
<div style="background-color: #d4edda; border: 1px solid #c3e6cb; border-radius: 4px; padding: 15px; margin: 10px 0;">
<p><strong>✓ AM273x MSS_L2 Memory Split Architecture</strong></p>

<p><strong>Memory Layout (960 KB Total):</strong></p>
<ul>
<li><strong>Bootloader/System Reserved:</strong> 0x10200000 - 0x1025FFFF (384 KB) - Not available for application use</li>
<li><strong>MSS_L2_A_R5F:</strong> 0x10260000 - 0x1027FFDF (131 KB usable)</li>
<li><strong>MSS_L2_B_R5F:</strong> 0x10280000 - 0x102BFFDC (448 KB usable)</li>
</ul>

<p><strong>R5F MPU Cacheable Access Limitation:</strong> When R5F performs cacheable access, if a 32-byte cache line falls within the last 32 bytes of an MPU region, the MPU incorrectly signals an access fault, triggering prefetch abort exception. Do not place cacheable code or data at 0x1027FFF0-0x1027FFDF (Bank A) or 0x102BFFDC-0x102BFFEC (Bank B). Non-cacheable access and access from other cores (C66x) are not affected. SysConfig automatically reserves these zones in linker configuration.

</div>
\endhtmlonly

### OS Kernel

OS              | Supported CPUs  | SysConfig Support | Key features tested                                                                                                                                                 | Key features not tested / NOT supported
----------------|-----------------|-------------------|---------------------------------------------------------------------------------------------------------------------------------------------------------------------|----------------------------------------
FreeRTOS Kernel | R5F, C66x       | NA                | Task, Task notification, interrupts, semaphores, mutexes, timers, event groups. ROV views in CCS IDE, Task load measurement using FreeRTOS run time statistics APIs. | -
FreeRTOS POSIX  | R5F, C66x       | NA                | pthread, queue, semaphore, clock                                                                                                                                   | -
NO RTOS         | R5F, C66x       | NA                | See **Driver Porting Layer (DPL)** below                                                                                                                            | -

### Driver Porting Layer (DPL)

Module            | Supported CPUs  | SysConfig Support | OS support                 | Key features tested                                           | Key features not tested / NOT supported
------------------|-----------------|-------------------|----------------------------|---------------------------------------------------------------|----------------------------------------
Cache             | R5F, C66x       | YES               | FreeRTOS, safeRTOS, NORTOS | Cache write back, invalidate, enable/disable                  | -
Clock             | R5F, C66x       | YES               | FreeRTOS, safeRTOS, NORTOS | Tick timer at user specified resolution, timeouts and delays  | -
CpuId             | R5F             | NA                | FreeRTOS, NORTOS           | Verify Core ID and Cluster ID that application is running     | -
CycleCounter      | R5F, C66x       | NA                | FreeRTOS, safeRTOS, NORTOS | Measure CPU cycles using CPU specific internal counters       | -
Debug             | R5F, C66x       | YES               | FreeRTOS, safeRTOS, NORTOS | Logging and assert to any combo of: UART, CCS, shared memory  | -
Heap              | R5F, C66x       | NA                | FreeRTOS, safeRTOS, NORTOS | Create arbitrary heaps in user defined memory segments        | -
Hwi               | R5F, C66x       | YES               | FreeRTOS, safeRTOS, NORTOS | Interrupt register, enable/disable/restore                    | -
MPU               | R5F             | YES               | FreeRTOS, safeRTOS, NORTOS | Setup MPU and control access to address space                 | -
Semaphore         | R5F, C66x       | NA                | FreeRTOS, safeRTOS, NORTOS | Binary, Counting Semaphore, recursive mutexes with timeout     | -
Task              | R5F, C66x       | NA                | FreeRTOS, safeRTOS         | Create, delete tasks                                          | -
Timer             | R5F, C66x       | YES               | FreeRTOS, safeRTOS, NORTOS | Configure arbitrary timers                                    | -
Event             | R5F, C66x       | YES               | FreeRTOS, safeRTOS         | Setting, getting, clearing, and waiting of Event bits         | -

## Previous Release Notes

\ref RELEASE_NOTES_11_02_00_PAGE "Release Notes 11.02.00"

---

**Date:** May 14, 2026
