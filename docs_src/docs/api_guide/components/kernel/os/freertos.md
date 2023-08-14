# FreeRTOS {#KERNEL_FREERTOS_PAGE}

[TOC]

## Introduction

FreeRTOS is a market-leading real-time operating system (RTOS) for microcontrollers and small microprocessors. Distributed freely under the MIT open source license, FreeRTOS includes a kernel and a growing set of libraries suitable for use across all industry sectors. FreeRTOS is built with an emphasis on reliability and ease of use.

MCU+ SDK supports FreeRTOS on below CPUS
\cond !SOC_AM65X
- ARM M4F
\endcond
\cond !SOC_AM62X
- ARM R5F
\cond SOC_AM64X
- ARM A53 (single core and SMP on both cores)
\endcond
\endcond

## Features Supported {#FREERTOS_SUPPORTED_FEATURES}

- FreeRTOS Kernel @VAR_FREERTOS_KERNEL_VERSION
  - Tasks, semaphores, mutex, queues, timers, list, heap, event groups
  - preemptive priority based scheduler
  - static and/or dynamic memory allocation mode
\cond SOC_AM64X
- FreeRTOS SMP Kernel @VAR_FREERTOS_SMP_KERNEL_VERSION
  - Tasks, semaphores, mutex, queues, timers, list, heap, event groups
  - preemptive priority based scheduler for multiple cores
  - static and/or dynamic memory allocation mode
\endcond
- FreeRTOS+POSIX
  - Limited POSIX API wrappers on top of FreeRTOS APIs
  - Clock, message queue, pthread, pthread cond, pthread mutex, semaphore, timer
- In order to keep the device drivers agnostic of FreeRTOS or NORTOS, additionally below \ref KERNEL_DPL_PAGE APIs are implemented to call FreeRTOS APIs underneath,
  - Clock, task, semaphore, heap, cache, MPU, debug logs, HW interrupts, HW timers
- Floating point save/restore with tasks (make sure to call portTASK_USES_FLOATING_POINT() before using floating point operations )
\cond !SOC_AM62X
- R5F ISRs,
  - IRQ mode,
    - FPU save/restore is supported.
    \cond !SOC_AM273X && !SOC_AM65X
    - Priority based interrupt masking in critical section is supported using configMAX_SYSCALL_INTERRUPT_PRIORITY, uncomment macro **EN_MAX_SYSCALL_INTR_PRI_CRIT_SECTION** in source/kernel/freertos/portable/TI_ARM_CLANG/ARM_CR5F/portmacro.h to enable.(Disabled by default)
    \endcond
    - nested interrupts are supported.
\endcond
\cond !SOC_AM65X
- M4F ISRs,
  - nested interrupts supported
\endcond
\cond SOC_AM64X
- A53 ISRs,
  - IRQ mode,
    - nested interrupts supported
\endcond

## Features Not Supported

- Co-routines, stream buffer are not enabled and are not compiled by default. Users can add these to the FreeRTOS config and makefile if they want to use these features.
- Tickless IDLE mode
- Task level memory protection wrapper
\cond !SOC_AM62X
- R5F ISRs,
  - IRQ mode,
    \cond SOC_AM273X
    - Priority based interrupt masking in critical section is not supported.
    \endcond
  - FIQ mode,
    - nested interrupts not supported
    - FPU save/restore not supported.
\endcond
- M4F ISRs,
  - FPU save/restore not supported.
\cond SOC_AM64X
- A53 ISRs,
  - FPU save/restore not supported.
\endcond

## SysConfig Features

@VAR_SYSCFG_USAGE_NOTE

SysConfig can be used to configure below modules with FreeRTOS
- Clock module, to setup system tick timer including the tick duration
- Debug Log module, to select the console to use for logging as well as enable/disable logging zones
\cond !SOC_AM65X
- MPU ARMv7, to setup different MPU regions for R5F and M4F CPUs
- Address Translate module, to setup  address translation regions, needed for M4F
\endcond
\cond SOC_AM65X
- MPU ARMv7, to setup different MPU regions for R5F CPUs
\endcond
\cond SOC_AM64X
- MMU ARMv8, to setup different MMU regions for A53 CPUs
\endcond
\cond !SOC_AM65X
- Address Translate module, to setup  address translation regions, needed for M4F
\endcond
- HW Timer module, to setup HW timer available on the SOC, including enabling timer interrupt and ISR registration

## Important files and directory structure

FreeRTOS source is distributed along with MCU+ SDK and given below are some important files and folders related to FreeRTOS.

<table>
<tr>
    <th>Folder/Files
    <th>Description
</tr>
<tr><td colspan="2" bgcolor=#F0F0F0> ${SDK_INSTALL_PATH}/source/kernel/</td></tr>
<tr>
    <td>dpl/
    <td>APIs to access FreeRTOS features in a OS agnostic way</td>
</tr>
<tr><td colspan="2" bgcolor=#F0F0F0> ${SDK_INSTALL_PATH}/source/kernel/freertos</td></tr>
<tr>
    <td>lib/
    <td>FreeRTOS library to link against. Linking to the library in this path enables the application to operate in FreeRTOS mode </td>
</tr>
<tr>
    <td>config/
    <td>FreeRTOS and FreeRTOS POSIX configuration header files for different CPUs within a SOC.
</tr>
<tr>
    <td>FreeRTOS-Kernel/
    <td>FreeRTOS Kernel source code. MCU+ SDK simply clones the code from FreeRTOS github and does not modify anything in this folder</td>
</tr>
\cond SOC_AM64X
<tr>
    <td>FreeRTOS-Kernel-smp/
    <td>FreeRTOS Kernel source code for SMP. MCU+ SDK simply clones the code from FreeRTOS github and does not modify anything in this folder</td>
</tr>
\endcond
<tr>
    <td>FreeRTOS-POSIX/
    <td>FreeRTOS POSIX wrapper source code. MCU+ SDK simply clones the code from FreeRTOS POSIX github and does not modify anything in this folder</td>
</tr>
<tr>
    <td>portable/
    <td>MCU+ SDK FreeRTOS porting related files for different CPUs</td>
</tr>
<tr><td colspan="2" bgcolor=#F0F0F0> ${SDK_INSTALL_PATH}/source/kernel/freertos/dpl</td></tr>
<tr>
    <td>common/
    <td>FreeRTOS DPL APIs that are common across all CPUs
</tr>
\cond !SOC_AM65X
<tr>
    <td>m4/
    <td>FreeRTOS DPL APIs that are specific to M4F CPUs
</tr>
\endcond
\cond !SOC_AM62X
<tr>
    <td>r5/
    <td>FreeRTOS APIs that are specific to R5F CPUs
</tr>
\cond SOC_AM64X
<tr>
    <td>a53/
    <td>FreeRTOS APIs that are specific to A53 CPUs
</tr>
\endcond
\endcond
</table>

In addition to above files, to enable features like HW interrupts, cache, MPU, the FreeRTOS library
in MCU+ SDK also shares some code with NORTOS, see to makefile in the folder `source\kernel\freertos` to see the exact list of files
that are included to build a freertos library.

## FreeRTOS usage guidelines

See \subpage KERNEL_FREERTOS_IMPORTANT_GUIDELINES_PAGE for FreeRTOS usage guidelines and comparison to SysBIOS.

## Additional references

Given below are some references to learn more about FreeRTOS.

<table>
<tr>
    <th>Document Description
    <th>Web link
</tr>
<tr>
    <td>Easy to read FreeRTOS book
    <td>https://www.freertos.org/Documentation/RTOS_book.html
</tr>
<tr>
    <td>FreeRTOS user docs
    <td>https://www.freertos.org/features.html
</tr>
<tr>
    <td>User API reference
    <td>https://www.freertos.org/a00106.html
</tr>
<tr>
    <td>FreeRTOS core kernel source code
    <td>https://github.com/FreeRTOS/FreeRTOS-Kernel
</tr>
\cond SOC_AM64X
<tr>
    <td>FreeRTOS core kernel source code for SMP
    <td>https://github.com/FreeRTOS/FreeRTOS-Kernel/tree/smp
</tr>
\endcond
<tr>
    <td>FreeRTOS core kernel example source code
    <td>https://github.com/FreeRTOS/FreeRTOS/tree/master/FreeRTOS
</tr>
<tr>
    <td>Additional FreeRTOS.org maintained libraries (POSIX, TCP, Filesystem, …)
    <td>https://www.freertos.org/FreeRTOS-Labs/index.html \n
    https://www.freertos.org/FreeRTOS-Plus/index.html \n
    https://github.com/FreeRTOS
</tr>
</table>

## See also

\ref KERNEL_DPL_PAGE, \ref KERNEL_NORTOS_PAGE
