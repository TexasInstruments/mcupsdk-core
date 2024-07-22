# NO RTOS {#KERNEL_NORTOS_PAGE}

[TOC]

## Introduction

NORTOS is a SW module that implements APIs which allow the upper layers of SW to run in no-RTOS mode.
i.e without any RTOS. The NO RTOS APIs are implemented underneath the \ref KERNEL_DPL_PAGE APIs. This allows
the device drivers to run either in no-RTOS mode or with a RTOS.

## Features Supported

Common across all CPUs,
- Clock APIs to initialize a system tick ISR and allow SW to create multiple SW timers using a single underlying HW timer.
\cond !SOC_AM65X
- Address translate APIs to translate system address to local address, needed for M4F
\endcond
- Heap APIs to create arbitrary heaps at user defined memory locations
- Semaphore APIs to model a semaphore in no-RTOS environment
- HW Timer APIs to setup user defined HW timers beyond the system tick timer.
- Logging APIs to log to different consoles like UART, CCS, shared memory, CPU local memory, including logging zones to enable/disable logging.

\cond !SOC_AM62X
R5F features,
- CPU start up code
- Cache APIs to enable, disable, invalidate, write back caches
- Memory protection unit (MPU) APIs to enable, disable multiple regions in the MPU
- Interrupt controller APIs to register ISRs, enable/disable interrupts
- ISR handlers and exception handlers
- Performance counter APIs
- R5F ISRs,
  - IRQ mode,
    - nested interrupts supported
    - FPU save/restore supported
\endcond
\cond !SOC_AM65X
M4F features,
- CPU start up code
- Memory protection unit (MPU) APIs to enable, disable multiple regions in the MPU
- Interrupt controller APIs to register ISRs, enable/disable interrupts
- ISR handlers and exception handlers
- Performance counter APIs
- SysTick timer APIs
- M4F ISRs
  - Nested interrupts
\endcond

\cond SOC_AM64X
A53 features,
- Single Core A53
- CPU start up code
- Memory management unit (MMU) APIs to enable, disable multiple regions in the MMU
- Interrupt controller APIs to register ISRs, enable/disable interrupts
- ISR handlers and exception handlers
- Performance counter APIs
- A53 ISRs
  - IRQ mode
    - nested interrupt supported
    - FPU save/restore supported
\endcond

## SysConfig Features

@VAR_SYSCFG_USAGE_NOTE

SysConfig can be used to configure below modules with NORTOS
- Clock module to setup system tick timer including the tick duration
- Debug Log module to select the console to use for logging as well as enable/disable logging zones
\if SOC_AM65X
- MPU ARMv7 to setup different MPU regions for R5F
\else
- MPU ARMv7 to setup different MPU regions for R5F and M4F CPUs
\endif
\cond SOC_AM64X
- MMU ARMV8 to setup different MMU regions for A53 CPU
\endcond
\if SOC_AM65X
- RAT to setup  address translation regions, needed for M3F
\else
- RAT to setup  address translation regions, needed for M4F
\endif
- Timer to setup HW timer available on the SOC, including enabling timer interrupt and ISR registration

## Features Not Supported

- Task APIs are not supported in NORTOS mode. Task APIs necessarily need a RTOS and cannot be used in no-RTOS mode
\cond !SOC_AM62X
- R5F ISRs,
  - FIQ mode,
    - nested interrupts not supported
    - FPU save/restore not supported.
\endcond
\cond !SOC_AM65X
- M4F ISRs,
  - FPU save/restore not supported.
\endcond
\cond SOC_AM64X
- A53 ISRs,
  - FIQ mode ISRs not supported.
- A53 multi-core SMP mode is not supported.
\endcond

## Important files and directory structure

<table>
<tr>
    <th>Folder/Files
    <th>Description
</tr>
<tr><td colspan="2" bgcolor=#F0F0F0> ${SDK_INSTALL_PATH}/source/kernel/</td></tr>
<tr>
    <td>dpl/
    <td>APIs to access NORTOS features</td>
</tr>
<tr><td colspan="2" bgcolor=#F0F0F0> ${SDK_INSTALL_PATH}/source/kernel/nortos</td></tr>
<tr>
    <td>lib/
    <td>NORTOS library to link against. Linking to the library in this path enables the application to operate in no-RTOS mode</td>
</tr>
<tr>
    <td>dpl/common/
    <td>NORTOS APIs that are common across all CPUs
</tr>
\cond !SOC_AM65X
<tr>
    <td>dpl/m4/
    <td>NORTOS APIs that are specific to M4F CPUs
</tr>
\endcond
\cond !SOC_AM62X
<tr>
    <td>dpl/r5/
    <td>NORTOS APIs that are specific to R5F CPUs
</tr>
\cond SOC_AM64X
<tr>
    <td>dpl/a53/
    <td>NORTOS APIs that are specific to A53 CPUs
</tr>
\endcond
\endcond
</table>

## Additional References {#NORTOS_ADDITIONAL_REFERENCES}

\note Precise web links are not provided since these can change, search for the document title in google to download the documents

\cond SOC_AM64X
Please also refer to below documents from ARM Ltd. to understand more about R5F, M4F, A53 CPUs including
features like MPU, MMU, cache and interrupts.
\endcond
\cond SOC_AM243X || SOC_AM263X || SOC_AM263PX || SOC_AM273X || SOC_AWR294x || SOC_AM261X
Please also refer to below documents from ARM Ltd. to understand more about R5F, M4F CPUs including
features like MPU, MMU, cache and interrupts.
\endcond

<table>
<tr>
    <th>Document Title
    <th>Description
</tr>
\cond !SOC_AM62X
<tr>
    <td>ARM v7R Architecture Reference Manual
    <td>Information about ARM architecture that is implemented by R5F. Should be used in conjunction with R5F TRM to understand R5F architecture details.
</tr>
\endcond
\cond !SOC_AM65X
<tr>
    <td>ARM v7M Architecture Reference Manual
    <td>Information about ARM architecture that is implemented by M4F. Should be used in conjunction with M4F TRM to understand M4F architecture details.
</tr>
\cond SOC_AM64X
<tr>
    <td>ARM v8A Architecture Reference Manual
    <td>Information about ARM architecture that is implemented by A53. Should be used in conjunction with A53 TRM to understand A53 architecture details.
</tr>
\endcond
\endcond
<tr>
    <td>ARM Cortex R5F Technical Reference Manual
    <td>Information about R5F CPU architecture.
</tr>
\cond !SOC_AM65X
<tr>
    <td>ARM Cortex M4F Technical Reference Manual
    <td>Information about M4F CPU architecture.
</tr>
\endcond
\cond SOC_AM64X
<tr>
    <td>ARM Cortex A53 Technical Reference Manual
    <td>Information about A53 CPU architecture.
</tr>
\endcond
</table>

## See also

\ref KERNEL_DPL_PAGE