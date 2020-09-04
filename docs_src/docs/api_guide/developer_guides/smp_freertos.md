# Running SMP FreeRTOS application on A53 {#SMP_FREERTOS_GUIDE}

[TOC]

## Introduction

\note Only example application that supports SMP mode can be run on both A53 cores in SMP mode.
\attention All SPI interrupts are routed to A53 Core 0.

SMP FreeRTOS kernel support is added in the DPL layer for A53 core. This makes use of both the A53 cores for running applications by making using of the FreeRTOS SMP kernel for scheduling tasks on both cores.

SMP application uses SGI interrupt interrupt number 0 for communication between A53 cores. So care must be taken not to use this interrupt in user applications.

SMP applications can be loaded via SBL as well as CCS. A53 cores must be run as sync group in CCS when running SMP applications.

## Running SMP application using CCS

\note Only example application that supports SMP mode can be run on both A53 cores in SMP mode.

### Loading and running a .out file for an SMP application
When runnig SMP application using CCS, the cores needs to be run as sync group.

A sync group can be created as follows. This ensures the same binary is executed on both cores from the same entry point.

\imageStyle{a53_sync_group_create.png,width:60%}
\image html a53_sync_group_create.png "Sync group creation in CCS"

### Building and running an SMP application using CCS

An SMP example for A53 can be imported and build just as a single core A53 application. But both cores must be selected and sync group must be created when launching the debug session.

\imageStyle{ccs_smp_debug_session_launch.png,width:50%}
\image html ccs_smp_debug_session_launch.png "Launching SMP debug session in CCS"

## Dhrystone benchmark values for SMP FreeRTOS on A53

\note The values obtained is from example - \ref EXAMPLES_KERNEL_FREERTOS_DHRYSTONE_BENCHMARK when build in release mode

The dhrystone values can be used to compare the performance between single core FreeRTOS on A53 and SMP FreeRTOS on A53.

No of Threads   | No of Iterations | Dhrystone per second for Single Core FreeRTOS on A53    | Dhrystone per second for SMP FreeRTOS on A53
----------------|------------------|---------------------------------------------------------|-----------------------------------------------
1               | 30000000         | 1266                                                    | 1268
2               | 30000000         | 577                                                     | 1155
5               | 30000000         | 164                                                     | 274
10              | 30000000         | 26                                                      | 53

## See also

For more details on debugging an SMP application using CCS, see \htmllink{https://software-dl.ti.com/ccs/esd/documents/ccs_smp-debug.html, here}