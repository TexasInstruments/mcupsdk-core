# Release Notes 08.00.02 {#RELEASE_NOTES_08_00_02_PAGE}

[TOC]

\attention Also refer to individual module pages for more details on each feature, unsupported features, important usage guidelines.

\note The examples will show usage of SW modules and APIs on a specific CPU instance and OS combination. \n
      Unless explicitly noted otherwise, the SW modules would work in both FreeRTOS and no-RTOS environment. \n
      Unless explicitly noted otherwise, the SW modules would work on any of the R5F's present on the SOC. \n

## Device and Validation Information

\cond SOC_AM263X
SOC   | Supported CPUs  | EVM                                                    | Host PC
------|-----------------|--------------------------------------------------------|-----------------------------------
AM263x| R5F             | AM263x ControlCard  (referred to as am263x-cc in code) | Windows 10 64b or Ubuntu 18.04 64b
\endcond


## Dependent Tools and Compiler Information

Tools                   | Supported CPUs | Version
------------------------|----------------|-----------------------
Code Composer Studio    | R5F            | @VAR_CCS_VERSION
SysConfig               | R5F            | @VAR_SYSCFG_VERSION, build @VAR_SYSCFG_BUILD
TI ARM CLANG            | R5F            | @VAR_TI_ARM_CLANG_VERSION
FreeRTOS Kernel         | R5F            | @VAR_FREERTOS_KERNEL_VERSION
LwIP                    | R5F            | @VAR_LWIP_VERSION

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
Bootloader | R5FSS0-0        | YES               | NORTOS           | Only compiled                                                               | -

### SOC Device Drivers

Peripheral   | Supported CPUs | SysConfig Support | Key features tested                                                                             | Key features not tested / NOT supported
-------------|----------------|-------------------|-------------------------------------------------------------------------------------------------|----------------------------------------
ADC          | R5F            | YES               | Single software triggered conversion                                                            | -
Bootloader   | R5F            | YES               | Only compiled                                                                                   | -
CMPSS        | R5F            | YES               | Only compiled                                                                                   | -
CPSW         | R5F            | YES               | MAC loopback, PHY loopback, LWIP: Getting IP, Ping, Iperf                                       | -
DAC          | R5F            | YES               | Constant voltage and square wave generation                                                     | -
ECAP         | R5F            | YES               | Only compiled                                                                                   | -
EDMA         | R5F            | YES               | DMA transfer using interrupt and polling mode, QDMA Transfer, Channel Chaining, PaRAM Linking   | -
EPWM         | R5F            | YES               | PWM outputs A and B in up-down count mode                                                       | -
EQEP         | R5F            | YES               | Only compiled                                                                                   | -
FSI          | R5F            | YES               | Only compiled                                                                                   | -
GPIO         | R5F            | YES               | Output functionality                                                                            | -
I2C          | R5F            | YES               | Only compiled                                                                                   | -
IPC Notify   | R5F            | YES               | Mailbox functionality, IPC between RTOS/NORTOS CPUs                                             | M4F core
IPC Rpmsg    | R5F            | YES               | RPMessage protocol based IPC                                                                    | M4F core
MCAN         | R5F            | YES               | Only compiled                                                                                   | -
MCSPI        | R5F            | YES               | Only compiled                                                                                   | -
MDIO         | R5F            | YES               | Only compiled                                                                                   | -
MPU Firewall | R5F            | YES               | Only compiled                                                                                   | -
PINMUX       | R5F            | YES               | Only compiled                                                                                   | -
PRUICSS      | R5F            | YES               | Only compiled                                                                                   | -
QSPI         | R5F            | YES               | Read direct, Write indirect, Read/Write commands, DMA for read                                  | -
SDFM         | R5F            | YES               | Only compiled                                                                                   | -
SOC          | R5F            | YES               | Lock/unlock MMRs, clock enable, set Hz, Xbar configuration                                      | -
SPINLOCK     | R5F            | NA                | Only compiled                                                                                   | -
UART         | R5F            | YES               | Basic read/write at baud rate 115200, polling, interrupt mode                                   | HW flow control not tested, DMA mode not supported
WATCHDOG     | R5F            | YES               | Only compiled                                                                                   | -

### Board Device Drivers

Peripheral | Supported CPUs | SysConfig Support | Key features tested                                         | Key features not tested
-----------|----------------|-------------------|-------------------------------------------------------------|------------------------
EEPROM     | R5F            | YES               | Only compiled                                               | -
ETHPHY     | R5F            | YES               | Only compiled                                               | -
FLASH      | R5F            | YES               | Only compiled                                               | -
LED        | R5F            | YES               | GPIO                                                        | -


### CMSIS

Module                      | Supported CPUs | SysConfig Support | OS Support        | Key features tested                                                                         | Key features not tested
----------------------------|----------------|-------------------|-------------------|---------------------------------------------------------------------------------------------|------------------------
-                           | -              | -                 | -                 | -                                                                                           |  -

### Industrial Communications Toolkit

Module                                | Supported CPUs | SysConfig Support | OS Support  | Key features tested                                                                                      | Key features not tested
--------------------------------------|----------------|-------------------|-------------|----------------------------------------------------------------------------------------------------------|------------------------
EtherCAT SubDevice FWHAL                  | R5F            | NO                | FreeRTOS    | -                                                                                                        | -

### Motor Control

Module                      | Supported CPUs | SysConfig Support | OS Support        | Key features tested                             | Key features not tested
----------------------------|----------------|-------------------|-------------------|-------------------------------------------------|------------------------
-                           | -              | -                 | -                 | -                                               |  -

### Networking

Module                      | Supported CPUs | SysConfig Support | OS Support  | Key features tested                                 | Key features not tested
----------------------------|----------------|-------------------|-------------|-----------------------------------------------------|------------------------
LwIP                        | R5F            | NO                | FreeRTOS    | TCP/UDP IP networking stack, DHCP, ping, TCP iperf, TCP/UDP IP                         | Other LwIP features, performance and memory optimizations pending, more robustness tests pending
Ethernet driver (ENET)      | R5F            | NO                | FreeRTOS    | Ethernet as port using CPSW and ICSS                                                   | Ethernet as switch


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


