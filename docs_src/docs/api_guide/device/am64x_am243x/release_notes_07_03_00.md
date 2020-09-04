# Release Notes 07.03.00 {#RELEASE_NOTES_07_03_00_PAGE}

[TOC]

\attention Also refer to individual module pages for more details on each feature, unsupported features, important usage guidelines.

\note The examples will show usage of SW modules and APIs on a specific CPU instance and OS combination. \n
      Unless explicitly noted otherwise, the SW modules would work in both FreeRTOS and no-RTOS environment. \n
      Unless explicitly noted otherwise, the SW modules would work on any of the R5F's present on the SOC. \n
      All SW modules will not work on M4F. M4F support for a module will be explicitly called out.

## Device and Validation Information

\cond SOC_AM64X
SOC   | Supported CPUs  | EVM                                             | Host PC
------|-----------------|-------------------------------------------------|-----------------------------------
AM64x | R5F, M4F        | AM64x GP EVM (referred to as am64x-evm in code) | Windows 10 64b or Ubuntu 18.04 64b
\endcond

\cond SOC_AM243X
SOC    | Supported CPUs  | EVM                                               | Host PC
-------|-----------------|---------------------------------------------------|-----------------------------------
AM243x | R5F, M4F        | AM243x GP EVM (referred to as am243x-evm in code) | Windows 10 64b or Ubuntu 18.04 64b
\endcond

## Dependent Tools and Compiler Information

Tools                   | Supported CPUs | Version
------------------------|----------------|-----------------------
Code Composer Studio    | R5F, M4F       | @VAR_CCS_VERSION
SysConfig               | R5F, M4F       | @VAR_SYSCFG_VERSION, build @VAR_SYSCFG_BUILD
TI ARM CLANG            | R5F, M4F       | @VAR_TI_ARM_CLANG_VERSION
FreeRTOS Kernel         | R5F, M4F       | @VAR_FREERTOS_KERNEL_VERSION

## Key Features

### OS Kernel

OS              | Supported CPUs  | SysConfig Support | Key features tested                                             | Key features not tested / NOT supported
----------------|-----------------|-------------------|-----------------------------------------------------------------|----------------------------------------
FreeRTOS Kernel | R5F, M4F        | NA                | Task, Task notification, interrupts, semaphores, mutexs, timers | Task load measurement using FreeRTOS run time statistics APIs. Limited support for ROV features.
FreeRTOS POSIX  | R5F, M4F        | NA                | pthread, mqueue, semaphore, clock                               | -
NO RTOS         | R5F, M4F        | NA                | See **Driver Porting Layer (DPL)** below                        | -

### Driver Porting Layer (DPL)

Module            | Supported CPUs  | SysConfig Support | OS support       | Key features tested                                           | Key features not tested / NOT supported
------------------|-----------------|-------------------|------------------|---------------------------------------------------------------|----------------------------------------
Address Translate | M4F             | YES               | FreeRTOS, NORTOS | Use RAT to allow M4F access to peripheral address space       | -
Cache             | R5F             | YES               | FreeRTOS, NORTOS | Cache write back, invalidate, enable/disable                  | -
Clock             | R5F, M4F        | YES               | FreeRTOS, NORTOS | Tick timer at user specified resolution, timeouts and delays  | -
CycleCounter      | R5F, M4F        | NA                | FreeRTOS, NORTOS | Measure CPU cycles using CPU specific internal counters       | -
Debug             | R5F, M4F        | YES               | FreeRTOS, NORTOS | Logging and assert to any combo of: UART, CCS, shared memory  | -
Heap              | R5F, M4F        | NA                | FreeRTOS, NORTOS | Create arbitrary heaps in user defined memory segments        | -
Hwi               | R5F, M4F        | YES               | FreeRTOS, NORTOS | Interrupt register, enable/disable/restore                    | -
MPU               | R5F, M4F        | YES               | FreeRTOS, NORTOS | Setup MPU and control access to address space                 | -
Semaphore         | R5F, M4F        | NA                | FreeRTOS, NORTOS | Binary, Counting Semaphore, recursive mutexs with timeout     | -
Task              | R5F, M4F        | NA                | FreeRTOS         | Create, delete tasks                                          | -
Timer             | R5F, M4F        | YES               | FreeRTOS, NORTOS | Configure arbitrary timers                                    | -

### Secondary Bootloader (SBL)

Module     | Supported CPUs  | SysConfig Support | OS support       | Key features tested                                                         | Key features not tested / NOT supported
-----------|-----------------|-------------------|------------------|-----------------------------------------------------------------------------|----------------------------------------
Bootloader | R5FSS0-0        | YES               | NORTOS           | Boot modes: OSPI, UART. All R5F's, M4F boot. RPRC, multi-core image format  | DDR init during SBL. OSPI with higher clocks and phy enabled. SBL booting Linux. SBL booting A53.

### SOC Device Drivers

Peripheral | Supported CPUs | SysConfig Support | Key features tested                                                          | Key features not tested / NOT supported
-----------|----------------|-------------------|------------------------------------------------------------------------------|----------------------------------------
ADC        | R5F            | YES               | Single conversion (one-shot mode), interrupt mode                            | Continuous conversions
CRC        | R5F            | NO                | Only compiled                                                                | Not tested
ECAP       | R5F            | NO                | Only compiled                                                                | Not tested
EPWM       | R5F            | YES               | Different Frequency, Duty cycle, interrupt mode, Deadband and chopper module | Tripzone module
EQEP       | R5F            | NO                | Only compiled                                                                | Not tested
FSI (RX/TX)| R5F            | YES               | RX, TX, polling, interrupt mode, single/dual lanes                           | -
GPIO       | R5F, M4F       | YES               | Basic input/output, GPIO as interrupt                                        | -
I2C        | R5F, M4F       | YES               | Controller mode, basic read/write, polling and interrupt mode                    | Target mode. M4F not tested due to EVM limitation
IPC Notify | R5F, M4F       | YES               | Low latency IPC between RTOS/NORTOS CPUs                                     | -
IPC Rpmsg  | R5F            | YES               | RPMessage protocol based IPC, All R5F and Linux A53 cores                    | M4F core
MCAN       | R5F            | YES               | RX, TX, interrupt mode                                                       | Polling mode not tested
MCSPI      | R5F, M4F       | YES               | Controller/Peripheral mode, basic read/write, polling, interrupt mode             | DMA mode
MDIO       | R5F            | NA                | Register read/write, link status and link interrupt enable API               | -
OSPI       | R5F            | YES               | Read direct, Write indirect, Read/Write commands, DMA for read               | PHY Mode, Interrupt Mode not supported yet
Pinmux     | R5F, M4F       | YES               | Tested with multiple peripheral pinmuxes                                     | -
PRUICSS    | R5F            | YES               | Tested with Ethercat FW HAL, HDSL, EnDat                                     | More protocols integration tests pending
SOC        | R5F, M4F       | YES               | lock/unlock MMRs, get CPU clock, CPU name, clock enable, set Hz              | -
Sciclient  | R5F, M4F       | YES               | Tested with clock setup, module on/off                                       | -
SPINLOCK   | R5F, M4F       | NA                | Lock, unlock HW spinlocks                                                    | -
UART       | R5F, M4F       | YES               | Basic read/write, polling, interrupt mode,                                   | HW flow control not tested, DMA mode not supported
UDMA       | R5F            | YES               | Basic memory copy, SW trigger, Chaining, ADC                                 | -

### Board Device Drivers

Peripheral | Supported CPUs | SysConfig Support | Key features tested                                         | Key features not tested
-----------|----------------|-------------------|-------------------------------------------------------------|------------------------
EEPROM     | R5F            | YES               | I2C based EEPROM                                            | -
ETHPHY     | R5F            | YES               | Ethernet Phy configuration for EtherCAT SubDevice example       | -
Flash      | R5F            | YES               | XSPI based flash, Octal mode, No PHY                        | Other flash types
LED        | R5F            | YES               | GPIO and I2C IO expander based LED control                  | -

### CMSIS

Module                      | Supported CPUs | SysConfig Support | OS Support        | Key features tested                                                                         | Key features not tested
----------------------------|----------------|-------------------|-------------------|---------------------------------------------------------------------------------------------|------------------------
CMSIS DSP                   | R5F            | NA                | FreeRTOS, NORTOS  | Basic math, complex math, controller, fast math, filtering, Matrix, statistics, transform   | -

### Motor Control

Module                      | Supported CPUs | SysConfig Support | OS Support        | Key features tested                             | Key features not tested
----------------------------|----------------|-------------------|-------------------|-------------------------------------------------|------------------------
Position Sense HDSL         | R5F            | YES               | FreeRTOS, NORTOS  | Freerun mode, Sync mode                         |  -
Position Sense EnDAT        | R5F            | YES               | FreeRTOS, NORTOS  | Single channel, Continuous mode                 |  Multi Channel

### Networking

Module                      | Supported CPUs | SysConfig Support | OS Support  | Key features tested                                 | Key features not tested
----------------------------|----------------|-------------------|-------------|-----------------------------------------------------|------------------------
EtherCAT SubDevice FW HAL       | R5F            | YES               | FreeRTOS    | Tested with Beckhoff's EtherCAT stack based example | Online firmware update, reset isolation
EtherCAT SubDevice ETG Stack    | R5F            | NO                | FreeRTOS    | Standard stack (NOT BUNDLED in SDK)                 | -
EtherNet/IP Adapter FW HAL  | R5F            | NO                | FreeRTOS    | Only compiled                                       | Not tested
Profinet Device FW HAL      | R5F            | NO                | FreeRTOS    | Only compiled                                       | Not tested
ICSS-EMAC                   | R5F            | YES               | FreeRTOS    | Only compiled                                       | Not tested
ICSS TimeSync               | R5F            | NO                | FreeRTOS    | Only compiled                                       | Not tested. "Event" construct related code is not compiled.

### Demos

Module                      | Supported CPUs | SysConfig Support | OS Support        | Key features tested                             | Key features not tested
----------------------------|----------------|-------------------|-------------------|-------------------------------------------------|------------------------
Benchmark demo              | 4xR5F's        | YES               | NORTOS            | CFFT, FIR and FOC benchmarks                    |  ADC/PWM benchmark

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
    <td> MCUSDK-177
    <td> DMTimer on M4F results in 2 interrupts instead of 1 for every DM timer expiry in release mode
    <td> DPL
    <td> 7.3.0
    <td> Use ClockP module which uses SysTick M4 timer and this does not have any issues.
</tr>
<tr>
    <td> MCUSDK-180
    <td> Bootloader: Flash_Open and other APIs in SBL fail when the bootmode is OSPI
    <td> Bootloader
    <td> 7.3.0
    <td> Use CCS scripting or SD card based SOC init method as mentioned in userguide when running `ospi_flash_io
</tr>
<tr>
    <td> MCUSDK-316
    <td> ROV does not work with DDR being used in application
    <td> FreeRTOS
    <td> 7.3.0
    <td> ROV cannot be used when operating from DDR
</tr>
<tr>
    <td> MCUSDK-317
    <td> OSPI: DMA reads fails after a flash write which was not block aligned
    <td> OSPI
    <td> 7.3.0
    <td> Use non-DMA read mode or use DMA mode but always write from start of a erased block
</tr>
<tr>
    <td> MCUSDK-353
    <td> ENDAT: EnDAT diagnostic does not support multi channel.
    <td> ENDAT
    <td> 7.3.0
    <td> Use single channel at a time.
</tr>
<tr>
    <td> MCUSDK-354
    <td> EtherCAT State Machine from EtherCAT Conformance Test fails intermittently
    <td> EtherCAT
    <td> 7.3.0
    <td> Use debug mode library for FreeRTOS. The makefile and projectspec for this example use debug mode library by default for all configurations
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
    <td> MCUSDK-208
    <td> gmake with -j can sometimes lock up Windows command prompt
    <td> Build
    <td> 7.3.0
    <td> Use bash for windows as part of git for windows or don't use -j option
</tr>
</table>

## Upgrade and Compatibility Information

\attention When migrating from Processor SDK RTOS, see \ref MIGRATION_GUIDES for more details

### SOC Device Drivers

<table>
<tr>
    <th> Module
    <th> Affected API
    <th> Change
    <th> Additional Remarks
</tr>
<tr>
    <td> BOOTLOADER
    <td> \ref Bootloader_loadSelfCpu
    <td> Takes argument as \ref Bootloader_CpuInfo, instead of \ref Bootloader_BootImageInfo
    <td> \ref Bootloader_loadSelfCpu should be called each time for each CPU vs calling it once for all "self" CPUs earlier.
</tr>
</table>


