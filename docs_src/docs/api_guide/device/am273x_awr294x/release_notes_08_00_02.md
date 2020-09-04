# Release Notes 08.00.02 {#RELEASE_NOTES_08_00_02_PAGE}

[TOC]

\attention Also refer to individual module pages for more details on each feature, unsupported features, important usage guidelines.

\note The examples will show usage of SW modules and APIs on a specific CPU instance and OS combination. \n
      Unless noted otherwise, the SW modules would work in both FreeRTOS and NORTOS environment. \n
      Unless noted otherwise, the SW modules would work on any of the CPU present on the SOC. \n
      Unless noted otherwise, the SW modules would work on all supported EVMs \n

## Extra Installation (On top of default CCS Installation)

- To use this SDK, SysConfig version @VAR_SYSCFG_VERSION_AM273X build @VAR_SYSCFG_BUILD_AM273X should be installed separately on top of the default CCS location.
- Contact the TI representative to get access to this SysConfig version
- Once the installation is complete (use default path), copy the sysconfig_@VAR_SYSCFG_VERSION_AM273X build folder to CCS folder C:/ti/ccs1040/ccs/utils as well.
This is needed to use CCS projects from CCS
\cond SOC_AWR294X
- Additionally, for AM294x the Sysconfig device data should be overridden with AWR294X.json present in <MCU+SDK Install Path>/tools/ccs_load/awr294x/AWR294X.json in below locations
 - C:/ti/sysconfig_@VAR_SYSCFG_VERSION_AM273X build/dist/deviceData/AWR294X
 - C:/ti/ccs1040/ccs/utils/sysconfig_@VAR_SYSCFG_VERSION_AM273X build/dist/deviceData/AWR294X
\endcond

## New in this Release

\attention This is an early adaptor release for AM273x/AWR294x device

Feature                                                             | Module
--------------------------------------------------------------------|--------------------------
R5F (No-RTOS, FreeRTOS), C66x (No-RTOS, FreeRTOS),                  | CPU/OS
LwIP with ethernet driver (ENET) for CPSW                           | LwIP, ENET
UART, RTI (Timer), EDMA                                             | Drivers
IPC, HWA, CSI2-RX, CRC, GPIO, MIBSPI                                | Drivers
QSPI, Flash writer, MCAN, I2C , ESM, Watchdog                       | Drivers
ADCBUF (AWR294x Only)                                               | Drivers
SBL booting R5F and C66x                                            | Bootloader

## Device and Validation Information

\cond SOC_AM273X
SOC     | Supported CPUs  | EVM                                                | Host PC
--------|-----------------|----------------------------------------------------|-----------------------------------
AM273x  | R5F, C66x       | AM273x GP EVM (referred to as am273x-evm in code)  | Windows 10 64b or Ubuntu 18.04 64b
\endcond

\cond SOC_AWR294X
SOC     | Supported CPUs  | EVM                                                 | Host PC
--------|-----------------|-----------------------------------------------------|-----------------------------------
AWR294x | R5F, C66x       | AWR294x GP EVM (referred to as awr294x-evm in code) | Windows 10 64b or Ubuntu 18.04 64b
\endcond

## Tools, Compiler and Other Open Source SW Module Information

Tools / SW module       | Supported CPUs | Version
------------------------|----------------|-----------------------
Code Composer Studio    | R5F, C66x      | @VAR_CCS_VERSION
SysConfig               | R5F, C66x      | @VAR_SYSCFG_VERSION_AM273X build, build @VAR_SYSCFG_BUILD_AM273X
TI ARM CLANG            | R5F            | @VAR_TI_ARM_CLANG_VERSION
TI C6000 Compiler       | C66x           | @VAR_TI_C6000_CGT_VERSION
FreeRTOS Kernel         | R5F, C66x      | @VAR_FREERTOS_KERNEL_VERSION

## Key Features

### OS Kernel

OS              | Supported CPUs  | SysConfig Support | Key features tested                                                                                                                                                 | Key features not tested / NOT supported
----------------|-----------------|-------------------|---------------------------------------------------------------------------------------------------------------------------------------------------------------------|----------------------------------------
FreeRTOS Kernel | R5F, C66x       | NA                | Task, Task notification, interrupts, semaphores, mutexs, timers, event groups. ROV views in CCS IDE, Task load measurement using FreeRTOS run time statistics APIs. | -
FreeRTOS POSIX  | R5F, C66x       | NA                | pthread, mqueue, semaphore, clock                                                                                                                                   | -
NO RTOS         | R5F, C66x       | NA                | See **Driver Porting Layer (DPL)** below                                                                                                                            | -

### Driver Porting Layer (DPL)

Module            | Supported CPUs  | SysConfig Support | OS support       | Key features tested                                           | Key features not tested / NOT supported
------------------|-----------------|-------------------|------------------|---------------------------------------------------------------|----------------------------------------
Cache             | R5F, C66x       | YES               | FreeRTOS, NORTOS | Cache write back, invalidate, enable/disable                  | -
Clock             | R5F, C66x       | YES               | FreeRTOS, NORTOS | Tick timer at user specified resolution, timeouts and delays  | -
CycleCounter      | R5F, C66x       | NA                | FreeRTOS, NORTOS | Measure CPU cycles using CPU specific internal counters       | -
Debug             | R5F, C66x       | YES               | FreeRTOS, NORTOS | Logging and assert to any combo of: UART, CCS, shared memory  | -
Heap              | R5F, C66x       | NA                | FreeRTOS, NORTOS | Create arbitrary heaps in user defined memory segments        | -
Hwi               | R5F, C66x       | YES               | FreeRTOS, NORTOS | Interrupt register, enable/disable/restore                    | -
MPU               | R5F             | YES               | FreeRTOS, NORTOS | Setup MPU and control access to address space                 | -
Semaphore         | R5F, C66x       | NA                | FreeRTOS, NORTOS | Binary, Counting Semaphore, recursive mutexs with timeout     | -
Task              | R5F, C66x       | NA                | FreeRTOS         | Create, delete tasks                                          | -
Timer             | R5F, C66x       | YES               | FreeRTOS, NORTOS | Configure arbitrary timers                                    | -
Event             | R5F, C66x       | YES               | FreeRTOS         | Setting, getting, clearing, and waiting of Event bits         | -

### SOC Device Drivers

Peripheral | Supported CPUs | SysConfig Support | Key features tested                                                                                       | Key features not tested / NOT supported
-----------|----------------|-------------------|-----------------------------------------------------------------------------------------------------------|----------------------------------------
ADCBUF     | R5F, C66x      | YES               | Source selection, Set chirp thresholds, continuous mode, configure modes                                  | -
CRC        | R5F, C66x      | YES               | Two channels, 8, 16, 32 and 64 bit data size, CPU mode                                                    | -
CSI-RX     | R5F, C66x      | YES               | Setup complexio, dphy, common and context settings, event callbacks                                       | -
EDMA       | R5F, C66x      | YES               | Basic memory copy, DMA/QDMA channels, Interrupt/Polled, Manual/Event trigger, Chaining                    | -
ESM        | R5F, C66x      | YES               | Group and Error number selection, Tested ESM notifier with watchdog module                                | -
GPIO       | R5F, C66x      | YES               | Basic input/output, GPIO as interrupt                                                                     | -
HWA        | R5F, C66x      | YES               | FFT, CFAR, compression/decompression and local maxima modules, Interrupt/Polled, Manual/DMA trigger       | -
I2C        | R5F, C66x      | YES               | Controller mode, basic read/write, polling and interrupt mode                                                 | Peripheral mode not supported. Driver not tested from C66x due to EVM limitations
IPC Notify | R5F, C66x      | YES               | Low latency IPC between RTOS/NORTOS CPUs                                                                  | -
IPC Rpmsg  | R5F, C66x      | YES               | RPMessage protocol based IPC for all R5F, C66x running NORTOS/FreeRTOS                                    | -
MCAN       | R5F            | YES               | RX, TX, interrupt and polling mode                                                                        | -
MIBSPI     | R5F, C66x      | YES               | Controller/Peripheral mode, basic read/write, Interrupt/Polled, icount enable/disable, CPU/DMA mode                | -
Pinmux     | R5F, C66x      | YES               | Tested with multiple peripheral pinmuxes                                                                  | -
QSPI       | R5F            | YES               | Read direct, Write indirect, Read/Write commands                                                          | Interrupt mode not supported, Dual and Quad writes are not supported
SOC        | R5F, C66x      | YES               | Lock/unlock MMRs, get CPU clock, CPU name, clock enable, set frequency                                    | -
UART       | R5F, C66x      | YES               | Basic read/write, polling, interrupt mode                                                                 | DMA mode not supported
WATCHDOG   | R5F, C66x      | YES               | Window size and Expiry time selections, Reset mode, Digital windowed                                      | -

### Secondary Bootloader (SBL)

Module     | Supported CPUs  | OS support       | Key features tested                                                                             | Key features not tested / NOT supported
-----------|-----------------|------------------|-------------------------------------------------------------------------------------------------|----------------------------------------
Bootloader | R5FSS0-0        | NORTOS           | Boot modes: QSPI, UART. R5F (Lockstep) and C66x core boot. RPRC, multi-core image format, Pll configuration.  | Dual Core R5 boot.

### Board Device Drivers

Peripheral | Supported CPUs | SysConfig Support | Key features tested                                                           | Key features not tested
-----------|----------------|-------------------|-------------------------------------------------------------------------------|------------------------
EEPROM     | R5F            | YES               | I2C based EEPROM                                                              | -
Flash      | R5F            | YES               | QSPI based flash                                                              | All vendor flash types not tested
LED        | R5F, C66x      | YES               | GPIO based LED control                                                        | -

### Networking

Module                      | Supported CPUs | SysConfig Support | OS Support  | Key features tested                                                                    | Key features not tested
----------------------------|----------------|-------------------|-------------|----------------------------------------------------------------------------------------|------------------------
LwIP                        | R5F            | NO                | FreeRTOS    | TCP/UDP IP networking stack, DHCP, ping, TCP iperf, TCP/UDP IP                         | Other LwIP features, performance and memory optimizations pending, more robustness tests pending
Ethernet driver (ENET)      | R5F            | NO                | FreeRTOS    | Ethernet as port using CPSW; MAC loopback and PHY loopback                             | Ethernet as switch


## Fixed Issues

<table>
<tr>
    <th> ID
    <th> Head Line
    <th> Module
    <th> Applicable Releases
    <th> Applicable Devices
    <th> Resolution/Comments
</tr>
<tr>
    <td> MCUSDK-1831
    <td> MIBSPI Analog Loopback Example Fails
    <td> MIBSPI
    <td> 8.00.01
    <td> AM273x
    <td> Fixed
</tr>
<tr>
    <td> MCUSDK-1973
    <td> MIBSPI peripheral test fails
    <td> MIBSPI
    <td> 8.00.02
    <td> AM273x
    <td> Fixed
</tr>
<tr>
    <td> MCUSDK-1953
    <td> DSS_L3 memory range in linker files are not correct.
    <td> L3 memory
    <td> 8.00.02
    <td> AWR294x
    <td> Fixed
</tr>
</table>

## Known Issues

<table>
<tr>
    <th> ID
    <th> Head Line
    <th> Module
    <th> Applicable Releases
    <th> Applicable Devices
    <th> Workaround
</tr>
<tr>
    <td> MCUSDK-1889
    <td> HWA: Context switch tests fails from C66x
    <td> HWA
    <td> 8.00.01
    <td> AWR294x
    <td> None
</tr>
<tr>
    <td> MCUSDK-1859
    <td> I2C Read Test Fails with SBL
    <td> I2C
    <td> 8.00.01
    <td> AM273x
    <td> None. Issue is not seen when the example is run from CCS
</tr>
<tr>
    <td> MCUSDK-1984
    <td> MCAN External Timestamp Interrupt Failing
    <td> MCAN
    <td> 8.00.02
    <td> AM273x, AWR294x
    <td> None
</tr>
<tr>
    <td> PDK-8405
    <td> MIBSPI non-DMA mode transfer doesn't complete when used in mmWaveSDK
    <td> MIBSPI
    <td> 8.00.01
    <td> AM273x, AWR294x
    <td> None. Issue is not seen in driver unit test
</tr>
<tr>
    <td> MCUSDK-1994
    <td> SBL: R5 in lock step mode is able to access only 16KB TCMA memory
    <td> SBL
    <td> 8.00.01
    <td> AM273x, AWR294x
    <td> Use 16KB TCMA memory for R5 application
</tr>
<tr>
    <td> MCUSDK-1995
    <td> SBL: App image loading is failing if both R5 core 0 and core 1 image are present in appimage
    <td> SBL
    <td> 8.00.01
    <td> AM273x, AWR294x
    <td> Use R5 in lock step mode
</tr>
<tr>
    <td> MCUSDK-1996
    <td> SBL: uniflash does not work if uart is configured in interrupt mode
    <td> SBL
    <td> 8.00.01
    <td> AM273x, AWR294x
    <td> Use uart in polled mode
</tr>
<tr>
    <td> MCUSDK-1776
    <td> MetaImagecreator.bat file setting for MSS only image
    <td> SBL
    <td> 8.00.02
    <td> AM273x
    <td> None
</tr>
<tr>
    <td> MCUSDK-2015
    <td> Converted MSS GPADC data has fluctuations in ext channel and temperature values
    <td> ADC
    <td> 8.00.02
    <td> AWR294x
    <td> Set GPADC FSM bypass to 0
</tr>
<tr>
    <td> MCUSDK-2020
    <td> WDT Interrupt Mode Test Fails for C66x core
    <td> WDT
    <td> 8.00.02
    <td> AM273x, AWR294x
    <td> None
</tr>
</table>

## Limitations

<table>
<tr>
    <th> ID
    <th> Head Line
    <th> Module
    <th> Reported in Release
    <th> Applicable Devices
    <th> Workaround
</tr>
<tr>
    <td> PDK-8404
    <td> QSPI test failing at 80 Mhz
    <td> QSPI
    <td> 8.00.01
    <td> AM273x, AWR294x
    <td> QSPI driver works at 40 Mhz
</tr>
</table>

## Upgrade and Compatibility Information

\attention When migrating from Processor SDK RTOS, see \ref MIGRATION_GUIDES for more details

This section lists changes which could affect user applications developed using older SDK versions.
Read this carefully to see if you need to do any changes in your existing application when migrating to this SDK version relative to
previous SDK version. Also refer to older SDK version release notes to see changes in
earlier SDKs.

NA. First Release
