# Release Notes 08.03.00 {#RELEASE_NOTES_08_03_00_PAGE}

[TOC]

\attention Also refer to individual module pages for more details on each feature, unsupported features, important usage guidelines.

\note The examples will show usage of SW modules and APIs on a specific CPU instance and OS combination. \n
      Unless noted otherwise, the SW modules would work in both FreeRTOS and NO-RTOS environment. \n
      Unless noted otherwise, the SW modules would work on any of the CPU present on the SOC. \n
      Unless noted otherwise, the SW modules would work on all supported EVMs \n

## New in this Release

Feature                                                                                         | Module
------------------------------------------------------------------------------------------------|--------------------------
32 Task Priority Levels for FreeRTOS Tasks                                                      | DPL, FreeRTOS
SBL JTAG Uniflash Example supported                                                             | SBL
SBL QSPI appimage copy done in EDMA mode                                                        | SBL
MCASP Driver support with DMA mode                                                              | MCASP
MCASP audio playback demo                                                                       | MCASP
Support enabled for DPL queues                                                                  | DPL
Enabled SFDP support in QSPI driver                                                             | QSPI, Flash
CPDMA multi-channel support                                                                     | CPSW
C++ build support and example project added                                                     | Generic

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

\cond SOC_AM273X
Tools / SW module       | Supported CPUs | Version
------------------------|----------------|-----------------------
Code Composer Studio    | R5F, C66x      | @VAR_CCS_VERSION
SysConfig               | R5F, C66x      | @VAR_SYSCFG_VERSION_AM273X build, build @VAR_SYSCFG_BUILD_AM273X
TI ARM CLANG            | R5F            | @VAR_TI_ARM_CLANG_VERSION
TI C6000 Compiler       | C66x           | @VAR_TI_C6000_CGT_VERSION
FreeRTOS Kernel         | R5F, C66x      | @VAR_FREERTOS_KERNEL_VERSION
DSP LIB                 | C66x           | @VAR_DSPLIB_VERSION

DSP LIB package is modified to fix the build in Linux environment from the base version dsplib_c66x_3_4_0_0
\endcond

\cond SOC_AWR294X
Tools / SW module       | Supported CPUs | Version
------------------------|----------------|-----------------------
Code Composer Studio    | R5F, C66x      | @VAR_CCS_VERSION
SysConfig               | R5F, C66x      | @VAR_SYSCFG_VERSION_AM273X build, build @VAR_SYSCFG_BUILD_AM273X
TI ARM CLANG            | R5F            | @VAR_TI_ARM_CLANG_VERSION
TI C6000 Compiler       | C66x           | @VAR_TI_C6000_CGT_VERSION
FreeRTOS Kernel         | R5F, C66x      | @VAR_FREERTOS_KERNEL_VERSION
\endcond

## Key Features

### Experimental Features

\attention Features listed below are early versions and should be considered as "experimental".
\attention Users can evaluate the feature, however the feature is not fully tested at TI side.
\attention TI would not support these feature on public e2e.
\attention Experimental features will be enabled with limited examples and SW modules.

Feature                                                             | Module
--------------------------------------------------------------------|--------------------------
GUI for UART Uniflash Tool                                          | Bootloader

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
Queue             | R5F, C66x       | NA                | FreeRTOS, safeRTOS, NORTOS | Enqueue, dequeue, status                                      | -

### SOC Device Drivers

Peripheral | Supported CPUs | SysConfig Support | DMA Supported | Key features tested                                                                                         | Key features not tested / NOT supported
-----------|----------------|-------------------|---------------|-------------------------------------------------------------------------------------------------------------|----------------------------------------
ADCBUF     | R5F, C66x      | YES               | No            | Source selection, Set chirp thresholds, continuous mode, configure modes                                    | -
CBUFF      | R5F, C66x      | YES               | YES           | stream data over LVDS interface                                                                             | -
CRC        | R5F, C66x      | YES               | NA            | Two channels, 8, 16, 32 and 64 bit data size, CPU mode                                                      | -
CSI-RX     | R5F, C66x      | YES               | NA            | Setup complexio, dphy, common and context settings, event callbacks                                         | -
ECAP       | R5F, C66x      | YES               | NA            | Frequency, Duty cycle, interrupt mode                                                                       | PWM mode not tested
EDMA       | R5F, C66x      | YES               | NA            | Basic memory copy, DMA/QDMA channels, Interrupt/Polled, Manual/Event trigger, Chaining                      | -
EPWM       | R5F            | YES               | NA            | Frequency, Duty cycle, interrupt mode                                                                       | Tripzone, Deadband and Chopper module not tested
ESM        | R5F, C66x      | YES               | NA            | Group and Error number selection, Tested ESM notifier with watchdog module                                  | -
GPADC      | R5F, C66x      | YES               | NA            | 10-bit ADC, Tested single/multiple buffer and on board temperature sensor read                              | -
GPIO       | R5F, C66x      | YES               | NA            | Basic input/output, GPIO as interrupt                                                                       | -
HWA        | R5F, C66x      | YES               | YES           | FFT, CFAR, compression/decompression and local maxima modules, Interrupt/Polled, Manual/DMA trigger         | -
I2C        | R5F, C66x      | YES               | No            | Controller mode, basic read/write, polling and interrupt mode                                                   | Peripheral mode not supported. Driver not tested from C66x due to EVM limitations
IPC Notify | R5F, C66x      | YES               | NA            | Low latency IPC between RTOS/NORTOS CPUs                                                                    | -
IPC Rpmsg  | R5F, C66x      | YES               | NA            | RPMessage protocol based IPC for all R5F, C66x running NORTOS/FreeRTOS                                      | -
MCAN       | R5F            | YES               | NA            | RX, TX, interrupt and polling mode                                                                          | -
MIBSPI     | R5F, C66x      | YES               | YES           | Controller/Peripheral mode, basic read/write, Interrupt/Polled, icount enable/disable, CPU/DMA mode                  | -
MCASP      | R5F, C66x      | YES               | YES           | Controller mode, transmit/receive, Interrupt/DMA, serializer config                                             | -
Pinmux     | R5F, C66x      | YES               | NA            | Tested with multiple peripheral pinmuxes                                                                    | -
QSPI       | R5F            | YES               | YES           | Read direct, Write indirect, Read/Write commands                                                            | Interrupt mode not supported, Dual and Quad writes are not supported
SOC        | R5F, C66x      | YES               | NA            | Lock/unlock MMRs, get CPU clock, CPU name, clock enable, set frequency, SW Warm Reset, Address Translation  | -
UART       | R5F, C66x      | YES               | YES           | Basic read/write, polling, interrupt mode, CPU/DMA mode                                                     | -
WATCHDOG   | R5F, C66x      | YES               | NA            | Window size and Expiry time selections, Reset mode, Digital windowed                                        | -

### Secondary Bootloader (SBL)

Module     | Supported CPUs  | OS support       | Key features tested                                                                                                   | Key features not tested / NOT supported
-----------|-----------------|------------------|-----------------------------------------------------------------------------------------------------------------------|----------------------------------------
Bootloader | R5FSS0-0        | NORTOS           | Boot modes: QSPI, UART. R5F (Lockstep/Dual Core) and C66x core boot. RPRC, multi-core image format, Pll configuration.| -

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
    <th> Resolution/Comments
</tr>
\cond SOC_AM273X
<tr>
    <td> MCUSDK-3940
    <td> GPIO clear interrupt API clears all set interrupts
    <td> GPIO
    <td> 8.02.00
    <td> GPIO_clearInterrupt updated to clear only single bit corresponding to pin number
</tr>
<tr>
    <td> MCUSDK-3888
    <td> EDMA Syscfg Attribute structure incorrect mask generation
    <td> EDMA
    <td> 8.02.00
    <td> Sysconfig template fixed
</tr>
<tr>
    <td> MCUSDK-3846
    <td> am273x uniflash fails for appimages >1MB
    <td> Flash
    <td> 8.02.00
    <td> Added error checks before file deletes and fixed the bug in send_file_by_parts API
</tr>
<tr>
    <td> MCUSDK-3588
    <td> docs - Enet Am273x - Remove ICSSG references
    <td> docs, Enet
    <td> 8.02.00
    <td> Fixed
</tr>
<tr>
    <td> MCUSDK-2232
    <td> "HWA_paramSetDonePolling" function in HWA driver not working properly
    <td> HWA_FFT
    <td> 8.01.00, 8.02.00
    <td> Addressed issues with invalid check for verifying param done status in HWA driver.
</tr>
<tr>
    <td> MCUSDK-3909
    <td> Enet - examples sysconfig has Enet config missing
    <td> ENET
    <td> 8.02.00
    <td> Fixed
</tr>
<tr>
    <td> MCUSDK-2037
    <td> AM273x: Observing issues with SPI communication on Two-Chip cascade board
    <td> MiBSPI
    <td> 8.01.00, 8.02.00
    <td> Fixed
</tr>
<tr>
    <td> MCUSDK-3633
    <td> AM273x examples - linker command cleanup
    <td> docs
    <td> 8.01.00, 8.02.00
    <td> Removed the invalid referances from linker cmd file
</tr>
\endcond
\cond SOC_AM273X || SOC_AWR294X
<tr>
    <td> MCUSDK-3948
    <td> Incorrect device name in flash driver
    <td> Flash
    <td> 8.01.00, 8.02.00
    <td> Fixed
</tr>
<tr>
    <td> MCUSDK-3947
    <td> [MCAN]MCAN message acceptance filter masking is incorrect
    <td> MCAN
    <td> 08_00_00
    <td> Fixed
</tr>
\endcond
\cond SOC_AWR294X
<tr>
    <td> MCUSDK-3277
    <td> xipGen option parsing issue
    <td> XIP
    <td> 8.02.00
    <td> None
</tr>
<tr>
    <td> MCUSDK-3142
    <td> DSTIQSWAP bit is not set in HWA driver code
    <td> HWA_FFT
    <td> 8.02.00
    <td> None
</tr>
\endcond
</table>

## Known Issues

<table>
<tr>
    <th> ID
    <th> Head Line
    <th> Module
    <th> Applicable Releases
    <th> Workaround
</tr>
\cond SOC_AM273X
<tr>
    <td> MCUSDK-3897
    <td> MCASP Audio playback demo does not work in interrupt mode
    <td> MCASP
    <td> 8.03.00
    <td> Use the McASP in DMA mode
</tr>
\endcond
\cond SOC_AM273X || SOC_AWR294X
<tr>
    <td> MCUSDK-3899
    <td> MIBSPI non-DMA mode transfer doesn't complete when used in mmWaveSDK
    <td> MIBSPI
    <td> 8.00.01
    <td> None. Issue is not seen in driver unit test
</tr>
<tr>
    <td> MCUSDK-2453
    <td> R5 - Init code crashes under certain conditions
    <td> FreeRTOS, No-RTOS
    <td> 8.02.00
    <td> None
</tr>
<tr>
    <td> MCUSDK-4190
    <td> A fixed MAC address is hard-coded, this will cause issue when two boards are connected in the same network
    <td> ENET
    <td> 8.02.00 onwards
    <td> None
</tr>
\endcond
</table>

## Limitations

<table>
<tr>
    <th> ID
    <th> Head Line
    <th> Module
    <th> Reported in Release
    <th> Workaround
</tr>
<tr>
    <td> PDK-8404
    <td> QSPI test failing at 80 Mhz
    <td> QSPI
    <td> 8.00.01
    <td> QSPI driver works at 40 Mhz
</tr>
</table>

## Upgrade and Compatibility Information

\attention When migrating from Processor SDK RTOS, see \ref MIGRATION_GUIDES for more details

This section lists changes which could affect user applications developed using older SDK versions.
Read this carefully to see if you need to do any changes in your existing application when migrating to this SDK version relative to
previous SDK version. Also refer to older SDK version release notes to see changes in
earlier SDKs.

### Compiler Options

<table>
<tr>
    <th> Module
    <th> Affected API
    <th> Change
    <th> Additional Remarks
</tr>
<tr>
    <td> Release Build
    <td> -
    <td> Updated optimization level in release build to 'Os' as part of bug fix MCUSDK-1980.
    <td> The release build with ti-arm-clang compiler has been updated from 'O3' to 'Os' optimization level. It is recommended that user rebuilds the existing libraries and applications with 'Os' option. This can be done by updating CFLAGS_release in library/example makefiles.
</tr>
</table>


