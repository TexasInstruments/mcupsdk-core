# Release Notes 08.02.00 {#RELEASE_NOTES_08_02_00_PAGE}

[TOC]

\attention Also refer to individual module pages for more details on each feature, unsupported features, important usage guidelines.

\note The examples will show usage of SW modules and APIs on a specific CPU instance and OS combination. \n
      Unless noted otherwise, the SW modules would work in both FreeRTOS and NORTOS environment. \n
      Unless noted otherwise, the SW modules would work on any of the CPU present on the SOC. \n
      Unless noted otherwise, the SW modules would work on all supported EVMs \n

## New in this Release
\cond SOC_AM273X
Feature                                                             | Module
--------------------------------------------------------------------|--------------------------
R5F (No-RTOS, FreeRTOS), C66x (No-RTOS, FreeRTOS),                  | CPU/OS
LwIP with ethernet driver (ENET) for CPSW                           | LwIP, ENET
UART, RTI (Timer), EDMA, EPWM, CBUFF                                | Drivers
IPC, HWA, CSI2-RX, CRC, GPIO, MIBSPI                                | Drivers
QSPI, Flash writer, MCAN, I2C , ESM, Watchdog, GPADC, ECAP          | Drivers
SBL booting R5F and C66x                                            | Bootloader
\endcond
\cond SOC_AWR294X
Feature                                                             | Module
--------------------------------------------------------------------|--------------------------
R5F (No-RTOS, FreeRTOS), C66x (No-RTOS, FreeRTOS),                  | CPU/OS
SafeRTOS DPL (C66x only)                                            | CPU/OS
LwIP with ethernet driver (ENET) for CPSW                           | LwIP, ENET
UART, RTI (Timer), EDMA, EPWM, CBUFF                                | Drivers
IPC, HWA, CSI2-RX, CRC, GPIO, MIBSPI, ADCBUF                        | Drivers
QSPI, Flash writer, MCAN, I2C , ESM, Watchdog, GPADC                | Drivers
SBL booting R5F and C66x                                            | Bootloader
\endcond

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

### OS Kernel

OS              | Supported CPUs  | SysConfig Support | Key features tested                                                                                                                                                 | Key features not tested / NOT supported
----------------|-----------------|-------------------|---------------------------------------------------------------------------------------------------------------------------------------------------------------------|----------------------------------------
FreeRTOS Kernel | R5F, C66x       | NA                | Task, Task notification, interrupts, semaphores, mutexs, timers, event groups. ROV views in CCS IDE, Task load measurement using FreeRTOS run time statistics APIs. | -
FreeRTOS POSIX  | R5F, C66x       | NA                | pthread, mqueue, semaphore, clock                                                                                                                                   | -
NO RTOS         | R5F, C66x       | NA                | See **Driver Porting Layer (DPL)** below                                                                                                                            | -

### Driver Porting Layer (DPL)

Module            | Supported CPUs  | SysConfig Support | OS support                 | Key features tested                                           | Key features not tested / NOT supported
------------------|-----------------|-------------------|----------------------------|---------------------------------------------------------------|----------------------------------------
Cache             | R5F, C66x       | YES               | FreeRTOS, safeRTOS, NORTOS | Cache write back, invalidate, enable/disable                  | -
Clock             | R5F, C66x       | YES               | FreeRTOS, safeRTOS, NORTOS | Tick timer at user specified resolution, timeouts and delays  | -
CycleCounter      | R5F, C66x       | NA                | FreeRTOS, safeRTOS, NORTOS | Measure CPU cycles using CPU specific internal counters       | -
Debug             | R5F, C66x       | YES               | FreeRTOS, safeRTOS, NORTOS | Logging and assert to any combo of: UART, CCS, shared memory  | -
Heap              | R5F, C66x       | NA                | FreeRTOS, safeRTOS, NORTOS | Create arbitrary heaps in user defined memory segments        | -
Hwi               | R5F, C66x       | YES               | FreeRTOS, safeRTOS, NORTOS | Interrupt register, enable/disable/restore                    | -
MPU               | R5F             | YES               | FreeRTOS, safeRTOS, NORTOS | Setup MPU and control access to address space                 | -
Semaphore         | R5F, C66x       | NA                | FreeRTOS, safeRTOS, NORTOS | Binary, Counting Semaphore, recursive mutexs with timeout     | -
Task              | R5F, C66x       | NA                | FreeRTOS, safeRTOS         | Create, delete tasks                                          | -
Timer             | R5F, C66x       | YES               | FreeRTOS, safeRTOS, NORTOS | Configure arbitrary timers                                    | -
Event             | R5F, C66x       | YES               | FreeRTOS, safeRTOS         | Setting, getting, clearing, and waiting of Event bits         | -

### SOC Device Drivers

Peripheral | Supported CPUs | SysConfig Support | Key features tested                                                                                       | Key features not tested / NOT supported
-----------|----------------|-------------------|-----------------------------------------------------------------------------------------------------------|----------------------------------------
ADCBUF     | R5F, C66x      | YES               | Source selection, Set chirp thresholds, continuous mode, configure modes                                  | -
CBUFF      | R5F, C66x      | YES               | stream data over LVDS interface                                                                           | -
CRC        | R5F, C66x      | YES               | Two channels, 8, 16, 32 and 64 bit data size, CPU mode                                                    | -
CSI-RX     | R5F, C66x      | YES               | Setup complexio, dphy, common and context settings, event callbacks                                       | -
ECAP       | R5F, C66x      | YES               | Frequency, Duty cycle, interrupt mode                                                                     | PWM mode not tested
EDMA       | R5F, C66x      | YES               | Basic memory copy, DMA/QDMA channels, Interrupt/Polled, Manual/Event trigger, Chaining                    | -
EPWM       | R5F            | YES               | Frequency, Duty cycle, interrupt mode                                                                     | Tripzone, Deadband and Chopper module not tested
ESM        | R5F, C66x      | YES               | Group and Error number selection, Tested ESM notifier with watchdog module                                | -
GPADC      | R5F, C66x      | YES               | 10-bit ADC, Tested single/multiple buffer and on board temperature sensor read                            | -
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
UART       | R5F, C66x      | YES               | Basic read/write, polling, interrupt mode, CPU/DMA mode                                                   | -
WATCHDOG   | R5F, C66x      | YES               | Window size and Expiry time selections, Reset mode, Digital windowed                                      | -

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
\cond SOC_AWR294X
<tr>
    <td> MCUSDK-2044
    <td> ADCBUF: Continuous Mode Test Fails in sbl
    <td> ADCBUF
    <td> 08.01.00, 08.00.02
    <td> Fixed
</tr>
\endcond
\cond SOC_AM273X
<tr>
    <td> MCUSDK-2457
    <td> Tools installed twice due to makefile based and ccs based builds
    <td> Build
    <td> 08.01.00
    <td> Fixed
</tr>
\endcond
\cond SOC_AM273X || SOC_AWR294X
<tr>
    <td> MCUSDK-2191
    <td> CPSW example doesn't work with SBL
    <td> CPSW
    <td> 08.01.00
    <td> Fixed
</tr>
\endcond
\cond SOC_AM273X
<tr>
    <td> MCUSDK-2346
    <td> docs - Document External JTAG issue workaround
    <td> docs
    <td> 08.01.00
    <td> Fixed
</tr>
\endcond
\cond SOC_AM273X || SOC_AWR294X
<tr>
    <td> MCUSDK-2396
    <td> Low SVC stack size with Interrupt nesting enabled
    <td> DPL
    <td> 08.01.00
    <td> Fixed
</tr>
\endcond
\cond SOC_AWR294X
<tr>
    <td> MCUSDK-2046
    <td> DPL: C66x Cache Fail in sbl
    <td> DPL
    <td> 08.00.02
    <td> Fixed
</tr>
\endcond
\cond SOC_AWR294X
<tr>
    <td> MCUSDK-2513
    <td> "EDMA event queue number 1" is not seen in sysconfig of MSS and DSS
    <td> EDMA
    <td> 08.01.00
    <td> Fixed
</tr>
\endcond
\cond SOC_AM273X
<tr>
    <td> MCUSDK-2301
    <td> Correct main task name from frertos_main to freertos_main
    <td> Enet
    <td> 08.01.00
    <td> Fixed
</tr>
\endcond
\cond SOC_AM273X || SOC_AWR294X
<tr>
    <td> MCUSDK-2244
    <td> Enet documentation needs to be enabled
    <td> Enet
    <td> 08.01.00
    <td> Fixed
</tr>
\endcond
\cond SOC_AWR294X
<tr>
    <td> MCUSDK-2509
    <td> ESMRegisterNotifier function returns notifyIndex than SystemPSuccess
    <td> ESM
    <td> 08.01.00
    <td> Fixed
</tr>
\endcond
\cond SOC_AWR294X
<tr>
    <td> MCUSDK-2193
    <td> FreeRTOS: Test Timer and interrupts in continuous mode test fail
    <td> FreeRTOS
    <td> 08.01.00, 08.00.02
    <td> Fixed
</tr>
\endcond
\cond SOC_AM273X || SOC_AWR294X
<tr>
    <td> MCUSDK-2151
    <td> I2C Probe does not display peripheral devices correctly
    <td> I2C
    <td> 08.01.00, 08.00.02
    <td> Fixed
</tr>
\endcond
\cond SOC_AM273X || SOC_AWR294X
<tr>
    <td> MCUSDK-2541
    <td> IPC developer guide block diagram shows invalid cores
    <td> IPC_Notify, IPC_RPMSG
    <td> 08.01.00
    <td> Fixed
</tr>
\endcond
\cond SOC_AWR294X
<tr>
    <td> MCUSDK-2184
    <td> MCAN External Loopback Interrupt Mode Sample Application Fails
    <td> MCAN
    <td> 08.01.00, 08.00.02
    <td> Fixed
</tr>
\endcond
\cond SOC_AWR294X
<tr>
    <td> MCUSDK-2176
    <td> MIBSPI Peripheral Read Fail
    <td> MiBSPI
    <td> 08.01.00, 08.00.02
    <td> Fixed
</tr>
\endcond
\cond SOC_AWR294X
<tr>
    <td> MCUSDK-2627
    <td> QSPI Flash Transfer and Flash DMA Mode Read Write example fails the subsequent tests.
    <td> QSPI
    <td> 08.01.00
    <td> awr294x-evm
    <td> Fixed
</tr>
\endcond
\cond SOC_AM273X
<tr>
    <td> MCUSDK-2507
    <td> [AM273] [QSPI] Missing QSPI driver documentation
    <td> QSPI
    <td> 08.01.00
    <td> Fixed
</tr>
\endcond
\cond SOC_AWR294X
<tr>
    <td> MCUSDK-2728
    <td> SBL bug in configuring PLL_CORE_HSDIVIDER_CLKOUT3
    <td> RCM
    <td> 08.01.01
    <td> Fixed
</tr>
\endcond
\cond SOC_AM273X || SOC_AWR294X
<tr>
    <td> MCUSDK-3266
    <td> Remove customer name from the code
    <td> SBL
    <td> 08.01.00
    <td> Fixed
</tr>
\endcond
\cond SOC_AWR294X
<tr>
    <td> MCUSDK-2402
    <td> SBL Memory reservation limitation impacting available memory to application
    <td> SBL
    <td> 08.01.00
    <td> Fixed
</tr>
\endcond
\cond SOC_AM273X || SOC_AWR294X
<tr>
    <td> MCUSDK-2401
    <td> Application size limitation with SBL_UART
    <td> SBL
    <td> 08.01.00
    <td> Fixed
</tr>
\endcond
\cond SOC_AM273X
<tr>
    <td> MCUSDK-2416
    <td> AM64X: Writing images to flash larger than 1024 KByte, uniflash finishes with errors
    <td> UART
    <td> 08.01.00
    <td> Fixed
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
\cond SOC_AWR294X
<tr>
    <td> MCUSDK-1889
    <td> HWA: Context switch tests fails from C66x
    <td> HWA
    <td> 8.00.01
    <td> None
</tr>
\endcond
\cond SOC_AM273X || SOC_AWR294X
<tr>
    <td> PDK-8405
    <td> MIBSPI non-DMA mode transfer doesn't complete when used in mmWaveSDK
    <td> MIBSPI
    <td> 8.00.01
    <td> None. Issue is not seen in driver unit test
</tr>
\endcond
\cond SOC_AM273X
<tr>
    <td> MCUSDK-2037
    <td> MIBSPI: Observing issues with SPI communication on Two-Chip cascade board
    <td> MIBSPI
    <td> 8.00.01
    <td> None. Issue is not seen when ccs log is enabled in the application.
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
\endcond
\cond SOC_AWR294X
<tr>
    <td> MCUSDK-3142
    <td> DSTIQSWAP bit is not set in HWA driver code
    <td> HWA_FFT
    <td> 8.02.00
    <td> None
</tr>
\endcond
\cond SOC_AM273X || SOC_AWR294X
<tr>
    <td> MCUSDK-2453
    <td> R5 - Init code crashes under certain conditions
    <td> FreeRTOS, No-RTOS
    <td> 8.02.00
    <td> None
</tr>
\endcond
\cond SOC_AM273X
<tr>
    <td> MCUSDK-2232
    <td> "HWA_paramSetDonePolling" function in HWA driver not working properly
    <td> HWA_FFT
    <td> 8.02.00
    <td> None
</tr>
\endcond
\cond SOC_AM273X || SOC_AWR294X
<tr>
    <td> MCUSDK-2183
    <td> Streaming of data over Ethernet interface not stable
    <td> Enet
    <td> 8.02.00
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


