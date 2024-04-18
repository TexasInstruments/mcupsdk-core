# Release Notes 08.05.00 {#RELEASE_NOTES_08_05_00_PAGE}

[TOC]

\attention Also refer to individual module pages for more details on each feature, unsupported features, important usage guidelines.

\note The examples will show usage of SW modules and APIs on a specific CPU instance and OS combination. \n
      Unless noted otherwise, the SW modules would work in both FreeRTOS and NO-RTOS environment. \n
      Unless noted otherwise, the SW modules would work on any of the CPU present on the SOC. \n
      Unless noted otherwise, the SW modules would work on all supported EVMs \n

\attention Klockwork Static Analysis report is not updated for this release

## New in this Release

Feature                                                                                         | Module
------------------------------------------------------------------------------------------------|--------------------------
HWA Drive improvements - Enabled BFP compression, Single Step API support, application buffer initialization | HWA
McASP Sysconfig enhancement, XREF/XTAL clocks support, Clock information display                | McASP
Ethernet Bare metal support                                                                     | Ethernet
Enet (CPSW) SysConfig support for MDIO, MAC PORT, ALE, Phy configurations etc                   | Ethernet
PTP Timesync demo support in \ref EXAMPLES_ENET_LAYER2_MULTI_CHANNEL_PTP example                    | Ethernet
Software Diagnostics Library                                                                    | SDL



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
Code Composer Studio    | R5F, C66x      | 12.1.0
SysConfig               | R5F, C66x      | 1.14.0 build, build 2667
TI ARM CLANG            | R5F            | 2.1.2.LTS
TI C6000 Compiler       | C66x           | 8.3.12
FreeRTOS Kernel         | R5F, C66x      | 10.4.3
DSP LIB                 | C66x           | 3.4.0.0

DSP LIB package is modified to fix the build in Linux environment from the base version dsplib_c66x_3_4_0_0

\attention TI ARM CLANG @VAR_TI_ARM_CLANG_VERSION is not part of CCS by default, Follow steps at \ref INSTALL_TIARMCLANG to install the compiler

\endcond

\cond SOC_AWR294X
Tools / SW module       | Supported CPUs | Version
------------------------|----------------|-----------------------
Code Composer Studio    | R5F, C66x      | 12.1.0
SysConfig               | R5F, C66x      | 1.14.0 build, build 2667
TI ARM CLANG            | R5F            | 2.1.2.LTS
TI C6000 Compiler       | C66x           | 8.3.12
FreeRTOS Kernel         | R5F, C66x      | 10.4.3
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
LwIP                        | R5F            | NO                | FreeRTOS    | TCP/UDP IP networking stack, DHCP, ping, TCP iperf, TCP/UDP IP, scatter-gather                        | Other LwIP features, more robustness tests pending
Ethernet driver (ENET)      | R5F            | NO                | FreeRTOS    | Ethernet as port using CPSW; MAC loopback and PHY loopback, interrupt pacing, MDIO Manual Mode       | Ethernet as switch

### Safety Diagnostic Library

Module            | Supported CPUs  | SysConfig Support | OS support       | Key features tested                                                                            | Key features not tested / NOT supported
------------------|-----------------|-------------------|------------------|------------------------------------------------------------------------------------------------|----------------------------------------
MCRC              | R5F, C66        | NA                |  NORTOS | Full CPU Mode, Auto CPU Mode.                                                         | Semi CPU Auto Mode on R5F and C66X.
DCC               | R5F, C66        | NA                |  NORTOS | Single Shot Mode, Continuous Mode                                   |
PBIST             | R5F             | NA                |  NORTOS | Memories supported by MSS and DSS PBIST controller.          |-
ESM               | R5F, C66        | NA                |  NORTOS | Tested in combination with RTI, DCC, ECC                                        |-
RTI               | R5F, C66        | NA                |  NORTOS | WINDOWSIZE_100_PERCENT, WINDOWSIZE_50_PERCENT ,Latency/Propagation timing error(early)(50% window),Latency/Propagation timing error(late)(50% window)                                     | -
ECC               | R5F, C66        | NA                |  NORTOS | ECC of MSS_L2, Mailbox, TPTC, R5SS TCM, MCAN     | R5F Cache, VIM, HSM and ICSSM
Bus Safety        | R5F, C66        | NA                |  NORTOS | Bus Safety of Mailbox, DSS L3, HWA, ADCBUF, DSS_PCR, MSS_TPTC, CORE A and B AHB, MCRC                | MSS_CR5, MSS_QSPI, HSM_DTHE, MSS_CPSW, MSS_PCR, HSM, MSS_L2, MSS_SWBUF, MSS_GPADC, MSS_DMM, MSS_TO_MDO, MSS_SCRP, DAP_R232, DSS_DSP, DSS_DSP_SDMA
HWA               | C66             | NA                |  NORTOS | Parity on Data Memories, Window Memory and FSM Lockstep                                                 | -
CCM               | R5F             | NA                |  NORTOS | CCM Self Test Mode.                                                 | Error Forcing Mode and Self Test Error Forcing Mode.
R5F STC(LBIST)    | R5F             | NA                |  NORTOS | STC of R5F.                                                 |-

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
    <td> MCUSDK-1016
    <td> Semaphore does not function as expected when "post" call is present in multiple ISRs at different priorities
    <td> DPL, FreeRTOS
    <td> 07.03.02 onwards
    <td> AM273x
    <td> Fixed
</tr>
<tr>
    <td> MCUSDK-2453
    <td> R5 - Init code crashes under certain conditions
    <td> DPL
    <td> 8.1.0 onwards
    <td> AM273x
    <td> Fixed
</tr>
<tr>
    <td> MCUSDK-3797
    <td> Fix for enabling --rom_model linker optiom
    <td> DPL
    <td> 08.01.00 onwards
    <td> AM273x
    <td> Fixed
</tr>
<tr>
    <td> MCUSDK-7363
    <td> [enet] Documentation of file name incorrect in sysconfig generated file
    <td> Enet
    <td> 8.03.00 onwards
    <td> AM273x
    <td> Fixed
</tr>
<tr>
    <td> MCUSDK-8423
    <td> [enet] MDIO manual mode delay implementation is incorrect
    <td> Enet
    <td> 08.04.00 onwards
    <td> AM273x
    <td> Fixed
</tr>
<tr>
    <td> MCUSDK-8446
    <td> QSPI SBL doesn't run the app image correctly on C66x core
    <td> SBL
    <td> 08.04.00 onwards
    <td> AM273x
    <td> Fixed
</tr>
<tr>
    <td> MCUSDK-8495
    <td> I2C Internal clock is not set correctly
    <td> I2C
    <td> 08.04.00 onwards
    <td> AM273x
    <td> Fixed
</tr>
<tr>
    <td> MCUSDK-8616
    <td> Changing RGMII mode in CPSW lwip example leads to assert
    <td> Enet
    <td> 08.04.00 onwards
    <td> AM243x, AM64x
    <td> Fixed
</tr>
\endcond
\cond SOC_AM273X || SOC_AWR294X
\endcond
\cond SOC_AWR294X
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
    <td> 8.03.00 onwards
    <td> Use the McASP in DMA mode
</tr>
<tr>
    <td> MCUSDK-5873
    <td> FIQ handler data missing in HwIP_armv7r_handlers_nortos_asm.S file
    <td> DPL
    <td> 8.03.00 onwards
    <td> None
</tr>
<tr>
    <td> MCUSDK-7811
    <td> CPSW: Ethernet Packet corruption occurs if CPDMA fetches a packet which spans across memory banks
    <td> CPSW
    <td> 8.03.00 onwards
    <td> Ensure from application side single ethernet packet does not span across memory banks
</tr>
<tr>
    <td> MCUSDK-8854
    <td> Enet_cpsw_tcpclient failing to send large sized data, no warning/assert
    <td> CPSW
    <td> 8.04.00 onwards
    <td> -
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
    <td> PROC_SDL-4558
    <td> Binary generated from MSS ECC CCS based example(sdl_ecc_r5_atcm0) does not work.
    <td> SDL
    <td> 8.5.0 onwards
    <td> Add the resetvecs.S manually in to CCS project / Use the binary generated from gmake.
</tr>
<tr>
    <td> PROC_SDL-4749
    <td> AXI DED Bus Safety fail.
    <td> SDL
    <td> 8.5.0 onwards
    <td> None.
</tr>
<tr>
    <td> PROC_SDL-4751
    <td> In CCM mode, only self test mode tested.Error Forcing Mode and Self Test Error Forcing Mode are not yet supported.
    <td> SDL
    <td> 8.5.0 onwards
    <td> None.
</tr>
<tr>
    <td> PROC_SDL-4755
    <td> MCRC Semi CPU mode is not supported.
    <td> SDL
    <td> 8.5.0 onwards
    <td> Use Full CPU mode ot Auto mode.
</tr>
<tr>
    <td> PROC_SDL-5159
    <td> SEC ECC Bus Safety for MSS_AXI_RD not supported.
    <td> SDL
    <td> 8.5.0 onwards
    <td> None.
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
    <td> -
    <td> -
    <td> Enabled copmpiler option -Oz and -flto for release mode build
    <td> These option are enabled for Code size and performance optimization
</table>

### SOC Device Drivers

<table>
<tr>
    <th> Module
    <th> Affected API
    <th> Change
    <th> Additional Remarks
</tr>
</table>

### Networking

<table>
<tr>
    <th> Module
    <th> Affected API
    <th> Change
    <th> Additional Remarks
</tr>
<tr>
    <td> ENET
    <td> SysConfig
    <td> For all LWIP based examples, packet buffer memory management is moved out of driver and expected to handle at application side. This requires user to regenerate the code via SysConfig
    <td> This is applicable only for RX, TX remains same. Custom pBuff enabled LwIP application to manage the buffer memory efficiently
</tr>
<tr>
    <td> ENET, LWIP
    <td> Library Names
    <td> Library 'enet-lwip-cpsw' is split into 'enet-cpsw' and 'lwipif-cpsw-freertos'. 'lwip-contrib' is renamed to 'lwip-contrib-freertos'
    <td> Additionally while using NoRTOS (bare metal) based examples nortos version of 'lwipif-cpsw' and 'lwip-contrib' needs to be used
</tr>
</table>



