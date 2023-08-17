# Release Notes 09.00.00 {#RELEASE_NOTES_09_00_00_PAGE}

[TOC]

\attention Also refer to individual module pages for more details on each feature, unsupported features, important usage guidelines.

\note The examples will show usage of SW modules and APIs on a specific CPU instance and OS combination. \n
      Unless noted otherwise, the SW modules would work in both FreeRTOS and NO-RTOS environment. \n
      Unless noted otherwise, the SW modules would work on any of the CPU present on the SOC. \n
      Unless noted otherwise, the SW modules would work on all supported EVMs \n

\attention Klockwork Static Analysis report is not updated for this release

## New in this Release

\cond SOC_AM273X
Feature                                                                                         | Module
------------------------------------------------------------------------------------------------|--------------------------
Standby (WFI) mode support DPL noRTOS                                                           | DPL
EDMA example to transfer between different memories                                             | EDMA
Interrupt profiling support and example                                                         | DPL
Support for loading HS-FS firmware view CCS load script                                         | Common
SBL over CAN example                                                                            | SBL
SDK migration script from ZCE to NZN package                                                    | Common
MbedTLS MQTT example                                                                            | Networking
Support added for Single Packet reception from DMA                                              | Networking
SDL ECC examples for DSS L1 and L2                                                              | SDL
SDL is safety certified from TuV for ISO26262/IEC61508 for MCU PLUS SDK version 8.6.0.          | SDL

\endcond
\cond SOC_AWR294X
Feature                                                                                         | Module
------------------------------------------------------------------------------------------------|--------------------------
-                                                                                               | -
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
Mbed-TLS                | R5F            | @VAR_MBEDTLS_VERSION

DSP LIB package is modified to fix the build in Linux environment from the base version dsplib_c66x_3_4_0_0

\attention TI ARM CLANG @VAR_TI_ARM_CLANG_VERSION is not part of CCS by default, Follow steps at \ref INSTALL_TIARMCLANG to install the compiler

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
PMU        | R5F            | NO                | NA            | Tested various PMU events                                                                                   | Counter overflow detection is not enabled
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
\cond SOC_AWR294X
Module                      | Supported CPUs | SysConfig Support | OS Support  | Key features tested                                                                    | Key features not tested
----------------------------|----------------|-------------------|-------------|----------------------------------------------------------------------------------------|------------------------
LwIP                        | R5F            | NO                | FreeRTOS    | TCP/UDP IP networking stack, DHCP, ping, TCP iperf, TCP/UDP IP, scatter-gather                        | Other LwIP features, more robustness tests pending
Ethernet driver (ENET)      | R5F            | NO                | FreeRTOS    | Ethernet as port using CPSW; MAC loopback and PHY loopback, interrupt pacing, MDIO Manual Mode       | Ethernet as switch
\endcond

\cond SOC_AM273X
Module                      | Supported CPUs | SysConfig Support | OS Support  | Key features tested                                                                    | Key features not tested
----------------------------|----------------|-------------------|-------------|----------------------------------------------------------------------------------------|------------------------
LwIP                        | R5F            | NO                | FreeRTOS    | TCP/UDP IP networking stack, TCP/UDP IP networking stack with server and client functionality, basic Socket APIs, netconn APIs and raw APIs, DHCP, ping, TCP iperf, scatter-gather,                         | Other LwIP features
Ethernet driver (ENET)      | R5F            | NO                | FreeRTOS    | Ethernet as port using CPSW, MAC loopback and PHY loopback,  Layer 2 MAC, Packet Timestamping, interrupt pacing, Policer and Classifier, MDIO Manual Mode, CBS (IEEE 802.1Qav)       | Ethernet as switch, MII and RMII modes
Mbed-TLS                    | R5F            | NO                | FreeRTOS    | Tested software cryptography after porting, used mbedTLS with LwIP to implement HTTPS server  | Hardware offloaded cryptography
\endcond

### Safety Diagnostic Library

Module            | Supported CPUs  | SysConfig Support | OS support       | Key features tested                                                                            | Key features not tested / NOT supported
------------------|-----------------|-------------------|------------------|------------------------------------------------------------------------------------------------|----------------------------------------
MCRC              | R5F, C66        | NA                |  NORTOS | Full CPU Mode, Auto CPU Mode and Semi CPU Mode.                                                            | -
DCC               | R5F, C66        | NA                |  NORTOS | Single Shot Mode, Continuous Mode                                   |-
PBIST             | R5F             | NA                |  NORTOS | Memories supported by MSS and DSS PBIST controller.          |-
ESM               | R5F, C66        | NA                |  NORTOS | Tested in combination with RTI, DCC, ECC                                        |-
RTI               | R5F, C66        | NA                |  NORTOS | WINDOWSIZE_100_PERCENT, WINDOWSIZE_50_PERCENT ,Latency/Propagation timing error(early)(50% window),Latency/Propagation timing error(late)(50% window)                                     | -
ECC               | R5F, C66        | NA                |  NORTOS | ECC of MSS_L2, Mailbox, TPTC, R5SS TCM, MCAN, VIM, DSP L1/L2/L3, HWA     | R5F Cache
ECC Bus Safety    | R5F, C66        | NA                |  NORTOS | Bus Safety of Mailbox, DSS L3, HWA, ADCBUF, DSS_PCR, MSS_TPTC, CORE A and B AHB, MCRC, MSS_CR5 ,MSS_QSPI, MSS_PCR, MSS_SWBUF, MSS_GPADC, DSS_DSP_SDMA,MSS_TO_MDO  | MSS_CPSW, MSS_L2, DAP_R232, DSS_DSP_MDMA
HWA               | C66             | NA                |  NORTOS | Parity on Data Memories, Window Memory and FSM Lockstep                                                 | -
CCM               | R5F             | NA                |  NORTOS | CCM Self Test Mode, Error Forcing Mode and Self Test Error Forcing Mode                                 | -
R5F STC(LBIST)    | R5F             | NA                |  NORTOS | STC of R5F and DSP.                                                 |-

## Fixed Issues

<table>
<tr>
    <th> ID
    <th> Head Line
    <th> Module
    <th> Applicable Releases
    <th> Resolution/Comments
</tr>
<tr>
    <td> MCUSDK-9811
    <td> IPC Notify unregister client always returns success
    <td> IPC
    <td> 8.05.00 onwards
    <td> Added status variable check
</tr>
<tr>
    <td> MCUSDK-9835
    <td> SBL should support HS-SE device build via CCS
    <td> SBL
    <td> 8.05.00 onwards
    <td> Added devconfig for CCS build
</tr>
<tr>
    <td> MCUSDK-10144
    <td> RTI driver Up Counter calculation in driver is incorrect
    <td> Timer
    <td> 8.06.00 onwards
    <td> Updated calculation to subtract reload value by one
</tr>
<tr>
    <td> MCUSDK-10690
    <td> EPWM Driver configures wrong register in "SOC_setEpwmTbClk" API
    <td> EPWM
    <td> 8.05.00 onwards
    <td> -
</tr>
<tr>
    <td> MCUSDK-10841
    <td> SBL needs to copy the vector table for self core right before the self-core reset release
    <td> SBL
    <td> 8.06.00 onwards
    <td> Move the image load for self core just before the release reset instead of doing it early
</tr>
<tr>
    <td> MCUSDK-11207
    <td> Incorrect IOCTL params for CPSW ENET_TIMESYNC_IOCTL_SET_TIMESTAMP
    <td> Networking
    <td> 8.5.0
    <td> -
</tr>
<tr>
    <td> MCUSDK-10775
    <td> Example build failing on enabling External Phy Management
    <td> Networking
    <td> 8.6.0
    <td> -
</tr>
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
    <td> MCUSDK-11518
    <td> IPC: Firewalling error due to Alignment not being per core pair.
    <td> IPC
    <td> 09.00.00
    <td> Firewalling should not be enabled for SafeIPC.
</tr>
<tr>
    <td> MCUSDK-10978
    <td> CCS build doesn't support for HS appimages generation
    <td> CCS
    <td> 09.00.00
    <td> Use make/gmake based build for HS-SE
</tr>
<tr>
    <td> <a href="https://mbed-tls.readthedocs.io/en/latest/tech-updates/security-advisories/mbedtls-security-advisory-2021-07-1/">mbedTLS-advisory</a> <br> MCUSDK-9082
    <td> MbedTLS - RSA exploit by kernel-privileged cache side-channel attackers
    <td> Mbed-TLS
    <td> 8.6.0 onwards
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
    <td> PROC_SDL-4749
    <td> AXI DED Bus Safety fail.
    <td> SDL
    <td> 8.5.0 onwards
    <td> None.
</tr>
<tr>
    <td> PROC_SDL-5159
    <td> SEC ECC Bus Safety for MSS_AXI_RD not supported.
    <td> SDL
    <td> 8.5.0 onwards
    <td> None.
</tr>
<tr>
    <td> PROC_SDL-5616
    <td> For ECC Bus Safety, SEC and DED are not supported for CPSW.
    <td> SDL
    <td> 8.6.0 onwards
    <td> None.
</tr>
<tr>
    <td> PROC_SDL-5617
    <td> ECC Bus safety for SEC and DED not supported for MSS_L2.
    <td> SDL
    <td> 8.6.0 onwards
    <td> None.
</tr>
<tr>
    <td> PROC_SDL-5650
    <td> ECC Bus safety for SEC and DED not supported for DSS_DSP_MDMA.
    <td> SDL
    <td> 8.6.0 onwards
    <td> None.
</tr>
<tr>
    <td> MCUSDK-11506
    <td> ENET: CPDMA Goes To Lockup State.
    <td> CPSW
    <td> 8.5.0 onwards
    <td> Disable THOST checksum Offload.
</tr>
<tr>
    <td> MCUSDK-11507
    <td> ENET: CPSW MAC port is stuck forever and dropping all the Rx/Tx packets with reception of corrupts preamble.
    <td> CPSW
    <td> 8.2.0 onwards
    <td> Disable hostRxTimestampEn flag in CPSW CPST configuration. This does not impact the CPTS Rx or Tx Timestamp Events for PTP packets and is orthogonal feature.
</tr>
\endcond
</table>

\cond SOC_AM273X
## Errata
<table>
<tr>
    <th> ID
    <th> Head Line
    <th> Module
    <th> SDK Status
</tr>
<tr>
    <td> i2338
    <td> Spurious RX DMA REQ From a Peripheral Mode MibSPI
    <td> MibSPI
    <td> Open
</tr>
<tr>
    <td> i2345
    <td> CPSW: Ethernet Packet corruption occurs if CPDMA fetches a packet which spans across memory banks
    <td> CPSW
    <td> Implemented
</tr>
<tr>
    <td> i2387
    <td> PLL: Boot fails sometimes because of possible glitch in R5F GCM
    <td> SBL
    <td> Implemented
</tr>
<tr>
    <td> i2389
    <td> Recommended PLL Configuration if locked below 1GHz
    <td> SBL
    <td> Implemented
</tr>
<tr>
    <td> i2394
    <td> Race condition in interrupt and error aggregator capture registers resulting in events miss
    <td> Interrupt
    <td> Open
</tr>
<tr>
    <td> i2392
    <td> Race condition in capture registers resulting in events miss
    <td> Interrupt
    <td> Open
</tr>
<tr>
    <td> i2339
    <td> MibSPI RX RAM RXEMPTY Bit Does Not Get Cleared After Reading
    <td> MibSPI
    <td> Open
</tr>
<tr>
    <td> i2390
    <td> Recommended HWA memInit Sequence
    <td> HWA
    <td> Open
</tr>
<tr>
    <td> i2342
    <td> 2D Stats sample value RAM processor write back issue during FFT execution on HWA
    <td> HWA
    <td> Open
</tr>
<tr>
    <td> i2341
    <td> Unallocated space access to DSP L2 - DSP IP is not blocking access to reserved space causing aliasing and L2 parity error
    <td> DSP-L2
    <td> Open
</tr>
<tr>
    <td> i2337
    <td> A Data Length Error is Generated Repeatedly in Peripheral Mode When IO Loopback is Enabled
    <td> MibSPI
    <td> Open
</tr>
<tr>
    <td> i2336
    <td> MibSPI in Peipheral Mode in 3- or 4-Pin Communication Transmits Data Incorrectly for Slow SPICLK Frequencies and for Clock Phase = 1
    <td> MibSPI
    <td> Open
</tr>
<tr>
    <td> i2297
    <td> CSI Careabouts
    <td> CSI
    <td> Open
</tr>
<tr>
    <td> i2294
    <td> Subsequent memory initialisation configuration of L3 Bank D will not trigger a memory initialisation
    <td> Common
    <td> Open
</tr>
<tr>
    <td> i2289
    <td> Unaligned access from DSS CM4 could cause data integrity failure and hang
    <td> HWA
    <td> Open
</tr>
<tr>
    <td> i2288
    <td> EDMA transfer that spans M1+M2 memories of HWA could result in data corruption
    <td> HWA
    <td> Open
</tr>
</table>
\endcond

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
</table>

### SOC Device Drivers

<table>
<tr>
    <th> Module
    <th> Affected API
    <th> Change
    <th> Additional Remarks
</tr>
\cond SOC_AWR294X
<tr>
    <td> ADCBUF
    <td> Changed the argument type of ADCBUFCmdParamCheck api from ADCBufMMWave_CMD to uint8_t.
    <td> Repace the enum type ADCBufMMWave_CMD with macros.
    <td> Safety driver update
</tr>
\endcond
</table>

### Networking

<table>
<tr>
    <th> Module
    <th> Affected API
    <th> Change
    <th> Additional Remarks
</tr>
</table>


