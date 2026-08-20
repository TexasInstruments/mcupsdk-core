# Release Notes 12.01.00 {#RELEASE_NOTES_12_01_00_PAGE}

[TOC]

\attention 1. Also refer to individual module pages for more details on each feature, unsupported features, important usage guidelines.

\attention 2. A53 support is applicable for AM64x only. It is NOT applicable for AM243x. \n

\attention 3. RPRC image format has been deprecated from this release. Multi Core ELF image format should be used. (\ref MCELF_LANDING).

\note The examples will show usage of SW modules and APIs on a specific CPU instance and OS combination. \n
      Unless noted otherwise, the SW modules would work in both FreeRTOS and NORTOS environment. \n
      Unless noted otherwise, the SW modules would work on any of the R5F's present on the SOC. \n
      Unless noted otherwise, the SW modules would work on all supported EVMs \n
      M4F drivers support only MCU domain peripheral and peripheral instance while R5/A53 supports MAIN domain peripheral and peripheral instance. \n

\attention Klockwork Static Analysis report is not updated for this release

## New in this Release

\cond SOC_AM64X
Feature                                                                                      | Module
---------------------------------------------------------------------------------------------|-----------------------------------
A53 core frequency set to maximum by default                                                | Bootloader
IPC RPMsg Linux example: Added remote core shutdown feature                                 | IPC
New example: Added OSPI PHY Grapher based on UART                                           | OSPI
New example: Added MCRC Auto and Semi CPU Mode                                              | SDL
New example: Added R5F TCM ECC memory test                                                  | SDL
\endcond

\cond SOC_AM243X
Feature                                                                                      | Module
---------------------------------------------------------------------------------------------|-----------------------------------
New example: Added OSPI PHY Grapher based on UART                                           | OSPI
New example: Added MCRC Auto and Semi CPU Mode                                              | SDL
New example: Added R5F TCM ECC memory test                                                  | SDL
\endcond

## Device and Validation Information

\cond SOC_AM64X
SOC   | Supported CPUs  | EVM                                             | Host PC
------|-----------------|-------------------------------------------------|-----------------------------------
AM64x | R5F, M4F, A53   | AM64x GP EVM (referred to as am64x-evm in code) | Windows 10 64b or Ubuntu 18.04 64b
^     | ^               | AM64x SK EVM (Limited examples are added) (referred to as am64x-sk in code) |^
\endcond

\cond SOC_AM243X
SOC    | Supported CPUs  | Boards                                                                                                      | Host PC
-------|-----------------|-------------------------------------------------------------------------------------------------------------|-----------------------------------
AM243x | R5F, M4F        | AM243x GP EVM (referred to as am243x-evm in code), \n AM243x LAUNCHPAD (referred to as am243x-lp in code)   | Windows 10 64b or Ubuntu 18.04 64b
\endcond

## Tools, Compiler and Other Open Source SW Module Information

Tools / SW module       | Supported CPUs | Version
------------------------|----------------|-----------------------
Code Composer Studio    | R5F, M4F, A53  | 20.5.0
SysConfig               | R5F, M4F, A53  | 1.27.0, build 4000
TI ARM CLANG            | R5F, M4F       | 4.0.4.LTS
GCC AARCH64             | A53            | 9.2-2019.12
FreeRTOS Kernel         | R5F, M4F, A53  | 11.1.0
FreeRTOS SMP Kernel     | A53            | 202110.00-SMP
Tiny USB                | R5F            | 0.14.0
LwIP                    | R5F            | STABLE-2_2_0_RELEASE
Mbed-TLS                | R5F            | mbedtls-2.13.1

## SDK Components

### SYSFW / DMSC

<table>
    <tr>
        <td>Version</td>
        <td>12.01.02</td>
    </tr>
    <tr>
        <td>Release Notes</td>
        <td>[LINK](https://software-dl.ti.com/tisci/esd/12_01_02/release_notes/release_notes.html)</td>
    </tr>
    <tr>
        <td>User Guide</td>
        <td>[LINK](https://software-dl.ti.com/tisci/esd/12_01_02/1_intro/TISCI.html)</td>
    </tr>
</table>

## Key Features

### Experimental Features {#EXPERIMENTAL_FEATURES}

\attention Features listed below are early versions and should be considered as "experimental".
\attention Users can evaluate the feature, however the feature is not fully tested at TI side.
\attention TI would not support these feature on public e2e.
\attention Experimental features will be enabled with limited examples and SW modules.

\cond SOC_AM64X
Feature                                                             | Module
--------------------------------------------------------------------|--------------------------
A53 NORTOS support and A53 NORTOS examples                          | DPL, NORTOS
A53 FreeRTOS support and A53 FreeRTOS examples                      | DPL, FreeRTOS
SBL booting A53 NORTOS                                              | Bootloader
A53 FreeRTOS AMP mode and A53 AMP FreeRTOS examples                 | DPL, FreeRTOS
GUI for UART Uniflash Tool (No support for EMMC flashing)           | Bootloader


\endcond

\cond SOC_AM243X

Feature                                                             | Module
--------------------------------------------------------------------|--------------------------
GUI for UART Uniflash Tool (No support for EMMC flashing)           | Bootloader

\endcond

### Features not supported in release

\cond SOC_AM64X
Feature                                                                                         | Module
------------------------------------------------------------------------------------------------|-------------
NORTOS based Ethernet MAC and switch driver support for A53 core                                | Ethernet
\endcond

\cond SOC_AM243X

### AM243X LAUNCHPAD not tested/not supported features

Below features are not support on AM243X LAUNCHPAD due to SOC or board constraints,

- DDR is not supported on the AM243X 11x11 SOC used in AM243X LAUNCHPAD.
- I2C temperature sensor example not validated, due to board limitation.
- M4F examples for UART, MCSPI and GPIO not validated, due to board limitation.
\endcond

### OS Kernel

OS                  | Supported CPUs  | SysConfig Support | Key features tested                                                                                 | Key features not tested / NOT supported
--------------------|-----------------|-------------------|-----------------------------------------------------------------------------------------------------|----------------------------------------
FreeRTOS Kernel     | R5F, M4F, A53   | NA                | Task, Task notification, interrupts, semaphores, mutexes, timers, event groups. ROV views in CCS IDE, Task load measurement using FreeRTOS run time statistics APIs. | Only single core A53 FreeRTOS is supported. Second core is NOT used.
FreeRTOS SMP Kernel | A53             | NA                | Task, Task notification, interrupts, semaphores, mutexes, timers, event groups. ROV views in CCS IDE, Task load measurement using FreeRTOS run time statistics APIs. | -
FreeRTOS POSIX      | R5F, M4F, A53   | NA                | pthread, mqueue, semaphore, clock                                                                   | -
NO RTOS             | R5F, M4F, A53   | NA                | See **Driver Porting Layer (DPL)** below                                                            | Only single core A53 NORTOS is supported. Second core is NOT used.

### Driver Porting Layer (DPL)

Module            | Supported CPUs  | SysConfig Support | OS support       | Key features tested                                           | Key features not tested / NOT supported
------------------|-----------------|-------------------|------------------|---------------------------------------------------------------|----------------------------------------
Address Translate | M4F             | YES               | FreeRTOS, NORTOS | Use RAT to allow M4F access to peripheral address space       | -
Cache             | R5F, A53        | YES               | FreeRTOS, NORTOS | Cache write back, invalidate, enable/disable                  | -
Clock             | R5F, M4F, A53   | YES               | FreeRTOS, NORTOS | Tick timer at user specified resolution, timeouts and delays  | -
CpuId             | R5F             | NA                | FreeRTOS, NORTOS | Verify Core ID and Cluster ID that application is running     | -
CycleCounter      | R5F, M4F, A53   | NA                | FreeRTOS, NORTOS | Measure CPU cycles using CPU specific internal counters       | -
Debug             | R5F, M4F, A53   | YES               | FreeRTOS, NORTOS | Logging and assert to any combo of: UART, CCS, shared memory  | -
Heap              | R5F, M4F, A53   | NA                | FreeRTOS, NORTOS | Create arbitrary heaps in user defined memory segments        | -
Hwi               | R5F, M4F, A53   | YES               | FreeRTOS, NORTOS | Interrupt register, enable/disable/restore                    | -
MPU               | R5F, M4F        | YES               | FreeRTOS, NORTOS | Setup MPU and control access to address space                 | -
MMU               | A53             | YES               | NORTOS           | Setup MMU and control access to address space                 | -
Semaphore         | R5F, M4F, A53   | NA                | FreeRTOS, NORTOS | Binary, Counting Semaphore, recursive mutexs with timeout     | -
Task              | R5F, M4F, A53   | NA                | FreeRTOS         | Create, delete tasks                                          | -
Timer             | R5F, M4F, A53   | YES               | FreeRTOS, NORTOS | Configure arbitrary timers                                    | -
Event             | R5F, M4F        | YES               | FreeRTOS         | Setting, getting, clearing, and waiting of Event bits         | -

### Secondary Bootloader (SBL)

Module     | Supported CPUs  | SysConfig Support | OS support       | Key features tested                                                                             | Key features not tested / NOT supported
-----------|-----------------|-------------------|------------------|-------------------------------------------------------------------------------------------------|----------------------------------------
Bootloader | R5FSS0-0        | YES               | NORTOS           | Boot modes: OSPI, OSPI XIP, UART, SD. All R5F's, M4F, A53 NORTOS/FreeRTOS/Linux boot. RPRC, multi-core image format, DDR init  | SBL OSPI XIP for A53

### SOC Device Drivers

Please refer to \ref BUILDSHEET for SOC Device Drivers support across different CPUs.

### Software Diagnostic Library (SDL)
\cond SOC_AM243X
SDL Module| Supported CPUs | SysConfig Support
----------|----------------|-------------------
ESM       | M4F, R5F       | NO
MCRC      | M4F, R5F       | NO
RTI       | M4F, R5F       | NO
DCC       | M4F, R5F       | NO
VTM       | M4F, R5F       | NO
STOG      | M4F, R5F       | NO
PBIST     | M4F, R5F       | NO
MTOG      | M4F            | NO
POK       | M4F, R5F       | NO
ECC       | M4F, R5F       | NO
ROM Checksum| R5F          | NO
\endcond

\cond SOC_AM64X
SDL Module| Supported CPUs | SysConfig Support
----------|----------------|-------------------
ESM       | M4F, R5F       | NO
MCRC      | M4F, R5F       | NO
RTI       | M4F, R5F       | NO
DCC       | M4F, R5F       | NO
VTM       | M4F, R5F       | NO
STOG      | M4F, R5F       | NO
PBIST     | M4F, R5F       | NO
MTOG      | M4F            | NO
POK       | M4F, R5F       | NO
ECC       | M4F, R5F       | NO
LBIST     | M4F            | NO
ROM Checksum| R5F          | NO
\endcond

### Board Device Drivers

Peripheral | Supported CPUs | SysConfig Support | Key features tested                                                                           | Key features not tested
-----------|----------------|-------------------|-----------------------------------------------------------------------------------------------|------------------------
EEPROM     | R5F            | YES               | I2C based EEPROM                                                                              | -
ETHPHY     | R5F            | YES               | Ethernet Phy configuration for EtherCAT SubDevice example                                         | -
Flash      | R5F            | YES               | XSPI, OSPI, QSPI based flash, Octal, Quad mode, DDR mode                                      | All vendor flash types not tested
LED        | R5F, A53       | YES               | GPIO , I2C IO expander based LED control, I2C based industrial LEDs(TPIC2810)                 | -

### File System

Module                      | Supported CPUs | SysConfig Support | OS Support        | Key features tested                                                                         | Key features not tested
----------------------------|----------------|-------------------|-------------------|---------------------------------------------------------------------------------------------|------------------------
FreeRTOS+FAT                | R5F            | YES               | FreeRTOS, NORTOS  | File read, write, create. FAT partition and mounting                                        | File I/O with FreeRTOS

### CMSIS

Module                      | Supported CPUs | SysConfig Support | OS Support        | Key features tested                                                                         | Key features not tested
----------------------------|----------------|-------------------|-------------------|---------------------------------------------------------------------------------------------|------------------------
CMSIS DSP                   | R5F            | NA                | FreeRTOS, NORTOS  | Basic math, complex math, controller, fast math, filtering, Matrix, statistics, transform   | -

### Networking

Module                  | Supported CPUs | SysConfig Support | OS Support          | Key features tested                                                                                            | Key features not tested
------------------------|----------------|-------------------|---------------------|----------------------------------------------------------------------------------------------------------------|------------------------
TSN                     | R5F            | NO                | FreeRTOS            | gPTP IEEE 802.1 AS-2020 compliant gPTP stack, End Nodes and Bridge mode support, YANG data model configuration | Multi-Clock Domain
^                       | A53            | NO                | FreeRTOS            | ^                                                                                                              | ^
LwIP                    | R5F            | YES               | FreeRTOS, NORTOS    | TCP/UDP IP networking stack with and without checksum offload enabled, TCP/UDP IP networking stack with server and client functionality, basic Socket APIs, netconn APIs and raw APIs, DHCP, ping, TCP iperf, scatter-gather, DSCP priority mapping, LwIP bridge, shared memory driver | Other LwIP features
^                       | A53            | YES               | FreeRTOS            | ^                                                                                                              | ^
Ethernet driver (ENET)  | R5F            | YES               | FreeRTOS, NORTOS    | Ethernet as port using CPSW,  MAC loopback and PHY loopback, Layer 2 MAC, Packet Timestamping, CPSW Switch, Policer and Classifier, MDIO Manual Mode, CBS (IEEE 802.1Qav) on CPSW, IET (IEEE 802.1Qbu) on CPSW, Strapped PHY (Early Ethernet), cut through switch on CPSW | RMII mode
^                       | A53            | YES               | FreeRTOS            | ^                                                                                                              | ^
Mbed-TLS                | R5F            | NO                | FreeRTOS            | Tested software cryptography after porting, used mbedTLS with LwIP to implement HTTPS server                   | Hardware offloaded cryptography

### USB

Module                      | Supported CPUs | SysConfig Support | OS Support         | Key features tested       | Key features not tested
----------------------------|----------------|-------------------|--------------------|---------------------------|------------------------
USB SoC Porting Layer       | R5F            | YES               | FreeRTOS, NORTOS   | USB 2.0 device mode       | USB 3.0
USB Device Driver           | R5F            | NO                | FreeRTOS, NORTOS   | USB 2.0 device mode       | USB Host driver
TinyUSB Core and CDC Driver | R5F            | NO                | FreeRTOS, NORTOS   | USB device with CDC class | USB Host, other USB device class drivers
TinyUSB Core and DFU Driver | R5F            | NO                | FreeRTOS, NORTOS   | USB device with DFU class | USB Host, other USB device class drivers

### SECURITY

Module          | Supported CPUs    | SysConfig Support | OS Support        | Key features tested                                                                                                                                                                       | Key features not tested
----------------|-------------------|-------------------|-------------------|-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|-------------------------------------------------
SA2UL AES       | R5F               | YES               | NORTOS            | AES CBC-128 encryption and decryption, AES CBC-256 encryption and decryption, AES ECB-128 encryption and decryption, AES ECB-256 encryption and decryption, AES CMAC-128, AES CMAC-256    | -
SA2UL SHA       | R5F               | YES               | NORTOS            | SHA 512 single shot and multi-shot, SHA 256 single shot and multi-shot, HMAC SHA-256, HMAC SHA-512, HMAC SHA-1                                                                            | -
SA2UL RNG       | R5F               | YES               | NORTOS            | RNG generate random number with size of 4 words(128 bit)                                                                                                                                  | -
SA2UL PKA       | R5F               | YES               | NORTOS            | RSA Encryption and Decryption support upto 4k bit, RSA Signing and Verification support upto 4k bit, ECDSA Signing and Verification support with P-256 and P-384 curves                   | -

### Demos

Module                      | Supported CPUs | SysConfig Support | OS Support        | Key features tested                             | Key features not tested
----------------------------|----------------|-------------------|-------------------|-------------------------------------------------|------------------------
Benchmark demo              | 4xR5F's        | YES               | NORTOS            | CFFT, FIR and FOC benchmarks                    |  ADC/PWM benchmark

## Fixed Issues

<table>
<tr>
    <th> ID
    <th> Head Line
    <th> Module
</tr>
<tr>
    <td> \htmllink{https://sir.ext.ti.com/jira/browse/EXT_SITMPUSW-263, EXT_SITMPUSW-263}
    <td> Design choice for Security Handover is not documented
    <td> Documentation
</tr>
<tr>
    <td> \htmllink{https://sir.ext.ti.com/jira/browse/EXT_SITMPUSW-338, EXT_SITMPUSW-338}
    <td> Flash writes fail if offset / address is not block aligned
    <td> OSPI
</tr>
<tr>
    <td> \htmllink{https://sir.ext.ti.com/jira/browse/EXT_SITMPUSW-329, EXT_SITMPUSW-329}
    <td> Incorrect Value of UART_EFR2_TIMEOUT_BEHAVE_MASK
    <td> UART
</tr>
<tr>
    <td> \htmllink{https://sir.ext.ti.com/jira/browse/EXT_EP-13067, EXT_EP-13067}
    <td> AM64x: PBIST: A53 forcebit power off failure in M4F applications
    <td> SDL-PBIST
</tr>
<tr>
    <td> \htmllink{https://sir.ext.ti.com/jira/browse/EXT_EP-13576, EXT_EP-13576}
    <td> Need to enable the POK bandgap bit for reference voltage
    <td> SDL-POK
</tr>
</table>

## Known Issues

<table>
<tr>
    <th> ID
    <th> Head Line
    <th> Module
</tr>
<tr>
    <td> \htmllink{https://sir.ext.ti.com/jira/browse/EXT_SITMPUSW-347, EXT_SITMPUSW-347}
    <td> QoS DDR docs are wrong
    <td> -
</tr>
<tr>
    <td> \htmllink{https://sir.ext.ti.com/jira/browse/EXT_SITMPUSW-353, EXT_SITMPUSW-353}
    <td> UART DMA ring mem is not allocated from separated pool, causing unwanted CacheP_inv on other areas
    <td> Baseport
</tr>
<tr>
    <td> \htmllink{https://sir.ext.ti.com/jira/browse/EXT_SITMPUSW-153, EXT_SITMPUSW-153}
    <td> MCU+ SDK CCS Project Build Generates Invalid/Redundant Boot Image Files
    <td> Build
</tr>
<tr>
    <td> \htmllink{https://sir.ext.ti.com/jira/browse/EXT_SITMPUSW-322, EXT_SITMPUSW-322}
    <td> IV is set incorrectly in the SA2UL driver
    <td> Crypto
</tr>
<tr>
    <td> \htmllink{https://sir.ext.ti.com/jira/browse/EXT_SITMPUSW-24, EXT_SITMPUSW-24}
    <td> [SA2UL][PKA] ECDSA Sign/verify not working with P-521 and BrainPool P-512R1 Curves
    <td> Crypto
</tr>
<tr>
    <td> \htmllink{https://sir.ext.ti.com/jira/browse/EXT_SITMPUSW-354, EXT_SITMPUSW-354}
    <td> Udma_defaultVirtToPhyFxn does not takes the compiler dependency into account
    <td> DMA
</tr>
<tr>
    <td> \htmllink{https://sir.ext.ti.com/jira/browse/EXT_SITMPUSW-351, EXT_SITMPUSW-351}
    <td> AM64X : ClockP_usleep function is not working properly in FreeRTOS
    <td> DPL
</tr>
<tr>
    <td> \htmllink{https://sir.ext.ti.com/jira/browse/EXT_SITMPUSW-346, EXT_SITMPUSW-346}
    <td> ENET-ICSS: Low Tx throughput for TCP iperf bidirectional tests
    <td> ENET_ICSS
</tr>
<tr>
    <td> \htmllink{https://sir.ext.ti.com/jira/browse/EXT_SITMPUSW-350, EXT_SITMPUSW-350}
    <td> ENET-LLD: UDP iperf parameters ignored by DUT
    <td> ENET_ICSS
</tr>
<tr>
    <td> \htmllink{https://sir.ext.ti.com/jira/browse/EXT_SITMPUSW-81, EXT_SITMPUSW-81}
    <td> MCU+ SDK: EPWM_tbTimebaseClkCfg does not choose the optimal pre-scaler combination
    <td> EPWM
</tr>
<tr>
    <td> \htmllink{https://sir.ext.ti.com/jira/browse/EXT_SITMPUSW-348, EXT_SITMPUSW-348}
    <td> PRIMASK Implementation based critical sections
    <td> FreeRTOS
</tr>
<tr>
    <td> \htmllink{https://sir.ext.ti.com/jira/browse/EXT_SITMPUSW-86, EXT_SITMPUSW-86}
    <td> The configurations of GPIO interrupt routers are based on banks rather than individual pins.
    <td> GPIO
</tr>
<tr>
    <td> \htmllink{https://sir.ext.ti.com/jira/browse/EXT_SITMPUSW-325, EXT_SITMPUSW-325}
    <td> IRQStatus Write To Clear
    <td> GPTIMER
</tr>
<tr>
    <td> \htmllink{https://sir.ext.ti.com/jira/browse/EXT_SITMPUSW-82, EXT_SITMPUSW-82}
    <td> AM64/AM243: M4F core getting hangs when doing IPC with Linux
    <td> IPC
</tr>
<tr>
    <td> \htmllink{https://sir.ext.ti.com/jira/browse/EXT_SITMPUSW-262, EXT_SITMPUSW-262}
    <td> mmcsd_raw_io_emmc_lld example is not working
    <td> MMCSD
</tr>
<tr>
    <td> \htmllink{https://sir.ext.ti.com/jira/browse/EXT_SITMPUSW-273, EXT_SITMPUSW-273}
    <td> AM64x PCIe as End Point throwing error when changing BAR aperture
    <td> PCIE
</tr>
<tr>
    <td> \htmllink{https://sir.ext.ti.com/jira/browse/EXT_SITMPUSW-26, EXT_SITMPUSW-26}
    <td> AM64X PCIe MSI error when connected to Linux Root Complex
    <td> PCIE
</tr>
<tr>
    <td> \htmllink{https://sir.ext.ti.com/jira/browse/EXT_SITMPUSW-22, EXT_SITMPUSW-22}
    <td> Pcie_benchmark, Pcie_buf_transfer, Pcie_legacy_irq, Pcie_msi_irq and Pcie_msix_irq are broken on 9.2.1 release
    <td> PCIE
</tr>
<tr>
    <td> \htmllink{https://sir.ext.ti.com/jira/browse/EXT_SITMPUSW-300, EXT_SITMPUSW-300}
    <td> Authentication of appimage fails
    <td> SBL
</tr>
<tr>
    <td> \htmllink{https://sir.ext.ti.com/jira/browse/EXT_SITMPUSW-25, EXT_SITMPUSW-25}
    <td> AM243x/AM64x: last 512KB of memory is not accessible in dev boot mode flow
    <td> SBL
</tr>
<tr>
    <td> \htmllink{https://sir.ext.ti.com/jira/browse/EXT_EP-13068, EXT_EP-13068}
    <td> AM64x: AM243x: ECC: Aggregator failures in ecc applications.
    <td> SDL-ECC
</tr>
<tr>
    <td> \htmllink{https://sir.ext.ti.com/jira/browse/EXT_EP-12276, EXT_EP-12276}
    <td> ECC: Firewall related aggregators failures
    <td> SDL-ECC
</tr>
</table>

## Errata
<table>
<tr>
    <th> ID
    <th> Head Line
    <th> Module
    <th> SDK Status
</tr>
<tr>
    <td> i2278
    <td> MCAN: Message Transmit order not guaranteed from dedicated Tx Buffers configured with same Message ID
    <td> MCAN
    <td> Open
</tr>
<tr>
    <td> i2279
    <td> MCAN: Specification Update for dedicated Tx Buffers and Tx Queues configured with same Message ID
    <td> MCAN
    <td> Open
</tr>
<tr>
    <td> i2310
    <td> USART: Erroneous clear/trigger of timeout interrupt
    <td> UART
    <td> Implemented
</tr>
<tr>
    <td> i2311
    <td> USART: Spurious DMA Interrupts
    <td> UART
    <td> Implemented
</tr>
<tr>
    <td> i2312
    <td> MMCSD: HS200 and SDR104 Command Timeout Window Too Small
    <td> MMCSD
    <td> Open
</tr>
<tr>
    <td> i2313
    <td> GPMC: Sub-32-bit read issue with NAND and FPGA/FIFO
    <td> GPMC
    <td> Implemented
</tr>
<tr>
    <td> i2326
    <td> PCIe: MAIN_PLLx operating in fractional mode, which is required for enabling SSC, is not compliant with PCIe Refclk jitter limits
    <td> PCIe
    <td> Open
</tr>
<tr>
    <td> i2329
    <td> MDIO interface corruption,
    <td> CPSW, ICSSG
    <td> Open
</tr>
<tr>
    <td> i2331
    <td> CPSW: Device lockup when reading CPSW registers
    <td> CPSW, SBL
    <td> Implemented
</tr>
<tr>
    <td> i2345
    <td> CPSW: Ethernet Packet corruption occurs if CPDMA fetches a packet which spans across memory banks
    <td> CPSW
    <td> Implemented
</tr>
<tr>
    <td> i2401
    <td> CPSW: Host Timestamps Cause CPSW Port to Lock up
    <td> CPSW
    <td> Open
</tr>
<tr>
    <td> i2402
    <td> CPSW: Ethernet to Host Checksum Offload does not work
    <td> CPSW
    <td> Open
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
    <td> \htmllink{https://sir.ext.ti.com/jira/browse/EXT_SITMPUSW-27, EXT_SITMPUSW-27}
    <td> gmake with -j can sometimes lock up Windows command prompt
    <td> Build
    <td> 7.3.0
    <td> AM64x, AM243x
    <td> Use bash for windows as part of git for windows or don't use -j option
</tr>
</table>

## Upgrade and Compatibility Information

\attention When migrating from Processor SDK RTOS, see \ref MIGRATION_GUIDES for more details

This section lists changes which could affect user applications developed using older SDK versions.
Read this carefully to see if you need to do any changes in your existing application when migrating to this SDK version relative to
previous SDK version. Also refer to older SDK version release notes to see changes in
earlier SDKs.

The below table captures the list of migration document sections when migrating from one version to another.
The migration for a particular module will be applicable, if you are migrating from older version listed to
newer version listed on the table below.

Module       | Migration guide                              | Older version  | Newer version
-------------|----------------------------------------------|----------------| -----------------
ICSS-EMAC    | Include paths - Update build configs to use source/networking/icss_emac_lld/ | <= 12.00.00  | >= 12.01.00
SDL VTM      | Recompile any code calling SDL_VTM_getTemp() (buffer type changed)           |
<= 12.00.00  | >= 12.01.00
Bootloader RPRC | Migrate to MCELF format; RPRC is deprecated | <= 12.00.00  | >= 12.01.00
Examples     |  \ref EXAMPLE_MIGRATION_11_01_00  &zwj;      |   <= 11.00.00  | >= 11.01.00
Networking   |  \ref enet_mcupsdk_10_00_update   &zwj;      |   <= 11.01.00  | >= 11.02.00
OSPI_HLD     |  \ref OSPI_HLD_MIGRATION_GUIDE    &zwj;      |   <= 11.01.00  | >= 11.02.00
OSPI_LLD     |  \ref OSPI_LLD_MIGRATION_GUIDE    &zwj;      |   <= 11.01.00  | >= 11.02.00
VTM (SDL)    |  SDL_VTM_getTemp API now takes int32_t pointer instead of uint32_t   |   <= 12.00.00  | >= 12.01.00
ICSS-EMAC    |  \ref ICSS_EMAC_SOURCE_PATH_CHANGE    &zwj;      |   <= 12.00.00  | >= 12.01.00
