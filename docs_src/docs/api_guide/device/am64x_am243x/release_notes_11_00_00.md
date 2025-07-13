# Release Notes 11.00.00 {#RELEASE_NOTES_11_00_00_PAGE}

[TOC]

\attention 1. Also refer to individual module pages for more details on each feature, unsupported features, important usage guidelines.

\attention 2. A53 support is applicable for AM64x only. It is NOT applicable for AM243x. \n


\note The examples will show usage of SW modules and APIs on a specific CPU instance and OS combination. \n
      Unless noted otherwise, the SW modules would work in both FreeRTOS and NORTOS environment. \n
      Unless noted otherwise, the SW modules would work on any of the R5F's present on the SOC. \n
      Unless noted otherwise, the SW modules would work on all supported EVMs \n
      M4F drivers support only MCU domain peripheral and peripheral instance while R5/A53 supports MAIN domain peripheral and peripheral instance. \n

\attention Klockwork Static Analysis report is not updated for this release

## New in this Release

\cond SOC_AM64X
Feature                                                                                         | Module
------------------------------------------------------------------------------------------------|-----------------------------------
LLD drivers for FSI                                                                             | Drivers
ECC example for R5F cache                                                                       | SDL
\endcond

\cond SOC_AM243X
Feature                                                                                         | Module
------------------------------------------------------------------------------------------------|-----------------------------------
LLD drivers for FSI                                                                             | Drivers
ECC example for R5F cache                                                                       | SDL
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
Code Composer Studio    | R5F, M4F, A53  | 12.8.1
SysConfig               | R5F, M4F, A53  | 1.22.0, build 3893
TI ARM CLANG            | R5F, M4F       | 4.0.1.LTS
GCC AARCH64             | A53            | 9.2-2019.12
GCC ARM                 | R5F            | 7-2017-q4-major (AM64x only)
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
        <td>11.00.07</td>
    </tr>
    <tr>
        <td>Release Notes</td>
        <td>[LINK](https://software-dl.ti.com/tisci/esd/11_00_07/release_notes/release_notes.html)</td>
    </tr>
    <tr>
        <td>User Guide</td>
        <td>[LINK](https://software-dl.ti.com/tisci/esd/11_00_07/1_intro/TISCI.html)</td>
    </tr>
</table>

## Key Features

### Experimental Features {#EXPERIMENTAL_FEATURES_11_00_00}

\attention Features listed below are early versions and should be considered as "experimental".
\attention Users can evaluate the feature, however the feature is not fully tested at TI side.
\attention TI would not support these feature on public e2e.
\attention Experimental features will be enabled with limited examples and SW modules.

\cond SOC_AM64X
Feature                                                             | Module
--------------------------------------------------------------------|--------------------------
A53 NORTOS support and A53 NORTOS examples                          | DPL, NORTOS
A53 FreeRTOS (single core) support and A53 FreeRTOS examples        | DPL, FreeRTOS
SBL booting A53 NORTOS                                              | Bootloader
GCC support for R5F for limited examples                            | R5F
A53 FreeRTOS dual core in SMP mode and A53 SMP FreeRTOS examples    | DPL, FreeRTOS
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

Peripheral | Supported CPUs | SysConfig Support |DMA Supported | Key features tested                                                                        | Key features not tested / NOT supported
-----------|----------------|-------------------|--------------|--------------------------------------------------------------------------------------------|----------------------------------------
ADC        | R5F, A53       | YES               | Yes          | Single conversion (one-shot mode), interrupt mode, DMA mode                                | Continuous conversion not tested
CRC        | R5F            | YES               | No           | CRC in full CPU mode                                                                       | -
DDR        | R5F            | YES               | No           | Tested LPDDR4 at 400MHz frequency.                                                         | -
ECAP       | R5F, A53       | YES               | No           | Frequency, Duty cycle, interrupt mode                                                      | -
EPWM       | R5F, A53       | YES               | No           | Different Frequency, Duty cycle, interrupt mode, Deadband and chopper module               | Tripzone module not tested
EQEP       | R5F, A53       | YES               | No           | Signal Frequency and Direction, interrupt mode                                             | -
FSI (RX/TX)| R5F            | YES               | No           | RX, TX, polling, interrupt mode, single/dual lanes                                         | -
GPIO       | R5F, M4F, A53  | YES               | No           | Basic input/output, GPIO as interrupt                                                      | -
GTC        | R5F, A53       | NA                | No           | Enable GTC, setting FID (Frequency indicator)                                              | -
I2C        | R5F, M4F, A53  | YES               | No           | Controller mode, basic read/write, polling and interrupt mode                              | Target mode not supported. M4F not tested due to EVM limitation
IPC Notify | R5F, M4F, A53  | YES               | No           | Low latency IPC between RTOS/NORTOS CPUs                                                   | -
IPC Rpmsg  | R5F, M4F, A53  | YES               | No           | RPMessage protocol based IPC for all R5F, M4F, A53 running NORTOS/FreeRTOS/Linux           | -
MCAN       | R5F, A53       | YES               | No           | RX, TX, interrupt and polling mode                                                         | -
MCSPI      | R5F, M4F, A53  | YES               | Yes          | Controller/Peripheral mode, basic read/write, polling, interrupt and DMA mode                       | -
MDIO       | R5F            | NA                | No           | Register read/write, link status and link interrupt enable API                             | -
MMCSD      | R5F, A53       | YES               | Yes          | Raw read/write and file I/O on MMCSD0 eMMC, and MMCSD1 SD. eMMC tested till HS SDR mode (8-bit data, 52 MHz), SD tested till SD HS mode (4-bit, 25 MHz)  | Interrupt mode not tested
OSPI       | R5F, A53       | YES               | Yes          | Read direct, Write indirect, Read/Write commands, DMA for read, PHY Mode                   | Interrupt mode not supported
PCIe       | R5F            | YES               | No           | Buffer Transfer between EP and RC modes. Legacy interrupt                                  | MSI and MSIx capability
Pinmux     | R5F, M4F, A53  | YES               | No           | Tested with multiple peripheral pinmuxes                                                   | -
PRUICSS    | R5F            | YES               | No           | Tested with Ethercat, EtherNet/IP, IO-Link, ICSS-EMAC, HDSL, EnDat                         | -
SOC        | R5F, M4F, A53  | YES               | No           | lock/unlock MMRs, get CPU clock, CPU name, clock enable, set frequency, SW Warm/POR Reset, Address Translation  | -
Sciclient  | R5F, M4F, A53  | YES               | No           | Tested with clock setup, module on/off                                                     | -
SPINLOCK   | R5F, M4F, A53  | NA                | No           | Lock, unlock HW spinlocks                                                                  | -
UART       | R5F, M4F, A53  | YES               | Yes          | Basic read/write, polling, interrupt mode,                                                 | HW flow control not tested. DMA mode not supported
UDMA       | R5F, A53       | YES               | Yes          | Basic memory copy, SW trigger, Chaining                                                    | -
WDT        | R5F, A53       | YES               | No           | Interrupt after watchdog expiry                                                            | Reset not supported

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
    <th> Applicable Releases
    <th> Applicable Devices
</tr>
<tr>
    <td> \htmllink{https://sir.ext.ti.com/jira/browse/EXT_SITMPUSW-61, EXT_SITMPUSW-61}
    <td> UART DMA LLD SysConfig generates incorrect files
    <td> UART
    <td> 10.01.00 onwards
    <td> AM64x, AM243x
</tr>
<tr>
    <td> \htmllink{https://sir.ext.ti.com/jira/browse/EXT_SITMPUSW-62, EXT_SITMPUSW-62}
    <td> SPI0_CS1 pin can't be added
    <td> Pinmux
    <td> 10.01.00 onwards
    <td> AM64x, AM243x
</tr>
<tr>
    <td> \htmllink{https://sir.ext.ti.com/jira/browse/EXT_SITMPUSW-63, EXT_SITMPUSW-63}
    <td> Undeclared Identifier for MCU GPIO interrupt router
    <td> GPIO
    <td> 10.01.00 onwards
    <td> AM64x, AM243x
</tr>
<tr>
    <td> \htmllink{https://sir.ext.ti.com/jira/browse/EXT_SITMPUSW-64, EXT_SITMPUSW-64}
    <td> AM64x: a53 core: Unable to flash Examples >1.2MB size using UART boot mode
    <td> Uniflash
    <td> 10.01.00 onwards
    <td> AM64x
</tr>
<tr>
    <td> \htmllink{https://sir.ext.ti.com/jira/browse/EXT_SITMPUSW-65, EXT_SITMPUSW-65}
    <td> At some random timing Pinmux registers (PADCFG) are not updated properly
    <td> Pinmux
    <td> 10.01.00 onwards
    <td> AM64x, AM243x
</tr>
<tr>
    <td> \htmllink{https://sir.ext.ti.com/jira/browse/EXT_SITMPUSW-66, EXT_SITMPUSW-66}
    <td> Dhrystone benchmark example has CPU clock hard code
    <td> Benchmark
    <td> 10.00.00 onwards
    <td> AM64x, AM243x
</tr>
<tr>
    <td> \htmllink{https://sir.ext.ti.com/jira/browse/EXT_SITMPUSW-67, EXT_SITMPUSW-67}
    <td> Comparing unsigned integer value with signed integer
    <td> Sysconfig
    <td> 10.00.00 onwards
    <td> AM64x, AM243x
</tr>
<tr>
    <td> \htmllink{https://sir.ext.ti.com/jira/browse/EXT_SITMPUSW-68, EXT_SITMPUSW-68}
    <td> Load from JSON button fails to load the JSON flash config file
    <td> Flash
    <td> 10.00.00 onwards
    <td> AM64x, AM243x
</tr>
<tr>
    <td> \htmllink{https://sir.ext.ti.com/jira/browse/EXT_SITMPUSW-69, EXT_SITMPUSW-69}
    <td> MMCSD_RAW_IO example does not boot with SBL_EMMC
    <td> Flash
    <td> 10.00.00 onwards
    <td> AM64x, AM243x
</tr>
<tr>
    <td> \htmllink{https://sir.ext.ti.com/jira/browse/EXT_SITMPUSW-70, EXT_SITMPUSW-70}
    <td> SBL_DFU booting but not initializing DFU interface to receive the application image
    <td> USB DFU
    <td> 10.00.00 onwards
    <td> AM64x, AM243x
</tr>
<tr>
    <td> \htmllink{https://sir.ext.ti.com/jira/browse/EXT_EP-12290, EXT_EP-12290}
    <td> SysConfig 1.21 creates incorrect MCSPI structs in ti_drivers_config.c
    <td> Sysconfig
    <td> 10.00.00 onwards
    <td> AM64x, AM243x
</tr>
<tr>
    <td> \htmllink{https://sir.ext.ti.com/jira/browse/EXT_SITMPUSW-71, EXT_SITMPUSW-71}
    <td> Not able to open drivers for MCU_I2C1
    <td> I2C
    <td> 10.01.00 onwards
    <td> AM64x, AM243x
</tr>
<tr>
    <td> \htmllink{https://sir.ext.ti.com/jira/browse/EXT_SITMPUSW-72, EXT_SITMPUSW-72}
    <td> Inline ECC init sequence not matching the steps mentioned in TRM for AM243/AM64x
    <td> DDR
    <td> 10.01.00 onwards
    <td> AM64x, AM243x
</tr>
<tr>
    <td> \htmllink{https://sir.ext.ti.com/jira/browse/EXT_EP-12324, EXT_EP-12324}
    <td> gSDL_pvt_poly_golden has to be used in SDL_vtmPrepLookupTable API instead of gSDL_pvt_poly
    <td> SDL
    <td> 11.00.00
    <td> AM64x, AM243x
</tr>
<tr>
    <td> \htmllink{https://sir.ext.ti.com/jira/browse/EXT_EP-12319, EXT_EP-12319}
    <td> ScrambleValue for the MCU PBIST instance has to updated for AM64x/AM243x
    <td> SDL
    <td> 11.00.00
    <td> AM64x, AM243x
</tr>
<tr>
    <td> \htmllink{https://sir.ext.ti.com/jira/browse/EXT_EP-12318, EXT_EP-12318}
    <td> SDL_ECC_aggrTransBaseAddressTable declared without extern keyword
    <td> SDL
    <td> 11.00.00
    <td> AM64x, AM243x
</tr>
<tr>
    <td> PROC_SDL-8891
    <td> AM243x: POK: Example is stuck on M4F core
    <td> SDL
    <td> 11.00.00
    <td> AM243x
</tr>
<tr>
    <td> \htmllink{https://sir.ext.ti.com/jira/browse/EXT_SITMPUSW-55, EXT_SITMPUSW-55}
    <td> AM64x: ENET : A53 : TCP Throughput drop in iperf application running on FreeRTOS A53 core
    <td> Ethernet
    <td> 11.00.00
    <td> AM64x
    <td> Issue is seen only with 1Gbps Full Duplex link speed. Any other link speed such as 100 Mbps link speed shall work seemlessly
</tr>

<tr>
    <td> \htmllink{https://sir.ext.ti.com/jira/browse/EXT_SITMPUSW-56, EXT_SITMPUSW-56}
    <td> AM64X: ENET: Data inconsitency with MAC loopback application with FreeRTOS running on A53 core
    <td> Ethernet
    <td> 11.00.00
    <td> AM64x
    <td> Issue is seen in release mode. In debug mode it works fine.
</tr>
<tr>
    <td> \htmllink{https://sir.ext.ti.com/jira/browse/EXT_EP-12337, EXT_EP-12337}
    <td> ICSSG: TAPRIO - Firmware can’t handle base-time which is not a multiple of cycle-time.
    <td> Ethernet (ICSSG)
    <td> 10.00.00 onwards
    <td> AM64x, AM243x
</tr>
<tr>
    <td> \htmllink{https://sir.ext.ti.com/jira/browse/EXT_EP-12338, EXT_EP-12338}
    <td> ICSSG: PA_STATS are not sequenced properly
    <td> Ethernet (ICSSG)
    <td> 09.02.00 onwards
    <td> AM64x, AM243x
</tr>
<tr>
    <td> \htmllink{https://sir.ext.ti.com/jira/browse/EXT_EP-12339, EXT_EP-12339}
    <td> Switch/MAC/HSR firmware does not drop corrupted packets
    <td> Ethernet (ICSSG)
    <td> 10.01.00 onwards
    <td> AM64x, AM243x
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
    <td> \htmllink{https://sir.ext.ti.com/jira/browse/EXT_SITMPUSW-22, EXT_SITMPUSW-22}
    <td> Pcie_benchmark, Pcie_buf_transfer, Pcie_legacy_irq, Pcie_msi_irq, Pcie_msix_irq and sbl_pcie are broken on 9.2.1 release
    <td> PCIE
    <td> 9.2.1 onwards
    <td> AM64x, AM243x
    <td> None.
</tr>
<tr>
    <td> \htmllink{https://sir.ext.ti.com/jira/browse/EXT_SITMPUSW-21, EXT_SITMPUSW-21}
    <td> DMA not working with ADC FIFO 1
    <td> ADC
    <td> 7.3.0 onwards
    <td> AM64x, AM243x
    <td> Use ADC FIFO 0
</tr>

<tr>
    <td> \htmllink{https://sir.ext.ti.com/jira/browse/EXT_SITMPUSW-23, EXT_SITMPUSW-23}
    <td> [Docs] Sysfw RM/PM documentation doesn't specify AM243x
    <td> Docs
    <td> 8.0.0 onwards
    <td> AM243x
    <td> -
</tr>
<tr>
    <td> \htmllink{https://sir.ext.ti.com/jira/browse/EXT_SITMPUSW-24, EXT_SITMPUSW-24}
    <td> PKA ECDSA sign verify is not working for P-521 and BrainPool P-512R1 curves
    <td> SECURITY
    <td> 8.2.0 onwards
    <td> AM64x, AM243x
    <td> -
</tr>
<tr>
    <td> \htmllink{https://sir.ext.ti.com/jira/browse/EXT_SITMPUSW-25, EXT_SITMPUSW-25}
    <td> Last 512KB of memory is not accessible in dev boot mode flow
    <td> SBL
    <td> 8.4.0
    <td> AM64x, AM243x
    <td> Use other boot modes
</tr>
<tr>
    <td> \htmllink{https://sir.ext.ti.com/jira/browse/EXT_SITMPUSW-26, EXT_SITMPUSW-26}
    <td> PCIe MSI error when connected to Linux Root Complex
    <td> PCIe
    <td> 8.6.0
    <td> AM64x, AM243x
    <td> -
</tr>
<tr>
    <td> \htmllink{https://sir.ext.ti.com/jira/browse/EXT_SITMPUSW-73, EXT_SITMPUSW-73}
    <td> OSPI_readDirect and OSPI_isPhyEnable do not correctly check if the PHY is enabled
    <td> OSPI
    <td> 10.1.0 onwards
    <td> AM64x, AM243x
    <td> None.
</tr>
<tr>
    <td> \htmllink{https://sir.ext.ti.com/jira/browse/EXT_SITMPUSW-77, EXT_SITMPUSW-77}
    <td> Modify Read capture delay logic for Tap Mode
    <td> SDL
    <td> 10.1.0 onwards
    <td> AM64x, AM243x
    <td> None
</tr>
<tr>
    <td> \htmllink{https://sir.ext.ti.com/jira/browse/EXT_SITMPUSW-81, EXT_SITMPUSW-81}
    <td> MCU+ SDK: EPWM_tbTimebaseClkCfg does not choose the optimal pre-scaler combination
    <td> EPWM
    <td> 7.3.0 onwards
    <td> AM64x, AM243x
    <td> None
</tr>
<tr>
    <td> \htmllink{https://sir.ext.ti.com/jira/browse/EXT_SITMPUSW-82, EXT_SITMPUSW-82}
    <td> AM64/AM243: M4F core getting hangs when doing IPC with Linux
    <td> IPC
    <td> 7.3.0 onwards
    <td> AM64x, AM243x
    <td> None
</tr>
<tr>
    <td> \htmllink{https://sir.ext.ti.com/jira/browse/EXT_SITMPUSW-83, EXT_SITMPUSW-83}
    <td> DPL - configure HwiP_MAX_INTERRUPTS based on SOC supported values
    <td> DPL
    <td> 7.3.0 onwards
    <td> AM64x, AM243x
    <td> None
</tr>
<tr>
    <td> \htmllink{https://sir.ext.ti.com/jira/browse/EXT_SITMPUSW-86, EXT_SITMPUSW-86}
    <td> The configurations of GPIO interrupt routers are based on banks rather than individual pins.
    <td> GPIO
    <td> 7.3.0 onwards
    <td> AM64x, AM243x
    <td> None
</tr>
<tr>
    <td> \htmllink{https://sir.ext.ti.com/jira/browse/EXT_EP-12271, EXT_EP-12271}
    <td> AM64x: AM243x: ECC: SDL_PCIE0_PCIE_G2X1_64_CORE_CORE_ECC_AGGR aggregator is failing
    <td> SDL
    <td> 10.1.0 onwards
    <td> AM64x, AM243x
    <td> No known workaround
</tr>
<tr>
    <td> \htmllink{https://sir.ext.ti.com/jira/browse/EXT_EP-12336, EXT_EP-12336}
    <td> Last 1B of payload missing in forwarding scenario
    <td> Ethernet (ICSSG)
    <td> 10.1.0 onwards
    <td> AM64x, AM243x
    <td> No known workaround
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

### Compiler Options

<table>
<tr>
    <th> Module
    <th> Affected API
    <th> Change
    <th> Additional Remarks
</tr>
</table>

### Examples

<table>
<tr>
    <th> Module
    <th> Affected API
    <th> Change
    <th> Additional Remarks
</tr>
<tr>
    <td> Ethernet
    <td> -
    <td> Path for Ethernet examples has been changed from ${MCU_PLUS_SDK_PATH}/examples/networking to ${MCU_PLUS_SDK_PATH}/source/networking/enet/core/examples
    <td> Refer \ref enet_mcupsdk_10_00_update
</tr>
</table>

### OS Kernel

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
