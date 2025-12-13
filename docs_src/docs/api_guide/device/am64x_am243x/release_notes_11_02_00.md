# Release Notes 11.02.00 {#RELEASE_NOTES_11_02_00_PAGE}

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
Example to demonstrate VTM triggered SoC reset is added                                         | SDL
\endcond

\cond SOC_AM243X
Feature                                                                                         | Module
------------------------------------------------------------------------------------------------|-----------------------------------
                                                                                                |  
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
Code Composer Studio    | R5F, M4F, A53  | 20.2.0
SysConfig               | R5F, M4F, A53  | 1.23.0, build 4000
TI ARM CLANG            | R5F, M4F       | 4.0.1.LTS
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
        <td>11.02.05</td>
    </tr>
    <tr>
        <td>Release Notes</td>
        <td>[LINK](https://software-dl.ti.com/tisci/esd/11_02_05/release_notes/release_notes.html)</td>
    </tr>
    <tr>
        <td>User Guide</td>
        <td>[LINK](https://software-dl.ti.com/tisci/esd/11_02_05/1_intro/TISCI.html)</td>
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
PRUICSS    | R5F            | YES               | No           | Tested with EtherCAT, EtherNet/IP, IO-Link, ICSS-EMAC, HDSL, EnDat                         | -
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
    <th> Applicable Devices
</tr>
<tr>
    <td> \htmllink{https://sir.ext.ti.com/jira/browse/EXT_SITMPUSW-236, EXT_SITMPUSW-236}
    <td> Incorrect calculation of ptrInput and ptrOutput in the SA2UL_contextProcess
    <td> Crypto
    <td> AM64x, AM243x
</tr>
<tr>
    <td> \htmllink{https://sir.ext.ti.com/jira/browse/EXT_SITMPUSW-244, EXT_SITMPUSW-244}
    <td> Incorrect handling of the donedataLen in the SA2UL_contextProcess
    <td> Crypto
    <td> AM64x, AM243x
</tr>
<tr>
    <td> \htmllink{https://sir.ext.ti.com/jira/browse/EXT_SITMPUSW-109, EXT_SITMPUSW-109}
    <td> Wrong comments on HwiP_inISR() API
    <td> Documentation
    <td> AM64x, AM243x
</tr>
<tr>
    <td> \htmllink{https://sir.ext.ti.com/jira/browse/EXT_SITMPUSW-238, EXT_SITMPUSW-238}
    <td> Incorrect address size for I2C EEPROM 
    <td> EEPROM
    <td> AM64x, AM243x
</tr>
<tr>
    <td> \htmllink{https://sir.ext.ti.com/jira/browse/EXT_SITMPUSW-233, EXT_SITMPUSW-233}
    <td> AM64X : Critical section implementaion in the A53 
    <td> FreeRTOS
    <td> AM64x
</tr>
<tr>
    <td> \htmllink{https://sir.ext.ti.com/jira/browse/EXT_SITMPUSW-239, EXT_SITMPUSW-239}
    <td> I2C gets hanging on communication errors
    <td> I2C
    <td> AM64x, AM243x
</tr>
<tr>
    <td> \htmllink{https://sir.ext.ti.com/jira/browse/EXT_SITMPUSW-117, EXT_SITMPUSW-117}
    <td> Linux Kernel IPC Examples are broken
    <td> IPC
    <td> AM64x, AM243x
</tr>
<tr>
    <td> \htmllink{https://sir.ext.ti.com/jira/browse/EXT_SITMPUSW-121, EXT_SITMPUSW-121}
    <td> FORCE bit not book keeped properly for MCSPI DMA mode of operation
    <td> MCSPI
    <td> AM64x, AM243x
</tr>
<tr>
    <td> \htmllink{https://sir.ext.ti.com/jira/browse/EXT_SITMPUSW-207, EXT_SITMPUSW-207}
    <td> Missing -p flag with strip command to reduce the size of *.out
    <td> Makefiles
    <td> AM64x, AM243x
</tr>
<tr>
    <td> \htmllink{https://sir.ext.ti.com/jira/browse/EXT_SITMPUSW-118, EXT_SITMPUSW-118}
    <td> OSPI tuning algorithm finds points in noisy region on rare scenarios 
    <td> OSPI
    <td> AM64x, AM243x
</tr>
<tr>
    <td> \htmllink{https://sir.ext.ti.com/jira/browse/EXT_SITMPUSW-187, EXT_SITMPUSW-187}
    <td> OSPI DAC Mode Configuration hardcoded and not based on flashsize
    <td> OSPI
    <td> AM64x, AM243x
</tr>
<tr>
    <td> \htmllink{https://sir.ext.ti.com/jira/browse/EXT_SITMPUSW-167, EXT_SITMPUSW-167}
    <td> SBL OSPI failing with large sized application
    <td> SBL
    <td> AM243x
</tr>
<tr>
    <td> \htmllink{https://sir.ext.ti.com/jira/browse/EXT_SITMPUSW-229, EXT_SITMPUSW-229}
    <td> AM243x-lp/AM243x-evm: Not able to load image of size 1050KB via UART bootloader
    <td> SBL
    <td> AM243x
</tr>
<tr>
    <td> \htmllink{https://sir.ext.ti.com/jira/browse/EXT_SITMPUSW-226, EXT_SITMPUSW-226}
    <td> Improve Sysconfig tool tips for all drivers
    <td> SysConfig
    <td> AM64x, AM243x
</tr>
<tr>
    <td> \htmllink{https://sir.ext.ti.com/jira/browse/EXT_SITMPUSW-116, EXT_SITMPUSW-116}
    <td> Application gets stuck with Multiple UARTS with DMA enabled
    <td> UDMA
    <td> AM64x, AM243x
</tr>
<tr>
    <td> \htmllink{https://sir.ext.ti.com/jira/browse/EXT_SITMPUSW-237, EXT_SITMPUSW-237}
    <td> Application gets stuck with BCDMA and PKTDMA instants
    <td> UDMA
    <td> AM64x, AM243x
</tr>
<tr>
    <td> \htmllink{https://sir.ext.ti.com/jira/browse/EXT_EP-13056, EXT_EP-13056}
    <td> VTM module sensor reset sequence modification for reliable functionality
    <td> SDL
    <td> AM64x, AM243x
</tr>
<tr>
    <td> \htmllink{https://sir.ext.ti.com/jira/browse/EXT_EP-13057, EXT_EP-13057}
    <td> SDL DCC Seed Value Calculation Deviates from TRM Guidelines
    <td> SDL
    <td> AM64x, AM243x
</tr>
<tr>
    <td> \htmllink{https://sir.ext.ti.com/jira/browse/EXT_EP-12719, EXT_EP-12719}
    <td> Throughput drops to 95.5% with bidirectional mixed traffic
    <td> Ethernet (ICSSG)
    <td> AM64x, AM243x
</tr>
<tr>
    <td> \htmllink{https://sir.ext.ti.com/jira/browse/EXT_EP-13075, EXT_EP-13075}
    <td> Forwarding stall with random size packets at 100% bi-directional 1Gbps load 
    <td> Ethernet (ICSSG) 
    <td> AM64x, AM243x
</tr>
</table>

## Known Issues

<table>
<tr>
    <th> ID
    <th> Head Line
    <th> Module
    <th> Applicable Devices
    <th> Workaround
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
    <td> \htmllink{https://sir.ext.ti.com/jira/browse/EXT_SITMPUSW-153, EXT_SITMPUSW-153}
    <td> MCU+ SDK CCS Project Build Generates Invalid/Redundant Boot Image Files
    <td> Build
    <td> 10.01.00 Onwards
    <td> AM64x AM243x
    <td> -
</tr>
<tr>
    <td> \htmllink{https://sir.ext.ti.com/jira/browse/EXT_SITMPUSW-83, EXT_SITMPUSW-83}
    <td> DPL - configure HwiP_MAX_INTERRUPTS based on SOC supported values
    <td> DPL
    <td> 09.00.00 Onwards
    <td> AM64x AM243x
    <td> -
</tr>
<tr>
    <td> \htmllink{https://sir.ext.ti.com/jira/browse/EXT_SITMPUSW-24, EXT_SITMPUSW-24}
    <td> [SA2UL][PKA] ECDSA Sign/verify not working with P-521 and BrainPool P-512R1 Curves
    <td> ECDSA
    <td> 08.03.00 Onwards
    <td> AM64x AM243x
    <td> -
</tr>
<tr>
    <td> \htmllink{https://sir.ext.ti.com/jira/browse/EXT_SITMPUSW-81, EXT_SITMPUSW-81}
    <td> MCU+ SDK: EPWM_tbTimebaseClkCfg does not choose the optimal pre-scaler combination
    <td> EPWM
    <td> 10.00.00 Onwards
    <td> AM64x AM243x
    <td> -
</tr>
<tr>
    <td> \htmllink{https://sir.ext.ti.com/jira/browse/EXT_SITMPUSW-251, EXT_SITMPUSW-251}
    <td> MCU+ SDK Example Projects using incorrect ARMv7 MPU Attributes for Peripheral Register Region
    <td> Examples
    <td> 11.01.00 Onwards
    <td> AM64x AM243x
    <td> -
</tr>
<tr>
    <td> \htmllink{https://sir.ext.ti.com/jira/browse/EXT_SITMPUSW-86, EXT_SITMPUSW-86}
    <td> The configurations of GPIO interrupt routers are based on banks rather than individual pins.
    <td> GPIO
    <td> 09.00.00 Onwards
    <td> AM64x AM243x
    <td> -
</tr>
<tr>
    <td> \htmllink{https://sir.ext.ti.com/jira/browse/EXT_SITMPUSW-82, EXT_SITMPUSW-82}
    <td> AM64/AM243: M4F core getting hangs when doing IPC with Linux
    <td> IPC
    <td> 08.06.00 Onwards
    <td> AM64x AM243x
    <td> -
</tr>
<tr>
    <td> \htmllink{https://sir.ext.ti.com/jira/browse/EXT_SITMPUSW-260, EXT_SITMPUSW-260}
    <td> MCSPI Performance Numbers under MCU+ SDK documentation are not updated
    <td> McSPI
    <td> 11.01.00 Onwards
    <td> AM64x AM243x
    <td> -
</tr>
<tr>
    <td> \htmllink{https://sir.ext.ti.com/jira/browse/EXT_SITMPUSW-263, EXT_SITMPUSW-263}
    <td> Design choice for Security Handover is not documented
    <td> MMCSD
    <td> 11.01.01 Onwards
    <td> AM64x AM243x
    <td> -
</tr>
<tr>
    <td> \htmllink{https://sir.ext.ti.com/jira/browse/EXT_SITMPUSW-262, EXT_SITMPUSW-262}
    <td> mmcsd_raw_io_emmc_lld example is not working
    <td> MMCSD
    <td> 11.01.00 Onwards
    <td> AM64x AM243x
    <td> -
</tr>
<tr>
    <td> \htmllink{https://sir.ext.ti.com/jira/browse/EXT_SITMPUSW-273, EXT_SITMPUSW-273}
    <td> AM64x PCIe as End Point throwing error when changing BAR aperture
    <td> PCIE
    <td> 08.06.00 Onwards
    <td> AM64x AM243x
    <td> -
</tr>
<tr>
    <td> \htmllink{https://sir.ext.ti.com/jira/browse/EXT_SITMPUSW-26, EXT_SITMPUSW-26}
    <td> AM64X PCIe MSI error when connected to Linux Root Complex
    <td> PCIE
    <td> 08.06.00 Onwards
    <td> AM64x AM243x
    <td> -
</tr>
<tr>
    <td> \htmllink{https://sir.ext.ti.com/jira/browse/EXT_SITMPUSW-22, EXT_SITMPUSW-22}
    <td> Pcie_benchmark, Pcie_buf_transfer, Pcie_legacy_irq, Pcie_msi_irq and Pcie_msix_irq are broken on 9.2.1 release"
    <td> PCIE
    <td> 09.02.01 Onwards
    <td> AM64x AM243x
    <td> -
</tr>
<tr>
    <td> \htmllink{https://sir.ext.ti.com/jira/browse/EXT_SITMPUSW-25, EXT_SITMPUSW-25}
    <td> AM243x/AM64x: last 512KB of memory is not accessible in dev boot mode flow
    <td> SBL
    <td> 08.04.00 Onwards
    <td> AM64x AM243x
    <td> -
</tr>
<tr>
    <td> \htmllink{https://sir.ext.ti.com/jira/browse/EXT_SITMPUSW-111, EXT_SITMPUSW-111}
    <td> UART DMA LLD Build issue
    <td> UART
    <td> 11.02.00 Onwards
    <td> AM64x AM243x
    <td> -
</tr>
<tr>
    <td> \htmllink{https://sir.ext.ti.com/jira/browse/EXT_SITMPUSW-257, EXT_SITMPUSW-257}
    <td> Roadmap for XIP Secure boot support
    <td> XIP
    <td> 11.01.00 Onwards
    <td> AM64x AM243x
    <td> -
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
    <td> \htmllink{https://sir.ext.ti.com/jira/browse/EXT_SITMPUSW-131, EXT_SITMPUSW-131}
    <td> AM64X/AM243X: ENET: TCP/IP bidirectional throughput drops during iperf testing
    <td> Ethernet (ICSSG)
    <td> 11.01.00 onwards
    <td> AM64x, AM243x
    <td> -
</tr>
<tr>
    <td> \htmllink{https://sir.ext.ti.com/jira/browse/EXT_SITMPUSW-132, EXT_SITMPUSW-132}
    <td> AM64X/AM243X: ENET: Enet VLAN ICSSG Example throws assert during channel open
    <td> Ethernet (ICSSG)
    <td> 11.01.00 onwards
    <td> AM64x, AM243x
    <td> -
</tr>
<tr>
    <td> \htmllink{https://sir.ext.ti.com/jira/browse/EXT_EP-12276, EXT_EP-12276}
    <td> ECC: Firewall related aggregators failures - ECC Aggregators SDL_SMS0_SMS_TIFS_ECC, SDL_SMS0_SMS_HSM_ECC, SDL_SA3_SS0_SA3SS_AM62A_DMSS_ECCAGGR and SDL_SA3_SS0_SA3SS_AM62A_SA_UL_ECC_AGGR fail because of firewall access issues faced by SDL
    <td> SDL
    <td> 10.01.00 onwards
    <td> AM64x, AM243x
    <td> No known workaround
</tr>
<tr>
    <td> \htmllink{https://sir.ext.ti.com/jira/browse/EXT_EP-13067, EXT_EP-13067}
    <td> AM64x: PBIST: A53 forcebit power off failure in M4F applications
    <td> SDL
    <td> 11.02.00 onwards
    <td> AM64x
    <td> No known workaround
</tr>
<tr>
    <td> \htmllink{https://sir.ext.ti.com/jira/browse/EXT_EP-13068, EXT_EP-13068}
    <td> AM64x: AM243x: ECC: Aggregator failures in ecc applications.
    <td> SDL
    <td> 11.02.00 onwards
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

The below table captures the list of migration document sections when migrating from one version to another.
The migration for a particular module will be applicable, if you are migrating from older version listed to
newer version listed on the table below.

Module       | Migration guide                              | Older version  | Newer version
-------------|----------------------------------------------|----------------| -----------------
Examples     |  \ref EXAMPLE_MIGRATION_11_01_00  &zwj;      |   <= 11.00.00  | >= 11.01.00
Networking   |  \ref enet_mcupsdk_10_00_update   &zwj;      |   <= 11.01.00  | >= 11.02.00
