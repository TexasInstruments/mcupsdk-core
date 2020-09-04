# Release Notes 08.03.00 {#RELEASE_NOTES_08_03_00_PAGE}

[TOC]

\attention Also refer to individual module pages for more details on each feature, unsupported features, important usage guidelines.

\attention A53 support is applicable for AM64x only. It is NOT applicable for AM243x. \n

\note The examples will show usage of SW modules and APIs on a specific CPU instance and OS combination. \n
      Unless noted otherwise, the SW modules would work in both FreeRTOS and NORTOS environment. \n
      Unless noted otherwise, the SW modules would work on any of the R5F's present on the SOC. \n
      Unless noted otherwise, the SW modules would work on all supported EVMs \n
      All SW modules will not work on M4F. M4F support for a module will be explicitly called out.

## New in this Release

\cond SOC_AM64X
Feature                                                                                         | Module
------------------------------------------------------------------------------------------------|-----------------------------------
32 Task Priority Levels for FreeRTOS Tasks                                                      | DPL, FreeRTOS
Upgrade to Beckhoff SubDevice Stack Code version 5.13 for EtherCAT examples                         | Industrial Communications Toolkit
PCIe Buffer Transfer between EP and RC modes. Legacy interrupt support. MSI and MSIx capability | PCIe
Firewall configuration to enable DMA for secure device                                          | DMA
Serial Interface implementation supporting ADS127L11 IC (1.067 MSPS)                            | PRU-IO
Memory footprint optimization in Enet LLD, LWIP, ICSSG                                          | ENET, ICSSG, LWIP
Buffered IO boot media support                                                                  | Bootloader
C++ build support and example project added                                                     | Generic
Added API to check if the interrupt router output is free                                       | Sciclient
ICSS based Ethernet, support for DSCP                                                           | ENET
Enabled combined boot flow (ROM loads SYSFW)                                                    | Bootloader
SBL Encryption support                                                                          | Bootloader, Security
Runtime SWREV configuration support (example) (only supported in HS-SE variant)                 | Security
Extended OTP programming support (example) (only supported in HS-SE variant)                    | Security

\endcond

\cond SOC_AM243X
Feature                                                                                         | Module
------------------------------------------------------------------------------------------------|-----------------------------------
32 Task Priority Levels for FreeRTOS Tasks                                                      | DPL, FreeRTOS
Upgrade to Beckhoff SubDevice Stack Code version 5.13 for EtherCAT examples                         | Industrial Communications Toolkit
PCIe Buffer Transfer between EP and RC modes. Legacy interrupt support. MSI and MSIx capability | PCIe
Firewall configuration to enable DMA for secure device                                          | DMA
Serial Interface implementation supporting ADS127L11 IC (1.067 MSPS)                            | PRU-IO
Memory footprint optimization in Enet LLD, LWIP, ICSSG                                          | ENET, ICSSG, LWIP
Buffered IO boot media support                                                                  | Bootloader
C++ build support and example project added                                                     | Generic
Added API to check if the interrupt router output is free                                       | Sciclient
ICSS based Ethernet, support for DSCP                                                           | ENET
Enabled combined boot flow (ROM loads SYSFW)                                                    | Bootloader
SBL Encryption support                                                                          | Bootloader, Security

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
Code Composer Studio    | R5F, M4F, A53  | @VAR_CCS_VERSION
SysConfig               | R5F, M4F, A53  | @VAR_SYSCFG_VERSION, build @VAR_SYSCFG_BUILD
TI ARM CLANG            | R5F, M4F       | @VAR_TI_ARM_CLANG_VERSION
GCC AARCH64             | A53            | @VAR_GCC_AARCH64_VERSION
GCC ARM                 | R5F            | @VAR_GCC_ARM_VERSION (AM64x only)
FreeRTOS Kernel         | R5F, M4F, A53  | @VAR_FREERTOS_KERNEL_VERSION
FreeRTOS SMP Kernel     | A53            | @VAR_FREERTOS_SMP_KERNEL_VERSION
Tiny USB                | R5F            | @VAR_TINYUSB_VERSION
LwIP                    | R5F            | @VAR_LWIP_VERSION

## Key Features

### Experimental Features

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
GUI for UART Uniflash Tool (No support for EMMC flashing)           | Bootloader

\endcond

\cond SOC_AM243X

Feature                                                             | Module
--------------------------------------------------------------------|--------------------------
GUI for UART Uniflash Tool (No support for EMMC flashing)           | Bootloader

\endcond

### Features not supported in release

- The **ROM** startup model for runtime initializations in TI ARM CLANG is not supported/tested in the SDK. We use the **RAM** model because of the boot-time advantage. For more information on startup models in TI ARM CLANG, refer \htmllink{https://software-dl.ti.com/codegen/docs/tiarmclang/compiler_tools_user_guide/compiler_manual/program_loading_and_running/run-time-initialization-slau1316977.html#ram-model-vs-rom-model, RAM model v/s ROM model}

\cond SOC_AM243X

### AM243X LAUNCHPAD not tested/not supported features

Below features are not support on AM243X LAUNCHPAD due to SOC or board constraints,

- DDR is not supported on the AM243X 11x11 SOC used in AM243X LAUNCHPAD.
- Motor control examples not validated, due to board limitation
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
ADC        | R5F            | YES               | No           | Single conversion (one-shot mode), interrupt mode, DMA mode                                | Continuous conversion not tested
CRC        | R5F            | YES               | No           | CRC in full CPU mode                                                                       | -
DDR        | R5F            | YES               | No           | Tested LPDDR4 at 400MHz frequency.                                                         | -
ECAP       | R5F            | YES               | No           | Frequency, Duty cycle, interrupt mode                                                      | PWM mode not tested
EPWM       | R5F            | YES               | No           | Different Frequency, Duty cycle, interrupt mode, Deadband and chopper module               | Tripzone module not tested
EQEP       | R5F            | YES               | No           | Signal Frequency and Direction, interrupt mode                                             | -
FSI (RX/TX)| R5F            | YES               | No           | RX, TX, polling, interrupt mode, single/dual lanes                                         | -
GPIO       | R5F, M4F, A53  | YES               | No           | Basic input/output, GPIO as interrupt                                                      | GPIO as interrupt is not tested for A53.
GTC        | R5F, A53       | NA                | No           | Enable GTC, setting FID (Frequency indicator)                                              | -
I2C        | R5F, M4F, A53  | YES               | No           | Controller mode, basic read/write, polling and interrupt mode                                  | Target mode not supported. M4F not tested due to EVM limitation
IPC Notify | R5F, M4F, A53  | YES               | No           | Low latency IPC between RTOS/NORTOS CPUs                                                   | -
IPC Rpmsg  | R5F, M4F, A53  | YES               | No           | RPMessage protocol based IPC for all R5F, M4F, A53 running NORTOS/FreeRTOS/Linux           | -
MCAN       | R5F            | YES               | No           | RX, TX, interrupt and polling mode                                                         | -
MCSPI      | R5F, M4F       | YES               | Yes          | Controller/Peripheral mode, basic read/write, polling, interrupt and DMA mode                       | -
MDIO       | R5F            | NA                | No           | Register read/write, link status and link interrupt enable API                             | -
MMCSD      | R5F            | YES               | Yes          | Raw read/write and file I/O on MMCSD0 eMMC, and MMCSD1 SD. eMMC tested till HS SDR mode (8-bit data, 52 MHz), SD tested till SD HS mode (4-bit, 25 MHz)  | Interrupt mode not tested
OSPI       | R5F            | YES               | Yes          | Read direct, Write indirect, Read/Write commands, DMA for read, PHY Mode                   | Interrupt mode not supported
PCIe       | R5F            | YES               | No           | Buffer Transfer between EP and RC modes. Legacy interrupt                                  | MSI and MSIx capability
Pinmux     | R5F, M4F, A53  | YES               | No           | Tested with multiple peripheral pinmuxes                                                   | -
PRUICSS    | R5F            | YES               | No           | Tested with Ethercat, EtherNet/IP, IO-Link, ICSS-EMAC, HDSL, EnDat                         | -
SOC        | R5F, M4F, A53  | YES               | No           | lock/unlock MMRs, get CPU clock, CPU name, clock enable, set frequency, SW Warm/POR Reset, Address Translation  | -
Sciclient  | R5F, M4F, A53  | YES               | No           | Tested with clock setup, module on/off                                                     | -
SPINLOCK   | R5F, M4F, A53  | NA                | No           | Lock, unlock HW spinlocks                                                                  | -
UART       | R5F, M4F, A53  | YES               | Yes          | Basic read/write, polling, interrupt mode,                                                 | HW flow control not tested. DMA mode not supported
UDMA       | R5F, A53       | YES               | Yes          | Basic memory copy, SW trigger, Chaining                                                    | -
WDT        | R5F, A53       | YES               | No           | Interrupt after watchdog expiry                                                            | Reset not supported

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
FreeRTOS+FAT                | R5F            | YES               | NORTOS            | File read, write, create. FAT partition and mounting                                        | File I/O with FreeRTOS

### CMSIS

Module                      | Supported CPUs | SysConfig Support | OS Support        | Key features tested                                                                         | Key features not tested
----------------------------|----------------|-------------------|-------------------|---------------------------------------------------------------------------------------------|------------------------
CMSIS DSP                   | R5F            | NA                | FreeRTOS, NORTOS  | Basic math, complex math, controller, fast math, filtering, Matrix, statistics, transform   | -

### Industrial Communications Toolkit

Module                                | Supported CPUs | SysConfig Support | OS Support  | Key features tested                                                                                      | Key features not tested
--------------------------------------|----------------|-------------------|-------------|----------------------------------------------------------------------------------------------------------|------------------------
EtherCAT SubDevice FWHAL                  | R5F            | NO                | FreeRTOS    | Tested with ethercat_slave_beckhoff_ssc_demo example                                                     | Reset isolation
EtherCAT SubDevice Evaluation Stack       | R5F            | NO                | FreeRTOS    | Tested with ethercat_slave_demo examples                                                                 | -
EtherNet/IP Adapter FWHAL             | R5F            | NO                | FreeRTOS    | Tested with ethernetip_adapter_demo examples                                                             | Multicast Filtering
EtherNet/IP Adapter Evaluation Stack  | R5F            | NO                | FreeRTOS    | Tested with ethernetip_adapter_demo examples                                                             | -
IO-Link Controller Evaluation Stack       | R5F            | NO                | FreeRTOS    | Tested with iolink_master_demo example                                                                   | -
Profinet Device FWHAL                 | R5F            | NO                | FreeRTOS    | Tested with profinet_device_demo examples                                                                | IRT, MRP
Profinet Device Evaluation Stack      | R5F            | NO                | FreeRTOS    | Tested with profinet_device_demo examples                                                                | -

### Motor Control

Module                      | Supported CPUs | SysConfig Support | OS Support        | Key features tested                             | Key features not tested
----------------------------|----------------|-------------------|-------------------|-------------------------------------------------|------------------------
Position Sense HDSL         | R5F            | YES               | FreeRTOS, NORTOS  | Freerun mode, Sync mode                         |  -
Position Sense EnDAT        | R5F            | YES               | FreeRTOS, NORTOS  | Single channel, Multi channel, Continuous mode  |  -
Position Sense Tamagawa     | R5F            | YES               | FreeRTOS, NORTOS  | Absolute position, Encoder ID, Reset            |  -

### Networking

Module                      | Supported CPUs | SysConfig Support | OS Support  | Key features tested                                                                             | Key features not tested
----------------------------|----------------|-------------------|-------------|-------------------------------------------------------------------------------------------------|--------------------------------------------------------------------------------------------------
LwIP                        | R5F            | YES               | FreeRTOS    | TCP/UDP IP networking stack, DHCP, ping, TCP iperf, TCP/UDP IP                                  | Other LwIP features, performance and memory optimizations pending, more robustness tests pending
Ethernet driver (ENET)      | R5F            | YES               | FreeRTOS    | Ethernet as port using CPSW and ICSS, Layer 2 MAC, Layer 2 PTP Timestamping, CPSW Switch                                                            | -
ICSS-EMAC                   | R5F            | YES               | FreeRTOS    | Tested switch mode with ethernetip_adapter_demo and profinet_device_demo examples               | EMAC mode, VLAN/Multicast Filtering
ICSS TimeSync               | R5F            | NO                | FreeRTOS    | Tested E2E mode with ethernetip_adapter_demo examples                                           | P2P mode, Transparent Clock mode

### USB

Module                      | Supported CPUs | SysConfig Support | OS Support         | Key features tested       | Key features not tested
----------------------------|----------------|-------------------|--------------------|---------------------------|------------------------
USB SoC Porting Layer       | R5F            | YES               | FreeRTOS, NORTOS   | USB 2.0 device mode       | USB 3.0
USB Device Driver           | R5F            | NO                | FreeRTOS, NORTOS   | USB 2.0 device mode       | USB Host driver
TinyUSB Core and CDC Driver | R5F            | NO                | FreeRTOS, NORTOS   | USB device with CDC class | USB Host, other USB device class drivers

### SECURITY

Module          | Supported CPUs    | SysConfig Support | OS Support        | Key features tested                                                                                                                                                                       | Key features not tested
----------------|-------------------|-------------------|-------------------|-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|-------------------------------------------------
SA2UL AES       | R5F               | YES               | NORTOS            | AES CBC-128 encryption and decryption, AES CBC-256 encryption and decryption, AES ECB-128 encryption and decryption, AES ECB-256 encryption and decryption, AES CMAC-128, AES CMAC-256    | -
SA2UL SHA       | R5F               | YES               | NORTOS            | SHA 512 single shot and multi-shot, SHA 256 single shot and multi-shot, HMAC SHA-256, HMAC SHA-512, HMAC SHA-1                                                                            | -
SA2UL RNG       | R5F               | YES               | NORTOS            | RNG generate random number with size of 4 words(128 bit)                                                                                                                                  | -
SA2UL PKA       | R5F               | YES               | NORTOS            | RSA Encryption and Decryption support upto 4k bit, RSA Signing and Verification support upto 4k bit, ECDSA Signing and Verification support with P-256 and P-384 curves                   | -

### PRU IO

Module          | Supported CPUs    | SysConfig Support | OS Support        | Key features tested                                           | Key features not tested
----------------|-------------------|-------------------|-------------------|---------------------------------------------------------------|-------------------------------------------------
ADS85x8         | R5F               | YES               | FREERTOS          | Parallel 8 and 16 Bit Interface with ADS8598H IC              | OSR Modes, Other ICs
ADS127L11       | R5F               | YES               | FREERTOS          | Serial Interface with ADS127L11 IC                            |
Empty           | PRU               | NO                | Bare Metal        | Empty project to get started with PRU firmware development    |

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
    <th> Resolution/Comments
</tr>
<tr>
    <td> MCUSDK-3811
    <td> Incorrect value for DLR Neighbor Check Interval in EtherNet/IP Adapter FWHAL
    <td> EtherNet/IP Adapter FWHAL
    <td> 7.3.0 onwards
    <td> AM64x, AM243x
    <td> Fixed
</tr>
<tr>
    <td> MCUSDK-3644
    <td> Removed GCC's objcopy in the boardcfg-gen makefile
    <td> Build
    <td> 8.1.0 onwards
    <td> AM64x, AM243x
    <td> Fixed
</tr>
<tr>
    <td> MCUSDK-3558
    <td> Secure Boot: DMA based applications fail on HS device
    <td> SBL SCIClient UDMA
    <td> 8.2.0
    <td> AM64x, AM243x
    <td> Fixed
</tr>
<tr>
    <td> MCUSDK-3424
    <td> SBL OSPI fails to load image using OSPI DMA in HS device
    <td> SBL OSPI UDMA
    <td> 8.2.0
    <td> AM64x, AM243x
    <td> Fixed
</tr>
<tr>
    <td> MCUSDK-3513
    <td> SBL JTAG Example flash write fails if file size is not aligned to multiple of flash page size
    <td> SBL
    <td> 8.2.0
    <td> AM64x, AM243x
    <td> Fixed
</tr>
<tr>
    <td> MCUSDK-3453
    <td> Enet: UDMA unable to process icssg rx timestamp and some bytes of packet content after Allocated Packets
    <td> Enet
    <td> 8.1.0 onwards
    <td> AM64x, AM243x
    <td> Fixed
</tr>
<tr>
    <td> MCUSDK-2753
    <td> SOC_controlModuleLockMMR() doesn't lock CTRLMMR registers
    <td> Common
    <td> 8.1.0 onwards
    <td> AM243x
    <td> Fixed
</tr>
<tr>
    <td> MCUSDK-3408
    <td> sbl_uart_uniflash write incorrect data to flash due to XMODEM 1024 byte alignment
    <td> SBL
    <td> 8.2.0
    <td> AM64x, AM243x
    <td> Fixed
</tr>
<tr>
    <td> MCUSDK-3398
    <td> [SOC]Address translation for R5FSS1 is missing
    <td> SOC
    <td> 7.3.0 onwards
    <td> AM64x, AM243x
    <td> Fixed
</tr>
<tr>
    <td> MCUSDK-3712
    <td> [FSI]Maximum value of external inputs for triggering frame transmission is limited to 32
    <td> SOC
    <td> 7.3.0 onwards
    <td> AM64x, AM243x
    <td> Fixed
</tr>
<tr>
    <td> MCUSDK-3879
    <td> DPL : Inconsistent MPU config for shared memory
    <td> DPL
    <td> 7.3.0 onwards
    <td> AM64x, AM243x
    <td> Fixed
</tr>
<tr>
    <td> MCUSDK-3453
    <td> Enet ICSSG: UDMA unable to process RX timestamp and packet content while reusing PDs
    <td> Enet
    <td> 8.1.0 onwards
    <td> AM64x, AM243x
    <td> Fixed
</tr>
<tr>
    <td> PINDSW-5325
    <td> Frame length and frame data is corrupted in MAC mode
    <td> ENET
    <td> 8.2.0
    <td> AM64x, AM243x
    <td> Fixed
</tr>
<tr>
    <td> PINDSW-5315
    <td> Zeroing of priority field for 802.1q ingress frames for icssg
    <td> ENET
    <td> 8.2.0
    <td> AM64x, AM243x
    <td> Fixed
</tr>
<tr>
    <td> PINDSW-5318
    <td> Switching stalls when traffic is applied to DUT during bootup
    <td> ENET
    <td> 8.2.0
    <td> AM64x, AM243x
    <td> Fixed
</tr>
<tr>
    <td> PINDSW-5280
    <td> Higher Cut-through latency and incorrect length(2000 byte) for a smaller packet size(~70 bytes) in 100M/10M link speeds
    <td> ENET
    <td> 8.2.0
    <td> AM64x, AM243x
    <td> Fixed
</tr>
<tr>
    <td> MCUSDK-3947
    <td> [MCAN]MCAN message acceptance filter masking is incorrect
    <td> MCAN
    <td> 07_03_00
    <td> Fixed
</tr>
<tr>
    <td> MCUSDK-1922
    <td> SBL UART appimage size limitation of less then 384 KB
    <td> SBL
    <td> 7.3.0 onwards
    <td> AM64x, AM243x
    <td> Use the buffered IO implementation of SBL UART
</tr>
<tr>
    <td> MCUSDK-3325
    <td> DDR_VTT_EN is not set by software
    <td> DDR
    <td> 8.1.0 onwards
    <td> AM64x, AM243x
    <td> -
</tr>
<tr>
    <td> MCUSDK-3124
    <td> AddrTranslateP_setRegion() clears bits in system address above LS 32-bits for R5F RAT
    <td> DPL
    <td> 8.2.0
    <td> AM64x, AM243x
    <td> -
</tr>
<tr>
    <td> MCUSDK-3408
    <td> sbl_uart_uniflash writing incorrect data to flash
    <td> SBL
    <td> 8.2.0
    <td> AM64x, AM243x
    <td> -
</tr>
<tr>
    <td> MCUSDK-3424
    <td> SBL OSPI fails to load image using OSPI DMA in HS device
    <td> OSPI, SBL, UDMA
    <td> 8.2.0
    <td> AM64x, AM243x
    <td> -
</tr>
<tr>
    <td> MCUSDK-3928
    <td> MCU+ SDK board library incorrect condition for while loop
    <td> BOARD
    <td> 8.2.0
    <td> AM243x
    <td> -
</tr>
<tr>
    <td> MCUSDK-4046
    <td> Enable I2C in sysconfig to build HDSL diagnostic application if using HDSL-AM64x-E1 Transceiver card
    <td> HDSL
    <td> 8.2.0
    <td> AM64x
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
    <th> Applicable Devices
    <th> Workaround
</tr>
<tr>
    <td> MCUSDK-626
    <td> DMA not working with ADC FIFO 1
    <td> ADC
    <td> 7.3.0 onwards
    <td> AM64x, AM243x
    <td> Use ADC FIFO 0
</tr>
<tr>
    <td> MCUSDK-1016
    <td> Semaphore does not function as expected when "post" call is present in multiple ISRs at different priorities
    <td> DPL
    <td> 7.3.0 onwards
    <td> AM64x, AM243x
    <td> Interrupt nesting should be disabled. SDK disables interrupt nesting by default.
</tr>
<tr>
    <td> MCUSDK-1900
    <td> UART Hardware Flow Control is not working
    <td> UART
    <td> 7.3.0 onwards
    <td> AM243x
    <td> -
</tr>
<tr>
    <td> MCUSDK-2113
    <td> [Docs] Sysfw RM/PM documentation doesn't specify AM243x
    <td> Docs
    <td> 8.0.0 onwards
    <td> AM243x
    <td> -
</tr>
<tr>
    <td> MCUSDK-2135
    <td> Insufficient UDMA TX channels allocated for Dual-mac mode
    <td> Enet
    <td> 8.1.0 onwards
    <td> AM64x, AM243x
    <td> Allocate minimum of 2 TX channels(ENET_CFG_TX_CHANNELS_NUM macro) in enet_cfg.h file
</tr>
<tr>
    <td> MCUSDK-2319
    <td> 2 PRU(ICSS) driver instances are added while changing Enet ICSSG instance to ICSSG0 in SysConfig
    <td> SYSCFG
    <td> 8.1.0 onwards
    <td> AM64x, AM243x
    <td> Please remove the extra one manually
</tr>
<tr>
    <td> MCUSDK-2409
    <td> MCRC CRCSetPSASeedSig() API sets channel mode to DATA capture
    <td> CRC
    <td> 8.1.0 onwards
    <td> AM64x
    <td>
</tr>
<tr>
    <td> MCUSDK-2419
    <td> MCSPI TX Only mode is not functional in DMA mode
    <td> MCSPI, UDMA
    <td> 8.2.0 onwards
    <td> AM64x, AM243x
    <td> Use TX/RX mode and ignore RX.
</tr>
<tr>
    <td> MCUSDK-2715
    <td> PKA ECDSA sign verify is not working for P-521 and BrainPool P-512R1 curves
    <td> SECURITY
    <td> 8.2.0 onwards
    <td> AM64x, AM243x
    <td> -
</tr>
<tr>
    <td> MCUSDK-3626
    <td> Enet: Phy tuning is not done correctly on AM64x/AM243x and AM263x platforms
    <td> Enet
    <td> 8.1.0 onwards
    <td> AM64x, AM243x
    <td> PHY delay is not tuned but set to value based on limited testing on a small set of boards.If packet drops are still seen, we can force the phy to set to 100mbps.Make below change in application code:
	  linkCfg->speed     = ENET_SPEED_100MBIT;
      linkCfg->duplexity = ENET_DUPLEX_FULL;
</tr>
<tr>
    <td> MCUSDK-4080
    <td> DDR_init call after warm reset causes crash in System in OSPI mode
    <td> DDR
    <td> 8.2.0
    <td> AM64x, AM243x
    <td> DDR_init should not be called in SBL if DDR is already initialized
</tr>
<tr>
    <td> MCUSDK-4379
    <td> Low Tx side throughput seen when tested using iperf application
    <td> DDR
    <td> 8.3.0
    <td> AM64x, AM243x
    <td> -
</tr>
<tr>
    <td> MCUSDK-4379
    <td> Low Tx side throughput seen when tested using iperf application
    <td> INDUS_PROTOCOL_HSR_PRP
    <td> 8.3.0
    <td> AM64x, AM243x
    <td>
\cond SOC_AM64X
    Replace mcu_plus_sdk\source\networking\lwip\lwip-config\am64x\lwipopts.h and mcu_plus_sdk\source\networking\lwip\lwip-config\am64x\lwippools.h from MCU PLUS SDK 8.2.0 release and rebuild lwip_freertos, lwip-contrib and icss_emac_lwip_if libraries.
\endcond

\cond SOC_AM243X
    Replace mcu_plus_sdk\source\networking\lwip\lwip-config\am243x\lwipopts.h and mcu_plus_sdk\source\networking\lwip\lwip-config\am243x\lwippools.h from MCU PLUS SDK 8.2.0 release and rebuild lwip_freertos, lwip-contrib and icss_emac_lwip_if libraries.
\endcond
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
    <td> MCUSDK-208
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
<tr>
    <td>
    <td>
    <td>
    <td>
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
    <td>
    <td>
    <td>
    <td>
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
<tr>
    <td>
    <td>
    <td>
    <td>
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
\cond SOC_AM243X
<tr>
    <td> Sysconfig
    <td> Sysconfig G5 and F5 pin data
    <td> In AM243 LP Sysconfig data file the G5 pin was incrrectly assigned to F5 which is fixed now and G5 is not a valid pin now
    <td> For AM243 LP any application using G5 pin earlier should use F5 pin in Sysconfig now
</tr>
\endcond
</table>

### Industrial Communications Toolkit

<table>
<tr>
    <th> Module
    <th> Affected API
    <th> Change
    <th> Additional Remarks
</tr>
<tr>
    <td> EtherCAT SubDevice FWHAL
    <td> bsp_init
    <td> Added new parameter `phy_rx_err_reg` in \ref bsp_params
    <td> Allows configuration of vendor specific 0x0E28 register in TI EtherCAT SubDevice Controller (ESC) for the RX Error counting during frames.
</tr>
</table>
