# Release Notes 08.04.00 {#RELEASE_NOTES_08_04_00_PAGE}

[TOC]

\attention Also refer to individual module pages for more details on each feature, unsupported features, important usage guidelines.

\attention A53 support is applicable for AM64x only. It is NOT applicable for AM243x. \n

\note The examples will show usage of SW modules and APIs on a specific CPU instance and OS combination. \n
      Unless noted otherwise, the SW modules would work in both FreeRTOS and NORTOS environment. \n
      Unless noted otherwise, the SW modules would work on any of the R5F's present on the SOC. \n
      Unless noted otherwise, the SW modules would work on all supported EVMs \n
      M4F drivers support only MCU domain peripheral and peripheral instance while R5/A53 supports drivers support MAIN domain peripheral and peripheral instance. \n

## New in this Release

\cond SOC_AM64X
Feature                                                                                         | Module
------------------------------------------------------------------------------------------------|-----------------------------------
SR 2.0 HS-FS device support, see \ref HSFS_MIGRATION_GUIDE for more details                     | Common
SysConfig Support for GPIO interrupt configuration                                              | GPIO
Syconfig support and flash driver redesign to enable new flash configuration                    | Flash
Optimized trigonometric funtion support on R5 core                                              | Common
McSPI Driver update for performance improvement                                                 | SBL, CPSW
Warm reset workaround for CPSW register lockup in MMCSD and OSPI SBL                            | SBL
TCP/IP Checksum offload to hardware support enabled in ethernet CPSW driver                     | CPSW
Simplified LwIP CPSW examples added for reference                                               | CPSW
CPSW driver memory footprint reduced by more than 40%                                           | CPSW
CPSW driver support added for MDIO manual mode                                                  | CPSW
CPSW ALE congurations are moved to SysConfig GUI                                                | CPSW
CPSW DSCP Priority mapping and Policer usecase is added in the CPSW example                     | CPSW
Multi-core (R5) support added for CPSW driver                                                   | CPSW
New CPSW Layer2 loop back example is added                                                      | CPSW
MDIO Manual Mode Support for EtherCAT SubDevice Beckhoff SSC examples and HSR-PRP examples          | Industrial Communications Toolkit
Removal of Profinet Device Examples                                                             | Industrial Communications Toolkit
HDSL: Short Message Read Feature                                                                | Motor Control
HSR(High-availability Seamless Redundancy) demo example                                         | Industrial Communications Toolkit
SDL: MCRC and ESM                                                                               | SDL
\endcond

\cond SOC_AM243X
Feature                                                                                         | Module
------------------------------------------------------------------------------------------------|-----------------------------------
SR 2.0 HS-FS device support, see \ref HSFS_MIGRATION_GUIDE for more details                     | Common
SysConfig Support for GPIO interrupt configuration                                              | GPIO
Syconfig support and flash driver redesign to enable new flash configuration                    | Flash
Optimized trigonometric funtion support on R5 core                                              | Common
McSPI Driver update for performance improvement                                                 | SBL, CPSW
Warm reset workaround for CPSW register lockup in MMCSD and OSPI SBL                            | SBL
TCP/IP Checksum offload to hardware support enabled in ethernet CPSW driver                     | CPSW
Simplified LwIP CPSW examples added for reference                                               | CPSW
CPSW driver memory footprint reduced by more than 40%                                           | CPSW
CPSW driver support added for MDIO manual mode                                                  | CPSW
CPSW ALE congurations are moved to SysConfig GUI                                                | CPSW
CPSW DSCP Priority mapping and Policer usecase is added in the CPSW example                     | CPSW
Multi-core (R5) support added for CPSW driver                                                   | CPSW
New CPSW Layer2 loop back example is added                                                      | CPSW
MDIO Manual Mode Support for EtherCAT SubDevice Beckhoff SSC examples and HSR-PRP examples          | Industrial Communications Toolkit
Removal of Profinet Device Examples                                                             | Industrial Communications Toolkit
HDSL: Short Message Read Feature                                                                | Motor Control
HSR(High-availability Seamless Redundancy) demo example                                         | Industrial Communications Toolkit
SDL: MCRC and ESM                                                                               | SDL
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
Code Composer Studio    | R5F, M4F, A53  | 12.0.0
SysConfig               | R5F, M4F, A53  | 1.13.0, build 2553
TI ARM CLANG            | R5F, M4F       | 2.1.1.LTS
GCC AARCH64             | A53            | 9.2-2019.12
GCC ARM                 | R5F            | 7-2017-q4-major (AM64x only)
FreeRTOS Kernel         | R5F, M4F, A53  | 10.4.3
FreeRTOS SMP Kernel     | A53            | 202110.00-SMP
Tiny USB                | R5F            | 0.10.0
LwIP                    | R5F            | STABLE-2_1_2_RELEASE

\attention TI ARM CLANG 2.1.1.LTS is not part of CCS by default, Follow steps at \ref INSTALL_TIARMCLANG to install the compiler

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
- Profinet Device Stack and example. For more information, see INDUSTRIAL_COMMS_TI_STACK_PROFINET_STACK_TRANSITION.
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
Profinet Device FWHAL                 | R5F            | NO                | FreeRTOS    | -                                                                                                        | RT, IRT, MRP
HSR-PRP FWHAL                         | R5F            | YES               | FreeRTOS    | Tested with hsr_prp_demo examples                                                                        | HSR/PRP RGMII Untested

### Motor Control

Module                      | Supported CPUs | SysConfig Support | OS Support        | Key features tested                                                                      | Key features not tested
----------------------------|----------------|-------------------|-------------------|------------------------------------------------------------------------------------------|------------------------
Position Sense HDSL         | R5F            | YES               | FreeRTOS, NORTOS  | Freerun mode, Sync mode, Short Message Read & Write, Long Message Read & Write           |  -
Position Sense EnDAT        | R5F            | YES               | FreeRTOS, NORTOS  | Single channel, Multi channel, Continuous mode                                           |  16 MHz Baud Rate
Position Sense Tamagawa     | R5F            | YES               | FreeRTOS, NORTOS  | Absolute position, Encoder ID, Reset                                                     |  -

### Networking

Module                      | Supported CPUs | SysConfig Support | OS Support  | Key features tested                                                                                                                                                                    | Key features not tested
----------------------------|----------------|-------------------|-------------|----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|--------------------------------------------------------------------------------------------------
LwIP                        | R5F            | YES               | FreeRTOS    | TCP/UDP IP networking stack with and without checksum offload enabled, DHCP, ping, TCP iperf, TCP/UDP IP                                                                               | Other LwIP features, performance and memory optimizations pending, more robustness tests pending, checksum offload with VLAN_Tag
Ethernet driver (ENET)      | R5F            | YES               | FreeRTOS    | Ethernet as port using CPSW and ICSS, Layer 2 MAC, Layer 2 PTP Timestamping, CPSW Switch, Policer, MDIO Manual Mode, independent ICSSG and CPSW drivers execution on different R5 cores| Independent ICSSG and CPSW drivers execution on same R5 cores not supported
ICSS-EMAC                   | R5F            | YES               | FreeRTOS    | Tested switch mode with ethernetip_adapter_demo and hsr_prp_demo examples                                                                                                              | EMAC mode, VLAN/Multicast Filtering
ICSS TimeSync               | R5F            | NO                | FreeRTOS    | Tested E2E mode with ethernetip_adapter_demo examples                                                                                                                                  | P2P mode, Transparent Clock mode

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

Module          | Supported CPUs    | SysConfig Support | OS Support        | Key features tested                                                               | Key features not tested
----------------|-------------------|-------------------|-------------------|-----------------------------------------------------------------------------------|-------------------------------------------------
ADS85x8         | R5F               | YES               | FREERTOS          | Parallel 8 and 16 Bit Interface with ADS8598H IC                                  | OSR Modes, Other ICs
ADS127L11       | R5F               | YES               | FREERTOS          | Serial Interface with ADS127L11 IC                                                | -
Empty           | PRU               | NO                | Bare Metal        | Empty project to get started with PRU firmware development                        | -
MDIO FW         | PRU               | NO                | Bare Metal        | MDIO Manual Mode firmware (tested with ethercat_slave_beckhoff_ssc_demo example)  | -

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
    <td> MCUSDK-6954
    <td> EIP_DLR_processDLRFrame API fails on large DLR Sign-On Frames
    <td> EtherNet/IP Adapter FWHAL
    <td> 7.3.0 onwards
    <td> AM64x, AM243x
    <td> Fixed
</tr>
<tr>
    <td> MCUSDK-8053
    <td> GTC read operation return incorrect values intermittently due tonon atomic read
    <td> SOC
    <td> 8.2.0 onwards
    <td> AM64x, AM243x
    <td> Fixed
</tr>
<tr>
    <td> MCUSDK-8039
    <td> CPSW: Syscfg should not have two instances of CPSW for enabling both RGMIIs
    <td> CPSW
    <td> 8.3.0 onwards
    <td> AM64x, AM243x
    <td> Fixed
</tr>
<tr>
    <td> MCUSDK-7371
    <td> [McSPI] : SPI 3 pin mode without CS pin selected in Sysconfig reports build error
    <td> McSPI
    <td> 8.3.0 onwards
    <td> AM64x, AM243x
    <td> Fixed
</tr>
<tr>
    <td> MCUSDK-7304
    <td> CPSW: LWIP example doesn't work on port 2
    <td> CPSW
    <td> 8.3.0 onwards
    <td> AM64x, AM243x
    <td> Fixed
</tr>
<tr>
    <td> MCUSDK-5867
    <td> Enet - LwIP ICSSG example build error with CCS
    <td> Enet
    <td> 8.3.0 onwards
    <td> AM64x, AM243x
    <td> Fixed
</tr>
<tr>
    <td> MCUSDK-5726
    <td> CPSW and sa2ul cannot be used together in a project via syscfg
    <td> CPSW & Security
    <td> 8.3.0 onwards
    <td> AM64x, AM243x
    <td> Fixed
</tr>
<tr>
    <td> MCUSDK-4368
    <td> Enet - running CPSW and ICSSG together fails
    <td> Enet
    <td> 8.2.0 onwards
    <td> AM243x
    <td> Fixed
</tr>
<tr>
    <td> MCUSDK-5474
    <td> SBL_OSPI crashes while trying to read CSL_secProxyGetMaxMsgSize in Sciclient_waitForBootNotification
    <td> SBL
    <td> 8.3.0 onwards
    <td> AM64x, AM243x
    <td> Fixed
</tr>
<tr>
    <td> MCUSDK-5444
    <td> Not able to define Pinmux configuration for both PRU cores
    <td> ICSS
    <td> 8.3.0 onwards
    <td> AM64x, AM243x
    <td> Fixed
</tr>
<tr>
    <td> MCUSDK-4531
    <td> USB CDC-echo example echo_serial_port on port 1 some times fails
    <td> USB
    <td> 8.2.0 onwards
    <td> AM64x, AM243x
    <td> Fixed
</tr>
<tr>
    <td> MCUSDK-4530
    <td> USB communication stops when the data packet size is over endpoint size
    <td> USB
    <td> 8.2.0 onwards
    <td> AM64x, AM243x
    <td> Fixed
</tr>
<tr>
    <td> MCUSDK-4430
    <td> CPSW: Device lockup when reading CPSW registers
    <td> CPSW
    <td> 8.2.0 onwards
    <td> AM64x, AM243x
    <td> Fixed
</tr>
<tr>
    <td> MCUSDK-4342
    <td> EEPROM template not able to handle multiple EEPROM config with same device type
    <td> EEPROM
    <td> 8.2.0 onwards
    <td> AM64x, AM243x
    <td> Fixed
</tr>
<tr>
    <td> MCUSDK-4080
    <td> DDR_init call after warm reset causes crash in System
    <td> DDR
    <td> 8.2.0 onwards
    <td> AM64x, AM243x
    <td> Fixed
</tr>
<tr>
    <td> MCUSDK-4050
    <td> Long CS assert/deassert delay before and after the MCSPI data transfer
    <td> McSPI
    <td> 8.2.0 onwards
    <td> AM64x, AM243x
    <td> Fixed
</tr>
<tr>
    <td> MCUSDK-2409
    <td> MCRC CRC_setPSASeedSig() API sets mode to DATA
    <td> CRC
    <td> 8.1.0 onwards
    <td> AM64x, AM243x
    <td> Fixed
</tr>
<tr>
    <td> MCUSDK-4346
    <td> Incorrect number of external port in syscfg template file
    <td> ICSS
    <td> 8.3.0 onwards
    <td> AM64x, AM243x
    <td> Fixed
</tr>
<tr>
    <td> MCUSDK-5472
    <td> S_PC_DATA should be used instead of PC_DATA for short message write functionality
    <td> HDSL
    <td> 8.2.0 onwards
    <td> AM64x, AM243x
    <td> Fixed
</tr>
<tr>
    <td> MCUSDK-6207
    <td> A53x, Linux references in documentation
    <td> Common
    <td> 8.3.0 onwards
    <td> AM64x, AM243x
    <td> Fixed
</tr>
<tr>
    <td> MCUSDK-7363
    <td> [enet] Documentation of file name incorrect in sysconfig generated file
    <td> Enet
    <td> 8.03.00 onwards
    <td> AM64x, AM243x
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
    <td> MCUSDK-4379
    <td> Low Tx side throughput seen when tested using iperf application
    <td> HSR-PRP
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
<tr>
    <td> MCUSDK-4527
    <td> USB recognition error occurs when USB conncect/disconnect executed repeatedly
    <td> SBL Linux
    <td> 8.2.0
    <td> AM64x, AM243x
    <td> -
</tr>
<tr>
    <td> MCUSDK-4575
    <td> ENET LWIP ICSSG Switch mode External Phy management is not functional when 2 ports are enabled
    <td> ICSSG, Enet
    <td> 8.3.0
    <td> AM64x, AM243x
    <td> -
</tr>
<tr>
    <td> MCUSDK-6318
    <td> Enet icssg - dhcp functionality unstable
    <td> Enet, ICSS
    <td> 8.3.0
    <td> AM243x
    <td> -
</tr>
<tr>
    <td> MCUSDK-7905
    <td> EtherNet/IP : MDIO access can have race condition due to two parallel PHY drivers
    <td> Ethernet/IP Adapter
    <td> 8.3.0
    <td> AM64x
    <td> -
</tr>
<tr>
    <td> MCUSDK-8106
    <td> 8MHZ endat encoder showing CRC failure
    <td> ENDAT
    <td> 8.4.0
    <td> AM64x
    <td> -
</tr>
<tr>
    <td> MCUSDK-8108
    <td> EtherNet/IP : PTP Device is unable to keep offset under 1000 ns
    <td> Ethernet/IP Adapter
    <td> 8.4.0
    <td> AM64x, AM243x
    <td> Value of OFFSET_THRESHOLD_FOR_RESET is set to 10000 ns by default in SDK.
</tr>
<tr>
    <td> MCUSDK-8239
    <td> EtherNet/IP : MDIO Manual Mode is not supported
    <td> Ethernet/IP Adapter
    <td> 8.4.0
    <td> AM64x, AM243x
    <td> MDIO Manual Mode is the work-around for issue "i2329 - MDIO: MDIO interface corruption (CPSW and PRU-ICSS)" (described in <a href="https://www.ti.com/lit/er/sprz457e/sprz457e.pdf">AM64x/AM243x Processor Silicon Revision 1.0, 2.0 (Rev. E)</a>)
</tr>
<tr>
    <td> MCUSDK-8242
    <td> EtherCAT : MDIO Manual Mode is not supported in ethercat_slave_demo examples
    <td> EtherCAT SubDevice
    <td> 8.4.0
    <td> AM64x, AM243x
    <td> MDIO Manual Mode is the work-around for issue "i2329 - MDIO: MDIO interface corruption (CPSW and PRU-ICSS)" (described in <a href="https://www.ti.com/lit/er/sprz457e/sprz457e.pdf">AM64x/AM243x Processor Silicon Revision 1.0, 2.0 (Rev. E)</a>). Please note that the work-around is available for ethercat_slave_beckhoff_ssc_demo examples.
</tr>
<tr>
    <td> MCUSDK-8243
    <td> EtherNet/IP : Examples do not work on HS-FS devices
    <td> Ethernet/IP Adapter
    <td> 8.4.0
    <td> AM64x, AM243x
    <td> -
</tr>
<tr>
    <td> MCUSDK-8234
    <td> HSR/PRP - PTP Device is unable to keep offset under 1000 ns
    <td> HSR-PRP
    <td> 8.4.0
    <td> AM64x, AM243x
    <td> -
</tr>
<tr>
    <td> MCUSDK-8236
    <td> HSR/PRP is not functional in rgmii mode
    <td> HSR-PRP
    <td> 8.4.0
    <td> AM64x, AM243x
    <td> -
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
    <td> -
    <td> -
    <td> Enabled copmpiler option -Oz and -flto for release mode build
    <td> These option are enabled for Code size and performance optimization
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
    <td> CPSW and ICSSG (enet)
    <td> All APIs mentioned in \ref enetmdio_manualmode
    <td> Application has to enable PHY register polling logic to fnd the link status as MDIO manual_mode is enabled by default. MDIO LINK_INT and USER_INT are disabled and hence applciation cannot register and LINK_INT callback function.
    <td> more details are here : \ref enetmdio_manualmode
</tr>
<tr>
    <td> Industrial Communications Toolkit
    <td> \ref DRV_MDIO_MODULE
    <td> If MDIO Manual Mode is used as the work-around for issue "i2329 - MDIO: MDIO interface corruption (CPSW and PRU-ICSS)" (described in <a href="https://www.ti.com/lit/er/sprz457e/sprz457e.pdf">AM64x/AM243x Processor Silicon Revision 1.0, 2.0 (Rev. E)</a>), usage of MDIO APIs is affected.
    <td> This work-around is enabled in EtherCAT SubDevice Beckhoff SSC examples and HSR-PRP examples. Please see INDUSTRIAL_COMMS_MDIO_MANUALMODE_FW_USAGE for more details.
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
<tr>
    <td> MCSPI
    <td> MCSPI_chConfig
    <td> Added parameter for trigger level configuration `txFifoTrigLvl`, `rxFifoTrigLvl` in \ref MCSPI_ChConfig
    <td> Allows configuring tragger level through sys config. Regenerate the sys config files
</tr>
<tr>
    <td> MCAN
    <td> MCAN_getTxBufCancellationIntrEnable
    <td> Name changed from MCAN_getTxBufCancellationIntrEnable to MCAN_txBufCancellationIntrEnable.
    <td> MCAN_getTxBufCancellationIntrEnable function sets (does not get). So renamed to MCAN_txBufCancellationIntrEnable.
</tr>
<tr>
    <td> OSPI
    <td> OSPI_setXferLines, OSPI_getXferLines
    <td> Removed APIs
    <td> Addition of more generic OSPI_setProtocol, OSPI_getProtocol APIs
</tr>
<tr>
    <td> Bootloader
    <td> Bootloader_socLoadSysFw
    <td> API removed. Bootflow change from legacy to combined bootflow
    <td> For more information please refer \ref BOOTFLOW_MIGRATION_GUIDE
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
    <td> ICSS TimeSync
    <td> TimeSync_drvInit
    <td> Added new members `delayReqSendTaskPriority`, `txTsTaskPriority`, `nrtTaskPriority`, and `backgroundTaskPriority` in `TimeSync_Config_t`
    <td> Allows configuration of task priority for task created by ICSS TimeSync driver
</tr>
</table>

### PRU-IO

<table>
<tr>
    <th> Module
    <th> Affected API
    <th> Change
    <th> Additional Remarks
</tr>
<tr>
    <td> ADC
    <td> None
    <td> Changed file names in `source/pru_io/.meta/adc` from `am64x_adapter.syscfg.js` to `adc_phi_pru_evm_adapter.syscfg.js`
    <td> Aligning the filenames with the actual adapter board name
</tr>
</table>
