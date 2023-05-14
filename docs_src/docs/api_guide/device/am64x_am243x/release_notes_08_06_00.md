# Release Notes 08.06.00 {#RELEASE_NOTES_08_06_00_PAGE}

[TOC]

\attention Also refer to individual module pages for more details on each feature, unsupported features, important usage guidelines.

\attention A53 support is applicable for AM64x only. It is NOT applicable for AM243x. \n

\note The examples will show usage of SW modules and APIs on a specific CPU instance and OS combination. \n
      Unless noted otherwise, the SW modules would work in both FreeRTOS and NORTOS environment. \n
      Unless noted otherwise, the SW modules would work on any of the R5F's present on the SOC. \n
      Unless noted otherwise, the SW modules would work on all supported EVMs \n
      M4F drivers support only MCU domain peripheral and peripheral instance while R5/A53 supports drivers support MAIN domain peripheral and peripheral instance. \n

\attention Klockwork Static Analysis report is not updated for this release

## New in this Release

\cond SOC_AM64X
Feature                                                                                         | Module
------------------------------------------------------------------------------------------------|-----------------------------------
Tamagawa Multi Channel                                                                          | Position Sense Tamagawa
HDSL FREE RUN MODE based on 300 MHz PRU-ICSS Core Clock Frequency                               | Position Sense HDSL
Enable EnDat multi-channel using load share mode in PRU-ICSS                                    | Position Sense EnDat
EnDat Safety Readiness: Recovery Time Measurement                                               | Position Sense EnDat
Mbed-TLS library support (software cryptography)                                                | Networking
SBL PCIe support                                                                                | SBL
PMU driver and example support                                                                  | PMU
ECAP as PWM example                                                                             | ECAP
CPSW_3G: IET support (packet preemption)                                                        | Networking (CPSW)
Cut-through switching support                                                                   | Networking (CPSW)
3G credit based shaper (802.1qav) support                                                       | Networking (CPSW)
CPSW-3G DSCP priority mapping feature w/ RTOS                                                   | Networking (CPSW)
Scatter/Gather support for receive packet buffer memory                                         | Networking (CPSW)
Support for ESM, MCRC, RTI, DCC, VTM, STOG, PBIST, MTOG, POK, ECC modules are added as part of SDL	|SDL
\endcond

\cond SOC_AM243X
Feature                                                                                         | Module
------------------------------------------------------------------------------------------------|-----------------------------------
Tamagawa Multi Channel                                                                          | Position Sense Tamagawa
HDSL FREE RUN MODE based on 300 MHz PRU-ICSS Core Clock Frequency                               | Position Sense HDSL
Enable EnDat multi-channel using load share mode in PRU-ICSS                                    | Position Sense EnDat
EnDat Safety Readiness: Recovery Time Measurement                                               | Position Sense EnDat
Mbed-TLS library support (software cryptography)                                                | Networking
SBL PCIe support                                                                                | SBL
PMU driver and example support                                                                  | PMU
ECAP as PWM example                                                                             | ECAP
CPSW_3G: IET support (packet preemption)                                                        | Networking (CPSW)
Cut-through switching support                                                                   | Networking (CPSW)
3G credit based shaper (802.1qav) support                                                       | Networking (CPSW)
CPSW-3G DSCP priority mapping feature w/ RTOS                                                   | Networking (CPSW)
Scatter/Gather support for receive packet buffer memory                                         | Networking (CPSW)
Support for ESM, MCRC, RTI, DCC, VTM, STOG, PBIST, MTOG, POK, ECC modules are added as part of SDL	|SDL
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
Mbed-TLS                | R5F            | @VAR_MBEDTLS_VERSION

\attention TI ARM CLANG @VAR_TI_ARM_CLANG_VERSION is not part of CCS by default, Follow steps at \ref INSTALL_TIARMCLANG to install the compiler

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

- Profinet Device Stack and example. For more information, see \ref INDUSTRIAL_COMMS_TI_STACK_PROFINET_STACK_TRANSITION.
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
ADC        | R5F            | YES               | Yes          | Single conversion (one-shot mode), interrupt mode, DMA mode                                | Continuous conversion not tested
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

### Software Diagnostic Library (SDL)

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

Module                      | Supported CPUs | SysConfig Support | OS Support        | Key features tested                                                                                             | Key features not tested
----------------------------|----------------|-------------------|-------------------|-----------------------------------------------------------------------------------------------------------------|------------------------
Position Sense HDSL         | R5F            | YES               | FreeRTOS, NORTOS  | Freerun mode(300MHz,225MHz), Sync mode(225MHz), Short Message Read & Write, Long Message Read & Write, Boosterpack with AM243x-LP           |  Long cables
Position Sense EnDat        | R5F            | YES               | FreeRTOS, NORTOS  | Single channel, Multi channel, Continuous mode for single channel, Load share mode, Recovery Time for 2.2 command set, Boosterpack with AM243x-LP              |  16 MHz Baud Rate Different cable lengths, Continuous clock mode for multi channel
Position Sense Tamagawa     | R5F            | YES               | FreeRTOS, NORTOS  | Absolute position, Encoder ID, Reset, EEPROM Read, EEPROM Write, 2.5 Mbps and 5 Mbps Encoder Support, Boosterpack with AM243x-LP            |  -

### Networking

Module                      | Supported CPUs | SysConfig Support | OS Support  | Key features tested                                                                                                                                                                    | Key features not tested
----------------------------|----------------|-------------------|-------------|----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|--------------------------------------------------------------------------------------------------
LwIP                        | R5F            | YES               | FreeRTOS    | TCP/UDP IP networking stack with and without checksum offload enabled, TCP/UDP IP networking stack with server and client functionality, basic Socket APIs, netconn APIs and raw APIs, DHCP, ping, TCP iperf, scatter-gather, DSCP priority mapping                         | Other LwIP features
Ethernet driver (ENET)      | R5F            | YES               | FreeRTOS    | Ethernet as port using CPSW and ICSS,  MAC loopback and PHY loopback, Layer 2 MAC, Packet Timestamping, CPSW Switch, ICSSG as two port switch, Policer and Classifier, MDIO Manual Mode, independent ICSSG and CPSW drivers execution on different R5 cores, CBS (IEEE 802.1Qav) on CPSW, IET (IEEE 802.1Qbu) on CPSW, Strapped PHY (Early Ethernet), cut through switch on CPSW,  | Independent ICSSG and CPSW drivers execution on same R5 cores not supported, RMII mode
ICSS-EMAC                   | R5F            | YES               | FreeRTOS    | Tested switch mode with ethernetip_adapter_demo and hsr_prp_demo examples                                                                                                              | EMAC mode, VLAN/Multicast Filtering
ICSS TimeSync               | R5F            | NO                | FreeRTOS    | Tested E2E mode with ethernetip_adapter_demo examples                                                                                                                                  | P2P mode, Transparent Clock mode
Mbed-TLS                    | R5F            | NO                | FreeRTOS    | Tested software cryptography after porting, used mbedTLS with LwIP to implement HTTPS server  | Hardware offloaded cryptography

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
    <td> MCUSDK-9458
    <td> Errata i2310 causes erroneous set of UART timeout interrut
    <td> UART
    <td> 8.0.0 onwards
    <td> AM64x, AM243x
    <td> Errata, Implemented the Workaround
</tr>
<tr>
    <td> MCUSDK-9401
    <td> AM243x LP: Bootloader: Uart uniflash setting incorrect QE bit
    <td> SBL
    <td> 8.5.0 onwards
    <td> AM243x
    <td> Updated the Sysconfig to set correct QE bit value
</tr>
<tr>
    <td> MCUSDK-9398
    <td> Change to GPIO interrupt router output allocation not working in Sysconfig
    <td> GPIO
    <td> 8.5.0 onwards
    <td> AM64x, AM243x
    <td> None
</tr>
<tr>
    <td> MCUSDK-9044
    <td> Strapping mode in phy is not giving correct link speed
    <td> Ethernet
    <td> 8.5.0 onwards
    <td> AM64x, AM243x
    <td> Phy configuration for strapped and forced mode was not correctly handled
</tr>
<tr>
    <td> MCUSDK-9042
    <td> Failure in Bootloader_loadSelfCpu for CORE_ID_R5FSS0_1 for AM2432 devices
    <td> SBL
    <td> 8.5.0 onwards
    <td> AM243x
    <td> Changes to check for dual core mode before doing init for second core
</tr>
<tr>
    <td> MCUSDK-8985
    <td> Potential Infinite loop in OSPI_utilLog2 defined in ospi_v0.c
    <td> OSPI
    <td> 8.4.0 onwards
    <td> AM64x, AM243x
    <td> Incorrect condition for loop termination
</tr>
<tr>
    <td> MCUSDK-8383
    <td> Load from JSON feature fails in SysConfig in Windows PC
    <td> Flash
    <td> 8.4.0 onwards
    <td> AM64x, AM243x
    <td> Updated the sysconfig to use OS agnostic copy function
</tr>
<tr>
    <td> MCUSDK-8106
    <td> 8MHZ endat encoder showing CRC failure
    <td> Position Sense EnDat
    <td> 8.4.0 onwards
    <td> AM64x
    <td> Endat Initialization was incorrect
</tr>
<tr>
    <td> MCUSDK-9304
    <td> LWIP CPSW Socket: Putting Udp application buffer in cached region of memory causes stale data to be sent out in Udp packets
    <td> ENET
    <td> 8.4.0 onwards
    <td> AM64x, AM243x
    <td> Fixed the udp examples and added udp client socket example
</tr>
<tr>
    <td> MCUSDK-9185
    <td> Enet Lwip CPSW example: Correct MAC address not available from EEPROM on custom board and Pg1.0 lp causes example crash
    <td> ENET
    <td> 8.4.0 onwards
    <td> AM64x, AM243x
    <td> -
</tr>
<tr>
    <td> MCUSDK-9644
    <td> Incorrect List toggle operation in profinet device driver
    <td> Profinet Device
    <td> 7.3.0 onwards
    <td> AM64x, AM243x
    <td> -
</tr>
<tr>
    <td> MCUSDK-9578
    <td> ICSS-EMAC : IOCTL for statistics always returns errors
    <td> ICSS-EMAC
    <td> 7.3.0 onwards
    <td> AM64x, AM243x
    <td> -
</tr>
<tr>
    <td> MCUSDK-9386
    <td> Profinet does not respond to ARP frames
    <td> Profinet Device
    <td> 8.4.0 onwards
    <td> AM64x, AM243x
    <td> -
</tr>
<tr>
    <td> MCUSDK-9582
    <td> Profinet, EtherNet/IP: Enabling MDIO Work-around leads to data memory corruption
    <td> Profinet Device, EtherNet/IP Adapter
    <td> 8.4.0 onwards
    <td> AM64x, AM243x
    <td> -
</tr>
<tr>
    <td> MCUSDK-9640
    <td> ICSS-EMAC : isNRT flag in ICSS_EMAC_pollPkt is not cleared appropriately
    <td> ICSS-EMAC
    <td> 8.2.0 onwards
    <td> AM64x, AM243x
    <td> -
</tr>
<tr>
    <td> MCUSDK-8983
    <td> EtherCAT : EDIO pins for AL event is not supported in firmware.
    <td> EtherCAT
    <td> 7.3.0 onwards
    <td> AM64x, AM243x
    <td> -
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
    <td> MCUSDK-8243
    <td> EtherNet/IP : Examples do not work on HS-FS devices
    <td> Ethernet/IP Adapter
    <td> 8.4.0
    <td> AM64x, AM243x
    <td> -
</tr>
<tr>
    <td> MCUSDK-8403
    <td> 1000000(1MHz) baud rate not working on UART
    <td> UART
    <td> 8.4.0
    <td> AM64x, AM243x
    <td> -
</tr>
<tr>
    <td> MCUSDK-8413
    <td> ICSSG: Disabling MDIO manual mode with board phy config cause failure
    <td> ICSSG
    <td> 8.4.0
    <td> AM64x, AM243x
    <td> -
</tr>
<tr>
    <td> MCUSDK-8490
    <td> HSR/PRP nodeTable semaphore causing a deadlock
    <td> HSR_PRP
    <td> 8.4.0
    <td> AM64x, AM243x
    <td> -
</tr>
<tr>
    <td> MCUSDK-9022
    <td> USB: Enumeration Issues while running connect - data transfer - disconnect
    <td> USB
    <td> 8.4.0 onwards
    <td> AM64x, AM243x
    <td> -
</tr>
<tr>
    <td> MCUSDK-9023
    <td> USB : CPU hangs with running connect/disconnect with power cycled inbetween
    <td> USB
    <td> 8.4.0 onwards
    <td> AM64x, AM243x
    <td> -
</tr>
<tr>
    <td> MCUSDK-9312
    <td> ospi_flash_diag example not working
    <td> OSPI
    <td> 8.5.0
    <td> AM64x, AM243x
    <td> dummyCycles initialized to 0
</tr>
<tr>
    <td> MCUSDK-9655
    <td> ENET PHY state Machine: support to disable timeout for auto-negotiation
    <td> ENET
    <td> 8.4.0 onwards
    <td> AM64x, AM243x
    <td> -
</tr>
<tr>
    <td> MCUSDK-9705
    <td> DPL: FreeRTOS tick interrupt preemption leading to System crash
    <td> DPL
    <td> 8.5.0
    <td> AM64x, AM243x
    <td> FreeRTOS timer tick increment shall be done from the critical section
</tr>
<tr>
    <td> MCUSDK-9889
    <td> Link Status Not getting updated correctly in lwip2emac interface
    <td> HSR-PRP
    <td> 8.3.0 onwards
    <td> AM64x, AM243x
    <td> Incorrect assignment of link status from emac to lwip interface APIs
</tr>
<tr>
    <td> MCUSDK-10640
    <td> DPL: Timer drift on R5 core
    <td> DPL
    <td> 8.4.0 onwards
    <td> AM64x, AM243x
    <td> Incorrect calculation for timer count value in timerP module
</tr>
<tr>
    <td> MCUSDK-2419
    <td> MCSPI TX Only mode is not functional in DMA mode
    <td> MCSPI, UDMA
    <td> 8.2.0 onwards
    <td> AM64x, AM243x
    <td> -
</tr>
<tr>
    <td> MCUSDK-8414
    <td> SBL UART Uniflash: OSPI fails to boot application image with size > 1MB
    <td> OSPI, Flash
    <td> 8.4.0
    <td> AM64x, AM243x
    <td> -
</tr>
<tr>
    <td> MCUSDK-9790
    <td> XIP Benchmark Example does not work
    <td> XIP
    <td> 8.5.0
    <td> AM64x, AM243x
    <td> -
</tr>
<tr>
    <td> MCUSDK-10205
    <td> GTC Errorneous value returned while using "GTC_getCount64" API
    <td> Timer
    <td> 8.5.0
    <td> AM64x, AM243x
    <td> -
</tr>
<tr>
    <td> MCUSDK-10206
    <td> Systick drift on M4
    <td> Timer
    <td> 8.5.0
    <td> AM64x, AM243x
    <td> -
</tr>
<tr>
    <td> MCUSDK-9596
    <td> Enet loopback example version cleanup
    <td> Networking
    <td> 8.3.0 onwards
    <td> AM64x, AM243x
    <td> Fixed
</tr>
<tr>
    <td> MCUSDK-9304
    <td> LWIP CPSW Socket: Putting Udp application buffer in cached region of memory causes stale data to be sent out in Udp packets
    <td> Networking
    <td> 8.4.0 onwards
    <td> AM64x, AM243x
    <td> Fixed
</tr>
<tr>
    <td> MCUSDK-9185
    <td> Enet Lwip CPSW example: Correct MAC address not available from EEPROM on custom board and Pg1.0 lp causes example crash
    <td> Networking
    <td> 8.4.0 onwards
    <td> AM64x, AM243x
    <td> Fixed
</tr>
<tr>
    <td> MCUSDK-9577
    <td> AM2432: Lwip concatenated pbuf isn't trasmitted complete
    <td> Networking
    <td> 8.5.0 onwards
    <td> AM64x, AM243x
    <td> Fixed
</tr>
<tr>
    <td> MCUSDK-9655
    <td> ENET PHY state Machine: support to disable timeout for auto-negotiation
    <td> Networking
    <td> 8.2.0 onwards
    <td> AM64x, AM243x
    <td> Fixed
</tr>
<tr>
    <td> MCUSDK-9532
    <td> Not getting PA-STATS for Port1 and Port2
    <td> Networking
    <td> 8.5.0 onwards
    <td> AM64x, AM243x
    <td> Fixed
</tr>
<tr>
    <td> MCUSDK-9598
    <td> IOCTL command "ENET_PHY_IOCTL_PRINT_REGS" not working
    <td> Networking
    <td> 8.5.0 onwards
    <td> AM64x, AM243x
    <td> Fixed
</tr>
<tr>
    <td> MCUSDK-9656
    <td> Documentation Missing: driver marks timeout and move to "STATE_FOUND driver re-starts the process from beginning (PHY reset)
    <td> Networking
    <td> 8.4.0 onwards
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
    <td> MCUSDK-2512
    <td> [UART]Driver always assumes functional clock as 48 MHz
    <td> UART
    <td> 8.3.0 onwards
    <td> AM64x, AM243x
    <td> -
</tr>
<tr>
    <td> MCUSDK-6262
    <td> [AM243X] : MMCSD read io example is not functional on eMMC if the APP_MMCSD_START_BLK is changed for MMCSD_write and MMCSD_read
    <td> MMCSD
    <td> 8.3.0 owards
    <td> AM243x, AM64x
    <td> -
</tr>
<tr>
    <td> MCUSDK-8842
    <td> OSPI Writes fail with multi threaded applications
    <td> OSPI
    <td> 8.4.0
    <td> AM64x, AM243x
    <td> -
</tr>
<tr>
    <td> MCUSDK-8938
    <td> Last 512KB of memory is not accessible in dev boot mode flow
    <td> SBL
    <td> 8.4.0
    <td> AM64x, AM243x
    <td> Use other boot modes
</tr>
<tr>
    <td> MCUSDK-8945
    <td> Boot Time Degradation is observed for HS-FS device
    <td> SBL
    <td> 8.5.0
    <td> AM64x, AM243x
    <td> Skip the authentication of application Image using SysConfig
</tr>
<tr>
    <td> PROC_SDL-6010
    <td> ECC is not supported for 2 instances. These are SDL_ECC_AGGR1 Ram ID 4 fails on interconnect ram ID 4 checker group 4-14 and  SDL_PCIE0_PCIE_G2X1_64_CORE_CORE_ECC_AGGR.
    <td> SDL
    <td> 8.6.0
    <td> AM64X, AM243X
</tr>
<tr>
    <td> <a href="https://mbed-tls.readthedocs.io/en/latest/tech-updates/security-advisories/mbedtls-security-advisory-2021-07-1/">mbedTLS-advisory</a> <br> MCUSDK-9082
    <td> MbedTLS - RSA exploit by kernel-privileged cache side-channel attackers
    <td> Mbed-TLS
    <td> 8.6.0
    <td> AM64x, AM243x, AM263X, AM273X
    <td> -
</tr>
<tr>
    <td> MCUSDK-10627
    <td> Jtag Uniflash erase operation failure
    <td> Flash
    <td> 8.6.0
    <td> AM64x, AM243x
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
    <td> MCUSDK-8108
    <td> EtherNet/IP : PTP Device is unable to keep offset under 1000 ns
    <td> Ethernet/IP Adapter
    <td> 8.4.0
    <td> AM64x, AM243x
    <td> Value of OFFSET_THRESHOLD_FOR_RESET is set to 10000 ns by default in SDK.
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
<tr>
    <td> MCUSDK-8242
    <td> EtherCAT : MDIO Manual Mode is not supported in ethercat_slave_demo examples
    <td> EtherCAT SubDevice
    <td> 8.4.0
    <td> AM64x, AM243x
    <td> MDIO Manual Mode is the work-around for issue "i2329 - MDIO: MDIO interface corruption (CPSW and PRU-ICSS)" (described in <a href="https://www.ti.com/lit/er/sprz457e/sprz457e.pdf">AM64x/AM243x Processor Silicon Revision 1.0, 2.0 (Rev. E)</a>). Please note that the work-around is available for ethercat_slave_beckhoff_ssc_demo examples.
</tr>
<tr>
    <td> MCUSDK-8376
    <td> LWIP web server application crashes in server stress test
    <td> Enet, LWIP
    <td> 8.3.0 onwards
    <td> AM64x, AM243x
    <td> -
</tr>
<tr>
    <td> MCUSDK-8564
    <td> SysConfig Code generation error with basic PRU config on ICSS_G0 and ICSS_G1
    <td> ICSSG
    <td> 8.4.0
    <td> AM64x, AM243x
    <td> -
</tr>
<tr>
    <td> MCUSDK-8721
    <td> Function for setting ICSSG SD/ENDAT alternate pin mux mode not working in all cases
    <td> ICSSG
    <td> 8.4.0
    <td> AM64x, AM243x
    <td> -
</tr>
<tr>
    <td> MCUSDK-8491
    <td> Enet_loopback example: Non zero vlan priority Packets not recieved in loopback example
    <td> Networking
    <td> 8.4.0
    <td> AM64x, AM243x
    <td> -
</tr>
<tr>
    <td> MCUSDK-8361
    <td> ENET Layer 2 CPSW Switch Port 2 does not link up for AM64x-SK baord
    <td> Networking
    <td> 8.4.0
    <td> AM64x
    <td> -
</tr>
<tr>
    <td> MCUSDK-9739
    <td> AM64B SK loss of packet on using CPSW switch
    <td> Networking
    <td> 8.5.0
    <td> AM64x
    <td> -
</tr>
<tr>
    <td> MCUSDK-10679
    <td> CPSW UDP Iperf test instability on AM243x
    <td> Networking
    <td> 8.6.0
    <td> AM243x
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
    <td> Position Sense Tamagawa
    <td> -
    <td> The entire example and driver is new and changed.
    <td> The previous example used a UART based implementation. The new example uses 3 channel Peripheral Interface.
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
<tr>
    <td> FSI
    <td> Macro FSI_RX_MASTER_CORE_RESET, FSI_TX_MASTER_CORE_RESET
    <td> API/MACRO/STRUCTURE name are updated while keeping the case sensitivity from **MASTER** to **MAIN**
    <td> Updated to use the inclusive naming
</tr>
<tr>
    <td> I2C
    <td> Structure I2C_Transaction member slaveAddress, masterMode
    <td> API/MACRO/STRUCTURE name are updated while keeping the case sensitivity from **master** to **controller** and **slave** to **target**, for example..\n
    slaveAddress->targetAddress
    <td> Updated to use the inclusive naming
</tr>
<tr>
    <td> McASP
    <td> MACRO MCASP_OPMODE_MASTER, MCASP_OPMODE_SLAVE
    <td> API/MACRO/STRUCTURE name are updated while keeping the case sensitivity from **master** to **controller**, for example..\n
    MCASP_OPMODE_MASTER->MCASP_OPMODE_CONTROLLER
    <td> Updated to use the inclusive naming
</tr>
<tr>
    <td> MibSPI
    <td> MACRO MIBSPI_SLAVEMODE_TRANS_GROUP, MIBSPI_SLAVE_MAX \n
    Enum MIBSPI_Mode members MIBSPI_MASTER, MIBSPI_SLAVE\n
    Structure MIBSPI_SlaveProfile\n
    Structure MIBSPI_SlaveModeParams\n
    Structure MIBSPI_Transaction member slaveIndex\n
    Structure MIBSPI_MasterModeParams
    <td> API/MACRO/STRUCTURE name are updated while keeping the case sensitivity from **master** to **controller** and **slave** to **peripheral**, for example..\n
    MIBSPI_SlaveProfile->MIBSPI_PeripheralProfile
    <td> Updated to use the inclusive naming
</tr>

### Motor Control

<table>
<tr>
    <th> Module
    <th> Affected API
    <th> Change
    <th> Additional Remarks
</tr>
<tr>
    <td> Position Sense EnDat
    <td> \ref endat_wait_initialization
    <td> Added one argument `mask`
    <td> It is used to pass the value of channel mask
</tr>
<tr>
    <td> Position Sense EnDat
    <td> \ref endat_init
    <td> Added one argument `slice`
    <td> It is used to pass the PRU-ICSSG Slice value
</tr>
<tr>
    <td> Position Sense EnDat
    <td> \ref endat_config_multi_channel_mask
    <td> Add one argument `loadshare`
    <td> It is used to enable/disable load share mode
</tr>
<tr>
    <td> Position Sense EnDat
    <td> `endat_pruss_xchg` structure
    <td> Added `endat_pruss_config` per channel, `endat_pruss_cmd` per channel, `endat_delay_125ns`, `endat_delay_5us`, `endat_delay_51us`, `endat_delay_1ms`, `endat_delay_380ms`, `endat_delay_900ms`, `endat_primary_core_mask`, `endat_ch0_syn_bit`, `endat_ch1_syn_bit`, `endat_ch2_syn_bit`
    <td> These changes are required to support multi-channel using load share mode
</tr>
<tr>
    <td> Position Sense EnDat
    <td> `endat_priv` structure
    <td> Added `pruicss_slicex`, `load_share`, `pos_rx_bits_21_RTUPRU`, `pos_rx_bits_21_PRU`, `pos_rx_bits_21_TXPRU`, `pos_rx_bits_22_RTUPRU`, `pos_rx_bits_22_PRU`, `pos_rx_bits_22_TXPRU`
    <td> These changes are required to support multi-channel using load share mode
</tr>
<tr>
    <th> Position Sense EnDat
    <th> `endat_pruss_xchg` structure
    <th> Added `endat_ch0_rt`, `endat_ch1_rt`, `endat_ch0_rt`, `icssg_clk`
    <th> These changes are required to store recovery time in DMEM.
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
    <td> ICSS-EMAC
    <td> \ref ICSS_EMAC_txPacket
    <td> Return value is updated to provide detailed error codes.
    <td> -
</tr>
<tr>
    <td> Ethernet CPSW
    <td> Structure \ref CpswHostPort_Cfg in \ref Cpsw_Cfg \n
    Function Enet_open
    <td> Replaced csumOffloadEn parameter with txCsumOffloadEn.
    <td> This controls both support to control TXP/DUP checksum offload along both Rx and Tx direction
</tr>
<tr>
    <td> Ethernet CPSW
    <td> LwIP Interface
    Function LwipifEnetApp_netifOpen
    Function LwipifEnetApp_startSchedule
    Function LWIPIF_LWIP_start
    <td> Added enetType and instId as additional arguments.
    <td> Based upon the association of LwIP NetIF, the above arguments needs to be populated.
</tr>
<tr>
    <td> Ethernet (CPSW and ICSSG) - Networking
    <td> LwIP Interface
    Function LwipifEnetApp_netifOpen
    Function LwipifEnetApp_startSchedule
    Function LWIPIF_LWIP_start
    <td> Added enetType and instId as additional arguments.
    <td> Based upon the association of LwIP NetIF, the above arguments needs to be populated.
</tr>
<tr>
    <td> Ethernet (CPSW and ICSSG) - Networking
    <td> ENET SYSCFG Interface
    Function EnetApp_getEnetInstInfo
    Function EnetApp_getEnetInstMacInfo
    <td> Added ENET INSTANCE NAME as input arguments.
    <td> Based upon the ENET modules that are instantiated in syscfg, the name of Instance has to be passed to EnetApp_getEnetInstInfo. Instance names can be found in ti_enet_config.h file(generated) as per the syscfg settings.
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
</table>
