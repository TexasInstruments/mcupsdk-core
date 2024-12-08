# Release Notes 08.02.00 {#RELEASE_NOTES_08_02_00_PAGE}

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
------------------------------------------------------------------------------------------------|--------------------------
MCSPI Loopback Example with DMA mode supported                                                  | MCSPI
Watchdog interrupt mode                                                                         | WDT
LPDDR4(Supported on AM64x SK)                                                                   | DDR
Simplified Hello World Example                                                                  | EXAMPLES
A53 FreeRTOS SMP kernel                                                                         | DPL, FreeRTOS
Secure boot on HS-SE devices (With extended boot auth)                                          | Bootloader, Sciclient
UART with DMA mode supported                                                                    | UART
RSA Encryption and Decryption, RSA Signing and Verification, ECDSA with P-256 and P-384 curves  | SECURITY
CPSW Switch supported                                                                           | ENET
PRP demo example                                                                                | Industrial Communications Toolkit
Tamagawa Encoder support over PRU UART                                                          | Tamagawa Diagnostic
Support higher frequencies for EnDAT Firmware                                                   | EnDAT Diagnostic
Add support for HDSL Long Messages                                                              | HDSL Diagnostic
Add support in HDSL FW to meet SICK IP v7.                                                      | HDSL Diagnostic
PRU-ADC Parallel Interface Support for ADS85x8 ICs                                              | PRU-IO
Combined Boot Mode for SBL image                                                                | SBL
SBL JTAG Uniflash Example supported                                                             | SBL

\endcond

\cond SOC_AM243X
Feature                                                                                         | Module
------------------------------------------------------------------------------------------------|--------------------------
MCSPI Loopback Example with DMA mode supported                                                  | MCSPI
Watchdog interrupt mode                                                                         | WDT
Simplified Hello World Example                                                                  | EXAMPLES
UART with DMA mode supported                                                                    | UART
RSA Encryption and Decryption, RSA Signing and Verification, ECDSA with P-256 and P-384 curves  | SECURITY
CPSW Switch supported                                                                           | ENET
PRP demo example                                                                                | Industrial Communications Toolkit
PRU-ADC Parallel Interface Support for ADS85x8 ICs                                              | PRU-IO
Combined Boot Mode for SBL image                                                                | SBL
SBL JTAG Uniflash Example supported                                                             | SBL

\endcond

## Device and Validation Information

\cond SOC_AM64X
SOC   | Supported CPUs  | EVM                                             | Host PC
------|-----------------|-------------------------------------------------|-----------------------------------
AM64x | R5F, M4F, A53   | AM64x GP EVM (referred to as am64x-evm in code) | Windows 10 64b or Ubuntu 18.04 64b
\endcond

\cond SOC_AM243X
SOC    | Supported CPUs  | Boards                                                                                                      | Host PC
-------|-----------------|-------------------------------------------------------------------------------------------------------------|-----------------------------------
AM243x | R5F, M4F        | AM243x GP EVM (referred to as am243x-evm in code), \n AM243x LAUNCHPAD (referred to as am243x-lp in code)   | Windows 10 64b or Ubuntu 18.04 64b
\endcond

## Tools, Compiler and Other Open Source SW Module Information

Tools / SW module       | Supported CPUs | Version
------------------------|----------------|-----------------------
Code Composer Studio    | R5F, M4F, A53  | 11.1.0
SysConfig               | R5F, M4F, A53  | 1.11.0, build 2225
TI ARM CLANG            | R5F, M4F       | 1.3.0.LTS
GCC AARCH64             | A53            | 9.2-2019.12
GCC ARM                 | R5F            | 7-2017-q4-major  (AM64x only)
FreeRTOS Kernel         | R5F, M4F, A53  | 10.4.3
FreeRTOS SMP Kernel     | A53            | 202110.00-SMP
Tiny USB                | R5F            | 0.10.0
LwIP                    | R5F            | 2.12.2

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

\endcond

\cond SOC_AM243X

NONE

\endcond


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
FreeRTOS Kernel     | R5F, M4F, A53   | NA                | Task, Task notification, interrupts, semaphores, mutexs, timers, event groups. ROV views in CCS IDE, Task load measurement using FreeRTOS run time statistics APIs. | Only single core A53 FreeRTOS is supported. Second core is NOT used.
FreeRTOS SMP Kernel | A53             | NA                | Task, Task notification, interrupts, semaphores, mutexs, timers, event groups. ROV views in CCS IDE, Task load measurement using FreeRTOS run time statistics APIs. | -
FreeRTOS POSIX      | R5F, M4F, A53   | NA                | pthread, mqueue, semaphore, clock                                                                   | -
NO RTOS             | R5F, M4F, A53   | NA                | See **Driver Porting Layer (DPL)** below                                                            | Only single core A53 NORTOS is supported. Second core is NOT used.

### Driver Porting Layer (DPL)

Module            | Supported CPUs  | SysConfig Support | OS support       | Key features tested                                           | Key features not tested / NOT supported
------------------|-----------------|-------------------|------------------|---------------------------------------------------------------|----------------------------------------
Address Translate | M4F             | YES               | FreeRTOS, NORTOS | Use RAT to allow M4F access to peripheral address space       | -
Cache             | R5F, A53        | YES               | FreeRTOS, NORTOS | Cache write back, invalidate, enable/disable                  | -
Clock             | R5F, M4F, A53   | YES               | FreeRTOS, NORTOS | Tick timer at user specified resolution, timeouts and delays  | -
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

Peripheral | Supported CPUs | SysConfig Support | Key features tested                                                          | Key features not tested / NOT supported
-----------|----------------|-------------------|------------------------------------------------------------------------------|----------------------------------------
ADC        | R5F            | YES               | Single conversion (one-shot mode), interrupt mode, DMA mode                  | Continuous conversion not tested
CRC        | R5F            | YES               | CRC in full CPU mode                                                         | -
DDR        | R5F            | YES               | Tested LPDDR4 at 400MHz frequency.                                           | -
ECAP       | R5F            | YES               | Frequency, Duty cycle, interrupt mode                                        | PWM mode not tested
EPWM       | R5F            | YES               | Different Frequency, Duty cycle, interrupt mode, Deadband and chopper module | Tripzone module not tested
EQEP       | R5F            | YES               | Signal Frequency and Direction, interrupt mode                               | -
FSI (RX/TX)| R5F            | YES               | RX, TX, polling, interrupt mode, single/dual lanes                           | -
GPIO       | R5F, M4F, A53  | YES               | Basic input/output, GPIO as interrupt                                        | GPIO as interrupt is not tested for A53.
GTC        | R5F, A53       | NA                | Enable GTC, setting FID (Frequency indicator)                                | -
I2C        | R5F, M4F, A53  | YES               | Controller mode, basic read/write, polling and interrupt mode                    | Target mode not supported. M4F not tested due to EVM limitation
IPC Notify | R5F, M4F, A53  | YES               | Low latency IPC between RTOS/NORTOS CPUs                                     | -
IPC Rpmsg  | R5F, M4F, A53  | YES               | RPMessage protocol based IPC for all R5F, M4F, A53 running NORTOS/FreeRTOS/Linux | -
MCAN       | R5F            | YES               | RX, TX, interrupt and polling mode                                           | -
MCSPI      | R5F, M4F       | YES               | Controller/Peripheral mode, basic read/write, polling, interrupt and DMA mode         | -
MDIO       | R5F            | NA                | Register read/write, link status and link interrupt enable API               | -
MMCSD      | R5F            | YES               | Raw read/write and file I/O on MMCSD0 eMMC, and MMCSD1 SD. eMMC tested till HS SDR mode (8-bit data, 52 MHz), SD tested till SD HS mode (4-bit, 25 MHz)  | Interrupt mode not tested
OSPI       | R5F            | YES               | Read direct, Write indirect, Read/Write commands, DMA for read, PHY Mode     | Interrupt mode not supported
Pinmux     | R5F, M4F, A53  | YES               | Tested with multiple peripheral pinmuxes                                     | -
PRUICSS    | R5F            | YES               | Tested with Ethercat, EtherNet/IP, IO-Link, ICSS-EMAC, HDSL, EnDat           | -
SOC        | R5F, M4F, A53  | YES               | lock/unlock MMRs, get CPU clock, CPU name, clock enable, set frequency       | -
Sciclient  | R5F, M4F, A53  | YES               | Tested with clock setup, module on/off                                       | -
SPINLOCK   | R5F, M4F, A53  | NA                | Lock, unlock HW spinlocks                                                    | -
UART       | R5F, M4F, A53  | YES               | Basic read/write, polling, interrupt mode,                                   | HW flow control not tested. DMA mode not supported
UDMA       | R5F, A53       | YES               | Basic memory copy, SW trigger, Chaining                                      | -
WDT        | R5F, A53       | YES               | Interrupt after watchdog expiry                                              | Reset not supported

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

Module          | Supported CPUs    | SysConfig Support | OS Support        | Key features tested                                      | Key features not tested
----------------|-------------------|-------------------|-------------------|----------------------------------------------------------|-------------------------------------------------
ADC             | R5F               | YES               | FREERTOS          | Parallel 8 and 16 Bit Interface with ADS8598H IC         | OSR Modes, Other ICs

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
    <td> MCUSDK-2131
    <td> EtherCAT SubDevice increments long frame errors for frames with size less than 2000 bytes
    <td> EtherCAT SubDevice FWHAL
    <td> 7.3.0 onwards
    <td> AM64x, AM243x
    <td> Fixed
</tr>
<tr>
    <td> MCUSDK-2202
    <td> DDR: High bandwidth data traffic at certain burst patterns causes faiures.
    <td> High bandwidth data traffic at certain burst patterns on the DDR bus cause resonance frequencies on the DDR IO voltage rail.  This in turn affects the DDR address bus and causes the failures.  The main fix in the updated configurations is enabling write DBI on LPDDR4, and read DBI on DDR4.  This helps to quite the data transitions on the DDR bus and greatly reduces the impact on the DDR IO voltage rail.
    <td> 8.1.0 onwards
    <td> AM64x, AM243x
    <td> Fixed.
</tr>
<tr>
    <td> MCUSDK-2261
    <td> CCS Target Configuration from SDK examples does not work
    <td> CCS
    <td> 8.1.0
    <td> AM64x, AM243x
    <td> Added 1-click debug in the documentation which uses default target config itself. Updated documentation to use either target configuration for SBL, but for CCS load, you need to use "AM64x_GP_EVM/AM24x_GP_EVM"
</tr>
<tr>
    <td> MCUSDK-2301
    <td> Correct main task name from frertos_main to freertos_main
    <td> FREERTOS EXAMPLES
    <td> 7.3.0 onwards
    <td> AM64x, AM243x
    <td> Fixed
</tr>
<tr>
    <td> MCUSDK-2338
    <td> LwIP fails to build with <sys/select.h> header
    <td> LWIP
    <td> 8.1.0 onwards
    <td> AM64x
    <td> Fixed
</tr>
<tr>
    <td> MCUSDK-2344
    <td> MCU+ SDK: Removing mmcsd driver in sysconfig file leads to linker error
    <td> MMCSD
    <td> 8.1.0 onwards
    <td> AM64x, AM243x
    <td> Fixed. MMCSD Sysconfig provides an option of choosing 'NO DEVICE' if there is no device connected.
</tr>
<tr>
    <td> MCUSDK-2347
    <td> Enet: PORT_MODE_SEL in CTRLMMR_ENET1_CTRL can't be changed
    <td> ENET
    <td> 8.1.0 onwards
    <td> AM64x, AM243x
    <td> Incorrect offset for kick0 and kick1 registers.
</tr>
<tr>
    <td> MCUSDK-2413
    <td> Enet - build failure when LwIP stats is disabled
    <td> Enet
    <td> 8.1.0 onwards
    <td> AM64x, AM243x
    <td> Fixed
</tr>
<tr>
    <td> MCUSDK-2416
    <td> AM64X: Writing images to flash larger than 1024 KByte, uniflash finishes with errors
    <td> UART, SBL
    <td> 8.1.0 onwards
    <td> AM64x, AM243x
    <td> Fixed
</tr>
<tr>
    <td> MCUSDK-2458
    <td> AM243x EVM: the SBL_NULL does not initialize DDR
    <td> SBL
    <td> 8.1.0 onwards
    <td> AM243x
    <td> Fixed
</tr>
<tr>
    <td> MCUSDK-2469
    <td> AM243x: EPWM example for AM243x LP is using the wrong pin
    <td> EPWM
    <td> 8.1.0 onwards
    <td> AM64x, AM243x
    <td> Fixed
</tr>
<tr>
    <td> MCUSDK-2510
    <td> Enet - Loopback example not supported but packaged
    <td> Enet
    <td> 8.1.0 onwards
    <td> AM64x, AM243x
    <td> Fixed
</tr>
<tr>
    <td> MCUSDK-2528
    <td> EtherCAT : Latch pins are not working
    <td> EtherCAT SubDevice Beckhoff SSC Demo
    <td> 7.3.0 onwards
    <td> AM64x, AM243x
    <td> Timesync router configuration was required for these pins.
</tr>
<tr>
    <td> MCUSDK-2531
    <td> I2C module should be disabled before doing softreset in I2C_ctrlInit
    <td> I2C
    <td> 8.1.0 onwards
    <td> AM64x
    <td> Fixed
</tr>
<tr>
    <td> MCUSDK-3275
    <td> Enet - build error on Dev assert enable
    <td> Enet
    <td> 8.1.0 onwards
    <td> AM243x
    <td> Fixed
</tr>
<tr>
    <td> MCUSDK-3276
    <td> Enet - Missing extern "C" statement when using enet.h with C++
    <td> Enet
    <td> 8.1.0 onwards
    <td> AM243x
    <td> Fixed
</tr>
<tr>
    <td> MCUSDK-3616
    <td> CPSW: packet drops seen in ping and low iperf performance numbers on am64x-evm and am243x-evm
    <td> Enet
    <td> 8.1.0 onwards
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
    <td> MCUSDK-1922
    <td> SBL UART appimage size limitation of less then 384 KB
    <td> SBL
    <td> 7.3.0 onwards
    <td> AM64x, AM243x
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
    <td> MCUSDK-3453
    <td> UDMA unable to process icssg rx timestamp and some bytes of packet content after Allocated Packets(Wraparound)
    <td> Enet
    <td> 8.1.0 onwards
    <td> AM64x, AM243x
    <td> -
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
    <td> MCUSDK-3424
    <td> SBL OSPI fails to load image using OSPI DMA in HS device
    <td> SBL, OSPI, UDMA
    <td> 8.2.0 onwards
    <td> AM64x, AM243x
    <td> Use non DMA mode
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
    <td> MCUSDK-2753
    <td> SOC_controlModuleLockMMR() API doesn't lockCTRLMMR registers
    <td> SDK
    <td> 8.1.0 onwards
    <td> AM243x
    <td> -
</tr>
<tr>
    <td> MCUSDK-3513
    <td> SBL JTAG Example flash write fails if the file size is not aligned to multiple of flash page size
    <td> SBL JTAG
    <td> 8.2.0 onwards
    <td> AM64x, AM243x
    <td> Aligned file size to page size by padding 0's at the end
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
    <td> Release Build
    <td> -
    <td> Updated optimization level in release build to 'Os' as part of bug fix MCUSDK-1980.
    <td> The release build with ti-arm-clang compiler has been updated from 'O3' to 'Os' optimization level. It is recommended that user rebuilds the existing libraries and applications with 'Os' option. This can be done by updating CFLAGS_release in library/example makefiles.
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
    <td> DPL
    <td> -
    <td> Renamed hello world example, examples/kernel/dpl/hello_world to examples/kernel/dpl/dpl_demo
    <td> This example was testing more DPL features rather than simple "hello world"
</tr>
<tr>
    <td> UART
    <td> -
    <td> Added new parameter 'Operational Mode' in UART SYSCFG as part of bug fix MCUSDK-1898.
    <td> Also removed partial mode configuration in UART ECHO SYSCFG and updated example to receive 8 characters and then echoing back the same.
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
    <td> Timer
    <td> \ref TimerP_Params
    <td> Added new parameter \ref TimerP_Params::periodInNsec to allow to specify timer period in units of nsecs
    <td> No change needed in existing applications if \ref TimerP_Params_init is being used to init \ref TimerP_Params
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
    <td> MCSPI SYSCFG
    <td> -
    <td> Removed `dataSize`(Data Frame Size) parameter from MCSPI Channel Configuration in SYSCFG.
    <td> Any examples using "MCSPI" module in SysConfig will be affected. Any `.syscfg` file using this module, needs to be updated by first removing and then adding this module back using SysConfig GUI.
</tr>
<tr>
    <td> MCSPI
    <td> -
    <td> Added `dataSize` and `csDisable` are added in \ref MCSPI_Transaction structure.
    <td> User need to set these parameters before doing MCSPI transaction.
</tr>
<tr>
    <td> UART
    <td> -
    <td> Removed `intrEnable` and added `transferMode` as a parameter in UART_Params structure as part of DMA support.
    <td>
</tr>
</table>

