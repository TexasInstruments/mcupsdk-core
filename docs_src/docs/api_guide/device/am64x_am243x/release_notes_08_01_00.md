# Release Notes 08.01.00 {#RELEASE_NOTES_08_01_00_PAGE}

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
Feature                                                             | Module
--------------------------------------------------------------------|--------------------------
EtherCAT examples running from On-chip RAM                          | Industrial Communications Toolkit
EtherNet/IP Adapter examples with Device Level Ring Support         | Industrial Communications Toolkit
Profinet RT and IRT examples                                        | Industrial Communications Toolkit
SBL OSPI booting R5F, M4F and Linux on A53                          | Bootloader
IPC RPMsg with M4F                                                  | IPC
MMCSD Raw Write/Read support for MMCSD0, eMMC                       | MMCSD
Layer 2 Timesync example                                            | CPSW
Layer 2 Switch/Dual-MAC example                                     | ICSSG
VLAN,FDB conformance example                                        | ICSSG
Crypto SHA and AES examples                                         | CRYPTO
FreeRTOS+FAT file system integrated to use in a NO-RTOS environment | File System
MMCSD File I/O support for MMCSD0, eMMC using FreeRTOS+FAT          | MMCSD
SBL MMCSD enabled to boot from SD card using File I/O               | Bootloader
SBL eMMC booting R5F, M4F and Linux on A53                          | Bootloader
\endcond

\cond SOC_AM243X
Feature                                                             | Module
--------------------------------------------------------------------|--------------------------
EtherCAT examples running from On-chip RAM                          | Industrial Communications Toolkit
EtherNet/IP Adapter examples with Device Level Ring Support         | Industrial Communications Toolkit
Profinet RT and IRT examples                                        | Industrial Communications Toolkit
IO-Link example on AM243x LAUNCHPAD                                 | Industrial Communications Toolkit
Layer 2 Timesync example                                            | CPSW
Layer 2 Switch/Dual-MAC example                                     | ICSSG
VLAN,FDB conformance example                                        | ICSSG
Crypto SHA and AES examples                                         | CRYPTO
MMCSD Raw Write/Read support for MMCSD0, eMMC                       | MMCSD
FreeRTOS+FAT file system integrated to use in a NO-RTOS environment | File System
MMCSD File I/O support for MMCSD0, eMMC using FreeRTOS+FAT          | MMCSD
SBL MMCSD enabled to boot from SD card using File I/O               | Bootloader
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
Code Composer Studio    | R5F, M4F, A53  | @VAR_CCS_VERSION
SysConfig               | R5F, M4F, A53  | @VAR_SYSCFG_VERSION, build @VAR_SYSCFG_BUILD
TI ARM CLANG            | R5F, M4F       | @VAR_TI_ARM_CLANG_VERSION
GCC AARCH64             | A53            | @VAR_GCC_AARCH64_VERSION
GCC ARM                 | R5F            | @VAR_GCC_ARM_VERSION (AM64x only)
FreeRTOS Kernel         | R5F, M4F       | @VAR_FREERTOS_KERNEL_VERSION
Tiny USB                | R5F            | @VAR_TINYUSB_VERSION
LwIP                    | R5F            | @VAR_LWIP_VERSION
Security mbedtls        | R5F            | @VAR_SECURITY_MBEDTLS_VERSION

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
A53 FreeRTOS (single core) support and A53 FreeRTOS examples        | DPL, FreeRTOS
SBL booting A53 NORTOS                                              | Bootloader
GCC support for R5F for limited examples                            | R5F

\note A53 FreeRTOS on dual core A53 in SMP mode is not supported

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

OS              | Supported CPUs  | SysConfig Support | Key features tested                                                                                 | Key features not tested / NOT supported
----------------|-----------------|-------------------|-----------------------------------------------------------------------------------------------------|----------------------------------------
FreeRTOS Kernel | R5F, M4F, A53   | NA                | Task, Task notification, interrupts, semaphores, mutexs, timers, event groups. ROV views in CCS IDE, Task load measurement using FreeRTOS run time statistics APIs. | Only single core A53 FreeRTOS is supported. Second core is NOT used.
FreeRTOS POSIX  | R5F, M4F, A53   | NA                | pthread, mqueue, semaphore, clock                                                                   | -
NO RTOS         | R5F, M4F, A53   | NA                | See **Driver Porting Layer (DPL)** below                                                            | Only single core A53 NORTOS is supported. Second core is NOT used.

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
MCSPI      | R5F, M4F       | YES               | Controller/Peripheral mode, basic read/write, polling, interrupt mode                 | DMA mode not supported
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
EtherCAT SubDevice Evaluation Stack       | R5F            | NO                | FreeRTOS    | Tested with examples from `examples\industrial_comms\ethercat_slave_demo` folder                         | -
EtherNet/IP Adapter FWHAL             | R5F            | NO                | FreeRTOS    | Tested with examples from `examples\industrial_comms\ethernetip_adapter_demo` folder                     | Multicast Filtering
EtherNet/IP Adapter Evaluation Stack  | R5F            | NO                | FreeRTOS    | Tested with examples from `examples\industrial_comms\ethernetip_adapter_demo` folder                     | -
IO-Link Controller Evaluation Stack       | R5F            | NO                | FreeRTOS    | Tested with iolink_master_demo example                                                                   | -
Profinet Device FWHAL                 | R5F            | NO                | FreeRTOS    | Tested with examples from `examples\industrial_comms\profinet_device_demo` folder                        | MRP
Profinet Device Evaluation Stack      | R5F            | NO                | FreeRTOS    | Tested with examples from `examples\industrial_comms\profinet_device_demo` folder                        | -

### Motor Control

Module                      | Supported CPUs | SysConfig Support | OS Support        | Key features tested                             | Key features not tested
----------------------------|----------------|-------------------|-------------------|-------------------------------------------------|------------------------
Position Sense HDSL         | R5F            | YES               | FreeRTOS, NORTOS  | Freerun mode, Sync mode                         |  -
Position Sense EnDAT        | R5F            | YES               | FreeRTOS, NORTOS  | Single channel, Multi channel, Continuous mode  |  -

### Networking

Module                      | Supported CPUs | SysConfig Support | OS Support  | Key features tested                                                                    | Key features not tested
----------------------------|----------------|-------------------|-------------|----------------------------------------------------------------------------------------|------------------------
LwIP                        | R5F            | NO                | FreeRTOS    | TCP/UDP IP networking stack, DHCP, ping, TCP iperf, TCP/UDP IP                         | Other LwIP features, performance and memory optimizations pending, more robustness tests pending
Ethernet driver (ENET)      | R5F            | NO                | FreeRTOS    | Ethernet as port using CPSW and ICSS                                                   | Ethernet as switch
ICSS-EMAC                   | R5F            | YES               | FreeRTOS    | Tested switch mode with ethernetip_adapter_mii and ethernetip_adapter_rgmii examples   | EMAC mode, VLAN/Multicast Filtering
ICSS TimeSync               | R5F            | NO                | FreeRTOS    | Tested with examples from `examples\industrial_comms\ethernetip_adapter_demo` folder   | -

### USB

Module                      | Supported CPUs | SysConfig Support | OS Support         | Key features tested       | Key features not tested
----------------------------|----------------|-------------------|--------------------|---------------------------|------------------------
USB SoC Porting Layer       | R5F            | YES               | FreeRTOS, NORTOS   | USB 2.0 device mode       | USB 3.0
USB Device Driver           | R5F            | NO                | FreeRTOS, NORTOS   | USB 2.0 device mode       | USB Host driver
TinyUSB Core and CDC Driver | R5F            | NO                | FreeRTOS, NORTOS   | USB device with CDC class | USB Host, other USB device class drivers

### SECURITY

Module                      | Supported CPUs | SysConfig Support | OS Support         | Key features tested      | Key features not tested
----------------------------|----------------|-------------------|--------------------|---------------------------|---------------------------
Crypto AES SW               | R5F            | YES               | NORTOS   | AES CBC-128 encryption and decryption, AES CBC-192 encryption and decryption, AES CBC-256 encryption and decryption, AES CMAC-256 single shot and multi-shot, AES CMAC-128 single shot and multi-shot, AES CMAC-192 single shot and multi-shot     | -
Crypto SHA SW               | R5F            | Yes               | NORTOS   | SHA 512 single shot and multi-shot, SHA 256 single shot and multi-shot, HMAC SHA-256, HMAC SHA-512, HMAC SHA-1,   |-
SA2UL RNG                   | R5F            | Yes               | NORTOS   | SA2UL Random number generator       |-
Crypto SHA HW               | R5F            | Yes               | NORTOS   | SA2UL Crypto single-shot SHA 512 and 256  |-

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
    <td> MCUSDK-750
    <td> Not all pins are supported by SYSCFG. Leads to manual pinmux of ICSSG PRU pins.
    <td> ICSS
    <td> 7.3.0 onwards
    <td> AM64x, AM243x
    <td> Fixed
 </tr>
 <tr>
    <td> MCUSDK-1564
    <td> time.h APIs do not work as expected with SBL OSPI
    <td> SBL, OSPI
    <td> 7.3.0 onwards
    <td> AM64x, AM243x
    <td> When the time.h functions are called from a CCS environment, the implementation uses queries the host PC to get the time. Hence this does not work with a non-CCS (SBL) environment. Clock APIs like \ref ClockP_getTimeUsec should be used instead.
 </tr>
<tr>
    <td> MCUSDK-1598
    <td> LWIP: UDP iperf test results in assert due to memory alloc failure
    <td> LWIP
    <td> 8.0 onwards
    <td> AM64x, AM243x
    <td> NONE
</tr>
<tr>
    <td> MCUSDK-1599
    <td> LWIP: LwIP Stack diagnostics prints like assert and printf are not redirected to DebugP_log, i.e UART terminal
    <td> LWIP
    <td> 8.0 onwards
    <td> AM64x, AM243x
    <td> look at CCS console for these logs
</tr>
<tr>
    <td> MCUSDK-1620
    <td> A53 NORTOS: A53 Interrupts doesn't work with CCS Reset and Reload
    <td> DPL
    <td> 8.1 onwards
    <td> AM64x
    <td> Power cycle the EVM before reloading application to A53
</tr>
<tr>
    <td> MCUSDK-1622
    <td> ClockP_start is not allowed from ISR context
    <td> DPL
    <td> 7.3.0 onwards
    <td> AM64x, AM243x
    <td> Fixed
</tr>
<tr>
    <td> MCUSDK-1623
    <td> A53 NORTOS: Application halts when using Console logs without debug port connected
    <td> DPL
    <td> 8.1 onwards
    <td> AM64x
    <td> Use UART logs when using booting through SBL
</tr>
<tr>
    <td> MCUSDK-1629
    <td> Timer configuration is not allowed in units of nsecs and does not allow to select different clock source
    <td> DPL / Timer
    <td> 7.3.0 onwards
    <td> AM64x, AM243x
    <td> Added a new parameter to allow set timer period in units of nsecs and updated SysConfig to allow selecting different timer clock sources
</tr>
<tr>
    <td> MCUSDK-1637
    <td> EtherNet/IP DLR Port 1 Interrupt comes multiple times
    <td> EtherNet/IP Adapter FWHAL
    <td> 7.3.0 onwards
    <td> AM64x, AM243x
    <td> Fixed an offset calculation in PORT 1 ISR
</tr>
<tr>
    <td> MCUSDK-1646
    <td> 16 bits accessed instead of 8 bits while checking for busy_s bit of queue in ICSS EMAC
    <td> ICSS-EMAC
    <td> 7.3.0 onwards
    <td> AM64x, AM243x
    <td> Changed HW_WR_REG16 to HW_WR_REG8
</tr>
<tr>
    <td> MCUSDK-1651
    <td> In EtherNet/IP DLR, Neighbour Check Requests are sent 4 times instead of 3
    <td> EtherNet/IP Adapter FWHAL
    <td> 7.3.0 onwards
    <td> AM64x, AM243x
    <td> Fixed
</tr>
<tr>
    <td> MCUSDK-1652
    <td> Data abort is seen in TimeSync_drvInit function of ICSS TimeSync driver
    <td> ICSS TimeSync
    <td> 7.3.0 onwards
    <td> AM64x, AM243x
    <td> Add "-mno-unaligned-access" flag for library compilation
</tr>
<tr>
    <td> MCUSDK-1736
    <td> TimeSync_drvDisable API does not work everytime
    <td> ICSS TimeSync
    <td> 7.3.0 onwards
    <td> AM64x, AM243x
    <td> NONE
</tr>
<tr>
    <td> MCUSDK-1737
    <td> ICSS TimeSync driver tries to create a Hwi with priority 20
    <td> ICSS TimeSync
    <td> 7.3.0 onwards
    <td> AM64x, AM243x
    <td> Changed priority to 15
</tr>
<tr>
    <td> MCUSDK-1743
    <td> [EQEP]Incorrect Maximum Limit Check for Pulse Width Period Cycles
    <td> EQEP
    <td> 7.3.0 onwards
    <td> AM64x, AM243x
    <td> Fixed
</tr>
<tr>
    <td> MCUSDK-1755
    <td> Interrupt callback does not happen for CPSW
    <td> CPSW
    <td> 7.3.0 onwards
    <td> AM64x, AM243x
    <td> Fixed
</tr>
<tr>
    <td> MCUSDK-1758
    <td> [UART]Read Return Partial Mode is not functional
    <td> UART
    <td> 7.3.0 onwards
    <td> AM64x, AM243x
    <td> Fixed
</tr>
<tr>
    <td> MCUSDK-1814
    <td> [MCSPI]Transfer Timeout feature is not functional
    <td> McSPI
    <td> 7.3.0 onwards
    <td> AM64x, AM243x
    <td> Fixed
</tr>
<tr>
    <td> MCUSDK-1815
    <td> [MCSPI]Multi Channel Transfer, Results in Failure
    <td> McSPI
    <td> 7.3.0 onwards
    <td> AM64x, AM243x
    <td> Enabled FIFO for all channels in case of Multi Channel Transfer.
</tr>
<tr>
    <td> MCUSDK-1824
    <td> A53 DPL: malloc not working with GCC AArch64 compiler
    <td> DPL
    <td> 8.0 onwards
    <td> AM64x
    <td> Linker command file updates to add symbols to mark heap start and heap end
</tr>
<tr>
    <td> MCUSDK-1830
    <td> [MCSPI]Incorrect channel configuration in Peripheral mode, Results in Failure
    <td> McSPI
    <td> 7.3.0 onwards
    <td> AM64x, AM243x
    <td> Fixed
</tr>
<tr>
    <td> MCUSDK-1898
    <td> UART Read fails at baud rate greater than 230.4kbps
    <td> UART
    <td> 7.3.0 onwards
    <td> AM64x, AM243x
    <td> Updated driver and SYSCFG to support baudrate greater than 230.4kbps.
</tr>
<tr>
    <td> MCUSDK-1907
    <td> [MCSPI]Channel number configuration is not 0 in peripheral mode
    <td> McSPI
    <td> 7.3.0 onwards
    <td> AM64x, AM243x
    <td> SYSCFG updates to configure channel number always to 0 in PERIPHERAL mode.
</tr>
<tr>
    <td> MCUSDK-2082
    <td> [MCSPI]Controller RX Only mode, Peripheral TX Only Mode transfer is not functional
    <td> McSPI
    <td> 7.3.0 onwards
    <td> AM64x, AM243x
    <td> Fixed
</tr>
<tr>
    <td> MCUSDK-1954
    <td> PHY delay values for RX and TX are incorrect in Profinet Device FWHAL
    <td> Profinet Device FWHAL
    <td> 7.3.0 onwards
    <td> AM64x, AM243x
    <td> Fixed
</tr>
<tr>
    <td> MCUSDK-1980
    <td> Change optimization level for ti-arm-clang from O3 to Os
    <td> Build
    <td> 7.3.0 onwards
    <td> AM64x, AM243x
    <td> Fixed
</tr>
<tr>
    <td> MCUSDK-2060
    <td> Need CMSIS example and benchmark data
    <td> Bench Mark Demo
    <td> 7.3.0 onwards
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
    <td> MCUSDK-1572
    <td> OSPI: Flash IO and Flash DMA examples fail for AM243x-LP when built from CCS in release mode
    <td> OSPI
    <td> 7.3.0 onwards
    <td> AM243x
    <td> i. Build via makefile <br>ii.Try the CCS build in debug mode <br>iii.Disable phy mode if performance is not concern <br>iv. Change memcpy version to slow version in CCS Project properties : <br>Properties->Build->Arm linker->Advanced options->Link-time optimization->choose "small" option for using memcpy slow version.
</tr>
<tr>
    <td> MCUSDK-2131
    <td> EtherCAT SubDevice increments long frame errors for frames with size less than 2000 bytes
    <td> EtherCAT SubDevice FWHAL
    <td> 7.3.0 onwards
    <td> AM64x, AM243x
    <td> -
</tr>
<tr>
    <td> MCUSDK-2135
    <td> Insufficient UDMA TX channels allocated for in Dual-mac mode
    <td> Enet
    <td> 8.1.0 onwards
    <td> AM64x, AM243x
    <td> Allocate minimum of 2 TX channels(ENET_CFG_TX_CHANNELS_NUM macro) in enet_cfg.h file
</tr>
<tr>
    <td> MCUSDK-2171
    <td> SBL OSPI, SBL Linux (OSPI and eMMC) pauses in between the SBL profiling logs when UART is in interrupt mode. The issue is observed in debug build only.
    <td> SBL, UART
    <td> 8.1.0 onwards
    <td> AM64x, AM243x
    <td> Change the UART mode to polling mode
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
    <td> UART
    <td> -
    <td> Added new parameter 'Operational Mode' in UART SYSCFG as part of bug fix MCUSDK-1898.
    <td> Also removed partial mode configuration in UART ECHO SYSCFG and updated example to receive 8 characters and then echoing back the same.
</tr>
<tr>
    <td> Industrial Communications Toolkit
    <td> -
    <td> EtherCAT, EtherNet/IP and IO-Link examples are moved from `${SDK_INSTALL_PATH}/examples/industrial_protocols` to `${SDK_INSTALL_PATH}/examples/industrial_comms`
    <td> -
</tr>
<tr>
    <td> Industrial Communications Toolkit
    <td> -
    <td> Profinet Device RT examples are moved from `${SDK_INSTALL_PATH}/examples/industrial_comms/profinet_device_rt_demo` to `${SDK_INSTALL_PATH}/examples/profinet_device_demo`
    <td> -
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
    <td> PRUICSS
    <td> -
    <td> Files for PRUICSS are moved from `${SDK_INSTALL_PATH}/source/drivers/pruicss/v0` to `${SDK_INSTALL_PATH}/source/drivers/pruicss/g_v0`
    <td> -
</tr>
<tr>
    <td> PRUICSS
    <td> \ref PRUICSS_setIcssCfgMiiMode and \ref PRUICSS_setIcssCfgTxFifo
    <td> These APIs were renamed. Old names were `PRUICSS_setIcssgCfgMiiMode` and `PRUICSS_setIcssgCfgTxFifo`.
    <td> -
</tr>
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
    <td> EtherCAT SubDevice FWHAL and stack, EtherNet/IP Adapter FWHAL and stack, IO-Link stack and Profinet Device FWHAL
    <td> -
    <td> Sources are moved from `${SDK_INSTALL_PATH}/source/industrial_protocols` to `${SDK_INSTALL_PATH}/source/industrial_comms`
    <td> -
</tr>
<tr>
    <td> SysConfig
    <td> -
    <td> Removed the module "Ethernet (ICSS)" and added "EtherNet/IP" and "Profinet" modules in SysConfig. Also removed inclusion of pins which are either not connected appropriately on the board or not required.
    <td> Any examples using "Ethernet (ICSS)" module in SysConfig will be affected. Any `.syscfg` file using this module, needs to be updated by removing lines related to "Ethernet (ICSS)" module using text editor and adding "EtherNet/IP" or "Profinet" module using SysConfig GUI.
</tr>
<tr>
    <td> SysConfig
    <td> -
    <td> In "EtherCAT" module, removed inclusion of pins which are either not connected appropriately on the board or not required.
    <td> Any examples using "EtherCAT" module in SysConfig will be affected. Any `.syscfg` file using this module, needs to be updated by first removing and then adding this module back using SysConfig GUI.
</tr>
<tr>
    <td> EtherNet/IP Adapter FWHAL
    <td> \ref EIP_drvInit
    <td> Added `pruicssHandle` in \ref EIP_Handle structure
    <td> `pruicssHandle` should be set by user before calling \ref EIP_drvInit API.
</tr>
<tr>
    <td> Profinet Device FWHAL
    <td> \ref PN_initDrv
    <td> Added `latchIntConfig` in \ref PN_IntConfig structure
    <td> `latchIntConfig` should be set by user before calling \ref PN_initDrv API, if ENABLE_LATCH_SUPPORT macro is enabled in `${SDK_INSTALL_PATH}/industrial_comms/profinet_device/icss_fwhal/IRT/pnDrvConfig.h` or `${SDK_INSTALL_PATH}/industrial_comms/profinet_device/icss_fwhal/RT_MRP/pnDrvConfig.h`.
</tr>
<tr>
    <td> ICSS TimeSync
    <td> `TimeSync_reset`
    <td> Added `ptpDrvStackReset` in `TimeSync_Config_t` structure
    <td> `ptpDrvStackReset` callback function should be provided by the user if a PTP stack reset callback is needed inside `TimeSync_reset`.
</tr>
</table>