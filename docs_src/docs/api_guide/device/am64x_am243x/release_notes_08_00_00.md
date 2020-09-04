# Release Notes 08.00.00 {#RELEASE_NOTES_08_00_00_PAGE}

[TOC]

\attention Also refer to individual module pages for more details on each feature, unsupported features, important usage guidelines.

\note The examples will show usage of SW modules and APIs on a specific CPU instance and OS combination. \n
      Unless noted otherwise, the SW modules would work in both FreeRTOS and no-RTOS environment. \n
      Unless noted otherwise, the SW modules would work on any of the R5F's present on the SOC. \n
      Unless noted otherwise, the SW modules would work on all supported EVMs \n
      All SW modules will not work on M4F. M4F support for a module will be explicitly called out.

## New in this Release

Feature                                                             | Module
--------------------------------------------------------------------|--------------------------
EtherCAT, EtherNet/IP and IO-Link examples                          | Industrial Protocols
LwIP with ethernet driver (ENET) for CPSW and ICSS (alpha quality)  | LwIP, ENET
Flash discovery using SFDP via OSPI diagnostic example              | OSPI, Flash
CRC example support                                                 | CRC
McSPI 8-bit data transfer performance example                       | McSPI
FreeRTOS task and CPU load measurement                              | DPL, FreeRTOS

## Device and Validation Information

\cond SOC_AM64X
SOC   | Supported CPUs  | EVM                                             | Host PC
------|-----------------|-------------------------------------------------|-----------------------------------
AM64x | R5F, M4F        | AM64x GP EVM (referred to as am64x-evm in code) | Windows 10 64b or Ubuntu 18.04 64b
\endcond

\cond SOC_AM243X
SOC    | Supported CPUs  | Boards                                                                                                      | Host PC
-------|-----------------|-------------------------------------------------------------------------------------------------------------|-----------------------------------
AM243x | R5F, M4F        | AM243x GP EVM (referred to as am243x-evm in code), \n AM243x LAUNCHPAD (referred to as am243x-lp in code)   | Windows 10 64b or Ubuntu 18.04 64b
\endcond

## Tools, Compiler and Other Open Source SW Module Information

Tools / SW module       | Supported CPUs | Version
------------------------|----------------|-----------------------
Code Composer Studio    | R5F, M4F       | @VAR_CCS_VERSION
SysConfig               | R5F, M4F       | @VAR_SYSCFG_VERSION, build @VAR_SYSCFG_BUILD
TI ARM CLANG            | R5F, M4F       | @VAR_TI_ARM_CLANG_VERSION
FreeRTOS Kernel         | R5F, M4F       | @VAR_FREERTOS_KERNEL_VERSION
Tiny USB                | R5F            | @VAR_TINYUSB_VERSION
LwIP                    | R5F            | @VAR_LWIP_VERSION

## Key Features

\cond SOC_AM243X
### AM243X LAUNCHPAD not tested/not supported features
- LWIP + Ethernet examples not validated in this release. This support will be added in a future release.
- DDR is not supported on the AM243X 11x11 SOC used in AM243X LAUNCHPAD.
- Motor control examples not validated, due to board limitation
- I2C temperature sensor example not validated, due to board limitation.
- M4F examples for UART, MCSPI and GPIO not validated, due to board limitation.
\endcond

### OS Kernel

OS              | Supported CPUs  | SysConfig Support | Key features tested                                                                                 | Key features not tested / NOT supported
----------------|-----------------|-------------------|-----------------------------------------------------------------------------------------------------|----------------------------------------
FreeRTOS Kernel | R5F, M4F        | NA                | Task, Task notification, interrupts, semaphores, mutexs, timers, event groups. ROV views in CCS IDE, Task load measurement using FreeRTOS run time statistics APIs. | -
FreeRTOS POSIX  | R5F, M4F        | NA                | pthread, mqueue, semaphore, clock                                                                   | -
NO RTOS         | R5F, M4F        | NA                | See **Driver Porting Layer (DPL)** below                                                            | -

### Driver Porting Layer (DPL)

Module            | Supported CPUs  | SysConfig Support | OS support       | Key features tested                                           | Key features not tested / NOT supported
------------------|-----------------|-------------------|------------------|---------------------------------------------------------------|----------------------------------------
Address Translate | M4F             | YES               | FreeRTOS, NORTOS | Use RAT to allow M4F access to peripheral address space       | -
Cache             | R5F             | YES               | FreeRTOS, NORTOS | Cache write back, invalidate, enable/disable                  | -
Clock             | R5F, M4F        | YES               | FreeRTOS, NORTOS | Tick timer at user specified resolution, timeouts and delays  | -
CycleCounter      | R5F, M4F        | NA                | FreeRTOS, NORTOS | Measure CPU cycles using CPU specific internal counters       | -
Debug             | R5F, M4F        | YES               | FreeRTOS, NORTOS | Logging and assert to any combo of: UART, CCS, shared memory  | -
Heap              | R5F, M4F        | NA                | FreeRTOS, NORTOS | Create arbitrary heaps in user defined memory segments        | -
Hwi               | R5F, M4F        | YES               | FreeRTOS, NORTOS | Interrupt register, enable/disable/restore                    | -
MPU               | R5F, M4F        | YES               | FreeRTOS, NORTOS | Setup MPU and control access to address space                 | -
Semaphore         | R5F, M4F        | NA                | FreeRTOS, NORTOS | Binary, Counting Semaphore, recursive mutexs with timeout     | -
Task              | R5F, M4F        | NA                | FreeRTOS         | Create, delete tasks                                          | -
Timer             | R5F, M4F        | YES               | FreeRTOS, NORTOS | Configure arbitrary timers                                    | -
Event             | R5F, M4F        | YES               | FreeRTOS         | Setting, getting, clearing, and waiting of Event bits         | -

### Secondary Bootloader (SBL)

Module     | Supported CPUs  | SysConfig Support | OS support       | Key features tested                                                                             | Key features not tested / NOT supported
-----------|-----------------|-------------------|------------------|-------------------------------------------------------------------------------------------------|----------------------------------------
Bootloader | R5FSS0-0        | YES               | NORTOS           | Boot modes: OSPI, OSPI XIP, UART. All R5F's, M4F boot. RPRC, multi-core image format, DDR init  | SBL booting Linux. SBL booting A53.

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
GPIO       | R5F, M4F       | YES               | Basic input/output, GPIO as interrupt                                        | -
I2C        | R5F, M4F       | YES               | Controller mode, basic read/write, polling and interrupt mode                    | Target mode not supported. M4F not tested due to EVM limitation
IPC Notify | R5F, M4F       | YES               | Low latency IPC between RTOS/NORTOS CPUs                                     | -
IPC Rpmsg  | R5F            | YES               | RPMessage protocol based IPC, All R5F and Linux A53 cores                    | M4F core not supported
MCAN       | R5F            | YES               | RX, TX, interrupt and polling mode                                           | -
MCSPI      | R5F, M4F       | YES               | Controller/Peripheral mode, basic read/write, polling, interrupt mode                 | DMA mode not supported
MDIO       | R5F            | NA                | Register read/write, link status and link interrupt enable API               | -
OSPI       | R5F            | YES               | Read direct, Write indirect, Read/Write commands, DMA for read, PHY Mode     | Interrupt mode not supported
Pinmux     | R5F, M4F       | YES               | Tested with multiple peripheral pinmuxes                                     | -
PRUICSS    | R5F            | YES               | Tested with Ethercat, EtherNet/IP, IO-Link, ICSS-EMAC, HDSL, EnDat           | More protocols integration tests pending
SOC        | R5F, M4F       | YES               | lock/unlock MMRs, get CPU clock, CPU name, clock enable, set frequency       | -
Sciclient  | R5F, M4F       | YES               | Tested with clock setup, module on/off                                       | -
SPINLOCK   | R5F, M4F       | NA                | Lock, unlock HW spinlocks                                                    | -
UART       | R5F, M4F       | YES               | Basic read/write, polling, interrupt mode,                                   | HW flow control not tested. DMA mode not supported
UDMA       | R5F            | YES               | Basic memory copy, SW trigger, Chaining                                      | -

### Board Device Drivers

Peripheral | Supported CPUs | SysConfig Support | Key features tested                                                                           | Key features not tested
-----------|----------------|-------------------|-----------------------------------------------------------------------------------------------|------------------------
EEPROM     | R5F            | YES               | I2C based EEPROM                                                                              | -
ETHPHY     | R5F            | YES               | Ethernet Phy configuration for EtherCAT SubDevice example                                         | -
Flash      | R5F            | YES               | XSPI, OSPI, QSPI based flash, Octal, Quad mode, DDR mode                                      | All vendor flash types not tested
LED        | R5F            | YES               | GPIO , I2C IO expander based LED control, I2C based industrial LEDs(TPIC2810)                 | -

### CMSIS

Module                      | Supported CPUs | SysConfig Support | OS Support        | Key features tested                                                                         | Key features not tested
----------------------------|----------------|-------------------|-------------------|---------------------------------------------------------------------------------------------|------------------------
CMSIS DSP                   | R5F            | NA                | FreeRTOS, NORTOS  | Basic math, complex math, controller, fast math, filtering, Matrix, statistics, transform   | -

### Industrial Protocols

Module                                | Supported CPUs | SysConfig Support | OS Support  | Key features tested                                                                                      | Key features not tested
--------------------------------------|----------------|-------------------|-------------|----------------------------------------------------------------------------------------------------------|------------------------
EtherCAT SubDevice FWHAL                  | R5F            | NO                | FreeRTOS    | Tested with ethercat_slave_beckhoff_ssc_demo example                                                     | Reset isolation
EtherCAT SubDevice Evaluation Stack       | R5F            | NO                | FreeRTOS    | Tested with ethercat_slave_simple_demo, ethercat_slave_cia402_demo, and ethercat_slave_ctt_demo examples | -
EtherNet/IP Adapter FWHAL             | R5F            | NO                | FreeRTOS    | Tested with ethernetip_adapter_mii_demo and ethernetip_adapter_rgmii_demo examples                       | DLR, PTP, Multicast Filtering
EtherNet/IP Adapter Evaluation Stack  | R5F            | NO                | FreeRTOS    | Tested with ethernetip_adapter_mii_demo and ethernetip_adapter_rgmii_demo examples                       | -
IO-Link Controller Evaluation Stack       | R5F            | NO                | FreeRTOS    | Tested with iolink_master_demo example                                                                   | -
Profinet Device FWHAL                 | R5F            | NO                | FreeRTOS    | Only compiled                                                                                            | Not tested

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
ICSS TimeSync               | R5F            | NO                | FreeRTOS    | Only compiled, not tested                                                               | Not tested

### USB

Module                      | Supported CPUs | SysConfig Support | OS Support         | Key features tested       | Key features not tested
----------------------------|----------------|-------------------|--------------------|---------------------------|------------------------
USB SoC Porting Layer       | R5F            | YES               | FreeRTOS, NORTOS   | USB 2.0 device mode       | USB 3.0
USB Device Driver           | R5F            | NO                | FreeRTOS, NORTOS   | USB 2.0 device mode       | USB Host driver
TinyUSB Core and CDC Driver | R5F            | NO                | FreeRTOS, NORTOS   | USB device with CDC class | USB Host, other USB device class drivers

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
    <td> MCUSDK-906
    <td> MCAN and ADC not functional in SBL UART bootmode with DDR enabled
    <td> Bootloader, DDR
    <td> 7.3.2
    <td> AM64x, AM243x
    <td> Fixed
</tr>
<tr>
    <td> MCUSDK-867
    <td> Rx is not selected in the pins for EtherCAT and Ethernet pinmux from SysConfig
    <td> EtherCAT SubDevice FWHAL, ICSS-EMAC
    <td> 7.3.0, 7.3.1
    <td> AM64x, AM243x
    <td> Fixed
</tr>
<tr>
    <td> MCUSDK-937
    <td> 16 bits accessed for 8 bit value of collision queue status in ICSS EMAC
    <td> ICSS-EMAC
    <td> 7.3.0, 7.3.1
    <td> AM64x, AM243x
    <td> Fixed
</tr>
<tr>
    <td> MCUSDK-1278
    <td> PRUICSS_clearEvent function should not be RMW
    <td> PRUICSS
    <td> 7.3.0, 7.3.1
    <td> AM64x, AM243x
    <td> Fixed
</tr>
<tr>
    <td> MCUSDK-317
    <td> OSPI: DMA reads fails after a flash write which was not block aligned
    <td> OSPI
    <td> 7.3.0 onwards
    <td> AM64x, AM243x
    <td> Fixed
</tr>
<tr>
    <td> MCUSDK-683
    <td> UDMA: DMA from SRAM to M4F IRAM/DRAM fails
    <td> UDMA, TIFS
    <td> 7.3.0 onwards
    <td> AM64x, AM243x
    <td> Fixed
</tr>
<tr>
    <td> MCUSDK-892
    <td> USB boot with SBL OSPI fails
    <td> USB, SBL, OSPI
    <td> 7.3.2
    <td> AM64x, AM243x
    <td> Fixed
</tr>
<tr>
    <td> MCUSDK-877
    <td> SYSFW reserved memory in MSRAM BANK 7 not accessible
    <td> SBL, SCICLIENT
    <td> 7.3.1
    <td> AM64x, AM243x
    <td> Fixed
</tr>
<tr>
    <td> MCUSDK-1232
    <td> SYSCFG does not support generic PHY for ICSS and CPSW
    <td> ICSS, CPSW
    <td> 7.3.2
    <td> AM64x, AM243x
    <td> Fixed
</tr>
<tr>
    <td> MCUSDK-1361
    <td> SYSCFG doesn't allow full MPU configuration flexibility
    <td> DPL, SYSCFG
    <td> 7.3.2
    <td> AM64x, AM243x
    <td> Fixed
</tr>
<tr>
    <td> MCUSDK-880
    <td> Project spec and post build steps are not CCS cloud friendly
    <td> CCS
    <td> 7.3.1
    <td> AM64x, AM243x
    <td> Fixed
</tr>
<tr>
    <td> MCUSDK-915
    <td> DMA Enabled examples failing when booted using SBL OSPI
    <td> SBL, OSPI, FLASH
    <td> 7.3.1
    <td> AM64x, AM243x
    <td> Fixed
</tr>
<tr>
    <td> MCUSDK-942
    <td> ECAP CSL Capture Event Counter Reset Function Fails
    <td> ECAP
    <td> 7.3.0 onwards
    <td> AM64x, AM243x
    <td> Fixed
</tr>
<tr>
    <td> MCUSDK-1010
    <td> Last character sometimes gets corrupted when UART driver is closed
    <td> UART
    <td> 7.3.0 onwards
    <td> AM64x, AM243x
    <td> Fixed
</tr>
<tr>
    <td> MCUSDK-1159
    <td> CCS script load_dmsc.js fails when trying to execute Init_M4() GEL function
    <td> CCS
    <td> 7.3.2
    <td> AM64x, AM243x
    <td> Fixed
</tr>
<tr>
    <td> MCUSDK-1161
    <td> OSPI DMA low latency application fails in CCS test setup
    <td> OSPI, FLASH
    <td> 7.3.2
    <td> AM64x, AM243x
    <td> Fixed
</tr>
<tr>
    <td> MCUSDK-1370
    <td> FreeRTOS: operations from stdatomics.h do not work with FreeRTOS task switch
    <td> FreeRTOS
    <td> 7.3.2
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
    <td> MCUSDK-750
    <td> ICSS PRU GPIO pins are not supported by SYSCFG
    <td> SysCfg
    <td> 7.3.0 onwards
    <td> AM64x, AM243x
    <td> Perform manual pinmux
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
    <td> MCUSDK-1564
    <td> time.h APIs do not work as expected with SBL OSPI
    <td> SBL, OSPI
    <td> 7.3.0 onwards
    <td> AM64x, AM243x
    <td> Use timestamps from HW timer
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

### Examples

<table>
<tr>
    <th> Module
    <th> Affected API
    <th> Change
    <th> Additional Remarks
</tr>
<tr>
    <td> EtherCAT
    <td> -
    <td> Bekchoff stack based EtherCAT example is moved from `examples\networking` to `examples\industrial_protocols`
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
    <td> FreeRTOS Kernel
    <td> FreeRTOS Config .h
    <td> configOPTIMIZE_FOR_LATENCY set to 0 by default to allow CPU load measurement to be enabled by default
    <td> -
</tr>
</table>

### Industrial Protocols

<table>
<tr>
    <th> Module
    <th> Affected API
    <th> Change
    <th> Additional Remarks
</tr>
<tr>
    <td> EtherCAT SubDevice FWHAL, EtherNet/IP Adapter FWHAL, Profinet Device FWHAL
    <td> -
    <td> Sources are moved from `source\networking` to `source\industrial_protocols`
    <td> -
</tr>
</table>