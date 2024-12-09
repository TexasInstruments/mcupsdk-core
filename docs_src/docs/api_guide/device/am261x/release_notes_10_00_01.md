# Release Notes 10.00.01 {#RELEASE_NOTES_10_00_01_PAGE}

[TOC]

\attention 1. Also refer to individual module pages for more details on each feature, unsupported features, important usage guidelines.

\attention 2. There is a known issue of PMIC Watchdog resetting the SOC every 10 mins in CCS Gel flow. This has been fixed in SBL flow by disabling PMIC Watchdog
              using I2C interface. Use Flash SBL NULL if CCS Debug is needed or Add the same logic for PMIC Watchdog disable in application if CCS debug using Gel
              flow is mandatory.
\attention 3. Networking examples support has been tested only on AM261x-LP board.
\attention 4. SDK Datasheet is not generated for AM261x. This will be available in the next SDK release.
\attention 5. This is an early access release for AM261x SOM Board with limited testing to enable early development for customers. Basic validation of every IP has been done
              except Networking components and USB.
\attention 6. There is a known issue of OSPI Phy Tuning not working on AM261x LP and SOM Board. So, Phy tuning is disabled by default in Examples and SBL OSPI. 
\attention 7. DFU Utils tool is not supported on Mac systems due to a build issue.

\note The examples will show usage of SW modules and APIs on a specific CPU instance and OS combination. \n
      Unless explicitly noted otherwise, the SW modules would work in both FreeRTOS and no-RTOS environment. \n
      Unless explicitly noted otherwise, the SW modules would work on any of the R5F's present on the SOC. \n

## New in this Release

Feature                                                                                         | Module
------------------------------------------------------------------------------------------------|----------------------------------- 
SOM Board Basic Support                                                                         | SDK Infra
Derby PMIC Driver and Example support                                                           | SDK Infra
Basic Ethernet MAC and Switch Support                                                           | Networking
LwIP TCP/IP stack and gPTP TimeSync stack                                                       | Networking
Auto PHY (DP83TG720-EVM-AM2) and Industrial PHY (DP83826-EVM-AM2) Support                       | Networking

# Modules Not tested/supported in this release

- USB and Networking support on AM261x SOM Board.
- OSPI Phy Tuning on AM261x LP and SOM Board.

## Device and Validation Information

SOC    | Supported CPUs  | EVM                                                                          | Host PC
-------|-----------------|------------------------------------------------------------------------------|-----------------------------------------
AM261x | R5F             | AM261x Launchpad     (referred to as am261x-lp in code). \n                  | Windows 10 64b or Ubuntu 18.04 64b or MacOS
AM261x | R5F             | AM261x SOM           (referred to as am261x-som in code). \n                 | Windows 10 64b or Ubuntu 18.04 64b or MacOS

## Dependent Tools and Compiler Information

Tools                   | Supported CPUs | Version
------------------------|----------------|-----------------------
Code Composer Studio    | R5F            | 12.8.1
SysConfig               | R5F            | 1.21.2 build, build 3837
TI ARM CLANG            | R5F            | 4.0.1.LTS
FreeRTOS Kernel         | R5F            | 11.1.0
LwIP                    | R5F            | STABLE-2_2_0_RELEASE
Mbed-TLS                | R5F            | 2.13.1


## Key Features

### Experimental Features

\attention Features listed below are early versions and should be considered as "experimental".
\attention Users can evaluate the feature, however the feature is not fully tested at TI side.
\attention TI would not support these feature on public e2e.
\attention Experimental features will be enabled with limited examples and SW modules.

Feature                                                             | Module
--------------------------------------------------------------------|--------------------------
-                                                                   | -

### OS Kernel

OS              | Supported CPUs  | SysConfig Support | Key features tested                                             | Key features not tested / NOT supported
----------------|-----------------|-------------------|-----------------------------------------------------------------|----------------------------------------
FreeRTOS Kernel | R5F             | NA                | Task, Task notification, interrupts, semaphores, mutexs, timers | Task load measurement using FreeRTOS run time statistics APIs. Limited support for ROV features.
FreeRTOS POSIX  | R5F             | NA                | pthread, mqueue, semaphore, clock                               | -
NO RTOS         | R5F             | NA                | See **Driver Porting Layer (DPL)** below                        | -

### Driver Porting Layer (DPL)

Module            | Supported CPUs  | SysConfig Support | OS support       | Key features tested                                           | Key features not tested / NOT supported
------------------|-----------------|-------------------|------------------|---------------------------------------------------------------|----------------------------------------
Cache             | R5F             | YES               | FreeRTOS, NORTOS | Cache write back, invalidate, enable/disable                  | -
Clock             | R5F             | YES               | FreeRTOS, NORTOS | Tick timer at user specified resolution, timeouts and delays  | -
CpuId             | R5F             | NA                | FreeRTOS, NORTOS | Verify Core ID and Cluster ID that application is currently running on    | -
CycleCounter      | R5F             | NA                | FreeRTOS, NORTOS | Measure CPU cycles using CPU specific internal counters       | -
Debug             | R5F             | YES               | FreeRTOS, NORTOS | Logging and assert to any combo of: UART, CCS, shared memory  | -
Heap              | R5F             | NA                | FreeRTOS, NORTOS | Create arbitrary heaps in user defined memory segments        | -
Hwi               | R5F             | YES               | FreeRTOS, NORTOS | Interrupt register, enable/disable/restore, Interrupt prioritization                    | -
MPU               | R5F             | YES               | FreeRTOS, NORTOS | Setup MPU and control access to address space                 | -
Semaphore         | R5F             | NA                | FreeRTOS, NORTOS | Binary, Counting Semaphore, recursive mutexs with timeout     | -
Task              | R5F             | NA                | FreeRTOS         | Create, delete tasks                                          | -
Timer             | R5F             | YES               | FreeRTOS, NORTOS | Configure arbitrary timers                                    | -

### Secondary Bootloader (SBL)

Module     | Supported CPUs  | SysConfig Support | OS support       | Key features tested                                                         | Key features not tested / NOT supported
-----------|-----------------|-------------------|------------------|-----------------------------------------------------------------------------|----------------------------------------------------
Bootloader | R5FSS0-0        | YES               | NORTOS           | Boot modes: OSPI, UART. All R5F's. RPRC, multi-core image format            | Force Dual Core Mode

### SOC Device Drivers

Peripheral   | Supported CPUs | SysConfig Support | DMA Supported                         | Key features tested                                                                                                                                             | Key features not tested / NOT supported
-------------|----------------|-------------------|---------------------------------------|-----------------------------------------------------------------------------------------------------------------------------------------------------------------|---------------------------------------------------
ADC          | R5F            | YES               | Yes. Examples:  adc_soc_continuous_dma, adc_alternate_dma_trigger | Single software triggered conversion, Multiple ADC trigger using PWM, Result read using DMA (normal and alternate triggers), EPWM trip through PPB limit, PPB features, Burst mode, Single and Differential mode, Interrupt with Offset from Aquisition Window, EPWM/ECAP/RTI triggered conversions, Trigger Repeater for Undersampling and Oversampling, Global Force on Multiple ADCs, Internal DAC Loopback to Calibration Channels, Safety Checker and Aggregator, Open Short Detection feature                 | External channel selection
Bootloader   | R5F            | YES               | Yes. DMA enabled for SBL OSPI         | Boot modes: OSPI, UART. All R5F's                                                                                                                               | -
CMPSS        | R5F            | YES               | NA                                    | Asynchronous PWM trip, digital filter                                                                                                                                           | CMPSS Dac LoopBack feature
CPSW         | R5F            | YES               | No                                    | MAC & PHY loopback(DP83826-EVM-AM2) with RMII 100Mbps, MAC & PHY loopback(DP83TG720-EVM-AM2) with RGMII 1Gbps, LWIP (DP83TG720-EVM-AM2, DP83826-EVM-AM2): Getting IP, Ping, Layer 2 MAC, Layer 2 PTP Timestamping and Ethernet CPSW Switch support, TSN stack                      | MII mode
DAC          | R5F            | YES               | Yes. Example: dac_sine_dma            | Constant voltage, Square wave generation, Sine wave generation with and without DMA, Ramp wave generation, Random Voltage generation                            | -
ECAP         | R5F            | YES               | yes. Example : ecap_edma              | ECAP APWM mode, PWM capture, DMA trigger in both APWM and Capture Modes                                                                                         | -
EDMA         | R5F            | YES               | NA                                    | DMA transfer using interrupt and polling mode, QDMA Transfer, Channel Chaining, PaRAM Linking                                                                   | -
EPWM         | R5F            | YES               | Yes. Example: epwm_dma, epwm_xcmp_dma | Multiple EPWM Sync from Top Module, PWM outputs A and B in up-down count mode, Trip zone, Update PWM using EDMA, Valley switching, High resolution time period adjustment, chopper module features, type5 features, global load and link feature           | -
EQEP         | R5F            | YES               | NA                                    | Speed and Position measurement. Frequency Measurement                                                                                                           | -
FSI          | R5F            | YES               | YES                                   | RX, TX, polling, interrupt, DMA mode, single lane loopback.                                                                                                     | - FSI Spi Mode
GPIO         | R5F            | YES               | NA                                    | Output, Input and Interrupt functionality                                                                                                                       | -
I2C          | R5F            | YES               | No                                    | Controller mode, basic read/write                                                                                                                               | -
IPC Notify   | R5F            | YES               | NA                                    | Mailbox functionality, IPC between RTOS/NORTOS CPUs                                                                                                             | M4F core
IPC Rpmsg    | R5F            | YES               | NA                                    | RPMessage protocol based IPC                                                                                                                                    | M4F core
LIN          | R5F            | YES               | YES                                   | RX, TX, polling, interrupt, DMA mode.                                                                                                                           | -
MCAN         | R5F            | YES               | No                                    | RX, TX, interrupt and polling mode, Corrupt Message Transmission Prevention, Error Passive state, Bus Off State, Bus Monitoring Mode                            | -
MCSPI        | R5F            | YES               | Yes. Example: mcspi_loopback_dma      | Controller/Peripheral mode, basic read/write, polling, interrupt and DMA mode                                                                                   | -
MDIO         | R5F            | YES               | NA                                    | Register read/write, link status and link interrupt enable API                                                                                                  | -
PINMUX       | R5F            | YES               | NA                                    | Tested with multiple peripheral pinmuxes                                                                                                                        | -
PMU          | R5F            | NO                | NA                                    | Tested various PMU events                                                                                                                                       | Counter overflow detection is not enabled
OptiFlash    | R5F            | Yes               | NA                                    | FLC, RL2, RAT functionality, XIP with RL2 enabled                                                                                                               | OptiShare
OSPI         | R5F            | YES               | Yes. Example: ospi_flash_dma          | Read direct, Write indirect, Read/Write commands, DMA for read                                                                                                  | Phy Support
RTI          | R5F            | YES               | No                                    | Counter read, timebase selection, comparator setup for Interrupt, DMA requests                                                                                  | Capture feature, fast enabling/disabling of events not tested
SDFM         | R5F            | YES               | No                                    | ECAP Clock LoopBack, Filter data read from CPU                 |  Filter data read with PWM sync, triggered DMA read from the Filter FIFO
SOC          | R5F            | YES               | NA                                    | Lock/unlock MMRs, clock enable, set Hz, Xbar configuration, SW Warm Reset, Address Translation                                                                  | -
SPINLOCK     | R5F            | NA                | NA                                    | Lock, unlock HW spinlock                                                                                                                                        | -
UART         | R5F            | YES               | Yes. Example: uart_echo_dma           | Basic read/write at baud rate 115200, polling, interrupt mode                                                                                                   | HW flow control not tested, DMA mode not supported
WATCHDOG     | R5F            | YES               | NA                                    | Reset mode, Interrupt mode                                                                                                                                      | -

### Trigonometric Operations

Peripheral   | Supported CPUs | SysConfig Support | DMA Supported                         | Key features tested                                                                                                                                             | Key features not tested / NOT supported
-------------|----------------|-------------------|---------------------------------------|-----------------------------------------------------------------------------------------------------------------------------------------------------------------|---------------------------------------------------
TMU          | R5F            | NO                | NA                                    | TMU Operations, Pipelining, Contex Save                                                                                                                         | Square Root, Division Operations. more than 1 Interrupt Nesting for the contex save is not Supported.

### Board Device Drivers

Peripheral | Supported CPUs | SysConfig Support | Key features tested                                         | Key features not tested
-----------|----------------|-------------------|-------------------------------------------------------------|------------------------
EEPROM     | R5F            | YES               | Only compiled                                               | -
FLASH      | R5F            | YES               | OSPI Flash                                                  | -
LED        | R5F            | YES               | GPIO                                                        | -
ETHPHY     | R5F            | YES               | Tested with ethercat_slave_beckhoff_ssc_demo example        | -
IOEXPANDER | R5F            | YES               | IO configurability                                          | -
PMIC       | R5F            | YES               | Watchdog Reset and disable                                  | -

### Networking

Module                      | Supported CPUs | SysConfig Support | OS Support  | Key features tested                                                                    | Key features not tested
----------------------------|----------------|-------------------|-------------|----------------------------------------------------------------------------------------|------------------------
Time-Sensitive Networking(gPTP-IEEE 802.1AS) | R5F            | NO                | FreeRTOS    | gPTP IEEE 802.1 AS-2020 compliant gPTP stack, End Nodes and Bridge mode support, YANG data model configuration  | Multi-Clock Domain
LwIP                                         | R5F            | YES               | FreeRTOS    | TCP/UDP IP networking stack with and without checksum offload enabled, TCP/UDP IP networking stack with server and client functionality, basic Socket APIs, netconn APIs and raw APIs, DHCP, ping, scatter-gather                         | Other LwIP features
Ethernet driver (ENET)                       | R5F            | YES               | FreeRTOS    | Ethernet as port using CPSW, MAC & PHY loopback with RMII 100Mbps(DP83826-EVM-AM2), MAC & PHY loopback with RMII 100Mbps(DP83TG720-EVM-AM2), Layer 2 MAC, Packet Timestamping, CPSW Switch, CPSW EST, interrupt pacing, Policer and Classifier  |  MII mode
ICSS-EMAC                   | R5F            | YES               | FreeRTOS    | Switch and MAC features, Storm Prevention (MAC), Host Statistics, Multicast Filtering  | Promiscuous Mode

<!-- Mbed-TLS                    | R5F            | NO                | FreeRTOS    | Tested software cryptography after porting, used mbedTLS with LwIP to implement HTTPS server  | Hardware offloaded cryptography -->

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
    <td> MCUSDK-13754
    <td> AM261x: Port 1 RX not working with DP83826-EVM-AM2 PHY
    <td> Networking
    <td> 10.00.00 onwards
    <td> -
    <td> -
</tr>
<tr>
    <td> MCUSDK-13641
    <td> Increased build time for examples using Link Time Optimization (-flto) with TI-ARM-CLANG 4.0.0 LTS
    <td> Build
    <td> 10.00.00 onwards
    <td> AM261x
    <td> Issue fixed in 4.0.1 LTS CLANG compiler
</tr>
<tr>
    <td> MCUSDK-13856
    <td> PRU Clock not configured in SBL.
    <td> PRU
    <td> 10.00.00
    <td> AM261x
    <td> ICSS Core clock configuration support added in SOC RCM module.
</tr>
<tr>
    <td> MCUSDK-13772
    <td> SysCfg showing smaller TCM size in memory configurator
    <td> Memory Configurator
    <td> 10.00.00
    <td> AM261x
    <td> Memory Configurator metadata updated with correct TCM Size.
</tr>
<tr>
    <td> MCUSDK-13851
    <td> AM261x does not have support for UART4 and UART5.
    <td> UART
    <td> 10.00.00
    <td> AM261x
    <td> Added UART4 and UART5 instance support in SysCfg.
</tr>
<tr>
    <td> MCUSDK-13773
    <td> EEPROM Read-Write not working properly.
    <td> I2C
    <td> 10.00.00
    <td> AM261x
    <td> EEPROM Address was incorrect.
</tr>
</table>

## Known Issues
<table>
<tr>
    <th> ID
    <th> Head Line
    <th> Module
    <th> Reported in release
    <th> Workaround
</tr>
<tr>
    <td> MCUSDK-13748
    <td> Am261x adc_soc_software_sync and adc_sw_interleaved_averaging example not working
    <td> ADC
    <td> 10.00.00 onwards
    <td> -
</tr>
<tr>
    <td> MCUSDK-13750
    <td> AM261x hrpwm_deadband_sfo example issue
    <td> EPWM
    <td> 10.00.00 onwards
    <td> -
</tr>
<tr>
    <td> MCUSDK-13164
    <td> AM26x: EPWM DeadBand example failure
    <td> EPWM
    <td> 10.00.00 onwards
    <td> -
</tr>
<tr>
    <td> MCUSDK-13755
    <td> AM261x: 10% RX align code and CRC errors in port 2
    <td> Networking
    <td> 10.00.00 onwards
    <td> -
</tr>
<tr>
    <td> SMCUAPPS-972
    <td> AM261x: Gel files upgrade to program the HSDIVIDER clock correctly
    <td> MCU Apps
    <td> 10.00.01 onwards
    <td> -
</tr>
<tr>
    <td> MCUSDK-13828
    <td> AM261x: ENET: Iperf TCP failing with 1Gbps
    <td> Networking
    <td> 10.00.01 onwards
    <td> -
</tr>
<tr>
    <td> MCUSDK-13829
    <td> AM261x: ENET: EST fails for priority 0 with IND phy
    <td> Networking
    <td> 10.00.01 onwards
    <td> -
</tr>
<tr>
    <td> MCUSDK-13836
    <td> Networking examples not working in SBL null and SBL OSPI boot mode
    <td> SBL
    <td> 10.00.01 onwards
    <td> Load the image in DEV/CCS boot mode
</tr>
<tr>
    <td> MCUSDK-13847
    <td> AM261x: GPTP lwIP debug example doesnt fit in RAM
    <td> Networking
    <td> 10.00.01 onwards
    <td> -
</tr>
<tr>
    <td> MCUSDK-13513
    <td> AM263Px, AM261x: UDP IPERF TX is unstable with 100Mbps link speed
    <td> Networking
    <td> 10.00.01 onwards
    <td> -
</tr>
<tr>
    <td> MCUSDK-14052
    <td> AM261x: OSPI Phy tuning fails on am261x LP
    <td> OSPI
    <td> 10.00.00 onwards
    <td> -
</tr>
<tr>
    <td> MCUSDK-14054
    <td> MMCSD: SysCfg sanity failure on AM26x devices
    <td> MMCSD
    <td> 10.00.01 onwards
    <td> Use default configuration as in SDK examples.
</tr>
<tr>
    <td> MCUSDK-14055
    <td> SBL DFU and SBL DFU Uniflash Example failure
    <td> USB
    <td> 10.00.01 onwards
    <td> -
</tr>
<tr>
    <td> MCUSDK-14056
    <td> Klocwork issues on USB Driver
    <td> USB
    <td> 10.00.01 onwards
    <td> -
</tr>
<tr>
    <td> PINDSW-7715
    <td> Dual EMAC instance not working with both ports together for icss_emac_lwip example
    <td> ICSS-EMAC
    <td> 10.00.01 onwards
    <td> None
</tr>
<tr>
    <td> PINDSW-7746
    <td> Low iperf values in TCP and UDP
    <td> ICSS-EMAC
    <td> 10.00.01 onwards
    <td> None
</tr>
<tr>
    <td> PINDSW-8118
    <td> Enabling DHCP mode in icss_emac_lwip example causes assert
    <td> ICSS-EMAC
    <td> 10.00.01 onwards
    <td> None
</tr>
<tr>
    <td> -
    <td> -
    <td> -
    <td> -
    <td> -
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
    <td> -
    <td> -
    <td> -
    <td> -
</tr>
</table>

## Limitations
<table>
<tr>
    <th> ID
    <th> Head Line
    <th> Module
    <th> Reported in release
    <th> Workaround
</tr>
<tr>
    <td> -
    <td> DP83TG720-EVM-AM2 and DP83826-EVM-AM2 dont work simultaneously for switching traffic in AM261-LP boards
    <td> Networking
    <td> 10.00.00 onwards
    <td> -
</tr>
<tr>
    <td> -
    <td> -
    <td> -
    <td> -
    <td> -
</tr>
</table>

## Upgrade and Compatibility Information
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
    <td> -
    <td> -
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
    <td> -
    <td> -
    <td> -
    <td> -
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
    <td> -
    <td> -
    <td> -
    <td> -
</tr>
</table>
