# Release Notes 11.00.00 {#RELEASE_NOTES_11_00_00_PAGE}

[TOC]

\attention 1. Also refer to individual module pages for more details on each feature, unsupported features, important usage guidelines.

\attention 2. RPRC image format has been deprecated from this release. Multi Core ELF image format should be used. (\ref MCELF_LANDING).

\attention 3. DFU Utils tool is not supported on Mac systems due to a build issue.

\attention 4. SDK has been updated to support CCS Theia Out of Box from this release. Refer to Compatibility section below for changes w.r.t Eclipse.

\attention 5. The default Stack size is 16KB and Heap size 32 KB for SDK examples. This can be adjusted as per application requirement through Memory Configurator in SysCfg or by updating Linker script in case of standalone applications.

\attention 6. For customer migrating from any release before SDK 10.00.00 to 10.00.00 or any release after, "DPL CFG" module has to be added in SysCfg in order to enable interrupts during DPL initialization. This module was added in SysCfg for 10.00.00 release to allow configuring Init time Enable/Disable of interrupts for RTOS applications (Default set to enabled).

\note The examples will show usage of SW modules and APIs on a specific CPU instance and OS combination. \n
      Unless explicitly noted otherwise, the SW modules would work in both FreeRTOS and no-RTOS environment. \n
      Unless explicitly noted otherwise, the SW modules would work on any of the R5F's present on the SOC. \n

## New in this Release

Feature                                                                                         | Module
------------------------------------------------------------------------------------------------|-----------------------------------
Clock Tree support for PLL and Peripheral clock configuration                                                                                   | Sysconfig
CCS Theia Support                                                                               | CCS
Multi Core FreeRTOS IPC Example                                                                 | IPC
OSPI Phy Graph Plotter Example                                                                  | OSPI
Rev A Launchpad Support                                                                         | Board
Rev A SOM Support                                                                               | Board
Board Level Sysconfig Support                                                                   | Sysconfig
USB Sysconfig support                                                                           | Sysconfig
McSPI External Loopback Example                                                                 | McSPI
USB NCM Class Support                                                                           | USB
FreeRTOS based CDC Example                                                                      | USB
LwIP stack upgrade to STABLE-2_2_1_RELEASE                                                      | Networking
IET Feature Enablement via Syscfg-GUI                                                           | Networking
XIP+RL2 support is included in Networking OOB example, referenced in EXAMPLES_ENET_LWIP_CPSW_UDPCLIENT | Networking
SENT Decoder and Encoder Examples                                                               | PRU-IO

# Modules Not tested/supported in this release

- MMCSD and GPMC drivers are not validated on AM261x LP or SOM Board.

## Device and Validation Information

SOC    | Supported CPUs  | EVM                                                                          | Host PC
-------|-----------------|------------------------------------------------------------------------------|-----------------------------------------
AM261x | R5F             | AM261x Launchpad Rev A    (referred to as am261x-lp in code). \n            | Windows 10 64b or Ubuntu 18.04 64b or MacOS
AM261x | R5F             | AM261x SOM Rev A          (referred to as am261x-som in code). \n           | Windows 10 64b or Ubuntu 18.04 64b or MacOS

## Dependent Tools and Compiler Information

Tools                   | Supported CPUs | Version
------------------------|----------------|-----------------------
Code Composer Studio    | R5F            | 20.3.0
SysConfig               | R5F            | 1.25.0 build, build 42688
TI ARM CLANG            | R5F            | 4.0.3.LTS
FreeRTOS Kernel         | R5F            | 11.1.0
LwIP                    | R5F            | STABLE-2_2_1_RELEASE
Mbed-TLS                | R5F            | 2.13.1
Uniflash                | R5F            | 9.3.0


## Key Features

### Experimental Features

\attention Features listed below are early versions and should be considered as "experimental".
\attention Users can evaluate the feature, however the feature is not fully tested at TI side.
\attention TI would not support these feature on public e2e.
\attention Experimental features will be enabled with limited examples and SW modules.

Feature                                                             | Module
--------------------------------------------------------------------|--------------------------
Ethernet MDIO with Clause 45 support                                | Networking

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
Bootloader | R5FSS0-0        | YES               | NORTOS           | Boot modes: OSPI, UART. All R5F's. Multi-core ELF image format            | Force Dual Core Mode

### SOC Device Drivers

Peripheral   | Supported CPUs | SysConfig Support | DMA Supported                         | Key features tested                                                                                                                                             | Key features not tested / NOT supported
-------------|----------------|-------------------|---------------------------------------|-----------------------------------------------------------------------------------------------------------------------------------------------------------------|---------------------------------------------------
ADC          | R5F            | YES               | Yes. Examples:  adc_soc_continuous_dma, adc_alternate_dma_trigger | Single software triggered conversion, Multiple ADC trigger using PWM, Result read using DMA (normal and alternate triggers), EPWM trip through PPB limit, PPB features, Burst mode, Single and Differential mode, Interrupt with Offset from Aquisition Window, EPWM/ECAP/RTI triggered conversions, Trigger Repeater for Undersampling and Oversampling, Global Force on Multiple ADCs, Internal DAC Loopback to Calibration Channels, Safety Checker and Aggregator, Open Short Detection feature                 | External channel selection
Bootloader   | R5F            | YES               | Yes. DMA enabled for SBL OSPI         | Boot modes: OSPI, UART. All R5F's                                                                                                                               | -
CMPSS        | R5F            | YES               | NA                                    | Asynchronous PWM trip, digital filter, Calibration                                       | CMPSS Dac LoopBack feature
CPSW         | R5F            | YES               | No                                    | MAC & PHY loopback(DP83826-EVM-AM2) with RMII and MII 100Mbps , MAC & PHY loopback(DP83TG720-EVM-AM2) with RGMII 1Gbps, LWIP (DP83TG720-EVM-AM2, DP83826-EVM-AM2): Getting IP, Ping, Layer 2 MAC, Layer 2 PTP Timestamping and Ethernet CPSW Switch support, TSN stack                      | -
DAC          | R5F            | YES               | Yes. Example: dac_sine_dma            | Constant voltage, Square wave generation, Sine wave generation with and without DMA, Ramp wave generation, Random Voltage generation                            | -
ECAP         | R5F            | YES               | yes. Example : ecap_edma              | ECAP APWM mode, PWM capture, DMA trigger in both APWM and Capture Modes, Signal Monitoring features                                                                                         | -
EDMA         | R5F            | YES               | NA                                    | DMA transfer using interrupt and polling mode, QDMA Transfer, Channel Chaining, PaRAM Linking, Error Handling                                                                 | -
EPWM         | R5F            | YES               | Yes. Example: epwm_dma, epwm_xcmp_dma | Multiple EPWM Sync from Top Module, PWM outputs A and B in up-down count mode, Trip zone, Update PWM using EDMA, Valley switching, High resolution time period adjustment, chopper module features, type5 features, global load and link feature           | -
EQEP         | R5F            | YES               | NA                                    | Speed and Position measurement. Frequency Measurement, speed direction, cw-ccw modes                                                                            | -
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
OptiFlash    | R5F            | Yes               | NA                                    | FLC, RL2, RAT functionality, XIP with RL2 enabled, OTFA, FOTA, Optishare, Smart Layout                                                                                 | -
OSPI         | R5F            | YES               | Yes. Example: ospi_flash_dma          | Read direct, Write indirect, Read/Write commands, DMA for read                                                                                                  | -
RTI          | R5F            | YES               | No                                    | Counter read, timebase selection, comparator setup for Interrupt, DMA requests                                                                                  | Capture feature, fast enabling/disabling of events not tested
SDFM         | R5F            | YES               | No                                    | ECAP Clock LoopBack, Filter data read from CPU, Filter data read with PWM sync, triggered DMA read from the Filter FIFO                                         |  -
SOC          | R5F            | YES               | NA                                    | Lock/unlock MMRs, clock enable, set Hz, Xbar configuration, SW Warm Reset, Address Translation                                                                  | -
SPINLOCK     | R5F            | NA                | NA                                    | Lock, unlock HW spinlock                                                                                                                                        | -
UART         | R5F            | YES               | Yes. Example: uart_echo_dma           | Basic read/write at baud rate 115200, polling, interrupt mode                                                                                                   | HW flow control not tested, DMA mode not supported
USB          | R5F            | No                | NA                                    | DFU, CDC Echo mode                                                                                                                                      | -
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
Ethernet driver (ENET)                       | R5F            | YES               | FreeRTOS    | Ethernet as port using CPSW, MAC & PHY loopback, Layer 2 MAC, Packet Timestamping, CPSW Switch, CPSW EST, interrupt pacing, Policer and Classifier  |  MII mode
ICSS-EMAC                   | R5F            | YES               | FreeRTOS    | Switch and MAC features, Storm Prevention (MAC), Host Statistics, Multicast Filtering  | Promiscuous Mode

<!-- Mbed-TLS                    | R5F            | NO                | FreeRTOS    | Tested software cryptography after porting, used mbedTLS with LwIP to implement HTTPS server  | Hardware offloaded cryptography -->

### Safety Diagnostic Library

Module            | Supported CPUs  | SysConfig Support | OS support       | Key features tested                                                                            | Key features not tested / NOT supported
------------------|-----------------|-------------------|------------------|------------------------------------------------------------------------------------------------|----------------------------------------
MCRC              | R5F             | NA                |  NORTOS | Full CPU, Auto CPU Mode and Semi CPU Auto Mode                                                          | -
DCC               | R5F             | NA                |  NORTOS | Single Shot and Continuous modes                                    | -
PBIST             | R5F             | NA                |  NORTOS | Memories supported by MSS PBIST controller.          | -
ESM               | R5F             | NA                |  NORTOS | Tested in combination with RTI, DCC                                        | -
RTI               | R5F             | NA                |  NORTOS | WINDOWSIZE_100_PERCENT, WINDOWSIZE_50_PERCENT ,Latency/Propagation timing error(early)(50% window),Latency/Propagation timing error(late)(50% window)                                     | -
ECC               | R5F             | NA                |  NORTOS | ECC of MSS_L2, R5F TCM, MCAN, VIM, ICSSM, TPTC      | FSS FOTA and OSPI
ECC Bus Safety    | R5F             | NA                |  NORTOS | AHB, AXI, TPTC                           | -
CCM               | R5F             | NA                |  NORTOS | CCM Self Test Mode,Error Forcing Mode and Self Test Error Forcing Mode. TMU and RL2 are also validated                      | -
R5F STC(LBIST), Static Register Read| R5F               | NA                |  NORTOS | STC of R5F, R5F CPU Static Register Read                                 |-
TMU ROM Checksum  | R5F             | NA                |  NORTOS | ROM checksum for TMU                                                                         | -
Time out Gasket(STOG)  | R5F             | NA                |  NORTOS | Timeout gasket feature                    | -
Thermal Monitor(VTM)| R5F             | NA                |  NORTOS | Over, under and thershold temperature interrupts                   | -
Integrated Example  | R5F             | NA                |FreeRTOS | Integrated example with all the SDL modules integrated in to one example.|  ECC for TPTC, ECC Bus Safety and STC.

**Note**: SDL is validated only on SOM Board.

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
    <td> MCUSDK-14055
    <td> SBL DFU and SBL DFU Uniflash Example failure
    <td> USB
    <td> 10.00.01 onwards
    <td> AM261x
    <td> Resolved in driver
</tr>
<tr>
    <td> MCUSDK-14502
    <td> PMIC WDG QA Example not working on AM261x-LP E2 board
    <td> PMIC
    <td> 10.02.00 onwards
    <td> AM261x
    <td> Fixed in driver by increasing sleep time for waiting to trigger wdg reset
</tr>
<tr>
    <td> MCUSDK-14547
    <td> XIP Flashing not supported in SBL JTAG Uniflash example
    <td> SBL
    <td> 10.00.00 onwards
    <td> AM263Px, AM261x
    <td> Added XIP flashing support in the example
</tr>
<tr>
    <td> MCUSDK-14609
    <td> FOTA Example Failure on AM261x SOM
    <td> OptiFlash
    <td> 10.02.00 onwards
    <td> AM261x
    <td> Resolved by reducing OSPI clock frequency from 166Mhz to 133Mhz
</tr>
<tr>
    <td> MCUSDK-14606
    <td> USB Enumeration fails randomly
    <td> USB
    <td> 10.02.00 onwards
    <td> AM261x
    <td> -
</tr>
<tr>
    <td> MCUSDK-14704
    <td> Adding multiple instances of UART DMA LLD causes failure
    <td> UART
    <td> 10.02.00 onwards
    <td> AM263x, AM263Px, AM261x
    <td> Fixed array indexing while assigning dma config
</tr>
<tr>
    <td> MCUSDK-14917
    <td> "Selected mode" in pinmux.csv.xdt file not being updated correctly in SysCfg
    <td> Pinmux
    <td> 10.01.00 onwards
    <td> AM263x, AM263Px, AM261x
    <td> Fixed in Pinmux CSV template
</tr>
<tr>
    <td> PROC_SDL-9179
    <td> Redefinition error in MCU_PBIST Sysconfig
    <td> SDL
    <td> 10.02.00 onwards
    <td> AM263Px, AM261x
    <td> Resolved in Source code
</tr>
<tr>
    <td> MCUSDK-14749
    <td> McSPI: Non Powers of 2 cannot be configured as fifo trigger levels in polling and interrupt mode
    <td> McSPI
    <td> 10.02.00 onwards
    <td> AM263x, AM263Px, AM261x
    <td> Fix in SysCfg Meta file.
</tr>
<tr>
    <td> MCUSDK-13966
    <td> All UART triggers levels not exposed in SysCfg
    <td> UART
    <td> 10.00.00 onwards
    <td> AM263x, AM263Px, AM261x
    <td> Update SysCfg to show all trigger levels from 1 to 64.
</tr>
<tr>
    <td> MCUSDK-14573
    <td> Incorrect handling of errata i2310 in UART isr
    <td> UART
    <td> 10.00.00 onwards
    <td> AM263x, AM263Px, AM261x
    <td> Reorder the ISR state machine for handling UART errata correctly.
</tr>
<tr>
    <td> MCUSDK-14706
    <td> GPIO Qual selection API missing
    <td> GPIO
    <td> 10.00.00 onwards
    <td> AM263x, AM263Px, AM261x
    <td> Qual sel API added in pinmux driver
</tr>
<tr>
    <td> MCUSDK-14569
    <td> UART Errata i2310 is missing a step
    <td> UART
    <td> 10.00.00 onwards
    <td> AM263x, AM263Px, AM261x
    <td> Added IIR register read to clear the interrupt
</tr>
<tr>
    <td> MCUSDK-14620
    <td> SDK build fails in Mac Machines
    <td> Build
    <td> 10.02.00 onwards
    <td> AM263x, AM263Px, AM261x
    <td> Added GMAC library for MAC into SDK
</tr>
<tr>
    <td> MCUSDK-14659
    <td> Incorrect RTI clock source mux address for RTI 4 to 7
    <td> RTI
    <td> 10.00.00 onwards
    <td> AM263Px, AM261x
    <td> Updated to correct mux addresses in SysCfg
</tr>
<tr>
    <td> MCUSDK-13182
    <td> SysCfg unexpectedly changes OSPI Pin
    <td> OSPI
    <td> 10.00.00 onwards
    <td> AM263Px, AM261x
    <td> The OSPI pins are locked in SDK examples.
</tr>
<tr>
    <td> MCUSDK-14857, MCUSDK-14731
    <td> Core 1 unhalted in SBL before FSM Trigger, Memory load
    <td> SBL
    <td> 10.00.00 onwards
    <td> AM263x, AM263Px, AM261x
    <td> Skip unhalting core 1 of both clusters in dual core mode
</tr>
<tr>
    <td> MCUSDK-14712
    <td> OSPI Reset Pin being used before configuration
    <td> OSPI
    <td> 10.00.00 onwards
    <td> AM263Px, AM261x
    <td> Configure OSPI reset in OSPI open instead of Flash open
</tr>
<tr>
    <td> PROC_SDL-9160
    <td> PBIST does not cover the VIM memories
    <td> SDL
    <td> 10.02.00 onwards
    <td> AM263x, AM263Px, AM261x
    <td> Fixed in Source and Used polling method instead of ISR to cover VIM memory
</tr>
<tr>
    <td> PROC_SDL-9154
    <td> VTM Example stuck in UC2
    <td> SDL
    <td> 10.02.00 onwards
    <td> AM263Px, AM261x
    <td> Fixed in example
</tr>
<tr>
    <td> MCUSDK-14695
    <td> SDFM_configComparator has incorrect input in examples
    <td> SDFM
    <td> 10.00.00 onwards
    <td> AM263x, AM263Px, AM261x
    <td> Updated example to pass correct value
</tr>
<tr>
    <td> MCUSDK-14696
    <td> ADC Sysconfig does not seem to generate codes for repeaters
    <td> ADC
    <td> 10.00.00 onwards
    <td> AM263Px, AM261x
    <td> Fixed syscfg template file to generate trigger repeater code for burst mode
</tr>
<tr>
    <td> MCUSDK-14645
    <td> Implementation of the ADC_selectSOCExtChannel
    <td> ADC
    <td> 10.01.00 onwards
    <td> AM263Px, AM261x
    <td> Fixed ADC_selectSOCExtChannel API implementation
</tr>
<tr>
    <td> PROC_SDL-9147
    <td> VTM Usecase stuck in integrated example
    <td> SDL
    <td> 10.02.00 onwards
    <td> AM263Px, AM261x
    <td> Fixed in integrated example
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
    <td> MCUSDK-14148
    <td> AM261x : ZNC : Package in the early samples has ADC issue. Needs SW checks undone for ADC
    <td> ADC
    <td> 10.00.00 onwards
    <td> \b Details : ZNC package currently (Jan 2025), has, ADC 0 Reference lines not connected with that of the ADC 1's. There are some packages were shipped with the ADC Reference Monitor checks are Bypassed for the ADC front end enablement in the EFUSE. the SW code gen from the syscfg will assert the ADC init if the monitor throws a fault. It is requested to bypass the SW check as well to enable the customer to use the ADC.\n
    \b Workaround : only applicable in the devices with forementioned conditions.\n 1. Go to the file \code source/sysconfig/drivers/.meta/soc_ctrl/templates/soc_ctrl_adc_config.c.xdt \endcode\n 2. Find line with \code DebugP_assert(SOC_getAdcReferenceStatus(`montior2AdcMap[monitor][0]`) == true); \endcode \n 3.Change it to \code //DebugP_assert(SOC_getAdcReferenceStatus(`montior2AdcMap[monitor][0]`) == true); \endcode
</tr>
<tr>
    <td> MCUSDK-13865
    <td> HRPWM Deadband sfo example has 1ns jitter
    <td> EPWM
    <td> 10.00.00 onwards
    <td> -
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
    <td> MCUSDK-14950
    <td> AM26x: Networking examples show up as "..." on TIREX
    <td> Networking
    <td> 10.01.00 onwards
    <td> -
</tr>
<tr>
    <td> MCUSDK-14692
    <td> Raw HTTP Server example does not work with Static IP
    <td> Networking
    <td> 10.02.00 onwards
    <td> -
</tr>
<tr>
    <td> MCUSDK-14792
    <td> AM261x LwIP ping app does not work (enet_lwip_cpsw)
    <td> Networking
    <td> 10.02.00 onwards
    <td> -
</tr>
<tr>
    <td> MCUSDK-13896
    <td> Syscfg does not let you configure ethernet interfaces pinmux independently
    <td> Networking
    <td> 10.02.00 onwards
    <td> -
</tr>
<tr>
    <td> MCUSDK-15051
    <td> ENET: am261x: UDP client example not working in CCS boot mode
    <td> Networking
    <td> 11.00.00 onwards
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
    <td> MCUSDK-13201
    <td> HRPWM waveform not generating (in updwon count) when prescaler is non-zero and HRPE is enabled
    <td> EPWM
    <td> 10.00.01 onwards
    <td> None
</tr>
<tr>
    <td> MCUSDK-13834
    <td> EQEP: EQEP frequency measurement example is not working as expected
    <td> EQEP
    <td> 10.00.01 onwards
    <td> None
</tr>
<tr>
    <td> MCUSDK-14059
    <td> CMPSS DE example has Glitch in PWM output
    <td> CMPSS
    <td> 10.00.01 onwards
    <td> None
</tr>
<tr>
    <td> PROC_SDL-8392
    <td> In ECC bus safety example, ECC error is not properly cleared at the source.
    <td> SDL
    <td> 10.02.00 onwards
    <td> None
</tr>
<tr>
    <td> PROC_SDL-8787
    <td> ECC TPTC and STC examples are not supported in SDL integrated example.
    <td> SDL
    <td> 10.02.00 onwards
    <td> Use standalone examples.
</tr>
<tr>
    <td> PROC_SDL-8857
    <td> SDL integrated example does not support ECC Bus Safety.
    <td> SDL
    <td> 10.02.00 onwards
    <td> Use standalone example.
</tr>
<tr>
    <td> PROC_SDL-9163
    <td> ECC Aggregators FSS FOTA and OSPI
    <td> SDL
    <td> 10.02.00 onwards
    <td> None
</tr>
<tr>
    <td> MCUSDK-14898
    <td> SDL apps fails on other than RFSS0-0 with SBL
    <td> SBL, SDL
    <td> 11.00.00 onwards
    <td> This because SBL brings the RFSS0-1 out of reset before the SBL UART prints gets flushed. This will be fixed in next release. As a workaround the application in R5FSS1-0 can delay the start of application till SBL UART prints gets completed.
</tr>
<tr>
    <td> MCUSDK-13513
    <td> Multiple chip selects cannot be configured in SysCfg
    <td> OSPI
    <td> 10.00.00 onwards
    <td> -
</tr>
<tr>
    <td> MCUSDK-14582
    <td> Flash: Incorrect flash name after Loading Flash JSON
    <td> OSPI
    <td> 10.00.00 onwards
    <td> -
</tr>
<tr>
    <td> MCUSDK-14613
    <td> XIP image loading not working in CCS for am261x-lp E2 board
    <td> CCS, XIP
    <td> 10.02.00 onwards
    <td> -
</tr>
<tr>
    <td> MCUSDK-14714
    <td> Bufnum of 6 and 12 will cause the vring indexes to get corrupted
    <td> IPC
    <td> 10.00.00 onwards
    <td> Use other VRING buffer numbers.
</tr>
<tr>
    <td> MCUSDK-14879
    <td> Potential system hang issue due to priority mask based critical sections.
    <td> FreeRTOS
    <td> 10.00.00 onwards
    <td> Not to use Priority mask based critical sections (Disabled by default in SDK).
</tr>
<tr>
    <td> MCUSDK-14893
    <td> Sub projects under system projects cannot be changed in CCS Theia
    <td> CCS
    <td> 11.00.00 onwards
    <td> -
</tr>
<tr>
    <td> MCUSDK-14819
    <td> Ram is getting erased/overwritten once warm reset is done in application
    <td> SBL
    <td> 11.00.00 onwards
    <td> -
</tr>
<tr>
    <td> MCUSDK-14895
    <td> UART LLD Rx error checking logic checks if all errors exist at once
    <td> UART
    <td> 10.00.00 onwards
    <td> -
</tr>
<tr>
    <td> MCUSDK-14613
    <td> XIP image loading not working in CCS for am261x-lp E2 board
    <td> CCS, XIP
    <td> 10.02.00 onwards
    <td> -
</tr>
<tr>
    <td> MCUSDK-14647
    <td> All CANFD standard ID configurations are not exposed in syscfg gui
    <td> CAN
    <td> 10.02.00 onwards
    <td> Configure Config type, ID's etc in application
</tr>
<tr>
    <td> MCUSDK-14705
    <td> Flash verify not working for XIP images
    <td> Flash
    <td> 10.02.00 onwards
    <td> -
</tr>
<tr>
    <td> MCUSDK-14914
    <td> Unable to add UART communication port to target configuration in Theia
    <td> Real Time Debug
    <td> 10.02.00 onwards
    <td> Use the CCXML file from CCS eclipse after updating to correct COM port.
</tr>
<tr>
    <td> MCUSDK-14941
    <td> McSPI: Non Multiple's of FIFO cannot be transferred in DMA mode
    <td> McSPI
    <td> 10.00.00 onwards
    <td> Use FIFO Trigger level of 1.
</tr>
<tr>
    <td> MCUSDK-13011
    <td> Data Abort in application when all cores are running freertos using Gel files(CCS)
    <td> FreeRTOS
    <td> 10.00.00 onwards
    <td> Flash and use SBL NULL instead of gel files
</tr>
<tr>
    <td> MCUSDK-14636
    <td> AM261x: Phy Tuning not supported for NAND Flash and PSRAM
    <td> OSPI, Flash
    <td> 10.02.00 onwards
    <td> -
</tr>
<tr>
    <td> MCUSDK-14656
    <td> Mac OS Support not available for DFU Utils tool
    <td> USB
    <td> 10.02.00 onwards
    <td> -
</tr>
<tr>
    <td> PINDSW-9499
    <td> Short serial message and enhanced serial message not working in SENT decoder using IEP ECAP example
    <td> PRU-IO
    <td> 11.00.00 onwards
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
    <td> i2189
    <td> OSPI: Controller PHY Tuning Algorithm
    <td> OSPI
    <td> Implemented
</tr>
<tr>
    <td> i2311
    <td> USART: Spurious DMA Interrupts
    <td> UART
    <td> Implemented
</tr>
<tr>
    <td> i2324
    <td> No synchronizer present between GCM and GCD status signals
    <td> Common
    <td> Implemented
</tr>
<tr>
    <td> i2345
    <td> CPSW: Ethernet Packet corruption occurs if CPDMA fetches a packet which spans across memory banks
    <td> CPSW
    <td> Implemented
</tr>
<tr>
    <td> i2351
    <td> OSPI: Controller does not support Continuous Read mode with NAND Flash
    <td> OSPI
    <td> Implemented
</tr>
<tr>
    <td> i2354
    <td> SDFM: Two Back-to-Back Writes to SDCPARMx Register Bit Fields CEVT1SEL, CEVT2SEL, and HZEN Within Three SD-Modulator Clock Cycles can Corrupt SDFM State Machine, Resulting in Spurious Comparator Events
    <td> SDFM
    <td> Open
</tr>
<tr>
    <td> i2356
    <td> ADC: Interrupts may Stop if INTxCONT (Continue-to-Interrupt Mode) is not Set
    <td> ADC
    <td> Implemented
</tr>
<tr>
    <td> i2375
    <td> SDFM: SDFM module event flags (SDIFLG.FLTx_FLG_CEVTx) do not get set again if the comparator event is still active and digital filter path (using SDCOMPxCTL.CEVTxDIGFILTSEL) is being selected
    <td> SDFM
    <td> Open
</tr>
<tr>
    <td> i2383
    <td> OSPI: 2-byte address is not supported in PHY DDR mode
    <td> OSPI
    <td> Implemented
</tr>
<tr>
    <td> i2479
    <td> OSPI Boot Issue with GPIO61 Reset Pin Configuration
    <td> OSPI
    <td> Open
</tr>
<tr>
    <td> i2480
    <td> 1µs Glitch on GPIO61/OSPI0_RESET_OUT0 during OSPI Boot
    <td> GPIO
    <td> Open
</tr>
<tr>
    <td> i2404
    <td> Race condition in mailbox registers resulting in events miss
    <td> IPC, Mailbox
    <td> Implemented
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
    <td> DP83TG720-EVM-AM2 and DP83826-EVM-AM2 dont work simultaneously for switching traffic in AM261-LP Rev E1 and E2 boards
    <td> Networking
    <td> 10.00.00 onwards
    <td> -
</tr>
<tr>
    <td> MCUSDK-13630
    <td> Cache should not be enabled at last 32B L2 Bank boundary
    <td> Cache
    <td> 10.01.00
    <td> Create MPU configurations for last 32B of each L2 Bank with Non Cached attribute
</tr>
<tr>
    <td> -
    <td> LP-AM261 Out-of-box CPSW networking examples supports Rev-A board version seamlessly with DP83869 PHY.
    <td> Networking
    <td> 11.00.00 onwards
    <td> To use LP-AM261 Rev-E1,Rev-E2 boards, follow the steps mentioned in \ref AM261X_LP_E1_E2_SUPPORT.
</tr>
</table>

## Upgrade and Compatibility Information

### LP / SOM Rev E2 to Rev A revision Changes
<table>
<tr>
    <th> Component
    <th> Change
    <th> Comments
</tr>
<tr>
    <td> ADC / CMPSS
    <td> ADC / CMPSS pin positions have changed in LP Rev A
    <td> ADC0 Channels 0,4,6 have changed their pin positions from pins 23,26,29 in Rev E2 to 26,29,23 in Rev A respectively. Updated Example documentations <br>\ref EXAMPLES_DRIVERS_ADC_BURST_MODE_EPWM <br>\ref EXAMPLES_DRIVERS_ADC_SOC_SOFTWARE <br>\ref EXAMPLES_DRIVERS_ADC_SOFTWARE_INTERLEAVED_AVERAGING
</tr>
<tr>
    <td> SDFM
    <td> SDFM Pin Positions have Changed in LP Rev A
    <td> SDFM1 Clk0, D0 and D3 have changed their default positions from 18,12,43 in Rev E2 to 6,3,10 in Rev A respectively. Please follow the Pinmux configurations accordingly. Updated Example sysconfig and documentation for \ref EXAMPLES_DRIVERS_SDFM_ECAP_LOOP_BACK .
</tr>
<tr>
    <td> IO Expander
    <td> IO Expander Pins have changed in LP Rev A
    <td> IO Expander at 0x21H has updated pins P0,P3,P4,P5 from CPSW/ICSS_BRD_CONN_DET1, MDIO/MDC_MUX_SEL1, MDIO/MDC_MUX_SEL2, CPSW/ICSS_BRD_CONN_DET2 in E2 to BP_MUX_SW_S6, BP_MUX_SW_S4, MDIO/MDC_MUX_SEL, BP_MUX_SW_S5 in REV A .
</tr>
<tr>
    <td> GPIO
    <td> GPIO Interrupt Pin has changed in LP and SOM Rev A
    <td> For Launchpad, the GPIO interrupt pin has changed from GPIO124 to GPIO5 in REV A. GPIO INT Crossbar has changed from GPIO_INT_XBAR_GPIO_0_BANK_INTR_7 to GPIO_INT_XBAR_GPIO_0_BANK_INTR_0 in REV A. <br>For SOM, the GPIO interrupt pin has changed from GPIO128 to GPIO120 in REV A. GPIO INT Crossbar has changed from GPIO_INT_XBAR_GPIO_0_BANK_INTR_8 to GPIO_INT_XBAR_GPIO_0_BANK_INTR_7 in REV A.
</tr>
<tr>
    <td> PMIC
    <td> Resolved PMIC watchdog reset issue on LP Rev A
    <td> With the hardware fix for the PMIC watchdog reset issue in LP Rev A, the software workaround has been removed from all SBLs. For LP Rev E2 boards, the workaround is still required. Ensure the PMIC module is enabled in the SBL sysconfig.
</tr>

### Migration to 400 MHz in ZFG package
<table>
<tr>
    <th> Component
    <th> Change
    <th> Comments
</tr>
<tr>
    <td> Clocktree
    <td> Switching between 500 MHz and 400 MHz in ZFG package has changed.
    <td> There are two variants available, catering to different R5F clock frequencies: 400 MHz and 500 MHz. The default variant is 500 MHz, but users can switch to the 400 MHz variant. This switching was earlier done through clock module in sysconfig but now it's done through Device View settings (\ref CLOCKTREE_VARIANT_SWITCHING)
</tr>
</table>

### Compiling examples in MacOS machines

Currently the gmac library packaged within SDK is compiled using gcc darwin23.6.0 on Apple MAC.
To build SDK examples on a different toolchain, recompile the gmac library by using these steps:

\code
    $ cd {SDK_PATH}/tools/boot/multicore-elf/c_modules/gmac
    $ make all
\endcode

- A new library will be created inside mac/dist.
- Rename this file to "gmac.arm64-apple-darwin.darwin.dylib".

### RPRC Image format is Deprecated and Corresponding SBL's are also removed from SDK

RPRC image format is no longer supported and MCELF will be the only file format. Older SBL's and
Cfg files which mapped to RPRC format are removed and replaced with MCELF variants.
Below is the list of updated SBL's and Cfg files:

<table>
<tr>
    <th> Deprecated SBL + CFG File
    <th> Supported SBL + CFG File
</tr>
<tr>
    <td> SBL QSPI (default_sbl_qspi.cfg)
    <td> SBL QSPI MULTICORE ELF (mcelf_sbl_qspi.cfg)
</tr>
<tr>
    <td> SBL UART
    <td> SBL UART MULTICORE ELF
</tr>
<tr>
    <td> SBL SD (default_sbl_sd.cfg)
    <td> SBL SD MULTICORE ELF (mcelf_sbl_sd.cfg)
</tr>
<tr>
    <td> SBL CAN (default_sbl_can.cfg)
    <td> SBL CAN MULTICORE ELF (mcelf_sbl_can.cfg)
</tr>
<tr>
    <td> SBL CAN UNIFLASH (default_sbl_can_uniflash.cfg, default_sbl_can_uniflash_app.cfg)
    <td> SBL CAN UNIFLASH MULTICORE ELF (mcelf_sbl_can_uniflash.cfg, mcelf_sbl_can_uniflash_app.cfg)
</tr>
</table>

Please refer to the updated SDK example makefiles for Infra changes.

### Flash Reset moved to SysCfg

Earlier, flash reset was done in board.c file within application which is now moved
to SysCfg. If Flash reset logic needs to be added, please enable "Enable Flash Reset API"
configurable in Flash module. This is by enabled out of box for all SDK Flash examples.
For custom flash, define the flash reset API in application and add the API name to
"Flash Reset Function" configurable.

### Module clock configuration through Clock Tree

Previously our SDK had a mix of hardcoded clock configurations and limited configuration flexibility through sysconfig for the modules.
With Clocktree, we now have a clear view of the entire clock tree with configurable components like PLL, DPLL, muxes, dividers added with validity checks.
Earlier, the Input clock source and frequency for any module was configured through the module view in SysCfg. From now, this has to be done through clocktree.

Please refer to \ref CLOCKTREE for more details.

### Migrating back from CCS Theia to CCS Eclipse

SDK was supporting CCS Eclipse until this release and now been migrated to support CCS Theia Out of Box. Below sections describe how to update the applications for Eclipse if needed.

#### Importing and Building in Eclipse

CCS Projectspec files are same for both Theia and Eclipse. Hence, no update is needed to import and build an SDK application on Eclipse.

#### CCS SBL Loading

JS Script for SBL loading on eclipse is updated to "load_sbl_eclipse.js". Please refer to \ref TOOLS_CCS for more details.

### Migrating examples to 11.00.00 from older versions

\cond !SOC_AM64X
\note Images are shown for AM64x. It is application for @VAR_SOC_NAME as well.
\endcond

#### Makefile Changes
##### Library Name change on makefile and CCS projects
From 11.00.00 SDK all the libraries are built separately for OS. There are separate libraries available for NoRTOS and FreeROTS.
So the makefiles needs to be updated accordingly. Please refer the sample changes on the makefile below. These changes are not applicable for the
libraries which were already built separately for NoRTOS/FreeRTOS like kernel libraries.

For NoRTOS/baremetal,

\imageStyle{example_migration1.png,width:40%}
\image html example_migration1.png "Library name change for NoRTOS example"

For FreeRTOS,

\imageStyle{example_migration2.png,width:40%}
\image html example_migration2.png "Library name change for FreeRTOS example"

similar change can be done on the CCS project as well

##### OS define on makefile and CCS projects
Additional macro OS_NORTOS or OS_FREERTOS should be defined on the makefile or CC project based on the OS of the project.

For NoRTOS/baremetal,

\imageStyle{example_migration3.png,width:20%}
\image html example_migration3.png "OS Macro addition for NoRTOS example"

For FreeRTOS,

\imageStyle{example_migration4.png,width:20%}
\image html example_migration4.png "OS Macro addition for FreeRTOS example"

### SDL PBIST Self test
VIM Memory groups are added to PBIST self test from this release. Because of this change, Self test (SDL_PBIST_selfTest) API has to be called in polling mode only and interrupt mode is not supported.

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
