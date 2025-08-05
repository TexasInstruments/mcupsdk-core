# Release Notes 11.00.00 {#RELEASE_NOTES_11_00_00_PAGE}

[TOC]
\attention 1. Also refer to individual module pages for more details on each feature, unsupported features, important usage guidelines.

\attention 2. Multi Core ELF image format support has been added (\ref MCELF_LANDING). RPRC format will be deprecated from SDK 11.0.

\attention 3. The default Stack size is 16KB and Heap size 32 KB for SDK examples. This can be adjusted as per application requirement through Memory Configurator in SysCfg or by updating Linker script in case of standalone applications.

\attention 4. SDK will be migrated to support CCS Theia from next release (SDK 11.0) and the support for CCS Eclipse will be deprecated.

\attention 5. There is a known issue that OSPI pins in SysCfg GUI are getting reset automatically during any module change. The workaround is that OSPI Pins should be locked after proper configuration according to the board Pinout.

\attention 6. The default SysCfg linked to CCS is an older version and needs to updated to the SDK supported version mentioned below. Please follow steps mentioned in \ref CCS_PACKAGE_CHECK.

\note The examples will show usage of SW modules and APIs on a specific CPU instance and OS combination. \n
      Unless explicitly noted otherwise, the SW modules would work in both FreeRTOS and no-RTOS environment. \n
      Unless explicitly noted otherwise, the SW modules would work on any of the R5F's present on the SOC. \n

## New in this Release

Feature                                                                                         | Module
------------------------------------------------------------------------------------------------|-----------------------------------
                                                                                                | -

# Modules Not tested/supported in this release

- -

## Device and Validation Information

SOC    | Supported CPUs  | EVM                                                                          | Host PC
-------|-----------------|------------------------------------------------------------------------------|-----------------------------------------
AM263Px| R5F             | AM263Px ControlCard Rev A    (referred to as am263px-cc in code). \n         | Windows 10 64b or Ubuntu 18.04 64b
AM263Px| R5F             | AM263Px LaunchPad  Rev E2    (referred to as am263px-lp in code). \n         | Windows 10 64b or Ubuntu 18.04 64b


## Dependent Tools and Compiler Information

Tools                   | Supported CPUs | Version
------------------------|----------------|-----------------------
Code Composer Studio    | R5F            | 12.8.1
SysConfig               | R5F            | 1.23.0 build, build 4000
TI ARM CLANG            | R5F            | 4.0.3.LTS
FreeRTOS Kernel         | R5F            | 11.1.0
LwIP                    | R5F            | STABLE-2_2_0_RELEASE
Mbed-TLS                | R5F            | 2.13.1


## Key Features

### Experimental Features

\attention Features listed below are early versions and should be considered as "experimental".
\attention Users can evaluate the feature, however the feature is not fully tested at TI side.
\attention TI would not support these feature on public e2e.
\attention Experimental features will be enabled with limited examples and SW modules.

Feature                                                                       | Module
------------------------------------------------------------------------------|--------------------------
GUI for UART Uniflash Tool                                                    | Bootloader
Ether-ring Driver Implementation                                              | Networking
Ether-ring Demo with Real Time Traffic Generator and Background LwIP traffic  | Networking

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
Bootloader | R5FSS0-0        | YES               | NORTOS           | Boot modes: OSPI, UART. All R5F's. RPRC, MCELF, multi-core image format     | Force Dual Core Mode

### SOC Device Drivers

Peripheral   | Supported CPUs | SysConfig Support | DMA Supported                         | Key features tested                                                                                                                                             | Key features not tested / NOT supported
-------------|----------------|-------------------|---------------------------------------|-----------------------------------------------------------------------------------------------------------------------------------------------------------------|---------------------------------------------------
ADC, ADC_R   | R5F            | YES               | Yes. Examples:  adc_soc_continuous_dma, adc_alternate_dma_trigger | Single software triggered conversion, Multiple ADC trigger using PWM, Result read using DMA (normal and alternate triggers), EPWM trip through PPB limit, PPB features, Burst mode, Single and Differential mode, Interrupt with Offset from Aquisition Window, EPWM/ECAP/RTI triggered conversions, Trigger Repeater for Undersampling and Oversampling, Global Force on Multiple ADCs, Internal DAC Loopback to Calibration Channels, Safety Checker and Aggregator, Open Short Detection feature                 | External channel selection
Bootloader   | R5F            | YES               | Yes. DMA enabled for SBL OSPI         | Boot modes: OSPI, UART. All R5F's                                                                                                                               | -
CMPSS        | R5F            | YES               | NA                                    | Asynchronous PWM trip, digital filter, Calibration, Diode Emulation example                                                                                                                             | CMPSS Dac LoopBack feature
CPSW         | R5F            | YES               | No                                    | MAC loopback, PHY loopback, LWIP: Getting IP, Ping, Iperf, Layer 2 MAC, Layer 2 PTP Timestamping and Ethernet CPSW Switch support, TSN stack                      | RMII, MII mode
DAC          | R5F            | YES               | Yes. Example: dac_sine_dma            | Constant voltage, Square wave generation, Sine wave generation with and without DMA, Ramp wave generation, Random Voltage generation                            | -
ECAP         | R5F            | YES               | yes. Example : ecap_edma              | ECAP APWM mode, PWM capture, DMA trigger in both APWM and Capture Modes, ecap signal monitoring example                                                         | -
EDMA         | R5F            | YES               | NA                                    | DMA transfer using interrupt and polling mode, QDMA Transfer, Channel Chaining, PaRAM Linking, Error Handling                                                                   | -
EPWM         | R5F            | YES               | Yes. Example: epwm_dma, epwm_xcmp_dma | Multiple EPWM Sync from Top Module, PWM outputs A and B in up-down count mode, Trip zone, Update PWM using EDMA, Valley switching, High resolution time period adjustment, chopper module features, type5 features           | -
EQEP         | R5F            | YES               | NA                                    | Speed and Position measurement. Frequency Measurement, Speed and Direction Measurement, cw-ccw modes                                                                                                                              | -
FSI          | R5F            | YES               | Yes. Example: fsi_loopback_dma        | RX, TX, polling, interrupt mode, Dma, single lane loopback.                                                                                                     | - FSI Spi Mode
GPIO         | R5F            | YES               | NA                                    | Output, Input and Interrupt functionality                                                                                                                       | -
I2C          | R5F            | YES               | No                                    | Controller mode, basic read/write                                                                                                                               | -
IPC Notify   | R5F            | YES               | NA                                    | Mailbox functionality, IPC between RTOS/NORTOS CPUs                                                                                                             | M4F core
IPC Rpmsg    | R5F            | YES               | NA                                    | RPMessage protocol based IPC                                                                                                                                    | M4F core
LIN          | R5F            | YES               | YES                                   | RX, TX, polling, interrupt, DMA mode.                                                                                                                           | -
MCAN         | R5F            | YES               | No                                    | RX, TX, interrupt and polling mode, Corrupt Message Transmission Prevention, Error Passive state, Bus Off State, Bus Monitoring Mode                            | -
MCSPI        | R5F            | YES               | Yes. Example: mcspi_loopback_dma      | Controller/Peripheral mode, basic read/write, polling, interrupt and DMA mode                                                                                   | -
MDIO         | R5F            | YES               | NA                                    | Register read/write, link status and link interrupt enable API                                                                                                  | -
MMCSD        | R5F            | YES               | NA                                    | MMCSD 4bit, Raw read/write                                                                                                                                      | file IO, eMMC
PINMUX       | R5F            | YES               | NA                                    | Tested with multiple peripheral pinmuxes                                                                                                                        | -
PMU          | R5F            | NO                | NA                                    | Tested various PMU events                                                                                                                                       | Counter overflow detection is not enabled
OptiFlash    | R5F            | Yes               | NA                                    | FLC, RL2, RAT functionality, XIP with RL2 enabled, OTFA, FOTA, Optishare, Smart Layout                                                                                                  | -
OSPI         | R5F            | YES               | Yes. Example: ospi_flash_dma          | Read direct, Write indirect, Read/Write commands, DMA for read                                                                                                  | -
RTI          | R5F            | YES               | No                                    | Counter read, timebase selection, comparator setup for Interrupt, DMA requests                                                                                  | Capture feature, fast enabling/disabling of events not tested
RESOLVER     | R5F            | YES               | No                                    | Angle and Speed Calcution. input Band Pass Filter, Manual Phase Gain Correction and Manual Ideal Sample Selection Mode calculation, Non-Rotational Safety Diagnostic features, Dual motor/Single motor redundant sensing                                                                                                | Tuning, Rotational Safety Diagnostic features
SDFM         | R5F            | YES               | yes. Example : sdfm_filter_sync_dmaread | Filter data read from CPU, Filter data read with PWM sync, triggered DMA read from the Filter FIFO, ECAP Clock LoopBack                                                                                                       | -
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
PMIC       | R5F            | YES               | LDO Voltage control                                         | -
IOEXPANDER | R5F            | YES               | IO configurability                                          | -

### Networking

Module                      | Supported CPUs | SysConfig Support | OS Support  | Key features tested                                                                    | Key features not tested
----------------------------|----------------|-------------------|-------------|----------------------------------------------------------------------------------------|------------------------
Time-Sensitive Networking(gPTP-IEEE 802.1AS) | R5F            | NO                | FreeRTOS    | gPTP IEEE 802.1 AS-2020 compliant gPTP stack, End Nodes and Bridge mode support, YANG data model configuration, IEEE 1722 compliant AVTP Stack  | Multi-Clock Domain
LwIP                                         | R5F            | YES               | FreeRTOS    | TCP/UDP IP networking stack with and without checksum offload enabled, TCP/UDP IP networking stack with server and client functionality, basic Socket APIs, netconn APIs and raw APIs, DHCP, ping, TCP iperf, scatter-gather, DSCP priority mapping                         | Other LwIP features
Ethernet driver (ENET)                       | R5F            | YES               | FreeRTOS    | Ethernet as port using CPSW, MAC loopback and PHY loopback, Layer 2 MAC, Packet Timestamping, CPSW Switch, CPSW EST, interrupt pacing, Policer and Classifier, MDIO Manual Mode, Credit Based Shaper (IEEE 802.1Qav), Strapped PHY (Early Ethernet)  | RMII, MII mode
ICSS-EMAC                   | R5F            | YES               | FreeRTOS    | Switch and MAC features, Storm Prevention (MAC), Host Statistics, Multicast Filtering  | Promiscuous Mode
Ether-ring Implementation | R5F            | NO                | FreeRTOS    | Duplicate Rejection, Ring termination and Packet Duplication, Latency measurement for different real-time traffic profiles, Performance KPIs | N/A

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

**Note**: SDL is validated only on ControlCard.

### PRU IO

Module          | Supported CPUs    | SysConfig Support | OS Support        | Key features tested                                                               | Key features not tested
----------------|-------------------|-------------------|-------------------|-----------------------------------------------------------------------------------|-------------------------------------------------
Empty           | PRU               | YES                | Bare Metal        | Empty project to get started with PRU firmware development                        | -


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
    <td> -
    <td> -
    <td> -
    <td> -
    <td> -
    <td> -
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
    <td> MCUSDK-13865
    <td> HRPWM Deadband sfo example has 1ns jitter
    <td> EPWM
    <td> 10.00.00 onwards
    <td> -
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
    <td> MCUSDK-13011
    <td> Multicore Empty project not working properly
    <td> FreeRTOS
    <td> 09.01.00 onwards
    <td> -
</tr>
<tr>
    <td> PINDSW-7715
    <td> Dual EMAC instance not working with both ports together for icss_emac_lwip example
    <td> ICSS-EMAC
    <td> 09.02.00 onwards
    <td> None
</tr>
<tr>
    <td> PINDSW-7746
    <td> Low iperf values in TCP and UDP
    <td> ICSS-EMAC
    <td> 09.02.00 onwards
    <td> None
</tr>
<tr>
    <td> PINDSW-8118
    <td> Enabling DHCP mode in icss_emac_lwip example causes assert
    <td> ICSS-EMAC
    <td> 09.02.00 onwards
    <td> None
</tr>
<tr>
    <td> MCUSDK-12756
    <td> MbedTLS - Timing side channel attack in RSA private operation exposing plaintext.
    <td> Mbed-TLS
    <td> 08.06.00 onwards
    <td> None
</tr>
<tr>
    <td> PROC_SDL-8392
    <td> In ECC bus safety example, ECC error is not properly cleared at the source.
    <td> SDL
    <td> 09.00.00 onwards
    <td> None
</tr>
<tr>
    <td> PROC_SDL-8787
    <td> ECC TPTC and STC examples are not supported in SDL integrated example.
    <td> SDL
    <td> 10.01.00 onwards
    <td> Use standalone examples.
</tr>
<tr>
    <td> PROC_SDL-8857
    <td> SDL integrated example does not support ECC Bus Safety.
    <td> SDL
    <td> 10.01.00 onwards
    <td> Use standalone example.
</tr>
<tr>
    <td> PROC_SDL-9148
    <td> ECC D-Data fail during release
    <td> SDL
    <td> 10.02.00 onwards
    <td> None
</tr>
<tr>
    <td> PROC_SDL-9149
    <td> ECC D-Tag stuck in System_Init during release R5FSS1-0
    <td> SDL
    <td> 10.02.00 onwards
    <td> None
</tr>
<tr>
    <td> PROC_SDL-9150
    <td> PBIST Example and Test apps are getting mixed data with bootloader data in R5FSS1-0
    <td> SDL
    <td> 10.02.00 onwards
    <td> Functionality works and log gets mixed. Disable DPL_Log.
</tr>
<tr>
    <td> PROC_SDL-9151
    <td> CCM Example and Test apps are getting mixed data with bootloader data in R5FSS1-0
    <td> SDL
    <td> 10.02.00 onwards
    <td> Functionality works and log gets mixed. Disable DPL_Log.
</tr>
<tr>
    <td> PROC_SDL-9152
    <td> R5F Utils Example and Test apps are getting mixed data with bootloader data
    <td> SDL
    <td> 10.02.00 onwards
    <td> Functionality works and log gets mixed. Disable DPL_Log.
</tr>
<tr>
    <td> PROC_SDL-9153
    <td> TOG Example stuck in System_Init during release
    <td> SDL
    <td> 10.02.00 onwards
    <td> None
</tr>
<tr>
    <td> PROC_SDL-9154
    <td> VTM Example stuck in UC2
    <td> SDL
    <td> 10.02.00 onwards
    <td> None
</tr>
<tr>
    <td> PROC_SDL-9163
    <td> ECC Aggregators FSS FOTA and OSPI
    <td> SDL
    <td> 10.02.00 onwards
    <td> None
</tr>
<tr>
    <td> PROC_SDL-9179
    <td> Redefinition error in MCU_PBIST Sysconfig
    <td> SDL
    <td> 10.02.00 onwards
    <td> Use sdl examples instead of sbl_null to enable MCU_PBIST SYSCFG and test.
</tr>
<tr>
    <td> MCUSDK-13652
    <td> Readelf throws warning while parsing RS note
    <td> SBL, QSPI
    <td> 10.00.00 onwards
    <td> -
</tr>
<tr>
    <td> MCUSDK-13182
    <td> SysCfg unexpectedly changes OSPI Pin
    <td> OSPI
    <td> 10.00.00 onwards
    <td> Lock the OSPI Pins in SysCfg.
</tr>
<tr>
    <td> MCUSDK-14110
    <td> Error building examples in CCS in mac
    <td> Infra
    <td> Example build fails in CCS only in MAC Machines
    <td> \ref CCS_MAC_ISSUE
</tr>
<tr>
    <td> MCUSDK-13513
    <td> AM263Px: UDP and TCP IPERF TX is unstable with 100Mbps link speed
    <td> Networking
    <td> 10.00.00 onwards
    <td> -
</tr>
<tr>
    <td> MCUSDK-14473
    <td> AM263Px: Multiple chip selects cannot be configured in SysCfg
    <td> OSPI
    <td> 10.00.00 onwards
    <td> -
</tr>
<tr>
    <td> MCUSDK-14509
    <td> AM263x/Am263px/AM261x: 10% Packet drop with UDP iperf in 100M bandwidth in 1Gbps FullDuplex linkspeed
    <td> Networking
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
    <td> MCUSDK-14102
    <td> Applications > 1MB not flashing using TI Uniflash tool
    <td> Uniflash tool
    <td> 10.00.00 onwards
    <td> \ref UNIFLASH_1MB_ISSUE
</tr>
<tr>
    <td> MCUSDK-14547
    <td> XIP Flashing not supported in SBL JTAG Uniflash example
    <td> SBL
    <td> 10.00.00 onwards
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
    <td> i2401
    <td> CPSW: Host Timestamps Cause CPSW Port to Lock up
    <td> CPSW
    <td> Open
</tr>
<tr>
    <td> i2404
    <td> Race condition in mailbox registers resulting in events miss
    <td> IPC, Mailbox
    <td> Implemented
</tr>
<tr>
    <td> i2405
    <td> CONTROLSS: Race condition OUTPUT_XBAR and PWM_XBAR resulting in event miss
    <td> Crossbar
    <td> Open
</tr>
<tr>
    <td> i2485
    <td> AM263PX: TMU: TCM Memory Corruption on R5SS0_CORE1 and R5SS1_CORE1 when writing to TMU Registers
    <td> TMU
    <td> Implemented a workaround \ref TMU_TCMA_ERRATA <br> Workaround: Do not use initial bytes (0x40-0x3A0) of ATCM from CPU1 allocation. Initial bytes (0x40-0x3A0 => 868 bytes) of CORE1 TCM are blocked using linker command settings of multi-core application/examples. <br> Refer \ref EXAMPLES_DRIVERS_TMU_CORES_SUPPORT
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
    <td> MCUSDK-13630
    <td> Cache should not be enabled at last 32B L2 Bank boundary
    <td> Cache
    <td> 10.01.00
    <td> Create MPU configurations for last 32B of each L2 Bank with Non Cached attribute
</tr>
</table>

## Upgrade and Compatibility Information
### How to run FOTA examples on AM263PX-SIP board

One difference between AM263PX-SIP and AM263PX is the in package flash. In SIP board, flash is of Non-RWW in nature whereas, it is of RWW in other case.
Example \ref EXAMPLES_FLSOPSKD_BENCHMARK is made to work out of box for AM263PX board but need some manual changes to make it work on AM263PX-SIP board.
Here, to make this example with AM263PX-SIP board, please removed the <code> RUN_XIP_IN_PARALLEL </code> macro. 

### Compiling examples in MacOS machines

Currently the gmac library packaged within SDK is compiled using gcc darwin23.6.0 on Apple MAC.
To build SDK examples on a different toolchain, recompile the gmac library by using these steps:

\code
    $ cd {SDK_PATH}/tools/boot/multicore-elf/c_modules/gmac
    $ make all
\endcode

- A new library will be created inside mac/dist. 
- Rename this file to "gmac.arm64-apple-darwin.darwin.dylib". 

### Compiler Options

<table>
<tr>
    <th> Module
    <th> Affected API
    <th> Change
    <th> Additional Remarks
</tr>
<tr>
    <th> -
    <th> -
    <th> -
    <th> -
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
    <th> -
    <th> -
    <th> -
    <th> -
</tr>
</table>

### Ethernet and Networking

<table>
<tr>
    <th> Module
    <th> Affected API
    <th> Change
    <th> Additional Remarks
</tr>
<tr>
    <th> -
    <th> -
    <th> -
    <th> -
</tr>
</table>
