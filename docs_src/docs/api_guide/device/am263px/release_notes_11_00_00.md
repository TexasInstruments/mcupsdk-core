# Release Notes 11.00.00 {#RELEASE_NOTES_11_00_00_PAGE}

[TOC]
\attention 1. Also refer to individual module pages for more details on each feature, unsupported features, important usage guidelines.

\attention 2. RPRC image format has been deprecated from this release. Multi Core ELF image format should be used. (\ref MCELF_LANDING).

\attention 3. The default Stack size is 16KB and Heap size 32 KB for SDK examples. This can be adjusted as per application requirement through Memory Configurator in SysCfg or by updating Linker script in case of standalone applications.

\attention 4. SDK has been migrated to CCS Theia from this release and the support for CCS Eclipse has been deprecated.

\attention 5. The default SysCfg linked to CCS is an older version and needs to updated to the SDK supported version mentioned below. Please follow steps mentioned in \ref CCS_PACKAGE_CHECK.

\attention 6. Uniflash 9.2.0 does not support out of the box flashing of AM263Px-CC Rev B and AM263Px-LP Rev A binaries. As a workaround, use Uniflash's custom flasher feature mentioned here \ref CUSTOM_FLASH. Out of box flashing support for these boards will be available in the next Uniflash release.

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
Board Level Sysconfig Support                                                                   | Sysconfig
McSPI External Loopback Example                                                                 | McSPI

# Modules Not tested/supported in this release

-

## Device and Validation Information

SOC    | Supported CPUs  | EVM                                                                          | Host PC
-------|-----------------|------------------------------------------------------------------------------|-----------------------------------------
AM263Px| R5F             | AM263Px ControlCard Rev B    (referred to as am263px-cc in code). \n         | Windows 10 64b or Ubuntu 18.04 64b or MacOS 
AM263Px| R5F             | AM263Px LaunchPad  Rev A    (referred to as am263px-lp in code). \n         | Windows 10 64b or Ubuntu 18.04 64b or MacOS


## Dependent Tools and Compiler Information

Tools                   | Supported CPUs | Version
------------------------|----------------|-----------------------
Code Composer Studio    | R5F            | 20.2.0
SysConfig               | R5F            | 1.24.2 build, build 4234
TI ARM CLANG            | R5F            | 4.0.3.LTS
FreeRTOS Kernel         | R5F            | 11.1.0
LwIP                    | R5F            | STABLE-2_2_1_RELEASE
Mbed-TLS                | R5F            | 2.13.1
Uniflash                | R5F            | 9.2.0


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
Smart Layout                                                                  | OptiFlash

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
Bootloader | R5FSS0-0        | YES               | NORTOS           | Boot modes: OSPI, UART. All R5F's. MCELF, multi-core image format     | -

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
MMCSD        | R5F            | YES               | NA                                    | MMCSD 4bit, Raw read/write, file IO, eMMC                                                                                                                                      | -
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
    <td> MCUSDK-14749
    <td> McSPI: Non Powers of 2 cannot be configured as fifo trigger levels in polling and interrupt mode
    <td> McSPI
    <td> 10.02.00 onwards
    <td> AM263x, AM263Px
    <td> Fix in SysCfg Meta file.
</tr>
<tr>
    <td> MCUSDK-13966
    <td> All UART triggers levels not exposed in SysCfg
    <td> UART
    <td> 10.00.00 onwards
    <td> AM263x, AM263Px
    <td> Update SysCfg to show all trigger levels from 1 to 64.
</tr>
<tr>
    <td> MCUSDK-14573
    <td> Incorrect handling of errata i2310 in UART isr
    <td> UART
    <td> 10.00.00 onwards
    <td> AM263x, AM263Px
    <td> Reorder the ISR state machine for handling UART errata correctly.
</tr>
<tr>
    <td> MCUSDK-14704
    <td> Adding multiple instances of UART DMA LLD causes failure
    <td> UART
    <td> 10.01.00 onwards
    <td> AM263x, AM263Px
    <td> SysCfg template update to pass the EDMA handle correctly
</tr>
<tr>
    <td> MCUSDK-14706
    <td> GPIO Qual selection API missing
    <td> GPIO
    <td> 10.00.00 onwards
    <td> AM263x, AM263Px
    <td> Qual sel API added in pinmux driver
</tr>
<tr>
    <td> MCUSDK-14569
    <td> UART Errata i2310 is missing a step
    <td> UART
    <td> 10.00.00 onwards
    <td> AM263x, AM263Px
    <td> Added IIR register read to clear the interrupt
</tr>
<tr>
    <td> MCUSDK-14620
    <td> SDK build fails in Mac Machines
    <td> Build
    <td> 10.02.00 onwards
    <td> AM263x, AM263Px
    <td> Added GMAC library for MAC into SDK
</tr>
<tr>
    <td> MCUSDK-14659
    <td> Incorrect RTI clock source mux address for RTI 4 to 7
    <td> RTI
    <td> 10.00.00 onwards
    <td> AM263Px
    <td> Updated to correct mux addresses in SysCfg
</tr>
<tr>
    <td> MCUSDK-13182
    <td> SysCfg unexpectedly changes OSPI Pin
    <td> OSPI
    <td> 10.00.00 onwards
    <td> AM263Px
    <td> The OSPI pins are locked in SDK examples.
</tr>
<tr>
    <td> MCUSDK-14857, MCUSDK-14731
    <td> Core 1 unhalted in SBL before FSM Trigger, Memory load
    <td> SBL
    <td> 10.00.00 onwards
    <td> AM263x, AM263Px
    <td> Skip unhalting core 1 of both clusters in dual core mode
</tr>
<tr>
    <td> MCUSDK-14712
    <td> OSPI Reset Pin being used before configuration
    <td> OSPI
    <td> 10.00.00 onwards
    <td> AM263Px
    <td> Configure OSPI reset in OSPI open instead of Flash open
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
    <td> PROC_SDL-9148
    <td> ECC D-Data fail during release
    <td> SDL
    <td> 10.02.00 onwards
    <td> AM263x, AM263Px
    <td> Due to the clang optimisation. So, added pragma to disable optimisation for error clear register.
</tr>
<tr>
    <td> MCUSDK-14695
    <td> SDFM_configComparator has incorrect input in examples
    <td> SDFM
    <td> 10.00.00 onwards
    <td> AM263x, AM263Px
    <td> Updated example to pass correct value
</tr>
<tr>
    <td> MCUSDK-13153
    <td> Self nesting of interrupts is not working
    <td> DPL
    <td> 09.01.00 onwards
    <td> AM263x, AM263Px
    <td> Added macros for handling self re-entrant IRQ
</tr>
<tr>
    <td> MCUSDK-11935
    <td> DPL Low Latency Interrupt Application: controlfnc section missing in linker command
    <td> DPL
    <td> 09.00.00 onwards
    <td> AM263x, AM263Px
    <td> Added missing .controlfnc section in linker command file of DPL Low Latency Interrupt example
</tr>
<tr>
    <td> MCUSDK-14696
    <td> ADC Sysconfig does not seem to generate codes for repeaters
    <td> ADC
    <td> 10.00.00 onwards
    <td> AM263Px
    <td> Fixed syscfg template file to generate trigger repeater code for burst mode
</tr>
<tr>
    <td> MCUSDK-14645
    <td> Implementation of the ADC_selectSOCExtChannel
    <td> ADC
    <td> 10.01.00 onwards
    <td> AM263Px
    <td> Fixed ADC_selectSOCExtChannel API implementation
</tr>
<tr>
    <td> MCUSDK-14746
    <td> TMU: Docs: QUAD feature listed under not supported section
    <td> TMU
    <td> 10.02.00 onwards
    <td> AM263Px
    <td> Updated documentation
</tr>
<tr>
    <td> MCUSDK-14661
    <td> Errata i2485 config missing in empty projects
    <td> TMU
    <td> 10.02.00 onwards
    <td> AM263Px
    <td> Updated example syscfg to have the part of TCMA blocked for R5SS1_CORE1
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
    <td> MCUSDK-14898
    <td> SDL apps fails on other than RFSS0-0 with SBL
    <td> SBL, SDL
    <td> 11.00.00 onwards
    <td> This because SBL brings the RFSS1-0 out of reset before the SBL UART prints gets flushed. This will be fixed in next release. As a workaround the application in R5FSS1-0 can delay the start of application till SBL UART prints gets completed.
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
    <td> Use standalone examples and for STC use sbl_null with syscfg enabled
</tr>
<tr>
    <td> PROC_SDL-8857
    <td> SDL integrated example does not support ECC Bus Safety.
    <td> SDL
    <td> 10.01.00 onwards
    <td> Use standalone example.
</tr>
<tr>
    <td> PROC_SDL-9154
    <td> VTM Example stuck in UC2
    <td> SDL
    <td> 10.02.00 onwards
    <td> Use debug profile
</tr>
<tr>
    <td> PROC_SDL-9163
    <td> ECC Aggregators FSS FOTA and OSPI
    <td> SDL
    <td> 10.02.00 onwards
    <td> None
</tr>
<tr>
    <td> MCUSDK-13652
    <td> Readelf throws warning while parsing RS note
    <td> SBL, QSPI
    <td> 10.00.00 onwards
    <td> -
</tr>
<tr>
    <td> MCUSDK-13513
    <td> AM263Px: UDP and TCP IPERF TX is unstable with 100Mbps link speed
    <td> Networking
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
    <td> MCUSDK-14647
    <td> All CANFD standard ID conigurations are not exposed in SysCfg
    <td> CAN
    <td> 10.02.00 onwards
    <td> Configure Config type, ID's,etc in application
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
    <td> MCUSDK-14851
    <td> Few parameters are incorrect in JEDEC table from ospi diag example
    <td> OSPI
    <td> 11.00.00 onwards
    <td> Refer flash datasheet and update
</tr>
<tr>
    <td> MCUSDK-14895
    <td> UART LLD Rx error checking logic checks if all errors exist at once
    <td> UART
    <td> 10.00.00 onwards
    <td> -
</tr>
<tr>
    <td> MCUSDK-13011
    <td> Data Abort in application when all cores are running freertos using Gel files(CCS)
    <td> FreeRTOS
    <td> 10.00.00 onwards
    <td> Flash and use SBL NULL instead of gel files
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
Here, to make this example with AM263PX-SIP board, please remove the <code> RUN_XIP_IN_PARALLEL </code> macro. 

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

### Migrating examples to 11.00.00 from older versions

\cond !SOC_AM64X
\note Images are shown for AM64x. It is application for @VAR_SOC_NAME as well.
\endcond

#### Makefile Changes
##### Library Name change on makefile and CCS projects
From 11.00.00 SDK all the libraries are built separately for OS. There are separate libraries available for NoRTOS and FreeROTS. 
So the makefiles needs to be updated accordingly. Please refer the sample changes on the makefile below. These changes are not applicbale for the 
librarries which were already built separately for NoRTOS/FreeRTOS like kernel libraries. 

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
