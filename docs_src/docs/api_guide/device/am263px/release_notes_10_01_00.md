# Release Notes 10.01.00 {#RELEASE_NOTES_10_01_00_PAGE}

[TOC]
\attention 1. Also refer to individual module pages for more details on each feature, unsupported features, important usage guidelines.

\attention 2. Multi Core ELF image format support has been added (\ref MCELF_LANDING). RPRC format will be deprecated from SDK 11.0.

\attention 3. Test report is not uploaded in SDK due to an issue with the report generation tool.

\note The examples will show usage of SW modules and APIs on a specific CPU instance and OS combination. \n
      Unless explicitly noted otherwise, the SW modules would work in both FreeRTOS and no-RTOS environment. \n
      Unless explicitly noted otherwise, the SW modules would work on any of the R5F's present on the SOC. \n

\attention Current PMIC support in SDK is bare minimum meant to power up the modules and should not be used beyond this including safety use-case etc

## New in this Release

Feature                                                                                         | Module
------------------------------------------------------------------------------------------------|-----------------------------------
Support added for Ethernet PHY Configuration for ENET (CPSW) in SysConfig Gui                   | Ethernet And Networking
Enablement of ethernet Ether-Ring Feature                                                       | Ethernet And Networking
Application load over Ethernet to external Flash (SBL Enet) example is added                    | SBL
Pinmux CSV Generation support in SysCfg                                                         | Pinmux
FreeRTOS FAT Example support                                                                    | FreeRTOS
FSI LLD Support                                                                                 | FSI
OSPI LLD Support                                                                                | OSPI
FreeRTOS Kernel migrated to 11.1.0 LTS                                                          | FreeRTOS
Optishare Tool Support                                                                          | Optiflash
OTFA Safety ECC Support                                                                         | Optiflash
FOTA with XIP Example                                                                           | Optiflash

# Modules Not tested/supported in this release

- -

## Device and Validation Information

\cond SOC_AM263PX
SOC    | Supported CPUs  | EVM                                                                          | Host PC
-------|-----------------|------------------------------------------------------------------------------|-----------------------------------------
AM263Px| R5F             | AM263Px ControlCard E2 Rev     (referred to as am263Px-cc in code). \n       | Windows 10 64b or Ubuntu 18.04 64b
AM263Px| R5F             | AM263Px LaunchPad              (referred to as am263Px-lp in code). \n       | Windows 10 64b or Ubuntu 18.04 64b
\endcond


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
GUI for UART Uniflash Tool                                          | Bootloader
Ether-ring Feature                                                  | Ethernet And Networking

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
EDMA         | R5F            | YES               | NA                                    | DMA transfer using interrupt and polling mode, QDMA Transfer, Channel Chaining, PaRAM Linking                                                                   | -
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
OptiFlash    | R5F            | Yes               | NA                                    | FLC, RL2, RAT functionality, XIP with RL2 enabled                                                                                                               | OptiShare
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

### Networking

Module                      | Supported CPUs | SysConfig Support | OS Support  | Key features tested                                                                    | Key features not tested
----------------------------|----------------|-------------------|-------------|----------------------------------------------------------------------------------------|------------------------
Time-Sensitive Networking(gPTP-IEEE 802.1AS) | R5F            | NO                | FreeRTOS    | gPTP IEEE 802.1 AS-2020 compliant gPTP stack, End Nodes and Bridge mode support, YANG data model configuration  | Multi-Clock Domain
LwIP                                         | R5F            | YES               | FreeRTOS    | TCP/UDP IP networking stack with and without checksum offload enabled, TCP/UDP IP networking stack with server and client functionality, basic Socket APIs, netconn APIs and raw APIs, DHCP, ping, TCP iperf, scatter-gather, DSCP priority mapping                         | Other LwIP features
Ethernet driver (ENET)                       | R5F            | YES               | FreeRTOS    | Ethernet as port using CPSW, MAC loopback and PHY loopback, Layer 2 MAC, Packet Timestamping, CPSW Switch, CPSW EST, interrupt pacing, Policer and Classifier, MDIO Manual Mode, Credit Based Shaper (IEEE 802.1Qav), Strapped PHY (Early Ethernet)  | RMII, MII mode
ICSS-EMAC                   | R5F            | YES               | FreeRTOS    | Switch and MAC features, Storm Prevention (MAC), Host Statistics, Multicast Filtering  | Promiscuous Mode
Ether-ring Implementation | R5F            | NO                | FreeRTOS    | Duplicate Rejection, Ring termination and Packet Duplication | Latency measurement, Performance KPIs

<!-- Mbed-TLS                    | R5F            | NO                | FreeRTOS    | Tested software cryptography after porting, used mbedTLS with LwIP to implement HTTPS server  | Hardware offloaded cryptography -->

### Safety Diagnostic Library

Module            | Supported CPUs  | SysConfig Support | OS support       | Key features tested                                                                            | Key features not tested / NOT supported
------------------|-----------------|-------------------|------------------|------------------------------------------------------------------------------------------------|----------------------------------------
MCRC              | R5F             | NA                |  NORTOS | Full CPU, Auto CPU Mode and Semi CPU Auto Mode                                                          | -
DCC               | R5F             | NA                |  NORTOS | Single Shot and Continuous modes                                    | -
PBIST             | R5F             | NA                |  NORTOS | Memories supported by MSS PBIST controller.          | -
ESM               | R5F             | NA                |  NORTOS | Tested in combination with RTI, DCC                                        | -
RTI               | R5F             | NA                |  NORTOS | WINDOWSIZE_100_PERCENT, WINDOWSIZE_50_PERCENT ,Latency/Propagation timing error(early)(50% window),Latency/Propagation timing error(late)(50% window)                                     | -
ECC               | R5F             | NA                |  NORTOS | ECC of MSS_L2, R5F TCM, MCAN, VIM, ICSSM, TPTC      | R5F Cache - DED
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
    <td>PROC_SDL-7615
    <td> ECC example fails for SEC and DED for TPTC memories.
    <td> SDL
    <td> 09.00.00 Onwards
    <td> AM263x, AM263Px
    <td> Fixed the example.
</tr>
<tr>
    <td>PROC_SDL-8393
    <td> In ECC bus safety, error injection test writes to address 0x0.
    <td> SDL
    <td> 09.01.00 Onwards
    <td> AM263x, AM263Px
    <td> Fixed the source code and example.
</tr>
<tr>
    <td> MCUSDK-13821
    <td> ADC reference monitor instance doesn't match the reference buffer instance
    <td> ADC
    <td> 10.00.00
    <td> AM263x, AM263Px
    <td> Update the monitor instances.
</tr>
<tr>
    <td> MCUSDK-12262
    <td> EPWM deadband example failure
    <td> EPWM
    <td> 09.02.00
    <td> AM263x, AM263Px
    <td> removed sync between the epwms and used the global tbclksync to synchronize the EPWMs
</tr>
<tr>
    <td> MCUSDK-13164
    <td> AM26x: EPWM DeadBand example failure
    <td> EPWM
    <td> 10.00.00 onwards
    <td> phase shift adds a tbclk delay. added another EPWM instance to sync.
</tr>
<tr>
    <td> MCUSDK-12265
    <td> SDFM example failure on am263px-lp
    <td> SDFM
    <td> 09.01.00
    <td> AM263Px
    <td> GEL updates on new CCS version, Previously working only with SBL
</tr>
<tr>
    <td> MCUSDK-13634
    <td> EPWM: Remove eventsUsed from Action Qualifier Syscfg.
    <td> EPWM 
    <td> 10.00.00 onwards
    <td> removed unused eventUsed element from the examples syscfg 
</tr>
<tr>
    <td> MCUSDK-13670
    <td> SDFM ECAP loopback example used explicit HW_REG_RD
    <td> SDFM 
    <td> 10.00.00 onwards
    <td> updated the register read with corresponding API.
</tr>
<tr>
    <td> MCUSDK-13641, CODEGEN-12832
    <td> Increased build time for examples using Link Time Optimization (-flto) with TI-ARM-CLANG 4.0.0 LTS
    <td> Build
    <td> 10.00.00 onwards
    <td> Issue fixed in latest 4.0.1 LTS compiler release.
</tr>
<tr>
    <td> MCUSDK-13727
    <td> OSPI Phy Tuning not working on AM263P LP Board with ISSI Flash
    <td> OSPI
    <td> 09.02.00 onwards
    <td> Fixed in E2 revision of AM263P LP board. For E1 board, board modification is needed for DQS and LBK signals as mentioned in the Board User guide.
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
    <td> MCUSDK-14051
    <td> EQEP : CW CCW example doesn't use polling or interrupt
    <td> EQEP
    <td> 10.00.01
    <td> None
</tr>
<tr>
    <td> MCUSDK-12312
    <td> ROM bootloader fails when booting from Macronix Flash on AM263Px-LP
    <td> SBL
    <td> 09.01.00
    <td> Use UART/CCS Boot
</tr>
<tr>
    <td> MCUSDK-12313
    <td> OSPI Phy Tuning not working on Macronix Flash on AM263Px-LP
    <td> SBL
    <td> 09.01.00
    <td> Use UART/CCS Boot
</tr>
<tr>
    <td> MCUSDK-12289
    <td> 32 KB TCM is used in examples sysconfig MPU module, size should be 64 KB
    <td> Common
    <td> 09.01.00
    <td> Use 64KB TCM in user application
</tr>
<tr>
    <td> MCUSDK-12265
    <td> SDFM example failure on am263px-lp
    <td> SDFM
    <td> 09.01.00
    <td> None
</tr>
<tr>
    <td> MCUSDK-11675
    <td> INDAC write only works if MPU for flash is only configured as Strongly-ordered
    <td> OSPI
    <td> 09.01.00
    <td> None
</tr>
<tr>
    <td> MCUSDK-13111
    <td> Memory Configurator/syscfg auto-linker generator doesn't support reordering
    <td> Build
    <td> 09.01.00 onwards
    <td> -
</tr>
<tr>
    <td> MCUSDK-13109
    <td> RTI Interrupt req is pulse type and not level type
    <td> RTI
    <td> 09.01.00 onwards
    <td> -
</tr>
<tr>
    <td> MCUSDK-13014
    <td> The memory read feature of uniflash erases the memory
    <td> Flash
    <td> 09.01.00 onwards
    <td> -
</tr>
<tr>
    <td> MCUSDK-13011
    <td> Multicore Empty project not working properly
    <td> FreeRTOS
    <td> 09.01.00 onwards
    <td> -
</tr>
<tr>
    <td> MCUSDK-12986
    <td> FreeRTOS: Barrier instructions missing in Interrupt Disable/Enable API's
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
    <td> PROC_SDL-8519
    <td> In ECC for R5F data cache only, double bit test is not supported.
    <td> SDL
    <td> 10.01.00 onwards
    <td> None.
</tr>
<tr>
    <td> PROC_SDL-8857
    <td> SDL integrated example does not support ECC Bus Safety.
    <td> SDL
    <td> 10.01.00 onwards
    <td> Use standalone example.
</tr>
<tr>
    <td> PROC_SDL-8859
    <td> STC example does not support R5FSS0.
    <td> SDL
    <td> 10.01.00 onwards
    <td> Update the example to run the test for R5FSS0 from R5FSS1.
</tr>
<tr>
    <td> PROC_SDL-8864
    <td> D-tag and D-data ECC examples fail in release profile.
    <td> SDL
    <td> 10.01.00 onwards
    <td> None.
</tr>
<tr>
    <td> PROC_SDL-8866
    <td> BTCM ECC example fails in release profile..
    <td> SDL
    <td> 10.01.00 onwards
    <td> Use debug profile.
</tr>
<tr>
    <td> MCUSDK-13473
    <td> UART Uniflash script fails with images > 1MB
    <td> SBL
    <td> 09.02.00 onwards
    <td> Use JTAG based flashing
</tr>
<tr>
    <td> MCUSDK-13013
    <td> OSPI missing ECC_FAIL signal
    <td> OSPI
    <td> 09.02.00 onwards
    <td> Configure pinmux for ECC_FAIL in application
</tr>
<tr>
    <td> MCUSDK-13517
    <td> FOTA firmware authentication does not happen on-the-fly
    <td> FOTA
    <td> 09.02.00 onwards
    <td> -
</tr>
<tr>
    <td> MCUSDK-13660
    <td> Watchdog interrupt example not working
    <td> Watchdog
    <td> 09.02.00 onwards
    <td> -
</tr>
<tr>
    <td> MCUSDK-12140
    <td> Flash Driver does not handle repeated id in read id
    <td> Flash
    <td> 09.02.00 onwards
    <td> -
</tr>
<tr>
    <td> MCUSDK-12703
    <td> Multicore XIP Fails
    <td> OptiFlash
    <td> 09.02.00 onwards
    <td> -
</tr>
<tr>
    <td> MCUSDK-13494
    <td> RL2 syscfg allows arbitrary flash size ranges
    <td> SBL
    <td> 09.02.00 onwards
    <td> Access only permitted space from application
</tr>
<tr>
    <td> MCUSDK-13495
    <td> Applications with XIP sections more than 1MB fails to boot
    <td> Flash, SBL
    <td> 09.02.00 onwards
    <td> -
</tr>
<tr>
    <td> MCUSDK-13375
    <td> "FOTA STIG Read" fails when pipeline is enabled
    <td> OptiFlash
    <td> 09.02.00 onwards
    <td> Disable pipeline during XIP
</tr>
<tr>
    <td> MCUSDK-13630
    <td> Cache should not be enabled at L2 Bank boundaries
    <td> Cache
    <td> 09.02.00 onwards
    <td> Create MPU configurations for end of each L2 Bank with Non Cached attribute
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
    <td> Reconfigure OSPI Pins to original state after updating OSPI configurables.
</tr>
<tr>
    <td> MCUSDK-13874
    <td> Syscfg load json function for flash configuration imports does not work
    <td> OSPI
    <td> Load flash json button action never completes, it just keeps loading.
    <td> -
</tr>
<tr>
    <td> MCUSDK-14110
    <td> Error building examples in CCS in mac
    <td> Infra
    <td> Example build fails in CCS only in MAC Machines
    <td> Use makefile based build
</tr>
<tr>
    <td> MCUSDK-13513
    <td> AM263Px: UDP and TCP IPERF TX is unstable with 100Mbps link speed
    <td> Networking
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
    <td> Open
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
    <td> i2350
    <td> McSPI data transfer using EDMA in 'ABSYNC' mode stops after 32 bits transfer
    <td> McSPI
    <td> Open
</tr>
<tr>
    <td> i2351
    <td> OSPI: Controller does not support Continuous Read mode with NAND Flash
    <td> OSPI
    <td> Open
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
    <td> i2383
    <td> OSPI: 2-byte address is not supported in PHY DDR mode
    <td> OSPI
    <td> Open
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
    <td> Cache should not be enabled at L2 Bank boundaries
    <td> Cache
    <td> 09.02.00 onwards
    <td> Create MPU configurations for end of each L2 Bank with Non Cached attribute
</tr>
<tr>
    <td> MCUSDK-13630
    <td> Cache should not be enabled at L2 Bank boundaries
    <td> Cache
    <td> 09.02.00 onwards
    <td> Create MPU configurations for end of each L2 Bank with Non Cached attribute
</tr>
<tr>
    <td> MCUSDK-13220
    <td> Multi Core Elf format has few limitations documented at \ref MCELF_LANDING
    <td> Infra
    <td> 10.00.00 onwards
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
    <th> Resolver
    <th> RDC_getDiagnosticsSinCosGainDriftData, RDC_setDiagnosticsSinCosGainDriftData
    <th> structure udpate Diag_Mon_SinCos_Gain_drift_data
    <th> threshold elements to type uint16_t from int16_t, gain_drift_en element name is updated to match.
</tr>
<tr>
    <th> Resolver
    <th> RDC_init
    <th> structure udpate Core_config_t
    <th> gainByass elements to type uint16_t from int16_t 
</tr>
<tr>
    <th> Resolver
    <th> RDC_getCalibrationData
    <th> return type changed form uint32_t to uint16_t
    <th> returns the 16 bit ADC sample data, hence the change
</tr>
<tr>
    <th> Resolver
    <th> RDC_enableDcOffsetAutoCorrection
    <th> Active low bit action update
    <th> - 
</tr>
<tr>
    <th> Resolver
    <th> RDC_getGainEstimation
    <th> return type changed form int16_t* to float*
    <th> Gain value is returned as gain squared. Please refer the API description for more details
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
    <th> Networking and Ethernet
    <th> None
    <th> Ethernet (CPSW) Examples are moved to a new diretory to source/networking/enet/core/examples from examples/networking
    <th> -
</tr>
<tr>
    <th> Networking and Ethernet
    <th> Ethernet PHY APIs
    <th> Ethernet PHY driver source code is moved to board driver component. The corresponding location is moved to source/board/ethphy/enet from source/networking/enet/core/src/phy
    <th> -
</tr>
</table>
