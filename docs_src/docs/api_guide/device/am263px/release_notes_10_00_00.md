# Release Notes 10.00.00 {#RELEASE_NOTES_10_00_00_PAGE}

[TOC]

\attention Also refer to individual module pages for more details on each feature, unsupported features, important usage guidelines.

\note The examples will show usage of SW modules and APIs on a specific CPU instance and OS combination. \n
      Unless explicitly noted otherwise, the SW modules would work in both FreeRTOS and no-RTOS environment. \n
      Unless explicitly noted otherwise, the SW modules would work on any of the R5F's present on the SOC. \n

\attention Current PMIC support in SDK is bare minimum meant to power up the modules and should not be used beyond this including safety use-case etc

\note CSP 12.7 or beyond needs to be used for XIP load/run from CCS. Refer \ref CSP_UPDATE for installation steps.

## New in this Release

Feature                                                                                         | Module
------------------------------------------------------------------------------------------------|-----------------------------------
Sysconfig support for PRU Projects                                                              | PRUICSS

# Modules Not tested/supported in this release


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
Code Composer Studio    | R5F            | 12.7.0
SysConfig               | R5F            | 1.20.0 build, build 3587
TI ARM CLANG            | R5F            | 3.2.2.LTS
FreeRTOS Kernel         | R5F            | 10.4.3
LwIP                    | R5F            | STABLE-2_2_0_RELEASE
Mbed-TLS                | R5F            | mbedtls-3.0.0


## Key Features

### Experimental Features

\attention Features listed below are early versions and should be considered as "experimental".
\attention Users can evaluate the feature, however the feature is not fully tested at TI side.
\attention TI would not support these feature on public e2e.
\attention Experimental features will be enabled with limited examples and SW modules.

Feature                                                             | Module
--------------------------------------------------------------------|--------------------------
GUI for UART Uniflash Tool                                          | Bootloader

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
ADC, ADC_R   | R5F            | YES               | Yes. Examples:  adc_soc_continuous_dma, adc_alternate_dma_trigger | Single software triggered conversion, Multiple ADC trigger using PWM, Result read using DMA (normal and alternate triggers), EPWM trip through PPB limit, PPB features, Burst mode, Single and Differential mode, Interrupt with Offset from Aquisition Window, EPWM/ECAP/RTI triggered conversions, Trigger Repeater for Undersampling and Oversampling, Global Force on Multiple ADCs, Internal DAC Loopback to Calibration Channels, Safety Checker and Aggregator, Open Short Detection feature                 | External channel selection
Bootloader   | R5F            | YES               | Yes. DMA enabled for SBL OSPI         | Boot modes: OSPI, UART. All R5F's                                                                                                                               | -
CMPSS        | R5F            | YES               | NA                                    | Asynchronous PWM trip, digital filter                                                                                                                                           | CMPSS Dac LoopBack feature
CPSW         | R5F            | YES               | No                                    | MAC loopback, PHY loopback, LWIP: Getting IP, Ping, Iperf, Layer 2 MAC, Layer 2 PTP Timestamping and Ethernet CPSW Switch support, TSN stack                      | RMII, MII mode
DAC          | R5F            | YES               | Yes. Example: dac_sine_dma            | Constant voltage, Square wave generation, Sine wave generation with and without DMA, Ramp wave generation, Random Voltage generation                            | -
ECAP         | R5F            | YES               | yes. Example : ecap_edma              | ECAP APWM mode, PWM capture, DMA trigger in both APWM and Capture Modes                                                                                         | -
EDMA         | R5F            | YES               | NA                                    | DMA transfer using interrupt and polling mode, QDMA Transfer, Channel Chaining, PaRAM Linking                                                                   | -
EPWM         | R5F            | YES               | Yes. Example: epwm_dma, epwm_xcmp_dma | Multiple EPWM Sync from Top Module, PWM outputs A and B in up-down count mode, Trip zone, Update PWM using EDMA, Valley switching, High resolution time period adjustment, chopper module features, type5 features           | -
EQEP         | R5F            | YES               | NA                                    | Speed and Position measurement. Frequency Measurement                                                                                                           | -
FSI          | R5F            | YES               | Yes. Example: fsi_loopback_dma        | RX, TX, polling, interrupt mode, Dma, single lane loopback.                                                                                                     | - FSI Spi Mode
GPIO         | R5F            | YES               | NA                                    | Output, Input and Interrupt functionality                                                                                                                       | -
I2C          | R5F            | YES               | No                                    | Controller mode, basic read/write                                                                                                                               | -
IPC Notify   | R5F            | YES               | NA                                    | Mailbox functionality, IPC between RTOS/NORTOS CPUs                                                                                                             | M4F core
IPC Rpmsg    | R5F            | YES               | NA                                    | RPMessage protocol based IPC                                                                                                                                    | M4F core
MCAN         | R5F            | YES               | No                                    | RX, TX, interrupt and polling mode, Corrupt Message Transmission Prevention, Error Passive state, Bus Off State, Bus Monitoring Mode                            | -
MCSPI        | R5F            | YES               | Yes. Example: mcspi_loopback_dma      | Controller/Peripheral mode, basic read/write, polling, interrupt and DMA mode                                                                                   | -
MDIO         | R5F            | YES               | NA                                    | Register read/write, link status and link interrupt enable API                                                                                                  | -
MMCSD        | R5F            | YES               | NA                                    | MMCSD 4bit, Raw read/write                                                                                                                                      | file IO, eMMC
PINMUX       | R5F            | YES               | NA                                    | Tested with multiple peripheral pinmuxes                                                                                                                        | -
PMU          | R5F            | NO                | NA                                    | Tested various PMU events                                                                                                                                       | Counter overflow detection is not enabled
OptiFlash    | R5F            | Yes               | NA                                    | FLC, RL2, RAT functionality, XIP with RL2 enabled                                                                                                               | OptiShare
OSPI         | R5F            | YES               | Yes. Example: ospi_flash_dma          | Read direct, Write indirect, Read/Write commands, DMA for read                                                                                                  | -
RTI          | R5F            | YES               | No                                    | Counter read, timebase selection, comparator setup for Interrupt, DMA requests                                                                                  | Capture feature, fast enabling/disabling of events not tested
RESOLVER     | R5F            | YES               | No                                    | Angle and Speed Calcution. input Band Pass Filter, Manual Phase Gain Correction and Manual Ideal Sample Selection Mode calculation                              | Tuning, Safety Diagnostic features
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

### Networking

Module                      | Supported CPUs | SysConfig Support | OS Support  | Key features tested                                                                    | Key features not tested
----------------------------|----------------|-------------------|-------------|----------------------------------------------------------------------------------------|------------------------
Time-Sensitive Networking(gPTP-IEEE 802.1AS) | R5F            | NO                | FreeRTOS    | gPTP IEEE 802.1 AS-2020 compliant gPTP stack, End Nodes and Bridge mode support, YANG data model configuration  | Multi-Clock Domain
LwIP                                         | R5F            | YES               | FreeRTOS    | TCP/UDP IP networking stack with and without checksum offload enabled, TCP/UDP IP networking stack with server and client functionality, basic Socket APIs, netconn APIs and raw APIs, DHCP, ping, TCP iperf, scatter-gather, DSCP priority mapping                         | Other LwIP features
Ethernet driver (ENET)                       | R5F            | YES               | FreeRTOS    | Ethernet as port using CPSW, MAC loopback and PHY loopback, Layer 2 MAC, Packet Timestamping, CPSW Switch, CPSW EST, interrupt pacing, Policer and Classifier, MDIO Manual Mode, Credit Based Shaper (IEEE 802.1Qav), Strapped PHY (Early Ethernet)  | RMII, MII mode
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
ECC               | R5F             | NA                |  NORTOS | ECC of MSS_L2, R5F TCM, MCAN, VIM, ICSSM, TPTC      | R5F Cache
ECC Bus Safety    | R5F             | NA                |  NORTOS | AHB, AXI, TPTC                           | -
CCM               | R5F             | NA                |  NORTOS | CCM Self Test Mode,Error Forcing Mode and Self Test Error Forcing Mode. TMU and RL2 are also validated                      | -
R5F STC(LBIST), Static Register Read| R5F               | NA                |  NORTOS | STC of R5F, R5F CPU Static Register Read                                 |-
TMU ROM Checksum  | R5F             | NA                |  NORTOS | ROM checksum for TMU                                                                         | -
Time out Gasket(STOG)  | R5F             | NA                |  NORTOS | Timeout gasket feature                    | -
Thermal Monitor(VTM)| R5F             | NA                |  NORTOS | Over, under and thershold temperature interrupts                   | -

**Note**: SDL is validate only on ControlCard.

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
    <td> PROC_SDL-7347
    <td> MCRC does not provide API to configure data width, CRC algo etc.
    <td> MCRC
    <td> 09.00.00 Onwards
    <td> AM263x, AM263Px
    <td>
<tr>
    <td> PROC_SDL-6910
    <td> Update to move some of the non static registers.
    <td> R5F CPU UTILS
    <td> 09.00.00 Onwards
    <td> AM263x, AM263Px
    <td> Updated R5F UTILS structure to move some of the non static registers.
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
    <td> MCUSDK-11507
    <td> ENET: CPSW MAC port is stuck forever and dropping all the Rx/Tx packets with reception of corrupts preamble
    <td> CPSW
    <td> 09.00.01
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
    <td> MCUSDK-12264
    <td> EQEP position speed example failure on am263px-lp
    <td> EQEP
    <td> 09.01.00
    <td> None
</tr>
<tr>
    <td> MCUSDK-12247
    <td> Syscfg: ouptutxbar generated code doesnt change the instance
    <td> XBAR
    <td> 09.01.00
    <td> Use XBAR driver API to configure the output xbar instance
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
    <td> MCUSDK-12756
    <td> MbedTLS - Timing side channel attack in RSA private operation exposing plaintext.
    <td> Mbed-TLS
    <td> 08.06.00 onwards
    <td> None
</tr>
<tr>
    <td> PROC_SDL-7615
    <td> ECC example fails for SEC and DED for TPTC memories.
    <td> SDL
    <td> 09.02.00 onwards
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
    <td> PROC_SDL-8393
    <td> In ECC bus safety, error injection test writes to address 0x0.
    <td> SDL
    <td> 09.00.00 onwards
    <td> None
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
