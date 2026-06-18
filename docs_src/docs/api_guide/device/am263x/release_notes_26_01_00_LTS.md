# Release Notes 26.01.00.LTS {#RELEASE_NOTES_26_01_00_PAGE}

[TOC]
\attention 1. Also refer to individual module pages for more details on each feature, unsupported features, important usage guidelines.

\attention 2. RPRC image format has been deprecated from 11.00.00 release. Multi Core ELF image format support should be used.(\ref MCELF_LANDING)

\attention 3. The default Stack size is 16KB and Heap size 32 KB for SDK examples. This can be adjusted as per application requirement through Memory Configurator in SysCfg or by updating Linker script in case of standalone applications.

\attention 4. SDK has been updated to support CCS Theia Out of Box from 11.00.00 release. Refer to Compatibility section below for changes w.r.t Eclipse.

\attention 5. For customer migrating from any release before SDK 10.00.00 to 10.00.00 or any release after, "DPL CFG" module has to be added in SysCfg in order to enable interrupts during DPL initialization. This module was added in SysCfg for 10.00.00 release to allow configuring Init time Enable/Disable of interrupts for RTOS applications (Default set to enabled).

\note The examples will show usage of SW modules and APIs on a specific CPU instance and OS combination. \n
      Unless explicitly noted otherwise, the SW modules would work in both FreeRTOS and no-RTOS environment. \n
      Unless explicitly noted otherwise, the SW modules would work on any of the R5F's present on the SOC. \n

## New in this Release

Feature                                                                                      | Module
---------------------------------------------------------------------------------------------|-----------------------------------
TBD                                                                                          | TBD

## Device and Validation Information

\cond SOC_AM263X
SOC   | Supported CPUs  | EVM                                                                          | Host PC
------|-----------------|------------------------------------------------------------------------------|-----------------------------------------
AM263x| R5F             | AM263x ControlCard Revision A  (referred to as am263x-cc in code). \n       | Windows 10 64b or Ubuntu 18.04 64b or MacOS
AM263x| R5F             | AM263x LaunchPad Revision Rev A (referred to as am263x-lp in code)           | Windows 10 64b or Ubuntu 18.04 64b or MacOS
\endcond

## Dependent Tools and Compiler Information

Tools                   | Supported CPUs | Version
------------------------|----------------|-----------------------
Code Composer Studio    | R5F            | 20.5.0
SysConfig               | R5F            | 1.25.0 build, build 4565
TI ARM CLANG            | R5F            | 4.0.4.LTS
FreeRTOS Kernel         | R5F            | 11.1.0
LwIP                    | R5F            | STABLE-2_2_1_RELEASE
Mbed-TLS                | R5F            | 2.13.1
Uniflash                | R5F            | 9.5.0

## Key Features

### Experimental Features

\attention Features listed below are early versions and should be considered as "experimental".
\attention Users can evaluate the feature, however the feature is not fully tested at TI side.
\attention TI would not support these feature on public e2e.
\attention Experimental features will be enabled with limited examples and SW modules.

Feature                                                                       | Module
------------------------------------------------------------------------------|--------------------------
Ether-ring Driver Implementation                                              | Networking
Ether-ring Demo with Real Time Traffic Generator and Background LwIP traffic  | Networking
EMMC Support with DMA                                                         | EMMC
Etherring demo showcasing the integration of vehicle CAN bus data through a CAN-Ethernet gateway | Networking
Ethernet MDIO with Clause 45 support                                          | Networking

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
Bootloader | R5FSS0-0        | YES               | NORTOS           | Boot modes: QSPI, UART. All R5F's. MCELF, multi-core image format           | -

### SOC Device Drivers

Peripheral   | Supported CPUs | SysConfig Support | DMA Supported                         | Key features tested                                                                                                                                             | Key features not tested / NOT supported
-------------|----------------|-------------------|---------------------------------------|-----------------------------------------------------------------------------------------------------------------------------------------------------------------|---------------------------------------------------
ADC          | R5F            | YES               | Yes. Example:  adc_soc_continuous_dma | Single software triggered conversion, Multiple ADC trigger using PWM, Result read using DMA, EPWM trip through PPB limit, PPB features, Burst mode, Single and Differential mode, Interrupt with Offset from Aquisition Window, EPWM/ECAP/RTI triggered conversions | -
Bootloader   | R5F            | YES               | Yes. DMA enabled for SBL QSPI         | Boot modes: QSPI, UART. All R5F's                                                                                                                               | -
CMPSS        | R5F            | YES               | NA                                    | Asynchronous PWM trip, Digital Filter, Calibration,                                                                                                                        | -
CPSW         | R5F            | YES               | No                                    | MAC loopback, PHY loopback, LWIP: Getting IP, Ping, Iperf, Layer 2 MAC, Layer 2 PTP Timestamping and Ethernet CPSW Switch support, TSN stack                      | RMII, MII mode
DAC          | R5F            | YES               | Yes. Example: dac_sine_dma            | Constant voltage, Square wave generation, Sine wave generation with and without DMA, Ramp wave generation, Random Voltage generation                            | -
ECAP         | R5F            | YES               | yes. Example : ecap_edma              | ECAP APWM mode, PWM capture, DMA trigger in both APWM and Capture Modes                                                                                                                                     | -
EDMA         | R5F            | YES               | NA                                    | DMA transfer using interrupt and polling mode, QDMA Transfer, Channel Chaining, PaRAM Linking, Error Handling                                                   | -
EPWM         | R5F            | YES               | Yes. Example: epwm_dma                | PWM outputs A and B in up-down count mode, Trip zone, Update PWM using EDMA, Valley switching, High resolution time period adjustment, type5 feature            | -
EQEP         | R5F            | YES               | NA                                    | Speed and Position measurement. Frequency Measurement, Speed and Direction Measurement, cw-ccw modes                                                                                                                              | -
FSI          | R5F            | YES               | Yes. Example: fsi_loopback_dma        | RX, TX, polling, interrupt mode, Dma, single lane loopback.                                                                                                     | - FSI Spi Mode
GPIO         | R5F            | YES               | NA                                    | Output, Input and Interrupt functionality                                                                                                                       | -
I2C          | R5F            | YES               | No                                    | Controller mode, basic read/write                                                                                                                                   | -
IPC Notify   | R5F            | YES               | NA                                    | Mailbox functionality, IPC between RTOS/NORTOS CPUs                                                                                                             | M4F core
IPC Rpmsg    | R5F            | YES               | NA                                    | RPMessage protocol based IPC                                                                                                                                    | M4F core
LIN          | R5F            | YES               | YES                                   | RX, TX, polling, interrupt, DMA mode.                                                                                                                           | -
MCAN         | R5F            | YES               | No                                    | RX, TX, interrupt and polling mode, Corrupt Message Transmission Prevention, Error Passive state, Bus Off State, Bus Monitoring Mode                            | -
MCSPI        | R5F            | YES               | Yes. Example: mcspi_loopback_dma      | Controller/Peripheral mode, basic read/write, polling, interrupt and DMA mode                                                                                            | -
MDIO         | R5F            | YES               | NA                                    | Register read/write, link status and link interrupt enable API                                                                                                  | -
MPU Firewall | R5F            | YES               | NA                                    | Only compiled (Works only on HS-SE  device)                                                                                                                     | -
MMCSD        | R5F            | YES               | NA                                    | MMCSD 4bit, Raw read/write, file IO                                                                                             | -
PINMUX       | R5F            | YES               | NA                                    | Tested with multiple peripheral pinmuxes                                                                                                  | -
PMU          | R5F            | NO                | NA                                    | Tested various PMU events                                                                                   | Counter overflow detection is not enabled
PRUICSS      | R5F            | YES               | NA                                    | Tested with Ethercat FW HAL                                                                                                                                     | -
QSPI         | R5F            | YES               | Yes. Example: qspi_flash_dma_transfer | Read direct, Write indirect, Read/Write commands, DMA for read                                                                                                  | -
RTI          | R5F            | YES               | No                                    | Counter read, timebase selction, comparator setup for Interrupt, DMA requests                                                                                   | Capture feature, fast enabling/disabling of events not tested
SDFM         | R5F            | YES               | Yes. Example: sdfm_filter_sync_dmaread| Filter data read from CPU, Filter data read with PWM sync                                                                                                       | -
SOC          | R5F            | YES               | NA                                    | Lock/unlock MMRs, clock enable, set Hz, Xbar configuration, SW Warm Reset, Address Translation                                                                  | -
SPINLOCK     | R5F            | NA                | NA                                    | Lock, unlock HW spinlocks                                                                                                                                       | -
UART         | R5F            | YES               | Yes. Example: uart_echo_dma           | Basic read/write at baud rate 115200, polling, interrupt mode                                                                                                   | HW flow control not tested, DMA mode not supported
WATCHDOG     | R5F            | YES               | NA                                    | Reset mode, Interrupt mode                                                                                       | -

### Board Device Drivers

Peripheral | Supported CPUs | SysConfig Support | Key features tested                                         | Key features not tested
-----------|----------------|-------------------|-------------------------------------------------------------|------------------------
EEPROM     | R5F            | YES               | Only compiled                                               | -
ETHPHY     | R5F            | YES               | Tested with ethercat_slave_beckhoff_ssc_demo example        | -
FLASH      | R5F            | YES               | QSPI Flash                                                  | -
LED        | R5F            | YES               | GPIO                                                        | -
IOEXPANDER | R5F            | YES               | IO configurability                                          | -

### CMSIS

Module                      | Supported CPUs | SysConfig Support | OS Support        | Key features tested                                                                         | Key features not tested
----------------------------|----------------|-------------------|-------------------|---------------------------------------------------------------------------------------------|------------------------
-                           | -              | -                 | -                 | -                                                                                           |  -


### Ethernet and Networking

Module                      | Supported CPUs | SysConfig Support | OS Support  | Key features tested                                                                    | Key features not tested
----------------------------|----------------|-------------------|-------------|----------------------------------------------------------------------------------------|------------------------
Time-Sensitive Networking(gPTP-IEEE 802.1AS) | R5F            | NO                | FreeRTOS    | gPTP IEEE 802.1 AS-2020 compliant gPTP stack, End Nodes and Bridge mode support, YANG data model configuration, IEEE 1722 compliant AVTP Stack  | Multi-Clock Domain
LwIP                                         | R5F            | YES               | FreeRTOS    | TCP/UDP IP networking stack with and without checksum offload enabled, TCP/UDP IP networking stack with server and client functionality, basic Socket APIs, netconn APIs and raw APIs, DHCP, ping, TCP iperf, scatter-gather, DSCP priority mapping                         | Other LwIP features
Ethernet driver (ENET)                       | R5F            | YES               | FreeRTOS    | Ethernet as port using CPSW, MAC loopback and PHY loopback, Layer 2 MAC, Packet Timestamping, CPSW Switch, CPSW EST, interrupt pacing, Policer and Classifier, MDIO Manual Mode, Credit Based Shaper (IEEE 802.1Qav), Strapped PHY (Early Ethernet)  | MII mode
ICSS-EMAC                   | R5F            | YES               | FreeRTOS    | Switch and MAC features, Storm Prevention (MAC), Host Statistics, Multicast Filtering  | Promiscuous Mode
Mbed-TLS                                     | R5F            | NO                | FreeRTOS    | Tested software cryptography after porting, used mbedTLS with LwIP to implement HTTPS server  | Hardware offloaded cryptography
Ether-ring Implementation | R5F            | NO                | FreeRTOS    | Duplicate Rejection, Ring termination and Packet Duplication, Latency measurement for different real-time traffic profiles, Performance KPIs | N/A

### Demos

Module                      | Supported CPUs | SysConfig Support | OS Support        | Key features tested                             | Key features not tested
----------------------------|----------------|-------------------|-------------------|-------------------------------------------------|------------------------
-                           | -              | -                 | -                 | -                                               |  -

### Safety Diagnostic Library

Module            | Supported CPUs  | SysConfig Support | OS support       | Key features tested                                                                            | Key features not tested / NOT supported
------------------|-----------------|-------------------|------------------|------------------------------------------------------------------------------------------------|----------------------------------------
MCRC              | R5F             | NA                |  NORTOS | Full CPU, Auto CPU Mode and Semi CPU Auto Mode                                                          | -
DCC               | R5F             | NA                |  NORTOS | Single Shot and Continuous modes                                    | -
PBIST             | R5F             | NA                |  NORTOS | Memories supported by MSS PBIST controller.          | -
ESM               | R5F             | NA                |  NORTOS | Tested in combination with RTI, DCC                                        | -
RTI               | R5F             | NA                |  NORTOS | WINDOWSIZE_100_PERCENT, WINDOWSIZE_50_PERCENT ,Latency/Propagation timing error(early)(50% window),Latency/Propagation timing error(late)(50% window)                                     | -
ECC               | R5F             | NA                |  NORTOS | ECC of MSS_L2, R5F TCM, MCAN, VIM, ICSSM, TPTC      | -
ECC Bus Safety    | R5F             | NA                |  NORTOS | AHB, AXI, TPTC                           | -
CCM               | R5F             | NA                |  NORTOS | CCM Self Test Mode,Error Forcing Mode and Self Test Error Forcing Mode.                      | -
R5F STC(LBIST), Static Register Read| R5F               | NA                |  NORTOS | STC of R5F, R5F CPU Static Register Read                                 |-
Integrated Example  | R5F             | NA                |FreeRTOS | Integrated example with all the SDL modules integrated in to one example.                | ECC Bu Safety and STC.

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
    <td> PROC_SDL-10008
    <td> AM263Px: Broken out-of-box SBL OSPI in v26.00.00.06
    <td> SDL
    <td> 26.00.00 onwards
    <td> AM263x, AM263Px, AM261x
    <td> Fixed in example
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
    <td> MCUSDK-13164
    <td> AM263x: EPWM deadband example failure
    <td> EPWM
    <td> 09.02.00
    <td> remove sync between the epwms and use the global tbclksync to synchronize the EPWMs
</tr>
<tr>
    <td> MCUSDK-7319
    <td> CONTROLSS-SDFM: Two Back-to-Back Writes to SDCPARMx Register Bit Fields CEVT1SEL, CEVT2SEL, and HZEN Within Three SD-Modulator Clock Cycles can Corrupt SDFM State Machine, Resulting in Spurious Comparator Events
    <td> SDFM
    <td> 08.04.00 onwards
    <td> Avoid back-to-back writes within three SD-modulator clock cycles or have the SDCPARMx register bit fields configured in one register write.
</tr>
<tr>
    <td> MCUSDK-9082
    <td> MbedTLS - RSA exploit by kernel-privileged cache side-channel attackers
    <td> Mbed-TLS
    <td> 08.06.00 onwards
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
    <td> PINDSW-7715
    <td> Dual EMAC instance not working with both ports together for icss_emac_lwip example
    <td> ICSS-EMAC
    <td> 09.02.00 onwards
    <td> None
</tr>
<tr>
    <td> PINDSW-7746
    <td> icss_emac_lwip example having low iperf values in TCP and UDP
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
    <td> MCUSDK-13164
    <td> AM263x: epwm deadband example validation failure
    <td> EPWM
    <td> 09.01.00 onwards
    <td> The Waveform of the EPWM is correct and is as expected.
</tr>
<tr>
    <td> MCUSDK-13202
    <td> Frame drops are seen in PRU GPIO based SENT decoder example while sending frames in burst format
    <td> PRU-IO
    <td> 09.00.00 onwards
    <td> None
</tr>
<tr>
    <td> PROC_SDL-8857
    <td> SDL integrated example does not support ECC Bus Safety.
    <td> SDL
    <td> 10.01.00 onwards
    <td> Use standalone example.
</tr>
<tr>
    <td> PROC_SDL-9596
    <td> SEC and DED failure in ecc_bus_safety example
    <td> SDL
    <td> 11.01.00 onwards
    <td> For AXI_RD_SEC nodes can test in specific core.
</tr>
<tr>
    <td> PROC_SDL-9617
    <td> AXI_S SEC and DED failure
    <td> SDL
    <td> 11.01.00 onwards
    <td> For AXI_S node, use CCS to test.
</tr>
<tr>
    <td> PROC_SDL-9130
    <td> EDMA_T4 diag failure
    <td> SDL
    <td> 10.02.00 onwards
    <td> None
</tr>
<tr>
    <td> PROC_SDL-9131
    <td> CLK-T1 MSS_DCCC and DCCA diag failure
    <td> SDL
    <td> 10.02.00 onwards
    <td> None
</tr>
<tr>
    <td> PROC_SDL-9670
    <td> ECC injection addresses for different banks in TCMB memory are not correct
    <td> SDL
    <td> 11.00.00 onwards
    <td> None
</tr>
<tr>
    <td> PROC_SDL-9700
    <td> The ESM callback is happening even before ECC injection for ATCM and BTCM memories
    <td> SDL
    <td> 11.00.00 onwards
    <td> None
</tr>
<tr>
    <td> PROC_SDL-9896
    <td> Removal of incorrect DCC examples
    <td> SDL
    <td> 11.01.00 onwards
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
    <td> MCUSDK-13652
    <td> Readelf throws warning while parsing RS note
    <td> SBL, QSPI
    <td> Readelf command throws error when trying to read the RS note segment from an mcelf file.
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
<tr>
    <td> MCUSDK-14914
    <td> Unable to add UART communication port to target configuration in Theia
    <td> Real Time Debug
    <td> 10.02.00 onwards
    <td> Use the CCXML file from CCS eclipse after updating to correct COM port.
</tr>
<tr>
    <td> MCUSDK-14644
    <td> EST Timestamp verification is failing after link down event
    <td> Networking
    <td> 11.00.00 onwards
    <td> Re-apply the EST configuration post link-up
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
    <td> i2311
    <td> USART: Spurious DMA Interrupts
    <td> UART
    <td> Implemented
</tr>
<tr>
    <td> i2313
    <td> GPMC: Sub-32-bit read issue with NAND and FPGA/FIFO
    <td> GPMC
    <td> Not supported in SDK
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
    <td> Implemented
</tr>
<tr>
    <td> i2354
    <td> SDFM: Two Back-to-Back Writes to SDCPARMx Register Bit Fields CEVT1SEL, CEVT2SEL, and HZEN Within Three SD-Modulator Clock Cycles can Corrupt SDFM State Machine, Resulting in Spurious Comparator Events
    <td> SDFM
    <td> Open
</tr>
<tr>
    <td> i2375
    <td> SDFM: SDFM module event flags (SDIFLG.FLTx_FLG_CEVTx) do not get set again if the comparator event is still active and digital filter path (using SDCOMPxCTL.CEVTxDIGFILTSEL) is being selected
    <td> SDFM
    <td> Open
</tr>
<tr>
    <td> i2355
    <td> ADC: DMA Read of Stale Result
    <td> ADC
    <td> Implemented
</tr>
<tr>
    <td> i2356
    <td> ADC: Interrupts may Stop if INTxCONT (Continue-to-Interrupt Mode) is not Set
    <td> ADC
    <td> Implemented
</tr>
<tr>
    <td> i2375
    <td> SDFM module event flags (SDIFLG.FLTx_FLG_CEVTx) do not get set again if the comparator event is still active and digital filter path (using SDCOMPxCTL.CEVTxDIGFILTSEL) is being selected
    <td> SDFM
    <td> Open
</tr>
<tr>
    <td> i2392
    <td> Race condition in capture registers resulting in events miss
    <td> Common
    <td> Open
</tr>
<tr>
    <td> i2394
    <td> Race condition in interrupt and error aggregator capture registers resulting in events miss
    <td> Common
    <td> Updated error aggregator registers in EDMA.
</tr>
<tr>
    <td> i2401
    <td> CPSW: Host Timestamps Cause CPSW Port to Lock up
    <td> CPSW
    <td> Open
</tr>
<tr>
    <td> i2402
    <td> CPSW: Ethernet to Host Checksum Offload does not work
    <td> CPSW
    <td> Open
</tr>
<tr>
    <td> i2404
    <td> Race condition in mailbox registers resulting in events miss
    <td> IPC
    <td> Implemented
</tr>
<tr>
    <td> i2405
    <td> CONTROLSS: Race condition OUTPUT_XBAR and PWM_XBAR resulting in event miss
    <td> XBAR
    <td> Open
</tr>
<tr>
    <td> i2427
    <td> SDL: RAM SEC can cause spurious RAM writes resulting in L2 and MBOX memory corruption
    <td> ECC
    <td> Workaround added as an example in examples/sdl/ecc/ (EDMA case only; PRU fix is ongoing)
</tr>
<tr>
    <td> i2499
    <td> SDL: Incorrect data returned to master on single error detection during burst read
    <td> ECC
    <td> Workaround added as an example in examples/sdl/ecc/ (EDMA case only; PRU fix is ongoing)
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
<tr>
    <td> -
    <td> -
    <td> -
    <td> -
    <td> -
</tr>
</table>

## Release Types

AM26xx SDK releases follow a tiered support model with three release types: Early Adopter (EA), Short Term Support (STS), and Long Term Support (LTS). Each release type serves different customer needs and provides varying levels of support, quality assurance, and maintenance.

### How to Identify Release Type

The release type for each AM26xx SDK release can be identified through the following methods:

Release Notes Introduction Section: Every release note’s Introduction section will specify the release type (EA, STS, or LTS) for that particular release.

### EA (Early Adopter)

Early Adopter releases are intended for customers who want early access to new features and capabilities before they are fully validated. These releases allow customers to evaluate new functionality and provide feedback during the development cycle.

Example Use Cases:
- Early evaluation of new features
- Proof-of-concept development
- Providing feedback to the development team
- Planning for future integration

### STS (Short Term Support)

Short Term Support releases provide a stable platform with validated features suitable for development and integration activities. These releases offer improved quality assurance compared to EA releases but are not intended for production deployment.

Example Use Cases:
- Development and integration activities
- System validation and testing
- Pre-production evaluation
- Feature completeness verification

### LTS (Long Term Support)

Long Term Support releases are production-ready releases that provide the highest level of quality assurance, safety certification support, and long-term maintenance. These releases are recommended for production deployment in automotive and safety-critical applications.

Example Use Cases:
- Production deployment
- Safety-critical applications
- Automotive production systems
- Applications requiring long-term stability and support

### Release Type Comparison

<table>
<tr>
  <th style="text-align: left;">Topic</th>
  <th style="text-align: left;">EA (Early Adopter)</th>
  <th style="text-align: left;">STS (Short Term Support)</th>
  <th style="text-align: left;">LTS (Long Term Support)</th>
</tr>
<tr>
  <td>Quality</td>
  <td>Baseline Quality</td>
  <td>Baseline Quality (new features)</td>
  <td>Functional Safety (if applicable)</td>
</tr>
<tr>
  <td>Testing</td>
  <td>Best Effort</td>
  <td>>95% pass</td>
  <td>>95% pass</td>
</tr>
<tr>
  <td>Safety CSP</td>
  <td>NA</td>
  <td>NA</td>
  <td>Yes (if FSQ)</td>
</tr>
<tr>
  <td>Safety Certification</td>
  <td>NA</td>
  <td>NA</td>
  <td>Yes (Internal / External)</td>
</tr>
<tr>
  <td>Maintenance</td>
  <td>No</td>
  <td>No</td>
  <td>Will maintain until next LTS release</td>
</tr>
<tr>
  <td>Bug Fix / Patch Release</td>
  <td>No</td>
  <td>Bugs that are prioritized will be fixed in the next release; no backporting to STS releases</td>
  <td>Bug fixes on the LTS maintenance branch will be prioritized based on the severity of the bug</td>
</tr>
<tr>
  <td>Feature Addition</td>
  <td>Yes</td>
  <td>Yes</td>
  <td>New features will be added in an LTS release, but post LTS release, new features will not be backported to the LTS maintenance branch</td>
</tr>
<tr>
  <td>Production Ready</td>
  <td>No</td>
  <td>Not recommended for Production</td>
  <td>Recommended for Production</td>
</tr>
</table>

## CI/CD Strategy

AM26xx SDK is also available on GitHub.
- AM263X - https://github.com/TexasInstruments/mcu_plus_sdk_am263x/
- AM263PX - https://github.com/TexasInstruments/mcu_plus_sdk_am263px/
- AM261X - https://github.com/TexasInstruments/mcu_plus_sdk_am261x/

These GitHub repositories provide early access to the latest features, fixes, and improvements as they are developed.

Disclaimer: The GitHub repositories contains software that is not fully tested and may include bugs, errors, or defects that could cause system instability or unexpected behavior. Always review and validate code before use in any application.

Official releases are published under the Releases section of the GitHub repository. These correspond to the versioned releases documented in these release notes and have undergone the quality and testing criteria.

## Software Bill of Materials

<table>
    <tr>
        <th style="text-align: left;">Component Type</th>
        <th style="text-align: left;">Component Name</th>
        <th style="text-align: left;">Production or Reference</th>
        <th style="text-align: left;">Process Compliance</th>
        <th style="text-align: left;">Certification</th>
        <th style="text-align: left;">Distribution</th>
    </tr>
    <tr>
        <td>Tools</td>
        <td>${SDK_INSTALL_PATH}/tools/*</td>
        <td>Reference</td>
        <td>Demo Quality</td>
        <td>No</td>
        <td>github</td>
    <tr>
        <td>Documentation</td>
        <td>${SDK_INSTALL_PATH}/docs/*</td>
        <td>Reference</td>
        <td>Baseline Quality</td>
        <td>No</td>
        <td>github</td>
    <tr>
        <td>Source</td>
        <td>${SDK_INSTALL_PATH}/source/*</td>
        <td>Production</td>
        <td>Baseline Quality</td>
        <td>No</td>
        <td>github</td>
    <tr>
        <td>Examples</td>
        <td>${SDK_INSTALL_PATH}/examples/*</td>
        <td>Reference</td>
        <td>Demo Quality</td>
        <td>No</td>
        <td>github</td>
</table>

## Upgrade and Compatibility Information
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

### SDL PBIST Self test
VIM Memory groups are added to PBIST self test from this release. Because of this change, Self test (SDL_PBIST_selfTest) API has to be called in polling mode only and interrupt mode is not supported.

### SDL STC Configuration
faultInsert and stcDiagnostic are removed from SDL_STC_Config structure as unused in SDL library.

### SDL Handler API name update
Handler API names are added prefix with "SDL_". For example, undefInstructionExptnHandler changed to SDL_undefInstructionExptnHandler.

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
