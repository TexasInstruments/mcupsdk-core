# Release Notes 09.00.00 {#RELEASE_NOTES_09_00_00_PAGE}

[TOC]

\attention Also refer to individual module pages for more details on each feature, unsupported features, important usage guidelines.

\note The examples will show usage of SW modules and APIs on a specific CPU instance and OS combination. \n
      Unless explicitly noted otherwise, the SW modules would work in both FreeRTOS and no-RTOS environment. \n
      Unless explicitly noted otherwise, the SW modules would work on any of the R5F's present on the SOC. \n

\attention Klockwork Static Analysis report is not updated for this release

## New in this Release

Feature                                                                                         | Module
------------------------------------------------------------------------------------------------|-----------------------------------
SFO calibration library and examples                                                            | EPWM
Optimized IRQ handler and low latency interrupt example                                         | DPL
Standby (WFI) mode support DPL noRTOS                                                           | DPL
EDMA example to transfer between different memories                                             | EDMA
SysConfig option to enable pBIST for SBL                                                        | SBL
Interrupt profiling support and example                                                         | DPL
Support for loading HS-FS firmware view CCS load script                                         | Common
Safe IPC support                                                                                | IPC
MbedTLS MQTT example                                                                            | Networking
Support added for Single Packet reception from DMA                                              | Networking

## Device and Validation Information

\cond SOC_AM263X
SOC   | Supported CPUs  | EVM                                                                          | Host PC
------|-----------------|------------------------------------------------------------------------------|-----------------------------------------
AM263x| R5F             | AM263x ControlCard Revision E1  (referred to as am263x-cc in code). \n       | Windows 10 64b or Ubuntu 18.04 64b
AM263x| R5F             | AM263x LaunchPad Revision E2  (referred to as am263x-lp in code)             | Windows 10 64b or Ubuntu 18.04 64b
\endcond

<!-- Refer here for information about using this release with E2 revision of ControlCard
- \subpage RELEASE_NOTES_08_03_00_EVM_REV_E2_SUPPORT_PAGE -->

## Dependent Tools and Compiler Information

Tools                   | Supported CPUs | Version
------------------------|----------------|-----------------------
Code Composer Studio    | R5F            | @VAR_CCS_VERSION_AM263X
SysConfig               | R5F            | @VAR_SYSCFG_VERSION_AM263X, build @VAR_SYSCFG_BUILD_AM263X
TI ARM CLANG            | R5F            | @VAR_TI_ARM_CLANG_VERSION
FreeRTOS Kernel         | R5F            | @VAR_FREERTOS_KERNEL_VERSION
LwIP                    | R5F            | @VAR_LWIP_VERSION
Mbed-TLS                | R5F            | @VAR_MBEDTLS_VERSION

\attention TI ARM CLANG @VAR_TI_ARM_CLANG_VERSION is not part of CCS by default, Follow steps at \ref INSTALL_TIARMCLANG to install the compiler

## Key Features

### Experimental Features

\attention Features listed below are early versions and should be considered as "experimental".
\attention Users can evaluate the feature, however the feature is not fully tested at TI side.
\attention TI would not support these feature on public e2e.
\attention Experimental features will be enabled with limited examples and SW modules.

Feature                                                             | Module
--------------------------------------------------------------------|--------------------------
SENT Encoder Example                                                | PRU_IO
SENT Decoder Example                                                | PRU_IO
Empty PRU firmware Example                                          | PRU_IO
GUI for UART Uniflash Tool                                          | Bootloader
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
Bootloader | R5FSS0-0        | YES               | NORTOS           | Boot modes: QSPI, UART. All R5F's. RPRC, multi-core image format            | Force Dual Core Mode, Disable Dual Core Switch and R5SS1 only not tested

### SOC Device Drivers

Peripheral   | Supported CPUs | SysConfig Support | DMA Supported                         | Key features tested                                                                                                                                             | Key features not tested / NOT supported
-------------|----------------|-------------------|---------------------------------------|-----------------------------------------------------------------------------------------------------------------------------------------------------------------|---------------------------------------------------
ADC          | R5F            | YES               | Yes. Example:  adc_soc_continuous_dma | Single software triggered conversion, Multiple ADC trigger using PWM, Result read using DMA, EPWM trip through PPB limit, PPB limits, PPB offsets, burst mode oversampling, differential mode, Offset, EPWM triggered conversion                | -
Bootloader   | R5F            | YES               | Yes. DMA enabled for SBL QSPI         | Boot modes: QSPI, UART. All R5F's                                                                                                                               | -
CMPSS        | R5F            | YES               | NA                                    | Asynchronous PWM trip                                                                                                                                           | -
CPSW         | R5F            | YES               | No                                    | MAC loopback, PHY loopback, LWIP: Getting IP, Ping, Iperf, Layer 2 MAC, Layer 2 PTP Timestamping and Ethernet CPSW Switch support                               | -
DAC          | R5F            | YES               | Yes. Example: dac_sine_dma            | Constant voltage, Square wave generation, Sine wave generation with and without DMA, Ramp wave generation, Random Voltage generation                            | -
ECAP         | R5F            | YES               | No                                    | ECAP APWM mode, PWM capture                                                                                                                                     | -
EDMA         | R5F            | YES               | NA                                    | DMA transfer using interrupt and polling mode, QDMA Transfer, Channel Chaining, PaRAM Linking                                                                   | -
EPWM         | R5F            | YES               | Yes. Example: epwm_dma                | PWM outputs A and B in up-down count mode, Trip zone, Update PWM using EDMA, Valley switching, High resolution time period adjustment, type5 feature            | -
EQEP         | R5F            | YES               | NA                                    | Speed and Position measurement. Frequency Measurement                                                                                                                                 | -
FSI          | R5F            | YES               | Yes. Example: fsi_loopback_dma        | RX, TX, polling, interrupt mode, Dma, single lane loopback.                                                                                                     | - FSI Spi Mode
GPIO         | R5F            | YES               | NA                                    | Output, Input and Interrupt functionality                                                                                                                       | -
I2C          | R5F            | YES               | No                                    | Controller mode, basic read/write                                                                                                                                   | -
IPC Notify   | R5F            | YES               | NA                                    | Mailbox functionality, IPC between RTOS/NORTOS CPUs                                                                                                             | M4F core
IPC Rpmsg    | R5F            | YES               | NA                                    | RPMessage protocol based IPC                                                                                                                                    | M4F core
MCAN         | R5F            | YES               | No                                    | RX, TX, interrupt and polling mode, Corrupt Message Transmission Prevention, Error Passive state, Bus Off State, Bus Monitoring Mode                            | -
MCSPI        | R5F            | YES               | Yes. Example: mcspi_loopback_dma      | Controller/Peripheral mode, basic read/write, polling, interrupt and DMA mode                                                                                            | -
MDIO         | R5F            | YES               | NA                                    | Register read/write, link status and link interrupt enable API                                                                                                  | -
MPU Firewall | R5F            | YES               | NA                                    | Only compiled (Works only on HS-SE  device)                                                                                                                     | -
MMCSD        | R5F            | YES               | NA                                    | MMCSD 4bit, Raw read/write                                                                                                  | - file IO, eMMC
PINMUX       | R5F            | YES               | NA                                    | Tested with multiple peripheral pinmuxes
PMU          | R5F            | NO                | NA                                    | Tested various PMU events                                                                                   | Counter overflow detection is not enabled                                                                                                                        | -
PRUICSS      | R5F            | YES               | NA                                    | Tested with Ethercat FW HAL                                                                                                                                     | -
QSPI         | R5F            | YES               | Yes. Example: qspi_flash_dma_transfer | Read direct, Write indirect, Read/Write commands, DMA for read                                                                                                  | -
RTI          | R5F            | YES               | No                                    | Counter read, timebase selction, comparator setup for Interrupt, DMA requests                                                                                   | Capture feature, fast enabling/disabling of events not tested
SDFM         | R5F            | YES               | No                                    | Filter data read from CPU, Filter data read with PWM sync                                                                                                       | -
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

### CMSIS

Module                      | Supported CPUs | SysConfig Support | OS Support        | Key features tested                                                                         | Key features not tested
----------------------------|----------------|-------------------|-------------------|---------------------------------------------------------------------------------------------|------------------------
-                           | -              | -                 | -                 | -                                                                                           |  -

### Industrial Communications Toolkit

Module                                | Supported CPUs | SysConfig Support | OS Support  | Key features tested                                                                                      | Key features not tested
--------------------------------------|----------------|-------------------|-------------|----------------------------------------------------------------------------------------------------------|------------------------
EtherCAT SubDevice FWHAL                  | R5F            | NO                | FreeRTOS    | Tested with ethercat_slave_beckhoff_ssc_demo example                                                     | Reset isolation

### Motor Control

Module                      | Supported CPUs | SysConfig Support | OS Support        | Key features tested                             | Key features not tested
----------------------------|----------------|-------------------|-------------------|-------------------------------------------------|------------------------
-                           | -              | -                 | -                 | -                                               |  -

### Networking

Module                      | Supported CPUs | SysConfig Support | OS Support  | Key features tested                                                                    | Key features not tested
----------------------------|----------------|-------------------|-------------|----------------------------------------------------------------------------------------|------------------------
LwIP                        | R5F            | YES               | FreeRTOS    | TCP/UDP IP networking stack with and without checksum offload enabled, TCP/UDP IP networking stack with server and client functionality, basic Socket APIs, netconn APIs and raw APIs, DHCP, ping, TCP iperf, scatter-gather, DSCP priority mapping                         | Other LwIP features
Ethernet driver (ENET)      | R5F            | YES               | FreeRTOS    | Ethernet as port using CPSW, MAC loopback and PHY loopback, Layer 2 MAC, Packet Timestamping, CPSW Switch, CPSW EST, interrupt pacing, Policer and Classifier, MDIO Manual Mode, CBS (IEEE 802.1Qav), Strapped PHY (Early Ethernet) | RMII mode
ICSS-EMAC                   | R5F            | YES               | FreeRTOS    | Only compiled                                                                          | Not tested
Mbed-TLS                    | R5F            | NO                | FreeRTOS    | Tested software cryptography after porting, used mbedTLS with LwIP to implement HTTPS server  | Hardware offloaded cryptography

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
ECC               | R5F             | NA                |  NORTOS | ECC of MSS_L2, R5F TCM, MCAN, VIM, ICSSM      | R5F Cache, HSM, CPSW
Bus Safety        | R5F             | NA                |  NORTOS | AHB, AXI, TPTC                           | -
CCM               | R5F             | NA                |  NORTOS | CCM Self Test Mode,Error Forcing Mode and Self Test Error Forcing Mode.                      | -
R5F STC(LBIST)    | R5F             | NA                |  NORTOS | STC of R5F.                                                 |-
PARITY            | R5F             | NA                |  NORTOS | TCM and DMA memories                                                |-
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
    <td> MCUSDK-8526
    <td> SBL examples should be calling Bootloader_loadSelfCpu for R5_0-0 core
    <td> SBL
    <td> 8.4.0 onwards
    <td> -
</tr>
<tr>
    <td> MCUSDK-9309
    <td> IpcRPmsg : Invalid TX RX queue buffer allocation via syscfg for multicore project
    <td> IPC
    <td> 8.1.0 onwards
    <td> Updated memory allocation for IPC
</tr>
<tr>
    <td> MCUSDK-9662
    <td> QSPI LLD EDMA Transfer fails for size (Unaligned) > MAX EDMA CNT
    <td> QSPI
    <td> 8.4.0 onwards
    <td> Initiate trasfer with aligned data
</tr>
<tr>
    <td> MCUSDK-9811
    <td> IPC Notify unregister client always returns success
    <td> IPC
    <td> 8.05.00 onwards
    <td> Added status variable check
</tr>
<tr>
    <td> MCUSDK-9835
    <td> SBL should support HS-SE device build via CCS
    <td> SBL
    <td> 8.05.00 onwards
    <td> Added devconfig for CCS build
</tr>
<tr>
    <td> MCUSDK-10102
    <td> Trip Zone example not working
    <td> EPWM
    <td> 8.6.0
    <td> Gpio changes needed on E1 to E2 migration
</tr>
<tr>
    <td> MCUSDK-10119
    <td> MCAN External read write example fails for multirun
    <td> MCAN
    <td> 8.5.0 onwards
    <td> Interrupt reset is required during init in example
</tr>
<tr>
    <td> MCUSDK-10144
    <td> RTI driver Up Counter calculation in driver is incorrect
    <td> Timer
    <td> 8.06.00 onwards
    <td> Updated calculation to subtract reload value by one
</tr>
<tr>
    <td> MCUSDK-10167
    <td> All the system project are made based on AM2631
    <td> CCS
    <td> 8.6.0
    <td> Updated to CCS12.4 and latest CSP
</tr>
<tr>
    <td> MCUSDK-10295
    <td> ECAP : driver : Flags read doesn't include the Signal Monitoring errors
    <td> ECAP
    <td> 8.5.0 onwards
    <td> -
</tr>
<tr>
    <td> MCUSDK-10550
    <td> Global Strobe Load Signal from EPWMs is not taking effect on ECAP Signal Monitoring
    <td> ECAP
    <td> 8.5.0 onwards
    <td> -
</tr>
<tr>
    <td> MCUSDK-10712
    <td> GPIO_Input_Interrupt Example not working
    <td> GPIO
    <td> 8.6.0
    <td> Added Interrupt config call in template
</tr>
<tr>
    <td> MCUSDK-10713
    <td> Interrupt registration fixed in the IPC Driver code
    <td> IPC
    <td> 8.5.0 onwards
    <td> Added Sysconfig based interrupt registration
</tr>
<tr>
    <td> MCUSDK-10841
    <td> SBL needs to copy the vector table for self core right before the self-core reset release
    <td> SBL
    <td> 8.06.00 onwards
    <td> Move the image load for self core just before the release reset instead of doing it early
</tr>
<tr>
    <td> MCUSDK-11290
    <td> EPWM: High Resolution DeadBand APIs does not write to registers properly
    <td> EPWM
    <td> 8.6.0
    <td> -
</tr>
<tr>
    <td> MCUSDK-11354
    <td> EPWM_clearTripZoneFlag() function in the SDK does not clear the CAPEVT signal
    <td> EPWM
    <td> 8.6.0
    <td> Updated EPWM_clearTripZoneFlag API to accept CAPEVT as input
</tr>
<tr>
    <td> MCUSDK-11401
    <td> Mailbox memory needs to be cleared after HSMRt load
    <td> SBL
    <td> 8.6.0
    <td> -
</tr>
<tr>
    <td> MCUSDK-11207
    <td> Incorrect IOCTL params for CPSW ENET_TIMESYNC_IOCTL_SET_TIMESTAMP
    <td> Networking
    <td> 8.5.0
    <td> -
</tr>
<tr>
    <td> MCUSDK-10775
    <td> Example build failing on enabling External Phy Management
    <td> Networking
    <td> 8.6.0
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
    <td> MCUSDK-4234
    <td> FSI RX Generic Trigger Test is not working
    <td> FSI
    <td> 8.3.0
    <td> -
</tr>
<tr>
    <td> SITARAAPPS-2040
    <td> Dual Core configuration issue with CSP 1.1.3 (Sitara MCU Device Support) on AM263x
    <td> CSP Gel Scripts
    <td> 8.2.1
    <td> Edit gel file as mentioned in  \ref PREREQUISITES while running  multi core applications.
</tr>
<tr>
    <td> MCUSDK-7030
    <td> Interrupt nesting is not functional as expected when you have 2 or more interrupts with different priorities
    <td> MCAN
    <td> 8.4.0
    <td> Keep the interrupt priority same in system
</tr>
<tr>
    <td> MCUSDK-7319
    <td> CONTROLSS-SDFM: Two Back-to-Back Writes to SDCPARMx Register Bit Fields CEVT1SEL, CEVT2SEL, and HZEN Within Three SD-Modulator Clock Cycles can Corrupt SDFM State Machine, Resulting in Spurious Comparator Events
    <td> SDFM
    <td> 8.4.0
    <td> Avoid back-to-back writes within three SD-modulator clock cycles or have the SDCPARMx register bit fields configured in one register write.
</tr>
<tr>
    <td> MCUSDK-8073
    <td> UART1 not working as expected while configuring two uarts i.e UART0 and UART1 for two different cores
    <td> UART
    <td> 8.4.0 onwards
    <td> UART1 configuration from other core should be done after UART0 is configured and initialized
</tr>
<tr>
    <td> MCUSDK-9082
    <td> MbedTLS - RSA exploit by kernel-privileged cache side-channel attackers
    <td> Mbed-TLS
    <td> 8.6.0
    <td> -
</tr>
<tr>
    <td> MCUSDK-10626
    <td> FSI DMA loopback example TX and RX mismatching on 2nd run
    <td> FSI
    <td> 8.6.0
    <td> Reset board between 2 runs
</tr>
<tr>
    <td> MCUSDK-11147
    <td> LIN Master does not works as a transmitter after a single pkt is sent and received as receiver
    <td> LIN
    <td> 8.6.0
    <td> -
</tr>
<tr>
    <td> MCUSDK-11462
    <td> EPWM: Illegal Combo Logic example fails
    <td> EPWM
    <td> 9.0.0
    <td> -
</tr>
<tr>
    <td> PROC_SDL-5616
    <td> For ECC Bus Safety, SEC and DED are not supported for CPSW.
    <td> SDL
    <td> 8.6.0 onwards
    <td> None.
</tr>
<tr>
    <td> PROC_SDL-5617
    <td> ECC Bus safety for SEC and DED not supported for MSS_L2.
    <td> SDL
    <td> 8.6.0 onwards
    <td> None.
</tr>
<tr>
    <td> PROC_SDL-4749
    <td> AXI DED Bus Safety fail.
    <td> SDL
    <td> 8.5.0 onwards
    <td> None.
</tr>
<tr>
    <td> MCUSDK-11506
    <td> ENET: CPDMA Goes To Lockup State.
    <td> CPSW
    <td> 8.5.0 onwards
    <td> Disable THOST checksum Offload.
</tr>
<tr>
    <td> MCUSDK-11507
    <td> ENET: CPSW MAC port is stuck forever and dropping all the Rx/Tx packets with reception of corrupts preamble.
    <td> CPSW
    <td> 8.2.0 onwards
    <td> Disable hostRxTimestampEn flag in CPSW CPST configuration. This does not impact the CPTS Rx or Tx Timestamp Events for PTP packets and is orthogonal feature.
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
    <td> i2354
    <td> SDFM: Two Back-to-Back Writes to SDCPARMx Register Bit Fields CEVT1SEL, CEVT2SEL, and HZEN Within Three SD-Modulator Clock Cycles can Corrupt SDFM State Machine, Resulting in Spurious Comparator Events
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
    <td> i2356
    <td> ADC: Interrupts may Stop if INTxCONT (Continue-to-Interrupt Mode) is not Set
    <td> ADC
    <td> Implemented
</tr>
<tr>
    <td> i2324
    <td> No synchronizer present between GCM and GCD status signals
    <td> Common
    <td> Open
</tr>
<tr>
    <td> i2375
    <td> SDFM module event flags (SDIFLG.FLTx_FLG_CEVTx) do not get set again if the comparator event is still active and digital filter path (using SDCOMPxCTL.CEVTxDIGFILTSEL) is being selected
    <td> SDFM
    <td> Open
</tr>
<tr>
    <td> i2313
    <td> GPMC: Sub-32-bit read issue with NAND and FPGA/FIFO
    <td> GPMC
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
    <td> MCUSDK-9471
    <td> Ethernet CPSW CPDMA stuck with SOF overrun when TCP/DUP checksum offload is enabled.
    <td> Ethernet CPSW
    <td> 08.05.00 onwards
    <td> Disable TCPUDP checksum offload in receive (THOST) direction.
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
    <td> ECAP driver versions v0 and v2
    <td> ECAP_continousmodeConfig
    <td> Spell corrected to "ECAP_continuousmodeConfig"
    <td> -
</tr>
<tr>
    <td> IPC Notify version v0 and v1
    <td> IpcNotify_init
    <td> IntrPriority added as an element in IpcNotify_Params structure to make interrupt priority configurable.
    <td> -
</tr>
<tr>
    <td> ClockP
    <td> ClockP_init
    <td> IntrPriority added as an element in ClockP_Params structure to make Tick interrupt priority configurable.
    <td> -
</tr>
</table>

<table>
<tr>
    <th> Module
    <th> Affected API
    <th> Change
    <th> Additional Remarks
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
</table>
