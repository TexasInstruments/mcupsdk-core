# Release Notes 10.00.00 {#RELEASE_NOTES_10_00_00_PAGE}

[TOC]
\attention 1. There are known issues about increased build time for **networking examples** having Link Time Optimizations (LTO) enabled.
              Similar issue will be observed when enabling LTO on other examples. See **Known Issues** below.

\attention 2. Also refer to individual module pages for more details on each feature, unsupported features, important usage guidelines.

\attention 3. Multi Core ELF image format support has been added (\ref MCELF_LANDING). RPRC format will be deprecated from SDK 11.0.

\note The examples will show usage of SW modules and APIs on a specific CPU instance and OS combination. \n
      Unless explicitly noted otherwise, the SW modules would work in both FreeRTOS and no-RTOS environment. \n
      Unless explicitly noted otherwise, the SW modules would work on any of the R5F's present on the SOC. \n

## New in this Release

Feature                                                                                                  | Module
---------------------------------------------------------------------------------------------------------|-----------------------------------
Sysconfig support for PRU Projects                                                                       | PRUICSS
Fast Boot support for Improved boot time (\ref FAST_SECURE_BOOT)                                         | SBL
MacOS support                                                                                            | Infra
Multi Core ELF(MCELF) image format support (\ref MCELF_LANDING)                                          | Build
QSPI Flash File System support (\ref EXAMPLES_DRIVERS_QSPI_FLASH_FILE_IO)                                | QSPI
MMCSD LLD support (\ref DRIVERS_MMCSD_V1_LLD_PAGE)                                                       | MMCSD
CANFD HLD support (\ref DRIVERS_CANFD_HLD_PAGE)                                                               | MCAN

## Device and Validation Information

\cond SOC_AM263X
SOC   | Supported CPUs  | EVM                                                                          | Host PC
------|-----------------|------------------------------------------------------------------------------|-----------------------------------------
AM263x| R5F             | AM263x ControlCard Revision E2  (referred to as am263x-cc in code). \n       | Windows 10 64b or Ubuntu 18.04 64b
AM263x| R5F             | AM263x LaunchPad Revision E2  (referred to as am263x-lp in code)             | Windows 10 64b or Ubuntu 18.04 64b
\endcond

<!-- Refer here for information about using this release with E2 revision of ControlCard
- \subpage RELEASE_NOTES_08_03_00_EVM_REV_E2_SUPPORT_PAGE -->

## Dependent Tools and Compiler Information

Tools                   | Supported CPUs | Version
------------------------|----------------|-----------------------
Code Composer Studio    | R5F            | 12.8.0
SysConfig               | R5F            | 1.21.0 build, build 3721
TI ARM CLANG            | R5F            | 4.0.0.LTS
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
SENT Encoder Example                                                | PRU-IO
SENT Decoder Example                                                | PRU-IO
SENT Decoder IEP CAP EXAMPLE                                        | PRU-IO
Empty PRU firmware Example                                          | PRU-IO
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
Bootloader | R5FSS0-0        | YES               | NORTOS           | Boot modes: QSPI, UART. All R5F's. RPRC, MCELF, multi-core image format     | Force Dual Core Mode, Disable Dual Core Switch and R5SS1 only not tested

### SOC Device Drivers

Peripheral   | Supported CPUs | SysConfig Support | DMA Supported                         | Key features tested                                                                                                                                             | Key features not tested / NOT supported
-------------|----------------|-------------------|---------------------------------------|-----------------------------------------------------------------------------------------------------------------------------------------------------------------|---------------------------------------------------
ADC          | R5F            | YES               | Yes. Example:  adc_soc_continuous_dma | Single software triggered conversion, Multiple ADC trigger using PWM, Result read using DMA, EPWM trip through PPB limit, PPB features, Burst mode, Single and Differential mode, Interrupt with Offset from Aquisition Window, EPWM/ECAP/RTI triggered conversions | -
Bootloader   | R5F            | YES               | Yes. DMA enabled for SBL QSPI         | Boot modes: QSPI, UART. All R5F's                                                                                                                               | -
CMPSS        | R5F            | YES               | NA                                    | Asynchronous PWM trip, Digital Filter                                                                                                                                           | -
CPSW         | R5F            | YES               | No                                    | MAC loopback, PHY loopback, LWIP: Getting IP, Ping, Iperf, Layer 2 MAC, Layer 2 PTP Timestamping and Ethernet CPSW Switch support, TSN stack                      | RMII, MII mode
DAC          | R5F            | YES               | Yes. Example: dac_sine_dma            | Constant voltage, Square wave generation, Sine wave generation with and without DMA, Ramp wave generation, Random Voltage generation                            | -
ECAP         | R5F            | YES               | yes. Example : ecap_edma              | ECAP APWM mode, PWM capture, DMA trigger in both APWM and Capture Modes                                                                                                                                     | -
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

### CMSIS

Module                      | Supported CPUs | SysConfig Support | OS Support        | Key features tested                                                                         | Key features not tested
----------------------------|----------------|-------------------|-------------------|---------------------------------------------------------------------------------------------|------------------------
-                           | -              | -                 | -                 | -                                                                                           |  -


### Ethernet and Networking

Module                      | Supported CPUs | SysConfig Support | OS Support  | Key features tested                                                                    | Key features not tested
----------------------------|----------------|-------------------|-------------|----------------------------------------------------------------------------------------|------------------------
Time-Sensitive Networking(gPTP-IEEE 802.1AS) | R5F            | NO                | FreeRTOS    | gPTP IEEE 802.1 AS-2020 compliant gPTP stack, End Nodes and Bridge mode support, YANG data model configuration  | Multi-Clock Domain
LwIP                                         | R5F            | YES               | FreeRTOS    | TCP/UDP IP networking stack with and without checksum offload enabled, TCP/UDP IP networking stack with server and client functionality, basic Socket APIs, netconn APIs and raw APIs, DHCP, ping, TCP iperf, scatter-gather, DSCP priority mapping                         | Other LwIP features
Ethernet driver (ENET)                       | R5F            | YES               | FreeRTOS    | Ethernet as port using CPSW, MAC loopback and PHY loopback, Layer 2 MAC, Packet Timestamping, CPSW Switch, CPSW EST, interrupt pacing, Policer and Classifier, MDIO Manual Mode, Credit Based Shaper (IEEE 802.1Qav), Strapped PHY (Early Ethernet)  | RMII, MII mode
ICSS-EMAC                   | R5F            | YES               | FreeRTOS    | Switch and MAC features, Storm Prevention (MAC), Host Statistics, Multicast Filtering  | Promiscuous Mode
Mbed-TLS                                     | R5F            | NO                | FreeRTOS    | Tested software cryptography after porting, used mbedTLS with LwIP to implement HTTPS server  | Hardware offloaded cryptography

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
ECC               | R5F             | NA                |  NORTOS | ECC of MSS_L2, R5F TCM, MCAN, VIM, ICSSM, TPTC      | R5F data Cache(DED)
ECC Bus Safety    | R5F             | NA                |  NORTOS | AHB, AXI, TPTC                           | -
CCM               | R5F             | NA                |  NORTOS | CCM Self Test Mode,Error Forcing Mode and Self Test Error Forcing Mode.                      | -
R5F STC(LBIST), Static Register Read| R5F               | NA                |  NORTOS | STC of R5F, R5F CPU Static Register Read                                 |-

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
    <td> 08.06.00 Onwards
    <td> AM263x, AM263Px
    <td>
</tr>
<tr>
    <td> PROC_SDL-6910
    <td> Update to move some of the non static registers.
    <td> R5F CPU UTILS
    <td> 09.00.00 Onwards
    <td> AM263x, AM263Px
    <td> Updated R5F UTILS structure to move some of the non static registers.
</tr>
<tr>
    <td> PROC_SDL-5979
    <td> ECC test on R5F cache memories needed to be implemented.
    <td> ECC on Cache memory
    <td> 08.06.00 Onwards
    <td> AM263x
    <td> Added examples for ECC test on R5F cache memories.
</tr>
<tr>
    <td> MCUSDK-13491
    <td> API EPWM_setActionQualifierShadowLoadMode does not set Shadow Mode
    <td> EPWM
    <td> 09.02.00
    <td> AM263x, AM263Px
    <td>
</tr>
<tr>
    <td> MCUSDK-13199
    <td> EPWM : HRPWM_setHiResCounterCompareValue writes incorrect value
    <td> EPWM
    <td> 09.02.00
    <td> AM263x, AM263Px
    <td>
</tr>
<tr>
    <td> PINDSW-8097
    <td> Wrong PHY Config when using the QSPI Boot mode
    <td> ICSS-EMAC
    <td> 09.02.00 Onwards
    <td> AM263x, AM263Px
    <td> Fixed the application intiialization sequence and added required delay for PHY Powerup to SMI ready.
</tr>
<tr>
    <td> MCUSDK-13531
    <td> UART DMA transfer fail
    <td> UART
    <td> 09.02.00 Onwards
    <td> AM263x, AM263Px
    <td> Added typecasting for UART Transaction in driver.
</tr>
<tr>
    <td> MCUSDK-13427
    <td> McSPI 3 Pin mode failure in DMA mode
    <td> McSPI
    <td> 09.02.00 Onwards
    <td> AM263x, AM263Px
    <td> Update XBAR config in 3 Pin mode.
</tr>
<tr>
    <td> MCUSDK-13275
    <td> UART Clock selection missing options in SysCfg
    <td> UART
    <td> 09.02.00 Onwards
    <td> AM263x, AM263Px
    <td> Updated SysCfg module to add UART clock selection.
</tr>
<tr>
    <td> MCUSDK-13210
    <td> GPMC PSRAM IO failure
    <td> GPMC
    <td> 09.02.00 Onwards
    <td> AM263x
    <td> BP Input pin pulled down in GPIO SysCfg.
</tr>
<tr>
    <td> MCUSDK-12651
    <td> Data flush missing DMA mode
    <td> UART
    <td> 09.02.00 Onwards
    <td> AM263x, AM263Px
    <td> Added Data flush in UART DMA TX ISR.
</tr>
<tr>
    <td> MCUSDK-9459
    <td> UART DMA transfer fail for Trigger level > 1
    <td> UART
    <td> 09.02.00 Onwards
    <td> AM263x, AM263Px
    <td> Added trigger level selection support in SysCfg.
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
    <td> MCUSDK-13702
    <td> am263x-lp: sbl sd not working for multicore appimages
    <td> MMCSD, SBL
    <td> 10.00.00
    <td> -
</tr>
<tr>
    <td> MCUSDK-13164
    <td> AM263x: EPWM deadband example failure
    <td> EPWM
    <td> 09.02.00
    <td> remove sync between the epwms and use the global tbclksync to synchronize the EPWMs
</tr>
<tr>
    <td> MCUSDK-13641, CODEGEN-12832
    <td> Increased build time for examples using Link Time Optimization (-flto) with TI-ARM-CLANG 4.0.0 LTS
    <td> Build
    <td> 10.00.00 onwards
    <td> -
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
    <td> MCUSDK-11730
    <td> A wrong counter is used for Event 2 in PMU configuration
    <td> PMU
    <td> 09.00.00 onwards
    <td> -
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
    <td> PROC_SDL-5979
    <td> R5F Cache ECC diagnostics are not supported.
    <td> SDL
    <td> 8.5.0 onwards
    <td> None.
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
    <td> PROC_SDL-7615
    <td> ECC example fails for SEC and DED for TPTC memories.
    <td> SDL
    <td> 09.02.00 onwards
    <td> None
</tr>
<tr>
    <td> MCUSDK-13202
    <td> Frame drops are seen in PRU GPIO based SENT decoder example while sending frames in burst format
    <td> PRU-IO
    <td> 09.00.00 onwards
    <td> None
</tr>
<tr>
    <td> PROC_SDL-8392
    <td> In ECC bus safety example, ECC error is not properly cleared at the source.
    <td> SDL
    <td> 08.06.00 onwards
    <td> None
</tr>
<tr>
    <td> PROC_SDL-8393
    <td> In ECC bus safety, error injection test writes to address 0x0.
    <td> SDL
    <td> 08.06.00 onwards
    <td> None
</tr>
<tr>
    <td> PROC_SDL-8519
    <td> In ECC for R5F data cache only, double bit test is not supported.
    <td> SDL
    <td> 10.00.00 onwards
    <td> ECC test for single bit injection on R5F data cache, release profile binary is showing some inconsistency on result.
</tr>
<tr>
    <td> PROC_SDL-8518
    <td> Integrated example should have checked ECC for TPTC, ATCM, BTCM memories.
    <td> SDL
    <td> 10.00.00 onwards
    <td> None.
</tr>
<tr>
    <td> MCUSDK-13466
    <td> UART Transfer fails in 10MHz Auto Baud mode
    <td> UART
    <td> 10.00.00 onwards
    <td> Use different mode for 10MHz clock
</tr>
<tr>
    <td> MCUSDK-13193
    <td> SBL SD transfer time increased w.r.t SDK 9.2
    <td> SBL
    <td> 10.00.00 onwards
    <td> None.
</tr>
<tr>
    <td> MCUSDK-13511
    <td> MPU region count incorrect in SysCfg
    <td> MPU Firewall
    <td> 09.02.00 onwards
    <td> None.
</tr>
<tr>
    <td> MCUSDK-13473
    <td> UART uniflash script fails with large images ( > 1MB)
    <td> SBL
    <td> 10.00.00 onwards
    <td> Use JTAG based flashing
</tr>
<tr>
    <td> MCUSDK-11730
    <td> A wrong counter is used for Event 2 in PMU configuration
    <td> PMU
    <td> 10.00.00 onwards
    <td> Comment out the code in PMU_init() whcih configures Cycle Counter.
</tr>
<tr>
    <td> MCUSDK-13630
    <td> Cache should not be enabled at L2 Bank boundaries
    <td> Cache
    <td> Cache should not be enabled at last 32Bytes of L2 Bank
    <td> Create MPU configurations for end of each L2 Bank with Non Cached attribute
</tr>
<tr>
    <td> MCUSDK-13165
    <td> SBL QSPI flow has incorrect addressing for Secure Boot flow
    <td> SBL, QSPI
    <td> SBL QSPI flow has incorrect addressing for Secure Boot flow since it expects MEMMAP configuration
    <td> Use MCELF Image format
</tr>
<tr>
    <td> MCUSDK-13652
    <td> Readelf throws warning while parsing RS note
    <td> SBL, QSPI
    <td> Readelf command throws error when trying to read the RS note segment from an mcelf file.
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
    <td> Open
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
    <td> Sysconfig
    <td> EPWM
    <td> TBCLKSYNC and Halt configurations moved to ti_drivers_open_close.c
    <td> The TBCLKSYNC should be not enabled until the init configurations are done. The individual control to enable the tbclksyn in the init or not is added. Refer to SOC_setMultipleEpwmTbClk for usage in the applications.
</tr>
<tr>
    <td> Sysconfig
    <td> ADC
    <td> Added Internal Refernece enable controls
    <td> Default is set as enabled, to maintain backward compatibility
</tr>
<tr>
    <td> ADC
    <td> SOC_enableAdcInternalReference, SOC_enableAdcReferenceMonitor, SOC_getAdcReferenceStatus
    <td> Added Internal Refernece enable controls in drivers
    <td> Reference monitoring status should be checked before powering up the ADC analog converter.
</tr>
<tr>
    <td> EPWM
    <td> EPWM_setActionQualifierShadowLoadMode
    <td> updated parenthesis for API operations
    <td> -
</tr>
<tr>
    <td> EPWM
    <td> HRPWM_setHiResCounterCompareValue
    <td> Updated the Assert check. Fixed Overwriting to the CMPx register.
    <td> -
</tr>
<tr>
    <td> Security
    <td> HSM Client, Secure IPC Notify, Crypto driver
    <td> These drivers are moved to "source/security/security_common". All drivers will be part of security libs.
    <td> Update the include paths and included libraries for application build.
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
</table>
