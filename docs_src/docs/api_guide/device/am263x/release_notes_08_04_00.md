# Release Notes 08.04.00 {#RELEASE_NOTES_08_04_00_PAGE}

[TOC]

\attention Also refer to individual module pages for more details on each feature, unsupported features, important usage guidelines.

\note The examples will show usage of SW modules and APIs on a specific CPU instance and OS combination. \n
      Unless explicitly noted otherwise, the SW modules would work in both FreeRTOS and no-RTOS environment. \n
      Unless explicitly noted otherwise, the SW modules would work on any of the R5F's present on the SOC. \n

## New in this Release

Feature                                                                                         | Module
------------------------------------------------------------------------------------------------|-----------------------------------
Syconfig support and flash driver redesign to enable new flash configuration                    | OSPI
Optimized trigonometric funtion support on R5 core                                              | Common
McSPI Driver update for performance improvement                                                 | McSPI
LIN driver and example support                                                                  | LIN
EPWM: PWM protection and latency example using PRU                                              | EPWM
SBL support to configure R5 in lockstep or dualcore mode                                        | SBL
Syncronization support for 2 SDFM instances                                                     | SDFM
High sampling rate examples for burst mode                                                      | ADC
TCP/IP Checksum offload to hardware support enabled in ethernet CPSW driver                     | CPSW
Simplified LwIP CPSW examples added for reference                                               | CPSW
CPSW Scatter-Gather and Interrupt pacing is enabled in enet driver and in reference example     | CPSW
CPSW driver memory footprint reduced by more than 40%                                           | CPSW
CPSW Layer-2 performance is benchmarked and added a reference example                           | CPSW
CPSW driver support added for MDIO manual mode                                                  | CPSW
CPSW ALE congurations are moved to SysConfig GUI                                                | CPSW
CPSW DSCP Priority mapping and Policer usecase is added in the CPSW example                     | CPSW
Multi-core (R5) support added for CPSW driver                                                   | CPSW


## Device and Validation Information

\cond SOC_AM263X
SOC   | Supported CPUs  | EVM                                                                          | Host PC
------|-----------------|------------------------------------------------------------------------------|-----------------------------------------
AM263x| R5F             | AM263x ControlCard Revision E1  (referred to as am263x-cc in code). \n       | Windows 10 64b or Ubuntu 18.04 64b
AM263x| R5F             | AM263x LaunchPad Revision E1  (referred to as am263x-lp in code)             | Windows 10 64b or Ubuntu 18.04 64b
\endcond

## Dependent Tools and Compiler Information

Tools                   | Supported CPUs | Version
------------------------|----------------|-----------------------
Code Composer Studio    | R5F            | @VAR_CCS_VERSION_AM263X
SysConfig               | R5F            | @VAR_SYSCFG_VERSION_AM263X, build @VAR_SYSCFG_BUILD_AM263X
TI ARM CLANG            | R5F            | @VAR_TI_ARM_CLANG_VERSION
FreeRTOS Kernel         | R5F            | @VAR_FREERTOS_KERNEL_VERSION
LwIP                    | R5F            | @VAR_LWIP_VERSION

\attention TI ARM CLANG @VAR_TI_ARM_CLANG_VERSION is not part of CCS by default, Follow steps at \ref INSTALL_TIARMCLANG to install the compiler

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
Bootloader | R5FSS0-0        | YES               | NORTOS           | Boot modes: QSPI, UART. All R5F's. RPRC, multi-core image format            | Force Dual Core Mode, Disable Dual Core Switch and R5SS1 only not tested

### SOC Device Drivers

Peripheral   | Supported CPUs | SysConfig Support | DMA Supported                         | Key features tested                                                                                                                                             | Key features not tested / NOT supported
-------------|----------------|-------------------|---------------------------------------|-----------------------------------------------------------------------------------------------------------------------------------------------------------------|---------------------------------------------------
ADC          | R5F            | YES               | Yes. Example:  adc_soc_continuous_dma | Single software triggered conversion, Multiple ADC trigger using PWM, Result read using DMA, EPWM trip through PPB limit, burst mode oversampling, differential mode                | -
Bootloader   | R5F            | YES               | Yes. DMA enabled for SBL QSPI         | Boot modes: QSPI, UART. All R5F's                                                                                                                               | -
CMPSS        | R5F            | YES               | NA                                    | Asynchronous PWM trip                                                                                                                                           | -
CPSW         | R5F            | YES               | No                                    | MAC loopback, PHY loopback, LWIP: Getting IP, Ping, Iperf, Layer 2 MAC, Layer 2 PTP Timestamping and CPSW Switch support                                        | -
DAC          | R5F            | YES               | Yes. Example: dac_sine_dma            | Constant voltage, Square wave generation, Sine wave generation with and without DMA, Ramp wave generation, Random Voltage generation                            | -
ECAP         | R5F            | YES               | No                                    | ECAP APWM mode, PWM capture                                                                                                                                     | -
EDMA         | R5F            | YES               | NA                                    | DMA transfer using interrupt and polling mode, QDMA Transfer, Channel Chaining, PaRAM Linking                                                                   | -
EPWM         | R5F            | YES               | Yes. Example: epwm_dma                | PWM outputs A and B in up-down count mode, Trip zone, Update PWM using EDMA, Valley switching, High resolution time period adjustment, type5 feature            | -
EQEP         | R5F            | YES               | NA                                    | Speed and Position measurement.                                                                                                                                 | Frequency Measurement not tested
FSI          | R5F            | YES               | Yes. Example: fsi_loopback_dma        | RX, TX, polling, interrupt mode, Dma, single lane loopback.                                                                                                     | - FSI Spi Mode
GPIO         | R5F            | YES               | NA                                    | Output, Input and Interrupt functionality                                                                                                                       | -
I2C          | R5F            | YES               | No                                    | Controller mode, basic read/write                                                                                                                                   | -
IPC Notify   | R5F            | YES               | NA                                    | Mailbox functionality, IPC between RTOS/NORTOS CPUs                                                                                                             | M4F core
IPC Rpmsg    | R5F            | YES               | NA                                    | RPMessage protocol based IPC                                                                                                                                    | M4F core
MCAN         | R5F            | YES               | No                                    | RX, TX, interrupt and polling mode, Corrupt Message Transmission Prevention, Error Passive state, Bus Off State, Bus Monitoring Mode                            | -
MCSPI        | R5F            | YES               | Yes. Example: mcspi_loopback_dma      | Controller/Peripheral mode, basic read/write, polling, interrupt and DMA mode                                                                                            | -
MDIO         | R5F            | YES               | NA                                    | Register read/write, link status and link interrupt enable API                                                                                                  | -
MPU Firewall | R5F            | YES               | NA                                    | Only compiled (Works only on HS-SE  device)                                                                                                                     | -
PINMUX       | R5F            | YES               | NA                                    | Tested with multiple peripheral pinmuxes                                                                                                                        | -
PRUICSS      | R5F            | YES               | NA                                    | Tested with Ethercat FW HAL                                                                                                                                     | -
QSPI         | R5F            | YES               | Yes. Example: qspi_flash_dma_transfer | Read direct, Write indirect, Read/Write commands, DMA for read                                                                                                  | -
RTI          | R5F            | YES               | No                                    | Counter read, timebase selction, comparator setup for Interrupt, DMA requests                                                                                   | Capture feature, fast enabling/disabling of events not tested
SDFM         | R5F            | YES               | No                                    | Filter data read from CPU, Filter data read with PWM sync                                                                                                       | -
SOC          | R5F            | YES               | NA                                    | Lock/unlock MMRs, clock enable, set Hz, Xbar configuration, SW Warm Reset, Address Translation                                                                  | -
SPINLOCK     | R5F            | NA                | NA                                    | Lock, unlock HW spinlocks                                                                                                                                       | -
UART         | R5F            | YES               | Yes. Example: uart_echo_dma           | Basic read/write at baud rate 115200, polling, interrupt mode                                                                                                   | HW flow control not tested, DMA mode not supported
WATCHDOG     | R5F            | YES               | NA                                    | Reset mode                                                                                                                                                      | Interrupt mode

### Board Device Drivers

Peripheral | Supported CPUs | SysConfig Support | Key features tested                                         | Key features not tested
-----------|----------------|-------------------|-------------------------------------------------------------|------------------------
EEPROM     | R5F            | YES               | Only compiled                                               | -
ETHPHY     | R5F            | YES               | Tested with ethercat_peripheral_beckhoff_ssc_demo example        | -
FLASH      | R5F            | YES               | QSPI Flash                                                  | -
LED        | R5F            | YES               | GPIO                                                        | -


### CMSIS

Module                      | Supported CPUs | SysConfig Support | OS Support        | Key features tested                                                                         | Key features not tested
----------------------------|----------------|-------------------|-------------------|---------------------------------------------------------------------------------------------|------------------------
-                           | -              | -                 | -                 | -                                                                                           |  -

### Industrial Communications Toolkit

Module                                | Supported CPUs | SysConfig Support | OS Support  | Key features tested                                                                                      | Key features not tested
--------------------------------------|----------------|-------------------|-------------|----------------------------------------------------------------------------------------------------------|------------------------
EtherCAT Peripheral FWHAL                  | R5F            | NO                | FreeRTOS    | Tested with ethercat_peripheral_beckhoff_ssc_demo example                                                     | Reset isolation

### Motor Control

Module                      | Supported CPUs | SysConfig Support | OS Support        | Key features tested                             | Key features not tested
----------------------------|----------------|-------------------|-------------------|-------------------------------------------------|------------------------
-                           | -              | -                 | -                 | -                                               |  -

### Networking

Module                      | Supported CPUs | SysConfig Support | OS Support  | Key features tested                                                                    | Key features not tested
----------------------------|----------------|-------------------|-------------|----------------------------------------------------------------------------------------|------------------------
LwIP                        | R5F            | YES               | FreeRTOS    | TCP/UDP IP networking stack with and without checksum offload enabled,, TCP/UDP IP networking stack, DHCP, ping, TCP iperf, TCP/UDP IP, scatter-gather                         | Other LwIP features, checksum offload with VLAN_Tag, more robustness tests pending
Ethernet driver (ENET)      | R5F            | YES               | FreeRTOS    | Ethernet as port using CPSW and ICSS,Layer 2 MAC, Layer 2 PTP Timestamping, CPSW Switch, CPSW EST, interrupt pacing, Policer, MDIO Manual Mode | -
ICSS-EMAC                   | R5F            | YES               | FreeRTOS    | Only compiled                                                                          | Not tested


### Demos

Module                      | Supported CPUs | SysConfig Support | OS Support        | Key features tested                             | Key features not tested
----------------------------|----------------|-------------------|-------------------|-------------------------------------------------|------------------------
-                           | -              | -                 | -                 | -                                               |  -

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
    <td> MCUSDK-4050
    <td> Long CS assert/deassert delay before and after the MCSPI data transfer
    <td> McSPI
    <td> 8.2.0 onwards
    <td> AM263x
    <td> Fixed
</tr>
<tr>
    <td> MCUSDK-4190
    <td> Fixed MAC address is hard-coded for CPDMA devices, this will cause issue when two boards are connected in the same network
    <td> Enet
    <td> 8.2.0 onwards
    <td> AM263x
    <td> Fixed
</tr>
<tr>
    <td> MCUSDK-4345
    <td> EDMA: Region interrupt not triggered if Channel Id and TCC are not equal
    <td> EDMA
    <td> 8.2.0 onwards
    <td> AM263x
    <td> Fixed
</tr>
<tr>
    <td> MCUSDK-4357
    <td> hw_types.h should include <stdint.h> to resolve dependent includes
    <td> Common
    <td> 8.2.0 onwards
    <td> AM263x
    <td> Fixed
</tr>
<tr>
    <td> MCUSDK-4538
    <td> [SBL] sbl qspi incorrect core number displayed for R5FSS1-0
    <td> SBL
    <td> 8.3.0 onwards
    <td> AM263x
    <td> Fixed
</tr>
<tr>
    <td> MCUSDK-6263
    <td> ADC interrupt are not cleared properly in ADC_soc_continuous_dma example
    <td> ADC
    <td> 8.3.0 onwards
    <td> AM263x
    <td> Fixed
</tr>
<tr>
    <td> MCUSDK-7303
    <td> MCAN_getTxBufCancellationIntrEnable is misnamed
    <td> MCAN
    <td> 8.3.0 onwards
    <td> AM263x
    <td> Fixed
</tr>
<tr>
    <td> MCUSDK-7304
    <td> CPSW: LWIP example doesn't work on port 2
    <td> CPSW
    <td> 8.3.0 onwards
    <td> AM263x
    <td> Fixed
</tr>
<tr>
    <td> MCUSDK-7308
    <td> Multiple RTI instances selection leads to duplicates variable declaration in auto generated files
    <td> RTI & Sysconfig
    <td> 8.3.0 onwards
    <td> AM263x
    <td> Fixed
</tr>
<tr>
    <td> MCUSDK-7322
    <td> ADC: minimum Acquistion window should be 16
    <td> ADC
    <td> 8.3.0 onwards
    <td> AM263x
    <td> Fixed
</tr>
<tr>
    <td> MCUSDK-7342
    <td> Unable to add more than one instance of Output XBAR in sysconfig
    <td> Sysconfig
    <td> 8.3.0 onwards
    <td> AM263x
    <td> Fixed
</tr>
<tr>
    <td> MCUSDK-7813
    <td> ADC: Interrupts may Stop if INTxCONT (Continue-to-Interrupt Mode) is not Set
    <td> ADC
    <td> 8.3.0 onwards
    <td> AM263x
    <td> Fixed
</tr>
<tr>
    <td> MCUSDK-7827
    <td> CMPSS: Hysteresis APIs overwrite other register fields
    <td> CMPSS
    <td> 8.3.0 onwards
    <td> AM263x
    <td> Fixed
</tr>
<tr>
    <td> MCUSDK-7833
    <td> CMPSS: Invalid macros supported for input parameters to CMPSS_configLowComparator API
    <td> CMPSS
    <td> 8.3.0 onwards
    <td> AM263x
    <td> Fixed
</tr>
<tr>
    <td> MCUSDK-7834
    <td> CMPSS sysconfig: COMPL does not show configurable positive input source
    <td> CMPSS
    <td> 8.3.0 onwards
    <td> AM263x
    <td> Fixed
</tr>
<tr>
    <td> MCUSDK-7835
    <td> CMPSS sysconfig: CMPSSB shows invalid configurables
    <td> CMPSS
    <td> 8.3.0 onwards
    <td> AM263x
    <td> Fixed
</tr>
<tr>
    <td> MCUSDK-7838
    <td> CMPSS sysconfig: Incorrect the PWM sync source range and Blanking source range
    <td> CMPSS
    <td> 8.3.0 onwards
    <td> AM263x
    <td> Fixed
</tr>
<tr>
    <td> MCUSDK-7946
    <td> Invalid interrupt source configuration for burst mode oversampling, differential mode ADC example
    <td> ADC
    <td> 8.3.0 onwards
    <td> AM263x
    <td> Fixed
</tr>
<tr>
    <td> MCUSDK-8039
    <td> CPSW: Syscfg should not have two instances of CPSW for enabling both RGMIIs
    <td> CPSW
    <td> 8.3.0 onwards
    <td> AM263x
    <td> Fixed
</tr>
<tr>
    <td> MCUSDK-6207
    <td> A53x, Linux references in documentation
    <td> Common
    <td> 8.3.0 onwards
    <td> Fixed
</tr>
<tr>
    <td> MCUSDK-7333
    <td> DEV_ID and Core IDs table is not mentioned in documentation
    <td> Common
    <td> 8.3.0 onwards
    <td> Fixed
</tr>
<tr>
    <td> MCUSDK-7363
    <td> [enet] Documentation of file name incorrect in sysconfig generated file
    <td> Enet
    <td> 8.3.0 onwards
    <td> Fixed
</tr>
<tr>
    <td> MCUSDK-8115
    <td> HRPWM_getHiResTimeBasePeriod() API function doesn't return correct value
    <td> EPWM
    <td> 8.2.0 onwards
    <td> Fixed
</tr>
<tr>
    <td> MCUSDK-8116
    <td> HRPWM_getCounterCompareValue() API function doesn't return correct value
    <td> EPWM
    <td> 8.2.0 onwards
    <td> Fixed
</tr>
<tr>
    <td> MCUSDK-8117
    <td> HRPWM_getHiResCounterCompareValueOnly() API function doesn't return correct value
    <td> EPWM
    <td> 8.2.0 onwards
    <td> Fixed
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
    <td> MCUSDK-1016
    <td> Semaphore does not function as expected when "post" call is present in multiple ISRs at different priorities
    <td> DPL, FreeRTOS
    <td> 8.0.0
    <td> Disable interrupt nesting
</tr>
<tr>
    <td> MCUSDK-2294
    <td> GPIO Pin Direction
    <td> GPIO. GPIO Pin Direction not getting automatically configured.
    <td> 8.0.0
    <td> Use GPIO_setDirMode to set pin direction for GPIO pin.
</tr>
<tr>
    <td> MCUSDK-2557
    <td> eqep_frequency_measurement example is failing
    <td> SBL
    <td> 8.2.0
    <td> -
</tr>
<tr>
    <td> MCUSDK-4059
    <td> AM263x: FSI first frame transmitted is incorrect in DMA mode
    <td> FSI
    <td> 8.3.0
    <td> Please refer fsi_loopback_dma example.
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
    <td> MCUSDK-6909
    <td> EPWM: Emulation mode doesn't work
    <td> EPWM
    <td> 8.4.0
    <td> -
</tr>
<tr>
    <td> MCUSDK-7030
    <td> [NORTOS]Interrupt nesting is not functional as expected when you have 2 or more interrupts with different priorities
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
    <td> MCUSDK-7811
    <td> CPSW: Ethernet Packet corruption occurs if CPDMA fetches a packet which spans across memory banks
    <td> CPSW
    <td> 8.3.0 onwards
    <td> Ensure from application side single ethernet packet does not span across memory banks.
</tr>
<tr>
    <td> MCUSDK-7915
    <td> SDFM: EPWM filter sync example does not configure and check the PWM synchronization
    <td> SDFM
    <td> 8.3.0 onwards
    <td> None
</tr>
<tr>
    <td> MCUSDK-8072
    <td> [Enet] EnetBoard_setupPorts does not provide config option to enable internal delay for RGMII
    <td> Enet
    <td> 8.4.0 onwards
    <td> None
</tr>
<tr>
    <td> MCUSDK-8073
    <td> UART1 not working as expected while configuring two uarts i.e UART0 and UART1 for two different cores
    <td> UART
    <td> 8.4.0 onwards
    <td> UART1 configuration from other core should be done after UART0 is configured and initialized
</tr>
<tr>
    <td> MCUSDK-8079
    <td> EDMA: Aggregated interrupt used instead of Region interrupt
    <td> EDMA
    <td> 8.4.0 onwards
    <td> None
</tr>
<tr>
    <td> MCUSDK-8235
    <td> AM263x LP: CMPSS: Example fails - PWM not tripped
    <td> CMPSS
    <td> 8.4.0 onwards
    <td> None
</tr>
<tr>
    <td> MCUSDK-8348
    <td> EnetDma_initPktInfo does not initialized chkSumInfo member
    <td> Enet
    <td> 8.4.0 onwards
    <td> All L2 based applications need to explicitly set EnetDma_initPktInfo.chkSumInfo = 0
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
    <td> Enabled copmpiler option -Oz and -flto for release mode build
    <td> These option are enabled for Code size and performance optimization
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
    <td> MCSPI
    <td> MCSPI_chConfig
    <td> Added parameter for trigger level configuration `txFifoTrigLvl`, `rxFifoTrigLvl` in \ref MCSPI_ChConfig
    <td> Allows configuring tragger level through sys config. Regenerate the sys config files
</tr>
<tr>
    <td> MCAN
    <td> MCAN_getTxBufCancellationIntrEnable
    <td> Name changed from MCAN_getTxBufCancellationIntrEnable to MCAN_txBufCancellationIntrEnable.
    <td> MCAN_getTxBufCancellationIntrEnable function sets (does not get). So renamed to MCAN_txBufCancellationIntrEnable.
</tr>
<tr>
    <td> CMPSS
    <td> CMPSS_configLowComparator
    <td> Values (macros) that can be passed to CMPSS_configLowComparator()
    <td> Changed from CMPSS_INSRC_DAC, CMPSS_INSRC_PIN to CMPSS_INSRC_PIN_INL and CMPSS_INSRC_PIN_INH
</tr>
<tr>
    <td> CMPSS sysconfig
    <td> lowCompPositive
    <td> In CMPSSA 'Low Comparator Configuration', 'Negative Input Source' changed to 'Positive Input Source'. Valid values changed to CMPSS_INSRC_PIN_INL and CMPSS_INSRC_PIN_INH
    <td> CMPSSA low comparator does not have configurable negative input source. Low comparator Positive input source is configurable between pin INL and pin INH
</tr>
<tr>
    <td> CMPSS sysconfig
    <td> CMPSSB highCompNegative and CMPSSB lowCompPositive
    <td> In CMPSSB 'High Comparator Configuration' -> 'Negative Input Source' and 'Low Comparator Configuration' -> 'Positive Input Source' are removed.
    <td> CMPSSB high comparator and low comparator does not have configurable positive/negative input source.
</tr>
<tr>
    <td> SBL
    <td> Bootloader_socCpuPowerOnReset
    <td> Added argument to configure the R5 sub system to lockstep or dualcore mode
    <td> Function Bootloader_socCpuPowerOnReset() requires 2 arguments, cpuID and cpu operating mode
</tr>

</table>
