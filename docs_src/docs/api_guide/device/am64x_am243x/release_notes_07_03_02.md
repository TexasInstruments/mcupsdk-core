# Release Notes 07.03.02 {#RELEASE_NOTES_07_03_02_PAGE}

[TOC]

\attention Also refer to individual module pages for more details on each feature, unsupported features, important usage guidelines.

\note The examples will show usage of SW modules and APIs on a specific CPU instance and OS combination. \n
      Unless explicitly noted otherwise, the SW modules would work in both FreeRTOS and no-RTOS environment. \n
      Unless explicitly noted otherwise, the SW modules would work on any of the R5F's present on the SOC. \n
      All SW modules will not work on M4F. M4F support for a module will be explicitly called out.

## New in this Release

Feature                                                         | Module
----------------------------------------------------------------|--------------------------
FreeRTOS APIs for event group                                   | DPL
DebugP_log to memory and view in ROV                            | DPL
TinyUSB stack with CDC                                          | USB
DDR driver support                                              | DDR
eQEP driver support                                             | eQEP
eCAP driver support                                             | eCAP
IPC RPMessage RX notify callback                                | IPC RPMessage
MCSPI Read RX data low-overhead API                             | McSPI
Generic NOR Flash APIs for 1s1s1s mode as part of OSPI driver   | OSPI

## Device and Validation Information

\cond SOC_AM64X
SOC   | Supported CPUs  | EVM                                             | Host PC
------|-----------------|-------------------------------------------------|-----------------------------------
AM64x | R5F, M4F        | AM64x GP EVM (referred to as am64x-evm in code) | Windows 10 64b or Ubuntu 18.04 64b
\endcond

\cond SOC_AM243X
SOC    | Supported CPUs  | EVM                                               | Host PC
-------|-----------------|---------------------------------------------------|-----------------------------------
AM243x | R5F, M4F        | AM243x GP EVM (referred to as am243x-evm in code) | Windows 10 64b or Ubuntu 18.04 64b
\endcond

## Dependent Tools and Compiler Information

Tools                   | Supported CPUs | Version
------------------------|----------------|-----------------------
Code Composer Studio    | R5F, M4F       | @VAR_CCS_VERSION
SysConfig               | R5F, M4F       | @VAR_SYSCFG_VERSION, build @VAR_SYSCFG_BUILD
TI ARM CLANG            | R5F, M4F       | @VAR_TI_ARM_CLANG_VERSION
FreeRTOS Kernel         | R5F, M4F       | @VAR_FREERTOS_KERNEL_VERSION

## Key Features

### OS Kernel

OS              | Supported CPUs  | SysConfig Support | Key features tested                                                                   | Key features not tested / NOT supported
----------------|-----------------|-------------------|---------------------------------------------------------------------------------------|----------------------------------------
FreeRTOS Kernel | R5F, M4F        | NA                | Task, Task notification, interrupts, semaphores, mutexs, timers. ROV views in CCS IDE | Task load measurement using FreeRTOS run time statistics APIs.
FreeRTOS POSIX  | R5F, M4F        | NA                | pthread, mqueue, semaphore, clock                                                     | -
NO RTOS         | R5F, M4F        | NA                | See **Driver Porting Layer (DPL)** below                                              | -

### Driver Porting Layer (DPL)

Module            | Supported CPUs  | SysConfig Support | OS support       | Key features tested                                           | Key features not tested / NOT supported
------------------|-----------------|-------------------|------------------|---------------------------------------------------------------|----------------------------------------
Address Translate | M4F             | YES               | FreeRTOS, NORTOS | Use RAT to allow M4F access to peripheral address space       | -
Cache             | R5F             | YES               | FreeRTOS, NORTOS | Cache write back, invalidate, enable/disable                  | -
Clock             | R5F, M4F        | YES               | FreeRTOS, NORTOS | Tick timer at user specified resolution, timeouts and delays  | -
CycleCounter      | R5F, M4F        | NA                | FreeRTOS, NORTOS | Measure CPU cycles using CPU specific internal counters       | -
Debug             | R5F, M4F        | YES               | FreeRTOS, NORTOS | Logging and assert to any combo of: UART, CCS, shared memory  | -
Heap              | R5F, M4F        | NA                | FreeRTOS, NORTOS | Create arbitrary heaps in user defined memory segments        | -
Hwi               | R5F, M4F        | YES               | FreeRTOS, NORTOS | Interrupt register, enable/disable/restore                    | -
MPU               | R5F, M4F        | YES               | FreeRTOS, NORTOS | Setup MPU and control access to address space                 | -
Semaphore         | R5F, M4F        | NA                | FreeRTOS, NORTOS | Binary, Counting Semaphore, recursive mutexs with timeout     | -
Task              | R5F, M4F        | NA                | FreeRTOS         | Create, delete tasks                                          | -
Timer             | R5F, M4F        | YES               | FreeRTOS, NORTOS | Configure arbitrary timers                                    | -
Event             | R5F, M4F        | YES               | FreeRTOS         | Setting, getting, clearing, and waiting of Event bits         | -

### Secondary Bootloader (SBL)

Module     | Supported CPUs  | SysConfig Support | OS support       | Key features tested                                                                             | Key features not tested / NOT supported
-----------|-----------------|-------------------|------------------|-------------------------------------------------------------------------------------------------|----------------------------------------
Bootloader | R5FSS0-0        | YES               | NORTOS           | Boot modes: OSPI, OSPI XIP, UART. All R5F's, M4F boot. RPRC, multi-core image format, DDR init  | SBL booting Linux. SBL booting A53.

### SOC Device Drivers

Peripheral | Supported CPUs | SysConfig Support | Key features tested                                                          | Key features not tested / NOT supported
-----------|----------------|-------------------|------------------------------------------------------------------------------|----------------------------------------
ADC        | R5F            | YES               | Single conversion (one-shot mode), interrupt mode, DMA mode                  | Continuous conversion not tested
CRC        | R5F            | NO                | Only compiled                                                                | Not tested
DDR        | R5F            | YES               | Tested LPDDR4 at 400MHz frequency.                                           | -
ECAP       | R5F            | YES               | Frequency, Duty cycle, interrupt mode                                        | PWM mode not tested
EPWM       | R5F            | YES               | Different Frequency, Duty cycle, interrupt mode, Deadband and chopper module | Tripzone module not tested
EQEP       | R5F            | YES               | Signal Frequency and Direction, interrupt mode                               | -
FSI (RX/TX)| R5F            | YES               | RX, TX, polling, interrupt mode, single/dual lanes                           | -
GPIO       | R5F, M4F       | YES               | Basic input/output, GPIO as interrupt                                        | -
I2C        | R5F, M4F       | YES               | Controller mode, basic read/write, polling and interrupt mode                    | Target mode not supported. M4F not tested due to EVM limitation
IPC Notify | R5F, M4F       | YES               | Low latency IPC between RTOS/NORTOS CPUs                                     | -
IPC Rpmsg  | R5F            | YES               | RPMessage protocol based IPC, All R5F and Linux A53 cores                    | M4F core not supported
MCAN       | R5F            | YES               | RX, TX, interrupt and polling mode                                           | -
MCSPI      | R5F, M4F       | YES               | Controller/Peripheral mode, basic read/write, polling, interrupt mode                 | DMA mode not supported
MDIO       | R5F            | NA                | Register read/write, link status and link interrupt enable API               | -
OSPI       | R5F            | YES               | Read direct, Write indirect, Read/Write commands, DMA for read, PHY Mode     | Interrupt mode not supported
Pinmux     | R5F, M4F       | YES               | Tested with multiple peripheral pinmuxes                                     | -
PRUICSS    | R5F            | YES               | Tested with Ethercat FW HAL, HDSL, EnDat                                     | More protocols integration tests pending
SOC        | R5F, M4F       | YES               | lock/unlock MMRs, get CPU clock, CPU name, clock enable, set frequency       | -
Sciclient  | R5F, M4F       | YES               | Tested with clock setup, module on/off                                       | -
SPINLOCK   | R5F, M4F       | NA                | Lock, unlock HW spinlocks                                                    | -
UART       | R5F, M4F       | YES               | Basic read/write, polling, interrupt mode,                                   | HW flow control not tested. DMA mode not supported
UDMA       | R5F            | YES               | Basic memory copy, SW trigger, Chaining                                      | -

### Board Device Drivers

Peripheral | Supported CPUs | SysConfig Support | Key features tested                                         | Key features not tested
-----------|----------------|-------------------|-------------------------------------------------------------|------------------------
EEPROM     | R5F            | YES               | I2C based EEPROM                                            | -
ETHPHY     | R5F            | YES               | Ethernet Phy configuration for EtherCAT SubDevice example       | -
Flash      | R5F            | YES               | XSPI, OSPI, QSPI based flash, Octal, Quad mode, DDR mode    | All vendor flash types not tested
LED        | R5F            | YES               | GPIO and I2C IO expander based LED control                  | -

### CMSIS

Module                      | Supported CPUs | SysConfig Support | OS Support        | Key features tested                                                                         | Key features not tested
----------------------------|----------------|-------------------|-------------------|---------------------------------------------------------------------------------------------|------------------------
CMSIS DSP                   | R5F            | NA                | FreeRTOS, NORTOS  | Basic math, complex math, controller, fast math, filtering, Matrix, statistics, transform   | -

### Motor Control

Module                      | Supported CPUs | SysConfig Support | OS Support        | Key features tested                             | Key features not tested
----------------------------|----------------|-------------------|-------------------|-------------------------------------------------|------------------------
Position Sense HDSL         | R5F            | YES               | FreeRTOS, NORTOS  | Freerun mode, Sync mode                         |  -
Position Sense EnDAT        | R5F            | YES               | FreeRTOS, NORTOS  | Single channel, Multi channel, Continuous mode  |  -

### Networking

Module                      | Supported CPUs | SysConfig Support | OS Support  | Key features tested                                 | Key features not tested
----------------------------|----------------|-------------------|-------------|-----------------------------------------------------|------------------------
EtherCAT SubDevice FW HAL       | R5F            | YES               | FreeRTOS    | Tested with Beckhoff's EtherCAT stack based example | Online firmware update, reset isolation
EtherCAT SubDevice ETG Stack    | R5F            | NO                | FreeRTOS    | Standard stack (NOT BUNDLED in SDK)                 | -
EtherNet/IP Adapter FW HAL  | R5F            | NO                | FreeRTOS    | Only compiled                                       | Not tested
Profinet Device FW HAL      | R5F            | NO                | FreeRTOS    | Only compiled                                       | Not tested
ICSS-EMAC                   | R5F            | YES               | FreeRTOS    | Only compiled                                       | Not tested
ICSS TimeSync               | R5F            | NO                | FreeRTOS    | Only compiled                                       | Not tested

### USB

Module                      | Supported CPUs | SysConfig Support | OS Support         | Key features tested       | Key features not tested
----------------------------|----------------|-------------------|--------------------|---------------------------|------------------------
USB SoC Porting Layer       | R5F            | YES               | FreeRTOS, NORTOS   | USB 2.0 device mode       | USB 3.0
USB Device Driver           | R5F            | NO                | FreeRTOS, NORTOS   | USB 2.0 device mode       | USB Host driver
TinyUSB Core and CDC Driver | R5F            | NO                | FreeRTOS, NORTOS   | USB device with CDC class | USB Host, other USB device class drivers

\cond SOC_AM64X
### Demos

Module                      | Supported CPUs | SysConfig Support | OS Support        | Key features tested                             | Key features not tested
----------------------------|----------------|-------------------|-------------------|-------------------------------------------------|------------------------
Benchmark demo              | 4xR5F's        | YES               | NORTOS            | CFFT, FIR and FOC benchmarks                    |  ADC/PWM benchmark
\endcond

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
    <td> MCUSDK-596
    <td> Zero init of globals and static does not happen with current compiler and linker option
    <td> Common
    <td> 7.3.0, 7.3.1
    <td> AM64x, AM243x
    <td> Fixed
</tr>
<tr>
    <td> MCUSDK-627
    <td> Bootloader: R5FSS0_0 ATCM is not enabled after NULL SBL boot
    <td> Bootloader
    <td> 7.3.0, 7.3.1
    <td> AM64x, AM243x
    <td> Fixed
</tr>
<tr>
    <td> MCUSDK-675
    <td> XIP: OSPI @ 166Mhz with Phy enable and Phy pipeline enable and XIP does not work
    <td> Bootloader, XIP
    <td> 7.3.0, 7.3.1
    <td> AM64x, AM243x
    <td> Fixed
</tr>
<tr>
    <td> MCUSDK-648
    <td> DLFO bit is set unconditionally during CacheP_init() for R5 Core
    <td> DPL
    <td> 7.3.0, 7.3.1
    <td> AM64x, AM243x
    <td> Fixed
</tr>
<tr>
    <td> MCUSDK-177
    <td> DMTimer on M4F results in 2 interrupts instead of 1 for every DM timer expiry in release mode
    <td> DPL
    <td> 7.3.0
    <td> AM64x, AM243x
    <td> Fixed
</tr>
<tr>
    <td> MCUSDK-795
    <td> ClockP_usleep sleeps for more than the value passed
    <td> DPL
    <td> 7.3.0, 7.3.1
    <td> AM64x, AM243x
    <td> Fixed
</tr>
<tr>
    <td> MCUSDK-316
    <td> ROV does not work with DDR being used in application
    <td> ROV
    <td> 7.3.0, 7.3.1
    <td> AM64x, AM243x
    <td> Fixed
</tr>
<tr>
    <td> MCUSDK-354
    <td> EtherCAT State Machine from EtherCAT Conformance Test fails intermittently
    <td> EtherCAT
    <td> 7.3.0, 7.3.1
    <td> AM64x, AM243x
    <td> Fixed
</tr>
<tr>
    <td> MCUSDK-686
    <td> Incorrect MAC is compared for processing PTP frames in EIP_processProtocolFrames
    <td> EtherNet/IP Adapter FW HAL
    <td> 7.3.0, 7.3.1
    <td> AM64x, AM243x
    <td> Fixed
</tr>
<tr>
    <td> MCUSDK-796
    <td> bsp_pdi_write_indication not called after writing EEPROM Control/Status register write in bsp_init
    <td> EtherCAT SubDevice FW HAL
    <td> 7.3.0, 7.3.1
    <td> AM64x, AM243x
    <td> Fixed
</tr>
<tr>
    <td> MCUSDK-757
    <td> Incorrect pin calculation in SysConfig generated code for I2C IO expander
    <td> LED
    <td> 7.3.0, 7.3.1
    <td> AM64x, AM243x
    <td> Fixed
</tr>
<tr>
    <td> MCUSDK-781
    <td> SysConfig for multiple GPIO LED instances generate same code
    <td> LED
    <td> 7.3.0, 7.3.1
    <td> AM64x, AM243x
    <td> Fixed
</tr>
</table>

## Known Issues

<table>
<tr>
    <th> ID
    <th> Head Line
    <th> Module
    <th> Applicable Releases
    <th> Applicable Devices
    <th> Workaround
</tr>
<tr>
    <td> MCUSDK-317
    <td> OSPI: DMA reads fails after a flash write which was not block aligned
    <td> OSPI
    <td> 7.3.0 onwards
    <td> AM64x, AM243x
    <td> Use non-DMA read mode or use DMA mode but always write from start of a erased block
</tr>
<tr>
    <td> MCUSDK-683
    <td> UDMA: DMA from SRAM to M4F IRAM/DRAM fails
    <td> UDMA, TIFS
    <td> 7.3.0 onwards
    <td> AM64x, AM243x
    <td> Don't use these memories for DMA transfer or perform manual copy
</tr>
<tr>
    <td> MCUSDK-867
    <td> Rx is not selected in the pins for EtherCAT and Ethernet pinmux from SysConfig
    <td> EtherCAT SubDevice FW HAL, ICSS-EMAC
    <td> 7.3.0 onwards
    <td> AM64x, AM243x
    <td> Rx checkbox should to be selected manually
</tr>
<tr>
    <td> MCUSDK-750
    <td> ICSS PRU GPIO pins are not supported by SYSCFG
    <td> SysCfg
    <td> 7.3.0 onwards
    <td> AM64x, AM243x
    <td> Perform manual pinmux
</tr>
<tr>
    <td> MCUSDK-906
    <td> MCAN and ADC not functional in SBL UART bootmode with DDR enabled
    <td> Bootloader, DDR
    <td> 7.3.2
    <td> AM64x, AM243x
    <td> Explicitly enable DDR in the application instead of SBL UART.
</tr>
<tr>
    <td> MCUSDK-892
    <td> USB boot with SBL OSPI fails
    <td> USB
    <td> 7.3.2
    <td> AM64x, AM243x
    <td> Use UART boot or CCS loading
</tr><tr>
    <td> MCUSDK-626
    <td> DMA not working with ADC FIFO 1
    <td> ADC
    <td> 7.3.0 onwards
    <td> AM64x, AM243x
    <td> Use ADC FIFO 0
</tr>
</table>

## Limitations

<table>
<tr>
    <th> ID
    <th> Head Line
    <th> Module
    <th> Reported in Release
    <th> Applicable Devices
    <th> Workaround
</tr>
<tr>
    <td> MCUSDK-208
    <td> gmake with -j can sometimes lock up Windows command prompt
    <td> Build
    <td> 7.3.0
    <td> AM64x, AM243x
    <td> Use bash for windows as part of git for windows or don't use -j option
</tr>
</table>

## Upgrade and Compatibility Information

\attention When migrating from Processor SDK RTOS, see \ref MIGRATION_GUIDES for more details

### Examples

<table>
<tr>
    <th> Module
    <th> Affected API
    <th> Change
    <th> Additional Remarks
</tr>
<tr>
    <td> All examples
    <td> Linker command file
    <td> To enable support for zero init of uninitialized globals in .bss section an update in linker command file is needed.
    Add below in your project linker.cmd file if not already there.
    \code
    ...
        .bss:    {} palign(8)   /* This is where uninitialized globals go */
        RUN_START(__BSS_START)  /* ADD THIS LINE after the .bss line */
        RUN_END(__BSS_END)      /* AND THIS LINE as well */
    ...
    \endcode

    <td> All SDK examples are updated for this change. This change is needed in your application if it based off a older SDK.
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
    <td> EtherCAT SubDevice FWHAL
    <td> bsp_init
    <td> Added new parameter `enhancedlink_enable` in bsp_params
    <td> Provides flexibility to enable/disable enhanced link detection.
</tr>
<tr>
    <td> EtherNet/IP Adapter FW HAL
    <td> -
    <td> Added new parameter `macId` in `EIP_DLRHandle` structure
    <td> This field should be filled by the user before calling `EIP_drvInit`
</tr>
<tr>
    <td> EtherNet/IP Adapter FW HAL
    <td> -
    <td> `cipSyncConfig_t` structure inside `eip_Config` structure is allocated statically
    <td> -
</tr>
<tr>
    <td> ICSS-EMAC
    <td> -
    <td> Added `phyToMacInterfaceMod` and `txInterruptEnable` in ICSS_EMAC_Attrs
    <td> ICSS configuration is done for MII/RGMII based on the mode selected using `phyToMacInterfaceMode`. Task and interrupt for TX will be created only if `txInterruptEnable` is enabled.
</tr>
<tr>
    <td> ICSS-EMAC
    <td> ICSS_EMAC_Params_init
    <td> Moved `macId` from ICSS_EMAC_Attrs to ICSS_EMAC_Params
    <td> It provides flexibility to provide the mac address during run-time.
</tr>
<tr>
    <td> ICSS TimeSync
    <td> -
    <td> Added new parameter `macId` in `TimeSync_ParamsHandle` structure
    <td> This field should be filled by the user.
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
    <td> OSPI
    <td> -
    <td> Removed parameters `pageSize` and `blkSize` from \ref OSPI_Attrs structure and added new API \ref OSPI_setDeviceSize to set this. This has to be called in respective flash driver's open to set this.
    <td> The parameters can't be assumed at the time of OSPI driver open, so this has to be set with the \ref OSPI_setDeviceSize API. If not using OSPI APIs directly, this change can be ignored. This is already taken care in the provided flash drivers.
</tr>
<tr>
    <td> OSPI
    <td> -
    <td> Removed fields `rdDummyClks` and `extRdDummyClks` from \ref OSPI_Attrs and `rdStatusCmd` from \ref OSPI_Object as these are not used anymore
    <td> -
</tr>
<tr>
    <td> OSPI
    <td> -
    <td> Added support to set which type of command extension to be used in case of flashes with Octal DDR support. Can use the \ref OSPI_setCmdExtType API in the respective flash driver's open to set this.
    <td> If not using OSPI APIs directly, this change can be ignored. This is already taken care in the provided flash drivers.
</tr>
<tr>
    <td> OSPI
    <td> Internal APIs in respective flash drivers were changed.
    <td> Revamped the \ref OSPI_ReadCmdParams and \ref OSPI_WriteCmdParams structure to separate cmd and address. Also added a new field `numAddrBytes`.
    <td> If not using OSPI APIs directly, this change can be ignored. This is already taken care in the provided flash drivers.
</tr>
<tr>
    <td> OSPI
    <td> -
    <td> Added new fields `rdDataCapDelay`, `phyRdDataCapDelay`, `numAddrBytes` and `cmdExtType` to \ref OSPI_Object for book keeping.
    <td> These will be filled in the flash driver using APIs \ref OSPI_setRdDataCaptureDelay, \ref OSPI_setNumAddrBytes and \ref OSPI_setCmdExtType. The phyRdDataCapDelay variable will be updated by the PHY tuning algorithm
</tr>
<tr>
    <td> OSPI
    <td> `OSPI_setXferOpcode`
    <td> `OSPI_setXferOpcode` API is changed to \ref OSPI_setXferOpCodes. The parameter `readStatusCmd` has been removed from this API since it is no longer needed
    <td> -
</tr>

</table>
