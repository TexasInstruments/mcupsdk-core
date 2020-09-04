# Release Notes 08.00.03 {#RELEASE_NOTES_08_00_03_PAGE}

[TOC]

\attention Also refer to individual module pages for more details on each feature, unsupported features, important usage guidelines.

\note The examples will show usage of SW modules and APIs on a specific CPU instance and OS combination. \n
      Unless explicitly noted otherwise, the SW modules would work in both FreeRTOS and no-RTOS environment. \n
      Unless explicitly noted otherwise, the SW modules would work on any of the R5F's present on the SOC. \n

## Device and Validation Information

\cond SOC_AM263X
SOC   | Supported CPUs  | EVM                                                    | Host PC
------|-----------------|--------------------------------------------------------|-----------------------------------
AM263x| R5F             | AM263x ControlCard  (referred to as am263x-cc in code) | Windows 10 64b or Ubuntu 18.04 64b
\endcond


## Dependent Tools and Compiler Information

Tools                   | Supported CPUs | Version
------------------------|----------------|-----------------------
Code Composer Studio    | R5F            | @VAR_CCS_VERSION
SysConfig               | R5F            | @VAR_SYSCFG_VERSION, build @VAR_SYSCFG_BUILD
TI ARM CLANG            | R5F            | @VAR_TI_ARM_CLANG_VERSION
FreeRTOS Kernel         | R5F            | @VAR_FREERTOS_KERNEL_VERSION
LwIP                    | R5F            | @VAR_LWIP_VERSION

## Key Features

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
CycleCounter      | R5F             | NA                | FreeRTOS, NORTOS | Measure CPU cycles using CPU specific internal counters       | -
Debug             | R5F             | YES               | FreeRTOS, NORTOS | Logging and assert to any combo of: UART, CCS, shared memory  | -
Heap              | R5F             | NA                | FreeRTOS, NORTOS | Create arbitrary heaps in user defined memory segments        | -
Hwi               | R5F             | YES               | FreeRTOS, NORTOS | Interrupt register, enable/disable/restore                    | -
MPU               | R5F             | YES               | FreeRTOS, NORTOS | Setup MPU and control access to address space                 | -
Semaphore         | R5F             | NA                | FreeRTOS, NORTOS | Binary, Counting Semaphore, recursive mutexs with timeout     | -
Task              | R5F             | NA                | FreeRTOS         | Create, delete tasks                                          | -
Timer             | R5F             | YES               | FreeRTOS, NORTOS | Configure arbitrary timers                                    | -

### Secondary Bootloader (SBL)

Module     | Supported CPUs  | SysConfig Support | OS support       | Key features tested                                                         | Key features not tested / NOT supported
-----------|-----------------|-------------------|------------------|-----------------------------------------------------------------------------|----------------------------------------
Bootloader | R5FSS0-0        | YES               | NORTOS           | Boot modes: QSPI, UART. All R5F's. RPRC, multi-core image format            | -

### SOC Device Drivers

Peripheral   | Supported CPUs | SysConfig Support | Key features tested                                                                             | Key features not tested / NOT supported
-------------|----------------|-------------------|-------------------------------------------------------------------------------------------------|----------------------------------------
ADC          | R5F            | YES               | Single software triggered conversion                                                            | -
Bootloader   | R5F            | YES               | Boot modes: QSPI, UART. All R5F's                                                               | -
CMPSS        | R5F            | YES               | Asynchronous PWM trip                                                                           | -
CPSW         | R5F            | YES               | MAC loopback, PHY loopback, LWIP: Getting IP, Ping, Iperf                                       | -
DAC          | R5F            | YES               | Constant voltage and square wave generation                                                     | -
ECAP         | R5F            | YES               | ECAP APWM mode                                                                                  | -
EDMA         | R5F            | YES               | DMA transfer using interrupt and polling mode, QDMA Transfer, Channel Chaining, PaRAM Linking   | -
EPWM         | R5F            | YES               | PWM outputs A and B in up-down count mode                                                       | -
EQEP         | R5F            | YES               | Only compiled                                                                                   | -
FSI          | R5F            | YES               | Only compiled                                                                                   | -
GPIO         | R5F            | YES               | Output, Input and Interrupt functionality                                                       | -
I2C          | R5F            | YES               | Controller mode, basic read/write                                                                   | -
IPC Notify   | R5F            | YES               | Mailbox functionality, IPC between RTOS/NORTOS CPUs                                             | M4F core
IPC Rpmsg    | R5F            | YES               | RPMessage protocol based IPC                                                                    | M4F core
MCAN         | R5F            | YES               | RX, TX, interrupt and polling mode                                                              | -
MCSPI        | R5F            | YES               | Controller/Peripheral mode, basic read/write, polling, interrupt mode                                    | -
MDIO         | R5F            | YES               | Register read/write, link status and link interrupt enable API                                  | -
MPU Firewall | R5F            | YES               | Only compiled                                                                                   | -
PINMUX       | R5F            | YES               | Tested with multiple peripheral pinmuxes                                                        | -
PRUICSS      | R5F            | YES               | Tested with Ethercat FW HAL                                                                     | -
QSPI         | R5F            | YES               | Read direct, Write indirect, Read/Write commands, DMA for read                                  | -
SDFM         | R5F            | YES               | Filter data read from CPU                                                                       | -
SOC          | R5F            | YES               | Lock/unlock MMRs, clock enable, set Hz, Xbar configuration                                      | -
SPINLOCK     | R5F            | NA                | Lock, unlock HW spinlocks                                                                       | -
UART         | R5F            | YES               | Basic read/write at baud rate 115200, polling, interrupt mode                                   | HW flow control not tested, DMA mode not supported
WATCHDOG     | R5F            | YES               | Only compiled                                                                                   | -

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
LwIP                        | R5F            | NO                | FreeRTOS    | TCP/UDP IP networking stack, DHCP, ping, TCP iperf, TCP/UDP IP                         | Other LwIP features, performance and memory optimizations pending, more robustness tests pending
Ethernet driver (ENET)      | R5F            | YES               | FreeRTOS    | Ethernet as port using CPSW and ICSS                                                   | Ethernet as switch
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
    <td> MCUSDK-1979
    <td> EPWM autogenerated files results in assert errors and repeated function calls
    <td> EPWM
    <td> 08_00_02
    <td> AM263x
    <td> Fixed.
</tr>
<tr>
    <td> MCUSDK-1980
    <td> Change optimization level for ti-arm-clang from O3 to Os
    <td> Build
    <td> 08_00_02
    <td> AM263x
    <td> Updated optimization level to Os for ti arm clang.Os is O2 plus additional optimizations that are designed to reduce code size.
</tr>
<tr>
    <td> MCUSDK-2120
    <td> QSPI Read, Write and Erase takes more time than expected.
    <td> QSPI
    <td> 08_00_02
    <td> AM263x
    <td> Fixed.
</tr>
<tr>
    <td> MCUSDK-2122
    <td> I2C displays two options for selecting the instance in SYSCFG.It should be one.
    <td> I2C
    <td> 08_00_02
    <td> AM263x
    <td> Fixed.
</tr>
<tr>
    <td> MCUSDK-2129
    <td> XBAR Input Configuration incorrect
    <td> Crossbar
    <td> 08_00_02
    <td> AM263x
    <td> Fixed.XBAR input selection in syscfg is incorrectly selecting the input value when the input is from a previous XBAR. The generated macros for current XBAR are wrong.
</tr>
<tr>
    <td> MCUSDK-2144
    <td> Time Sync XBAR's are interchanged in CSL and Syscfg.
    <td> Crossbar
    <td> 08_00_02
    <td> AM263x
    <td> Fixed.
</tr>
<tr>
    <td> MCUSDK-2231
    <td> QSPI EDMA transfer size limited to 32KB
    <td> QSPI,EDMA
    <td> 08_00_02
    <td> AM263x
    <td> Fixed.
</tr>
<tr>
    <td> MCUSDK-2255
    <td> EPWM High Resolution APIs
    <td> EPWM
    <td> 08_00_02
    <td> AM263x
    <td> Fixed. Added EPWM High Resolution APIs.
</tr>
<tr>
    <td> MCUSDK-2256
    <td> Added API for enabling ADC Reference
    <td> ADC
    <td> 08_00_02
    <td> AM263x
    <td> Fixed. Added API to enable ADC Reference and integrated with Sysconfig. ADC Reference will be enabled when ADC is selected from Sysconfig.
</tr>
<tr>
    <td> MCUSDK-2257
    <td> Added macros in EPWM clsr file
    <td> EPWM
    <td> 08_00_02
    <td> AM263x
    <td> Fixed. EPWM cslr files contains register macros for XCMP and Diode Emulation macros.
</tr>
<tr>
    <td> MCUSDK-2258
    <td> ADC Channel 6 and 7
    <td> ADC
    <td> 08_00_02
    <td> AM263x
    <td> Fixed. Added ADC channels 6 and 7 in ADC Channel enum. These channels are used for calibration.
</tr>
<tr>
    <td> MCUSDK-2259
    <td> Outputxbar pinmux issue
    <td> XBAR
    <td> 08_00_02
    <td> AM263x
    <td> Fixed. This issue earlier prevented outputxbar from functioning. After the fix, the outputxbar works as expected.
</tr>
<tr>
    <td> MCUSDK-2260
    <td> Fixed EPWM driver
    <td> EPWM
    <td> 08_00_02
    <td> AM263x
    <td> Fixed. A bug that wrote corrupted values to deadband configuration registers.
</tr>
<tr>
    <td> MCUSDK-2247
    <td> Time Sync XBAR naming mismatch between SDK and TRM.
    <td> Crossbar
    <td> 08_00_02
    <td> AM263x
    <td> Fixed. Existing Syscfg applications making use of XBAR's will have to be modified to reflect the change.
</tr>
<tr>
    <td> MCUSDK-2248
    <td> QSPI Timeout issue during Flash Block erase
    <td> QSPI
    <td> 08_00_02
    <td> AM263x
    <td> Fixed. The timeout value has been increased.
</tr>
<tr>
    <td> MCUSDK-2249
    <td> \ref EXAMPLES_DRIVERS_QSPI_FLASH_TRANSFER and \ref EXAMPLES_DRIVERS_QSPI_FLASH_DMA_TRANSFER example were not using Board flash driver
    <td> QSPI
    <td> 08_00_02
    <td> AM263x
    <td> Fixed. \ref BOARD_FLASH_PAGE should be used for all QSPI Flash applications. Refer to the updated examples, \ref EXAMPLES_DRIVERS_QSPI_FLASH_TRANSFER and \ref EXAMPLES_DRIVERS_QSPI_FLASH_DMA_TRANSFER.
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
    <td> MCUSDK-2246
    <td> XDS110 JTAG Speed
    <td> JTAG. ISO7221ADR U53 limits TDI/TDO to 1Mbps operation. Need to replace U53 with faster speed grade component.
    <td> 08_00_02
    <td> Replace U53 with faster ISO72x or ISO6721x components. Keep XDS110 TCLK speed at 1 MHz or below until U53 is replaced.
</tr>
<tr>
    <td> MCUSDK-2253
    <td> Flash ID read
    <td> Flash. Flash ID read is not supported in Board Flash driver.
    <td> 08_00_02
    <td> Edit Flash driver to send command and read ID in Flash_open.
</tr>
<tr>
    <td> MCUSDK-2252
    <td> GPIO Pin Direction
    <td> GPIO. GPIO Pin Direction not getting automatically configured.
    <td> 08_00_02
    <td> Use GPIO_setDirMode to set pin direction for GPIO pin.
</tr>
<tr>
    <td> MCUSDK-2251
    <td> Prefetch abort with Debug mode .out's
    <td> Interrupt. Debug mode .out's which uses multiple interrupts with different priority causes prefetch abort
    <td> 08_00_02
    <td> -
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

Not Applicable

### SOC Device Drivers

<table>
<tr>
    <th> Module
    <th> Affected API
    <th> Change
    <th> Additional Remarks
</tr>
<tr>
    <td> QSPI
    <td> `QSPI_norFlashInit`

    `QSPI_norFlashReadId`

    `QSPI_norFlashWrite`

    `QSPI_norFlashRead`

    `QSPI_norFlashErase`
    <td> Board flash driver(\ref BOARD_FLASH_PAGE) should be used instead of NOR Flash driver(qspi_nor_flash.c) in flash applications. An example \ref EXAMPLES_DRIVERS_QSPI_FLASH_DIAG has been added to enable basic bring up of flash in 1s1s1s mode. qspi_nor_flash.c has been removed from driver support and kept as a part of the example. The example doesn't support QUAD mode operation of flash. The API changes required to migrate from NOR Flash driver to Board flash driver are listed below -

    `QSPI_norFlashInit` - Not required, since QSPI initialization will be done already in \ref Flash_open.

    `QSPI_norFlashReadId` - Currently, not a part of the Board flash driver.

    `QSPI_norFlashWrite` - Use \ref Flash_write. No argument change required.

    `QSPI_norFlashRead` - Use \ref Flash_read. No argument change required.

    `QSPI_norFlashErase` - Use \ref Flash_eraseBlk. This API requires Block number as argument instead of address. Address to block number conversion can be done using \ref Flash_offsetToBlkPage.
    <td> The qspi_nor_flash_1s driver is a part of the \ref EXAMPLES_DRIVERS_QSPI_FLASH_DIAG example and should be used only for basic flash bringup in 1s1s1s mode.
</tr>
<tr>
    <td> Crossbar
    <td> \ref SOC_xbarSelectTimesyncXbar0InputSource
    \ref SOC_xbarSelectTimesyncXbar0InputSource
    \ref SOC_xbarSelectGpioIntrXbarInputSource
    \ref SOC_xbarSelectEdmaTrigXbarInputSource
    \ref SOC_xbarSelectDMAXBarInputSource
    <td> XBAR Input and Output Macros have been updated. Update the arguments to these API's to reflect the change if XBAR configuration is being done without using Syscfg.
    <td> -
</tr>
</table>


