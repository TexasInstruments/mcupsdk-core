# Release Notes 08.02.00 {#RELEASE_NOTES_08_02_00_PAGE}

[TOC]

\attention Also refer to individual module pages for more details on each feature, unsupported features, important usage guidelines.

\note The examples will show usage of SW modules and APIs on a specific CPU instance and OS combination. \n
      Unless explicitly noted otherwise, the SW modules would work in both FreeRTOS and no-RTOS environment. \n
      Unless explicitly noted otherwise, the SW modules would work on any of the R5F's present on the SOC. \n

## New in this Release

Feature                                                                                         | Module
------------------------------------------------------------------------------------------------|--------------------------
SBL JTAG Uniflash Example supported                                                             | SBL

## Device and Validation Information

\cond SOC_AM263X
SOC   | Supported CPUs  | EVM                                                    | Host PC
------|-----------------|--------------------------------------------------------|-------------------------------------------------------------------------------------------
AM263x| R5F             | AM263x ControlCard  (referred to as am263x-cc in code)  | Windows 10 64b or Ubuntu 18.04 64b
\endcond


## Dependent Tools and Compiler Information

Tools                   | Supported CPUs | Version
------------------------|----------------|-----------------------
Code Composer Studio    | R5F            | @VAR_CCS_VERSION_AM263X
SysConfig               | R5F            | @VAR_SYSCFG_VERSION_AM263X, build @VAR_SYSCFG_BUILD_AM263X
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

Peripheral   | Supported CPUs | SysConfig Support | Key features tested                                                                                                     | Key features not tested / NOT supported
-------------|----------------|-------------------|-------------------------------------------------------------------------------------------------------------------------|----------------------------------------
ADC          | R5F            | YES               | Single software triggered conversion, Multiple ADC trigger using PWM, Result read using DMA, EPWM trip through PPB limit | -
Bootloader   | R5F            | YES               | Boot modes: QSPI, UART. All R5F's                                                                                       | -
CMPSS        | R5F            | YES               | Asynchronous PWM trip                                                                                                   | -
CPSW         | R5F            | YES               | MAC loopback, PHY loopback, LWIP: Getting IP, Ping, Iperf, Layer 2 MAC, Layer 2 PTP Timestamping and CPSW Switch support                                                               | -
DAC          | R5F            | YES               | Constant voltage, Square wave generation, Sine wave generation with and without DMA                                                         | -
ECAP         | R5F            | YES               | ECAP APWM mode, PWM capture                                                                                             | -
EDMA         | R5F            | YES               | DMA transfer using interrupt and polling mode, QDMA Transfer, Channel Chaining, PaRAM Linking                           | -
EPWM         | R5F            | YES               | PWM outputs A and B in up-down count mode, Trip zone, Update PWM using EDMA, Valley switching functionality, High resolution time period adjustment capability                                                        | -
EQEP         | R5F            | YES               | Frequency Measurement, Speed and Position measurement.                                                                                                         | -
FSI          | R5F            | YES               | RX, TX, polling, interrupt mode, single lane loopback.                                                                                                           | -
GPIO         | R5F            | YES               | Output, Input and Interrupt functionality                                                                               | -
I2C          | R5F            | YES               | Controller mode, basic read/write                                                                                           | -
IPC Notify   | R5F            | YES               | Mailbox functionality, IPC between RTOS/NORTOS CPUs                                                                     | M4F core
IPC Rpmsg    | R5F            | YES               | RPMessage protocol based IPC                                                                                            | M4F core
MCAN         | R5F            | YES               | RX, TX, interrupt and polling mode                                                                                      | -
MCSPI        | R5F            | YES               | Controller/Peripheral mode, basic read/write, polling, interrupt mode                                                            | -
MDIO         | R5F            | YES               | Register read/write, link status and link interrupt enable API                                                          | -
MPU Firewall | R5F            | YES               | Only compiled (Works only on HS-SE  device)                                                                                                            | -
PINMUX       | R5F            | YES               | Tested with multiple peripheral pinmuxes                                                                                | -
PRUICSS      | R5F            | YES               | Tested with Ethercat FW HAL                                                                                             | -
QSPI         | R5F            | YES               | Read direct, Write indirect, Read/Write commands, DMA for read                                                          | -
SDFM         | R5F            | YES               | Filter data read from CPU, Filter data read with PWM sync                                                                                               | -
SOC          | R5F            | YES               | Lock/unlock MMRs, clock enable, set Hz, Xbar configuration                                                              | -
SPINLOCK     | R5F            | NA                | Lock, unlock HW spinlocks                                                                                               | -
UART         | R5F            | YES               | Basic read/write at baud rate 115200, polling, interrupt mode                                                           | HW flow control not tested, DMA mode not supported
WATCHDOG     | R5F            | YES               | Reset mode                                                                                                           | Interrupt mode

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
LwIP                        | R5F            | YES               | FreeRTOS    | TCP/UDP IP networking stack, DHCP, ping, TCP iperf, TCP/UDP IP                         | Other LwIP features, performance and memory optimizations pending, more robustness tests pending
Ethernet driver (ENET)      | R5F            | YES               | FreeRTOS    | Ethernet as port using CPSW and ICSS,Layer 2 MAC, Layer 2 PTP Timestamping, CPSW Switch| -
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
    <td> MCUSDK-2285
    <td> Enet lld - Debug gels scripts are not updated for AM263x
    <td> Enet
    <td> 08_02_00
    <td> AM263x
    <td> Fixed. debug gels for am263x created.
</tr>
<tr>
    <td> MCUSDK-2343
    <td> Documentation update for out2rprc binary dependencies
    <td> Build, Common, SBL
    <td> 08_02_00
    <td> AM263x
    <td> Fixed. Documentation Gap. out2rprc.exe is replaced by elf2rprc java scripts there by removing dependency of .NET framework tool.
</tr>
<tr>
    <td> MCUSDK-2396
    <td> Low SVC stack size with Interrupt nesting enabled
    <td> DPL
    <td> 08_02_00
    <td> AM263x
    <td> Fixed. Stack size was not modified for few SOC's. Increased SVC stack size
</tr>
<tr>
    <td> MCUSDK-2457
    <td> Tools installed twice due to makefile based and ccs based builds
    <td> Build
    <td> 08_02_00
    <td> AM263x
    <td> Fixed. Updated the make file to select the ti arm clang either from CCS or from c:/ti based on availability
</tr>
<tr>
    <td> MCUSDK-2541
    <td> IPC developer guide block diagram shows invalid cores
    <td> IPC_Notify, IPC_RPMSG
    <td> 08_02_00
    <td> AM263x
    <td> Fixed. Documentation corrected
</tr>
<tr>
    <td> MCUSDK-2544
    <td> AM263x: MCAN driver doesn't support MCAN2,3
    <td> MCAN
    <td> 08_02_00
    <td> AM263x
    <td> Fixed. Updated driver to support MCAN2 and MCAN3 instances.
</tr>
<tr>
    <td> MCUSDK-2555
    <td> EPWM driver bug in 8.1 SDK
    <td> EPWM
    <td> 08_02_00
    <td> AM263x
    <td> Fixed. Reverted the code to prepare and write the whole 16 bits value. Fixed the macro used to write to RED/FED registers.
</tr>
<tr>
    <td> MCUSDK-2559
    <td> AM263x_LP:FSI interrupt mode example
    <td> FSI
    <td> 08_02_00
    <td> AM263x
    <td> Fixed. The INTXBAR0_G3_SEL values for FSI TX and RX in CSL files were incorrect. Replaced with correct values. Also updated example syscfg to add interrupt xbar configuration.
</tr>
<tr>
    <td> MCUSDK-2626
    <td> QSPI Flash DMA transfer example fails the subsequent tests.
    <td> QSPI
    <td> 08_02_00
    <td> AM263x
    <td> Fixed. QSPI test was overwriting the SBL which caused this issue. QSPI test flash address changed.
</tr>
<tr>
    <td> MCUSDK-2704
    <td> Clock Jitter is observed after reconfiguring core PLL in SBL
    <td> CPSW, SBL
    <td> 08_02_00
    <td> AM263x
    <td> Fixed. Need to follow below sequence before reconfiguring CORE PLL.
        1) Before we program the PLLs the R5F source clock is changed to
        25MHz clock and also change the peripheral clock sources that
        are used.
        2) Sigma delta divider for optimum jitter bit is programmed as per the requirement, update that as well for CPSW 2G need.
</tr>
<tr>
    <td> MCUSDK-2874
    <td> Enet - LWIP example fails to load in CCS with SBL null mode
    <td> LWIP, SBL
    <td> 08_02_00
    <td> AM263x
    <td> Fixed. Bank2 and Bank3 of L2 memory needs to be initialized
before loading any app image which is placed in this memory.
ROM does only does for Bank0 and Bank1 of L2memory. Added this in SBL before loading any appimage.
</tr>
<tr>
    <td> MCUSDK-3302
    <td> UART: UART_Params_init() configures intrNum incorrectly
    <td> UART
    <td> 08_02_00
    <td> AM263x
    <td> Fixed. UART_Params_init() configures
    intrNum with 210. This is valid only for AM243/64.
    Fixed the issue by initializing intrNum with invalid value.
    This needs to be modified with correct interrupt line for UART,
    before calling UART_open()
</tr>
<tr>
    <td> MCUSDK-3335
    <td> AM263x: SBL signing scripts fail
    <td> Build
    <td> 08_02_00
    <td> AM263x
    <td> Fixed. Added missing template file am263x_x509template.txt needed by signing script.
</tr>
<tr>
    <td> MCUSDK-3393
    <td> IPC testcase fails in SBL testing
    <td> IPC_Notify
    <td> 08_02_00
    <td> AM263x
    <td> Fixed. SBL fixed by updating Mailbox init API's
</tr>
<tr>
    <td> MCUSDK-3413
    <td> Section Writing flash driver for a custom flash device is missing in AM263x documentation
    <td> docs
    <td> 08_02_00
    <td> AM263x
    <td> Fixed. Added documentation on how to write flash driver for a custom flash device.
</tr>
<tr>
    <td> MCUSDK-3489
    <td> MPU Firewall: MPU_FIREWALL_getFirewallConfig() incorrect argument
    <td> MPU Firewall
    <td> 08_02_00
    <td> AM263x
    <td> Fixed. Source pointer passed as argument was not updated. Used double pointer as argument to MPU_FIREWALL_getFirewallConfig(). This makes sure that the pointer at source is updated.
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
    <td> 08_00_03
    <td> Disable interrupt nesting
</tr>
<tr>
    <td> MCUSDK-2252
    <td> GPIO Pin Direction
    <td> GPIO. GPIO Pin Direction not getting automatically configured.
    <td> 08_00_02
    <td> Use GPIO_setDirMode to set pin direction for GPIO pin.
</tr>
<tr>
    <td> MCUSDK-2254
    <td> SBL QSPI bootmode is not working with DMA enabled
    <td> SBL
    <td> 08_00_03
    <td> -
</tr>
<tr>
    <td> MCUSDK-2257
    <td> eqep_frequency_measurement example is failing
    <td> SBL
    <td> 08_02_00
    <td> -
</tr>
<tr>
    <td> MCUSDK-2464
    <td> ADC sysconfig code generation issue
    <td> ADC
    <td> 08_00_03
    <td> -
</tr>
<tr>
    <td> MCUSDK-2703
    <td> Interrupt XBAR instance for FSI is static
    <td> FSI
    <td> 08_00_03
    <td> -
</tr>
<tr>
    <td> MCUSDK-3336
    <td> FSI is not functional with clock divider value more than 8
    <td> FSI
    <td> 08_02_00
    <td> -
</tr>
<tr>
    <td> MCUSDK-3398
    <td> Address translation for R5FSS1 is missing
    <td> SBL
    <td> 08_02_00
    <td> -
</tr>
<tr>
    <td> MCUSDK-3436
    <td> QSPI flashing fails with uniflash
    <td> Flash
    <td> 08_02_00
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
</table>


