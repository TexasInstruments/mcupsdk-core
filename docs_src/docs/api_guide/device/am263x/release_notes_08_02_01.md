# Release Notes 08.02.01 {#RELEASE_NOTES_08_02_01_PAGE}

[TOC]

\attention Also refer to individual module pages for more details on each feature, unsupported features, important usage guidelines.

\note The examples will show usage of SW modules and APIs on a specific CPU instance and OS combination. \n
      Unless explicitly noted otherwise, the SW modules would work in both FreeRTOS and no-RTOS environment. \n
      Unless explicitly noted otherwise, the SW modules would work on any of the R5F's present on the SOC. \n

## New in this Release

Feature                                                                                         | Module
------------------------------------------------------------------------------------------------|--------------------------
AM263x LaunchPad support                                                                        | board/EVM support
MCSPI DMA mode of operation                                                                     | drivers
ADC burst mode oversampling example                                                             | drivers
Ramp wave and random voltage examples for DAC peripheral                                        | drivers
Driver APIs and sysconfig support for EPWM XCMP feature                                         | drivers
FSI TX delay line API, SOC Address translation APIs                                             | drivers
CPSW Enhaced Scheduled Traffic (EST) support                                                    | drivers
CpuId and Queue implementation                                                                  | DPL

## Device and Validation Information

\cond SOC_AM263X
SOC   | Supported CPUs  | EVM                                                                                                                                                                   | Host PC
------|-----------------|-----------------------------------------------------------------------------------------------------------------------------------------------------------------------|-------------------------------------------------------------------------------------------
AM263x| R5F             | AM263x ControlCard  (referred to as am263x-cc in code). \n **Limited validation done with this release (08.02.01) on AM263x ControlCard. Please use 08.02.00 SDK for ControlCard**  | Windows 10 64b or Ubuntu 18.04 64b
AM263x| R5F             | AM263x LaunchPad  (referred to as am263x-lp in code)                                                                                                                  | Windows 10 64b or Ubuntu 18.04 64b
\endcond


## Dependent Tools and Compiler Information

Tools                   | Supported CPUs | Version
------------------------|----------------|-----------------------
Code Composer Studio    | R5F            | 11.1.0
SysConfig               | R5F            | 1.11.0 build, build 2255
TI ARM CLANG            | R5F            | 1.3.0.LTS
FreeRTOS Kernel         | R5F            | 10.4.3
LwIP                    | R5F            | 2.12.2

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
ADC          | R5F            | YES               | Yes. Example:  adc_soc_continuous_dma | Single software triggered conversion, Multiple ADC trigger using PWM, Result read using DMA, EPWM trip through PPB limit, burst mode oversampling               | -
Bootloader   | R5F            | YES               | Yes. DMA enabled for SBL QSPI         | Boot modes: QSPI, UART. All R5F's                                                                                                                               | -
CMPSS        | R5F            | YES               | NA                                    | Asynchronous PWM trip                                                                                                                                           | -
CPSW         | R5F            | YES               | No                                    | MAC loopback, PHY loopback, LWIP: Getting IP, Ping, Iperf, Layer 2 MAC, Layer 2 PTP Timestamping and CPSW Switch support                                        | -
DAC          | R5F            | YES               | Yes. Example: dac_sine_dma            | Constant voltage, Square wave generation, Sine wave generation with and without DMA, Ramp wave generation, Random Voltage generation                            | -
ECAP         | R5F            | YES               | No                                    | ECAP APWM mode, PWM capture                                                                                                                                     | -
EDMA         | R5F            | YES               | NA                                    | DMA transfer using interrupt and polling mode, QDMA Transfer, Channel Chaining, PaRAM Linking                                                                   | -
EPWM         | R5F            | YES               | Yes. Example: epwm_dma                | PWM outputs A and B in up-down count mode, Trip zone, Update PWM using EDMA, Valley switching, High resolution time period adjustment                           | XCMP feature not tested
EQEP         | R5F            | YES               | NA                                    | Speed and Position measurement.                                                                                                                                 | Frequency Measurement not tested
FSI          | R5F            | YES               | No                                    | RX, TX, polling, interrupt mode, single lane loopback.                                                                                                          | -
GPIO         | R5F            | YES               | NA                                    | Output, Input and Interrupt functionality                                                                                                                       | -
I2C          | R5F            | YES               | No                                    | Controller mode, basic read/write                                                                                                                                   | -
IPC Notify   | R5F            | YES               | NA                                    | Mailbox functionality, IPC between RTOS/NORTOS CPUs                                                                                                             | M4F core
IPC Rpmsg    | R5F            | YES               | NA                                    | RPMessage protocol based IPC                                                                                                                                    | M4F core
MCAN         | R5F            | YES               | No                                    | RX, TX, interrupt and polling mode                                                                                                                              | Bus Off
MCSPI        | R5F            | YES               | Yes. Example: mcspi_loopback_dma      | Controller/Peripheral mode, basic read/write, polling, interrupt and DMA mode                                                            | -
MDIO         | R5F            | YES               | NA                                    | Register read/write, link status and link interrupt enable API                                                                                                  | -
MPU Firewall | R5F            | YES               | NA                                    | Only compiled (Works only on HS-SE  device)                                                                                                                     | -
PINMUX       | R5F            | YES               | NA                                    | Tested with multiple peripheral pinmuxes                                                                                                                        | -
PRUICSS      | R5F            | YES               | NA                                    | Tested with Ethercat FW HAL                                                                                                                                     | -
QSPI         | R5F            | YES               | Yes. Example: qspi_flash_dma_transfer | Read direct, Write indirect, Read/Write commands, DMA for read                                                                                                  | -
SDFM         | R5F            | YES               | No                                    | Filter data read from CPU, Filter data read with PWM sync                                                                                                       | -
SOC          | R5F            | YES               | NA                                    | Lock/unlock MMRs, clock enable, set Hz, Xbar configuration                                                                                                      | -
SPINLOCK     | R5F            | NA                | NA                                    | Lock, unlock HW spinlocks                                                                                                                                       | -
UART         | R5F            | YES               | No                                    | Basic read/write at baud rate 115200, polling, interrupt mode                                                                                                   | HW flow control not tested, DMA mode not supported
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
LwIP                        | R5F            | YES               | FreeRTOS    | TCP/UDP IP networking stack, DHCP, ping, TCP iperf, TCP/UDP IP                         | Other LwIP features, performance and memory optimizations pending, more robustness tests pending
Ethernet driver (ENET)      | R5F            | YES               | FreeRTOS    | Ethernet as port using CPSW and ICSS,Layer 2 MAC, Layer 2 PTP Timestamping, CPSW Switch, CPSW EST | -
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
    <td> MCUSDK-2254
    <td> SBL QSPI bootmode is not working with DMA enabled
    <td> SBL
    <td> 08_02_01
    <td> AM263x
    <td> EDMA Parameters were configured incorrectly. Enabled DMA by default in SBL SYSCFG.
</tr>
<tr>
    <td> MCUSDK-3398
    <td> Address translation for R5FSS1 is missing
    <td> SBL
    <td> 08_02_00
    <td> AM263x
    <td> Fixed
</tr>
<tr>
    <td> MCUSDK-3408
    <td> sbl_uart_uniflash extra bytes written to the flash when flashing images
    <td> SBL
    <td> 08_02_00
    <td> AM263x
    <td> Added a parameter actual file size in the uniflash header. This parameter will be used to determine how many bytes to flash
</tr>
<tr>
    <td> MCUSDK-3618
    <td> AM263x: ePWM_setPhaseShift() API function doesn't correctly configure TBPHS
    <td> EPWM
    <td> 08_02_00
    <td> AM263x
    <td> EPWM_setPhaseShift() uses 16-bit read/write, but TBPHS is a 32-bit register. Hence EPWM_setPhaseShift() doesn't take effect. Problem is resolved in 32-bit read/write is used instead of 16-bit read/write.
</tr>
<tr>
    <td> MCUSDK-3773
    <td> Enet LWIP example fails with SBL on AM263X-LP
    <td> SBL, LWIP
    <td> 08_02_01
    <td> AM263x
    <td> Increased the memory region for updated Descriptor memory
</tr>
<tr>
    <td> MCUSDK-3774
    <td> Multi Core application with SBL QSPI DMA enabled is failing on AM263X-LP
    <td> EDMA, IPC_Notify, SBL
    <td> 08_02_01
    <td> AM263x
    <td> For R5SS0-1 core, 0x8000 is the address where reset vectors needs to be copied. But in SBL QSPI with DMA enabled, actual physical address need to be passed while copying reset vectors via EDMA. During SBL execution R5SS0-0 core runs in Lockstep mode so Virtual to Physical Address calculation need to use LockStep TCM Size i.e. 64KB but only 32KB(R5SS0-0) is used.Updated this API.
</tr>
<tr>
    <td> MCUSDK-3804
    <td> SDFM: API doc/comments: Valid value macros have unexpected CSL_ prefix
    <td> SDFM
    <td> 08_02_00
    <td> AM263x
    <td> Fixed. Removed unwanted CSL_ prefix used in the documentation/comments for SDFM macros
</tr>
<tr>
    <td> MCUSDK-3823
    <td> Wrong boot mode documentation for LP
    <td> Docs
    <td> 08_02_01
    <td> AM263x
    <td> Boot mode switch setting not consistent in LP and CC. Fixed BOOT MODE documentation in Getting Started -> EVM Setup page.
</tr>
<tr>
    <td> MCUSDK-3864
    <td> DAC: Incorrect reference voltage selection macros
    <td> DAC
    <td> 08_02_01
    <td> AM263x
    <td> Fixed. enum DAC_ReferenceVoltage is now corrected with "DAC_REF_VREF and DAC_REF_VDDA" from "DAC_REF_VDAC and DAC_REF_ADC_VREFHI"
</tr>
<tr>
    <td> MCUSDK-3930
    <td> DAC Sysconfig: Incorrect argument passed to DAC_setPWMSyncSignal results in ASSERT
    <td> DAC
    <td> 08_02_01
    <td> AM263x
    <td> Fixed. Sysconfig scripts fixed to pass correct argument (1-32 for EPWM0-EPWM31) as per the DAC_setPWMSyncSignal requirement
</tr>
<tr>
    <td> MCUSDK-3712
    <td> [FSI]Maximum value of external inputs for triggering frame transmission is limited to 32
    <td> SOC
    <td> 08_00_00 onwards
    <td> AM263x
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
    <td> MCUSDK-2464
    <td> ADC sysconfig code generation issue
    <td> ADC
    <td> 08_00_03
    <td> -
</tr>
<tr>
    <td> MCUSDK-2557
    <td> eqep_frequency_measurement example is failing
    <td> SBL
    <td> 08_02_00
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
    <td> MCUSDK-3436
    <td> QSPI flashing fails with uniflash
    <td> Flash
    <td> 08_02_00
    <td> -
</tr>
<tr>
    <td> MCUSDK-3869
    <td> AM263x -  LwIP ICMP tests fails
    <td> Enet, LWIP
    <td> 08_02_00
    <td> -
</tr>
<tr>
    <td> MCUSDK-3886
    <td> MDIO - Phy link issue for MAC PORT 1 on am263x-lp
    <td> Enet, LWIP
    <td> 08_02_01
    <td> -
</tr>
<tr>
    <td> SITARAAPPS-2040
    <td> Dual Core configuration issue with CSP 1.1.3 (Sitara MCU Device Support) on AM263x
    <td> CSP Gel Scripts
    <td> 08_02_01
    <td> Edit gel file as mentioned in  \ref PREREQUISITES while running  multi core applications.
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
    <td> Bootloader
    <td> SOC_rcmSetR5Clock, SOC_rcmGetR5Clock & SOC_rcmsetR5SysClock
    <td> Added additional parameter `cpuId` to set R5F clock based on R5F Cluster 0 and 1.
    <td> -
</tr>
<tr>
    <td> DAC
    <td> DAC_setReferenceVoltage
    <td> The enum DAC_ReferenceVoltage is now corrected with "DAC_REF_VREF and DAC_REF_VDDA" from "DAC_REF_VDAC and DAC_REF_ADC_VREFHI"
    <td> -
</tr>
</table>
