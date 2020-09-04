# Release Notes 08.03.01 {#RELEASE_NOTES_08_03_01_PAGE}

[TOC]

\attention Also refer to individual module pages for more details on each feature, unsupported features, important usage guidelines.

\note The examples will show usage of SW modules and APIs on a specific CPU instance and OS combination. \n
      Unless explicitly noted otherwise, the SW modules would work in both FreeRTOS and no-RTOS environment. \n
      Unless explicitly noted otherwise, the SW modules would work on any of the R5F's present on the SOC. \n

## New in this Release

Feature                                                                                         | Module
------------------------------------------------------------------------------------------------|--------------------------
AM263x LaunchPad E2 rev support                                                                 | board/EVM support
LIN driver and example support                                                                  | drivers
EPWM: PWM protection and letency example using PRU                                              | drivers
UART DMA mode – driver, Sysconfig support and example: uart_echo_dma                            | drivers
MCSPI DMA mode – driver, Sysconfig support and example: mcspi_loopback_dma                      | drivers
RTI timer driver and example: rti_led_blink                                                     | drivers
SOC driver APIs for Address translation, R5 set frequency, warm reset…                          | drivers
MCAN driver features (Corrupt Message Transmission Prevention, Error Passive state,  Bus Off State, Bus Monitoring Mode)                              | drivers
Boot time profiling support                                                                     | drivers
QSPI SFDP support                                                                               | drivers
hsmclient driver for TIFS-MCU support                                                           | drivers
secure IPC notify driver for TIFS-MCU support                                                   | drivers
Cortex-M4 DPL for TIFS-MCU support                                                              | drivers
EPWM Type5 features - driver, Sysconfig support and example                                     | drivers
EPWM: Sysconfig support for HRPWM                                                               | drivers
FSI DMA driver support and example: fsi_loopback_dma, FSI TX delay line API                     | drivers
ADC Differential Mode example, ADC burst mode oversampling example, ADC Odd Channel Selection   | drivers
Ramp wave and random voltage examples for DAC peripheral                                        | drivers
32 Task Priority Levels for FreeRTOS Tasks                                                      | DPL, FreeRTOS
CpuId and Queue implementation                                                                  | DPL
CPSW Enhaced Scheduled Traffic (EST) support                                                    | networking
Enet - CPDMA multiple channel support, example                                                  | networking
ENET sysconfig support                                                                          | networking
C++ build support and example: hello_world_cpp                                                  | build
Real time serial monitor                                                                        | tools


## Device and Validation Information

\cond SOC_AM263X
SOC   | Supported CPUs  | EVM                                                                                                                                                                   | Host PC
------|-----------------|-----------------------------------------------------------------------------------------------------------------------------------------------------------------------|-------------------------------------------------------------------------------------------
AM263x| R5F             | AM263x ControlCard Revision E1  (referred to as am263x-cc in code). \n                                                                                                | Windows 10 64b or Ubuntu 18.04 64b
AM263x| R5F             | AM263x LaunchPad Revision E2  (referred to as am263x-lp in code)                                                                                                      | Windows 10 64b or Ubuntu 18.04 64b
\endcond

Refer here for information about using this release with E2 revision of ControlCard
- \subpage RELEASE_NOTES_08_03_00_EVM_REV_E2_SUPPORT_PAGE

## Dependent Tools and Compiler Information

Tools                   | Supported CPUs | Version
------------------------|----------------|-----------------------
Code Composer Studio    | R5F            | @VAR_CCS_VERSION_AM263X
SysConfig               | R5F            | @VAR_SYSCFG_VERSION_AM263X, build @VAR_SYSCFG_BUILD_AM263X
TI ARM CLANG            | R5F            | @VAR_TI_ARM_CLANG_VERSION
FreeRTOS Kernel         | R5F            | @VAR_FREERTOS_KERNEL_VERSION
LwIP                    | R5F            | @VAR_LWIP_VERSION

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
    <td>MCUSDK-2254
    <td>[SBL] SBL QSPI bootmode is not working with DMA enabled
    <td> SBL
    <td> 08_02_01
    <td> AM263x
    <td>Fixed.  EDMA Parameters were configured incorrectly.
</tr>
<tr>
    <td> MCUSDK-2703
    <td> Interrupt XBAR instance for FSI is static
    <td> FSI
    <td> 08_00_03
    <td> AM263x
    <td> Fixed.  Enabled Dynamic FSI interrupt config. Updated AM263 FSI SysCfg to dynamically configure Interrupt XBAR's
</tr>
<tr>
    <td> MCUSDK-3336
    <td> FSI is not functional with clock divider value more than 8
    <td> FSI
    <td> 08_02_00
    <td> AM263x
    <td> Fixed
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
    <td> sbl_uart_uniflash incorrect data writing to flash
    <td> SBL
    <td> 08_02_01
    <td> AM263x
    <td> xModem uses a 1k buffer for the reception of data, and it pads the received data with zeros to make it align with 1 KB. We were using the filesize returned by the xmodem receive API to determine how many bytes to flash. This was causing the extra bytes to get flashed.
        Added a parameter actual file size in the uniflash header. This parameter will be used to determine how many bytes to flash
</tr>
<tr>
    <td> MCUSDK-3436
    <td> QSPI flashing fails with uniflash
    <td> QSPI
    <td> 08_02_01
    <td> AM263x
    <td> The internal pull up's in flash are weak. Enabled pull up's on D2 and D3 pins.
</tr>
<tr>
    <td> MCUSDK-3440
    <td> [Errata] USART: Spurious DMA Interrupts
    <td> UART
    <td> 08_02_01
    <td> AM263x
    <td> Workaround: Use power of 2 values for TX/RX FIFO trigger levels (1, 2, 4, 8, 16, and 32).
        Resolution: UART FIFO trigger level is set to 1 always which is a power of 2. Please find the attached email for more details.
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
    <td> MCUSDK-3712
    <td> [FSI]Maximum value of external inputs for triggering frame transmission is limited to 32
    <td> FSI
    <td> 08_02_01
    <td> AM263x
    <td> Updated FSI_TX_MAX_NUM_EXT_TRIGGERS to correct value i.e. 64
</tr>
<tr>
    <td> MCUSDK-3747
    <td> Comments in R5F linker command file templates contradict SDK docs
    <td> Common
    <td> 08_02_01
    <td> AM263x
    <td> Updated the comment and documentation to reflect the correct Interrupt nesting support
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
    <td> MCUSDK-3847
    <td> am263x: cpsw: Ethernet cable disconnect not working
    <td> CPSW
    <td> 08_02_01
    <td> AM263x
    <td> Rootcase: CPSW driver disables PHY state machine ticks once link is up. The driver
relies on MDIO link monitoring to detect when cable is disconnected and
reenables the PHY state machine in order to be able to detect new link
partners.

While MDIO link monitor was enabled at MDIO level, the MDIO event still
needed to be enabled in CPSW wrapper but it was not. This
caused that cable disconnect/connect events were not detected.

Fix: Added MDIO events to CPSW wrapper event mask.

</tr>
<tr>
    <td> MCUSDK-3854
    <td> docs - Elaborate why mailbox init needs to be done by SBL/GEL
    <td> MAILBOX
    <td> 08_02_01
    <td> AM263x
    <td> Updated the documentation/comment
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
    <td> MCUSDK-3879
    <td> DPL : Inconsistent MPU config for shared memory across SOC's
    <td> DPL
    <td> 08_02_01
    <td> AM263x
    <td> Accidentally used Strong Ordered attribute instead of Non Cached attribute for shared memory.
Updated all example.syscfg files.
</tr>
<tr>
    <td> MCUSDK-3888
    <td> The attribute structure generated is not proper when the channel numbers 0 to 15 are not allocated and channels between 16 to 31 are allocated from gui.
    <td> EDMA
    <td> 08_02_01
    <td> AM263x
    <td> Fixed
</tr>
<tr>
    <td> MCUSDK-3896
    <td> [RCM]MSS_RCM.R5SSx_POR_RST_CTRL0 source is missing from the R5FSS internal reset list
    <td> SOC RCM
    <td> 08_02_01
    <td> AM263x
    <td> Added missing reset cause SOC_RcmResetCause_POR_RST_CTRL0
</tr>
<tr>
    <td> MCUSDK-3909
    <td> Enet - examples sysconfig has Enet config missing
    <td> ENET
    <td> 08_02_01
    <td> AM263x
    <td> Networking component missing from product.json
Added networking as component in product.json
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
    <td> MCUSDK-3932
    <td> docs - fix e2e support forum link in SDK user guide
    <td> docs
    <td> 08_02_01
    <td> AM263x
    <td> Updated the link in doc
</tr>
<tr>
    <td> MCUSDK-3939
    <td> networking: layer2_multichannel example only transmits 16 packets
    <td> networking
    <td> 08_02_01
    <td> AM263x
    <td> tx queue for PTP traffic was not handled properly
</tr>
<tr>
    <td> MCUSDK-3947
    <td> [MCAN]MCAN message acceptance filter masking is incorrect
    <td> MCAN
    <td> 08_00_00
    <td> AM263x
    <td> Fixed
</tr>
<tr>
    <td> MCUSDK-3949
    <td> DAC Driver abort on 8.2 SDK
    <td> DAC
    <td> 08_02_01
    <td> AM263x
    <td> The Reset APIs were writing 0x7 to the respective control registers. but not 0x0 back.
</tr>
<tr>
    <td> MCUSDK-4015
    <td> AM263x: FSI functional clock not properly selected
    <td> FSI
    <td> 08_02_00
    <td> AM263x
    <td> Fixed
</tr>
<tr>
    <td> MCUSDK-4026
    <td> SDFM: SDFM_configCompEventHighFilter writing to incorrect registers
    <td> SDFM
    <td> 08_02_01
    <td> AM263x
    <td> Fixed SDFM_SDFIL_OFFSET -> SDFM_DIGFIL_OFFSET in SDFM_configCompEventLowFilter and SDFM_configCompEventHighFilter
</tr>
<tr>
    <td> MCUSDK-4047
    <td> Crossbar: Incorrect DMA XBAR macros for FSI triggers
    <td> XBAR
    <td> 08_02_01
    <td> AM263x
    <td> Updated the DMA XBAR Macros with correct names and values.
</tr>
<tr>
    <td> MCUSDK-4056
    <td> am263x: enet: CPSW EST timestamp checks are incorrect after EST disable
    <td> ENET
    <td> 08_02_01
    <td> AM263x
    <td> The CPSW EST example application performs a check on the EST timestamps generated for transmitted packets by testing whether or not the packet was transmitted in the correct EST window.

For this mechanism to work, it's critical to have an accurate ESTF start time.  This can be ensured by starting ESTF at a future time, which the application enforces by setting the first admin base time to be currentTime + ENET_APP_EST_ADMIN_LIST_DELAY (100 msecs).

After this initial EST schedule is cleared (i.e. explicitly cleared by user via app menu or when link is lost), it's no longer possible for the application to ensure that an admin base time at future time will be configured.  The user can program a new EST schedule and re-enable EST via two different app menu options, but starting ESTF at a future time cannot be effectively enforced anymore as it would depend on timing of user interaction.

So, the timestamp verification mechanism will be disabled after the initial EST schedule is cleared.

Resolution: Disable EST timestamp check if user enters a new EST schedule via app menu.
</tr>
<tr>
    <td> MCUSDK-4070
    <td> SOC driver: ICSS GPI MUX selection API
    <td> SOC
    <td> 08_02_01
    <td> AM263x
    <td> Added SOC API for ICSS GPI MUX selection
</tr>
<tr>
    <td> MCUSDK-4071
    <td> ICSS CSL - remove unsupported features (PWM)
    <td> ICSS
    <td> 08_02_01
    <td> AM263x
    <td> Updated CSLR file with PWM macros removed
</tr>
<tr>
    <td> MCUSDK-4072
    <td> SDFM: Incorrect naming used for input macros to SDFM_enableInterrupt API
    <td> SDFM
    <td> 08_02_01
    <td> AM263x
    <td> Values that can be passed to SDFM_enableInterrupt and SDFM_disableInterrupt
            SDFM_LOW_LEVEL_THRESHOLD_INTERRUPT changed to SDFM_CEVT2_INTERRUPT
            SDFM_HIGH_LEVEL_THRESHOLD_INTERRUPT changed to SDFM_CEVT1_INTERRUPT
</tr>
<tr>
    <td> MCUSDK-4112
    <td> ECAP: ECAP_setSyncInPulseSource has incorrect parameter documentation
    <td> ECAP
    <td> 08_02_01
    <td> AM263x
    <td> Fixed supported values of source (type ECAP_SyncInPulseSource) argument to ECAP_setSyncInPulseSource
</tr>
<tr>
    <td> MCUSDK-4113
    <td> ECAP: Remove V2 from AM263x MCUSDK package
    <td> ECAP
    <td> 08_02_01
    <td> AM263x
    <td> Excluded ECAP v2 from package
</tr>
<tr>
    <td> MCUSDK-4334
    <td> EPWM Trip preprocessor define not included in SDK
    <td> EPWM
    <td> 08_02_01
    <td> AM263x
    <td> Incorrect description in TRM/register spec. Assumed compatible with C2000 and missed a macro in EPWM driver for TripInput 13 selection
Added missing macro in EPWM driver for TripInput 13 selection
</tr>
<tr>
    <td> MCUSDK-4339
    <td> [pre-release testing] UDP test failure in LWIP example
    <td> networking
    <td> 08_02_01
    <td> AM263x
    <td> UDP lite mode was enabled so udp ifperf was not working properly
Disabled UDP lite mode
</tr>
<tr>
    <td> MCUSDK-4190
    <td> A fixed MAC address is hard-coded for CPDMA devices, this will cause issue when two boards are connected in the same network
    <td> ENET
    <td> 08_03_00
    <td> AM263x
    <td> Reading MAC address from EFuse and Eeprom instead of fixed hardcoded MAC Address.
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
    <td> MCUSDK-2294
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
    <td> 08_02_00
    <td>
Issue summary: CPSW and ICSSG test cases fail to transfer packets due to PHY getting stuck.\n
Impacted boards : AM263x_LP E1 and AM263x_LP E2. Both PORT 1 and PORT 2.\n
Reproducibility: 100% reproducible on ICSS unit test. CPSW example tests shown failures only in long run tests (8+ hrs).\n
Root cause: MDIO IO and CLK lines had overshoot/undershoot that is more than the allowed margin, resulted in PHY getting hung.\n
Fix: Replace the MDIO IO and CLK lines termination resistors (R71, R72, R61, R62) of 0 Ohm with 33 Ohm resistor.\n
</tr>
<tr>
    <td> MCUSDK-4050
    <td> Long delay before and after the MCSPI data transfer
    <td> MCSPI
    <td> 08_02_00
    <td> None
</tr>
<tr>
    <td> MCUSDK-4059
    <td> AM263x: FSI first frame transmitted is incorrect in DMA mode
    <td> FSI
    <td> 08_03_00
    <td> Please refer fsi_loopback_dma example.
</tr>
<tr>
    <td> MCUSDK-4234
    <td> FSI RX Generic Trigger Test is not working
    <td> FSI
    <td> 08_03_00
    <td> -
</tr>
<tr>
    <td> MCUSDK-4345
    <td> EDMA: Region interrupt not triggered if Channel Id and TCC are not equal
    <td> EDMA
    <td> 08_02_00
    <td> -
</tr>
<tr>
    <td> MCUSDK-4437
    <td> [AM263x] : Changing the order of Bootloader_loadCpu in SBL can lead to incorrect behaviour
    <td> SBL
    <td> 08_02_00
    <td> -
</tr>
<tr>
    <td> MCUSDK-4506
    <td> DPL: interrupt_prioritization example for FreeRTOS fails
    <td> DPL
    <td> 08_03_00
    <td> None. Interrupt nesting for FreeRTOS is disabled to fix UDP test failure in LWIP example
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
    <td> SDFM
    <td> SDFM_enableInterrupt and SDFM_disableInterrupt
    <td> Values that can be passed to SDFM_enableInterrupt and SDFM_disableInterrupt: SDFM_LOW_LEVEL_THRESHOLD_INTERRUPT changed to SDFM_CEVT2_INTERRUPT, SDFM_HIGH_LEVEL_THRESHOLD_INTERRUPT changed to SDFM_CEVT1_INTERRUPT
    <td> -
</tr>
<tr>
    <td> ADC
    <td> ADC_setupSOC
    <td> Values that can be passed to ADC_setupSOC: addition of ADC_CH_ADCIN1_ADCIN0, ADC_CH_ADCIN3_ADCIN2, ADC_CH_ADCIN5_ADCIN4 macros
    <td> -
</tr>
<tr>
    <td> DAC
    <td> DAC_setReferenceVoltage
    <td> The enum DAC_ReferenceVoltage is now corrected with "DAC_REF_VREF and DAC_REF_VDDA" from "DAC_REF_VDAC and DAC_REF_ADC_VREFHI"
    <td> -
</tr>
<tr>
    <td> ECAP
    <td> ECAP_setSyncInPulseSource
    <td> Supported values of source (type ECAP_SyncInPulseSource) argument to ECAP_setSyncInPulseSource are ECAP_SYNC_IN_PULSE_SRC_SYNCOUT_EPWM0-31
    <td> -
</tr>
<tr>
    <td> Bootloader
    <td> Bootloader_socLoadHsmRtFw
    <td> API addition
    <td> -
</tr>
<tr>
    <td> Bootloader
    <td> SOC_rcmSetR5Clock, SOC_rcmGetR5Clock & SOC_rcmsetR5SysClock
    <td> Added additional parameter `cpuId` to set R5F clock based on R5F Cluster 0 and 1.
    <td> -
</tr>

</table>
