# Buildsheet {#BUILDSHEET}

[TOC]

## Introduction

Build Sheet of supported features and modules for this SDK release. The following table lists the supported features and modules for the corresponding category, along with the support status for RTOS on all the cores.

The support status is indicated by the following codes:

Cod                   | Note
----------------------|------------------------------
Yes                   | The feature or module is supported
No                    | The feature or module is not supported
SDKx.y                | The feature or module will be supported in a future version of the SDK
NA                    | The feature or module is not applicable in the hardware

## Software Buildsheet
| Category                          | Module                                                                       | SubModule                | RTOS on R5F    | RTOS on M4F   |
| --------------------------------- | ---------------------------------------------------------------------------- | ------------------------ | -------------- | ------------- |
| Memory Map                        | MAIN Domain Memory Map                                                       |                          | Yes            | Yes           |
|                                   | MCU Domain Memory Map                                                        |                          | Yes            | Yes           |
|                                   | Processors View Memory Map                                                   |                          | NA             | NA            |
|                                   | Region-based Address Translation                                             |                          | Yes            | Yes           |
| System Interconnect               |                                                                              |                          | NA             | NA            |
| Initialization                    | I2C Bootloader Operation                                                     |                          | NA             | NA            |
|                                   | SPI Bootloader Operation                                                     |                          | NA             | NA            |
|                                   | QSPI Bootloader Operation                                                    |                          | Yes            | NA            |
|                                   | OSPI Bootloader Operation                                                    |                          | Yes            | NA            |
|                                   | PCIe Bootloader Operation                                                    |                          | NA             | NA            |
|                                   | GPMC Bootloader Operation                                                    | NOR                      | No             | NA            |
|                                   |                                                                              | NAND                     | No             | NA            |
|                                   | Ethernet Bootloader Operation                                                |                          | No             | NA            |
|                                   | USB Bootloader Operation                                                     | Host                     | No             | NA            |
|                                   |                                                                              | Device                   | No             | NA            |
|                                   | MMCSD Bootloader Operation                                                   | SD Card (4 bit)          | Yes            | NA            |
|                                   |                                                                              | SD Card (8 bit)          | No             | NA            |
|                                   |                                                                              | eMMC                     | Yes            | NA            |
|                                   | UART Bootloader Operation                                                    |                          | Yes            | NA            |
| Device Configuration              | Power                                                                        |                          | Yes            | Yes           |
|                                   | Reset                                                                        |                          | Yes            | Yes           |
|                                   | Clocking                                                                     |                          | Yes            | Yes           |
| Processors and Accelerators       | Dual-R5F MCU Subsystem                                                       |                          | Yes            | NA            |
|                                   | Dual-A53 MPU Subsystem                                                       |                          | NA             | NA            |
|                                   | Cortex-M4F Subsystem                                                         |                          | NA             | Yes           |
|                                   | Programmable Real-Time Unit and Industrial Communication Subsystem - Gigabit | General PRU Use          | Yes            | No            |
|                                   |                                                                              | EtherCAT Device          | Yes            | No            |
|                                   |                                                                              | Profinet RT Device       | Yes            | No            |
|                                   |                                                                              | Profinet IRT Device      | Yes            | No            |
|                                   |                                                                              | EtherNet/IP adapter      | Yes            | No            |
|                                   |                                                                              | Ethernet Endpoint (EMAC) | Yes            | No            |
|                                   |                                                                              | Ethernet Switch          | No             | No            |
|                                   |                                                                              | Ethernet HSR             | No             | No            |
|                                   |                                                                              | IO Link Primary          | Yes            | No            |
|                                   |                                                                              | HDSL, EnDat 2.2          | Yes            | No            |
| Interprocessor Communication      | Mailbox                                                                      |                          | Yes            | Yes           |
|                                   | Spinlock                                                                     |                          | Yes            | Yes           |
| Memory Controllers                | DDR Subsystem (DDRSS)                                                        | DDR4                     | Yes            | NA            |
|                                   |                                                                              | LPDDR4                   | No             | NA            |
|                                   |                                                                              | Inline ECC               | Yes            | NA            |
|                                   | Region-based Address Translation (RAT) Module                                |                          | Yes            | Yes           |
| Interrupts                        | MCU Domain Interrupt Maps                                                    |                          | Yes            | Yes           |
|                                   | MAIN Domain Interrupt Maps                                                   |                          | Yes            | NA            |
| Time Sync                         | Time Sync Module (CPTS)                                                      |                          | Yes            | No            |
|                                   | Timer Manager                                                                |                          | No             | No            |
|                                   | Time Sync and Compare Events                                                 |                          | No             | No            |
| Data Movement Architecture (DMA)  | Data Movement Subsystem (DMSS)                                               |                          | Yes            | NA            |
|                                   | Peripheral DMA (PDMA)                                                        |                          | Yes            | NA            |
|                                   | RingAcc                                                                      |                          | Yes            | NA            |
|                                   | Secure Proxy                                                                 |                          | Yes            | NA            |
|                                   | Interrup Aggregator                                                          |                          | Yes            | NA            |
|                                   | Packet Streaming Interface Link                                              |                          | Yes            | NA            |
| General Connectivity Peripherals  | Analog-to-Digital Converter (ADC)                                            |                          | Yes            | No            |
|                                   | General-Purpose Interface (GPIO)                                             |                          | Yes            | Yes           |
|                                   | Inter-Integrated Circuit (I2C) Interface                                     | Controller               | Yes            | Yes           |
|                                   |                                                                              | Target                   | Yes            | Yes           |
|                                   | Multichannel Serial Peripheral Interface (MCSPI)                             | Controller               | Yes            | Yes           |
|                                   |                                                                              | Peripheral               | Yes            | Yes           |
|                                   | Universal Asynchronous Receiver/Transmitter (UART)                           | UART                     | Yes            | Yes           |
|                                   |                                                                              | RS-485                   | No             | No            |
|                                   |                                                                              | IrDA                     | No             | No            |
| High-speed Serial Interfaces      | Gigabit Ethernet Switch (CPSW0)                                              | Switch                   | Yes            | NA            |
|                                   |                                                                              | EndPoint                 | Yes            | NA            |
|                                   | Peripheral Component Interconnect Express (PCIe) Subsystem                   | Root Complex             | Yes            | NA            |
|                                   |                                                                              | EndPoint                 | Yes            | NA            |
|                                   | Universal Serial Bus Subsystem (USBSS)                                       | Host 3.0                 | No             | NA            |
|                                   |                                                                              | Device 3.0               | No             | NA            |
|                                   |                                                                              | Host 2.0                 | No             | NA            |
|                                   |                                                                              | Device 2.0               | Yes            | NA            |
|                                   | Serializer/Deserializer (SerDes)                                             |                          | Yes            | NA            |
| Memory Interfaces                 | Flash Subsystem (FSS)                                                        |                          | No             | NA            |
|                                   | Octal Serial Peripheral Interface (OSPI)                                     |                          | Yes            | NA            |
|                                   | General-Purpose Memory Controller (GPMC)                                     | FPGA                     | No             | NA            |
|                                   |                                                                              | NAND                     | No             | NA            |
|                                   |                                                                              | NOR                      | No             | NA            |
|                                   |                                                                              | etc.                     | No             | NA            |
|                                   | Error Location Module (ELM)                                                  |                          | No             | NA            |
|                                   | Multimedia Card Secure Digital (MMCSD) Interface                             | 4-bit                    | Yes            | NA            |
|                                   |                                                                              | 8-bit                    | Yes            | NA            |
| Industrial and Control Interfaces | Enhanced Capture (ECAP) Module                                               | Capture                  | Yes            | NA            |
|                                   |                                                                              | PWM                      | No             | NA            |
|                                   | Enhanced Pulse Width Modulation (EPWM) Module                                |                          | Yes            | NA            |
|                                   | Enhanced Quadrature Encoder Pulse (EQEP) Module                              |                          | Yes            | NA            |
|                                   | Controller Area Network (MCAN)                                               | CAN                      | Yes            | NA            |
|                                   |                                                                              | CAN FD                   | Yes            | NA            |
|                                   | FSI                                                                          | Receiver                 | Yes            | NA            |
|                                   |                                                                              | Transmitter              | Yes            | NA            |
| Timer Modules                     | Global Timebase Counter (GTC)                                                |                          | Yes            | Yes           |
|                                   | Windowed Watchdog Timer (WWDT)                                               |                          | Yes            | No            |
|                                   | Timers                                                                       | Timer                    | Yes            | Yes           |
|                                   |                                                                              | Capture                  | No             | No            |
|                                   |                                                                              | Compare                  | No             | No            |
|                                   |                                                                              | PWM                      | No             | No            |
| Internal Diagnostics Modules      | Dual Clock Comparator (DCC)                                                  |                          | Yes            | Yes           |
|                                   | Error Signaling Module (ESM)                                                 |                          | Yes            | Yes           |
|                                   | RTI(WWDG)                                                                    |                          | Yes            | Yes           |
|                                   | Voltage and Thermal Management(VTM)                                          |                          | Yes            | Yes           |
|                                   | Interconnect Isolation Gasket(STOG)                                          |                          | Yes            | Yes           |
|                                   | Interconnect Isolation Gasket(MTOG)                                          |                          | No             | Yes           |
|                                   | Power OK(POK)                                                                |                          | Yes            | Yes           |
|                                   | PBIST(Built In Self Test)                                                    |                          | Yes            | Yes           |
|                                   | LBIST(Built In Self Test)                                                    |                          | No             | Yes           |
|                                   | Memory Cyclic Redundancy Check (MCRC) Controller                             |                          | Yes            | Yes           |
|                                   | ECC Aggregator                                                               |                          | Yes            | Yes           |
|  On-Chip Debug                    |                                                                              |                          | Yes            | Yes           |