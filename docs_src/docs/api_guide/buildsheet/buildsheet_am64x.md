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

| Category                           | Module                                                                       | SubModule                | RTOS on R5F    |   RTOS on M4  | RTOS on A53    |
| ---------------------------------- | ---------------------------------------------------------------------------- | ------------------------ | -------------- | ------------- | -------------- |
| Memory Map                         | MAIN Domain Memory Map                                                       |                          | Yes            | Yes           | No             |
|                                    | MCU Domain Memory Map                                                        |                          | Yes            | Yes           | No             |
|                                    | Processors View Memory Map                                                   |                          | N/A            | N/A           | No             |
|                                    | Region-based Address Translation                                             |                          | Yes            | Yes           | No             |
| System Interconnect                |                                                                              |                          | N/A            | N/A           | N/A            |
| Initialization                     | I2C Bootloader Operation                                                     |                          | N/A            | N/A           | N/A            |
|                                    | SPI Bootloader Operation                                                     |                          | N/A            | N/A           | N/A            |
|                                    | QSPI Bootloader Operation                                                    |                          | Yes            | N/A           | N/A            |
|                                    | OSPI Bootloader Operation                                                    |                          | Yes            | N/A           | N/A            |
|                                    | PCIe Bootloader Operation                                                    |                          | N/A            | N/A           | N/A            |
|                                    | GPMC Bootloader Operation                                                    | NOR                      | No             | N/A           | N/A            |
|                                    |                                                                              | NAND                     | No             | N/A           | N/A            |
|                                    | Ethernet Bootloader Operation                                                |                          | No             | N/A           | N/A            |
|                                    | USB Bootloader Operation                                                     | Host                     | No             | N/A           | N/A            |
|                                    |                                                                              | Device                   | No             | N/A           | N/A            |
|                                    | MMCSD Bootloader Operation                                                   | SD Card (4 bit)          | Yes            | N/A           | N/A            |
|                                    |                                                                              | SD Card (8 bit)          | No             | N/A           | N/A            |
|                                    |                                                                              | eMMC                     | Yes            | N/A           | N/A            |
|                                    | UART Bootloader Operation                                                    |                          | Yes            | N/A           | N/A            |
| Device Configuration               | Power                                                                        |                          | Yes            | Yes           | No             |
|                                    | Reset                                                                        |                          | Yes            | Yes           | Yes            |
|                                    | Clocking                                                                     |                          | Yes            | Yes           | Yes            |
| Processors and Accelerators        | Dual-R5F MCU Subsystem                                                       |                          | Yes            | N/A           | N/A            |
|                                    | Dual-A53 MPU Subsystem                                                       |                          | N/A            | N/A           | Yes            |
|                                    | Cortex-M4F Subsystem                                                         |                          | N/A            | Yes           | N/A            |
|                                    | Programmable Real-Time Unit and Industrial Communication Subsystem - Gigabit | General PRU Use          | Yes            | No            | No             |
|                                    |                                                                              | EtherCAT Device          | Yes            | No            | No             |
|                                    |                                                                              | Profinet RT Device       | Yes            | No            | No             |
|                                    |                                                                              | Profinet IRT Device      | Yes            | No            | No             |
|                                    |                                                                              | EtherNet/IP adapter      | Yes            | No            | No             |
|                                    |                                                                              | Ethernet Endpoint (EMAC) | Yes            | No            | No             |
|                                    |                                                                              | Ethernet Switch          | No             | No            | No             |
|                                    |                                                                              | Ethernet HSR             | No             | No            | No             |
|                                    |                                                                              | IO Link Primary          | Yes            | No            | No             |
|                                    |                                                                              | HDSL, EnDat 2.2          | Yes            | No            | No             |
| Interprocessor Communication (IPC) | Mailbox                                                                      |                          | Yes            | Yes           | Yes            |
|                                    | Spinlock                                                                     |                          | Yes            | Yes           | Yes            |
| Memory Controllers                 | DDR Subsystem (DDRSS)                                                        | DDR4                     | Yes            | N/A           | N/A            |
|                                    |                                                                              | LPDDR4                   | No             | N/A           | N/A            |
|                                    |                                                                              | Inline ECC               | Yes            | N/A           | N/A            |
|                                    | Region-based Address Translation (RAT) Module                                |                          | Yes            | Yes           | No             |
| Interrupts                         | MCU Domain Interrupt Maps                                                    |                          | Yes            | Yes           | Yes            |
|                                    | MAIN Domain Interrupt Maps                                                   |                          | Yes            | N/A           | Yes            |
| Time Sync                          | Time Sync Module (CPTS)                                                      |                          | Yes            | No            | No             |
|                                    | Timer Manager                                                                |                          | No             | No            | No             |
|                                    | Time Sync and Compare Events                                                 |                          | No             | No            | No             |
| Data Movement Architecture (DMA)   | Data Movement Subsystem (DMSS)                                               |                          | Yes            | N/A           | Yes            |
|                                    | Peripheral DMA (PDMA)                                                        |                          | Yes            | N/A           | No             |
|                                    | RingAcc                                                                      |                          | Yes            | N/A           | No             |
|                                    | Secure Proxy                                                                 |                          | Yes            | N/A           | No             |
|                                    | Interrup Aggregator                                                          |                          | Yes            | N/A           | No             |
|                                    | Packet Streaming Interface Link                                              |                          | Yes            | N/A           | No             |
| General Connectivity Peripherals   | Analog-to-Digital Converter (ADC)                                            |                          | Yes            | No            | Yes            |
|                                    | General-Purpose Interface (GPIO)                                             |                          | Yes            | Yes           | Yes            |
|                                    | Inter-Integrated Circuit (I2C) Interface                                     | Controller               | Yes            | Yes           | Yes            |
|                                    |                                                                              | Target                   | Yes            | Yes           | Yes            |
|                                    | Multichannel Serial Peripheral Interface (MCSPI)                             | Controller               | Yes            | Yes           | Yes            |
|                                    |                                                                              | Peripheral               | Yes            | Yes           | Yes            |
|                                    | Universal Asynchronous Receiver/Transmitter (UART)                           | UART                     | Yes            | Yes           | Yes            |
|                                    |                                                                              | RS-485                   | No             | No            | No             |
|                                    |                                                                              | IrDA                     | No             | No            | No             |
| High-speed Serial Interfaces       | Gigabit Ethernet Switch (CPSW0)                                              | Switch                   | Yes            | N/A           | No             |
|                                    |                                                                              | EndPoint                 | Yes            | N/A           | No             |
|                                    | Peripheral Component Interconnect Express (PCIe) Subsystem                   | Root Complex             | Yes            | N/A           | No             |
|                                    |                                                                              | EndPoint                 | Yes            | N/A           | No             |
|                                    | Universal Serial Bus Subsystem (USBSS)                                       | Host 3.0                 | No             | N/A           | No             |
|                                    |                                                                              | Device 3.0               | No             | N/A           | No             |
|                                    |                                                                              | Host 2.0                 | No             | N/A           | No             |
|                                    |                                                                              | Device 2.0               | Yes            | N/A           | No             |
|                                    | Serializer/Deserializer (SerDes)                                             |                          | Yes            | N/A           | No             |
| Memory Interfaces                  | Flash Subsystem (FSS)                                                        |                          | No             | N/A           | No             |
|                                    | Octal Serial Peripheral Interface (OSPI)                                     |                          | Yes            | N/A           | Yes            |
|                                    | General-Purpose Memory Controller (GPMC)                                     | FPGA                     | No             | N/A           | No             |
|                                    |                                                                              | NAND                     | No             | N/A           | No             |
|                                    |                                                                              | NOR                      | No             | N/A           | No             |
|                                    |                                                                              | etc.                     | No             | N/A           | No             |
|                                    | Error Location Module (ELM)                                                  |                          | No             | N/A           | No             |
|                                    | Multimedia Card Secure Digital (MMCSD) Interface                             | 4-bit                    | Yes            | N/A           | Yes            |
|                                    |                                                                              | 8-bit                    | Yes            | N/A           | Yes            |
| Industrial and Control Interfaces  | Enhanced Capture (ECAP) Module                                               | Capture                  | Yes            | N/A           | Yes            |
|                                    |                                                                              | PWM                      | No             | N/A           | Yes            |
|                                    | Enhanced Pulse Width Modulation (EPWM) Module                                |                          | Yes            | N/A           | Yes            |
|                                    | Enhanced Quadrature Encoder Pulse (EQEP) Module                              |                          | Yes            | N/A           | Yes            |
|                                    | Controller Area Network (MCAN)                                               | Classic CAN              | Yes            | N/A           | Yes            |
|                                    |                                                                              | Classic CAN FD           | Yes            | N/A           | Yes            |
|                                    | FSI                                                                          | Receiver                 | Yes            | N/A           | No             |
|                                    |                                                                              | Transmitter              | Yes            | N/A           | No             |
| Timer Modules                      | Global Timebase Counter (GTC)                                                |                          | Yes            | Yes           | Yes            |
|                                    | Windowed Watchdog Timer (WWDT)                                               |                          | Yes            | No            | Yes            |
|                                    | Timers                                                                       | Timer                    | Yes            | Yes           | Yes            |
|                                    |                                                                              | Capture                  | No             | No            | No             |
|                                    |                                                                              | Compare                  | No             | No            | No             |
|                                    |                                                                              | PWM                      | No             | No            | No             |
| Internal Diagnostics Modules       | Dual Clock Comparator (DCC)                                                  |                          | Yes            | Yes           | No             |
|                                    | Error Signaling Module (ESM)                                                 |                          | Yes            | Yes           | No             |
|                                    | RTI(WWDG)                                                                    |                          | Yes            | Yes           | No             |
|                                    | Voltage and Thermal Management(VTM)                                          |                          | Yes            | Yes           | No             |
|                                    | Interconnect Isolation Gasket(STOG)                                          |                          | Yes            | Yes           | No             |
|                                    | Interconnect Isolation Gasket(MTOG)                                          |                          | No             | Yes           | No             |
|                                    | Power OK(POK)                                                                |                          | Yes            | Yes           | No             |
|                                    | PBIST(Built In Self Test)                                                    |                          | Yes            | Yes           | No             |
|                                    | LBIST(Built In Self Test)                                                    |                          | No             | Yes           | No             |
|                                    | Memory Cyclic Redundancy Check (MCRC) Controller                             |                          | Yes            | Yes           | No             |
|                                    | ECC Aggregator                                                               |                          | Yes            | Yes           | No             |
|  On-Chip Debug                     |                                                                              |                          | Yes            | Yes           | No             |