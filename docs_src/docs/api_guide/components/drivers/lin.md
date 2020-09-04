# LIN {#DRIVERS_LIN_PAGE}

[TOC]

The Local Interconnect Network (LIN) is a serial communication protocol which is a low cost alternative of CAN
(Controller Area Network) where bandwidth and fault tolerence is not required. The communication concept is
single-commander/multiple-responder with a message identification for multi-cast transmission between any network nodes.

## Features Supported

- Compatible to LIN 1.3, 2.0 and 2.1 protocols
- Multi-buffered receive and transmit units
- DMA capability for minimal CPU intervention
- Identification masks for message filtering
- 2 Interrupt lines with priority encoding
- Automatic Bus Idle Detection

## SysConfig Features

@VAR_SYSCFG_USAGE_NOTE

SysConfig can be used to configure below parameters apart from common configuration like Clock, MPU, RAT and others.
- LIN instances and pin configurations.
- Enable Loopback support with Loopback Mode and Type of Loopback.
- Enable Interrupts with different lines and triggers.
- Different LIN Modes, Communication Modes, Debug Mode, Checksum Type, Message Filter, Enable Parity Check.

## Usage

### DMA Support
To reduce CPU load when receiving a LIN N-byte (with N from 1 to 8) response in interrupt mode or DMA mode the SCI/LIN module has receive buffers to able to store a whole LIN response in receive buffers.

A recieve interrupt (RX Interrupt), a receive ready RXRDY flag in SCIFLR (9th bit) as well as a DMA request could occur after receiving a response.

A transmit interrupt (TX Interrupt), a Transmit ready TXRDY flag as well as a DMA request (TXDMA) could occur after transmitting a response. A DMA request can be generated for each transmitted byte or for the entire response depending if the multibuffer is enabled or not.

For Buffered SCI Mode, the TX DMA request is initiated as the SET_TX_DMA (16th bit) in SCISETINT is set.
The RX DMA request is initiated if the SET_RX_DMA (17th bit) is set and the data is received in RDy (LIN Receive buffers).

For LIN Mode, the TX DMA request is initiated if the SET_TX_DMA (16th bit) is already set and ID BYTE in LINID is not written. The RX DMA request is initiated if the if the SET_RX_DMA (17th bit) is set and the data is received in RDy (LIN Receive buffers) only after LINID is recieved on the RX Line.

### Interrupt Management
The LIN module has 2 interrupt lines : level 0 and level 1 to VIM (Vector Interrupt Manager) module.

Each interrupt has a bit to enable/disable the interrupt in the SCISETINT and SCICLRINT registers. This can also be done via SYSCFG.

Each interrupt has to be set via interrupt level 0 or to be set as interrupt level 1. By default interrupts are in line level 0.

Please refer table "SCI/BLIN Interrupts" in technical reference manual for more details.

### Message filtering
Message filtering uses the entire identified to determine which nodes will participate in a response. The participation could be to either recieve or transmit response. All nodes use the RX-ID mask and the TX-ID mask (in LINMASK) to filter the bits of the identifier that should not be compared. The masked bits become don't care for the comparison.

#### Filter Example
To build a mask to accept IDs 0x26 and 0x25, compare 5MSBs and filter 3 LSBs the configuration to be done is :
 - Configure LINID[7:0]=0x20.
 - The TX ID MASK and the RX ID MASK to be configured as 0xFC.

### Low Power Mode
The LIN module may enter low power mode either when there was no activity on LINRX for more than 4s or when a Sleep Command frame was received.

#### Global Low Power Mode
Global low-power mode is asserted by the system and is not controlled by the BLIN module. During global low-power mode, all clocks to the SCI/BLIN are turned off so module is completely inactive.

\note Global Low power mode can be acheived through CLKSTOP_REQ/CLKSTOP_ACK interface.

#### Local Low Power Mode
If the POWERDOWN bit is set and then the module receives a Sleep Command, it will enter local low-power mode.

The Sleep Command consists of a diagnostic commander request fram with identifier 0x3C (60) with first data field as 0x00.

#### Wakeup
The wake-up interrupt is used to allow the LIN module to automatically exit low-power mode. A wakeup is triggered when a falling edge is detected on the receive RX pin, and this clears the POWDERDOWN bit.

\imageStyle{drivers/lin_wakeup_signal.PNG,width:50%}

\code 0.25ms <= T(WUSIG) <= 5ms
\endcode
\note The wakeup signal should be atleast 5 Tbits for LIN bus baudrates. A write of 0xF0 to TD0 will load the transmitter to meet the wakeup signal timing requirement of T(WUSIG).

## Example Usage {#MLIN_EXAMPLE_USAGE}
Include the below file to access the APIs
\snippet Lin_sample.c include

LIN Initialization
Default LIN Initialization can be done by selecting default configuration option via SYSCFG. To override the default initialization, use SYSCFG tool. The generated code for initialization is part of System_init() in ti_drivers_config.c
\snippet Lin_sample.c App_linSysInit

LIN Interrupt Registration
\snippet Lin_sample.c App_linIntrReg

LIN Send Data Command
Data is placed on LIN Tx Registers via this function but the transmit is not initiated till the LIN ID is sent.
\snippet Lin_sample.c App_SendReceive_1

LIN Get Data Command
Data placed on LIN Rx Registers can be further processed by the CPU.
\snippet Lin_sample.c App_SendReceive_2

LIN Configuration in SCI Mode
LIN can be configured in SCI Mode. This can be done directly via using a function example as mentioned below.
\snippet Lin_sample.c App_configureSCIMode

## API

\ref DRV_LIN_MODULE
