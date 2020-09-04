# MCAN {#DRIVERS_MCAN_PAGE}

[TOC]

The Controller Area Network (CAN) is a serial communication protocol which efficiently supports distributed real-time control.
The MCAN module supports both classic CAN and CAN FD (CAN with Flexible Data-Rate) specifications. CAN FD feature allows
high throughput and increased payload per data frame. The driver provides API's to configure the MCAN module.
Below are the high level features supported by the MCAN driver.

## Features Supported

- MCAN module conforms with CAN Protocol 2.0 A, B and ISO 11898-1:2015
- Configure CAN FD mode to support Full CAN FD (up to 64 data bytes)
- Transmit and Receive classic CAN and and CAN FD message formats
- Configure Up to 32 dedicated Transmit Buffers
- Configure Transmit FIFO, up to 32 elements(Buffer + FIFO combined up to 32)
- Configure Transmit Event FIFO, up to 32 elements
- Configure Up to 64 dedicated Receive Buffers
- Configure Two configurable Receive FIFOs, up to 64 elements each
- Configure Up to 128 Standard Id filter elements
- Configure Up to 64 Extended Id filter elements
- Configure Bit timing parameters
- Configure Global filter to receive non-matching messages
- Configure Maskable interrupts and two interrupt lines
- Api's to read protocol status register(useful in case of communication failures)

## SysConfig Features

@VAR_SYSCFG_USAGE_NOTE

SysConfig can be used to configure below parameters apart from common configuration like Clock, MPU, RAT and others.
- MCAN instances and pin configurations.
- Defualt values for bit time parameter initialization set by MCAN_initSetBitTimeParams

## Features NOT Supported

- DMA mode of operation

## Usage

###Message Ram Initialization
The MCAN module has implemented Message RAM. The main purpose of the Message RAM is to store:

- Receive Messages
- Transmit Messages
- Tx Event Elements
- Message ID Filter Elements

User just needs to configure the number of elements required based on their usecase.

###Acceptance filtering
The MCAN module is capable to configure two sets of acceptance filters - one set for standard and one set for
extended identifiers. These filters can be assigned to an Rx Buffer or to one of the two Rx FIFOs.
The main features of the filter elements are:

- Each filter element can be configured as:

    - Range Filter (from - to)
    - Filter for specific IDs (for one or two dedicated IDs)
    - Classic Bit Mask Filter

- Each filter element can be enabled/disabled individually
- Each filter element can be configured for acceptance or rejection filtering
- Filters are checked sequentially and execution (acceptance filtering procedure) stops at the first matching filter element or when the end of the filter list is reached

Depending on the configuration of the filter element if filter matches, one of the following actions is performed:
- Received frame is stored in FIFO 0 or FIFO 1
- Received frame is stored in Rx Buffer

To receive all the messages without acceptance filtering, you need to configure **anfe** and **anfs** fields of Global filter register.
Please refer section "Acceptance Filtering" in technical reference manual for more details.

###Filter Types
Basically there are 3 types of filter which MCAN module supports.

####Range Filter
Each filter element can be configured to operate as Range Filter (Standard Filter Type SFT = 00/Extended Filter
Type EFT = 00). The filter matches for all received message frames with IDs in the range from SFID1 to SFID2
(SFID2 >= SFID1) and respectively in the range from EFID1 to EFID2 (EFID2 >= EFID1).

There are two options for range filtering of extended frames:
- Extended Filter Type EFT = 00: The Extended ID AND Mask is used for Range Filtering. The Message ID of received frames is ANDed with the Extended ID AND Mask before the  range filter is applied.
- Extended Filter Type EFT = 11: The Extended ID AND Mask (MCAN_XIDAM) is not used for Range Filtering.

####Filter for specific IDs
Each filter element can be configured to filter one or two dedicated Message IDs (Standard Filter Type SFT =01/
Extended Filter Type EFT =01). To filter only one specific Message ID, the filter element has to be configured
with SFID1 = SFID2 respectively EFID1 = EFID2.

####Classic Bit Mask Filter
Classic bit mask filtering can filter groups of Message IDs (Standard Filter Type SFT =10/Extended Filter Type
EFT =10). This is done by masking single bits of a received Message ID. In this case SFID1/EFID1 element is
used as Message ID filter, while SFID2/EFID2 element is used as filter mask.
A 0 bit at the filter mask (SFID2/EFID2) will mask out the corresponding bit position of the configured Message
ID filter (SFID1/EFID1) and the value of the received Message ID at that bit position is not relevant for
acceptance filtering. Only those bits of the received Message ID where the corresponding mask bits are 1 are
relevant for acceptance filtering. There are two interesting cases:

- All mask bits are 1: a match occurs only when the received Message ID and the configured Message ID filter are identical.
- All mask bits are 0: all Message IDs match.

\note For example: if sfid1(message id) is 0xC0 and sfid2(mask) is 0xF then
                   all the messages ranged from 0xC0 to 0xCF will be received.
                   In case you want to receive all messages, sfid2(mask) need to be set to 0x0.

###Filter Element Configuration
Up to 128 filter elements can be configured for 11-bit standard IDs and 64 filter
elements can be configured for 29-bit extended IDs.
Each filter element have 4 following parameters to be configured.

- sfid1/efid1 : Defines the ID of the message to be stored
- sfid2/efid2 : Defines the second ID.This field has a different meaning depending on sfec/efec.
                In case buffer is configured then sfid2/efid2 filters to Rx buffer number[0-63]
- sfec/efec   : Place to store message(Buffer/FIFO0/FIFO1)
- sft/eft     : Filter Types

###Tx Buffer/FIFO Configuration
The MCAN module supports up to 32 Tx Buffers. These Tx Buffers can be configured as
dedicated Tx Buffers, Tx FIFO and as combination of dedicated Tx Buffers/Tx FIFO.
In the message RAM, FIFO is allocated only after the number of buffers allocated.
i.e.

   \code
   txFifoStartAddr = txBufStartAddr + txBufCnt;
   \endcode

Dedicated Tx Buffers are intended for message transmission under complete control of the Host CPU.
There are two options:
- Each dedicated Tx Buffer is configured with a specific Message ID.
- Two or more dedicated Tx Buffers are configured with the same Message ID. In this case the Tx Buffer with
the lowest buffer number is transmitted first.

\note Please refer \ref DRV_MCAN_MODULE for more details and \ref MCAN_EXAMPLE_USAGE for code snippets.

## Important Usage Guidelines

###Bit Rate Calculation

\imageStyle{drivers/mcan_bit_timing.PNG,width:50%}
\image html drivers/mcan_bit_timing.PNG "MCAN Bit Timing"

Formula for bit-rate is:

   \code
   bit rate(bits per second) = (CAN clock in Hz) / BRP / (1 + TSEG1 + TSEG2)
   Sampling Point(%) = (1 + TSEG1) / (1 + TSEG1 + TSEG2)

   CAN clock is functional clock of CAN module (80MHz)
   BRP: Bit rate pre-scalar value
   TSEG1, TSEG2: Time Segments expressed in terms of number of Tq(Time Quantum)

   Tq: the period of the CAN module functional clock.
   \endcode

- Sampling point is the point of time at which the bus level is read and interpreted as the value at that respective time. Typical value of sampling point is between 75-90%.
- Bit timing can be configured by calling the #MCAN_setBitTime function passing the #MCAN_BitTimingParams structure.
- The default initialization for the #MCAN_BitTimingParams strcuture is done by calling #MCAN_initSetBitTimeParams.
- Sysconfig provides an interface to program the default initialization values set in #MCAN_initSetBitTimeParams function. under Global Parameters for MCAN module. This is common for all instances of MCAN.
- The default parameter corresponds to 1Mbps of NOM bit rate and 5Mbps of DATA bit rate.
- If the application needs different bit timing for any instance of MCAN it should appropriately update the #MCAN_BitTimingParams structure passed to #MCAN_setBitTime API.

\note Values selected in Sysconfig / the values set in structure are directly programmed in MCAN register bit fields. The actual interpretation by the hardware of this value is such that one more than the value programmed.

   \code
    Default values used for NOM bit rate (1Mbps) are
    MCAN_BitTimingParams::nomRatePrescalar = 7 (BRP = 8)
    MCAN_BitTimingParams::nomTimeSeg1 = 5 (TSEG1 = 6)
    MCAN_BitTimingParams::nomTimeSeg2 = 2 (TSEG2 = 3)

    bit rate = 80MHz / 8 / (1 + 6 + 3) = 1Mbps

   \endcode

###CAN bus termination

CAN bus termination is needed to minimize the reflection, to reduce the noise.
The transmission lines must be terminated to make sure noise does not cause communication failure.

There are two types of CAN termination:

- Standard termination:
    The traditional 120 Ohm resistor(RL) is used on each end of the bus.

- Split Termination
    Two 60 Ohm(RL/2) resistors along with capacitor(CL) is used to suppress the
    noise at both ends of the bus. Typical value of CL for 1.1 Mbps is 4.7 nF.

Please note RL and CL values are dependent on a signaling rate.

###Transceiver Delay Compensation Value(TDCV)

CAN is a broadcast protocol, every node on the bus sees the message on the bus
even if it is Tx node. Tx node uses this data on Rx line for integrity and error checks locally.
Overview of a bit flow through different stages in communication is:

   \code
   CAN module core -> CAN module Tx line -> SoC Tx Pin/PAD -> Transceiver Out -> CAN Bus -> Transceiver In -> SoC Rx Pin/PAD -> CAN module Rx line -> CAN module core
   \endcode

In the above path flow, introduces latency between data going out from Tx line to data
coming in on Rx line. Major part of this latency comes from Transceiver.
Hence it is called Transceiver Delay Compensation Value.
This needs to programmed as controller has to adjust it's Secondary Sampling Point(SSP)
for sampling Rx line at the correct time.

- MCAN provides functionality to measure delay between transition on Tx line and same transition for Rx line. User need not to measure transceiver loop-back delay or refer transceiver documentation for the same. User just needs to program SSP position which could be same as Sampling Point(SP).

###Things to consider/check before sending a message over CAN bus

Make sure following is done/checked before attempting communication on bus:

- ISO-11898 compatible transceivers have to be connected to a CAN bus.
- CAN bus should have proper termination.
- CAN_L lines from nodes shall be connected together and same goes for CAN_H and GND lines as well.
- Make sure all connections are firm and all pins properly connected.
- All node on the bus are configured for same bit rate.
- In case of CAN FD, make sure Transceiver Delay Compensation value is correctly programmed.
- CAN module is powered up and configured properly.
- SoC pins/PADs for CAN are configured properly.

###Debug CAN communication failure

When any CAN message transmission/reception fails, controller updates
LEC (Last Error Code) and DLEC (Data Phase Last Error Code) fields in status
register('MCAN_PSR')

**LEC**: this field is updated when error happened in arbitration phase in case of CAN FD message or during full message for classic CAN message.

**DLEC**: this field is updated when error happened in data phase in case of CAN FD message.

LEC/DLEC fields have following errors:

- **Stuff Error**

    - Node type: Rx
    - Description: More than 5 equal bits in a sequence have occurred in a part of a received message where this is not allowed.
    - Check if all the nodes on the bus have same bit-rate.

- **Form Error**

    - Node type: Rx
    - Description: A fixed format part of a received frame has the wrong format.
    - This could happen if receiver node have drift in CAN functional clock momentarily or bus have interference.

- **Ack Error**

    - Node Type: Tx
    - Description: The message transmitted by the MCAN module was not acknowledged by another node.
    - CAN bus should have more than one nodes(apart for transmitting node) up and running(not in sleep or power down state).

- **Bit1 Error and Bit0 Error**

    - Node Type: Tx
    - Description:
        - **Bit1 Error**: During the transmission of a message (with the exception of the arbitration field), the device wanted to send a recessive level (bit of logical value 1), but the monitored bus value was dominant.
        - **Bit0 Error**: During the transmission of a message (acknowledge bit, active error flag, or overload flag), the device wanted to send a dominant level (data or identifier bit logical value 0), but the monitored bus value was recessive.
    - Tx and Rx lines are sampled at Sampling Point (SP) and Secondary Sampling Point (SSP) respectively have different values. Position of SSP is configurable.
    - This can happen in following cases:
        - Check if tx and rx pins are configured properly. These pins should be pulled high by default i.e. when bus is idle(no communication is going on).
        - Check if CAN transceiver is enabled.
        - For LEC:
            - If configured SP is too large for given node.
            - Segment after SP(TSEG2) should be large enough to compensate for Transceiver Delay.
        - For DLEC:
            - Configured Transceiver Delay Compensation value is less/more than required.

- **CRC Error**

    - Node type: Rx
    - Description: The CRC check sum of a received message was incorrect. The CRC of an incoming message does not match with the CRC calculated from the received data.
    - This could happen if receiver node have drift in CAN functional clock momentarily or bus have interference.

###BusOff Recovery Sequence

Below is directly adopted from the Bosch app note.

The MCAN enters Busoff state according to CAN protocol conditions. The Busoff state is reported by setting PSR.BO. Additionally, the MCAN sets CCCR.INIT to stop all CAN operation.
To restart CAN operation, the application software needs to clear CCCR.INIT. After CCCR.INIT is cleared, the MCAN’s CAN state machine waits for the completion of the Busoff Recovery Sequence according to CAN protocol (at least 128 occurrences of Bus Idle Condition, which is the detection of 11 consecutive recessive bits).
In the MCAN User’s Manual the description of Bus_Off recovery states that “Once CCCR.INIT has been cleared by the CPU, the device will then wait for 129 occurrences of Bus Idle (129 * 11 consecutive recessive bits) before resuming normal operation. At the end of the Bus_Off recovery sequence, the Error Management Counters will be reset”.

## Example Usage {#MCAN_EXAMPLE_USAGE}

Include the below file to access the APIs
\snippet Mcan_sample.c include

MCAN Interrupt Registration
\snippet Mcan_sample.c App_mcanIntrReg

MCAN Message RAM Configuration
\snippet Mcan_sample.c App_mcanInitMsgRamConfigParams

MCAN Rx Standard Filter Element Configuration
\snippet Mcan_sample.c App_mcanInitStdFilterElemParams

MCAN Tx Message Configuration
\snippet Mcan_sample.c App_mcanConfigTxMsg

MCAN Message Transmit/Receive
\snippet Mcan_sample.c App_mcanTxRxMsg

MCAN Interrupt Service Routine
\snippet Mcan_sample.c App_mcanIntrISR

MCAN Interrupt De-Registration
\snippet Mcan_sample.c App_mcanIntrDeReg

## API

\ref DRV_MCAN_MODULE
