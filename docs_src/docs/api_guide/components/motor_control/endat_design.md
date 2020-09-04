# ENDAT Protocol Design {#ENDAT_DESIGN}

[TOC]

## Introduction

This design implements EnDat Receiver (a.k.a subsequent electronics) on TI Sitara™ AM64x/AM243x EVM.
EnDat is a digital bidirectional serial interface for position encoders, also suited fo safety related applications.
Only four signal lines are required, differential pair each for clock and data.
Clock is provided by receiver and data is bidirectional. Data is transmitted in synchronism with clock.
Transfer between receiver and encoder at the physical layer is in accordance with RS485, with transceiver at both ends.

## System Overview

Position feedback system consists of a position encoder attached to a motor, up to 100 meter of cable which provides power and serial communication and the receiver interface for position encoder.
In case of Sitara™ AM64x processor the receiver interface for position encoder is just one function of a connected drive controller.
The AM64x provides in addition to the resources for Industrial Ethernet and motor control application including on-chip ADCs, Delta Sigma demodulator for current measurement.
EnDat Receiver on Sitara™AM64x processor uses one (among the quad core) Programmable Real-time Units (PRU).
Three EnDat channels are available per PRU core.
Clock, data transmit, data receive and receive enable signals from PRU1 of ICSS_G is available in AM64x/AM243x EVM.
For channel 0, signal lines are connected to an onboard RS485 transceiver, output from RS485 along with power lines are available on M12 connector.
Encoder is to be connected to M12 connector. All three EnDat channel signals from PRU are also available via J16 jumper – in this case, external transceiver has to be used.

### Sitara™ AM64x/AM243x processor

Refer TRM for details

#### PRU-ICSS

Refer PRU-ICSS chapter of AM64x/AM243x Technical Reference Manual

## System Implementation

The EnDat receiver function is implemented on TI Sitara™ AM64x/AM243x EVM.
Encoder is connected to IDK via universal Digital Interface TIDA-00179(https://www.ti.com/tool/TIDA-00179), TIDEP-01015(3-axis board) and 3 Axis Interface card.
Design is split into three parts – EnDat hardware support in PRU, firmware running in PRU and driver running in ARM.
Application is supposed to use the EnDat driver API’s to leverage EnDat functionality.
EnDat hardware capability in PRU1 of ICSS_G is used, and so the firmware.
Remaining PRUs in the AM64x/AM243x EVM are available for Industrial Ethernet communication and/or motor control interfaces.


### System Specifications

<table>
<tr>
    <th>Parameter
    <th>Value
	<th>Details
</tr>
<tr>
    <td>Maximum Cable Length
    <td>100m
	<td>Supports up-to 8MHz with delay compensation
</tr>
<tr>
    <td>Maximum Frequency
    <td>16 MHz
	<td>Supports up-to 20m cable
</tr>
<tr>
    <td>Startup/Initialization Frequency
    <td>200 KHz
	<td>After power on or reset
</tr>
<tr>
    <td>Frequencies supported
    <td>Upto 16 MHz
	<td>Changeable at run-time
</tr>
<tr>
    <td>CRC
    <td>6 bits
	<td>Position/data verification
</tr>
<tr>
    <td>Receive oversample ratio
    <td>8
	<td>
</tr>
</table>

### EnDat PRU hardware

Refer TRM for details

### EnDat Firmware Implementation

Following section describes the firmware implementation of EnDat receiver on PRU-ICSS.
Deterministic behavior of the 32 bit RISC core running at 200MHz provides 5 ns resolution on sampling external signals and generating external signals.
It makes uses of EnDat hardware support in PRU for data transmission.

\image html endat_module_integration.png "ARM, PRU, EnDat module Integration"


####	Firmware Architecture

\image html endat_overall_block_diagram.png "Overall block diagram"

Firmware first does initialization of PRU EnDat hardware and EnDat encoder.
Then it waits for the user to provide command (user after setting up the command, sets command trigger bit), upon detecting trigger, first it checks whether the command requested is a continuous mode or a normal command.

If it is a normal command, reads command, it’s attribute like transmit bits, receive bits etc., then it transmits the data and collected the data sent by the encoder stored onto a buffer with one byte representing a bit (since oversample ration of 8 is used).
Next it checks whether there is 2.2 command supplement to be transmitted based on attributes, if so it transmits it.
The received data is now downsampled to extract bit from oversampled 8 bits and the result written to the defined PRU RAM locations.

If command requested is continuous mode, 2.1 position command will be transmitted. During receive, it is different from the normal mode that downsampling is done on-the-fly, i.e. downsampling is done as soon as each bit is received. This is done due to the timing constraints with continuous mode, as data is continuously being received.

At the end of transaction as requested by the user, trigger bit that is set by the user is unset.
User can wait on this bit to know that the command has been completed.
EnDat driver provides API to achieve this.


#####	 Initialization

\image html endat_initialization.png "Initilization"

PRU is set to EnDat mode first, clock is configured to 200KHz, with oversample ration of 8, hence receive clock would be 200 * 8 KHz.
The entire EnDat configuration MMR’s are cleared. Through the defined interface (PRU RAM location), user requested channel is determined.
ECAP timer is initialized (used during propagation delay estimation).
Then power-on-init as per specification is implemented, after which encoder is reset by sending reset command.
Firmware setups the command and it’s attribute for all the commands that are sent during initialization. Alarms, errors and warning are cleared.
Firmware then determines number of clock pulses for position and whether encoder supports EnDat 2.2. Propagation delay is then estimated.
If user has required for clock to be configured, it is obeyed, else it defaults to 8MHz. At the end of the initialization status is updated.

#####	Send and Receive

\image html endat_send_receive.png "Send And Receive"


If requested command attribute indicates 2.2 command supplement, clock is configured to free run stop low mode, else to free run stop high.
Command is written to the transmit fifo and send routine followed by receive is invoked.

######	Send

\image html endat_send.png "Send"

Transmit and receive frame sizes are configured in PRU EnDat hardware.
With long cables, it may be required to configure receive frame size lesser than receive bits so that extra clocks are not sent to the encoder.
If transmit was going on, it will till it has finished and then transmit GO bit is set, which would start the new transmission.

######	Receive

\image html endat_receive.png "Receive"


Receive bits obtained via command attribute is stored as header (initial 2 bytes) in the receive buffer.
Then wait’s till receive valid flag has been set, once set, 1 byte corresponding 1 bit (because of oversampling of 8) is read and stored in receive buffer and the flags are cleared.
Receive buffer pointer is incremented & receive bit count decremented. This continues till count is zero, once zero, it extracts receive data one more time to take care of SB (receive count excludes SB).
If 2.2 command supplement is not present, transmit re-init is done.

###### EnDat 2.2 Command Supplement Send

\image html endat_2_2_supplement_send.png "EnDAT 2.2 command supplement send"

Clock mode is configured to stop low after transmit. 2.2 command supplement to transmitted is written to fifo preceded by 7 dummy bits & SB.
Transmission is configure to transmit till end of the fifo. Transmission is started after making sure that transmit module is not busy.

###### Receive Downsample

\image html endat_receive_downsampling.png "Downsampling"

This is the most complex portion of the firmware. Received data is exposed through PRU interface in four bytes.
First two words (word = 4 bytes) holds the position data, third holds additional information 2 (if only second additional info is present or both present) or 1 (if only first additional info is present) & the last additional information 1 (if both present).
The order is as mentioned in EnDat 2.2 specification. Splitting the received data on word boundaries when additional info’s are present causes the complexity here.

If command is neither 2.2 nor position request or if no additional info is present, handling is easy – just copy the received data into initial 2 words in the order it is received.
If command is 2.2 position command and depending on the number of additional info’s, markers (used in the downsampling loop) are set to write additional info’s to next word boundaries.
If only one addinfo is present, marker “rx pos bits” stores clocks required to receive position (inclusive of CRC, F1 & F2).
If both addinfo’s are present another marker is set to 2 words (first 2 words holds the position) plus 30 bit to account for addinfo.
If markers are not required, then their values are set so that it never matches the counting receive bits, hence the value “0xff”.

After updating the marker, number of received bits is retrieved from the receive buffer header. SB is skipped for downsampling and the result registers are cleared.
Next, each byte (8 bit, oversample of 8) is read from the receive buffer, 4th bit of each decide the actual received bit. This is continued till the end of receive buffer.
If during the loop, receive bit count matches any of the marker, bit count is updated appropriately. This helps is naturally bringing the received data as per the word format specified by the interface.
In the loop, as the number of bits reaches word boundary, it will start saving received data to next word. At the end of the loop, the last word is copied to the result register.

###### Continuous mode

\image html endat_continuous_mode.png "Continuous Mode"

2.1 position command as well as it’s attribute that been setup by the user is read first. Clock is configured for free run mode. Position command is written fifo and send routine is invoked.
Then receive is done along with on-the-fly downsampling, this is required as time between receipt of successive position data is less than the time that dowsampling routine (mentioned earlier) takes.
Once data is read and dowsampled on-the-fly, command trigger interface is read to see if user wants to stop continuous mode, if so, do transmit re-init, disable receive and wait till the end of re-init.

###### Receive and On-The-Fly Downsample

\image html endat_on_fly_downsampling.png "Endat on the fly Downsampling"

Two registers (a word each) that hold the result are cleared initially. Upon receiving the first receive valid, it discards it and proceeds to wait for the next one as the first one is SB.
Thereafter for every valid flag set, 4th bit in the received byte is checked to find the actual received bit and it stored, word crossing is also taken care.
After all the bits for a position command is received, receive is disabled and is activated only after 2T clock cycles – this is to prevent falsely detecting SB immediately (upon calling this routine back-to-back as mentioned in previous section) after encoder has finished sending data as it can pull data line high for 2T more clock cycles.

### EnDat Hardware interface

The physical data transmission in EnDat is done using RS-485 standard. The data is transmitted as differential signals using the RS485 between the EnDat Receiver and the Encoder.

The Receiver sends the clock to the EnDat encoder, data transmission in either direction (one at a time) occurs in synchronism with the clock. The design uses two differential signals for each of the lines (clock and data).

EnDat Receiver and the encoder is connected using the RS-485 transceiver. Data is transmitted differentially over RS-485. It has the advantages of high noise immunity and long distance transmission capabilities.

#### AM64x/AM243x EVM Pin-Multiplexing

<table>
<tr>
    <th>Pin name
    <th>Signal name
	<th>Function
</tr>
<tr>
    <td>PRG0_PRU1_GPO0
    <td>pru1_endat0_clk
	<td>Channel 0 clock
</tr>
<tr>
    <td>PRG0_PRU1_GPO1
    <td>pru1_endat0_out
	<td>Channel 0 transmit
</tr>
<tr>
    <td>PRG0_PRU1_GPO2
    <td>pru1_endat0_outen
	<td>Channel 0 transmit enable
</tr>
<tr>
    <td>PRG0_PRU1_GPI13
    <td>pru1_endat0_in
	<td>Channel 0 receive
</tr>
<tr>
    <td>PRG0_PRU1_GPO3
    <td>pru1_endat1_clk
	<td>Channel 1 clock
</tr>
<tr>
    <td>PRG0_PRU1_GPO4
    <td>pru1_endat1_out
	<td>Channel 1 transmit
</tr>
<tr>
    <td>PRG0_PRU1_GPO5
    <td>pru1_endat1_outen
	<td>Channel 1 transmit enable
</tr>
<tr>
    <td>PRG0_PRU1_GPI14
    <td>pru1_endat1_in
	<td>Channel 1 receive
</tr>
<tr>
    <td>PRG0_PRU1_GPO6
    <td>pru1_endat2_clk
	<td>Channel 2 clock
</tr>
<tr>
    <td>PRG0_PRU1_GPO12
    <td>pru1_endat2_out
	<td>Channel 2 transmit
</tr>
<tr>
    <td>PRG0_PRU1_GPO8
    <td>pru1_endat2_outen
	<td>Channel 2 transmit enable
</tr>
<tr>
    <td>PRG0_PRU1_GPI11
    <td>pru1_endat2_in
	<td>Channel 2 receive
</tr>
<tr>
    <td>GPIO42
    <td>endat_en
	<td>Onboard RS485 receive enable
</tr>
</table>