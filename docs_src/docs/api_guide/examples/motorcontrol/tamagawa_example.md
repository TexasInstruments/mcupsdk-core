#  TAMAGAWA Diagnostic {#EXAMPLE_MOTORCONTROL_TAMAGAWA}
[TOC]

## Introduction

Tamagawa diagnostic application does below,

Configures pinmux, GPIO, UART, ICSS clock to 200MHz,
Initializes ICSS0-PRU1,
Loads the initialization section of PRU firmware & executes it.
This application is controlled with a terminal interface using a serial over USB connection between the PC host and the EVM.
Please connect a USB cable between the PC and the EVM.
A serial terminal application (like teraterm/ hyperterminal/ minicom) is then run on the host.
To configure, select the serial port corresponding to the port emulated over USB by the EVM.
The host serial port should be configured to 115200 baud, no parity, 1 stop bit and no flow control.
Connect the Tamagawa encoder via RS485 transceiver to the EVM.
The connections between EVM and RS485 logic signals are:

PRU UART Rx -> PRG0_PRU1_GPO9
PRU UART Tx -> PRG0_PRU1_GPO10
Tx Enable -> PRG0_PRU1_GPO0
It is necessary to provide 3.3V to the RS485 logic side. The receive transceiver is always kept enabled with bus side connected as in Tamagawa Specification.

The Tamagawa receiver firmware running on ICSS0-PRU1 provides a defined interface. The Tamagawa diagnostic application interacts with the Tamagawa receiver firmware interface. It then presents the user with menu options to select Data ID code (as defined by Tamagawa) to be sent to the encoder. The application collects the data entered by the user and configures the relevant interface. Then via the Tamagawa receiver interface, the command is triggered. Once the command completion is indicated by the interface, the status of the transaction is checked. If the Status indicates success, the result is presented to the user.

## Important files and directory structure

<table>
<tr>
    <th>Folder/Files
    <th>Description
</tr>
<tr><td colspan="2" bgcolor=#F0F0F0> ${SDK_INSTALL_PATH}/examples/motor_control/tamagawa_diagnostic</td></tr>
<tr>
    <td>tamagawa_diagnostic.c</td>
    <td>Tamagawa diagnostic application</td>
</tr>
<tr><td colspan="2" bgcolor=#F0F0F0> ${SDK_INSTALL_PATH}/source/motor_control/position_sense/tamagawa</td></tr>
<tr>
    <td>firmware/</td>
    <td>Folder containing TAMAGAWA PRU firmware sources.</td>
</tr>
</table>

# Supported Combinations {#EXAMPLES_MOTORCONTROL_TAMAGAWA_COMBOS}

\cond SOC_AM64X || SOC_AM243X

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0 freertos
 ICSSG          | ICSSG0
 PRU            | PRU1
 Toolchain      | ti-arm-clang
 Board          | @VAR_BOARD_NAME_LOWER
 Example folder | examples/motorcontrol/tamagawa_example

\endcond

# Steps to Run the Example

## Hardware Prerequisites

-  AM64x/AM243x EVM
-  Profibus AM64x Card
-  RS485 Half-duplex EVM
-  Tamagawa Encoder

## Hardware Setup

\imageStyle{Tamagawa_setup.jpg,width:60%}
\image html Tamagawa_setup.jpg "Hardware Setup"

\imageStyle{Tamagawa_Connections.png,width:60%}
\image html Tamagawa_Connections.png


## Build, load and run

- **When using CCS projects to build**, import the CCS project and build it using the CCS project menu (see \ref CCS_PROJECTS_PAGE).
- **When using makefiles to build**, note the required combination and build using
  make command (see \ref MAKEFILE_BUILD_PAGE)
- Launch a CCS debug session and run the executable, see \ref CCS_LAUNCH_PAGE
- Refer to UART terminal for user interface menu options.

### Sample Output

Shown below is a sample output when the application is run:

\imageStyle{Tamagawa_SampleOutput.png,width:60%}
\image html Tamagawa_SampleOutput.png "Tamagawa Sample Output"

### Test Case Description

<table>
    <tr>
        <th>Data_ID
        <th>Name
        <th>Description
    </tr>
    <tr>
        <td>Data ID 0</td>
        <td>Data readout (absolute position data)</td>
        <td>In this command we will receive:
		Absolute rotor position value in field name ABS..
		Errors and warnings in field name SF(status field)
		</td>
    </tr>
	<tr>
        <td>Data ID 1</td>
        <td>Data readout (multi-turn data)</td>
        <td>In this command we will receive data about:
		No. of rotor turns in field name ABM.
		Errors and warnings in field name SF(status field).
		</td>
    </tr>
	<tr>
        <td>Data ID 2</td>
        <td>Endoder-ID</td>
        <td>In this command we will receive data about :
		Tamagawa encoder make-ID in ENID field .
		Errors and warnings in field name SF(status field)
		</td>
    </tr>
	<tr>
        <td>Data ID 3</td>
        <td>Data readout(absolute+multiturn+encoder-ID)</td>
        <td>In this command we will receive :
		Absolute rotor position value in field name ABS.
		No. of rotor turns in field name ABM.
		Tamagawa encoder make-ID in ENID field .
		Errors and warnings in field name SF(status field)
		Other warnings in field name ALMC
		</td>
    </tr>
	<tr>
        <td>Data ID 5</td>
        <td>Reset-Error</td>
        <td>This command used to reset errors. </td>
    </tr>
	<tr>
        <td>Data ID 6</td>
        <td>Reset- absolute</td>
        <td>This command used to reset absolute position data(ABS) </td>
    </tr>    <tr>
        <td>Data ID 7</td>
        <td>Reset- multiturn</td>
        <td>This command used to reset multi-turn data(ABM) </td>
    </tr>
</table>