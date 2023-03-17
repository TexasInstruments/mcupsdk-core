# Tamagawa over Uart Example {#EXAMPLE_MOTORCONTROL_TAMAGAWA_OVER_UART}
[TOC]


## Introduction

Tamagawa over uart application does below,

- Configures pinmux, GPIO, UART (UART clock to 192MHz, Baud rate, etc.)
- Initializes UART0 for debug log & UART1 for communication
- Load and executes tamagawa example on R5_0


Connect the Tamagawa encoder via RS-485 Half-Duplex EVM to Am263x-LP.
The connections between AM263x LP and RS-485

UART RX Pin(UART1_RXD)->JMP1-R,
UART TX Pin(UART1_TXD)->JMP4-D,
GPIO Pin(GPIO62)->JM3-DE

The tamagawa over uart example runs on R5 and communicates with tamagawa encoder by UART1. It presents the user with menu options to select Data ID code (as defined by Tamagawa) to be sent to the encoder. The application collects the data entered by the user and configures the relevant command. Then via the UART1 write API, the command is passed to encoder. Once the command is sent, the encoder starts to respond, and UART1 read API starts to read this response. Response is stored in the tamagawa interface, the status of the transaction is check by CRC calculation. If the status indicates success, the result is presented to the user otherwise print CRC failure.





### Example Flow-Chart

\image html Tamagawa_uart_flow_chart.png "Tamagawa UART example flow-chart"

## Important files and directory structure
<table>
<tr>
    <th>Folder/Files
    <th>Description
</tr>
<tr><td colspan="2" bgcolor=#F0F0F0> ${SDK_INSTALL_PATH}/examples/motor_control/tamagawa_diagnostic_over_soc_uart/uart_tamagawa.c</td></tr>
<tr>
    <td>uart_tamagawa.c</td>
    <td>Tamagawa UART application</td>
</tr>
<tr><td colspan="2" bgcolor=#F0F0F0> ${SDK_INSTALL_PATH}/source/motor_control/position_sense/tamagawa_over_soc_uart</td></tr>
<tr>
    <td>include/</td>
    <td>Folder containing tamagawa interface file.</td>
</tr>
<tr>
    <td>driver/</td>
    <td>Tamagawa Uart driver.</td>
</tr>
</table>

# Supported Combinations
\cond SOC_AM263X

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0 freertos
 Toolchain      | ti-arm-clang
 Board          | @VAR_LP_BOARD_NAME_LOWER
 Example folder | examples/motor_control/tamagawa_diagnostic_over_soc_uart

\endcond


# Steps to Run the Example

## Hardware Prerequisites
-  Tamagawa Encoders
-  AM263x LP
-  RS-485 Half Duplex EVM
-  5V and 3.3V power supplier

## Hardware Setup

\imageStyle{Tamagawa_Uart_Hw_Setup.PNG,width:60%}
\image html Tamagawa_Uart_Hw_Setup.PNG "Tamagawa Encoder Hardware Setup with AM263x"

\imageStyle{Tamagawa_Setup_image.jpg,width:60%}
\image html Tamagawa_Setup_image.jpg "Hardware Setup For AM263x-LP"


## Build, load and run

- **When using CCS projects to build**, import the CCS project and build it using the CCS project menu (see \ref CCS_PROJECTS_PAGE).
- **When using makefiles to build**, note the required combination and build using
  make command (see \ref MAKEFILE_BUILD_PAGE)
- Launch a CCS debug session and run the executable, see \ref CCS_LAUNCH_PAGE
- Refer to UART terminal for user interface menu options.

### Sample Output

Shown below is a sample output when the application is run:

\imageStyle{Tamagawa_UART_output.PNG,width:60%}
\image html Tamagawa_UART_output.PNG "Tamagawa UART Sample Output"

### Test Case Description

<table>
    <tr>
        <th>Data_ID
        <th>Name
        <th>Description
        <th>Pass/fail Criteria
    </tr>
    <tr>
        <td>Data ID 0</td>
        <td>Data readout (absolute position data)</td>
        <td>In this command we will receive:
		Absolute rotor position value in field name ABS.
		Errors and warnings in field name SF(status field)
		</td>
        <td>CRC success with ABS, SF, CF and CRC values printed in the terminal.</td>
    </tr>
	<tr>
        <td>Data ID 1</td>
        <td>Data readout (multi-turn data)</td>
        <td>In this command we will receive data about:
		No. of rotor turns in field name ABM.
		Errors and warnings in field name SF(status field).
		</td>
        <td>CRC success with ABM, SF, CF and CRC values printed in the terminal.</td>
    </tr>
	<tr>
        <td>Data ID 2</td>
        <td>Endoder-ID</td>
        <td>In this command we will receive data about :
		Tamagawa encoder make-ID in ENID field.
		Errors and warnings in field name SF(status field)
		</td>
        <td>CRC success with ENID, SF, CF and CRC values printed in the terminal.</td>
    </tr>
	<tr>
        <td>Data ID 3</td>
        <td>Data readout(absolute+multiturn+encoder-ID)</td>
        <td>In this command we will receive :
		Absolute rotor position value in field name ABS.
		No. of rotor turns in field name ABM.
		Tamagawa encoder make-ID in ENID field.
		Errors and warnings in field name SF(status field)
		Other warnings in field name ALMC
		</td>
        <td>CRC success with ABS, ENID, ABM, ALMC, SF, CF and CRC values printed in the terminal.</td>
    </tr>
    <tr>
        <td>Data ID 6</td>
        <td>Writing to EEPROM</td>
        <td>In this command you provide :
        Proper address of the EEPROM where you want to write
		Proper data that you want to write.<br>
        As a response you recieve:
        Control Field for EEPROM Write command
        EEPROM address that you want to write to
        Data that you want to write to the EEPROM
        CRC value
		</td>
        <td>CRC success with EDF, ADF, CF and CRC values printed in the terminal.</td>
    </tr>
    <tr>
        <td>Data ID D</td>
        <td>Readout from EEPROM</td>
        <td>In this command you provide :
        Proper address of the EEPROM that you want to read.<br>
		As a response you recieve:
        Control Field for EEPROM Write command
        EEPROM address that you want to write to
        Data that you want to write to the EEPROM
        CRC value
		</td>
        <td>CRC success with EDF, ADF, CF and CRC values printed in the terminal.</td>
    </tr>
	<tr>
        <td>Data ID 7</td>
        <td>Reset-Error</td>
        <td>This command used to reset errors. </td>
        <td>CRC success with ABS, SF, CF and CRC values printed in the terminal.</td>
    </tr>
	<tr>
        <td>Data ID 8</td>
        <td>Reset- absolute</td>
        <td>This command used to reset absolute position data(ABS) </td>
        <td>CRC success with ABS, SF, CF and CRC values printed in the terminal.</td>
    </tr>    <tr>
        <td>Data ID C</td>
        <td>Reset- multiturn</td>
        <td>This command used to reset multi-turn data(ABM) </td>
        <td>CRC success with ABS, SF, CF and CRC values printed in the terminal.</td>
    </tr>
</table>

