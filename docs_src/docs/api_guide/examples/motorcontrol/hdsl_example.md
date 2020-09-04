#  HDSL Diagnostic {#EXAMPLE_MOTORCONTROL_HDSL}
[TOC]

## Introduction
The HDSL diagnostic application described here interacts with the firmware interface.

HDSL diagnostic application does below,
- Configures pinmux, GPIO, ICSS clock to 225MHz,
- Initializes ICSS0-PRU1, ICSS0-IEP0 and IEP1(for SYNC mode support.Timesync router is used to latch the loopback.),
- Loads lookup table for encoding/decoding of Hiperface data
- Loads the initialization section of PRU firmware & executes it.

Firmware is split to three sections, initialization, datalink and transport.
At startup, the application displays details about encoder and status.
It then presents the user with menu options, based on the option selected, application communicates with HDSL interface and the result is presented to the user.

## Important files and directory structure

<table>
<tr>
    <th>Folder/Files
    <th>Description
</tr>
<tr><td colspan="2" bgcolor=#F0F0F0> ${SDK_INSTALL_PATH}/examples/motor_control/hdsl_diagnostic</td></tr>
<tr>
    <td>hdsl_diagnostic.c
    hdsl_diagnostic.h</td>
	<td> Source and Header files </td>
</tr>
<tr><td colspan="2" bgcolor=#F0F0F0> ${SDK_INSTALL_PATH}/source/motor_control/position_sense/hdsl</td></tr>
<tr>
    <td>driver/</td>
    <td>Folder containing HDSL PRU driver sources.</td>
</tr>
<tr>
    <td>include/</td>
    <td>Folder containing HDSL PRU header sources.</td>
</tr>
<tr>
    <td>firmware/</td>
    <td>Folder containing HDSL PRU firmware sources.</td>
</tr>

</table>

\cond SOC_AM64X || SOC_AM243X

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0 freertos
 ICSSG          | ICSSG0
 PRU            | PRU1
 Toolchain      | ti-arm-clang
 Board          | @VAR_BOARD_NAME_LOWER
 Example folder | examples/motorcontrol/hdsl_example

\endcond

# Steps to Run the Example

## Hardware Prerequisites

Other than the basic EVM setup mentioned in \ref EVM_SETUP_PAGE, below additional HW is required to run this demo
- HDSL encoder
- Below are two options to connect encoder to AM64x/AM243x EVM.
    - **Option 1**
        - TIDA-00179 Universal Digital Interface to Absolute Position Encoders, http://www.ti.com/tool/TIDA-00179
        - TIDEP-01015 3 Axis board
        - Interface card connecting EVM and TIDEP-01015 3 Axis board
        - Connect the Hiperface DSL encoder to HDSL+/-(Pin number 6 and 7) signals available on header J7 or Sub-D15 connector of the "Universal Digital Interface to Absolute Position Encoders" board.
    - **Option 2**
        - HDSL AM64xE1 Transceiver. If application is using this card, define the macro HDSL_AM64xE1_TRANSCEIVER in the CCS project/make file.
        - Connect the Hiperface DSL encoder to J10.
		- HDSL AM64xE1 Transceiver supports two channels that can be used to support HDSL safety, multi axis servo drives.
		- Schematics are shared in the MCU+SDK package. For more design details of the transceiver card, please contact TI via E2E/FAE.
		- \htmllink{../am64x_am243x/HDSL_AM64xE1_Schematics.pdf, HDSL Transceiver Card Schematics} document.

## Hardware Setup(Using TIDA-00179, TIDEP-01015 and Interface board)

\imageStyle{HDSL_Connections.png,width:40%}
\image html HDSL_Connections.png "Hardware Setup"

## Hardware Setup(Using HDSL AM64xE1 Transceiver)

\imageStyle{HDSL_AM64xE1.png,width:60%}
\image html HDSL_AM64xE1.png "Hardware Setup"

## Build, load and run

- **When using CCS projects to build**, import the CCS project and build it using the CCS project menu (see \ref CCS_PROJECTS_PAGE).
- **When using makefiles to build**, note the required combination and build using
  make command (see \ref MAKEFILE_BUILD_PAGE)
- Launch a CCS debug session and run the executable, see \ref CCS_LAUNCH_PAGE
- Refer to UART terminal for user interface menu options.

# Sync Mode:

- Note

This is a test feature, in real application - PWM syncout will be connected to Latch input instead of IEP1 sync.

## Synchronization with external Pulse
According to the Hiperface DSL specification, the falling edge inside the EXTRA window should coincide with the external synchronization pulse.
At the beginning of the startup phase, the firmware measures the time interval of the external pulse and calculates the required number of bits for the H-Frame.
Based on this number the stuffing length and EXTRA window size is derived.
Afterwards, the PRU waits to match its timing with the timing of the external synchronization pulse and starts the transmission.
Since it is possible to use time intervals for the external pulse that are not multiples of the bit duration, the firmware needs to adjust the H-Frame size on the fly.
Furthermore, during the EXTRA window the PRU transmits the data (sample edge) with a granularity of 13.3ns to increase the synchronization accuracy. Figure "Synchronization of External Pulse with Sample Edge in EXTRA Window" and "Illustration of Synchronization Algorithm" depict the concept.
The EXTRA_TIME_WINDOW is a fixed value that is calculated at startup to match the external pulse frequency. The TIME_REST value gives the number of overclocked ‘1’ that needs to be sent during the last bit of the EXTRA window.

\imageStyle{hdsl_external_sync.png,width:40%}
\image html hdsl_external_sync.png "Synchronization of External Pulse with Sample Edge in EXTRA Window"

In other words, the TIME_REST value represents the sample edge in a fine granularity dimension (13.3ns). While the sample edge can be send with a finer granularity, the granularity of the size of the EXTRA window is still in whole bit durations (106.67ns).
Consequently, there is an overhead, if the external pulse period is not a multiple of the bit duration. This overhead is compensated in the next H-Frame by changing the size of the EXTRA window. As a result, the size of the H-Frame is varying over time.
It is possible that these calculations lead to the excess of the maximum or minimum EXTRA window size. Therefore, the number of bits for the stuffing and EXTRA window is readjusted on a violation.

\imageStyle{hdsl_sync_algo.png,width:40%}
\image html hdsl_sync_algo.png "Illustration of Synchronization Algorithm"

The algorithm is given as C code in the following:

			/* EXTRA_SIZE equals the number of bits for the EXTRA window minus 1 */
			if(EXTRA_EDGE == 0)
				TIME_REST += 8;
			short b = (EXTRA_SIZE << 3) + TIME_REST;
			short overhead = (EXTRA_SIZE << 3) + 8 - TIME_EXTRA_WINDOW;
			EXTRA_SIZE = (b - overhead) >> 3;
			TIME_REST = (b - overhead) - (EXTRA_SIZE << 3);

			if(EXTRA_SIZE < 3) {
				EXTRA_SIZE += 6;
				NUM_STUFFING -= 1;
				TIME_EXTRA_WINDOW += (8*6);
			}
if(EXTRA_SIZE > 8) {
				EXTRA_SIZE -= 6;
				NUM_STUFFING += 1;
				TIME_EXTRA_WINDOW -= (8*6);
			}


EXTRA_EDGE represents the TIME_REST value in a format that can be pushed to the TX FIFO for transmission. For instance, if TIME_REST is 4, EXTRA_EDGE is 0xf0. The edge would be in the middle of the bit duration. The value NUM_STUFFING gives the number of stuffing blocks (each block consist of 6 bits).


For further improvement of the synchronization, the time difference (∆t) between the external pulse and the sample edge we transmit is measured (Figure "Time difference between External Pulse and Sample Edge").

\imageStyle{hdsl_external_sync_sample_edge.png,width:40%}
\image html hdsl_external_sync_sample_edge.png "Time difference between External Pulse and Sample Edge"


# Sample Output

Shown below is a sample output when the application is run:

- Freerun mode:
\imageStyle{hdsl_default_uart_menu.PNG,width:60%}
\image html hdsl_default_uart_menu.PNG "HDSL DDR UART Default Menu"

- Sync Mode:
\imageStyle{HDSL_SYNC.png,width:60%}
\image html HDSL_SYNC.png "HDSL Diagnostic in SYNC mode"
