# LIN External Commander Transmit {#EXAMPLES_DRIVERS_LIN_EXTERNAL_COMMANDER}

[TOC]

# Introduction

\cond SOC_AM263X || SOC_AM263PX
This example is a application which demonstrates the LIN message
communication to external PC via PLIN-USB (from PEAK Systems -> IPEH-004052)
Instance LIN1 is set as a Commander in Transmit Mode.

The LIN1 instance initiates the transmission by sending LINID followed by message.
The size of the message is also increased with increasing id.
\endcond

# Supported Combinations {#EXAMPLES_DRIVERS_LIN_EXTERNAL_COMMANDER_COMBOS}

\cond SOC_AM263X || SOC_AM263PX

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0 freertos
 ^              | r5fss0-0 nortos
 Toolchain      | ti-arm-clang
 Boards         | @VAR_BOARD_NAME_LOWER
 Example folder | examples/drivers/lin/lin_external

\endcond

\note This Example is only functional on am263x-cc E2 board. This is not supported on am263x-cc E1 or am263x-lp. For support on these boards, please reach out to your TI representative. This example is only functional on Windows setup due to limited availibility of PLIN-View Pro.

# Steps to Run the Example

- **Board Configuration**, change the SW9 to ON mode, and change the SW10 to 1-2.
\imageStyle{lin_external_sw9_config.PNG,width:12.5%}
\image html lin_external_sw9_config.PNG LIN SW9 set to ON

\imageStyle{lin_external_sw10_config.PNG,width:12.5%}
\image html lin_external_sw10_config.PNG LIN SW10 connected in 1-2

- **Hardware Conectivity**, connect the PLIN-USB module to PC from USB and Serial Port to be connected as mentioned in the image below.
\imageStyle{lin_external_hw_connect.PNG,width:50%}
\image html lin_external_hw_connect.PNG LIN Hardware Connectivity with PLIN USB.

- **Software Setup**, Download and Install the PLIN-View Pro from https://www.peak-system.com/PLIN-USB.485.0.html?&L=1
- Click on LIN in the menu bar and connect to PLIN-USB. Make sure the LINMode for PLIN is Responder with Baud Rate of 19200.
\imageStyle{lin_plin_connection_settings.PNG,width:25%}
\image html lin_plin_connection_settings.PNG LIN Hardware Connectivity with PLIN USB.

- After successful connection with PLIN-USB, the status of connection is shown as the bottom of the screen.
\imageStyle{lin_plin_connection_success.PNG,width:50%}
\image html lin_plin_connection_success.PNG PLIN Connectivity status.

- **When using CCS projects to build**, import the CCS project for the required combination
  and build it using the CCS project menu (see \ref CCS_PROJECTS_PAGE).
- **When using makefiles to build**, note the required combination and build using
  make command (see \ref MAKEFILE_BUILD_PAGE)
- Launch a CCS debug session and run the executable, see \ref CCS_LAUNCH_PAGE

# Overridding the default configurations

- To override the default configurations or initializations, uncheck the box of Default Configuration and rebuild the example. The same example will work without any other changes.

\imageStyle{lin_override_default_config.png,width:50%}
\image html lin_override_default_config.png Bypassing Default Configuration.

# See Also

\ref DRIVERS_LIN_PAGE

# Sample Output

Shown below is a sample output when the application is run,

\code
[LIN] LIN mode external, application started ...
[I2C] LIN Volatage Level Shifter started ...
[LIN] : New Data Sent = 12
[LIN] : New Data Sent = 34
[LIN] : New Data Sent = 56
[LIN] : New Data Sent = 78
[LIN] : New Data Sent = 9a
[LIN] : New Data Sent = ab
[LIN] : New Data Sent = cd
[LIN] : New Data Sent = ef
All tests have passed!!
\endcode

Shown below is a sample output on PLIN-View Pro
\imageStyle{lin_plin_external_result.png,width:50%}
\image html lin_plin_external_result.png Output on PLIN-View Pro