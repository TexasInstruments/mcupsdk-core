# LIN Responder LLD {#EXAMPLES_DRIVERS_LIN_RESPONDER_LLD}

[TOC]

# Introduction

This example demonstrates LIN Responder mode data write and read operation using LLD driver where the Instance LIN1 is set as a Responder. The transfer is initiated by PLIN-USB (from PEAK Systems -> IPEH-004052) device connected to external PC.

The LIN1 instance waits for appropriate ID. On ID match the instance responds with data then waits for Reception of a Message from remote Commander.

Example Can be run in Polling, Interrupt and DMA mode, The Operating mode is configurable in SYSCONFIG.


# Supported Combinations {#EXAMPLES_DRIVERS_LIN_RESPONDER_LLD_COMBOS}

\cond SOC_AM263X || SOC_AM263PX

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0 nortos
 ^              | r5fss0-0 freertos
 Toolchain      | ti-arm-clang
 Boards         | @VAR_BOARD_NAME_LOWER
 Example folder | examples/drivers/lin/lin_responder_lld

\endcond

# Steps to Run the Example

\cond SOC_AM263X
- **Board Configuration**, Change the SW9 to ON state, and change the SW10 to 1-2.
\imageStyle{lin_external_sw9_config.PNG,width:12.5%}
\image html lin_external_sw9_config.PNG LIN SW9 set to ON

\imageStyle{lin_external_sw10_config.PNG,width:12.5%}
\image html lin_external_sw10_config.PNG LIN SW10 connected in 1-2

- **Hardware Conectivity**, Connect the PLIN-USB module to PC from USB and Serial Port to be connected as mentioned in the image below.
\imageStyle{lin_external_hw_connect.PNG,width:50%}
\image html lin_external_hw_connect.PNG LIN Hardware Connectivity with PLIN USB - J32 Header
\endcond

\cond SOC_AM263PX
- **Board Configuration**, Change the SW5 to ON state, and change the SW4 to 1-2.
\imageStyle{am263p_lin_external_connections_config.png,width:20%}
\image html am263p_lin_external_connections_config.png LIN SW5 and SW4 settings

- **Hardware Conectivity**, Connect the PLIN-USB module to PC from USB and Serial Port to be connected as mentioned in the image below.
\imageStyle{am263p_lin_external_hw_connect.png,width:60%}
\image html am263p_lin_external_hw_connect.png LIN Hardware Connectivity with PLIN USB - J10 Header.
\endcond


- **Software Setup**, Download and Install the PLIN-View Pro from https://www.peak-system.com/PLIN-USB.485.0.html?&L=1
- Click on LIN in the menu bar and connect to PLIN-USB. Make sure the LINMode for PLIN is Commander with Baud Rate of 19200.
\imageStyle{LIN_HLD_Responder_Write_PLIN_Setup.PNG,width:25%}
\image html LIN_HLD_Responder_Write_PLIN_Setup.PNG LIN Hardware Connectivity with PLIN USB

- Create A new Commander Read Frame with following configuration. 
\imageStyle{LIN_HLD_Responder_Write_PLIN_Frame_Setup.PNG,width:25%}
\image html LIN_HLD_Responder_Write_PLIN_Frame_Setup.PNG LIN Commander Read Frame

- Then create new Commander Write Frame Frame with following configuration.
\imageStyle{lin_plin_connection_success.PNG,width:50%}
\image html lin_plin_connection_success.PNG PLIN Connectivity status

- Create A new Frame with following configuration.
\imageStyle{LIN_HLD_Responder_Read_PLIN_Frame_Setup.PNG,width:25%}
\image html LIN_HLD_Responder_Read_PLIN_Frame_Setup.PNG LIN Commander Write Frame

- **When using CCS projects to build**, import the CCS project for the required combination
  and build it using the CCS project menu (see \ref CCS_PROJECTS_PAGE).
- **When using makefiles to build**, note the required combination and build using
  make command (see \ref MAKEFILE_BUILD_PAGE)
- Launch a CCS debug session and run the executable, see \ref CCS_LAUNCH_PAGE
- First send the Commander Read Frame. Then send Commander Write Frame

# See Also

\ref DRIVERS_LIN_V0_HLD_PAGE

# Sample Output

Shown below is a sample output when the application is run,

\code
[LIN]  LIN Responder LLD Application Started ...
[I2C] LIN Volatage Level Shifter started ...
[LIN] Responder Write ... !!!
[LIN] Responder Read ... !!!
Received Message ID: 0x18
Received Message Data:
0x56 
0x78 
0x90 
0xab 
0xcd 
0xef 
0x12 
0x34 
All tests have passed!!
\endcode

Shown below is a sample output on PLIN-View Pro
\imageStyle{LIN_HLD_Responder_Read_PC_Output.PNG,width:50%}
\image html LIN_HLD_Responder_Read_PC_Output.PNG Output on PLIN-View Pro