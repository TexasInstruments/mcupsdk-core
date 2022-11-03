#  EnDAT Diagnostic {#EXAMPLE_MOTORCONTROL_ENDAT}

[TOC]

The EnDat diagnostic application, described here,
demonstrates the EnDat receiver operation.

The EnDat driver provides a well defined set of APIs to expose EnDat
receiver interface.



The diagnostic invokes these APIs to
- initialize EnDat,
- select one configuration among concurrent multi channel with Encoders of Same make, multi channel with Encoders of Different Make and single channel configuration based on SysConfig.
- select the channel (channels in the case of concurrent multi channel with encoders of same make or multi channel with Encoders of Different Make),
- configure the host trigger mode,
- and run the firmware.
- If using "Multi Channel with Encoders of Different Make" configuration is selected" :
    - enable load share mode.
    - select primary core for global configuration.
    - configuration of synchronization bits.


Once these steps are executed,
- the driver waits for the EnDat to be initialized.
- It then sets clock frequency to 200KHz (as propagation delay is not yet compensated)
- and obtains the encoder details including serial number, position resolution etc, and displays on the console/UART.
- Based on the whether encoder is 2.2 or 2.1 type, it sets clock to either 8MHz or 1MHz respectively.
- While configuring clock, propagation delay is taken care using the automatically estimated propagation delay (user can override it too).
- In the case of concurrent Multi Channel with Encoders of Same Make or Multi Channel with Encoders of Different Make, if propagation delay between various channels are different, that too is automatically taken care.

Once initial setup is over,
- the diagnostic provides the user with a self explanatory menu.
- Two types of menu options are presented. One type (1-14) will send an EnDat command as per EnDat 2.2 specification.
- The other type (100-108) allows the user to configure clock frequency, various timing parameters, simulate motor control loop using 2.1 command as well as 2.2 command with safety (redundant position information), switch to continuous clock mode and monitor raw data.
- Concurrent multi channel with Encoder of Same Make configuration can work simultaneously for up-to three encoders with identical part number, all variants of 2.2 position commands as well as the 2.1 position command is supported and an additional option (109) to configure wire delay (useful when propagation delay in each channel is different) is available.
- Single PRU core handles enabled channels in single channel and Multi Channel with Encoders of Same Make configuration.
- Application by default, handles wire delay as required, the menu option provides a way to override it.

After the user selects an EnDat command,
- the diagnostic asks for more details to frame the command and performs a basic sanity check on the user entered values.
- Then the EnDat APIs are invoked to process the command set, set the host trigger bit and waiting until the host trigger bit cleared, If multi-channel with Encoders of Different make is used, these operations are done for each channel".
- The received EnDat is processed & validated using the defined APIs. The result is then presented to the user.

### Channel Selection In Sysconfig

\image html EnDat_channel_selection_In_sysconfig.PNG      "Channel Selection In Sysconfig"


\image html Endat_channel_selection_configuration.png     "EnDAT configuration seletion between Single/Multi channel "

### Endat Example Implementation

Following section describes the Example implementation of EnDat on ARM(R5F).
\image html Endat_Example_Implementation.png "Endat Example"

## Important files and directory structure

<table>
<tr>
    <th>Folder/Files
    <th>Description
</tr>
<tr><td colspan="2" bgcolor=#F0F0F0> ${SDK_INSTALL_PATH}/examples/motor_control/endat_diagnostic</td></tr>
<tr>
    <td>endat_diagnostic.c</td>
    <td>EnDAT diagnostic application</td>
</tr>
<tr><td colspan="2" bgcolor=#F0F0F0> ${SDK_INSTALL_PATH}/source/motor_control/position_sense/endat</td></tr>
<tr>
    <td>firmware/</td>
    <td>Folder containing EnDAT firmware sources.</td>
</tr>
<tr>
    <td>driver/</td>
    <td>EnDAT diagnostic driver.</td>
</tr>
</table>

# Supported Combinations {#EXAMPLES_MOTORCONTROL_ENDAT_COMBOS}

\cond SOC_AM64X || SOC_AM243X

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0 freertos
 ICSSG          | ICSSG0
 PRU            | PRU1
 Toolchain      | ti-arm-clang
 Board          | @VAR_BOARD_NAME_LOWER
 Example folder | examples/motorcontrol/endat_example

\endcond


# Steps to Run the Example

## Hardware Prerequisites

Other than the basic EVM setup mentioned in \ref EVM_SETUP_PAGE, below additional HW is required to run this demo
- EnDAT encoder
- TIDA-00179 Universal Digital Interface to Absolute Position Encoders, http://www.ti.com/tool/TIDA-00179
- TIDEP-01015 3 Axis board
- Interface card connecting EVM and TIDEP-01015 3 Axis board

## Hardware Setup

\imageStyle{EnDAT_Connections.png,width:40%}
\image html EnDAT_Connections.png "Hardware Setup"

## Build, load and run

- **When using CCS projects to build**, import the CCS project and build it using the CCS project menu (see \ref CCS_PROJECTS_PAGE).
- **When using makefiles to build**, note the required combination and build using
  make command (see \ref MAKEFILE_BUILD_PAGE)
- Launch a CCS debug session and run the executable, see \ref CCS_LAUNCH_PAGE
- Refer to UART terminal for user interface menu options.

### Sample Output

Shown below is a sample output when the application is run:

\imageStyle{EnDAT_Initialization_UART_PRINT.png,width:60%}
\image html EnDAT_Initialization_UART_PRINT.png "EnDAT Usage"

### Test Case Description

<table>
    <tr>
        <th>Test detail
        <th>Steps
        <th>Pass/fail crieteria
    </tr>
    <tr>
        <td>To check position value</td>
        <td>1. Select Single channel mode from UART menu </td>
        <td> </td>
    </tr>
    <tr>
        <td></td>
        <td>2. Enter 0 to select channel 0 </td>
        <td> </td>
    </tr>
    <tr>
        <td></td>
        <td>3. Enter 1 to select "Encoder send position values"</td>
        <td>Crc success </td>
    </tr>
    <tr>
        <td>To receive encoder's operating parameters(error messege)</td>
        <td>1. Select Single channel mode from UART menu </td>
        <td> </td>
    </tr>
    <tr>
        <td></td>
        <td>2. Enter 0 to select channel 0 </td>
        <td> </td>
    </tr>
    <tr>
        <td></td>
        <td>3. Enter 2 to select "Selection of memory area"</td>
        <td>Crc success </td>
    </tr>
    <tr>
        <td></td>
        <td>4. Enter "B9"  in MRS code to select "Operating parameters"</td>
        <td>Crc success </td>
    </tr>
    <tr>
        <td></td>
        <td>5. Enter 4 to select "Encoder to send parameter"</td>
        <td>Crc success </td>
    </tr>
    <tr>
        <td></td>
        <td>6. Enter 0 in "parameter address" for selecting "Error message"</td>
        <td>Crc success </td>
    </tr>
    <tr>
        <td>To receive encoder's manufacture parameters for Endat 2.2</td>
        <td>1. Select Single channel mode from UART menu </td>
        <td> </td>
    </tr>
    <tr>
        <td></td>
        <td>2. Enter 0 to select channel 0 </td>
        <td> </td>
    </tr>
    <tr>
        <td></td>
        <td>3. Enter 2 to select "Selection of memory area"</td>
        <td>Crc success </td>
    </tr>
    <tr>
        <td></td>
        <td>4. Enter "BD"  in MRS code to select "Parameters of encoder manufacturer for Endat 2.2"</td>
        <td>Crc success </td>
    </tr>
    <tr>
        <td></td>
        <td>5. Enter 4 to select "Encoder to send parameter"</td>
        <td>Crc success </td>
    </tr>
    <tr>
        <td></td>
        <td>6. Enter 0 in "parameter address" for selecting "Status of additional info"</td>
        <td>Crc success </td>
    </tr>
    <tr>
        <td>To set values to encoder's operating parameters (error message)</td>
        <td>1. Select Single channel mode from UART menu </td>
        <td> </td>
    </tr>
    <tr>
        <td></td>
        <td>2. Enter 0 to select channel 0 </td>
        <td> </td>
    </tr>
    <tr>
        <td></td>
        <td>3. Enter 2 to select "Selection of memory area"</td>
        <td>Crc success </td>
    </tr>
    <tr>
        <td></td>
        <td>4. Enter "B9"  in MRS code to select "Operating parameters"</td>
        <td>Crc success </td>
    </tr>
    <tr>
        <td></td>
        <td>5. Enter 3 to select "Encoder to receive parameter"</td>
        <td>Crc success </td>
    </tr>
    <tr>
        <td></td>
        <td>6. Enter 0 in "parameter address" for selecting "Error message"</td>
        <td>Crc success </td>
    </tr>
    <tr>
        <td></td>
        <td>7. Enter 0 in "parameter value"  for seting value in " Error message"</td>
        <td>Crc success </td>
    </tr>
    <tr>
        <td>To set values to encoder's manufacturing parameters for Endat 2.2(Status of additional info)</td>
        <td>1. Select Single channel mode from UART menu </td>
        <td> </td>
    </tr>
    <tr>
        <td></td>
        <td>2. Enter 0 to select channel 0 </td>
        <td> </td>
    </tr>
    <tr>
        <td></td>
        <td>3. Enter 2 to select "Selection of memory area"</td>
        <td>Crc success </td>
    </tr>
    <tr>
        <td></td>
        <td>4. Enter "BD"  in MRS code to select "Parameters of encoder manufacturer for Endat 2.2"</td>
        <td>Crc success </td>
    </tr>
    <tr>
        <td></td>
        <td>5. Enter 3 to select "Encoder to receive parameter"</td>
        <td>Crc success </td>
    </tr>
    <tr>
        <td></td>
        <td>6. Enter 0 in "parameter address" for selecting "Status of additional info"</td>
        <td>Crc success </td>
    </tr>
    <tr>
        <td></td>
        <td>7. Enter 0 in "parameter value"  for seting value in " Status of additional info"</td>
        <td>Crc success </td>
    </tr>
    <tr>
        <td>To reset encoder</td>
        <td>1. Select Single channel mode from UART menu </td>
        <td> </td>
    </tr>
    <tr>
        <td></td>
        <td>2. Enter 0 to select channel 0 </td>
        <td> </td>
    </tr>
    <tr>
        <td></td>
        <td>3. Enter 5 to select "Encoder receive reset"</td>
        <td>Crc success </td>
    </tr>
    <tr>
        <td>To receive test values from encoder with port address "0"</td>
        <td>1. Select Single channel mode from UART menu </td>
        <td> </td>
    </tr>
    <tr>
        <td></td>
        <td>2. Enter 0 to select channel 0 </td>
        <td> </td>
    </tr>
    <tr>
        <td></td>
        <td>3. Enter 7 to select "Encoder receive test command"</td>
        <td> </td>
    </tr>
    <tr>
        <td></td>
        <td>4. Enter 0 in "enter port address" </td>
        <td> </td>
    </tr>
    <tr>
        <td></td>
        <td>5. Enter 6 to select "Encoder send test values"</td>
        <td>Crc success </td>
    </tr>
    <tr>
        <td>To receive test values from encoder with port address "E" </td>
        <td>1. Select Single channel mode from UART menu </td>
        <td> </td>
    </tr>
    <tr>
        <td></td>
        <td>2. Enter 0 to select channel 0 </td>
        <td> </td>
    </tr>
    <tr>
        <td></td>
        <td>3. Enter 7 to select "Encoder receive test command"</td>
        <td> </td>
    </tr>
    <tr>
        <td></td>
        <td>4. Enter "E" in "enter port address" </td>
        <td> </td>
    </tr>
    <tr>
        <td></td>
        <td>5. Enter 6 to select "Encoder send test values"</td>
        <td>Crc success </td>
    </tr>
    <tr>
        <td>To check position value with aditional info.</td>
        <td>1. Select Single channel mode from UART menu </td>
        <td> </td>
    </tr>
    <tr>
        <td></td>
        <td>2. Enter 0 to select channel 0 </td>
        <td> </td>
    </tr>
    <tr>
        <td></td>
        <td>3. Enter 8 to select "Encoder send position values + AI(s)"</td>
        <td>Crc success </td>
    </tr>
    <tr>
        <td>To receive encoder's operating parameters(error messege)
		+receive position value with  additional info
		</td>
        <td>1. Select Single channel mode from UART menu </td>
        <td> </td>
    </tr>
    <tr>
        <td></td>
        <td>2. Enter 0 to select channel 0 </td>
        <td> </td>
    </tr>
    <tr>
        <td></td>
        <td>3. Enter 9 to select "Encoder send position values + AI(s) and Selection of memory area"</td>
        <td> </td>
    </tr>
    <tr>
        <td></td>
        <td>4. Enter "B9"  in MRS code to select "Operating parameters"</td>
        <td>Crc success </td>
    </tr>
    <tr>
        <td></td>
        <td>5. Enter 11 to select "Encoder send position values + AI(s) and  send parameter"</td>
        <td> </td>
    </tr>
    <tr>
        <td></td>
        <td>6. Enter 0 in "parameter address" for selecting "Error message"</td>
        <td> </td>
    </tr>
    <tr>
        <td>To receive encoder's manufacture parameters
		for Endat 2.2 +receive position value with  additional info
		</td>
        <td>1. Select Single channel mode from UART menu </td>
        <td> </td>
    </tr>
    <tr>
        <td></td>
        <td>2. Enter 0 to select channel 0 </td>
        <td> </td>
    </tr>
    <tr>
        <td></td>
        <td>3. Enter 9 to select "Encoder send position values + AI(s) and Selection of memory area"</td>
        <td> </td>
    </tr>
    <tr>
        <td></td>
        <td>4. Enter "BD"  in MRS code to select "Parameters of encoder manufacturer for Endat 2.2"</td>
        <td>Crc success </td>
    </tr>
    <tr>
        <td></td>
        <td>5. Enter 11 to select "Encoder send position values + AI(s) and send parameter"</td>
        <td> </td>
    </tr>
    <tr>
        <td></td>
        <td>6. Enter 0 in "parameter address" for selecting "Status of additional info"</td>
        <td> </td>
    </tr>
    <tr>
        <td>To set values to encoder's operating parameters (error message)
		+receive position value with additional info
		</td>
        <td>1. Select Single channel mode from UART menu </td>
        <td> </td>
    </tr>
    <tr>
        <td></td>
        <td>2. Enter 0 to select channel 0 </td>
        <td> </td>
    </tr>
    <tr>
        <td></td>
        <td>3. Enter 9 to select "Encoder send position values + AI(s) and Selection of memory area"</td>
        <td> </td>
    </tr>
    <tr>
        <td></td>
        <td>4. Enter "B9"  in MRS code to select "Operating parameters"</td>
        <td>Crc success </td>
    </tr>
    <tr>
        <td></td>
        <td>5. Enter 10 to select "Encoder send position values + AI(s) and receive parameter"</td>
        <td> </td>
    </tr>
    <tr>
        <td></td>
        <td>6. Enter 0 in "parameter address" for selecting "Error message"</td>
        <td> </td>
    </tr>
    <tr>
        <td></td>
        <td>7. Enter 0 in "parameter value"  for seting value in " Error message"</td>
        <td> </td>
    </tr>
    <tr>
        <td>To set values to encoder's manufacturing parameters for Endat 2.2(Status of additional info)
		+receive position value with additional info
		</td>
        <td>1. Select Single channel mode from UART menu </td>
        <td> </td>
    </tr>
    <tr>
        <td></td>
        <td>2. Enter 0 to select channel 0 </td>
        <td> </td>
    </tr>
    <tr>
        <td></td>
        <td>3. Enter 9 to select "Encoder send position values + AI(s) and Selection of memory area"</td>
        <td> </td>
    </tr>
    <tr>
        <td></td>
        <td>4. Enter "BD"  in MRS code to select "Parameters of encoder manufacturer for Endat 2.2"</td>
        <td>Crc success </td>
    </tr>
    <tr>
        <td></td>
        <td>5. Enter 10 to select "Encoder send position values + AI(s) and receive parameter"</td>
        <td> </td>
    </tr>
    <tr>
        <td></td>
        <td>6. Enter 0 in "parameter address" for selecting "Status of additional info"</td>
        <td> </td>
    </tr>
    <tr>
        <td></td>
        <td>7. Enter 0 in "parameter value"  for seting value in " Status of additional info"</td>
        <td> </td>
    </tr>
    <tr>
        <td>To reset encoder +receive position value with additional info</td>
        <td>1. Select Single channel mode from UART menu </td>
        <td> </td>
    </tr>
    <tr>
        <td></td>
        <td>2. Enter 0 to select channel 0 </td>
        <td> </td>
    </tr>
    <tr>
        <td></td>
        <td>5. Enter 12 to select "Encoder send position values + AI(s) and receive error reset"</td>
        <td>Crc success </td>
    </tr>
    <tr>
        <td></td>
        <td>1. Select Single channel mode from UART menu </td>
        <td> </td>
    </tr>
    <tr>
        <td></td>
        <td>2. Enter 0 to select channel 0 </td>
        <td> </td>
    </tr>
    <tr>
        <td></td>
        <td>3. Enter 14 to select "Encoder receive communication command"</td>
        <td>Crc success </td>
    </tr>
    <tr>
        <td></td>
        <td>4. Enter ______in "enter encoder address" </td>
        <td>Crc success </td>
    </tr>
    <tr>
        <td></td>
        <td>5. Enter _____ in  "instruction hex value"</td>
        <td>Crc success </td>
    </tr>
    <tr>
        <td></td>
        <td>1. Select Single channel mode from UART menu </td>
        <td> </td>
    </tr>
    <tr>
        <td></td>
        <td>2. Enter 0 to select channel 0 </td>
        <td> </td>
    </tr>
    <tr>
        <td></td>
        <td>3. Enter 14 to select "Encoder receive communication command"</td>
        <td>Crc success </td>
    </tr>
    <tr>
        <td></td>
        <td>4. Enter ______in "enter encoder address" </td>
        <td>Crc success </td>
    </tr>
    <tr>
        <td></td>
        <td>5. Enter _____ in  "instruction hex value"</td>
        <td>Crc success </td>
    </tr>
    <tr>
        <td>Configure Clock </td>
        <td>1. Select Single channel mode from UART menu </td>
        <td> </td>
    </tr>
    <tr>
        <td></td>
        <td>2. Enter 0 to select channel 0 </td>
        <td> </td>
    </tr>
    <tr>
        <td></td>
        <td>3. Enter 100 to select "configure clock"</td>
        <td>Crc success(Tested up to 8MHz)</td>
    </tr>
    <tr>
        <td></td>
        <td>4. Enter ___ for clock frequency(in Hz)</td>
        <td> </td>
    </tr>
    <tr>
        <td>Simulate motor control 2.1 position loop</td>
        <td>1. Select Single channel mode from UART menu </td>
        <td> </td>
    </tr>
    <tr>
        <td></td>
        <td>2. Enter 0 to select channel 0 </td>
        <td> </td>
    </tr>
    <tr>
        <td></td>
        <td>3. Enter 101 to select "Simulate motor control 2.1 position loop"</td>
        <td> </td>
    </tr>
    <tr>
        <td></td>
        <td>4. Enter 10000 to select "clock frequency"</td>
        <td> </td>
    </tr>
    <tr>
        <td></td>
        <td>5. Rotate the rotor of motor and see the changes in Position value on UART</td>
        <td>Position Values are changing when rotor moves </td>
    </tr>
    <tr>
        <td>Toggle raw data display</td>
        <td>1. Select Single channel mode from UART menu </td>
        <td> </td>
    </tr>
    <tr>
        <td></td>
        <td>2. Enter 0 to select channel 0 </td>
        <td> </td>
    </tr>
    <tr>
        <td></td>
        <td>3. Enter 102 to select "Toggle raw data display"</td>
        <td> </td>
    </tr>
    <tr>
        <td></td>
        <td>4. Enter 1 to select "Encoder send position value"</td>
        <td>raw data can be displayed </td>
    </tr>
    <tr>
        <td>Configure TST delay</td>
        <td></td>
        <td> </td>
    </tr>
    <tr>
        <td>Start continous mode</td>
        <td>1. Select Single channel mode from UART menu </td>
        <td> </td>
    </tr>
    <tr>
        <td></td>
        <td>2. Enter 0 to select channel 0 </td>
        <td> </td>
    </tr>
    <tr>
        <td></td>
        <td>3. Enter 104 to select "Start continous mode"</td>
        <td> </td>
    </tr>
    <tr>
        <td></td>
        <td>4. Rotate the rotor of motor and see the changes in Position value on UART</td>
        <td>Position Values are changing when rotor moves </td>
    </tr>
    <tr>
        <td>Configure rx arm counter</td>
        <td>1. Select Single channel mode from UART menu </td>
        <td> </td>
    </tr>
    <tr>
        <td></td>
        <td>2. Enter 0 to select channel 0 </td>
        <td> </td>
    </tr>
    <tr>
        <td></td>
        <td>3. Enter 105 to select "Configure rx arm counter"</td>
        <td> </td>
    </tr>
    <tr>
        <td></td>
        <td>4. Enter ___ to "select time in ns"</td>
        <td> </td>
    </tr>
    <tr>
        <td>configure rx clock disable time</td>
        <td>1. Select Single channel mode from UART menu </td>
        <td> </td>
    </tr>
    <tr>
        <td></td>
        <td>2. Enter 0 to select channel 0 </td>
        <td> </td>
    </tr>
    <tr>
        <td></td>
        <td>3. Enter 106 to select "configure rx clock disable time"</td>
        <td> </td>
    </tr>
    <tr>
        <td></td>
        <td>4. Enter ___ to "select time in ns"</td>
        <td> </td>
    </tr>
    <tr>
        <td>Simulate motor control 2.2 position loop(safety)</td>
        <td>1. Select Single channel mode from UART menu </td>
        <td> </td>
    </tr>
    <tr>
        <td></td>
        <td>2. Enter 0 to select channel 0 </td>
        <td> </td>
    </tr>
    <tr>
        <td></td>
        <td>3. Enter 107 to select "Simulate motor control 2.2 position loop"</td>
        <td> </td>
    </tr>
    <tr>
        <td></td>
        <td>4. Enter 10000 to select "clock frequency"</td>
        <td> </td>
    </tr>
    <tr>
        <td></td>
        <td>5. Rotate the rotor of motor and see the changes in Position value on UART</td>
        <td>Position Values are changing when rotor moves </td>
    </tr>
    <tr>
        <td>Configure propogation delay(td)</td>
        <td>1. Select Single channel mode from UART menu </td>
        <td> </td>
    </tr>
    <tr>
        <td></td>
        <td>2. Enter 0 to select channel 0 </td>
        <td> </td>
    </tr>
    <tr>
        <td></td>
        <td>3. Enter 108 to select "configure propagation delay"</td>
        <td> </td>
    </tr>
</table>