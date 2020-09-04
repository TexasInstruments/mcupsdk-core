# Enabling Real Time Debug {#REAL_TIME_DEBUG_SUPPORT_GUIDE}

[TOC]

## Introduction

With Real time debug, global variables can be added to expression window in CCS and read/written during continuous run of the program. It is enabled by UART connection between CCS and SOC. The connection is built by debug program in the listed files.

- Serial_Cmd_Monitor.c
- Serial_Cmd_Monitor.h
- Serial_Cmd_HAL.c
- Serial_Cmd_HAL.h

Even though there are four files listed here, there are only two functions required in application program. One
is "SerialCmd_init()" called in initialization and the other is "SerialCmd_read()" called in background loop of
BareMetal or low priority task of RTOS. This section focus on how to create the UART connection and how to
launch real time debug in CCS.

## Confirm CCS Features

It is recommended to check the following CCS driver file if the CCS version is older than 11.1. The configuration
of Cortex_R5 should be similar to below figure. If any line is missing, it is necessary to add the line showing in
Figure. As for content of the lines, COM Port and Baud Rate need to be updated in target configuration file,
which is included in the next step.

* ccs\ccs_base\common\targetdb\drivers\gti_uart_driver.xml

 \code
	<isa Type="Cortex_R5" ProcID="0x75803400">
		<driver file="../../../DebugServer/drivers/XPCOMToGTIAdapter.dvr">
			<property Type="stringfield" Value="COM14" id="COM Port" />
			<property Type="stringfield" Value="9600" id="Baud Rate" />
			<property Type="hiddenfield" Value="Little Endian" id="Endianness" />
			<property Type="hiddenfield" Value="32" id="Word Size Page 0" />
			<property Type="hiddenfield" Value="8" id="Minimum Addressable Size Page 0" />
			<property Type="hiddenfield" Value="@ti.com/UARTMonitor;1" id="XPCOM Class ID" />
			<property Type="hiddenfield" Value="Flash DLL Delegate" id="TargetAccess" />
			<connectionType Type="UARTConnection"/>
        </driver>
	</isa>
 \endcode

## Create Target Configuration File

For a control card in below Figure, it offers both JTAG and UART ports in one USB port. Details on hardware
connection can be found in AM263x Control Card User's Guide. It is necessary to create a target configuration
file for the debug ports. A step-to-step guide is given below. A target configuration file is created and then
configured with both JTAG and UART. The UART COM Port in target configuration should match PC Device Manager COM Port for
JTAG probe Application/User UART. The Baud Rate in target configuration should be consistent with SoC UART Baud Rate configured
in next step.

  \imageStyle{evm_overview_3.png,width:80%}
  \image html evm_overview_3.png "AM263x Control Card"

- **Step 1**: Creating a new Target configuration file for the SOC.

  \imageStyle{new_target_config.png,width:40%}
  \image html new_target_config.png "Create New Target Configuration File"

- **Step 2**: Select proper JTAG connection and Device in Basic configuration.

  \imageStyle{jtag_and_device.png,width:60%}
  \image html jtag_and_device.png "Select JTAG Connection and Device"

- **Step 3**: Add UART communication port for UART monitor as shown below.

  \imageStyle{add_uart_port.png,width:60%}
  \image html add_uart_port.png "Add UART Communication Port"

- **Step 4**: Open Advanced target configuration and add "Cortex_R5" as component.

  \imageStyle{adv_target_config.png,width:80%}
  \image html adv_target_config.png "Open Advanced Target Configuration"

  \imageStyle{add_component.png,width:40%}
  \image html add_component.png "Add Component"

- **Step 5**: Update the COM PORT and Baud Rate for the component. UART COM port can be
located as shown in the next step. The Baud rate should be same as for the UART configured in SOC.

  \imageStyle{update_cpu_properties.png,width:60%}
  \image html update_cpu_properties.png "Select CPU Properties"

- **Step 6**: Find the XDS110 UART COM port from device manager.

  \imageStyle{uart_com_port.png,width:60%}
  \image html uart_com_port.png "Find XDS110 UART COM Port"

##  Add Serial Command Monitor Software

There are multiple ways to use UART0 as a debug interface. They are Debug Log and Serial Command
Monitor. Debug Log is a built-in tool located at Driver Porting Layer of SDK. Like Serial Cmd Monitor, its function
must be located out of interrupt callback. It is a handy tool enabling string input and output. But, input and
output go through UART console only. There is no built-in GUI like Expression Window and Graph in CCS.
It is recommended to disable UART0 in Debug Log SysCfg as shown below and configure UART0 instance for Serial
Command Monitor. As the name of UART instance in Sysconfig "CONFIG_UART_CONSOLE", matches
the instance name in "Serial_Cmd_HAL.c", it not necessary to modify the two functions required by initialization
and background loop. They can be simply inserted into any application.

  \imageStyle{disable_uart_log.png,width:40%}
  \image html disable_uart_log.png "Disable UART Log in Debug Log"

  \imageStyle{configure_uart_instance.png,width:40%}
  \image html configure_uart_instance.png "Configure UART0 Instance"

## Launch Real Time Debug

After building the program, debug window should be opened with the target configuration file created above.
If the created target configuration file is not already opened, it can be located by following figure
and looking into "User Defined" folder of the "Target Configuration" window.

  \imageStyle{locate_target_config.png,width:20%}
  \image html locate_target_config.png "Locate Target Configuration File"

The created target configuration file should be under folder named as "User Defined". After right click on the file,
a menu shows up and there is a option "Launch Selected Configuration" as shown below.

  \imageStyle{launch_target_config.png,width:40%}
  \image html launch_target_config.png "Launch Selected Configuration"

Then, debug window shows up. The steps to connect target, load image and run via JTAG can be found in many CCS tutorials.
The processor must be running continuously before connecting to UART. As the UART connection is based on continuous operation of
the program, UART connection will be broken and CCS will be frozen by Break-point, Suspend, Terminate or any
other events stopping the Serial Command Monitor program from running. Sometimes, it is just a habit to use
those features when they are available. It is recommended to disconnect target via JTAG as shown below while using UART connection.

  \imageStyle{disconnect_jtag.png,width:60%}
  \image html disconnect_jtag.png "Disconnect JTAG Connection"

When the processor is running, UART connection can be established by simply select the UART connection → Run → Load → Load Symbols as shown below.

  \imageStyle{establish_uart_connection.png,width:80%}
  \image html establish_uart_connection.png "Establish UART Connection"

Expression window can be opened from View -> Expressions. Global variables can be added by clicking on "Add button". Continuous refresh can be enabled for the value to get continuously updated in CCS.

  \imageStyle{expression_value.png,width:40%}
  \image html expression_value.png "Observe the global variable in CCS"

