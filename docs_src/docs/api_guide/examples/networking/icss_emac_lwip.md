# ICSS-EMAC Lwip Example {#EXAMPLES_ICSS_EMAC_LWIP}

[TOC]

# Introduction

\note lwIP features are made available as is from public lwIP project. SDK configuration may only enable and exercise a subset of these features.

This example is an ICSS-EMAC application using the LwIP networking stack.

The examples does the following:
- Initializes the ICSS-EMAC driver for the underlying HW
- Initializes the LwIP stack
- Allows users to run and test basic networking features like ping, iperf with TCP/UDP.
- Allows users to configure between ICSS_EMAC Switch and MAC modes.
\cond SOC_AM263PX || SOC_AM263X
- Example is configured to run in Switch mode.
\endcond

\cond SOC_AM261X
# RevE2 Backward Compatibility

The ICSS-EMAC Lwip example in SDK 11.00 and above supports RevA boards by default. For RevA boards, the PHY configuration is automatically detected using the Board ID information stored in the EEPROM. However, to enable backward compatibility with RevE2 boards, which have a different PHY and configuration, manual changes to the syscfg file are required. 

The RevE2 boards use a different PHY model (DP83826E) compared to RevA boards (DP83869), and the MDIO addresses are also different. Additionally, the routing of ICSSM MDIO signals differs between the two board revisions (which is handled in the application based on the Board ID read from EEPROM). The following syscfg changes are needed to properly configure the example for RevE2 boards:

## Syscfg Changes for RevE2 Support

1. Open the Syscfg configuration for the example.
2. Navigate to the ETHPHY (ICSS-EMAC) module.
3. Change the "ETHPHY Device" from "DP83869" to "DP83826E" and change the "MDIO Phy Address" correspondingly as shown below:

   <div style="display: flex; justify-content: space-between;">
     <div style="width: 48%;">
       \imageStyle{ICSS_EMAC_LWIP_RevE2_PHY0_config.PNG,width:100%}
       \image html ICSS_EMAC_LWIP_RevE2_PHY0_config.PNG ETHPHY 0 Configuration
     </div>
     <div style="width: 48%;">
       \imageStyle{ICSS_EMAC_LWIP_RevE2_PHY1_config.PNG,width:100%}
       \image html ICSS_EMAC_LWIP_RevE2_PHY1_config.PNG ETHPHY 1 Configuration
     </div>
   </div>

4. Navigate to ICSS-EMAC module and change the "Phy Address 0" and "Phy Address 1" correspondingly as shown below:

   \imageStyle{ICSS_EMAC_LWIP_RevE2_config.PNG,width:30%}
   \image html ICSS_EMAC_LWIP_RevE2_config.PNG ICSS-EMAC Configuration

5. Save the configurations.

\note For convenience, the example directory includes two pre-configured syscfg files:
- `example.syscfg`: Default configuration for RevA boards.
- `rev_e2_example.syscfg`: Ready-to-use configuration for RevE2 boards.

- To use the RevE2 configuration, you can either:
  - Make the manual changes described above to your existing syscfg file.
  - Rename `rev_e2_example.syscfg` to `example.syscfg` to replace the default configuration.

Refer to the README.txt file in the example directory for more details.

## Hardware Differences

Key differences between RevE2 and RevA are the following:
- RevA has DP83869 on-board PHY and RevE2 has DP83826E add-on PHY. 
- RevA needs an additional muxing to route ICSSM MDIO and RevE2 has ICSSM MDIO routed by default. 

\note Ensure you have selected the correct board revision before building the example to avoid communication issues.

\endcond
# Supported Combinations

\cond SOC_AM261X

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0_freertos
 Toolchain      | ti-arm-clang
 Boards         | @VAR_LP_BOARD_NAME_LOWER
 Example folder | examples/networking/lwip/icss_emac_lwip

\endcond

\cond SOC_AM263X

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0_freertos
 Toolchain      | ti-arm-clang
 Boards         | @VAR_BOARD_NAME_LOWER, @VAR_LP_BOARD_NAME_LOWER
 Example folder | examples/networking/lwip/icss_emac_lwip

\endcond

\cond SOC_AM263PX

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0_freertos
 Toolchain      | ti-arm-clang
 Boards         | @VAR_BOARD_NAME_LOWER, @VAR_LP_BOARD_NAME_LOWER
 Example folder | examples/networking/lwip/icss_emac_lwip

\endcond
# Configuring Syscfg

- The following Syscfg parameters in the ICSS-EMAC instance should be changed to swap between Switch and EMAC examples.

<table>
<tr>
    <th>Parameter
    <th>Status/Value for MAC
    <th>Status/Value for Switch
    <th>Description
    <th>Remarks/Default Setting
</tr>
<tr>
    <td>EMAC Mode
    <td>MAC1/MAC2/MAC1 of DUAL EMAC/MAC2 of DUAL EMAC
    <td>Switch
    <td>Parameter to choose between EMAC and Switch modes.
    <td>Default is Switch.
</tr>
<tr>
    <td>RT/NRT Priority Separation Queue
    <td>QUEUE1
    <td>QUEUE4
    <td>If packets are in Queue <= RT/NRT Priority separation queue, they will be forwarded to RT callback and others to NRT callback.
    <td>Default is QUEUE4.
</tr>
<tr>
    <td>Learning Enable
    <td>DISABLED
    <td>ENABLED
    <td>Learning only applicable for Switch
    <td>Default is ENABLED.
</tr>
</table>

# Iperf using LWIP

- Iperf is a tool for network performance measurement and tuning. It is a cross-platform tool that can produce standardized performance measurements for any network. Iperf has client and server functionality, and can create data streams to measure the throughput between the two ends in one or both directions.

- Iperf version to be used is version 2.0.9-win64(on windows) and version 2.+ on linux(Ubuntu 18.04 64bit).

- The lwip app has iperf enabled by default.

## Important Iperf Arguments:

 Argument      | Meaning
 ---------------|-----------
 -s             | Run server
 -c             | Run Client [Ex: -c 192.168.1.102]
 -u             | UDP
 -b             | Bandwidth [Used in UDP, Ex: 100M M->Mbits]
 -i             | Output interval in Sec [Ex: -i1 1sec interval]
 -t             | Time in sec [Ex: -t60 60sec]
 -p             | Port number [Ex: -p 5555]
 -w             | Windows size [Ex: -w 1M M->Mbits]
 -d             | Bi-directional traffic
 -l             | Length [Ex: -l 1046 1046bytes]
 -V             | Used when IPv6 address is used instead of IPv4


# Steps to Run the Example

## Build the example

- When using CCS projects to build, import the CCS project for the required combination
  and build it using the CCS project menu (see \ref CCS_PROJECTS_PAGE).
- When using makefiles to build, note the required combination and build using
  make command (see \ref MAKEFILE_BUILD_PAGE)

## HW Setup

\note Make sure you have setup the EVM with cable connections as shown here, \ref EVM_SETUP_PAGE.
      In addition do below steps.

## Create a network between EVM and host PC

- The EVM will get an IP address of 192.168.0.200. Make sure to connect the other end of the cable
to a network which is in the same sub-network.

- To get started, create a simple local network between the EVM and the host PC as shown below.

  \imageStyle{icss_emac_lwip_example_01.png,width:30%}
  \image html icss_emac_lwip_example_01.png Local network between PC and EVM

- After we run the example on the EVM (in the next step), it will be assigned an IP address using which the host
  can communicate with the EVM.

## Run the example

\attention If you need to reload and run again, a CPU power-cycle is MUST

- Launch a CCS debug session and run the example executable, see \ref CCS_LAUNCH_PAGE
- You will see logs in the UART terminal as shown in the next section.
- Note the IP address seen in the log, this is what we will use to communicate with the EVM.

## Sample output for ICSS-EMAC LWIP example

\cond SOC_AM263X || SOC_AM263PX || SOC_AM261X

\code

MII mode
Firmware load to PRU0 passed
Firmware load to PRU1 passed
Starting lwIP, local interface IP is 192.168.0.200
[LWIPIF_LWIP_EMAC]Link is down[LWIPIF_LWIP_EMAC] Interface layer handle is Initialised 
[LWIPIF_LWIP_EMAC] NETIF INIT SUCCESS
status_callback==UP, local interface IP is 192.168.0.200
UDP server listening on port 5001
link_callback==UP
      5.653s : CPU load =   1.39 %
     10.653s : CPU load =   0.56 %

\endcode

\endcond

## Communicate with the EVM using ethernet

- Firstly you can try to reach the EVM using ping as shown below, using a command shell on the host PC

        > ping 192.168.0.200

- Next you can run `iperf` tests as shown below. Below steps have been tried with a Windows 10 host PC running.

 - Install `iperf` if not installed by doing below

        > sudo apt install iperf

 - Invoke iperf to test TCP bi-directional RX+TX connection as shown below
    \code
    > iperf -c 192.168.0.200 -i 5 -t 20 -d
    \endcode

## Measuring the throughput using Iperf:

- Once we get the IP after running the example, we can use following iperf command on windows to get the throughput.
	- iperf.exe -c 192.168.0.200 -r

## Sample output for iperf command

DUT side:

\code
MII mode
Firmware load to PRU0 passed
Firmware load to PRU1 passed
Starting lwIP, local interface IP is 192.168.0.200
[LWIPIF_LWIP_EMAC]Link is down[LWIPIF_LWIP_EMAC] Interface layer handle is Initialised 
[LWIPIF_LWIP_EMAC] NETIF INIT SUCCESS
status_callback==UP, local interface IP is 192.168.0.200
UDP server listening on port 5001
link_callback==UP
      5.653s : CPU load =   1.39 %
     10.653s : CPU load =   0.56 %
     15.653s : CPU load =   0.56 %
     20.653s : CPU load =   0.53 %
     25.653s : CPU load =   0.53 %
     30.653s : CPU load =   4.77 %
     35.653s : CPU load =  90.97 %
IPERF report: type=0, remote: 192.168.0.33:53365, total bytes: 86114328, duration in ms: 10056, kbits/s: 68504
                             40.653s : CPU load =  90.87 %
     45.653s : CPU load =  44.60 %
IPERF report: type=1, remote: 192.168.0.33:5001, total bytes: 55255064, duration in ms: 10000, kbits/s: 44200
                            50.653s : CPU load =  43.35 %
     55.653s : CPU load =   0.54 % 

\endcode

PC Side:

\code 
C:\ti\iperf-2.0.9-win64>iperf -c 192.168.0.200 -r
------------------------------------------------------------
Server listening on TCP port 5001
TCP window size:  208 KByte (default)
------------------------------------------------------------
------------------------------------------------------------
Client connecting to 192.168.0.200, TCP port 5001
TCP window size:  208 KByte (default)
------------------------------------------------------------
[  4] local 192.168.0.33 port 53365 connected with 192.168.0.200 port 5001
[ ID] Interval       Transfer     Bandwidth
[  4]  0.0-10.0 sec  82.1 MBytes  68.8 Mbits/sec
[  4] local 192.168.0.33 port 5001 connected with 192.168.0.200 port 54911
[  4]  0.0-10.0 sec  52.7 MBytes  44.2 Mbits/sec 

\endcode

## Troubleshooting issues

- Refer to \ref ICSS_EMAC_DEBUG_GUIDE for troubleshooting issues.

# See Also
\cond SOC_AM263X || SOC_AM263PX || SOC_AM261X
\ref NETWORKING
\endcond
