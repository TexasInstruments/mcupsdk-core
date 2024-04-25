# ICSS-EMAC Lwip Example {#EXAMPLES_ICSS_EMAC_LWIP}

[TOC]

# Introduction

\note lwIP features are made available as is from public lwIP project. SDK configuration may only enable and exercise a subset of these features.

This example is ICSS-EMAC application using the LwIP networking stack.

The examples do below
- Initializes the ICSS-EMAC driver for the underlying HW
- Initializes the LwIP stack
- Allows user to run and test basic networking features like ping, iperf with TCP/UDP.
- Allows user to configure between ICSS_EMAC Switch and MAC modes.
\cond SOC_AM263PX || SOC_AM263X
- Example is configured to run in Switch mode.
\endcond

# Supported Combinations

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

- The following Syscfg parameters in ICSS-EMAC instance is to be changed to swap between Switch and EMAC examples.

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
    <td>If packets are in Queue <= RT/NRT Priority seperation queue, they will be forwarded to RT callback and others to NRT callback.
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
to a network which is in the same sub-network..

- To get started, create a simple local network
  between the EVM and the host PC as shown below.

  \imageStyle{icss_emac_lwip_example_01.png,width:30%}
  \image html icss_emac_lwip_example_01.png Local network between PC and EVM

- After we run the example on the EVM (next step), the EVM will be assigned the IP address, and then host
  can communicate with the EVM using the assigned IP address.

## Run the example

\attention If you need to reload and run again, a CPU power-cycle is MUST

- Launch a CCS debug session and run the example executable, see \ref CCS_LAUNCH_PAGE
- You will see logs in the UART terminal as shown in the next section.
- Note the IP address seen in the log, this is what we will use to communicate with the EVM.

## Sample output for ICSS-EMAC LWIP example

\cond SOC_AM263X || SOC_AM263PX

\code

MII mode
load to PRU0 passed
load to PRU1 passed
Starting lwIP, local interface IP is 192.168.0.200
[LWIPIF_LWIP]Link is down[LWIPIF_LWIP] Interface layer handle is Initialised
[LWIPIF_LWIP] NETIF INIT SUCCESS
status_callback==UP, local interface IP is 192.168.0.200
UDP server listening on port 5001
link_callback==UP
      6. 39s : CPU load =   0.82 %

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

- Once we get the ip after running the example, we can use following iperf command on windows to get the throughput.
	- iperf.exe -c 192.168.0.200 -r

## Sample output for iperf command

\code

DUT side:
load to PRU0 passed
load to PRU1 passed
MII mode
Starting lwIP, local interface IP is 192.168.0.200
[LWIPIF_LWIP]Link is down[LWIPIF_LWIP] Interface layer handle is Initialised
[LWIPIF_LWIP] NETIF INIT SUCCESS
status_callback==UP, local interface IP is 192.168.0.200
UDP server listening on port 5001
link_callback==UP
      6. 33s : CPU load =   0.68 %
     11. 33s : CPU load =   0.58 %
     16. 33s : CPU load =   0.55 %
     21. 33s : CPU load =   0.54 %
     26. 33s : CPU load =   0.53 %
     31. 33s : CPU load =   0.54 %
     36. 33s : CPU load =  44.89 %
     41. 33s : CPU load =  82.92 %
IPERF report: type=0, remote: 192.168.0.50:51717, total bytes: 67502104, duration in ms: 10059, kbits/s: 53680
                                                                                                                   46. 33s : CPU load =  36.46 %
     51. 33s : CPU load =   0.54 %
IPERF report: type=2, remote: 192.168.0.50:5001, total bytes: 3096, duration in ms: 9679, kbits/s: 0
                                                                                                         56. 33s : CPU load =   0.55 %
     61. 33s : CPU load =   0.54 %


PC Side:

..\iperf-2.0.9-win64\iperf-2.0.9-win64>iperf.exe -c 192.168.0.200 -r
------------------------------------------------------------
Server listening on TCP port 5001
TCP window size:  208 KByte (default)
------------------------------------------------------------
------------------------------------------------------------
Client connecting to 192.168.0.200, TCP port 5001
TCP window size:  208 KByte (default)
------------------------------------------------------------
[  4] local 192.168.0.50 port 51717 connected with 192.168.0.200 port 5001
[ ID] Interval       Transfer     Bandwidth
[  4]  0.0-10.0 sec  64.4 MBytes  53.8 Mbits/sec
[  4] local 192.168.0.50 port 5001 connected with 192.168.0.200 port 54911
[  4]  0.0-20.2 sec  1.02 KBytes   416 bits/sec


\endcode

## Troubleshooting issues

- Refer to \ref ICSS_EMAC_DEBUG_GUIDE for troubleshooting issues.

# See Also
\cond SOC_AM263X || SOC_AM263PX
\ref NETWORKING
\endcond
