# Enet Lwip CPSW Example{#EXAMPLES_ENET_LWIP_CPSW}

[TOC]

# Introduction

\note lwIP features are made available as is from public lwIP project. SDK configuration may only enable and exercise a subset of these features.

This example is a TCP/UDP IP application using the LwIP networking stack, coupled with
ethernet driver (ENET)

On @VAR_SOC_NAME, we can do ethernet based communication using CPSW as HW mechanism
  - CPSW is a standard ethernet switch + port HW
  - It uses ethernet driver underneath with LwIP TCP/IP networking stack
  - CPSW can be configured in two modes: Switch or MAC. For more details, \ref ENET_LWIP_CPSW_OPERATING_MODES

The examples do below
- Initializes the ethernet driver for the underlying HW
- Initializes the LwIP stack for TCP/UDP IP
- Allows user to run and test basic networking features like DHCP, ping, iperf with TCP/UDP.
\cond SOC_AM273X || SOC_AWR294X 
- Example is configured to run in Switch mode.
\endcond
\cond SOC_AM64X || SOC_AM243X || SOC_AM263X
- Example is configured to run in Dual MAC mode.
\endcond

\cond SOC_AM263X
Note: In this example, Different Priority Packets are received using a single channel by enabling the default thread Id (for this enChOverrideFlag is set in dmacfg), which allows packets with no classifer match to be received by the host.
\endcond
# Supported Combinations

\cond SOC_AM64X

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0_freertos r5fss0-1_freertos
 Toolchain      | ti-arm-clang
 Board          | @VAR_BOARD_NAME_LOWER
 Example folder | examples/networking/lwip/enet_lwip_cpsw

\endcond

\cond SOC_AM243X

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0_freertos r5fss0-1_freertos
 Toolchain      | ti-arm-clang
 Boards         | @VAR_BOARD_NAME_LOWER, @VAR_LP_BOARD_NAME_LOWER
 Example folder | examples/networking/lwip/enet_lwip_cpsw

\endcond

\cond SOC_AM273X

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0_freertos r5fss0-1_freertos
 Toolchain      | ti-arm-clang
 Boards         | @VAR_BOARD_NAME_LOWER
 Example folder | examples/networking/lwip/enet_lwip_cpsw

\endcond

\cond SOC_AWR294X

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0_freertos r5fss0-1_freertos
 Toolchain      | ti-arm-clang
 Boards         | @VAR_BOARD_NAME_LOWER
 Example folder | examples/networking/lwip/enet_lwip_cpsw

\endcond

\cond SOC_AM263X

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0_freertos r5fss0-1_freertos
 Toolchain      | ti-arm-clang
 Boards         | @VAR_BOARD_NAME_LOWER, @VAR_LP_BOARD_NAME_LOWER
 Example folder | examples/networking/lwip/enet_lwip_cpsw

\endcond

# Configuring Syscfg

- Following Syscfg option allows flexibility to configure memory foot print based on required use case like: Number of DMA descriptors and buffering.

- Supported Options with default configuration

<table>
<tr>
    <th>Feature
    <th>Section
    <th>Description
    <th>Remarks/Default Setting
</tr>

<tr>
    <td>Mdio Manual Mode Enable
    <td>TI Networking / Enet (CPSW)
    <td>Flag to enable MDIO manual mode in example. Driver support for Manual mode is enabled, so this parameter configures manual mode in the example.
    <td>Default is true. If your silicon is affected with errata <a href="https://www.ti.com/lit/er/sprz457e/sprz457e.pdf" target="_blank">i2329— MDIO interface corruption</a>, then TI suggests to use MDIO_MANUAL_MODE as software workaround.
</tr>

\cond SOC_AM64X || SOC_AM243X || SOC_AM263X
<tr>
    <td>Disable Mac Port1, Disable Mac Port2
    <td>TI Networking / Enet (CPSW)
    <td>Select which port to use.
    <td>Default is Port1 enabled. If both Port1 and Port 2 are ticked any port can be used and it als enables traffic switching between the two ports.
</tr>
\endcond

<tr>
    <td>Enable Packet Pool Allocation
    <td>TI Networking / Enet (CPSW)
    <td>Flag to enable packet allocation from enet utils library. It should be disabled to avoid utils memory wastage, in case application allots packet via other mechanism. (Ex- Lwip pools)
    <td>Default is true. It is disabled for lwip based examples. If enabled size of pkt pool size depends on 'Large Pool Packet Size', 'Large Pool Packet Count', 'Medium Pool Packet Size', 'Medium Pool Packet Count', 'Small Pool Packet Size' and 'Small Pool Packet Count'.
</tr>

<tr>
    <td>Number of Tx Packet
    <td>TI Networking / Enet (CPSW) / DMA channel config
    <td>No of Tx packets required for DMA channel
    <td>Default is 16. It contributes to the size of Pkt Mem Pool, DMA ring buffer and accessories.
</tr>

<tr>
    <td>Number of Rx Packet
    <td>TI Networking / Enet (CPSW) / DMA channel config
    <td>No of Rx packets required for DMA channel
    <td>Default is 32. It contributes to the size of Pkt Mem Pool, DMA ring buffer and accessories size.
</tr>

<tr>
    <td>Netif instance
    <td>TI Networking / Enet (CPSW) / LWIP Interface config
    <td>No of netifs allocated by the example
    <td>Only one netif should be set to default when more than one netif is allocated.
</tr>
</table>

# Iperf using LWIP

- Iperf is a tool for network performance measurement and tuning. It is a cross-platform tool that can produce standardized performance measurements for any network. Iperf has client and server functionality, and can create data streams to measure the throughput between the two ends in one or both directions.

- Iperf version to be used is version 2.0.9-win64(on windows) and version 2.+ on linux(Ubuntu 18.04 64bit).

- The lwip app has iperf enabled by default. To disable the iperf functionality, disable the macro (LWIP_LWIPERF_APP) in the application file lwipcfg.h under path (examples/networking/lwip/enet_lwip_cpsw).

- The data streams can be either Transmission Control Protocol (TCP) or User Datagram Protocol (UDP).

 - UDP: When used for testing UDP capacity, iperf allows the user to specify the datagram size and provides results for the datagram throughput and the packet loss.
 - TCP: When used for testing TCP capacity, iperf measures the throughput of the payload. Iperf uses 1024 × 1024 for mebibytes and 1000 × 1000 for megabytes.

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

\cond SOC_AM64X

### AM64X-EVM

#### For CPSW based example

- Connect a ethernet cable to the EVM from host PC as shown below

  \imageStyle{am64x_evm_lwip_example_00.png,width:30%}
  \image html am64x_evm_lwip_example_00.png Ethernet cable for CPSW based ethernet

\endcond

\cond SOC_AM243X

### AM243X-EVM

#### For CPSW based example

- Connect a ethernet cable to the EVM from host PC as shown below

  \imageStyle{am64x_evm_lwip_example_00.png,width:30%}
  \image html am64x_evm_lwip_example_00.png Ethernet cable for CPSW based ethernet

### AM243X-LP

\note AM243X-LP has two ethernet Ports which can be configured as both CPSW ports.

#### For CPSW based examples

- Connect a ethernet cable to the AM243X-LP from host PC as shown below

  \imageStyle{am243x_lp_lwip_example_00.png,width:30%}
  \image html am243x_lp_lwip_example_00.png Ethernet cable for CPSW based ethernet

\endcond

## Create a network between EVM and host PC

- The EVM will get an IP address using DHCP, so make sure to connect the other end of the cable
to a network which has a DHCP server running.

- To get started one can create a simple local network
  between the EVM and the host PC by using a home broadband/wifi router as shown below.
  Most such routers run a DHCP server

  \imageStyle{lwip_example_01.png,width:30%}
  \image html lwip_example_01.png Local network between PC and EVM

- To check the router connection with host PC, recommend to disconnect all other networking conenctions
  on the PC, sometimes you may need to disable firewall SW, and make sure the router is able
  to assign a IP address to your host PC

- After we run the example on the EVM (next step), the EVM will similarly be assigned a IP address, and then host
  can communicate with the EVM using the assigned IP address.

## Run the example

\attention If you need to reload and run again, a CPU power-cycle is MUST

- Launch a CCS debug session and run the example executable, see \ref CCS_LAUNCH_PAGE
- You will see logs in the UART terminal as shown in the next section.
- Note the IP address seen in the log, this is what we will use to communicate with the EVM.

## Sample output for CPSW example

\cond SOC_AM64X || SOC_AM243X

\code


==========================
      ENET LWIP App
==========================
Enabling clocks!
EnetPhy_bindDriver:
EnetPhy_bindDriver:
PHY 0 is alive
PHY 3 is alive
Starting lwIP, local interface IP is dhcp-enabled
Host MAC address: ac:1f:0f:84:0c:33
[LWIPIF_LWIP] Enet has been started successfully
[LWIPIF_LWIP] NETIF INIT SUCCESS
status_callback==UP, local interface IP is 0.0.0.0
UDP server listening on port 5001
Cpsw_handleLinkUp:
Cpsw_handleLinkUp:
MAC Port 1: link up
MAC Port 2: link up
link_callback==UP
status_callback==UP, local interface IP is 192.168.0.175
      5. 41s : CPU load =   1.95 %

\endcode

\endcond

\cond SOC_AM273X || SOC_AWR294X || SOC_AM263X

\code

==========================
      ENET LWIP App
==========================
EnetPhy_bindDriver:
PHY 0 is alive
Starting lwIP, local interface IP is dhcp-enabled
Host MAC address: 70:ff:76:1d:ec:f2
[LWIPIF_LWIP] Enet has been started successfully
[LWIPIF_LWIP] NETIF INIT SUCCESS
status_callback==UP, local interface IP is 0.0.0.0
UDP server listening on port 5001
Cpsw_handleLinkUp:
MAC Port 1: link up
link_callback==UP
      5. 34s : CPU load =   2.04 %


\endcode

\endcond

\cond SOC_AM243X
- Note: For AM243LP we have external Phy Management enabled in syscfg and the application log appears as:

\code

==========================
      ENET LWIP App
==========================
EnetMod_ioctl:1305
Mdio_ioctl:581
EnetMod_ioctl:1305
Mdio_ioctl:581
EnetMod_ioctl:1305
Mdio_ioctl:581
EnetMod_ioctl:1305
Mdio_ioctl:581
EnetMod_ioctl:1305
Mdio_ioctl:581
EnetMod_ioctl:1305
Mdio_ioctl:581
EnetMod_ioctl:1305
Cpsw_handleExternalPhyLinkUp:2588
Cpsw_handleLinkDown:1995
Cpsw_handleExternalPhyLinkUp:2588
Starting lwIP, local interface IP is dhcp-enabled
Host MAC address: f4:84:4c:f9:8c:50
[LWIPIF_LWIP] Enet has been started successfully
[LWIPIF_LWIP] NETIF INIT SUCCESS
status_callback==UP, local interface IP is 0.0.0.0
UDP server listening on port 5001
link_callback==UP
status_callback==UP, local interface IP is 192.168.1.10
      6.859s : CPU load =   0.08 %
     11.859s : CPU load =   1.22 %

\endcode
\endcond
## Communicate with the EVM using ethernet

- Firstly you can try to reach the EVM using ping as shown below, using a command shell on the host PC

        > ping 192.168.1.100

- Next you can run `iperf` tests as shown below. Below steps have been tried with a Linux Ubuntu 18.04 host PC running bash shell

 - Install `iperf` if not installed by doing below

        > sudo apt install iperf

 - Invoke iperf to test TCP bi-directional RX+TX connection as shown below
    \code
    > iperf -c 192.168.1.100 -i 5 -t 20 -d
    \endcode

## Measuring the throughput using Iperf:

- Once we get the ip after running the example, we can use following iperf command on windows to get the throughput.
	- iperf.exe -c 192.168.1.200 -r

## Sample output for iperf command

\code

DUT side:
==========================
      ENET LWIP App
==========================
Enabling clocks!
EnetPhy_bindDriver:
EnetPhy_bindDriver:
PHY 0 is alive
PHY 3 is alive
Starting lwIP, local interface IP is dhcp-enabled
Host MAC address: ac:1f:0f:84:0c:33
[LWIPIF_LWIP] Enet has been started successfully
[LWIPIF_LWIP] NETIF INIT SUCCESS
status_callback==UP, local interface IP is 0.0.0.0
UDP server listening on port 5001
Cpsw_handleLinkUp:
Cpsw_handleLinkUp:
MAC Port 1: link up
MAC Port 2: link up
link_callback==UP
status_callback==UP, local interface IP is 192.168.0.175
5. 41s : CPU load =   1.95 %
10. 41s : CPU load =   1.34 %
15. 41s : CPU load =  39.57 %
20. 41s : CPU load =  45.32 %
IPERF report: type=0, remote: 192.168.0.107:33748, total bytes: 116785176, duration in ms: 10021, kbits/s: 93232
                              25. 41s : CPU load =  54.81 %
30. 41s : CPU load =  56.59 %
IPERF report: type=1, remote: 192.168.0.107:5001, total bytes: 116040824, duration in ms: 10000, kbits/s: 92832
                            35. 41s : CPU load =   8.91 %


PC Side:

..\iperf-2.0.9-win64>iperf.exe -c 192.168.1.175 -r
------------------------------------------------------------
Server listening on TCP port 5001
TCP window size: 208 KByte (default)
------------------------------------------------------------
------------------------------------------------------------
Client connecting to 192.168.1.200, TCP port 5001
TCP window size: 208 KByte (default)
------------------------------------------------------------
[ 4] local 192.168.1.4 port 55209 connected with 192.168.1.200 port 5001
[ ID] Interval Transfer Bandwidth
[ 4] 0.0-10.0 sec 110 MBytes 93.4 Mbits/sec
[ 4] local 192.168.1.4 port 5001 connected with 192.168.1.200 port 54911
[ 4] 0.0-10.0 sec 18.0 MBytes 92.8 Mbits/sec




\endcode

## Troubleshooting issues

\cond SOC_AM64X || SOC_AM243X
- If you see MAC address as `00:00:00:00:00:00`, likely you are using a very early Si sample which does not
  have MAC address "fused" in, in this case do below steps

   - Open file `source\networking\enet\soc\j7x\am64x_am243x\enet_soc.c`
   - Uncomment below line
        \code
        #define ENET_MAC_ADDR_HACK (TRUE)
        \endcode
   - Rebuild the libraries and examples (\ref MAKEFILE_BUILD_PAGE)
\endcond

- If you see a valid, non-zero MAC address but IP address is `0.0.0.0` then
   - Make sure you see `link_callback==UP` message, if not check the ethernet cable
   - Check the local network and check if the DHCP server is indeed running as expected
   - When using a home broadband/wifi router, its possible to check the clients connected to the DHCP server via a web
     browser. Check your router user manual for more details.

# See Also
\cond SOC_AM64X || SOC_AM243X
\ref NETWORKING
\endcond
