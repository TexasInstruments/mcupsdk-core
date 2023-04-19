# Enet Lwip UDP IGMP Server Example{#EXAMPLES_ENET_LWIP_CPSW_UDP_IGMP}

[TOC]

# Introduction

\note LwIP features are made available as is from public lwIP project. SDK configuration may only enable and exercise a subset of these features.

This example shows about how to implement a simple UDP IGMP-Server on LwIP networking stack using BSD-Socket API coupled with ethernet driver (ENET)

On @VAR_SOC_NAME, we can do ethernet based communication using CPSW as HW mechanism
  - CPSW is a standard ethernet switch + port HW
  - It uses ethernet driver underneath with LwIP TCP/IP networking stack
  - CPSW can be configured in two modes: Switch or MAC. For more details, \ref ENET_LWIP_CPSW_OPERATING_MODES

The example does below
- Initializes the ethernet driver for the underlying HW
- Initializes the LwIP stack for TCP/UDP IP and Starts UDP (echo) IGMP Server task.
- UDP Server task waits for a msg to the multicast IP address from client on port 2638 and echoes back the same message.

\cond SOC_AM263X
NOTE: DSCP priority mapping is configured in the example but for the host port to recieve different priority pkts on the same dma channel, user needs to enable channel override (enChOverrideFlag) flag in dmacfg. Refer enet_lwip_cpsw example.
\endcond

# Supported Combinations

\cond SOC_AM64X

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0_freertos
 Toolchain      | ti-arm-clang
 Board          | @VAR_BOARD_NAME_LOWER
 Example folder | examples/networking/lwip/enet_cpsw_udp_igmp

\endcond

\cond SOC_AM243X

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0_freertos
 Toolchain      | ti-arm-clang
 Boards         | @VAR_BOARD_NAME_LOWER, @VAR_LP_BOARD_NAME_LOWER
 Example folder | examples/networking/lwip/enet_cpsw_udp_igmp

\endcond

\cond SOC_AM273X

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0_freertos
 Toolchain      | ti-arm-clang
 Boards         | @VAR_BOARD_NAME_LOWER
 Example folder | examples/networking/lwip/enet_cpsw_udp_igmp

\endcond

\cond SOC_AWR294X

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0_freertos
 Toolchain      | ti-arm-clang
 Boards         | @VAR_BOARD_NAME_LOWER
 Example folder | examples/networking/lwip/enet_cpsw_udp_igmp

\endcond

\cond SOC_AM263X

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0_freertos
 Toolchain      | ti-arm-clang
 Boards         | @VAR_BOARD_NAME_LOWER, @VAR_LP_BOARD_NAME_LOWER
 Example folder | examples/networking/lwip/enet_cpsw_udp_igmp

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

# UDP Client using ncat tool

Ncat is a general-purpose command-line tool for reading, writing, redirecting, and encrypting data across a network. It aims to be your network Swiss Army knife, handling a wide variety of security testing and administration tasks. Ncat is suitable for interactive use or as a network-connected back end for other tools.
 - Ncat is started as server to which EVM connects.
 - Version used for this example Version 7+ ( https://nmap.org/ncat )

# Steps to Run the Example

## Build the example

- When using CCS projects to build, import the CCS project for the required combination
  and build it using the CCS project menu (see \ref CCS_PROJECTS_PAGE).
- When using makefiles to build, note the required combination and build using
  make command (see \ref MAKEFILE_BUILD_PAGE)

Note: For UDP transmission on LwIP based application, application should perform cache coherency operation on payload before submitting the packet for transmission. This is because zero copy is enabled for UDP packet transmission and cache operation in driver are disabled for UDP payload portions. This example application does this using CacheP_wbInv.
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

\code

============================
  CPSW LWIP IGMP UDP SERVER 
============================
EnetPhy_bindDriver:1718 
PHY 3 is alive
PHY 12 is alive
Starting lwIP, local interface IP is dhcp-enabled
Host MAC address: 34:08:e1:77:fb:0c
[LWIPIF_LWIP] Enet has been started successfully
[LWIPIF_LWIP] NETIF INIT SUCCESS
Enet IF UP Event. Local interface IP:0.0.0.0
Waiting for network UP ...
Cpsw_handleLinkUp:1320 
MAC Port 1: link up
Network Link UP Event
Waiting for network UP ...
Enet IF UP Event. Local interface IP:10.24.69.14
Network is UP ...
UDP server: Port 2638
UDP server: joined Multicast group 
     10.107s : CPU load =   2.25 %
     15.108s : CPU load =   1.70 %
     20.109s : CPU load =   1.82 %
     25.110s : CPU load =   1.79 %
     30.111s : CPU load =   1.75 %
     35.112s : CPU load =   1.72 %
     40.113s : CPU load =   1.71 %
     45.114s : CPU load =   1.68 %
     50.115s : CPU load =   1.71 %
     55.116s : CPU load =   1.66 %
     60.117s : CPU load =   1.65 %
     65.118s : CPU load =   1.70 %
     70.119s : CPU load =   1.65 %
     75.120s : CPU load =   1.71 %
     80.121s : CPU load =   1.74 %
     85.122s : CPU load =   1.73 %
     90.123s : CPU load =   1.69 %
     95.124s : CPU load =   1.66 %
    100.125s : CPU load =   1.70 %
    105.126s : CPU load =   1.67 %
    110.127s : CPU load =   1.69 %
    115.128s : CPU load =   1.70 %
    120.129s : CPU load =   1.70 %
    125.130s : CPU load =   1.75 %
    130.131s : CPU load =   1.72 %
    135.132s : CPU load =   2.07 %
    140.133s : CPU load =   1.77 %
    145.134s : CPU load =   1.82 %
    150.135s : CPU load =   1.70 %
    155.136s : CPU load =   1.73 %
    160.137s : CPU load =   1.76 %
    165.138s : CPU load =   1.72 %
    170.139s : CPU load =   1.70 %
    175.140s : CPU load =   1.75 %
    180.141s : CPU load =   1.82 %
    185.142s : CPU load =   1.81 %
    190.143s : CPU load =   1.72 %
    195.144s : CPU load =   1.70 %
    200.145s : CPU load =   1.71 %
    205.146s : CPU load =   1.70 %
    210.147s : CPU load =   1.72 %
    215.148s : CPU load =   1.68 %
    220.149s : CPU load =   1.76 %
    225.150s : CPU load =   1.73 %
    230.151s : CPU load =   1.73 %
    235.152s : CPU load =   1.69 %
    240.153s : CPU load =   1.69 %
    245.154s : CPU load =   1.70 %
    250.155s : CPU load =   1.83 %
    255.156s : CPU load =   1.77 %
    260.157s : CPU load =   1.82 %
    265.158s : CPU load =   1.72 %
    270.159s : CPU load =   1.69 %
    275.160s : CPU load =   1.73 %
    280.161s : CPU load =   1.76 %
    285.162s : CPU load =   1.68 %
    290.163s : CPU load =   1.75 %
    295.164s : CPU load =   1.69 %
    300.165s : CPU load =   1.70 %
    305.166s : CPU load =   1.71 %
Packet recieved 
Client: Hi

Echo pkt completed
    310.167s : CPU load =   1.73 %
    315.168s : CPU load =   1.72 %
    320.169s : CPU load =   1.69 %
    325.170s : CPU load =   1.67 %
    330.171s : CPU load =   1.68 %
    335.172s : CPU load =   1.68 %
    340.173s : CPU load =   1.84 %
    345.174s : CPU load =   1.76 %
    350.175s : CPU load =   1.71 %
    355.176s : CPU load =   1.70 %
    360.177s : CPU load =   1.68 %
    365.178s : CPU load =   1.68 %
    370.179s : CPU load =   1.67 %
    375.180s : CPU load =   1.68 %
    380.181s : CPU load =   1.70 %
    385.182s : CPU load =   1.70 %
    390.183s : CPU load =   1.66 %
    395.184s : CPU load =   1.72 %
    400.185s : CPU load =   1.71 %
    405.186s : CPU load =   1.69 %
    410.187s : CPU load =   1.70 %
    415.188s : CPU load =   1.66 %
    420.189s : CPU load =   1.68 %
    425.190s : CPU load =   1.68 %
Packet recieved 
Client: 
Echo pkt completed
    430.191s : CPU load =   1.69 %
    435.192s : CPU load =   1.68 %
    440.193s : CPU load =   1.69 %
    445.194s : CPU load =   1.69 %
    450.195s : CPU load =   1.68 %

\endcode

## Steps to execute

1. Run example on EVM

2. Try to reach the EVM using ping as shown below, using a command shell on the host PC
    \code
    $ping 192.168.1.10
    \endcode
    "192.168.1.10" should be replaced with IP of EVM.

3. Start UDP client using 'ncat' cmds as shown below. Below steps have been tried with a Linux Ubuntu 18.04 host PC running bash shell

   Install 'ncat' if not installed by doing below
    \code
    $sudo apt install ncat
    \endcode

   Invoke 'ncat' to connect to Multicast IP '224.0.1.129' on port 2638
    \code
    $ncat -u 224.0.1.129 2638 
    $
    \endcode

5. Send any msg in ncat terminal and observe that it is recieved in EVM

6. Close the ncat using Ctrl + C.

## Troubleshooting issues

\cond SOC_AM64X || SOC_AM243X
- If you see MAC address as `00:00:00:00:00:00`, likely you are using a very early Si sample which does not
  have MAC address "fused" in, in this case do below steps

   - Open file `source/networking/.meta/enet_cpsw/templates/am64x_am243x/enet_soc_cfg.c.xdt`
   - Uncomment below line
        \code
        #define ENET_MAC_ADDR_HACK (TRUE)
        \endcode
   - Rebuild the libraries and examples (\ref MAKEFILE_BUILD_PAGE)
\endcond

- If you see a valid, non-zero MAC address and continuosly seieing "Waiting for network UP..." prints in UART terminal
   - Make sure you see `Enet IF UP Event.` message, if not check the ethernet cable
   - Check the local network and check if the DHCP server is indeed running as expected
   - When using a home broadband/wifi router, its possible to check the clients connected to the DHCP server via a web
     browser. Check your router user manual for more details.

# See Also
\cond SOC_AM64X || SOC_AM243X
\ref NETWORKING
\endcond