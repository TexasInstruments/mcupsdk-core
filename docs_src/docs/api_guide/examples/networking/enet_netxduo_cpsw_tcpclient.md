# Enet NetxDuo TCP Client Example{#EXAMPLES_ENET_NETXDUO_CPSW_TCPCLIENT}

[TOC]

# Introduction

\note NetxDuo features are made available as is from public lwIP project. SDK configuration may only enable and exercise a subset of these features.

This example shows about how to implement simple TCP Client on NetxDuo networking stack using netconn interface coupled with ethernet driver (ENET)

On @VAR_SOC_NAME, we can do ethernet based communication using CPSW as HW mechanism
  - CPSW is a standard ethernet switch + port HW
  - It uses ethernet driver underneath with NetxDuo TCP/IP networking stack
  - CPSW can be configured in two modes: Switch or MAC. For more details, \ref ENET_LWIP_CPSW_OPERATING_MODES

The example does below
- Initializes the ethernet driver for the underlying HW
- Initializes the NetxDuo stack for TCP/UDP IP and Starts TCP Client task.
- TCP Client gets server IP using UART terminal menu from USER and connect to server IP on port 8888
- TCP client connects to server, sends data and expects the data from server.


# Supported Combinations


 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0_freertos
 Toolchain      | ti-arm-clang
 Boards         | @VAR_BOARD_NAME_LOWER, @VAR_LP_BOARD_NAME_LOWER
 Example folder | source/networking/enet/core/examples/netxduo/enet_netxduo_cpsw_tcpclient

Note: To run the example on any core other than r5fss0-0, user needs to change the DMA channel resource ownership accordingly using the resource partioning tool in \ref RESOURCE_ALLOCATION_GUIDE and build the new SBL.

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

<tr>
    <td>Disable Mac Port1, Disable Mac Port2
    <td>TI Networking / Enet (CPSW)
    <td>Select which port to disable.
    <td>Default is Port1 enabled. If both Port1 and Port 2 are enabled, any port can be used and  if operating in switch mode, it enables traffic switching between the two ports.
</tr>

<tr>
    <td>Enable Packet Pool Allocation
    <td>TI Networking / Enet (CPSW)
    <td>Flag to enable packet buffer memory allocation from enet utils library. It should be disabled to avoid utils memory wastage, in case application allots packet via other mechanism.
    <td>Default is true. If enabled size of pkt pool size depends on 'Large Pool Packet Size', 'Large Pool Packet Count', 'Medium Pool Packet Size', 'Medium Pool Packet Count', 'Small Pool Packet Size' and 'Small Pool Packet Count'. EnetMem_allocEthPkt API uses this memory to allocate the DMA Ethernet packet.
</tr>

<tr>
    <td>Only Enable Packet Info Allocation
    <td>TI Networking / Enet (CPSW)
    <td>Flag to allocate only the DMA Packet Info structures, this does not include the buffer memory. This is useful when the buffer memory is internally allocated by the application.
    <td>Default is true. If enabled "PktInfoMem Only Count" determines the number of additional DMA Packet Info structures allocated. EnetMem_allocEthPktInfoMem uses this memory to allocate empty DMA Packet Info structures.
</tr>

<tr>
    <td>Number of Tx Packet
    <td>TI Networking / Enet (CPSW) / DMA channel config
    <td>No of Tx packets required for DMA channel
    <td>Default is 16. The size of the NetxDuo packet pool will be automatically set at runtime to match the number of packets configured here.
</tr>

<tr>
    <td>Number of Rx Packet
    <td>TI Networking / Enet (CPSW) / DMA channel config
    <td>No of Rx packets required for DMA channel
    <td>Default is 32. It contributes to the size of Pkt Mem Pool, DMA ring buffer and accessories size. The size of the NetxDuo packet pool will be automatically set at runtime to match the number of packets configured here.
</tr>

<tr>
    <td>Interface instance
    <td>TI Networking / NetxDuo
    <td>No of interfaces allocated by the example
    <td>Only one interface should be set to default when more than one interface is allocated.
</tr>
</table>

# TCP Server using ncat tool

Ncat is a general-purpose command-line tool for reading, writing, redirecting, and encrypting data across a network. It aims to be your network Swiss Army knife, handling a wide variety of security testing and administration tasks. Ncat is suitable for interactive use or as a network-connected back end for other tools.
 - Ncat is started as server to which EVM connects.
 - Version used for this example Version 7+ ( https://nmap.org/ncat )

# Steps to Run the Example

## Build the example

- When using CCS projects to build, import the CCS project for the required combination
  and build it using the CCS project menu (see \ref CCS_PROJECTS_PAGE).
- When using makefiles to build, note the required combination and build using
  make command (see \ref MAKEFILE_BUILD_PAGE)

## HW Setup

\note Make sure you have setup the EVM with cable connections as shown here, \ref EVM_SETUP_PAGE.
      In addition do below steps.


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

- To enable static IP, set the static IP in the ipAddr variable in the App_setupNetif() before passing it as arguement
  to initiate the netif, and stop the dhcp from starting in the App_allocateIPAddress() function.

## Run the example

\attention If you need to reload and run again, a CPU power-cycle is MUST

- Launch a CCS debug session and run the example executable, see \ref CCS_LAUNCH_PAGE
- You will see logs in the UART terminal as shown in the next section.
- Note the IP address seen in the log, this is what we will use to communicate with the EVM.

## Steps to execute

1. Start TCP server using 'ncat' cmds as shown below. Below steps have been tried with a Linux Ubuntu 18.04 host PC running bash shell

2. Install 'ncat' if not installed by doing below
    \code
    $sudo apt install ncat
    \endcode

3. Invoke 'ncat' to start TCP echo server  on port 8888 as below
    \code
    $ncat -e /bin/cat -kv -l 8888
    \endcode

4. Then, run the example on EVM.
5. Enter host PC IP address when example prompts on UART terminal.


## Sample output for CPSW example

\code

=============================
   CPSW NETXDUO TCP CLIENT   
=============================
Enabling clocks!
EnetAppUtils_reduceCoreMacAllocation: Reduced Mac Address Allocation for CoreId:1 From 4 To 2 
Open MAC port 1
EnetPhy_bindDriver: PHY 0: OUI:080028 Model:23 Ver:01 <-> 'DP83867' : OK

Open MAC port 2
EnetPhy_bindDriver: PHY 3: OUI:080028 Model:0f Ver:01 <-> 'DP83869' : OK

PHY 0 is alive
PHY 3 is alive
Cpsw_internalIoctl_handler_ENET_IOCTL_REGISTER_DSTMAC_RX_FLOW: CPSW: Registered MAC address.ALE entry:1, Policer Entry:0
Waiting for link up...
Waiting for link up...
Cpsw_handleLinkUp: Port 1: Link up: 1-Gbps Full-Duplex

MAC Port 1: link up
Waiting for link up...
Waiting for address from DHCP server on primary interface...
Local Interface IP is: 192.168.1.100
Enter the server IP address.
192.168.50.77
Socket created
Connection with the server is established
"Hello over TCP 1" was sent to the Server
Successfully received the packet 1
"Hello over TCP 2" was sent to the Server
Successfully received the packet 2
"Hello over TCP 3" was sent to the Server
Successfully received the packet 3
"Hello over TCP 4" was sent to the Server
Successfully received the packet 4
"Hello over TCP 5" was sent to the Server
Successfully received the packet 5
Connection closed
Socket deleted
Socket created
Connection with the server is established
"Hello over TCP 1" was sent to the Server
Successfully received the packet 1
"Hello over TCP 2" was sent to the Server
Successfully received the packet 2
"Hello over TCP 3" was sent to the Server
Successfully received the packet 3
"Hello over TCP 4" was sent to the Server
Successfully received the packet 4
"Hello over TCP 5" was sent to the Server
Successfully received the packet 5
Connection closed
Socket deleted
Done

\endcode


## Troubleshooting issues

- If you see MAC address as `00:00:00:00:00:00`, likely you are using a very early Si sample which does not
  have MAC address "fused" in, in this case do below steps

   - Open file `source/networking/.meta/enet_cpsw/templates/am64x_am243x/enet_soc_cfg.c.xdt`
   - Uncomment below line
        \code
        #define ENET_MAC_ADDR_HACK (TRUE)
        \endcode
   - Rebuild the libraries and examples (\ref MAKEFILE_BUILD_PAGE)

- If the execution hangs at `Waiting for address from DHCP server on primary interface...`
   - Make sure you see `MAC Port X: link up` message, if not check the ethernet cable
   - Check the local network and check if the DHCP server is indeed running as expected
   - When using a home broadband/wifi router, its possible to check the clients connected to the DHCP server via a web
     browser. Check your router user manual for more details.

# See Also
\ref NETWORKING
