# Enet CPSW Industrial Example {#EXAMPLES_ENET_CPSW_INDUSTRIAL_APP}

[TOC]

# Introduction
This example demonstrates the effectiveness of CPSW in low-latency applications, where the device handles two types of traffic

- Raw-UDP traffic : Low-latency data that requires fast packet processing and transmission
- TCP traffic :Other background traffic that can tolerate some delay in transmission and processing

The application uses separate channels for each type of traffic, with real-time traffic receiving priority to guarantee low latency. The packets received in the this channel undergo modifications and are sent back to the source node. 
The TCP server runs on LwIP stack and echoes back packets received in the designated channel.The application uses two DMA TX channels and two DMA RX channels. Channels TX1 and RX1 are used for raw-UDP traffic; TX0 and RX0 are used for running the TCP server.

# Overview

## Raw-UDP Application
The Raw-UDP channel is configured to facilitate efficient and low-latency packet processing. The setup involves: 
- Initializing DMA Channel 1: The DMA channel is opened, and Tx and Rx queues are initialized to enable data transfer.
- Creating a Packet Processing Task: A task is created to handle packet processing, which is triggered upon the arrival of a packet. 

#### Packet Processing

When a packet enters either of the MAC ports, the destination IP address is compared to the real-time address of the device by the address lookup engine. If a match is found, the packet is routed to the corresponding DMA channel for transmission to the host.  The IP address is configured based on the node ID

After receiving a packet, the payload is modified, and the packet is echoed back to the sender. The RawUdpApp_modifyPkt() function in app_rawudp_priv.c is used for modifying the packet payload. This function can be customized to meet specific requirements, allowing for the transmission of tailored packets. By default, it provides an optimized way to transmit a packet to the sender with a minimally modified payload.

## TCP/IP server
The application implements a TCP server using the standard LwIP stack and uses netconn APIs with cpsw as the ethernet driver

- Initialization of lwIP: The lwIP library is initialized to prepare for network stack setup. A single netif is enabled by default, which can be modified through the system configuration (syscfg).

- Configuration of Network Interface:The network interface (netif) is configured with the necessary settings, including IP address, netmask, and gateway. Static IP addressing and dynamic IP assignment using DHCP are supported.

- Creating a New Thread for the Server Task: A new thread is created to execute the echo server task, which waits for incoming connections and echoes back received packets.

\image html cpsw_industrial_application.png Application Overview  width=15%

## Profiling
The application has the macro PROFILE_EN for profiling software latency of the raw-UDP channel. The start time is recorded in the receive ISR and the end time is recorded after the packets to be transmitted have been submitted to the DMA.The average and maximum values are printed on the console after the configured number of packets( ENETAPP_PKT_PROFILE ) are processed. Profiling is disabled by default it can be enabled in enetapp_cfg.h by uncommenting the macro PROFILE_EN.

# Supported Combinations 


\cond SOC_AM261X
 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0_freertos
 Toolchain      | ti-arm-clang
 Boards         | am261x-lp
 Example folder | source/networking/enet/core/examples/enet_cpsw_industrial_app
 \endcond

\cond SOC_AM263X
 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0_freertos
 Toolchain      | ti-arm-clang
 Boards         | am263x-lp
 Example folder | source/networking/enet/core/examples/enet_cpsw_industrial_app
 \endcond

\cond SOC_AM263PX
 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0_freertos
 Toolchain      | ti-arm-clang
 Boards         | am263px-lp
 Example folder | source/networking/enet/core/examples/enet_cpsw_industrial_app
 \endcond

# Packet pool configuration
To change packet pool configuration from syscfg, please refer to \ref PACKETPOOL_CONFIG_TOP

# Steps to Run the Example

## Build the example

- When using CCS projects to build, import the CCS project for the required combination
  and build it using the CCS project menu (see \ref CCS_PROJECTS_PAGE).
- When using makefiles to build, note the required combination and build using
  make command (see \ref MAKEFILE_BUILD_PAGE)

## Hardware Setup
The EVM must be setup with cable connections as shown in the figure below 
\image html cpsw_industrial_setup.png EVM Setup  width=60%

## Run the example
1. Connect the MAC port as described in **Hardware Setup** section.
2. Launch a CCS debug session and run the example executable, see \ref CCS_LAUNCH_PAGE.
3. Configure the node id for each node. Make sure you change the maximum number of nodes in the application according to your setup.
4. The IP addresses for raw-UDP and TCP channels of each node for different channels will be displayed on the console as shown in the sample output. 
5. Wait for both the links to be up and for the TCP server to be setup. Send packets to the desired node from the source node.
6. Echoed packets from the destination node should be received at the source node with modified payloads.

For using tcp channel please refer to enet_lwip_tcpserver.md

## Sample Output

\code
=======================================================
           CPSW Industrial Application
=======================================================
Enter the nodeId between 1 to 10: 
10
NodeId: 10
Enabled clocks for all ENET instances
EnetAppUtils_reduceCoreMacAllocation: Reduced Mac Address Allocation for CoreId:0 From 4 To 3 
Open MAC port 1
EnetPhy_bindDriver:1873 
Open MAC port 2
EnetPhy_bindDriver:1873 
PHY 3 is alive
PHY 12 is alive

Attach core id 0 on all peripherals
----------------------------------------------
Waiting for link up...
Waiting for port 1 link up...
Cpsw_handleLinkUp:1653 
MAC Port 1: link up

All links up 
Host MAC Address-0 : 70:ff:76:1f:68:ed

Setting up Raw-UDP channel 
initQs() txFreePktInfoQ initialized with 8 pkts
Created Rx task for Raw-UDPP channel 1
IP for Raw-UDP Traffic is 10.24.0.10

Starting lwIP, local interface IP is dhcp-enabled
[0]: Static IP with LWIP-Stack is 192.168.1.10
[LWIPIF_LWIP] NETIF INIT SUCCESS
Host MAC address-0 : 70:ff:76:1f:68:ed
[0]Enet IF UP Event. Local interface IP:192.168.1.10
[LWIPIF_LWIP] Enet has been started successfully
[0]Waiting for network UP ...
[0]Network Link UP Event
Network is UP ...
=======================================================
      SOFTWARE PROFILING FOR RAW-UDP CHANNEL 
=======================================================
Number of Packets = 10000 
Average Profile Time: 17.021572
Max Profile Time: 43.902500
======================================================

\endcode

# See also

\ref NETWORKING 