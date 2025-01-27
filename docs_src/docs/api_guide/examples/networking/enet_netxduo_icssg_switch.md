\cond THREADX
# Enet NetxDuo ICSSG Switch Example{#EXAMPLES_ENET_NETXDUO_ICSSG_SWITCH}

[TOC]

# Introduction

\note NetxDuo features are made available as is from public NetxDuo project. SDK configuration may only enable and exercise a subset of these features.

This example is a TCP/UDP IP application using the NetxDuo networking stack, coupled with ethernet driver (ENET-LLD)


On @VAR_SOC_NAME, we can do ethernet based communication using ICSSG Hardware peripheral
- ICSS
  - This is a firmware enabled ethernet switch + port HW
  - This HW can be used with industrial communication protocols as well
  - In this example we use ICSS as a standard ethernet port

It uses ENET ethernet driver underneath with NetxDuo TCP/IP networking stack

This example does the following
- Initializes the ethernet driver for the underlying HW
- Initializes the NetxDuo stack for TCP/UDP IP
- Allows user to run and test basic networking features like DHCP and ping with TCP/UDP.
- Is configured in Switch mode.

# Supported Combinations

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0_freertos
 Toolchain      | ti-arm-clang
 Boards         | @VAR_BOARD_NAME_LOWER, @VAR_LP_BOARD_NAME_LOWER
 Example folder | source/networking/enet/core/examples/netxduo/enet_netxduo_icssg_switch


# Configuring Syscfg

- Following Syscfg option allows flexibility to configure memory foot print based on required use case like: Gigabit Ethernet Support Enable, premption support, McM Support and QoS level required.

- Supported Options with default configuration

<table>
<tr>
    <th>Feature
    <th>Description
    <th>Remarks/Default Setting
</tr>

<tr>
    <td>Pkt Pool Enable Flag
    <td>Flag to enable packet allocation from enet utils library. It should be disabled to avoid utils memory wastage, in case application allots packet via other mechanism.
    <td>Default is true. If enabled size of pkt pool size depends on Number of Tx Packet and Number of Rx Packet.
</tr>

<tr>
    <td>Number of Tx Packet
    <td>No of Tx packets required for DMA channel
    <td>Default is 16. It contributes to the size of Pkt Mem Pool, DMA ring buffer and accessories.
</tr>

<tr>
    <td>Number of Rx Packet
    <td>No of Rx packets required for DMA channel
    <td>Default is 32. It contributes to the size of Pkt Mem Pool, DMA ring buffer and accessories size.

<tr>
    <td>QoS Level
    <td>No of QoS level required
    <td>Can be in between 1-8. Higher QoS level will be serviced by adding more number of buffers.
</tr>

<tr>
    <td>Premption Enable
    <td>Flag to enable premption
    <td>Default is false. If enabled will add premption buffer to service the feature.
</tr>

<tr>
    <td>Gigabit Support
    <td>Decides buffer pool allocation based on interface speed selected
    <td>Default is true. Enabling this option will increase buffer requirement as more buffering required at gigabit speed.
</tr>

<tr>
    <td>Interface instance
    <td>No of interfaces allocated by the example
    <td>Only one interface should be set to default when more than one interface is allocated.
</tr>
</table>

# Steps to Run the Example


## Build the example

- When using CCS projects to build, import the CCS project for the required combination
  and build it using the CCS project menu (see \htmllink{@VAR_MCU_SDK_DOCS_PATH/CCS_PROJECTS_PAGE.html, Using SDK with CCS Projects}).
- When using makefiles to build, note the required combination and build using
  make command (see \htmllink{@VAR_MCU_SDK_DOCS_PATH/MAKEFILE_BUILD_PAGE.html, Using SDK with Makefiles})

## HW Setup

\note Make sure you have setup the EVM with cable connections as shown here, \htmllink{@VAR_MCU_SDK_DOCS_PATH/EVM_SETUP_PAGE.html, EVM Setup}.
      In addition do below steps.


### AM243X-EVM

#### For ICSS based example

- Connect a ethernet cable to the EVM from host PC as shown below

  \imageStyle{am64x_evm_lwip_example_01.png,width:30%}
  \image html am64x_evm_lwip_example_01.png Ethernet cable for ICSS based ethernet

### AM243X-LP

\note AM243X-LP has two ethernet Ports which can be configured as ICSS ports.

#### For ICSS based examples

- Connect a ethernet cable to the AM243X-LP from host PC as shown below

  \imageStyle{am243x_lp_lwip_example_00.png,width:30%}
  \image html am243x_lp_lwip_example_00.png Ethernet cable for ICSS based ethernet


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

- Launch a CCS debug session and run the example executable, see see \ref CCS_LAUNCH_PAGE
- You will see logs in the UART terminal as shown in the next section.
- Note the IP address seen in the log, this is what we will use to communicate with the EVM.


## Sample output for ICSS example

\code
==============================
   NETXDUO ICSSG SWITCH MODE  
==============================
Enabling clocks!
EnetAppUtils_reduceCoreMacAllocation: Reduced Mac Address Allocation for CoreId:1 From 4 To 1 
Mdio_open: MDIO Manual_Mode enabled

Open MAC port 1
EnetPhy_bindDriver: PHY 15: OUI:080028 Model:0f Ver:01 <-> 'DP83869' : OK

Open MAC port 2
EnetPhy_bindDriver: PHY 3: OUI:080028 Model:0f Ver:01 <-> 'DP83869' : OK

PHY 3 is alive
PHY 15 is alive
Waiting for address from DHCP server on primary interface...
Icssg_handleLinkUp: icssg1: Port 1: Link up: 1-Gbps Full-Duplex

Local Interface IP is: 192.168.1.100
\endcode


## Communicate with the EVM using ethernet

- You can try to reach the EVM using ping as shown below, using a command shell on the host PC

        > ping 192.168.1.100

## Troubleshooting issues

- If you see MAC address as `00:00:00:00:00:00`, likely you are using a very early Si sample which does not
  have MAC address "fused" in, in this case do below steps

   - Open file `source\networking\enet\soc\j7x\am64x_am243x\enet_soc.c`
   - Uncomment below line
        \code
        #define ENET_MAC_ADDR_HACK (TRUE)
        \endcode
   - Rebuild the libraries and examples (\htmllink{@VAR_MCU_SDK_DOCS_PATH/MAKEFILE_BUILD_PAGE.html, Using SDK with Makefiles})

- If the execution hangs at `Waiting for address from DHCP server on primary interface...`
   - Make sure you see `MAC Port X: link up` message, if not check the ethernet cable
   - Check the local network and check if the DHCP server is indeed running as expected
   - When using a home broadband/wifi router, its possible to check the clients connected to the DHCP server via a web
     browser. Check your router user manual for more details.

# See Also
\ref NETWORKING
\endcond