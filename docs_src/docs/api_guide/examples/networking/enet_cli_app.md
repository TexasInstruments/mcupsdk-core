# Enet CLI Example {#EXAMPLES_ENET_CLI_APP}

[TOC]

## Introduction

This example demonstrates the usage of the Enet CLI library.

This example does the following:
- Initializes and opens the ethernet drivers.
- Initializes the Enet CLI library and enables built-in commands of the library.
- Registers a few custom commands for various operations like sending and recieving packets, adding unicast address, configuring VLAN, etc.
- It also registers commands to launch the gPTP stack and LwIP shell.
- On running the application, it launches a command line interface on the UART terminal.

## Supported Combinations

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0_freertos
 Toolchain      | ti-arm-clang
 Boards         | am243x-lp
 Example folder | examples/networking/enet_cli_app

## Steps to Run the Example

### Build the Example

- When using CCS projects to build, import the CCS project for the required combination
  and build it using the CCS project menu (see \ref CCS_PROJECTS_PAGE).
- When using makefiles to build, note the required combination and build using
  make command (see \ref MAKEFILE_BUILD_PAGE)

### Sample Outputs

Once the application has started, using the `help` commands will give the following output:

\code

Enabling clocks!
EnetAppUtils_reduceCoreMacAllocation: Reduced Mac Address Allocation for CoreId:1 From 4 To 2 
Mdio_open: MDIO Manual_Mode enabled
EnetPhy_bindDriver: PHY 3: OUI:080028 Model:0f Ver:01 <-> 'dp83869' : OK
EnetPhy_bindDriver: PHY 15: OUI:080028 Model:0f Ver:01 <-> 'dp83869' : OK
PHY 3 is alive
PHY 15 is alive
Cpsw_internalIoctl_handler_ENET_IOCTL_REGISTER_DSTMAC_RX_FLOW: CPSW: Registered MAC address.ALE entry:2, Policer Entry
Cpsw_handleLinkUp: Port 1: Link up: 100-Mbps Full-Duplex
---------------------------------------

CLI for AM243x
Use 'help' to list all available commands

> help

help:
 Lists all the registered commands

enet_addtxchn <tx_ch_num>:
 Opens a Tx channel.

enet_addrxchn <rx_ch_num>:
 Opens an Rx channel.

enet_sendraw <tx_ch_num> [-dm <dest_mac_addr>] [-sm <src_mac_addr>] [-v <vlan_id>] [-pcp <priority>] [-m <message>]:
 Sends a raw ethernet packet.

enet_capture {start | stop} <rx_ch_num>:
 Start/stop capturing incoming ethernet packets.

enet_capturedump <rx_ch_num>:
 Returns the last 4 packets recieved at the specified channel.

quit:
 Closes the CLI application.

enet_adducast <mac_addr> [-d]:
 Adds a unicast ALE entry with the given MAC address.
 Use the -d tag to make the MAC address as the default source address for sending packets.

enet_remucast <mac_addr>:
 Removes unicast ALE entry with the given MAC address.

enet_addvlan <vlan_id> {<port1> ...}:
 Configures the given ports to the specified VLAN id.

enet_remvlan <vlan_id>:
 Removes VLAN config with specified ID.

enet_ptpd {start | stop} <dma_channel> [-p1 <priority_1>] [-p2 <priority_2>] [-s <sync_interval>]:
 Start/stop gPTP stack. 

enet_hostmacaddr:
 Prints the host port MAC address. 

lwip_shell {start | stop}:
 Opens an LwIP shell.

enet_cfg {help|mqprio|tracelvl|classifier}:
 Commands to modify ethernet configurations.

enet_dbg {help|cpswstats|dumpale|dumppolicer}:
 Commands to print debug data.

phy {help|scan|status|dump|write|read}:
 Commands to access ethernet PHYs.

utils {help|cpuload|readmem|writemem}:
 Utility commands for SOC.


> 

\endcode

The commands `enet_cfg`, `enet_dbg`, `phy` and `utils` are the built-in commands provided by the Enet CLI library. The rest of the commands are user defined.

### Accessing the LwIP Shell

The LwIP shell can be launched by using the command `lwip_shell start`. Starting the LwIP shell will give the following output:

\code

> lwip_shell start
Starting lwIP, local interface IP is dhcp-enabled
[LWIPIF_LWIP] NETIF INIT SUCCESS
Host MAC address-2 : 70:ff:76:1e:95:b7
[0]Enet IF UP Event. Local interface IP:0.0.0.0
[LWIPIF_LWIP] Enet has been started successfully
[0]Network Link UP Event
[0]Enet IF UP Event. Local interface IP:172.24.227.48
Shell Opened

> 

\endcode 

> DMA channel 2 is dedicated for the LwIP stack.

The LwIP shell can be accessed on any telnet terminal. The below steps have been tried with a Linux Ubuntu 20.04 host PC running bash shell

1. Install '`telnet`' package if not installed by using the below command:
\code
$ sudo apt install telnet
\endcode

2. Open a telnet connection using the command:
\code
$ telnet <ip_addr>
\endcode
Here the `<ip_addr>` is the IP address of the microcontroller.

After launching the telnet terminal and connecting to the microcontroller, using the `help` command in the host PC will give the following output:
\code
Trying 172.24.227.48...
Connected to 172.24.227.48.
Escape character is '^]'.



lwIP simple interactive shell.
(c) Copyright 2001, Swedish Institute of Computer Science.
Written by Adam Dunkels.
For help, try the "help" command.
> help
Available commands:
open [IP address] [TCP port]: opens a TCP connection to the specified address.
lstn [TCP port]: sets up a server on the specified port.
acpt [connection #]: waits for an incoming connection request.
send [connection #] [message]: sends a message on a TCP connection.
udpc [local UDP port] [IP address] [remote port]: opens a UDP "connection".
udpl [local UDP port] [IP address] [remote port]: opens a UDP-Lite "connection".
udpn [local UDP port] [IP address] [remote port]: opens a UDP "connection" without checksums.
udpb [local port] [remote port]: opens a UDP broadcast "connection".
usnd [connection #] [message]: sends a message on a UDP connection.
recv [connection #]: receives data on a TCP or UDP connection.
clos [connection #]: closes a TCP or UDP connection.
stat: prints out lwIP statistics.
idxtoname [index]: outputs interface name from index.
nametoidx [name]: outputs interface index from name.
gethostnm [name]: outputs IP address of host.
quit: quits
> 
\endcode

These are the built-in commands provided by the LwIP shell.

## See Also

\ref NETWORKING | \ref ENET_CLI
