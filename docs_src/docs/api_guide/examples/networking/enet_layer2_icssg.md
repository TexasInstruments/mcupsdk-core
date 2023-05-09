# Enet Layer 2 ICSSG Example {#EXAMPLES_ENET_LAYER2_ICSSG}

[TOC]

# Introduction

This layer 2 icssg example demonstrates usage of Enet with ICSSG peripheral operation.

\cond SOC_AM64X || SOC_AM243X

On @VAR_SOC_NAME, we can do ethernet based communication using ICSSG HW Mechanism

  - PRU firmware based Ethernet Switch and Dual MAC implementation
  - This HW can be used with industrial communication protocols as well (see \ref EXAMPLES_INDUSTRIAL_COMMS)

\endcond

\note ICSSG Switch mode is not functional in 8.6.0. Kindly follow and use 8.5.0 for switch functionality.

This example do below:
- Target-side application running on a Cortex R5F core.
	- Application receives the packet, copies the payload into a new packet which is then sent back.
	- The application has a menu to enable/disable features, such as getting mac address and stats. This menu along with application logs are implemented via UART.
- Host-side functionality
	- Software applications like Colasoft Pkt Builder or packETH tool could be used to generate and send packets, Wireshark can be used to receive and verify packet contents

- The data path enabled in this example is as follows:
	- Host side (PC) application sends a packet to MAC port.
    - Based on Switch mode or Dual-EMAC mode the data flow will differ.
    - In Switch mode:
        - If packet is non directed unicast(UC) packet, it will be only forwarded.
        - If packet is directed unicast(UC) packet, it will be sent only to target application.
        - If packet is multicast(MC) or broadcast(BC) packet, it will be forwarded as well as sent to target application.
    - In Dual-mac mode:
        - If packet is Non directed unicast(UC) packet it will be dropped.
        - If packet is directed unicast(UC) or multicast(MC) or broadcast(BC) packet, it will be sent to target application.
        - Currently to test both MAC ports simultaneously in Dual-MAC mode it requires additional enet lld(UDMA channel allocation) changes.
	- Target side application receives the packet, updates the MAC addresses in the Layer-2 header and sends the packet back.
	- Application like Wireshark (PC) receives the packet and can be seen in the capture window.


# Supported Combinations

\cond SOC_AM64X

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0_freertos
 Toolchain      | ti-arm-clang
 Board          | @VAR_BOARD_NAME_LOWER
 Icssg Instance | ICSSG1
 Example folder | examples/networking/enet_layer2_icssg

\endcond

\cond SOC_AM243X

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0_freertos
 Toolchain      | ti-arm-clang
 Boards         | @VAR_BOARD_NAME_LOWER, @VAR_LP_BOARD_NAME_LOWER
 Icssg Instance | ICSSG1
 Example folder | examples/networking/enet_layer2_icssg

\endcond

# Configuring ICSSG DUAL MAC with two MAC ports

- Selecting DUAL MAC mode in ICSSG does not imply both the MAC ports being functional.
- To setup Dual MAC configuration, two instances of ICSSG have to be opened.
- Both of the instances have to be setup as shown above in the MAC configuration.
- Unlike shown in the image, set the QoS level to 3 for layer 2 use cases.
- Dual MAC mode Port of the first instance has to be set to MAC_PORT_1.
  \imageStyle{icssg_dmac_sysconfig_1.png,width:30%}
  \image html icssg_dmac_sysconfig_1.png ICSSG DUAL MAC PORT-1 configuration.
  
- Dual MAC mode Port of the second instance has to be set to MAC_PORT_2.
- Uncheck the option "Enable MDIO MDC Config" in secode ICSSG isntance.
- Set the QoS level to 3, same as the first instance.
  \imageStyle{icssg_dmac_sysconfig_2.png,width:30%}
  \image html icssg_dmac_sysconfig_2.png ICSSG DUAL MAC PORT-2 configuration.

# Steps to Run the Example

## Build the example

- When using CCS projects to build, import the CCS project for the required combination
  and build it using the CCS project menu (see \ref CCS_PROJECTS_PAGE).
- When using makefiles to build, note the required combination and build using
  make command (see \ref MAKEFILE_BUILD_PAGE)

\note Enable the ENET_TEST_MII_MODE macro in test application and change the mode to MII from RGMII in SysConfig and rebuid the application to test the ICSSG ports in MII mode.

## HW Setup

\note Make sure you have setup the EVM with cable connections as shown here, \ref EVM_SETUP_PAGE.
      In addition do below steps.

\cond SOC_AM64X

### AM64X-EVM

#### For ICSSG based example

- Connect two ethernet cables to the EVM from two host PC's as shown below

  \imageStyle{am64x_evm_lwip_example_01.png,width:30%}
  \image html am64x_evm_lwip_example_01.png Ethernet cable for ICSS based ethernet

\endcond

\cond SOC_AM243X

### AM243X-EVM

#### For ICSSG based example

- Connect two ethernet cables to the EVM from two host PC's as shown below

  \imageStyle{am64x_evm_lwip_example_01.png,width:30%}
  \image html am64x_evm_lwip_example_01.png Ethernet cable for ICSS based ethernet

### AM243X-LP

\note AM243X-LP has two ethernet Ports which can be configured as both CPSW/ICSS ports.

#### For ICSSG based examples

- Connect two ethernet cables to the AM243X-LP from two host PC's as shown below

  \imageStyle{am243x_lp_lwip_example_00.png,width:30%}
  \image html am243x_lp_lwip_example_00.png Ethernet cable for ICSSG based ethernet

\endcond


## Run the example

\attention If you need to reload and run again, a CPU power-cycle is MUST

- Launch a CCS debug session and run the example executable, see \ref CCS_LAUNCH_PAGE
- You will see logs in the UART terminal as shown in the next section.
- We can start sending packets from Colasoft Pkt Builder or packETH tool and capture the packets in Wireshark.

## Sample output for Layer2 ICSSG example

\code

==========================
      MULTIPORT TEST
==========================

Init all peripheral clocks
----------------------------------------------
Enabling clocks!

Open all peripherals
----------------------------------------------

Init  configs EnetType:2, InstId :1
----------------------------------------------
icssg1: Open port 1
icssg1: Open port 2
EnetPhy_bindDriver:
PHY 3 is alive
PHY 15 is alive
icssg1: Register async IOCTL callback
icssg1: Register TX timestamp callback

Attach core id 1 on all peripherals
----------------------------------------------
icssg1: Attach core

Create RX tasks
----------------------------------------------
icssg1: Create RX task
icssg1: Waiting for link up...

Enet Multiport Menu:
 'T'  -  Enable timestamp prints
 't'  -  Disable timestamp prints
 's'  -  Print statistics
 'r'  -  Reset statistics
 'm'  -  Show allocated MAC addresses
 'd'  -  Enable dscp based priority mapping
 'x'  -  Stop the test

EnetPhy_bindDriver:
Icssg_handleLinkUp:
Icssg_handleLinkUp:
icssg1: Port 1 link is up
icssg1: Set port state to 'Forward'
icssg1: Async IOCTL completed
icssg1: Async IOCTL completed
icssg1: Port 2 link is up
icssg1: Set port state to 'Forward'
icssg1: Async IOCTL completed
icssg1: Async IOCTL completed
icssg1: Open DMA
initQs() txFreePktInfoQ initialized with 8 pkts
icssg1: Set MAC addr: 70:ff:76:1d:92:c1
icssg1: MAC port addr: 70:ff:76:1d:92:c1

Invalid option, try again...

Enet Multiport Menu:
 'T'  -  Enable timestamp prints
 't'  -  Disable timestamp prints
 's'  -  Print statistics
 'r'  -  Reset statistics
 'm'  -  Show allocated MAC addresses
 'd'  -  Enable dscp based priority mapping
 'x'  -  Stop the test

s

Print statistics
----------------------------------------------

 icssg1 - PA statistics
--------------------------------
  port1Q0Overflow            = 234


 icssg1 - Port 1 statistics
--------------------------------
  rxGoodFrames            = 25337838
  rxClass8                = 25337838
  rxClass9                = 25337838
  rxBucket2SizedFrame     = 65535
  rxTotalByte             = 3243243264
  rxTxTotalByte           = 4294967295
  txGoodFrame             = 25337838
  txBucket3SizedFrame     = 65535
  txTotalByte             = 3445945968


 icssg1 - Port 2 statistics
--------------------------------
  rxGoodFrames            = 25337838
  rxClass8                = 25337838
  rxClass9                = 25337838
  rxBucket2SizedFrame     = 65535
  rxTotalByte             = 3243243264
  rxTxTotalByte           = 4294967295
  txGoodFrame             = 25337604
  txBucket3SizedFrame     = 65535
  txTotalByte             = 3445914144

\endcode

- On Wireshark we can see the packets received:

  \imageStyle{layer2_icssg_example_wireshark_log.png,width:50%}
  \image html layer2_icssg_example_wireshark_log.png Wireshark log for Layer 2 ICSSG Example

# See Also

\ref DRV_ENET_MODULE  \ref NETWORKING
