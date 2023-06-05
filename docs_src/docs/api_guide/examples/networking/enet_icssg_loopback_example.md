# Enet ICSSG Loopback Example {#EXAMPLES_ENET_ICSSG_LOOPBACK}

[TOC]

# Introduction


This example exercises the PHY loopback functionality of the external PHY (layer1) SoC. The ICSSG peripheral is opened with ethernet initialization (in MAC only mode) parameters and the PHY loopback is enabled.
Note that ICSSG pheripheral does not support MAC loopback mode, unlike CPSW.

On @VAR_SOC_NAME, we can do ethernet based communication using ICSSG Hardware peripheral
- ICSS
  - This is a firmware enabled ethernet switch + port HW
  - This HW can be used with industrial communication protocols as well (see \ref EXAMPLES_INDUSTRIAL_COMMS)
  - In this example we use ICSS as a standard ethernet port


The examples do below
- A Tx channel and a Rx flow are opened to enable data transfers. Packets are transmitted from the Switch R5F (Main R5F0_0) to the host port using the Tx channel. These packets are routed back to the host port by the switch hardware as the internal loopback feature is enabled. These packets are then transmitted to the Switch R5F by the Rx flow and the application is notified.
- The Tx and Rx functions in the example are set to transmit and receive 1000 packets. After reaching the count of 1000, the application closes the Tx channel, Rx flow, ICSSG and restarts the application for a configurable number of times. Restarting the loopback test application ensures that there aren’t any memory leaks, and the hardware is closed properly and can be reopened any time.

# Supported Combinations

\cond SOC_AM64X

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0_nortos
 Toolchain      | ti-arm-clang
 Boards         | @VAR_BOARD_NAME_LOWER
 Example folder | examples/networking/enet_loopback/enet_icssg_loopback

\endcond

\cond SOC_AM243X

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0_nortos
 Toolchain      | ti-arm-clang
 Boards         | @VAR_BOARD_NAME_LOWER, @VAR_LP_BOARD_NAME_LOWER
 Example folder | examples/networking/enet_loopback/enet_icssg_loopback

\endcond

# Steps to Run the Example

## Build the example

- When using CCS projects to build, import the CCS project for the required combination
  and build it using the CCS project menu (see \ref CCS_PROJECTS_PAGE).
- When using makefiles to build, note the required combination and build using
  make command (see \ref MAKEFILE_BUILD_PAGE)

## Sample output for PHY Loopback

\code
=============================
 Enet Loopback: Iteration 1 
=============================
ICSSG_DUALMAC Test
Enabling clocks!

Init  configs EnetType:1, InstId :3
----------------------------------------------
Mdio_open: MDIO Manual_Mode enabled
EnetPhy_bindDriver: PHY 15: OUI:080028 Model:0f Ver:01 <-> 'dp83869' : OK
PHY 3 is alive
PHY 15 is alive
initQs() txFreePktInfoQ initialized with 16 pkts

 - HOST PORT statistics
--------------------------------


 Mac 1 statistics
--------------------------------

Icssg_handleLinkUp: icssg1-2: Port 2: Link up: 100-Mbps Full-Duplex
completed

 - HOST PORT statistics
--------------------------------
  hostRxByteCnt              = 518000
  hostTxByteCnt              = 514000
  hostRxPktCnt               = 1000
  hostTxPktCnt               = 1000


 Mac 1 statistics
--------------------------------
  rxGoodFrames            = 1000
  rxBCastFrames           = 1000
  rxMCastFrames           = 1000
  rxClass8                = 1000
  rxClass9                = 1000
  rxBucket5SizedFrame     = 1000
  rxTotalByte             = 518000
  rxTxTotalByte           = 1044000
  txGoodFrame             = 1000
  txBcastFrame            = 1000
  txMcastFrame            = 1000
  txBucket5SizedFrame     = 1000
  txTotalByte             = 526000

Icssg_unregisterEventCb: icssg1-2: event not registered 1
Unregister TX timestamp callback
Icssg_unregisterEventCb: icssg1-2: event not registered 64
Icssg_handleLinkDown: icssg1-2: Port 2: Link down
Disabling clocks for ENET: 1, inst:3!
Test complete: PASS
Loopback application completed
All tests have passed!!

\endcode

# See Also

\ref NETWORKING
