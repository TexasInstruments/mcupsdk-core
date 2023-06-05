# Enet CPSW Loopback Example {#EXAMPLES_ENET_CPSW_LOOPBACK}

[TOC]

# Introduction


This example exercises the MAC loopback and PHY loopback functionality of the hardware. The CPSW hardware is opened with default initialization parameters and either the MAC loopback or PHY loopback is enabled based on the user input.

\cond SOC_AM273X || SOC_AWR294X || SOC_AM263X || SOC_AM64X || SOC_AM243X

On @VAR_SOC_NAME, we can do ethernet based communication using CPSW as HW mechanism
  - CPSW is a standard ethernet switch + port HW
  - It uses ethernet driver underneath with LwIP TCP/IP networking stack

\endcond

The examples do below
- A Tx channel and a Rx flow are opened to enable data transfers. Packets are transmitted from the Switch R5F (Main R5F0_0) to the host port using the Tx channel. These packets are routed back to the host port by the switch hardware as the internal loopback feature is enabled. These packets are then transmitted to the Switch R5F by the Rx flow and the application is notified.
- The Tx and Rx functions in the example are set to transmit and receive 5000 packets. After reaching the count of 5000, the application closes the Tx channel, Rx flow, CPSW and restarts the application for a configurable number of times. Restarting the loopback test application ensures that there aren’t any memory leaks, and the hardware is closed properly and can be reopened any time.

# Supported Combinations

\cond SOC_AM273X

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0_freertos
 Toolchain      | ti-arm-clang
 Boards         | @VAR_BOARD_NAME_LOWER
 Example folder | examples/networking/enet_loopback/enet_cpsw_loopback

\endcond

\cond SOC_AWR294X

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0_freertos
 Toolchain      | ti-arm-clang
 Boards         | @VAR_BOARD_NAME_LOWER
Example folder | examples/networking/enet_loopback/enet_cpsw_loopback

\endcond

\cond SOC_AM263X

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0_freertos
 Toolchain      | ti-arm-clang
 Boards         | @VAR_BOARD_NAME_LOWER, @VAR_LP_BOARD_NAME_LOWER
 Example folder | examples/networking/enet_loopback/enet_cpsw_loopback

\endcond

\cond SOC_AM64X

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0_freertos
 Toolchain      | ti-arm-clang
 Boards         | @VAR_BOARD_NAME_LOWER @VAR_SK_BOARD_NAME_LOWER
 Example folder | examples/networking/enet_loopback/enet_cpsw_loopback

\endcond

\cond SOC_AM243X

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0_freertos
 Toolchain      | ti-arm-clang
 Boards         | @VAR_BOARD_NAME_LOWER, @VAR_LP_BOARD_NAME_LOWER
Example folder | examples/networking/enet_loopback/enet_cpsw_loopback

\endcond

# Steps to Run the Example

## Build the example

- When using CCS projects to build, import the CCS project for the required combination
  and build it using the CCS project menu (see \ref CCS_PROJECTS_PAGE).
- When using makefiles to build, note the required combination and build using
  make command (see \ref MAKEFILE_BUILD_PAGE)

## Sample output for MAC Loopback

\code

0: Internal MAC loopback
1: External PHY loopback
Enter option:0

Create periodic tick task
=============================
 Enet Loopback: Iteration 1
=============================
CPSW_3G Test
initQs() txFreePktInfoQ initialized with 16 pkts
Host MAC address: 70:ff:76:1d:ec:f2
PHY 0 is alive
Received 5000 packets
Delete EnetLpbk_rxTask() and exit..
Transmitted 5000 packets
Delete EnetLpbk_txTask() and exit..

 Port 0 Statistics
-----------------------------------------
  rxGoodFrames            = 5000
  rxBcastFrames           = 5000
  rxOctets                = 2590000
  txGoodFrames            = 5000
  txBcastFrames           = 5000
  txOctets                = 2590000
  octetsFrames512to1023   = 10000
  netOctets               = 5180000


 Port 1 Statistics
-----------------------------------------
  rxGoodFrames            = 5000
  rxBcastFrames           = 5000
  rxOctets                = 2590000
  txGoodFrames            = 5000
  txBcastFrames           = 5000
  txOctets                = 2590000
  octetsFrames512to1023   = 10000
  netOctets               = 5180000
  txPri[0]                = 5000
  txPriBcnt[0]            = 2590000

Cpsw_handleLinkDown: Port 1: Link down
Test complete: PASS
Delete EnetLpbk_tickTask() and exit..
Create periodic tick task
=============================
 Enet Loopback: Iteration 2
=============================
CPSW_3G Test
initQs() txFreePktInfoQ initialized with 16 pkts
Host MAC address: 70:ff:76:1d:ec:f2
PHY 0 is alive
Received 5000 packets
Delete EnetLpbk_rxTask() and exit..
Transmitted 5000 packets
Delete EnetLpbk_txTask() and exit..

 Port 0 Statistics
-----------------------------------------
  rxGoodFrames            = 5000
  rxBcastFrames           = 5000
  rxOctets                = 2590000
  txGoodFrames            = 5000
  txBcastFrames           = 5000
  txOctets                = 2590000
  octetsFrames512to1023   = 10000
  netOctets               = 5180000


 Port 1 Statistics
-----------------------------------------
  rxGoodFrames            = 5000
  rxBcastFrames           = 5000
  rxOctets                = 2590000
  txGoodFrames            = 5000
  txBcastFrames           = 5000
  txOctets                = 2590000
  octetsFrames512to1023   = 10000
  netOctets               = 5180000
  txPri[0]                = 5000
  txPriBcnt[0]            = 2590000

Cpsw_handleLinkDown: Port 1: Link down
Test complete: PASS
Delete EnetLpbk_tickTask() and exit..
Create periodic tick task
=============================
 Enet Loopback: Iteration 3
=============================
CPSW_3G Test
initQs() txFreePktInfoQ initialized with 16 pkts
Host MAC address: 70:ff:76:1d:ec:f2
PHY 0 is alive
Received 5000 packets
Delete EnetLpbk_rxTask() and exit..
Transmitted 5000 packets
Delete EnetLpbk_txTask() and exit..

 Port 0 Statistics
-----------------------------------------
  rxGoodFrames            = 5000
  rxBcastFrames           = 5000
  rxOctets                = 2590000
  txGoodFrames            = 5000
  txBcastFrames           = 5000
  txOctets                = 2590000
  octetsFrames512to1023   = 10000
  netOctets               = 5180000


 Port 1 Statistics
-----------------------------------------
  rxGoodFrames            = 5000
  rxBcastFrames           = 5000
  rxOctets                = 2590000
  txGoodFrames            = 5000
  txBcastFrames           = 5000
  txOctets                = 2590000
  octetsFrames512to1023   = 10000
  netOctets               = 5180000
  txPri[0]                = 5000
  txPriBcnt[0]            = 2590000

Cpsw_handleLinkDown: Port 1: Link down
Test complete: PASS
Delete EnetLpbk_tickTask() and exit..
Create periodic tick task
=============================
 Enet Loopback: Iteration 4
=============================
CPSW_3G Test
initQs() txFreePktInfoQ initialized with 16 pkts
Host MAC address: 70:ff:76:1d:ec:f2
PHY 0 is alive
Received 5000 packets
Delete EnetLpbk_rxTask() and exit..
Transmitted 5000 packets
Delete EnetLpbk_txTask() and exit..

 Port 0 Statistics
-----------------------------------------
  rxGoodFrames            = 5000
  rxBcastFrames           = 5000
  rxOctets                = 2590000
  txGoodFrames            = 5000
  txBcastFrames           = 5000
  txOctets                = 2590000
  octetsFrames512to1023   = 10000
  netOctets               = 5180000


 Port 1 Statistics
-----------------------------------------
  rxGoodFrames            = 5000
  rxBcastFrames           = 5000
  rxOctets                = 2590000
  txGoodFrames            = 5000
  txBcastFrames           = 5000
  txOctets                = 2590000
  octetsFrames512to1023   = 10000
  netOctets               = 5180000
  txPri[0]                = 5000
  txPriBcnt[0]            = 2590000

Cpsw_handleLinkDown: Port 1: Link down
Test complete: PASS
Delete EnetLpbk_tickTask() and exit..
Create periodic tick task
=============================
 Enet Loopback: Iteration 5
=============================
CPSW_3G Test
initQs() txFreePktInfoQ initialized with 16 pkts
Host MAC address: 70:ff:76:1d:ec:f2
PHY 0 is alive
Received 5000 packets
Delete EnetLpbk_rxTask() and exit..
Transmitted 5000 packets
Delete EnetLpbk_txTask() and exit..

 Port 0 Statistics
-----------------------------------------
  rxGoodFrames            = 5000
  rxBcastFrames           = 5000
  rxOctets                = 2590000
  txGoodFrames            = 5000
  txBcastFrames           = 5000
  txOctets                = 2590000
  octetsFrames512to1023   = 10000
  netOctets               = 5180000


 Port 1 Statistics
-----------------------------------------
  rxGoodFrames            = 5000
  rxBcastFrames           = 5000
  rxOctets                = 2590000
  txGoodFrames            = 5000
  txBcastFrames           = 5000
  txOctets                = 2590000
  octetsFrames512to1023   = 10000
  netOctets               = 5180000
  txPri[0]                = 5000
  txPriBcnt[0]            = 2590000

Cpsw_handleLinkDown: Port 1: Link down
Test complete: PASS
Delete EnetLpbk_tickTask() and exit..
Loopback application completed

\endcode

## Sample output for PHY Loopback

\code

0: Internal MAC loopback
1: External PHY loopback
Enter option:1

Create periodic tick task
=============================
 Enet Loopback: Iteration 1
=============================
CPSW_3G Test
EnetPhy_bindDriver: PHY 0: OUI:080028 Model:0f Ver:01 <-> 'dp83869' : OK
initQs() txFreePktInfoQ initialized with 16 pkts
Host MAC address: 70:ff:76:1d:ec:f2
PHY 0 is alive
Cpsw_handleLinkUp: Port 1: Link up: 100-Mbps Full-Duplex
Received 5000 packets
Delete EnetLpbk_rxTask() and exit..
Transmitted 5000 packets
Delete EnetLpbk_txTask() and exit..

 Port 0 Statistics
-----------------------------------------
  rxGoodFrames            = 5000
  rxBcastFrames           = 5000
  rxOctets                = 2590000
  txGoodFrames            = 5000
  txBcastFrames           = 5000
  txOctets                = 2590000
  octetsFrames512to1023   = 10000
  netOctets               = 5180000


 Port 1 Statistics
-----------------------------------------
  rxGoodFrames            = 5000
  rxBcastFrames           = 5000
  rxOctets                = 2590000
  txGoodFrames            = 5000
  txBcastFrames           = 5000
  txOctets                = 2590000
  octetsFrames512to1023   = 10000
  netOctets               = 5180000
  txPri[0]                = 5000
  txPriBcnt[0]            = 2590000

Cpsw_handleLinkDown: Port 1: Link down
Test complete: PASS
Delete EnetLpbk_tickTask() and exit..
Create periodic tick task
=============================
 Enet Loopback: Iteration 2
=============================
CPSW_3G Test
EnetPhy_bindDriver: PHY 0: OUI:080028 Model:0f Ver:01 <-> 'dp83869' : OK
initQs() txFreePktInfoQ initialized with 16 pkts
Host MAC address: 70:ff:76:1d:ec:f2
PHY 0 is alive
Cpsw_handleLinkUp: Port 1: Link up: 100-Mbps Full-Duplex
Received 5000 packets
Delete EnetLpbk_rxTask() and exit..
Transmitted 5000 packets
Delete EnetLpbk_txTask() and exit..

 Port 0 Statistics
-----------------------------------------
  rxGoodFrames            = 5000
  rxBcastFrames           = 5000
  rxOctets                = 2590000
  txGoodFrames            = 5000
  txBcastFrames           = 5000
  txOctets                = 2590000
  octetsFrames512to1023   = 10000
  netOctets               = 5180000


 Port 1 Statistics
-----------------------------------------
  rxGoodFrames            = 5000
  rxBcastFrames           = 5000
  rxOctets                = 2590000
  txGoodFrames            = 5000
  txBcastFrames           = 5000
  txOctets                = 2590000
  octetsFrames512to1023   = 10000
  netOctets               = 5180000
  txPri[0]                = 5000
  txPriBcnt[0]            = 2590000

Cpsw_handleLinkDown: Port 1: Link down
Test complete: PASS
Delete EnetLpbk_tickTask() and exit..
Create periodic tick task
=============================
 Enet Loopback: Iteration 3
=============================
CPSW_3G Test
EnetPhy_bindDriver: PHY 0: OUI:080028 Model:0f Ver:01 <-> 'dp83869' : OK
initQs() txFreePktInfoQ initialized with 16 pkts
Host MAC address: 70:ff:76:1d:ec:f2
PHY 0 is alive
Cpsw_handleLinkUp: Port 1: Link up: 100-Mbps Full-Duplex
Received 5000 packets
Delete EnetLpbk_rxTask() and exit..
Transmitted 5000 packets
Delete EnetLpbk_txTask() and exit..

 Port 0 Statistics
-----------------------------------------
  rxGoodFrames            = 5000
  rxBcastFrames           = 5000
  rxOctets                = 2590000
  txGoodFrames            = 5000
  txBcastFrames           = 5000
  txOctets                = 2590000
  octetsFrames512to1023   = 10000
  netOctets               = 5180000


 Port 1 Statistics
-----------------------------------------
  rxGoodFrames            = 5000
  rxBcastFrames           = 5000
  rxOctets                = 2590000
  txGoodFrames            = 5000
  txBcastFrames           = 5000
  txOctets                = 2590000
  octetsFrames512to1023   = 10000
  netOctets               = 5180000
  txPri[0]                = 5000
  txPriBcnt[0]            = 2590000

Cpsw_handleLinkDown: Port 1: Link down
Test complete: PASS
Delete EnetLpbk_tickTask() and exit..
Create periodic tick task
=============================
 Enet Loopback: Iteration 4
=============================
CPSW_3G Test
EnetPhy_bindDriver: PHY 0: OUI:080028 Model:0f Ver:01 <-> 'dp83869' : OK
initQs() txFreePktInfoQ initialized with 16 pkts
Host MAC address: 70:ff:76:1d:ec:f2
PHY 0 is alive
Cpsw_handleLinkUp: Port 1: Link up: 100-Mbps Full-Duplex
Received 5000 packets
Delete EnetLpbk_rxTask() and exit..
Transmitted 5000 packets
Delete EnetLpbk_txTask() and exit..

 Port 0 Statistics
-----------------------------------------
  rxGoodFrames            = 5000
  rxBcastFrames           = 5000
  rxOctets                = 2590000
  txGoodFrames            = 5000
  txBcastFrames           = 5000
  txOctets                = 2590000
  octetsFrames512to1023   = 10000
  netOctets               = 5180000


 Port 1 Statistics
-----------------------------------------
  rxGoodFrames            = 5000
  rxBcastFrames           = 5000
  rxOctets                = 2590000
  txGoodFrames            = 5000
  txBcastFrames           = 5000
  txOctets                = 2590000
  octetsFrames512to1023   = 10000
  netOctets               = 5180000
  txPri[0]                = 5000
  txPriBcnt[0]            = 2590000

Cpsw_handleLinkDown: Port 1: Link down
Test complete: PASS
Delete EnetLpbk_tickTask() and exit..
Create periodic tick task
=============================
 Enet Loopback: Iteration 5
=============================
CPSW_3G Test
EnetPhy_bindDriver: PHY 0: OUI:080028 Model:0f Ver:01 <-> 'dp83869' : OK
initQs() txFreePktInfoQ initialized with 16 pkts
Host MAC address: 70:ff:76:1d:ec:f2
PHY 0 is alive
Cpsw_handleLinkUp: Port 1: Link up: 100-Mbps Full-Duplex
Received 5000 packets
Delete EnetLpbk_rxTask() and exit..
Transmitted 5000 packets
Delete EnetLpbk_txTask() and exit..

 Port 0 Statistics
-----------------------------------------
  rxGoodFrames            = 5000
  rxBcastFrames           = 5000
  rxOctets                = 2590000
  txGoodFrames            = 5000
  txBcastFrames           = 5000
  txOctets                = 2590000
  octetsFrames512to1023   = 10000
  netOctets               = 5180000


 Port 1 Statistics
-----------------------------------------
  rxGoodFrames            = 5000
  rxBcastFrames           = 5000
  rxOctets                = 2590000
  txGoodFrames            = 5000
  txBcastFrames           = 5000
  txOctets                = 2590000
  octetsFrames512to1023   = 10000
  netOctets               = 5180000
  txPri[0]                = 5000
  txPriBcnt[0]            = 2590000

Cpsw_handleLinkDown: Port 1: Link down
Test complete: PASS
Delete EnetLpbk_tickTask() and exit..
Loopback application completed

\endcode

# See Also

\ref NETWORKING
