# Enet CPSW Loopback Example {#EXAMPLES_ENET_CPSW_LOOPBACK}

[TOC]

# Introduction


This example exercises the MAC loopback and PHY loopback functionality of the hardware. The CPSW hardware is opened with default initialization parameters and either the MAC loopback or PHY loopback is enabled based on the user input.

\cond SOC_AM273X || SOC_AWR294X || SOC_AM263X || SOC_AM263PX || SOC_AM64X || SOC_AM243X || SOC_AM261X

On @VAR_SOC_NAME, we can do ethernet based communication using CPSW as HW mechanism
  - CPSW is a standard ethernet switch + port HW
  - It uses ethernet driver underneath with LwIP TCP/IP networking stack

\endcond

The examples do the following:
- A Tx channel and an Rx flow are opened to enable data transfers. Packets are transmitted from the Switch (R5F (Main R5F0_0) \cond SOC_AM64X 
or A53(A53SS_0))  \endcond to the host port using the Tx channel. These packets are routed back to the host port by the switch hardware as the internal loopback feature is enabled. These packets are then transmitted to the Switch (R5F  \cond SOC_AM64X 
or A53(A53SS_0) ) \endcond by the Rx flow and the application is notified.
- The Tx and Rx functions in the example are set to transmit and receive 5000 packets. After reaching the count of 5000, the application closes the Tx channel, Rx flow, CPSW and restarts the application for a configurable number of times. Restarting the loopback test application ensures that there aren’t any memory leaks, and the hardware is closed properly and can be reopened at any time.

# Supported Combinations

\cond SOC_AM273X

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0_freertos
 Toolchain      | ti-arm-clang
 Boards         | @VAR_BOARD_NAME_LOWER
 Example folder | source/networking/enet/core/examples/enet_loopback/enet_cpsw_loopback

\endcond

\cond SOC_AWR294X

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0_freertos
 Toolchain      | ti-arm-clang
 Boards         | @VAR_BOARD_NAME_LOWER
Example folder  | source/networking/enet/core/examples/enet_loopback/enet_cpsw_loopback

\endcond

\cond SOC_AM263X

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0_freertos
 Toolchain      | ti-arm-clang
 Boards         | @VAR_BOARD_NAME_LOWER, @VAR_LP_BOARD_NAME_LOWER
 Example folder | source/networking/enet/core/examples/enet_loopback/enet_cpsw_loopback

\endcond

\cond SOC_AM263PX

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0_freertos, r5fss0-1_freertos
 Toolchain      | ti-arm-clang
 Boards         | @VAR_BOARD_NAME_LOWER, @VAR_LP_BOARD_NAME_LOWER, @VAR_BOARD_NAME_ADDON_AUTO
 Example folder | source/networking/enet/core/examples/enet_loopback/enet_cpsw_loopback
 
 For support on @VAR_BOARD_NAME_ADDON_AUTO, please refer \ref ETHERNET_ADDON_BOARDS_TOP

\endcond

\cond SOC_AM261X

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0_freertos
 Toolchain      | ti-arm-clang
 Boards         | @VAR_LP_BOARD_NAME_LOWER, @VAR_BOARD_NAME_LOWER
 Example folder | source/networking/enet/core/examples/enet_loopback/enet_cpsw_loopback
 
 For support on @VAR_LP_BOARD_NAME_LOWER please refer \ref ETHERNET_ADDON_BOARDS_TOP

\endcond

\cond SOC_AM64X

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0_freertos
  ^             | a53ss0-0 freertos
 Toolchain      | ti-arm-clang
 ^              | gcc-aarch64
 Boards         | @VAR_BOARD_NAME_LOWER @VAR_SK_BOARD_NAME_LOWER
 Example folder | source/networking/enet/core/examples/enet_loopback/enet_cpsw_loopback

\endcond

\cond SOC_AM243X

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0_freertos
 Toolchain      | ti-arm-clang
 Boards         | @VAR_BOARD_NAME_LOWER, @VAR_LP_BOARD_NAME_LOWER
Example folder | source/networking/enet/core/examples/enet_loopback/enet_cpsw_loopback

\endcond

# Packet pool configuration
To change packet pool configuration from syscfg, please refer to \ref PACKETPOOL_CONFIG_TOP

# Steps to Run the Example

## Build the example

- When using CCS projects to build, import the CCS project for the required combination
  and build it using the CCS project menu (see \ref CCS_PROJECTS_PAGE).
- When using makefiles to build, note the required combination and build using
  make command (see \ref MAKEFILE_BUILD_PAGE)

## Sample output for MAC Loopback

\code

=============================
 Enet Loopback: Iteration 1 
=============================
CPSW_3G Test
Open MAC port 1
Setting in NO-PHY mode for MAC port 1
PHY 0 is alive
PHY 3 is alive
initQs() txFreePktInfoQ initialized with 16 pkts
Received 5000 packets
Delete EnetApp_rxTask() and exit..
Transmitted 5000 packets 
Delete EnetApp_txTask() and exit..

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
  txPri[0]                = 5000
  txPriBcnt[0]            = 2590000


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

Cpsw_handleLinkDown:1679 
Test complete: PASS
=============================
 Enet Loopback: Iteration 2 
=============================
CPSW_3G Test
EnetAppUtils_reduceCoreMacAllocation: Reduced Mac Address Allocation for CoreId:0 From 4 To 1 
Open MAC port 1
Setting in NO-PHY mode for MAC port 1
PHY 0 is alive
PHY 3 is alive
initQs() txFreePktInfoQ initialized with 16 pkts
Received 5000 packets
Delete EnetApp_rxTask() and exit..
Transmitted 5000 packets 
Delete EnetApp_txTask() and exit..

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
  txPri[0]                = 5000
  txPriBcnt[0]            = 2590000


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

Cpsw_handleLinkDown:1679 
Test complete: PASS
=============================
 Enet Loopback: Iteration 3 
=============================
CPSW_3G Test
EnetAppUtils_reduceCoreMacAllocation: Reduced Mac Address Allocation for CoreId:0 From 4 To 1 
Open MAC port 1
Setting in NO-PHY mode for MAC port 1
PHY 0 is alive
PHY 3 is alive
initQs() txFreePktInfoQ initialized with 16 pkts
Received 5000 packets
Delete EnetApp_rxTask() and exit..
Transmitted 5000 packets 
Delete EnetApp_txTask() and exit..

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
  txPri[0]                = 5000
  txPriBcnt[0]            = 2590000


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

Cpsw_handleLinkDown:1679 
Test complete: PASS
=============================
 Enet Loopback: Iteration 4 
=============================
CPSW_3G Test
EnetAppUtils_reduceCoreMacAllocation: Reduced Mac Address Allocation for CoreId:0 From 4 To 1 
Open MAC port 1
Setting in NO-PHY mode for MAC port 1
PHY 0 is alive
PHY 3 is alive
initQs() txFreePktInfoQ initialized with 16 pkts
Received 5000 packets
Delete EnetApp_rxTask() and exit..
Transmitted 5000 packets 
Delete EnetApp_txTask() and exit..

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
  txPri[0]                = 5000
  txPriBcnt[0]            = 2590000


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

Cpsw_handleLinkDown:1679 
Test complete: PASS
=============================
 Enet Loopback: Iteration 5 
=============================
CPSW_3G Test
EnetAppUtils_reduceCoreMacAllocation: Reduced Mac Address Allocation for CoreId:0 From 4 To 1 
Open MAC port 1
Setting in NO-PHY mode for MAC port 1
PHY 0 is alive
PHY 3 is alive
initQs() txFreePktInfoQ initialized with 16 pkts
Received 5000 packets
Delete EnetApp_rxTask() and exit..
Transmitted 5000 packets 
Delete EnetApp_txTask() and exit..

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
  txPri[0]                = 5000
  txPriBcnt[0]            = 2590000


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

Cpsw_handleLinkDown:1679 
Test complete: PASS
Loopback application completed
All tests have passed!!

\endcode

## Sample output for PHY Loopback

\code

=============================
 Enet Loopback: Iteration 1 
=============================
CPSW_3G Test
Open MAC port 1
EnetPhy_bindDriver:1873 
PHY 0 is alive
PHY 3 is alive
initQs() txFreePktInfoQ initialized with 16 pkts
Cpsw_handleLinkUp:1653 
Received 5000 packets
Delete EnetApp_rxTask() and exit..
Transmitted 5000 packets 
Delete EnetApp_txTask() and exit..

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
  txPri[0]                = 5000
  txPriBcnt[0]            = 2590000


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

Cpsw_handleLinkDown:1679 
Test complete: PASS
=============================
 Enet Loopback: Iteration 2 
=============================
CPSW_3G Test
EnetAppUtils_reduceCoreMacAllocation: Reduced Mac Address Allocation for CoreId:0 From 4 To 1 
Open MAC port 1
EnetPhy_bindDriver:1873 
PHY 0 is alive
PHY 3 is alive
initQs() txFreePktInfoQ initialized with 16 pkts
Cpsw_handleLinkUp:1653 
Received 5000 packets
Delete EnetApp_rxTask() and exit..
Transmitted 5000 packets 
Delete EnetApp_txTask() and exit..

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
  txPri[0]                = 5000
  txPriBcnt[0]            = 2590000


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

Cpsw_handleLinkDown:1679 
Test complete: PASS
=============================
 Enet Loopback: Iteration 3 
=============================
CPSW_3G Test
EnetAppUtils_reduceCoreMacAllocation: Reduced Mac Address Allocation for CoreId:0 From 4 To 1 
Open MAC port 1
EnetPhy_bindDriver:1873 
PHY 0 is alive
PHY 3 is alive
initQs() txFreePktInfoQ initialized with 16 pkts
Cpsw_handleLinkUp:1653 
Received 5000 packets
Delete EnetApp_rxTask() and exit..
Transmitted 5000 packets 
Delete EnetApp_txTask() and exit..

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
  txPri[0]                = 5000
  txPriBcnt[0]            = 2590000


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

Cpsw_handleLinkDown:1679 
Test complete: PASS
=============================
 Enet Loopback: Iteration 4 
=============================
CPSW_3G Test
EnetAppUtils_reduceCoreMacAllocation: Reduced Mac Address Allocation for CoreId:0 From 4 To 1 
Open MAC port 1
EnetPhy_bindDriver:1873 
PHY 0 is alive
PHY 3 is alive
initQs() txFreePktInfoQ initialized with 16 pkts
Cpsw_handleLinkUp:1653 
Received 5000 packets
Delete EnetApp_rxTask() and exit..
Transmitted 5000 packets 
Delete EnetApp_txTask() and exit..

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
  txPri[0]                = 5000
  txPriBcnt[0]            = 2590000


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

Cpsw_handleLinkDown:1679 
Test complete: PASS
=============================
 Enet Loopback: Iteration 5 
=============================
CPSW_3G Test
EnetAppUtils_reduceCoreMacAllocation: Reduced Mac Address Allocation for CoreId:0 From 4 To 1 
Open MAC port 1
EnetPhy_bindDriver:1873 
PHY 0 is alive
PHY 3 is alive
initQs() txFreePktInfoQ initialized with 16 pkts
Cpsw_handleLinkUp:1653 
Received 5000 packets
Delete EnetApp_rxTask() and exit..
Transmitted 5000 packets 
Delete EnetApp_txTask() and exit..

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
  txPri[0]                = 5000
  txPriBcnt[0]            = 2590000


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

Cpsw_handleLinkDown:1679 
Test complete: PASS
Loopback application completed
All tests have passed!!

\endcode

## Troubleshooting issues

\cond SOC_AM261X
- For @VAR_LP_BOARD_NAME_LOWER, software configures MDIO based on the Board version(E1/E2) which is read from EEPROM. It expects 
  on-board EEPROM to be pre-programmed for E2 EVMs. If EEPROM(0x51) is not programmed, the software considers 
  EVM as "E1" version.
\endcond

\cond SOC_AM263PX
- To ensure compatibility with CPSW RGMII2 port on @VAR_BOARD_NAME_LOWER board, make sure that SW-15(switch) should be set to `LOW` for board versions(E2/A/B versions). This configuration ensures seamless out-of-box support and functionality with the CPSW RGMII2 port on the specified versions.
\endcond

# See Also

\ref NETWORKING
