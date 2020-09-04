# Enet VLAN ICSSG Example {#EXAMPLES_ENET_VLAN_ICSSG}

[TOC]

# Introduction

This Enet VLAN icssg example demonstrates UNH-IOL VLAN conformance and FDB conformance using Enet with ICSSG peripheral.

\cond SOC_AM64X || SOC_AM243X

On @VAR_SOC_NAME, we can do ethernet based communication using ICSSG HW Mechanism

  - PRU firmware based Ethernet Switch and Dual MAC implementation
  - This HW can be used with industrial communication protocols as well (see \ref EXAMPLES_INDUSTRIAL_COMMS)

\endcond

This example do below:
- Target-side application running on a Cortex R5F core.
	- Application receives the packet, copies the payload into a new packet which is then sent back.
	- The application has a menu to select main test cases or sub test cases which are under main test cases.
	- VLAN test, FDB test, UTILS test, VLAN Aware/Unaware mode and Unit test default settings are in main test cases in the menu and there are many sub test cases under VLAN and FDB tests which are available at UNH-IOL conformance tests to verify the standard conformance criteria.
	- UTILS test is to print/reset the statistics, packet receive count, generate Unicast traffic and etc. This menu along with application logs are implemented via UART.
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
    - Based on test case selction and Filtering options DUT will decide to forward the packet or to drop the packet.
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
 Example folder | examples/networking/enet_vlan_icssg

\endcond

\cond SOC_AM243X

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0_freertos
 Toolchain      | ti-arm-clang
 Boards         | @VAR_BOARD_NAME_LOWER, @VAR_LP_BOARD_NAME_LOWER
 Icssg Instance | ICSSG1
 Example folder | examples/networking/enet_vlan_icssg

\endcond

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
- Select the Unit test default settings test case(5) from main menu and configure switch in VLAN Aware mode by selecting testcase number 4 and provide input as 1 to enable VLAN aware mode and 0 for VLAN unaware mode.
- We can start sending packets from Colasoft Pkt Builder or packETH tool and capture the packets in Wireshark.

## Sample output for VLAN ICSSG example

\code

================================
      ENET ICSSG UNIT TEST
================================

Init all peripheral clocks
----------------------------------------------
Enabling clocks!

Init all configs
----------------------------------------------

Init  configs EnetType:2, InstId :1
----------------------------------------------
icssg1: Open port 1
EnetPhy_bindDriver:
icssg1: Open port 2
EnetPhy_bindDriver:
PHY 3 is alive
PHY 15 is alive

Open all peripherals
----------------------------------------------
icssg1: Register async IOCTL callback
icssg1: Register TX timestamp callback

Attach core id 1 on all peripherals
----------------------------------------------
icssg1: Attach core

Create RX tasks
----------------------------------------------
icssg1: Create RX task
icssg1: Waiting for link up...

***************************************************
                UNIT TEST MENU
***************************************************
1.  TC_VLAN_testing
2.  TC_FDB_testing
3.  UTILS_testing
4.  TC_configure_vlan_aware_unaware
5.  setup_unit_test_default_settings

Test case id: Icssg_handleLinkUp:
icssg1: Port 1 link is up
icssg1: Set port state to 'Forward'
icssg1: Async IOCTL completed
icssg1: Async IOCTL completed
Icssg_handleLinkUp:
icssg1: Port 2 link is up
icssg1: Set port state to 'Forward'
icssg1: Async IOCTL completed
icssg1: Async IOCTL completed
icssg1: Open DMA
initQs() txFreePktInfoQ initialized with 8 pkts
icssg1: Set MAC addr: 70:ff:76:1d:92:c1
icssg1: MAC port addr: 70:ff:76:1d:92:c1


 Invalid test case please select valid test case

***************************************************
                UNIT TEST MENU
***************************************************
1.  TC_VLAN_testing
2.  TC_FDB_testing
3.  UTILS_testing
4.  TC_configure_vlan_aware_unaware
5.  setup_unit_test_default_settings

Test case id: 3
In UTILS related testing...
Please select Test no to choose particular test case to execute
1.  UTILS_display_icssg_hw_consolidated_statistics
2.  UTILS_clear_icssg_hw_consolidated_statistics
3.  UTILS_NRT_add_del_mac_fdb_entry
4.  UTILS_NRT_cofig_port_state
5.  UTILS_NRT_add_fid_vid_entry for PORT1 and PORT2
6.  UTILS_config_PVID
7.  UTILS_get_receive_packet_count
8.  UTILS_clear_receive_packet_count
9.  UTILS_Transmit_UC_packets
1
*************************************************************************
HW STAT PARAM                   PORT1           PORT2
*************************************************************************

Print statistics
----------------------------------------------

 icssg1 statistics
--------------------------------
  rxGoodFrames            = 76991
  rxClass8                = 76991
  rxClass9                = 76991
  rxBucket2SizedFrame     = 65535
  rxTotalByte             = 9854848
  rxTxTotalByte           = 20325624
  txGoodFrame             = 468106
  txBucket3SizedFrame     = 65535
  txTotalByte             = 63662416


Print statistics
----------------------------------------------

 icssg1 statistics
--------------------------------
  rxGoodFrames            = 468106
  rxClass8                = 468106
  rxClass9                = 468106
  rxBucket2SizedFrame     = 65535
  rxTotalByte             = 59917568
  rxTxTotalByte           = 123579984
  txGoodFrame             = 76991
  txBucket3SizedFrame     = 65535
  txTotalByte             = 10470776


Test UTILS_display_icssg_hw_consolidated_statistics run successfully from DUT side...

*****************************************************
        TEST UTILITY EXECUTION SUMMARY
*****************************************************

Test UTILS_testing run successfully from DUT side...

***************************************************
                UNIT TEST MENU
***************************************************
1.  TC_VLAN_testing
2.  TC_FDB_testing
3.  UTILS_testing
4.  TC_configure_vlan_aware_unaware
5.  setup_unit_test_default_settings

Test case id: 3
In UTILS related testing...
Please select Test no to choose particular test case to execute
1.  UTILS_display_icssg_hw_consolidated_statistics
2.  UTILS_clear_icssg_hw_consolidated_statistics
3.  UTILS_NRT_add_del_mac_fdb_entry
4.  UTILS_NRT_cofig_port_state
5.  UTILS_NRT_add_fid_vid_entry for PORT1 and PORT2
6.  UTILS_config_PVID
7.  UTILS_get_receive_packet_count
8.  UTILS_clear_receive_packet_count
9.  UTILS_Transmit_UC_packets
9
Enter port number(PORT1 = 1 and PORT2 = 2) to start transmitting the packets:
1
Enter number of packets to transmit:
100

Test UTILS_Transmit_UC_packets run successfully from DUT side...

*****************************************************
        TEST UTILITY EXECUTION SUMMARY
*****************************************************

Test UTILS_testing run successfully from DUT side...

***************************************************
                UNIT TEST MENU
***************************************************
1.  TC_VLAN_testing
2.  TC_FDB_testing
3.  UTILS_testing
4.  TC_configure_vlan_aware_unaware
5.  setup_unit_test_default_settings

Test case id: 3
In UTILS related testing...
Please select Test no to choose particular test case to execute
1.  UTILS_display_icssg_hw_consolidated_statistics
2.  UTILS_clear_icssg_hw_consolidated_statistics
3.  UTILS_NRT_add_del_mac_fdb_entry
4.  UTILS_NRT_cofig_port_state
5.  UTILS_NRT_add_fid_vid_entry for PORT1 and PORT2
6.  UTILS_config_PVID
7.  UTILS_get_receive_packet_count
8.  UTILS_clear_receive_packet_count
9.  UTILS_Transmit_UC_packets
9
Enter port number(PORT1 = 1 and PORT2 = 2) to start transmitting the packets:
2
Enter number of packets to transmit:
100

Test UTILS_Transmit_UC_packets run successfully from DUT side...

*****************************************************
        TEST UTILITY EXECUTION SUMMARY
*****************************************************

Test UTILS_testing run successfully from DUT side...

***************************************************
                UNIT TEST MENU
***************************************************
1.  TC_VLAN_testing
2.  TC_FDB_testing
3.  UTILS_testing
4.  TC_configure_vlan_aware_unaware
5.  setup_unit_test_default_settings

Test case id:

\endcode

- On Wireshark we can see the packets received:

  \imageStyle{layer2_icssg_example_wireshark_log.png,width:50%}
  \image html layer2_icssg_example_wireshark_log.png Wireshark log for VLAN ICSSG Example

# See Also

\ref DRV_ENET_MODULE   \ref NETWORKING