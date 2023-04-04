# Enet Layer 2 Multi-Channel Example {#EXAMPLES_ENET_LAYER2_MULTI_CHANNEL}

[TOC]

# Introduction

\cond SOC_AM263X || SOC_AM273X || SOC_AWR294X

The Multi-channel example illustrates the usage of multiple channels of CPDMA. The application creates two independent TX and RX channels each, one for L2 echo server and the second for TimeSync PTP stack.

This example illustrates the usage of multiple CPDMA channels to carry two types of traffic: echo test and PTP. The segregation of Ethernet packets into separate CPDMA channel is based on the priority field (PCP) of the 802.1Q VLAN tagged packets. PTP packets are expected to have a priority of 1, and echo test packets a priority of 0.
The source generator for packets must send packets with 802.1Q VLAN tag enabled to the application.

  \imageStyle{802.1Q_vlan_tag_format.png,width:25%}
  \image html 802.1Q_vlan_tag_format.png 802.1Q VLAN Tag Format

  \imageStyle{cpdma_channels_control_flow.png,width:40%}
  \image html cpdma_channels_control_flow.png CPDMA Channels Control Flow

\endcond

\cond SOC_AM64X || SOC_AM243X

The Multi-channel example illustrates the usage of multiple channels of UDMA. The application creates two independent TX and RX channels each, one for L2 echo server and the second for TimeSync PTP stack.

\endcond

- Enet-lld utilizes hardware timestamping provided by CPTS(CPSW) and provides IOCTL commands for enabling Packet Timestamping functionality.
- The Multi-channel example illustrates using three such IOCTL Commands to exercise the CPTS IOCTLs from application(\ref ENET_TIMESYNC_IOCTL_GET_CURRENT_TIMESTAMP, \ref ENET_TIMESYNC_IOCTL_GET_ETH_RX_TIMESTAMP, \ref ENET_TIMESYNC_IOCTL_GET_ETH_TX_TIMESTAMP). Appropriate apis are defined for calling these IOCTLs and enet-lld has implementation for the IOCTLs to execute the desired feature.
- This example also illustrates on registering for CPTS event notification by using IOCTL ref\ CPSW_CPTS_IOCTL_REGISTER_STACK .
- Moreover, the example also has a demo of 'PTP Ordinary Clock' using a combined Timesync PTP stack using P2P delay mechanism.

- Please refer \ref ENET_MOD_TIMESYNC for details on all other IOCTLs supported by ENET-LLD for Timesync.

\cond SOC_AM64X || SOC_AM243X|| SOC_AM263X|| SOC_AM273X || SOC_AWR294X

On @VAR_SOC_NAME, we can do ethernet based communication using CPSW HW mechanism

  - CPSW is a standard ethernet switch + port HW
  - It uses ethernet driver underneath with LwIP TCP/IP networking stack

\endcond


\cond SOC_AM263X || SOC_AM273X || SOC_AWR294X

# Configuration Parameters

1.	Opening TX 0 and TX 1 channels using EnetAppUtils_openTxCh () api by passing channel number as a parameter to this api.
2.	Opening RX channels 0 and 1 using EnetAppUtils_openRxCh () api by passing channel number as a parameter to this api.
3.	Separate RX task and RX packet callback need to be implemented for each channel creation. Existing flows for channels can be referred.
4.	Set the vlanLType1 field of macport stats in EnetApp_setPortTsEventPrms () api to 0x8100, to make CPTS aware of VLAN tagging enabled for packets.
5.  The CPDMA supports upto 8 TX/RX channels, but by default applications has only create 3 TX/RX channels and this can be changed by modifying the macros ENET_CFG_CPDMA_CPSW_MAX_TX_CH/ENET_CFG_CPDMA_CPSW_MAX_RX_CH in enet_cfg.h file.
\cond SOC_AM273X || SOC_AWR294X
6.  On @VAR_SOC_NAME, Multiple CPDMA channels open is necessary to receive packets with different VLAN priority. Else, CPDMA cannot forward any packet with priority non-zero to R5F core.
\endcond
## Configuring the Packeth tool to send vlan tagged packets

In packeth tool select the 802.1q field to configure vlan parameters(priority).

  \imageStyle{packeth_setup.png,width:40%}
  \image html packeth_setup.png Packeth tool configuration for 802.1q vlan tag

# Application Functionality

  \imageStyle{multi_channel_app_functionality.png,width:40%}
  \image html multi_channel_app_functionality.png Application Functionality

We configure Packeth tool with 802.1Q (PCP) field with 0 or 1 to send either L2 packets or PTP packets. CPSW peripheral maps the value of PCP field to appropriate DMA channel queue for the receiving packets.

# Application Flow

  \imageStyle{multi_channel_app_flow.png,width:40%}
  \image html multi_channel_app_flow.png Application Flow

The Multichannel channel App on start of its main task initializes Enet driver, memory and queues initialization. Followed by this, the app opens the CPSW peripheral and creates two pairs of TX and RX channels:
1. L2 echo server
2. PTP Timestamping

Each RX channel will have a separate OS task to process each traffic type separately. Additionally, the test enables timestamping of PTP packets.
The Multichannel App continues to run until user decides to terminate the app by pressing ‘x’ from the App menu.

\endcond

\cond SOC_AM263X

# Channel Overriding
To override the channel mapping, we can use ALE classifier along with CPDMA_CONTROL.
	- The example shows the usage of ALE clasfier where we are creating a classfier based on Ethertype and routing the matched traffic to RX channel 1.
	- CPDMA provides channel override feature using the the thost_ch_override bit in CPDMA_CONTROL register.
	- When set, the RX channel is overridden with the ALE classfication match value. This value is what we set in ALE Classfier as threadId.
	- We need to set the default channel when we are doing overriding which will handle unclassified traffic, this is passed as a parameter to EnetApp_setCpswAleClassifier() api. The application sets RX channel 0 to be the default channel for handling unclassified traffic.
	- We need to set the enChOverrideFlag Flag in the application if we want to use the channel overriding feature. This is done in EnetApp_open() api.

\endcond

# Supported Combinations

\cond SOC_AM64X

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0_freertos r5fss0-1_freertos
 Toolchain      | ti-arm-clang
 Board          | @VAR_BOARD_NAME_LOWER
 Example folder | examples/networking/enet_layer2_multi_channel/V0

\endcond

\cond SOC_AM243X

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0_freertos r5fss0-1_freertos
 Toolchain      | ti-arm-clang
 Boards         | @VAR_BOARD_NAME_LOWER, @VAR_LP_BOARD_NAME_LOWER
 Example folder | examples/networking/enet_layer2_multi_channel/V0

\endcond

\cond SOC_AM263X

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0_freertos r5fss0-1_freertos
 Toolchain      | ti-arm-clang
 Boards         | @VAR_BOARD_NAME_LOWER
 Example folder | examples/networking/enet_layer2_multi_channel/V1

\endcond

\cond SOC_AM273X || SOC_AWR294X

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0_freertos r5fss0-1_freertos
 Toolchain      | ti-arm-clang
 Board          | @VAR_BOARD_NAME_LOWER
 Example folder | examples/networking/enet_layer2_multi_channel/V1

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

\note AM243X-LP has two ethernet Ports which can be configured as both CPSW/ICSS ports.

#### For CPSW based examples

- Connect a ethernet cable to the AM243X-LP from host PC as shown below

  \imageStyle{am243x_lp_lwip_example_00.png,width:30%}
  \image html am243x_lp_lwip_example_00.png Ethernet cable for CPSW based ethernet

\endcond


## Run the example

\attention If you need to reload and run again, a CPU power-cycle is MUST

- Launch a CCS debug session and run the example executable, see \ref CCS_LAUNCH_PAGE
- You will see logs in the UART terminal as shown in the next section.
\cond SOC_AM64X || SOC_AM243X
- We can start sending packets with Multicast addresses (01:80:C2:00:00:0E) PTP frames (Ethertype 0x88F7U) from Colasoft Pkt Builder or packETH tool and capture the packets in Wireshark. Packets must be sent without 802.1Q VLAN tag enabled to the application.
\endcond
\cond SOC_AM263X || SOC_AM273X || SOC_AWR294X
- We can start sending packets with Multicast addresses (01:80:C2:00:00:0E) PTP frames (Ethertype 0x88F7U) from Colasoft Pkt Builder or packETH tool and capture the packets in Wireshark. Packets must be sent with 802.1Q VLAN tag enabled to the application.
\endcond
- By selecting 't' option from the menu we can toggle the timestamp printing on the Terminal.

## Sample output for Multi-Channel example

\code

==========================
      Multi-Channel Test
==========================
Init Enet's OSAL and utils to use defaults
Init memory utils
Create clock and task for periodic tick
Create periodic tick task
Create periodic tick clock
Open Main UDMA driver

Init all peripheral clocks
----------------------------------------------
Enabling clocks!

Init all configs
----------------------------------------------
cpsw-3g: init config

Create RX tasks
----------------------------------------------
cpsw-3g: Create RX task

Open all peripherals
----------------------------------------------
cpsw-3g: Open enet

Attach core id 1 on all peripherals
----------------------------------------------
cpsw-3g: Attach core
cpsw-3g: Open port 1
cpsw-3g: Open port 1 link
cpsw-3g: Open DMA
initQs() txFreePktInfoQ initialized with 16 pkts
cpsw-3g: Waiting for link up...
MAC Port 1: link up
cpsw-3g: Port 1 link is up
cpsw-3g: MAC port addr: f4:84:4c:f9:88:c4

Enet Multi-Channel Menu:
 'c'  -  GetCurrentTime
 't'  -  Toggle Printing timestamps
 's'  -  Print statistics
 'r'  -  Reset statistics
 'm'  -  Show allocated MAC addresses
 'x'  -  Stop the test

t

Enable Timestamp Printing
RX PTP time is : 13219154567
TX PTP time is : 13219204642
RX PTP time is : 14222561337
TX PTP time is : 14222608577
RX PTP time is : 15237477912
TX PTP time is : 15237519097
RX PTP time is : 16238653702
TX PTP time is : 16238698897

\endcode

# See Also

\ref NETWORKING
