# Ethernet LLDP Example {#EXAMPLES_ENET_CPSW_LLDP}

[TOC]

# Introduction
This ethernet LLDP example illustrates the usage of LLDP IEEE 802.1AB stack with CPSW peripheral.

In this example, the DUT MAC port is connected to a neighbor PC which also have opensource lldp agent.
Yang based configuration is also supported. Currently File System is not supported, will be added in future releases.

See also :\ref ENET_CPSW_LLDP

# Supported Combinations

\cond SOC_AM64X

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0_freertos
 Toolchain      | ti-arm-clang
 Board          | @VAR_BOARD_NAME_LOWER
 Example folder | examples/networking/tsn/lldp_cpsw_app

\endcond

\cond SOC_AM243X

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0_freertos
 Toolchain      | ti-arm-clang
 Boards         | @VAR_BOARD_NAME_LOWER, @VAR_LP_BOARD_NAME_LOWER
 Example folder | examples/networking/tsn/lldp_cpsw_app

\endcond

\cond SOC_AM263X

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0_freertos
 Toolchain      | ti-arm-clang
 Boards         | @VAR_BOARD_NAME_LOWER, @VAR_LP_BOARD_NAME_LOWER
 Example folder | examples/networking/tsn/lldp_cpsw_app

\endcond

\cond SOC_AM273X

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0_freertos
 Toolchain      | ti-arm-clang
 Boards         | @VAR_BOARD_NAME_LOWER
 Example folder | examples/networking/tsn/lldp_cpsw_app

\endcond

## Prerequisites
- EVM Board
- Cat6 ethernet cable
- PC with Linux Ubuntu OS with wireshark, docker installed
- The LLDP opensource ``https://github.com/lldpd/lldpd`` can be installed follow the guideline. But a quick start can be using docker:
\code
docker run --rm --net=host --uts=host \
           -v /etc/os-release:/etc/os-release \
           --cap-add=NET_RAW --cap-add=NET_ADMIN \
           --name lldpd \
           ghcr.io/lldpd/lldpd:latest
\endcode

## Build the example

Refer \ref EXAMPLES_ENET_CPSW_TSN_GPTP to build the lldp example.

## HW Setup

Refer \ref EXAMPLES_ENET_CPSW_TSN_GPTP for HW Setup.

## Create a network between EVM and host PC
EVM and PC has to connected directly as shown below using CAT6 or CAT5 cable
  \imageStyle{gptp_topology_evm_pc.png,width:30%}
  \image html gptp_topology_evm_pc.png Local network between PC and EVM

PORT1 instead of PORT0 on EVM can be used as well.

## Run the example

\attention If you need to reload and run again, a CPU power-cycle is MUST

Refer :\ref ENET_CPSW_LLDP for how to configure LLDP parameters.

## LLDP verification

### Verification #1 LLDPDU Tx interval and LLDPDU content (per one dest-mac-address)
With default setting
\code
{"message-tx-interval" , "30"},
\endcode

Capture LLDPDU via wireshark and confirm the TX interval is around 30s
\imageStyle{lldp_tx_interval.png,width:100%}
\image html lldp_tx_interval.png LLDP TX Interval

Capture LLDPDU via wireshark and confirm the LLDPDU content is match with configuration params in ``lldp_init.c``
\imageStyle{lldpdu_content.png,width:60%}
\image html lldpdu_content.png LLDPDU Content

### Verification #2 Fast TX in case of new neighbor exist (per one dest-mac-address)
On Linux PC, start opensource LLDP application by:
\code
docker run --rm --net=host --uts=host \
           -v /etc/os-release:/etc/os-release \
           --cap-add=NET_RAW --cap-add=NET_ADMIN \
           --name lldpd \
           ghcr.io/lldpd/lldpd:latest
\endcode

With default setting
\code
{"message-fast-tx" , "1"},
{"tx-fast-init" , "2"},
\endcode

Capture LLDPDU via wireshark and confirm after receive Linux PC LLDPDU, the DUT process fast tx flow by one TX LLDPDU immediately,
and follow up by two (tx-fast-init) more LLDPDUs, with one second interval (message-fast-tx).

\imageStyle{fast_tx_flow.png,width:100%}
\image html fast_tx_flow.png LLDPDU Fast TX Flow

After receive next LLDPDU from Linux PC, no more fast tx flow happened on DUT side.

### Verification #3 Delete neighbor info in case of Receiving LLDP shutdown (per one dest-mac-address)
Continue 2nd verification, then turn OFF LLDPDU by `Ctrl + C` in Linux PC docker command. Verify DUT can detect and delete correct neighbor info include Port and Chassis ID.

DUT log
\code
INF:lldp:collect_ttl_info: Recv SHUTDOWN LLDPDU TTL=[0]
INF:lldp:tilld0: Process shutdown for existed neighbor:
INF:lldp:chassis id: bc-5f-f4-73-e5-5a
INF:lldp:port id:    1c-fd-08-72-59-61
\endcode

User can rerun 2nd verification and confirm Fast TX flow happen again.

### Verification #4 Detected Neighbor info change (per one dest-mac-address)
On Linux PC side, change port description of corresponding port which is connect to DUT. In this example, enp3s0 is used.
\code
docker exec lldpd lldpcli configure port enp3s0 lldp portdescription xxxxxxx
\endcode

DUT log
\code
INF:lldp:is_remote_system_data_change: Detected port desc changed
INF:lldp:tilld0: Got updated for neighbor [TTL=120]:
INF:lldp:chassis id: bc-5f-f4-73-e5-5a
INF:lldp:port id:    1c-fd-08-72-59-61
INF:lldp:update_remote_system_data: changed port_desc [enp3s0-> xxxxxxx]

\endcode

### Verification #5 Multiple Destination MAC addresses
Configure 3 message-tx-interval values differently each other, like below:

\code
{
.dest_mac = "01-80-c2-00-00-0e",
    {"message-tx-interval" , "30"},
}

{
.dest_mac = "01-80-c2-00-00-03",
    {"message-tx-interval" , "20"},
}

{
.dest_mac = "01-80-c2-00-00-00",
    {"message-tx-interval" , "25"},
}
\endcode

Capture wireshark and verify the tx interval flow for:
- Nearest bridge 01-80-C2-00-00-0E (LLDP Multicast) is 30s
- Nearest non-TPMR bridge 01-80-C2-00-00-03 is 20s
- Nearest Customer Bridge (Spanning-tree-(for-bridges)_00) 01-80-C2-00-00-00 is 25s

\imageStyle{lldp_multiple_dest_mac.png,width:100%}
\image html lldp_multiple_dest_mac.png LLDP Multiple Destination MAC Addresses

Repeat verification #1 -> #4 (The LLDP opensource is using Nearest bridge 01-80-C2-00-00-0E as destination MAC address) and confirm only Nearest bridge Dest MAC address in DUT takes effect.

# See Also

\ref NETWORKING |
\ref ENET_CPSW_LLDP