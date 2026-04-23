# Enet CPSW Ether-Ring TSN Example {#EXAMPLES_ENET_CPSW_ETHERRING}

[TOC]

# Introduction
This example application illustrates the configuration and usage of Ether-Ring topology with 4 nodes connected in loop where each node works as central compute, zone left,
zone right and zone tail.
The example application enables both MAC ports by default and each node is connected to it's neighbouring node in a loop. This application uses Concurrence execution of gPTP IEEE 802.1AS stack and IEEE 802.1 Qbv (EST) configuration and Ether-ring to support Packet Duplication, Ring Termination in Hardware and Duplicate Rejection in Software.

The example application opens two DMA TX channels and two DMA RX channels.  The TX0 and RX0 DMA channel will be used by the application to send the Class-A and Class-D stream packets via Ether-Ring Driver. The TX0 and RX0 DMA channel will be used for GPTP.

The application will also open the both MAC port and will wait until the port gets
link up with neighbouring node. The application waits for GPTP synchronization to happen and then applies the EST Schedule on both MAC ports. 
The application uses \ref ENET_MOD_TAS API for EST configuration ,\ref ENET_CPSW_TSN_GPTP for gPTP and \ref ENET_DMA_API

API for packet transmission.

# Ether-Ring overview
Please refer **[here](\ref ETHERRING_OVERVIEW)** for Ether-Ring overview

# Configuration Parameters

Typically, the application's parameters that a developer may want to change are:

- **Number of Nodes**.  By default the example application configures the etherring for 4 Nodes,
but the user can choose to update the number of nodes in etherring using
  `NODES_COUNT_IN_ETHERRING = 4` in `config.h`.
- **Number of Class A Streams**.  By default the number of Class A Streams are configured as 1 for each Node.
User can update the configuration bye changes `NUM_CLASSA_STREAMS` in `config.h`.
- **Class A Payload Length**.  Class A payload length is configured as 1000 be default and user can update
`CLASSA_PAYLOAD_LENGTH` in `config.h` .
- **Class D Payload Length**.  Class D payload length is configured as 1450 be default and user can update
`CLASSD_PAYLOAD_LENGTH` in `config.h` .



# Application Functionality

## Ether-Ring Application Flow

The Application creates Class-A, Class-D stream RTOS tasks which creates the 802.1Q Ethernet Multicast packet with 
stream configuration and calls "EtherRing_submitTxPktQ" API from Ether-Ring Driver. "EtherRing_submitTxPktQ" creates a custom 
Custom header(4 bytes) which consists of CB-like Ethertype(0x88B5), unique sequence ID for each Ethernet packet and adds the custom Header to the packet sent from application without modifying the payload using zero-copy (using Transmit Scatter-Gather feature of HW). The packet is submitted to DMA and CPSW duplicates the multicast packet and sent it to both MAC ports.

  \image html etherring_tx.png Ether-Ring Transmit Flow  width=60%

 When the packets are received RX DMA ISR will be triggered and "EtherRing_retrieveRxPktQ"
is called from RTOS Rx application task. "EtherRing_retrieveRxPktQ" calls the DMA Retrieve API to retrieve the packets and Duplicate Packet Rejection is done using {SrcMacAddr, Sequence Number} based Look-up Table. Original stream packets found using look-up table are sent to Application and Duplicate packets received are submitted back to the DMA.

  \image html etherring_rx.png Ether-Ring Receive Flow  width=60%
## EST Schedule

This EST schedule is 1 msec long and is composed of 8 intervals, each with
a gate mask that enables transmission of 2,3 are configured for Class A and Class D streams.

<CENTER>
<table>
<caption>EST schedule parameters</caption>
<tr><th>Parameter <th>Time Value
<tr><td> BaseTime   <td> Current time + `MAX_BASE_TIME_US`
<tr><td> CycleTime  <td> 1 msec
</table>

<table>
<caption>Gate control list</caption>
<tr><th>Gate Control <th>Time Interval
<tr><td> `oCCCooCC`  <td> 125 usecs
<tr><td> `oCCCCCCC`  <td> 125 usecs
<tr><td> `oCCCCCCC`  <td> 125 usecs
<tr><td> `oCCCCCCC`  <td> 125 usecs
<tr><td> `oCCCCCCC`  <td> 125 usecs
<tr><td> `oCCCCCCC`  <td> 125 usecs
<tr><td> `oCCCCCCC`  <td> 125 usecs
<tr><td> `oCCCCCCC`  <td> 125 usecs
</table>
</CENTER>


The *priority* being referred to in the EST schedule described above corresponds
to the CPSW destination port hardware *switch priority*.  For VLAN tagged packets
that ingress through CPSW host port, the hardware switch priority can be determined
by either the CPPI channel number (`P0_RX_REMAP_VLAN = 0`) or the packet priority
value from the VLAN tag (`P0_RX_REMAP_VLAN = 1`).

\code{.c}
/* Hardware switch priority is taken from packet's PCP or DSCP */
hostPortCfg->rxVlanRemapEn     = true;
hostPortCfg->rxDscpIPv4RemapEn = true;
hostPortCfg->rxDscpIPv6RemapEn = true;
\endcode

## Application Startup Flow - EST, GENF, and Traffic Generation

The application follows a coordinated startup sequence using GPTP synchronization, EST scheduling, and hardware pulse generation:

### Phase 1: Initialization & GPTP Synchronization
All nodes initialize networking stack and wait for GPTP (802.1AS) clock synchronization:
- GPTP daemon exchanges timing messages across ring topology
- Nodes transition through states: INITIALIZING → SYNCING → LOCKED
- Once synchronized, all nodes share common wall-clock time reference

### Phase 2: EST Gate Scheduling (gBaseTime Reference)
After GPTP reaches LOCKED state, the EST schedule is applied:
- EST gate control lists applied on both CPSW MAC ports
- Gates scheduled starting at `gBaseTime` (default: 30,000,000,000 ns = 30 seconds)
- Gate windows define when each priority queue (Class A, Class D, Best Effort) can transmit
- All nodes' gates open/close at same wall-clock time due to GPTP synchronization
- See EST schedule table above for gate mask timing (125μs intervals)

### Phase 3: GENF0 Enable (Immediate)
- GENF0 (General Function 0) configured for 50MHz reference pulse (4 clocks = 80ns)
- Starts immediately after EST configuration
- Provides general timing reference used by EST for gate transition synchronization

### Phase 4: GENF1 Enable (gBaseTime - 20μs)
- GENF1 (General Function 1) configured for 125μs pulse interval (25000*2 clocks)
- **Starts 20 microseconds BEFORE gBaseTime** to pre-trigger traffic generation
- **Reason**: Allows packet preparation and transmission to complete before EST gate closes
- Ensures packets meet scheduled transmission deadline and gate window efficiency
- Generates periodic pulses that trigger stream task wake-ups

### Phase 5: RTI Timer & Periodic Traffic Generation
- Real-Time Interrupt (RTI) timer created with GENF1 pulse as clock source
- GENF1 interrupt fires every 125μs → wakes stream generation tasks
- Stream tasks for each enabled stream:
  - Allocate packets from free queue
  - Fill packet with StreamId (sequence number at offset 30 bytes) + payload
  - Submit to CPSW TX queue (respects EST gate status)
  - **Class A streams**: up to 1000 packets per test cycle
  - **Class D streams**: up to 125 packets per test cycle (lower priority, reduced load)

### Phase 6: RX Processing & EtherRing Duplicate Detection
- RX task collects arriving packets from network
- EtherRing module performs sequence-based duplicate detection:
  - Tracks StreamId (sequence number) in 64-packet sliding window per stream
  - **Rejects exact duplicates** from ring redundancy artifacts
  - **Accepts out-of-order packets** within history window
- RX timestamps stored for jitter and latency analysis
- Final output prints RX/TX packet counts and latency statistics

### Application Startup Sequence

<CENTER>
<table>
<caption>Startup Phase Timeline</caption>
<tr><th>Phase <th>Timing <th>Event Description
<tr><td>1 <td>t=0 <td>GPTP Synchronization begins
<tr><td>1 <td>t=~3s <td>GPTP reaches LOCKED state (all nodes synchronized)
<tr><td>2 <td>t=~5s <td>EST gate control lists applied to both MAC ports
<tr><td>2 <td>t=~5s <td>EST schedule reference set to gBaseTime (30,000,000,000 ns)
<tr><td>3 <td>t=~5s <td>GENF0 enabled (50MHz, immediate operation)
<tr><td>4 <td>t=(gBaseTime - 20μs) <td>GENF1 enabled (125μs pulse interval begins)
<tr><td>4 <td>t=(gBaseTime - 20μs) <td>RTI Timer created with GENF1 as clock source
<tr><td>5 <td>t=(gBaseTime - 20μs) <td>Stream TX tasks begin execution
<tr><td>5 <td>t=gBaseTime <td>EST gates OPEN for Class A and Class D queues
<tr><td>5 <td>t=gBaseTime + 20μs <td>Packets transmitted within EST gate window
<tr><td>5 <td>t=(gBaseTime + interval) <td>EST gates CLOSE, packets complete transmission
<tr><td>6 <td>Continuous <td>RX task receives packets and performs EtherRing deduplication
<tr><td>6 <td>Continuous <td>Timestamps logged for latency and jitter analysis
</table>
</CENTER>

## RX Host Time stamping
RX Host time stamping is enabled from syscfg to measure the jitter in latency for the Received packets and 
timestamp value is stored in "rxPktTs" for each received packet(pktInfo). The timestamp values are stored in Ether-Ring driver for Jitter measurement.

## ALE Configuration
- The Default Multicast ALE Entry for PTP has been updated to send PTP Packets only to the Host port. The port mask has been updated to `0x1`
and ALE Entry is added.
- Vlan entry for default port vlan is updated to enable `forceUntaggedEgressMask` for all ports so that vlan tag is not added to ptp packets 
when ALE is in vlanAware mode.

## Ether-Ring Start Menu

The following helper options are also provided in the application's start menu to configure the each node uniquely in Etherring:

\code
0 - Central Compute Node
1 - Zone Left Node
2 - Zone Right Node
3 - Zone Tail Node
Enter the nodeId : 
\endcode

# Supported Combinations
\cond SOC_AM263X || SOC_AM263PX
 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0_freertos
 Toolchain      | ti-arm-clang
 Boards         | @VAR_LP_BOARD_NAME_LOWER
 Example folder | source/networking/enet/core/examples/ether_ring
 \endcond

# Packet pool configuration
To change packet pool configuration from syscfg, please refer to \ref PACKETPOOL_CONFIG_TOP

# Steps to Run the Example

## Build the example

- When using CCS projects to build, import the CCS project for the required combination
  and build it using the CCS project menu (see \ref CCS_PROJECTS_PAGE).
- When using makefiles to build, note the required combination and build using
  make command (see \ref MAKEFILE_BUILD_PAGE)

## HW Setup

Make sure you have setup the EVM with cable connections as shown in below Diagram.
In addition, follow the steps in the next section.

\image html etherring_evm.png Ether-Ring EVM Connections width=30%

\note Ethernet cable must be connected to both the ports in loop for the example application to run with expected results.

## Run the example

### Basic Test

The simplest test that one can run with the ETHERRING example consists of:
- Configuring the each node uniqly in the Ether-ring Network and wait for PTP synchronization to happen for all the Nodes. 
- Enabling EST and programming a custom EST schedule on the both MAC ports once PTP is synchronized.
- Sending Class A and Class D stream packets with different priorities to the adjacent node in clock-wise and anti clock-wise direction in the loop. 

The steps to run this test are:

- Connect the MAC port as described in **HW Setup** section and configure the Nodes using UART Terminal.
- Launch a CCS debug session and run the example executable, see \ref CCS_LAUNCH_PAGE.
- Application logs such as cycle time, time intervals and gate masks will be printed
  in the serial console.  Sample logs are shown in the next section.
- Configure the each node as mentioned in **Ether-Ring Start Menu** section using UART Terminal around the same time in all the Nodes to start the application and wait for `----------ETHERRING DEMONSTRATION COMPLETED----------` to be printed on UART Terminal.


## Sample output

\code
==========================
       EtherRing App      
==========================
0 - Central Compute Node
1 - Zone Left Node
2 - Zone Right Node
3 - Zone Tail Node
Enter the nodeId : 
0
start to open driver.
EnetAppUtils_reduceCoreMacAllocation: Reduced Mac Address Allocation for CoreId:0 From 4 To 2 

Init all configs
----------------------------------------------
sitara-cpsw: init config
Open MAC port 1
EnetPhy_bindDriver:1843 
Open MAC port 2
EnetPhy_bindDriver:1843 
PHY 3 is alive
PHY 12 is alive
sitara-cpsw: Create RX task for regular traffic 
initQs() txFreePktInfoQ initialized with 48 pkts
MAC port addr: 70:ff:76:1f:6a:46
sitara-cpsw: default RX flow started
Waiting for GPTP ready!!
Start: uniconf_task
EnetApp_uniconfTask: dbname: NULL
EnetApp_gptpYangConfig:domain=0
EnetApp_gptpNonYangConfig:XL4_EXTMOD_XL4GPTP_SINGLE_CLOCK_MODE=1
EnetApp_gptpNonYangConfig:XL4_EXTMOD_XL4GPTP_USE_HW_PHASE_ADJUSTMENT=1
EnetApp_gptpNonYangConfig:XL4_EXTMOD_XL4GPTP_CLOCK_COMPUTE_INTERVAL_MSEC=100
EnetApp_gptpNonYangConfig:XL4_EXTMOD_XL4GPTP_FREQ_OFFSET_IIR_ALPHA_START_VALUE=1
EnetApp_gptpNonYangConfig:XL4_EXTMOD_XL4GPTP_FREQ_OFFSET_IIR_ALPHA_STABLE_VALUE=4
EnetApp_gptpNonYangConfig:XL4_EXTMOD_XL4GPTP_PHASE_OFFSET_IIR_ALPHA_START_VALUE=1
EnetApp_gptpNonYangConfig:XL4_EXTMOD_XL4GPTP_PHASE_OFFSET_IIR_ALPHA_STABLE_VALUE=4
EnetApp_gptpNonYangConfig:XL4_EXTMOD_XL4GPTP_MAX_DOMAIN_NUMBER=1
Start: gptp2d_task
portIdx=0 netdev tilld0

Current port-state: 3 
portIdx=1 netdev tilld1

GPTP ready!!
Current port-state: 3 
Waiting errfor PTP clock to be synchronized!
Start: melcoApp_EST
EnetApp_initTsn:TSN app start done!
ClassA stream count:1
ClassD stream count:1
Waiting for GPTP ready!!
GPTP ready!!
Waiting for GPTP ready!!
GPTP ready!!
Logger_task: started
unibase-1.1.4
INF:cbase:tilld0: has mac: 70:FF:76:1F:6A:46
INF:cbase:tilld1: has mac: 00:00:00:00:00:00
INF:cbase:cb_lld_task_create: uniconf_task stack_size=8192
INF:uconf:simpledb_open:no data is imported
INF:uconf:get_next_nameid:a new mod=xl4-data, enum=0
INF:uconf:get_next_nameid:a new mod=xl4-extmod, enum=1
INF:uconf:uc_hwal_open:
INF:cbase:cb_rawsock_open:combase-1.1.3
INF:cbase:cb_rawsock_open:dmaTxChId=-1 numRxChannels=1 dmaRxChId=-1 nTxPkts=0 nRxPkts=0 pktSize=0
INF:uconf:get_next_nameid:a new mod=ietf-interfaces, enum=2
INF:uconf:get_next_nameid:a new mod=ieee1588-ptp-tt, enum=3
INF:uconf:get_next_nameid:a new mod=ieee802-dot1q-bridge, enum=4
INF:uconf:get_next_nameid:a new mod=excelfore-tsn-remote, enum=5
INF:uconf:get_next_nameid:a new mod=excelfore-netconf-server, enum=6
INF:uconf:get_next_nameid:a new mod=ietf-netconf-monitoring, enum=7
INF:uconf:get_next_nameid:a new mod=ietf-yang-library, enum=8
INF:uconf:get_next_nameid:a new mod=ieee802-dot1ab-lldp, enum=9
INF:uconf:get_nWRN:gptp:gptpgcfg_link_check:can't read speed
portIdx=0 netdev tilld0

Current port-state: 3 
portIdx=1 netdev tilld1

Current port-state: 3 
Waiting errfor PTP clock to be synchronized!
INF:uconf:create_semname_with_dbname:null dbname is specified.
INF:uconf:create_semname_with_dbname:null dbname is specified.
portIdx=0 netdev tilld0

Current port-state: 3 
portIdx=1 netdev tilld1

Current port-state: 3 
Waiting errfor PTP clock to be synchronized!
INF:uconf:create_semname_with_dbname:null dbname is specified.
INF:uconf:create_semname_with_dbname:null dbname is specified.
portIdx=0 netdev tilld0

Current port-state: 3 
portIdx=1 netdev tilld1

Current port-state: 3 
Waiting errfor PTP clock to be synchronized!
INF:uconf:create_semname_with_dbname:null dbname is specified.
INF:uconf:create_semname_with_dbname:null dbname is specified.
portIdx=0 netdev tilld0

Current port-state: 3 
portIdx=1 netdev tilld1

Current port-state: 3 
Waiting errfor PTP clock to be synchronized!
INF:uconf:create_semname_with_dbname:null dbname is specified.
INF:uconf:create_semname_with_dbname:null dbname is specified.
Cpsw_handleLinkUp:1645 
MAC Port 1: link up
INF:cbase:cbl_query_response:tilld0: link UP, speed=1000, duplex=1 !!!!
portIdx=0 netdev tilld0

Current port-state: 3 
portIdx=1 netdev tilld1

Current port-state: 3 
Waiting errfor PTP clock to be synchronized!
INF:uconf:create_semname_with_dbname:null dbname is specified.
INF:uconf:create_semname_with_dbname:null dbname is specified.
WRN:gptp:000013-500229:waiting_for_pdelay_interval_timer_proc:portIndex=1, sourcePortIdentity=34:08:E1:FF:FE:84:DB:CF, thisClock=70:FF:76:FF:FE:1F:6A:46, neighborPropDelay=392
Cpsw_handleLinkUp:1645 
MAC Port 2: link up
INF:cbase:cbl_query_response:tilld1: link UP, speed=1000, duplex=1 !!!!
portIdx=0 netdev tilld0

Current port-state: 3 
portIdx=1 netdev tilld1

Current port-state: 3 
Waiting errfor PTP clock to be synchronized!
INF:uconf:create_semname_with_dbname:null dbname is specified.
INF:uconf:create_semname_with_dbname:null dbname is specified.
WRN:gptp:000014-625027:waiting_for_pdelay_interval_timer_proc:portIndex=2, sourcePortIdentity=34:08:E1:FF:FE:84:DD:31, thisClock=70:FF:76:FF:FE:1F:6A:46, neighborPropDelay=392
portIdx=0 netdev tilld0

syncFlag: 0
 
Current port-state: 9 
EnetApp_enableCBS
INF:uconf:create_semname_with_dbname:null dbname is specified.
INF:uconf:create_semname_with_dbname:null dbname is specified.
INF:uconf:ydbi_get_item_ifk4vk1:no data
INF:uconf:get_queue_map_params:netdev=tilld0, num_tc=0
The following AdminList param will be configured for EST:
GateMask[7..0]=oCCCoCCC (0x88), start=0 ns, end=19999 ns, dur=20000 ns
GateMask[7..0]=oCCCCoCC (0x84), start=20000 ns, end=124999 ns, dur=105000 ns
GateMask[7..0]=oCCCoCCC (0x88), start=125000 ns, end=144999 ns, dur=20000 ns
GateMask[7..0]=oCCCCCCC (0x80), start=145000 ns, end=249999 ns, dur=105000 ns
GateMask[7..0]=oCCCoCCC (0x88), start=250000 ns, end=269999 ns, dur=20000 ns
GateMask[7..0]=oCCCCCCC (0x80), start=270000 ns, end=374999 ns, dur=105000 ns
GateMask[7..0]=oCCCoCCC (0x88), start=375000 ns, end=394999 ns, dur=20000 ns
GateMask[7..0]=oCCCCCCC (0x80), start=395000 ns, end=499999 ns, dur=105000 ns
GateMask[7..0]=oCCCoCCC (0x88), start=500000 ns, end=519999 ns, dur=20000 ns
GateMask[7..0]=oCCCCCCC (0x80), start=520000 ns, end=624999 ns, dur=105000 ns
GateMask[7..0]=oCCCoCCC (0x88), start=625000 ns, end=644999 ns, dur=20000 ns
GateMask[7..0]=oCCCCCCC (0x80), start=645000 ns, end=749999 ns, dur=105000 ns
GateMask[7..0]=oCCCoCCC (0x88), start=750000 ns, end=769999 ns, dur=20000 ns
GateMask[7..0]=oCCCCCCC (0x80), start=770000 ns, end=874999 ns, dur=105000 ns
GateMask[7..0]=oCCCoCCC (0x88), start=875000 ns, end=894999 ns, dur=20000 ns
GateMask[7..0]=oCCCCCCC (0x80), start=895000 ns, end=999999 ns, dur=105000 ns
Base time=20000000000ns,Cycle time=1000000ns
Set admin control list succesfully
EnetApp_enableCBS
INF:uconf:create_semname_with_dbname:null dbname is specified.
INF:uconf:ydbi_get_item_ifk4vk1:no data
INF:uconf:get_queue_map_params:netdev=tilld1, num_tc=0
INF:uconf:get_queue_map_params:netdev=tilld0, num_tc=8
INF:cbase:TAS state is set to 2
INF:cbase:TAS operational list status updated:
INF:cbase:TAS state is set to 1
INF:cbase:Successfully configure TAS 
The following AdminList param will be configured for EST:
GateMask[7..0]=oCCCoCCC (0x88), start=0 ns, end=19999 ns, dur=20000 ns
GateMask[7..0]=oCCCCoCC (0x84), start=20000 ns, end=124999 ns, dur=105000 ns
GateMask[7..0]=oCCCoCCC (0x88), start=125000 ns, end=144999 ns, dur=20000 ns
GateMask[7..0]=oCCCCCCC (0x80), start=145000 ns, end=249999 ns, dur=105000 ns
GateMask[7..0]=oCCCoCCC (0x88), start=250000 ns, end=269999 ns, dur=20000 ns
GateMask[7..0]=oCCCCCCC (0x80), start=270000 ns, end=374999 ns, dur=105000 ns
GateMask[7..0]=oCCCoCCC (0x88), start=375000 ns, end=394999 ns, dur=20000 ns
GateMask[7..0]=oCCCCCCC (0x80), start=395000 ns, end=499999 ns, dur=105000 ns
GateMask[7..0]=oCCCoCCC (0x88), start=500000 ns, end=519999 ns, dur=20000 ns
GateMask[7..0]=oCCCCCCC (0x80), start=520000 ns, end=624999 ns, dur=105000 ns
GateMask[7..0]=oCCCoCCC (0x88), start=625000 ns, end=644999 ns, dur=20000 ns
GateMask[7..0]=oCCCCCCC (0x80), start=645000 ns, end=749999 ns, dur=105000 ns
GateMask[7..0]=oCCCoCCC (0x88), start=750000 ns, end=769999 ns, dur=20000 ns
GateMask[7..0]=oCCCCCCC (0x80), start=770000 ns, end=874999 ns, dur=105000 ns
GateMask[7..0]=oCCCoCCC (0x88), start=875000 ns, end=894999 ns, dur=20000 ns
GateMask[7..0]=oCCCCCCC (0x80), start=895000 ns, end=999999 ns, dur=105000 ns
Base time=20000000000ns,Cycle time=1000000ns
Set admin control list succesfully
INF:uconf:get_queue_map_params:netdev=tilld1, num_tc=8
INF:cbase:TAS state is set to 2
INF:cbase:TAS operational list status updated:
INF:cbase:TAS state is set to 1
INF:cbase:Successfully configure TAS 
WRN:gptp:000018-746101:md_pdelay_resp_sm_recv_req: port=2, expected SeqID=10723, but received=10724
Starting the Stream Traffic
RxTs and CurrentTs values stored

----------ETHERRING DEMONSTRATION COMPLETED----------
[RXTS]: 29954385101
[RXTS]: 29954510117
[RXTS]: 29954635123
[RXTS]: 29954760134
[RXTS]: 29954885145
[RXTS]: 29955010066
[RXTS]: 29955135082
[RXTS]: 29955260083
[RXTS]: 29955385094
[RXTS]: 29955510120
[RXTS]: 29955635121
[RXTS]: 29955760127
[RXTS]: 29955885143
[RXTS]: 29956010069
[RXTS]: 29956135075
[LAT]: 31248385523
[LAT]: 31248509968
[LAT]: 31248635684
[LAT]: 31248760014
[LAT]: 31248885115
[LAT]: 31249010280
[LAT]: 31249185376
[LAT]: 31249260041
[LAT]: 31249385041
[LAT]: 31249510402
[LAT]: 31249675958
[LAT]: 31249759993
[LAT]: 31249885083
[LAT]: 31250010974
[RX] Packet Count of stream0: 30000 
[TX] Packet Count of stream0: 30000 


\endcode

## Troubleshooting issues

\cond SOC_AM263PX
- To ensure compatibility with CPSW RGMII2 port on @VAR_BOARD_NAME_LOWER board, make sure that SW-15(switch) should be set to `LOW` for board versions(E2/A/B versions). This configuration ensures seamless out-of-box support and functionality with the CPSW RGMII2 port on the specified versions.
\endcond

# See Also

\ref NETWORKING
