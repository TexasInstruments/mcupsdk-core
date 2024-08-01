# Ethernet TSN ICSSG gPTP TimeTransmitter (gPTP Master) Example  {#EXAMPLES_ENET_ICSSG_TSN_GPTP_TT}

[TOC]

# Introduction
This ethernet TSN example illustrates the usage of gPTP IEEE 802.1AS stack with ICSSG peripheral, in gPTP End-Point time_transmitter mode (i.e. master mode).

However, the application used here supports all the below modes:
    - gPTP End-Point time_transmitter mode (i.e. master mode)
    - gPTP End-Point time_receiver mode (i.e. slave mode)
    - gPTP Bridge mode

In this example, ptp4l on connected Host PC is configured to force gPTP time_receiver mode, so that the DUT becomes time_transmitter (i.e. gPTP master) mode.
Yang based configuration is also supported. Currently File System is not supported, will be added in future releases.

In this example, We use dedicated Rx and Tx DMA channels for gPTP traffic.This example is supported in both ICSSG Switch and Dual Mac modes.
In ICSSG Dual Mac mode, Currently only Mac Port 1 is supported for gPTP.

Along with PTP traffic, application also handles non-PTP traffic in a separate RTOS task and DMA Channel. Received non-PTP packets are echoed back by the application.

See also :\ref ENET_CPSW_TSN_GPTP for gPTP stack documentation.

# Supported Combinations

\cond SOC_AM64X

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0_freertos
 Toolchain      | ti-arm-clang
 Board          | @VAR_BOARD_NAME_LOWER
 Example folder | examples/networking/tsn/gptp_icssg_app

\endcond

\cond SOC_AM243X

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0_freertos
 Toolchain      | ti-arm-clang
 Boards         | @VAR_BOARD_NAME_LOWER, @VAR_LP_BOARD_NAME_LOWER
 Example folder | examples/networking/tsn/gptp_icssg_app

\endcond

# Steps to Run the Example

## Prerequisites
- EVM Board
- Cat6 ethernet cable
- PC with Linux Ubuntu OS (or any PC running bash shell) and PTP capable network card
- Install `linuxptp`
\code
$ sudo apt-get install linuxptp
$ ptp4l -v
\endcode
- Configure linuxptp
\code
$ wget https://raw.githubusercontent.com/richardcochran/linuxptp/v4.3/configs/gPTP.cfg -O ~/gptp_config.cfg
\endcode
\attention Change the value of `priority1` in ~/gptp_config.cfg file to `255` to enforce PC to gPTP slave

\attention For some network cards, there is a bug with internal propagation delay. So, in those cases you might need to increase the `neighborPropDelayThresh` in ptp_config.cfg as below-

\code
$ cat ~/gptp_config.cfg
\#
\# 802.1AS example configuration containing those attributes which
\# differ from the defaults.  See the file, default.cfg, for the
/# complete list of available options.
/#
[global]
gmCapable               1
priority1               255
priority2               248
logAnnounceInterval     0
logSyncInterval         -3
syncReceiptTimeout      3
neighborPropDelayThresh 10000
min_neighbor_prop_delay -20000000
assume_two_step         1
path_trace_enabled      1
follow_up_info          1
transportSpecific       0x1
ptp_dst_mac             01:80:C2:00:00:0E
network_transport       L2
delay_mechanism         P2P

\endcode
- `priority1` in ~/gptp_config.cfg file can be changed to make the Linux PC master or slave. Lower the number higher the priority to become master.

## Build the example

- When using CCS projects to build, import the CCS project for the required combination
  and build it using the CCS project menu (see \ref CCS_PROJECTS_PAGE).
- When using makefiles to build, note the required combination and build using
  make command (see \ref MAKEFILE_BUILD_PAGE)

## HW Setup

\note Make sure you have setup the EVM with cable connections as shown here, \htmllink{@VAR_MCU_SDK_DOCS_PATH/EVM_SETUP_PAGE.html, EVM Setup}.
      In addition do below steps.

\cond SOC_AM64X

### AM64X-EVM

#### For ICSS based example

- Connect a ethernet cable to the EVM from host PC as shown below

  \imageStyle{am64x_evm_lwip_example_01.png,width:30%}
  \image html am64x_evm_lwip_example_01.png Ethernet cable for ICSS based ethernet

\endcond

\cond SOC_AM243X

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

\endcond

## Create a network between EVM and host PC
EVM and PC has to connected directly as shown below using CAT6 or CAT5 cable. If there is ethernet switch placed in between, make sure the switch is gPTP capable.
  \imageStyle{gptp_topology_evm_pc.png,width:30%}
  \image html gptp_topology_evm_pc.png Local network between PC and EVM
 
PORT1 instead of PORT0 on EVM can be used as well. Please note that EVM is in switch mode, both need to connected to two end-points to execute the example.

## Run the example
  
\attention If you need to reload and run again, a CPU power-cycle is MUST

- Execute the below command in PC terminal to start the gPTP on linux: 
\code
$ sudo ptp4l -i eno1 -m -l 6 -q -f ~/gptp_config.cfg
\endcode
Replace eno1 with the network interface connected to your PC.

- You will see logs in the UART terminal as shown in the next section. PC side logs are with Intel i210 card.
- Launch a CCS debug session and run the example executable, see \ref CCS_LAUNCH_PAGE
- Connect board and PC as mentioned in "HW Setup" above.

## Sample Log Output
### PC Output
\code
ptp4l[8056.842]: selected /dev/ptp0 as PTP clock
ptp4l[8056.892]: port 1: INITIALIZING to LISTENING on INIT_COMPLETE
ptp4l[8056.892]: port 0: INITIALIZING to LISTENING on INIT_COMPLETE
ptp4l[8059.849]: port 1: link down
ptp4l[8059.849]: port 1: LISTENING to FAULTY on FAULT_DETECTED (FT_UNSPECIFIED)
ptp4l[8059.871]: selected local clock 6805ca.fffe.c87ac2 as best master
ptp4l[8059.871]: assuming the grand master role
ptp4l[8062.744]: port 1: link up
ptp4l[8062.784]: port 1: FAULTY to LISTENING on INIT_COMPLETE
ptp4l[8066.197]: port 1: LISTENING to MASTER on ANNOUNCE_RECEIPT_TIMEOUT_EXPIRES
ptp4l[8066.197]: selected local clock 6805ca.fffe.c87ac2 as best master
ptp4l[8066.197]: assuming the grand master role
ptp4l[8066.699]: port 1: new foreign master f4844c.fffe.fbc042-1
ptp4l[8068.699]: selected best master clock f4844c.fffe.fbc042
ptp4l[8068.699]: port 1: MASTER to UNCALIBRATED on RS_SLAVE
ptp4l[8069.078]: port 1: UNCALIBRATED to SLAVE on MASTER_CLOCK_SELECTED
ptp4l[8069.700]: rms 3879921009606 max 7759842019460 freq -34943 +/- 7727 delay   200 +/-   0
ptp4l[8070.700]: rms 2254 max 3442 freq -27102 +/- 3073 delay   199 +/-   0
ptp4l[8071.700]: rms 3762 max 3942 freq -20229 +/- 1002 delay   198 +/-   0
...
...
ptp4l[8087.700]: rms    9 max   15 freq -20755 +/-   7 delay   199 +/-   0
ptp4l[8088.700]: rms    7 max   11 freq -20750 +/-   5 delay   200 +/-   0

\endcode
### DUT output
\code
==========================
          gPTP App        
==========================
Enabling clocks!
start to open driver.
Mdio_open: MDIO Manual_Mode enabled
sitara-icssg: Open port 1
EnetPhy_bindDriver: PHY 3: OUI:080028 Model:0f Ver:01 <-> 'dp83869' : OK
sitara-icssg: Open port 2
EnetPhy_bindDriver: PHY 15: OUI:080028 Model:0f Ver:01 <-> 'dp83869' : OK
PHY 3 is alive
PHY 15 is alive
sitara-icssg: Create RX task for regular traffic 
initQs() txFreePktInfoQ initialized with 8 pkts
MAC port addr: f4:84:4c:fb:c0:42
unibase-1.1.4
INF:cbase:tilld0: has mac: F4:84:4C:FB:C0:42
INF:cbase:tilld1: has mac: 00:00:00:00:00:00
Start: uniconf_task
sitara-icssg: default RX flow started
EnetApp_uniconfTask: dbname: NULL
INF:uconf:simpledb_open:no data is imported
INF:uconf:uc_hwal_open:
INF:cbase:cb_rawsock_open:combase-1.1.3
INF:cbase:cb_rawsock_open:dmaTxChId=-1 numRxChannels=0 dmaRxChId=-1 nTxPkts=0 nRxPkts=0 pktSize=0
INF:uconf:create_semname_with_dbname:null dbname is specified.
INF:cbase:cb_lld_task_create:alloc stack size=16384
INF:uconf:000000-269046:uniconf_main:uniconf started
EnetApp_gptpYangConfig:domain=0
INF:uconf:get_exmodid_in_db:first xl4gptp:exmodid=0
INF:uconf:create_semname_with_dbname:null dbname is specified.
EnetApp_gptpNonYangConfig:XL4_EXTMOD_XL4GPTP_SINGLE_CLOCK_MODE=1
EnetApp_gptpNonYangConfig:XL4_EXTMOD_XL4GPTP_USE_HW_PHASE_ADJUSTMENT=1
EnetApp_gptpNonYangConfig:XL4_EXTMOD_XL4GPTP_CLOCK_COMPUTE_INTERVAL_MSEC=100
EnetApp_gptpNonYangConfig:XL4_EXTMOD_XL4GPTP_FREQ_OFFSET_IIR_ALPHA_START_VALUE=1
EnetApp_gptpNonYangConfig:XL4_EXTMOD_XL4GPTP_FREQ_OFFSET_IIR_ALPHA_STABLE_VALUE=4
EnetApp_gptpNonYangConfig:XL4_EXTMOD_XL4GPTP_PHASE_OFFSET_IIR_ALPHA_START_VALUE=1
EnetApp_gptpNonYangConfig:XL4_EXTMOD_XL4GPTP_PHASE_OFFSET_IIR_ALPHA_STABLE_VALUE=4
EnetApp_gptpNonYangConfig:XL4_EXTMOD_XL4GPTP_MAX_DOMAIN_NUMBER=1
Start: gptp2d_task
EnetApp_initTsn:TSN app start done!
INF:cbase:cbl_query_response:tilld0 link DOWN !!!!
INF:cbase:cbl_query_response:tilld1 link DOWN !!!!
INF:cbase:tilld1: alloc mac: 70:FF:76:1E:3A:07
INF:gptp:gptpman_run:max_domains=1, max_ports=2
INF:cbase:cb_rawsock_open:combase-1.1.3
INF:cbase:cb_rawsock_open:dmaTxChId=1 numRxChannels=2 dmaRxChId=1 nTxPkts=8 nRxPkts=8 pktSize=1536
INF:cbase:rxChId 1 has owner dmaRxShared 0
INF:cbase:rxChId 3 has owner dmaRxShared 0
For ICSSG, EthType and VlanId are not used to match the packet only dest addr is used 
For ICSSG, EthType and VlanId are not used to match the packet only dest addr is used 
INF:gptp:dev:tilld0 open success
INF:gptp:dev:tilld1 open success
INF:gptp:gptpnet_init:Open lldtsync OK!
INF:gptp:IEEE1588-2019 performance monitoring disabled.
INF:uconf:ydbi_get_item_ifk3vk0:no data
INF:uconf:ydbi_get_item_ifk3vk0:no data
INF:gptp:onenet_activate:tilld0 status=0, duplex=1, speed=0Mbps
INF:uconf:ydbi_get_item_ifk3vk0:no data
INF:uconf:ydbi_get_item_ifk3vk0:no data
INF:gptp:onenet_activate:tilld1 status=0, duplex=1, speed=0Mbps
INF:ubase:GPTP_MEDIUM_ALLOC: fragsize=16 fragused/fragnum=830/921 (90%)
INF:ubase:GPTP_SMALL_ALLOC: fragsize=4 fragused/fragnum=19/71 (26%)
INF:ubase:SM_DATA_INST: fragsize=8 fragused/fragnum=2002/2002 (100%)
INF:gptp:gptpman_run:GPTPNET_INTERVAL_TIMEOUT_NSEC=125000000
INF:gptp:000000-627984:domainIndex=0, GM changed old=00:00:00:00:00:00:00:00, new=F4:84:4C:FF:FE:FB:C0:42
INF:gptp:gptpclock_set_gmsync:gptpInstanceIndex=0, domainIndex=0, gmstate=2
INF:gptp:set_phase_offsetGM:domainIndex=0, New adjustment(New GM?)
Icssg_handleLinkUp: icssg1: Port 2: Link up: 1-Gbps Full-Duplex
INF:cbase:cbl_query_response:tilld1: link UP, speed=1000, duplex=1 !!!!
INF:gptp:index=2 speed=1000, duplex=full, ptpdev=tilld1
Icssg_handleLinkUp: icssg1: Port 1: Link up: 1-Gbps Full-Duplex
INF:cbase:cbl_query_response:tilld0: link UP, speed=1000, duplex=1 !!!!
INF:gptp:md_pdelay_resp_sm_recv_req:port=1, set receivedNonCMLDSPdelayReq=1
INF:gptp:index=1 speed=1000, duplex=full, ptpdev=tilld0
WRN:gptp:000004-258771:waiting_for_pdelay_interval_timer_proc:portIndex=1, sourcePortIdentity=F4:84:4C:FF:FE:FB:C0:5C, thisClock=F4:84:4C:FF:FE:FB:C0:42, neighborPropDelay=456
INF:gptp:waiting_for_pdelay_interval_timer_proc:portIndex=1, not asCapable
ERR:gptp:provide_rxframe: RX not ETH_P_1588 packet 0x88CC
INF:gptp:waiting_for_pdelay_interval_timer_proc:set asCapableAcrossDomains, portIndex=1
INF:gptp:set asCapable for domainIndex=0, portIndex=1
INF:gptp:000005-384967:gptpgcfg_set_asCapable:domainInde=0, portIndex=1, ascapable=1
INF:gptp:000005-392349:gm_stable:gm_unstable_proc:domainIndex=0
INF:gptp:gptpclock_set_gmsync:gptpInstanceIndex=0, domainIndex=0, gmstate=1
INF:gptp:000005-404375:gm_stable:gm_unstable_proc:domainIndex=0
      5.383s : CPU load =   5.27 %
INF:gptp:000005-499279:setSyncTwoStep_txSync:domainIndex=0, portIndex=1, sync gap=5500msec
INF:gptp:000005-504754:setFollowUp_txFollowUp:domainIndex=0, portIndex=1, fup gap=5507msec
INF:gptp:000006-499386:gm_stable:gm_stable_proc:domainIndex=0
INF:gptp:gptpclock_set_gmsync:gptpInstanceIndex=0, domainIndex=0, gmstate=2
domain=0, offset=0nsec, hw-adjrate=0ppb
        gmsync=true, last_setts64=0nsec
     10.384s : CPU load =   1.81 %
     15.385s : CPU load =   1.70 %
..........
..........
\endcode

# See Also

\ref NETWORKING |
\ref EXAMPLES_ENET_ICSSG_TSN_GPTP_BRIDGE |
\ref EXAMPLES_ENET_ICSSG_TSN_GPTP_TR |
\ref EXAMPLES_ENET_ICSSG_TSN_GPTP_TT |
\ref ENET_CPSW_TSN_GPTP