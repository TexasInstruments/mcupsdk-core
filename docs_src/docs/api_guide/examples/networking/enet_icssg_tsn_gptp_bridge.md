# Ethernet TSN ICSSG gPTP Bridge Example{#EXAMPLES_ENET_ICSSG_TSN_GPTP_BRIDGE}

[TOC]

# Introduction
This ethernet TSN example illustrates the usage of gPTP IEEE 802.1AS stack with ICSSG peripheral, in gPTP Bridge mode.

However, the application used here supports all the below modes:
    - gPTP End-Point time_transmitter mode (i.e. master mode)
    - gPTP End-Point time_receiver mode (i.e. slave mode)
    - gPTP Bridge mode

Please note that only ICSSG switch mode supports gPTP bridge. In this example, there are two PCs connected to EVM (DUT). One of the PC is configured as PTP master and another as PTP slave.

Yang based configuration is also supported. Currently File System is not supported, will be added in future releases.

Along with PTP traffic, application also handles non-PTP traffic in a separate RTOS task and DMA Channel. Received non-PTP packets are echoed back by the application.

See also :\ref ENET_CPSW_TSN_GPTP for gPTP stack documentation.

# Supported Combinations

\cond SOC_AM64X

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0_freertos
 Toolchain      | ti-arm-clang
 Board          | @VAR_BOARD_NAME_LOWER
 Example folder | examples/networking/tsn/gptp_icssg_app/gptp_icssg_switch

\endcond

\cond SOC_AM243X

 Parameter      | Value
 ---------------|-----------
 CPU + OS       | r5fss0-0_freertos
 Toolchain      | ti-arm-clang
 Boards         | @VAR_BOARD_NAME_LOWER, @VAR_LP_BOARD_NAME_LOWER
 Example folder | examples/networking/tsn/gptp_icssg_app/gptp_icssg_switch

\endcond

# Steps to Run the Example

## Prerequisites
-  EVM Board
- Cat6 ethernet cable
- Two PCs with Linux Ubuntu OS (or any PC running bash shell) and both with PTP capable network card.
- Install `linuxptp` on both the PCs
\code
$ sudo apt-get install linuxptp
$ ptp4l -v
\endcode
- Configure linuxptp
\code
$ wget https://raw.githubusercontent.com/richardcochran/linuxptp/v4.3/configs/gPTP.cfg -O ~/gptp_config.cfg
\endcode
\attention On PC1, change the value of `priority1` in ~/gptp_config.cfg file to `100`, to enforce PC1 to gPTP master.

\attention On PC2, change the value of `priority1` in ~/gptp_config.cfg file to `255`, to enforce PC2 to gPTP slave.

\attention For some network cards, there is a bug with internal propagation delay. So, in those cases you might need to increase the `neighborPropDelayThresh` in ptp_config.cfg as below-

####On PC1
\code
$ cat ~/gptp_config.cfg
\#
\# 802.1AS example configuration containing those attributes which
\# differ from the defaults.  See the file, default.cfg, for the
/# complete list of available options.
/#
[global]
gmCapable               1
priority1               100
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
####On PC2
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
### PC1 Output
\code
$ sudo ptp4l -i eno1 -m -l 6 -q -f ~/gptp_config.cfg
ptp4l[7727.045]: selected /dev/ptp0 as PTP clock
ptp4l[7727.088]: port 1: INITIALIZING to LISTENING on INIT_COMPLETE
ptp4l[7727.088]: port 0: INITIALIZING to LISTENING on INIT_COMPLETE
ptp4l[7727.088]: port 1: link down
ptp4l[7727.088]: port 1: LISTENING to FAULTY on FAULT_DETECTED (FT_UNSPECIFIED)
ptp4l[7727.127]: selected local clock 6805ca.fffe.c87ac2 as best master
ptp4l[7727.127]: assuming the grand master role
ptp4l[7729.260]: port 1: link up
ptp4l[7729.300]: port 1: FAULTY to LISTENING on INIT_COMPLETE
ptp4l[7732.330]: port 1: new foreign master f4844c.fffe.fbc042-1
ptp4l[7732.354]: port 1: LISTENING to MASTER on ANNOUNCE_RECEIPT_TIMEOUT_EXPIRES
ptp4l[7732.354]: selected local clock 6805ca.fffe.c87ac2 as best master
ptp4l[7732.354]: assuming the grand master role
\endcode

### PC2 Output
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

### DUT output
\code
==========================
          gPTP App        
==========================
Enabling clocks!
start to open driver.
Mdio_open:294 
sitara-icssg: Open port 1
EnetPhy_bindDriver:1828 
sitara-icssg: Open port 2
EnetPhy_bindDriver:1828 
PHY 3 is alive
PHY 15 is alive
sitara-icssg: Create RX task for regular traffic 
initQs() txFreePktInfoQ initialized with 8 pkts
MAC port addr: f4:84:4c:fb:c0:5c
unibase-1.1.4
INF:cbase:tilld0: has mac: F4:84:4C:FB:C0:5C
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
INF:uconf:000000-211150:uniconf_main:uniconf started
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
INF:gptp:gptpman_run:max_domains=1, max_ports=2
INF:cbase:cb_rawsock_open:combase-1.1.3
INF:cbase:cb_rawsock_open:dmaTxChId=1 numRxChannels=2 dmaRxChId=1 nTxPkts=8 nRxPkts=8 pktSize=1536
INF:cbase:rxChId 1 has owner dmaRxShared 0
INF:cbase:rxChId 3 has owner dmaRxShared 0
For ICSSG, EthType and VlanId are not used to match the packet only dest addr is used 
For ICSSG, EthType and VlanId are not used to match the packet only dest addr is used 
INF:cbase:cbl_query_response:tilld0 link DOWN !!!!
INF:cbase:cbl_query_response:tilld1 link DOWN !!!!
INF:cbase:tilld1: alloc mac: 70:FF:76:1E:90:E9
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
INF:gptp:000000-500553:domainIndex=0, GM changed old=00:00:00:00:00:00:00:00, new=F4:84:4C:FF:FE:FB:C0:5C
INF:gptp:gptpclock_set_gmsync:gptpInstanceIndex=0, domainIndex=0, gmstate=2
INF:gptp:set_phase_offsetGM:domainIndex=0, New adjustment(New GM?)
Icssg_handleLinkUp:2585 
Icssg_handleLinkUp:2585 
INF:cbase:cbl_query_response:tilld0: link UP, speed=1000, duplex=1 !!!!
INF:cbase:cbl_query_response:tilld1: link UP, speed=1000, duplex=1 !!!!
INF:gptp:index=1 speed=1000, duplex=full, ptpdev=tilld0
INF:gptp:index=2 speed=1000, duplex=full, ptpdev=tilld1
WRN:gptp:000003-132482:waiting_for_pdelay_interval_timer_proc:portIndex=1, sourcePortIdentity=F4:84:4C:FF:FE:FB:C0:42, thisClock=F4:84:4C:FF:FE:FB:C0:5C, neighborPropDelay=466
INF:gptp:waiting_for_pdelay_interval_timer_proc:portIndex=1, not asCapable
INF:gptp:md_pdelay_resp_sm_recv_req:port=2, set receivedNonCMLDSPdelayReq=1
WRN:gptp:000003-253408:waiting_for_pdelay_interval_timer_proc:portIndex=2, sourcePortIdentity=F4:84:4C:FF:FE:FC:34:F0, thisClock=F4:84:4C:FF:FE:FB:C0:5C, neighborPropDelay=436
INF:gptp:waiting_for_pdelay_interval_timer_proc:portIndex=2, not asCapable
INF:gptp:md_pdelay_resp_sm_recv_req:port=1, set receivedNonCMLDSPdelayReq=1
INF:gptp:waiting_for_pdelay_interval_timer_proc:set asCapableAcrossDomains, portIndex=1
INF:gptp:set asCapable for domainIndex=0, portIndex=1
INF:gptp:000004-134133:gptpgcfg_set_asCapable:domainInde=0, portIndex=1, ascapable=1
INF:gptp:000004-141544:gm_stable:gm_unstable_proc:domainIndex=0
INF:gptp:gptpclock_set_gmsync:gptpInstanceIndex=0, domainIndex=0, gmstate=1
INF:gptp:000004-153735:gm_stable:gm_unstable_proc:domainIndex=0
INF:gptp:000004-249201:setSyncTwoStep_txSync:domainIndex=0, portIndex=1, sync gap=4250msec
INF:gptp:000004-254467:setFollowUp_txFollowUp:domainIndex=0, portIndex=1, fup gap=4257msec
INF:gptp:waiting_for_pdelay_interval_timer_proc:set asCapableAcrossDomains, portIndex=2
INF:gptp:set asCapable for domainIndex=0, portIndex=2
INF:gptp:000004-274917:gptpgcfg_set_asCapable:domainInde=0, portIndex=2, ascapable=1
INF:gptp:000004-499189:setSyncTwoStep_txSync:domainIndex=0, portIndex=2, sync gap=4500msec
INF:gptp:000004-504521:setFollowUp_txFollowUp:domainIndex=0, portIndex=2, fup gap=4507msec
INF:gptp:000004-841971:domainIndex=0, GM changed old=F4:84:4C:FF:FE:FB:C0:5C, new=F4:84:4C:FF:FE:FB:C0:42
INF:gptp:gptpclock_set_gmsync:gptpInstanceIndex=0, domainIndex=0, gmstate=0
INF:gptp:000004-855062:gm_stable:gm_unstable_proc:domainIndex=0
INF:gptp:gptpclock_set_gmsync:gptpInstanceIndex=0, domainIndex=0, gmstate=1
INF:gptp:000004-962646:setSyncTwoStep_txSync:domainIndex=0, portIndex=2, sync gap=210msec
INF:gptp:set_phase_offsetGM:domainIndex=0, New adjustment(New GM?)
INF:gptp:set_phase_offsetGM:domainIndex=0, offset adjustment, diff=-2135313174
INF:gptp:000004-981623:setFollowUp_txFollowUp:domainIndex=0, portIndex=2, fup gap=234msec
INF:gptp:set_phase_offsetGM:domainIndex=0, offset adjustment by Freq., diff=12338
INF:gptp:set_phase_offsetGM:domainIndex=0, offset adjustment by Freq., diff=12356
IFV:gptp:domainIndex=0, clock_master_sync_receive:the master clock rate to 12499ppb, GMdiff=12356nsec
      5.300s : CPU load =   3.31 %
INF:gptp:set_phase_offsetGM:domainIndex=0, offset adjustment by Freq., diff=10872
INF:gptp:clock_master_sync_receive:computeGmRateRatio:domainIndex=0 unstable rate=-11878ppb (timeleap_past)
IFV:gptp:domainIndex=0, clock_master_sync_receive:the master clock rate to 11493ppb, GMdiff=10872nsec
INF:gptp:set_phase_offsetGM:domainIndex=0, stable
IFV:gptp:domainIndex=0, clock_master_sync_receive:the master clock rate to 9514ppb, GMdiff=9444nsec
IFV:gptp:domainIndex=0, clock_master_sync_receive:the master clock rate to 2173ppb, GMdiff=8268nsec
IFV:gptp:domainIndex=0, clock_master_sync_receive:the master clock rate to 2194ppb, GMdiff=8020nsec
IFV:gptp:domainIndex=0, clock_master_sync_receive:the master clock rate to 2089ppb, GMdiff=7764nsec
INF:gptp:000005-874072:gm_stable:gm_stable_proc:domainIndex=0
INF:gptp:gptpclock_set_gmsync:gptpInstanceIndex=0, domainIndex=0, gmstate=2
IFV:gptp:domainIndex=0, clock_master_sync_receive:the master clock rate to 1893ppb, GMdiff=7276nsec
IFV:gptp:domainIndex=0, clock_master_sync_receive:the master clock rate to 1988ppb, GMdiff=7067nsec
..........
..........
IFV:gptp:domainIndex=0, clock_master_sync_receive:the master clock rate to 166ppb, GMdiff=7nsec
IFV:gptp:domainIndex=0, clock_master_sync_receive:the master clock rate to 184ppb, GMdiff=15nsec
IFV:gptp:domainIndex=0, clock_master_sync_receive:the master clock rate to 170ppb, GMdiff=7nsec
IFV:gptp:domainIndex=0, clock_master_sync_receive:the master clock rate to 189ppb, GMdiff=17nsec
...........
\endcode

# See Also

\ref NETWORKING |
\ref EXAMPLES_ENET_ICSSG_TSN_GPTP_BRIDGE |
\ref EXAMPLES_ENET_ICSSG_TSN_GPTP_TR |
\ref EXAMPLES_ENET_ICSSG_TSN_GPTP_TT |
\ref ENET_CPSW_TSN_GPTP